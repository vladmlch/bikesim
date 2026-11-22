#include "step_closure_bundle.hpp"

#include "../app/prepare.hpp"
#include "../configuration/load.hpp"
#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../geometry/specs.hpp"
#include "../initialization/capture.hpp"
#include "../initialization/launch_preparation.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../model/builder.hpp"
#include "../model/export.hpp"
#include "../model/rider_geometry.hpp"
#include "../model_access.hpp"
#include "../platform/files.hpp"
#include "../runtime/run_limits.hpp"
#include "../runtime/runtime.hpp"
#include "../runtime/samples.hpp"
#include "../stepper.hpp"
#include "../terrain/trackfile.hpp"
#include "../validation/runtime_access.hpp"
#include "../writers/drivetrain.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace bike::golden {
namespace {

static_assert(std::endian::native == std::endian::little,
              "step_closure.bin assumes little-endian host representation");

struct DataDeleter {
    void operator()(mjData *data) const noexcept {
        if (data != nullptr) {
            mj_deleteData(data);
        }
    }
};
using DataOwner = std::unique_ptr<mjData, DataDeleter>;

constexpr std::array<std::string_view, 8> kSuspensionComponents{{
    "fork_spring",
    "fork_damper",
    "shock_coil",
    "shock_bumper",
    "shock_damper",
    "shock_top_out",
    "shock_upper_stop",
    "shock_hbo",
}};

constexpr std::array<std::string_view, 2> kResistanceComponents{{
    "road_rolling",
    "aerodynamic",
}};

[[nodiscard]] io::Value int_value(std::int64_t v) {
    return io::Value{io::Number{std::to_string(v)}};
}

[[nodiscard]] io::Value double_value(double v) {
    if (!std::isfinite(v)) {
        throw std::invalid_argument(
            "step_closure_bundle: nonfinite value in JSON metadata");
    }
    return io::Value{io::Number{geometry::detail::py_g(v, 17)}};
}

[[nodiscard]] std::string format_cog_name(std::size_t idx) {
    return std::string("freehub_cog_") + (idx < 10 ? "0" : "") +
           std::to_string(idx);
}

[[nodiscard]] std::vector<double> copy_doubles(const double *ptr, mjtSize count,
                                               std::string_view label) {
    const auto span = model_access::readonly_buffer(ptr, count, label);
    return {span.begin(), span.end()};
}

[[nodiscard]] std::vector<double> copy_hfield_as_doubles(const mjModel &m) {
    const auto span =
        model_access::readonly_buffer(m.hfield_data, m.nhfielddata, "hfield_data");
    std::vector<double> out;
    out.reserve(span.size());
    for (const float v : span) {
        out.push_back(static_cast<double>(v));
    }
    return out;
}

void write_binary_parts(const std::filesystem::path &path,
                        std::initializer_list<std::span<const std::byte>> parts) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
    if (ec) {
        throw std::runtime_error("cannot prepare bundle output file " +
                                 path.string() + ": " + ec.message());
    }
    auto handle = platform::open_exclusive(path);
    for (const auto part : parts) {
        if (!part.empty()) {
            platform::write_all(handle, part);
        }
    }
    handle.close();
}

void write_bytes_file(const std::filesystem::path &path,
                      std::span<const std::byte> bytes) {
    write_binary_parts(path, {bytes});
}

void write_text_file(const std::filesystem::path &path, std::string_view text) {
    write_bytes_file(path, std::as_bytes(std::span<const char>{text}));
}

[[nodiscard]] std::string sha256_parts(
    std::initializer_list<std::span<const std::byte>> parts) {
    static constexpr std::string_view kDigits = "0123456789abcdef";
    io::Sha256 hash;
    for (const auto part : parts) {
        if (!part.empty()) {
            hash.update(part);
        }
    }
    std::string hex;
    hex.reserve(64);
    for (const std::uint8_t byte : hash.finish()) {
        hex += kDigits[byte >> 4U];
        hex += kDigits[byte & 0x0fU];
    }
    return hex;
}

[[nodiscard]] std::vector<std::byte> read_bounded_bytes(
    const std::filesystem::path &path, std::size_t max_bytes) {
    const auto handle = platform::open_regular(path);
    struct stat info {};
    if (::fstat(handle.get(), &info) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "step_closure_bundle: fstat " + path.string());
    }
    if (info.st_size < 0 || std::cmp_greater(info.st_size, max_bytes)) {
        throw std::invalid_argument(
            "step_closure_bundle: file exceeds read bound: " + path.string());
    }
    std::vector<std::byte> bytes(static_cast<std::size_t>(info.st_size));
    std::size_t at = 0;
    while (at < bytes.size()) {
        const ssize_t got = ::read(
            handle.get(), std::span{bytes}.subspan(at).data(),
            bytes.size() - at);
        if (got < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::system_error(errno, std::generic_category(),
                                    "step_closure_bundle: read " + path.string());
        }
        if (got == 0) {
            throw std::invalid_argument(
                "step_closure_bundle: file truncated while reading: " +
                path.string());
        }
        at += static_cast<std::size_t>(got);
    }
    std::byte extra{};
    ssize_t tail = 0;
    while (true) {
        tail = ::read(handle.get(), &extra, 1);
        if (tail < 0 && errno == EINTR) {
            continue;
        }
        break;
    }
    if (tail < 0) {
        throw std::system_error(errno, std::generic_category(),
                                "step_closure_bundle: read tail " + path.string());
    }
    if (tail != 0) {
        throw std::invalid_argument(
            "step_closure_bundle: file changed while reading: " +
            path.string());
    }
    return bytes;
}

[[nodiscard]] std::string read_text_file(const std::filesystem::path &path) {
    const auto raw = io::read_numpy_bytes(path);
    std::string out(raw.size(), '\0');
    std::ranges::transform(raw, out.begin(), [](std::byte b) {
        return static_cast<char>(b);
    });
    return out;
}

template <typename T>
[[nodiscard]] T decode_pod(std::span<const std::byte> bytes) {
    if (bytes.size() != sizeof(T)) {
        throw std::invalid_argument("step_closure_bundle: POD byte size mismatch");
    }
    std::array<std::byte, sizeof(T)> buf{};
    std::ranges::copy(bytes, buf.begin());
    return std::bit_cast<T>(buf);
}

[[nodiscard]] std::vector<double> decode_doubles(
    std::span<const std::byte> bytes) {
    if (bytes.size() % sizeof(double) != 0) {
        throw std::invalid_argument(
            "step_closure_bundle: float64 payload is not 8-byte aligned");
    }
    const std::size_t count = bytes.size() / sizeof(double);
    std::vector<double> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        out.push_back(decode_pod<double>(
            bytes.subspan(i * sizeof(double), sizeof(double))));
    }
    return out;
}

[[nodiscard]] std::size_t compute_row_doubles(std::size_t nq, std::size_t nv,
                                              std::size_t na, std::size_t nu,
                                              std::size_t ntendon) {
    // 8 scalar fields (step_index, time_s, dt_s, wrap_prm_freehub, cog_index,
    // ncon, nefc, solver_niter) + 2 wheel frictionloss + 2*ntendon ranges +
    // 2*nq (qpos, next_qpos) + 11*nv (qvel, qacc_warmstart, qfrc_applied,
    // 6 component vectors, next_qvel, next_qacc) + nu + na.
    return 8U + 2U + 2U * ntendon + 2U * nq + 11U * nv + nu + na;
}

void append_doubles(std::vector<double> &dst, std::span<const double> src) {
    dst.insert(dst.end(), src.begin(), src.end());
}

void read_doubles(std::span<const double> src, std::size_t &offset,
                  std::vector<double> &dst, std::size_t count) {
    if (offset + count > src.size()) {
        throw std::runtime_error("step_closure.bin row truncated");
    }
    dst.assign(src.begin() + static_cast<std::ptrdiff_t>(offset),
               src.begin() + static_cast<std::ptrdiff_t>(offset + count));
    offset += count;
}

[[nodiscard]] int find_cog_index(const std::vector<int> &cassette,
                                 int rear_teeth) {
    for (std::size_t i = 0; i < cassette.size(); ++i) {
        if (cassette[i] == rear_teeth) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void accumulate_component(std::vector<double> &target,
                          const std::vector<double> &src) {
    if (target.size() != src.size()) {
        throw std::runtime_error("force component width mismatch");
    }
    for (std::size_t i = 0; i < target.size(); ++i) {
        target[i] += src[i];
    }
}

void assign_hfield_from_doubles(const mjModel &m,
                                std::span<const double> values) {
    const auto dst = model_access::mutable_buffer(m.hfield_data, m.nhfielddata,
                                                  "hfield_data");
    if (dst.size() != values.size()) {
        throw std::invalid_argument(
            "hfield_data.bin element count does not match compiled mjModel");
    }
    for (std::size_t i = 0; i < dst.size(); ++i) {
        dst[i] = static_cast<float>(values[i]);
    }
}

[[nodiscard]] std::string replace_range_attr(std::string attrs) {
    const std::string key = "range=\"";
    const auto pos = attrs.find(key);
    if (pos == std::string::npos) {
        attrs += " range=\"-1e12 1e12\"";
        return attrs;
    }
    const auto val_start = pos + key.size();
    const auto val_end = attrs.find('"', val_start);
    if (val_end == std::string::npos) {
        throw std::invalid_argument("malformed range attribute in model.xml");
    }
    attrs.replace(val_start, val_end - val_start, "-1e12 1e12");
    return attrs;
}

void patch_12gears_tendon_range(const mjModel &m12,
                                const StepClosureBundle &bundle,
                                const StepClosureRow &row, int clutch_tid_12) {
    const auto tr = model_access::mutable_buffer(
        m12.tendon_range, static_cast<mjtSize>(2) * m12.ntendon, "tendon_range");
    for (int t = 0; t < m12.ntendon; ++t) {
        const auto base =
            static_cast<std::size_t>(2) * static_cast<std::size_t>(t);
        tr[base] = -1e12;
        tr[base + 1U] = 1e12;
    }
    const int freehub_tid = bundle.freehub_tendon_id;
    const int clutch_tid = bundle.clutch_tendon_id;
    if (freehub_tid >= 0 &&
        2U * static_cast<std::size_t>(freehub_tid) + 1U < row.tendon_range.size() &&
        row.cog_index >= 0 &&
        row.cog_index < m12.ntendon - (clutch_tid_12 >= 0 ? 1 : 0)) {
        const auto cbase =
            2U * static_cast<std::size_t>(row.cog_index);
        const auto fbase =
            2U * static_cast<std::size_t>(freehub_tid);
        tr[cbase] = row.tendon_range[fbase];
        tr[cbase + 1U] = row.tendon_range[fbase + 1U];
    }
    if (clutch_tid_12 >= 0 && clutch_tid >= 0 &&
        2U * static_cast<std::size_t>(clutch_tid) + 1U < row.tendon_range.size()) {
        const auto clbase = 2U * static_cast<std::size_t>(clutch_tid_12);
        const auto sbase = 2U * static_cast<std::size_t>(clutch_tid);
        tr[clbase] = row.tendon_range[sbase];
        tr[clbase + 1U] = row.tendon_range[sbase + 1U];
    }
}

struct BundleModelDims {
    mjtSize nv = 0;
    mjtSize ntendon = 0;
};

void validate_header_indices(const BundleModelDims &dims,
                             const StepClosureBundle &bundle) {
    if (bundle.front_wheel_dof < 0 || bundle.front_wheel_dof >= dims.nv ||
        bundle.rear_wheel_dof < 0 || bundle.rear_wheel_dof >= dims.nv) {
        throw std::invalid_argument(
            "step_closure.bin wheel DOF index out of [0, nv) range");
    }
    if (bundle.freehub_wrap_adr < 0) {
        throw std::invalid_argument(
            "step_closure.bin freehub_wrap_adr must be nonnegative");
    }
    if (bundle.freehub_tendon_id < 0 ||
        bundle.freehub_tendon_id >= dims.ntendon) {
        throw std::invalid_argument(
            "step_closure.bin freehub_tendon_id out of [0, ntendon) range");
    }
    if (bundle.clutch_tendon_id != -1 &&
        (bundle.clutch_tendon_id < 0 ||
         bundle.clutch_tendon_id >= dims.ntendon)) {
        throw std::invalid_argument(
            "step_closure.bin clutch_tendon_id out of [0, ntendon) range");
    }
}

void restore_cold_data(const mjModel &m, mjData &d, const StepClosureRow &row) {
    engine::reset_data(&m, &d);
    d.time = row.time_s;
    std::ranges::copy(row.qpos,
                      model_access::mutable_buffer(d.qpos, m.nq, "qpos").begin());
    std::ranges::copy(row.qvel,
                      model_access::mutable_buffer(d.qvel, m.nv, "qvel").begin());
    std::ranges::copy(row.act,
                      model_access::mutable_buffer(d.act, m.na, "act").begin());
    std::ranges::copy(
        row.qacc_warmstart,
        model_access::mutable_buffer(d.qacc_warmstart, m.nv, "qacc_warmstart")
            .begin());
    std::ranges::copy(row.ctrl,
                      model_access::mutable_buffer(d.ctrl, m.nu, "ctrl").begin());
    std::ranges::copy(
        row.qfrc_applied,
        model_access::mutable_buffer(d.qfrc_applied, m.nv, "qfrc_applied")
            .begin());
    std::ranges::fill(
        model_access::mutable_buffer(d.xfrc_applied,
                                     static_cast<mjtSize>(6) * m.nbody,
                                     "xfrc_applied"),
        0.);
}

[[nodiscard]] double max_abs_diff(std::span<const double> a,
                                  std::span<const double> b) {
    if (a.size() != b.size()) {
        throw std::runtime_error("buffer size mismatch in closure comparison");
    }
    double m = 0.;
    for (std::size_t i = 0; i < a.size(); ++i) {
        m = std::max(m, std::abs(a[i] - b[i]));
    }
    return m;
}

[[nodiscard]] bool bitwise_equal(std::span<const double> a,
                                 std::span<const double> b) {
    return std::ranges::equal(std::as_bytes(a), std::as_bytes(b));
}

} // namespace

std::string expand_model_12gears_xml(
    std::string_view model_xml,
    const configuration::SimulationPhysicsConfig &physics) {
    if (physics.drive.transmission_model != "ideal_mid_drive") {
        throw std::invalid_argument(
            "expand_model_12gears_xml requires ideal_mid_drive transmission");
    }
    const auto &cassette = physics.drive.shifting.cassette;
    if (cassette.empty()) {
        throw std::invalid_argument(
            "expand_model_12gears_xml requires a nonempty shifting cassette");
    }
    const std::string needle = "<fixed name=\"ideal_mid_drive_freehub\"";
    const auto start = model_xml.find(needle);
    if (start == std::string_view::npos) {
        throw std::invalid_argument(
            "model.xml does not contain ideal_mid_drive_freehub tendon");
    }
    const auto head_end = model_xml.find('>', start);
    if (head_end == std::string_view::npos) {
        throw std::invalid_argument(
            "malformed ideal_mid_drive_freehub opening tag in model.xml");
    }
    const std::string close_tag = "</fixed>";
    const auto block_end = model_xml.find(close_tag, head_end);
    if (block_end == std::string_view::npos) {
        throw std::invalid_argument(
            "unclosed ideal_mid_drive_freehub block in model.xml");
    }
    const auto attrs = replace_range_attr(
        std::string(model_xml.substr(start + needle.size(),
                                     head_end - (start + needle.size()))));
    const std::string driver =
        physics.drive.motor_clutch ? "drive_shaft_spin" : "crank_spin";
    const double front_teeth =
        static_cast<double>(physics.drive.gearing.front_teeth);

    std::string replacement;
    for (std::size_t c = 0; c < cassette.size(); ++c) {
        if (c > 0) {
            replacement += "\n    ";
        }
        const double coef = front_teeth / static_cast<double>(cassette[c]);
        replacement += "<fixed name=\"" + format_cog_name(c) + "\"" + attrs +
                       ">\n";
        replacement += "      <joint joint=\"" + driver + "\" coef=\"" +
                       geometry::detail::py_g(coef, 17) + "\"/>\n";
        replacement += "      <joint joint=\"rear_wheel_spin\" coef=\"-1\"/>\n";
        replacement += "    </fixed>";
    }

    std::string out;
    out.reserve(model_xml.size() + replacement.size());
    out.append(model_xml.substr(0, start));
    out.append(replacement);
    out.append(model_xml.substr(block_end + close_tag.size()));
    return out;
}

std::size_t max_step_closure_steps(std::size_t nq, std::size_t nv,
                                   std::size_t na, std::size_t nu,
                                   std::size_t ntendon) noexcept {
    if (kMaxStepClosureBytes <= sizeof(StepClosureFileHeader)) {
        return 0;
    }
    const std::size_t payload_cap =
        kMaxStepClosureBytes - sizeof(StepClosureFileHeader);
    const std::size_t row_doubles =
        compute_row_doubles(nq, nv, na, nu, ntendon);
    if (row_doubles == 0 ||
        row_doubles > std::numeric_limits<std::size_t>::max() / sizeof(double)) {
        return 0;
    }
    return payload_cap / (row_doubles * sizeof(double));
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<StepClosureBundle> capture_step_closure_bundle(
    const StepClosureCaptureSpec &spec) {
    auto req = std::make_unique<initialization::SetupRequest>();
    req->origin = spec.physics_config_label.empty() ? "bike-gpu-export"
                                                    : spec.physics_config_label;
    req->asset_root = spec.asset_root.empty() ? std::filesystem::current_path()
                                              : spec.asset_root;
    req->track = spec.track;
    req->seed_decimal = spec.seed_decimal;
    req->model.mode = "ride";
    req->model.field = terrain::HeightFieldSpec::for_track(req->track);
    req->model.physics = spec.physics;
    (void)initialization::prepare_request(*req);
    configuration::post_init(*req->model.physics);
    if (req->model.physics->tires.backend == "pneumatic") {
        req->model.bike.front_wheel_radius =
            req->model.physics->tires.pneumatic.front.outer_radius_mm;
        req->model.bike.rear_wheel_radius =
            req->model.physics->tires.pneumatic.rear.outer_radius_mm;
    }
    const std::string rider_name =
        req->model.physics->drive_mode == "articulated_effort"
            ? "articulated_planar"
            : spec.rider_variant;
    req->model.rider = model::rider_from_args(
        rider_name, spec.rider_mass_kg, spec.rider_height_m, std::nullopt);
    req->model.include_rider = req->model.rider->present();
    req->model.crank_joint = true;
    req->runtime.speed_kph = 25.0;
    req->runtime.strict = spec.strict;
    req->runtime.measurement_decimation = 1;
    req->runtime.record_retention = runtime::RetentionMode::full;

    const double dt = req->model.physics->timestep_s;
    const auto limits =
        runtime::RunLimits::for_track(req->track.length_m, req->start_x_m, dt);
    double requested_steps = static_cast<double>(limits.max_steps);
    if (spec.duration_s.has_value()) {
        if (!std::isfinite(*spec.duration_s) || *spec.duration_s <= 0.) {
            throw std::invalid_argument(
                "duration_s must be finite and positive");
        }
        requested_steps = std::ceil(*spec.duration_s / dt);
    }
    if (!std::isfinite(requested_steps) ||
        requested_steps >=
            static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        throw std::invalid_argument("duration exceeds step range");
    }
    auto target_steps = static_cast<std::size_t>(requested_steps);
    if (spec.max_steps.has_value()) {
        target_steps = std::min(target_steps, *spec.max_steps);
    }
    if (target_steps == 0) {
        throw std::invalid_argument(
            "capture_step_closure_bundle requires at least 1 step");
    }

    auto bundle = std::make_unique<StepClosureBundle>();
    bundle->physics_config_label = spec.physics_config_label;
    bundle->track_label =
        spec.track_label.empty() ? req->track.name : spec.track_label;
    bundle->dt_s = dt;
    bundle->physics_config_json =
        configuration::detail::plain_document(configuration::to_value(*req->model.physics));
    bundle->resolved_bike = req->model.bike;
    bundle->track_spec = req->track;
    bundle->front_teeth = req->model.physics->drive.gearing.front_teeth;
    bundle->cassette = req->model.physics->drive.shifting.cassette;
    bundle->model_xml = model::generate_mjcf(req->model);
    bundle->model_12gears_xml =
        expand_model_12gears_xml(bundle->model_xml, *req->model.physics);

    auto prepared = app::prepare(std::move(req));
    auto ride = app::make_ride(std::move(prepared));
    ride->set_finish_position(limits.finish_x_m);

    validation::RuntimeRigAccess::inspect(
        *ride, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            bundle->nq = static_cast<int>(m->nq);
            bundle->nv = static_cast<int>(m->nv);
            bundle->na = static_cast<int>(m->na);
            bundle->nu = static_cast<int>(m->nu);
            bundle->ntendon = static_cast<int>(m->ntendon);
            bundle->hfield_data = copy_hfield_as_doubles(*m);
            const auto image = initialization::capture_model_image(*m);
            bundle->model_mjb = image.mjb;
            bundle->model_mjb_sha256 = image.sha256;

            const int fw_jnt =
                model_access::resolve_id(m, mjOBJ_JOINT, "front_wheel_spin");
            const int rw_jnt =
                model_access::resolve_id(m, mjOBJ_JOINT, "rear_wheel_spin");
            const auto jnt_dof =
                model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
            bundle->front_wheel_dof = jnt_dof[static_cast<std::size_t>(fw_jnt)];
            bundle->rear_wheel_dof = jnt_dof[static_cast<std::size_t>(rw_jnt)];

            bundle->freehub_tendon_id = model_access::resolve_id(
                m, mjOBJ_TENDON, "ideal_mid_drive_freehub");
            const auto ten_adr = model_access::readonly_buffer(
                m->tendon_adr, m->ntendon, "tendon_adr");
            bundle->freehub_wrap_adr =
                ten_adr[static_cast<std::size_t>(bundle->freehub_tendon_id)];
            bundle->clutch_tendon_id =
                mj_name2id(m, mjOBJ_TENDON, "crank_clutch");
        });

    const std::size_t max_steps_allowed = max_step_closure_steps(
        static_cast<std::size_t>(bundle->nq),
        static_cast<std::size_t>(bundle->nv),
        static_cast<std::size_t>(bundle->na),
        static_cast<std::size_t>(bundle->nu),
        static_cast<std::size_t>(bundle->ntendon));
    if (target_steps > max_steps_allowed) {
        ride->close();
        throw std::invalid_argument(
            "requested step count (" + std::to_string(target_steps) +
            ") exceeds step_closure.bin bundle cap (" +
            std::to_string(max_steps_allowed) + " steps / " +
            std::to_string(kMaxStepClosureBytes) + " bytes)");
    }

    bundle->rows.reserve(target_steps);
    for (std::size_t k = 0; k < target_steps; ++k) {
        StepClosureRow row;
        row.step_index = static_cast<std::int64_t>(k);
        row.dt_s = dt;

        validation::RuntimeRigAccess::inspect(
            *ride, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
                const mjModel *m = stepper.model();
                const mjData *d = stepper.data();
                row.time_s = d->time;
                row.qpos = copy_doubles(d->qpos, m->nq, "qpos");
                row.qvel = copy_doubles(d->qvel, m->nv, "qvel");
                row.act = copy_doubles(d->act, m->na, "act");
                row.qacc_warmstart =
                    copy_doubles(d->qacc_warmstart, m->nv, "qacc_warmstart");
            });

        const auto adv = ride->advance(static_cast<std::int64_t>(k) + 1,
                                       runtime::RideControl{}, 0.0, 0.0,
                                       std::nullopt);

        validation::RuntimeRigAccess::inspect(
            *ride, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
                const mjModel *m = stepper.model();
                const mjData *d = stepper.data();
                row.ctrl = copy_doubles(d->ctrl, m->nu, "ctrl");
                row.qfrc_applied =
                    copy_doubles(d->qfrc_applied, m->nv, "qfrc_applied");
                const auto dfl = model_access::readonly_buffer(
                    m->dof_frictionloss, m->nv, "dof_frictionloss");
                row.dof_frictionloss_wheels = {
                    dfl[static_cast<std::size_t>(bundle->front_wheel_dof)],
                    dfl[static_cast<std::size_t>(bundle->rear_wheel_dof)],
                };
                const auto wprm = model_access::readonly_buffer(
                    m->wrap_prm, m->nwrap, "wrap_prm");
                row.wrap_prm_freehub =
                    wprm[static_cast<std::size_t>(bundle->freehub_wrap_adr)];
                row.tendon_range = copy_doubles(
                    m->tendon_range, static_cast<mjtSize>(2) * m->ntendon,
                    "tendon_range");

                if (stepper.has_drive()) {
                    const int rear_teeth =
                        stepper.drive().state().shifting.rear_teeth;
                    row.cog_index = find_cog_index(bundle->cassette, rear_teeth);
                }

                row.ncon = d->ncon;
                row.nefc = d->nefc;
                row.solver_niter = d->solver_niter[0];
                row.next_qpos = copy_doubles(d->qpos, m->nq, "next_qpos");
                row.next_qvel = copy_doubles(d->qvel, m->nv, "next_qvel");
                row.next_qacc = copy_doubles(d->qacc, m->nv, "next_qacc");
            });

        ride->release_samples();
        bundle->rows.push_back(std::move(row));

        if (adv.outcome.has_value()) {
            break;
        }
    }

    ride->flush();
    const auto retained = ride->recorded_intervals();
    if (retained.size() != bundle->rows.size()) {
        throw std::runtime_error(
            "retained sample count (" + std::to_string(retained.size()) +
            ") disagrees with captured step count (" +
            std::to_string(bundle->rows.size()) + ")");
    }

    const auto nv = static_cast<std::size_t>(bundle->nv);
    for (std::size_t k = 0; k < bundle->rows.size(); ++k) {
        const auto &sample = *retained[k];
        auto &row = bundle->rows[k];
        row.qfrc_tires.assign(nv, 0.);
        row.qfrc_suspension.assign(nv, 0.);
        row.qfrc_resistance.assign(nv, 0.);
        row.qfrc_rider_joint_envelope.assign(nv, 0.);
        row.qfrc_brake_hold.assign(nv, 0.);
        row.qfrc_injected_other.assign(nv, 0.);

        for (const auto &[cname, cvec] : sample.forces) {
            if (cname == "tires") {
                accumulate_component(row.qfrc_tires, cvec);
            } else if (std::ranges::find(kSuspensionComponents, cname) !=
                       kSuspensionComponents.end()) {
                accumulate_component(row.qfrc_suspension, cvec);
            } else if (std::ranges::find(kResistanceComponents, cname) !=
                       kResistanceComponents.end()) {
                accumulate_component(row.qfrc_resistance, cvec);
            } else if (cname == "rider_joint_envelope") {
                accumulate_component(row.qfrc_rider_joint_envelope, cvec);
            } else if (cname == "brake_hold") {
                accumulate_component(row.qfrc_brake_hold, cvec);
            }
        }
        for (std::size_t j = 0; j < nv; ++j) {
            row.qfrc_injected_other[j] =
                row.qfrc_applied[j] -
                (row.qfrc_tires[j] + row.qfrc_suspension[j] +
                 row.qfrc_resistance[j] + row.qfrc_rider_joint_envelope[j] +
                 row.qfrc_brake_hold[j]);
        }
    }

    ride->close();
    return bundle;
}
NATIVE_DIAG_POP

void write_step_closure_bundle(const StepClosureBundle &bundle,
                               const std::filesystem::path &out_dir) {
    if (out_dir.empty()) {
        throw std::invalid_argument("bundle output directory cannot be empty");
    }
    if (bundle.nq <= 0 || bundle.nv <= 0 || bundle.ntendon <= 0 ||
        bundle.rows.empty() || bundle.model_mjb.empty()) {
        throw std::invalid_argument(
            "step closure bundle must have positive dimensions, >= 1 row, and non-empty model_mjb");
    }
    std::filesystem::create_directories(out_dir);

    const auto model_path = out_dir / "model.xml";
    const auto model12_path = out_dir / "model_12gears.xml";
    const auto model_mjb_path = out_dir / "model.mjb";
    const auto hfield_path = out_dir / "hfield_data.bin";
    const auto physics_path = out_dir / "physics_config.json";
    const auto trace_path = out_dir / "step_closure.bin";
    const auto manifest_path = out_dir / "manifest.json";

    write_text_file(model_path, bundle.model_xml);
    write_text_file(model12_path, bundle.model_12gears_xml);
    write_bytes_file(model_mjb_path, bundle.model_mjb);
    const std::string model_mjb_digest = io::sha256(bundle.model_mjb);
    write_bytes_file(hfield_path, std::as_bytes(std::span{bundle.hfield_data}));
    const std::string physics_text =
        io::canonical_json(bundle.physics_config_json) + "\n";
    write_text_file(physics_path, physics_text);

    const auto nq = static_cast<std::size_t>(bundle.nq);
    const auto nv = static_cast<std::size_t>(bundle.nv);
    const auto na = static_cast<std::size_t>(bundle.na);
    const auto nu = static_cast<std::size_t>(bundle.nu);
    const auto ntendon = static_cast<std::size_t>(bundle.ntendon);
    const std::size_t row_doubles = compute_row_doubles(nq, nv, na, nu, ntendon);
    const std::size_t max_steps_allowed =
        max_step_closure_steps(nq, nv, na, nu, ntendon);
    if (bundle.rows.size() > max_steps_allowed) {
        throw std::invalid_argument(
            "step closure bundle exceeds byte cap (" +
            std::to_string(kMaxStepClosureBytes) + " bytes)");
    }

    StepClosureFileHeader header{};
    header.n_steps = static_cast<std::uint64_t>(bundle.rows.size());
    header.row_doubles = static_cast<std::uint32_t>(row_doubles);
    header.nq = static_cast<std::uint32_t>(nq);
    header.nv = static_cast<std::uint32_t>(nv);
    header.na = static_cast<std::uint32_t>(na);
    header.nu = static_cast<std::uint32_t>(nu);
    header.ntendon = static_cast<std::uint32_t>(ntendon);
    header.front_wheel_dof = bundle.front_wheel_dof;
    header.rear_wheel_dof = bundle.rear_wheel_dof;
    header.freehub_wrap_adr = bundle.freehub_wrap_adr;
    header.freehub_tendon_id = bundle.freehub_tendon_id;
    header.clutch_tendon_id = bundle.clutch_tendon_id;

    std::vector<double> flat;
    flat.reserve(bundle.rows.size() * row_doubles);
    for (const auto &row : bundle.rows) {
        flat.push_back(static_cast<double>(row.step_index));
        flat.push_back(row.time_s);
        flat.push_back(row.dt_s);
        append_doubles(flat, row.qpos);
        append_doubles(flat, row.qvel);
        append_doubles(flat, row.act);
        append_doubles(flat, row.qacc_warmstart);
        append_doubles(flat, row.ctrl);
        append_doubles(flat, row.qfrc_applied);
        append_doubles(flat, row.qfrc_tires);
        append_doubles(flat, row.qfrc_suspension);
        append_doubles(flat, row.qfrc_resistance);
        append_doubles(flat, row.qfrc_rider_joint_envelope);
        append_doubles(flat, row.qfrc_brake_hold);
        append_doubles(flat, row.qfrc_injected_other);
        append_doubles(flat, row.dof_frictionloss_wheels);
        flat.push_back(row.wrap_prm_freehub);
        append_doubles(flat, row.tendon_range);
        flat.push_back(static_cast<double>(row.cog_index));
        flat.push_back(static_cast<double>(row.ncon));
        flat.push_back(static_cast<double>(row.nefc));
        flat.push_back(static_cast<double>(row.solver_niter));
        append_doubles(flat, row.next_qpos);
        append_doubles(flat, row.next_qvel);
        append_doubles(flat, row.next_qacc);
    }

    const auto header_bytes = std::as_bytes(std::span{&header, 1});
    const auto payload_bytes = std::as_bytes(std::span{flat});
    write_binary_parts(trace_path, {header_bytes, payload_bytes});
    const std::string step_closure_digest =
        sha256_parts({header_bytes, payload_bytes});

    io::Value::Array layout_json;
    for (const char *col : {
             "step_index",
             "time_s",
             "dt_s",
             "qpos[nq]",
             "qvel[nv]",
             "act[na]",
             "qacc_warmstart[nv]",
             "ctrl[nu]",
             "qfrc_applied[nv]",
             "qfrc_tires[nv]",
             "qfrc_suspension[nv]",
             "qfrc_resistance[nv]",
             "qfrc_rider_joint_envelope[nv]",
             "qfrc_brake_hold[nv]",
             "qfrc_injected_other[nv]",
             "dof_frictionloss_wheels[2]",
             "wrap_prm_freehub",
             "tendon_range[2*ntendon]",
             "cog_index",
             "ncon",
             "nefc",
             "solver_niter",
             "next_qpos[nq]",
             "next_qvel[nv]",
             "next_qacc[nv]",
         }) {
        layout_json.emplace_back(std::string(col));
    }

    io::Value::Array cassette_json;
    for (const int teeth : bundle.cassette) {
        cassette_json.push_back(int_value(teeth));
    }

    io::Value::Object manifest{
        {"schema", io::Value{std::string("bike.step_closure_bundle.v1")}},
        {"steps", int_value(static_cast<std::int64_t>(bundle.rows.size()))},
        {"dt_s", double_value(bundle.dt_s)},
        {"nq", int_value(bundle.nq)},
        {"nv", int_value(bundle.nv)},
        {"na", int_value(bundle.na)},
        {"nu", int_value(bundle.nu)},
        {"ntendon", int_value(bundle.ntendon)},
        {"row_doubles", int_value(static_cast<std::int64_t>(row_doubles))},
        {"header_bytes", int_value(sizeof(StepClosureFileHeader))},
        {"front_wheel_dof", int_value(bundle.front_wheel_dof)},
        {"rear_wheel_dof", int_value(bundle.rear_wheel_dof)},
        {"freehub_wrap_adr", int_value(bundle.freehub_wrap_adr)},
        {"freehub_tendon_id", int_value(bundle.freehub_tendon_id)},
        {"clutch_tendon_id", int_value(bundle.clutch_tendon_id)},
        {"front_teeth", int_value(bundle.front_teeth)},
        {"cassette", io::Value{std::move(cassette_json)}},
        {"nhfielddata",
         int_value(static_cast<std::int64_t>(bundle.hfield_data.size()))},
        {"physics_config", io::Value{bundle.physics_config_label}},
        {"track", io::Value{bundle.track_label}},
        {"model_sha256",
         io::Value{io::sha256(std::as_bytes(
             std::span<const char>{bundle.model_xml}))}},
        {"model_12gears_sha256",
         io::Value{io::sha256(std::as_bytes(
             std::span<const char>{bundle.model_12gears_xml}))}},
        {"model_mjb_sha256", io::Value{model_mjb_digest}},
        {"hfield_sha256",
         io::Value{io::sha256(std::as_bytes(std::span{bundle.hfield_data}))}},
        {"step_closure_sha256", io::Value{step_closure_digest}},
        {"layout", io::Value{std::move(layout_json)}},
    };
    if (bundle.resolved_bike.has_value()) {
        manifest.emplace_back(
            "resolved_bike",
            model::bike_specs_value(*bundle.resolved_bike));
    }
    if (bundle.track_spec.has_value()) {
        io::Value track_val;
        try {
            track_val = terrain::track_to_dict(*bundle.track_spec);
        } catch (const terrain::TrackFileError &) {
            track_val = terrain::identity_value(*bundle.track_spec);
        }
        manifest.emplace_back("track_spec", std::move(track_val));
    }

    const std::string manifest_text =
        io::canonical_json(io::Value{std::move(manifest)}) + "\n";
    write_text_file(manifest_path, manifest_text);
}

std::unique_ptr<StepClosureBundle> load_step_closure_bundle(
    const std::filesystem::path &bundle_dir) {
    auto bundle = std::make_unique<StepClosureBundle>();
    bundle->model_xml = read_text_file(bundle_dir / "model.xml");
    bundle->model_12gears_xml = read_text_file(bundle_dir / "model_12gears.xml");
    bundle->physics_config_json =
        io::parse_json(read_text_file(bundle_dir / "physics_config.json"));

    const auto manifest =
        io::parse_json(read_text_file(bundle_dir / "manifest.json"));
    const auto &mobj = io::as_object(manifest, "manifest");
    if (const auto *v = io::find_member(mobj, "physics_config")) {
        bundle->physics_config_label = io::as_string(*v, "physics_config");
    }
    if (const auto *v = io::find_member(mobj, "track")) {
        bundle->track_label = io::as_string(*v, "track");
    }
    if (const auto *v = io::find_member(mobj, "dt_s")) {
        bundle->dt_s = io::as_double(*v, "dt_s");
    }
    if (const auto *v = io::find_member(mobj, "front_teeth")) {
        bundle->front_teeth =
            static_cast<int>(io::as_int64(*v, "front_teeth"));
    }
    if (const auto *v = io::find_member(mobj, "cassette")) {
        for (const auto &item : io::as_array(*v, "cassette")) {
            bundle->cassette.push_back(
                static_cast<int>(io::as_int64(item, "cassette[]")));
        }
    }
    if (const auto *v = io::find_member(mobj, "resolved_bike")) {
        const auto &robj = io::as_object(*v, "resolved_bike");
        geometry::BikeSpecs bike{};
        const auto read_double = [&](std::string_view key, double &dst) {
            if (const auto *m = io::find_member(robj, key)) {
                dst = io::as_double(*m, key);
            }
        };
        const auto read_int = [&](std::string_view key, int &dst) {
            if (const auto *m = io::find_member(robj, key)) {
                dst = static_cast<int>(io::as_int64(*m, key));
            }
        };
        read_double("reach", bike.reach);
        read_double("stack", bike.stack);
        read_double("head_angle_deg", bike.head_angle_deg);
        read_double("effective_seat_angle_deg", bike.effective_seat_angle_deg);
        read_double("bb_drop", bike.bb_drop);
        read_double("wheelbase", bike.wheelbase);
        read_double("headtube_length", bike.headtube_length);
        read_double("front_wheel_radius", bike.front_wheel_radius);
        read_double("rear_wheel_radius", bike.rear_wheel_radius);
        read_double("wheel_radius", bike.wheel_radius);
        read_double("crank_length", bike.crank_length);
        read_double("fork_travel", bike.fork_travel);
        read_double("fork_offset", bike.fork_offset);
        read_double("rear_wheel_travel", bike.rear_wheel_travel);
        read_double("shock_eye_to_eye", bike.shock_eye_to_eye);
        read_double("shock_stroke", bike.shock_stroke);
        read_double("fork_stiffness", bike.fork_stiffness);
        read_double("fork_damping", bike.fork_damping);
        read_double("shock_stiffness", bike.shock_stiffness);
        read_double("shock_damping", bike.shock_damping);
        read_int("fork_hsc", bike.fork_hsc);
        read_int("fork_lsc", bike.fork_lsc);
        read_int("fork_rebound", bike.fork_rebound);
        read_double("fork_initial_psi", bike.fork_initial_psi);
        read_int("fork_air_tokens", bike.fork_air_tokens);
        read_int("shock_hsc", bike.shock_hsc);
        read_int("shock_lsc", bike.shock_lsc);
        read_int("shock_rebound", bike.shock_rebound);
        read_int("shock_hbo", bike.shock_hbo);
        if (const auto *m = io::find_member(robj, "shock_lockout")) {
            if (!std::holds_alternative<bool>(m->data)) {
                throw std::invalid_argument(
                    "resolved_bike.shock_lockout must be boolean");
            }
            bike.shock_lockout = std::get<bool>(m->data);
        }
        bundle->resolved_bike = std::move(bike);
    }
    if (const auto *v = io::find_member(mobj, "track_spec")) {
        const auto &tobj = io::as_object(*v, "track_spec");
        if (io::find_member(tobj, "native_track_identity_schema") == nullptr) {
            bundle->track_spec = terrain::track_from_dict(*v);
        }
    }
    const auto model_mjb_path = bundle_dir / "model.mjb";
    const auto *mjb_sha_val = io::find_member(mobj, "model_mjb_sha256");
    if (std::filesystem::exists(model_mjb_path)) {
        bundle->model_mjb =
            read_bounded_bytes(model_mjb_path, kMaxStepClosureBytes);
        bundle->model_mjb_sha256 = io::sha256(bundle->model_mjb);
        if (mjb_sha_val != nullptr &&
            io::as_string(*mjb_sha_val, "model_mjb_sha256") !=
                bundle->model_mjb_sha256) {
            throw std::invalid_argument(
                "step_closure_bundle: model.mjb sha256 mismatch");
        }
    } else if (mjb_sha_val != nullptr) {
        throw std::invalid_argument(
            "step_closure_bundle: manifest declares model_mjb_sha256 but model.mjb is missing");
    }

    const auto hf_bytes = io::read_numpy_bytes(bundle_dir / "hfield_data.bin");
    bundle->hfield_data = decode_doubles(hf_bytes);

    const auto trace_bytes = read_bounded_bytes(
        bundle_dir / "step_closure.bin", kMaxStepClosureBytes);
    if (trace_bytes.size() < sizeof(StepClosureFileHeader)) {
        throw std::invalid_argument("step_closure.bin shorter than header");
    }
    const auto hdr = decode_pod<StepClosureFileHeader>(
        std::span<const std::byte>{trace_bytes}
            .first(sizeof(StepClosureFileHeader)));
    if (hdr.magic != kStepClosureMagic || hdr.version != kStepClosureVersion ||
        hdr.header_bytes != sizeof(StepClosureFileHeader)) {
        throw std::invalid_argument("step_closure.bin header magic/version mismatch");
    }
    if (hdr.nq == 0 || hdr.nv == 0 || hdr.ntendon == 0 || hdr.n_steps == 0) {
        throw std::invalid_argument(
            "step_closure.bin header requires positive dimensions and >= 1 step");
    }
    constexpr auto kMaxInt =
        static_cast<std::uint32_t>(std::numeric_limits<int>::max());
    if (hdr.nq > kMaxInt || hdr.nv > kMaxInt || hdr.na > kMaxInt ||
        hdr.nu > kMaxInt || hdr.ntendon > kMaxInt) {
        throw std::invalid_argument(
            "step_closure.bin header dimensions exceed int range");
    }

    bundle->nq = static_cast<int>(hdr.nq);
    bundle->nv = static_cast<int>(hdr.nv);
    bundle->na = static_cast<int>(hdr.na);
    bundle->nu = static_cast<int>(hdr.nu);
    bundle->ntendon = static_cast<int>(hdr.ntendon);
    bundle->front_wheel_dof = hdr.front_wheel_dof;
    bundle->rear_wheel_dof = hdr.rear_wheel_dof;
    bundle->freehub_wrap_adr = hdr.freehub_wrap_adr;
    bundle->freehub_tendon_id = hdr.freehub_tendon_id;
    bundle->clutch_tendon_id = hdr.clutch_tendon_id;
    validate_header_indices({.nv = bundle->nv, .ntendon = bundle->ntendon},
                            *bundle);

    const auto nq = static_cast<std::size_t>(hdr.nq);
    const auto nv = static_cast<std::size_t>(hdr.nv);
    const auto na = static_cast<std::size_t>(hdr.na);
    const auto nu = static_cast<std::size_t>(hdr.nu);
    const auto ntendon = static_cast<std::size_t>(hdr.ntendon);
    const auto row_doubles = static_cast<std::size_t>(hdr.row_doubles);
    if (row_doubles != compute_row_doubles(nq, nv, na, nu, ntendon)) {
        throw std::invalid_argument("step_closure.bin row_doubles mismatch");
    }
    const std::size_t max_steps_allowed =
        max_step_closure_steps(nq, nv, na, nu, ntendon);
    if (max_steps_allowed == 0 || hdr.n_steps > max_steps_allowed) {
        throw std::invalid_argument(
            "step_closure.bin step count exceeds bundle byte cap");
    }

    const std::size_t expected_payload =
        static_cast<std::size_t>(hdr.n_steps) * row_doubles * sizeof(double);
    if (trace_bytes.size() != sizeof(StepClosureFileHeader) + expected_payload) {
        throw std::invalid_argument("step_closure.bin byte size mismatch");
    }

    const std::vector<double> flat = decode_doubles(
        std::span<const std::byte>{trace_bytes}
            .subspan(sizeof(StepClosureFileHeader)));

    bundle->rows.reserve(static_cast<std::size_t>(hdr.n_steps));
    for (std::size_t k = 0; k < static_cast<std::size_t>(hdr.n_steps); ++k) {
        const auto row_span = std::span<const double>{flat}.subspan(
            k * row_doubles, row_doubles);
        std::size_t off = 0;
        StepClosureRow row;
        row.step_index = static_cast<std::int64_t>(row_span[off++]);
        row.time_s = row_span[off++];
        row.dt_s = row_span[off++];
        read_doubles(row_span, off, row.qpos, nq);
        read_doubles(row_span, off, row.qvel, nv);
        read_doubles(row_span, off, row.act, na);
        read_doubles(row_span, off, row.qacc_warmstart, nv);
        read_doubles(row_span, off, row.ctrl, nu);
        read_doubles(row_span, off, row.qfrc_applied, nv);
        read_doubles(row_span, off, row.qfrc_tires, nv);
        read_doubles(row_span, off, row.qfrc_suspension, nv);
        read_doubles(row_span, off, row.qfrc_resistance, nv);
        read_doubles(row_span, off, row.qfrc_rider_joint_envelope, nv);
        read_doubles(row_span, off, row.qfrc_brake_hold, nv);
        read_doubles(row_span, off, row.qfrc_injected_other, nv);
        row.dof_frictionloss_wheels[0] = row_span[off++];
        row.dof_frictionloss_wheels[1] = row_span[off++];
        row.wrap_prm_freehub = row_span[off++];
        read_doubles(row_span, off, row.tendon_range, 2U * ntendon);
        row.cog_index = static_cast<int>(row_span[off++]);
        row.ncon = static_cast<int>(row_span[off++]);
        row.nefc = static_cast<int>(row_span[off++]);
        row.solver_niter = static_cast<int>(row_span[off++]);
        read_doubles(row_span, off, row.next_qpos, nq);
        read_doubles(row_span, off, row.next_qvel, nv);
        read_doubles(row_span, off, row.next_qacc, nv);
        if (off != row_doubles) {
            throw std::runtime_error("step_closure.bin decode offset mismatch");
        }
        bundle->rows.push_back(std::move(row));
    }

    return bundle;
}

ClosureVerificationResult verify_bundle_closure(
    const std::filesystem::path &bundle_dir) {
    const auto bundle = load_step_closure_bundle(bundle_dir);
    auto m = model::compile_mjcf(bundle->model_xml, bundle_dir);
    if (m->nq != bundle->nq || m->nv != bundle->nv || m->na != bundle->na ||
        m->nu != bundle->nu || m->ntendon != bundle->ntendon) {
        throw std::invalid_argument(
            "model.xml dimensions differ from step_closure.bin");
    }
    validate_header_indices({.nv = m->nv, .ntendon = m->ntendon}, *bundle);
    if (bundle->freehub_wrap_adr >= m->nwrap) {
        throw std::invalid_argument(
            "step_closure.bin freehub_wrap_adr out of [0, nwrap) range");
    }
    assign_hfield_from_doubles(*m, bundle->hfield_data);
    DataOwner d(engine::make_data(m.get()));
    const DataOwner d_const(engine::make_data(m.get()));

    auto m12 = model::compile_mjcf(bundle->model_12gears_xml, bundle_dir);
    if (m12->nq != bundle->nq || m12->nv != bundle->nv ||
        m12->na != bundle->na || m12->nu != bundle->nu) {
        throw std::invalid_argument(
            "model_12gears.xml dimensions differ from step_closure.bin");
    }
    assign_hfield_from_doubles(*m12, bundle->hfield_data);
    DataOwner d12(engine::make_data(m12.get()));
    const int clutch_tid_12 = mj_name2id(m12.get(), mjOBJ_TENDON, "crank_clutch");
    if (clutch_tid_12 >= 0 && bundle->clutch_tendon_id < 0) {
        throw std::invalid_argument(
            "step_closure.bin missing clutch_tendon_id for crank_clutch model");
    }

    ClosureVerificationResult result;
    result.total_steps = bundle->rows.size();

    const auto fw_idx = static_cast<std::size_t>(bundle->front_wheel_dof);
    const auto rw_idx = static_cast<std::size_t>(bundle->rear_wheel_dof);
    const auto w_idx = static_cast<std::size_t>(bundle->freehub_wrap_adr);

    for (std::size_t k = 0; k < bundle->rows.size(); ++k) {
        const auto &row = bundle->rows[k];

        const auto dfl = model_access::mutable_buffer(m->dof_frictionloss, m->nv,
                                                      "dof_frictionloss");
        dfl[fw_idx] = row.dof_frictionloss_wheels[0];
        dfl[rw_idx] = row.dof_frictionloss_wheels[1];

        const auto wprm =
            model_access::mutable_buffer(m->wrap_prm, m->nwrap, "wrap_prm");
        if (wprm[w_idx] != row.wrap_prm_freehub) {
            wprm[w_idx] = row.wrap_prm_freehub;
            engine::set_const(m.get(), d_const.get());
        }

        std::ranges::copy(
            row.tendon_range,
            model_access::mutable_buffer(m->tendon_range,
                                         static_cast<mjtSize>(2) * m->ntendon,
                                         "tendon_range")
                .begin());

        restore_cold_data(*m, *d, row);
        engine::step(m.get(), d.get());

        const auto cold_qpos =
            model_access::readonly_buffer(d->qpos, m->nq, "cold qpos");
        const auto cold_qvel =
            model_access::readonly_buffer(d->qvel, m->nv, "cold qvel");
        const bool qpos_ok = bitwise_equal(cold_qpos, row.next_qpos);
        const bool qvel_ok = bitwise_equal(cold_qvel, row.next_qvel);
        if (qpos_ok) {
            ++result.zero_bit_qpos_steps;
        }
        if (qvel_ok) {
            ++result.zero_bit_qvel_steps;
        }
        if ((!qpos_ok || !qvel_ok) && result.first_fail_step < 0) {
            result.first_fail_step = static_cast<int>(k);
        }
        result.max_qpos_err =
            std::max(result.max_qpos_err, max_abs_diff(cold_qpos, row.next_qpos));
        result.max_qvel_err =
            std::max(result.max_qvel_err, max_abs_diff(cold_qvel, row.next_qvel));

        engine::forward(m.get(), d.get());
        const auto cold_qacc =
            model_access::readonly_buffer(d->qacc, m->nv, "cold qacc");
        if (bitwise_equal(cold_qacc, row.next_qacc)) {
            ++result.zero_bit_qacc_steps;
        }
        result.max_qacc_err =
            std::max(result.max_qacc_err, max_abs_diff(cold_qacc, row.next_qacc));

        if (row.cog_index >= 0 &&
            static_cast<std::size_t>(row.cog_index) < bundle->cassette.size()) {
            const double steady_ratio =
                static_cast<double>(bundle->front_teeth) /
                static_cast<double>(
                    bundle->cassette[static_cast<std::size_t>(row.cog_index)]);
            if (std::abs(row.wrap_prm_freehub - steady_ratio) <= 1e-14) {
                const auto dfl12 = model_access::mutable_buffer(
                    m12->dof_frictionloss, m12->nv, "m12 dof_frictionloss");
                dfl12[fw_idx] = row.dof_frictionloss_wheels[0];
                dfl12[rw_idx] = row.dof_frictionloss_wheels[1];
                patch_12gears_tendon_range(*m12, *bundle, row, clutch_tid_12);
                restore_cold_data(*m12, *d12, row);
                engine::step(m12.get(), d12.get());
                const auto q12 = model_access::readonly_buffer(
                    d12->qpos, m12->nq, "m12 qpos");
                const auto v12 = model_access::readonly_buffer(
                    d12->qvel, m12->nv, "m12 qvel");
                result.steady_gear_max_qpos_diff =
                    std::max(result.steady_gear_max_qpos_diff,
                             max_abs_diff(q12, row.next_qpos));
                result.steady_gear_max_qvel_diff =
                    std::max(result.steady_gear_max_qvel_diff,
                             max_abs_diff(v12, row.next_qvel));
                ++result.steady_gear_checked;
            }
        }
    }

    return result;
}

} // namespace bike::golden
