#include "playback.hpp"
#include "rebuild.hpp"
#include "input_detail.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../initialization/capture.hpp"
#include "../spatial/frames.hpp"
#include "../spatial/topology.hpp"
#include "../terrain/profile.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <utility>

namespace bike::replay {
namespace {
using namespace detail;
constexpr std::size_t maximum_state_values = std::size_t{64} * 1024 * 1024;
struct DataDeleter { void operator()(mjData* data) const noexcept { mj_deleteData(data); } };
using DataOwner = std::unique_ptr<mjData, DataDeleter>;

void append(PlaybackData& output, const Value& row, std::size_t& stored_values) {
    auto q = double_vector(field(row, "qpos"), "playback.qpos");
    auto v = double_vector(field(row, "qvel"), "playback.qvel");
    if (q.size() > maximum_state_values - stored_values || v.size() > maximum_state_values - stored_values - q.size())
        throw recording::RecordingError("playback saved-state memory limit exceeded");
    stored_values += q.size() + v.size();
    output.times_s.push_back(io::as_double(field(row, "time_s"), "playback.time_s"));
    output.ends_s.push_back(io::as_double(field(row, "interval_end_s"), "playback.interval_end_s"));
    output.qpos.push_back(std::move(q)); output.qvel.push_back(std::move(v));
}
runtime::SpatialPresentation saved_spatial(const Value& row) {
    runtime::SpatialPresentation out;
    const auto vector = [&](std::string_view key, auto& destination) {
        const auto values = double_vector(field(row,key),key);
        if(values.size()!=destination.size() || !std::ranges::all_of(values,[](double x){return std::isfinite(x);}))
            throw recording::RecordingError("stored spatial presentation vector has invalid width/value");
        std::ranges::copy(values,destination.begin());
    };
    vector("root_position_world_m",out.position_world_m);
    vector("root_quat_world_from_body_wxyz",out.quat_wxyz);
    out.attitude_valid=io::as_bool(field(row,"attitude_valid"),"attitude_valid");
    if(out.attitude_valid) {
        out.heading_rad=io::as_double(field(row,"heading_rad"),"heading_rad");
        out.roll_rad=io::as_double(field(row,"roll_rad"),"roll_rad");
    }
    out.steer_angle_rad=io::as_double(field(row,"steer_angle_rad"),"steer_angle_rad");
    out.steer_torque_nm=io::as_double(field(row,"steer_torque_nm"),"steer_torque_nm");
    for(std::size_t i=0;i<2;++i) {
        const std::string side=i==0?"front":"rear";
        out.contact_valid[i]=io::as_bool(field(row,side+"_contact_valid"),"contact_valid");
        if(out.contact_valid[i])out.lateral_slip_mps[i]=io::as_double(field(row,side+"_lateral_slip_mps"),"lateral_slip_mps");
    }
    // Scored container2 checkpoints require the bootstrap hold to be removed.
    out.hold_active=false;
    return out;
}
runtime::ModelImage reconstruct_model(const ReplayInput& input) {
    FixedAssets const assets(input);
    auto request = request_from_input(input, assets);
    // Recreate visual topology from the immutable recipe. This is model
    // compilation only: no settle(), controller construction or stepping.
    auto model = model::compile_mjcf(model::generate_mjcf(request->setup.model), assets.path());
    const int road = mj_name2id(model.get(), mjOBJ_HFIELD, "road");
    if (road < 0) throw recording::RecordingError("playback model lacks recorded road heightfield");
    const auto heights = terrain::build_field_data(request->setup.track, request->setup.model.field);
    const auto road_index=static_cast<std::size_t>(road);
    const auto rows=model_access::readonly_buffer(model->hfield_nrow,model->nhfield);
    const auto cols=model_access::readonly_buffer(model->hfield_ncol,model->nhfield);
    if(rows[road_index]<2 || cols[road_index]<2)throw recording::RecordingError("playback heightfield needs at least two rows and columns");
    auto const raster=model_access::mutable_buffer(model->hfield_data,model->nhfielddata);
    const auto count = model_access::checked_product(static_cast<std::size_t>(rows[road_index]),static_cast<std::size_t>(cols[road_index]),raster.size());
    if (heights.size() != count) throw recording::RecordingError("playback heightfield dimensions differ from recipe");
    const auto address = model_access::readonly_buffer(model->hfield_adr,model->nhfield)[road_index];
    if (address < 0 || address > model->nhfielddata || std::cmp_greater(count ,model->nhfielddata - address))
        throw recording::RecordingError("playback road address is outside heightfield storage");
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(heights[i]) || heights[i] < 0. || heights[i] > 1.) throw recording::RecordingError("playback normalized road height is outside [0,1]");
        raster[static_cast<std::size_t>(address) + i] = static_cast<float>(heights[i]);
    }
    return initialization::capture_model_image(*model);
}
std::pair<std::size_t, double> bracket(const PlaybackData& data, double time_s) {
    if (!std::isfinite(time_s)) throw std::invalid_argument("pose_at requires a finite time");
    if (data.times_s.empty()) throw std::invalid_argument("playback has no recorded frames");
    const double time = std::clamp(time_s, 0., data.duration_s);
    const auto after = std::ranges::upper_bound(data.times_s, time);
    if (after == data.times_s.begin()) return {0, 0.};
    const auto index = static_cast<std::size_t>(after - data.times_s.begin() - 1);
    if (index + 1 == data.times_s.size()) return {index, 0.};
    return {index, (time - data.times_s[index]) / (data.times_s[index + 1] - data.times_s[index])};
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
std::vector<double> interpolate(const std::vector<std::vector<double>>& rows, std::size_t index, double weight) {
    auto value = rows.at(index);
    if (weight == 0.) return value;
    const auto& next = rows.at(index + 1);
    if (next.size() != value.size()) throw std::invalid_argument("playback frame widths disagree");
    for (std::size_t i = 0; i < value.size(); ++i) value[i] += (next[i] - value[i]) * weight;
    return value;
}
int joint(const mjModel& model, const char* name) { return mj_name2id(&model, mjOBJ_JOINT, name); }
double position(const mjModel& model, const mjData& data, const char* name) {
    const int id = joint(model, name);if(id<0)return 0.;
    const auto address=model_access::readonly_buffer(model.jnt_qposadr,model.njnt)[static_cast<std::size_t>(id)];
    model_access::require_id(address,model.nq,"playback position joint");
    return model_access::readonly_buffer(data.qpos,model.nq)[static_cast<std::size_t>(address)];
}
double velocity(const mjModel& model, const mjData& data, const char* name) {
    const int id = joint(model, name);if(id<0)return 0.;
    const auto address=model_access::readonly_buffer(model.jnt_dofadr,model.njnt)[static_cast<std::size_t>(id)];
    model_access::require_id(address,model.nv,"playback velocity joint");
    return model_access::readonly_buffer(data.qvel,model.nv)[static_cast<std::size_t>(address)];
}
bike::spatial::BodyPose presentation_pose(const mjModel& model, mjData& data) {
    const int frame = mj_name2id(&model, mjOBJ_BODY, "frame");
    model_access::require_id(frame, model.nbody, "playback spatial frame");
    // Root body geometry and velocity come from the interpolated saved
    // coordinates for both free6 and constrained4. This updates only the
    // kinematic caches needed by XBODY velocity, never dynamics or sensors.
    struct Refresh {
        const mjModel* model = nullptr;
        mjData* data = nullptr;
        bike::spatial::RootBinding root;
        bike::spatial::BodyPose pose;
    } refresh{.model=&model, .data=&data, .root={.body_id=frame}, .pose={}};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* opaque) noexcept {
        auto& r = *static_cast<Refresh*>(opaque);
        mj_kinematics(r.model, r.data);
        mj_comPos(r.model, r.data);
        mj_comVel(r.model, r.data);
        r.pose = bike::spatial::body_pose(*r.model, *r.data, r.root);
    }, &refresh, error)) engine::throw_failure(error);
    return refresh.pose;
}
} // namespace

void validate_playback(const PlaybackData& data, std::size_t nq, std::size_t nv) {
    const auto size = data.times_s.size();
    if (size < 2) throw recording::RecordingError("stored-state playback needs at least two recorded frames");
    if (data.ends_s.size() != size || data.qpos.size() != size || data.qvel.size() != size)
        throw recording::RecordingError("playback arrays have inconsistent frame counts");
    if(!data.spatial.empty() && data.spatial.size()!=size)
        throw recording::RecordingError("stored spatial truth and physical frame counts differ");
    for (std::size_t i = 0; i < size; ++i) {
        const auto label = "frame " + std::to_string(i);
        if (data.qpos[i].size() != nq || data.qvel[i].size() != nv) throw recording::RecordingError(label + ": recorded state dimensions differ from model");
        if (!std::isfinite(data.times_s[i]) || !std::isfinite(data.ends_s[i]) || data.times_s[i] < 0. || data.ends_s[i] <= data.times_s[i])
            throw recording::RecordingError(label + ": interval_end_s must exceed finite nonnegative time_s");
        if (i && data.times_s[i] <= data.times_s[i-1]) throw recording::RecordingError(label + ": time_s is not strictly increasing");
        for (const auto& values : {std::cref(data.qpos[i]), std::cref(data.qvel[i])})
            if (!std::ranges::all_of(values.get(), [](double value) { return std::isfinite(value); })) throw recording::RecordingError(label + ": nonfinite state");
        for (const auto start : data.quaternion_starts) {
            if (start > nq || nq - start < 4) throw recording::RecordingError("playback quaternion block is outside qpos");
            double norm = 0.; for (std::size_t j = 0; j < 4; ++j) norm = std::hypot(norm, data.qpos[i][start + j]);
            if (!(norm > 0.)) throw recording::RecordingError(label + ": zero quaternion");
        }
    }
    if (!std::isfinite(data.duration_s) || data.duration_s != data.ends_s.back() || data.duration_s <= data.times_s.back())
        throw recording::RecordingError("playback duration differs from last saved interval end");
}

PlaybackData load_playback(const std::filesystem::path& path, std::optional<std::int64_t> episode) {
    PlaybackData output; std::size_t values = 0;
    const bool container = std::filesystem::exists(path / "manifest.json") || std::filesystem::exists(path / "run.duckdb") ||
        std::filesystem::exists(path / "UNSEALED") || std::filesystem::is_symlink(path / "manifest.json") ||
        std::filesystem::is_symlink(path / "run.duckdb") || std::filesystem::is_symlink(path / "UNSEALED");
    if (container) {
        const recording::Recording reader(path); const auto selected = reader.episode(episode);
        output.episode_id = count(field(selected, "episode_id"));
        output.summary = io::parse_recording_json(string(field(selected, "summary_json")));
        ReplayInput input; input.summary = output.summary; input.episode_id = output.episode_id;
        const std::array params{integer(output.episode_id)};
        for (const auto& row : reader.rows("SELECT name,content FROM assets WHERE episode_id=? ORDER BY name", params))
            input.assets.emplace(string(field(row, "name")), std::get<io::Bytes>(field(row, "content").data).value);
        if (const auto image = input.assets.find("model.mjb"); image != input.assets.end())
            output.model = {.mjb=image->second, .sha256=io::sha256(image->second)};
        else {
            // Stored poses remain viewable for interrupted/unqualified runs.
            // Their container and asset hashes still validate through reader.
            input.manifest = io::parse_recording_json(asset_text(input, "recipe.json"));
            if (count(field(input.manifest, "replay_contract_version")) != 3 || string(field(input.manifest, "state_signature")) != "mjSTATE_INTEGRATION")
                throw recording::RecordingError("unsupported stored-state playback recipe");
            output.model = reconstruct_model(input);
        }
        auto cursor = reader.stream("SELECT time_s,interval_end_s,qpos,qvel FROM physics WHERE episode_id=? ORDER BY interval_id", params);
        for (;;) { const auto batch = cursor.next_batch(); if (batch.empty()) break; for (const auto& row : batch) append(output, Value{row}, values); }
        if(reader.schema_version()==2) {
            auto truth=reader.stream("SELECT * FROM channel_spatial WHERE episode_id=? ORDER BY interval_id",params);
            for(;;){const auto batch=truth.next_batch();if(batch.empty())break;
                for(const auto& row:batch)output.spatial.push_back(saved_spatial(Value{row}));}
        }
    } else {
        const auto input = load_legacy_input(path, episode); output.summary = input.summary;
        output.model = reconstruct_model(input);
        if (const auto intervals = input.assets.find("intervals.jsonl"); intervals != input.assets.end()) {
            for (const auto& row : json_lines(asset_text(input, "intervals.jsonl"))) append(output, row, values);
        } else {
            // Schema 1 never hashed intervals. Preserve that distinction and
            // read only the fixed local filename; this is display, not proof.
            output.intervals_hash_verified = false;
            for (const auto& row : json_lines(read_text(path / "intervals.jsonl"))) append(output, row, values);
        }
    }
    auto model = make_playback_model(output);
    const auto types=model_access::readonly_buffer(model->jnt_type,model->njnt);
    const auto addresses=model_access::readonly_buffer(model->jnt_qposadr,model->njnt);
    for (std::size_t j = 0; j < types.size(); ++j) {
        model_access::require_id(addresses[j],model->nq,"playback joint position address");
        const auto address = static_cast<std::size_t>(addresses[j]);
        if (types[j] == mjJNT_FREE) output.quaternion_starts.push_back(address + 3);
        else if (types[j] == mjJNT_BALL) output.quaternion_starts.push_back(address);
    }
    if (!output.ends_s.empty()) output.duration_s = output.ends_s.back();
    validate_playback(output, static_cast<std::size_t>(model->nq), static_cast<std::size_t>(model->nv));
    return output;
}

std::vector<double> pose_at(const PlaybackData& data, double time_s) {
    const auto [index, weight] = bracket(data, time_s);
    auto pose = interpolate(data.qpos, index, weight);
    for (const auto start : data.quaternion_starts) {
        if (start > pose.size() || pose.size() - start < 4) throw std::invalid_argument("playback quaternion block is outside qpos");
        // Preserve source linear interpolation. Antipodal endpoint signs
        // represent the same rotation; align their hemisphere to avoid the
        // source's zero-norm midpoint while leaving other interpolation intact.
        if (weight != 0.) {
            double dot = 0.; for (std::size_t i = 0; i < 4; ++i) dot += data.qpos[index][start+i] * data.qpos[index+1][start+i];
            if (dot < 0.) for (std::size_t i = 0; i < 4; ++i)
                pose[start+i] = (1. - weight) * data.qpos[index][start+i] - weight * data.qpos[index+1][start+i];
        }
        double norm = 0.; for (std::size_t i = 0; i < 4; ++i) norm = std::hypot(norm, pose[start+i]);
        if (!(norm > 0.) || !std::isfinite(norm)) throw recording::RecordingError("playback quaternion cannot be normalized");
        for (std::size_t i = 0; i < 4; ++i) pose[start+i] /= norm;
    }
    return pose;
}
model::ModelOwner make_playback_model(const PlaybackData& data) {
    if (data.model.mjb.empty() || io::sha256(data.model.mjb) != data.model.sha256) throw recording::RecordingError("playback model image checksum mismatch");
    model::ModelOwner owner{engine::load_model_image(data.model.mjb)};
    if (!owner) throw recording::RecordingError("cannot load playback model image");
    return owner;
}

struct PlaybackSession::Impl {
    PlaybackData evidence;
    model::ModelOwner model;
    DataOwner data;
    explicit Impl(PlaybackData input) : evidence(std::move(input)), model(make_playback_model(evidence)), data(engine::make_data(model.get())) {
        if (!data) throw recording::RecordingError("cannot allocate playback presentation state");
        validate_playback(evidence, static_cast<std::size_t>(model->nq), static_cast<std::size_t>(model->nv));
    }
};
PlaybackSession::PlaybackSession(const std::filesystem::path& path, std::optional<std::int64_t> episode)
    : PlaybackSession(load_playback(path, episode)) {}
PlaybackSession::PlaybackSession(PlaybackData data) : impl_(std::make_unique<Impl>(std::move(data))) {}
PlaybackSession::~PlaybackSession() = default;
const runtime::ModelImage& PlaybackSession::model_image() const noexcept { return impl_->evidence.model; }
double PlaybackSession::duration_s() const noexcept { return impl_->evidence.duration_s; }
const PlaybackData& PlaybackSession::data() const noexcept { return impl_->evidence; }
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
runtime::RenderFrame PlaybackSession::frame(double time_s, int generation) {
    if (generation < 0) throw std::invalid_argument("playback generation must be nonnegative");
    auto& p = *impl_; const auto index = bracket(p.evidence, time_s).first;
    const auto qpos = pose_at(p.evidence, time_s);
    // Source playback_viewer uses searchsorted's right keyframe for velocity;
    // only generalized position is interpolated between saved intervals.
    const auto found = std::ranges::lower_bound(p.evidence.times_s, std::clamp(time_s, 0., p.evidence.duration_s));
    const auto velocity_index = std::min(static_cast<std::size_t>(found - p.evidence.times_s.begin()), p.evidence.qvel.size() - 1);
    const auto& qvel = p.evidence.qvel[velocity_index];
    engine::reset_data(p.model.get(), p.data.get());
    std::ranges::copy(qpos,model_access::mutable_buffer(p.data->qpos,p.model->nq).begin());
    std::ranges::copy(qvel,model_access::mutable_buffer(p.data->qvel,p.model->nv).begin());
    p.data->time = std::clamp(time_s, 0., p.evidence.duration_s);
    // The viewer derives geometry from this owned state. No forward dynamics,
    // controller, force probe or physical integration runs on stored playback.
    runtime::RenderFrame frame; frame.model_sha256 = p.evidence.model.sha256;
    frame.state.generation = generation; frame.state.step = static_cast<std::int64_t>(index); frame.state.time_s = p.data->time;
    frame.state.integration_state.resize(static_cast<std::size_t>(engine::state_size(p.model.get(), mjSTATE_INTEGRATION)));
    engine::get_state(p.model.get(), p.data.get(), frame.state.integration_state.data(), mjSTATE_INTEGRATION);
    auto& view = frame.presentation;
    view.time_s = p.data->time; view.position_m = position(*p.model, *p.data, "root_x"); view.z_m = position(*p.model, *p.data, "root_z");
    view.speed_mps = velocity(*p.model, *p.data, "root_x"); view.pitch_rad = position(*p.model, *p.data, "root_pitch");
    const int root = joint(*p.model, "root_x");
    if (root >= 0) view.root_x_dofadr = model_access::readonly_buffer(p.model->jnt_dofadr,p.model->njnt)[static_cast<std::size_t>(root)];
    if(!p.evidence.spatial.empty()) {
        view.spatial=p.evidence.spatial.at(index);
        auto& spatial=*view.spatial;
        const auto pose=presentation_pose(*p.model,*p.data);
        const auto attitude=bike::spatial::euler_zyx(pose.quat_world_from_body_wxyz);
        spatial.position_world_m=pose.position_world_m;
        spatial.quat_wxyz=pose.quat_world_from_body_wxyz;
        spatial.attitude_valid=attitude.valid;
        spatial.heading_rad=attitude.valid?std::optional(attitude.yaw_rad):std::nullopt;
        spatial.roll_rad=attitude.valid?std::optional(attitude.roll_rad):std::nullopt;
        spatial.generation=static_cast<std::uint64_t>(generation);
        view.position_m=pose.position_world_m[0];view.z_m=pose.position_world_m[2];
        view.pitch_rad=attitude.pitch_rad;
        const auto heading=bike::spatial::rotate_body_to_world(pose.quat_world_from_body_wxyz,{1.,0.,0.});
        const double length=std::hypot(heading[0],heading[1]);
        view.speed_mps=length>1e-12?(heading[0]*pose.linear_velocity_world_mps[0]+
            heading[1]*pose.linear_velocity_world_mps[1])/length:0.;
        view.root_x_dofadr=-1; // heading-relative speed is not a scalar qvel alias
    }
    const auto copy = [](const mjtNum* first, mjtSize count,std::size_t width=1) {
        if(count<0)throw recording::RecordingError("negative playback model array extent");
        const auto extent=model_access::checked_product(static_cast<std::size_t>(count),width,static_cast<std::size_t>(std::numeric_limits<mjtSize>::max()));
        const auto values=model_access::readonly_buffer(first,static_cast<mjtSize>(extent));
        return std::vector<double>(values.begin(),values.end());
    };
    view.model_fields = {copy(p.model->dof_frictionloss,p.model->nv),copy(p.model->site_pos,p.model->nsite,3),
        copy(p.model->tendon_stiffness,p.model->ntendon),copy(p.model->tendon_damping,p.model->ntendon),
        copy(p.model->tendon_lengthspring,p.model->ntendon,2),copy(p.model->tendon_range,p.model->ntendon,2)};
    frame.state.view = view.as_wire();
    frame.state.view.emplace_back("stored_state_playback", runtime::Wire{true});
    frame.state.view.emplace_back("intervals_hash_verified", runtime::Wire{p.evidence.intervals_hash_verified});
    return frame;
}
} // namespace bike::replay
