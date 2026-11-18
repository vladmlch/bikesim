// bundle_runner.cpp — Standalone MuJoCo 3.12 Golden Bundle attribution runner.
// Compiles identically as FP64 (mjtNum = double) against .deps/mujoco-3.12.0
// or as FP32 (-DmjUSESINGLE -ffp-contract=off, mjtNum = float) against
// mujoco-312-install-f32 without linking DuckDB, CLI, or controller code.

#include <mujoco/mujoco.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct StepClosureFileHeader {
    char magic[8];
    std::uint32_t version;
    std::uint32_t header_bytes;
    std::uint64_t n_steps;
    std::uint32_t row_doubles;
    std::uint32_t nq;
    std::uint32_t nv;
    std::uint32_t na;
    std::uint32_t nu;
    std::uint32_t ntendon;
    std::int32_t front_wheel_dof;
    std::int32_t rear_wheel_dof;
    std::int32_t freehub_wrap_adr;
    std::int32_t freehub_tendon_id;
    std::int32_t clutch_tendon_id;
    std::uint32_t reserved0;
    std::uint64_t reserved1;
};
static_assert(sizeof(StepClosureFileHeader) == 80, "expected 80-byte header");

struct Job {
    std::int32_t row;
    std::int32_t arg; // mode=parity: cog_idx (-1 => trace cog_index); mode=window: nsteps
};

std::vector<char> read_bin(const fs::path &p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open " + p.string());
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

inline mjtNum to_num(double v, bool quantize_f32) {
    if (quantize_f32) return static_cast<mjtNum>(static_cast<float>(v));
    return static_cast<mjtNum>(v);
}

} // namespace

int main(int argc, char **argv) {
    try {
        fs::path model_p, hfield_p, trace_p, jobs_p, out_p, save_mjb_p;
        std::string mode = "parity";
        bool quantize_f32 = false;
        bool use_ratio = false;
        bool reconstruct_forces = false;
        int cog_override = -1;

        for (int i = 1; i < argc; ++i) {
            const std::string k(argv[i]);
            auto next = [&]() -> std::string {
                if (++i >= argc) throw std::invalid_argument(k + ": missing value");
                return argv[i];
            };
            if (k == "--model") model_p = next();
            else if (k == "--hfield") hfield_p = next();
            else if (k == "--trace") trace_p = next();
            else if (k == "--jobs") jobs_p = next();
            else if (k == "--out") out_p = next();
            else if (k == "--save-mjb") save_mjb_p = next();
            else if (k == "--mode") mode = next();
            else if (k == "--cog-override") cog_override = std::stoi(next());
            else if (k == "--quantize-f32") quantize_f32 = true;
            else if (k == "--use-ratio") use_ratio = true;
            else if (k == "--reconstruct-forces") reconstruct_forces = true;
            else throw std::invalid_argument("unknown arg: " + k);
        }

        if (!save_mjb_p.empty()) {
            if (model_p.empty() || hfield_p.empty()) {
                throw std::invalid_argument(
                    "usage: bundle_runner --model XML --hfield BIN --save-mjb MJB");
            }
            char err[1024] = "";
            mjModel *m = mj_loadXML(model_p.c_str(), nullptr, err, sizeof(err));
            if (!m) throw std::runtime_error(std::string("mj_loadXML: ") + err);
            const auto hf = read_bin(hfield_p);
            if (hf.size() != static_cast<std::size_t>(m->nhfielddata) * sizeof(double)) {
                mj_deleteModel(m);
                throw std::runtime_error("hfield size mismatch");
            }
            std::vector<double> hf_doubles(static_cast<std::size_t>(m->nhfielddata));
            std::memcpy(hf_doubles.data(), hf.data(), hf.size());
            for (int i = 0; i < m->nhfielddata; ++i) {
                m->hfield_data[i] = static_cast<float>(hf_doubles[static_cast<std::size_t>(i)]);
            }
            const mjtSize mjb_bytes = mj_sizeModel(m);
            if (mjb_bytes <= 0) {
                mj_deleteModel(m);
                throw std::runtime_error("mj_sizeModel returned non-positive size");
            }
            std::vector<char> mjb_buf(static_cast<std::size_t>(mjb_bytes));
            mj_saveModel(m, nullptr, mjb_buf.data(), static_cast<int>(mjb_bytes));
            mj_deleteModel(m);
            std::ofstream mjb_out(save_mjb_p, std::ios::binary | std::ios::trunc);
            if (!mjb_out) {
                throw std::runtime_error("cannot write " + save_mjb_p.string());
            }
            mjb_out.write(mjb_buf.data(), static_cast<std::streamsize>(mjb_buf.size()));
            if (!mjb_out) {
                throw std::runtime_error("failed writing " + save_mjb_p.string());
            }
            return 0;
        }

        if (model_p.empty() || hfield_p.empty() || trace_p.empty() ||
            jobs_p.empty() || out_p.empty()) {
            throw std::invalid_argument(
                "usage: bundle_runner --model XML --hfield BIN --trace BIN "
                "--jobs BIN --out BIN [--mode parity|window] [--use-ratio] "
                "[--quantize-f32] [--reconstruct-forces] [--cog-override N]");
        }

        const auto trace_raw = read_bin(trace_p);
        if (trace_raw.size() < sizeof(StepClosureFileHeader)) {
            throw std::runtime_error("step_closure.bin shorter than header");
        }
        StepClosureFileHeader hdr{};
        std::memcpy(&hdr, trace_raw.data(), sizeof(hdr));
        if (std::memcmp(hdr.magic, "BGSC0001", 8) != 0 || hdr.version != 1 ||
            hdr.header_bytes != sizeof(StepClosureFileHeader)) {
            throw std::runtime_error("step_closure.bin magic/version mismatch");
        }
        if (hdr.nq == 0 || hdr.nv == 0 || hdr.ntendon == 0 || hdr.n_steps == 0) {
            throw std::runtime_error("step_closure.bin header requires positive dimensions and >= 1 step");
        }
        const std::size_t expected_row_doubles =
            8U + 2U + 2U * static_cast<std::size_t>(hdr.ntendon) +
            2U * static_cast<std::size_t>(hdr.nq) +
            11U * static_cast<std::size_t>(hdr.nv) +
            static_cast<std::size_t>(hdr.nu) + static_cast<std::size_t>(hdr.na);
        if (static_cast<std::size_t>(hdr.row_doubles) != expected_row_doubles) {
            throw std::runtime_error("step_closure.bin row_doubles mismatch");
        }
        constexpr std::size_t kMaxBundleBytes = std::size_t{1024} * 1024U * 1024U;
        if (trace_raw.size() > kMaxBundleBytes ||
            hdr.n_steps > (kMaxBundleBytes - hdr.header_bytes) /
                              (expected_row_doubles * sizeof(double))) {
            throw std::runtime_error("step_closure.bin exceeds byte bound");
        }
        const std::size_t expected_bytes =
            hdr.header_bytes +
            static_cast<std::size_t>(hdr.n_steps) * expected_row_doubles * sizeof(double);
        if (trace_raw.size() != expected_bytes) {
            throw std::runtime_error("step_closure.bin size mismatch");
        }
        std::vector<double> trace_data(
            static_cast<std::size_t>(hdr.n_steps) * hdr.row_doubles);
        if (!trace_data.empty()) {
            std::memcpy(trace_data.data(), trace_raw.data() + hdr.header_bytes,
                        trace_data.size() * sizeof(double));
        }

        char err[1024] = "";
        mjModel *m = mj_loadXML(model_p.c_str(), nullptr, err, sizeof(err));
        if (!m) throw std::runtime_error(std::string("mj_loadXML: ") + err);
        {
            const auto hf = read_bin(hfield_p);
            if (hf.size() != static_cast<std::size_t>(m->nhfielddata) * sizeof(double)) {
                throw std::runtime_error("hfield size mismatch");
            }
            std::vector<double> hf_doubles(static_cast<std::size_t>(m->nhfielddata));
            std::memcpy(hf_doubles.data(), hf.data(), hf.size());
            for (int i = 0; i < m->nhfielddata; ++i) {
                m->hfield_data[i] = static_cast<float>(hf_doubles[static_cast<std::size_t>(i)]);
            }
        }

        const int nq = static_cast<int>(hdr.nq);
        const int nv = static_cast<int>(hdr.nv);
        const int na = static_cast<int>(hdr.na);
        const int nu = static_cast<int>(hdr.nu);
        const int nt = static_cast<int>(hdr.ntendon);
        if (m->nq != nq || m->nv != nv || m->na != na || m->nu != nu) {
            throw std::runtime_error("model dimensions differ from step_closure.bin");
        }
        if (use_ratio && m->ntendon != nt) {
            throw std::runtime_error("model ntendon differs from step_closure.bin under --use-ratio");
        }
        if (hdr.front_wheel_dof < 0 || hdr.front_wheel_dof >= nv ||
            hdr.rear_wheel_dof < 0 || hdr.rear_wheel_dof >= nv) {
            throw std::runtime_error("step_closure.bin wheel DOF index out of [0, nv) range");
        }
        if (hdr.freehub_tendon_id < 0 || hdr.freehub_tendon_id >= nt) {
            throw std::runtime_error("step_closure.bin freehub_tendon_id out of [0, ntendon) range");
        }
        if (hdr.clutch_tendon_id != -1 &&
            (hdr.clutch_tendon_id < 0 || hdr.clutch_tendon_id >= nt)) {
            throw std::runtime_error("step_closure.bin clutch_tendon_id out of [0, ntendon) range");
        }
        if (use_ratio && (hdr.freehub_wrap_adr < 0 || hdr.freehub_wrap_adr >= m->nwrap)) {
            throw std::runtime_error("step_closure.bin freehub_wrap_adr out of [0, nwrap) range");
        }

        const int clutch_tid_12 = mj_name2id(m, mjOBJ_TENDON, "crank_clutch");
        if (!use_ratio && clutch_tid_12 >= 0 && hdr.clutch_tendon_id < 0) {
            throw std::runtime_error("step_closure.bin missing clutch_tendon_id for crank_clutch model");
        }
        const std::size_t row_doubles = hdr.row_doubles;

        const std::size_t O_TIME = 1;
        const std::size_t O_QPOS = 3;
        const std::size_t O_QVEL = O_QPOS + static_cast<std::size_t>(nq);
        const std::size_t O_ACT = O_QVEL + static_cast<std::size_t>(nv);
        const std::size_t O_WS = O_ACT + static_cast<std::size_t>(na);
        const std::size_t O_CTRL = O_WS + static_cast<std::size_t>(nv);
        const std::size_t O_QFA = O_CTRL + static_cast<std::size_t>(nu);
        const std::size_t O_QTIRES = O_QFA + static_cast<std::size_t>(nv);
        const std::size_t O_DFL = O_QTIRES + static_cast<std::size_t>(6 * nv);
        const std::size_t O_WRAP = O_DFL + 2;
        const std::size_t O_TR = O_WRAP + 1;
        const std::size_t O_COG = O_TR + static_cast<std::size_t>(2 * nt);

        auto field = [&](std::size_t row, std::size_t off) -> const double * {
            if (row >= hdr.n_steps) throw std::out_of_range("row out of range");
            return trace_data.data() + row * row_doubles + off;
        };

        const auto jobs_raw = read_bin(jobs_p);
        if (jobs_raw.size() % sizeof(Job) != 0) {
            throw std::runtime_error("jobs binary size is not a multiple of sizeof(Job)");
        }
        const std::size_t njobs = jobs_raw.size() / sizeof(Job);
        std::vector<Job> jobs(njobs);
        if (njobs > 0) std::memcpy(jobs.data(), jobs_raw.data(), jobs_raw.size());

        mjData *d = mj_makeData(m);
        mjData *d_const = mj_makeData(m);
        std::ofstream out(out_p, std::ios::binary | std::ios::trunc);
        if (!out) throw std::runtime_error("cannot write " + out_p.string());

        auto write_row = [&](std::int32_t r, std::int32_t arg) {
            const double meta[5] = {
                static_cast<double>(r),
                static_cast<double>(arg),
                static_cast<double>(d->ncon),
                static_cast<double>(d->nefc),
                static_cast<double>(d->solver_niter[0]),
            };
            std::vector<double> qp(static_cast<std::size_t>(nq));
            std::vector<double> qv(static_cast<std::size_t>(nv));
            for (int i = 0; i < nq; ++i) qp[static_cast<std::size_t>(i)] = static_cast<double>(d->qpos[i]);
            for (int i = 0; i < nv; ++i) qv[static_cast<std::size_t>(i)] = static_cast<double>(d->qvel[i]);
            out.write(reinterpret_cast<const char *>(meta), sizeof(meta));
            out.write(reinterpret_cast<const char *>(qp.data()),
                      static_cast<std::streamsize>(qp.size() * sizeof(double)));
            out.write(reinterpret_cast<const char *>(qv.data()),
                      static_cast<std::streamsize>(qv.size() * sizeof(double)));
        };

        auto apply_controls = [&](std::size_t r, int explicit_cog) {
            const double *ctrl = field(r, O_CTRL);
            const double *qfa = field(r, O_QFA);
            const double *dfl = field(r, O_DFL);
            const double *wrap = field(r, O_WRAP);
            const double *tr = field(r, O_TR);
            const int trace_cog = static_cast<int>(field(r, O_COG)[0]);
            const int cog = explicit_cog >= 0 ? explicit_cog : trace_cog;

            for (int i = 0; i < nu; ++i) d->ctrl[i] = to_num(ctrl[i], quantize_f32);
            if (reconstruct_forces) {
                const double *comps = field(r, O_QTIRES);
                for (int i = 0; i < nv; ++i) {
                    double sum = 0.;
                    for (int c = 0; c < 6; ++c) {
                        sum += comps[static_cast<std::size_t>(c * nv + i)];
                    }
                    d->qfrc_applied[i] = to_num(sum, quantize_f32);
                }
            } else {
                for (int i = 0; i < nv; ++i) d->qfrc_applied[i] = to_num(qfa[i], quantize_f32);
            }

            m->dof_frictionloss[hdr.front_wheel_dof] = to_num(dfl[0], quantize_f32);
            m->dof_frictionloss[hdr.rear_wheel_dof] = to_num(dfl[1], quantize_f32);

            if (use_ratio) {
                const mjtNum new_ratio = to_num(wrap[0], quantize_f32);
                if (m->wrap_prm[hdr.freehub_wrap_adr] != new_ratio) {
                    m->wrap_prm[hdr.freehub_wrap_adr] = new_ratio;
                    mj_setConst(m, d_const);
                }
                for (int i = 0; i < 2 * nt; ++i) {
                    m->tendon_range[i] = to_num(tr[i], quantize_f32);
                }
            } else {
                for (int t = 0; t < m->ntendon; ++t) {
                    m->tendon_range[2 * t] = static_cast<mjtNum>(-1e12);
                    m->tendon_range[2 * t + 1] = static_cast<mjtNum>(1e12);
                }
                const int n_cogs = m->ntendon - (clutch_tid_12 >= 0 ? 1 : 0);
                if (cog >= 0 && cog < n_cogs &&
                    hdr.freehub_tendon_id >= 0 && hdr.freehub_tendon_id < nt) {
                    const int fbase = 2 * hdr.freehub_tendon_id;
                    m->tendon_range[2 * cog] = to_num(tr[fbase], quantize_f32);
                    m->tendon_range[2 * cog + 1] = to_num(tr[fbase + 1], quantize_f32);
                }
                if (clutch_tid_12 >= 0 &&
                    hdr.clutch_tendon_id >= 0 && hdr.clutch_tendon_id < nt) {
                    const int cbase = 2 * hdr.clutch_tendon_id;
                    m->tendon_range[2 * clutch_tid_12] = to_num(tr[cbase], quantize_f32);
                    m->tendon_range[2 * clutch_tid_12 + 1] = to_num(tr[cbase + 1], quantize_f32);
                }
            }
        };

        for (std::size_t j = 0; j < njobs; ++j) {
            const std::size_t r = static_cast<std::size_t>(jobs[j].row);
            mj_resetData(m, d);
            d->time = to_num(field(r, O_TIME)[0], quantize_f32);
            for (int i = 0; i < nq; ++i) d->qpos[i] = to_num(field(r, O_QPOS)[i], quantize_f32);
            for (int i = 0; i < nv; ++i) d->qvel[i] = to_num(field(r, O_QVEL)[i], quantize_f32);
            for (int i = 0; i < na; ++i) d->act[i] = to_num(field(r, O_ACT)[i], quantize_f32);
            for (int i = 0; i < nv; ++i) d->qacc_warmstart[i] = to_num(field(r, O_WS)[i], quantize_f32);

            if (mode == "parity") {
                const int cog = cog_override >= 0 ? cog_override : jobs[j].arg;
                apply_controls(r, cog);
                mj_step(m, d);
                write_row(jobs[j].row, jobs[j].arg);
            } else if (mode == "window") {
                const int steps = jobs[j].arg;
                for (int s = 0; s < steps; ++s) {
                    const std::size_t rk = r + static_cast<std::size_t>(s);
                    apply_controls(rk, cog_override);
                    mj_step(m, d);
                }
                write_row(jobs[j].row, steps);
            } else {
                throw std::invalid_argument("bad mode: " + mode);
            }
        }

        mj_deleteData(d_const);
        mj_deleteData(d);
        mj_deleteModel(m);
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
