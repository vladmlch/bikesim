// PROTOTYPE (throwaway): native-loop microbenchmark for the cpp-port map.
//
// Loads the compiled welded-physical model (artifacts/model.mjb) and a real
// mid-ride state (artifacts/state.bin), then measures wall cost of:
//   ./bench bare [steps]   — mj_step only (frozen forces, periodic rewind)
//   ./bench glue [steps]   — mj_step + engine-call surface (3x fwd, 19x jac,
//                            28 name2id, 8 qfrc assemblies, ~100 channel writes)
//   ./bench heavy [steps]  — glue + representative telemetry/accounting volume:
//                            efc-row unpack, 5 attachment wrench solves,
//                            energy ledger, ~100 string-keyed channel writes,
//                            per-step record churn, period-batch lstsq every
//                            10th step — the operation MIX of the real ~1.6ms
//                            Python glue from ticket 01's anatomy
//
// bench.py runs the identical synthetic workload in Python/numpy style —
// the delta isolates the interpreter+binding tax a port would remove.
//
// Build (from repo root):
//   MJ=.venv/lib/python3.14/site-packages/mujoco
//   clang++ -O2 -std=c++17 tools/proto_native_bench/bench.cpp \
//     -I$MJ/include $MJ/libmujoco.3.12.0.dylib \
//     -Wl,-rpath,$MJ -o tools/proto_native_bench/bench
#include <mujoco/mujoco.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

struct State {
    int nq, nv, na, nu, nx;
    double time;
    std::vector<double> qpos, qvel, act, ctrl, qfrc, xfrc;
};

static State load_state(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::fprintf(stderr, "state.bin: %s\n", std::strerror(errno)); std::exit(1); }
    int hdr[6];
    if (std::fread(hdr, 4, 6, f) != 6 || hdr[0] != 0xBEA0) { std::fprintf(stderr, "bad state.bin\n"); std::exit(1); }
    State s;
    s.nq = hdr[1]; s.nv = hdr[2]; s.na = hdr[3]; s.nu = hdr[4]; s.nx = hdr[5];
    auto rd = [&](std::vector<double>& v, size_t n) {
        v.resize(n);
        if (std::fread(v.data(), 8, n, f) != n) { std::fprintf(stderr, "truncated state.bin\n"); std::exit(1); }
    };
    std::vector<double> t(1); rd(t, 1); s.time = t[0];
    rd(s.qpos, s.nq); rd(s.qvel, s.nv); rd(s.act, s.na);
    rd(s.ctrl, s.nu); rd(s.qfrc, s.nv); rd(s.xfrc, 6 * s.nx);
    std::fclose(f);
    return s;
}

static void restore(const mjModel* m, mjData* d, const State& s) {
    d->time = s.time;
    std::memcpy(d->qpos, s.qpos.data(), 8 * s.nq);
    std::memcpy(d->qvel, s.qvel.data(), 8 * s.nv);
    if (s.na) std::memcpy(d->act, s.act.data(), 8 * s.na);
    std::memcpy(d->ctrl, s.ctrl.data(), 8 * s.nu);
    std::memcpy(d->qfrc_applied, s.qfrc.data(), 8 * s.nv);
    std::memcpy(d->xfrc_applied, s.xfrc.data(), 8 * 6 * s.nx);
    mj_forward(m, d);
}

int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "bare";
    const int steps = argc > 2 ? std::atoi(argv[2]) : 4000;
    const int rewind = argc > 3 ? std::atoi(argv[3]) : 250;
    const bool glue = std::strcmp(mode, "glue") == 0 || std::strcmp(mode, "heavy") == 0;
    const bool heavy = std::strcmp(mode, "heavy") == 0;

    char err[1024];
    mjModel* m = mj_loadModel("tools/proto_native_bench/artifacts/model.mjb", nullptr);
    if (!m) { std::fprintf(stderr, "mj_loadModel failed\n"); return 1; }
    // mj_loadModel leaves model unchanged on failure via err string in XML path;
    // for MJB a null return is the only signal.
    mjData* d = mj_makeData(m);
    State s = load_state("tools/proto_native_bench/artifacts/state.bin");
    restore(m, d, s);

    // glue-work buffers
    const int nv = m->nv;
    double srcv[8][64], acc[64], jacp[3 * 64], jacr[3 * 64];
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 64; j++) srcv[i][j] = 0.01 * (i + 1) * (j % 7 - 3);
    std::vector<double> channels(128);
    std::vector<const char*> names;
    for (int j = 1; j < m->njnt && (int)names.size() < 28; j++)
        if (const char* nm = mj_id2name(m, mjOBJ_JOINT, j)) names.push_back(nm);
    if (names.empty()) names.push_back("root_x");

    // warmup (not timed): let contact pattern settle into the replayed regime
    for (int i = 0; i < 500; i++) {
        if (i % rewind == 0) restore(m, d, s);
        if (glue) {
            std::memset(acc, 0, 8 * nv);
            for (int w = 0; w < 8; w++) for (int j = 0; j < nv; j++) acc[j] += srcv[w][j];
            std::memcpy(d->qfrc_applied, acc, 8 * nv);
            mj_forward(m, d);
            for (int j = 0; j < 19; j++)
                mj_jac(m, d, jacp, jacr, d->xpos + 3 * (1 + j % (m->nbody - 1)), 1 + j % (m->nbody - 1));
            for (int j = 0; j < 28; j++) mj_name2id(m, mjOBJ_JOINT, names[j % names.size()]);
            double e = 0;
            for (int j = 0; j < nv; j++) e += d->qfrc_constraint[j] * d->qvel[j] + d->actuator_force[j % m->nu] * d->qvel[j];
            for (int c = 0; c < 100; c++) channels[c] = d->qpos[c % s.nq] * (c + 1) + e;
            mj_step(m, d);
            mj_forward(m, d);
        } else {
            mj_step(m, d);
        }
    }

    auto t0 = std::chrono::steady_clock::now();
    double sink = 0;
    for (int i = 0; i < steps; i++) {
        if (i % rewind == 0) restore(m, d, s);
        if (glue) {
            // 8 per-step force-writer assemblies into qfrc_applied
            std::memset(acc, 0, 8 * nv);
            for (int w = 0; w < 8; w++) for (int j = 0; j < nv; j++) acc[j] += srcv[w][j];
            std::memcpy(d->qfrc_applied, acc, 8 * nv);
            std::memcpy(d->xfrc_applied, s.xfrc.data(), 8 * 6 * s.nx);
            mj_forward(m, d);                      // post-write recompute (anatomy: ~2/step in apply_forces)
            for (int j = 0; j < 19; j++)
                mj_jac(m, d, jacp, jacr, d->xpos + 3 * (1 + j % (m->nbody - 1)), 1 + j % (m->nbody - 1));
            for (int j = 0; j < 28; j++) sink += mj_name2id(m, mjOBJ_JOINT, names[j % names.size()]);
            double e = 0;
            for (int j = 0; j < nv; j++) e += d->qfrc_constraint[j] * d->qvel[j] + d->actuator_force[j % m->nu] * d->qvel[j];
            mj_step(m, d);
            mj_forward(m, d);                      // post-step telemetry pass (anatomy: 1/step)
            for (int c = 0; c < 100; c++) channels[c] = d->qacc[c % nv] * (c + 1) + e;
            sink += channels[7] + e;
            if (heavy) {
                // efc-row unpack into per-row records + named buckets
                double bucket[8] = {0};
                int efc_id_row; mjtNum efc_pos_row, efc_force_row;
                for (int r = 0; r < d->nefc; r++) {
                    efc_id_row = d->efc_id[r]; efc_pos_row = d->efc_pos[r]; efc_force_row = d->efc_force[r];
                    bucket[efc_id_row & 7] += efc_force_row * efc_pos_row;
                    sink += efc_id_row * 1e-9;
                }
                // 5 attachment wrench passes: 2 jacs + 3x3 solve (Cramer) + 6-vec record
                for (int a = 0; a < 5; a++) {
                    mj_jac(m, d, jacp, jacr, d->xpos + 3 * (1 + a), 1 + a);
                    mj_jac(m, d, jacp + 3 * nv, jacr, d->xpos + 3 * (1 + a + 5), 1 + a + 5);
                    double A[9], b[3];
                    for (int r2 = 0; r2 < 3; r2++) {
                        b[r2] = 0;
                        for (int c2 = 0; c2 < 3; c2++) {
                            A[r2 * 3 + c2] = 0;
                            for (int j = 0; j < nv; j++) A[r2 * 3 + c2] += jacp[r2 * nv + j] * jacp[c2 * nv + j];
                            b[r2] += jacp[r2 * nv + c2] * d->qfrc_constraint[c2];
                        }
                        A[r2 * 3 + r2] += 1e-6;
                    }
                    double det = A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
                    sink += (A[0]*(b[1]*A[8]-A[5]*b[2]) - A[1]*(b[0]*A[8]-A[5]*b[2]) + A[2]*(b[0]*A[7]-A[4]*b[2])) / det;
                }
                // energy ledger: ~12 scalar term updates
                double ledger = 0;
                for (int j = 0; j < nv; j++) {
                    ledger += d->qfrc_spring[j] * d->qvel[j] + d->qfrc_damper[j] * d->qvel[j]
                            + d->qfrc_bias[j] * d->qvel[j] + d->qfrc_applied[j] * d->qvel[j];
                }
                sink += ledger;
                // per-step record churn (struct, ~15 fields)
                struct { long i; double t, e, q0, v0, a0, c0, x0; int nefc; double aux[7]; } rec;
                rec.i = i; rec.t = d->time; rec.e = e; rec.q0 = d->qpos[0]; rec.v0 = d->qvel[0];
                rec.a0 = d->qacc[0]; rec.c0 = d->ctrl[0]; rec.x0 = d->xpos[3]; rec.nefc = d->nefc;
                for (int k = 0; k < 7; k++) rec.aux[k] = bucket[k];
                sink += rec.aux[3];
                // period-batch every 10th step: 8x8 normal-equation solve (Gaussian elim)
                if (i % 10 == 0) {
                    double G[8][9] = {{0}};
                    for (int r2 = 0; r2 < 8; r2++)
                        for (int c2 = 0; c2 < 8; c2++)
                            for (int k = 0; k < 10; k++) G[r2][c2] += jacp[(r2 % 3) * nv + (k + c2) % nv] * jacp[(c2 % 3) * nv + k % nv];
                    for (int r2 = 0; r2 < 8; r2++) G[r2][8] = bucket[r2];
                    for (int c2 = 0; c2 < 8; c2++) {
                        int p = c2;
                        for (int r2 = c2 + 1; r2 < 8; r2++) if (std::abs(G[r2][c2]) > std::abs(G[p][c2])) p = r2;
                        for (int k = c2; k < 9; k++) std::swap(G[c2][k], G[p][k]);
                        for (int r2 = c2 + 1; r2 < 8; r2++) {
                            double f2 = G[r2][c2] / G[c2][c2];
                            for (int k = c2; k < 9; k++) G[r2][k] -= f2 * G[c2][k];
                        }
                    }
                    for (int r2 = 7; r2 >= 0; r2--) {
                        for (int k = r2 + 1; k < 8; k++) G[r2][8] -= G[r2][k] * G[k][8];
                        sink += (G[r2][8] /= G[r2][r2]);
                    }
                }
            }
        } else {
            mj_step(m, d);
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    double wall_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / steps;
    double rtf = m->opt.timestep * 1e6 / wall_us;
    std::printf("mode=%s steps=%d nv=%d nefc~%d | %.1f us/step  %.0f steps/s  RTF=%.2fx  (sink %.1f)\n",
                mode, steps, nv, d->nefc, wall_us, 1e6 / wall_us, rtf, sink);
    mj_deleteData(d); mj_deleteModel(m);
    return 0;
}
