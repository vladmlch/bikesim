"""PROTOTYPE (throwaway): Python twin of bench.cpp — identical synthetic
per-step workload, interpreter+binding tax made measurable.

  uv run python tools/proto_native_bench/bench.py bare [steps]
  uv run python tools/proto_native_bench/bench.py glue [steps]
"""
import sys
import time
from types import SimpleNamespace

import mujoco
import numpy as np

ART = "tools/proto_native_bench/artifacts"


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "bare"
    steps = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
    rewind = 250
    glue = mode in ("glue", "heavy")
    heavy = mode == "heavy"

    model = mujoco.MjModel.from_binary_path(f"{ART}/model.mjb")
    data = mujoco.MjData(model)
    z = np.load(f"{ART}/state.npz")
    qpos, qvel, act = z["qpos"], z["qvel"], z["act"]
    ctrl, qfrc_snap, xfrc_snap = z["ctrl"], z["qfrc_applied"], z["xfrc_applied"]

    def restore():
        data.time = float(z["time"][0])
        data.qpos[:] = qpos
        data.qvel[:] = qvel
        if act.size:
            data.act[:] = act
        data.ctrl[:] = ctrl
        data.qfrc_applied[:] = qfrc_snap
        data.xfrc_applied[:] = xfrc_snap
        mujoco.mj_forward(model, data)

    nv = model.nv
    srcv = [np.array([0.01 * (w + 1) * (j % 7 - 3) for j in range(nv)]) for w in range(8)]
    jacp = np.zeros((3, nv))
    jacr = np.zeros((3, nv))
    jacp2 = np.zeros((3, nv))
    ledger = {"spring": 0., "damper": 0., "bias": 0., "applied": 0.,
              "constraint": 0., "actuator": 0.}
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
             for j in range(1, model.njnt)][:28]
    if not names:
        names = ["root_x"]
    channels = {}
    sink = [0.0]

    def glue_work(i):
        acc = np.zeros(nv)
        for w in range(8):
            acc += srcv[w]                    # 8 force-writer assemblies
        data.qfrc_applied[:] = acc
        data.xfrc_applied[:] = xfrc_snap
        mujoco.mj_forward(model, data)        # post-write recompute
        for j in range(19):
            mujoco.mj_jac(model, data, jacp, jacr,
                          data.xpos[1 + j % (model.nbody - 1)],
                          1 + j % (model.nbody - 1))
        for j in range(28):
            sink[0] += mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                         names[j % len(names)])
        e = float(np.dot(data.qfrc_constraint, data.qvel)
                  + np.dot(data.actuator_force, data.qvel[:model.nu]))
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)        # post-step telemetry pass
        sample = {"i": i, "e": e, "nefc": data.nefc, "time": data.time,
                  "qpos0": data.qpos[0], "qvel0": data.qvel[0],
                  "ctrl0": data.ctrl[0], "qacc0": data.qacc[0],
                  "x0": data.xpos[1, 0], "cfrc0": data.cfrc_ext[0] if data.nefc else 0.,
                  "a": 1, "b": 2, "c": 3, "d": 4, "e2": 5}   # per-step object churn
        for c in range(100):
            channels[c] = data.qacc[c % nv] * (c + 1) + e
        sink[0] += channels[7] + e + sample["e"]
        if heavy:
            # efc-row unpack: per-row dicts + named buckets (real code iterates
            # efc rows into attachment/weld records every step)
            buckets = {}
            for r in range(data.nefc):
                row = {"id": int(data.efc_id[r]), "pos": float(data.efc_pos[r]),
                       "force": float(data.efc_force[r]),
                       "type": int(data.efc_type[r])}
                key = row["id"] % 8
                buckets[key] = buckets.get(key, 0.) + row["force"] * row["pos"]
            # 5 attachment wrench passes: 2 jacs + 3x3 solve + record
            wrenches = {}
            for a in range(5):
                mujoco.mj_jac(model, data, jacp, jacr, data.xpos[1 + a], 1 + a)
                mujoco.mj_jac(model, data, jacp2, jacr,
                              data.xpos[1 + a + 5], 1 + a + 5)
                A = jacp @ jacp.T + 1e-6 * np.eye(3)
                w = np.linalg.solve(A, jacp @ data.qfrc_constraint)
                wrenches[a] = (float(w[0]), float(w[1]), float(w[2]))
            # energy ledger: ~12 scalar term updates into dict-of-terms
            ledger["spring"] += float(np.dot(data.qfrc_spring, data.qvel))
            ledger["damper"] += float(np.dot(data.qfrc_damper, data.qvel))
            ledger["bias"] += float(np.dot(data.qfrc_bias, data.qvel))
            ledger["applied"] += float(np.dot(data.qfrc_applied, data.qvel))
            ledger["constraint"] += float(
                np.dot(data.qfrc_constraint, data.qvel))
            ledger["actuator"] += float(
                np.dot(data.actuator_force, data.qvel[:model.nu]))
            # per-step record churn: dict + namespace (RawStep/PhysicalSample)
            rec = {"i": i, "t": data.time, "e": e, "nefc": data.nefc,
                   "b0": buckets.get(0, 0.), "b1": buckets.get(1, 0.),
                   "w0": wrenches[0][0], "w1": wrenches[1][0],
                   "q0": data.qpos[0], "v0": data.qvel[0]}
            snap = SimpleNamespace(
                i=i, e=e, nefc=data.nefc, q0=data.qpos[0], v0=data.qvel[0],
                a0=data.qacc[0], c0=data.ctrl[0], x0=data.xpos[1, 0],
                s0=ledger["spring"], s1=ledger["damper"])
            sink[0] += rec["w0"] + snap.s0
            # period-batch every 10th step: stacked small lstsq
            if i % 10 == 0:
                W = np.concatenate([jacp[:, :8] for _ in range(10)])
                b8 = W @ np.ones(8)
                sol = np.linalg.lstsq(
                    W.T @ W + 1e-6 * np.eye(8), W.T @ b8, rcond=None)[0]
                sink[0] += float(sol[0])

    restore()
    for i in range(500):                      # warmup, untimed
        if i % rewind == 0:
            restore()
        if glue:
            glue_work(i)
        else:
            mujoco.mj_step(model, data)

    t0 = time.perf_counter()
    for i in range(steps):
        if i % rewind == 0:
            restore()
        if glue:
            glue_work(i)
        else:
            mujoco.mj_step(model, data)
    wall_us = (time.perf_counter() - t0) / steps * 1e6
    rtf = model.opt.timestep * 1e6 / wall_us
    print(f"mode={mode} steps={steps} nv={nv} nefc~{data.nefc} | "
          f"{wall_us:.1f} us/step  {1e6 / wall_us:.0f} steps/s  RTF={rtf:.2f}x  "
          f"(sink {sink[0]:.1f})")


if __name__ == "__main__":
    main()
