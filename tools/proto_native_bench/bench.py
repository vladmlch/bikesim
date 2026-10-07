"""Python twin of bench.cpp — the matched per-step workloads plus the
retained R2 approximation.

Modes:
  bare           mj_step only (frozen replayed state, periodic rewind)
  glue           bare + matched engine-call surface
  heavy          glue + matched telemetry/accounting volume
  operation_mix  the retained R2 approximation — intentionally does NOT
                 match bench.cpp's per-step mix and carries no parity claim

CLI mirrors native_bench:

  uv run python tools/proto_native_bench/bench.py \
      {bare,glue,heavy,operation_mix} [steps] [rewind] \
      --artifacts <directory> [--emit-json <path>]

--artifacts names an artifacts root (a current.json pointer resolved once)
or a concrete bundle directory holding model.mjb / state.npz / state.bin /
manifest.json.

bare/glue/heavy are the *matched* workloads: both languages run the same
warmup/rewind phases, stage order, operation counts, attachment Jacobian +
1e-6*I-regularized 3x3 solve sequence, and the same actuator->velocity
mapping (the engine-projected qfrc_actuator, never qvel[:nu] slicing).
--emit-json writes operation counts, per-stage checksum contributions,
solver outputs, and provenance; tests/reference/test_native_benchmark.py
compares them against the C++ emission (counts and the declared exact
scalar stages byte-equal, solver outputs at rtol=atol=1e-12).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from state_format import (  # noqa: E402
    BIN_FILE,
    MODEL_FILE,
    WORKLOAD,
    StateFormatError,
    crc64_ecma,
    read_state,
    resolve_bundle,
)

MAX_COUNT = 10_000_000
DEFAULT_STEPS = 4_000
DEFAULT_REWIND = 250

# Canonical matched-workload constants — mirrored in bench.cpp; the emitted
# workload JSON must be parsed-equal to the C++ emission.
WARMUP_STEPS = int(WORKLOAD["warmup_steps"])
WRITERS = int(WORKLOAD["writers"])
CHANNELS = int(WORKLOAD["channels"])
NAME_LOOKUPS_MAX = int(WORKLOAD["name_lookups_max"])
EFC_BUCKETS = int(WORKLOAD["efc_buckets"])
EFC_ROWS_MAX = int(WORKLOAD["efc_rows_max"])
ATTACHMENT_COUNT = int(WORKLOAD["attachment_count"])
POINTS_PER_ATTACHMENT = int(WORKLOAD["points_per_attachment"])
RECORD_CHURN = int(WORKLOAD["record_churn"])
LEDGER_TERMS = list(WORKLOAD["ledger_terms"])
BATCH = WORKLOAD["batch"]
REGULARIZATION = float(WORKLOAD["regularization"])
SOLVE_ETA = float(WORKLOAD["solve_eta"])
FORWARDS_PER_STEP = int(WORKLOAD["forwards_per_step"])
SEED = int(WORKLOAD["seed"], 16)

HEAVY_MIN_BODIES = 1 + 2 * ATTACHMENT_COUNT
HEAVY_MIN_NV = 3


class Splitmix64:
    """Mirrored in bench.cpp — fills the writer sources bit-identically."""

    def __init__(self, seed: int) -> None:
        self.state = seed & 0xFFFFFFFFFFFFFFFF

    def next_u64(self) -> int:
        self.state = (self.state + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        return z ^ (z >> 31)

    def next_f64(self) -> float:
        return (self.next_u64() >> 11) * (1.0 / 9007199254740992.0)


def _solve3x3_spd(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Regularized SPD 3x3 solve via lower Cholesky — the same scalar
    sequence as bench.cpp's solve3x3_spd (same rank policy: fixed
    regularization, never a rank-revealing fallback). A nonpositive pivot
    raises instead of returning a NaN."""
    l11 = math.sqrt(a[0, 0])
    if not (l11 > 0.0) or not math.isfinite(l11):
        raise RuntimeError(
            "attachment normal matrix is singular after regularization"
        )
    l21 = a[1, 0] / l11
    l31 = a[2, 0] / l11
    l22 = math.sqrt(a[1, 1] - l21 * l21)
    if not (l22 > 0.0) or not math.isfinite(l22):
        raise RuntimeError(
            "attachment normal matrix is singular after regularization"
        )
    l32 = (a[2, 1] - l31 * l21) / l22
    l33 = math.sqrt(a[2, 2] - l31 * l31 - l32 * l32)
    if not (l33 > 0.0) or not math.isfinite(l33):
        raise RuntimeError(
            "attachment normal matrix is singular after regularization"
        )
    y1 = b[0] / l11
    y2 = (b[1] - l21 * y1) / l22
    y3 = (b[2] - l31 * y1 - l32 * y2) / l33
    x3 = y3 / l33
    x2 = (y2 - l32 * x3) / l22
    x1 = (y1 - l21 * x2 - l31 * x3) / l11
    result = np.array([x1, x2, x3])
    if not np.isfinite(result).all():
        raise RuntimeError("attachment wrench solve produced non-finite output")
    return result


def _new_stats() -> SimpleNamespace:
    return SimpleNamespace(
        counts=dict(
            engine_steps=0, rewinds=0, forwards=0, jacobian_calls=0,
            name_lookups=0, qfrc_elements=0, channel_writes=0,
            energy_terms=0, efc_rows=0, attachment_solves=0, point_evals=0,
            ledger_terms=0, records=0, batch_events=0, batch_iterations=0,
        ),
        ck=dict(
            trajectory_time=0.0, forces=0.0, energy=0.0, jacobians=0.0,
            attachments=0.0, channels=0.0, efc=0.0, ledger=0.0,
            records=0.0, batch=0.0, name_ids=0,
        ),
        solve_outputs=[],
        mix_sink=0.0,
    )


def _matched_step(model, data, ctx, st, mode, i):
    """The matched per-step workload — stage order identical to bench.cpp:
    forces -> forward -> energy -> jacobians/solves -> names -> channels ->
    (heavy: efc, ledger, records) -> step -> forward -> (heavy: batch) ->
    trajectory probe."""
    nv = ctx.nv
    heavy = mode == "heavy"

    # Stage 1 — applied-force assembly: eight writer sources summed into
    # qfrc_applied (elementwise, identical-order), recorded xfrc re-staged,
    # then a forward recompute.
    ctx.acc[:] = 0.0
    for w in range(WRITERS):
        ctx.acc += ctx.sources[w]
    data.qfrc_applied[:] = ctx.acc
    data.xfrc_applied[:] = ctx.state_xfrc
    mujoco.mj_forward(model, data)
    st.counts["forwards"] += 1
    st.counts["qfrc_elements"] += WRITERS * nv
    st.ck["forces"] += float(ctx.acc.sum())

    # Stage 2 — energy probe. The actuator side uses the engine-projected
    # qfrc_actuator (moment^T . actuator_force, length nv) — the explicit
    # supported mapping, never qvel[:nu] slicing.
    energy = float(
        np.dot(data.qfrc_constraint, data.qvel)
        + np.dot(data.qfrc_actuator, data.qvel)
    )
    st.ck["energy"] += energy
    st.counts["energy_terms"] += 2 * nv

    # Stage 3 — point Jacobians (glue: one pair on body 1; heavy: five
    # attachment pairs) plus the heavy attachment wrench solves.
    if not heavy:
        point = data.xpos[1]
        mujoco.mj_jac(model, data, ctx.jacp, ctx.jacr, point, 1)
        mujoco.mj_jac(model, data, ctx.jacp2, None, point, 1)
        st.counts["jacobian_calls"] += 2
        st.ck["jacobians"] += float(
            ctx.jacp.sum() + ctx.jacr.sum() + ctx.jacp2.sum()
        )
    else:
        for attachment in ctx.attachments:
            mujoco.mj_jac(
                model, data, ctx.jacp, ctx.jacr,
                data.xpos[attachment["body"]], attachment["body"],
            )
            mujoco.mj_jac(
                model, data, ctx.jacp2, None,
                data.xpos[attachment["peer"]], attachment["peer"],
            )
            st.counts["jacobian_calls"] += 2
            st.ck["jacobians"] += float(
                ctx.jacp.sum() + ctx.jacr.sum() + ctx.jacp2.sum()
            )
            normal = ctx.jacp @ ctx.jacp.T + REGULARIZATION * np.eye(3)
            rhs = ctx.jacp @ data.qfrc_constraint
            wrench = _solve3x3_spd(normal, rhs)
            st.counts["attachment_solves"] += 1
            st.solve_outputs.extend(float(c) for c in wrench)
            for k in range(POINTS_PER_ATTACHMENT):
                st.ck["attachments"] += attachment["gain"] * float(
                    wrench[k % 3]
                )
            st.counts["point_evals"] += POINTS_PER_ATTACHMENT

    # Stage 4 — name lookups; the exact integer id sum is the byte-equal
    # scalar stage.
    for name in ctx.names:
        st.ck["name_ids"] += mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        st.counts["name_lookups"] += 1

    # Stage 5 — 100 channel writes per step.
    for c in range(CHANNELS):
        ctx.channels[c] = data.qpos[c % ctx.nq] * (c + 1) + i * 1e-9
    st.counts["channel_writes"] += CHANNELS
    st.ck["channels"] += float(ctx.channels.sum())

    if heavy:
        # Stage 6 — efc-row unpack, batched; sized from reported nefc only.
        n_read = min(max(data.nefc, 0), EFC_ROWS_MAX)
        bucket = np.zeros(EFC_BUCKETS, dtype=np.int64)
        bpos = np.zeros(EFC_BUCKETS)
        for r in range(n_read):
            k = int(data.efc_id[r]) & (EFC_BUCKETS - 1)
            bucket[k] += 1
            bpos[k] += data.efc_pos[r] + data.efc_force[r]
        st.ck["efc"] += float(bucket[0]) + float(bpos[1])
        st.counts["efc_rows"] += n_read

        # Stage 7 — energy ledger: the six declared terms, each an nv-length
        # reduction in the declared order.
        qvel = data.qvel
        ledger_sum = 0.0
        for term in (
            data.qfrc_spring, data.qfrc_damper, data.qfrc_bias,
            data.qfrc_applied, data.qfrc_constraint, data.qfrc_actuator,
        ):
            ledger_sum += float(np.dot(term, qvel))
        st.ck["ledger"] += ledger_sum
        st.counts["ledger_terms"] += len(LEDGER_TERMS)

        # Stage 8 — record churn: six per-step telemetry records.
        records = [[i, float(data.qpos[0]), float(data.ctrl[0])]
                   for _ in range(RECORD_CHURN)]
        records[-1][2] += ctx.channels[7]
        for record in records:
            st.ck["records"] += record[1] + record[2]
        st.counts["records"] += RECORD_CHURN

    # Stage 9/10 — the engine step and the post-step telemetry forward.
    mujoco.mj_step(model, data)
    st.counts["engine_steps"] += 1
    mujoco.mj_forward(model, data)
    st.counts["forwards"] += 1

    if heavy and i % BATCH["period"] == 0:
        # Stage 11 — period-batch solve: 16 gradient-free iterations over
        # six dof triples.
        for c in range(BATCH["batches"]):
            for it in range(BATCH["iterations"]):
                g = 0.0
                for k2 in range(BATCH["width"]):
                    g += data.qacc[(k2 + c) % nv]
                ctx.solve[it] -= SOLVE_ETA * g
            st.ck["batch"] += float(ctx.solve[c]) * 1e-6
        st.solve_outputs.extend(float(v) for v in ctx.solve)
        st.counts["batch_events"] += 1
        st.counts["batch_iterations"] += BATCH["batches"] * BATCH["iterations"]

    # Stage 12 — trajectory probe: bit-identical engine output accumulated
    # in identical order, the declared byte-equal scalar stage.
    st.ck["trajectory_time"] += float(data.time)


def _mix_step(model, data, ctx, st, i):
    """The retained R2 operation mix (Python flavor) — preserved verbatim as
    the old heavy path: it intentionally does NOT match bench.cpp's mix and
    carries no parity claim. Its engine call shape stays inside the same
    supported domains."""
    nv = ctx.nv
    acc = np.zeros(nv)
    for w in range(WRITERS):
        acc += ctx.sources[w]
    data.qfrc_applied[:] = acc
    data.xfrc_applied[:] = ctx.state_xfrc
    mujoco.mj_forward(model, data)
    for j in range(19):
        mujoco.mj_jac(model, data, ctx.jacp, ctx.jacr,
                      data.xpos[1 + j % (model.nbody - 1)],
                      1 + j % (model.nbody - 1))
    for j in range(28):
        st.mix_sink += mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT,
            ctx.mix_names[j % len(ctx.mix_names)],
        )
        st.counts["name_lookups"] += 1
    e = float(
        np.dot(data.qfrc_constraint, data.qvel)
        # R2's approximate slice mapping (retained); crashes on nu>nv were
        # never a supported domain for this approximation anyway — the
        # matched modes use qfrc_actuator instead.
        + np.dot(data.actuator_force, data.qvel[: model.nu])
    )
    mujoco.mj_step(model, data)
    st.counts["engine_steps"] += 1
    mujoco.mj_forward(model, data)
    sample = {"i": i, "e": e, "nefc": data.nefc, "time": data.time,
              "qpos0": data.qpos[0], "qvel0": data.qvel[0],
              "ctrl0": data.ctrl[0], "qacc0": data.qacc[0],
              "x0": data.xpos[1, 0],
              "cfrc0": data.cfrc_ext[0] if data.nefc else 0.,
              "a": 1, "b": 2, "c": 3, "d": 4, "e2": 5}
    channels = {}
    for c in range(100):
        channels[c] = data.qacc[c % nv] * (c + 1) + e
    st.counts["channel_writes"] += 100
    st.mix_sink += channels[7] + e + sample["e"]
    # R2 heavy extras: efc-row unpack, attachment wrenches, ledger, records,
    # period-batch solve — same shapes as before.
    buckets = {}
    for r in range(data.nefc):
        row = {"id": int(data.efc_id[r]), "pos": float(data.efc_pos[r]),
               "force": float(data.efc_force[r]),
               "type": int(data.efc_type[r])}
        key = row["id"] % 8
        buckets[key] = buckets.get(key, 0.) + row["force"] * row["pos"]
    wrenches = {}
    for a in range(ATTACHMENT_COUNT):
        mujoco.mj_jac(model, data, ctx.jacp, ctx.jacr,
                      data.xpos[1 + a], 1 + a)
        mujoco.mj_jac(model, data, ctx.jacp2, ctx.jacr,
                      data.xpos[1 + a + ATTACHMENT_COUNT],
                      1 + a + ATTACHMENT_COUNT)
        A = ctx.jacp @ ctx.jacp.T + 1e-6 * np.eye(3)
        w = np.linalg.solve(A, ctx.jacp @ data.qfrc_constraint)
        wrenches[a] = (float(w[0]), float(w[1]), float(w[2]))
    ctx.ledger["spring"] += float(np.dot(data.qfrc_spring, data.qvel))
    ctx.ledger["damper"] += float(np.dot(data.qfrc_damper, data.qvel))
    ctx.ledger["bias"] += float(np.dot(data.qfrc_bias, data.qvel))
    ctx.ledger["applied"] += float(np.dot(data.qfrc_applied, data.qvel))
    ctx.ledger["constraint"] += float(np.dot(data.qfrc_constraint, data.qvel))
    ctx.ledger["actuator"] += float(
        np.dot(data.actuator_force, data.qvel[: model.nu]))
    rec = {"i": i, "t": data.time, "e": e, "nefc": data.nefc,
           "b0": buckets.get(0, 0.), "b1": buckets.get(1, 0.),
           "w0": wrenches[0][0], "w1": wrenches[1][0],
           "q0": data.qpos[0], "v0": data.qvel[0]}
    snap = SimpleNamespace(
        i=i, e=e, nefc=data.nefc, q0=data.qpos[0], v0=data.qvel[0],
        a0=data.qacc[0], c0=data.ctrl[0], x0=data.xpos[1, 0],
        s0=ctx.ledger["spring"], s1=ctx.ledger["damper"])
    st.mix_sink += rec["w0"] + snap.s0
    if i % BATCH["period"] == 0:
        W = np.concatenate([ctx.jacp[:, :8] for _ in range(10)])
        b8 = W @ np.ones(8)
        sol = np.linalg.lstsq(
            W.T @ W + 1e-6 * np.eye(8), W.T @ b8, rcond=None)[0]
        st.mix_sink += float(sol[0])
        st.solve_outputs.extend(float(v) for v in sol)
        st.counts["batch_events"] += 1
    st.mix_sink += float(data.time)


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="bench.py",
        description="Python twin of native_bench (matched workloads + "
                    "operation_mix)",
    )
    parser.add_argument(
        "mode", choices=["bare", "glue", "heavy", "operation_mix"])
    parser.add_argument("steps", nargs="?", type=int,
                        default=DEFAULT_STEPS)
    parser.add_argument("rewind", nargs="?", type=int,
                        default=DEFAULT_REWIND)
    parser.add_argument("--artifacts", required=True,
                        help="artifacts root (current.json pointer) or a "
                             "concrete bundle directory")
    parser.add_argument("--emit-json", default=None, dest="emit_json",
                        help="write the operation/checksum/provenance report")
    args = parser.parse_args(argv)
    for field in ("steps", "rewind"):
        value = getattr(args, field)
        if not 1 <= value <= MAX_COUNT:
            parser.error(
                f"{field} must be an integer in [1, {MAX_COUNT}], got "
                f"{value!r}"
            )
    return args


def _jacobian_calls_per_step(mode: str) -> int:
    return {"bare": 0, "glue": 2,
            "heavy": 2 * ATTACHMENT_COUNT,
            "operation_mix": 2 * ATTACHMENT_COUNT}[mode]


def _workload_block(mode: str, claim: str, steps: int, rewind: int,
                    ctx) -> dict:
    return {
        "format": 1,
        "mode": mode,
        "seed": WORKLOAD["seed"],
        "claim": claim,
        "warmup_steps": WARMUP_STEPS,
        "measured_steps": steps,
        "rewind_period": rewind,
        "writers": WRITERS,
        "channels": CHANNELS,
        "name_lookups_max": NAME_LOOKUPS_MAX,
        "names": list(ctx.names),
        "efc_buckets": EFC_BUCKETS,
        "efc_rows_max": EFC_ROWS_MAX,
        "attachments": ctx.attachments,
        "points_per_attachment": POINTS_PER_ATTACHMENT,
        "record_churn": RECORD_CHURN,
        "ledger_terms": LEDGER_TERMS,
        "batch": dict(BATCH),
        "regularization": REGULARIZATION,
        "solve_eta": SOLVE_ETA,
        "forwards_per_step": FORWARDS_PER_STEP,
        "jacobian_calls_per_step": _jacobian_calls_per_step(mode),
    }


def run(argv=None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    mode = args.mode
    matched = mode != "operation_mix"
    claim = "matched" if matched else "operation_mix"

    artifacts = Path(args.artifacts)
    pointer = artifacts / "current.json"
    bundle_dir = resolve_bundle(artifacts)
    model_path = bundle_dir / MODEL_FILE
    state_path = bundle_dir / BIN_FILE

    model = mujoco.MjModel.from_binary_path(str(model_path))
    data = mujoco.MjData(model)
    state = read_state(state_path, model_path)
    via_pointer = pointer.is_file()

    if mode == "bare":
        pass  # any engine-loadable model, including an empty one, may step
    else:
        for field, minimum in (("nq", 1), ("nv", 1), ("nu", 1), ("nbody", 2)):
            if int(getattr(model, field)) < minimum:
                raise StateFormatError(
                    f"mode '{mode}' requires a model with {field} >= "
                    f"{minimum} (got {int(getattr(model, field))})"
                )
        if mode in ("heavy", "operation_mix"):
            if model.nbody < HEAVY_MIN_BODIES:
                raise StateFormatError(
                    f"mode '{mode}' requires nbody >= {HEAVY_MIN_BODIES} to "
                    f"resolve {ATTACHMENT_COUNT} attachment body pairs "
                    f"(got {model.nbody})"
                )
            if model.nv < HEAVY_MIN_NV:
                raise StateFormatError(
                    f"mode '{mode}' requires nv >= {HEAVY_MIN_NV} "
                    f"(got {model.nv})"
                )

    nv = model.nv
    stats = _new_stats()

    ctx = SimpleNamespace(
        nv=nv,
        nq=model.nq,
        nu=model.nu,
        nbody=model.nbody,
        acc=np.zeros(nv),
        jacp=np.zeros((3, nv)),
        jacr=np.zeros((3, nv)),
        jacp2=np.zeros((3, nv)),
        channels=np.zeros(CHANNELS),
        solve=np.zeros(BATCH["iterations"]),
        ledger={name: 0.0 for name in LEDGER_TERMS},
        state_xfrc=state["xfrc_applied"],
        names=[],
        mix_names=[],
        attachments=[
            {"body": i + 1, "peer": i + 1 + ATTACHMENT_COUNT,
             "gain": 0.5 + 0.1 * i}
            for i in range(ATTACHMENT_COUNT)
        ],
    )
    if mode != "bare":
        rng = Splitmix64(SEED)
        if matched:
            ctx.sources = [np.array([0.1 * rng.next_f64() - 0.05
                                     for _ in range(nv)])
                           for _ in range(WRITERS)]
        else:
            # R2's closed-form fill, retained under operation_mix.
            ctx.sources = [
                np.array([0.01 * (w + 1) * (j % 7 - 3) for j in range(nv)])
                for w in range(WRITERS)
            ]
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(1, model.njnt)
        ]
        ctx.names = [n for n in names if n is not None][:NAME_LOOKUPS_MAX]
        if not ctx.names:
            ctx.names = ["root_x"]
        ctx.mix_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(1, model.njnt)
        ][:28]
        ctx.mix_names = [n for n in ctx.mix_names if n is not None]
        if not ctx.mix_names:
            ctx.mix_names = ["root_x"]

    def restore():
        data.time = float(state["time"][0])
        data.qpos[:] = state["qpos"]
        data.qvel[:] = state["qvel"]
        if state["act"].size:
            data.act[:] = state["act"]
        data.ctrl[:] = state["ctrl"]
        data.qfrc_applied[:] = state["qfrc_applied"]
        data.xfrc_applied[:] = state["xfrc_applied"]
        data.qacc_warmstart[:] = state["qacc_warmstart"]
        mujoco.mj_forward(model, data)
        stats.counts["rewinds"] += 1
        stats.counts["forwards"] += 1

    def step_phase(index: int):
        if mode == "bare":
            mujoco.mj_step(model, data)
            stats.counts["engine_steps"] += 1
            stats.ck["trajectory_time"] += float(data.time)
        elif matched:
            _matched_step(model, data, ctx, stats, mode, index)
        else:
            _mix_step(model, data, ctx, stats, index)

    restore()
    for i in range(WARMUP_STEPS):
        if i % args.rewind == 0:
            restore()
        step_phase(i)

    t0 = time.perf_counter()
    for i in range(args.steps):
        if i % args.rewind == 0:
            restore()
        step_phase(i)
    wall_us = (time.perf_counter() - t0) * 1e6

    checksum = (
        stats.ck["trajectory_time"] + stats.ck["forces"] + stats.ck["energy"]
        + stats.ck["jacobians"] + stats.ck["attachments"]
        + stats.ck["channels"] + stats.ck["efc"] + stats.ck["ledger"]
        + stats.ck["records"] + stats.ck["batch"] + float(stats.ck["name_ids"])
        + stats.mix_sink
    )
    if not math.isfinite(checksum):
        raise RuntimeError(
            "benchmark checksum is not finite — refusing to report a "
            "successful NaN sink"
        )

    us_per_step = wall_us / args.steps
    rtf = model.opt.timestep * 1e6 / us_per_step
    print(
        f"mode={mode} steps={args.steps} nv={nv} nefc~{data.nefc} | "
        f"{us_per_step:.1f} us/step  {1e6 / us_per_step:.0f} steps/s  "
        f"RTF={rtf:.2f}x  (checksum {checksum:.6e})"
    )

    if args.emit_json:
        model_crc = crc64_ecma(model_path.read_bytes())
        state_crc = crc64_ecma(state_path.read_bytes()[80:])
        report = {
            "format": 1,
            "comparison": claim,
            "mode": mode,
            "workload": _workload_block(
                mode, claim, args.steps, args.rewind, ctx
            ),
            "operation_counts": stats.counts,
            "stage_checksums": stats.ck,
            "solve_outputs": stats.solve_outputs,
            "checksum": checksum,
            "timing": {
                "wall_us": wall_us,
                "us_per_step": us_per_step,
                "rtf": rtf,
            },
            "provenance": {
                "language": "python",
                "interpreter": sys.version.split()[0],
                "numpy": np.__version__,
                "mujoco": mujoco.__version__,
                "bundle": str(bundle_dir),
                "via_pointer": via_pointer,
                "model_crc64": f"0x{model_crc:016x}",
                "state_crc64": f"0x{state_crc:016x}",
            },
        }
        Path(args.emit_json).write_text(json.dumps(report) + "\n")
    return 0


def main() -> int:
    try:
        return run()
    except (StateFormatError, mujoco.FatalError, RuntimeError, OSError,
            ValueError) as error:
        print(f"bench.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
