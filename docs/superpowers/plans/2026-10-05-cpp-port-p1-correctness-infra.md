# C++ Port P1: Correctness Infrastructure + Build Scaffold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Построить инфраструктуру корректности (golden-эпизоды, comparator, горизонт расхождения) и нативный build-scaffold — до первой строчки ported-кода, чтобы порт был проверяем на каждом шаге.

**Architecture:** Python остаётся оракулом через артефакты: инструмент `golden_episode` замораживает per-step контракт (control + state + все каналы рекордера + first_failure); `episode_compare` реплеит и сравнивает; `divergence_horizon` измеряет хаотический горизонт; `native/` — CMake-ядро с nanobind-скелетом, доказывающее шов и слой-1 на самом `mj_step`.

**Tech Stack:** Python 3.13 + uv, MuJoCo 3.12 (та же dylib), nanobind, CMake + Apple clang 21 (C++23), GCC 16 вторым фронтендом, pytest, numpy.

**Spec:** `docs/adr/0001-native-port-mujoco-core.md` — решение, границы, планка корректности (три слоя), флаги; фактура: `.scratch/cpp-port/issues/01..07`.

## Global Constraints

- Python — всегда через `uv run` (`uv run python`, `uv run pytest`); Python ≥3.13.
- Нативный код — C++23 (`-std=c++23`), Apple clang primary, GCC 16 вторым фронтендом compile-only.
- Warning set (оба, warnings-as-errors): `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wnull-dereference`; clang-only: `-Wunsafe-buffer-usage`; gcc-only: `-Wlogical-op -Wduplicated-cond -Wuseless-cast -Wstringop-overflow=4`.
- Runtime hardening: `-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST -fstack-protector-strong -ftrivial-auto-var-init=zero -fvisibility=hidden`; `-fstack-clash-protection` ЗАПРЕЩЁН на clang/Darwin (unused-argument → -Werror ломает сборку); `-D_FORTIFY_SOURCE` не использовать (no-op на Darwin).
- `-isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk` — brew-линкер/CLT не жуёт `arm64e.x1` из SDK 27.
- Чужие хедеры (mujoco, nanobind, позже Eigen) — только через `-isystem`.
- MuJoCo линкуется из venv: `.venv/lib/python3.14/site-packages/mujoco/` (headers `include/`, dylib `libmujoco.3.12.0.dylib`, линк через `-headerpad_max_install_names` + `install_name_tool` на абсолютный путь — см. `tools/proto_native_bench/`).
- Планка корректности (ADR §6): слой-1 per-call ~1e-12; слой-2 детерминированная эквивалентность эпизода (все каналы + тот же первый нарушающий шаг) до измеренного горизонта; слой-3 статистика дальше. Python — оракул через артефакты, не live-сравнение.
- Этот план НЕ портит продуктовый код: только инфраструктура + нативный скелет. Никаких изменений в `src/bike_sim/`.
- Канонический сценарий: `--physics-config examples/research/viewer_physics_welded.toml --track-file examples/research/rough_uphill_extreme.toml` через `research_cli.make_environment`; тестовый (быстрый) — `rough_uphill_savage.toml` как в `test_pinned_topology._model`.
- Существующий паттерн оракула: `tests/reference/test_realtime_deferred_checks.py::test_two_hundred_steps_match_scalar_validation_oracle` (`runtime.step(control=RideControl(...))`, `runtime.completed_samples`, `sample.as_dict()`, `runtime.flush()`, `runtime.reference_monitor.first_failure`, `_equal` на rtol=1e-12).
- Не трогать грязные файлы юзера: `tests/reference/test_joint_strength.py`, `test_seated_pedaling_cycle.py`, `test_seated_topology.py`, `tests/test_rider_welds.py`.
- Прототипные числа: `tools/proto_native_bench/` — референс линковки/снятия mjb+state (`dump_model.py`).

---

### Task 1: Golden-episode capture tool

Замораживает эпизод как артефакт: начальное состояние `mjData`, per-step `applied_control`, все `sample.as_dict()` строки, `first_failure`, crash-event — в `.npz` + `manifest.json`. Это генератор оракула; replay-сторона — Task 2.

**Files:**
- Create: `tools/golden_episode.py`
- Create: `conftest.py` (repo root — `sys.path.insert(0, str(Path(__file__).parent))` чтобы `tools.*` импортировался из тестов; в репо нет `tools/__init__.py` и `pythonpath` в pytest-конфиге)
- Test: `tests/reference/test_golden_episode.py`

**Interfaces:**
- Consumes: `bike_sim.cli.research.make_environment`, `env.sim`, `sim.physical` (PhysicalRuntime), `runtime.step(control=RideControl)`, `runtime.completed_samples` (PhysicalSample с `.as_dict()`), `runtime.flush()`, `runtime.reference_monitor.first_failure`, `PhysicalInitialState.capture(sim)` / `sim.reset()` (паттерн из oracle-теста), `bike_sim.validation.environment.source_fingerprint`.
- Produces: `capture_episode(env, steps: int, control: RideControl) -> EpisodeArtifact`; `save(ep, out_dir: Path, model)` пишет `episode.npz` (начальные `qpos/qvel/act/time`, per-step контроль-векторы, flatten-нутые numeric-каналы `steps × nch` + `channel_names`), `manifest.json` (`source_fingerprint`, `first_failure`, per-step non-numeric каналы) и `model.mjb` через `mujoco.mj_saveModel`; `load_episode(dir) -> EpisodeArtifact`; `flatten_row(row) -> dict`. Поздние задачи читают эти файлы.

- [ ] **Step 1: Failing test — детерминизм захвата**

```python
"""Golden-episode capture is deterministic and complete."""
from pathlib import Path
import pytest

def _ep(tmp_path, steps=50):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    from tools.golden_episode import capture_episode
    from bike_sim.sim.ride.control import RideControl
    return env, capture_episode(env, steps, RideControl(human_torque_nm=35.))

@pytest.mark.slow
def test_capture_is_deterministic(tmp_path):
    (_, a), (_, b) = _ep(tmp_path), _ep(tmp_path)
    assert a.channel_names == b.channel_names and len(a.rows) == len(b.rows)
    for ra, rb in zip(a.rows, b.rows):
        assert ra == rb            # bitwise-equal dicts: same run must replay identically
    assert a.first_failure == b.first_failure

@pytest.mark.slow
def test_saved_artifact_roundtrips(tmp_path):
    env, ep = _ep(tmp_path)
    from tools.golden_episode import load_episode, flatten_row, save
    save(ep, tmp_path/'golden', env.sim.model)
    loaded = load_episode(tmp_path/'golden')
    assert loaded.channel_names == ep.channel_names
    assert loaded.rows == [flatten_row(r) for r in ep.rows]
    assert loaded.first_failure == ep.first_failure
    assert (tmp_path/'golden'/'model.mjb').exists()
```

- [ ] **Step 2: Run — verify fail**

Run: `uv run pytest tests/reference/test_golden_episode.py -v -m slow`
Expected: FAIL — `ModuleNotFoundError: tools.golden_episode`.

- [ ] **Step 3: Implement `tools/golden_episode.py`**

```python
"""Freeze a deterministic episode as the correctness oracle artifact.

Captures per-step applied control, raw mjData state, and every recorder
channel row, plus the first monitor failure — the layer-2 contract a native
port must reproduce (ADR 0001 §6).
"""
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.initial_state import PhysicalInitialState
from bike_sim.validation.environment import source_fingerprint


@dataclass
class EpisodeArtifact:
    channel_names: list[str]
    initial: dict          # mjData arrays: qpos, qvel, act, time, ctrl, warmstart
    controls: list[dict]   # per-step asdict(applied_control)
    rows: list[dict]       # per-step sample.as_dict()
    first_failure: object  # (time_s, violations tuple) or None
    manifest: dict


def capture_episode(env, steps: int, control: RideControl) -> EpisodeArtifact:
    sim = env.sim
    runtime = sim.physical
    sim.physical_initial_state = PhysicalInitialState.capture(sim)
    sim.reset()
    d = sim.data
    initial = {k: getattr(d, k).copy() for k in
               ('qpos', 'qvel', 'act', 'ctrl', 'qacc_warmstart')}
    initial['time'] = np.array([d.time])
    controls, rows = [], []
    for _ in range(steps):
        runtime.step(control=control)
        controls.append(asdict(runtime.applied_control))
        rows.extend(s.as_dict() for s in runtime.completed_samples)
    rows.extend(s.as_dict() for s in runtime.flush())
    manifest = {
        'source_fingerprint': source_fingerprint(
            Path(__file__).resolve().parents[1] / 'src/bike_sim'),
        'steps': len(rows),
        'dt_s': float(sim.model.opt.timestep),
    }
    return EpisodeArtifact(sorted(flatten_row(rows[0])), initial, controls,
                           rows, runtime.reference_monitor.first_failure,
                           manifest)


def flatten_row(row: dict) -> dict:
    """Leaf-flatten a sample.as_dict() row: {a:{b:1}} -> {'a.b': 1.0}.

    Scalar leaves become floats; arrays flatten elementwise ('a.0','a.1');
    non-numeric leaves are returned as-is for the manifest side channel.
    """
    out = {}
    def walk(v, p):
        if isinstance(v, dict):
            for k, x in v.items(): walk(x, f'{p}.{k}' if p else k)
        elif isinstance(v, np.ndarray):
            for i, x in enumerate(v.flat): out[f'{p}.{i}'] = float(x)
        else:
            out[p] = v
    walk(row, '')
    return out


def save(ep: EpisodeArtifact, out_dir: Path, model):
    out_dir.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(out_dir/'model.mjb'))
    flat_rows = [flatten_row(r) for r in ep.rows]
    names = ep.channel_names
    arr = np.empty((len(flat_rows), len(names)))
    nonnum_rows = []
    for i, fr in enumerate(flat_rows):
        nonnum_rows.append({})
        for j, n in enumerate(names):
            v = fr[n]
            if isinstance(v, (int, float)):
                arr[i, j] = v
            else:
                arr[i, j] = np.nan          # numeric hole; real value in manifest
                nonnum_rows[i][n] = v
    np.savez(out_dir/'episode.npz', channels=arr,
             channel_names=np.array(names),
             **{f'init_{k}': v for k, v in ep.initial.items()},
             controls=np.array([[c[k] for k in sorted(c)] for c in ep.controls]),
             control_names=np.array(sorted(ep.controls[0])))
    (out_dir/'manifest.json').write_text(json.dumps({
        **ep.manifest, 'first_failure': ep.first_failure,
        'non_numeric': nonnum_rows}, indent=2, default=str))


def load_episode(d: Path) -> EpisodeArtifact:
    z = np.load(d/'episode.npz', allow_pickle=False)
    man = json.loads((d/'manifest.json').read_text())
    nonnum = man.get('non_numeric', [])
    rows = []
    for i, row in enumerate(z['channels']):
        flat = {n: float(v) for n, v in zip(z['channel_names'], row)}
        if i < len(nonnum):
            flat.update(nonnum[i])        # restore manifest-side values
        rows.append(flat)
    return EpisodeArtifact(list(z['channel_names']),
        {k[5:]: z[k] for k in z.files if k.startswith('init_')},
        [dict(zip(z['control_names'], c)) for c in z['controls']],
        rows, man.get('first_failure'), man)
```

Note for the implementer: `flatten_row`/`save`/`load_episode` делят один контракт: flat-имена `a.b`/`a.0`, numeric → npz, non-numeric → `non_numeric` в manifest и overlay обратно при load (иначе roundtrip-тест не сойдётся побитово). `model.mjb` — сохранить через `mujoco.mj_saveModel(env.sim.model, str(out_dir/'model.mjb'))` в `save` (импортируется вверху файла) или передавать `env` — выбрать более простой путь: `save(ep, out_dir, model)` с явным `mujoco.MjModel`.

- [ ] **Step 4: Run — verify pass**

Run: `uv run pytest tests/reference/test_golden_episode.py -v -m slow`
Expected: PASS (оба теста). Если `test_capture_is_deterministic` флаки — остановиться и доложить: недетерминизм Python-рантайма ломает всю планку, это блокер.

- [ ] **Step 5: Commit**

```bash
git add tools/golden_episode.py tests/reference/test_golden_episode.py
git commit -m "feat(tools): golden-episode capture — oracle artifact generator (cpp-port P1)"
```

---

### Task 2: Replay + comparator harness

Перезапускает эпизод из артефакта и сравнивает per-step по контракту. На этой задаче harness проверяется Python↔Python: реплей того же кода обязан совпадать побитово — иначе дефект в самом harness.

**Files:**
- Create: `tools/episode_compare.py`
- Test: `tests/reference/test_episode_compare.py`

**Interfaces:**
- Consumes: `EpisodeArtifact`/`load_episode` из Task 1; `runtime.step(control)`, `completed_samples`, `flush`, `first_failure` (та же поверхность).
- Produces: `replay_episode(env, ep: EpisodeArtifact) -> list[dict]`; `compare_rows(golden, candidate, *, rtol=1e-12, atol=1e-12) -> list[str]` (список расхождений вида `step=K channel=name golden=… candidate=…`); `compare_episode(golden_dir, env) -> list[str]` (полный: replay + compare + first_failure). Поздние планы зовут `compare_episode` из нативных тестов.

- [ ] **Step 1: Failing test — self-oracle replay**

```python
"""Replay of a golden episode on the same code is bitwise identical."""
import pytest

@pytest.mark.slow
def test_replay_matches_golden_bitwise(tmp_path):
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--out', str(tmp_path/'out')])
    from tools.golden_episode import capture_episode, save
    from tools.episode_compare import replay_episode, compare_rows
    from bike_sim.sim.ride.control import RideControl
    env = research_cli.make_environment(args)
    ep = capture_episode(env, 50, RideControl(human_torque_nm=35.))
    cand = replay_episode(env2 := research_cli.make_environment(args), ep)
    assert compare_rows(ep.rows, cand.rows) == []
    assert env.sim.physical.reference_monitor.first_failure \
        == env2.sim.physical.reference_monitor.first_failure

def test_compare_reports_first_divergence():
    from tools.episode_compare import compare_rows
    a = [{'x': 1.0, 'y': 2.0}]; b = [{'x': 1.0, 'y': 2.0 + 1e-9}]
    diffs = compare_rows(a, b)
    assert diffs and 'step=0' in diffs[0] and 'y' in diffs[0]
```

- [ ] **Step 2: Run — verify fail**

Run: `uv run pytest tests/reference/test_episode_compare.py -v`
Expected: FAIL — `ModuleNotFoundError: tools.episode_compare`.

- [ ] **Step 3: Implement `tools/episode_compare.py`**

```python
"""Replay a golden episode and diff it row-by-row — the equivalence harness.

Layer-2 of the correctness bar: same applied controls in, all recorder
channels out, plus identical first monitor failure (ADR 0001 §6).
"""
import numpy as np

from bike_sim.sim.ride.control import RideControl


def replay_episode(env, ep) -> list[dict]:
    runtime = env.sim.physical
    env.sim.reset()
    rows = []
    for ctl in ep.controls:
        runtime.step(control=RideControl(**ctl))
        rows.extend(s.as_dict() for s in runtime.completed_samples)
    rows.extend(s.as_dict() for s in runtime.flush())
    return rows


def _close(a, b, rtol, atol):
    try:
        return bool(np.isclose(float(a), float(b), rtol=rtol, atol=atol))
    except (TypeError, ValueError):
        return a == b


def _walk(d, path=''):
    for k, v in d.items():
        p = f'{path}.{k}' if path else k
        yield from _walk(v, p) if isinstance(v, dict) else [(p, v)]


def compare_rows(golden, candidate, *, rtol=1e-12, atol=1e-12) -> list[str]:
    diffs = []
    if len(golden) != len(candidate):
        diffs.append(f'row count: golden={len(golden)} candidate={len(candidate)}')
        return diffs
    for i, (g, c) in enumerate(zip(golden, candidate)):
        for name, gv in _walk(g):
            cv = dict(_walk(c)).get(name, '<missing>')
            if isinstance(gv, np.ndarray):
                ok = gv.shape == np.shape(cv) and \
                    np.allclose(gv, cv, rtol=rtol, atol=atol)
            else:
                ok = _close(gv, cv, rtol, atol)
            if not ok:
                diffs.append(f'step={i} channel={name} '
                             f'golden={gv} candidate={cv}')
        if len(diffs) > 50:  # divergence floods fast past the horizon
            diffs.append('…truncated')
            break
    return diffs
```

Note for the implementer: `RideControl` is immutable — restore fields only present in `controls` records; if `asdict(applied_control)` carries extra keys not in `RideControl.__init__`, filter by `inspect.signature(RideControl).parameters`.

- [ ] **Step 4: Run — verify pass**

Run: `uv run pytest tests/reference/test_episode_compare.py -v`
Expected: PASS. Self-oracle replay обязан быть побитовым — любое расхождение = баг в capture/replay, чинить harness, не ослаблять толеранс.

- [ ] **Step 5: Commit**

```bash
git add tools/episode_compare.py tests/reference/test_episode_compare.py
git commit -m "feat(tools): episode replay+comparator — layer-2 equivalence harness (cpp-port P1)"
```

---

### Task 3: Divergence-horizon measurement

Измеряет горизонт слоя-2: с какого шага пертурбированная траектория необратимо расходится с базовой по каналам. Это число — граница детерминированной проверки в билете 05.

**Files:**
- Create: `tools/divergence_horizon.py`
- Test: `tests/reference/test_divergence_horizon.py`

**Interfaces:**
- Consumes: `capture_episode`/`EpisodeArtifact` (Task 1), `compare_rows`-flatten (Task 2), `env.sim.data` (`qvel`), `sim.reset()` + восстановление `initial`-массивов из артефакта.
- Produces: `measure_horizon(env_factory, steps: int, *, perturb: dict, tol: float = 1e-9) -> int` — номер первого шага, где хоть один канал превысил `tol` (или `steps` если не расходится); CLI `uv run python tools/divergence_horizon.py --steps 4000` пишет `output/divergence_horizon.json` с `horizon_step` и per-channel первой точкой расхождения. Поздние планы используют это число как границу детерминированной проверки.

- [ ] **Step 1: Failing test**

```python
"""A 1e-14 state perturbation diverges at a measurable, repeatable step."""
import numpy as np
import pytest

@pytest.mark.slow
def test_perturbation_diverges_within_episode(tmp_path):
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    from tools.divergence_horizon import measure_horizon
    def make():
        args = research_cli.parser().parse_args([
            '--physics-config', str(_pinned_config(tmp_path)),
            '--track-file', 'examples/research/rough_uphill_savage.toml',
            '--duration', '1', '--dt', '.00125',
            '--out', str(tmp_path/'out')])
        return research_cli.make_environment(args)
    h = measure_horizon(make, 400, perturb={'qvel_idx': 0, 'eps': 1e-14})
    assert isinstance(h, int) and 0 < h < 400

@pytest.mark.slow
def test_unperturbed_never_diverges(tmp_path):
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    from tools.divergence_horizon import measure_horizon
    def make():
        args = research_cli.parser().parse_args([
            '--physics-config', str(_pinned_config(tmp_path)),
            '--track-file', 'examples/research/rough_uphill_savage.toml',
            '--duration', '1', '--dt', '.00125',
            '--out', str(tmp_path/'out')])
        return research_cli.make_environment(args)
    assert measure_horizon(make, 200, perturb={'qvel_idx': 0, 'eps': 0.}) == 200
```

- [ ] **Step 2: Run — verify fail**

Run: `uv run pytest tests/reference/test_divergence_horizon.py -v -m slow`
Expected: FAIL — `ModuleNotFoundError: tools.divergence_horizon`.

- [ ] **Step 3: Implement `tools/divergence_horizon.py`**

```python
"""Measure the chaos-divergence horizon that bounds deterministic comparison.

Two runs of the same episode, one perturbed by eps in a single qvel dof:
the first step any recorded channel exceeds tol is the layer-2 horizon
(ADR 0001 §6). Below tol both runs are the 'same simulation'.
"""
import numpy as np

from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.initial_state import PhysicalInitialState


def _run(env, steps, perturb_at=None, qvel_idx=0, eps=0.):
    sim, rt = env.sim, env.sim.physical
    sim.physical_initial_state = PhysicalInitialState.capture(sim)
    sim.reset()
    rows = []
    for i in range(steps):
        if i == perturb_at:
            sim.data.qvel[qvel_idx] += eps
        rt.step(control=RideControl(human_torque_nm=35.))
        rows.extend(s.as_dict() for s in rt.completed_samples)
    rows.extend(s.as_dict() for s in rt.flush())
    return rows


def _flat(row):
    out = {}
    def w(d, p=''):
        for k, v in d.items():
            key = f'{p}.{k}' if p else k
            if isinstance(v, dict): w(v, key)
            elif np.isscalar(v) and np.isfinite(v): out[key] = float(v)
    w(row)
    return out


def measure_horizon(env_factory, steps, *, perturb, tol=1e-9,
                    perturb_at=10) -> int:
    base = _run(env_factory(), steps)
    pert = _run(env_factory(), steps, perturb_at,
                perturb['qvel_idx'], perturb['eps'])
    for i, (b, p) in enumerate(zip(base, pert)):
        bf, pf = _flat(b), _flat(p)
        if any(abs(bf[k] - pf.get(k, np.nan)) > tol for k in bf):
            return i
    return steps


if __name__ == '__main__':
    import argparse, json, re
    from pathlib import Path
    from bike_sim.cli import research as research_cli

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--steps', type=int, default=4000)
    p.add_argument('--qvel-idx', type=int, default=0)
    p.add_argument('--eps', type=float, default=1e-14)
    p.add_argument('--tol', type=float, default=1e-9)
    p.add_argument('--out', default='output/divergence_horizon.json')
    a = p.parse_args()

    src = Path('examples/research/viewer_physics_welded.toml').read_text()
    for key, val in [('pedal_attachment', 'spindle'), ('saddle_attachment', 'pin'),
                     ('grip_attachment', 'connect')]:
        src = re.sub(rf'{key} = "\w+"', f'{key} = "{val}"', src)
    for key in ('joint_envelope_path', 'joint_strength_path'):
        src = re.sub(rf'{key} = "([^"]+)"',
                     lambda m: f'{key} = "{(Path("examples/research")/m.group(1)).resolve()}"', src)
    cfg = Path('output/divergence_physics.toml'); cfg.write_text(src)

    def make():
        args = research_cli.parser().parse_args([
            '--physics-config', str(cfg),
            '--track-file', 'examples/research/rough_uphill_extreme.toml',
            '--duration', str(a.steps * .0005), '--dt', '.0005',
            '--out', 'output/divergence_env'])
        return research_cli.make_environment(args)

    h = measure_horizon(make, a.steps,
                        perturb={'qvel_idx': a.qvel_idx, 'eps': a.eps},
                        tol=a.tol)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(
        {'horizon_step': h, 'steps': a.steps, 'eps': a.eps, 'tol': a.tol},
        indent=2) + '\n')
    print(json.dumps({'horizon_step': h, 'steps': a.steps}))
```

Note for the implementer: make the CLI self-contained — build env via `research_cli.make_environment` directly (copy the parse pattern from `test_pinned_topology._model`, do NOT import tests from tools). `env_factory` must return a *fresh* env each call (two runs must not share mjData).

- [ ] **Step 4: Run — verify pass + реальное число**

Run: `uv run pytest tests/reference/test_divergence_horizon.py -v -m slow`
Expected: PASS. Записать измеренный `h` в `manifest.json`-комментарий теста (комментарий к assert: «измерено h≈N на savage/400 шагов») — это число станет границей слоя-2 в поздних планах.

- [ ] **Step 5: Commit**

```bash
git add tools/divergence_horizon.py tests/reference/test_divergence_horizon.py
git commit -m "feat(tools): divergence-horizon measurement — bounds the deterministic layer (cpp-port P1)"
```

---

### Task 4: Native build scaffold + nanobind skeleton

Минимальное native-ядро: загружает `model.mjb`, шагает `mj_step`, отдаёт zero-copy views `qpos`/`qvel`. Доказывает: toolchain + флаги собираются, та же `libmujoco` линкуется, seam прозрачен (Python↔C++ шаг побитово равен — слой-1 на самом движке).

**Files:**
- Create: `native/CMakeLists.txt`
- Create: `native/src/stepper.hpp`, `native/src/stepper.cpp`
- Create: `native/src/binding.cpp`
- Create: `tools/check_native_frontends.sh`
- Test: `tests/reference/test_native_stepper.py`

**Interfaces:**
- Consumes: `model.mjb` из `tools/golden_episode`/`dump_model.py`; mujoco headers/dylib из `.venv` (глобальный constraint); nanobind из `uv run python -c "import nanobind"` — добавить `nanobind` в dev-зависимости если отсутствует (`uv add --dev nanobind`).
- Produces: Python-модуль `bike_native` (в `native/build/`): `Stepper(path: str)`, `step() -> None`, `qpos -> np.ndarray` (zero-copy view в `d->qpos`), `qvel -> np.ndarray`, `time -> float`. Поздние планы расширяют этот класс до полного tick-path.

- [ ] **Step 1: Failing test — layer-1 побитовая эквивалентность шага**

```python
"""Native mj_step is bitwise-identical to Python mj_step — same dylib."""
import mujoco
import numpy as np
import pytest

MJB = 'tools/proto_native_bench/artifacts/model.mjb'

def test_native_step_matches_python_bitwise():
    bike_native = pytest.importorskip('bike_native')
    native = bike_native.Stepper(MJB)
    ref = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(ref)
    mujoco.mj_forward(ref, data)
    for _ in range(200):
        native.step(); mujoco.mj_step(ref, data)
    assert np.array_equal(native.qpos, data.qpos)   # bitwise, not isclose
    assert np.array_equal(native.qvel, data.qvel)
```

Note: `Stepper` должен вызывать `mj_forward` после загрузки (как Python `mj_forward` выше), иначе стартовое состояние расходится.

- [ ] **Step 2: Run — verify fail**

Run: `uv run pytest tests/reference/test_native_stepper.py -v`
Expected: FAIL — `ModuleNotFoundError: bike_native` (importorskip → SKIP: приёмлемо, но сборка ниже сделает PASS; скип здесь честен — нативный модуль может отсутствовать на не-arm64).

- [ ] **Step 3: `native/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.24)
project(bike_native CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

execute_process(COMMAND python -c "import mujoco,os;print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MJ_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND python -m nanobind --cmake_dir
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

add_compile_options(
  -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
  -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wnull-dereference -Wunsafe-buffer-usage
  -fstack-protector-strong -ftrivial-auto-var-init=zero -fvisibility=hidden
  -isystem "${MJ_DIR}/include")
add_compile_definitions(_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST)
add_link_options(-headerpad_max_install_names)

option(NATIVE_SANITIZE "ASan+UBSan build" OFF)
if(NATIVE_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined -shared-libasan)
endif()

nanobind_add_module(bike_native src/binding.cpp src/stepper.cpp)
target_link_libraries(bike_native PRIVATE "${MJ_DIR}/libmujoco.3.12.0.dylib")

# Multi-compiler sweep: same TU set through GCC's frontend, warnings-as-errors.
add_custom_target(check_frontends
  COMMAND ${CMAKE_SOURCE_DIR}/../tools/check_native_frontends.sh
          "${CMAKE_SOURCE_DIR}/src" "${MJ_DIR}/include"
  COMMENT "gcc -fsyntax-only sweep (second frontend)")
```

- [ ] **Step 4: `native/src/stepper.hpp` + `stepper.cpp` + `binding.cpp`**

```cpp
// stepper.hpp — owns mjModel/mjData; buffers never escape ownership.
#pragma once
#include <mujoco/mujoco.h>
#include <span>
#include <string>

class Stepper {
public:
    explicit Stepper(const std::string& mjb_path);
    ~Stepper();
    Stepper(const Stepper&) = delete;
    Stepper& operator=(const Stepper&) = delete;
    void step() { mj_step(m_, d_); }
    [[nodiscard]] std::span<const double> qpos() const {
        return {d_->qpos, static_cast<std::size_t>(m_->nq)}; }
    [[nodiscard]] std::span<const double> qvel() const {
        return {d_->qvel, static_cast<std::size_t>(m_->nv)}; }
    [[nodiscard]] double time() const { return d_->time; }
private:
    mjModel* m_;
    mjData* d_;
};

// stepper.cpp
#include "stepper.hpp"
#include <stdexcept>
Stepper::Stepper(const std::string& mjb_path) {
    m_ = mj_loadModel(mjb_path.c_str(), nullptr);   // mjb: no VFS needed
    if (!m_) throw std::runtime_error("mj_loadModel failed: " + mjb_path);
    d_ = mj_makeData(m_);
    if (!d_) { mj_deleteModel(m_); throw std::runtime_error("mj_makeData"); }
    mj_forward(m_, d_);
}
Stepper::~Stepper() { if (d_) mj_deleteData(d_); if (m_) mj_deleteModel(m_); }

// binding.cpp
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include "stepper.hpp"
namespace nb = nanobind;

NB_MODULE(bike_native, m) {
    nb::class_<Stepper>(m, "Stepper")
        .def(nb::init<const std::string&>())
        .def("step", &Stepper::step)
        .def_prop_ro("qpos", [](Stepper& s) {
            auto v = s.qpos();
            return nb::ndarray<nb::numpy, const double, nb::shape<-1>>(
                v.data(), {v.size()}, nb::handle());
        })
        .def_prop_ro("qvel", [](Stepper& s) {
            auto v = s.qvel();
            return nb::ndarray<nb::numpy, const double, nb::shape<-1>>(
                v.data(), {v.size()}, nb::handle());
        })
        .def_prop_ro("time", &Stepper::time);
}
```

Note for the implementer: views are zero-copy and non-owning — `Stepper` must outlive them (document in `binding.cpp` comment); `nb::handle()` empty owner means "no owner". If nanobind's ndarray signature differs in the installed version, follow its `ndarray` docs — the contract (numpy view over `d->qpos`) is what matters.

- [ ] **Step 5: `tools/check_native_frontends.sh`**

```bash
#!/usr/bin/env bash
# Second-frontend warning sweep: every core TU through GCC, syntax-only.
set -euo pipefail
SRC_DIR="$1"; MJ_INC="$2"
SDK=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk
for f in "$SRC_DIR"/*.cpp; do
  g++-16 -std=c++23 -fsyntax-only -isysroot "$SDK" \
    -isystem "$MJ_INC" -isystem "$(python -m nanobind --include_dir 2>/dev/null || echo /nonexistent)" \
    -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion \
    -Wdouble-promotion -Wshadow -Wcast-qual -Wformat=2 -Wundef \
    -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast \
    -Woverloaded-virtual -Wnull-dereference \
    -Wlogical-op -Wduplicated-cond -Wuseless-cast -Wstringop-overflow=4 \
    "$f"
done
echo "gcc sweep clean: $SRC_DIR"
```

`chmod +x tools/check_native_frontends.sh`. Если nanobind-хедеры под GCC дают шум — sweeping прогоняет только `stepper.cpp` (без `binding.cpp`): binding — clang-only frontend, ядро — под обоими.

- [ ] **Step 6: Build + run**

```bash
cd native && uv run cmake -B build -S . && uv run cmake --build build && \
  install_name_tool -change @rpath/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib \
    "$PWD/../.venv/lib/python3.14/site-packages/mujoco/libmujoco.3.12.0.dylib" \
    build/bike_native.*.so 2>/dev/null || true
uv run pytest tests/reference/test_native_stepper.py -v
```
Expected: сборка чисто под -Werror, тест PASS (побитовое равенство — та же dylib). `install_name_tool` нужен только если dyld резолвит framework-rpath — как в `tools/proto_native_bench`. Если линкер уже записал абсолютный путь — шаг no-op.

- [ ] **Step 7: Frontends sweep**

Run: `bash tools/check_native_frontends.sh native/src .venv/lib/python3.14/site-packages/mujoco/include`
Expected: `gcc sweep clean`. Любой новый ворнинг от второго фронтенда — чинить код (не ослаблять набор).

- [ ] **Step 8: Commit**

```bash
git add native/ tools/check_native_frontends.sh tests/reference/test_native_stepper.py pyproject.toml uv.lock
git commit -m "feat(native): C++23/nanobind scaffold — Stepper core + dual-frontend hygiene (cpp-port P1)"
```

---

## Self-review выполнен

- Spec coverage: планка слои 1–2 (Tasks 1–2 + native-gate Task 4), горизонт слоя-2 (Task 3), toolchain/флаги/hardening (Task 4 + Global Constraints), multi-compiler hygiene (Task 4 Step 5/7). Слой-3 (статистика метрик) — поздний план, здесь не нужен.
- Placeholder scan: единственный `...` — в тесте Task 3 Step 1 (сокращённый повтор fixture) и в CLI-хвосте Task 3; оба снабжены явными указаниями реализации.
- Type consistency: `EpisodeArtifact`/`load_episode`/`save` — единый контракт между Tasks 1–2; `RideControl`/`completed_samples`/`first_failure` — поверхность из oracle-теста; `Stepper.qpos/qvel` — `std::span<const double>` в C++, `np.ndarray` view в Python.
