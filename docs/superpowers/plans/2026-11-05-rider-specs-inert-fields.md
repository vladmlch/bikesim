# RiderSpecs Inert-Field Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Standing user instruction for this tree:** no subagents. Execute inline with superpowers:executing-plans.

**Goal:** Remove the ten seated-rider fields that no remaining rider variant reads from `RiderSpecs`. Old recordings and stored initial states keep loading: default values are dropped silently, and any other value is rejected.

**Architecture:** The pose solver is narrowed to the geometry its two live callers use (`geometry_pose`, `compute_rider_centers_of_mass`). That removes every reader of the fields except validation and serialization. A new `configuration/legacy_rider.hpp` follows the `legacy_battery.hpp` pattern. It names the removed members with their shipped defaults, finds a non-default one, erases them, and re-adds the defaults so the old initial-state contract can be recomputed. Replay and the initial-state contract use it. The physics outputs (MJCF, goldens, binary fixtures) stay bitwise unchanged.

**Tech Stack:** C++23 native tree (`native/`), CMake/CTest, `mise` tasks, JSON fixtures under `tests/fixtures/`.

**Spec:** No separate spec. The decisions below are the contract. Background: `docs/superpowers/plans/2026-11-04-remove-legacy-mode.md`, the paragraph "Inert seated-rider fields of `RiderSpecs`" (line 47), and `native/src/configuration/legacy_battery.hpp` (the pattern to mirror). Read `docs/agents/native-engineering.md` before Task 1.

## Global Constraints

- Removed fields (exactly ten): `legs`, `saddle_share`, `pedal_share`, `bar_share`, `torso_resonance_hz`, `leg_resonance_hz`, `arm_resonance_hz`, `damping_ratio`, `saddle_interface_k_n_m`, `saddle_interface_c_ns_m`.
- Their shipped defaults: `"rigid"`, `0.55`, `0.33`, `0.12`, `7.0`, `5.0`, `4.0`, `0.40`, `101000.0`, `2762.0`.
- Remaining `RiderSpecs` fields: `variant`, `mass_kg`, `height_m`, `inseam_m`, `helmet_mass_kg`, in that order.
- Physics goldens, `.mjb`/`.npz`/`.f64` contents of kept arrays, and every `tests/fixtures/golden/**` file stay byte-identical. Do not re-record anything.
- Do not edit these legacy-evidence fixtures; the compatibility code must accept them as they are:
  - `tests/fixtures/initial_state/**`
  - `tests/fixtures/golden/*/**/initial_state.json`
  - `tests/fixtures/replay/**`
  - `tests/fixtures/ft2/**`
- Fixture JSON edits are deletions, plus rewritten `_notes`/`_produced_by` lines. Files are written with `json.dumps(doc, indent=2, sort_keys=True) + "\n"`, which is their current format; the scripts assert this before writing.
- No Linux runs (`tools/linux/container.sh` is not used). Linux fixtures need no change because none are re-recorded.
- Commit messages: Conventional Commits as in `git log` (`refactor(rider): …`, `test(model): …`). End each with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Work in `.worktrees/no-joint-damping`. It has a built `build/native-release`.
- Test budget, from `<git-common-dir>/bike-gate/timings.tsv`:
  - **Baseline.** `dbc6ab7e full passed 1413 s`. It is main HEAD, so it is reused, not re-run.
  - **Per task.** A targeted `mise run test '<pattern>'` (about 1 min), then `bash tools/check_change.sh <base>` (≤5 min) at the end of Tasks 1–4: about 20 min.
  - **Task 4 extra.** The slow tests that read stored initial states: about 5–8 min.
  - **Final proof.** One level 2 `bash tools/run_tests.sh native` (measured 642 s on `20e2d779`): about 11 min.
  - **Total.** About 40 min.
  - **Why not level 3.** This is not a physics change. The slow goldens that read the touched contract run in Task 4.

## Decisions

- **D1 — Scope is all ten fields.** The remove-legacy-mode plan listed seven: `legs` plus the six resonance/damping/interface fields. The three load shares are added here for two reasons. Their only reader is `seated_path_masses`, which feeds only the removed seated bodies. And `geometry_pose` already overrides them with constants before solving.
- **D2 — The pose solver keeps only geometry.** These have no production reader (verified by grep on `dbc6ab7e`; only `test_inertia.cpp` reads them) and are deleted:
  - `RiderGeom`, `RiderBody`, `validate_rider_body`, `LegChain`;
  - `SeatedPose::bodies`, `path_masses_kg`, `leg_chains`, `body()`, `total_mass_kg()`, `interface_loads_n()`, `interface_shares()`;
  - `spring_params`, `seated_path_masses`, `pitch_xz`, `pedal_spindle_pos`, `solve_leg_joints`, `solve_leg_qpos`, `LegFkResult`, `fk_leg`, `Mat32`, `leg_jacobian`, `crank_torque_share`;
  - `saddle_path_apparent_mass` and its complex helpers;
  - `segment_radius_m`, `has_segment_radius`, `kSegmentRadii`, `mass_of`.

  One behaviour change is accepted. `geometry_pose` no longer throws the path-mass errors ("bar share …", "pedal share …", "saddle share … leaves … kg for the pelvis") or the `RiderBody` domain errors. With the forced shares those errors were reachable only for a helmet mass near the rider mass, and no CLI option or random sampler sets the helmet mass.
- **D3 — Old documents.**
  - A removed member that holds its shipped default is dropped silently.
  - Any other value, or a member of the wrong JSON kind, fails closed:
    - replay throws `IncompatibleExecution` naming `resolved_config.rider.<field>`;
    - the initial-state contract does not match.
  - This mirrors the battery rule, even though Task 1 shows that the values never reached a model. The replay contract stays strict: only documents that shipped configs could write are promised.
- **D4 — Initial-state contract.** `validated()` accepts four forms: {current, legacy battery} × {current, legacy rider}. The legacy rider form re-adds the ten defaults to the identity rider section. `canonical_json` sorts keys (`native/src/io/json.cpp:1735`), so only the number lexemes matter, not member position. Stored initial states are not re-recorded.
- **D5 — No compatibility shim for:**
  - **The canonical equilibrium cache key** (`initialization/equilibrium_cache.cpp` `context_fingerprint`). It includes `source_identity`, the native source hash, so any code change already invalidates it.
  - **The historical NPZ branch** (`legacy == true`, opt-in `--legacy-source-root`). It already hashes the current physics document without `drive.battery`, so Python-era payloads stopped matching at the battery removal. No test reaches it.

## File Structure

| File | Change | Responsibility after the change |
|---|---|---|
| `native/src/model/rider_geometry.hpp` / `.cpp` | Modify (Task 2, Task 4) | Seated geometry (`solve_seated_pose`, `geometry_pose`) and rider centres of mass only |
| `native/tests/model/test_inertia.cpp` | Modify (Task 2, Task 4) | Geometry, helper and centre-of-mass parity tests |
| `tests/fixtures/model/seated_pose.json` + `.arrays/` | Modify (Task 2) | Geometry of the default and 75 kg riders |
| `tests/fixtures/model/rider_helpers.json` + `.arrays/` | Modify (Task 2) | IK and flexion cases |
| `tests/fixtures/model/rider_legs_centers.json` + `.arrays/` | Modify (Task 2) | Per-variant centre-of-mass tables (the file keeps its name: the `$array` paths inside embed it) |
| `native/tests/model/test_mjcf.cpp` | Modify (Task 1, Task 4) | MJCF contracts |
| `native/src/configuration/legacy_rider.hpp` | Create (Task 3) | Reading documents written before the fields were removed |
| `native/tests/config/test_schema.cpp` | Modify (Task 3, Task 4) | `RiderSpecs` schema and the legacy-rider helpers |
| `native/src/configuration/rider_spec.hpp` | Modify (Task 4) | The five-field record |
| `native/src/configuration/validate.cpp` | Modify (Task 4) | `post_init(RiderSpecs&)` without the legs/shares checks |
| `native/src/replay/rebuild.cpp` | Modify (Task 4) | Rejects non-default removed members; strips them before the recipe compare |
| `native/src/initialization/initial_state.cpp` | Modify (Task 4) | Accepts the legacy rider contract form |
| `native/tests/reference/test_replay_recording_v3.cpp` | Modify (Task 4) | The removed-rider-fields gate |
| `native/tests/random/test_seeded.cpp`, `tests/fixtures/random/rider-samples.json` | Modify (Task 4) | Sampled-rider parity without the removed fields |
| `tests/fixtures/model/rider_specs.json` | Modify (Task 4) | `resolve_rider`/`post_init` parity without the removed fields |
| `docs/TESTING.md` | Modify (Task 5) | The legacy-document rule |

---

### Task 0: Branch and baseline

**Files:** none.

- [ ] **Step 1: Create the branch in the built worktree**

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/no-joint-damping
git status --short            # expected: empty
git rev-parse HEAD main       # expected: both dbc6ab7e5390b29b1a302aa9603cd79af91b3d13
git switch -c rider-specs-inert-fields
```

- [ ] **Step 2: Record the baseline instead of re-running it**

```bash
grep dbc6ab7e "$(git rev-parse --git-common-dir)/bike-gate/timings.tsv"
```

Expected: a row `… dbc6ab7e full release … passed 1413 …`. This is the baseline. Do not run a gate here.

---

### Task 1: Pin that all ten fields are inert (characterization)

The existing `seated_parameters_are_inert` covers six fields. Extend it to the three shares and `legs`. This test is the evidence for D1/D3. It must pass on unchanged production code.

**Files:**
- Modify: `native/tests/model/test_mjcf.cpp:222-240` (`seated_parameters_are_inert`)

**Interfaces:** Consumes `mdl::BuildRequest`, `mdl::generate_mjcf`, `cfg::load_physics`, `bt::fixture`, as used in the existing test. Produces nothing used later.

- [ ] **Step 1: Extend the test**

Replace the whole function (comment included) with:

```cpp
// The seated-only parameters (legs mode, load shares, resonances,
// damping ratio, saddle interface) do not reach any remaining model:
// changing them leaves the MJCF of the lumped and articulated_planar
// riders byte-identical. This is the evidence that lets RiderSpecs drop
// them (configuration/legacy_rider.hpp reads old documents).
[[gnu::noinline]] void seated_parameters_are_inert() {
    for(const auto* variant:{"lumped","articulated_planar"}) {
        mdl::BuildRequest request;request.mode="ride";request.include_rider=true;
        request.rider=cfg::RiderSpecs{};request.rider->variant=variant;
        if(std::string_view(variant)=="articulated_planar")
            request.physics=cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
        const auto before=mdl::generate_mjcf(request);
        auto& rider=*request.rider;
        rider.legs="articulated";
        // Shares still sum to one so post_init accepts the record.
        rider.saddle_share=.50;rider.pedal_share=.35;rider.bar_share=.15;
        rider.torso_resonance_hz=9.;rider.leg_resonance_hz=8.;rider.arm_resonance_hz=7.;
        rider.damping_ratio=.9;rider.saddle_interface_k_n_m=1.;rider.saddle_interface_c_ns_m=1.;
        cfg::detail::post_init(rider);
        bt::require(mdl::generate_mjcf(request)==before,
                    std::string(variant)+": seated-only parameters do not change the model");
    }
}
```

If `cfg::detail::post_init` is not visible in `test_mjcf.cpp`, add `#include "configuration/validate.hpp"`. Find the header that declares `post_init(RiderSpecs&)` with `grep -rn "void post_init(RiderSpecs" native/src/configuration/*.hpp`, and include that header.

- [ ] **Step 2: Run it**

```bash
mise run test 'model_mjcf'
```

Expected: PASS. A FAIL means one of the ten fields still reaches the model. Stop, and report which field (bisect by setting one field at a time). The plan's premise is then wrong.

- [ ] **Step 3: Second, independent check (native-engineering rule 2)**

```bash
grep -rn "\.legs\b\|saddle_share\|pedal_share\|bar_share\|resonance_hz\|damping_ratio\|saddle_interface" native/src \
  | grep -v "configuration/rider_spec.hpp\|configuration/validate.cpp\|replay/rebuild.cpp"
```

Expected: hits only in `native/src/model/rider_geometry.cpp`:
- `seated_path_masses`;
- the `spring_params(...)` calls and the `RiderBody pelvis` interface fields in `solve_seated_pose`;
- the `rider.legs == "articulated"` branch;
- the overrides in `geometry_pose`.

Each of these feeds only the dead outputs listed in D2. Record the output in the task report.

- [ ] **Step 4: Stage check and commit**

```bash
bash tools/check_change.sh dbc6ab7e
git add native/tests/model/test_mjcf.cpp
git commit -m "test(model): pin all ten seated-only rider fields as inert

The legs mode and load shares join the resonance/damping/interface set:
none of them changes the lumped or articulated_planar MJCF. This is the
evidence for dropping them from RiderSpecs.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

Expected: `check_change.sh` exits 0.

---

### Task 2: Narrow the pose solver to geometry

After this task the ten fields have no reader except `post_init`, `geometry_pose`'s overrides, replay parsing and the schema table.

**Files:**
- Modify: `native/src/model/rider_geometry.hpp` — delete the declarations listed in D2. Keep `two_link_ik`, `flexion_deg`, `de_leva_mass_fraction`, `has_de_leva_mass_fraction`, `lumped_com_m`, the constants, `SaddleGeometry`, the trimmed `SeatedPose`, `solve_seated_pose`, both `seated_pose` overloads, `ArticulatedPose`, `geometry_pose`, `RiderCenter`, `compute_rider_centers_of_mass`, `rider_from_args`, `resolve_rider` and the `rider_segments` section.
- Modify: `native/src/model/rider_geometry.cpp`:
  - the file-head comment, lines 1-12;
  - the `Cpx` block, about lines 123-183;
  - `kSegmentRadii` and `mass_of`;
  - `segment_radius_m` / `has_segment_radius`;
  - lines 262-378 (`validate_rider_body`, `RiderBody::*`, `SeatedPose::*` methods);
  - lines 432-583 (`spring_params` … `crank_torque_share`);
  - the `make_geom` anonymous namespace;
  - the tail of `solve_seated_pose`;
  - `saddle_path_apparent_mass`, about lines 1003-1045.
- Modify: `native/tests/model/test_inertia.cpp`:
  - `expect_body`, `expect_chain` and `expect_pose` (885-1043);
  - `test_seated_pose` (1045-1068);
  - `test_rider_helpers` (1072-1175);
  - `test_rider_legs_centers` (1177-1265);
  - the comment at about line 642;
  - `main` at about line 2248.
- Modify: `tests/fixtures/model/{seated_pose,rider_helpers,rider_legs_centers}.json` and their `.arrays/` directories.

**Interfaces:**
- Consumes: nothing new.
- Produces the trimmed `SeatedPose` with exactly these members: `saddle`, `hip`, `shoulder`, `elbow`, `wrist`, `grip`, `head_center`, `knee_front`, `knee_rear`, `ankle_front`, `ankle_rear`, `pedal_front`, `pedal_rear`, `torso_lean_deg`, `knee_flexion_front_deg`, `knee_flexion_rear_deg`, `knee_flexion_bdc_deg`.
- The test function `test_rider_legs_centers` is renamed to `test_rider_centers`.

- [ ] **Step 1: Trim the tests first**

In `test_inertia.cpp`:

1. Delete `expect_body` and `expect_chain` entirely.
2. In `expect_pose`, delete everything from `const Value::Array &bodies =` to the end of the function, through the `missing_body_error` block. Keep the saddle, point, lean and flexion checks.
3. In `test_seated_pose`, delete the `artic` record and the `legs_articulated` `expect_pose` call. The function becomes:

```cpp
void test_seated_pose() {
    const Value doc = doc_of("model/seated_pose");
    const geo::BikeSpecs specs;

    mdl::RiderSpecs rider_75;
    rider_75.mass_kg = 75.0;
    rider_75.height_m = 1.75;
    cfg::detail::post_init(rider_75);

    expect_pose(member(doc, "default", "doc"),
                mdl::solve_seated_pose(specs, mdl::RiderSpecs{}),
                "default");
    expect_pose(member(doc, "rider_75_175", "doc"),
                mdl::solve_seated_pose(specs, rider_75), "rider_75_175");
}
```

4. In `test_rider_helpers`, delete the `spring_cases` loop and the `path_cases` loop. The function ends after the `flexion_cases` loop.
5. Replace `test_rider_legs_centers` with:

```cpp
// ---- model/rider_legs_centers (centre-of-mass tables) -----------------------

void test_rider_centers() {
    const Value doc = doc_of("model/rider_legs_centers");
    const geo::BikeSpecs specs;
    const Value &variants = member(doc, "center_variants", "doc");
    const auto rider_for = [](std::string_view tag) {
        mdl::RiderSpecs r;
        r.variant = std::string(tag);
        cfg::detail::post_init(r);
        return r;
    };
    for (const std::string_view tag:
             {"none", "lumped", "articulated_planar"}) {
        const Value &v = member(variants, tag, "variants");
        const mdl::RiderSpecs r = rider_for(tag);
        bt::require(outcome_accepted(v, "variant " + std::string(tag)),
                    "center variant must accept: " + std::string(tag));
        expect_centers(*outcome_result(v, "variant " + std::string(tag)),
                       mdl::compute_rider_centers_of_mass(r, specs),
                       "variant " + std::string(tag));
    }
}
```

6. In `main`, replace `test_rider_legs_centers();` with `test_rider_centers();`.
7. The comment at about line 642 ("bitwise-pinned by center_variants.lumped in rider_legs_centers.json") stays true. Leave it.
8. After the edits, delete helpers that became unused: `expect_pairs`, `member_opt`, `bool_of`, `expect_key_error`, `scalar_list`, `pairs_of`. Check each with `grep -n "<name>(" native/tests/model/test_inertia.cpp`, and delete only those with zero remaining callers. Unused functions in the anonymous namespace fail the build (`-Wunused-function`).

- [ ] **Step 2: Trim the fixtures**

Run from the worktree root:

```bash
python3 -I - <<'EOF'
import json, pathlib
root = pathlib.Path('tests/fixtures')

def load(name):
    path = root / f'{name}.json'
    text = path.read_text()
    doc = json.loads(text)
    assert json.dumps(doc, indent=2, sort_keys=True) + '\n' == text, f'{name}: formatting would change'
    return path, doc

def save(path, doc):
    path.write_text(json.dumps(doc, indent=2, sort_keys=True) + '\n')

def prune_arrays(name, doc):
    """Delete .arrays/*.f64 files the edited document no longer references."""
    refs = set()
    def walk(v):
        if isinstance(v, dict):
            if '$array' in v: refs.add(v['$array'])
            for x in v.values(): walk(x)
        elif isinstance(v, list):
            for x in v: walk(x)
    walk(doc)
    folder = root / f'{name}.arrays'
    removed = 0
    for f in sorted(folder.iterdir()):
        if f'{name}.arrays/{f.name}' not in refs:
            f.unlink(); removed += 1
    missing = [r for r in refs if not (root / r).exists()]
    assert not missing, missing
    print(name, 'removed', removed, 'kept', len(refs))

path, doc = load('model/seated_pose')
del doc['legs_articulated']
for case in ('default', 'rider_75_175'):
    for key in ('bodies', 'interface_loads_n', 'interface_shares', 'leg_chains',
                'missing_body_error', 'path_masses_kg', 'total_mass_kg'):
        del doc[case][key]
doc['_notes'] = 'solved SeatedPose geometry for the default and 75 kg riders'
doc['_produced_by'] = 'solve_seated_pose over two RiderSpecs'
save(path, doc); prune_arrays('model/seated_pose', doc)

path, doc = load('model/rider_helpers')
del doc['spring_cases'], doc['path_cases']
doc['_notes'] = 'two-link IK branches and knee flexion'
doc['_produced_by'] = 'rider.py _two_link_ik/_flexion_deg'
save(path, doc); prune_arrays('model/rider_helpers', doc)

path, doc = load('model/rider_legs_centers')
del doc['leg_cases'], doc['apparent_mass']
doc['_notes'] = 'per-variant rider CoM tables'
doc['_produced_by'] = 'rider.py compute_rider_centers_of_mass'
save(path, doc); prune_arrays('model/rider_legs_centers', doc)
EOF
git diff -U0 -- tests/fixtures/model/*.json | grep '^+[^+]'
```

Expected:
- The script prints three `removed N kept M` lines, with no assertion error.
- The final `grep` prints only the six `"_notes"` / `"_produced_by"` lines. Any other added line means the formatting changed; restore with `git checkout -- tests/fixtures/model` and investigate.

- [ ] **Step 3: Run the trimmed tests on the unchanged solver**

```bash
mise run test 'model_inertia'
```

Expected: PASS. The kept geometry still matches bitwise before any production change.

- [ ] **Step 4: Delete the dead solver code**

In `rider_geometry.cpp`, cut `solve_seated_pose` right after `head_center` is computed. Delete from `const auto masses = seated_path_masses(rider);` through the `path_masses` loop. The return becomes:

```cpp
    return SeatedPose{
            .saddle = saddle,
            .hip = hip,
            .shoulder = shoulder,
            .elbow = elbow,
            .wrist = wrist,
            .grip = grip,
            .head_center = head_center,
            .knee_front = knee_front,
            .knee_rear = knee_rear,
            .ankle_front = ankle_front,
            .ankle_rear = ankle_rear,
            .pedal_front = pedal_front,
            .pedal_rear = pedal_rear,
            .torso_lean_deg = torso_lean,
            .knee_flexion_front_deg =
                    flexion_deg(hip, knee_front, ankle_front),
            .knee_flexion_rear_deg =
                    flexion_deg(hip, knee_rear, ankle_rear),
            .knee_flexion_bdc_deg = knee_flexion_bdc,
    };
```

Then delete the D2 definitions listed under **Files** above. Rewrite the file-head comment. Drop the sentence about `saddle_path_apparent_mass`'s complex ops, and keep the `pyfloat` and sin/cos notes.

Check whether the frame-size suppression is still needed: delete the `NATIVE_DIAG_PUSH` / `NATIVE_DIAG_IGNORE("-Wframe-larger-than")` / `NATIVE_DIAG_POP` around `solve_seated_pose`. If the build then warns on `-Wframe-larger-than`, put them back unchanged. If it does not warn, also delete the comment above them (it explains the suppression).

Remove includes that the deleted code alone needed. `clang-tidy misc-include-cleaner` in `check_change.sh` names them; `numeric_sum.hpp` is the likely one.

In `rider_geometry.hpp`, delete the D2 declarations and their comments. Then update two places:
- The `seated_pose` comment, to: "the solver takes the record as geometry input; only positions and angles are produced".
- The line-20 comment that mentions `saddle_path_apparent_mass`.

`geometry_pose` keeps its `legs`/share overrides in this task; Task 4 removes them with the fields.

- [ ] **Step 5: Build and run the targeted tests**

```bash
mise run test 'model_inertia|model_mjcf|initialization_|research_environment|reference.native_rider_contacts'
```

Expected: PASS. The `geometry_pose` consumers (`runtime/project_config.cpp`, the articulated rider) are exercised by `research_environment` and `initialization_*`.

- [ ] **Step 6: Audit that nothing else read the deleted outputs**

```bash
grep -rn "RiderBody\|LegChain\|spring_params\|seated_path_masses\|saddle_path_apparent_mass\|interface_shares\|path_masses_kg\|leg_chains\|segment_radius" native/src native/tests docs/*.md
```

Expected: no output.

- [ ] **Step 7: Stage check and commit**

```bash
bash tools/check_change.sh "$(git rev-parse HEAD)"
git add -A native/src/model/rider_geometry.hpp native/src/model/rider_geometry.cpp native/tests/model/test_inertia.cpp tests/fixtures/model
git commit -m "refactor(rider): keep only seated geometry in the pose solver

The spring-damper bodies, leg chains, path masses and apparent mass fed
only the removed seated variant. The two remaining callers read
positions and angles, which stay bitwise equal (seated_pose fixture).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

Expected: `check_change.sh` exits 0. Its `base` is the Task 1 commit, which is HEAD before this commit.

---

### Task 3: `configuration/legacy_rider.hpp`

**Files:**
- Create: `native/src/configuration/legacy_rider.hpp`
- Test: `native/tests/config/test_schema.cpp` (new function `legacy_rider_fields`, called from `main`)

**Interfaces** (namespace `bike::configuration::legacy`; Task 4 relies on these exact names):
- `inline constexpr std::string_view kRemovedRiderLegsKey` = `"legs"`
- `inline constexpr std::string_view kRemovedRiderLegsDefault` = `"rigid"`
- `inline constexpr std::array<std::pair<std::string_view, double>, 9> kRemovedRiderReals`
- `[[nodiscard]] std::optional<std::string_view> non_default_rider_field(const io::Value& rider)`
- `void erase_removed_rider_fields(io::Value& rider)`
- `[[nodiscard]] io::Value with_removed_rider_defaults(io::Value rider)`

- [ ] **Step 1: Write the failing test**

Add `#include "configuration/legacy_rider.hpp"` and `#include "configuration/load.hpp"` to `test_schema.cpp`, if they are not already present. Add after `test_rider()`:

```cpp
// The declared default rider exactly as the record writer serialized it
// before the seated-only fields were removed: key-sorted canonical JSON
// with the writer's number lexemes. Initial-state contracts written then
// hash this text (via source_json, which is ASCII-identical here).
constexpr std::string_view kPreRemovalDefaultRider =
    R"({"arm_resonance_hz":4.0,"bar_share":0.12,"damping_ratio":0.4,)"
    R"("height_m":1.8,"helmet_mass_kg":0.4,"inseam_m":null,)"
    R"("leg_resonance_hz":5.0,"legs":"rigid","mass_kg":80.0,)"
    R"("pedal_share":0.33,"saddle_interface_c_ns_m":2762.0,)"
    R"("saddle_interface_k_n_m":101000.0,"saddle_share":0.55,)"
    R"("torso_resonance_hz":7.0,"variant":"lumped"})";

// Documents written before the seated-only fields were removed still
// carry them. Default members are inert and erase; any other value is
// named; re-adding the defaults rebuilds the exact pre-removal text the
// old initial-state contracts hashed. Holds before and after the fields
// leave RiderSpecs.
void legacy_rider_fields() {
    namespace legacy = cfg::legacy;
    const io::Value current = cfg::detail::plain_document(cfg::detail::as_object(
        cfg::detail::kRiderSpecsSchema, &cfg::detail::kDeclaredRiderSpecs));
    io::Value current_erased = current;
    legacy::erase_removed_rider_fields(current_erased);
    bt::require(io::canonical_json(legacy::with_removed_rider_defaults(current_erased)) ==
                    kPreRemovalDefaultRider,
                "re-added defaults rebuild the pre-removal serialization");

    const io::Value old = io::parse_json(kPreRemovalDefaultRider);
    bt::require(!legacy::non_default_rider_field(old).has_value(),
                "the shipped defaults are inert");
    io::Value old_erased = old;
    legacy::erase_removed_rider_fields(old_erased);
    bt::require(io::canonical_json(old_erased) == io::canonical_json(current_erased),
                "erasing leaves exactly the current record");
    bt::require(!legacy::non_default_rider_field(old_erased).has_value(),
                "absent members are inert");

    const auto mutated = [&](std::string_view key, io::Value value) {
        io::Value doc = old;
        for (auto& member : std::get<io::Value::Object>(doc.data))
            if (member.first == key) member.second = std::move(value);
        return doc;
    };
    bt::require(legacy::non_default_rider_field(mutated("legs", io::Value{std::string("articulated")})) ==
                    std::optional<std::string_view>{"legs"},
                "articulated legs are named");
    for (const auto& [name, value] : legacy::kRemovedRiderReals) {
        bt::require(legacy::non_default_rider_field(mutated(name, io::Value{io::Number{"0.5"}})) ==
                        std::optional<std::string_view>{name},
                    std::string(name) + ": a non-default value is named");
        bt::require(legacy::non_default_rider_field(mutated(name, io::Value{std::string("0.55")})) ==
                        std::optional<std::string_view>{name},
                    std::string(name) + ": a wrong-kind value is named");
    }
    io::Value twice = legacy::with_removed_rider_defaults(current_erased);
    bt::invalid([&] { (void)legacy::with_removed_rider_defaults(std::move(twice)); });
}
```

Call `legacy_rider_fields();` from `main` right after `test_rider();`.

Check the test-support names before building:
- `bt::invalid` is used by `native/tests/model/test_mjcf.cpp`.
- `io::Number{"0.5"}` is the aggregate used in `legacy_battery.hpp`.
- If `test_schema.cpp` has its own `expect_invalid` helper instead of `bt::invalid`, use `expect_invalid` with the message `"legacy rider section already carries legs"`.

- [ ] **Step 2: Run it and see it fail**

```bash
mise run test 'config_schema'
```

Expected: a build FAIL, `configuration/legacy_rider.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

```cpp
// configuration/legacy_rider.hpp — reading documents written before the
// seated-only fields were removed from RiderSpecs.
//
// The seated rider variant (removed 2026-11-04) was the only reader of
// ten RiderSpecs fields: the legs mode, the saddle/pedal/bar load shares,
// the three body resonances, the damping ratio and the saddle interface
// spring/damper. No remaining variant reads them (pinned before removal
// by test_mjcf seated_parameters_are_inert). Recordings and initial-state
// contracts written earlier still carry them in the resolved rider
// section. Readers accept a section whose removed members hold the
// shipped defaults and reject any other value: only documents that a
// shipped configuration could write are promised to replay.
#pragma once

#include "fields.hpp"
#include "../io/value.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace bike::configuration::legacy {

inline constexpr std::string_view kRemovedRiderLegsKey = "legs";
inline constexpr std::string_view kRemovedRiderLegsDefault = "rigid";
inline constexpr std::array<std::pair<std::string_view, double>, 9> kRemovedRiderReals{{
    {"saddle_share", .55},
    {"pedal_share", .33},
    {"bar_share", .12},
    {"torso_resonance_hz", 7.},
    {"leg_resonance_hz", 5.},
    {"arm_resonance_hz", 4.},
    {"damping_ratio", .40},
    {"saddle_interface_k_n_m", 101'000.},
    {"saddle_interface_c_ns_m", 2'762.}}};

// The first removed member of a recorded rider section that does not hold
// its shipped default (or has the wrong JSON kind); nullopt when every
// removed member present is inert. Absent members are inert: documents
// written after the removal carry none.
[[nodiscard]] inline std::optional<std::string_view>
non_default_rider_field(const io::Value& rider) {
    const auto& object = io::as_object(rider, "rider");
    if (const auto* legs = io::find_member(object, kRemovedRiderLegsKey)) {
        const auto* text = std::get_if<std::string>(&legs->data);
        if (text == nullptr || *text != kRemovedRiderLegsDefault)
            return kRemovedRiderLegsKey;
    }
    for (const auto& [name, value] : kRemovedRiderReals) {
        const auto* member = io::find_member(object, name);
        if (member == nullptr) continue;
        const auto* number = std::get_if<io::Number>(&member->data);
        if (number == nullptr) return name;
        bool ok = false;
        const double parsed = io::detail::float_from_lexeme(number->lexeme, ok);
        if (!ok || parsed != value) return name;
    }
    return std::nullopt;
}

// Erases every removed member, in place. Callers check
// non_default_rider_field first; erasing never inspects values.
inline void erase_removed_rider_fields(io::Value& rider) {
    auto* object = std::get_if<io::Value::Object>(&rider.data);
    if (object == nullptr)
        throw std::invalid_argument("legacy rider section is not an object");
    std::erase_if(*object, [](const auto& member) {
        return member.first == kRemovedRiderLegsKey ||
               std::ranges::any_of(kRemovedRiderReals, [&](const auto& removed) {
                   return member.first == removed.first;
               });
    });
}

// The current rider section as pre-removal configs serialized it: the ten
// removed members re-added at their shipped defaults with the record
// writer's repr_double lexemes. Contracts hash key-sorted JSON
// (io::canonical_json), so member position does not matter.
[[nodiscard]] inline io::Value with_removed_rider_defaults(io::Value rider) {
    auto* object = std::get_if<io::Value::Object>(&rider.data);
    if (object == nullptr)
        throw std::invalid_argument("legacy rider section is not an object");
    if (io::find_member(*object, kRemovedRiderLegsKey) != nullptr)
        throw std::invalid_argument("legacy rider section already carries legs");
    object->emplace_back(std::string(kRemovedRiderLegsKey),
                         io::Value{std::string(kRemovedRiderLegsDefault)});
    for (const auto& [name, value] : kRemovedRiderReals)
        object->emplace_back(std::string(name),
                             io::Value{io::Number{detail::repr_double(value)}});
    return rider;
}

} // namespace bike::configuration::legacy
```

- [ ] **Step 4: Run it and see it pass**

```bash
mise run test 'config_schema'
```

Expected: PASS. If the first `require` fails, the literal does not match the live writer. Print `io::canonical_json(current)` from a scratch line in the test, and compare it character by character with `kPreRemovalDefaultRider`. Fix whichever side is wrong (a lexeme like `0.4` vs `0.40`), then remove the scratch line. While the fields still exist, `canonical_json(current)` must equal the literal exactly.

- [ ] **Step 5: Stage check and commit**

```bash
bash tools/check_change.sh "$(git rev-parse HEAD)"
git add native/src/configuration/legacy_rider.hpp native/tests/config/test_schema.cpp
git commit -m "feat(config): read rider sections written before the seated fields left

Mirrors legacy_battery.hpp: removed members at their shipped defaults
are inert, any other value is named, and re-adding the defaults
rebuilds the pre-removal text that old initial-state contracts hashed.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Remove the fields and wire the readers

**Files:**
- Modify: `native/src/configuration/rider_spec.hpp`:
  - the struct fields, lines 66-82;
  - the `legs` and share/resonance/damping/interface `FieldDecl` entries, lines 252-311;
  - the comments at lines 37-41 and 58-63.
- Modify: `native/src/configuration/validate.cpp:868-887` (`post_init(RiderSpecs&)`), and `format_fixed6` (line 92) if it becomes unused.
- Modify: `native/src/model/rider_geometry.cpp` (`geometry_pose`, about line 1203).
- Modify: `native/src/replay/rebuild.cpp` (`rider_specs` 61-72, `configuration_hashable` 138-143, `request_from_input` ~181).
- Modify: `native/src/initialization/initial_state.cpp`:
  - `contract`, line 50 and the rider member at line 76;
  - `validated()`, lines 108-112;
  - the include at line 5.
- Modify tests:
  - `native/tests/model/test_mjcf.cpp` (`Case.legs` 202-212, `seated_parameters_are_inert`);
  - `native/tests/config/test_schema.cpp` (`test_rider` 1352-1396);
  - `native/tests/model/test_inertia.cpp` (`expect_rider_record` 720-755, the `spec_errors` branch ~844);
  - `native/tests/random/test_seeded.cpp:633-653`;
  - `native/tests/reference/test_replay_recording_v3.cpp` (new `removed_rider_fields_gate`).
- Modify fixtures: `tests/fixtures/model/rider_specs.json`, `tests/fixtures/random/rider-samples.json`.

**Interfaces:** Consumes the Task 3 names exactly. Produces `RiderSpecs` with five fields.

- [ ] **Step 1: Write the failing replay gate**

In `test_replay_recording_v3.cpp`, add after `removed_battery_gate`:

```cpp
// The seated-only rider fields left RiderSpecs; legacy recordings still
// carry them in resolved_config.rider. At the shipped defaults they are
// inert and the recording replays; any other value is refused by name.
void removed_rider_fields_gate() {
    auto input = rr::replay::load_legacy_input(bt::fixture("replay/legacy_v2"));
    const auto& recorded_rider = field(field(input.summary, "resolved_config"), "rider");
    bt::require(rr::io::find_member(rr::io::as_object(recorded_rider, "rider"), "saddle_share") != nullptr,
                "the legacy fixture still records the seated-only rider fields");
    {
        const rr::replay::FixedAssets assets(input);
        bt::require(static_cast<bool>(rr::replay::request_from_input(input, assets)),
                    "default seated-only rider fields are inert and replay");
    }
    for (const auto& [key, value] :
         {std::pair<std::string_view, rr::io::Value>{"legs", rr::io::Value{std::string("articulated")}},
          std::pair<std::string_view, rr::io::Value>{"damping_ratio", rr::io::Value{rr::io::Number{"0.9"}}}}) {
        auto changed = input;
        auto resolved = field(changed.summary, "resolved_config");
        auto rider = field(resolved, "rider");
        rr::set(rider, std::string(key), value);
        rr::set(resolved, "rider", std::move(rider));
        rr::set(changed.summary, "resolved_config", std::move(resolved));
        const rr::replay::FixedAssets assets(changed);
        rr::incompatible(
            [&] { (void)rr::replay::request_from_input(changed, assets); },
            "rider." + std::string(key));
    }
}
```

Call it from `main` after `removed_battery_gate();`. If `ReplayInput` is not copyable, reload with `load_legacy_input` inside the loop instead of `auto changed = input;`. If `rr::set` or `rr::incompatible` take other parameter types, match their use in `removed_battery_gate`.

- [ ] **Step 2: Run it and see it fail**

```bash
mise run test 'reference.replay_recording_v3'
```

Expected: FAIL in `removed_rider_fields_gate`. Today `legs = "articulated"` and `damping_ratio = 0.9` are accepted, so no `IncompatibleExecution` is thrown.

- [ ] **Step 3: Remove the fields from the record**

`rider_spec.hpp` struct becomes:

```cpp
struct RiderSpecs {
    std::string variant = "lumped";
    double mass_kg = 80.0;
    double height_m = 1.80;
    std::optional<double> inseam_m;
    double helmet_mass_kg = 0.40;
    // … the existing methods, unchanged …
};
```

Delete the ten matching `FieldDecl` entries from `kRiderSpecsFields`. The remaining five keep their order: `variant`, `mass_kg`, `height_m`, `inseam_m`, `helmet_mass_kg`.

Replace the comment at lines 37-41 with:

```cpp
// rider.py RIDER_VARIANTS / DEFAULT_RIDER_VARIANT — the 'seated' variant
// was removed with its slide-joint spring-damper model (2026-11-04), and
// the ten fields only it read left the record afterwards; documents
// written before carry them and are read through
// configuration/legacy_rider.hpp.
```

In the struct comment at lines 58-63, replace "the input-side record keeps every declared field and every pure derived helper" with "the input-side record keeps the fields the remaining variants read and every pure derived helper".

`validate.cpp` `post_init(RiderSpecs&)`: delete the `legs` check and the shares-sum check. Then run `grep -n "format_fixed6" native/src/configuration/validate.cpp`. If only its definition remains, delete the definition too.

`rider_geometry.cpp` `geometry_pose`:

```cpp
ArticulatedPose geometry_pose(const RiderSpecs &rider,
                              const BikeSpecs &specs) {
    // A fresh validated record, as the Python original's
    // dataclasses.replace(...) produced; the seated pose solver takes it
    // purely as geometry input.
    RiderSpecs checked = rider;
    configuration::detail::post_init(checked);
    const SeatedPose pose = seated_pose(checked, specs);
```

Keep the rest of the function unchanged.

- [ ] **Step 4: Wire replay**

In `rebuild.cpp`, add `#include "../configuration/legacy_rider.hpp"` next to the `legacy_battery.hpp` include. Then:

```cpp
configuration::RiderSpecs rider_specs(const Value& value) {
    configuration::RiderSpecs out;
    if (const auto* variant = find(value, "variant")) out.variant = string(*variant);
    if (const auto* inseam = find(value, "inseam_m"); inseam && !io::is_null(*inseam)) out.inseam_m = io::as_double(*inseam, "inseam_m");
#define BIKE_RIDER(name) {#name, &configuration::RiderSpecs::name}
    reals(out, value, {BIKE_RIDER(mass_kg), BIKE_RIDER(height_m), BIKE_RIDER(helmet_mass_kg)});
#undef BIKE_RIDER
    configuration::detail::post_init(out); return out;
}
// legacy recordings: the seated-only rider members removed from RiderSpecs
// are tolerated only at their shipped defaults
// (configuration/legacy_rider.hpp). Any other value was never written by
// a shipped config, so the run is not one replay promises to reproduce.
Value inert_rider(Value rider) {
    if (const auto removed = configuration::legacy::non_default_rider_field(rider))
        throw IncompatibleExecution(
            "resolved_config.rider." + std::string(*removed) + ": a non-default value cannot be "
            "replayed — the seated rider model that read it no longer exists");
    configuration::legacy::erase_removed_rider_fields(rider);
    return rider;
}
```

In `configuration_hashable`, before `return hashable;`, add:

```cpp
    set(hashable, "rider", inert_rider(field(hashable, "rider")));
```

In `request_from_input`:

```cpp
    request.setup.model.rider = rider_specs(inert_rider(field(resolved, "rider")));
```

Update the `configuration_hashable` comment ("with the removed inert knobs stripped") to also name the rider members.

- [ ] **Step 5: Wire the initial-state contract**

In `initial_state.cpp`, add `#include "../configuration/legacy_rider.hpp"`.

Change the `contract` signature to:

```cpp
std::string contract(const SetupContext& context,std::optional<int> station_count=std::nullopt,bool legacy_battery=false,bool legacy_rider=false){
```

In its final `hash(...)` call, replace `{"rider",plain(field(identity,"rider"))}` with:

```cpp
{"rider",legacy_rider?configuration::legacy::with_removed_rider_defaults(plain(field(identity,"rider"))):plain(field(identity,"rider"))}
```

In `validated()`:

```cpp
    // legacy artifacts: the contract over the pre-removal serialization —
    // with the disabled drive.battery section and/or the seated-only rider
    // members at their defaults — is also accepted; any other divergence,
    // including an enabled recorded battery or a non-default removed rider
    // member, fails closed.
    const auto matches=[&]{for(const bool battery:{false,true})for(const bool rider:{false,true})if(stored_sha==contract(context,std::nullopt,battery,rider))return true;return false;};
```

New writes (`contract(context)`, `contract(context,count)`) keep the current form.

- [ ] **Step 6: Update the remaining tests**

- **`test_mjcf.cpp`.**
  - Delete `seated_parameters_are_inert` and its call in `main`. The fields no longer exist, so the type system now enforces what it pinned. Its evidence lives in the Task 1 commit.
  - In the `Case` struct of the no-joint-damping loop (lines 202-212), delete the `legs` member, `request.rider->legs=c.legs;`, and the `+"/"+c.legs` label parts.
- **`test_schema.cpp` `test_rider`.**
  - The defaults check becomes `specs.variant == "lumped" && specs.mass_kg == 80. && specs.height_m == 1.80 && !specs.inseam_m.has_value() && specs.helmet_mass_kg == 0.40`.
  - Delete the `bad.legs` and `bad.saddle_share` rejection blocks.
  - Add after the defaults check:

```cpp
    // The serialized record carries exactly the five remaining fields.
    const io::Value doc = cfg::detail::plain_document(
        cfg::detail::as_object(cfg::detail::kRiderSpecsSchema, &specs));
    std::set<std::string> keys;
    for (const auto& member : io::as_object(doc, "rider")) keys.insert(member.first);
    bt::require(keys == std::set<std::string>{"variant", "mass_kg", "height_m", "inseam_m", "helmet_mass_kg"},
                "RiderSpecs serializes only the remaining fields");
```

- **`test_inertia.cpp`.**
  - In `expect_rider_record`, delete the `legs` check and the nine removed scalar checks.
  - In the `spec_errors` loop, delete the `bad_legs` and `shares_not_one` branches.
- **`test_seeded.cpp`.** Delete the nine removed `rider_scalar(...)` calls and the `legs` `bt::require`. Keep `mass_kg`, `height_m`, `helmet_mass_kg` and `inseam_m`.

- [ ] **Step 7: Trim the two record fixtures**

```bash
python3 -I - <<'EOF'
import json, pathlib
root = pathlib.Path('tests/fixtures')
REMOVED = {'legs', 'saddle_share', 'pedal_share', 'bar_share', 'torso_resonance_hz',
           'leg_resonance_hz', 'arm_resonance_hz', 'damping_ratio',
           'saddle_interface_k_n_m', 'saddle_interface_c_ns_m'}

def load(name):
    path = root / f'{name}.json'
    text = path.read_text()
    doc = json.loads(text)
    assert json.dumps(doc, indent=2, sort_keys=True) + '\n' == text, f'{name}: formatting would change'
    return path, doc

def strip(record):
    missing = REMOVED - record.keys()
    assert not missing, missing
    for key in REMOVED: del record[key]

path, doc = load('model/rider_specs')
strip(doc['default_specs'])
strip(doc['rider_from_args_75_175'])
for case in doc['resolve_cases']:
    if case.get('accepted') and isinstance(case.get('result'), dict):
        strip(case['result'])
before = len(doc['spec_errors'])
doc['spec_errors'] = [c for c in doc['spec_errors'] if c['tag'] not in ('bad_legs', 'shares_not_one')]
assert len(doc['spec_errors']) == before - 2
path.write_text(json.dumps(doc, indent=2, sort_keys=True) + '\n')

path, doc = load('random/rider-samples')
for key, sample in doc.items():
    if not key.startswith('_'): strip(sample['rider'])
path.write_text(json.dumps(doc, indent=2, sort_keys=True) + '\n')
EOF
git diff -U0 -- tests/fixtures/model/rider_specs.json tests/fixtures/random/rider-samples.json | grep -c '^+[^+]'
```

Expected:
- The script finishes without an assertion error.
- The final count is `0` (deletions only).
- If any accepted `resolve_cases` result is not a dict (for example the `none` variant), the `isinstance` guard skips it. Confirm with `git diff` that every remaining record lost exactly the ten keys.

- [ ] **Step 8: Prove no reader is left**

```bash
grep -rn "\.legs\b\|saddle_share\|pedal_share\|bar_share\|resonance_hz\|damping_ratio\|saddle_interface" native/src native/tests \
  | grep -v "configuration/legacy_rider.hpp\|test_replay_recording_v3.cpp\|config/test_schema.cpp"
```

Expected: no output.

- [ ] **Step 9: Run the targeted tests, including the slow readers of stored initial states**

```bash
mise run test 'config_schema|model_inertia|model_mjcf|random_seeded|replay_|reference.replay_|reference.native_replay|reference.pedelec_replay_contract|research_|reference.native_research|reference.native_runtime|reference.native_viewer_snapshot|initialization_|reference.period_buffer|reference.native_rider_allocation|reference.control_clock|reference.native_pneumatic_tire|reference.native_distributed_tire'
```

Expected:
- PASS, including `removed_rider_fields_gate`.
- `reference.period_buffer`, `reference.native_rider_allocation`, `reference.control_clock`, `reference.native_pneumatic_tire` and `reference.native_distributed_tire` load stored initial states unchanged. They pass only through the new legacy rider contract form.
- `reference.native_research_recording` replays an export of the ft2 `research_flat` capture, whose recorded rider still carries the ten fields. It passes only through the `configuration_hashable` strip.

Failure guide:
- "initial-state physical model contract differs": the re-added lexemes do not match. Re-check Task 3 Step 4.
- "reconstructed configuration differs from the recorded recipe": the strip is missing in `configuration_hashable`.

- [ ] **Step 10: Stage check and commit**

```bash
bash tools/check_change.sh "$(git rev-parse HEAD)"
git add -A native/src native/tests tests/fixtures/model/rider_specs.json tests/fixtures/random/rider-samples.json
git commit -m "refactor(rider): drop the ten seated-only fields from RiderSpecs

No remaining variant read them (pinned by the previous inertness test).
Recordings and initial states written earlier keep loading: replay and
the initial-state contract accept the removed members at their shipped
defaults and refuse any other value by name.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

Expected: `check_change.sh` exits 0.

---

### Task 5: Docs and the completion proof

**Files:**
- Modify: `docs/TESTING.md:432-435` (the legacy battery paragraph)

- [ ] **Step 1: Extend the legacy-document paragraph**

Replace the paragraph that starts "Legacy documents written before the battery energy model was removed" with:

```markdown
Legacy documents written before the battery energy model was removed still
carry `physics.drive.battery`. Replay and initial-state transfer accept that
section only when it is disabled (`configuration/legacy_battery.hpp`); a
recorded enabled battery is rejected as an incompatible execution.

Documents written before the seated-only rider fields left `RiderSpecs`
(`legs`, the three load shares, the three resonances, `damping_ratio`, the
saddle interface spring/damper) still carry them in the rider section.
Replay and initial-state transfer accept them only at their shipped
defaults (`configuration/legacy_rider.hpp`); any other value is rejected by
name.
```

- [ ] **Step 2: Completion proof (level 2, once)**

```bash
bash tools/run_tests.sh native
```

Expected: exits 0, about 11 min. It appends a `native … passed` row for HEAD to `timings.tsv`. Do not run `--from`; a partial run is not the proof. Do not run `full`. The slow readers of the touched contract ran in Task 4 Step 9 on the same code.

- [ ] **Step 3: Commit the docs**

```bash
git add docs/TESTING.md
git commit -m "docs(testing): name the legacy rider-field rule

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

A docs-only commit after the gate does not invalidate it: AGENTS.md level 2 covers C++/CMake/tooling diffs, and this commit touches none of them.

- [ ] **Step 4: Report**

Report:
- the commits;
- the `timings.tsv` row;
- the Task 1 Step 3 and Task 4 Step 8 grep outputs;
- the D2 behaviour change;
- the D5 scope limits.

The merge into `main` needs the user's go-ahead.

---

## Self-Review

- **Coverage.**
  - D1: Tasks 1 and 4.
  - D2: Task 2.
  - D3: Task 3 for the helpers; Task 4 Steps 1, 4 and 5 for the readers.
  - D4: Task 4 Steps 5 and 9.
  - D5: stated with its evidence; no task by design.
  - Docs: Task 5.
- **Names.** `non_default_rider_field`, `erase_removed_rider_fields`, `with_removed_rider_defaults`, `kRemovedRiderReals`, `kRemovedRiderLegsKey` and `kRemovedRiderLegsDefault` are spelled the same in Tasks 3 and 4. `inert_rider` is local to `rebuild.cpp`. `test_rider_centers` replaces `test_rider_legs_centers` in Task 2 only.
- **Known uncertainty.**
  - Whether `ReplayInput` is copyable (Task 4 Step 1 gives the fallback).
  - Where `post_init` is declared for `test_mjcf.cpp` (Task 1 Step 1 gives the lookup).
  - Whether the `-Wframe-larger-than` suppression is still needed (Task 2 Step 4 gives the rule).
