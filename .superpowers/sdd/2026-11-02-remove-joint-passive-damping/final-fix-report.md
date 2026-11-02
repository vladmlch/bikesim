# Final Branch-Review Fix Report

Status: **DONE** — all three findings implemented and verified. The one
ctest red (`reference.native_research_recording`) is proven pre-existing
at HEAD `27901fbb` (identical failure with the changes stashed), not a
regression from this fix.

Commits: `9efb6163` (fixtures+manifest), `078aa6df` (native src + test).

## 1. Recorded-artifact restore — VERIFIED byte-identical

`tests/fixtures/migration/replay/{legacy_v1,legacy_v2,historical_v2}/summary.json`
restored from `a1268978` (`git show a1268978:<path> > <path>`).

- `git diff a1268978 -- tests/fixtures/migration/replay/` → **empty**
  (every file in the dir, including the untouched
  `native_v3/summary.md` and `tampered/summary.md`, is identical to
  `a1268978`).
- `migration/oracle-manifest.json` `files[]` re-pinned with the actual
  bytes: `legacy_v1`/`legacy_v2` = 32323 B /
  `ef0376a3c9ecc2e54aae9f9d3f205d44629e128527a1f558822d2e9c923d4ed0`,
  `historical_v2` = 41440 B /
  `7c9ed153371a00b9b2f9a0e353be3dc1f1ad4f239de61d19f90fff9cff237caa`.
  These are exactly the pre-sweep pins (`git show
  6c639e5d^:migration/oracle-manifest.json` agrees). `migration_T05`
  (manifest sha verification) passes.

## 2. dtype normalize() scoped — `database_validate.cpp`

`validate_declared_catalog` no longer case-folds every `field_catalog`
dtype (~490 rows). Both sides now compare byte-exact except the single
`channel_rider.rider_control` row — the row the removed struct children
(`passive_damping_nm`, `solved_passive_nm`) lived in:

- declared side: `row[2]` normalized only when
  `row[0]=="channel_rider" && row[1]=="rider_control"`;
- recorded side: `normalize()` + `erase_normalized_member` only under
  `legacy_struct`; every other row pushes the raw recorded dtype.
- `kLegacyFields`/`kLegacyStructChildren` semantics unchanged (legacy
  field rows still dropped; `strip_legacy_children` still applies to
  `definition_json` of the same row only).

## 3. Null asymmetry — TIGHTENED, evidence below

**Claim verified.** Pre-removal (`git show 838bc427^`):
`ArticulatedConfig::joint_passive_damping_nms_rad` was
`std::optional<double>` (`physical_config.hpp:392`, FieldDecl
`OptionalReal` at :882 — serializes `null` when unset), and
`passive_damping_nms_rad() = joint_passive_damping_nms_rad
.value_or(joint_kd_nms_rad)` (:478–481) — an unset knob meant
~15 N·m·s/rad (recorded `joint_kd_nms_rad`, default 15). Its only
physics consumer was the MJCF `damping` write on articulated-rider
hinges (`articulated_rider.cpp:187`), which exist iff
`rider.variant == "articulated_planar"` (`model/builder.cpp:28`;
`seated` gets rigid legs, `lumped`/`none` have no hinges). The write
was not drive_mode-conditioned, so the doc-driven check is
`resolved_config.rider.variant`, not `drive_mode`.

**In-repo evidence.** `rg -a '"joint_passive_damping_nms_rad"\s*:\s*null'`
over the whole tree (incl. binary `run.duckdb` via `strings`) finds
**zero** recorded docs with `null` — every recorded doc carries `0.0`
(the three restored `summary.json`, `native_v3`/`tampered` `summary.md`
lines 407/1403, both `run.duckdb` payloads). The only in-repo input
class that serializes `null` ("sets neither knob", plan R1) is
`tests/fixtures/migration/config/legacy.toml`
(`physics_mode="legacy"` stub) consumed by
`requests/legacy_flat.toml` (`rider_variant="lumped"`,
`drive_mode="coast"`) — rider-less.

**Decision: tighten.** The pre-removal schema imposed no coupling
between `rider_variant` and the knob (`validate.cpp` checked it only
when set) — `articulated_planar` + unset knob was legal and would record
`null` alongside live hinges running at 15 N·m·s/rad. `null` therefore
*can* occur with an active articulated rider; silently popping it would
bless a physically different reconstruction. `drop_removed_passive_
damping` now takes `recorded_rider_hinges(resolved)` — `null` +
`variant == "articulated_planar"` throws `IncompatibleExecution` like
nonzero; `null` on any other variant (no rider hinges) stays accepted;
`0.0` stays accepted unconditionally. Both call sites
(`request_from_input`, `configuration_hashable`) read the variant from
the doc under examination.

**Regression added** — `removed_passive_damping_gate` in
`test_replay_recording_v3.cpp`: loads `replay/legacy_v2` via
`load_legacy_input`, proves the recorded `0.0` + `articulated_planar`
rebuild succeeds (also end-to-end exercise of the restored fixture
through the drop), then mutates the recorded key to `null` in-memory and
asserts `IncompatibleExecution` naming the key. Fails under the old
unconditional-null acceptance.

## Build + test evidence

- `cmake --build --preset release` — green (rebuild.cpp,
  database_validate.cpp, test target all compiled/linked; `-Werror`
  clean).
- `ctest --test-dir build/native-release -R
  'replay|migration_T0[68]|migration_T25|migration_T38|record'
  --output-on-failure` → **27/28 pass**. Sole failure:
  `reference.native_research_recording`
  (`FAIL export.intervals.jsonl.values: 0 removed-key placements fit —
  expected exactly 1`) — reproduced identically at HEAD `27901fbb` with
  the working tree stashed; pre-existing, documented provenance.
- `migration_T05` (manifest digests), `T06`, `T08`, `T25`, `T38` — pass.
- `reference.replay_recording_v3` (incl. the new gate) — pass.

## Remaining scope

`state_codec.cpp:1959–1963` (`decode_spindle_config`) has the same null
asymmetry: it accepts `joint_passive_damping_nms_rad` null/0.0 and the
decoded payload (`config.rider_controller`) only exists when an
articulated rider ran — so a recorded null there is likewise non-inert.
Outside the three-finding scope; flag for a follow-up review decision.
