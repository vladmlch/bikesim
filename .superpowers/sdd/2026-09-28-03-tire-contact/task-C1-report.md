# C1 implementation report — physical wheel contact snapshot

## Scope and revision

- Started from `pedals` at `1fcd925a6f92aae095e64061de7ac67d33d0bfe5`.
- Committed as `d54b060` (`feat: expose physical contact wrench and wheel kinematics`).
- Implemented only C1 contact state, absolute wheel-point kinematics, native query conversion, compatibility adapters, and focused tests.
- Left the user-owned untracked plans/specifications and `.claude/skills/` unchanged. No worktree was created or switched.

## Implementation

- `contact_state.py` defines frozen `ContactPatch` and `WheelContactSnapshot`. All exposed vectors are owned, finite, read-only copies backed by immutable bytes, so neither caller arrays nor MuJoCo buffer reuse can change a retained snapshot. The patch validates a unit normal in the X-Z plane and nonnegative normal load. The snapshot validates time, interval ID and backend; distinguishes geometric from loaded contact; and exposes separate normal-load, normal-vertical, longitudinal tangent, full world force, vertical resultant, wheel-axis moment, effective radius and slip aggregates.
- `wheel_kinematics.py` validates and computes `v_center + omega_world × offset`. `wheel_point_velocity` obtains the wheel body's absolute world twist through `mj_objectVelocity(..., flg_local=0)` and transports from the body's known world origin to the contact point. It does not substitute relative wheel-hinge `qvel` for contact slip.
- `contacts.py` transforms both force and couple from every relevant `mj_contactForce` row through the full contact frame, including the geom1/geom2 sign. It constructs immutable per-wheel physical snapshots with an interval ID derived from simulation time and timestep. The native contact direction is projected onto the X-Z plane to satisfy the planar patch contract because real sphere-heightfield contacts have tiny lateral normal components; the lateral force component is retained separately in the world resultant. The legacy bridged controller load and unbridged support channels remain available with their previous behavior. Existing pneumatic outputs receive a copied `legacy_pneumatic` snapshot adapter.
- The effective radius is the normal-load-weighted projection of wheel-axis-to-contact distance onto each patch normal; zero-load geometric patches use equal weights. The wheel-axis moment includes contact force lever arms and MuJoCo contact couples. The patch slip signal is absolute contact-point velocity along the defined longitudinal tangent.

## Verification

1. Red: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_kinematics.py -q` failed at collection with the expected missing `contact_state` module.
2. Focused final: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_kinematics.py tests/test_ride_controllers.py -q --tb=short` — **57 passed**. Tests cover contact sign after swapping geom1/geom2, slope force decomposition, immutable copies, world velocity with zero relative hinge speed, a real native query's force against MuJoCo rows, and its slip against an independent MuJoCo Jacobian.
3. Full suite: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q --tb=short` — **882 passed, 4 setup errors** in 245.43 s. All four are `tests/test_render_comparison.py` setup failures from `mujoco.cgl.cgl.CGLError: invalid CoreGraphics connection` while creating a renderer in the sandbox. No assertion failed. A later edit only tightened scalar typing/documentation and strengthened an existing integration test; the focused suite was rerun after that edit.
4. `git diff --check` — clean.

## Remaining considerations

- Renderer output was not validated in this sandbox because CoreGraphics cannot establish a connection. This does not affect contact/controller assertion results.
- The snapshot's native force includes very small lateral solver components from a nominally planar model. Contact normals are projected to X-Z, so reconstructed X-Z force can differ from the raw native wrench by tiny second-order projection error; the integration test checks the resultant within `1e-5 N`.
- C2 still owns time-based grounded filtering. C5 still owns compliant-contact geometry and force application; this commit provides the snapshot and native-reference boundary they can use.

## Review fix round 1 — exact wrench and catch-plane identity

Committed as `37f292d` (`fix: preserve native contact force and road source`), on top of `d54b060`.

The two review findings supersede the earlier projection-error consideration above. Each native patch now owns an immutable byte-backed copy of the exact transformed world-force vector. `ContactPatch.world_force_n` returns that vector for native rows; patches without a native vector retain the planar `Fn * normal + Ft * tangent` formula. The snapshot sums exact patch vectors, including lateral force and the vertical contribution of all contact-frame force components. Planar `normal`, `tangent_force_n`, and `normal_vertical_n` remain distinct decomposition fields.

Each patch now records `source_geom` as `terrain` or `catch_plane`. `WheelContactSnapshot.road_loaded_contact` is true only when a `terrain` patch has positive normal load. A catch-plane-only snapshot may be physically loaded while `road_loaded_contact` remains false. The existing bridged legacy `TerrainContacts.front_load_n` / `rear_load_n` and unbridged support channels still count either terrain geom, as before; C2 can use the new road-only state for physical controller gating.

Verification for this fix:

- Red: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_kinematics.py -q --tb=short` — three expected failures for missing native-force, source, and road-loaded interfaces (`3 failed, 16 passed`).
- Exact-resultant focused tests: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_kinematics.py -q -k 'native_world_resultant or native_query_exposes' --tb=short` — `2 passed, 17 deselected`. The synthetic row uses a material `normal_y=0.6` and asserts the exact `[20, 64, 77] N` world force after the source array is overwritten. The native integration test now compares the snapshot to summed MuJoCo rows at `1e-12 N` absolute tolerance.
- Catch-plane focused tests: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_kinematics.py -q -k 'catch_plane' --tb=short` — `2 passed, 17 deselected`. One test covers catch-only, mixed, and unloaded-road patches. The other relabels real contact rows to `catch_plane` and confirms snapshot classification changes while legacy loads/support remain equal.
- Final requested focused suite: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_kinematics.py tests/test_ride_controllers.py -q --tb=short` — **60 passed in 18.25 s**; `git diff --check` clean. Per the fix-round instruction, the full suite was not rerun.
