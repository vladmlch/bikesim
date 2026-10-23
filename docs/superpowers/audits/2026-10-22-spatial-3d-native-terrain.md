# Spatial terrain — W06–W12 source handoff

Source delivery: 2026-10-23. **Status: `source_written`; qualification:
`pending_verification`.** No configuration, compilation, test, simulation,
fixture capture, benchmark or memory measurement was executed in this delivery.

## Authored paths

The native owner retains binary64 elevations, cell materials, immutable surface
properties and canonical asset identity. The generator reuses the existing
longitudinal `build_profile` producer, adds bank/roughness/features, and rejects
non-finite or excessively steep output. Roughness uses the existing
`NumpyRng`/`SeedSequence` stream with the fixed `terrain3d/roughness` spawn key,
ordered Gaussian sums, mirrored boundaries and at most two full elevation
buffers. The second buffer dies before materials and serialization are created.

`ProfileExtrusion3d` owns the original knots and delegates height to the same
`biketyre::np_interp` primitive used by the planar path. Materials, original
knots, source text, corridor and surface coefficients all enter its fingerprint.
The nonmoving `Terrain3dSource` binds allocation-free borrowed query views.

`detail::load_terrain3d_source` handles asset/genspec/extrusion requests on the
cold boundary, resolves a base-track path relative to its genspec, rejects
immutable-asset sampling overrides, and checks the centred corridor. Its owner
is shared by startup, runtime and render. Rendering integrates bicubic patch
areas into a separate raster; constant surfaces retain their datum and normalize
to zero. The physical source is never resampled in place.

The canonical genspec document has exactly `base_track` and `terrain3d` root
keys. A file may use a string `base_track` path. The file boundary resolves it
to the existing longitudinal track document before `read_track3d_spec`.
Feature amplitude is a positive authored magnitude; `rut` applies its negative
sign in the height law.

## V02 / V05 evidence to acquire

| Target | Required acceptance | macOS arm64 | Linux aarch64 |
|---|---|---|---|
| `spatial.terrain_codec` | canonical roundtrip, corrupted input/path diagnostics, transpose detector | pending | pending |
| `spatial.terrain_sampling` | plane/normal 1e-12; seams 1e-9; closed-domain edges | pending | pending |
| `spatial.terrain_raycast` | first/tangent hit 1e-6; near miss; explicit exhaustion | pending | pending |
| `spatial.terrain_generator` | repeatable bytes, authored feature/bank/mirror, input rejection, qualified hashes | pending | pending |
| `spatial.terrain_extrusion` | exact node/interior height; real legacy ray comparator; oblique distance 1e-9 | pending | pending |
| `spatial.terrain_startup` | source/render identity, every hfield row, collision masks, immutable physical samples | pending | pending |
| `spatial.terrain_acceptance` | registered fixtures finite; no exhausted query | pending | pending |
| `spatial.terrain_budget_300m` | 60001×301, 144482408 elevation bytes, RSS ≤512 MiB, owner ≤256 MiB | pending | pending |

All observed hashes, measured bytes, source/executable/provider identities,
commands and exit codes remain null or pending. The generator test deliberately
fails when a required independent per-platform hash is still null. Its own
second generation checks repeatability only; it is not a golden acquisition.
The 300 m test is `slow` and must run in a fresh process on each platform.

## Contract issues requiring V00/C10 or V02 resolution

1. **Legacy scalar ray delegation.** The existing legacy API requires an owning
   uniform `RoadProfile` plus a `RayRing` of at least three fixed downward rays.
   W00 extrusion owns arbitrary knots and accepts arbitrary unit directions in
   a const, allocation-free call. There is no compatible single-ray entry point.
   The W10 implementation retains the literal segment cross-product arithmetic
   and avoids renormalization for `dir.y==0`; its test compares with the real
   legacy kernel. Extracting a shared span-based scalar primitive is a C10 seam
   change, and exact delegated ray qualification is not claimed here.
2. **Reference tracks and slope bounds.** The existing savage track includes
   90 mm square edges. At 5 mm spacing their sampled slope exceeds `tan(60°)`.
   The required generator rejects this input rather than silently smoothing the
   planar road. Both reference genspecs and seed entries are authored inputs;
   successful generation is pending. V02 must decide an explicit base-track or
   contract change with provenance before publishing successful captures.
3. **Query statistics.** The frozen `RayHit3d` has no cell/subdivision counters.
   The acceptance source reports actual direct `RootSearchResult.subdivisions`
   and query/exhaustion counts, and leaves whole-ray cell visits null. A claimed
   complete traversal histogram requires an agreed instrumentation interface.
4. **Owner allocation accounting.** Frozen accessors expose payload spans, not
   vector capacities or allocator overhead. The slow test reports visible owned
   payload bytes and process peak RSS; allocator overhead remains null. V02
   must measure the complete immutable owner with a suitable allocation hook
   before treating the full 256 MiB owner gate as verified.

No issue above opens the runtime capability gate or supplies tyre/balance
qualification. V02 accepts terrain in isolation; V05 owns startup integration.
