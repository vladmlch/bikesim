# Performance log

Standard measurement: `tools/measure_realtime.py --no-strict --duration 10 --runs 3 --warmup 1`,
track `rough_uphill_savage`, physics `viewer_physics_welded.toml` (dt 1.25 ms), this Mac.
`per-step` is from `tools/step_profile.py` (1600 loop steps, construction excluded).

Note: the dated rows below were measured on the python-only bikesim_new tree
that this branch ports from; numbers on this branch may differ.

| Date | Commit | Change | median RTF | per-step ms |
|---|---|---|---|---|
| 2026-10-11 | dfe4da1 | python-only baseline | 0.513 | 2.101 |
| 2026-10-11 | 8591bb1 | validate specs at construction, not per call | 0.586 | 1.949 |
| 2026-10-11 | dacc33b | clone drivetrain policies instead of deepcopy per step | 0.572 | 1.931 |
| 2026-10-11 | 70a7a42 | freeze force arrays once per step; cache contact vectors; drop scalar numpy overhead | 0.674 | 1.786 |
| 2026-10-11 | b614c5d | energy_state + mass_observations only on full samples (decimation-gated telemetry) | 0.729 | 1.725 |
| 2026-10-11 | 34129dc | skip apply_forces entry mj_forward in running loop (3.25->2.25 calls/step) | 0.710 | 1.70 |
| 2026-10-11 | dacf00d | numba njit(cache) on directional_capacity/_brush_step/_normal_contact/_box_pad_contact | 0.774 | 1.54 |
| 2026-10-12 | e3b1d9e | impl/cpp-port-p2 post-port measurement (this branch); note: this repo's measure_realtime.py has no --no-strict flag and every run aborts at step 4 (reason=invalid_controller, pre-existing `energy.constraint_work` reference violation at t=0.005s — same on base d52ed38), so median RTF is a 4-step prefix figure; per-step is the honest 1600-step number | 0.576 | 1.609 |
