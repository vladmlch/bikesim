# Episode golden `pinned_savage_40` re-freeze — freehub closure work (2026-11-04)

Cause: `tendon_limit:` constraint rows (freehub, crank clutch, motor
freewheel) moved from numerical constraint work to the closure channel in
`native/src/runtime/accounting.cpp` `raw_work`.

- `-before.txt`: `bike-recording golden compare` of the old darwin-arm64
  golden against the new code (exit 1, output truncated by the tool).
- `-after.txt`: the same compare against the re-frozen golden
  (`bike-recording golden freeze --request
  tests/fixtures/golden/episode/pinned_savage_40/request.toml --steps 40`):
  exit 0, "golden episode matches".
- Array-by-array diff of `episode.npz` (old vs new): 19 of 21 arrays are
  byte-identical (state, controls, forces, model image). Only
  `channel_names.npy` and `channels.npy` differ. Bitwise column comparison
  of the 40×579 → 40×575 channel table: changed columns are exactly
  `energy.{closure_absolute_j, closure_signed_j, constraint_absolute_j,
  constraint_signed_j, constraint_work_ok, constraint_work_ratio,
  solver_constraint_work_j}` and `model_status.model_valid`; removed
  columns are the four `model_status.first_model_violation.*` /
  `model_violation_counts.energy.constraint_work` entries (no violation is
  recorded any more). Every other channel is bitwise unchanged.
- `model.mjb` is byte-identical. linux-aarch64 golden not re-frozen
  (Linux deferred by the user).
