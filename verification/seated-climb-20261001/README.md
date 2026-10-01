# Seated climbing execution ledger

Execution authorized with `$executing-plans начинай` on 2026-10-01.

Base: `2e9358580725bf12738ae2bfb637cd827bc31515`, branch `antiwheelie-refocus`.
Existing untracked plans, verification outputs, `.claude/skills/` and `split_expenses.py` are preserved. Execution is sequential in this checkout. No commits, branch changes or default-profile changes.

## Work status

| Task | Status | Evidence |
|---|---|---|
| C1 Fresh support-loss diagnosis | evidence verified; cause analysis continues in C2 | three current-source runs;18 report/replay tests pass |
| C2 Demonstrated contact/control fix | local fix passes; integration checks running |158 focused tests,0.625 ms automatic5 s succeeds |
| C3 Bounded seated rider | implementation in progress; qualification open | pure/API checks pass; physical candidate exposed a trapped return foot |
| C4 Climb limit diagnosis | pending | |
| D1 Timing and provenance | pending | |
| D2 Contact/drive/resolution | pending | |
| D3 Realtime optimization | pending | |
| D4 30T/extreme/end-to-end | pending | |

## C1 baseline

- `UV_CACHE_DIR=/private/tmp/uv-cache-antiwheelie-plan PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --no-sync pytest -q tests/test_physical_pedaling_regression.py tests/test_pedal_contact_stability.py tests/test_finite_pedal_contact.py tests/test_physical_coasting.py`: **81 passed in 1.25 s**.
- Source has not been modified while collecting the three automatic baseline runs. Their exceptions/invalidity are diagnostic outcomes, not passing plant evidence. CLI exit 0 alone is insufficient.
- Inputs: original `viewer_physics_fast.toml`, articulated rider, authored `rough_uphill_extreme.toml`, default `RideControl()`, initial rest, 8 simulated seconds; dt 1.25/0.625/0.3125 ms. Independent settled starts diagnose timestep sensitivity; strict common-state convergence comes in D2.

| dt | Full horizon | Maximum coast gap | Maximum both-unloaded coast duration | Maximum energy residual |
|---|---|---|---|---|
| 1.25 ms | 8 s |0.104313 m |0.207500 s |0.004458 |
| 0.625 ms | 8 s |0.097623 m |0.203750 s |0.003585 |
| 0.3125 ms | 8 s |0.093201 m |0.202187 s |0.003006 |

All baseline reports share source fingerprint `4b6950f17a6aff68ec6c885508edf8956013c7aa0df0ee18061988e129c22ce3`. All model and interval-energy gates pass over these eight seconds. Large rear-foot loss persists as dt decreases; it is not explained solely by the known coarse pedal instability. First sustained simultaneous loss begins around1.6 s. These runs precede report-infrastructure changes.

Report infrastructure red test confirms old `run_replay` discarded accepted rows on a later force exception (`KeyError: rows`). Streaming row preservation and interval-based support/numerical evidence now pass18 focused tests. Completeness, unknown energy and latched model invalidity are explicit; a successful CLI or stale numeric flag cannot certify a partial run.

### C2 demonstrated control mismatch

During initial coast, commanded crank rate has reached0 while the actual crank is still moving, subsequently reversing. The existing leg derivative targets use that desired rate instead of the moving platform rate. At the peak rear gap, a17 cm horizontal target error exists despite reachable IK. Investigate separating actual-platform tracking from bounded intentional crank braking, implemented through the same internal leg torques and unilateral friction cones. This is a hypothesis awaiting a discriminating regression and physical comparison, not yet an accepted cause/fix.

The existing integrated automatic-coast regression fails on the current finer baseline:0.2021875 s simultaneous loss versus0.20 s maximum. A new kinematic regression independently fails with desired target derivative0 versus actual moving-support8 rad/s. Two signed braking tests confirm the absence of a separate compressive stopping request before the fix.

Local correction: IK derivatives now follow actual platform motion. Desired crank deceleration generates a finite signed contact-effort request through the existing leg actuators and unilateral friction projection, with synthetic damping2 N·m·s/rad and60 N·m request cap. It applies no root force or direct human crank torque and changes no contact materials. At a stopped actual crank the request is zero.

`coast-tracking-brake-dt625.json.gz`: full5 s automatic run, no simultaneous unloading, maximum coast gap0.000275 m, useful resumed work120.205 J over1.5056 crank turns; contact-model and numerical gates pass.158 focused compiled/FK/contact/internal-momentum tests pass. Full finer automatic and10 s flat/incline resume regressions are the remaining C2 gates.

The finer automatic and10 s flat resume checks passed. The human-only5% incline34T does not complete the full-turn requirement: net−0.896 rad and6.916 J in4–10 s despite close, loaded feet and valid energy. A single-factor30T case improves to+1.337 rad/43.799 J, still below the required full turn. This gate is **open**, not weakened or replaced by a successful label. The source of insufficient torque near the stopped phase still needs a causal mechanical explanation.

Independent review found and regressions reproduced three report defects (unknown support becoming confirmed loss, loss of later validity failures, and evaluation errors preventing save). These are repaired with unknown-aware loss metrics, all failure reasons, valid-prefix statistics and a protected final save. The braking waveform is now normalized so its instantaneous force request respects the60 N·m limit at every tested phase.192 focused tests passed before seated-runtime additions;81 current compiled/pure checks pass after static-mode separation.

## C3 physical findings and timing review

- New high-level seated intent:100 Hz clock,150 ms reaction delay, bounded torso-only lean,225 W mean crank-power intention,60 N·m effort amplitude ceiling and300 N·m/s request slew. It is opt-in and never changes motor fields.225 W is a shaft intention, not a claim about measured delivery or total joint power.
- Seated candidate includes finite synthetic joint envelope/600 W total active joint power/400 N grip, distributed256 tire and geometric drive,30T. It remains unqualified.
- `seated-candidate-first.json.gz`: initialization fails, residual4.9579 >0.05, zero accepted intervals. Do not use it as run evidence.
- Riding-platform derivatives/brake requests had also entered static relaxation. A failing compiled test established that an8 rad/s transient in steady-state initialization requested−16 N·m of riding brake. Static mode now retains stationary derivatives and excludes riding braking.
- `seated-candidate-static-init.json.gz`: initialization meets0.012086 acceleration residual and the full8 s run is model/energy-valid. However progress reaches onlyx5.679 m. The return foot is caught beneath a pedal: at2 s its reaction on the rider is downward317 N, and near8 s downward349 N. Positive normal-load magnitude was incorrectly sufficient for support availability. This is a rider/contact recovery defect, **not an acceptable physical climbing limit**.
- The same candidate's separate flat-track initialization fails residual7.4519. Candidate mechanical qualification remains open. Runtime timing/replay tests use the existing stable ideal-drive fixture to isolate those contracts; that does not qualify the distributed candidate.
- Review additionally demonstrated that motor transport delayed automatic rider startup and that first rider inertial input came from holding brakes rather than research startup. New timing regressions cover300 ms motor delay vs150 ms rider reaction, physics-rate rider programs and common initial solved sensors. Motor/rider transport is separated and replay records program inputs explicitly; physical/runtime checks are running.

### Verified timing and continuing mechanical work

`rider-timing-verified.xml`: five end-to-end tests pass on the stable ideal-drive fixture, including exact replay beyond the150 ms reaction delay, non-mutating probes,300 ms motor transport independent of human effort, and common initial rider/policy sensors. The earlier program/replay/environment run had21 passing cases; its sole remaining failure was a test comparing floating time to exactly0.3 instead of the authoritative integer step480. The corrected test checks step indices, without changing motor timing.

Physical investigation identifies two independent issues in the bounded candidate: the return shoe gets caught under the finite pedal, and a tangential force intention loses useful crank torque when projected onto flat-pedal friction. Recovery now uses finite release/escape/raise/return goals through existing bounded limb actuators, with contact forces retained; it does not teleport a shoe through the platform. Four waypoint tests and a compiled real underside-contact/state-preservation regression pass.

A vertical pressure prototype restores human-only5% incline resume with the original34T: `human-incline-cone-allocation.json.gz` completes10 s;4–10 s has23.309 rad/415.817 J and19.817 N·m delivered mean torque, with no unavailable support and valid energy/model. This artifact predates the subsequent friction-aware allocation refinement and is diagnostic, not final-source certification.

`seated-pressure-recovery.json.gz`: full8 s atx10.941 m; resumes5.188 crank turns and159.241 J, but a0.319 s simultaneous unloading/0.247 m maximum gap occurs during early recovery. The bounded rider is not qualified by this result. The allocator is refined to share compressive load smoothly at dead centres and keep a20% friction reserve; three independent moment/cone tests and139 compiled/contact/internal-momentum cases pass. Current eight-second follow-up is `seated-cone-force-allocation.json.gz`.

## Accepted requirements

Platform pedals; seated80 kg/1.80 m rider with torso leaning toward/away from bars; prepared recreational effort200–250 W at the crank; existing85 N·m/600 W motor and cassette; front34T/30T comparison. Primary track is extreme100 m. Physical limits are admissible only after excluding broken support, model scope and numerical failure. Required realtime includes the research path, not only preview.

Detailed contracts: `docs/superpowers/specs/2026-10-01-seated-realtime-climbing-design.md` and plans03/04.

## Execution efficiency

User requested reducing model round trips for long test polling. Group known checks into sequential batches; handle readiness polling inside tool orchestration rather than repeated model reasoning. Return one bounded final output with exit code and failure details. Keep separate source-changing experiments sequential and preserve source identity. Do independent read-only work while a batch runs when useful; do not repeatedly ask only whether a process finished. Plain readiness checks do not need scientific reasoning.

The next physics verification remains blocked by a controller-goal issue, not accepted as a physical limit: bounded-allocator-accounted/run-0000.json fails at4.808125 s with `requested pedal support load is not reachable`. The final support/posture wrench adds to normal request before IK, so normal request capping alone is insufficient. Project final actuator sole goals to reachable finite geometry and expose saturation, leaving contact force laws/energy gates strict.

Review checkpoints still open: exclude recovering feet before torque allocation; apply friction reserve to measured-load bounds; preserve saturation/stuck evidence for unreachable recovery waypoints; qualify independent flat initialization. Preview now receives an always-computed vertical support-force signal, so recovery detection uses the same physical input in compact and detailed modes. Eight pure timing/overwrite tests pass for the new benchmark; no realtime profile is qualified.
