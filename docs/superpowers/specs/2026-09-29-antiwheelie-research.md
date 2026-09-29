# Anti-wheelie research plant

## Goal
Make the existing physical planar bicycle a reproducible plant for developing
motor-torque control on uneven grades, with a dynamic rider and causal sensors.
The existing physical force, work and contact ownership is retained. The legacy
model and existing default command lines remain compatible.

## Contracts
- No dependency downloads. Python 3.13 and the supplied MuJoCo 3.12 wheel are used.
- No external pitch stabilizer, speed servo, pose repair or load-transfer force
  in the research plant. Mid-drive torque passes through the physical drivetrain.
- A per-step immutable control can request motor torque, cap assistance, vary
  human effort, and request bounded internal rider posture changes.
- Brake, motor envelope and battery energy limits always outrank controller input.
- Grade is independent of obstacle geometry; road material can vary by station.
  The compliant tire must resolve the material at its actual contact, not the root.
- Measurements exposed to policies are distinct from privileged simulation truth.
  Sensor noise, delay and control sample-and-hold have deterministic, explicit units.
- Front unloading, geometric front lift, sustained wheelie candidates, rear lift
  and both-wheel flight are separate. Incline alone is not a wheelie.
- Metrics accumulate at physics rate, not CSV/control rate. Invalid geometry or
  numerical failures are failures, never successful episodes or fabricated samples.
- Ship regression tests, repeatable experiments, usage examples, a patch against
  the supplied source and a working offline bundle containing the updated wheel.

## Non-claims
The plant is longitudinal X-Z, not lateral balance or deformable soil. Default
material, motor and anatomical parameters remain synthetic/unvalidated. A
wheelie candidate is a kinematic/contact classification, not proof that motor
torque caused a particular front lift. No real-bike safety certification is made.
