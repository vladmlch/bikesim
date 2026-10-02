"""
Pneumatic Tyre Model (docs/RIDE.md sections 3.1 and 4.1).

Per-step kernels for the `pneumatic` tyre, in metres:

- `geometry` -- the ring of radial elements against the road profile: where each ray meets
  the road, element deflections, contact patches, coverage.
- `carcass` -- element forces: pressure and carcass terms, rate stiffening, hysteresis, rim.
- `brush` -- the tread: transient slip, lumped and discretised brush, tangential force.
- `model` -- one wheel's tyre: the kernels in sequence, with the state they carry.
- `applier` -- the only module that writes to MuJoCo: both tyres into `xfrc_applied`.

Everything but `applier` is pure NumPy and testable without compiling a model.
"""
