"""Pure SI material laws for the ``compliant_2d`` tire backend.

These laws are independent of native MuJoCo solver parameters and the legacy
``tyre`` module. They do not select a backend or apply any simulator force.
Synthetic material parameters are not experimental calibration.
"""

from dataclasses import dataclass
from math import isfinite
from numbers import Real


def _scalar(value: float, name: str) -> float:
    """Accept finite real scalars, not strings, booleans or mutable arrays."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real scalar")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite real scalar") from exc
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _finite_result(*values: float) -> None:
    if not all(isfinite(value) for value in values):
        raise ArithmeticError("tire calculation exceeds finite floating-point range")


@dataclass(frozen=True)
class TireSpec:
    """Explicit parameters for a linear radial material law; all units are SI.

    ``pressure_pa_gauge`` records the input pressure; it does not silently
    rescale stiffness. ``valid_load_range_n`` is an applicability declaration,
    not a force limiter or proof of calibration. Use ``is_load_in_valid_range``
    to flag out-of-range results. There is deliberately no calibrated/table
    mode here: measured curves require their own range-checked interpolation
    and force-integral energy, not relabelling these linear defaults.

    No values default to a particular tire. Native ``solref``/``solimp`` belong
    to the separate native-reference configuration, never this material law.
    """

    radial_k_n_m: float
    radial_c_ns_m: float
    pressure_pa_gauge: float
    provenance: str
    valid_load_range_n: tuple[float, float]

    def __post_init__(self) -> None:
        for name in ("radial_k_n_m", "radial_c_ns_m", "pressure_pa_gauge"):
            object.__setattr__(self, name, _scalar(getattr(self, name), name))
        if self.radial_k_n_m <= 0.0:
            raise ValueError("radial_k_n_m must be positive")
        if self.radial_c_ns_m < 0.0 or self.pressure_pa_gauge < 0.0:
            raise ValueError("radial damping and gauge pressure must be nonnegative")
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError("provenance must be a nonempty description, e.g. synthetic")
        try:
            lower, upper = self.valid_load_range_n
        except (TypeError, ValueError) as exc:
            raise ValueError("valid_load_range_n must contain two finite bounds") from exc
        lower = _scalar(lower, "minimum valid load")
        upper = _scalar(upper, "maximum valid load")
        if not 0.0 <= lower < upper:
            raise ValueError("valid load bounds must satisfy 0 <= minimum < maximum")
        object.__setattr__(self, "valid_load_range_n", (lower, upper))

    def normal_contact(
        self, penetration_m: float, penetration_rate_m_s: float
    ) -> tuple[float, float]:
        """Evaluate this instance's parameters; return force [N], energy [J]."""
        return normal_contact(
            penetration_m, penetration_rate_m_s,
            self.radial_k_n_m, self.radial_c_ns_m,
        )

    def elastic_response(self, penetration_m: float) -> tuple[float, float]:
        penetration = max(0., _scalar(penetration_m, 'tire deflection'))
        force = self.radial_k_n_m * penetration
        energy = .5 * self.radial_k_n_m * penetration**2
        _finite_result(force, energy)
        return force, energy

    def is_load_in_valid_range(self, load_n: float) -> bool:
        """Report applicability without modifying physical force or pressure."""
        load = _scalar(load_n, "load_n")
        lower, upper = self.valid_load_range_n
        return lower <= load <= upper


def normal_contact(
    delta: float, delta_dot: float, k: float, c: float
) -> tuple[float, float]:
    """Return unilateral radial force [N] and elastic spring energy [J].

    Penetration ``delta`` is in m and its rate in m/s (positive on approach).
    Stiffness ``k`` is N/m; damping ``c`` is N*s/m. No force or stored energy
    exists at nonpositive penetration, even for an approaching wheel. During
    unloading force is clipped at zero, but elastic energy remains k*delta^2/2.
    """
    delta = _scalar(delta, "delta")
    delta_dot = _scalar(delta_dot, "delta_dot")
    k = _scalar(k, "k")
    c = _scalar(c, "c")
    if k <= 0.0 or c < 0.0:
        raise ValueError("radial stiffness must be positive and damping nonnegative")
    return _normal_contact(delta, delta_dot, k, c)


def _normal_contact(delta: float, delta_dot: float, k: float, c: float) -> tuple[float, float]:
    """Core of `normal_contact` for callers that already validated inputs."""
    if delta <= 0.0:
        return 0.0, 0.0
    spring_force = k * delta
    damping_force = c * delta_dot
    energy = 0.5 * spring_force * delta
    total_force = spring_force + damping_force
    _finite_result(spring_force, damping_force, total_force, energy)
    return max(0.0, total_force), energy


def brush_step(
    xi: float, u: float, v_roll: float, Fn: float,
    k: float, mu: float, length: float, dt: float,
) -> tuple[float, float, float]:
    """Advance tangential shear and return (shear [m], force [N], loss [J]).

    ``u`` is wheel-surface velocity relative to the road at the contact point,
    not merely root speed minus a relative joint speed. ``v_roll`` is rolling
    speed [m/s], ``Fn`` normal load [N], ``k`` tangential stiffness [N/m], ``mu``
    friction coefficient, ``length`` relaxation length [m], and ``dt`` time [s].

    Force opposes stored shear; when that spring unloads, instantaneous force
    need not oppose slip. The discrete identity is
    ``force*u*dt + (new_energy-old_energy) + loss = 0`` to roundoff.
    Loss includes relaxation, saturation and implicit numerical dissipation;
    it must not be labelled measured rubber hysteresis. At lift-off all old
    spring energy is released as loss. This function owns no persistent state:
    callers keep front/rear state, tangent transport and branch-release events.
    """
    xi = _scalar(xi, "xi")
    u = _scalar(u, "u")
    v_roll = _scalar(v_roll, "v_roll")
    Fn = _scalar(Fn, "Fn")
    k = _scalar(k, "k")
    mu = _scalar(mu, "mu")
    length = _scalar(length, "length")
    dt = _scalar(dt, "dt")
    if Fn < 0.0 or k <= 0.0 or mu < 0.0 or length <= 0.0 or dt <= 0.0:
        raise ValueError("invalid brush load, stiffness, friction, length or timestep")
    return _brush_step(xi, u, v_roll, Fn, k, mu, length, dt)


def _brush_step(
    xi: float, u: float, v_roll: float, Fn: float,
    k: float, mu: float, length: float, dt: float,
) -> tuple[float, float, float]:
    """Core of `brush_step` for callers that already validated inputs."""
    old_energy = 0.5 * k * xi * xi
    _finite_result(old_energy)
    if Fn == 0.0 or mu == 0.0:
        return 0.0, 0.0, old_energy

    relaxation = dt * (abs(v_roll) / length)
    denominator = 1.0 + relaxation
    trial_numerator = xi + dt * u
    friction_limit = mu * Fn
    _finite_result(relaxation, denominator, trial_numerator, friction_limit)
    trial = trial_numerator / denominator
    limit = friction_limit / k
    _finite_result(trial, limit)
    new_xi = max(-limit, min(limit, trial))
    force = -k * new_xi
    new_energy = 0.5 * k * new_xi * new_xi

    # Algebraically the work residual, expressed as nonnegative terms to avoid
    # subtracting nearly equal spring energies. Projection ensures that
    # new_xi*(trial-new_xi) >= 0, even when the normal load suddenly drops.
    change = new_xi - xi
    implicit_loss = 0.5 * k * change * change
    relaxation_loss = 2.0 * relaxation * new_energy
    projection_loss = (k * new_xi) * (denominator * (trial - new_xi))
    loss = implicit_loss + relaxation_loss + projection_loss
    _finite_result(force, new_energy, loss)
    if loss < 0.0:
        raise ArithmeticError("brush update violates discrete passivity")
    return new_xi, force, loss
