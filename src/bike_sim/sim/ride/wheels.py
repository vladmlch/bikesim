"""
Ride-Mode Wheel Spin Handles and Resistive Torque Sign.

The two wheel-spin coordinates, and the one sign convention every resistive torque acting
on them shares: oppose the current rotation, and vanish at rest.

Braking and rolling resistance both need this. A resistive torque that keeps its magnitude
through zero speed stops being resistive and becomes a source -- it drives a standing wheel
backwards, and then forwards again, indefinitely (docs/RIDE.md section 6). Both are
therefore tapered through a small speed band rather than switched on `sign()`.
"""

from dataclasses import dataclass

import mujoco


@dataclass(frozen=True)
class WheelSpin:
    """
    Addresses and radius of one wheel-spin coordinate.

    Attributes:
        name: Name of the wheel-spin hinge joint.
        dofadr: Index of the joint in `qvel` and `qfrc_applied`.
        radius_m: Radius of the wheel's contact sphere, in metres. Read from the compiled
            model rather than from `BikeSpecs`, so it cannot drift from the geometry the
            contact actually uses.
    """

    name: str
    dofadr: int
    radius_m: float

    def omega_radps(self, data: mujoco.MjData) -> float:
        """Returns the wheel's current angular velocity in rad/s."""
        return float(data.qvel[self.dofadr])


def resolve_wheel_spin(model: mujoco.MjModel, joint_name: str, contact_geom: str) -> WheelSpin:
    """
    Resolves a wheel-spin joint and the contact sphere that rides on it.

    Args:
        model: Compiled ride-mode model.
        joint_name: Name of the wheel-spin hinge joint.
        contact_geom: Name of that wheel's contact sphere, whose radius sets the lever arm
            between a contact force and a wheel torque.

    Returns:
        The wheel's spin handle.

    Raises:
        ValueError: If the joint or the geom is missing, or the geom is not a sphere -- in
            which case `geom_size[0]` would not be its radius.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"model has no joint '{joint_name}'; ride-mode wheel torques need it")

    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, contact_geom)
    if gid < 0:
        raise ValueError(f"model has no geom '{contact_geom}'; ride-mode wheel torques need its radius")
    if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_SPHERE:
        raise ValueError(f"geom '{contact_geom}' is not a sphere, so its size is not a wheel radius")

    return WheelSpin(
        name=joint_name,
        dofadr=int(model.jnt_dofadr[jid]),
        radius_m=float(model.geom_size[gid][0]),
    )


def opposing_torque(magnitude_nm: float, omega_radps: float, taper_radps: float) -> float:
    """
    Signs a resistive magnitude against the current rotation and fades it out at rest.

    Args:
        magnitude_nm: Non-negative resistive torque magnitude.
        omega_radps: Current wheel angular velocity, in rad/s.
        taper_radps: Width of the speed band over which the magnitude is ramped in. Below
            it the torque is scaled down linearly, reaching exactly zero at zero speed.

    Returns:
        Generalized torque for the wheel-spin coordinate: negative for a forward-spinning
        wheel, positive for a backward-spinning one, zero at rest.

    Raises:
        ValueError: If the taper band is not positive, which would divide by zero and
            restore the discontinuity the taper exists to remove.
    """
    if taper_radps <= 0.0:
        raise ValueError(f"taper_radps must be positive, got {taper_radps}")
    if omega_radps == 0.0:
        return 0.0
    scale = min(abs(omega_radps) / taper_radps, 1.0)
    return -abs(magnitude_nm) * scale * (1.0 if omega_radps > 0.0 else -1.0)


__all__ = ["WheelSpin", "resolve_wheel_spin", "opposing_torque"]
