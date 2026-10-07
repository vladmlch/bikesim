"""Zero-backlash one-way transmission using a solved unilateral tendon limit."""
import copy

import mujoco
import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.physical_mapping import resolve_id
from bike_sim.sim.ride.wheels import resolve_scalar_joint


class IdealFreehubConstraint:
    """One-way coupling driver -> driven through an upper-limited fixed tendon.

    The tendon measures ``ratio*driver - driven``; the upper boundary ratchets
    down with any relative overrun so forward re-engagement has zero lash.
    Defaults keep the cassette freehub (crank_spin drives rear_wheel_spin).
    The motor freewheel reuses this class with rotor_spin -> crank_spin at
    ratio 1; the legacy crank clutch uses crank_spin -> drive_shaft_spin.

    Mutations are transactions: the candidate is validated against an owned
    scratch model (candidate ``mj_setConst`` lands on the copy, never on the
    committed model), live model writes then run, and the plain attributes
    are published last. A rejected candidate leaves every attribute and model
    row untouched.
    """
    def __init__(self, model, ratio, *, tendon_name='ideal_mid_drive_freehub',
                 driver='crank_spin', driven='rear_wheel_spin'):
        self.ratio = scalar(ratio, 'gear ratio', positive=True)
        self.tendon_id = resolve_id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        self.driver_qpos, self.driver_dof = resolve_scalar_joint(model, driver)
        self.driven_qpos, self.driven_dof = resolve_scalar_joint(model, driven)
        driver_joint = model.joint(driver).id
        path_start = int(model.tendon_adr[self.tendon_id])
        path_end = path_start + int(model.tendon_num[self.tendon_id])
        coefficients = [index for index in range(path_start, path_end)
                        if model.wrap_type[index] == mujoco.mjtWrap.mjWRAP_JOINT
                        and model.wrap_objid[index] == driver_joint]
        if len(coefficients) != 1:
            raise ValueError('ideal freehub needs one driver joint coefficient')
        self.driver_coefficient = coefficients[0]
        self.constant_data = mujoco.MjData(model)
        # Owned scratch world for candidate validation. It is resynced from
        # the live model before every staged engine call, so earlier rejected
        # candidates never leak into a later attempt.
        self.scratch_model = copy.deepcopy(model)
        self.scratch_data = mujoco.MjData(self.scratch_model)
        self.boundary = None

    def _relative_angle(self, data):
        return float(self.ratio * data.qpos[self.driver_qpos] - data.qpos[self.driven_qpos])

    def relative_rate(self, data):
        """Tendon-length rate: positive while the driver outruns the driven."""
        return float(self.ratio * data.qvel[self.driver_dof] - data.qvel[self.driven_dof])

    def _sync_scratch(self, model):
        scratch = self.scratch_model
        scratch.wrap_prm[:] = model.wrap_prm
        scratch.tendon_range[:] = model.tendon_range
        scratch.opt.timestep = model.opt.timestep
        return scratch

    def reset(self, model, data):
        boundary = scalar(self._relative_angle(data), 'transmission boundary')
        model.tendon_range[self.tendon_id, 1] = boundary
        self.boundary = boundary

    def prepare(self, model, data):
        relative = self._relative_angle(data)
        boundary = scalar(relative if self.boundary is None
                          else min(self.boundary, relative), 'transmission boundary')
        model.tendon_range[self.tendon_id, 1] = boundary
        self.boundary = boundary

    def set_ratio(self, model, data, ratio):
        ratio = scalar(ratio, 'gear ratio', positive=True)
        if ratio == self.ratio:
            return
        previous_relative = self._relative_angle(data)
        current = ratio * data.qpos[self.driver_qpos] - data.qpos[self.driven_qpos]
        boundary = scalar(current if self.boundary is None
                          else self.boundary + current - previous_relative,
                          'transmission boundary')
        # Rehearse the commit on the owned scratch model in the same order it
        # will run below: a fatal mj_setConst lands on the copy, leaving live
        # constants and attributes untouched.
        scratch = self._sync_scratch(model)
        scratch.wrap_prm[self.driver_coefficient] = ratio
        mujoco.mj_setConst(scratch, self.scratch_data)
        scratch.tendon_range[self.tendon_id, 1] = boundary
        # Commit: guarded live writes first, then the memory-only state swap.
        model.wrap_prm[self.driver_coefficient] = ratio
        mujoco.mj_setConst(model, self.constant_data)
        model.tendon_range[self.tendon_id, 1] = boundary
        self.ratio = ratio
        self.boundary = boundary

    def solved_qfrc(self, model, data):
        force = np.zeros(model.nv)
        selected = ((data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)
                    & (data.efc_id[:data.nefc] == self.tendon_id))
        if np.any(selected):
            multipliers = np.zeros(data.nefc)
            multipliers[selected] = data.efc_force[:data.nefc][selected]
            mujoco.mj_mulJacTVec(model, data, force, multipliers)
        return force
