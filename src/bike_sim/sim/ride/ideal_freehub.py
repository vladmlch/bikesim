"""Zero-backlash one-way transmission using a solved unilateral tendon limit."""
import mujoco
import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.physical_mapping import resolve_id


class IdealFreehubConstraint:
    def __init__(self, model, ratio):
        self.ratio = scalar(ratio, 'gear ratio', positive=True)
        self.tendon_id = resolve_id(model, mujoco.mjtObj.mjOBJ_TENDON, 'ideal_mid_drive_freehub')
        self.crank_qpos = int(model.joint('crank_spin').qposadr[0])
        self.wheel_qpos = int(model.joint('rear_wheel_spin').qposadr[0])
        self.wheel_dof = int(model.joint('rear_wheel_spin').dofadr[0])
        crank_joint = model.joint('crank_spin').id
        path_start = int(model.tendon_adr[self.tendon_id])
        path_end = path_start + int(model.tendon_num[self.tendon_id])
        coefficients = [index for index in range(path_start, path_end)
                        if model.wrap_type[index] == mujoco.mjtWrap.mjWRAP_JOINT
                        and model.wrap_objid[index] == crank_joint]
        if len(coefficients) != 1:
            raise ValueError('ideal freehub needs one crank joint coefficient')
        self.crank_coefficient = coefficients[0]
        self.constant_data = mujoco.MjData(model)
        self.boundary = None

    def _relative_angle(self, data):
        return float(self.ratio * data.qpos[self.crank_qpos] - data.qpos[self.wheel_qpos])

    def reset(self, model, data):
        self.boundary = self._relative_angle(data)
        model.tendon_range[self.tendon_id, 1] = self.boundary

    def prepare(self, model, data):
        relative = self._relative_angle(data)
        self.boundary = relative if self.boundary is None else min(self.boundary, relative)
        model.tendon_range[self.tendon_id, 1] = self.boundary

    def set_ratio(self, model, data, ratio):
        ratio = scalar(ratio, 'gear ratio', positive=True)
        if ratio == self.ratio:
            return
        previous_relative = self._relative_angle(data)
        self.ratio = ratio
        model.wrap_prm[self.crank_coefficient] = ratio
        mujoco.mj_setConst(model, self.constant_data)
        relative = self._relative_angle(data)
        self.boundary = (relative if self.boundary is None
                         else self.boundary + relative - previous_relative)
        model.tendon_range[self.tendon_id, 1] = self.boundary

    def solved_qfrc(self, model, data):
        force = np.zeros(model.nv)
        selected = ((data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)
                    & (data.efc_id[:data.nefc] == self.tendon_id))
        if np.any(selected):
            multipliers = np.zeros(data.nefc)
            multipliers[selected] = data.efc_force[:data.nefc][selected]
            mujoco.mj_mulJacTVec(model, data, force, multipliers)
        return force
