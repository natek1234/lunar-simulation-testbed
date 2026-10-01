"""Kinematics from the compiled model itself, on a scratch MjData with the base at the
origin so W == B. No per-robot kinematics table anywhere -- fk/jacobian/limits are all
read straight off the compiled model through the Handles the composer already resolved.

mj_jacSite silently returns a WRONG jacobian if mj_comPos hasn't been called since the
last mj_kinematics -- mj_kinematics alone updates site_xpos/site_xmat (what fk() reads)
but not subtree_com, which mj_jacSite's rotational term depends on even for a site
target. Measured directly on this composed model at the home pose: calling mj_jacSite
right after mj_kinematics (skipping mj_comPos) differs from the comPos'd result by up to
1.38 (linear) / 1.0 (angular) -- not a rounding difference, a wrong answer
[measured: scratch probe, 2026-09-18]. `_set` always calls both, in that order, before
either fk() or jacobian() reads anything."""
import mujoco
import numpy as np

from testbed.control.records import Pose
from testbed.sim.handles import Handles


class MujocoKinematics:
    def __init__(self, h: Handles):
        self._m = h.model
        self._d = mujoco.MjData(h.model)
        self._h = h
        self.n = len(h.arm_qpos)
        self._d.qpos[h.base_qpos:h.base_qpos + 7] = [0, 0, 0, 1, 0, 0, 0]
        self._jacp = np.zeros((3, self._m.nv))
        self._jacr = np.zeros((3, self._m.nv))
        lo = self._m.jnt_range[[self._m.dof_jntid[d] for d in h.arm_dof], 0]
        hi = self._m.jnt_range[[self._m.dof_jntid[d] for d in h.arm_dof], 1]
        self._limits = (lo.copy(), hi.copy())
        self.arm_base_B_m = self._arm_base_B_m(h)

    def _arm_base_B_m(self, h: Handles) -> np.ndarray:
        """The arm's own root body position in B (base at the origin, per __init__), read
        once off the compiled model rather than trusted from the spec's `mount_pos_B_m` --
        the root body doesn't sit on the arm's own kinematic chain (nothing below it moves
        it), so one mj_kinematics at the all-zero scratch qpos is enough; it never needs
        recomputing. Used by testbed.control.envelope.measure_reach_m to measure reach
        from the true arm base rather than assume the spec's mount frame and the compiled
        root body coincide exactly (attach() composes them through a quaternion, and the
        child model's own root body can carry a nonzero local offset)."""
        r = h.compiled.robot.arm
        name = r.attach_prefix + r.root_body
        bid = mujoco.mj_name2id(self._m, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            return np.asarray(r.mount_pos_B_m, dtype=float)
        mujoco.mj_kinematics(self._m, self._d)
        return self._d.xpos[bid].copy()

    def _set(self, q: np.ndarray) -> None:
        self._d.qpos[self._h.arm_qpos] = q
        mujoco.mj_kinematics(self._m, self._d)
        mujoco.mj_comPos(self._m, self._d)

    def fk(self, q_rad: np.ndarray) -> Pose:
        self._set(q_rad)
        s = self._h.flange_site
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self._d.site_xmat[s])
        return Pose(self._d.site_xpos[s].copy(), quat)

    def jacobian(self, q_rad: np.ndarray) -> np.ndarray:
        self._set(q_rad)
        mujoco.mj_jacSite(self._m, self._d, self._jacp, self._jacr, self._h.flange_site)
        return np.vstack([self._jacp[:, self._h.arm_dof], self._jacr[:, self._h.arm_dof]])

    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._limits
