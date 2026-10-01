"""RCU bodies (superellipsoid meshes, freejoints) and slot sites; the Worksite runtime
object owns spawn, the oracle/tag/goal channels, and gripper attachment detection.

Gripper attachment itself lives in the gripper strategy modules (`gripper_magnet.py`,
`gripper_parallel.py`) -- this module only supplies the bodies/sites they act on
and the test-only `seat_in_gripper` helper that places a real RCU where the end effector
would hold it, so the attachment logic can be exercised against a real body."""
import mujoco
import numpy as np

from testbed.control import quat as Q
from testbed.control.records import Goal, OracleObject, Pose
from testbed.sim.handles import Handles
from testbed.spec.scenario import WorksiteSpec

SLOT_SITE_SIZE_M = 0.02      # [policy default: visual marker only]


def superellipsoid_vertices(half_extents, p: float, n: int = 12) -> np.ndarray:
    """A superquadric box-with-rounded-corners hull, `n` latitude x `2n` longitude
    samples, as `add_mesh(uservert=...)` wants: a flat (N, 3) point cloud MuJoCo's own
    convex-hull compiler turns into a mesh -- there is no explicit face list to author."""
    a, b, c = half_extents
    verts = []
    for th in np.linspace(-np.pi / 2, np.pi / 2, n):
        for ph in np.linspace(-np.pi, np.pi, 2 * n, endpoint=False):
            ct, st, cp, sp = np.cos(th), np.sin(th), np.cos(ph), np.sin(ph)
            f = lambda x, e: np.sign(x) * abs(x) ** e
            verts.append([a * f(ct, 2 / p) * f(cp, 2 / p), b * f(ct, 2 / p) * f(sp, 2 / p), c * f(st, 2 / p)])
    return np.asarray(verts)


def add_worksite(mj: mujoco.MjSpec, ws: WorksiteSpec, surface) -> None:
    mesh = mj.add_mesh(name="rcu_mesh")
    mesh.uservert = superellipsoid_vertices([e / 2 for e in ws.rcu.extents_m], ws.rcu.corner_p).ravel()
    for p in ws.pallets:
        g = mj.worldbody.add_geom(name=p.id, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(p.xyz_W_m), size=list(p.half_extents_m))
        g.rgba = [0.4, 0.4, 0.45, 1]
    for r in ws.staged:
        body = mj.worldbody.add_body(name=r.id, pos=list(r.xyz_W_m),
                                      quat=list(Q.from_rotvec(np.array([0, 0, r.yaw_rad]))))
        body.add_freejoint(name=f"{r.id}_free")
        g = body.add_geom(name=f"{r.id}_geom", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="rcu_mesh", mass=ws.rcu.mass_kg)
        g.friction = list(ws.rcu.friction)
        g.rgba = [0.55, 0.45, 0.3, 1]
        patch = body.add_site(name=f"{r.id}_patch", pos=[0, 0, ws.rcu.extents_m[2] / 2])   # ferromagnetic patch, top face centre, +z normal
        patch.size = [0.02, 0, 0]
        patch.rgba = [0.6, 0.6, 0.7, 0.8]
    for s in ws.slots:
        site = mj.worldbody.add_site(name=s.id, pos=list(s.xyz_W_m), quat=list(Q.from_rotvec(np.array([0, 0, s.yaw_rad]))))
        site.size = [SLOT_SITE_SIZE_M, 0, 0]
        site.rgba = [0.1, 0.9, 0.1, 0.6]


class Worksite:
    def __init__(self, spec: WorksiteSpec):
        self.spec = spec
        self.filled = [False] * len(spec.slots)

    def reset(self, h: Handles, rng: np.random.Generator) -> None:
        m, d = h.model, h.data
        self.filled = [False] * len(self.spec.slots)
        x, y, yaw = self.spec.robot_spawn_xy_yaw
        d.qpos[h.base_qpos:h.base_qpos + 2] = [x, y]
        d.qpos[h.base_qpos + 3:h.base_qpos + 7] = Q.from_rotvec(np.array([0, 0, yaw]))
        for r in self.spec.staged:
            adr = m.jnt_qposadr[m.joint(f"{r.id}_free").id]
            d.qpos[adr:adr + 3] = [r.xyz_W_m[0], r.xyz_W_m[1], r.xyz_W_m[2] + 0.005]
            d.qpos[adr + 3:adr + 7] = Q.from_rotvec(np.array([0, 0, r.yaw_rad]))
        self.rcu_bodies = {r.id: int(m.body(r.id).id) for r in self.spec.staged}

    def rcu_pose_W(self, h: Handles, rcu_id: str) -> Pose:
        adr = h.model.jnt_qposadr[h.model.joint(f"{rcu_id}_free").id]
        return Pose(h.data.qpos[adr:adr + 3].copy(), h.data.qpos[adr + 3:adr + 7].copy())

    def slot_pose_W(self, slot_id: str) -> Pose:
        s = next(s for s in self.spec.slots if s.id == slot_id)
        return Pose(np.asarray(s.xyz_W_m, dtype=float), Q.from_rotvec(np.array([0, 0, s.yaw_rad])))

    def mark_filled(self, slot_id: str) -> None:
        self.filled[[s.id for s in self.spec.slots].index(slot_id)] = True

    def observe(self, h: Handles):
        oracle = tuple(OracleObject(r.id, self.rcu_pose_W(h, r.id), float(h.model.body_mass[self.rcu_bodies[r.id]]))
                        for r in self.spec.staged)
        ee = h.data.site_xpos[h.interface_site]
        rng = h.compiled.robot.sensors.tag_read_range_m
        tags = tuple((o.id, o.mass_kg) for o in oracle if np.linalg.norm(o.pose_W.pos_m - ee) <= rng)
        target = next((i for i, f in enumerate(self.filled) if not f), None)
        goal = Goal(tuple(self.slot_pose_W(s.id) for s in self.spec.slots), tuple(self.filled), target,
                    tuple(s.id for s in self.spec.slots))
        return oracle, tags, goal

    def seat_in_gripper(self, h: Handles, rcu_id: str) -> None:
        """Test helper: put the unit where the end effector would hold it, per gripper kind.
        magnet: patch centre at the magnet face, patch normal opposing the face normal
        (unit hangs below the magnet, top face up when the tool points down).
        parallel: unit between the pads, its 0.10 m face across the gap (unit z -> hand y)."""
        m, d = h.model, h.data
        mujoco.mj_forward(m, d)
        pos = d.site_xpos[h.interface_site].copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, d.site_xmat[h.interface_site])
        kind = h.compiled.robot.gripper.kind
        if kind == "magnet":
            half_h = self.spec.rcu.extents_m[2] / 2
            unit_in_tool = Q.from_rotvec(np.array([np.pi, 0, 0]))          # unit +z (patch normal) -> tool -z
            offset = Q.rotate(quat, [0, 0, half_h])                          # centre sits half a height along tool +z
        else:
            unit_in_tool = Q.from_rotvec(np.array([np.pi / 2, 0, 0]))      # unit z -> hand y
            offset = np.zeros(3)
        adr = m.jnt_qposadr[m.joint(f"{rcu_id}_free").id]
        d.qpos[adr:adr + 3] = pos + offset
        d.qpos[adr + 3:adr + 7] = Q.mul(quat, unit_in_tool)
        d.qvel[m.jnt_dofadr[m.joint(f"{rcu_id}_free").id]:][:6] = 0
        mujoco.mj_forward(m, d)
