"""`app.py physics`: the compiled model's own truths for one robot on one surface."""
import mujoco
import numpy as np

from testbed.sim import compose as C, handles as H
from testbed.spec import robot as R, surface as S


def physics_report(robot_name: str, surface_name: str) -> int:
    robot = R.load_robot(R.ROBOTS_DIR / f"{robot_name}.yaml")
    surface = S.load_surface(S.SURFACES_DIR / f"{surface_name}.yaml")
    h = H.resolve(C.compose(robot, surface))
    m, d = h.model, h.data
    mujoco.mj_forward(m, d)
    total = float(m.body_subtreemass[h.base_body])
    print(f"robot {robot.name} on {surface.kind}: nbody={m.nbody} nq={m.nq} nu={m.nu} nsensor={m.nsensor}")
    print(f"  timestep {m.opt.timestep} s  gravity {m.opt.gravity[2]:+.2f} m/s^2  total mass {total:.1f} kg")
    print("  modules: " + ", ".join(mod.name for mod in robot.base.modules) + f"  wheel r {robot.base.modules[0].wheel_radius_m} m")
    arm_root = m.body(robot.arm.attach_prefix + robot.arm.root_body).id
    reach = float(np.linalg.norm(d.site_xpos[h.flange_site] - d.xpos[arm_root]))
    print(f"  flange at q=0: {d.site_xpos[h.flange_site].round(3)} W; |flange - arm base| {reach:.3f} m")
    print(f"  force budget {robot.force_budget_n} N; gripper kind {robot.gripper.kind}"
          + (f" (hold {robot.gripper.magnet.hold_force_n} N, capture {robot.gripper.magnet.capture_distance_m} m)"
             if robot.gripper.kind == "magnet" else f" (ctrl open/close {robot.gripper.cmd_open}/{robot.gripper.cmd_close})"))
    return 0
