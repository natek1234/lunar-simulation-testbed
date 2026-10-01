"""compose(): RobotSpec + SurfaceSpec (+ WorksiteSpec) -> one live MjSpec, compiled in memory.
Singletons (timestep, gravity, integrator) are written here, last, and asserted by tests.
Never round-tripped through MjSpec.to_xml() -- the composed spec is compiled directly."""
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from testbed.sim import base_swerve, gripper_magnet, gripper_parallel, surface as SURF
from testbed.sim.scaling import scale_spec
from testbed.spec.robot import RobotSpec
from testbed.spec.surface import SurfaceSpec

REPO = Path(__file__).resolve().parents[2]
TIMESTEP_S = 0.002     # [policy default: 500 Hz physics; the vendored 0.005 is overridden on purpose]

BASE_STRATEGIES = {"swerve": base_swerve}
GRIPPER_STRATEGIES = {"magnet": gripper_magnet, "parallel": gripper_parallel}


def gripper_strategy(robot: RobotSpec):
    return GRIPPER_STRATEGIES[robot.gripper.kind]


@dataclass(frozen=True)
class Compiled:
    model: mujoco.MjModel
    robot: RobotSpec
    surface: SurfaceSpec


def compose(robot: RobotSpec, surface: SurfaceSpec, worksite=None) -> Compiled:
    base = BASE_STRATEGIES[robot.base.kind]
    grip = GRIPPER_STRATEGIES[robot.gripper.kind]

    mj = mujoco.MjSpec.from_file(str(REPO / robot.base.chassis_mjcf))
    base.compose(robot.base, mj)
    base.add_actuators(robot.base, mj)
    SURF.add_surface(mj, surface)

    arm = mujoco.MjSpec.from_file(str(REPO / robot.arm.mjcf))
    scale_spec(arm, robot.arm.scale)
    mount = mj.body(robot.base.body).add_frame(name="arm_mount", pos=list(robot.arm.mount_pos_B_m),
                                                quat=list(robot.arm.mount_quat_wxyz))
    mj.attach(arm, prefix=robot.arm.attach_prefix, frame=mount)

    flange_site = mj.site(robot.arm.flange_site)
    flange_body = flange_site.parent.name
    child = mujoco.MjSpec.from_file(str(REPO / robot.gripper.mjcf)) if robot.gripper.mjcf else None
    grip.compose(robot.gripper, mj, child, flange_body, np.asarray(flange_site.pos))

    for sname, stype in (("ft_force", mujoco.mjtSensor.mjSENS_FORCE), ("ft_torque", mujoco.mjtSensor.mjSENS_TORQUE)):
        mj.add_sensor(name=sname, type=stype, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=robot.sensors.ft_site)

    if worksite is not None:
        from testbed.sim import worksite as WS
        WS.add_worksite(mj, worksite, surface)
        if robot.gripper.kind == "magnet":
            gripper_magnet.add_weld_candidates(mj, [r.id for r in worksite.staged])

    mj.option.timestep = TIMESTEP_S
    mj.option.gravity = [0.0, 0.0, -surface.gravity_mps2]
    mj.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model = mj.compile()
    _assert_singletons(model, surface)
    return Compiled(model=model, robot=robot, surface=surface)


def _assert_singletons(model: mujoco.MjModel, surface: SurfaceSpec) -> None:
    if (model.opt.timestep != TIMESTEP_S or not np.isclose(model.opt.gravity[2], -surface.gravity_mps2)
            or model.opt.integrator != mujoco.mjtIntegrator.mjINT_IMPLICITFAST):
        raise RuntimeError(f"singleton drift: timestep={model.opt.timestep} gravity={model.opt.gravity} "
                            f"integrator={model.opt.integrator}")
