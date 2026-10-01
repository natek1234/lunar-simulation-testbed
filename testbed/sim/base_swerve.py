"""Swerve base strategy: rover-scales the vendored chassis, adds steer (position) and
drive (velocity) actuators per module. Suspension is not implemented yet.

Chassis envelope for the inertia box model: 0.65 x 0.505 x 0.38 m [derived: MPO-700
mesh bbox measured off assets/mpo700/meshes/MPO-700-BODY.obj, x footprint_scale in xy;
mass unchanged]."""
import mujoco
import numpy as np

from testbed.spec.robot import BaseSpec

CHASSIS_ENVELOPE_M = (0.65, 0.505, 0.38)  # [derived: MPO-700 vendored mesh bbox, unscaled]


def compose(spec: BaseSpec, mj: mujoco.MjSpec) -> None:
    """Called with `mj` already loaded from spec.chassis_mjcf. Applies footprint and
    module scaling in place so the compiled geometry matches spec.modules."""
    fs, ms = spec.footprint_scale, spec.module_scale
    chassis = mj.body(spec.body)
    for me in mj.meshes:                      # chassis meshes: xy only; wheel/head meshes: uniform
        sc = np.asarray(me.scale, dtype=float)
        me.scale = sc * np.array([fs, fs, 1.0]) if "BODY" in me.name else sc * ms
    for mod in spec.modules:
        caster = mj.joint(mod.steer_joint).parent
        caster.pos = np.array([mod.xy_B_m[0], mod.xy_B_m[1], caster.pos[2]])
        _scale_module_body(caster, ms)
        wheel = mj.body(mod.wheel_body)
        wheel.pos = np.asarray(wheel.pos) * ms
        _scale_module_body(wheel, ms)
        for g in wheel.geoms:
            if g.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                g.size = np.array([mod.wheel_radius_m, 0, 0])
    # Chassis inertia: box model at the scaled envelope, mass unchanged.
    lx, ly, lz = CHASSIS_ENVELOPE_M[0] * fs, CHASSIS_ENVELOPE_M[1] * fs, CHASSIS_ENVELOPE_M[2]
    m = chassis.mass
    chassis.inertia = np.array([m / 12 * (ly**2 + lz**2), m / 12 * (lx**2 + lz**2), m / 12 * (lx**2 + ly**2)])


def _scale_module_body(body, s: float) -> None:
    if body.mass > 0:
        body.mass = body.mass * s ** 3
        body.inertia = np.asarray(body.inertia) * s ** 5


def add_actuators(spec: BaseSpec, mj: mujoco.MjSpec) -> None:
    for mod in spec.modules:
        a = mj.add_actuator(name=spec.steer_actuator_prefix + mod.name, target=mod.steer_joint,
                             trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm[0] = spec.steer_kp
        a.biasprm[1] = -spec.steer_kp
        a.biasprm[2] = -spec.steer_kv
        a.ctrlrange = [-spec.steer_limit_rad, spec.steer_limit_rad]
        a.ctrllimited = True
        a = mj.add_actuator(name=spec.drive_actuator_prefix + mod.name, target=mod.drive_joint,
                             trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm[0] = spec.drive_kv
        a.biasprm[2] = -spec.drive_kv
