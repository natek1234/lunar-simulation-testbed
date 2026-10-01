"""RobotSpec — a robot is data. Units SI; frame B = the base body (x forward, z up).
Every name here is resolved against the compiled model by testbed.sim.handles."""
from dataclasses import dataclass
from pathlib import Path

from testbed.spec.yaml_io import from_dict, load_yaml

ROBOTS_DIR = Path(__file__).resolve().parents[2] / "robots"  # [policy default: repo layout, testbed/spec/ is two levels under the repo root]


@dataclass(frozen=True)
class ModuleSpec:
    name: str
    xy_B_m: tuple[float, float]         # steer axis position in B
    steer_joint: str
    drive_joint: str
    wheel_body: str
    wheel_radius_m: float
    lateral_offset_m: float             # |wheel centre off the steer axis| (caster offset)


@dataclass(frozen=True)
class SuspensionSpec:
    travel_m: tuple[float, float]       # (droop <0, bump >0) along caster z
    ride_hz: float
    zeta: float


@dataclass(frozen=True)
class BaseSpec:
    kind: str                           # swerve | skid | floating
    body: str                           # chassis body name (root, carries the freejoint)
    chassis_mjcf: str | None            # repo-relative path, or None for an authored box
    footprint_scale: float              # applied to module xy and chassis mesh xy
    module_scale: float                 # applied to wheel radius / offsets / module masses
    modules: tuple[ModuleSpec, ...]
    steer_actuator_prefix: str          # actuator names = prefix + module name
    drive_actuator_prefix: str
    steer_limit_rad: float
    steer_rate_max_radps: float
    steer_kp: float
    steer_kv: float
    drive_kv: float
    max_vx_mps: float
    max_vy_mps: float
    max_wz_radps: float
    spawn_z_W_m: float                  # base body z at spawn on a flat surface
    suspension: SuspensionSpec | None = None


@dataclass(frozen=True)
class ArmSpec:
    mjcf: str                           # repo-relative MJCF path
    scale: float
    root_body: str                      # the child body attached at the mount (e.g. "base")
    joints: tuple[str, ...]
    actuators: tuple[str, ...]
    home_q_rad: tuple[float, ...]
    flange_site: str
    mount_pos_B_m: tuple[float, float, float]
    mount_quat_wxyz: tuple[float, float, float, float]
    attach_prefix: str = ""             # "" keeps the vendored names verbatim


@dataclass(frozen=True)
class MagnetSpec:
    radius_m: float                     # electromagnet face radius (authored cylinder)
    height_m: float
    mass_kg: float
    capture_distance_m: float           # interface site to RCU patch site distance that engages
    capture_angle_rad: float            # max angle between magnet face normal and patch normal
    hold_force_n: float                 # breakaway: |F_flange| above this releases the weld


@dataclass(frozen=True)
class GripperSpec:
    kind: str                           # magnet | parallel
    mount_quat_wxyz: tuple[float, float, float, float]   # TARGET orientation rel. flange
    interface_site: str                 # the tool point (magnet face centre / pad centre); ADDED by the composer
    interface_offset_m: tuple[float, float, float]        # site pos in the gripper root frame
    # magnet kind (authored geometry, no MJCF, no actuator)
    magnet: MagnetSpec | None = None
    # parallel kind (vendored MJCF with its own actuator)
    mjcf: str | None = None
    scale: float = 1.0
    root_body: str | None = None
    attach_prefix: str = ""
    actuator: str | None = None         # name AFTER prefixing
    cmd_open: float = 0.0
    cmd_close: float = 0.0
    finger_joints: tuple[str, ...] = ()
    gain_override: float | None = None  # actuator gain multiplier (None = vendored)


@dataclass(frozen=True)
class CameraSpec:
    name: str
    parent_body: str
    pos_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    fovy_deg: float
    width_px: int
    height_px: int


@dataclass(frozen=True)
class SensorSpec:
    ft_site: str                        # flange site the F/T sensor reads at
    cameras: tuple[CameraSpec, ...] = ()
    tag_read_range_m: float = 2.0       # [policy default: RFID read range assumed by the single-placement baseline scenario]


@dataclass(frozen=True)
class RobotSpec:
    name: str
    base: BaseSpec
    arm: ArmSpec
    gripper: GripperSpec
    sensors: SensorSpec
    force_budget_n: float               # flange-load limit the envelope enforces (not a pure contact-force limit -- see docs/reference/requirements.md)


def validate(spec: RobotSpec) -> RobotSpec:
    n = len(spec.arm.joints)
    if len(spec.arm.actuators) != n:
        raise ValueError(f"arm.actuators has {len(spec.arm.actuators)} entries, joints {n}")
    if len(spec.arm.home_q_rad) != n:
        raise ValueError(f"arm.home_q_rad has {len(spec.arm.home_q_rad)} entries, joints {n}")
    if spec.base.kind not in ("swerve", "skid", "floating"):
        raise ValueError(f"base.kind {spec.base.kind!r}")
    g = spec.gripper
    if g.kind == "magnet" and g.magnet is None:
        raise ValueError("gripper.kind magnet needs a `magnet:` block")
    if g.kind == "parallel" and not (g.mjcf and g.root_body and g.actuator and g.finger_joints):
        raise ValueError("gripper.kind parallel needs mjcf, root_body, actuator, finger_joints")
    if g.kind not in ("parallel", "magnet"):
        raise ValueError(f"gripper.kind {g.kind!r}")
    names = [m.name for m in spec.base.modules]
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate module names {names}")
    return spec


def load_robot(path: Path) -> RobotSpec:
    return validate(from_dict(RobotSpec, load_yaml(path)))
