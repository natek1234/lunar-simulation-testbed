"""Scenario = worksite (RCUs, slots, spawn) + the scripted baseline + timeout."""
from dataclasses import dataclass
from pathlib import Path

from testbed.spec.yaml_io import from_dict, load_yaml

SCENARIOS_DIR = Path(__file__).resolve().parents[2] / "scenarios"  # [policy default: repo layout, testbed/spec/ is two levels under the repo root]


@dataclass(frozen=True)
class RcuSpec:
    extents_m: tuple[float, float, float]     # full L x W x H
    mass_kg: float
    friction: tuple[float, float, float]
    corner_p: float                           # superellipsoid exponent (2 = ellipsoid, large = box)


@dataclass(frozen=True)
class RcuPlacement:
    id: str
    xyz_W_m: tuple[float, float, float]       # CENTRE at spawn (resting: z = H/2 above what it sits on)
    yaw_rad: float


@dataclass(frozen=True)
class PalletSpec:
    id: str
    xyz_W_m: tuple[float, float, float]       # centre of a fixed rigid box
    half_extents_m: tuple[float, float, float]


@dataclass(frozen=True)
class SlotSpec:
    id: str
    xyz_W_m: tuple[float, float, float]       # target CENTRE of the seated RCU
    yaw_rad: float


@dataclass(frozen=True)
class WorksiteSpec:
    rcu: RcuSpec
    staged: tuple[RcuPlacement, ...]          # every free RCU: the unit(s) to place AND pre-placed foundation units
    pallets: tuple[PalletSpec, ...]
    slots: tuple[SlotSpec, ...]
    robot_spawn_xy_yaw: tuple[float, float, float]
    to_place: tuple[str, ...]                 # ids of the staged units the script is expected to move


@dataclass(frozen=True)
class ScriptStep:
    skill: str
    params: dict


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    worksite: WorksiteSpec
    script_steps: tuple[ScriptStep, ...]
    timeout_s: float

    @property
    def script(self):
        return tuple((s.skill, dict(s.params)) for s in self.script_steps)


def load_scenario(path: Path) -> ScenarioSpec:
    return from_dict(ScenarioSpec, load_yaml(path))
