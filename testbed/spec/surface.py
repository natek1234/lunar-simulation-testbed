"""SurfaceSpec — the flat plane ships today; a future heightfield generator (not
implemented yet) extends this dataclass rather than replacing it."""
from dataclasses import dataclass
from pathlib import Path

from testbed.spec.yaml_io import from_dict, load_yaml

SURFACES_DIR = Path(__file__).resolve().parents[2] / "surfaces"  # [policy default: repo layout, testbed/spec/ is two levels under the repo root]


@dataclass(frozen=True)
class SurfaceSpec:
    kind: str                                   # flat (hfield kinds not implemented yet)
    gravity_mps2: float
    friction: tuple[float, float, float]        # sliding, torsional, rolling
    texture: str | None                         # repo-relative PNG or None
    extent_m: float                              # half-size of the ground square


def load_surface(path: Path) -> SurfaceSpec:
    s = from_dict(SurfaceSpec, load_yaml(path))
    if s.kind != "flat":
        raise ValueError(f"surface kind {s.kind!r} not available in this build (hfield kinds not implemented yet)")
    return s
