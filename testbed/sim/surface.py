"""Ground for the flat surface kind: one plane geom, textured. Heightfield generation
would extend this module (not implemented yet)."""
from pathlib import Path

import mujoco

from testbed.spec.surface import SurfaceSpec

REPO = Path(__file__).resolve().parents[2]  # [policy default: repo layout, testbed/sim/ is two levels under the repo root]
GROUND_GEOM_NAME = "ground"
TEXREPEAT = (40, 40)   # [policy default: dense enough that the 1k texture doesn't look stretched at extent_m~25]


def add_surface(spec: mujoco.MjSpec, surface: SurfaceSpec) -> None:
    if surface.kind != "flat":
        raise ValueError(surface.kind)
    material = None
    if surface.texture:
        spec.add_texture(name="ground_tex", type=mujoco.mjtTexture.mjTEXTURE_2D,
                          file=str(REPO / surface.texture))
        mat = spec.add_material(name="ground_mat")
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "ground_tex"
        mat.texrepeat = list(TEXREPEAT)
        material = mat.name
    g = spec.worldbody.add_geom(name=GROUND_GEOM_NAME, type=mujoco.mjtGeom.mjGEOM_PLANE,
                                 size=[surface.extent_m, surface.extent_m, 0.1])   # [policy default: plane geom's z-size entry is a rendering-only extent, per MuJoCo docs]
    g.friction = list(surface.friction)
    if material:
        g.material = material
