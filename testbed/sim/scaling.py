"""Uniform geometric scaling of a child MjSpec (positions x s, masses x s^3, inertias x s^5).
Scaling only positions/masses/inertias (never orientations) reproduces a uniformly-scaled
kinematic chain exactly, since rotation matrices commute with scalar multiplication:
R @ (s * p) == s * (R @ p) -- cross-checked against a live FK forward pass
[measured: scratch probe, 2026-09-18] on the vendored FR3 at scale 1.0 and 1.5."""
import mujoco
import numpy as np


def scale_spec(spec: mujoco.MjSpec, s: float) -> None:
    if s == 1.0:
        return
    for body in spec.bodies:
        body.pos = np.asarray(body.pos) * s
        if body.mass > 0:
            body.mass = body.mass * s ** 3
            body.inertia = np.asarray(body.inertia) * s ** 5
            body.ipos = np.asarray(body.ipos) * s
    for g in spec.geoms:
        g.pos = np.asarray(g.pos) * s
        g.size = np.asarray(g.size) * s
        g.fromto = np.asarray(g.fromto) * s
    for st in spec.sites:
        st.pos = np.asarray(st.pos) * s
    for j in spec.joints:
        j.pos = np.asarray(j.pos) * s
    for me in spec.meshes:
        me.scale = np.asarray(me.scale) * s
    for c in spec.cameras:
        c.pos = np.asarray(c.pos) * s
