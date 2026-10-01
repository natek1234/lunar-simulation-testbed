"""Our own GLFW window + mjr render loop: physics paced to wall time, render decoupled at
`fps`, keyboard polled every control tick, HUD as an mjr text overlay. Close paths (window
close, ESC, Ctrl+C) all fall through the same `finally`, so the caller always saves.

The render path (MjvCamera/MjvScene/MjrContext/mjv_updateScene/mjr_render/mjr_overlay) was
probed end-to-end against a real GLFW window and the composed fr3_mpo700 model before this
file was written [measured: 2026-09-20, a full render + text overlay + swap_buffers cycle
against mujoco 3.13.0 / glfw 2.10.2 completed with no exception].

`hud_render` is injected (a HudSnapshot -> (left, right) callable), not imported here:
`tests/test_architecture.py::test_package_import_boundaries` forbids `sim` from importing
`testbed.viz` (viz stays a leaf nothing else in `testbed/` depends on except through this
seam), so the composition root (`testbed/bench/launch.py`, which -- unlike `sim` -- is free
to import both `sim` and `viz`) passes `testbed.viz.hud_text.render` in. With no
`hud_render` (e.g. this module's own viewer-tier test), the 3D scene still renders; only
the text overlay is skipped."""
import signal
import time

import glfw
import mujoco
import numpy as np

from testbed.control.keyboard_rc import KeyboardRc
from testbed.sim.keyboard import KeyboardDevice
from testbed.sim.runtime import RunResult, Runtime

CAM_DISTANCE_M = 6.0      # [policy default: whole-robot framing for a 1.2 m wheelbase]
CAM_AZIMUTH_DEG = 135.0   # [policy default]
CAM_ELEVATION_DEG = -25.0 # [policy default]

# Key bindings per docs/guide/teleop.md.
# [policy default]
HELP_TEXT = ("W/S A/D Q/E base  I/K J/L U/O ee  [ ] yaw  1-7 =/- joint  M mode  SPACE grip  "
             "X stop  R re-arm  H help  F follow  ESC quit")


class Viewer:
    def __init__(self, rt: Runtime, *, hud_render=None, fps: float = 60.0, width: int = 1280, height: int = 800,
                 title: str = "lunar testbed"):
        self.rt, self.fps, self.hud_render = rt, fps, hud_render
        if not glfw.init():
            raise RuntimeError("glfw.init failed")
        self.window = glfw.create_window(width, height, title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("glfw.create_window failed")
        glfw.make_context_current(self.window)
        glfw.swap_interval(0)
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.scn = mujoco.MjvScene(rt.h.model, maxgeom=10000)
        self.ctx = mujoco.MjrContext(rt.h.model, mujoco.mjtFontScale.mjFONTSCALE_100)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = CAM_DISTANCE_M, CAM_AZIMUTH_DEG, CAM_ELEVATION_DEG
        self.follow = True
        self.device = KeyboardDevice(self.window)
        self.rc = rt.controller if isinstance(rt.controller, KeyboardRc) else rt.rc
        self._close = False
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "_close", True))

    def loop(self, max_s: float | None = None) -> RunResult:
        rt = self.rt
        t_wall0 = time.perf_counter()
        next_render = 0.0
        rtf = 1.0
        try:
            while not (self._close or glfw.window_should_close(self.window)):
                if rt.tick_i % rt.decim == 0:
                    keys = self.device.poll()
                    if self.rc is not None:
                        self.rc.update(keys)
                        if self.rc.wants_rearm:
                            rt.rearm()
                        if self.rc.wants_close:
                            break
                    if "F" in keys.pressed:
                        self.follow = not self.follow
                rt.tick()
                if max_s is not None and rt.t_s >= max_s:
                    break
                if rt.controller.done(rt.obs) and self.rc is None:
                    break
                wall = time.perf_counter() - t_wall0
                if rt.t_s > wall:
                    time.sleep(rt.t_s - wall)
                if wall >= next_render:
                    rtf = rt.t_s / max(wall, 1e-6)
                    self._render(rtf)
                    next_render = wall + 1.0 / self.fps
        finally:
            glfw.terminate()
        return RunResult(rt.tick_i, rt.stopped, rt.help_count, rt.interventions)

    def _render(self, rtf: float):
        rt = self.rt
        if self.follow:
            self.cam.lookat[:] = rt.h.data.qpos[rt.h.base_qpos:rt.h.base_qpos + 3]
        w, h = glfw.get_framebuffer_size(self.window)
        vp = mujoco.MjrRect(0, 0, w, h)
        mujoco.mjv_updateScene(rt.h.model, rt.h.data, self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scn)
        mujoco.mjr_render(vp, self.scn, self.ctx)
        if self.hud_render is not None:
            mode = self.rc.mode if self.rc is not None else "task"
            sel = self.rc.selected_joint if self.rc is not None else 0
            left, right = self.hud_render(rt.hud_snapshot(rt.controller.name, rtf, sel, mode))
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, vp, left, right, self.ctx)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, vp, HELP_TEXT, "", self.ctx)
        glfw.swap_buffers(self.window)
