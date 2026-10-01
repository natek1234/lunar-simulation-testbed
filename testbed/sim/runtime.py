"""The loop. Physics at dt; the controller at control_hz; every action level resolved
top-down (skill -> task -> joint -> ctrl); stop/help handled here; every tick recorded
into `episode` when one is attached (a duck-typed object, e.g. `testbed.bench.episode.
EpisodeLog` -- this module never imports `testbed.bench`, per the one-directional
bench -> sim import boundary in tests/test_architecture.py).

`run(duration_s)` with an EXPLICIT `duration_s` always executes exactly
`int(duration_s / dt)` physics ticks -- it does NOT stop early when `controller.done()`
turns true partway through (e.g. a scripted script whose skills sum to less than the
requested duration): measured against `tests/test_runtime.py::
test_scripted_drive_moves_the_base_forward_and_jog_lifts_the_flange` (script totals 3.2 s
of skills, `run(3.3)` must still return `ticks == int(3.3/0.002) == 1649`, not stop around
tick 1600 when the script itself finishes). Once a script is exhausted,
`ScriptedSequencer.act()` returns a bare `TaskAction()` (hold), so the extra ticks just
hold position. `run(None)` (no duration given, e.g. `app.py run --controller scripted
--headless` with no `--duration-s`) instead runs UNTIL `controller.done()` -- this is what
`docs/guide/running.md`'s `run --controller scripted --headless --duration-s 8` demo means
by "runs as fast as the CPU allows" when no bound is given, and it is why `Controller.done()`
is in the protocol at all."""
from dataclasses import dataclass
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from testbed.control import skills as SK  # noqa: F401 -- import registers "hold"/"drive"/"jog" AND, via
                                           # skills.py's own bottom-of-file import, placement_skills' own skills
from testbed.control.envelope import Envelope
from testbed.control.ik import IkConfig, ResolvedRateIk
from testbed.control.records import (ActionChunk, GripperCmd, HudSnapshot, JointAction, Pose, SkillAction,
                                     TaskAction, Twist)
from testbed.control.swerve import ModuleCommand, ModuleFirmware, SwerveAllocator
from testbed.sim import sensors as SEN
from testbed.sim.compose import TIMESTEP_S, gripper_strategy
from testbed.sim.handles import Handles
from testbed.sim.kinematics_mujoco import MujocoKinematics

CONTROL_HZ_DEFAULT = 50      # [policy default: docs/design.md, controller update rate]
SETTLE_S = 1.0                # [policy default: reset settles the suspension-less base and the arm servo]


@dataclass(frozen=True)
class ResolvedCommand:
    q_target_rad: np.ndarray
    modules: tuple[ModuleCommand, ...]
    gripper: GripperCmd
    twist_B: Twist
    ee_target_B: Pose | None
    level: int                 # 1 skill, 2 task, 3 joint


@dataclass(frozen=True)
class RunResult:
    ticks: int
    stopped: bool
    help_count: int
    interventions: int


class Runtime:
    def __init__(self, h: Handles, controller, *, control_hz: int = CONTROL_HZ_DEFAULT, seed: int = 0,
                 rc=None, episode=None, worksite=None):
        self.h, self.controller, self.rc, self.episode, self.worksite = h, controller, rc, episode, worksite
        self.robot = h.compiled.robot
        self.dt = TIMESTEP_S
        self.decim = int(round(1.0 / (control_hz * self.dt)))
        self.seed = seed
        self.kin = MujocoKinematics(h)
        self.ik = ResolvedRateIk(self.kin, np.array(self.robot.arm.home_q_rad), IkConfig())
        self.envelope = Envelope(self.robot, self.kin)
        self._env_active = False
        b = self.robot.base
        self.alloc = SwerveAllocator(tuple(m.xy_B_m for m in b.modules), b.modules[0].wheel_radius_m, b.steer_limit_rad)
        self.fw = ModuleFirmware(b.steer_rate_max_radps, self.dt)
        self.executor = None
        self._executor_name = None
        self.rng = np.random.default_rng(seed)

    # ---- lifecycle -------------------------------------------------------
    def reset(self):
        m, d, h = self.h.model, self.h.data, self.h
        mujoco.mj_resetData(m, d)
        d.qpos[h.base_qpos:h.base_qpos + 7] = [0, 0, self.robot.base.spawn_z_W_m, 1, 0, 0, 0]
        home = np.array(self.robot.arm.home_q_rad)
        d.qpos[h.arm_qpos] = home
        d.ctrl[h.arm_act] = home
        self.grip = gripper_strategy(self.robot)
        self.grip.apply(h, GripperCmd.OPEN)
        self.t_s, self.tick_i = 0.0, 0
        self.stopped, self.help_count, self.interventions = False, 0, 0
        self._help_prev, self._rc_active = False, False
        self.envelope.reset()
        self._env_active = False
        self.gripper_cmd, self.attached_id = GripperCmd.OPEN, None
        self.q_target = home.copy()
        self.ee_target_B = None
        self.module_targets = tuple(ModuleCommand(0.0, 0.0) for _ in h.modules)
        self.cmd = ResolvedCommand(home.copy(), self.module_targets, GripperCmd.OPEN, Twist(), None, 2)
        self.executor = None
        self._executor_name = None
        self.controller.reset(self.robot, self.seed)
        if self.worksite is not None:
            self.worksite.reset(h, self.rng)
        for _ in range(int(SETTLE_S / self.dt)):
            mujoco.mj_step(m, d)
        # The tool (gripper interface site) pose relative to the flange site, measured
        # once off the settled compiled model rather than trusted from any spec value --
        # same reasoning as envelope.py's arm-base measurement: the composer attaches the
        # gripper through a quaternion/frame chain (compose.py, gripper_magnet.py /
        # gripper_parallel.py) that nothing guarantees matches a hand-derived offset, and
        # "B" here means "the flange frame" (pose_W_to_B is frame-agnostic -- it just
        # needs a base pose and a pose to express relative to it).
        mujoco.mj_forward(m, d)
        tool_W = SEN.site_pose_W(h, h.interface_site)
        fl_W = SEN.site_pose_W(h, h.flange_site)
        self.tool_offset_F = SEN.pose_W_to_B(fl_W, tool_W)
        self.obs = self._observe()

    def _observe(self):
        phase = getattr(self.controller, "phase", None)
        oracle, tags, goal = (self.worksite.observe(self.h) if self.worksite is not None else (None, None, None))
        return SEN.observe(self.h, self.t_s, self.tick_i, gripper_cmd=self.gripper_cmd, phase=phase,
                           attached_id=self.attached_id, oracle=oracle, tags=tags, goal=goal,
                           envelope_active=self._env_active, tool_offset_F=self.tool_offset_F,
                           tool_kind=self.robot.gripper.kind)

    # ---- one physics tick --------------------------------------------------
    def tick(self):
        if self.tick_i % self.decim == 0:
            self.obs = self._observe()
            action = self._controller_action(self.obs)
            self.cmd = self._resolve(action, self.obs)
            self.q_target = self.cmd.q_target_rad
            self.gripper_cmd = self.cmd.gripper
            self.module_targets = self.cmd.modules
        slewed = self.fw.step(self.module_targets, np.array([self.h.data.qpos[mh.steer_qpos] for mh in self.h.modules]))
        self._apply(self.q_target, slewed, self.gripper_cmd)
        mujoco.mj_step(self.h.model, self.h.data)
        self.tick_i += 1
        self.t_s = self.tick_i * self.dt
        if self.worksite is not None:
            self.attached_id = self.grip.attachment(self.h, self.gripper_cmd, self.attached_id, self.worksite.rcu_bodies)
        if self.episode is not None:
            self.episode.append(self._observe(), self.cmd.level, self.cmd, self._env_active, self.interventions,
                                interventions_by_kind=self.envelope.counts)

    def run(self, duration_s: float | None = None) -> RunResult:
        if duration_s is not None:
            for _ in range(int(duration_s / self.dt)):
                self.tick()
        else:
            while not self.controller.done(self.obs):
                self.tick()
        return RunResult(self.tick_i, self.stopped, self.help_count, self.interventions)

    # ---- stop / help -------------------------------------------------------
    def request_stop(self):
        self.stopped = True

    def rearm(self):
        self.stopped = False

    def _controller_action(self, obs):
        """`help_count` counts RISING EDGES of the controller's `help` flag -- one intervention
        per help event, however many control ticks the flag stays raised (it used to count every
        tick the flag was set when there was no `rc`, but only once when there was one, because
        the same variable doubled as the hand-over latch). Handing control to `rc` on a help edge
        is a separate state, `_rc_active`. Once handed over the controller is no longer consulted,
        and returning control to it is undefined (not designed yet); no shipped path passes an
        `rc` (`launch.py`), so today the hand-over is reachable only from tests."""
        src = self.rc if (self._rc_active and self.rc is not None) else self.controller
        action = src.act(obs)
        if isinstance(action, ActionChunk):
            action = action.actions[0]      # consuming further chunk entries one per tick is not implemented yet
        if action.stop:
            self.stopped = True
        if src is self.controller:
            if action.help and not self._help_prev:
                self.help_count += 1
                self._rc_active = self.rc is not None
            self._help_prev = bool(action.help)
        return action

    # ---- resolution --------------------------------------------------------
    def _resolve(self, action, obs) -> ResolvedCommand:
        if self.stopped:
            return ResolvedCommand(obs.q_rad.copy(), tuple(ModuleCommand(m.steer_rad, 0.0) for m in obs.modules),
                                   self.gripper_cmd, Twist(), self.ee_target_B, 3)
        if isinstance(action, SkillAction):
            action = self._run_skill(action, obs)
        if isinstance(action, TaskAction):
            return self._resolve_task(action, obs)
        if isinstance(action, JointAction):
            q = obs.q_rad.copy() if action.q_target_rad is None else np.asarray(action.q_target_rad, dtype=float)
            q, rep = self.envelope.clamp_joint(q, obs)
            self._env_active = rep.active
            self.interventions = sum(self.envelope.counts.values())
            mods = self.module_targets if action.module_targets is None else tuple(ModuleCommand(s, w) for s, w in action.module_targets)
            g = self.gripper_cmd if action.gripper is GripperCmd.HOLD else action.gripper
            return ResolvedCommand(q, mods, g, Twist(), None, 3)
        raise TypeError(type(action))

    def _run_skill(self, action: SkillAction, obs):
        if self.executor is None or self._executor_name != action.name:
            self.executor = SK.REGISTRY[action.name]()
            self._executor_name = action.name
            self.executor.start(action.params, obs)
        out = self.executor.act(obs)
        if self.executor.done(obs):
            if action.name == "verify" and self.worksite is not None:
                self.worksite.mark_filled(action.params["target"])
            self.executor = None
            if hasattr(self.controller, "advance"):
                self.controller.advance()
        return out

    def _resolve_task(self, a: TaskAction, obs) -> ResolvedCommand:
        # `self.ee_target_B` is the controller's INTENDED standing target and only a
        # controller's own TaskAction ever writes it. The envelope FILTERS it: what
        # `clamp_task` returns (`ee_cmd_B`) goes to IK for THIS control tick only and is
        # never stored back. Writing it back (the original `self.ee_target_B, tw, rep =
        # clamp_task(self.ee_target_B, ...)`) made the shield a one-way ratchet: a force
        # freeze's 5 mm/tick back-off, or a reach/floor clamp applied while the base was
        # still far from its stance, became the standing command and outlived the hazard
        # [measured: 2026-09-21, DECISIONS.md D15 -- the carried unit was dragged along the
        # ground for ~2 m in `transport`].
        if a.ee_target is not None:
            self.ee_target_B = a.ee_target if a.ee_frame == "B" else SEN.pose_W_to_B(obs.base_pose_W, a.ee_target)
        ee_cmd_B, tw, rep = self.envelope.clamp_task(self.ee_target_B, a.base_twist_B, obs)
        self._env_active = rep.active
        self.interventions = sum(self.envelope.counts.values())
        if ee_cmd_B is not None:
            step = self.ik.step(self.q_target, ee_cmd_B, self.decim * self.dt)
            q = step.q_next
        else:
            q = self.q_target
        steer_now = np.array([m.steer_rad for m in obs.modules])
        mods = self.alloc.allocate(tw, steer_now)
        g = self.gripper_cmd if a.gripper is GripperCmd.HOLD else a.gripper
        return ResolvedCommand(q, mods, g, tw, ee_cmd_B, 2)

    def _apply(self, q_target, mods, gripper: GripperCmd):
        d, h = self.h.data, self.h
        d.ctrl[h.arm_act] = q_target
        for mh, c in zip(h.modules, mods):
            d.ctrl[mh.steer_act] = c.steer_rad
            d.ctrl[mh.drive_act] = mh.drive_sign * c.wheel_radps    # see handles.ModuleHandles.drive_sign
        self.grip.apply(h, gripper)

    # ---- outputs -----------------------------------------------------------
    def hud_snapshot(self, controller_name: str, rtf: float, selected_joint: int, mode: str) -> HudSnapshot:
        o = self.obs
        return HudSnapshot(mode=mode, controller=controller_name, phase=o.phase, twist_cmd_B=self.cmd.twist_B,
                           twist_meas_B=o.base_twist_B, modules=o.modules, q_rad=o.q_rad,
                           ee_pos_B_m=o.ee_pose_B.pos_m, ee_pos_W_m=o.ee_pose_W.pos_m,
                           ee_target_B_m=None if self.ee_target_B is None else self.ee_target_B.pos_m,
                           gripper=o.gripper, force_n=float(np.linalg.norm(o.ft_flange_B[:3])),
                           force_budget_n=self.robot.force_budget_n, interventions=self.interventions,
                           rtf=rtf, stopped=self.stopped, selected_joint=selected_joint)

    def save(self, run_dir: Path, extra_meta: dict | None = None) -> Path:
        meta = {"seed": self.seed, "surface": self.h.compiled.surface.kind, "controller": self.controller.name,
                "ticks": self.tick_i, "dt_s": self.dt, "control_hz": 1.0 / (self.decim * self.dt),
                "git": _git_rev(), "stops": int(self.stopped), "helps": self.help_count, **(extra_meta or {})}
        if self.worksite is not None:
            ws = self.worksite.spec
            meta["unit_ids"] = [r.id for r in ws.staged]
            meta["slots"] = [{"id": s.id, "xyz_W_m": list(s.xyz_W_m), "yaw_rad": s.yaw_rad} for s in ws.slots]
            meta["to_place"] = list(ws.to_place)
        return self.episode.save(run_dir, meta)


def _git_rev() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"
