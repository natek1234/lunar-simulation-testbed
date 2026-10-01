"""Per-tick episode recording: episode.npz (columns) + meta.json. Never imports mujoco --
it only ever sees the Observation/ResolvedCommand records the runtime hands it."""
import json
from pathlib import Path

import numpy as np

from testbed.control.envelope import KINDS as INTERVENTION_KINDS
from testbed.control.records import GripperCmd, Observation

EPISODE_FILE = "episode.npz"
META_FILE = "meta.json"
SCHEMA = 3  # [policy default: episode file format version, bumped on any column/meaning change --
            # 2 adds unit_pos_W/unit_quat_W/wheel_contact (placement metrics); 3 adds
            # interventions_by_kind (force vs twist interventions were
            # indistinguishable in the single cumulative `interventions` column)]
GRIPPER_CODE = {GripperCmd.OPEN: 0, GripperCmd.CLOSE: 1, GripperCmd.HOLD: 2}  # [policy default: stable int codes for the npz column]


class EpisodeLog:
    def __init__(self, robot_spec):
        self.robot = robot_spec
        self.cols: dict[str, list] = {}
        self.phases: list[str] = []

    def _put(self, k, v):
        self.cols.setdefault(k, []).append(v)

    def append(self, obs: Observation, level: int, cmd, envelope_active: bool, interventions: int,
               interventions_by_kind: dict | None = None) -> None:
        self._put("t_s", obs.t_s); self._put("tick", obs.tick)
        self._put("base_pos_W", obs.base_pose_W.pos_m); self._put("base_quat_W", obs.base_pose_W.quat_wxyz)
        self._put("base_twist_B", [obs.base_twist_B.vx_mps, obs.base_twist_B.vy_mps, obs.base_twist_B.wz_radps])
        self._put("q_rad", obs.q_rad); self._put("qd_radps", obs.qd_radps); self._put("tau_nm", obs.tau_nm)
        self._put("steer_rad", [m.steer_rad for m in obs.modules]); self._put("wheel_radps", [m.wheel_radps for m in obs.modules])
        self._put("gripper_aperture_m", obs.gripper.aperture_m if obs.gripper.aperture_m is not None else np.nan)
        self._put("attached", int(obs.gripper.attached))
        self._put("ft_B", obs.ft_flange_B)
        self._put("ee_pos_W", obs.ee_pose_W.pos_m); self._put("ee_quat_W", obs.ee_pose_W.quat_wxyz); self._put("ee_pos_B", obs.ee_pose_B.pos_m)
        self._put("cmd_twist_B", [cmd.twist_B.vx_mps, cmd.twist_B.vy_mps, cmd.twist_B.wz_radps])
        self._put("cmd_q_rad", cmd.q_target_rad)
        self._put("cmd_modules", [[c.steer_rad, c.wheel_radps] for c in cmd.modules])
        self._put("cmd_gripper", GRIPPER_CODE[cmd.gripper]); self._put("level", level)
        self._put("envelope_active", int(envelope_active)); self._put("interventions", interventions)
        by_kind = interventions_by_kind or {}
        self._put("interventions_by_kind", [int(by_kind.get(k, 0)) for k in INTERVENTION_KINDS])
        phase = obs.phase or ""
        if phase not in self.phases:
            self.phases.append(phase)
        self._put("phase", self.phases.index(phase))
        if obs.oracle is not None:
            self._put("unit_pos_W", [o.pose_W.pos_m for o in obs.oracle])
            self._put("unit_quat_W", [o.pose_W.quat_wxyz for o in obs.oracle])
        if obs.wheel_contact is not None:
            self._put("wheel_contact", obs.wheel_contact)

    def save(self, run_dir: Path, meta: dict) -> Path:
        run_dir = Path(run_dir); run_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(run_dir / EPISODE_FILE, **{k: np.asarray(v) for k, v in self.cols.items()})
        (run_dir / META_FILE).write_text(json.dumps({**meta, "schema": SCHEMA, "robot": self.robot.name,
                                                      "phases": self.phases, "intervention_kinds": list(INTERVENTION_KINDS)},
                                                     indent=1), encoding="utf-8")
        return run_dir


def load_episode(run_dir: Path):
    run_dir = Path(run_dir)
    with np.load(run_dir / EPISODE_FILE) as z:
        arrays = {k: z[k] for k in z.files}
    return arrays, json.loads((run_dir / META_FILE).read_text(encoding="utf-8"))
