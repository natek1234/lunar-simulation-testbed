"""Placement metrics from a recorded episode (never from commands, never from live sim state --
`placement_metrics` only ever reads `arrays`/`meta` off disk, the same load path `app.py bench`
uses). Tolerances default to the placement position/orientation tolerances
(`docs/reference/requirements.md`): 25 mm, 6 deg.

**Peak-force resolution.** `Runtime.tick()` calls `mujoco.mj_step()` once per physics tick
(500 Hz) but only refreshes the CONTROLLER-facing `self.obs` (and anything read off it,
e.g. `rt.obs.ft_flange_B`) once every `decim=10` ticks (50 Hz) -- this under-resolved an
early hand measurement of this scenario's peak force (124.2 N vs the true 411.4 N,
`docs/benchmarks/single_placement.md`). The EPISODE LOG does not share that limitation:
`Runtime.tick()`'s call to `self.episode.append(self._observe(), ...)` computes a FRESH
`Observation` after every `mujoco.mj_step()`, entirely decoupled from the decimated
`self.obs` the controller sees -- confirmed empirically (2026-09-21): re-running
`tests/test_single_placement.py::run_placement(record=True)` and reading `arrays["ft_B"]` back from the saved
episode reproduces the exact same 411.44 N peak at the exact same tick (9632) the earlier
hand measurement found by manual physics-tick sampling, and `tests/test_runtime.py::
test_episode_records_every_tick` already pins the row count at one-per-PHYSICS-tick (50
rows for `rt.run(0.1)` at `dt=0.002 s`, not 5 for the 50 Hz control rate). So `peak_force_n`
below is already computed at the correct (physics-tick) resolution -- no undersampling
caveat needed for a metric computed from `arrays["ft_B"]`."""
import math
from dataclasses import dataclass

import numpy as np

from testbed.control import quat as Q

EPS_P_M = 0.025                    # [ported: docs/reference/requirements.md, placement position tolerance]
EPS_YAW_RAD = math.radians(6.0)    # [ported: docs/reference/requirements.md, placement orientation tolerance]
Z_TOL_M = 0.02                     # [policy default: seated on the course, not dropped beside it]
SEAT_WINDOW_S = 0.5                # [policy default: force averaged over the last 0.5 s of `seat`]


@dataclass(frozen=True)
class PlacementMetrics:
    success: bool
    pos_err_m: float
    yaw_err_rad: float
    tilt_err_rad: float
    cycle_s: float
    phase_s: dict
    peak_force_n: float
    seat_force_n: float
    attach_confirmed: bool
    release_confirmed: bool
    foundation_disp_m: float
    interventions: int
    force_interventions: int             # cumulative `force`-kind envelope interventions over the whole run
    carry_force_interventions: int       # ... of which occurred in the CARRY window (a carried payload should not trip the force clamp)
    carried_min_z_m: float               # min world z of the placed unit while ATTACHED in the carry window (nan if it was never carried)
    stops: int
    helps: int
    wheel_contact_frac: float


def judge(pos_err_m, yaw_err_rad, z_ok, eps_p_m=EPS_P_M, eps_yaw_rad=EPS_YAW_RAD) -> bool:
    return bool(z_ok and pos_err_m <= eps_p_m and yaw_err_rad <= eps_yaw_rad)


def phase_durations_s(t: np.ndarray, ph: np.ndarray, phases: list) -> dict:
    """Seconds spent in each named phase: the SUM of each contiguous VISIT's own duration.
    `EpisodeLog` gives one index per phase NAME, and a script may visit a name more than once
    (the scenario runs `park` twice, and now `approach` twice); taking `t.max() - t.min()` over every
    row with that index -- what this used to do -- times the whole window from the first visit's
    start to the last visit's end and swallows every phase in between (`park` "took" 41.8 s,
    and the phase sum was 107 s against a 67 s cycle)."""
    out: dict = {}
    if len(t) == 0:
        return out
    edges = np.flatnonzero(np.diff(ph) != 0) + 1
    starts = np.concatenate(([0], edges)); ends = np.concatenate((edges, [len(t)])) - 1
    for a, b in zip(starts, ends):
        name = phases[int(ph[a])]
        if name:
            out[name] = out.get(name, 0.0) + float(t[b] - t[a])
    return out


def placement_metrics(arrays: dict, meta: dict, *, eps_p_m=EPS_P_M, eps_yaw_rad=EPS_YAW_RAD) -> PlacementMetrics:
    t = arrays["t_s"]
    ids = list(meta["unit_ids"])
    k = ids.index(meta["to_place"][0])
    slot = meta["slots"][0]
    slot_p = np.asarray(slot["xyz_W_m"]); slot_q = Q.from_rotvec(np.array([0, 0, slot["yaw_rad"]]))
    p_end = arrays["unit_pos_W"][-1, k]; q_end = arrays["unit_quat_W"][-1, k]
    pos_err = float(np.linalg.norm(p_end[:2] - slot_p[:2]))
    yaw_err = abs(float((Q.yaw_rad(q_end) - Q.yaw_rad(slot_q) + np.pi) % (2 * np.pi) - np.pi))
    up = Q.rotate(q_end, [0, 0, 1]); tilt = float(np.arccos(np.clip(up[2], -1, 1)))
    z_ok = abs(p_end[2] - slot_p[2]) < Z_TOL_M
    phases = meta["phases"]; ph = arrays["phase"]
    phase_s = phase_durations_s(t, ph, phases)
    cycle_s = float(t[-1] - t[0])
    fmag = np.linalg.norm(arrays["ft_B"][:, :3], axis=1)
    seat_i = phases.index("seat") if "seat" in phases else None
    if seat_i is not None and (ph == seat_i).any():
        ts = t[ph == seat_i]; win = (ph == seat_i) & (t >= ts.max() - SEAT_WINDOW_S)
        seat_force = float(fmag[win].mean())
    else:
        seat_force = float("nan")
    att = arrays["attached"].astype(bool)
    # The CARRY window: the rows after `acquire` ends and before `seat` begins -- the unit is
    # airborne on the arm (whatever the script calls the legs in between: `approach`/`transport`/
    # `park`/`place`). Acquire's lift-off and seat's descent are excluded on purpose: the unit
    # legitimately starts on the ground and ends on the course.
    carry_window = np.zeros(len(t), dtype=bool)
    if "acquire" in phases and "seat" in phases:
        acq_rows, seat_rows = np.flatnonzero(ph == phases.index("acquire")), np.flatnonzero(ph == phases.index("seat"))
        if acq_rows.size and seat_rows.size:
            carry_window[acq_rows[-1] + 1:seat_rows[0]] = True
    kinds = list(meta.get("intervention_kinds", ()))
    if "force" in kinds and "interventions_by_kind" in arrays:      # schema >= 3; older episodes carry no per-kind column
        force_cum = arrays["interventions_by_kind"][:, kinds.index("force")]
        force_total = int(force_cum[-1])
        idx = np.flatnonzero(carry_window)
        carry_force = int(force_cum[idx[-1]] - force_cum[idx[0] - 1]) if idx.size else 0
    else:
        force_total = carry_force = 0
    carried = carry_window & att
    carried_min_z = float(arrays["unit_pos_W"][carried, k, 2].min()) if carried.any() else float("nan")
    attach_confirmed = bool(att.any())
    release_confirmed = bool(att.any() and not att[-1] and z_ok)
    others = [i for i in range(len(ids)) if i != k]
    disp = float(max((np.linalg.norm(arrays["unit_pos_W"][-1, i] - arrays["unit_pos_W"][0, i]) for i in others), default=0.0))
    contact = arrays["wheel_contact"] if "wheel_contact" in arrays else np.ones((len(t), 1))
    return PlacementMetrics(success=judge(pos_err, yaw_err, z_ok, eps_p_m, eps_yaw_rad), pos_err_m=pos_err, yaw_err_rad=yaw_err,
                     tilt_err_rad=tilt, cycle_s=cycle_s, phase_s=phase_s, peak_force_n=float(fmag.max()),
                     seat_force_n=seat_force, attach_confirmed=attach_confirmed, release_confirmed=release_confirmed,
                     foundation_disp_m=disp, interventions=int(arrays["interventions"][-1]),
                     force_interventions=force_total, carry_force_interventions=carry_force,
                     carried_min_z_m=carried_min_z,
                     stops=int(meta.get("stops", 0)), helps=int(meta.get("helps", 0)),
                     wheel_contact_frac=float(contact.mean()))
