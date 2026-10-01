"""Acceptance: the scripted single-placement episode places rcu_0 on the foundation on the flat surface."""
import numpy as np
import pytest

from testbed.bench import metrics as MET
from testbed.bench.episode import EpisodeLog
from testbed.control import quat as Q
from testbed.control.controller import ScriptedSequencer
from testbed.sim import compose as C, handles as H, runtime as RT, worksite as WS
from testbed.spec import robot as R, scenario as SC, surface as S


# --- gates (DECISIONS.md D15/D16); provenance in each tag ---
POS_GATE_M = 0.001         # [policy default: numerical-noise floor -- measured 0.076 mm (2x = 0.15 mm), floored at 1 mm for cross-platform float noise, DECISIONS.md D16]
YAW_GATE_DEG = 0.1         # [policy default: numerical-noise floor -- measured 0.0086 deg, DECISIONS.md D16]
TILT_GATE_DEG = 0.5        # [policy default: numerical-noise floor -- measured 0.0104 deg; the ~4 deg tilt this replaces was a disturbed-foundation artefact, DECISIONS.md D16]
CARRY_CLEARANCE_M = 0.09   # [derived: about half the measured carry lift -- min carried z 0.2398 m over the 0.05 m rest height, 0.190 m, DECISIONS.md D15]
CARRY_FORCE_INTERVENTIONS_MAX = 0   # [measured: 0 force interventions over the whole carry window (via approach + transport + park + place), magnet robot, 2026-09-21 -- carrying must not trip the force clamp]
FOUNDATION_TOL_M = 0.010   # [ported: docs/reference/requirements.md, placed-unit disturbance limit <= 10 mm]


def run_placement(seed=0, record=False):
    """Tick until the script itself reports done, capped at `sc.timeout_s` as a hang guard
    -- NOT `rt.run(sc.timeout_s)` in one call. `Runtime.run(duration_s)` with an explicit
    duration always executes the FULL duration regardless of `controller.done()` (its own
    docstring), so running it would always report the full timeout as the cycle time instead
    of when the script actually finished; `sc.timeout_s` (285 s) is a conservative 1.5x safety
    margin over the cycle-time budget (docs/reference/requirements.md, 190 s), and this
    choreography completes in ~81 s.

    (An earlier version of this docstring, and D13, said the seated unit slides and topples
    by t~210 s if left idle, blamed on the slot sitting on the seam between the two
    foundation blocks. That was measured with a foundation the chassis had already knocked
    off its blueprint; with the foundation undisturbed (DECISIONS.md D16) the unit stays flat
    (tilt 0.01 deg) and still on the slot (0.08 -> 0.19 mm creep) through the full 285 s.)

    `record=True` attaches an `EpisodeLog` before `reset()` so the whole run (not just a
    re-attached tail) lands in the episode -- `tests/test_metrics.py` wants a
    real recorded episode to compute `testbed.bench.metrics.placement_metrics` from."""
    sc = SC.load_scenario(SC.SCENARIOS_DIR / "single_placement.yaml")
    ws = WS.Worksite(sc.worksite)
    h = H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml"), ws.spec))
    rt = RT.Runtime(h, ScriptedSequencer(sc.script), seed=seed, worksite=ws, episode=EpisodeLog(h.compiled.robot) if record else None)
    rt.reset()
    while not rt.controller.done(rt.obs) and rt.t_s < sc.timeout_s:
        rt.tick()
    return rt, ws, sc


def test_scripted_places_the_unit_on_the_slot(placement_run):
    rt, ws, sc = placement_run.rt, placement_run.ws, placement_run.sc
    assert rt.controller.done(rt.obs), f"script stalled in phase {rt.obs.phase} at t={rt.t_s:.1f}s"
    unit = ws.rcu_pose_W(rt.h, "rcu_0")
    slot = ws.slot_pose_W("slot_0")
    pos_err = np.linalg.norm(unit.pos_m[:2] - slot.pos_m[:2])
    yaw_err = abs((Q.yaw_rad(unit.quat_wxyz) - Q.yaw_rad(slot.quat_wxyz) + np.pi) % (2 * np.pi) - np.pi)
    print(f"single placement, flat: cycle_t={rt.t_s:.1f}s pos_err={pos_err*1000:.1f}mm yaw_err={np.degrees(yaw_err):.2f}deg "
          f"z={unit.pos_m[2]:.3f} interventions={rt.interventions}")
    assert abs(unit.pos_m[2] - slot.pos_m[2]) < 0.02          # sitting on the foundation, not dropped
    # Gates against the row recorded 2026-09-21 after D15/D16 (payload carried, foundation
    # undisturbed): pos_err 0.076 mm, yaw_err 0.0086 deg, tilt 0.0104 deg. 2x those would sit below the
    # float noise a contact-rich 81 s run can carry across platforms (CI runs ubuntu AND windows), so
    # each gate is floored at a stated noise floor; all three are still 10x+ tighter than the
    # placement position/orientation tolerances (docs/reference/requirements.md, 25 mm / 6 deg)
    # and than the old 15 mm / 1 deg gate.
    assert pos_err < POS_GATE_M and yaw_err < np.radians(YAW_GATE_DEG)
    up = Q.rotate(unit.quat_wxyz, [0, 0, 1])
    assert np.degrees(np.arccos(np.clip(up[2], -1, 1))) < TILT_GATE_DEG, "seated unit is resting tilted"
    assert not rt.stopped and rt.obs.gripper.attached is False
    assert rt.t_s < sc.timeout_s, f"script did not complete within the {sc.timeout_s}s timeout"


# --- gates for the class of defect DECISIONS.md D15 fixed (the run "passed" while dragging the
# payload along the ground, because the only assertions were on the placed unit's FINAL pose) ---
def test_carried_unit_clears_the_ground_during_transport(placement_run):
    """The unit is held in the air the whole time it is carried (the carry window: after `acquire`
    ends, before `seat` begins -- the via `approach` leg, `transport`, `park`, `place`). Regression for the D15 defect:
    the envelope's force-freeze back-off was written back into the standing EE target and
    ratcheted the carried unit down to z=0.069 m (rest height 0.05 m) -- a drag along the ground."""
    m = MET.placement_metrics(placement_run.arrays, placement_run.meta)
    rest_z = next(r.xyz_W_m[2] for r in placement_run.sc.worksite.staged if r.id == "rcu_0")
    assert m.attach_confirmed
    assert m.carried_min_z_m >= rest_z + CARRY_CLEARANCE_M, f"carried unit sank to z={m.carried_min_z_m:.4f} m"


def test_carrying_does_not_trip_the_force_clamp(placement_run):
    m = MET.placement_metrics(placement_run.arrays, placement_run.meta)
    assert m.carry_force_interventions <= CARRY_FORCE_INTERVENTIONS_MAX


def test_does_not_disturb_the_placed_foundation(placement_run):
    m = MET.placement_metrics(placement_run.arrays, placement_run.meta)
    assert m.foundation_disp_m <= FOUNDATION_TOL_M, f"foundation displaced {m.foundation_disp_m*1000:.1f} mm"
