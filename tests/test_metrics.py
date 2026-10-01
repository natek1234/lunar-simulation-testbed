"""PlacementMetrics computed from a real recorded episode, plus the pass/fail judge in isolation."""
import math

import numpy as np

from testbed.bench import metrics as MET


def test_placement_metrics_from_a_recorded_episode(placement_run):
    # The shared `placement_run` fixture records the WHOLE run (EpisodeLog attached before reset()),
    # saves it and loads it back, so `placement_metrics` reads exactly what a benchmark reviewer would.
    m = MET.placement_metrics(placement_run.arrays, placement_run.meta)
    assert set(m.phase_s) >= {"approach", "acquire", "seat", "release"}
    assert m.cycle_s > 0
    assert m.peak_force_n > 0 and math.isfinite(m.pos_err_m)
    # This scenario's scripted script completes and releases cleanly on this seed
    # (tests/test_single_placement.py's own gate) -- pin the SAME thing metrics.py independently measures.
    assert m.success
    assert m.attach_confirmed and m.release_confirmed
    assert 0.0 <= m.wheel_contact_frac <= 1.0
    assert m.stops == 0
    # The phases partition the run: their durations cannot sum past the cycle (they did, 107 s vs 67 s,
    # when a repeated phase name was timed first-visit-start to last-visit-end).
    assert sum(m.phase_s.values()) <= m.cycle_s + 1e-9
    assert m.phase_s["park"] < 5.0      # `park` runs twice, ~1 s and ~0.6 s; the window between the visits is ~58 s
    assert math.isfinite(m.carried_min_z_m) and m.force_interventions >= m.carry_force_interventions >= 0


def test_success_requires_both_tolerances():
    # Straddled against the module's own constants, not against literals: a literal that sits
    # outside today's tolerance silently moves INSIDE a widened one, and the `not judge(...)`
    # assertion then passes for the wrong reason.
    base = dict(pos_err_m=0.005, yaw_err_rad=math.radians(1), z_ok=True)
    assert MET.judge(**base)
    assert MET.judge(**{**base, "pos_err_m": MET.EPS_P_M})                      # the bound is inclusive
    assert MET.judge(**{**base, "yaw_err_rad": MET.EPS_YAW_RAD})
    assert not MET.judge(**{**base, "pos_err_m": MET.EPS_P_M * 1.01})           # position alone fails it
    assert not MET.judge(**{**base, "yaw_err_rad": MET.EPS_YAW_RAD * 1.01})     # yaw alone fails it
    assert not MET.judge(**{**base, "z_ok": False})


def test_phase_durations_sum_each_visit_of_a_repeated_phase_not_the_window_between_them():
    """Synthetic episode: `park` (index 1) runs twice with `acquire` (index 2) between them."""
    t = np.arange(10) * 1.0                                   # 0..9 s, one row per second
    ph = np.array([0, 0, 1, 1, 2, 2, 2, 1, 1, 3])             # approach, approach, park, park, acquire x3, park, park, verify
    phases = ["approach", "park", "acquire", "verify"]
    d = MET.phase_durations_s(t, ph, phases)
    assert d == {"approach": 1.0, "park": 1.0 + 1.0, "acquire": 2.0, "verify": 0.0}
    assert sum(d.values()) <= t[-1] - t[0]
    # the old first-start-to-last-end rule would have said park = 6 s
    assert (t[ph == 1].max() - t[ph == 1].min()) == 6.0


def test_phase_durations_skip_the_unnamed_phase_and_handle_empty():
    t = np.arange(4) * 1.0
    assert MET.phase_durations_s(t, np.array([0, 0, 1, 1]), ["", "seat"]) == {"seat": 1.0}
    assert MET.phase_durations_s(np.array([]), np.array([], dtype=int), ["a"]) == {}
