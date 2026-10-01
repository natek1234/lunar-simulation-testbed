import numpy as np
import pytest

from testbed.control.controller import ScriptedSequencer
from testbed.sim import compose as C, handles as H, runtime as RT
from testbed.spec import robot as R, surface as S

DEMO = (("hold", {"duration_s": 0.2}), ("drive", {"vx_mps": 0.3, "duration_s": 2.0}), ("jog", {"dpos_B_m": [0, 0, 0.1], "duration_s": 1.0}))


def make(seed=0):
    h = H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml")))
    rt = RT.Runtime(h, ScriptedSequencer(DEMO), seed=seed)
    rt.reset()
    return rt


def test_reset_leaves_the_robot_at_rest_at_home():
    rt = make()
    assert abs(rt.obs.base_twist_B.vx_mps) < 1e-3
    assert np.allclose(rt.obs.q_rad, rt.h.compiled.robot.arm.home_q_rad, atol=0.02)


def test_scripted_drive_moves_the_base_forward_and_jog_lifts_the_flange():
    rt = make()
    z0 = rt.obs.ee_pose_W.pos_m[2]
    res = rt.run(3.3)
    assert res.ticks == int(3.3 / 0.002)
    assert rt.obs.base_pose_W.pos_m[0] > 0.4          # ~0.3 m/s for 2 s minus ramp
    assert rt.obs.ee_pose_W.pos_m[2] - z0 > 0.05


def test_same_seed_is_bit_identical():
    """Proves the run is DETERMINISTIC (the same construction twice gives bit-identical state). It
    cannot detect seed sensitivity: nothing in `testbed/` draws from the seeded `rng` yet (`Worksite.reset`
    accepts it and never uses it), so seed 0 and seed 3 episodes are identical too. When condition
    sampling (not implemented yet) makes the seed matter, add a "different seeds differ" test beside this one."""
    a, b = make(3), make(3)
    a.run(1.0); b.run(1.0)
    assert np.array_equal(a.h.data.qpos, b.h.data.qpos)


def test_stop_latches_until_rearmed():
    rt = make()
    rt.run(0.5)
    rt.request_stop()
    rt.run(0.5)
    assert rt.stopped and abs(rt.obs.base_twist_B.vx_mps) < 0.02
    rt.rearm()
    assert not rt.stopped


def test_episode_records_every_tick(tmp_path):
    from testbed.bench.episode import EpisodeLog, load_episode
    h = H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml")))
    rt = RT.Runtime(h, ScriptedSequencer(DEMO), episode=EpisodeLog(h.compiled.robot))
    rt.reset(); rt.run(0.1)
    run_dir = rt.save(tmp_path / "run")
    arrays, meta = load_episode(run_dir)
    assert arrays["t_s"].shape == (50,) and arrays["q_rad"].shape == (50, 7) and arrays["ft_B"].shape == (50, 6)
    assert meta["robot"] == "fr3_mpo700" and meta["schema"] == 3


class _HelpPattern:
    """A controller that raises `help` on the control ticks named in `raised` (0-based), holds otherwise."""
    name = "help_pattern"

    def __init__(self, raised):
        self.raised, self.n = set(raised), 0

    def reset(self, robot, seed):
        self.n = 0

    def act(self, obs):
        from testbed.control.records import TaskAction
        a = TaskAction(help=self.n in self.raised)
        self.n += 1
        return a

    def done(self, obs):
        return False


def test_help_counts_rising_edges_not_ticks():
    """`help_count` is the declared "helps" metric: one per help EVENT. A flag held for 5 control
    ticks is one help, a second later burst another (it used to count 5 + 2 = 7 when no rc was attached)."""
    rt = make()
    rt.controller = _HelpPattern(raised={2, 3, 4, 5, 6, 10, 11})
    rt.controller.reset(rt.robot, 0)
    rt.run(0.4)                                    # 20 control ticks at 50 Hz
    assert rt.help_count == 2


def test_help_hands_control_to_rc_once_and_counts_one_event_regardless():
    from testbed.control.records import TaskAction, Twist

    class Rc:
        name = "rc"

        def __init__(self):
            self.calls = 0

        def act(self, obs):
            self.calls += 1
            return TaskAction(base_twist_B=Twist(vx_mps=0.1))

    rt = make()
    rt.rc = Rc()
    rt.controller = _HelpPattern(raised=set(range(3, 100)))       # the flag stays up forever
    rt.controller.reset(rt.robot, 0)
    rt.run(0.4)
    assert rt.help_count == 1                       # one event; the latch and the counter are separate
    assert rt.rc.calls > 10 and rt.controller.n == 4   # the controller was consulted through the raising tick only
    rt.reset()                                     # a fresh episode clears the hand-over
    assert rt.help_count == 0 and rt._rc_active is False
