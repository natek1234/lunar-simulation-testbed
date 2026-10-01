import mujoco
import numpy as np
import pytest

from testbed.control.records import GripperCmd, Pose
from testbed.sim import compose as C, handles as H, runtime as RT, worksite as WS
from testbed.spec import robot as R, scenario as SC, surface as S


@pytest.fixture(scope="module")
def scenario():
    return SC.load_scenario(SC.SCENARIOS_DIR / "single_placement.yaml")


@pytest.fixture
def rt(scenario):
    ws = WS.Worksite(scenario.worksite)
    h = H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml"), ws.spec))
    from testbed.control.controller import ScriptedSequencer
    r = RT.Runtime(h, ScriptedSequencer(()), worksite=ws)
    r.reset()
    return r


def test_has_one_unit_to_place_a_foundation_and_one_slot(scenario):
    ws = scenario.worksite
    assert ws.to_place == ("rcu_0",) and len(ws.staged) == 3 and len(ws.slots) == 1 and ws.pallets == ()
    assert ws.rcu.mass_kg == 22.0


def test_units_rest_where_staged_and_are_observable(rt):
    o = rt.obs
    by_id = {r.id: r for r in o.oracle}
    assert abs(by_id["rcu_0"].pose_W.pos_m[2] - 0.05) < 0.01        # on the ground
    assert abs(by_id["found_1"].pose_W.pos_m[2] - 0.05) < 0.01
    assert o.tags == ()                                             # nothing within read range at spawn
    assert o.goal.target_slot == 0 and o.goal.filled == (False,) and o.goal.slot_ids == ("slot_0",)
    assert rt.h.model.site("rcu_0_patch").id >= 0                   # the ferromagnetic patch on the top face


def test_rcu_mass_is_22kg(rt):
    m = rt.h.model
    assert abs(m.body_mass[m.body("rcu_0").id] - 22.0) < 0.05


def _lift_controller(rt, dz):
    from testbed.control.records import TaskAction
    target = Pose(rt.obs.ee_pose_B.pos_m + np.array([0, 0, dz]), rt.obs.ee_pose_B.quat_wxyz)
    class Lift:
        name = "lift"
        def reset(self, r, s): pass
        def act(self, obs): return TaskAction(ee_target=target, gripper=GripperCmd.CLOSE)
        def done(self, obs): return False
    return Lift()


def test_energised_magnet_on_the_patch_captures_holds_lifts_and_breaks_away(rt):
    """The magnet gate: put the unit's patch against the magnet face, energise, lift 0.25 m
    by IK and keep it; then confirm the breakaway releases above hold_force_n."""
    h, ws = rt.h, rt.worksite
    m, d = h.model, h.data
    ws.seat_in_gripper(h, "rcu_0")            # helper: patch centre at the magnet face, normals opposed
    rt.gripper_cmd = GripperCmd.CLOSE
    rt.controller = _lift_controller(rt, 0.0)
    rt.run(0.2)
    assert rt.attached_id == "rcu_0"
    assert d.eq_active[m.equality("magnet_weld_rcu_0").id] == 1
    z0 = ws.rcu_pose_W(h, "rcu_0").pos_m[2]
    rt.controller = _lift_controller(rt, 0.25)
    rt.run(4.0)
    z1 = ws.rcu_pose_W(h, "rcu_0").pos_m[2]
    assert z1 - z0 > 0.20, f"unit lifted only {z1 - z0:.3f} m"
    print(f"magnet holding 22 kg at lunar g: |F| = {np.linalg.norm(rt.obs.ft_flange_B[:3]):.1f} N (weight 35.6 N)")
    # breakaway: pin the unit to the world and pull up -> force exceeds hold_force_n -> released
    adr = m.jnt_dofadr[m.joint("rcu_0_free").id]
    pin = m.equality("magnet_weld_rcu_0").id
    rt.controller = _lift_controller(rt, 0.5)
    d.qvel[adr:adr + 6] = 0
    m.body_mass[m.body("rcu_0").id] = 1000.0           # crude anchor: the arm cannot lift a tonne
    rt.run(3.0)
    m.body_mass[m.body("rcu_0").id] = 22.0
    assert rt.attached_id is None and d.eq_active[pin] == 0


def test_hand_variant_grasps_the_unit_by_its_end(scenario):
    """The parallel kind still ships: with the Franka hand, a unit seated between the pads
    (0.10 m face across the gap) is held and lifted. Decides gain_override for that file."""
    ws = WS.Worksite(scenario.worksite)
    h = H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700_hand.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml"), ws.spec))
    from testbed.control.controller import ScriptedSequencer
    rt = RT.Runtime(h, ScriptedSequencer(()), worksite=ws)
    rt.reset()
    ws.seat_in_gripper(h, "rcu_0")
    rt.gripper_cmd = GripperCmd.CLOSE
    rt.controller = _lift_controller(rt, 0.0)
    rt.run(1.0)
    assert rt.attached_id == "rcu_0"
    z0 = ws.rcu_pose_W(h, "rcu_0").pos_m[2]
    rt.controller = _lift_controller(rt, 0.25)
    rt.run(4.0)
    assert ws.rcu_pose_W(h, "rcu_0").pos_m[2] - z0 > 0.20
