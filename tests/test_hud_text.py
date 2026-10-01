from testbed.viz.hud_text import render
from tests.test_runtime import make


def test_hud_columns_align_and_carry_the_numbers():
    rt = make()
    left, right = render(rt.hud_snapshot("scripted", 1.0, 0, "task"))
    assert left.count("\n") == right.count("\n")
    assert "RTF" in left and "1.00" in right
