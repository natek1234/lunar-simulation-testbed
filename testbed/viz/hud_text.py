"""HudSnapshot -> two aligned text columns for mjr_overlay (labels, values)."""
import numpy as np

from testbed.control.records import HudSnapshot


def render(s: HudSnapshot) -> tuple[str, str]:
    rows = [
        ("controller", f"{s.controller} [{s.mode}]" + ("  STOPPED (R to re-arm)" if s.stopped else "")),
        ("phase", s.phase or "-"),
        ("base cmd vx vy wz", f"{s.twist_cmd_B.vx_mps:+.2f} {s.twist_cmd_B.vy_mps:+.2f} {s.twist_cmd_B.wz_radps:+.2f}"),
        ("base meas vx vy wz", f"{s.twist_meas_B.vx_mps:+.2f} {s.twist_meas_B.vy_mps:+.2f} {s.twist_meas_B.wz_radps:+.2f}"),
        ("modules steer deg", " ".join(f"{np.degrees(m.steer_rad):+5.0f}" for m in s.modules)),
        ("modules wheel rad/s", " ".join(f"{m.wheel_radps:+5.1f}" for m in s.modules)),
        ("arm q rad", " ".join(("*" if i == s.selected_joint else " ") + f"{q:+.2f}" for i, q in enumerate(s.q_rad))),
        ("ee pos B m", " ".join(f"{v:+.3f}" for v in s.ee_pos_B_m)),
        ("ee target B m", "-" if s.ee_target_B_m is None else " ".join(f"{v:+.3f}" for v in s.ee_target_B_m)),
        ("ee pos W m", " ".join(f"{v:+.3f}" for v in s.ee_pos_W_m)),
        ("gripper", f"{s.gripper.cmd.value}" + (f" aperture {s.gripper.aperture_m*1000:.0f} mm" if s.gripper.aperture_m is not None else "")
                    + (f" attached {s.gripper.attached_id}" if s.gripper.attached else "")),
        ("|F| flange N", f"{s.force_n:6.1f} / budget {s.force_budget_n:.1f}" + ("  OVER" if s.force_n > s.force_budget_n else "")),
        ("envelope interventions", str(s.interventions)),
        ("RTF", f"{s.rtf:.2f}"),
    ]
    return "\n".join(r[0] for r in rows), "\n".join(r[1] for r in rows)
