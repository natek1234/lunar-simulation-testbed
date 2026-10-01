"""What IK needs from a robot, and nothing more. sim/kinematics_mujoco.py implements it
against the real compiled MuJoCo model; this module stays mujoco-free (testbed.control
never imports mujoco) so IK itself can be unit-tested against any Kinematics."""
from typing import Protocol

import numpy as np

from testbed.control.records import Pose


class Kinematics(Protocol):
    n: int      # number of controlled joints (arm DOFs)

    def fk(self, q_rad: np.ndarray) -> Pose: ...                  # flange pose in B
    def jacobian(self, q_rad: np.ndarray) -> np.ndarray: ...      # (6, n): linear xyz then angular xyz, in B
    def limits(self) -> tuple[np.ndarray, np.ndarray]: ...        # (lo, hi), each (n,) rad
