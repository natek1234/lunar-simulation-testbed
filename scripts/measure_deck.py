"""Measure the MPO-700 chassis deck top (max mesh z) in base_link -- the arm mount height.
Records: [measured: scripts/measure_deck.py, 2026-09-18] -> 0.3478 m."""
from pathlib import Path
import numpy as np

obj = Path(__file__).resolve().parents[1] / "assets/mpo700/meshes/MPO-700-BODY.obj"
v = np.array([[float(x) for x in l.split()[1:4]] for l in obj.read_text().splitlines() if l.startswith("v ")]) * 0.001
print(f"deck top z = {v[:, 2].max():.4f} m; mesh xy extent {v[:, 0].min():.3f}..{v[:, 0].max():.3f}, {v[:, 1].min():.3f}..{v[:, 1].max():.3f}")
