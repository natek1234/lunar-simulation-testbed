"""Vendor the licensed assets at pinned upstream commits and (re)write the sha256 ledger.

Usage:  python scripts/vendor_assets.py fetch     # clones upstreams into a temp dir, copies files
        python scripts/vendor_assets.py ledger    # rewrites assets/PROVENANCE.md digests
Sources (spec §1.2): swerve-nav (MIT) for the MPO-700, mujoco_menagerie (Apache-2.0) for
FR3 and the Franka hand, Poly Haven (CC0) for the two regolith textures.

Note (2026-09-18): mujoco_menagerie is a large monorepo hosting dozens of robots' meshes.
A plain `--filter=blob:none --no-checkout` clone followed by `git checkout <commit>` would
materialize the ENTIRE tree at that commit (every robot's meshes), not just franka_fr3/
franka_emika_panda. `_clone_menagerie` instead runs `git sparse-checkout init --cone` +
`git sparse-checkout set franka_fr3 franka_emika_panda LICENSE` BEFORE the checkout step,
so only those directories' blobs are fetched. swerve-nav is small and does not need this.
"""
import hashlib
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ASSETS = REPO / "assets"
SWERVE_NAV = ("https://github.com/FunanShi/swerve-nav.git", "0fb97e36dda5d072f4e4cc6cce87ca1f6a98cfe0")
MENAGERIE = ("https://github.com/google-deepmind/mujoco_menagerie.git", "c1a4eeb85694ae1dffe33ff1797d4e528928a133")
POLYHAVEN = {
    "moon_01_diff_1k.png": "https://dl.polyhaven.org/file/ph-assets/Textures/png/1k/moon_01/moon_01_diff_1k.png",
    "moon_02_diff_1k.png": "https://dl.polyhaven.org/file/ph-assets/Textures/png/1k/moon_02/moon_02_diff_1k.png",
}
HAND_FILES = ["hand.xml", "assets/hand.stl", "assets/hand_0.obj", "assets/hand_1.obj", "assets/hand_2.obj",
              "assets/hand_3.obj", "assets/hand_4.obj", "assets/finger_0.obj", "assets/finger_1.obj"]


def _clone(url: str, commit: str, into: Path) -> Path:
    subprocess.run(["git", "clone", "--filter=blob:none", "--no-checkout", url, str(into)], check=True)
    subprocess.run(["git", "-C", str(into), "checkout", commit], check=True)
    return into


def _clone_menagerie(url: str, commit: str, into: Path) -> Path:
    """Same as _clone, but sparse to franka_fr3/franka_emika_panda/LICENSE only -- see
    module docstring. menagerie hosts dozens of robots; a full checkout at this commit
    would fetch every one of their meshes."""
    subprocess.run(["git", "clone", "--filter=blob:none", "--no-checkout", url, str(into)], check=True)
    subprocess.run(["git", "-C", str(into), "sparse-checkout", "init", "--cone"], check=True)
    subprocess.run(["git", "-C", str(into), "sparse-checkout", "set",
                     "franka_fr3", "franka_emika_panda", "LICENSE"], check=True)
    subprocess.run(["git", "-C", str(into), "checkout", commit], check=True)
    return into


def fetch() -> None:
    with tempfile.TemporaryDirectory() as td:
        sn = _clone(*SWERVE_NAV, Path(td) / "swerve-nav")
        dst = ASSETS / "mpo700"
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy2(sn / "robot" / "mpo_700.xml", dst / "mpo_700.xml")
        shutil.copytree(sn / "robot" / "meshes", dst / "meshes", dirs_exist_ok=True)
        shutil.copy2(sn / "LICENSE", dst / "LICENSE")
        mg = _clone_menagerie(*MENAGERIE, Path(td) / "menagerie")
        shutil.copytree(mg / "franka_fr3", ASSETS / "franka_fr3", dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns("*.png", "scene.xml"))
        shutil.copy2(mg / "LICENSE", ASSETS / "franka_fr3" / "LICENSE")
        hand = ASSETS / "franka_hand"
        (hand / "assets").mkdir(parents=True, exist_ok=True)
        for rel in HAND_FILES:
            shutil.copy2(mg / "franka_emika_panda" / rel, hand / rel)
        shutil.copy2(mg / "LICENSE", hand / "LICENSE")
    tex = ASSETS / "textures"
    tex.mkdir(exist_ok=True)
    for name, url in POLYHAVEN.items():
        urllib.request.urlretrieve(url, tex / name)


def ledger() -> None:
    rows = []
    for p in sorted(ASSETS.rglob("*")):
        if p.is_file() and p.name != "PROVENANCE.md":
            rel = str(p.relative_to(REPO)).replace("\\", "/")
            rows.append(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {rel}")
    head = (ASSETS / "PROVENANCE.md").read_text(encoding="utf-8").split("## sha256")[0]
    (ASSETS / "PROVENANCE.md").write_text(
        head + "## sha256\n\n```\n" + "\n".join(rows) + "\n```\n",
        encoding="utf-8", newline="\n",
    )


if __name__ == "__main__":
    {"fetch": fetch, "ledger": ledger}[sys.argv[1]]()
