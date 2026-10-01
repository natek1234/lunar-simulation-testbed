"""Every vendored file hashes to the digest recorded in assets/PROVENANCE.md."""
import hashlib
import re

from tests.conftest import REPO

ASSETS = REPO / "assets"
ROW = re.compile(r"^([0-9a-f]{64})\s+(assets/\S+)$", re.M)


def _ledger() -> dict[str, str]:
    text = (ASSETS / "PROVENANCE.md").read_text(encoding="utf-8")
    return {path: digest for digest, path in ROW.findall(text)}


def test_ledger_covers_every_vendored_file():
    ledger = _ledger()
    on_disk = {str(p.relative_to(REPO)).replace("\\", "/")
               for p in ASSETS.rglob("*") if p.is_file() and p.name != "PROVENANCE.md"}
    assert on_disk == set(ledger), {"unledgered": on_disk - set(ledger), "missing": set(ledger) - on_disk}


def test_every_vendored_file_matches_its_digest():
    bad = []
    for path, digest in _ledger().items():
        actual = hashlib.sha256((REPO / path).read_bytes()).hexdigest()
        if actual != digest:
            bad.append(path)
    assert not bad, bad


def test_each_asset_package_carries_its_license():
    for pkg in ("mpo700", "franka_fr3", "franka_hand"):
        assert (ASSETS / pkg / "LICENSE").is_file(), pkg
