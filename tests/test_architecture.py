"""Import boundaries from the spec §2.1, enforced by an AST scan of every module."""
import ast
from pathlib import Path

import pytest

from tests.conftest import REPO

PKG = REPO / "testbed"

# package -> forbidden import roots (absolute form; relative forms resolved below)
FORBIDDEN = {
    "spec": {"mujoco", "glfw", "testbed.control", "testbed.sim", "testbed.bench", "testbed.viz"},
    "control": {"mujoco", "glfw", "testbed.sim", "testbed.bench", "testbed.viz"},
    "viz": {"mujoco", "glfw", "testbed.sim", "testbed.bench"},
    "bench": {"mujoco", "glfw"},
    "sim": {"testbed.bench", "testbed.viz"},
}
MUJOCO_IMPORTERS = {"sim"}


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out = set()
    pkg_parts = path.relative_to(REPO).with_suffix("").parts  # ('testbed','sim','x')
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:  # relative: resolve against this file's package
                base = list(pkg_parts[:-node.level])
                mod = ".".join(base + ([node.module] if node.module else []))
            else:
                mod = node.module or ""
            out.add(mod)
            out.update(f"{mod}.{a.name}" for a in node.names)
    return out


def _violates(imported: str, forbidden: set[str]) -> bool:
    return any(imported == f or imported.startswith(f + ".") for f in forbidden)


@pytest.mark.parametrize("subpkg", sorted(FORBIDDEN))
def test_package_import_boundaries(subpkg):
    bad = []
    for py in (PKG / subpkg).rglob("*.py"):
        for imp in _imports(py):
            if _violates(imp, FORBIDDEN[subpkg]):
                bad.append(f"{py.relative_to(REPO)} imports {imp}")
    assert not bad, "\n".join(bad)


def test_only_sim_imports_mujoco():
    importers = set()
    for py in PKG.rglob("*.py"):
        if any(i == "mujoco" or i.startswith("mujoco.") for i in _imports(py)):
            importers.add(py.relative_to(PKG).parts[0])
    assert importers <= MUJOCO_IMPORTERS, importers


def test_nothing_imports_app():
    bad = [str(py) for py in PKG.rglob("*.py") if "app" in _imports(py)]
    assert not bad, bad


LOCKED_SUBPACKAGES = {"spec", "control", "sim", "bench", "viz"}


def test_locked_package_layout():
    """The set of subpackages under `testbed/` is exactly the five the boundaries above are written for: a
    missing one means a boundary check silently scans nothing; an EXTRA one is a package no boundary rule
    covers (it could import mujoco or anything else unchecked). Adding a package means adding it to
    FORBIDDEN/MUJOCO_IMPORTERS and to this set, on purpose."""
    dirs = {p.name for p in PKG.iterdir() if p.is_dir() and p.name != "__pycache__"}
    assert dirs == LOCKED_SUBPACKAGES, f"extra: {sorted(dirs - LOCKED_SUBPACKAGES)}, missing: {sorted(LOCKED_SUBPACKAGES - dirs)}"
    for sub in LOCKED_SUBPACKAGES:
        assert (PKG / sub / "__init__.py").is_file(), sub
    assert set(FORBIDDEN) == LOCKED_SUBPACKAGES, "every locked subpackage needs an import-boundary rule"
