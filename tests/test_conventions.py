"""Provenance tags on numeric module constants, and cited paths must exist."""
import ast
import re
from pathlib import Path

import pytest

from tests.conftest import REPO

PKG = REPO / "testbed"
TAG = re.compile(r"\[(ported|authored-[a-z]+|policy default|measured|derived)(:[^\]]*)?\]")
NUMERIC = re.compile(r"(?<![\w.])\d+(\.\d+)?(e-?\d+)?(?![\w])")
PATH_CITE = re.compile(r"`((?:testbed|tests|docs|assets|robots|surfaces|scenarios|app\.py)[\w./-]*)`")


def _module_constants(path: Path):
    """Yield (name, lineno) for module-level ALL_CAPS assignments whose source contains a number."""
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src)
    lines = src.splitlines()
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name.isupper():
                seg = "\n".join(lines[node.lineno - 1:node.end_lineno])
                if NUMERIC.search(ast.get_source_segment(src, node.value) or ""):
                    yield name, node.lineno, seg, lines


def _tag_in_reach(lineno: int, lines: list[str]) -> bool:
    """A tag counts if it is on the constant's own lines or in the contiguous
    comment/constant paragraph above it (a blank line ends the reach)."""
    i = lineno - 1
    while i >= 0 and lines[i].strip():
        if TAG.search(lines[i]):
            return True
        i -= 1
    j = lineno
    while j < len(lines) and lines[j].strip() and not lines[j].lstrip().startswith(("def ", "class ")):
        if TAG.search(lines[j]):
            return True
        if not lines[j].lstrip().startswith("#") and "=" in lines[j] and j != lineno - 1:
            break
        j += 1
    return False


def test_numeric_constants_carry_provenance_tags():
    missing = []
    for py in PKG.rglob("*.py"):
        for name, lineno, seg, lines in _module_constants(py):
            if not _tag_in_reach(lineno, lines):
                missing.append(f"{py.relative_to(REPO)}:{lineno} {name}")
    assert not missing, "untagged numeric constants:\n" + "\n".join(missing)


# A token that LOOKS like a provenance tag (starts with one of the vocabulary's head words) must be a
# real one. The original check only ever flagged `[authored:`, so `[policy]`, `[measured-ish: x]` and
# `[derivedfrom: y]` all passed while CONTRIBUTING.md advertised a closed vocabulary.
TAG_SHAPED = re.compile(r"\[(?:ported|authored|policy|measured|derived)[^\]]*\]")
TAG_VOCAB = re.compile(r"\[(?:(?:ported|measured|derived):\s+[^\]]+|authored-[a-z]+(?::\s+[^\]]+)?|policy default(?::\s+[^\]]+)?)\]")


def _bad_tags(text: str) -> list[str]:
    return [m.group(0) for m in TAG_SHAPED.finditer(text) if not TAG_VOCAB.fullmatch(m.group(0))]


def _tag_scan_files():
    """Where tags live and are meant to be real: the code, the tests, the YAML that describes a run, and the
    living docs. NOT scanned, each for a reason: DECISIONS.md is an append-only ledger whose old entries are
    prose (`[measured 2026-09-20: ...]`) and are corrected by dated addenda, never rewritten;
    docs/plans and docs/specs are forward-looking drafts; CONTRIBUTING.md states the grammar itself
    (`[authored-<domain>: why]`); this file quotes malformed tags on purpose."""
    files = list(PKG.rglob("*.py")) + [p for p in (REPO / "tests").rglob("*.py") if p.name != "test_conventions.py"]
    files += [p for d in ("robots", "scenarios", "surfaces") for p in (REPO / d).glob("*.yaml")]
    files += [p for p in (REPO / "docs").rglob("*.md") if not {"plans", "specs"} & set(p.parts)] + [REPO / "README.md"]
    return sorted(files)


def test_tag_vocabulary_is_closed():
    bad = [f"{f.relative_to(REPO)}: {t[:80]!r}" for f in _tag_scan_files() for t in _bad_tags(f.read_text(encoding="utf-8"))]
    assert not bad, "provenance tags outside the closed vocabulary:\n" + "\n".join(bad)


@pytest.mark.parametrize("tag", ["[policy]", "[measured-ish: x]", "[derivedfrom: y]", "[authored: x]", "[authored-: x]",
                                 "[measured]", "[ported]", "[measured 2026-09-20: x]", "[derived:]"])
def test_the_vocabulary_check_rejects_malformed_tags(tag):
    assert _bad_tags(f"x = 1  # {tag}") == [tag]


@pytest.mark.parametrize("tag", ["[policy default]", "[policy default: why]", "[ported: docs/reference/requirements.md]", "[authored-geometry: rover scale]",
                                 "[measured: script, 2026-09-21]", "[derived: from x\n#  continued]"])
def test_the_vocabulary_check_accepts_the_documented_forms(tag):
    assert _bad_tags(f"x = 1  # {tag}") == []


# docs/plans and docs/specs are forward-looking planning artifacts: they name files not
# yet built, on purpose, so they are exempt from "the path exists today" (D3, DECISIONS.md).
# Living docs (guide/, reference/, benchmarks/, README, CONTRIBUTING) are not exempt -- a
# path they cite must be real now, except for a short, named list of paths a living doc has
# already committed to describing before the code that creates them lands.
_LIVING_DOCS = sorted(
    p for p in (REPO / "docs").rglob("*.md") if not {"plans", "specs"} & set(p.parts)
) + [REPO / "README.md", REPO / "CONTRIBUTING.md"]

EXEMPT_PLANNED_PATHS: dict[str, str] = {
    # app.py, assets/, assets/PROVENANCE.md, robots/, surfaces/, scenarios/ were here while
    # the repo's living docs described them ahead of the code that created them; all are
    # removed now that each exists -- an exemption for a path that already exists
    # checks nothing. Empty on purpose: add an entry only for a path a living doc names
    # BEFORE the task that creates it lands, with that task named as the reason.
}


def test_exempt_planned_paths_names_only_paths_that_do_not_exist_yet():
    stale = sorted(p for p in EXEMPT_PLANNED_PATHS if (REPO / p).exists())
    assert not stale, f"exemptions for paths that now exist (delete them): {stale}"


@pytest.mark.parametrize("doc", _LIVING_DOCS)
def test_cited_paths_exist(doc):
    text = doc.read_text(encoding="utf-8")
    missing = sorted(
        {p for p in PATH_CITE.findall(text) if not (REPO / p).exists()} - EXEMPT_PLANNED_PATHS.keys()
    )
    assert not missing, f"{doc.name} cites missing paths: {missing}"
