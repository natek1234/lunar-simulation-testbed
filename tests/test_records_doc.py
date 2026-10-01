"""`docs/reference/records.md` is cited as THE field reference for policy authors
(docs/guide/controllers_and_policies.md), so every field of every record must have a row there --
`tool_offset_F`, `tool_kind` and `wheel_contact` were added to `Observation` and never documented."""
import dataclasses
import re

import pytest

from testbed.control import records as R
from tests.conftest import REPO

DOC = (REPO / "docs" / "reference" / "records.md").read_text(encoding="utf-8")
SECTIONS = {s.split("\n", 1)[0].strip("` "): set(re.findall(r"^\| `(\w+)` \|", s, re.M)) for s in re.split(r"\n## ", DOC)[1:]}
RECORDS = [c for c in vars(R).values() if dataclasses.is_dataclass(c)]


@pytest.mark.parametrize("cls", RECORDS, ids=lambda c: c.__name__)
def test_every_record_field_is_documented_and_nothing_extra_is(cls):
    assert cls.__name__ in SECTIONS, f"records.md has no section for {cls.__name__}"
    fields = {f.name for f in dataclasses.fields(cls)}
    assert fields == SECTIONS[cls.__name__], (f"undocumented: {sorted(fields - SECTIONS[cls.__name__])}, "
                                              f"stale: {sorted(SECTIONS[cls.__name__] - fields)}")
