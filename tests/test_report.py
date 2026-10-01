"""`report.append_row` / `row`: append-only markdown with a header written once, LF newlines, and a fresh table header
(never a rewrite) when the column set changes."""
import dataclasses

from testbed.bench import report as REP
from testbed.bench.metrics import PlacementMetrics

M = PlacementMetrics(success=True, pos_err_m=0.0075, yaw_err_rad=0.0075, tilt_err_rad=0.01, cycle_s=81.22, phase_s={"seat": 7.8},
              peak_force_n=411.44, seat_force_n=179.76, attach_confirmed=True, release_confirmed=True, foundation_disp_m=0.0009,
              interventions=1048, force_interventions=0, carry_force_interventions=0, carried_min_z_m=0.2402, stops=0, helps=0,
              wheel_contact_frac=0.8)
META = {"git": "abc1234", "seed": 0, "surface": "flat"}


def test_row_formats_every_column_the_header_names():
    r = REP.row(M, META)
    cells = [c.strip() for c in r.strip().strip("|").split("|")]
    header = [c.strip() for c in REP.HEADER.splitlines()[0].strip().strip("|").split("|")]
    assert len(cells) == len(header) == 15
    assert cells[1:5] == ["abc1234", "0", "flat", "PASS"]
    assert cells[5:8] == ["7.5", "0.43", "81.2"] and cells[8] == "411.4" and cells[10] == "0.9"
    assert cells[12] == "0" and cells[13] == "0.240" and cells[14] == "1/1"
    assert "fail" in REP.row(dataclasses.replace(M, success=False), META)


def test_append_row_creates_the_file_with_one_header_then_only_appends(tmp_path):
    md = tmp_path / "sub" / "b.md"                             # parent directory is created too
    REP.append_row(md, "scen", M, META)
    first = md.read_bytes()
    assert first.decode().startswith("# Benchmark: scen") and first.decode().count("| date |") == 1
    REP.append_row(md, "scen", dataclasses.replace(M, cycle_s=90.0), META)
    second = md.read_bytes()
    assert second.startswith(first)                            # append-only: the old bytes are a prefix
    assert second.decode().count("| date |") == 1 and second.decode().count("| PASS |") == 2


def test_append_row_preserves_existing_prose_and_starts_a_new_table_when_columns_change(tmp_path):
    md = tmp_path / "b.md"
    old = ("# Single placement\n\nHand-written prose.\n\n| date | commit | seed |\n|---|---|---|\n| 2026-09-21 | e9c1e5c | 0 |\n")
    md.write_bytes(old.encode("utf-8"))
    REP.append_row(md, "scen", M, META)
    new = md.read_bytes().decode("utf-8")
    assert new.startswith(old)                                 # nothing before the append was touched
    tail = new[len(old):]
    assert tail.startswith("\n" + REP.NEW_TABLE_NOTE) and REP.HEADER in tail and tail.rstrip().endswith("|")
    REP.append_row(md, "scen", M, META)                        # the second append now sees the CURRENT header last: no third table
    assert md.read_text(encoding="utf-8").count(REP.HEADER.splitlines()[0]) == 1


def test_append_row_writes_lf_only_even_onto_a_crlf_looking_platform(tmp_path):
    md = tmp_path / "b.md"
    REP.append_row(md, "scen", M, META)
    REP.append_row(md, "scen", M, META)
    assert b"\r" not in md.read_bytes()
