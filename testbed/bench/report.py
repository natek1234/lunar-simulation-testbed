"""Append-only markdown benchmark rows with provenance (commit, seed, date). Never
rewrites an existing row -- `append_row` only ever creates the file (with its header) if
missing and appends new lines otherwise, per CONTRIBUTING.md house rule 6: "a number
is recorded before it is gated"."""
import datetime as _dt
import math
from pathlib import Path

from testbed.bench.metrics import PlacementMetrics

HEADER = ("| date | commit | seed | surface | success | pos err mm | yaw err deg | cycle s | peak F N | seat F N "
          "| found. disp mm | interventions | force interv | min carried z m | attach/release |\n"
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
NEW_TABLE_NOTE = "Columns changed since the table above (see the header below); rows above are unchanged history.\n\n"


def row(m: PlacementMetrics, meta: dict) -> str:
    d = _dt.date.today().isoformat()
    return (f"| {d} | {meta.get('git', '?')} | {meta.get('seed')} | {meta.get('surface')} | {'PASS' if m.success else 'fail'} "
            f"| {m.pos_err_m*1000:.1f} | {math.degrees(m.yaw_err_rad):.2f} | {m.cycle_s:.1f} | {m.peak_force_n:.1f} | {m.seat_force_n:.1f} "
            f"| {m.foundation_disp_m*1000:.1f} | {m.interventions} | {m.force_interventions} | {m.carried_min_z_m:.3f} "
            f"| {int(m.attach_confirmed)}/{int(m.release_confirmed)} |\n")


def append_row(md_path: Path, scenario: str, m: PlacementMetrics, meta: dict) -> None:
    """Append one row; never rewrite anything. Writes use `newline="\\n"` so a Windows
    checkout with autocrlf cannot leave CRLF in a tracked file (`.gitattributes` pins
    `docs/benchmarks/*.md` to LF as well, DECISIONS.md D18). If the file's LAST table
    header is not the current one (the column set changed), a fresh table header is appended
    first, so old rows stay under their own header."""
    md_path = Path(md_path)
    if not md_path.exists():
        md_path.parent.mkdir(parents=True, exist_ok=True)
        with md_path.open("w", encoding="utf-8", newline="\n") as f:
            f.write(f"# Benchmark: {scenario}\n\nAppend-only. Tolerances: 25 mm / 6 deg "
                    "(docs/reference/requirements.md, placement position/orientation tolerances).\n\n" + HEADER)
    else:
        headers = [ln for ln in md_path.read_text(encoding="utf-8").splitlines() if ln.startswith("| date |")]
        if not headers or headers[-1] != HEADER.splitlines()[0]:
            with md_path.open("a", encoding="utf-8", newline="\n") as f:
                f.write("\n" + NEW_TABLE_NOTE + HEADER)
    with md_path.open("a", encoding="utf-8", newline="\n") as f:
        f.write(row(m, meta))
