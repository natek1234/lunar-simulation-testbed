# Contributing

## House rules (all enforced by the default test tier)

1. **Import boundaries** — `tests/test_architecture.py`. `testbed/sim` is the only
   package that imports `mujoco`; `control`, `viz`, `spec` never do.
2. **Provenance tags** — every numeric module constant carries one of
   `[ported: src]`, `[authored-<domain>: why]`, `[policy default]`,
   `[measured: how, date]`, `[derived: from]` (`tests/test_conventions.py`), and any
   tag-shaped token in code, tests, YAML or the living docs must be one of those exact
   forms (`[policy]`, `[measured-ish: x]`, `[authored: x]` fail the build).
   `[policy default]` is the honest tag for an arbitrary number.
3. **Cited paths exist** — any `path/in/backticks` in docs must exist.
4. **Vendored assets are byte-exact** with `assets/PROVENANCE.md`.
5. **Decisions are ledgered** — a non-obvious ruling gets a numbered entry in
   `DECISIONS.md` with the evidence behind it.
6. **Benchmarks are append-only** — `docs/benchmarks/*.md` rows carry commit,
   seed, conditions and date; a number is recorded before it is gated.

## Test tiers

- Default: `python -m pytest -q` — headless, CPU-only.
- Viewer: `python -m pytest -m viewer` — opens a real window.

## Workflow

One branch, `main`. Commit after every green test cycle with a conventional-commit
message (`feat:`, `fix:`, `docs:`, `test:`, `chore:`). CI (ubuntu + windows) must be
green before the next task starts.
