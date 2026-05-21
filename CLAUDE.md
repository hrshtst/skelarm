# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

`skelarm` is a planar (2D) robot-arm dynamics simulator: a configurable "skeleton" of N links with kinematics (FK/IK), dynamics (inverse/forward via Recursive Newton-Euler), and PyQt6 / Matplotlib visualization. Core modules live in `src/skelarm/` (`skeleton.py`, `kinematics.py`, `dynamics.py`, `canvas.py`, `plotting.py`).

## Commands

Tooling runs through `uv` and the `Makefile`:

- `make all` — format + type-check + test (run this before declaring a code change done)
- `make format` — ruff auto-format and fixable-lint (use this, not bare `ruff`)
- `make lint` / `make type-check` — ruff check (no fix) / pyright
- `make test` — full pytest suite; `make test-fast` skips slow tests; `make test-cov` adds coverage
- `make nox` — run tests across Python 3.12 / 3.13 / 3.14
- Single test: `uv run pytest tests/test_dynamics.py -k 'name'`
- `make docs-serve` — preview MkDocs site locally

First-time setup: `uv sync --all-extras && uv run pre-commit install`.

## Code style (differs from Python defaults)

- Line length is **120** (not 88); double quotes.
- Every module must start with `from __future__ import annotations` (enforced by ruff isort).
- Public functions need full type hints and **NumPy-style docstrings**; use `np2typing` for array types.
- ruff runs rule set `"ALL"` with project-specific ignores — run `make format` rather than guessing.
- Type checker is **pyright** (basic mode), not mypy.

## Physics gotchas

- **Gravity is ignored** — the arm operates on a horizontal plane. Do not add gravity terms to dynamics.
- TOML joint limits `qmin`/`qmax` are given in **degrees** and converted to radians internally.
- Property tests rely on round-trip invariants (`FK(IK(p))≈p`, `ID(FD(τ))≈τ`, energy conservation) — preserve them when touching kinematics/dynamics.

## Testing

- Tests use `pytest` + `hypothesis`. Markers: `slow`, `serious`, `integration` (e.g. `make test-serious`).
- PyQt6 tests need headless system libs in CI: `libegl1 libopengl0 libxkbcommon-x11-0 libdbus-1-3`.

## Workflow

- Commits land directly on `main`. Use Conventional Commit prefixes (`feat:`, `fix:`, `docs:`, `chore:`).
- Pre-commit (ruff + pyright) gates every commit; if ruff reformats during the hook, `git add` again before retrying.
- AI-generated code must be disclosed in PR descriptions (see README §"AI Assistance").
- Releases follow `RELEASING.md` (or the `/release` skill).
