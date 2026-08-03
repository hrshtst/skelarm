# Development and Testing

How to work on `skelarm` itself.

## Setup

```bash
git clone https://github.com/hrshtst/skelarm.git
cd skelarm
uv sync --all-extras
uv run pre-commit install
```

## Everyday commands

Tooling runs through `uv` and the `Makefile`:

| Command | What it does |
| --- | --- |
| `make all` | Format + type-check + test — run before declaring a change done. |
| `make format` | `ruff` auto-format and fixable lint. |
| `make lint` / `make type-check` | `ruff check` (no fix) / `basedpyright` + `mypy` (both must pass). |
| `make test` | Full `pytest` suite; `make test-fast` skips slow tests; `make test-cov` adds coverage. |
| `make nox` | Tests across Python 3.12 / 3.13 / 3.14. |
| `make docs-build` / `make docs-serve` | Build / locally serve this documentation (MkDocs). |

Run a single test with `uv run pytest tests/test_dynamics.py -k 'name'`.

## Testing philosophy

Development is test-first (red → green → refactor). Physics behavior is pinned
with `hypothesis` property tests wherever it is a mathematical relationship —
`FK(IK(p)) ≈ p`, `ID(FD(τ)) ≈ τ`, energy conservation — rather than single
examples; preserve those invariants when touching kinematics or dynamics.
Markers `slow`, `serious`, and `integration` gate the heavier tests
(`make test-serious`).

## Style

- Line length **120**, double quotes; every module starts with
  `from __future__ import annotations`.
- Public functions carry full type hints and **NumPy-style docstrings**; arrays
  are typed `numpy.typing.NDArray[np.float64]`.
- `ruff` runs rule set `ALL` with project-specific ignores — use `make format`
  rather than guessing.

## AI assistance & contributor policy

This project is developed with the assistance of AI coding agents: the
maintainer, [Hiroshi Atsuta](https://github.com/hrshtst), writes the project
guidance and the theoretical reference chapters, the AI implements against
them, and the maintainer reviews, tests, and revises every change. All
responsibility for the code lies with the maintainer.

External contributors are welcome to use AI tools under the same standard: if
you use AI to generate code for a pull request, **disclose it in the PR
description** and make sure you have thoroughly reviewed and tested the code.
If you find code that appears unoriginal or rights-protected, please file an
issue immediately.

## Related

- [Architecture](../api/architecture.md) — how the modules fit together.
- [Defining a Task](defining_a_task.md) / [Defining a Controller](defining_a_controller.md)
  — the extension points.
