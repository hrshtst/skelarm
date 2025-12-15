"""Configuration for the nox test runner."""

from __future__ import annotations

import nox

# Use uv to manage virtual environments for faster setup
nox.options.default_venv_backend = "uv"


@nox.session(python=["3.12", "3.13", "3.14"])
def tests(session: nox.Session) -> None:
    """Run the test suite."""
    # Install dev dependencies for testing
    session.install("pytest", "hypothesis", "pytest-cov", "pytest-randomly", "pytest-xdist")
    # Install the package itself
    session.install(".")

    session.run("pytest")


@nox.session
def lint(session: nox.Session) -> None:
    """Run linting using ruff."""
    session.install("ruff")
    session.run("ruff", "check", ".")


@nox.session
def type_check(session: nox.Session) -> None:
    """Run static type checking using pyright."""
    # Install pyright and the package (so dependencies are available for type checking)
    session.install("pyright")
    session.install(".")
    session.run("pyright")
