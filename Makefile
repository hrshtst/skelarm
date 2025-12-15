.PHONY: install lint format type-check test test-cov test-fast test-serious nox clean all help

# Initialize project: installs dependencies and sets up git hooks
install:
	uv sync --all-extras
	uv run pre-commit install

# Check for linting errors
lint:
	uv run ruff check . --config=pyproject.toml

# Format code and fix auto-fixable lint errors
format:
	uv run ruff check --fix . --config=pyproject.toml
	uv run ruff format . --config=pyproject.toml

# Run static type checking
type-check:
	uv run pyright

# Run tests
test:
	uv run pytest

# Run tests with coverage report
test-cov:
	uv run pytest --cov=src --cov-report=html --cov-report=term

# Fast tests: Skip tests marked with @pytest.mark.slow
test-fast:
	uv run pytest -m "not slow"

# Serious tests: Run only tests marked with @pytest.mark.serious
test-serious:
	uv run pytest -m "serious"

# Run tests in multiple environments using nox
nox:
	uv run nox

# Clean up cache files
clean:
	rm -rf .venv
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".mypy_cache" -exec rm -rf {} +
	find . -type d -name ".ruff_cache" -exec rm -rf {} +
	find . -type d -name "htmlcov" -exec rm -rf {} +
	find . -type f -name ".coverage" -delete
	find . -type d -name ".nox" -exec rm -rf {} +


# Run all checks (useful for CI or before big commits)
all: format type-check test

help:
	@echo "Usage: make <command>"
	@echo ""
	@echo "Commands:"
	@echo "  install      : Install dependencies and set up git hooks."
	@echo "  lint         : Check for linting errors."
	@echo "  format       : Format code and fix auto-fixable lint errors."
	@echo "  type-check   : Run static type checking."
	@echo "  test         : Run all tests."
	@echo "  test-cov     : Run tests with coverage report."
	@echo "  test-fast    : Run tests skipping slow tests."
	@echo "  test-serious : Run only serious tests."
	@echo "  nox          : Run tests in multiple environments using nox."
	@echo "  clean        : Clean up cache files."
	@echo "  all          : Run all checks (format, type-check, test)."
