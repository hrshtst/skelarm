.PHONY: install lint format type-check test test-cov test-fast test-serious clean all

# Initialize project: installs dependencies and sets up git hooks
install:
	uv sync --all-extras
	uv run pre-commit install

# Check for linting errors
lint:
	uv run ruff check .

# Format code and fix auto-fixable lint errors
format:
	uv run ruff check --fix .
	uv run ruff format .

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


# Run all checks (useful for CI or before big commits)
all: format type-check test
