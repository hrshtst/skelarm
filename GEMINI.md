# GEMINI.md - Context & Requirements for "skelarm"

## 1. Project Overview
**Project Name:** `skelarm`
**Type:** Python Library & Simulation Tool
**License:** GPLv3 (Required due to PyQt6 dependency)
**Description:**
A lightweight, physics-based dynamics simulator for a configurable planar robot arm. The robot acts as a "skeleton" with no shape or collision properties, focusing purely on kinematics and dynamics. Users can configure link lengths, mass properties (mass, inertia), and centers of mass (COM) for an arbitrary number of links.

**Note on Gravity:** As the robot arm operates on a horizontal plane, gravitational effects are ignored in the dynamics calculations.

## 2. Technical Stack & Tooling
The project enforces modern Python best practices and tooling.

### Core Dependencies
* **Python:** >= 3.12
* **Math/Physics:** `numpy`, `scipy`
* **Visualization (Interactive):** `PyQt6`
* **Visualization (Static/Analysis):** `matplotlib`
* **Configuration:** `tomllib` (built-in Python 3.11+)

### Development Environment & Quality Assurance
* **Package Manager:** `uv` (for fast dependency resolution and venv management).
* **Linter & Formatter:** `ruff` (strict configuration).
* **Type Checker:** `pyright` (basic mode).
* **Testing:**
    * `pytest` (standard test runner).
    * `hypothesis` (property-based testing for physics validation).
* **Automation:** `Makefile` (entry points for lint, test, run).
* **Git Hooks:** `pre-commit` (enforce ruff/pyright before commit).

## 3. Directory Structure (Src-Layout)
The project must use the `src` layout to prevent import parity issues.

```text
skelarm/
├── .venv/                       # Managed by uv
├── .github/
│   ├── workflows/
│   │   ├── ci.yml               # CI runner (test, lint, type check)
│   │   └── benchmark.yml        # Performance benchmarks
│   ├── dependabot.yml           # Configuration of dependabot
│   ├── ISSUE_TEMPLATE/          # Issue template
│   └── PULL_REQUEST_TEMPLATE.md # Pull Request template
├── src/
│   └── skelarm/
│       ├── __init__.py
│       ├── py.typed         # Marker file for static type checker
│       ├── skeleton.py      # Core Skeleton class (state, config via TOML)
│       ├── kinematics.py    # FK / IK solvers
│       ├── dynamics.py      # ID / FD solvers (Equations of Motion)
│       ├── canvas.py        # PyQt6 interactive viewer
│       ├── plotting.py      # Matplotlib trajectory utilities
│       ├── simulator.py     # Simulator class
│       ├── controller.py    # Controller class
│       └── logger.py        # Data logger class
├── tests/
│   ├── __init__.py
│   ├── test_skeleton.py
│   ├── test_kinematics.py   # Heavy use of Hypothesis here
│   ├── test_dynamics.py     # Heavy use of Hypothesis here
│   ├── test_canvas.py
│   ├── test_plotting.py
│   ├── test_simulator.py
│   ├── test_controller.py
│   ├── test_logger.py
│   ├── test_config.py       # Tests for TOML configuration loading
│   ├── test_four_dof.py     # Tests for 4-DOF robot example
│   └── conftest.py
├── examples/                # Runnable scripts demonstrating usage
│   ├── __init__.py
│   ├── basic_kinematics.py
│   ├── basic_plotting.py    # Basic Matplotlib plotting
│   ├── dynamics_simulation.py
│   ├── interactive_gui.py
│   ├── simulate_four_dof.py # 4-DOF robot simulation
│   ├── simple_robot.toml    # Example configuration
│   ├── four_dof_robot.toml  # 4-DOF configuration
│   └── reaching_control.py
├── docs/                    # Documentations behind implementation
├── scripts/                 # Runnable utility scripts
├── pyproject.toml           # Unified config for uv, ruff, pyright, pytest
├── Makefile                 # Task runner
├── .pre-commit-config.yaml  # Configurations of pre-commit
└── README.md
```

## 4. Implementation Requirements

### A. Core Physics (`src/skelarm/skeleton.py`, `src/skelarm/dynamics.py`, `src/skelarm/kinematics.py`)

1.  **Configuration:**
      * The robot (skeleton) is defined by a list of `Link` objects.
      * **TOML Support:** Robots can be configured via TOML files using `Skeleton.from_toml()`.
      * Each `LinkProp` has: `length`, `mass`, `inertia`, `comx`, `comy` (relative to joint) `qmin`, `qmax`.
      * Each `Link` is instantiated by `LinkProp` object or `Mapping` object.
        *  Properties: `l`, `m`, `i`, `rgx`, `rgy` (COM relative to joint) `qmin`, `qmax`.
        *  States: `q`, `dq`, `ddq`, `x`, `y`, `vx`, `vy`, `ax`, `ay`, `xg`, `yg`, `agx`, `agy`, `jx`, `jy`, `hx`, `hy`, `fx`, `fy`, `tau`, `fex`, `fey`, `rex`, `rey`, `xe`, `ye`.
      * The robot must support $N$ links (generic implementation).
2.  **Kinematics:**
      * **Forward Kinematics (FK):** Compute $(x, y)$ of all joints and tip given joint angles $\mathbf{q}$.
      * **Inverse Kinematics (IK):** Numerical solution (e.g., Levenberg-Marquardt via `scipy.optimize` or Jacobian transpose/pseudo-inverse) to find $\mathbf{q}$ given tip $(x, y)$.
3.  **Dynamics:**
      * **Inverse Dynamics (ID):** Calculate required torques $\boldsymbol{\tau}$ given $\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}$. (Suggested algorithm: Recursive Newton-Euler). Gravity is ignored (horizontal plane).
      * **Forward Dynamics (FD):** Calculate $\ddot{\mathbf{q}}$ given $\mathbf{q}, \dot{\mathbf{q}}, \boldsymbol{\tau}$. This requires solving $\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{h}(\mathbf{q}, \dot{\mathbf{q}}) = \boldsymbol{\tau}$. Gravity is ignored.
      * Integrator: Use `scipy.integrate.odeint` or `solve_ivp` (RK45) for simulation steps.

### B. Visualization

1.  **Matplotlib (`src/skelarm/plotting.py`):**
      * Function to draw the "skeleton" (lines between joints) given a state.
      * Function to plot tip trajectory history.
2.  **PyQt6 Visualizer (`src/skelarm/canvas.py`):**
      * Real-time animation of the robot arm.
      * Sliders to control joint angles (in Kinematic Mode).
      * Clean drawing canvas (likely `QGraphicsScene` or custom `QWidget.paintEvent`).

### C. Testing Strategy

1.  **Property-Based Testing (`hypothesis`):**
      * *Kinematics:* $FK(IK(pos)) \approx pos$.
      * *Dynamics:* $ID(FD(torque)) \approx torque$.
      * *Energy:* In the absence of friction and input torque, total kinetic energy should be constant (potential energy is constant/zero as gravity is ignored).

## 5. Development Guidelines

  * **Docstrings:** All public functions must have NumPy-style docstrings.
  * **Typing:** All function signatures must be fully type-hinted. Use `nptyping` or `numpy.typing.NDArray` for array specs.
  * **Error Handling:** Raise descriptive errors (e.g., `ConfigurationError` if link mass < 0).

## 6. Initial Task List

1.  [x] Initialize project with `uv` and configure `pyproject.toml`.
2.  [x] Implement `Link` class and `Skeleton` container.
3.  [x] Implement FK and verify with simple tests.
4.  [x] Implement basic Matplotlib visualization.
5.  [x] Implement Recursive Newton-Euler for Inverse Dynamics (Gravity Ignored).
6.  [x] Implement Forward Dynamics and Integrator (Gravity Ignored).
7.  [x] Add `hypothesis` tests for physics consistency (Dynamics consistency and Local Energy Conservation).
8.  [x] Build PyQt6 visualizer.
9.  [x] Implement TOML configuration support.
10. [x] Add 4-DOF robot example.

## 7. Development Workflow & Commands

This project uses `Makefile` to abstract complex commands and `uv` to manage the environment.

### A. First-Time Setup
Run these commands immediately after cloning the repository to set up the environment and git hooks:
```bash
uv sync                 # Install dependencies from pyproject.toml
uv run pre-commit install  # Install git hooks (ensures checks run before commit)
```

### B. Quality Assurance (Manual)

You can run these checks manually at any time.

  * **Linting & Formatting (Ruff):**

    ```bash
    make lint           # Check for linting errors (no auto-fix)
    make format         # Auto-format code and fix fixable lint errors
    # Raw command: uv run ruff check . && uv run ruff format .
    ```

  * **Type Checking (Pyright):**

    ```bash
    make type-check     # Run static type analysis
    # Raw command: uv run pyright
    ```

  * **Testing (Pytest):**

    ```bash
    make test           # Run all tests
    make test-fast      # Run tests skipping slow hypothesis generation (optional)
    # Raw command: uv run pytest
    ```

### C. The Commit Process

We use `pre-commit` to act as a gatekeeper.

1.  **Stage your changes:**
    ```bash
    git add <file1> <file2> ...
    ```
2.  **Commit:**
    ```bash
    git commit -m "feat: implement recursive newton-euler"
    ```
      * *Note:* This will automatically trigger `ruff` and `pyright`.
      * If they fail, the commit is aborted. Fix the errors and try again.
      * If `ruff` auto-formats code during the hook, you must `git add` the changes again before committing.

### D. Makefile Reference

The `Makefile` should include at least these targets:

```makefile
.PHONY: install lint format type-check test clean

install:
	uv sync
	uv run pre-commit install

lint:
	uv run ruff check .

format:
	uv run ruff check --fix .
	uv run ruff format .

type-check:
	uv run pyright

test:
	uv run pytest

clean:
	rm -rf .pytest_cache .ruff_cache .venv
	find . -type d -name "__pycache__" -exec rm -rf {} +
```

```

### Next Steps for You (The User):

1.  Create a folder named `skelarm`.
2.  Save the content above into `skelarm/GEMINI.md`.
3.  Run `uv init` in that folder (if you have `uv` installed).
4.  **Prompt me:** "Read GEMINI.md and help me step 1: Initialize the project configuration (pyproject.toml, Makefile)."
```
