# skelarm

A lightweight, physics-based dynamics simulator for a configurable planar robot arm. `skelarm` focuses on kinematics and dynamics simulation without collision detection or complex shape rendering, treating the robot as a "skeleton" of links.

## Features

*   **Configurable Robot:** Define arbitrary planar robots with custom link lengths, masses, inertias, and centers of mass. Support for TOML configuration files.
*   **Kinematics:**
    *   Forward Kinematics (FK) to compute joint and end-effector positions.
*   **Dynamics (Planar, No Gravity):**
    *   Inverse Dynamics (ID) using Recursive Newton-Euler algorithm.
    *   Forward Dynamics (FD) using mass matrix and Coriolis/centrifugal terms.
    *   Physics integration using `scipy.integrate.solve_ivp`.
    *   **Note:** Gravity is explicitly ignored as the robot operates on a horizontal plane.
*   **Visualization:**
    *   Static plotting with `matplotlib`.
    *   Interactive GUI visualizer with `PyQt6` and joint sliders.
*   **Quality Assurance:** Fully typed, tested with `pytest` and `hypothesis`, and linted with `ruff`.

## Getting Started

### Prerequisites

*   Python 3.12 or higher.
*   `uv` package manager (recommended) or standard `pip`.

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/yourusername/skelarm.git
    cd skelarm
    ```

2.  **Install dependencies using `uv` (Recommended):**
    ```bash
    uv sync
    ```
    Or using `make`:
    ```bash
    make install
    ```

    *Alternatively, using pip:*
    ```bash
    pip install .
    ```

## Usage Examples

### TOML Configuration

You can define robot configurations in TOML files. See `examples/simple_robot.toml` or `examples/four_dof_robot.toml`.

```toml
[[link]]
length = 1.0
mass = 2.0
inertia = 0.5
com = [0.5, 0.0]        # Center of mass [x, y] relative to joint
limits = [-3.14, 3.14]  # Joint limits [min, max] in radians

[[link]]
length = 0.8
# ...
```

Load it using `Skeleton.from_toml`:

```python
from skelarm import Skeleton
skeleton = Skeleton.from_toml("path/to/robot.toml")
```

### 4-DOF Simulation Example

Run a dynamic simulation of a 4-DOF robot loaded from a TOML file:

```bash
uv run python examples/simulate_four_dof.py
```

### Interactive Visualizer

Launch the PyQt6 GUI to manipulate a 3-link robot arm with sliders:

```bash
uv run python examples/interactive_gui.py
```

### Basic Kinematics & Plotting

Run a script that defines a robot, computes its kinematics, and plots it using Matplotlib:

```bash
uv run python examples/basic_plotting.py
```

### Dynamics Simulation

You can use the library to simulate robot motion. See `src/skelarm/dynamics.py` and `tests/test_dynamics.py` for API usage.

```python
from skelarm import LinkProp, Skeleton, simulate_robot
import numpy as np

# Define a single link
link = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
skeleton = Skeleton([link])

# Initial state
skeleton.q = np.array([0.0])
skeleton.dq = np.array([0.0])

# Simulation parameters
time_span = (0.0, 1.0)
def control_torques(t, skel):
    return np.array([0.0]) # Zero torque

# Run simulation
times, q_traj, dq_traj = simulate_robot(skeleton, time_span, control_torques)
```

## Running Tests

This project uses `pytest` for unit testing and `hypothesis` for property-based testing of physics consistency.

To run the full test suite:

```bash
make test
# OR
uv run pytest
```

To run tests with coverage report:

```bash
make test-cov
```

## Development

We use `ruff` for linting and formatting, and `pyright` for static type checking.

*   **Linting:** `make lint`
*   **Formatting:** `make format`
*   **Type Checking:** `make type-check`
*   **Run all checks:** `make all`

## License

GPLv3
