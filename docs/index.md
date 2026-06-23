# skelarm

A lightweight, physics-based dynamics simulator for a configurable planar robot arm. `skelarm` focuses on kinematics and dynamics simulation without collision detection or complex shape rendering, treating the robot as a "skeleton" of links.

## Features

*   **Configurable Robot:** Define arbitrary planar robots with custom link lengths, masses, inertias, and centers of mass, plus a configurable fixed base link. Support for TOML configuration files.
*   **Kinematics:**
    *   Forward Kinematics (FK): recursive computation of joint, tip, and center-of-mass position, velocity, and acceleration.
    *   Differential Kinematics: endpoint Jacobian and centripetal/Coriolis basis, with helpers to evaluate endpoint velocity and acceleration.
*   **Dynamics (Planar, No Gravity):**
    *   Inverse Dynamics (ID) using Recursive Newton-Euler algorithm.
    *   Forward Dynamics (FD) using mass matrix and Coriolis/centrifugal terms.
    *   Physics integration using `scipy.integrate.solve_ivp`.
    *   **Note:** Gravity is explicitly ignored as the robot operates on a horizontal plane.
*   **Visualization:**
    *   Static plotting with `matplotlib`.
    *   Interactive `PyQt6` GUI: pose by joint sliders (FK) or click/drag the tip (IK).
*   **Quality Assurance:** Fully typed, tested with `pytest` and `hypothesis`, and linted with `ruff`.

## Getting Started

### Prerequisites

*   Python 3.12 or higher.
*   `uv` package manager (recommended) or standard `pip`.

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/hrshtst/skelarm.git
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

You can define robot configurations in TOML files. See `examples/simple_robot.toml`, `examples/four_dof_robot.toml`, or `examples/base_offset_robot.toml` (which sets a non-zero `base_length`).

```toml
[skeleton]
base_length = 0.0  # Optional fixed base link offsetting the first joint along +x

[[skeleton.link]]
length = 1.0
mass = 2.0
inertia = 0.5
com = [0.5, 0.0]        # Center of mass [x, y] relative to joint
limits = [-180.0, 180.0]  # Joint limits [min, max] in degrees

[[skeleton.link]]
length = 0.8
# ...

[initial]
q = [30.0, 0.0]   # Initial joint angles (degrees), one per joint
# dq = [0.0, 0.0]  # Optional initial joint velocities (deg/s)
```

Skeleton keys live under the `[skeleton]` table so the robot can later share a single combined file with `[task]` / `[controller]` sections for reproducible runs; `Skeleton.from_toml` reads only the `[skeleton]` section and ignores the rest. (Legacy flat configs with top-level `base_length` / `[[link]]` are still accepted.)

The initial state is a *run condition* in its own `[initial]` section, so you can compare the same robot from different starting postures by swapping just that block. `q` (degrees) and the optional `dq` (degrees/second) each take one value per joint, and a length mismatch with the robot's DOF raises an error. `[initial].q` supersedes the older per-link `q0` keys.

Joint `limits` are enforced: setting joint angles clamps them into each joint's `[min, max]` range and warns when a value is out of range. A link that omits `limits` defaults to `[-180, 180]` degrees, i.e. one full revolution — so an unspecified joint is capped at ±180°, not left unbounded.

Load it using `Skeleton.from_toml`:

```python
from skelarm import Skeleton
skeleton = Skeleton.from_toml("path/to/robot.toml")
```

For the full per-link key reference and the `[initial]` schema, see the
[Robot Configuration](guides/robot_configuration.md) guide.

### 4-DOF Simulation Example

Run a dynamic simulation of a 4-DOF robot loaded from a TOML file:

```bash
uv run python examples/simulate_four_dof.py
```

For an interactive version, launch the real-time GUI simulator and press/drag in the canvas to apply an external force at the tip (shown as a red arrow):

```bash
uv run python examples/interactive_dynamics.py
```

To simulate an arbitrary robot, use the generalized tool in `tools/`, which adds pause/resume, single-step, reset, a live viscous-friction control (joint damping that dissipates energy), a status panel (kinetic energy and tip position/speed), and an optional tip-trajectory plot when the window closes:

```bash
uv run python tools/dynamics_simulator.py examples/four_dof_robot.toml
```

Like the kinematics inspector it accepts `--show-com`, `--pose`, and `--initial`, plus `--stiffness <N/m>`, `--friction <N·m·s/rad>`, and `--no-plot`.

### Replaying a Recorded Run

The dynamics simulator records the run (joint angles, velocities, torque, and the external tip force) and can export it as a `*.sklog.npz` state log (recording is on by default; use the **Export…** button). Replay it later — the motion is reconstructed and animated from the log *without* re-running the dynamics — and plot any channel versus time for analysis:

```bash
uv run python tools/player.py run.sklog.npz
```

Scrub the timeline, play/pause at a chosen speed (`--speed`), toggle the centers of mass (`--show-com`), and open per-channel plots with the **Plot channels…** button. When the log recorded an external tip force, it is drawn as a red arrow at the tip (toggle it with **Show external force**). The log embeds the robot geometry, so the file replays on its own.

### Reaching & Trajectory Control

A combined scenario file describes the robot (`[skeleton]` / `[initial]`), the reaching goal (`[task]`), and the controller (`[controller]`) in one place. `examples/reaching.py` runs a scripted reach and plots it; `tools/reaching_simulator.py` opens an interactive GUI where the controller drives the arm to the target (a purple marker) and you press/drag the left mouse button to apply an external force at the tip. Add `--save PATH` to run headlessly and write a `*.sklog.npz` for replay (handy for batch comparison):

```bash
uv run python examples/reaching.py                            # scripted reach + plot
uv run python tools/reaching_simulator.py examples/reach.toml              # interactive GUI
uv run python tools/reaching_simulator.py examples/reach.toml --save reach.sklog.npz   # headless batch
uv run python tools/player.py reach.sklog.npz                 # replay a saved run
```

The `[controller].type` selects the control law — trajectory tracking (`computed_torque`, `inverse_dynamics_pd`, `joint_pd`), human-like reaching (`virtual_spring_damper`, `time_varying_stiffness`, `online_shaping`, `position_dependent_shaping`, `adaptive_shaping`), or `mpc` — and the remaining keys are its gains. The control library is also usable directly via `skelarm.load_scenario` and `skelarm.run_scenario`. The exported log embeds the full scenario config, so a run can be re-simulated later with `skelarm.rerun_log` (exactly for the deterministic controllers, within a small tolerance for MPC), or exported as an editable config (`skelarm.export_scenario_toml`, or `tools/export_config.py`) to tweak a parameter and re-run for comparison. See the [Control Configuration](guides/control_configuration.md) guide for the full scenario schema and per-controller config keys, and the [Trajectory Tracking](reference/07_control.md) and [Reaching Control](reference/08_reaching_control.md) references for the theory.

### Interactive Kinematics (FK & IK)

Launch the PyQt6 GUI to pose a robot arm with the joint sliders (forward kinematics) or by clicking/dragging in the canvas to move the tip (inverse kinematics):

```bash
uv run python examples/interactive_kinematics.py
```

To inspect an arbitrary robot, use the generalized tool in `tools/`, which takes a config path:

```bash
uv run python tools/kinematics_inspector.py examples/four_dof_robot.toml
```

It also accepts `--method <ik-method>`, `--show-com`, and an initial pose via `--pose 20,45,60,30` (degrees) or `--initial pose.toml` (a TOML file with an `[initial]` table).

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

We use `ruff` for linting and formatting, and `basedpyright` and `mypy` for static type checking.

*   **Linting:** `make lint`
*   **Formatting:** `make format`
*   **Type Checking:** `make type-check`
*   **Run all checks:** `make all`

## Documentation

The project documentation is built using MkDocs.

*   **Build Documentation:** `make docs-build`
*   **Serve Documentation Locally:** `make docs-serve`

## License

GPLv3

## AI Assistance & Development Workflow

This project is developed with the assistance of AI coding agents. The AI is also used to generate commit messages and parts of the documentation, including API and theoretical reference sections.

**Workflow:**
1.  **Context & Theory (Human):** The maintainer, **[Hiroshi Atsuta](https://github.com/hrshtst)**, establishes the project guidance in `AGENTS.md` and writes the theoretical background implemented as documentation in [docs/reference/](https://github.com/hrshtst/skelarm/tree/main/docs/reference).
2.  **Scaffolding (AI):** The AI assistant uses these documents and the constraints defined in `AGENTS.md` to implement code scaffolding and initial logic.
3.  **Review & Revision (Human):** The maintainer reviews, tests, and revises the generated code to ensure quality and correctness. This cycle is repeated during the development.

**Responsibility:**
All responsibilities for the code hosted in this repository lie with the maintainer. The AI serves strictly as an implementation assistant; final architectural decisions and code quality are human-led.

**Feedback:**
If you identify problems, or find code that appears to be unoriginal or rights-protected, please notify the maintainer immediately by filing an issue.

**Contributor Policy:**
External contributors are welcome to use AI tools for assistance, provided they adhere to the same standard of review and responsibility. If you use AI to generate code for a Pull Request, please disclose it in the PR description and ensure you have thoroughly reviewed and tested the code.
