# Simulate Dynamics

Run a robot under real physics — free motion, external tip forces, and viscous
friction — without a task or controller.

## The interactive dynamics simulator

```bash
uv run python tools/dynamics_simulator.py examples/four_dof_robot.toml
```

The window opens **paused** with a media-player transport bar (`Space`
play/pause, `→`/`F` single-step while paused, `R` reset, `Q` quit; pass `--run`
to start immediately). Press and drag in the canvas to apply a spring force at
the tip, drawn as a red arrow. The tool adds a live viscous-friction spin box
(joint damping that dissipates energy), a status panel (kinetic energy, tip
position and speed), and an optional tip-trajectory plot when the window closes.

<video controls loop muted playsinline width="640" src="../../assets/dynamics_drag.mp4"></video>

*A captured drag session replayed from its log: the red arrow is the recorded
tip force, and the panel's friction row follows the live spin-box change.*

Flags: `--show-com`, `--pose`, `--initial` (as in the
[kinematics inspector](kinematics_and_posing.md)), plus `--stiffness <N/m>` for
the drag spring, `--friction <N·m·s/rad>`, and `--no-plot`. Joint limits act as
hard stops in the dynamics by default; `--no-joint-limits` drops the hard stop
and leaves the limits on the kinematics only ([Joint Limits](joint_limits.md)).

The run is recorded (joint angles, velocities, torque, and the external tip
force); press **Export…** to save a `*.sklog.npz` log for the
[player](recording_replay.md).

## Scripted simulation

A ready-made 4-DOF example, headless and interactive:

```bash
uv run python examples/simulate_four_dof.py       # scripted run + plot
uv run python examples/interactive_dynamics.py    # minimal interactive window
```

From Python, `simulate_robot` integrates the uncontrolled dynamics with adaptive
`solve_ivp`, while `simulate_controlled` runs the fixed-step control loop — see
the [Python API quick start](python_api.md).

## What the physics is

The arm is planar and horizontal, so **gravity is zero** throughout. Torques map
to accelerations through the mass matrix and Coriolis terms assembled by
Recursive Newton-Euler; the interactive tools integrate with fixed-step
semi-implicit Euler. The theory lives in
[Inverse Dynamics](../reference/03_inverse_dynamics.md),
[Forward Dynamics](../reference/04_forward_dynamics.md), and
[Numerical Methods](../reference/05_numerical_methods.md).

## Related

- [Run Controlled Scenarios](running_scenarios.md) — add a task and controller.
- [Record, Replay, and Re-simulate](recording_replay.md) — what the exported log
  can do.
