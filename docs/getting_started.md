# Getting Started

Five minutes from clone to a controlled reach.

## Prerequisites

- Python 3.12 or higher.
- The [`uv`](https://docs.astral.sh/uv/) package manager (recommended; plain
  `pip install .` also works).

## Install

```bash
git clone https://github.com/hrshtst/skelarm.git
cd skelarm
uv sync
```

## 1. Pose a robot (kinematics)

Open the kinematics inspector on a bundled 4-DOF arm — move the joint sliders
(forward kinematics) or click/drag the tip in the canvas (inverse kinematics):

```bash
uv run python tools/kinematics_inspector.py examples/four_dof_robot.toml
```

## 2. Simulate its dynamics

Launch the real-time dynamics simulator. The window opens **paused**: press
`Space` (or the play button) to start, then press and drag in the canvas to pull
the tip with a spring force, drawn as a red arrow:

```bash
uv run python tools/dynamics_simulator.py examples/four_dof_robot.toml
```

## 3. Run a controlled reach

Run a complete scenario — robot, start pose, task, and controller from one TOML
file. The controller drives the arm to the purple target; drag the tip to
disturb it and watch it recover:

```bash
uv run python tools/reaching_simulator.py examples/reach.toml
```

<video controls loop muted playsinline width="640" src="../assets/reach.mp4"></video>

Every simulator window shares the same transport bar (`Space` play/pause, `→`
step while paused, `R` reset, `Q` quit) and records the run — press **Export…**
to save a `*.sklog.npz` log, then replay it:

```bash
uv run python tools/player.py reach.sklog.npz
```

## Where next

- [Configure a robot](guides/robot_configuration.md) of your own.
- [Run controlled scenarios](guides/running_scenarios.md) — the scenario file
  model, tasks, and controllers.
- [Record, replay, and re-simulate](guides/recording_replay.md) — what the logs
  capture and what "reproducible" means.
- [Python API quick start](guides/python_api.md) — use the library from code.
