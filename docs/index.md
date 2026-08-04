# skelarm

A lightweight, physics-based dynamics simulator for a configurable planar robot
arm. `skelarm` treats the robot as a "skeleton" of N links and focuses on
kinematics and dynamics — there is no collision detection or detailed shape
rendering. The supported model is a **horizontal, gravity-free plane**: it
models a planar arm lying flat.

What it does, in one pass: define a robot in TOML, pose it interactively (FK/IK),
simulate its dynamics under torque control, run controlled tasks (reaching,
curve tracing, trajectory tracking, MPC) from a single scenario file, and record
every run to a self-contained log that replays, plots, exports to video, and —
for headless runs — re-simulates reproducibly.

<video autoplay controls loop muted playsinline width="640" src="assets/reach.mp4"></video>

## Where to go

- **[Getting Started](getting_started.md)** — install and drive your first robot
  in five minutes.
- **User Guides** — task-oriented walkthroughs:
  [configure a robot](guides/robot_configuration.md),
  [pose and inspect it](guides/kinematics_and_posing.md),
  [simulate dynamics](guides/simulate_dynamics.md),
  [run controlled scenarios](guides/running_scenarios.md),
  [record and replay](guides/recording_replay.md),
  [teach trajectories](guides/teaching_trajectories.md),
  [joint limits](guides/joint_limits.md), and the
  [tool & CLI reference](guides/tools_reference.md).
- **Developer Guides** — use and extend the library from Python:
  [API quick start](guides/python_api.md),
  [define a task](guides/defining_a_task.md),
  [define a controller](guides/defining_a_controller.md),
  [architecture](api/architecture.md), and
  [development & testing](guides/development.md).
- **[Theory Reference](reference/index.md)** — the mathematics behind the
  implementation, from forward kinematics to reaching control.
- **API Reference** — per-module API docs generated from the source (the
  [Architecture](api/architecture.md) overview lives under Developer Guides).

## Capabilities at a glance

- **Configurable robot** — arbitrary planar chains from TOML: lengths, masses,
  inertias, centers of mass, joint limits, optional base offset.
- **Kinematics** — recursive FK for positions/velocities/accelerations, endpoint
  Jacobian, and numerical IK (Sugihara-style Levenberg-Marquardt and friends).
- **Dynamics** — Recursive Newton-Euler inverse dynamics, mass-matrix forward
  dynamics, fixed-step semi-implicit Euler and adaptive `solve_ivp` integration.
- **Control** — trajectory-tracking laws, human-like reaching controllers, and
  joint-space MPC, all configured from one scenario TOML and extensible at
  runtime.
- **Recording & replay** — self-contained `*.sklog.npz` logs with an analysis
  player, headless `.mp4`/`.gif` export, and reproducible headless re-simulation.
- **Quality** — fully typed, tested with `pytest` + `hypothesis` property tests,
  linted with `ruff`.
