# Tool and CLI Reference

Every interactive tool lives in `tools/` and is launched as
`uv run python tools/<tool>.py <config.toml> [flags]`. This page is the compact
catalogue; each tool's workflow has its own guide.

## The tools

| Tool | Purpose | Guide |
| --- | --- | --- |
| `kinematics_inspector.py` | Pose a robot with sliders (FK) or by dragging the tip (IK). | [Kinematics and Posing](kinematics_and_posing.md) |
| `dynamics_simulator.py` | Free dynamics under drag forces and viscous friction. | [Simulate Dynamics](simulate_dynamics.md) |
| `reaching_simulator.py` | Reaching scenario GUI / headless batch runs. | [Run Controlled Scenarios](running_scenarios.md) |
| `multi_target_simulator.py` | Reaching with several switchable targets. | [Run Controlled Scenarios](running_scenarios.md) |
| `periodic_curve_simulator.py` | Trace a closed task-space curve. | [Run Controlled Scenarios](running_scenarios.md) |
| `trajectory_tracking_simulator.py` | Track a recorded tip / per-joint reference. | [Teach and Track Trajectories](teaching_trajectories.md) |
| `trajectory_recorder.py` | Teach a trajectory by dragging the tip. | [Teach and Track Trajectories](teaching_trajectories.md) |
| `player.py` | Replay, plot, and export a saved log. | [Record, Replay, and Re-simulate](recording_replay.md) |
| `export_config.py` | Write a log's embedded scenario config to TOML. | [Record, Replay, and Re-simulate](recording_replay.md) |

## Shared flags

| Flag | Tools | Meaning |
| --- | --- | --- |
| `--pose 20,45,…` | all simulators, inspector, recorder | Start joint angles in degrees (one per joint). |
| `--initial FILE` | same | Start pose/velocity from a TOML `[initial]` table. |
| `--show-com` | inspector, dynamics, player | Draw the link centers of mass. |
| `--run` | all simulators | Start simulating immediately (windows open paused by default). |
| `--no-joint-limits` | dynamics, scenario tools, recorder (`dynamics` mode) | Drop the dynamics hard stop; limits stay on the kinematics. The resolved choice is recorded in the log. |
| `--task FILE` / `--controller FILE` | scenario tools | Override the named section from a separate file. |
| `--save PATH` | scenario tools | Headless run (no GUI); write the log directly. |
| `--stiffness N` | dynamics, scenario tools, recorder | Spring constant (N/m) of the mouse drag force. |
| `--friction C` | dynamics, recorder | Viscous joint damping (N·m·s/rad). |
| `--method NAME` | inspector, recorder (`ik` mode) | Numerical IK method. |
| `--speed` / `--fps` / `--export PATH` | player | Playback speed, export frame rate, headless video/GIF export. |
| `--sample-rate` / `--duration` | recorder | Teaching logger configuration. |
| `--no-plot` | dynamics, recorder | Skip the plot shown when the window closes. |

## Keyboard shortcuts

Every transport-bar window (all simulators, the recorder, and the player):

| Key | Action |
| --- | --- |
| `Space` | Play / pause |
| `→` / `F` | Single step (simulators) / next frame (player), while paused |
| `R` | Reset the simulation (pauses and zeroes the clock); in the inspector: reset the pose |
| `Q` | Close the window |

Tool-specific:

| Key | Tool | Action |
| --- | --- | --- |
| `←` / `B` | player | Previous frame, while paused |
| `Home` / `End` | player | Jump to the first / last frame (`R` also returns to start) |
| `F` | recorder | Finish the recording |
| `1` … `9` | multi-target simulator | Switch the active target live |

## Timing

The interactive simulators integrate at the scenario's `[simulator].dt`
(default 5 ms standalone): each ~20 ms render tick runs as many physics substeps
of `dt` as fill it, and a `dt` above the frame period slows the render timer to
match, so wall clock tracks simulated time. A paused single step advances one
render tick (substeps × `dt`).
