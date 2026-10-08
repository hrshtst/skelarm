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
| `player.py` | Replay, plot, and export a saved log; several logs open a playlist. | [Record, Replay, and Re-simulate](recording_replay.md) |
| `export_config.py` | Write a log's embedded scenario config to TOML. | [Record, Replay, and Re-simulate](recording_replay.md) |

## Shared flags

| Flag | Tools | Meaning |
| --- | --- | --- |
| `--pose 20,45,…` | all simulators, inspector, recorder | Start joint angles in degrees (one per joint). |
| `--initial FILE` | same | Start pose/velocity from a TOML `[initial]` table. |
| `--show-com` | inspector, dynamics, recorder, player | Draw the link centers of mass. |
| `--run` | all simulators | Start simulating immediately (windows open paused by default). |
| `--no-joint-limits` | dynamics, scenario tools, recorder (`dynamics` mode) | Drop the dynamics hard stop; limits stay on the kinematics. Scenario tools record the resolved choice in the log's run metadata; the dynamics simulator and recorder apply it without embedding it. |
| `--task FILE` / `--controller FILE` | scenario tools | Override the named section from a separate file. The recorder and the inspector also take `--task FILE`, only to draw it. |
| `--save PATH` | scenario tools | Headless run (no GUI); write the log directly. |
| `--duration S` | scenario tools (with `--save`), recorder | Override the task's simulated duration / cap the recording length (recorder: a reached cap stops and saves the take but keeps the window open; `0` or negative records until you save). |
| `--show-tip-trail` / `--show-past-trails` | recorder | Draw the current take's logged tip path / the faint tip paths of the takes saved in this session (also checkboxes in the panel). |
| `--past-trail-history {all,last}` | recorder | Which saved takes the faint overlay draws: every take saved in this session (`all`, default) or only the most recently saved one (`last`); saved files and the history are unchanged. |
| `--multi-take` / `--start-on-grab` | recorder | Number the outputs from the `--output` base / start recording on the first grab instead of Space. |
| `--output PATH` | recorder, `export_config.py` (`-o`) | Output file path (`teach.sklog.npz` / the log path with `.toml` by default). |
| `--stiffness N` | dynamics, scenario tools, recorder | Spring constant (N/m) of the mouse drag force. |
| `--friction C` | dynamics, recorder | Viscous joint damping (N·m·s/rad). |
| `--method NAME` | inspector, recorder (`ik` mode) | Numerical IK method. |
| `--speed` / `--fps` / `--export PATH` | player | Playback speed, export frame rate, headless video/GIF export. |
| `--panel` | player | Composite a simulator-style side panel (time, sliders, parameter readouts) into the `--export` frames. |
| `--sample-rate HZ` | recorder | Requested teaching logger sampling rate (best effort; the achieved rate is reported on save). |
| `--plot` | dynamics, recorder | Plot the tip trajectory / recorded motion when the window closes (off by default). |

## Keyboard shortcuts

Every transport-bar window (all simulators and the player — the trajectory
recorder has no transport bar; its own keys are listed below):

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
| `N` / `P` | player (playlist) | Load the next / previous log of the playlist |
| `Space` | recorder | Start a take from the reset posture (`--start-on-grab` starts on the first grab instead) |
| `S` / `Shift+S` | recorder | Save the take and keep it visible / save and prepare the next take |
| `R` | recorder | Reset; discards only an unsaved take |
| `Q` | recorder | Close, warning first when unsaved samples exist |
| `1` … `9` | multi-target simulator | Switch the active target live |

## Timing

The interactive simulators integrate at the scenario's `[simulator].dt`
(default 5 ms standalone): each ~20 ms render tick runs as many physics substeps
of `dt` as fill it, and a `dt` above the frame period slows the render timer to
match, so wall clock tracks simulated time. A paused single step advances one
render tick (substeps × `dt`).
