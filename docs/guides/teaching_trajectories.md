# Teach and Track Trajectories

Author a joint trajectory by demonstration, then have a controller track it.

## Teach by demonstration

Grab the robot's tip with the left mouse button and drag it to teach a motion,
recorded to a `*.sklog.npz` log. Recording starts on the first grab and stops at
the max duration (or the **Finish** button, shortcut `F`), sampling at the
configured rate; a plot of the recorded motion is shown afterward:

```bash
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml                 # ik mode (default)
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --mode dynamics  # force + forward dynamics
uv run python tools/player.py teach.sklog.npz                                            # replay the recording
```

<video controls loop muted playsinline width="640" src="../../assets/teach_mouse.mp4"></video>

*A teaching session (screen-recorded with the pointer): the tip is grabbed and
guided toward the target while the recorder samples the motion.*

Two modes turn the task-space teaching into joint angles:

- **`ik`** — the tip tracks the cursor via the IK solver (`--method`); joint
  angles come from clamped kinematic posing.
- **`dynamics`** — the drag applies a tip force integrated under forward
  dynamics with viscous friction (`--stiffness`, `--friction`, and
  `--no-joint-limits` to drop the dynamics hard stop).

`--sample-rate` and `--duration` configure the logger; `--initial` / `--pose`
set the start pose, and an optional `[task]` in the config draws a target. The
log records the per-joint angles and the tip path.

## Track the recording

Use the log as the reference of a *trajectory-tracking* scenario — either the
recorded **tip path** converted to joint angles by IK (`trajectory_tracking`),
or the recorded **per-joint angles** directly (`joint_trajectory_tracking`):

```toml
[task]
type = "joint_trajectory_tracking"
file = "teach.sklog.npz"
filter = { kind = "butterworth", cutoff_hz = 8.0, order = 4 }  # smooth a jaggy recording
interpolator = "cubic_spline"
# duration defaults to the reference's length when omitted

[simulator]
dt = 0.002

[controller]
type = "computed_torque"
kp = 200.0
kd = 30.0
```

```bash
uv run python tools/trajectory_tracking_simulator.py track.toml
```

<video controls loop muted playsinline width="640" src="../../assets/trajectory_tracking.mp4"></video>

*The taught motion tracked by computed torque: the gray overlay is the (smoothed)
demonstrated tip path, and the target marker from the teaching scenario is kept.*

Hand-taught motions are jaggy; the `filter` table pre-smooths the reference
(zero-phase low-pass, Butterworth, moving average, or Savitzky–Golay) and the
`interpolator` resamples it — see the
[Control Configuration](control_configuration.md#task-types) reference for the
keys and [Trajectory Filtering & Interpolation](../reference/09_trajectory_filtering.md)
for the theory. The reference content is embedded in the run log, so a saved
tracking run replays and re-simulates without the original teaching file.

## Related

- [Record, Replay, and Re-simulate](recording_replay.md) — the log format and
  player.
- [Run Controlled Scenarios](running_scenarios.md) — the scenario model the
  tracking tasks plug into.
