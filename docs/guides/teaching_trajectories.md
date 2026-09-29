# Teach and Track Trajectories

Author a joint trajectory by demonstration, then have a controller track it.

## Teach by demonstration

Grab the robot's tip with the left mouse button and drag it to teach a motion,
recorded to a `*.sklog.npz` log. Press **Space** to start a take from the reset
posture, guide the tip, then press **S** to save (or **Shift+S** to save and
prepare the next take); `--plot` shows a plot of the last visible take after the
window closes:

```bash
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml                 # ik mode (default)
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --mode dynamics  # force + forward dynamics
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --output reach.sklog.npz --multi-take
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --multi-take --show-tip-trail --show-past-trails
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --multi-take --show-tip-trail --show-past-trails --past-trail-history last
uv run python tools/player.py reach_001.sklog.npz                                        # replay a take
```

### Controls

| Key | Action | Result |
| --- | --- | --- |
| Space | Start | Start a take from the ready (reset) posture; the reset state is logged at `t = 0` and the still pre-roll is recorded until you move. A repeated or held Space changes nothing. |
| S | Save | Stop and save the take, keeping it visible; no reset. |
| Shift+S | Save and next take | Save, then reset posture, velocity, drag state, log, and clock, and wait for Space. An already-saved take is not written again. |
| R | Reset | Discard an *unsaved* take (no file, no take number consumed); a saved take keeps its file. Reset and wait for Space. **S then R** equals **Shift+S**. |
| Q | Close | Warn when unsaved samples exist: *Save and close* / *Discard and close* / *Cancel* (default; dismissing the dialog cancels). Empty or saved takes close silently, writing nothing. |

Each key has a matching button with the shortcut in its label; the shortcuts
work with the focus on the drawing canvas and never auto-repeat. There is no
Finish action any more: a `--duration` cap stops **and saves** the take but
leaves the window open, and `--start-on-grab` restores the legacy start on the
first grab. Saving never opens a dialog or a plot.

### Outputs

`--output` names one exact file; a second take of the same session is then
refused rather than overwriting it. A name without the `.npz` suffix gets it
appended, as NumPy would, so the file that is checked is the file that is
written. With `--multi-take` the name is the base of
numbered takes (`reach.sklog.npz` → `reach_001.sklog.npz`, `reach_002.sklog.npz`,
…); numbering continues after any file of that base already present. An existing
file is never overwritten: the save is refused, reported in the status area and
on the terminal, and the take stays for a retry. Each take is written to a
temporary file beside its target and published only once complete, by a hard
link that refuses an existing file, so a failed write leaves nothing behind. On a
file system without hard links (FAT/exFAT, some network mounts) the recorder
instead reserves the name by creating it exclusively, which refuses an existing
file just the same, and replaces that empty reservation with the complete take.
A take holding nothing beyond the `t = 0` frame is never written and consumes no
number. File names enumerate
attempts; whether a take qualifies for an experiment is decided offline.

### Trails

`--show-tip-trail` draws the current take's tip path while you record, built
from the logged (forward-kinematics) tip samples rather than the cursor path, so
what you see is exactly what the log holds. `--show-past-trails` keeps the tip
paths of the takes saved in this session as faint, transparent lines behind the
current one, so later takes can follow earlier ones. Both overlays are also
checkboxes in the side panel and can be hidden independently; toggling them
never moves the robot or changes a logged sample.

`--past-trail-history` chooses which saved takes the faint overlay draws: `all`
(the default) draws every take saved in this session, `last` only the most
recently saved one, so a session shows at most one faint trail behind the
current one. Only the drawing differs: every take is still saved to its file and
enters the session history the same way in both modes.

Only saved takes enter the history, once each: **S** then **R** and
**Shift+S** leave the same history, **R** drops an unsaved trail together with
its take, and pressing **R** while already ready changes nothing. A new session
always starts with an empty history, whatever files already exist on disk, so a
practice session leaves no traces in a later one. Each saved log records under
`[extra.display]` the overlay settings, the display-history mode
(`past_trail_history`), the color/opacity policy, the takes in the history
(`history_takes`), and the saved takes that were actually drawn while that take
was recorded (`visible_source_takes` / `visible_source_files`: every earlier
saved take with `all`, at most the last one with `last`, and none for the first
take of a session or when the faint overlay stayed hidden).

### Acquisition clock

The recorder runs one timer tick per sample period, so `--sample-rate` must give
a whole number of milliseconds (100, 50, 25, 20, 10 Hz, …). Every tick performs
exactly one pose update (one IK solve toward the current cursor, or the dynamics
substeps) and records exactly one sample. Starting a take restarts the timer, so
the first sample follows the `t = 0` frame by one full period (with
`--start-on-grab`, the tick that sees the grab only logs `t = 0`). The sample's
`time` is the actual elapsed time since the take started, read from the wall
clock when the tick's pose update begins, so a late timer tick shows up as a
longer interval rather than being hidden; the nominal tick clock `k × period` is
kept beside it as the `nominal_time` channel. No sample is ever duplicated or invented, and the time
spent in the unsaved-take warning is excluded from both clocks. The realized tick
spacing of each take (mean, maximum, and the count of ticks slower than 1.5 × the
period) is also summarized under `[extra.acquisition]` in the log, so the achieved
rate can be verified after a session. The display repaints at most every 20 ms,
independently of sampling.

<video controls loop muted playsinline width="640" src="../../assets/teach_mouse.mp4"></video>

*A teaching session (screen-recorded with the pointer): the tip is grabbed and
guided toward the target while the recorder samples the motion.*

Two modes turn the task-space teaching into joint angles:

- **`ik`** — the tip tracks the cursor via the IK solver (`--method`); joint
  angles come from clamped kinematic posing.
- **`dynamics`** — the drag applies a tip force integrated under forward
  dynamics with viscous friction (`--stiffness`, `--friction`, and
  `--no-joint-limits` to drop the dynamics hard stop).

`--sample-rate` and `--duration` configure the logger (a `--duration` of `0` or
less drops the time cap and records until you save); `--initial` / `--pose`
set the start pose, which is also the reset posture of every take, and an
optional `[task]` in the config draws a target. The log records the per-joint
angles and the tip path (plus `dq`, the tip force, and the friction in
`dynamics` mode).

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
