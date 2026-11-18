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
uv run python tools/trajectory_recorder.py examples/multi_target.toml --prepare --multi-take --output reach.sklog.npz
uv run python tools/player.py reach_001.sklog.npz                                        # replay a take
uv run python tools/player.py reach_*.sklog.npz                                          # replay every take as a playlist
```

### Grabbing the tip

A left press grabs the tip only within `--grab-radius` of it (5 cm by default),
shown as a dashed circle around the tip while nothing is grabbed; a press farther
away grabs nothing. In `ik` mode the tip then moves as the cursor moves from where
you pressed, so grabbing slightly off the tip's center never makes the tip jump to
the cursor, and the take holds no such jump. In `dynamics` mode the spring pulls the
tip toward the cursor itself.

```bash
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --grab-radius 0.03   # grab only within 3 cm
```

### Controls

| Key | Action | Result |
| --- | --- | --- |
| Space | Start | Start a take from the ready (reset) posture; the reset state is logged at `t = 0` and the still pre-roll is recorded until you move. A repeated or held Space changes nothing. |
| S | Save | Stop and save the take, keeping it visible; no reset. |
| Shift+S | Save and next take | Save, then reset posture, velocity, drag state, log, and clock, and wait for Space. An already-saved take is not written again. |
| R | Reset | Discard an *unsaved* take (no file, no take number consumed); a saved take keeps its file. Reset and wait for Space. **S then R** equals **Shift+S**. |
| E | Prepare (`--prepare`) | Enter the preparation phase from the ready state or after a saved take; press again to make the posed arm the ready posture. Refused during a take or while a take is not saved. |
| 1 … 9 | Choose the target (`--prepare`) | Make that target of a `multi_target_reaching` task the active one; only while preparing. |
| Q | Close | Warn when unsaved samples exist: *Save and close* / *Discard and close* / *Cancel* (default; dismissing the dialog cancels). Empty or saved takes close silently, writing nothing. |

Each key has a matching button with the shortcut in its label; the shortcuts
work with the focus on the drawing canvas and never auto-repeat. There is no
Finish action any more: a `--duration` cap stops **and saves** the take but
leaves the window open, and `--start-on-grab` restores the legacy start on the
first grab. Saving never opens a dialog or a plot.

### Preparing each take

With `--prepare`, **E** switches between recording and a preparation phase that sets
the next take's start posture and, for a `multi_target_reaching` task, its target, so a
session can record many takes from many starts:

```bash
uv run python tools/trajectory_recorder.py examples/multi_target.toml --prepare --multi-take --output reach.sklog.npz
```

E enters the phase from the ready state, or after the visible take is saved; the arm
then stays where that take ended, so the next start can be posed from there. While
preparing, grab the tip and drag it to pose the arm: inverse kinematics moves it, in
both modes, at rest. Nothing is logged, the clock does not run, and Space, S, and
Shift+S do nothing. The keys **1** to **9**, or the target buttons in the side panel,
choose the active target; outside the phase they do nothing, so a take's target never
changes during it. **E** again ends the phase: the posed arm becomes the ready posture
of the following takes, and **R** and **Shift+S** return to it until the next
preparation. **R** during the phase cancels it, back to the previous ready posture and
target. During a take, or while a take is not saved, E is refused until you save (S) or
discard (R) it. The **Prepare (E)** button is pressed while the phase is on, and the
status names the phase and the active target.

Each take of a multi-target task stores the task with `active` set to the target that
was active during it, and its first sample is the prepared posture. The player draws
the targets with that one active, and a script reads it back:

```python
from skelarm import StateLog, Task, active_target_index, multi_target_specs

log = StateLog.load("reach_001.sklog.npz")
task = Task.from_dict(log.extra["playback"]["task"])
index = active_target_index(task)
pos, label, color, tolerance = multi_target_specs(task)[index]
start = log.channel("q")[0]  # the prepared start posture
```

### Outputs

`--output` names one exact file; a second take of the same session is then
refused rather than overwriting it. A name without the `.npz` suffix gets it
appended, as NumPy would, so the file that is checked is the file that is
written. With `--multi-take` the name is the base of
numbered takes (`reach.sklog.npz` → `reach_001.sklog.npz`, `reach_002.sklog.npz`,
…); numbering continues after any file of that base already present, and a
number taken meanwhile (e.g. by another session) is skipped, never overwritten.
In single-file mode an existing file is never overwritten either: the save is
refused, reported in the status area and on the terminal, and the take stays for
a retry. Each take is written straight to its name, and a failed write removes
the partial file. A take holding nothing beyond the `t = 0` frame is never
written and consumes no number. File names enumerate attempts; whether a take
qualifies for an experiment is decided offline. When the config (or `--task`) has a
`[task]`, each take stores it under `[extra.playback.task]`, so the player draws the
target when replaying or exporting the take. A `multi_target_reaching` task draws every
target, the active one emphasized: the configuration's `active`, until a preparation
chooses another.

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
practice session leaves no traces in a later one. The overlays are a drawing aid
only and never enter the saved log.

### Acquisition clock

`--sample-rate` is a best-effort request. The recorder runs one timer tick per
sample period (rounded to whole milliseconds), and every tick performs one pose
update (one IK solve toward the current cursor, or the dynamics substeps) and
records one sample. The sample's `time` is the real elapsed time since the take
started, so a late tick shows up as a longer interval and a replay follows the
motion as you performed it, even when you request more than the machine can
keep up with. Dynamics mode simulates that same elapsed time in fixed
substeps, capped at a few periods after a stall. Starting a take restarts the
timer, so the first sample follows the `t = 0` frame by one full period (with
`--start-on-grab`, the tick that sees the grab only logs `t = 0`); the time
spent in the unsaved-take warning is excluded. Each saved log records the
requested and achieved rates under `[extra.acquisition]`, and the save message
prints them. The display repaints about every 20 ms, independently of sampling.

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
set the start pose, which is also the reset posture of every take until a
preparation (`--prepare`) poses another, and an optional `[task]` in the config
draws a target. The log records the per-joint
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

*The taught motion tracked by computed torque: the gray overlay is the demonstrated
tip path as recorded (the smoothing applies only to the controller's reference),
and the target marker from the teaching scenario is kept.*

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
