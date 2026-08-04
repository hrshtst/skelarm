# Run Controlled Scenarios

A **scenario** is one TOML file that describes a complete controlled run: the
robot, its start state, the task, the integration settings, and the controller.
Load it in an interactive GUI, run it headlessly, or drive it from Python.

```bash
uv run python tools/reaching_simulator.py examples/reach.toml             # interactive GUI (drag to perturb)
uv run python tools/reaching_simulator.py examples/reach.toml --save reach.sklog.npz   # headless batch run + log
uv run python tools/player.py reach.sklog.npz                # replay and analyze a saved run
```

<video controls loop muted playsinline width="640" src="../../assets/reach_disturb.mp4"></video>

*Disturbing an interactive reach: the tip is dragged off the target and the
controller pulls it back — the drag force is recorded and drawn in the replay.*

## The five-table model

| Section | Purpose | Loader |
| --- | --- | --- |
| `[skeleton]` | Robot geometry (links, base length, limits) | `Skeleton.from_toml` |
| `[initial]` | Start pose / velocity (degrees) | applied by `Skeleton.from_toml` |
| `[task]` | The goal and how the motion is shaped | `Task.from_toml` |
| `[simulator]` | How the dynamics are integrated (`dt`, `enforce_limits`) | `Simulator.from_dict` |
| `[controller]` | The control law and its gains | `build_controller` |

A minimal complete scenario:

```toml
[skeleton]
base_length = 0.0
[[skeleton.link]]
length = 1.0
mass = 1.0
inertia = 0.1
com = [0.5, 0.0]
limits = [-180.0, 180.0]
[[skeleton.link]]
length = 0.8
mass = 0.8
inertia = 0.05
com = [0.4, 0.0]
limits = [-180.0, 180.0]

[initial]
q = [34.4, 57.3]        # degrees

[task]
type = "reaching"       # the task kind (required)
target = [0.55, 1.21]    # endpoint goal (x, y) in meters (required for reaching)
duration = 2.0
schedule = "minimum_jerk"

[simulator]
dt = 0.002              # control / integration step
enforce_limits = true   # joint-limit hard stop in the dynamics

[controller]
type = "computed_torque"
kp = 200.0
kd = 30.0
```

Swap the `[controller]` block to try a different law — for example a compliant,
human-like reach:

```toml
[controller]
type = "adaptive_shaping"
k_task = 150.0
d_task = 25.0
t_adapt = 5.0
```

Every `[task]` / `[simulator]` / `[controller]` key — the five built-in task
types, all controllers and their gains, curve parameters, and reference
filtering — is listed in the [Control Configuration](control_configuration.md)
reference.

## The scenario simulators

Each built-in task type has a dedicated interactive simulator, all sharing the
same controls (transport bar, drag-to-perturb, **Record** / **Export…**, the
override flags below, and `--save` for a headless batch run):

```bash
uv run python tools/reaching_simulator.py examples/reach.toml                   # reach a target
uv run python tools/multi_target_simulator.py examples/multi_target.toml        # several targets; press 1–N to switch live
uv run python tools/periodic_curve_simulator.py examples/periodic_curve.toml    # trace a closed curve
uv run python tools/trajectory_tracking_simulator.py track.toml                 # track a recorded tip / per-joint reference
```

Their runs replay in `tools/player.py` with the task overlay (target, curve, or
reference) drawn — see
[Record, Replay, and Re-simulate](recording_replay.md).

<video controls loop muted playsinline width="640" src="../../assets/multi_target_switch.mp4"></video>

*Live target switching: number keys retarget the controller mid-run; the
recorded active-target index drives both the overlay emphasis and the panel row
in this replay.*

## Overriding sections for comparison

The scenario tools can override the `[initial]`, `[task]`, and `[controller]`
sections from separate files, so one base config can be reused across a
comparison sweep without editing it. Each override file supplies the named table
(e.g. a file with just a `[controller]` block). With `--save PATH` the run is
headless and the log is written directly, which is convenient for a scripted
sweep:

```bash
# Same robot and task, different controllers:
uv run python tools/reaching_simulator.py base.toml --controller computed_torque.toml --save ct.sklog.npz
uv run python tools/reaching_simulator.py base.toml --controller mpc.toml             --save mpc.sklog.npz

# Same controller, different tasks:
uv run python tools/reaching_simulator.py base.toml --task near.toml --save near.sklog.npz
uv run python tools/reaching_simulator.py base.toml --task far.toml  --save far.sklog.npz
```

Without `--save`, the same overrides configure the interactive GUI instead.
`--initial FILE` replaces the initial pose from a file's `[initial]` table, and
`--pose 20,45` then overrides just the joint angles (degrees) — matching the
kinematics and dynamics tools. The override values are merged into the scenario,
so each saved run's log still embeds its exact (overridden) config for
reproduction.

## From Python

```python
from skelarm import load_scenario, run_scenario

scenario = load_scenario("examples/reach.toml")
log = run_scenario(scenario)  # duration from [task], dt from [simulator]
log.save("reach.sklog.npz")  # replay/analyze with tools/player.py
```

`run_scenario` runs the fixed-step control loop (like `simulate_controlled`) but
also embeds the scenario in the log for later reproduction. `build_controller`
can also be called directly with a `[controller]` mapping, a `Task`, and a
`Skeleton`, so controllers can be constructed without a file — see the
[Python API quick start](python_api.md).

## Related

- [Control Configuration](control_configuration.md) — the full key-by-key
  schema reference.
- [Record, Replay, and Re-simulate](recording_replay.md) — logs, playback, and
  the reproducibility tiers.
- [Joint Limits](joint_limits.md) — what `enforce_limits` and
  `--no-joint-limits` do.
- [Defining a Task](defining_a_task.md) / [Defining a Controller](defining_a_controller.md)
  — extend the built-in types.
