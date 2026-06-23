# Control Configuration

A **scenario** file describes a complete controlled run in one TOML document: the
robot, its start state, the reaching task, and the controller that drives it. It
adds `[task]` and `[controller]` sections on top of the
[Robot Configuration](robot_configuration.md) (`[skeleton]` / `[initial]`), and is
loaded by `skelarm.load_scenario`.

```bash
uv run python tools/reaching_simulator.py examples/reach.toml             # interactive reach GUI (drag to perturb)
uv run python tools/reaching_simulator.py examples/reach.toml --save reach.sklog.npz   # headless batch run + log
uv run python tools/player.py reach.sklog.npz                # replay and analyze a saved run
```

A scenario combines four sections:

| Section | Purpose | Loader |
| --- | --- | --- |
| `[skeleton]` | Robot geometry (links, base length, limits) | `Skeleton.from_toml` |
| `[initial]` | Start pose / velocity (degrees) | applied by `Skeleton.from_toml` |
| `[task]` | The reaching goal and run conditions | `Task.from_toml` |
| `[controller]` | The control law and its gains | `build_controller` |

See [Trajectory Tracking Control](../reference/07_control.md) and
[Reaching Control](../reference/08_reaching_control.md) for the underlying theory.

## The `[task]` section

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `type` | str | `"reaching"` | Task kind. Only `reaching` is implemented today; more are planned. |
| `target` | `[x, y]` or table | *required* | Endpoint goal in meters (see below). |
| `duration` | float | `2.0` | Total simulated time, in seconds. |
| `dt` | float | `0.002` | Control / integration step, in seconds. |
| `schedule` | str | `"minimum_jerk"` | Time scaling for planned trajectories: `linear`, `cubic`, `quintic`, or `minimum_jerk`. |
| `enforce_limits` | bool | `true` | Apply the joint limits as hard stops in the dynamics. Set `false` to let the limits constrain only the kinematics (see below). |

`duration`, `schedule`, and `target` define the planned task-space trajectory used
by the *trajectory-tracking* controllers; the *reaching* controllers use only
`target` and generate motion online. `dt` is the fixed step of the simulation loop
([`simulate_controlled`](../reference/07_control.md)).

`enforce_limits` is a *run condition* of the simulation: with the default `true`
the fixed-step loop pins each joint at its `[qmin, qmax]` bound (a hard stop);
with `false` the bounds are dropped from the dynamics and apply only to the
kinematics (posing and inverse kinematics). Because it lives in the scenario
config, the setting is embedded in the run's reproduction metadata, so a saved log
re-runs with the same choice. `tools/reaching_simulator.py --no-joint-limits`
overrides it off for a single run (and that resolved value is what gets recorded).
See the [Joint Limits](joint_limits.md) guide for the underlying mechanics.

### Target

`target` is either a plain `[x, y]` array (position only) or a table that carries
the position plus optional attributes:

| Target key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `pos` | `[x, y]` | *required* | Endpoint position in meters. |
| `label` | str | none | Name for the target (used to pick one among several; multi-target tasks come later). |
| `color` | str | `"purple"` | Marker color (any Qt/SVG name or `#rrggbb`). |
| `tolerance` | float | none | Admittable tip-to-target distance (m). The reach is "reached" within it, and the marker's hollow ring is drawn at this radius. |

```toml
[task]
type = "reaching"
target = { pos = [0.55, 1.21], label = "goal", color = "purple", tolerance = 0.02 }
# array shorthand (position only) also works:
# target = [0.55, 1.21]
```

## The `[controller]` section

`type` selects the control law; the remaining keys are its gains (any omitted key
falls back to the default below). Diagonal PD gains (`kp`, `kd`) and task-space
gains (`k_task`, `d_task`, `c_joint`) are isotropic scalars.

### Trajectory tracking

These build a joint reference by converting the planned task trajectory with
inverse kinematics, then track it. See
[Trajectory Tracking Control](../reference/07_control.md).

| `type` | Controller | Keys (default) |
| --- | --- | --- |
| `computed_torque` | `ComputedTorque` | `kp` (200), `kd` (30) |
| `inverse_dynamics_pd` | `InverseDynamicsFeedforwardPD` | `kp` (100), `kd` (20) |
| `joint_pd` | `JointPD` | `kp` (300), `kd` (40) |

### Reaching

These are endpoint-feedback controllers that drive the tip to `target` without a
preplanned trajectory. See [Reaching Control](../reference/08_reaching_control.md).

| `type` | Controller | Keys (default) |
| --- | --- | --- |
| `virtual_spring_damper` | `VirtualSpringDamper` | `k_task` (150), `d_task` (25), `c_joint` (0) |
| `time_varying_stiffness` | `TimeVaryingStiffness` | `k0` (150), `alpha` (6), `zeta1` (0.15), `c_joint` (0) |
| `online_shaping` | `OnlineReferenceShaping` | `k_task` (150), `d_task` (25), `c_joint` (0), `r` (0.5), `t1` (0.2), `t2` (0.2) |
| `position_dependent_shaping` | `PositionDependentShaping` | `k_task` (150), `d_task` (25), `c_joint` (0), `a` (0.01), `t1` (0.2), `t2` (0.2) |
| `adaptive_shaping` | `AdaptiveReferenceShaping` | `k_task` (150), `d_task` (25), `c_joint` (0), `epsilon` (0.01), `t_adapt` (5.0), `t1` (0.2), `t2` (0.2) |

### Model predictive control

| `type` | Controller | Keys (default) |
| --- | --- | --- |
| `mpc` | `JointSpaceMPC` | `horizon` (6), `q_weight` (10), `dq_weight` (1), `tau_weight` (0.001), `terminal_weight` (50), `tau_max` (none), `limit_weight` (0), `max_iter` (20) |

MPC predicts with the simulation step, so its control interval is `[task].dt`.
Re-optimizing every step is expensive at a small `dt`; use a larger `[task].dt`
(for example `0.05`) for MPC scenarios.

!!! note "Reach time vs. settling"
    For trajectory-tracking controllers the planned motion spans `[0, duration]`,
    so the endpoint arrives at the target at `t = duration`. The reaching
    controllers converge asymptotically, so give `duration` enough margin for the
    endpoint to settle.

## Example

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
target = [0.55, 1.21]   # endpoint goal (x, y) in meters
duration = 2.0
dt = 0.002
schedule = "minimum_jerk"

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

## Overriding sections for comparison

`tools/reaching_simulator.py` can override the `[initial]`, `[task]`, and `[controller]`
sections from separate files, so one base config can be reused across a comparison
sweep without editing it. Each override file supplies the named table (e.g. a file
with just a `[controller]` block). With `--save PATH` the run is headless (no GUI)
and the log is written directly, which is convenient for a scripted sweep:

```bash
# Same robot and task, different controllers:
uv run python tools/reaching_simulator.py base.toml --controller computed_torque.toml --save ct.sklog.npz
uv run python tools/reaching_simulator.py base.toml --controller mpc.toml             --save mpc.sklog.npz

# Same controller, different tasks:
uv run python tools/reaching_simulator.py base.toml --task near.toml --save near.sklog.npz
uv run python tools/reaching_simulator.py base.toml --task far.toml  --save far.sklog.npz
```

Without `--save`, the same overrides configure the interactive GUI instead — e.g.
`tools/reaching_simulator.py base.toml --controller pd.toml` opens the reach GUI driven by the
PD controller. `--initial FILE` replaces the initial pose from a file's `[initial]`
table, and `--pose 20,45` then overrides just the joint angles (degrees) — matching
the kinematics and dynamics tools. The override values are merged into the scenario,
so each saved run's log still embeds its exact (overridden) config for reproduction.

## Using it from Python

```python
from skelarm import load_scenario, run_scenario

scenario = load_scenario("examples/reach.toml")
log = run_scenario(scenario)   # uses the task's duration / dt
log.save("reach.sklog.npz")    # replay/analyze with tools/player.py
```

`run_scenario` runs the fixed-step control loop (like `simulate_controlled`) but
also embeds the scenario in the log for later reproduction. `build_controller` can
also be called directly with a `[controller]` mapping, a `Task`, and a `Skeleton`,
so controllers can be constructed without a file.

## Reproducible runs

A log written by `run_scenario` (and by `tools/reaching_simulator.py`) is a self-contained,
re-runnable record. It embeds — in the log's `[extra]` metadata — the **original
source config** (the full `[skeleton]` / `[initial]` / `[task]` / `[controller]`
tables, exactly as loaded), the actual run parameters (`duration` / `dt` /
`grav_vec` / `enforce_limits`), and the `skelarm` / `numpy` / `scipy` versions.
`enforce_limits` records the *resolved* joint-limit choice, so a `--no-joint-limits`
override is reproduced on re-run even though the source config still reads `true`.

`rerun_log` reconstructs the scenario and re-simulates it:

```python
from skelarm import rerun_log
from skelarm.recording import StateLog

log = StateLog.load("reach.sklog.npz")
again = rerun_log(log)   # rebuilds the scenario and re-runs the dynamics
```

Reconstruction reparses the embedded source config, so identical input gives
identical state. The deterministic controllers (PD, computed torque,
inverse-dynamics feedforward, and the reaching controllers) reproduce the recorded
channels **exactly** on the same machine. MPC calls `scipy.optimize.minimize`,
which is deterministic but only bit-identical for the same `scipy` / BLAS build, so
an MPC re-run matches within a small numerical tolerance rather than exactly.

### Export an editable config for comparison

To tweak parameters and compare, export the embedded config to an editable TOML and
re-run it. Because the export is the original config verbatim, re-running it
**unedited** reproduces the run exactly; editing a value gives a controlled variant:

```python
from skelarm import export_scenario_toml, load_scenario, run_scenario
from skelarm.recording import StateLog

export_scenario_toml(StateLog.load("reach.sklog.npz"), "edited.toml")
# ... edit a gain / target / gravity in edited.toml ...
variant = run_scenario(load_scenario("edited.toml"))
```

From the command line, `tools/export_config.py` writes the config from a saved log:

```bash
uv run python tools/export_config.py reach.sklog.npz --output edited.toml
uv run python tools/reaching_simulator.py edited.toml                       # explore the edited scenario in the GUI
uv run python tools/reaching_simulator.py edited.toml --save edited.sklog.npz   # or re-run it headlessly
```

!!! note "What is not captured"
    A controller built programmatically (not from a config) has no embedded
    config, so its run is recorded without reproduction metadata and `rerun_log`
    (and `export_scenario_toml`) reject it. Re-running is available for scenarios
    loaded from TOML.
