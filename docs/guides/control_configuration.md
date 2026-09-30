# Control Configuration

The key-by-key reference for the `[task]`, `[simulator]`, and `[controller]`
tables of a scenario file. For the scenario model itself — the five-table
layout, a complete example, section overrides, and running scenarios from the
tools or Python — start at [Run Controlled Scenarios](running_scenarios.md);
the `[skeleton]` / `[initial]` tables are covered in
[Robot Configuration](robot_configuration.md).

See [Trajectory Tracking Control](../reference/07_control.md) and
[Reaching Control](../reference/08_reaching_control.md) for the underlying theory.

## The `[task]` section

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `type` | str | *required* | Task kind — the discriminator that decides what else the task needs. Five types are built in (see [Task types](#task-types)), and you can [add your own](defining_a_task.md). |
| `target` | `[x, y]` or table | *required for `reaching`* | Endpoint goal in meters (see below). Other task types may omit it. |
| `duration` | float | `2.0` | Total simulated time / planned-motion horizon, in seconds. |
| `schedule` | str | `"minimum_jerk"` | Time scaling for planned trajectories: `linear`, `cubic`, `quintic`, or `minimum_jerk`. |

`type` is the one always-required key: it names the kind of task, and the kind
determines the rest (`reaching` requires a `target`). `duration`, `schedule`, and
`target` define the planned task-space trajectory used by the *trajectory-tracking*
controllers; the *reaching* controllers use only `target` and generate motion
online. The numerical-integration parameters (`dt`, `enforce_limits`) live in their
own [`[simulator]`](#the-simulator-section) table, not here.

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

### Task types

`type` selects the task kind. Beyond `reaching`, two **trajectory-tracking** types
track a reference loaded from a `.sklog.npz` file (e.g. one recorded with
`tools/trajectory_recorder.py`):

| `type` | Reference | Tracked by |
| --- | --- | --- |
| `reaching` | A planned point-to-point reach to `target`. | any controller |
| `multi_target_reaching` | Several candidate targets; the active one is reached (switchable live in the GUI). | any controller (live switching retargets reaching controllers only) |
| `periodic_curve` | A closed task-space curve traced repeatedly, converted to joint angles by IK. | trajectory-tracking controllers |
| `trajectory_tracking` | The recorded **tip** `(x, y)` path, converted to joint angles by IK. | trajectory-tracking controllers |
| `joint_trajectory_tracking` | The recorded **per-joint** `q(t)` series directly (no IK). | joint-space controllers |

Each task type has a dedicated interactive simulator —
`tools/reaching_simulator.py`, `tools/multi_target_simulator.py`,
`tools/periodic_curve_simulator.py`, and `tools/trajectory_tracking_simulator.py` —
sharing the same controls (drag to apply disturbance forces, **Record** / **Export…**,
the `--initial` / `--pose` / `--task` / `--controller` overrides, and `--save` for a
headless batch). Their runs replay in `tools/player.py`, which draws the task overlay
(target, curve, or reference).

A `periodic_curve` task names a `curve` kind plus its parameters and a `period` (one
loop); `duration` sets how many loops are traced. Built-in curves:

| `curve` | Parameters | Shape |
| --- | --- | --- |
| `circle` | `center`, `radius`, `phase` | a circle |
| `ellipse` | `center`, `a`, `b`, `phase` | an axis-aligned ellipse |
| `lemniscate` | `center`, `a` | the Bernoulli ∞ (horizontal) |
| `vertical_lemniscate` | `center`, `a`, `b` | an upright figure-eight |
| `rose` | `center`, `a`, `k` | a rhodonea (`k` petals if odd, `2k` if even) |

```toml
[task]
type = "periodic_curve"
curve = "rose"
center = [0.8, 0.0]
a = 0.4
k = 3
period = 4.0     # seconds per loop
duration = 12.0  # three loops
```

A `multi_target_reaching` task lists several candidate `targets` (each a `[x, y]` or a
`{ pos, label, color, tolerance }` table) and an `active` index (default 0). The active
target flows through the ordinary reaching pipeline; `tools/multi_target_simulator.py`
draws all candidates and switches the active one live when you press a number key
(`1`..`N`), retargeting a reaching controller (e.g. `virtual_spring_damper`) on the fly.
The trajectory-tracking controllers and MPC follow a reference planned when the run
starts, so a live switch moves only the marker: they keep reaching for the target
that was active at the start.

```toml
[task]
type = "multi_target_reaching"
active = 0
targets = [
  { pos = [1.2, 0.4], label = "A", tolerance = 0.03 },
  { pos = [0.3, 1.2], label = "B", color = "teal" },
]
```

The trajectory-tracking types take these extra `[task]` keys:

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `file` | str | *required* | Path to the reference `.sklog.npz`. |
| `filter` | table | none | Pre-smoothing (see below): `{ kind = …, cutoff_hz, order, window, polyorder }`. |
| `interpolator` | str | `"cubic_spline"` | Resampling scheme: `cubic_spline`, `linear`, or `lagrange`. |

The `filter.kind` selects the smoother and which keys it needs (zero-phase in every
case): `none` (off); `lowpass` / `butterworth` take a `cutoff_hz` (and Butterworth an
`order`); `moving_average` / `savgol` take a `window` (odd, in samples), and `savgol`
also a `polyorder`. See the [theory chapter](../reference/09_trajectory_filtering.md).

```toml
[task]
type = "joint_trajectory_tracking"
file = "teach.sklog.npz"
filter = { kind = "butterworth", cutoff_hz = 8.0, order = 4 }  # smooth a jaggy recording
interpolator = "cubic_spline"
# duration defaults to the reference's length when omitted
```

If `duration` is omitted, it defaults to the reference trajectory's length. The
reference content is **embedded** in the run log, so `rerun_log` and exported configs
reproduce the run without the original file. Curve and DOF rules: a
`joint_trajectory_tracking` reference must have the same joint count as the robot.

The IK-based conversions (`periodic_curve`, `trajectory_tracking`) solve each sample
with numerical IK and keep the best-effort joint angles even where a sample cannot
be reached; if any sample ends noticeably off the task path (beyond 0.1 mm), one
aggregated `UserWarning` reports how many samples deviated and where.

Only these are built in. To add a goal that is not a single target point, or a new
reference source, see [Defining a Task](defining_a_task.md).

## The `[simulator]` section

How the dynamics are integrated, separate from the task (the desired motion). The whole
table is optional; an absent `[simulator]` uses the defaults.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `dt` | float | `0.002` | The fixed control / integration step of the simulation loop ([`simulate_controlled`](../reference/07_control.md)). It is also the MPC prediction step. |
| `enforce_limits` | bool | `true` | Apply the joint limits as hard stops in the dynamics. Set `false` to let the limits constrain only the kinematics. |

```toml
[simulator]
dt = 0.002
enforce_limits = true
```

Headless runs and the interactive simulators integrate with the same `dt`: the GUI
fits as many physics substeps of `dt` as fill its ~20 ms render frame (e.g. ten
substeps at `dt = 0.002`), and for a `dt` above the frame period it runs one substep
per tick with the render timer slowed to match, so wall clock tracks simulated time.

`enforce_limits` is a *run condition*: with the default `true` the fixed-step loop pins
each joint at its `[qmin, qmax]` bound (a hard stop); with `false` the bounds are dropped
from the dynamics and apply only to the kinematics (posing and inverse kinematics).
Because it lives in the scenario config, the resolved value is embedded in the run's
reproduction metadata, so a saved log re-runs with the same choice. The interactive
simulators' `--no-joint-limits` flag overrides it off for a single run (and that resolved
value is what gets recorded). See the [Joint Limits](joint_limits.md) guide for the
underlying mechanics.

## The `[controller]` section

`type` selects the control law; the remaining keys are its gains (any omitted key
falls back to the default below). The joint-space PD gains `kp` and `kd` take either
one scalar for every joint or a per-joint array (e.g. `kp = [300.0, 200.0]`); the
task-space gains (`k_task`, `d_task`, `c_joint`) are isotropic scalars. To plug in a control
law of your own, see [Defining a Controller](defining_a_controller.md).

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

MPC predicts with the simulation step, so its control interval is `[simulator].dt`.
Re-optimizing every step is expensive at a small `dt`; use a larger `[simulator].dt`
(for example `0.05`) for MPC scenarios — `examples/mpc.toml` is a complete one:

<video controls loop muted playsinline width="640" src="../../assets/mpc_reach.mp4"></video>

!!! note "Reach time vs. settling"
    For trajectory-tracking controllers the planned motion spans `[0, duration]`,
    so the *reference* reaches the target at `t = duration`; the endpoint follows
    with the controller's tracking error. The model-based controllers keep that
    error small, while plain joint PD lags visibly: on `examples/reach.toml`,
    `computed_torque` ends about 0.1 mm from the target at `t = duration` and
    `joint_pd` about 7 mm. The reaching controllers converge asymptotically. Either
    way, give `duration` some margin when the endpoint must settle.

## Related

- [Run Controlled Scenarios](running_scenarios.md) — the scenario model, a
  complete example, overrides, and the tools.
- [Record, Replay, and Re-simulate](recording_replay.md) — run metadata and the
  reproducibility tiers.
- [Joint Limits](joint_limits.md) — the `enforce_limits` mechanics.
- [Defining a Task](defining_a_task.md) / [Defining a Controller](defining_a_controller.md)
  — add your own types.
