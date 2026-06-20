# Control Configuration

A **scenario** file describes a complete controlled run in one TOML document: the
robot, its start state, the reaching task, and the controller that drives it. It
extends the per-section schema used by [`Skeleton.from_toml`](../api/skeleton.md)
with `[task]` and `[controller]` sections, and is loaded by
`skelarm.load_scenario`.

```bash
uv run python tools/reach.py examples/reach.toml   # simulate the reach, export a log
uv run python tools/replay.py reach.sklog.npz      # replay and analyze it
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
| `target` | `[x, y]` | *required* | Endpoint goal in meters. |
| `duration` | float | `2.0` | Total simulated time, in seconds. |
| `dt` | float | `0.002` | Control / integration step, in seconds. |
| `schedule` | str | `"minimum_jerk"` | Time scaling for planned trajectories: `linear`, `cubic`, `quintic`, or `minimum_jerk`. |

`duration`, `schedule`, and `target` define the planned task-space trajectory used
by the *trajectory-tracking* controllers; the *reaching* controllers use only
`target` and generate motion online. `dt` is the fixed step of the simulation loop
([`simulate_controlled`](../reference/07_control.md)).

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

## Using it from Python

```python
from skelarm import load_scenario, simulate_controlled

scenario = load_scenario("examples/reach.toml")
log = simulate_controlled(
    scenario.skeleton, scenario.controller, duration=scenario.task.duration, dt=scenario.task.dt
)
log.save("reach.sklog.npz")   # replay/analyze with tools/replay.py
```

`build_controller` can also be called directly with a `[controller]` mapping, a
`Task`, and a `Skeleton`, so controllers can be constructed without a file.
