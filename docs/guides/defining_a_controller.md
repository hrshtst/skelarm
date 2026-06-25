# Defining a Controller

A **controller** is the logic that drives the arm: each control step it reads the
robot state and returns the **joint torque** `τ`. It is the active half of a
scenario — the [task](defining_a_task.md) says *where* to go, the controller decides
*how*. This guide covers the controller interface, using a built-in, and **writing
and registering a new controller** of your own.

The built-in control laws and their theory are documented in
[Trajectory Tracking Control](../reference/07_control.md) and
[Reaching Control](../reference/08_reaching_control.md); their config keys are in
[Control Configuration](control_configuration.md#the-controller-section).

## The controller interface

Subclass `Controller` ([API](../api/control.md)). Only `control` is required:

| Method | Required | Purpose |
| --- | --- | --- |
| `control(t, skeleton) -> tau` | **yes** | Return the joint torque (shape `(num_joints,)`) for the current time and state. |
| `reset(skeleton)` | no | Initialize internal state at movement onset (`t = 0`). |
| `update(t, skeleton, dt)` | no | Advance internal state once per fixed control step. |
| `log_channels() -> dict` | no | Expose internal signals to record each step (e.g. `q_ref`, `error`). |

A controller is also callable — `controller(t, skeleton)` forwards to `control`, so a
stateless controller works directly as a [`simulate_robot`](../reference/05_numerical_methods.md)
torque callback. Stateful controllers use `reset` / `update` together with the
fixed-step loop (`simulate_controlled` / `run_scenario`).

!!! note "Gravity is ignored"
    The arm is planar and horizontal, so do **not** add gravity terms. See the
    physics notes in the [Inverse Dynamics](../reference/03_inverse_dynamics.md) reference.

## Option 1 — use a built-in controller

Pick a `type` in the `[controller]` table and set its gains; no code needed. The
available types are listed by `controller_types()` and documented in
[Control Configuration](control_configuration.md#the-controller-section).

## Option 2 — write a new controller

### A task-space example

This controller pulls the tip toward the task target with a virtual spring, mapping
the task-space force to joint torque through the endpoint Jacobian, plus joint
damping for stability (the same `Jᵀ·force` pattern the reaching controllers use):

```python
import numpy as np
from skelarm import Controller, compute_jacobian

class TaskSpaceSpring(Controller):
    """Tip spring toward a fixed target: τ = Jᵀ k (p* − p) − d q̇."""

    def __init__(self, target, *, k_task=200.0, d_joint=20.0):
        self.target = np.asarray(target, dtype=float)
        self.k_task = k_task
        self.d_joint = d_joint

    def control(self, t, skeleton):
        tip = skeleton.links[-1]
        position = np.array([tip.xe, tip.ye])
        force = self.k_task * (self.target - position)        # task-space restoring force
        return compute_jacobian(skeleton).T @ force - self.d_joint * skeleton.dq

    def log_channels(self):
        return {}   # optionally record internal signals here
```

Drive it directly — no registration needed for one-off experiments:

```python
from skelarm import Skeleton, simulate_controlled

skeleton = Skeleton.from_toml("examples/four_dof_robot.toml")
log = simulate_controlled(skeleton, TaskSpaceSpring([1.0, 0.5]), duration=2.0, dt=0.002)
```

### Tracking a reference

For trajectory-tracking laws, subclass `TrackingController` (it stores a
`JointReference` and PD gains, and logs `q_ref` / `error`) and read the reference in
`control`:

```python
import numpy as np
from skelarm import TrackingController

class WeightedPD(TrackingController):
    def control(self, t, skeleton):
        q_r, dq_r, _ = self.reference.sample(t)   # the joint reference at time t
        error = q_r - skeleton.q
        self._store(q_r, error)                   # makes log_channels emit q_ref / error
        return self.kp * error + self.kd * (dq_r - skeleton.dq)
```

Build a joint reference from a task-space path with `ik_joint_reference` (or supply
your own object implementing `JointReference.sample(t) -> (q_r, dq_r, ddq_r)`); see
the [Control API](../api/control.md).

## Make it config-driven and reproducible

To select your controller from a `[controller].type`, register a **builder** —
`(params, skeleton, task) -> Controller` — with `register_controller`. `params` is
the `[controller]` table without `type`, plus `params["dt"]` — the control step from
`[simulator]` — for controllers that need it (e.g. MPC, whose prediction step must match
it); use `task` for the target and run conditions:

```python
from skelarm import register_controller

def build_task_space_spring(params, skeleton, task):
    return TaskSpaceSpring(task.require_target(),   # raises clearly if the task has no target
                           k_task=params.get("k_task", 200.0),
                           d_joint=params.get("d_joint", 20.0))

register_controller("task_space_spring", build_task_space_spring)
```

```toml
[controller]
type = "task_space_spring"
k_task = 250.0
```

```python
from skelarm import load_scenario, run_scenario

log = run_scenario(load_scenario("reach.toml"))   # builds and runs your controller
log.save("reach.sklog.npz")
```

`run_scenario` embeds the `[controller]` config (gains and all) in the log, so
`rerun_log` and an exported config reproduce the run exactly (see
[Reproducible runs](control_configuration.md#reproducible-runs)). **Register the
builder at your module's import time** so re-running a saved log always finds the
type — otherwise `build_controller` raises *unknown controller type*. Re-registering
a name replaces it; list the registered types with `controller_types()`.

## Related

- [Defining a Task](defining_a_task.md) — the goal a controller drives toward.
- [Control Configuration](control_configuration.md) — built-in `[controller]` types and keys.
- [Trajectory Tracking Control](../reference/07_control.md) and [Reaching Control](../reference/08_reaching_control.md) — the theory behind the built-ins.
- [Control API](../api/control.md) and [Scenario API](../api/scenario.md) — `Controller`, `TrackingController`, `register_controller`, `controller_types`.
