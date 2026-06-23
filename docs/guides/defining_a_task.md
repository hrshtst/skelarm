# Defining a Task

A **task** describes *what the arm should do* and the *run conditions* for it. In a
scenario it is the `[task]` table; in code it is a `Task` ([API](../api/scenario.md)).
A task does not move the arm on its own — a [controller](defining_a_controller.md)
reads the task (its target and parameters) and produces the joint torques. This
guide covers configuring the built-in task and **defining a new task type** of your own.

For the full `[task]` schema (target, `duration`, `dt`, `schedule`, `tolerance`,
`enforce_limits`) see [Control Configuration](control_configuration.md#the-task-section).

## What a task carries

| Field | Meaning |
| --- | --- |
| `target` | The task-space goal `(x, y)` in meters (**required**). Drawn as a marker by the tools. |
| `type` | The task kind — a label a controller interprets (default `"reaching"`). |
| `duration` / `dt` | Run conditions: total simulated time and the fixed control step. |
| `schedule` | Time scaling for planned trajectories (`minimum_jerk`, `quintic`, …). |
| `tolerance` / `label` / `color` | Success radius and marker presentation. |
| `enforce_limits` | Whether the joint limits act as a dynamics hard stop (see [Joint Limits](joint_limits.md)). |
| `params` | **Any extra `[task]` keys**, kept verbatim — how a custom task carries its own data. |

The only built-in `type` is `reaching`: move the endpoint to `target`. Because the
task type is just a *label that a controller interprets*, "defining a new task"
means two things together: **registering the type** so configs validate, and
**giving a controller the logic** that reads the task's `params`.

## Option 1 — configure the built-in reaching task

Most goals are a reaching task with different targets / run conditions; no code is
needed. See the [Control Configuration](control_configuration.md#the-task-section)
guide and override the `[task]` per run with `tools/reaching_simulator.py --task`.

## Option 2 — define a new task type

Use this when the goal is not a single reaching point — for example tracing a
circle, where the task needs a center, radius, and period.

### 1. Register the type and carry parameters

Allow the new `type` with `register_task_type`. Any `[task]` keys beyond the
recognized ones are preserved on `task.params`, so your type carries its own
configuration:

```toml
[task]
type = "tracing"
target = [1.2, 0.0]   # still required: the path's start point (drawn as the marker)
duration = 6.0
dt = 0.002
# custom keys -> task.params
center = [0.8, 0.0]
radius = 0.4
period = 3.0
```

```python
from skelarm import Task, register_task_type

register_task_type("tracing")          # now [task].type = "tracing" validates
task = Task.from_dict({
    "type": "tracing",
    "target": [1.2, 0.0],
    "center": [0.8, 0.0], "radius": 0.4, "period": 3.0,
})
assert task.params["radius"] == 0.4    # extra keys land here
```

`target` is the one required field, so set it to a representative point (here the
path's start); the GUI and plots draw it as the marker.

### 2. Give a controller the task's logic

A task type only has meaning through a controller that reads its `params`. Pair it
with a custom controller (see [Defining a Controller](defining_a_controller.md)),
whose builder receives the `Task`:

```python
import numpy as np
from skelarm import Controller, compute_jacobian, register_controller

class CircleTracer(Controller):
    """Pull the tip along a circle parameterized by the task's params."""

    def __init__(self, center, radius, period, *, k_task=200.0, d_joint=20.0):
        self.center, self.radius, self.period = np.asarray(center, float), radius, period
        self.k_task, self.d_joint = k_task, d_joint

    def control(self, t, skeleton):
        phase = 2.0 * np.pi * t / self.period
        goal = self.center + self.radius * np.array([np.cos(phase), np.sin(phase)])
        tip = skeleton.links[-1]
        force = self.k_task * (goal - np.array([tip.xe, tip.ye]))
        return compute_jacobian(skeleton).T @ force - self.d_joint * skeleton.dq

def build_circle_tracer(params, skeleton, task):
    return CircleTracer(task.params["center"], task.params["radius"], task.params["period"],
                        k_task=params.get("k_task", 200.0), d_joint=params.get("d_joint", 20.0))

register_controller("circle_tracer", build_circle_tracer)
```

A scenario then wires the two together and runs reproducibly:

```toml
[controller]
type = "circle_tracer"
k_task = 200.0
```

```python
from skelarm import load_scenario, run_scenario

log = run_scenario(load_scenario("tracing.toml"))
log.save("tracing.sklog.npz")   # replay with tools/player.py
```

## Reproducibility note

The custom `[task]` keys live in the scenario config, which `run_scenario` embeds
verbatim in the log (see [Reproducible runs](control_configuration.md#reproducible-runs)).
A re-run with `rerun_log` or an exported config reproduces the task exactly —
**provided the type is registered first**. Call `register_task_type` (and
`register_controller`) at import time of your module so loading a log that uses the
type always finds it; otherwise `Task.from_dict` raises *unknown task type*. Inspect
the registered types with `task_types()`.

## Related

- [Defining a Controller](defining_a_controller.md) — the logic half of a task.
- [Control Configuration](control_configuration.md) — the full `[task]` / `[controller]` schema.
- [Joint Limits](joint_limits.md) — the `enforce_limits` run condition.
- [Scenario API](../api/scenario.md) — `Task`, `register_task_type`, `task_types`.
