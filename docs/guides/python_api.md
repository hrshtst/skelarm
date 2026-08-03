# Python API Quick Start

Use `skelarm` as a library. Everything shown here is importable from the
top-level `skelarm` package; the per-module details live in the API Reference
(start at [Architecture](../api/architecture.md)).

## Build a robot and compute kinematics

From a config file, or directly in code:

```python
import numpy as np

from skelarm import LinkProp, Skeleton, compute_forward_kinematics

skeleton = Skeleton.from_toml("examples/four_dof_robot.toml")  # from TOML

link = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
skeleton = Skeleton([link])  # or programmatically

skeleton.q = np.array([0.5])  # setters clamp into the joint limits (and warn)
compute_forward_kinematics(skeleton)
tip = skeleton.links[-1]
print(tip.xe, tip.ye)
```

## Run a scenario headlessly

The highest-level entry point mirrors the GUI tools — one config, one call:

```python
from skelarm import load_scenario, rerun_log, run_scenario

scenario = load_scenario("examples/reach.toml")
log = run_scenario(scenario)  # duration from [task], dt from [simulator]
log.save("reach.sklog.npz")

again = rerun_log(log)  # deterministic re-simulation from the embedded config
```

The returned `StateLog` gives channel arrays for analysis
(`log.channel("q")`, `log.times`) and replays in `tools/player.py` — see
[Record, Replay, and Re-simulate](recording_replay.md).

## Simulate uncontrolled dynamics

`simulate_robot` integrates the free dynamics with adaptive `solve_ivp`, taking
a torque callback `f(t, skeleton) -> tau`:

```python
import numpy as np

from skelarm import LinkProp, Skeleton, simulate_robot

link = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
skeleton = Skeleton([link])
skeleton.q = np.array([0.0])
skeleton.dq = np.array([0.0])


def control_torques(t, skel):
    return np.array([0.0])  # zero torque


times, q_traj, dq_traj = simulate_robot(skeleton, (0.0, 1.0), control_torques)
```

For a *stateful* controller (anything with an `update` hook, MPC in particular),
use the fixed-step `simulate_controlled` loop instead — adaptive `solve_ivp` may
call the torque callback several times per output interval, which breaks
controller state.

## Lower-level building blocks

```python
from skelarm import (
    compute_forward_dynamics,  # tau -> ddq (mass matrix solve)
    compute_inverse_dynamics,  # motion -> tau (Recursive Newton-Euler)
    compute_inverse_kinematics,  # endpoint target -> q (check result.success)
    compute_jacobian,  # endpoint Jacobian
    ik_joint_reference,  # task path -> joint reference (warns on deviation)
    simulate_controlled,  # fixed-step control loop
)
```

Conventions to keep in mind:

- **No gravity** — the arm is horizontal; the `grav_vec` parameter on the
  dynamics functions is an advanced/testing hook outside the supported model.
- **Joint limits** — kinematic setters clamp and warn; the fixed-step
  integrators apply hard stops; `simulate_robot` is unconstrained
  ([Joint Limits](joint_limits.md)).
- **Radians internally** — TOML configs use degrees, the API uses radians.
- Arrays are `numpy.typing.NDArray[np.float64]` throughout.

## Related

- [Defining a Task](defining_a_task.md) / [Defining a Controller](defining_a_controller.md)
  — register your own types so scenario configs and saved logs can use them.
- [Theory Reference](../reference/index.md) — the equations behind each helper.
