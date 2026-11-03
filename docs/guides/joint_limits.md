# Joint Limits

Each movable joint has an angle range `[qmin, qmax]`. This page explains how that
range is enforced — which is **not** in the dynamics equations, but in the layer
that drives them. The behavior therefore depends on *which* simulation path you
use.

For how to declare limits in a config file (they are given in **degrees** and
default to `[-180, 180]` when omitted), see
[Robot Configuration](robot_configuration.md#per-link-keys-skeletonlink).

## Where limits are defined

Limits live on each link as `LinkProp.qmin` / `qmax`, stored in **radians**
(converted from the degrees in TOML). The fixed base link has no range.

## The dynamics equations are limit-agnostic

`compute_forward_dynamics` and `compute_inverse_dynamics` (and the mass-matrix and
Coriolis helpers) solve the equations of motion with no knowledge of `qmin` /
`qmax`. Any limit handling is added by the integration layer on top of them.

## Enforcement depends on the simulation path

| Path | Limit handling |
| --- | --- |
| `Skeleton.q` / `set_state` (kinematics) | Clamp into range **with a warning** |
| `simulate_controlled` (fixed step) | **Hard stop**: clip angle, zero velocity, or lock the joint (`stops`); toggleable via `enforce_limits` |
| `SkelarmSimulator` GUI (fixed step) | **Hard stop**: clip angle, zero velocity, or lock the joint (`stops`); toggleable via `enforce_limits` |
| `simulate_robot` (adaptive `solve_ivp`) | **None** — unconstrained integration |
| `JointSpaceMPC` | Soft penalty in the rollout (+ plant clamp via the driving loop) |

### Fixed-step integrators: hard stops

`simulate_controlled` and the interactive `SkelarmSimulator` use semi-implicit
(symplectic) Euler and apply limits as **hard stops** each step:

```python
dq = dq + ddq * dt
q = q + dq * dt
q_clamped = np.clip(q, lower, upper)  # pin the angle at the bound
dq = np.where(q_clamped != q, 0.0, dq)  # zero the velocity of any joint that hit a limit
```

This is a simple **per-joint projection**: a joint that reaches its bound is pinned
there and its velocity is set to zero — no bounce and no restoring spring. It does
not model the impact: the other joints keep their velocities, and because the
joints' inertias are coupled, zeroing one joint's velocity does not necessarily
remove energy — the arm's total kinetic energy can even rise at the stop. Treat
it as a way to keep the angles in range, not as an energy-dissipating contact
model ([locked stops](#locked-stops) are one). These loops write `link.q` / `link.dq` directly, bypassing the
warning-emitting setter, so a run that rides a limit does not flood the log with
warnings.

The shared one-step integrator `integrate_with_limits(skeleton, tau, dt, lower,
upper)` takes the bounds as arguments; passing `lower=upper=None` (the default)
**skips** the clamp/zero block entirely, integrating the unconstrained step.

### Locked stops

The projection also leaves the dynamics of the other joints as they were: while a
joint is pressed into its stop, they keep the accelerations they would have with it
free, $\ddot{q} = H^{-1}(\tau - b)$. With coupled inertias they then accelerate faster
than the arm can, and it gains energy the motors never gave it. On a two-link arm
held at joint 1's stop and driven at joint 2 alone, the kinetic energy after 0.1 s is
1.6 times the motors' work. `stops = "locked"` models the stop as a constraint
instead:

- a stop **holds** the joint pressed into it: the joint does not accelerate, and the
  stop's reaction $\lambda = H \ddot{q} + b - \tau$ is whatever keeps it there. The
  other, free joints move by the reduced dynamics $H_{ff} \ddot{q}_f = (\tau - b)_f$, so
  the stop does no work (in the example, the kinetic energy is the motors' work, less
  semi-implicit Euler's 1%);
- it **lets go** once holding the joint would take a pull: a stop only pushes, up at a
  lower stop and down at an upper one;
- a joint that **reaches** its stop stops as in a plastic impact: the free joints keep
  their generalized momentum $(H \dot{q})_f$, and the kinetic energy drops.

Which joints the stops hold is a small linear complementarity problem: a held joint
has no acceleration and a reaction that pushes; a released one has no reaction and does
not accelerate into its stop. With $H$ positive definite its solution is unique, and
`integrate_with_limits` finds it among the subsets of the joints at their stops (one
solve while no joint is at a stop, at most two while one is). Away from the stops, the
locked step is the projected one to the bit.

Choose the model in the scenario config, where it is part of the reproduction
metadata:

```toml
[simulator]
stops = "locked"  # the default is "projected"
```

or per call: `run_scenario(..., stops=...)`, `simulate_controlled(..., stops=...)`,
`integrate_with_limits(..., stops=...)`, and `SkelarmSimulator(..., stops=...)`. The
scenario simulators use the scenario's; the dynamics simulator and the trajectory
recorder read no `[simulator]` table and keep the projected stops.

### Disabling the hard stop

Both fixed-step paths can drop the hard stop so the limits constrain only the
kinematics (posing and IK) — not the dynamics. There are three ways in, ordered
from most to least reproducible:

1. **Scenario config** — set `enforce_limits = false` in the `[simulator]` table. This
   is the reproducible toggle: `run_scenario` embeds the resolved value in the
   log's run metadata, so `rerun_log` and an exported config re-run with the same
   choice. See [Control Configuration](control_configuration.md#the-simulator-section).
2. **`run_scenario(..., enforce_limits=...)` / `simulate_controlled(..., enforce_limits=False)`** —
   the programmatic override; `None` (the `run_scenario` default) defers to the
   scenario's `simulator.enforce_limits`. `run_scenario` records the resolved value
   in the log's run metadata; a direct `simulate_controlled` call applies it
   without embedding any reproduction metadata.
3. **`--no-joint-limits` CLI flag** on the interactive tools (and the
   `SkelarmSimulator` / recorder classes' `enforce_limits=False` argument), which
   omits the bounds from the integrator for that run:

```bash
uv run python tools/dynamics_simulator.py examples/four_dof_robot.toml --no-joint-limits
uv run python tools/reaching_simulator.py examples/reach.toml --no-joint-limits   # also the --save path
uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml --mode dynamics --no-joint-limits
```

For the *scenario* simulators the flag *overrides* `[simulator].enforce_limits` off,
and the resolved value is recorded in the run metadata. (A re-run of an interactive
recording reproduces the unperturbed scenario — mouse-drag forces are not replayed;
see [Record, Replay, and Re-simulate](recording_replay.md#the-three-reproducibility-tiers).)
The generic dynamics simulator and the trajectory recorder apply the flag the same
way but write plain state logs with no reproduction metadata, so the choice is not
embedded for re-simulation there.

The default keeps the hard stop on. In the recorder this only affects `dynamics`
mode; `ik` mode always poses through the clamping kinematic setter regardless.

### Adaptive `solve_ivp`: no enforcement

!!! warning "`simulate_robot` does not enforce joint limits"
    `simulate_robot` integrates the unconstrained ODE with `scipy`'s adaptive
    RK45, so joints can pass straight through their limits. If you need limits
    respected during a dynamic simulation, use `simulate_controlled` (or the GUI
    simulator), which is also why the stateful controllers are run through it.

### MPC: a soft penalty

`JointSpaceMPC` keeps limits in its prediction rollout as a **soft penalty**
(`limit_weight · overshoot²`) and its prediction model is deliberately *unclamped*.
Hard limits are not imposed in the optimizer itself — `L-BFGS-B` only bounds the
torque variables (`tau_max`). The real clamping still happens in the
`simulate_controlled` loop that drives the controller.

## Kinematic setters (posing and IK)

Outside the integrators, assigning `Skeleton.q` or calling `set_state` clamps the
requested angles into `[qmin, qmax]` and emits a `UserWarning` for any value that
was out of range. This is the posing / forward-kinematics path; the integration
loops above deliberately avoid it. Numerical inverse kinematics likewise clamps
each proposed step to the limits and reports whether any were hit
(`IKResult.joint_limits_hit`).

## Related

- [Robot Configuration](robot_configuration.md) — declaring `limits` in a config.
- [Trajectory Tracking Control](../reference/07_control.md) and
  [Reaching Control](../reference/08_reaching_control.md) — the controllers run
  through the fixed-step loop.
