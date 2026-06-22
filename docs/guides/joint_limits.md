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
| `simulate_controlled` (fixed step) | **Hard stop**: clip angle, zero velocity |
| `SkelarmSimulator` GUI (fixed step) | **Hard stop**: clip angle, zero velocity |
| `simulate_robot` (adaptive `solve_ivp`) | **None** — unconstrained integration |
| `JointSpaceMPC` | Soft penalty in the rollout (+ plant clamp via the driving loop) |

### Fixed-step integrators: hard stops

`simulate_controlled` and the interactive `SkelarmSimulator` use semi-implicit
(symplectic) Euler and apply limits as **hard stops** each step:

```python
dq = dq + ddq * dt
q  = q  + dq * dt
q_clamped = np.clip(q, lower, upper)          # pin the angle at the bound
dq = np.where(q_clamped != q, 0.0, dq)        # zero the velocity of any joint that hit a limit
```

This is a **fully inelastic, per-joint stop**: a joint that reaches its bound is
pinned there and its velocity is set to zero — no bounce, no restoring spring, and
the kinetic energy in that joint is removed at contact. These loops write
`link.q` / `link.dq` directly, bypassing the warning-emitting setter, so a run
that rides a limit does not flood the log with warnings.

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
