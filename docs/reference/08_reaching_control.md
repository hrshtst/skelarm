# Reaching Control

This chapter focuses on reaching: moving the endpoint from its current position
to a desired task-space target

$$
p^\ast =
\begin{bmatrix}
{}^d x \\
{}^d y
\end{bmatrix}.
$$

The [Trajectory Tracking Control](07_control.md) chapter can already solve this
task: plan a smooth trajectory from the initial endpoint $p_0$ to $p^\ast$,
convert it into joint references, and track it with PD, inverse-dynamics
feedforward, or computed torque control. That route is repeatable and easy to
test.

Human-like reaching adds a different emphasis. Human unconstrained reaching often
shows roughly straight hand paths and bell-shaped speed profiles, and a robot
used near people should also behave gently when disturbed. The methods reviewed
here use endpoint feedback to generate reaching motion online instead of only
tracking a preplanned time trajectory.

## 1. Planned reaching as a baseline

The simplest reaching controller is trajectory tracking. Choose

$$
p_r(t) = p_0 + s(t)(p^\ast - p_0)
$$

with a smooth schedule such as a quintic or minimum-jerk profile, then track
$p_r(t)$ through IK or resolved motion rate control. This can generate smooth
endpoint motion in an undisturbed simulation.

The limitation is that the reference keeps advancing with time even if the
endpoint is held by an obstacle or external force. When the constraint is
released, the accumulated tracking error can produce a large restoring torque and
a sudden acceleration. This is the motivation for feedback-generated reaching.

## 2. Virtual spring-damper reaching

A time-free reaching primitive is a virtual spring-damper in task space. Let
$p(q)$ and $\dot{p}=J(q)\dot{q}$ be the endpoint position and velocity. A direct
target spring can be written in `skelarm` sign convention as

$$
F_d = K_{\mathrm{task}}(p^\ast-p) - D_{\mathrm{task}}\dot{p},
\qquad
\tau = J^{T}F_d - C_q\dot{q}.
$$

This is the basic structure of the virtual spring-damper hypothesis proposed for
human-like reaching with redundant arms:

- Arimoto and Sekimoto (2006),
  ["Human-like Movements of Robotic Arms with Redundant DOFs: Virtual
  Spring-Damper Hypothesis to Tackle the Bernstein
  Problem"](https://doi.org/10.1109/ROBOT.2006.1641977).

Here $K_{\mathrm{task}}$ is task-space stiffness, $D_{\mathrm{task}}$ is
task-space damping, and $C_q$ is joint damping. This does not require a target
arrival time, and the Jacobian transpose maps the virtual endpoint force into
joint torques. `skelarm` implements it as the `VirtualSpringDamper` controller,
using isotropic (scalar) `k_task`, `d_task`, and `c_joint` gains.

This control law is compliant in the sense that external forces can displace the
endpoint. Its weakness is the initial force: if $p^\ast$ is far from $p_0$, then
$K_{\mathrm{task}}(p^\ast-p_0)$ is immediately large. Seto and Sugihara describe
this as the reason conventional virtual spring-damper reaching can produce a
large initial acceleration rather than a human-like bell-shaped speed profile.

## 3. Time-varying stiffness and virtual damping

Sekimoto and Arimoto later studied the virtual spring-damper approach on a
redundant industrial arm and introduced an explicitly time-varying endpoint
stiffness:

- Sekimoto and Arimoto (2006),
  ["Experimental Study on Reaching Movements of Robot Arms with Redundant DOFs
  Based upon Virtual Spring-Damper
  Hypothesis"](https://doi.org/10.1109/IROS.2006.282375).

The paper uses the term "time-variable stiffness." In this document,
**time-varying stiffness** is used because it is the more common control
terminology: the spring stiffness is an explicit function of time during the
reach.

With the same sign convention as `skelarm`, the planar form is

$$
\tau =
- C_q\dot{q}
+ J^{T}
\left(
k(t)(p^\ast-p) - \zeta_1 k_0\dot{p}
\right).
$$

Here $C_q$ is joint damping, $k(t)$ is the scalar endpoint stiffness, $k_0$ is
its saturated value, and $\zeta_1 k_0$ is the task-space virtual damping gain.
Sekimoto and Arimoto used a gamma-distribution-shaped stiffness schedule,

$$
k(t) =
k_0
\left[
1 -
\left(
1 + \alpha t + \frac{\alpha^2 t^2}{2}
\right)e^{-\alpha t}
\right],
$$

which starts near zero and gradually approaches $k_0$. The effect is different
from a constant target spring:

- the initial torque is small because $k(0)=0$;
- the spring force becomes stronger in the middle and final phases, improving
  convergence to the target;
- the endpoint speed tends to become bell-shaped rather than peaking
  immediately;
- task-space virtual damping makes endpoint paths straighter, especially for
  redundant arms.

`skelarm` implements this as the `TimeVaryingStiffness` controller, a useful
intermediate between a constant virtual spring and the later online
reference-shaping methods. It takes the saturated stiffness `k0`, the rate `alpha`,
and a damping ratio `zeta1` (so the damping gain is `zeta1 * k0`), and needs no IK,
Jacobian inverse, or planned joint trajectory. Unlike the Seto-Sugihara online
shaper below, however, $k(t)$ is tied to elapsed time from movement onset. If the
endpoint is blocked, the stiffness continues to grow even though the arm has not
progressed, so contact behavior may still need force limits, reset logic, or the
online feedback shaping described next.

Because `skelarm` assumes planar motion without gravity by default,
`TimeVaryingStiffness` omits the gravity compensation term used in the 3-D PA-10
experiments; it would only be needed if the dynamics model were extended. The
paper's viscosity compensation is likewise unnecessary unless explicit joint
friction is added to the dynamics.

## 4. Online reference shaping

Seto and Sugihara's reaching-control papers address the initial-acceleration and
external-force problems by replacing the fixed spring equilibrium $p^\ast$ with a
shaped equilibrium $p_s$:

- Seto and Sugihara (2009),
  ["Online reference shaping with end-point position feedback for large
  acceleration avoidance on manipulator
  control"](https://doi.org/10.1109/IROS.2009.5353889).
- Seto and Sugihara (2009),
  ["Online nonlinear reference shaping with end-point position feedback for
  human-like smooth reaching motion"](https://doi.org/10.1109/ICHR.2009.5379562).

The endpoint spring-damper becomes

$$
F_d = K_{\mathrm{task}}(p_s-p) - D_{\mathrm{task}}\dot{p},
\qquad
\tau = J^{T}F_d - C_q\dot{q}.
$$

The shaped equilibrium $p_s$ is generated online from the target and current
endpoint:

$$
p_s =
\frac{1}{(T_1s+1)(T_2s+1)}
\left(r p^\ast + (1-r)p\right),
\qquad
0 < r \le 1.
$$

The second-order lag filter makes the initial reference velocity and acceleration
smooth. Feeding back the current endpoint $p$ keeps $p_s$ near the arm when the
endpoint is constrained by an external contact. That reduces excessive pushing
force and avoids a large acceleration after release.

The parameter $r$ controls the tradeoff:

- small $r$ places the shaped equilibrium near the current endpoint, reducing
  acceleration and pushing force but slowing convergence;
- large $r$ places it near the target, improving convergence but behaving more
  like a conventional target spring.

This fixed-$r$ controller is the core idea of the IROS 2009 paper. It produces
smoother reaching than a direct target spring and makes the behavior more pliant
under endpoint constraints. `skelarm` implements it as the `OnlineReferenceShaping`
controller, which realizes the second-order lag as two cascaded first-order lags
with time constants `t1` and `t2`, advanced once per fixed control step, and takes
the fixed ratio as `r`.

## 5. Position-dependent shaping ratio

The Humanoids 2009 paper makes $r$ position dependent so the controller starts
gently and converges faster near the target. Define

$$
d =
\frac{\lVert p^\ast - p\rVert}
{\lVert p^\ast - p_0\rVert},
$$

where $p_0$ is the endpoint position at the start of the reach. The proposed
schedule is

$$
r(d) =
\frac{1}{2}\left(
\sqrt{(d-1)^2+4a} - (d-1)
\right),
\qquad
0 < a \ll 1.
$$

At the start, $d \approx 1$ and $r \approx \sqrt{a}$, so the controller starts
with a small shaped-equilibrium shift. Near the target, $d \approx 0$ and $r$
approaches one, improving convergence. If an external force pushes the endpoint
away and $d>1$, $r$ decreases again, making the motion compliant.

`skelarm` implements this as the `PositionDependentShaping` controller (built on
`OnlineReferenceShaping`) and exposes the schedule as the `shaping_ratio` helper,
parameterized by `a`. It clamps the computed value to $0 < r \le 1$ before using it
in the reference shaper, because the formula can slightly exceed one near $d=0$ for
nonzero $a$.

## 6. Adaptation to target changes and external force

Seto and Sugihara later extended the online reference-shaping controller for
cases where the target changes during motion or an external force moves the
endpoint far from the current reach:

- Seto and Sugihara (2010),
  ["Motion Control with Slow and Rapid Adaptation for Smooth Reaching Movement
  under External Force Disturbance"](https://doi.org/10.1109/IROS.2010.5652721).

The key idea is to adapt the apparent initial endpoint used by the online
parameter tuning. Let $p_a$ denote this apparent initial endpoint, $p^\ast$ the
target, and $p$ the current endpoint. A simple tuning law from the paper is

$$
p_s = r p^\ast + (1-r)p,
\qquad
r =
1 - (1-\epsilon)
\frac{\lVert p^\ast-p\rVert}{\lVert p^\ast-p_a\rVert},
\qquad
0 < \epsilon \ll 1.
$$

When $p=p_a$, the shaping ratio is $r=\epsilon$, so the spring equilibrium is
only slightly shifted toward the target and the initial endpoint acceleration is
small. As $p$ approaches $p^\ast$, $r$ increases toward one and the controller
recovers target convergence. In implementation, clamp $r$ to $[\epsilon,1]$ and
guard against a small denominator when $p_a$ is already very close to $p^\ast$.

The 2010 paper proposes two adaptations for $p_a$:

- **Slow adaptation** moves $p_a$ toward the current endpoint with a first-order
  lag,

$$
\dot{p}_a = \frac{p-p_a}{T}.
$$

This moderates the change of $r$ and allows the controller to restart a smooth
reach when the target is changed during motion. Without this adaptation, $p_a$
remains tied to the original movement onset and the shaping ratio may no longer
increase smoothly toward the new target.

- **Rapid adaptation** resets $p_a$ when the endpoint has been displaced outside
  the current reaching region:

$$
p_a \leftarrow p
\quad\text{if}\quad
\frac{\lVert p^\ast-p\rVert}{\lVert p^\ast-p_a\rVert} > 1.
$$

This makes $r$ return to $\epsilon$ at the displaced endpoint, reducing the
virtual spring force so the arm can behave compliantly under a large external
disturbance. After the disturbance is removed, $r$ increases again from the new
position and the reach resumes smoothly. For a backdrivable manipulator this can
be done from endpoint position feedback alone; a non-backdrivable robot may need
force sensing or a separate admittance layer to realize the same compliant
behavior.

`skelarm` implements this as the `AdaptiveReferenceShaping` controller, built on
`OnlineReferenceShaping`, with the ratio computed by the `adaptive_shaping_ratio`
helper. Each fixed control step it stores $p_a$, applies the rapid reset before
computing $r$, then drifts $p_a$ with the slow-adaptation lag (`t_adapt` is the
time constant $T$). One detail differs from the paper: the 2010 paper uses the
internally dividing point $r p^\ast + (1-r)p$ directly as the spring equilibrium
(the second-order lag of the earlier papers is dropped), whereas `skelarm` feeds it
through the same second-order lag as §4, so the shaped equilibrium $p_s$ inherits
the smooth-start behavior of the fixed-$r$ shaper. Because both adaptations mutate controller state, the controller
runs through the fixed-step `simulate_controlled` loop via its `update` hook.

## 7. Implementation in `skelarm`

All five controllers above live in `skelarm.reaching` and share an
`EndpointController` base whose `control` step is an endpoint-force law:

1. Run forward kinematics (handled by the eager setters on the simulated clone).
2. Compute $p$ from the tip link and $\dot{p}$ with `compute_endpoint_velocity`.
3. Update the shaped reference state $p_s$ in the controller's `update` hook.
4. Compute $F_d = K_{\mathrm{task}}(p_s-p)-D_{\mathrm{task}}\dot{p}$.
5. Compute $\tau = J^T F_d - C_q\dot{q}$ with `compute_jacobian`.
6. Return $\tau$, which `simulate_controlled` passes to `compute_forward_dynamics`.

The reference shaper is dynamic state, not a pure function of the current
skeleton. With `simulate_robot` and adaptive `solve_ivp`, mutating this state
inside a torque callback is unsafe because the integrator may call the callback at
repeated or non-monotonic trial times. `skelarm` therefore advances the filter and
adaptation state in the controller's `update` hook, which `simulate_controlled`
calls exactly once per fixed control step.

These properties are verified in `tests/test_reaching.py`:

- a direct target spring has larger initial endpoint acceleration than shaped
  reaching for the same target;
- time-varying stiffness starts with near-zero spring torque and converges to
  the saturated stiffness $k_0$;
- fixed-$r$ shaping reaches the target with a bell-shaped endpoint speed profile
  in an undisturbed simulation;
- position-dependent $r(d)$ converges faster than a conservative fixed small
  $r$ while keeping the initial acceleration small;
- slow adaptation drifts the apparent initial endpoint toward the current
  endpoint;
- rapid adaptation resets the apparent initial endpoint after a large endpoint
  displacement;
- the shaping-ratio helpers match the paper values at the endpoints of their
  ranges.

Like the trackers of the previous chapter, these controllers can also be driven
from a TOML scenario file (`virtual_spring_damper`, `time_varying_stiffness`,
`online_shaping`, `position_dependent_shaping`, `adaptive_shaping`); see the
[Control Configuration](../guides/control_configuration.md) guide.
