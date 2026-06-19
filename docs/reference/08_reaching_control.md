# Reaching Control

This chapter focuses on reaching: moving the endpoint from its current position
to a desired task-space target

$$
x^\ast =
\begin{bmatrix}
{}^d x \\
{}^d y
\end{bmatrix}.
$$

The [Trajectory Tracking Control](07_control.md) chapter can already solve this
task: plan a smooth trajectory from the initial endpoint $x_0$ to $x^\ast$,
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
x_r(t) = x_0 + s(t)(x^\ast - x_0)
$$

with a smooth schedule such as a quintic or minimum-jerk profile, then track
$x_r(t)$ through IK or resolved motion rate control. This can generate smooth
endpoint motion in an undisturbed simulation.

The limitation is that the reference keeps advancing with time even if the
endpoint is held by an obstacle or external force. When the constraint is
released, the accumulated tracking error can produce a large restoring torque and
a sudden acceleration. This is the motivation for feedback-generated reaching.

## 2. Virtual spring-damper reaching

A time-free reaching primitive is a virtual spring-damper in task space. Let
$x(q)$ and $\dot{x}=J(q)\dot{q}$ be the endpoint position and velocity. A direct
target spring can be written in `skelarm` sign convention as

$$
F_d = K_x(x^\ast-x) - D_x\dot{x},
\qquad
\tau = J^{T}F_d - C_q\dot{q}.
$$

Here $K_x$ is task-space stiffness, $D_x$ is task-space damping, and $C_q$ is
joint damping. This does not require a target arrival time, and the Jacobian
transpose maps the virtual endpoint force into joint torques.

This control law is compliant in the sense that external forces can displace the
endpoint. Its weakness is the initial force: if $x^\ast$ is far from $x_0$, then
$K_x(x^\ast-x_0)$ is immediately large. Seto and Sugihara describe this as the
reason conventional virtual spring-damper reaching can produce a large initial
acceleration rather than a human-like bell-shaped speed profile.

## 3. Online reference shaping

Seto and Sugihara's reaching-control papers address the initial-acceleration and
external-force problems by replacing the fixed spring equilibrium $x^\ast$ with a
shaped equilibrium $x_s$:

- Seto and Sugihara (2009),
  ["Online reference shaping with end-point position feedback for large
  acceleration avoidance on manipulator
  control"](https://doi.org/10.1109/IROS.2009.5353889).
- Seto and Sugihara (2009),
  ["Online nonlinear reference shaping with end-point position feedback for
  human-like smooth reaching motion"](https://doi.org/10.1109/ICHR.2009.5379562).

The endpoint spring-damper becomes

$$
F_d = K_x(x_s-x) - D_x\dot{x},
\qquad
\tau = J^{T}F_d - C_q\dot{q}.
$$

The shaped equilibrium $x_s$ is generated online from the target and current
endpoint:

$$
x_s =
\frac{1}{(T_1s+1)(T_2s+1)}
\left(r x^\ast + (1-r)x\right),
\qquad
0 < r \le 1.
$$

The second-order lag filter makes the initial reference velocity and acceleration
smooth. Feeding back the current endpoint $x$ keeps $x_s$ near the arm when the
endpoint is constrained by an external contact. That reduces excessive pushing
force and avoids a large acceleration after release.

The parameter $r$ controls the tradeoff:

- small $r$ places the shaped equilibrium near the current endpoint, reducing
  acceleration and pushing force but slowing convergence;
- large $r$ places it near the target, improving convergence but behaving more
  like a conventional target spring.

This fixed-$r$ controller is the core idea of the IROS 2009 paper. It produces
smoother reaching than a direct target spring and makes the behavior more pliant
under endpoint constraints.

## 4. Position-dependent shaping ratio

The Humanoids 2009 paper makes $r$ position dependent so the controller starts
gently and converges faster near the target. Define

$$
d =
\frac{\lVert x^\ast - x\rVert}
{\lVert x^\ast - x_0\rVert},
$$

where $x_0$ is the endpoint position at the start of the reach. The proposed
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

In implementation, clamp the computed value to the valid interval $0 < r \le 1$
before using it in the reference shaper. The formula can slightly exceed one
near $d=0$ for nonzero $a$, while `skelarm` should keep the interpolation
semantics explicit.

## 5. Implementation notes for `skelarm`

The controller can be implemented as an endpoint-force controller:

1. Run forward kinematics.
2. Compute $x$ from the tip link and $\dot{x}$ with `compute_endpoint_velocity`.
3. Update the shaped reference state $x_s$.
4. Compute $F_d = K_x(x_s-x)-D_x\dot{x}$.
5. Compute $\tau = J^T F_d - C_q\dot{q}$ with `compute_jacobian`.
6. Pass $\tau$ to `compute_forward_dynamics` or return it from a
   `simulate_robot` torque callback.

The reference shaper is dynamic state, not a pure function of the current
skeleton. With `simulate_robot` and adaptive `solve_ivp`, avoid mutating this
state inside a torque callback because the integrator may call the callback at
repeated or non-monotonic trial times. A robust implementation should either
augment the ODE state with the filter states or provide a fixed-step simulation
loop for stateful controllers.

Suggested first tests:

- a direct target spring has larger initial endpoint acceleration than shaped
  reaching for the same target;
- fixed-$r$ shaping reaches the target with a bell-shaped endpoint speed profile
  in an undisturbed simulation;
- position-dependent $r(d)$ converges faster than a conservative fixed small
  $r$ while keeping the initial acceleration small;
- when the endpoint is temporarily constrained, online shaping produces smaller
  stored endpoint force than a direct target spring;
- after release from a temporary constraint, the endpoint resumes toward the
  target without a large velocity spike.
