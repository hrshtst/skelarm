# Reaching Control

This chapter sketches the control layer that should sit above the kinematics and
dynamics helpers in `skelarm`. The motivating task is planar reaching: a desired
endpoint target is specified in task space as

$$
x^\ast =
\begin{bmatrix}
{}^d x \\
{}^d y
\end{bmatrix},
\qquad
x(q) =
\begin{bmatrix}
x(q) \\
y(q)
\end{bmatrix}.
$$

The control problem is to move the endpoint from its current position to
$x^\ast$ while respecting joint limits, avoiding excessive acceleration, and
keeping the simulated motion numerically stable. There are two broad approaches:

- **Plan then track.** Build a desired trajectory first, convert it into joint
  space, then track it with a joint controller.
- **Generate reaching by feedback.** Use endpoint feedback to shape the reference
  online, so the motion adapts when the arm is disturbed.

Both approaches are useful in `skelarm`: planned trajectories are simple and
repeatable, while feedback-generated reaching is closer to the human-like control
ideas reviewed by Seto and Sugihara.

## 1. Task-space trajectory planning

A trajectory planner turns the fixed target $x^\ast$ into a time-indexed
reference $x_r(t)$. For a reaching task, the simplest path is a straight segment
from the initial endpoint $x_0$ to the target:

$$
x_r(t) = x_0 + s(t)(x^\ast - x_0),
\qquad
s(0)=0,\quad s(T)=1.
$$

The scalar schedule $s(t)$ controls smoothness:

- **Linear interpolation** uses $s=t/T$. It is easy, but has velocity jumps at
  the start and end.
- **Cubic smoothing** uses $s=3u^2-2u^3$ with $u=t/T$. It gives zero endpoint
  velocity.
- **Quintic smoothing** uses $s=10u^3-15u^4+6u^5$. It gives zero endpoint
  velocity and acceleration.
- **Minimum-jerk style profiles** are a natural first choice for human-like
  planned reaching, since human unconstrained reaching often shows roughly
  straight hand paths and bell-shaped speed profiles.

The planned trajectory should provide at least $x_r(t)$ and $\dot{x}_r(t)$.
Acceleration $\ddot{x}_r(t)$ is useful when converting the plan into a dynamic
feedforward term, but is not required for basic position control.

## 2. Converting task-space plans to joint-space references

`skelarm` actuates joints, so a task-space plan must become a joint reference
before it can be tracked by a joint controller.

### IK-based position conversion

At each sample time, solve

$$
x(q_r(t)) \approx x_r(t)
$$

with `compute_inverse_kinematics`. The previous solution is the best seed for the
next sample:

$$
q_r(t_{k+1}) =
\operatorname{IK}\left(x_r(t_{k+1});\ q_0=q_r(t_k)\right).
$$

This gives a feasible position reference and naturally respects the joint limits
enforced by the IK solver. Joint velocities and accelerations can be estimated by
finite differences if a tracking controller needs $\dot{q}_r$ and $\ddot{q}_r$.

This method is straightforward but sample dependent. If the IK solver switches
between redundant postures, the resulting joint trajectory can become jerky.
Tests should therefore check both endpoint accuracy and joint-reference
continuity.

### Resolved motion rate conversion

Resolved motion rate control converts task velocity into joint velocity:

$$
\dot{q}_r = J^\#(q_r)\dot{x}_r,
$$

where $J^\#$ is a pseudoinverse or damped pseudoinverse. For the endpoint task,
a stable default is

$$
\dot{q}_r =
J^{T}\left(JJ^{T}+\mu I_2\right)^{-1}\dot{x}_r.
$$

Then integrate $\dot{q}_r$ forward in time. This method produces smoother joint
references than independent per-sample IK when the time step is small, but it can
drift from the task path. A practical implementation should occasionally correct
position error with IK or with a task-space feedback term:

$$
\dot{q}_r =
J^\#\left(\dot{x}_r + K_x(x_r - x(q_r))\right).
$$

## 3. Tracking planned joint trajectories

After conversion, the controller receives $q_r(t)$ and often $\dot{q}_r(t)$,
$\ddot{q}_r(t)$. The output is the joint torque vector $\tau$ passed to
`compute_forward_dynamics` or `simulate_robot`.

### Direct joint-space PD control

The simplest dynamic controller is

$$
\tau =
K_p(q_r-q) + K_d(\dot{q}_r-\dot{q}).
$$

$K_p$ and $K_d$ are positive diagonal gain matrices. This controller is easy to
implement as a `simulate_robot` torque callback. It does not compensate for the
arm's inertia or Coriolis terms, so the same gains can behave differently across
configurations.

### Inverse-dynamics feedforward plus feedback

If a full joint reference is available, inverse dynamics can compute the torque
that would realize the reference motion in the nominal model:

$$
\tau_{\mathrm{ff}}
=
H(q_r)\ddot{q}_r + b(q_r,\dot{q}_r).
$$

Then add feedback:

$$
\tau =
\tau_{\mathrm{ff}}
+ K_p(q_r-q)
+ K_d(\dot{q}_r-\dot{q}).
$$

In `skelarm`, this can be implemented by copying a skeleton, setting its
`q`, `dq`, and `ddq` to the reference state, and running
`compute_inverse_dynamics` to read `tau`. This is useful when the planned
trajectory is trusted and the feedback term only needs to correct tracking error.

### Computed torque control

Computed torque control uses the current model to choose a desired joint
acceleration

$$
v =
\ddot{q}_r
+ K_d(\dot{q}_r-\dot{q})
+ K_p(q_r-q),
$$

then commands

$$
\tau = H(q)v + b(q,\dot{q}).
$$

With an exact model and no torque saturation, the tracking error follows the
linear second-order system

$$
\ddot{e}_q + K_d\dot{e}_q + K_p e_q = 0,
\qquad
e_q=q_r-q.
$$

In `skelarm`, computed torque can be implemented with `compute_mass_matrix` and
`compute_coriolis_gravity_vector`, or by setting a copy of the current skeleton's
`ddq` to $v$ and calling `compute_inverse_dynamics`.

!!! note "Gravity convention"
    The default `skelarm` arm moves in a horizontal plane, so gravity is zero
    unless a non-zero `grav_vec` is explicitly supplied. Control formulas should
    be written against the same convention as `compute_forward_dynamics`.

## 4. Human-like reaching by online reference shaping

Planned trajectories explicitly depend on time. That is useful for repeatable
motions, but it can be brittle when an external force or obstacle holds the
endpoint while the planned reference keeps moving. Once the constraint is
released, the accumulated position error can cause a large acceleration.

Seto and Sugihara's reaching-control papers address this problem by combining a
virtual spring-damper endpoint controller with an online shaped equilibrium point:

- Seto and Sugihara (2009),
  ["Online reference shaping with end-point position feedback for large
  acceleration avoidance on manipulator
  control"](https://doi.org/10.1109/IROS.2009.5353889).
- Seto and Sugihara (2009),
  ["Online nonlinear reference shaping with end-point position feedback for
  human-like smooth reaching motion"](https://doi.org/10.1109/ICHR.2009.5379562).

The endpoint spring-damper can be written in a `skelarm` sign convention as

$$
F_d = K_x(x_s-x) - D_x\dot{x},
\qquad
\tau = J^{T}F_d - C_q\dot{q},
$$

where:

- $x_s$ is the shaped endpoint equilibrium;
- $K_x$ is task-space stiffness;
- $D_x$ is task-space damping;
- $C_q$ is joint damping.

The shaped equilibrium $x_s$ is not the final target itself. It is generated
online from the target and current endpoint:

$$
x_s =
\frac{1}{(T_1s+1)(T_2s+1)}
\left(r x^\ast + (1-r)x\right),
\qquad
0 < r \le 1.
$$

The second-order lag filter makes the initial reference velocity and acceleration
smooth. Feeding back the current endpoint $x$ keeps $x_s$ near the arm when the
endpoint is constrained by an external contact, which reduces excessive pushing
force and avoids a large acceleration after release.

The parameter $r$ controls the tradeoff:

- small $r$ places the shaped equilibrium near the current endpoint, reducing
  acceleration and pushing force but slowing convergence;
- large $r$ places it near the target, improving convergence but behaving more
  like a conventional target spring.

The later paper makes $r$ position dependent. Let

$$
d =
\frac{\lVert x^\ast - x\rVert}
{\lVert x^\ast - x_0\rVert},
$$

where $x_0$ is the endpoint position at the start of the reach. A smooth schedule
used by Seto and Sugihara is

$$
r(d) =
\frac{1}{2}\left(
\sqrt{(d-1)^2+4a} - (d-1)
\right),
\qquad
0 < a \ll 1.
$$

At the start, $d \approx 1$ and $r \approx \sqrt{a}$, so the controller starts
gently. Near the target, $d \approx 0$ and $r$ approaches one, improving
convergence. If an external force pushes the endpoint away and $d>1$, $r$
decreases again, making the motion compliant. In implementation, clamp the
computed value to the valid interval $0 < r \le 1$ before using it in the
reference shaper.

!!! note "Reference-shaper state"
    The filter that produces $x_s$ is a dynamic state, not a pure function of the
    current skeleton. With `simulate_robot` and adaptive `solve_ivp`, avoid
    mutating this state inside a torque callback because the integrator may call
    the callback at repeated or non-monotonic trial times. A robust implementation
    should either augment the ODE state with the filter states or provide a
    fixed-step simulation loop for stateful controllers.

## 5. Implementation roadmap

The control layer can be built incrementally.

1. Add trajectory helpers that produce sampled $x_r$, $\dot{x}_r$, and optionally
   $\ddot{x}_r$ for linear, cubic, quintic, and minimum-jerk schedules.
2. Add samplewise IK conversion using `compute_inverse_kinematics`.
3. Add resolved motion rate conversion using damped pseudoinverse velocity
   conversion.
4. Add torque controller helpers for direct joint-space PD, inverse-dynamics
   feedforward plus PD, and computed torque control.
5. Add online reference shaping as a stateful controller with fixed-$r$ shaping,
   position-dependent $r(d)$ shaping, and endpoint spring-damper torque output.
6. Add tests covering endpoint convergence, straight-path tracking,
   computed-torque tracking quality, reduced initial acceleration under online
   shaping, and reduced endpoint force when the endpoint is temporarily
   constrained.

The first implementation should prefer clear, inspectable controllers over a
large abstraction. A plain callable `f(t, skeleton) -> tau` is enough for
stateless PD and computed torque controllers. Stateful reference shaping should
use an explicit controller state object or an augmented simulator state.
