# Trajectory Tracking Control

This chapter collects the control techniques needed to track planned robot-arm
motion in `skelarm`. Unlike the following reaching chapter, it does not assume a
specific goal such as moving the hand to a point. The reference may come from any
task: drawing a curve, tracking a sampled demonstration, following a joint-space
test trajectory, or reaching a target through a planned path.

The usual pipeline is:

1. Plan a desired task-space or joint-space trajectory.
2. Convert task-space references into joint-space references if needed.
3. Track the joint-space reference with a torque controller.

The endpoint trajectory notation is

$$
p_r(t) =
\begin{bmatrix}
p_{r,x}(t) \\
p_{r,y}(t)
\end{bmatrix},
\qquad
p(q) =
\begin{bmatrix}
x(q) \\
y(q)
\end{bmatrix},
$$

where $p_r(t)$ is the desired task-space reference and $p(q)$ is the endpoint
position from forward kinematics. A reaching task is one important application:
choose $p_r(t)$ to move from the initial endpoint $p_0$ to the target $p^\ast$,
then track the resulting reference with the methods below.

## 1. Trajectory planning

A trajectory planner creates references that are smooth enough for the controller
and dynamics. For a task-space path from $p_0$ to $p_1$, a common form is

$$
p_r(t) = p_0 + s(t)(p_1 - p_0),
\qquad
s(0)=0,\quad s(T)=1.
$$

The scalar schedule $s(t)$ controls endpoint velocity and acceleration:

- **Linear interpolation** uses $s=t/T$. It is easy, but has velocity jumps at
  the start and end.
- **Cubic smoothing** uses $s=3u^2-2u^3$ with $u=t/T$. It gives zero endpoint
  velocity.
- **Quintic smoothing** uses $s=10u^3-15u^4+6u^5$. It gives zero endpoint
  velocity and acceleration.
- **Minimum-jerk profiles** use the same quintic time law for rest-to-rest
  point-to-point motion and are a useful baseline for smooth movements.

The planned reference should provide at least position and velocity. Acceleration
is needed for inverse-dynamics feedforward and computed torque control:

$$
p_r(t), \qquad \dot{p}_r(t), \qquad \ddot{p}_r(t).
$$

Joint-space trajectories can be planned in the same way by replacing $p_r$ with
$q_r$. That avoids inverse kinematics during tracking, but it does not directly
shape the endpoint path.

## 2. Task-space to joint-space conversion

`skelarm` actuates joints. A task-space trajectory must therefore become a joint
reference before a joint torque controller can track it.

### IK-based position conversion

At each sample time, solve

$$
p(q_r(t)) \approx p_r(t)
$$

with `compute_inverse_kinematics`. The previous solution is the best seed for the
next sample:

$$
q_r(t_{k+1}) =
\operatorname{IK}\left(p_r(t_{k+1});\ q_0=q_r(t_k)\right).
$$

This gives a feasible position reference and naturally respects the joint limits
enforced by the IK solver. Joint velocities and accelerations can be estimated by
finite differences if a tracking controller needs $\dot{q}_r$ and $\ddot{q}_r$.

The implementation should check joint-reference continuity. Redundant arms can
reach the same endpoint with multiple postures; abrupt posture switching gives a
valid endpoint trajectory but a poor joint trajectory.

### Resolved motion rate conversion

Resolved motion rate control converts task velocity into joint velocity:

$$
\dot{q}_r = J^\#(q_r)\dot{p}_r,
$$

where $J^\#$ is a pseudoinverse or damped pseudoinverse. For the endpoint task,
a stable default is

$$
\dot{q}_r =
J^{T}\left(JJ^{T}+\mu I_2\right)^{-1}\dot{p}_r.
$$

Then integrate $\dot{q}_r$ forward in time. This method can produce smoother
joint references than independent per-sample IK when the time step is small, but
it can drift from the desired task path. A practical implementation should add
task-space feedback:

$$
\dot{q}_r =
J^\#\left(\dot{p}_r + K_{\mathrm{task}}(p_r - p(q_r))\right).
$$

This is velocity-level control, not torque control. In a dynamic simulation it is
usually used to generate $q_r(t)$ and $\dot{q}_r(t)$ for a lower-level torque
controller.

## 3. Joint-space trajectory tracking

After conversion, the tracking controller receives $q_r(t)$ and often
$\dot{q}_r(t)$, $\ddot{q}_r(t)$. The output is the joint torque vector $\tau$
passed to `compute_forward_dynamics` or `simulate_robot`.

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

## 4. Implementation roadmap

The trajectory-tracking layer can be built incrementally:

1. Add trajectory helpers that produce sampled $p_r$, $\dot{p}_r$, and optionally
   $\ddot{p}_r$ for linear, cubic, quintic, and minimum-jerk schedules.
2. Add samplewise IK conversion using `compute_inverse_kinematics`.
3. Add resolved motion rate conversion using damped pseudoinverse velocity
   conversion.
4. Add torque controller helpers for direct joint-space PD, inverse-dynamics
   feedforward plus PD, and computed torque control.
5. Add tests covering endpoint path tracking, joint-reference continuity,
   resolved-rate drift correction, and computed-torque tracking quality.

A plain callable `f(t, skeleton) -> tau` is enough for stateless PD,
feedforward, and computed torque controllers. Controllers with their own dynamic
state need additional care; the reaching chapter discusses this issue for online
reference shaping.
