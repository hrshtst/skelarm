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
- **Minimum-jerk profiles** minimize the integral of squared jerk. For the
  rest-to-rest point-to-point problem above, the solution is mathematically the
  same quintic time law as quintic smoothing:

$$
\min_{p_r(t)}
\int_0^T \left\|\dddot{p}_r(t)\right\|^2\,dt
$$

subject to

$$
p_r(0)=p_0,\quad p_r(T)=p_1,\quad
\dot{p}_r(0)=\dot{p}_r(T)=0,\quad
\ddot{p}_r(0)=\ddot{p}_r(T)=0.
$$

With these boundary conditions,

$$
p_r(t)=p_0+\left(10u^3-15u^4+6u^5\right)(p_1-p_0),
\qquad u=t/T.
$$

In the current `skelarm` trajectory-planning problem, therefore, a `quintic`
schedule and a rest-to-rest `minimum_jerk` schedule can share the same
implementation. They become distinct when the problem statement changes: for
example, if endpoint velocity or acceleration is non-zero, if multiple via-points
or multiple movement segments are optimized together, if segment durations are
also optimized, or if the objective includes task constraints such as obstacle
avoidance or effort penalties. In those cases, quintic interpolation is a local
polynomial construction, while minimum jerk is an optimization problem whose
solution may require solving for polynomial coefficients or a larger trajectory
optimization.

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

### Model predictive control

Model predictive control (MPC) is another model-based tracking method, but it is
not the same as computed torque control. Computed torque chooses an instantaneous
model-cancelling torque. MPC repeatedly solves a finite-horizon optimization
problem, applies only the first control input, then replans from the next measured
state.

A simple `skelarm` formulation can start in joint space. Define the discrete
state and input as

$$
z_k =
\begin{bmatrix}
q_k \\
\dot{q}_k
\end{bmatrix},
\qquad
u_k = \tau_k.
$$

Using a fixed control interval $\Delta t$, the prediction model can use
`compute_forward_dynamics`:

$$
\ddot{q}_k = \operatorname{FD}(q_k,\dot{q}_k,\tau_k),
$$

with a simple semi-implicit Euler update

$$
\dot{q}_{k+1} = \dot{q}_k + \Delta t\,\ddot{q}_k,
\qquad
q_{k+1} = q_k + \Delta t\,\dot{q}_{k+1}.
$$

For a horizon of $N$ steps, a basic tracking objective is

$$
\begin{aligned}
\min_{\tau_0,\dots,\tau_{N-1}}\quad
&\sum_{k=0}^{N-1}
\left(
e_{q,k}^{T}Q_q e_{q,k}
+ e_{\dot{q},k}^{T}Q_{\dot{q}} e_{\dot{q},k}
+ \tau_k^{T}R\tau_k
\right)
+ e_{q,N}^{T}Q_f e_{q,N},
\end{aligned}
$$

where

$$
e_{q,k}=q_k-q_{r,k},
\qquad
e_{\dot{q},k}=\dot{q}_k-\dot{q}_{r,k}.
$$

The first implementation can use torque bounds and soft joint-limit penalties.
Hard constraints can be added once the optimizer interface is stable:

$$
q_{\min} \le q_k \le q_{\max},
\qquad
\tau_{\min} \le \tau_k \le \tau_{\max}.
$$

In Python, a practical first version can use `scipy.optimize.minimize` with a
small horizon, flattened torque sequence, and warm start from the previous
solution. The controller loop is:

1. Read the current state $(q,\dot{q})$.
2. Slice the next $N$ samples from the reference trajectory.
3. Optimize $\tau_0,\dots,\tau_{N-1}$ by rolling out `compute_forward_dynamics`.
4. Apply only $\tau_0$.
5. Shift the optimized sequence by one step to warm-start the next solve.

Task-space MPC can be added by including an endpoint term,

$$
(p(q_k)-p_{r,k})^{T}Q_p(p(q_k)-p_{r,k}),
$$

but joint-space MPC is the cleaner first target because it avoids mixing IK,
trajectory conversion, and constrained optimal control in the same initial
implementation.

!!! note "Fixed-step simulation"
    MPC is naturally discrete and stateful. The existing `simulate_robot` helper
    uses adaptive `solve_ivp`, which may call a torque callback multiple times per
    output interval. A robust MPC implementation should use a fixed-step
    simulation loop or an MPC-specific simulator that calls the optimizer once per
    control interval.

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
5. Add a fixed-step simulation path for stateful controllers.
6. Add a simple joint-space MPC tracker with torque bounds and warm starts.
7. Add tests covering endpoint path tracking, joint-reference continuity,
   resolved-rate drift correction, computed-torque tracking quality, and MPC
   torque-bound handling.

A plain callable `f(t, skeleton) -> tau` is enough for stateless PD,
feedforward, and computed torque controllers. Controllers with their own dynamic
state, such as MPC and online reference shaping, need a fixed control interval or
an augmented simulator state.
