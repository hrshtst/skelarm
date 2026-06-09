# Numerical Inverse Kinematics

!!! warning "Rough draft"
    This chapter is an early draft sketching the approach; the title and details
    are still to be refined, and the method is not yet implemented in `skelarm`.

The [Kinematics](01_kinematics.md) chapter solved inverse kinematics in closed
form for a two-joint arm. That tidy solution does not survive past two joints:
arms with more joints are typically **redundant** — infinitely many
configurations reach the same endpoint — and no general algebraic inverse exists.
The standard remedy is to solve the problem **numerically**, iterating from an
initial guess until the endpoint error is small. The machinery needed is already
in place: the endpoint [Jacobian](02_differential_kinematics.md) and a
[linear solver](05_numerical_methods.md).

## 1. Problem statement

Let $p(q) = (x(q), y(q))$ be the forward-kinematics map and $p^\ast$ the desired
endpoint. We seek joint angles $q$ with

$$
p(q) = p^\ast, \qquad \text{equivalently} \qquad e(q) \equiv p^\ast - p(q) = 0.
$$

## 2. Newton / Jacobian iteration

Near a current guess $q_k$, forward kinematics is well approximated by its first
order (Jacobian) expansion:

$$
p(q_k + \Delta q) \approx p(q_k) + J(q_k)\, \Delta q,
$$

so driving the endpoint toward $p^\ast$ amounts to solving the linear system

$$
J(q_k)\, \Delta q = e(q_k)
$$

for a step $\Delta q$, then updating

$$
q_{k+1} = q_k + \alpha\, \Delta q,
$$

with a step size $\alpha \in (0, 1]$. Iterating until $\lVert e(q_k) \rVert$ falls
below a tolerance yields a configuration that reaches (or approaches) the target.

## 3. Square, redundant, and singular cases

$J$ is the $2 \times n$ endpoint Jacobian, so the shape of the linear solve
depends on the number of joints:

- **$n = 2$ (square).** $J$ is $2 \times 2$; solve $J\,\Delta q = e$ directly (for
  example with the Gaussian elimination of the [linear solver](05_numerical_methods.md)
  chapter) wherever $J$ is non-singular.
- **$n > 2$ (redundant).** The system is under-determined. The minimum-norm step
  uses the right pseudo-inverse,
  $\Delta q = J^{T}(J J^{T})^{-1} e$, which moves the joints as little as possible
  for the requested endpoint change.
- **Near singularities.** $J J^{T}$ becomes ill-conditioned when the arm nears a
  singular posture (e.g. fully stretched), producing huge steps. **Damped least
  squares** (Levenberg–Marquardt) regularizes the solve,
  $\Delta q = J^{T}(J J^{T} + \lambda^{2} I)^{-1} e$,
  trading a little accuracy for stability via the damping factor $\lambda$.

## 4. Algorithm sketch

1. Choose an initial guess $q_0$ (e.g. the current pose).
2. Compute $p(q_k)$ via forward kinematics and the error $e = p^\ast - p(q_k)$.
3. If $\lVert e \rVert < \varepsilon$, stop.
4. Build $J(q_k)$ and solve for $\Delta q$ (direct, pseudo-inverse, or damped).
5. Update $q_{k+1} = q_k + \alpha\, \Delta q$, optionally clamping to joint limits,
   and repeat.

## 5. Practical notes (to expand)

- **Initial guess and local minima.** The iteration converges to a solution near
  the starting pose; different seeds can reach different postures, and a poor seed
  may stall.
- **Unreachable targets.** When $p^\ast$ lies outside the workspace, the residual
  $\lVert e \rVert$ plateaus at a non-zero value; the iteration should report
  failure rather than loop forever.
- **Joint limits.** Clamping or null-space projection keeps the redundant solution
  within the configured `qmin`/`qmax` range.

*Planned implementation:* a `compute_inverse_kinematics` helper that wraps this
loop around the existing `compute_jacobian` and forward-kinematics routines.
