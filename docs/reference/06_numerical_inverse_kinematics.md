# Numerical Inverse Kinematics

The [Kinematics](01_kinematics.md) chapter gives a closed-form inverse for a
two-joint planar arm. That tidy result does not generalize well: with more joints
the arm is usually **redundant**, near straight postures the endpoint Jacobian can
become **singular**, and targets may be outside the workspace. Numerical inverse
kinematics (IK) handles all three cases by iterating from a seed configuration and
driving the endpoint residual down.

This chapter is the implementation basis for a future numerical IK helper in
`skelarm`. It uses the endpoint Jacobian from
[Differential Kinematics](02_differential_kinematics.md) and follows the
Levenberg-Marquardt damping argument in Sugihara (2009),
["Solvability-unconcerned inverse kinematics based on Levenberg-Marquardt method
with robust damping"](https://doi.org/10.1109/ICHR.2009.5379515).

## 1. Problem statement

For the current planar library, the task is endpoint position IK. Let

$$
p(q) =
\begin{bmatrix}
x(q) \\
y(q)
\end{bmatrix},
\qquad
p^\ast =
\begin{bmatrix}
x^\ast \\
y^\ast
\end{bmatrix},
\qquad
e(q) = p^\ast - p(q).
$$

The exact IK problem is

$$
e(q) = 0.
$$

For an arm with $n$ movable joints, $q \in \mathbb{R}^{n}$, $e \in
\mathbb{R}^{2}$, and the endpoint Jacobian is

$$
J(q) = \frac{\partial p}{\partial q} \in \mathbb{R}^{2 \times n}.
$$

Because $e = p^\ast - p(q)$, its derivative is $-J(q)$. A first-order expansion
around the current iterate $q_k$ is therefore

$$
e(q_k + \Delta q) \approx e_k - J_k \Delta q,
$$

where $e_k = e(q_k)$ and $J_k = J(q_k)$. Every solver below chooses a joint step
$\Delta q_k$ that makes the linearized residual smaller, then updates

$$
q_{k+1} = q_k + \alpha_k \Delta q_k,
$$

with a step scale $\alpha_k \in (0, 1]$. A first implementation can use
$\alpha_k = 1$ and add line search later if needed.

## 2. Newton-Raphson method

Newton-Raphson attacks the exact nonlinear equation $e(q)=0$. In the square,
non-singular case, the linearized equation

$$
e_k - J_k \Delta q_k = 0
$$

gives

$$
J_k \Delta q_k = e_k,
\qquad
\Delta q_k = J_k^{-1} e_k.
$$

This is the direct Jacobian iteration:

1. Compute forward kinematics at $q_k$.
2. Compute $e_k = p^\ast - p(q_k)$.
3. Compute $J_k$.
4. Solve $J_k \Delta q_k = e_k$.
5. Set $q_{k+1} = q_k + \alpha_k \Delta q_k$.

Newton-Raphson is attractive because convergence is fast when the seed is close
to a regular solution. Its assumptions are also strict:

- the number of independent constraints must equal the number of unknowns;
- $J_k$ must be square and full rank;
- an exact solution must exist;
- the initial guess must lie in the basin of attraction of that solution.

For `skelarm`, this direct method is mainly useful as a reference path for
two-joint endpoint IK. For $n > 2$, the linear system is underdetermined. At a
singularity, the solve becomes ill-conditioned. For an unreachable target, the
equation has no zero and Newton-Raphson has no meaningful exact root to find.

## 3. Pseudoinverse and Gauss-Newton steps

When $J_k$ is not square, the linearized equation is better viewed as a least
squares problem:

$$
\Delta q_k = \arg\min_{\Delta q}
\frac{1}{2}\lVert e_k - J_k \Delta q \rVert^2.
$$

For a redundant planar arm ($J_k \in \mathbb{R}^{2 \times n}$ with $n>2$) and
full row rank, the minimum-joint-motion solution is

$$
\Delta q_k = J_k^{T}(J_k J_k^{T})^{-1} e_k.
$$

This is the Moore-Penrose pseudoinverse step. It chooses the smallest
$\lVert\Delta q_k\rVert$ among all steps that satisfy the linearized endpoint
equation.

A weighted form is useful once the solver supports more task components. Define
the objective

$$
E(q) = \frac{1}{2} e(q)^{T} W_e e(q),
$$

where $W_e$ is a positive diagonal task-weight matrix. For the current endpoint
task, $W_e = I_2$ is the natural default, while non-uniform weights could
prioritize $x$ or $y$ accuracy. Omitting second-derivative terms in the Hessian of
$E$ gives the Gauss-Newton normal equation

$$
J_k^{T} W_e J_k \Delta q_k = J_k^{T} W_e e_k.
$$

The right-hand side

$$
g_k = J_k^{T} W_e e_k
$$

is the negative gradient direction written with the residual convention
$e = p^\ast - p(q)$. Gauss-Newton is the optimization counterpart of the
pseudoinverse iteration, but it still inherits the singularity problem: if
$J_k^{T} W_e J_k$ loses rank, the normal equation is not safely invertible.

## 4. Singularity-robust inverse

The singularity-robust inverse (SR inverse) of Nakamura and Hanafusa adds a
positive damping term to the pseudoinverse solve. For the unweighted endpoint
case,

$$
\Delta q_k
= J_k^{T}\left(J_k J_k^{T} + \mu I_2\right)^{-1} e_k,
\qquad
\mu > 0.
$$

Equivalently, in the joint-space normal-equation form,

$$
\Delta q_k
= \left(J_k^{T}J_k + \mu I_n\right)^{-1} J_k^{T} e_k.
$$

The two forms are algebraically equivalent for scalar damping; the first solves a
$2 \times 2$ task-space system, while the second matches the Levenberg-Marquardt
form used later.

The SR inverse replaces each singular-direction gain $1/\sigma_i$ with

$$
\frac{\sigma_i}{\sigma_i^2 + \mu}.
$$

Thus very small singular values no longer create huge joint steps. The price is
that the linearized residual is not driven to zero in one step: the solver accepts
a small task error in exchange for bounded joint motion and numerical stability.

The practical weakness is choosing $\mu$. Too small behaves like the unstable
pseudoinverse near singularities. Too large makes the solver slow and overly
conservative even in well-conditioned poses.

## 5. Levenberg-Marquardt method

Levenberg-Marquardt (LM) applies the same damping idea to the weighted nonlinear
least-squares objective $E(q)$. At each iteration, solve

$$
H_k \Delta q_k = g_k,
$$

with

$$
H_k = J_k^{T} W_e J_k + W_n,
\qquad
g_k = J_k^{T} W_e e_k.
$$

$W_n$ is a positive diagonal joint-space damping matrix. Because $W_n$ is
positive definite, $H_k$ is positive definite even when $J_k$ is rank deficient,
so the step is well-defined.

LM also has a useful local minimization interpretation. The step minimizes

$$
\frac{1}{2} r_k^{T} W_e r_k
+
\frac{1}{2} \Delta q_k^{T} W_n \Delta q_k
\quad \text{where} \quad
r_k = e_k - J_k \Delta q_k.
$$

The first term asks the linearized endpoint error to be small. The second term
penalizes large joint changes. This is why LM behaves well for redundant arms:
among steps with similar task error, it favors smaller joint displacement.

With $W_e = I$ and $W_n = \mu I$, LM reduces to the SR inverse / damped
least-squares update. The distinction is mostly one of viewpoint:

- SR inverse emphasizes regularizing the Jacobian inverse near singularities.
- LM emphasizes minimizing a nonlinear residual with a trust-region-like penalty
  on the joint step.

## 6. Sugihara's robust damping weight

Sugihara's key proposal is to make the LM damping depend on the current residual
energy:

$$
W_n = E_k I_n + \overline{W}_n,
\qquad
E_k = \frac{1}{2} e_k^{T} W_e e_k,
$$

where $\overline{W}_n$ is a small positive diagonal bias. A simple default is

$$
\overline{W}_n = \bar{w} I_n.
$$

Sugihara reported $\bar{w}=10^{-3}$ as a robust value in his experiments, but
`skelarm` should expose this as a solver option because joint units, link scales,
and task weights affect the numerical scale.

The conditioning argument is easiest to see from the singular value decomposition
of the weighted Jacobian. For the redundant case, use a full right-singular-vector
matrix $V \in \mathbb{R}^{n \times n}$ and a padded diagonal matrix
$D \in \mathbb{R}^{n \times n}$ whose diagonal entries are the squared singular
values of $W_e^{1/2}J_k$, with zeros in any null-space directions:

$$
W_e^{1/2} J_k = U \Sigma V^{T},
\qquad
D = \operatorname{diag}(\sigma_1^2, \dots, \sigma_r^2, 0, \dots, 0),
$$

where $r = \operatorname{rank}(J_k)$ and the nonzero singular values are ordered
$\sigma_1 \ge \dots \ge \sigma_r > 0$.

For scalar bias $\overline{W}_n = \bar{w}I_n$,

$$
H_k
= V\left(D + (E_k + \bar{w})I_n\right)V^{T},
$$

so the condition number is

$$
\kappa(H_k)
=
\frac{\sigma_1^2 + E_k + \bar{w}}
{\sigma_{\min}^2 + E_k + \bar{w}},
$$

where $\sigma_{\min}$ is the smallest padded singular value. For redundant
endpoint IK, $\sigma_{\min}=0$ whenever the arm has a Jacobian null space.

This explains the behavior Sugihara wanted:

- **Solvable target, regular solution.** As the iterate approaches the solution,
  $E_k \to 0$, so the damping approaches the small bias $\overline{W}_n$. The
  method behaves like lightly damped Gauss-Newton / LM and can converge quickly.
- **Solvable target near a singularity.** The bias $\overline{W}_n$ keeps $H_k$
  from degenerating even when $\sigma_{\min} \approx 0$. Since $g_k \approx 0$
  near a solution, a small positive bias is usually enough to prevent unstable
  steps without blocking convergence.
- **Unreachable target.** The residual cannot vanish, so $E_k$ stays positive.
  Larger residuals automatically produce stronger damping. As $E_k$ dominates
  the singular values, $\kappa(H_k)$ moves toward $1$, and the step magnitude is
  suppressed instead of exploding.

This is why Sugihara calls the method "solvability-unconcerned": the same
iteration can be used whether the exact equation is solvable, redundant,
singular, or unreachable. If the target is reachable, the residual should go to
zero. If it is unreachable, the solver should return the configuration that
minimizes the weighted residual while keeping joint deviations small.

Sugihara notes that a line search is the formal way to strengthen global
convergence. His experiments found the residual-based damping reliable even
without one. For `skelarm`, the first implementation can start without line
search, but the solver should record enough status information to add it later.

## 7. Recommended `skelarm` solver behavior

A future `compute_inverse_kinematics` helper should be explicit about which step
rule it uses. The recommended default is Sugihara-style LM because it handles the
largest set of practical cases.

Suggested method names:

| Method | Step equation | Use |
| --- | --- | --- |
| `"nr"` | $J\Delta q=e$ | Square, full-rank Newton-Raphson reference path for two-joint IK |
| `"pseudoinverse"` | $J^{T}(JJ^{T})^{-1}e$ | Redundant, well-conditioned reference path |
| `"sr_inverse"` | $J^{T}(JJ^{T}+\mu I)^{-1}e$ | Damped least-squares baseline |
| `"lm"` | $(J^{T}W_eJ+W_n)\Delta q=J^{T}W_e e$ | General weighted LM |
| `"lm_sugihara"` | LM with $W_n=E_kI+\overline{W}_n$ | Recommended default |

The loop should be:

1. Copy or set the seed $q_0$.
2. Run `compute_forward_kinematics(skeleton)`.
3. Read the endpoint from `skeleton.links[-1].xe, skeleton.links[-1].ye`.
4. Compute $e_k = p^\ast - p(q_k)$ and $E_k = \frac{1}{2}e_k^TW_e e_k$.
5. Stop with success if $\lVert e_k\rVert \le \varepsilon_e$.
6. Build $J_k$ with `compute_jacobian(skeleton)`.
7. Compute $\Delta q_k$ with the selected method.
8. Stop with stagnation if $\lVert\Delta q_k\rVert \le \varepsilon_q$ or if the
   residual improvement is below a tolerance.
9. Apply $q_{k+1}=q_k+\alpha_k\Delta q_k$.
10. Clamp or reject values outside each joint's `[qmin, qmax]`.
11. Repeat until success, stagnation, or `max_iterations`.

The result should report at least:

- final `q`;
- final endpoint position;
- final residual vector and residual norm;
- number of iterations;
- status such as `"converged"`, `"unreachable_or_stalled"`, `"singular"`, or
  `"max_iterations"`;
- whether joint limits were encountered.

## 8. Joint limits and unreachable targets

Joint limits make the optimization constrained. A simple first implementation can
apply the existing joint clamping after each proposed step, but clamping changes
the actual step and can cause stagnation at a limit. The solver should therefore
check the residual after clamping and report failure or partial success instead
of looping indefinitely.

For unreachable targets, success should not mean "zero residual." It should mean
one of:

- the target was reached within $\varepsilon_e$;
- the solver found a stationary residual minimum, reported with a nonzero
  residual norm.

Sugihara-style LM is useful here because the damping grows with residual energy
instead of relying on the caller to know in advance whether the target is inside
the workspace.

## 9. Testing targets for implementation

The implementation should be developed with red-green-refactor tests. Useful
tests include:

- a two-joint reachable target matches the analytic IK endpoint within tolerance;
- a redundant arm reaches the same endpoint from different seeds with small
  residuals;
- a fully stretched or nearly singular seed does not produce an enormous step
  when using SR inverse or LM;
- an unreachable target terminates with a nonzero residual and a clear status;
- joint limits are respected after every accepted step;
- for targets generated from random valid configurations, `FK(IK(FK(q)))`
  returns the original endpoint within tolerance.
