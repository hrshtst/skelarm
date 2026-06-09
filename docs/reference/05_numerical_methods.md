# Numerical Methods

Two numerical building blocks turn the dynamics equations into a running
simulation: a **linear solver** that extracts the joint acceleration from the
equation of motion at each instant, and an **ODE integrator** that advances the
state through time. `skelarm` leans on mature library routines for both, but the
classic methods behind them are worth seeing.

## 1. Linear equation solver

The [forward dynamics](04_forward_dynamics.md) chapter leaves a linear system
$H \ddot{q} = \tau - b + J_E^{T} f_E$ to solve for $\ddot{q}$ at every step — a
problem of the general form $A x = b$. Because $H$ is symmetric positive definite,
it can be solved robustly (Cholesky is the natural specialized choice); `skelarm`
delegates to NumPy's `numpy.linalg.solve`, an LU-based LAPACK routine. The
textbook method underlying such solvers is **Gaussian elimination**.

### Gaussian elimination

The idea is to reduce $A$ to triangular form and then read off the unknowns:

1. **Forward elimination.** For each pivot column $k = 1, \dots, n-1$ and each row
   $i > k$, subtract a multiple $f = A_{ik}/A_{kk}$ of row $k$ from row $i$ so that
   $A_{ik}$ becomes zero, applying the same operation to the right-hand side:
   $A_{ij} \leftarrow A_{ij} - f A_{kj}$ and $b_i \leftarrow b_i - f b_k$. This
   leaves an upper-triangular system.
2. **Back substitution.** Solve from the last row upward,
   $x_i = \bigl(b_i - \sum_{j>i} A_{ij} x_j\bigr) / A_{ii}$.

Robust implementations add **pivoting** — reordering rows so the pivot $|A_{kk}|$
is large — for numerical stability. The equation of motion's $H$ is regular,
symmetric, and positive definite, so even a naive pivot-free elimination is safe
here.

## 2. Time integration

Simulating motion means integrating the joint acceleration twice, to velocity and
then position. Stacking the state as $X = [q^{T}, \dot{q}^{T}]^{T}$ recasts the
second-order dynamics as a first-order system $\dot{X} = f(t, X)$, whose velocity
half is $\dot{q}$ and whose acceleration half is the
[forward-dynamics](04_forward_dynamics.md) solve. `skelarm` integrates it with
SciPy's `scipy.integrate.solve_ivp` using the adaptive `RK45` (Dormand–Prince)
method, which adjusts its step size automatically to control the error. The two
fixed-step schemes below are the ideas it refines.

### Euler's method

The simplest scheme advances the state along its current slope over a fixed step
$\Delta t$:

$$
\dot{q}_{k+1} = \dot{q}_k + \ddot{q}_k\, \Delta t, \qquad
q_{k+1} = q_k + \dot{q}_k\, \Delta t.
$$

It is easy but only first-order accurate (global error $O(\Delta t)$), and it
drifts off the true trajectory for large steps or stiff dynamics.

### Runge–Kutta (RK4)

The classical fourth-order Runge–Kutta method samples the slope at four points
across the step and blends them, which buys much higher accuracy. For a state $y$
(standing in for the stacked $X$),

$$
y_{k+1} = y_k + \frac{\Delta t}{6}\,(k_1 + 2 k_2 + 2 k_3 + k_4),
$$

with

$$
\begin{aligned}
k_1 &= f(t_k, y_k), \\
k_2 &= f\!\left(t_k + \tfrac{\Delta t}{2},\ y_k + \tfrac{\Delta t}{2} k_1\right), \\
k_3 &= f\!\left(t_k + \tfrac{\Delta t}{2},\ y_k + \tfrac{\Delta t}{2} k_2\right), \\
k_4 &= f\!\left(t_k + \Delta t,\ y_k + \Delta t\, k_3\right).
\end{aligned}
$$

Its global error is $O(\Delta t^{4})$, making it the usual workhorse for smooth
mechanical systems — and the fixed-step sibling of the adaptive RK45 that
`skelarm` actually uses.
