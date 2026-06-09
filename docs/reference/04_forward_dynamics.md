# Forward Dynamics

Forward dynamics is the predictive direction: given the actuator torques, find the
joint accelerations they produce, so the motion can be integrated forward in time.
It is harder than inverse dynamics in general — the constraint forces are unknown —
but for a planar serial chain it can be assembled cleanly from a Lagrangian.

## 1. Lagrangian and the equation of motion

The arm is gravity-free and elasticity-free, so it stores no potential energy and
its Lagrangian equals the kinetic energy

$$
K = \frac{1}{2} \sum_{k=1}^{n} \left\{ m_k (\dot{x}_{Gk}^2 + \dot{y}_{Gk}^2) + I_k \dot{\theta}_k^2 \right\}.
$$

Applying Lagrange's equation $\frac{d}{dt}\frac{\partial K}{\partial \dot{\theta}_i} - \frac{\partial K}{\partial \theta_i} = n_i$
(with $n_i$ the generalized force for the absolute angle $\theta_i$) produces, after
some algebra, the matrix form

$$
H_\theta \ddot{\theta} + b_\theta = n.
$$

The derivation rests on a set of geometric coefficients $h_{xkj}, h_{ykj}$ — the
contribution of joint $j$ to the position of the CoM of link $k$ — which satisfy
the convenient identities $\dot{h}_{xkj} = -h_{ykj}\dot{\theta}_j$,
$\dot{h}_{ykj} = h_{xkj}\dot{\theta}_j$, and $\partial h_{xkj}/\partial \theta_j = -h_{ykj}$,
$\partial h_{ykj}/\partial \theta_j = h_{xkj}$.

## 2. From absolute angles to joint coordinates

The system is actuated in **joint** coordinates $q$, related to the absolute
angles by $\theta = T q$, where $T$ is the lower-triangular matrix of ones
($\theta_i = \sum_{j \le i} q_j$). Mapping the generalized force $n$ onto the
actuator torque $\tau$ and an external load $f_E$ via the principle of virtual
work gives $\tau + J_E^{T} f_E = T^{T} n$, and hence the equation of motion in
joint space:

$$
H(q)\, \ddot{q} + b(q, \dot{q}) = \tau + J_E^{T}(q) f_E.
$$

Here:

- $H = T^{T} H_\theta T \in \mathbb{R}^{n \times n}$ is the **system inertia
  matrix** (symmetric, positive definite),
- $b = T^{T} b_\theta \in \mathbb{R}^{n}$ is the **system bias force vector**
  (the velocity-dependent centripetal/Coriolis terms),
- $\tau$ is the actuator torque vector,
- $J_E$ is the Jacobian of the external-force application point $p_E$, so
  $\dot{p}_E = J_E \dot{q}$.

In closed form, the entries are

$$
h_{ij} = \sum_{k=\max(i,j)}^{n} \left[ m_k \{ (x_{Gk} - x_{i-1})(x_{Gk} - x_{j-1}) + (y_{Gk} - y_{i-1})(y_{Gk} - y_{j-1}) \} + I_k \right],
$$

$$
b_i = \sum_{k=i}^{n} m_k \sum_{j=1}^{n} \left\{ (y_{Gk} - y_{i-1}) h_{xkj} - (x_{Gk} - x_{i-1}) h_{ykj} \right\} \dot{\theta}_j^2.
$$

## 3. Solving for the acceleration

Rearranging the equation of motion gives

$$
\ddot{q} = H^{-1} \left( \tau - b + J_E^{T} f_E \right).
$$

Inverting $H$ explicitly is wasteful, so in practice one solves the linear system

$$
H \ddot{q} = \tau - b + J_E^{T} f_E
$$

with a standard solver (Gaussian elimination, or — since $H$ is symmetric
positive definite — Cholesky). See [Numerical Methods](05_numerical_methods.md).

## 4. How `skelarm` builds $H$ and $b$

Rather than evaluating the closed forms directly, `skelarm` assembles $H$ and $b$
by reusing the inverse-dynamics routine, which is algebraically equivalent and
reuses tested code:

- **`compute_mass_matrix`** builds $H$ one column at a time. Setting $\dot{q} = 0$,
  no external load, and a unit acceleration $\ddot{q} = e_j$, inverse dynamics
  returns exactly the $j$-th column of $H$.
- **`compute_coriolis_gravity_vector`** evaluates the bias term by running inverse
  dynamics with $\ddot{q} = 0$. The returned vector carries the Coriolis term and,
  when a non-zero `grav_vec` is supplied, gravity; any external loads stored on the
  links are folded in here with the correct sign.
- **`compute_forward_dynamics`** then solves $H \ddot{q} = \tau - b$ with
  `numpy.linalg.solve`, raising a clear error if $H$ is singular (e.g. a link with
  zero mass and inertia).

!!! note "Gravity and external loads"
    The arm lives on a horizontal plane, so **gravity is ignored by default** and
    only contributes if a non-zero `grav_vec` is passed. The external term
    $J_E^{T} f_E$ enters through the `fex`/`fey` and `rex`/`rey` fields on each
    `Link`; `compute_forward_dynamics` keeps its torque-only signature and reads
    those loads from the skeleton state.

## 5. Consistency with inverse dynamics

Forward and inverse dynamics are inverses of one another, which gives a strong
round-trip test. Starting from a state $(q, \dot{q})$ and a torque $\tau$:

1. forward dynamics yields $\ddot{q} = \text{FD}(q, \dot{q}, \tau)$;
2. feeding that $\ddot{q}$ back through inverse dynamics must recover the original
   $\tau$, i.e. $\text{ID}(q, \dot{q}, \ddot{q}) \approx \tau$.

The `tests/test_dynamics.py` suite asserts this round trip (including with gravity
and external forces), checks that $H$ is symmetric positive definite, and confirms
local energy conservation — $dK/dt = \dot{q}^{T}\tau = 0$ when $\tau = 0$.
