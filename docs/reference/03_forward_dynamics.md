# Forward Dynamics

This document details the forward dynamics formulation. Forward dynamics finds the acceleration $\ddot{q}$ produced by a given set of actuation forces $\tau$.

## 1. Equation of Motion

The equation of motion for the system is derived using Lagrange's method and is expressed in the standard form:

$$ H(q) \ddot{q} + b(q, \dot{q}) = \tau + J_E^T(q) f_E $$

where:
- $H(q) \in \mathbb{R}^{n \times n}$ is the **System Inertia Matrix** (symmetric, positive definite).
- $b(q, \dot{q}) \in \mathbb{R}^{n}$ is the **System Bias Force Vector** (containing Coriolis, centrifugal, and gravity terms).
- $\tau \in \mathbb{R}^{n}$ is the vector of joint actuation torques.
- $f_E$ is the external force vector acting at a specific point.
- $J_E$ is the Jacobian of the external force application point.

## 2. Derivation and Components

### Kinetic Energy
The kinetic energy $K$ of the system is:
$$ K = \frac{1}{2} \sum_{k=1}^n \left\{ m_k (\dot{x}_{Gk}^2 + \dot{y}_{Gk}^2) + I_k \dot{\theta}_k^2 \right\} $$

### System Inertia Matrix ($H$)
The elements $h_{ij}$ of the matrix $H$ are computed as:

$$ h_{ij} = \sum_{k=\max(i,j)}^n \left[ m_k \{ (x_{Gk} - x_{i-1})(x_{Gk} - x_{j-1}) + (y_{Gk} - y_{i-1})(y_{Gk} - y_{j-1}) \} + I_k \right] $$

This matrix represents the inertial coupling between joints.

### Bias Force Vector ($b$)
The elements $b_i$ of the vector $b$ represent the forces arising from velocity-dependent terms (Coriolis and Centrifugal):

$$ b_i = \sum_{k=i}^n m_k \sum_{j=1}^n \left\{ (y_{Gk} - y_{i-1}) h_{xkj} - (x_{Gk} - x_{i-1}) h_{ykj} \right\} \dot{\theta}_j^2 $$

where $h_{xkj}$ and $h_{ykj}$ are geometric coefficients defined in the kinematic analysis.

## 3. Solving for Acceleration

To simulate motion, we need to solve for $\ddot{q}$:

$$ \ddot{q} = H^{-1} (\tau - b + J_E^T f_E) $$

Since inverting $H$ directly is computationally expensive and unnecessary, we typically solve the linear system:

$$ H \ddot{q} = \text{RHS} $$
where $\text{RHS} = \tau - b + J_E^T f_E$. This is done using a linear equation solver (e.g., Gauss elimination).

## 4. Consistency Check

A crucial validation step is to ensure that the Forward Dynamics and Inverse Dynamics implementations are consistent.
- **Inverse Dynamics:** Given $q, \dot{q}, \ddot{q}$, calculate $\tau$.
- **Forward Dynamics:** Given $q, \dot{q}, \tau$, calculate $\ddot{q}$.

If we compute $\tau$ from a known $\ddot{q}$ using Inverse Dynamics, and then feed that $\tau$ into Forward Dynamics, we should recover the original $\ddot{q}$ (within numerical limits).

The function `skeleton_fd` should implement the construction of $H$ and $b$ and solve for $\ddot{q}$.
