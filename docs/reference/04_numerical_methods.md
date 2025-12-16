# Numerical Methods

This document describes the numerical methods used to solve the mathematical models of the robot arm.

## 1. Linear Equation Solver

To solve the forward dynamics equation $H \ddot{q} = \text{RHS}$, we solve a linear system of the form $Ax = b$.
Since $H$ is symmetric and positive definite, robust methods like Cholesky decomposition could be used, but for general purposes, **Gaussian Elimination** is implemented.

### Gaussian Elimination
The process involves transforming the augmented matrix $[A | b]$ into an upper triangular form (forward elimination) and then performing back substitution to find $x$.

**Algorithm:**
1.  **Forward Elimination:**
    For each column $k$ from 1 to $n-1$:
    - For each row $i$ from $k+1$ to $n$:
        - Compute factor $f = A_{ik} / A_{kk}$.
        - Subtract row $k$ multiplied by $f$ from row $i$.
        - $A_{ij} \leftarrow A_{ij} - f \cdot A_{kj}$ for $j=k \dots n$.
        - $b_i \leftarrow b_i - f \cdot b_k$.

2.  **Back Substitution:**
    For row $i$ from $n$ down to 1:
    - $x_i = (b_i - \sum_{j=i+1}^n A_{ij} x_j) / A_{ii}$.

*Note: Pivoting (swapping rows to maximize the pivot element $|A_{kk}|$) is often necessary for numerical stability, though simple implementations might skip it if $A$ is known to be well-behaved.*

## 2. Differential Equation Solver (Integration)

To simulate the motion over time, we integrate the joint acceleration $\ddot{q}$ to get velocity $\dot{q}$ and position $q$.
The state of the system is $X = [q^T, \dot{q}^T]^T$. The system dynamics can be written as $\dot{X} = f(t, X)$.

### Euler's Method
The simplest numerical integration method. It approximates the next state based on the current slope.

$$
\begin{aligned}
\dot{q}_{k+1} &= \dot{q}_k + \ddot{q}_k \Delta t \\
q_{k+1} &= q_k + \dot{q}_k \Delta t
\end{aligned}
$$
where $\Delta t$ is the time step.
**Error:** $O(\Delta t)$ (Global error). It is often unstable or inaccurate for stiff systems or large steps.

### Runge-Kutta Method (RK4)
The classical 4th-order Runge-Kutta method provides much higher accuracy by sampling the slope at four points within the time step.

Let $y_k$ be the state variable (e.g., $q$ or $\dot{q}$).
$$
y_{k+1} = y_k + \frac{\Delta t}{6} (k_1 + 2k_2 + 2k_3 + k_4)
$$

where:
- $k_1 = f(t_k, y_k)$
- $k_2 = f(t_k + \frac{\Delta t}{2}, y_k + \frac{\Delta t}{2} k_1)$
- $k_3 = f(t_k + \frac{\Delta t}{2}, y_k + \frac{\Delta t}{2} k_2)$
- $k_4 = f(t_k + \Delta t, y_k + \Delta t k_3)$

For our second-order system $\ddot{q} = g(q, \dot{q}, \tau)$, we apply this to the state vector $X = [q, \dot{q}]$.

**Error:** $O(\Delta t^4)$. This is the standard method for simulating smooth mechanical systems.
