# Differential Kinematics

This document details the differential kinematics for a planar serial chain mechanism (e.g., a robot arm), as implemented in the `skelarm` project. It covers the relationship between joint velocities/accelerations and endpoint velocities/accelerations, as well as the computation of the Jacobian matrix.

## 1. Overview

Forward kinematics maps a set of joint angles to the endpoint position. Differential kinematics extends this to describe the relationship between the *rate of change* of joint angles (joint velocities) and the endpoint velocity.

Consider a planar $n$-link arm.
- Let $q_i$ be the joint angle of the $i$-th joint relative to the previous link.
- Let $\theta_i$ be the absolute angle of the $i$-th link.
- Let $(x_i, y_i)$ be the position of the $i$-th joint (or endpoint of the $i$-th link).
- Let $l_i$ be the length of the $i$-th link.

## 2. Recursive Velocity and Acceleration

### Velocity
The velocity of each link and joint can be computed recursively from the base (link 0) to the endpoint (link $n$).

**Base conditions (fixed base):**
$$
\begin{aligned}
\dot{\theta}_0 &= 0 \\
\dot{x}_0 &= 0 \\
\dot{y}_0 &= 0
\end{aligned}
$$

**Recursive step (for $i = 1, \dots, n$):**
$$
\begin{aligned}
\dot{\theta}_i &= \dot{\theta}_{i-1} + \dot{q}_i \\
\dot{x}_i &= \dot{x}_{i-1} - \dot{\theta}_i l_i \sin \theta_i \\
\dot{y}_i &= \dot{y}_{i-1} + \dot{\theta}_i l_i \cos \theta_i
\end{aligned}
$$

### Acceleration
Similarly, acceleration can be computed recursively.

**Base conditions:**
$$
\begin{aligned}
\ddot{\theta}_0 &= 0 \\
\ddot{x}_0 &= 0 \\
\ddot{y}_0 &= 0
\end{aligned}
$$

**Recursive step (for $i = 1, \dots, n$):**
$$
\begin{aligned}
\ddot{\theta}_i &= \ddot{\theta}_{i-1} + \ddot{q}_i \\
\ddot{x}_i &= \ddot{x}_{i-1} - \dot{\theta}_i^2 l_i \cos \theta_i - \ddot{\theta}_i l_i \sin \theta_i \\
\ddot{y}_i &= \ddot{y}_{i-1} - \dot{\theta}_i^2 l_i \sin \theta_i + \ddot{\theta}_i l_i \cos \theta_i
\end{aligned}
$$

## 3. Jacobian Matrix

The relationship between the endpoint velocity $(\dot{x}, \dot{y})$ and the joint velocities $\dot{q} = [\dot{q}_1, \dots, \dot{q}_n]^T$ is linear and defined by the Jacobian matrix $J$:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = J \dot{q} = \begin{bmatrix} j_{x1} & \cdots & j_{xn} \\ j_{y1} & \cdots & j_{yn} \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \vdots \\ \dot{q}_n \end{bmatrix}
$$

Or effectively:
$$
\dot{x} = \sum_{i=1}^n j_{xi} \dot{q}_i, \quad \dot{y} = \sum_{i=1}^n j_{yi} \dot{q}_i
$$

### Computing Jacobian Elements
The elements of the Jacobian matrix can be computed recursively *backward* from the endpoint to the base.

**Base conditions (virtual link $n+1$):**
$$j_{x(n+1)} = 0, \quad j_{y(n+1)} = 0$$

**Recursive step (for $i = n, \dots, 1$):**
$$
\begin{aligned}
j_{xi} &= j_{x(i+1)} - l_i \sin \theta_i \\
j_{yi} &= j_{y(i+1)} + l_i \cos \theta_i
\end{aligned}
$$

Alternatively, geometrically:
$$
\begin{aligned}
j_{xi} &= -(y_n - y_{i-1}) \\
j_{yi} &= x_n - x_{i-1}
\end{aligned}
$$
where $(x_n, y_n)$ is the endpoint position and $(x_{i-1}, y_{i-1})$ is the position of the $(i-1)$-th joint (the origin of link $i$).

## 4. Centripetal and Coriolis Forces

The acceleration relationship can be expressed as:

$$
\begin{aligned}
\ddot{x} &= \sum_{i=1}^n (j_{xi} \ddot{q}_i + h_{xi} \dot{q}_i) \\
\ddot{y} &= \sum_{i=1}^n (j_{yi} \ddot{q}_i + h_{yi} \dot{q}_i)
\end{aligned}
$$

where $h_{xi}$ and $h_{yi}$ represent the basis for centripetal and Coriolis accelerations.

**Recursive computation (backward):**
$$
\begin{aligned}
h_{xi} &= -(\dot{y}_n - \dot{y}_{i-1}) = -\sum_{j=i}^n j_{yj} \dot{q}_j \\
h_{yi} &= \dot{x}_n - \dot{x}_{i-1} = \sum_{j=i}^n j_{xj} \dot{q}_j
\end{aligned}
$$

## 5. Implementation Notes

The `skeleton_t` and `link_t` structures in the codebase should maintain these values.
- **Forward pass (`skeleton_update_state`)**: Compute $\theta_i, x_i, y_i$ and their derivatives.
- **Backward pass (`skeleton_update_jacobi`)**: Compute Jacobian columns $j_{xi}, j_{yi}$ and Coriolis bases $h_{xi}, h_{yi}$.

This allows for efficient computation of endpoint velocity and acceleration given the joint state.
