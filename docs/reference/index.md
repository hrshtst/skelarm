# Reference Documentation

This section contains theoretical reference documentation for the `skelarm` robot kinematics and dynamics library.
The content is derived from the project's foundational lessons.

## Contents

1.  [Differential Kinematics](01_differential_kinematics.md)
    -   Recursive position, velocity, and acceleration.
    -   Jacobian matrix formulation.
2.  [Inverse Dynamics](02_inverse_dynamics.md)
    -   Newton-Euler equations.
    -   Force and torque balance recursive calculation.
3.  [Forward Dynamics](03_forward_dynamics.md)
    -   Lagrangian formulation.
    -   System Inertia Matrix ($H$) and Bias Force Vector ($b$).
    -   Equation of Motion: $H\ddot{q} + b = \tau$.
4.  [Numerical Methods](04_numerical_methods.md)
    -   Linear Equation Solver (Gaussian Elimination).
    -   ODE Solver (Euler, Runge-Kutta).
