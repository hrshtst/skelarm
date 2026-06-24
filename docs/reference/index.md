# Reference Documentation

This section contains theoretical reference documentation for the `skelarm` robot kinematics and dynamics library.
The content is derived from the project's foundational lessons.

The arm is modeled around a fixed **base link** of length $l_0$ (the zeroth
link), followed by the actuated links. So a "two-link arm" holds three links in
total; in code, `links[0]` is the base and `links[1:]` are the movable joints.

## Contents

1.  [Kinematics](01_kinematics.md)
    -   Forward kinematics of a two-joint arm.
    -   Analytic inverse kinematics (elbow-up / elbow-down).
2.  [Differential Kinematics](02_differential_kinematics.md)
    -   From a two-joint arm to the general $n$-joint recursion.
    -   Recursive position, velocity, and acceleration.
    -   Jacobian matrix and centripetal/Coriolis basis (with endpoint
        velocity/acceleration helpers).
3.  [Inverse Dynamics](03_inverse_dynamics.md)
    -   Newton-Euler equations and mass properties.
    -   Backward force and torque balance recursion.
    -   Static-equilibrium validation.
4.  [Forward Dynamics](04_forward_dynamics.md)
    -   Lagrangian formulation.
    -   System Inertia Matrix ($H$) and Bias Force Vector ($b$).
    -   Equation of Motion: $H\ddot{q} + b = \tau + J_E^{T} f_E$.
5.  [Numerical Methods](05_numerical_methods.md)
    -   Linear Equation Solver (Gaussian Elimination).
    -   Least-squares, pseudoinverse, and damped least-squares solves.
    -   ODE Solver (Euler, Runge-Kutta).
6.  [Numerical Inverse Kinematics](06_numerical_inverse_kinematics.md)
    -   Newton-Raphson, pseudoinverse, SR inverse, and Levenberg-Marquardt IK.
    -   Sugihara-style residual-based damping for robust numerical IK.
7.  [Trajectory Tracking Control](07_control.md)
    -   Task-space trajectory planning and joint-space tracking.
    -   PD, inverse-dynamics feedforward, and computed torque control.
8.  [Reaching Control](08_reaching_control.md)
    -   Reaching as a trajectory-tracking task.
    -   Virtual spring-damper reaching and Seto/Sugihara online reference
        shaping for human-like motion.
9.  [Trajectory Filtering & Interpolation](09_trajectory_filtering.md)
    -   Linear, natural cubic spline, and barycentric Lagrange interpolation.
    -   First-order and Butterworth low-pass smoothing, applied zero-phase.
