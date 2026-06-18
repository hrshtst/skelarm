"""Numerical inverse kinematics example for skelarm.

Solve endpoint inverse kinematics for a redundant (3-link) planar arm with the
default Sugihara-style Levenberg-Marquardt solver, then plot the resulting pose
together with the target.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from skelarm import LinkProp, Skeleton, compute_inverse_kinematics, draw_skeleton


def main() -> None:
    """Solve IK for a reachable target and visualize the solution."""
    # 1. Define a redundant 3-link arm.
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(3)]
    skeleton = Skeleton(link_props)

    # 2. Solve for a reachable endpoint target (the solver drives the skeleton there).
    target = np.array([1.5, 1.0])
    result = compute_inverse_kinematics(skeleton, target)

    print(f"status: {result.status} (success={result.success})")
    print(f"iterations: {result.iterations}, residual norm: {result.residual_norm:.3e}")
    print(f"joint angles (deg): {np.rad2deg(result.q)}")

    # 3. Plot the solved pose and the target.
    _fig, ax = plt.subplots(figsize=(7, 7))
    draw_skeleton(ax, skeleton, label="IK solution")
    ax.plot(target[0], target[1], marker="x", color="red", markersize=12, linestyle="none", label="target")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
