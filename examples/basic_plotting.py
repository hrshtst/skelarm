"""Demonstrate the basic usage of the skelarm plotting functions."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from skelarm import (
    LinkProp,
    Skeleton,
    compute_forward_kinematics,
    draw_skeleton,
    plot_trajectory,
)


def main() -> None:
    """Run the basic plotting example."""
    # 1. Define the robot arm
    link_props = [
        LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=0.8, i=0.8, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)  # Pass as List[LinkProp]

    # 2. Set some joint angles for visualization
    skeleton.q = np.array([np.pi / 4, np.pi / 2])  # 45 degrees, then 90 degrees relative

    # 3. Compute forward kinematics to get joint and tip positions
    compute_forward_kinematics(skeleton)

    # 4. Create a figure and axes for plotting
    _fig, ax = plt.subplots(figsize=(8, 8))

    # 5. Draw the skeleton
    draw_skeleton(ax, skeleton, color="green")

    # 6. Generate a dummy trajectory for demonstration
    time = np.linspace(0, 2 * np.pi, 100)
    trajectory_x = np.sin(time) + 1.5
    trajectory_y = np.cos(time) + 0.5
    plot_trajectory(ax, trajectory_x, trajectory_y)

    # 7. Show the plot
    plt.show()


if __name__ == "__main__":
    main()
