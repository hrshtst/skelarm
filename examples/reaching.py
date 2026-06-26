"""Reaching simulation example for skelarm.

Loads the combined scenario in ``reach.toml`` (robot + start pose + task +
controller), simulates the controlled reach toward the task-space target, prints
the final endpoint error, and plots the tip trajectory with the goal and final
pose. For an interactive version (drag to apply external forces), run the
``tools/reaching_simulator.py`` GUI on the same config.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from skelarm import draw_skeleton, draw_target, load_scenario, plot_trajectory, run_scenario


def main() -> None:
    """Run a reaching simulation from reach.toml and plot the result."""
    # 1. Load the combined scenario (robot + initial pose + task + controller).
    config_path = Path(__file__).parent / "reach.toml"
    scenario = load_scenario(config_path)
    print(f"Loaded {scenario.skeleton.num_joints}-DOF robot, {type(scenario.controller).__name__} controller")

    # 2. Simulate the controlled reach (fixed-step loop; see skelarm.run_scenario).
    log = run_scenario(scenario)

    # 3. Reconstruct the tip trajectory from the recorded joint angles.
    arm = scenario.skeleton.clone()
    xs: list[float] = []
    ys: list[float] = []
    for q in log.channel("q"):
        arm.q = q
        xs.append(arm.links[-1].xe)
        ys.append(arm.links[-1].ye)

    target = scenario.task.require_target()
    error = float(np.hypot(xs[-1] - target[0], ys[-1] - target[1]))
    print(f"start ({xs[0]:.3f}, {ys[0]:.3f}) m -> target ({target[0]:.3f}, {target[1]:.3f}) m")
    print(f"final ({xs[-1]:.3f}, {ys[-1]:.3f}) m, error {error * 1000:.2f} mm in {len(log)} frames")

    # 4. Plot the final pose, the tip path, and the goal.
    import matplotlib.pyplot as plt

    arm.q = log.channel("q")[-1]
    _, ax = plt.subplots()
    draw_skeleton(ax, arm, title="Reaching simulation")
    plot_trajectory(ax, np.array(xs), np.array(ys), title=None)
    draw_target(ax, target, color=scenario.task.color, tolerance=scenario.task.tolerance, label="target")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
