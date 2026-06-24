"""Periodic-curve tracing example for skelarm.

Loads ``periodic_curve.toml`` (robot + a closed task-space curve + a tracking
controller), starts the arm on the curve, simulates the controlled trace, and plots
the tip path against the reference curve. Change ``[task].curve`` in the config
(``circle``, ``ellipse``, ``lemniscate``, ``vertical_lemniscate``, ``rose``) to trace a
different shape.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from skelarm import (
    build_curve,
    compute_inverse_kinematics,
    draw_skeleton,
    load_scenario,
    plot_trajectory,
    run_scenario,
)


def main() -> None:
    """Run a periodic-curve tracing simulation and plot the traced path."""
    # 1. Load the combined scenario (robot + curve task + tracking controller).
    config_path = Path(__file__).parent / "periodic_curve.toml"
    scenario = load_scenario(config_path)
    curve = build_curve(str(scenario.task.params["curve"]), scenario.task.params)

    # 2. Start the arm on the curve (theta = 0) so it traces cleanly from the first loop.
    compute_inverse_kinematics(scenario.skeleton, curve(0.0))
    print(f"Tracing a '{scenario.task.params['curve']}' with {type(scenario.controller).__name__}")

    # 3. Simulate the controlled trace (fixed-step loop; see skelarm.run_scenario).
    log = run_scenario(scenario)

    # 4. Reconstruct the tip trajectory from the recorded joint angles.
    arm = scenario.skeleton.clone()
    xs: list[float] = []
    ys: list[float] = []
    for q in log.channel("q"):
        arm.q = q
        xs.append(arm.links[-1].xe)
        ys.append(arm.links[-1].ye)
    print(f"traced {len(log)} frames over {scenario.task.duration:.1f} s")

    # 5. Plot the final pose, the traced tip path, and the reference curve.
    import matplotlib.pyplot as plt

    thetas = np.linspace(0.0, 2.0 * np.pi, 240)
    reference = np.array([curve(theta) for theta in thetas])
    arm.q = log.channel("q")[-1]
    _, ax = plt.subplots()
    draw_skeleton(ax, arm, title="Periodic curve tracing")
    ax.plot(reference[:, 0], reference[:, 1], linestyle="--", color="gray", label="reference curve")
    plot_trajectory(ax, np.array(xs), np.array(ys), title=None)
    ax.set_aspect("equal")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
