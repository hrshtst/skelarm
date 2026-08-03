# Record, Replay, and Re-simulate

Every simulator records its run to a self-contained `*.sklog.npz` **state log**:
the robot geometry, the recorded channels (joint angles, velocities, torques,
external tip force), and — for scenario runs — the full scenario config and
resolved run settings. This guide covers what you can do with a log: replay it,
plot it, export it to video, and re-simulate it.

## Replay in the player

```bash
uv run python tools/player.py run.sklog.npz
```

The motion is reconstructed and animated from the log *without* re-running the
dynamics. Scrub the timeline and drive playback from the transport bar (`Space`
play/pause, `←`/`B` and `→`/`F` previous/next frame while paused, `R`/`Home`
back to start, `End` last frame) at a chosen speed (`--speed`), toggle the
centers of mass (`--show-com`), and open per-channel plots with **Plot
channels…**. A recorded external tip force is drawn as a red arrow (**Show
external force**). When the log embeds a task (any scenario simulator records
it), the player draws the task context — the target (the active one emphasized
for multi-target tasks), the periodic curve, or the reference trajectory — each
toggled with **Show target(s)** / **Show reference**.

## Export to video

Pass `--export PATH` to render the replay headlessly to a video or animated GIF
— the format comes from the extension, each frame is drawn by the same canvas
(task overlay, centers of mass, and force arrow included), and `--fps` sets the
output frame rate (`--speed` and `--show-com` apply too):

```bash
uv run python tools/player.py run.sklog.npz --export run.mp4            # headless mp4
uv run python tools/player.py run.sklog.npz --export run.gif --fps 24   # headless animated gif
```

## The three reproducibility tiers

"Reproducible" means different things for different run types. Three tiers, from
strongest to weakest:

1. **Recorded-state playback** — every saved log replays in the player and
   plots/exports exactly as recorded. This always works: the channels *are* the
   run.
2. **Deterministic headless re-simulation** — a `run_scenario` log can be
   re-simulated with `rerun_log`. The deterministic controllers reproduce the
   recorded channels exactly; MPC matches within a small numerical tolerance
   (details below).
3. **Interactive (GUI) runs** — a GUI recording carries the same config and
   resolved settings, but mouse-drag tip forces and any GUI friction shaped the
   recorded motion and are **not replayed** by `rerun_log`: a re-simulation
   gives the *unperturbed* scenario, not the recorded motion. Use playback
   (tier 1) to revisit a perturbed run; the drag force is recorded as the
   `ext_force` channel for analysis.

A log written by `run_scenario` or the interactive scenario simulators embeds —
in the log's `[extra]` metadata — the **original source config** (the full
`[skeleton]` / `[initial]` / `[task]` / `[simulator]` / `[controller]` tables,
exactly as loaded), the resolved run parameters (`dt` / `grav_vec` /
`enforce_limits`, plus `duration` for headless runs — a GUI run is open-ended,
so its log records no duration and a re-run falls back to the task's), and the
`skelarm` / `numpy` / `scipy` versions. `enforce_limits` records the *resolved*
joint-limit choice, so a `--no-joint-limits` override is reproduced on re-run
even though the source config still reads `true`.

## Re-simulate with `rerun_log`

```python
from skelarm import rerun_log
from skelarm.recording import StateLog

log = StateLog.load("reach.sklog.npz")
again = rerun_log(log)  # rebuilds the scenario and re-runs the dynamics
```

Reconstruction reparses the embedded source config, so identical input gives
identical state. The deterministic controllers (PD, computed torque,
inverse-dynamics feedforward, and the reaching controllers) reproduce the
recorded channels **exactly** on the same machine. MPC calls
`scipy.optimize.minimize`, which is deterministic but only bit-identical for the
same `scipy` / BLAS build, so an MPC re-run matches within a small numerical
tolerance rather than exactly.

## Export an editable config for comparison

To tweak parameters and compare, export the embedded config to an editable TOML
and re-run it. The export is the **original source config verbatim**: re-running
it unedited reproduces an unperturbed, config-driven run exactly, while editing
a value gives a controlled variant. Call-time overrides (a `duration=` or
`enforce_limits=` argument, a `--no-joint-limits` flag) live only in the run
metadata — they are honored by `rerun_log` but **not** written into the exported
TOML:

```python
from skelarm import export_scenario_toml, load_scenario, run_scenario
from skelarm.recording import StateLog

export_scenario_toml(StateLog.load("reach.sklog.npz"), "edited.toml")
# ... edit a gain / target / duration in edited.toml ...
variant = run_scenario(load_scenario("edited.toml"))
```

From the command line, `tools/export_config.py` writes the config from a saved
log:

```bash
uv run python tools/export_config.py reach.sklog.npz --output edited.toml
uv run python tools/reaching_simulator.py edited.toml                       # explore the edited scenario in the GUI
uv run python tools/reaching_simulator.py edited.toml --save edited.sklog.npz   # or re-run it headlessly
```

!!! note "What is not captured"
    A controller built programmatically (not from a config) has no embedded
    config, so its run is recorded without reproduction metadata and `rerun_log`
    (and `export_scenario_toml`) reject it. Re-running is available for
    scenarios loaded from TOML.

## Related

- [Run Controlled Scenarios](running_scenarios.md) — producing the logs.
- [Teach and Track Trajectories](teaching_trajectories.md) — logs as motion
  references.
- [Recording API](../api/recording.md) — `StateLog` itself.
