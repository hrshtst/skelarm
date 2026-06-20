# Robot Configuration

A robot is defined by a `[skeleton]` section in a TOML file and loaded with
[`Skeleton.from_toml`](../api/skeleton.md). The same file can also carry an
`[initial]` start state and, for a controlled run, `[task]` / `[controller]`
sections (see [Control Configuration](control_configuration.md)).

```python
from skelarm import Skeleton

skeleton = Skeleton.from_toml("examples/four_dof_robot.toml")
```

## Links and the base

The arm is a chain of links. A fixed **base link** (`links[0]`) of length
`base_length` offsets the first joint from the origin along +x; the **actuated
links** follow as `[[skeleton.link]]` entries, one per joint. So a "two-link arm"
has three links in total — the base plus two movable links — and `num_joints` is
the number of `[[skeleton.link]]` entries.

## The `[skeleton]` section

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `base_length` | float | `0.0` | Fixed base-link length in meters (origin → first joint). |
| `link` | array of tables | *required* | One `[[skeleton.link]]` per actuated joint. |

### Per-link keys (`[[skeleton.link]]`)

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `length` | float | *required* | Link length in meters. |
| `mass` | float | *required* | Link mass in kilograms. |
| `inertia` | float | *required* | Moment of inertia about the center of mass (kg·m²). |
| `com` | `[x, y]` | `[0.0, 0.0]` | Center-of-mass offset from the joint, in meters. |
| `limits` | `[min, max]` | `[-180.0, 180.0]` | Joint angle limits in **degrees**. |
| `q0` | float | `0.0` | Per-link initial angle in degrees (soft-deprecated; prefer `[initial].q`). |

`length`, `mass`, and `inertia` are required; the rest have defaults. `com` may be
given as the component keys `rgx` / `rgy` instead of `com = [x, y]`, and `limits`
as `qmin` / `qmax` instead of `limits = [min, max]` — the array forms are
preferred.

!!! note "Joint limits are enforced"
    Limits are stored in radians internally and enforced on the kinematics angle
    setters: a value outside `[min, max]` is clamped into range with a warning. A
    link that omits `limits` defaults to `[-180, 180]` degrees — one full
    revolution — so an unspecified joint is capped, not left unbounded. Limits are
    intentionally **not** enforced inside the dynamics solvers.

## The `[initial]` section

The start state is a *run condition* in its own `[initial]` table, so you can
compare the same robot from different postures by swapping just this block.

| Key | Type | Default | Meaning |
| --- | --- | --- | --- |
| `q` | `[deg, …]` | per-link `q0` (else `0`) | Initial joint angles in degrees, one per joint. |
| `dq` | `[deg/s, …]` | `0` | Initial joint velocities in degrees/second, one per joint. |

`q` and `dq` are converted to radians internally and must each have exactly one
value per joint; a length mismatch with the robot's DOF raises an error. Within a
single file the initial pose is resolved as: all zeros → per-link `q0` →
`[initial].q`. The command-line tools extend this further (`--initial <file>`
then `--pose <degrees>` override the file's `[initial]`).

A separate posture file can also be applied to an already-loaded robot:

```python
skeleton.apply_initial_toml("pose.toml")  # reads only the [initial] table
```

## Example

```toml
[skeleton]
base_length = 0.05  # fixed base link; the first joint sits at (0.05, 0)

[[skeleton.link]]
length = 1.0
mass = 2.0
inertia = 0.5
com = [0.5, 0.0]          # center of mass 0.5 m along the link from the joint
limits = [-180.0, 180.0]  # degrees

[[skeleton.link]]
length = 0.8
mass = 1.0
inertia = 0.1
com = [0.4, 0.0]
# limits omitted -> defaults to [-180, 180]

[initial]
q = [30.0, -45.0]   # degrees, one per joint
# dq = [0.0, 0.0]   # optional, degrees/second
```

See `examples/simple_robot.toml`, `examples/four_dof_robot.toml`, and
`examples/base_offset_robot.toml` for complete files.

## Legacy flat configs

Older configs that put `base_length` and `[[link]]` at the top level (without the
`[skeleton]` table) still load: `Skeleton.from_toml` falls back to reading the
top level when no `[skeleton]` section is present. New configs should use the
nested `[skeleton]` form so the robot can share one file with `[initial]`,
`[task]`, and `[controller]`.

## Related

- [Control Configuration](control_configuration.md) — adding `[task]` /
  `[controller]` for a controlled run.
- [Kinematics](../reference/01_kinematics.md) and
  [Inverse Dynamics](../reference/03_inverse_dynamics.md) — what the link mass
  properties mean physically.
