# Architecture

This page maps the `skelarm` class structure: how the robot model, the
controllers, the configuration glue, and the PyQt6 GUI relate to one another. It
complements the per-module API pages (starting with [Skeleton](skeleton.md)) and
the theory chapters that derive each piece.

The codebase has four structures worth seeing together:

- the **robot model** (`Skeleton` / `Link` / `LinkProp`);
- the **controller** hierarchy (the trackers of
  [Trajectory Tracking Control](../reference/07_control.md) and the endpoint
  controllers of [Reaching Control](../reference/08_reaching_control.md));
- the **scenario glue** that turns a TOML config into a runnable controller;
- the **PyQt6 GUI** widgets and windows.

```mermaid
classDiagram
    %% ── Robot model ─────────────────────────────
    class Skeleton {
        +links: list~Link~
        +q  dq  ddq  tau
        +clone()  to_dict()
    }
    class Link {
        +prop: LinkProp
        +q  dq  ddq
    }
    class LinkProp {
        <<dataclass>>
        +length mass inertia qmin qmax
    }
    Skeleton "1" *-- "N" Link
    Link "1" *-- "1" LinkProp

    %% ── Controllers ─────────────────────────────
    class Controller {
        <<abstract>>
        +reset(skeleton)
        +control(t, skeleton)* NDArray
        +update(t, skeleton, dt)
        +log_channels() dict
        +__call__(t, skeleton) NDArray
    }
    class TrackingController {
        +reference: JointReference
        +kp, kd
    }
    Controller <|-- TrackingController
    Controller <|-- EndpointController
    Controller <|-- JointSpaceMPC
    TrackingController <|-- JointPD
    TrackingController <|-- InverseDynamicsFeedforwardPD
    TrackingController <|-- ComputedTorque
    EndpointController <|-- VirtualSpringDamper
    EndpointController <|-- TimeVaryingStiffness
    EndpointController <|-- OnlineReferenceShaping
    OnlineReferenceShaping <|-- PositionDependentShaping
    OnlineReferenceShaping <|-- AdaptiveReferenceShaping
    EndpointController ..> Skeleton : reads endpoint
    StateLog ..> Skeleton : records / rebuilds

    %% ── Joint reference (structural Protocol) ────
    class JointReference {
        <<protocol>>
        +sample(t) tuple
    }
    JointReference <|.. SampledJointReference : realizes
    TrackingController ..> JointReference : tracks
    JointSpaceMPC ..> JointReference : tracks

    %% ── Scenario glue (config → runnable) ────────
    class Scenario {
        <<dataclass>>
    }
    class Task {
        <<dataclass>>
        +target duration dt schedule
    }
    Scenario *-- Skeleton
    Scenario *-- Task
    Scenario *-- Controller
    Scenario ..> Trajectory : plans

    %% ── PyQt6 GUI ───────────────────────────────
    class QWidget {
        <<PyQt6>>
    }
    class QMainWindow {
        <<PyQt6>>
    }
    QWidget <|-- SkelarmCanvas
    SkelarmCanvas <|-- SimulatorCanvas
    QMainWindow <|-- SkelarmViewer
    QMainWindow <|-- SkelarmSimulator
    SkelarmCanvas *-- Skeleton
```

*Click the diagram to enlarge it; click anywhere, the × button, or press Esc to
close.*

## Robot model

[`Skeleton`](skeleton.md) owns the ordered list of `Link` objects (`links[0]` is
the fixed base; `links[1:]` are the actuated joints), and each `Link` holds an
immutable `LinkProp` dataclass with its geometry, mass properties, and joint
limits. The kinematics and dynamics functions operate on a `Skeleton`, and the
controllers below read endpoint state from it each step.

## Controllers

Every controller derives from the abstract [`Controller`](control.md), which is
callable as `f(t, skeleton) -> tau` and exposes the `reset` / `control` /
`update` / `log_channels` hooks used by the fixed-step `simulate_controlled`
loop. Two families sit beneath it:

- [`TrackingController`](control.md) tracks a joint reference and logs the
  reference and error; `JointPD`, `InverseDynamicsFeedforwardPD`, and
  `ComputedTorque` are its concrete trackers.
- [`EndpointController`](reaching.md) implements the shared task-space
  spring-damper law; `VirtualSpringDamper`, `TimeVaryingStiffness`, and
  `OnlineReferenceShaping` extend it, and `PositionDependentShaping` /
  `AdaptiveReferenceShaping` further specialize the online shaper.

[`JointSpaceMPC`](mpc.md) derives from `Controller` directly rather than from
`TrackingController`, because it consumes the joint reference through the
optimizer rather than through a PD law.

The trackers and MPC read a joint reference through the `JointReference`
[`Protocol`](control.md). `SampledJointReference` satisfies it structurally (it
is not an explicit subclass), so any object with a matching `sample(t)` method
can serve as a reference.

## Scenario glue

[`Scenario`](scenario.md) is the runnable bundle assembled from one TOML file: a
`Skeleton`, a `Task` (target, duration, control step, schedule), and a
ready-to-run `Controller`. The loader plans a `Trajectory` to build the joint
reference for the planned trackers. See the
[Control Configuration](../guides/control_configuration.md) guide for the config
schema.

## PyQt6 GUI

The interactive tools subclass PyQt6 base classes. [`SkelarmCanvas`](canvas.md)
(a `QWidget`) renders a `Skeleton` and is extended by
[`SimulatorCanvas`](simulator.md); `SkelarmViewer` and `SkelarmSimulator` are the
`QMainWindow` shells that host them.

!!! note "Value types not shown"
    A few standalone value types are omitted from the diagram to keep it
    readable: `IKResult` (returned by `compute_inverse_kinematics`),
    [`Trajectory`](trajectory.md), and [`StateLog`](recording.md) (the recorded
    run produced by `simulate_controlled`).
