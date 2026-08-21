# Architecture Target

This repository is being rebuilt as a modular robotics data-and-control platform.
Its architectural goal is to support SO-ARM, Seeed RS, Seeed DM, bimanual robots,
future robot hardware, local policies, and cloud policies without coupling those
concerns to one another.

## Layering

```text
Dora workflows
  collection / teleoperation / calibration orchestration / replay
  local inference / cloud inference / observability
        |
Robot adapter
  stable observation / action / lifecycle / capability boundary
        |
Device drivers
  RS MIT / DM CAN or MIT / SO-ARM / cameras / grippers / future SDKs

LeRobot is the system of record for datasets, training, evaluation, and policies.
It integrates with workflows through explicit bridges; it does not own device
protocols or safety-critical hardware control.
```

## Responsibilities

### Dora workflows

- Dora is the local runtime and dataflow orchestrator.
- Workflows compose nodes for collection, teleoperation, calibration, replay,
  local inference, cloud inference, synchronization, and monitoring.
- Graphs contain composition and configuration only. They must not contain device
  protocols, robot-specific kinematics, hidden policy logic, or hard-coded ports
  and filesystem paths.

### Robot adapters

- A robot adapter is the stable boundary between workflows and concrete hardware.
- It exposes a robot-level observation, action, lifecycle, and capability model.
- Lifecycle includes connecting, calibration, enable/disable, fault reporting,
  safe stop, and disconnecting.
- An adapter may compose motors, grippers, cameras, and other sensors into one
  robot semantic model.
- Bimanual robots are compositions of left and right adapters plus shared
  synchronization, calibration, and safety behavior; they are not a one-off
  hardware special case.

### Device drivers

- Drivers own only device-specific concerns: transport, protocol, register or
  frame encoding, low-level reads/writes, and device fault codes.
- Drivers do not depend on LeRobot datasets, policies, task semantics, or Dora
  graph topology.
- Adding RS, DM, SO-ARM, or a future robot must be possible by adding or
  replacing drivers and adapters without changing dataset or policy semantics.

### LeRobot bridges

- LeRobot is the canonical data and model layer.
- Recorder bridges convert workflow messages into the current LeRobot dataset
  format. Policy bridges convert observations to policy inputs and policy actions
  back to the common robot action model.
- Training, evaluation, replay, and policy deployment must use LeRobot-compatible
  datasets and policy contracts; legacy repository-specific recording formats are
  not the target architecture.

## Cross-cutting constraints

- Observation and action schemas are explicit, versioned, unit-aware, and
  timestamped. Coordinate frames, joint ordering, image metadata, and gripper
  conventions must be unambiguous.
- Adapters expose capabilities instead of forcing all hardware into a lowest-common
  denominator: control modes, joint count, gripper, cameras, force sensing, and
  supported rates may differ.
- Safety is local and mandatory. Every action, including one from a cloud policy,
  passes local enable-state, limit, rate, and fault checks before reaching a
  driver.
- Cloud inference is optional. It has deadlines, model-version identity, and a
  deterministic local fallback; connectivity must never be required for safe
  stopping or low-level control.
- Calibration is orchestrated and visualized by workflows, while its authoritative
  device state is owned and persisted by adapters/drivers.
- New models extend the policy bridge; new hardware extends the driver/adapter
  boundary. Neither kind of extension should require redesigning the other.

## Non-goals

- Do not make Dora replace LeRobot's dataset, training, evaluation, or policy
  abstractions.
- Do not expose device protocols or serial/CAN details to workflows or policies.
- Do not use remote services for safety-critical control loops.
