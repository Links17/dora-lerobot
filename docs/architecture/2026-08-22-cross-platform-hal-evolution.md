# 跨端机器人平台与 HAL 架构演进

## 决策

平台采用五层结构。Dora 是与业务无关的执行底座；Seeed HAL 是与机器人
语义无关的硬件资源底座。两者均不得承载产品业务。

```text
Product Applications
Electron / Web / Mobile
projects · devices · jobs · datasets · models · users · observability
        |
Rust Control Plane
API · WebSocket · workflow requests · configuration · audit · desktop lifecycle
        |
Dora Execution Plane
workflow scheduling · node communication · stream observability
        |
Robot Runtime
robot adapters · device-protocol drivers · local safety
        |
Seeed HAL
identity · discovery · sessions · leases · Serial/CAN/USB/GPIO/Camera
        |
OS drivers and physical hardware
```

LeRobot is a data-and-model ecosystem plugin in the robot runtime. It remains
the system of record for datasets, training, evaluation and policy contracts;
it is not the product control plane or hardware resource owner.

## Responsibility boundaries

### Product applications

Applications expose the user journey: device setup, calibration experience,
teleoperation task creation, collection, dataset review, training, model
deployment and observability. They express intent through stable APIs and do
not depend on Dora node IDs, graph ports, serial paths or CAN frames.

### Rust control plane

The control plane owns product API, authentication, authorization, persistent
job state, audit, configuration, desktop sidecar lifecycle and workflow launch
requests. It translates a request such as "collect on robot X with policy Y"
into an execution request. It never writes a motor command.

### Dora execution plane

Dora owns local or distributed dataflow execution: node scheduling, stream
delivery, synchronization and execution telemetry. A workflow expresses
composition only. It does not own users, project state, product permissions,
device protocols or hardware safety semantics.

### Robot runtime

Robot adapters expose versioned observation, action, capability and lifecycle
contracts. Device-protocol drivers implement Feetech, Damiao, RobStride, camera
and future protocols. Every action passes local enable-state, limits, rate and
fault checks before a driver writes to hardware.

This layer owns domain safety: torque disable, safe hold, damped stop, homing
and calibration semantics. These remain local even if the policy or control
plane is remote.

### Seeed HAL

Seeed HAL is the sole transport and hardware-resource abstraction: resource
identity, discovery, session/lease ownership, fencing generations, cancellation,
backpressure, diagnostics and cross-platform Serial, CAN, USB, GPIO and Camera
I/O. It has no understanding of robots, motors, calibration, teleoperation,
datasets or physical safety behavior.

Rust consumers use it in process. Python, Node, Electron and multi-process
consumers use its local broker. Renderer processes never access the broker or
hardware directly.

## Control and data paths

```text
Cloud/local policy -> Dora -> local adapter safety gate -> protocol driver -> HAL -> hardware
hardware -> HAL -> protocol driver -> adapter -> Dora -> LeRobot recorder/policy bridge
```

The first path is mandatory-local: cloud disconnection, a Dora failure or a
control-plane failure must never prevent safe stopping. The second path records
the synchronized robot contract into LeRobot-compatible datasets.

## Hardware ownership rule

One physical resource has one active transport owner. A vendor runtime that
opens serial/CAN directly cannot run concurrently with a HAL session for the
same resource.

During transition, vendor-direct and HAL-backed drivers are separate deployment
modes. The end state is that protocol drivers use HAL sessions, or vendor
runtimes receive a HAL-backed transport adapter. No product workflow may mix the
two modes for one device.

For high-rate RS MIT control, the preferred end state is an in-process Rust
protocol driver over a HAL CAN session. Python/LeRobot provides policy actions,
datasets and training; it does not sit on the high-frequency physical control
loop. DM, SO-ARM and camera drivers follow the same ownership rule while keeping
their protocol-specific behavior above HAL.

## Desktop packaging direction

The desktop product packages Electron, the Rust control-plane sidecar, Dora,
the Seeed HAL broker and only the selected runtime components. Python is an
optional worker component for LeRobot, vendor SDKs and Python policies; it no
longer hosts product APIs or desktop lifecycle.

Runtime components are independently versioned and may be installed on demand:

- base desktop: Electron, Rust control plane, Dora and HAL;
- hardware component: selected driver/protocol runtime;
- LeRobot component: dataset, policy and vendor Python dependencies;
- training component: PyTorch and accelerator-specific dependencies.

User hardware bindings, calibration data, datasets, models and credentials live
in user-data storage, not inside immutable application bundles.

## Migration sequence

1. Use Seeed HAL for device discovery, persistent identity and exclusive leases
   while retaining explicitly selected vendor-direct runtime mode.
2. Introduce the Rust control plane alongside the existing product application;
   make it own workflow requests, structured state, logs and desktop sidecar
   lifecycle.
3. Replace FastAPI-to-LeRobot-child orchestration with Dora workflow launch and
   observation. Preserve the existing application UI and task semantics.
4. Move each hardware family to a HAL-backed protocol driver: SO-ARM serial,
   DM transport, RS CAN/MIT and camera capture. Qualify each device on hardware
   before deprecating its vendor-direct mode.
5. Move RS high-rate MIT control into a Rust driver only after matching vendor
   safety, gravity compensation, thermal handling and fault behavior in real
   hardware qualification.
6. Make Python a versioned LeRobot worker component, then split base, hardware,
   inference and training runtime packages for desktop distribution.

## Non-goals

- Do not put robot protocols or joint semantics into Seeed HAL.
- Do not put business workflows, users or persistence schemas into Dora.
- Do not rely on broker lease expiry as a physical safe-stop mechanism.
- Do not perform a big-bang rewrite of the current application.
- Do not permit concurrent direct vendor access and HAL access to the same port,
  CAN channel or camera.
