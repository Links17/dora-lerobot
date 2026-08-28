# Dora-LeRobot

<p align="center">
  <strong>A local-first robotics runtime that connects Dora workflows to LeRobot-compatible robots.</strong><br>
  Hardware adapters and safety gates stay local; datasets, training and policies stay in LeRobot.
</p>

<p align="center">
  <a href="https://github.com/Links17/dora-lerobot/blob/main/LICENSE"><img src="https://img.shields.io/github/license/Links17/dora-lerobot?color=blue" alt="Apache-2.0 license"></a>
  <a href="https://github.com/Links17/dora-lerobot/issues"><img src="https://img.shields.io/github/issues/Links17/dora-lerobot" alt="open issues"></a>
  <a href="https://www.python.org/downloads/release/python-3120/"><img src="https://img.shields.io/badge/python-3.12-3776ab" alt="Python 3.12"></a>
  <a href="https://www.rust-lang.org/"><img src="https://img.shields.io/badge/rust-1.98.0-orange" alt="Rust 1.98.0"></a>
  <a href="https://github.com/Links17/robot-hal"><img src="https://img.shields.io/badge/transport-Robot%20HAL-5b5bd6" alt="Robot HAL"></a>
</p>

> [!WARNING]
> This is alpha robotics infrastructure, not a safety-certified controller.
> Never connect a real robot without completing the relevant acceptance
> checklist and having an operator and emergency stop available.

## Contents

- [Why this project?](#why-this-project)
- [Architecture](#architecture)
- [Supported paths](#supported-paths)
- [Quick start](#quick-start-hardware-free)
- [Run a workflow on hardware](#run-a-workflow-on-hardware)
- [Contracts and safety](#contracts-and-safety)
- [Repository map](#repository-map)
- [Development](#development)
- [Roadmap](#roadmap)
- [License](#license)

## Why this project?

Most robot-learning stacks mix workflow orchestration, device protocols,
datasets and policy execution in one process. Dora-LeRobot keeps those concerns
replaceable:

- **Dora** composes collection, teleoperation, replay and inference workflows.
- **Robot adapters** expose versioned observations, actions, capabilities and
  lifecycle state, with local limit/rate/fault checks.
- **Drivers** implement only robot/device protocols (Feetech, Damiao, RobStride).
- **[Robot HAL](https://github.com/Links17/robot-hal)** owns transport resources,
  discovery, leases and Serial/CAN/USB/GPIO/camera I/O.
- **LeRobot bridges** record LeRobotDataset episodes and translate policy inputs
  and outputs without owning hardware.

The result is a platform where adding a robot does not redesign datasets or
policies, and adding a policy does not bypass hardware safety.

## Architecture

The main path is:

```text
local/cloud policy
        │ actions
        ▼
Dora workflow ── contract ──▶ robot adapter ── safe action ──▶ protocol driver
        ▲                                                       │
        │ observations                                          ▼
LeRobot bridge ◀──────────────────────────────────────────── Robot HAL ──▶ hardware
```

Explore the [interactive architecture diagram](docs/architecture/robot-hal-platform.html)
or inspect its [versioned source](docs/architecture/robot-hal-platform.architecture.json).
For the detailed boundary and non-goals, see [AGENTS.md](AGENTS.md) and
[`docs/architecture/`](docs/architecture).

## Supported paths

| Path | Hardware / runtime | Status |
| --- | --- | --- |
| SO-ARM | Feetech serial, Robot HAL node, LeRobot recorder/policy bridge | End-to-end reference |
| B601 DM | Damiao MIT over USB-CAN | Adapter and Dora workflow |
| B601 RS | RobStride MIT over SocketCAN, optional gravity feed-forward | Adapter and Dora workflow |
| Bimanual | Composition of two conforming single-arm adapters | Contract and tests |
| Legacy robot examples | ALOHA, Reachy, SO-100 and related nodes | Compatibility/reference material |

The first real-hardware path to try is SO-ARM. B601 setup is documented in the
[DM](docs/acceptance/b601-dm-hal-v1.md) and [RS](docs/acceptance/b601-rs-hal-v1.md)
acceptance checklists.

## Quick start (hardware-free)

The test suite and contract checks run without a robot. Use the pinned toolchain:

```bash
uv python install 3.12
uv sync --python 3.12 --all-groups
rustup toolchain install 1.98.0

uv run pytest -q
cargo test --workspace
```

Install the matching Dora 1.0 RC CLI/daemon for your operating system and
confirm that `dora --version` reports `v1.0.0-rc.4`. Do not install the
unrelated PyPI `dora` 0.5 package.

## Run a workflow on hardware

1. Copy the appropriate file in [`configs/runtime/`](configs/runtime/) outside
   the repository and fill in the discovered Robot HAL resource, calibration,
   joint order and limits.
2. Complete the matching [SO-ARM acceptance checklist](docs/acceptance/so-arm-v1.md)
   or B601 checklist before enabling torque.
3. Start a workflow from [`workflows/`](workflows/), for example:

   ```bash
   export DORA_LEROBOT_SO_ARM_HAL_CONFIG=/secure/so-arm.hal.yaml
   dora build workflows/so_arm/teleoperate.yaml
   dora run workflows/so_arm/teleoperate.yaml
   ```

   Exact Dora CLI invocation can vary by the installed 1.0 RC build; use
   `dora --help` if your build uses a different launch command.

4. Lifecycle transitions are explicit: connect torque-disabled, calibrate,
   enable, then disable/safe-stop. Graph shutdown also invokes local safe-stop.

Available SO-ARM workflows are `teleoperate`, `record`, `replay`,
`inference_local` and `inference_cloud`. The B601 directories expose the same
workflow shape.

## Contracts and safety

- Observations and actions are versioned (`v1`), timestamped in UTC epoch
  nanoseconds, expressed in radians and validated for finite values.
- Joint ordering, coordinate transforms, gripper limits and calibration are
  configuration/adapter responsibilities, not graph conventions.
- Every action—including a cloud-policy action—passes local lifecycle, limit,
  rate, stale-timestamp and driver-fault checks before reaching hardware.
- Cloud connectivity is optional and cannot prevent local stop behavior.
- Hardware-free tests are not evidence of physical qualification. Follow the
  acceptance checklists and native runbooks for the exact device.

## Repository map

```text
src/dora_lerobot/       Python contracts, adapters, drivers and LeRobot bridges
crates/                 Rust protocol adapters and local safety boundaries
nodes/                  Dora 1.0 Rust hardware nodes
workflows/              Composition-only Dora graphs
configs/                Safe example configurations (no operator secrets)
docs/                   Architecture, contracts and physical acceptance runbooks
tests/                  Hardware-free Python/Rust integration and contract tests
```

## Development

```bash
# Python
uv run ruff check src tests
uv run pytest -q

# Rust (focused workspace packages; the legacy ALOHA example has a known
# formatting-only issue outside the core runtime)
cargo fmt -p dora-lerobot-hal -p so-arm-hal-node -p b601-dm-hal-node -p b601-rs-hal-node -- --check
cargo test --workspace
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for change boundaries. Changes that
touch hardware behavior should include a contract test and an updated
acceptance note. Please report security issues privately using
[SECURITY.md](SECURITY.md).

## Roadmap

- Replace local development path dependencies with released Robot HAL tags.
- Add reproducible simulation/virtual-hardware demos for each workflow.
- Publish API and schema compatibility guarantees for adapters and bridges.
- Qualify native hardware matrices independently from hardware-free CI.
- Add observability and control-plane integrations without moving safety out of
  the local adapter boundary.

## License

Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).

Robot HAL is maintained separately under its own Apache-2.0 licensed repository:
[github.com/Links17/robot-hal](https://github.com/Links17/robot-hal).
