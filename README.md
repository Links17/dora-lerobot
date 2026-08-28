# Dora-LeRobot

This repository is a Dora 1.0 runtime and hardware-adapter platform for
LeRobot-compatible robots. It is an alpha reference implementation, not a
certified safety system. Its target architecture is defined in
[`AGENTS.md`](AGENTS.md); the detailed design and implementation plan live in
[`docs/architecture`](docs/architecture) and [`docs/superpowers/plans`](docs/superpowers/plans).

`dora-lerobot` sits above the reusable [`robot-hal`](https://github.com/Links17/robot-hal)
transport/runtime. Robot protocols and local safety stay here; Robot HAL owns
resource identity, leases and transport I/O. Neither project owns product
accounts, datasets or cloud control planes.

## Architecture

See the explorable [platform architecture diagram](docs/architecture/robot-hal-platform.html)
and its source ([JSON](docs/architecture/robot-hal-platform.architecture.json)).

## Supported platform paths

- SO-ARM / Feetech is the first end-to-end software slice.
- Seeed RS / RobStride MIT and Seeed DM / Damiao MIT implement the same driver
  and adapter boundary.
- A bimanual adapter composes any two conforming single-arm adapters.
- Dora workflows cover collection, teleoperation, calibration orchestration,
  replay, local policy inference, and deadline-bound cloud policy inference.

## Toolchain

The repository pins Python 3.12 and Rust 1.98.0. Dora CLI, daemon, Python Node
API, and Rust Node API must all use `v1.0.0-rc.4`; do not install a PyPI Dora
0.5 wheel in its place.

```bash
uv python install 3.12
uv sync --python 3.12 --all-groups
rustup toolchain install 1.98.0
```

Install the matching Dora CLI release for the host platform from the Dora 1.0
RC release assets, then confirm `dora --version` reports the same revision.

## Safety

All hardware-directed actions pass through an enabled adapter, joint limits, and
driver fault handling. Cloud policy responses always return through that local
boundary. See the physical acceptance checklist before connecting a real robot:
[`docs/acceptance/so-arm-v1.md`](docs/acceptance/so-arm-v1.md).

Never connect hardware without completing the relevant acceptance checklist.
This software is provided as-is; operators remain responsible for emergency
stop, mechanical limits, calibration and supervision.

## Development

```bash
uv sync --python 3.12 --all-groups
uv run pytest -q
cargo test --workspace
```

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the supported checks and change
boundaries. Hardware qualification is separate from the hardware-free tests.
