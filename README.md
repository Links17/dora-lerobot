# Dora-LeRobot

This repository is a Dora 1.0 runtime and hardware-adapter platform for
LeRobot-compatible robots. Its target architecture is defined in
[`AGENTS.md`](AGENTS.md); the detailed design and implementation plan live in
[`docs/architecture`](docs/architecture) and [`docs/superpowers/plans`](docs/superpowers/plans).

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
