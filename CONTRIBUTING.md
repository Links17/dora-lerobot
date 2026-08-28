# Contributing

This project is experimental robotics infrastructure. Keep changes small,
reviewable and explicit about whether they affect hardware safety, contracts,
or LeRobot compatibility.

Before opening a pull request:

```bash
uv run pytest -q
uv run ruff check src tests
cargo fmt --all --check
cargo test --workspace
```

Do not put serial/CAN details in Dora workflows, bypass adapter safety gates,
commit operator configuration, credentials, datasets or model artifacts, or
claim physical qualification from simulated tests. New transports belong in
`robot-hal`; robot semantics belong in adapters and drivers here.

By submitting a contribution, you agree that it is provided under the Apache
License, Version 2.0, subject to any separate written agreement with the
copyright holder.
