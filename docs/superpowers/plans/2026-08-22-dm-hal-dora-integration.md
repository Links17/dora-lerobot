# DM HAL Dora Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a HAL-backed B601-DM path from Dora workflows through a safety adapter, while preserving LeRobot as the dataset and policy system of record.

**Architecture:** A Rust DM Dora node owns the HAL lease and the `DmAdapter`. It receives versioned actions and emits versioned observations/status. Existing Python recorder and policy nodes remain bridges only; workflows contain topology and configuration, never serial paths or motor protocol details.

**Tech Stack:** Rust 2024, Dora 1, Robot HAL, LeRobot Python bridges, Cargo tests, Dora graph build validation.

**Spec:** `AGENTS.md` — Architecture Target

## Global Constraints

- No graph contains a device path, motor protocol, automatic enable, or automatic mechanical zeroing.
- Every action, including cloud-policy output, passes local lifecycle, timestamp, limit, rate, and fault validation.
- B601-DM joints 1–6 use `POS_VEL`; joint 7 uses `FORCE_POS`.
- HAL controls resource discovery and exclusive leases; the adapter owns calibration and safety state.
- LeRobot remains the canonical dataset, training, evaluation, and policy layer.

---

### Task 1: Complete DM adapter safety contract

**Files:**
- Modify: `crates/dora-lerobot-hal/src/dm_adapter.rs`
- Modify: `crates/dora-lerobot-hal/tests/dm_adapter.rs`

**Produces:** `DmAdapter::observe(timestamp_ns)`, local limits, temperature/status validation, and fail-closed transport error handling.

- [ ] Write failing tests for fault feedback, thermal feedback, missing feedback, and out-of-order actions.
- [ ] Implement the smallest feedback/action validation needed for each test.
- [ ] Run `cargo test -p dora-lerobot-hal --quiet` and `cargo clippy -p dora-lerobot-hal --all-targets -- -D warnings`.
- [ ] Commit the passing adapter slice.

### Task 2: Add a HAL-backed DM Dora node

**Files:**
- Create: `nodes/b601-dm-hal-node/Cargo.toml`
- Create: `nodes/b601-dm-hal-node/src/lib.rs`
- Create: `nodes/b601-dm-hal-node/src/main.rs`
- Create: `nodes/b601-dm-hal-node/tests/lifecycle.rs`
- Modify: workspace `Cargo.toml`

**Produces:** A node with explicit `calibrate`, `enable`, `disable`, `tick`, and `action` inputs plus `observation`, `safe_action`, and `status` outputs.

- [ ] Write a failing lifecycle/configuration test.
- [ ] Implement YAML configuration with stable HAL resource identity and 921600 serial configuration.
- [ ] Implement the node using `open_damiao_serial` and `DmAdapter`, reporting action rejection without crashing.
- [ ] Build the node and run its tests.
- [ ] Commit the passing node slice.

### Task 3: Replace legacy DM workflow ownership

**Files:**
- Modify: `workflows/b601_dm/record.yaml`
- Modify: `workflows/b601_dm/replay.yaml`
- Modify: `workflows/b601_dm/inference_local.yaml`
- Modify: `workflows/b601_dm/inference_cloud.yaml`
- Create: `workflows/b601_dm/teleoperate.yaml`

**Produces:** Collection, teleoperation, replay, local inference, and cloud inference graphs all wired through `b601-dm-hal-node`.

- [ ] Write graph validation expectations for all six workflows.
- [ ] Replace `dora_lerobot.runtime.b601_node` with the Rust DM node and add lifecycle/status wiring.
- [ ] Run Dora graph build validation for every workflow.
- [ ] Commit the passing workflow slice.

### Task 4: Verify the LeRobot bridges and handoff

**Files:**
- Modify: relevant `dora_lerobot` recorder/policy/replay bridge tests only when their versioned contracts differ from the DM node.
- Create: `docs/hardware/b601-dm-acceptance.md`

**Produces:** Offline verification plus a manual hardware acceptance procedure that begins disabled and gives the operator explicit enable/stop steps.

- [ ] Add contract tests for observation/action schema compatibility where needed.
- [ ] Run Rust workspace tests, Python tests, clippy, formatting, and Dora graph builds.
- [ ] Write hardware steps: discovery, calibration acceptance, enable, motion envelope, safe stop, and fault recovery.
- [ ] Commit and push the verified code; hand off only then for operator-run hardware testing.
