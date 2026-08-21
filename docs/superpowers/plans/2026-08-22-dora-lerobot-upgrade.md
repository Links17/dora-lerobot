# Dora-LeRobot Platform Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the repository around Dora 1.0 and LeRobot 0.6.1, proving one safe SO-ARM collection-to-policy loop before adding RS, DM, or bimanual hardware.

**Architecture:** Dora graphs orchestrate local dataflow; robot adapters isolate workflow nodes from hardware drivers; LeRobot bridges are the only path to datasets and policies. Safety enforcement remains local to the adapter boundary and cloud policy is optional.

**Tech Stack:** Python 3.12, Rust 1.98.0, Dora v1.0.0-rc.4, LeRobot 0.6.1 dataset v3, Arrow, pytest, uv.

**Spec:** `docs/architecture/2026-08-22-target-architecture.md`

## Global Constraints

- Dora CLI/daemon, Python Node API, and Rust Node API use the exact `v1.0.0-rc.4` revision.
- LeRobot data is written and read only through dataset v3 APIs; `dataset.parquet` is legacy input only.
- Python baseline is exactly `3.12`; Rust baseline is exactly `1.98.0`.
- Hardware protocols remain below the adapter boundary; policies and graphs must never access serial or CAN directly.
- Every hardware-directed action passes a local safety check before a driver receives it.
- The first product slice is SO-ARM only. RS, DM, dual arm, and cloud inference are later milestones.

---

## Planned Repository Structure

```text
pyproject.toml                     Root uv workspace and tool constraints
rust-toolchain.toml                Rust 1.98.0 pin
src/dora_lerobot/contracts/        Versioned observation/action/lifecycle contracts
src/dora_lerobot/adapters/         Robot-level implementations and safety boundary
src/dora_lerobot/drivers/          Hardware-specific implementations
src/dora_lerobot/bridges/          Dora-to-LeRobot recorder and policy nodes
workflows/so_arm/                  Dora 1.0 collection, replay, and inference graphs
configs/robots/                    Versioned robot, camera, and calibration configuration
tests/contracts/                   Schema and safety tests
tests/adapters/                    Adapter behavior tests using fake drivers
tests/bridges/                     Dataset v3 and policy-bridge integration tests
docs/architecture/                 Architecture decisions and migration records
```

Legacy `node-hub/`, `robots/`, and `gym_dora/` remain read-only migration
references until the SO-ARM slice passes acceptance; they are not upgraded in place.

### Task 1: Establish the Reproducible Target Toolchain

**Files:**
- Create: `pyproject.toml`
- Create: `rust-toolchain.toml`
- Create: `.python-version`
- Create: `README.md`
- Test: `tests/test_environment.py`

**Interfaces:**
- Produces one `uv` environment with `dora-rs` sourced at Dora `v1.0.0-rc.4` and `lerobot==0.6.1`.
- Produces a Rust toolchain selected by Cargo without relying on a machine default.

- [ ] **Step 1: Write the environment contract test**

```python
def test_runtime_versions():
    import sys
    import lerobot
    assert sys.version_info[:2] == (3, 12)
    assert lerobot.__version__ == "0.6.1"
```

- [ ] **Step 2: Run the test before creating the environment**

Run: `uv run pytest tests/test_environment.py::test_runtime_versions -v`

Expected: FAIL because the root workspace and Python 3.12 environment do not yet exist.

- [ ] **Step 3: Add root toolchain configuration**

Pin Python to `3.12`, Rust to `1.98.0`, LeRobot to `0.6.1`, and Dora Python Node API to the exact Dora `v1.0.0-rc.4` source revision. Document the matching Dora CLI installation command in `README.md`; do not install a PyPI `0.5.x` wheel as a substitute.

- [ ] **Step 4: Build the environment and rerun the contract test**

Run: `uv sync --python 3.12 && uv run pytest tests/test_environment.py::test_runtime_versions -v`

Expected: PASS.

- [ ] **Step 5: Verify the Rust pin**

Run: `rustc --version && cargo --version`

Expected: both commands report `1.98.0`.

- [ ] **Step 6: Commit the toolchain baseline**

```bash
git add pyproject.toml rust-toolchain.toml .python-version README.md tests/test_environment.py uv.lock
git commit -m "build: establish Dora and LeRobot toolchain"
```

### Task 2: Define Versioned Robot Contracts

**Files:**
- Create: `src/dora_lerobot/contracts/models.py`
- Create: `src/dora_lerobot/contracts/protocols.py`
- Create: `src/dora_lerobot/contracts/__init__.py`
- Test: `tests/contracts/test_models.py`

**Interfaces:**
- Produces `Observation`, `Action`, `RobotLifecycleState`, and `RobotCapabilities` contracts.
- Produces `RobotAdapter` protocol with `connect`, `calibrate`, `read_observation`, `apply_action`, `safe_stop`, and `disconnect` operations.

- [ ] **Step 1: Write failing schema tests**

```python
def test_action_requires_matching_joint_order_and_units():
    with pytest.raises(ValueError, match="joint_names"):
        Action(joint_names=("joint_1",), positions_rad=(0.0, 1.0), timestamp_ns=1)

def test_observation_is_versioned_and_timestamped():
    observation = Observation(schema_version="v1", timestamp_ns=1, joints_rad={"joint_1": 0.0})
    assert observation.schema_version == "v1"
```

- [ ] **Step 2: Run the contract tests**

Run: `uv run pytest tests/contracts/test_models.py -v`

Expected: FAIL because contracts do not exist.

- [ ] **Step 3: Implement immutable, validated contract models and protocol**

Use explicit SI/radian naming, integer nanosecond timestamps, a schema version, and a capability declaration. Reject mismatched action vector lengths, non-finite actions, and unknown control modes at construction time.

- [ ] **Step 4: Run tests and static checks**

Run: `uv run pytest tests/contracts/test_models.py -v && uv run ruff check src tests`

Expected: PASS.

- [ ] **Step 5: Commit the contracts**

```bash
git add src/dora_lerobot/contracts tests/contracts
git commit -m "feat: define versioned robot contracts"
```

### Task 3: Create the SO-ARM Driver and Safe Adapter Slice

**Files:**
- Create: `src/dora_lerobot/drivers/feetech.py`
- Create: `src/dora_lerobot/adapters/so_arm.py`
- Create: `configs/robots/so_arm.example.yaml`
- Test: `tests/adapters/test_so_arm.py`

**Interfaces:**
- Consumes: `RobotAdapter`, `Action`, `Observation`, and `RobotCapabilities` from Task 2.
- Produces: `SoArmAdapter` that exposes normalized SO-ARM observations and accepts validated joint-position actions.

- [ ] **Step 1: Write fake-driver adapter tests**

```python
def test_adapter_clamps_action_before_driver_write(fake_driver, so_arm):
    so_arm.apply_action(Action(("shoulder",), (99.0,), 1))
    assert fake_driver.last_written_positions_rad == {"shoulder": so_arm.max_position("shoulder")}

def test_adapter_rejects_actions_while_disabled(so_arm):
    with pytest.raises(RuntimeError, match="disabled"):
        so_arm.apply_action(Action(("shoulder",), (0.0,), 1))
```

- [ ] **Step 2: Run the adapter tests**

Run: `uv run pytest tests/adapters/test_so_arm.py -v`

Expected: FAIL because the driver and adapter do not exist.

- [ ] **Step 3: Implement a Feetech-only driver and SO-ARM adapter**

Keep serial I/O in `feetech.py`. Keep calibration conversion, joint ordering, enable state, limits, and safe-stop behavior in `so_arm.py`. Load physical ports and calibrated limits only from `configs/robots/so_arm.yaml`, never from a workflow graph.

- [ ] **Step 4: Run unit tests without physical hardware**

Run: `uv run pytest tests/adapters/test_so_arm.py -v`

Expected: PASS using the fake driver.

- [ ] **Step 5: Run the manual hardware smoke check**

Run: `uv run python -m dora_lerobot.adapters.so_arm --config configs/robots/so_arm.yaml --smoke-check`

Expected: connects, reads one observation, performs no motion until explicitly enabled, then exits by calling `safe_stop`.

- [ ] **Step 6: Commit the SO-ARM slice**

```bash
git add src/dora_lerobot/drivers src/dora_lerobot/adapters configs/robots tests/adapters
git commit -m "feat: add safe SO-ARM adapter"
```

### Task 4: Prove Dora 1.0 Node Compatibility

**Files:**
- Create: `src/dora_lerobot/nodes/so_arm_node.py`
- Create: `workflows/so_arm/teleoperate.yaml`
- Test: `tests/nodes/test_so_arm_node.py`

**Interfaces:**
- Consumes: `SoArmAdapter` from Task 3.
- Produces: a Dora 1.0 node that publishes `Observation` messages and receives `Action` messages.

- [ ] **Step 1: Write a node contract test using a fake adapter**

```python
def test_node_converts_one_observation_to_the_v1_message_schema(fake_adapter):
    message = build_observation_message(fake_adapter.read_observation())
    assert message["schema_version"] == "v1"
    assert message["timestamp_ns"] > 0
```

- [ ] **Step 2: Run the node test**

Run: `uv run pytest tests/nodes/test_so_arm_node.py -v`

Expected: FAIL because the Dora node and message converter do not exist.

- [ ] **Step 3: Implement the Dora 1.0 node and graph**

Use only the Dora 1.0 RC Node API pinned in Task 1. The graph declares node wiring and config references; it must not contain physical port names or joint conversion factors.

- [ ] **Step 4: Run node tests and graph validation**

Run: `uv run pytest tests/nodes/test_so_arm_node.py -v && dora build workflows/so_arm/teleoperate.yaml`

Expected: PASS and a successfully built Dora graph.

- [ ] **Step 5: Commit Dora integration**

```bash
git add src/dora_lerobot/nodes workflows/so_arm tests/nodes
git commit -m "feat: run SO-ARM through Dora 1.0"
```

### Task 5: Replace Legacy Recording with a LeRobot Dataset v3 Bridge

**Files:**
- Create: `src/dora_lerobot/bridges/lerobot_recorder.py`
- Create: `workflows/so_arm/record.yaml`
- Test: `tests/bridges/test_lerobot_recorder.py`

**Interfaces:**
- Consumes: synchronized `Observation` and `Action` messages from Dora.
- Produces: a valid local LeRobot dataset v3 and episode metadata.

- [ ] **Step 1: Write an empty-dataset recording test**

```python
def test_recorder_writes_a_dataset_v3_episode(tmp_path, sample_observation, sample_action):
    recorder = LeRobotRecorder.create(root=tmp_path, robot_type="so_arm", fps=30)
    recorder.append(sample_observation, sample_action, task="pick up block")
    recorder.close_episode()
    assert LeRobotDataset(repo_id="local/so_arm", root=tmp_path).num_episodes == 1
```

- [ ] **Step 2: Run the recorder test**

Run: `uv run pytest tests/bridges/test_lerobot_recorder.py -v`

Expected: FAIL because the bridge does not exist.

- [ ] **Step 3: Implement the recorder bridge with LeRobot 0.6.1 APIs**

Create the dataset with explicit features for joint observations, actions, images, timestamps, task text, robot configuration version, and calibration version. Finalize videos and metadata through LeRobot APIs; do not generate `dataset.parquet` manually.

- [ ] **Step 4: Verify the dataset with the official reader**

Run: `uv run pytest tests/bridges/test_lerobot_recorder.py -v`

Expected: PASS and the test opens the recorded dataset through `LeRobotDataset`.

- [ ] **Step 5: Commit the recording bridge**

```bash
git add src/dora_lerobot/bridges workflows/so_arm tests/bridges
git commit -m "feat: record Dora workflows as LeRobot v3 datasets"
```

### Task 6: Add Replay and Local Policy Bridges

**Files:**
- Create: `src/dora_lerobot/bridges/lerobot_replay.py`
- Create: `src/dora_lerobot/bridges/lerobot_policy.py`
- Create: `workflows/so_arm/replay.yaml`
- Create: `workflows/so_arm/inference_local.yaml`
- Test: `tests/bridges/test_replay.py`
- Test: `tests/bridges/test_policy.py`

**Interfaces:**
- Consumes: LeRobot dataset v3 episodes and policies plus `SoArmAdapter` action validation.
- Produces: deterministic replay actions and safety-filtered policy actions.

- [ ] **Step 1: Write replay and safety-boundary tests**

```python
def test_replay_preserves_recorded_action_order(recorded_dataset):
    assert list(replay_actions(recorded_dataset)) == recorded_dataset_actions(recorded_dataset)

def test_policy_action_is_filtered_before_adapter_call(policy_bridge, fake_adapter):
    policy_bridge.step(sample_observation())
    assert fake_adapter.apply_action_called
    assert fake_adapter.last_action_is_within_limits
```

- [ ] **Step 2: Run the tests**

Run: `uv run pytest tests/bridges/test_replay.py tests/bridges/test_policy.py -v`

Expected: FAIL because replay and policy bridges do not exist.

- [ ] **Step 3: Implement LeRobot-native replay and local policy bridges**

Read data with `LeRobotDataset`, preserve recorded timestamps, and route every replay or policy action through `SoArmAdapter.apply_action`. The policy bridge reports the loaded model identifier and refuses a schema-version mismatch.

- [ ] **Step 4: Run integration tests and a hardware-gated smoke test**

Run: `uv run pytest tests/bridges/test_replay.py tests/bridges/test_policy.py -v && dora build workflows/so_arm/replay.yaml && dora build workflows/so_arm/inference_local.yaml`

Expected: PASS. On physical hardware, run replay only after explicit operator enable and confirm `safe_stop` executes on graph shutdown.

- [ ] **Step 5: Commit replay and inference**

```bash
git add src/dora_lerobot/bridges workflows/so_arm tests/bridges
git commit -m "feat: add LeRobot replay and local inference bridges"
```

### Task 7: Gate the SO-ARM End-to-End Acceptance Slice

**Files:**
- Create: `docs/acceptance/so-arm-v1.md`
- Modify: `README.md`
- Test: `tests/integration/test_so_arm_contract.py`

**Interfaces:**
- Consumes: Tasks 1–6.
- Produces: documented proof that SO-ARM records, reads, replays, and executes a local policy under the target architecture.

- [ ] **Step 1: Write the failing acceptance test**

```python
def test_recorded_episode_is_readable_and_replayable(tmp_path):
    dataset = record_fixture_episode(tmp_path)
    assert LeRobotDataset(repo_id="local/so_arm", root=dataset).num_episodes == 1
    assert next(replay_actions(dataset)).timestamp_ns > 0
```

- [ ] **Step 2: Run the acceptance test**

Run: `uv run pytest tests/integration/test_so_arm_contract.py -v`

Expected: FAIL until all preceding slice components are integrated.

- [ ] **Step 3: Integrate the SO-ARM collection, recorder, replay, and policy workflows**

Keep hardware execution behind an explicit operator enable flag. Add a documented manual checklist for calibration, emergency stop verification, a recorded episode, official dataset read, replay, and local policy execution.

- [ ] **Step 4: Run automated and manual acceptance**

Run: `uv run pytest tests/integration/test_so_arm_contract.py -v`

Expected: PASS. Complete every item in `docs/acceptance/so-arm-v1.md` on the physical robot before declaring the milestone complete.

- [ ] **Step 5: Commit the acceptance slice**

```bash
git add docs/acceptance README.md tests/integration workflows/so_arm src/dora_lerobot
git commit -m "feat: validate SO-ARM Dora-LeRobot closed loop"
```

### Task 8: Extend Only After the SO-ARM Gate Passes

**Files:**
- Create: `src/dora_lerobot/drivers/robstride.py`
- Create: `src/dora_lerobot/drivers/damiao.py`
- Create: `src/dora_lerobot/adapters/rs.py`
- Create: `src/dora_lerobot/adapters/dm.py`
- Create: `src/dora_lerobot/adapters/bimanual.py`
- Create: `docs/architecture/rs-dm-bimanual-extension.md`
- Test: `tests/adapters/test_rs.py`
- Test: `tests/adapters/test_dm.py`
- Test: `tests/adapters/test_bimanual.py`

**Interfaces:**
- Consumes: Task 2 contracts and the SO-ARM acceptance patterns.
- Produces: RS, DM, and bimanual implementations without modifying LeRobot recorder/policy schemas.

- [ ] **Step 1: Write adapter conformance tests before drivers**

```python
@pytest.mark.parametrize("adapter_factory", [make_rs_adapter, make_dm_adapter])
def test_adapter_conforms_to_robot_contract(adapter_factory):
    adapter = adapter_factory(fake_transport=True)
    adapter.connect()
    assert adapter.read_observation().schema_version == "v1"
    adapter.safe_stop()

def test_bimanual_preserves_left_right_namespaces(bimanual):
    assert set(bimanual.read_observation().joints_rad) == {"left.shoulder", "right.shoulder"}
```

- [ ] **Step 2: Run conformance tests**

Run: `uv run pytest tests/adapters/test_rs.py tests/adapters/test_dm.py tests/adapters/test_bimanual.py -v`

Expected: FAIL because the new adapters do not exist.

- [ ] **Step 3: Implement one hardware family at a time**

Implement RS first, complete its hardware smoke and conformance tests, then DM, then bimanual composition. Do not alter `LeRobotRecorder` or `LeRobotPolicyBridge` to accommodate device-specific protocols.

- [ ] **Step 4: Verify contract reuse**

Run: `uv run pytest tests/adapters tests/bridges -v`

Expected: all adapter conformance and existing bridge tests PASS unchanged.

- [ ] **Step 5: Commit each independently accepted hardware extension**

```bash
git add src/dora_lerobot/drivers src/dora_lerobot/adapters tests/adapters configs/robots docs/architecture
git commit -m "feat: add RS robot adapter"
```

Use separate commits with `DM` and `bimanual` in place of `RS` after their own acceptance gates.

## Plan Self-Review

- Architecture coverage: Tasks 2–4 establish the Dora/adapter/driver boundaries; Tasks 5–6 establish LeRobot bridges; Tasks 3, 6, and 7 enforce local safety; Task 8 exercises RS, DM, and bimanual extension without changing model/data layers.
- Scope coverage: the SO-ARM closed loop is the first independently testable deliverable. Cloud inference is deliberately deferred until the local policy and safety path are accepted.
- Dependency consistency: all tasks consume the contracts from Task 2; every policy and replay path terminates at `RobotAdapter.apply_action`.
