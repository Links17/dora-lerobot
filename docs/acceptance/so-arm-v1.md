# SO-ARM v1 Acceptance Checklist

This checklist is hardware-gated. Automated tests verify contracts, bridges, and
the in-memory safety path; an operator completes this checklist on the robot.

There are two explicitly exclusive deployment modes. The default HAL mode uses
`configs/runtime/so_arm.hal.example.yaml`: copy it outside the repository, use
HAL discovery to obtain the follower's stable serial resource ID, and enter the
measured zero ticks/directions and limits. It does not use `/dev/tty*` in a
workflow or Python runtime. Set `DORA_LEROBOT_SO_ARM_HAL_CONFIG` to that file.

The legacy vendor-direct mode uses `configs/runtime/so_arm.hardware.example.yaml`
and is only a migration fallback. Never run it while HAL owns the follower
serial resource. Its calibration command remains interactive:

```bash
uv run dora-lerobot-so-arm-hardware --hardware-config /secure/so-arm.yaml --calibrate follower
uv run dora-lerobot-so-arm-hardware --hardware-config /secure/so-arm.yaml --calibrate leader
```

HAL workflow launches connect with torque disabled. Press `c` to accept the
specified persisted calibration identity, then press `e` to enable. `d`, graph
shutdown, or a node error runs local torque disable independently of cloud or
Python connectivity.

- [ ] Confirm configuration joint order matches the physical robot.
- [ ] Confirm calibrated limits and zero offsets before enabling torque.
- [ ] Confirm emergency stop and `safe_stop` disable torque.
- [ ] Record one teleoperated episode through `workflows/so_arm/record.yaml`.
- [ ] Open the result through LeRobot `LeRobotDataset` and inspect metadata.
- [ ] Replay the episode at reduced speed with an operator present.
- [ ] Run a local policy only after replay completes safely.
- [ ] Verify graph shutdown invokes `safe_stop` and leaves torque disabled.

## B601 DM / RS extension

Use the corresponding templates in `configs/runtime/`. DM needs the Damiao
USB-CAN bridge; RS needs an already configured SocketCAN channel. Both use an
explicit mechanical-zero procedure and normal operation starts torque-disabled:

```bash
uv run dora-lerobot-dm-hardware --hardware-config /secure/b601-dm.yaml --calibrate
uv run dora-lerobot-rs-hardware --hardware-config /secure/b601-rs.yaml --calibrate
```

For a Dora B601 workflow, set `DORA_LEROBOT_B601_CONFIG` to that secure file and
`DORA_LEROBOT_B601_KIND` to `dm` or `rs`. The RS configuration owns its vendor
MIT gains, target cap and gravity-compensation setting; workflows do not.
