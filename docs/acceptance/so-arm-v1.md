# SO-ARM v1 Acceptance Checklist

This checklist is hardware-gated. Automated tests verify contracts, bridges, and
the in-memory safety path; an operator completes this checklist on the robot.

Before connecting a real robot, copy `configs/runtime/so_arm.hardware.example.yaml`
outside the repository, set the two serial ports, and calibrate each device
explicitly. The calibration command is interactive and is the only command that
may invoke LeRobot calibration:

```bash
uv run dora-lerobot-so-arm-hardware --hardware-config /secure/so-arm.yaml --calibrate follower
uv run dora-lerobot-so-arm-hardware --hardware-config /secure/so-arm.yaml --calibrate leader
```

It writes the matching local calibration identity profiles. Normal graph launches
require `DORA_LEROBOT_SO_ARM_CONFIG=/secure/so-arm.yaml`; they connect with torque
disabled and only enable it from the explicit Dora `enable` input.

- [ ] Confirm configuration joint order matches the physical robot.
- [ ] Confirm calibrated limits and zero offsets before enabling torque.
- [ ] Confirm emergency stop and `safe_stop` disable torque.
- [ ] Record one teleoperated episode through `workflows/so_arm/record.yaml`.
- [ ] Open the result through LeRobot `LeRobotDataset` and inspect metadata.
- [ ] Replay the episode at reduced speed with an operator present.
- [ ] Run a local policy only after replay completes safely.
- [ ] Verify graph shutdown invokes `safe_stop` and leaves torque disabled.
