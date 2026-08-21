# SO-ARM v1 Acceptance Checklist

This checklist is hardware-gated. Automated tests verify contracts, bridges, and
the in-memory safety path; an operator completes this checklist on the robot.

- [ ] Confirm configuration joint order matches the physical robot.
- [ ] Confirm calibrated limits and zero offsets before enabling torque.
- [ ] Confirm emergency stop and `safe_stop` disable torque.
- [ ] Record one teleoperated episode through `workflows/so_arm/record.yaml`.
- [ ] Open the result through LeRobot `LeRobotDataset` and inspect metadata.
- [ ] Replay the episode at reduced speed with an operator present.
- [ ] Run a local policy only after replay completes safely.
- [ ] Verify graph shutdown invokes `safe_stop` and leaves torque disabled.
