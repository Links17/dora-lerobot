# B601-RS HAL v1 Acceptance Checklist

1. Copy `configs/runtime/b601_rs.hal.example.yaml` outside the repository and
   set the stable HAL CAN resource id discovered with `cargo run --quiet -p
   b601-rs-hal-node -- --discover`.
2. Export `DORA_LEROBOT_B601_RS_HAL_CONFIG` and start a workflow. Connection
   sends Disable to all seven motors; it never performs mechanical zero or
   Enable implicitly.
3. After a human-approved mechanical calibration, issue `calibrate`, then
   `enable` through Dora's lifecycle control channel.
4. Verify each configured zero offset and direction at reduced speed. Confirm
   the gripper torque cap before applying grasp actions. Then test replay,
   local inference, cloud inference, and graph shutdown.

The adapter enforces the RS MIT frame layout, joint limits, per-step relative
target cap, monotonic timestamps, thermal stop at 80°C, and local safe-stop.
Cloud actions cannot bypass these checks.

Gravity compensation is a separate dynamics node. Before starting an RS graph
that includes it, set the deployment-owned URDF path (outside the workflow):

```bash
export DORA_LEROBOT_RS_GRAVITY_URDF=/secure/00-arm-rs_asm-v3.urdf
```

It produces arm torque feed-forward only; the gripper remains passive. Every
value is bounded once in the dynamics bridge and again by the local RS adapter
before MIT encoding. Do not enable it for a first motion test.
