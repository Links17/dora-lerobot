# B601-RS HAL v1 Acceptance Checklist

1. Copy `configs/runtime/b601_rs.hal.example.yaml` outside the repository and
   set the stable HAL CAN resource id discovered with `cargo run --quiet -p
   b601-rs-hal-node -- --discover`.
2. Export `DORA_LEROBOT_B601_RS_HAL_CONFIG` and start a workflow. Connection
   sends Disable to all seven motors; it never performs mechanical zero or
   Enable implicitly.
3. After a human-approved mechanical calibration, issue `calibrate`, then
   `enable` through Dora's lifecycle control channel.
4. Verify MIT observations and direction at reduced speed, then test replay,
   local inference, cloud inference, and graph shutdown.

The adapter enforces the RS MIT frame layout, joint limits, per-step relative
target cap, monotonic timestamps, thermal stop at 80°C, and local safe-stop.
Cloud actions cannot bypass these checks.
