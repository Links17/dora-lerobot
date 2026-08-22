# B601-DM HAL v1 Acceptance Checklist

This procedure is the hardware gate after automated verification. It never
enables a motor implicitly. Keep the emergency stop accessible and operate
inside a clear motion envelope.

1. Copy `configs/runtime/b601_dm.hal.example.yaml` outside the repository.
2. With the arm mechanically safe and disabled, discover the bridge identity:

   ```bash
   cargo run --quiet -p b601-dm-hal-node -- --discover
   ```

3. Put the discovered stable HAL resource ID — never a device path — in the
   private configuration and export it:

   ```bash
   export DORA_LEROBOT_B601_DM_HAL_CONFIG=/secure/b601-dm.hal.yaml
   ```

4. Start `workflows/b601_dm/teleoperate.yaml`. Connecting sends Disable to all
   seven motors and does not issue mechanical zero or Enable.
5. After the operator has completed the mechanical calibration procedure and
   verified the persisted calibration identity, issue lifecycle commands in
   this strict order:

   ```bash
   dora param set b601-dm lifecycle '"calibrate"' --dataflow <dataflow>
   dora param set b601-dm lifecycle '"enable"' --dataflow <dataflow>
   ```

6. Confirm a 20 Hz `observation` stream, each joint remains within its approved
   envelope, and a small leader action generates the expected direction.
7. Check `dora param set b601-dm lifecycle '"disable"' --dataflow <dataflow>`
   disables all seven motors. Also check stopping the graph while enabled.
8. Before recording, replay, local inference, or cloud inference, repeat the
   reduced-speed motion and disable checks for that workflow.

The adapter disables every motor if a feedback frame is missing or mismatched,
the Damiao status is not enabled, or MOS/rotor temperature reaches 80°C. Cloud
connectivity is not involved in this stop path.
