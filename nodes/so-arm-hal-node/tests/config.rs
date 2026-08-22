use so_arm_hal_node::RuntimeConfig;

#[test]
fn configuration_uses_hal_identity_not_a_serial_path() {
    let config = RuntimeConfig::from_yaml(
        r#"
owner: dora-so-arm
resource:
  id: serial:usb:303a:1001:arm-a
  minimum_identity_quality: Strong
  transport: Serial
serial:
  baud_rate: 1000000
  read_timeout_ms: 20
robot:
  id: so-arm-a
  calibration_id: so-arm-a-v1
  joints:
    - {name: shoulder_pan, minimum_rad: -1.0, maximum_rad: 1.0, max_velocity_rad_s: 1.0, zero_tick: 2048, direction: 1}
    - {name: shoulder_lift, minimum_rad: -1.0, maximum_rad: 1.0, max_velocity_rad_s: 1.0, zero_tick: 2048, direction: 1}
    - {name: elbow_flex, minimum_rad: -1.0, maximum_rad: 1.0, max_velocity_rad_s: 1.0, zero_tick: 2048, direction: 1}
    - {name: wrist_flex, minimum_rad: -1.0, maximum_rad: 1.0, max_velocity_rad_s: 1.0, zero_tick: 2048, direction: 1}
    - {name: wrist_roll, minimum_rad: -1.0, maximum_rad: 1.0, max_velocity_rad_s: 1.0, zero_tick: 2048, direction: 1}
    - {name: gripper, minimum_rad: 0.0, maximum_rad: 1.0, max_velocity_rad_s: 1.0, zero_tick: 2048, direction: 1}
"#,
    )
    .unwrap();

    assert_eq!(config.owner.as_str(), "dora-so-arm");
    assert_eq!(config.resource.id().as_str(), "serial:usb:303a:1001:arm-a");
    assert_eq!(config.serial.baud_rate, 1_000_000);
    assert_eq!(config.robot.calibration_id, "so-arm-a-v1");
}
