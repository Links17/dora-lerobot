use b601_rs_hal_node::RuntimeConfig;

const CONFIG: &str = r#"
owner: dora-b601-rs
resource:
  id: can:virtual:b601-rs
  minimum_identity_quality: Weak
  transport: Can
can:
  receive_timeout_ms: 20
robot:
  calibration_id: b601-rs-v1
  motor_ids: [1, 2, 3, 4, 5, 6, 7]
  max_relative_target_deg: 10.0
  joint_zero_offsets_rad: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
  joint_directions: [1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 6.0]
rs_control:
  mit_kp: [50.0, 50.0, 50.0, 30.0, 50.0, 50.0, 12.0]
  mit_kd: [3.0, 5.0, 5.0, 3.0, 4.0, 4.0, 0.05]
  gripper_torque_limit_nm: 3.5
"#;

#[test]
fn rs_config_requires_calibration_transform_and_gripper_limit() {
    let config = RuntimeConfig::from_yaml(CONFIG).unwrap();
    assert_eq!(config.zero_offsets_rad, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]);
    assert_eq!(config.directions, [1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 6.0]);
    assert_eq!(config.gripper_torque_limit_nm, 3.5);
}

#[test]
fn rs_config_rejects_non_unit_arm_direction() {
    let invalid = CONFIG.replace(
        "joint_directions: [1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 6.0]",
        "joint_directions: [0.5, 1.0, -1.0, -1.0, -1.0, 1.0, 6.0]",
    );
    assert!(RuntimeConfig::from_yaml(&invalid).is_err());
}
