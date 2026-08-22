use b601_dm_hal_node::{RuntimeConfig, is_discovery_request, parse_lifecycle};

const CONFIG: &str = r#"
owner: dora-b601-dm
resource:
  id: serial/usb/seeed-damiao-bridge
  minimum_identity_quality: Weak
  transport: Serial
serial:
  baud_rate: 921600
  read_timeout_ms: 20
robot:
  calibration_id: b601-dm-v1
  motor_ids: [1, 2, 3, 4, 5, 6, 7]
"#;

#[test]
fn dm_node_requires_stable_hal_resource_and_canonical_motor_order() {
    let config = RuntimeConfig::from_yaml(CONFIG).unwrap();
    assert_eq!(config.serial.baud_rate, 921600);
    assert!(
        RuntimeConfig::from_yaml(&CONFIG.replace("[1, 2, 3, 4, 5, 6, 7]", "[2, 1, 3, 4, 5, 6, 7]"))
            .is_err()
    );
}

#[test]
fn dm_node_lifecycle_and_discovery_are_explicit() {
    assert!(is_discovery_request(["b601-dm-hal-node", "--discover"]));
    assert_eq!(
        parse_lifecycle(&serde_json::json!("enable")).unwrap(),
        "enable"
    );
    assert!(parse_lifecycle(&serde_json::json!("zero")).is_err());
}
