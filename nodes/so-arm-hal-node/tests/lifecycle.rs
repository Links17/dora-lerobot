use so_arm_hal_node::{LifecycleCommand, parse_lifecycle};

#[test]
fn only_explicit_safe_lifecycle_commands_are_accepted() {
    assert_eq!(
        parse_lifecycle(&serde_json::json!("calibrate")).unwrap(),
        LifecycleCommand::Calibrate
    );
    assert_eq!(
        parse_lifecycle(&serde_json::json!("enable")).unwrap(),
        LifecycleCommand::Enable
    );
    assert_eq!(
        parse_lifecycle(&serde_json::json!("disable")).unwrap(),
        LifecycleCommand::Disable
    );
    assert!(parse_lifecycle(&serde_json::json!("move")).is_err());
    assert!(parse_lifecycle(&serde_json::json!({"enable": true})).is_err());
}
