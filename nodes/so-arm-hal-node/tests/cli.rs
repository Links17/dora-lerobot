use so_arm_hal_node::is_discovery_request;

#[test]
fn discovery_mode_requires_an_explicit_flag() {
    assert!(is_discovery_request(["so-arm-hal-node", "--discover"]));
    assert!(!is_discovery_request(["so-arm-hal-node"]));
    assert!(!is_discovery_request(["so-arm-hal-node", "--unknown"]));
}
