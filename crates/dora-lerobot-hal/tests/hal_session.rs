use dora_lerobot_hal::{JointCalibration, JointLimit, SoArmConfig, SoArmState, open_so_arm};
use seeed_hal_core::{IdentityQuality, OwnerId, ResourceId, ResourceSelector, TransportKind};
use seeed_hal_runtime::HalRuntime;
use seeed_hal_serial::SerialConfig;
use seeed_hal_testkit::VirtualSerialAdapter;

fn config() -> SoArmConfig {
    SoArmConfig::new(
        "so-arm-test",
        [JointLimit::new(-1.0, 1.0, 1.0); 6],
        [JointCalibration::new(2048, 1); 6],
    )
    .unwrap()
}

fn selector() -> ResourceSelector {
    ResourceSelector::exact(
        ResourceId::parse("so-arm-virtual").unwrap(),
        IdentityQuality::Strong,
        TransportKind::Serial,
    )
}

#[tokio::test]
async fn hal_session_is_exclusive_and_released_after_safe_close() {
    let runtime = HalRuntime::builder()
        .serial_adapter(VirtualSerialAdapter::loopback("so-arm-virtual"))
        .build();
    let owner = OwnerId::parse("so-arm-node").unwrap();
    let first = open_so_arm(
        &runtime,
        owner.clone(),
        selector(),
        SerialConfig::default(),
        config(),
    )
    .await
    .unwrap();
    assert_eq!(first.state(), SoArmState::Disconnected);

    assert!(
        open_so_arm(
            &runtime,
            owner.clone(),
            selector(),
            SerialConfig::default(),
            config()
        )
        .await
        .is_err()
    );

    first.close().await.unwrap();
    open_so_arm(
        &runtime,
        owner,
        selector(),
        SerialConfig::default(),
        config(),
    )
    .await
    .unwrap()
    .close()
    .await
    .unwrap();
}
