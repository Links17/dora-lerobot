use dora_lerobot_hal::{
    DamiaoCanFrame, HalSerialIo, JointCalibration, JointLimit, SoArmConfig, SoArmState,
    open_damiao_serial, open_so_arm,
};
use robot_hal_core::{IdentityQuality, OwnerId, ResourceId, ResourceSelector, TransportKind};
use robot_hal_runtime::HalRuntime;
use robot_hal_serial::SerialConfig;
use robot_hal_testkit::VirtualSerialAdapter;

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

#[tokio::test]
async fn damiao_transport_uses_the_hal_serial_lease() {
    // This catches a DM transport that opens a device path outside HAL, bypassing
    // the exclusive resource ownership contract.
    let runtime = HalRuntime::builder()
        .serial_adapter(VirtualSerialAdapter::loopback("damiao-virtual"))
        .build();
    let owner = OwnerId::parse("dm-node").unwrap();
    let selector = ResourceSelector::exact(
        ResourceId::parse("damiao-virtual").unwrap(),
        IdentityQuality::Strong,
        TransportKind::Serial,
    );
    let mut bus = open_damiao_serial(
        &runtime,
        owner.clone(),
        selector.clone(),
        SerialConfig::default(),
    )
    .await
    .unwrap();

    assert!(
        HalSerialIo::open(&runtime, owner, selector, SerialConfig::default())
            .await
            .is_err()
    );
    bus.send(DamiaoCanFrame::standard(1, [0; 8])).await.unwrap();

    bus.into_serial().close().await.unwrap();
}
