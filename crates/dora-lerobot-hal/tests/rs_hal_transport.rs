use dora_lerobot_hal::{HalRsCanTransport, RsCanFrame, RsMitTransport};
use seeed_hal_can::{CanFrame, CanId};
use seeed_hal_core::{IdentityQuality, OwnerId, ResourceId, ResourceSelector, TransportKind};
use seeed_hal_runtime::HalRuntime;
use seeed_hal_testkit::VirtualCanAdapter;
use std::time::Duration;

fn selector() -> ResourceSelector {
    ResourceSelector::exact(
        ResourceId::parse("can:virtual:b601-rs").unwrap(),
        IdentityQuality::Strong,
        TransportKind::Can,
    )
}

#[tokio::test]
async fn rs_transport_uses_hal_can_lease_for_send_and_receive() {
    let can = VirtualCanAdapter::loopback("can:virtual:b601-rs");
    let runtime = HalRuntime::builder().can_adapter(can.clone()).build();
    let owner = OwnerId::parse("rs-node").unwrap();
    let mut transport = HalRsCanTransport::open(
        &runtime,
        owner.clone(),
        selector(),
        Duration::from_millis(10),
    )
    .await
    .unwrap();

    transport
        .send_frame(RsCanFrame::standard(1, [0xff; 8]))
        .await
        .unwrap();
    assert_eq!(can.transmitted_frames().len(), 1);
    can.inject_received(
        CanFrame::classic_data(CanId::standard(0xfd).unwrap(), vec![3, 1, 2, 3, 4, 5, 6, 7])
            .unwrap(),
        None,
    )
    .unwrap();
    let received = transport.receive_frame().await.unwrap().unwrap();
    assert_eq!(received.arbitration_id(), 0xfd);
    assert_eq!(received.data(), [3, 1, 2, 3, 4, 5, 6, 7]);

    assert!(
        HalRsCanTransport::open(&runtime, owner, selector(), Duration::from_millis(10))
            .await
            .is_err()
    );
}
