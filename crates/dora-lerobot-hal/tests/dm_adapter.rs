use async_trait::async_trait;
use dora_lerobot_hal::{
    DamiaoAction, DamiaoCanFrame, DamiaoControlMode, DamiaoError, DamiaoTransport, DmAdapter,
    DmState, encode_damiao_mode,
};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

#[derive(Clone, Default)]
struct FakeTransport {
    writes: Arc<Mutex<Vec<DamiaoCanFrame>>>,
    reads: Arc<Mutex<VecDeque<DamiaoCanFrame>>>,
}

#[async_trait]
impl DamiaoTransport for FakeTransport {
    async fn send_frame(&mut self, frame: DamiaoCanFrame) -> Result<(), DamiaoError> {
        self.writes.lock().unwrap().push(frame);
        Ok(())
    }

    async fn receive_frame(&mut self) -> Result<Option<DamiaoCanFrame>, DamiaoError> {
        Ok(self.reads.lock().unwrap().pop_front())
    }
}

#[tokio::test]
async fn dm_adapter_enables_modes_only_after_all_mode_acks() {
    let transport = FakeTransport::default();
    for id in 1..=6 {
        transport
            .reads
            .lock()
            .unwrap()
            .push_back(DamiaoCanFrame::standard(
                id + 0x10,
                encode_damiao_mode(id, DamiaoControlMode::PositionVelocity)
                    .unwrap()
                    .data(),
            ));
    }
    transport
        .reads
        .lock()
        .unwrap()
        .push_back(DamiaoCanFrame::standard(
            0x17,
            encode_damiao_mode(7, DamiaoControlMode::ForcePosition)
                .unwrap()
                .data(),
        ));
    let writes = transport.writes.clone();
    let mut adapter = DmAdapter::new(transport);
    adapter.connect().await.unwrap();
    adapter.accept_calibration("b601-dm-v1").unwrap();
    adapter.enable().await.unwrap();
    assert_eq!(adapter.state(), DmState::Enabled);
    let frames = writes.lock().unwrap();
    assert_eq!(frames.iter().filter(|f| f.data()[7] == 0xfc).count(), 7);
}

#[tokio::test]
async fn dm_adapter_applies_fixed_joint_order_only_when_enabled() {
    let transport = FakeTransport::default();
    for id in 1..=6 {
        transport
            .reads
            .lock()
            .unwrap()
            .push_back(DamiaoCanFrame::standard(
                id + 0x10,
                encode_damiao_mode(id, DamiaoControlMode::PositionVelocity)
                    .unwrap()
                    .data(),
            ));
    }
    transport
        .reads
        .lock()
        .unwrap()
        .push_back(DamiaoCanFrame::standard(
            0x17,
            encode_damiao_mode(7, DamiaoControlMode::ForcePosition)
                .unwrap()
                .data(),
        ));
    let writes = transport.writes.clone();
    let mut adapter = DmAdapter::new(transport);
    adapter.connect().await.unwrap();
    adapter.accept_calibration("b601-dm-v1").unwrap();
    adapter.enable().await.unwrap();
    adapter
        .apply_action(DamiaoAction::new([0.1; 7], 42, 1.0, 0.2))
        .await
        .unwrap();
    let frames = writes.lock().unwrap();
    let action_ids: Vec<_> = frames[frames.len() - 7..]
        .iter()
        .map(|frame| frame.arbitration_id())
        .collect();
    assert_eq!(action_ids, vec![1, 2, 3, 4, 5, 6, 7]);
}

#[tokio::test]
async fn dm_adapter_observes_all_feedback_and_fails_closed_on_fault() {
    let transport = FakeTransport::default();
    for id in 1..=6 {
        transport
            .reads
            .lock()
            .unwrap()
            .push_back(DamiaoCanFrame::standard(
                id + 0x10,
                encode_damiao_mode(id, DamiaoControlMode::PositionVelocity)
                    .unwrap()
                    .data(),
            ));
    }
    transport
        .reads
        .lock()
        .unwrap()
        .push_back(DamiaoCanFrame::standard(
            0x17,
            encode_damiao_mode(7, DamiaoControlMode::ForcePosition)
                .unwrap()
                .data(),
        ));
    for id in 1..=7 {
        transport
            .reads
            .lock()
            .unwrap()
            .push_back(DamiaoCanFrame::standard(
                id + 0x10,
                [0x10 | id as u8, 0x80, 0, 0x80, 0, 0, 40, 40],
            ));
    }
    let mut adapter = DmAdapter::new(transport);
    adapter.connect().await.unwrap();
    adapter.accept_calibration("b601-dm-v1").unwrap();
    adapter.enable().await.unwrap();
    let observation = adapter.observe(42).await.unwrap();
    assert_eq!(observation.timestamp_ns, 42);
    assert_eq!(observation.feedback[6].motor_id, 7);
    assert_eq!(adapter.state(), DmState::Enabled);

    let fault = FakeTransport::default();
    for id in 1..=6 {
        fault
            .reads
            .lock()
            .unwrap()
            .push_back(DamiaoCanFrame::standard(
                id + 0x10,
                encode_damiao_mode(id, DamiaoControlMode::PositionVelocity)
                    .unwrap()
                    .data(),
            ));
    }
    fault
        .reads
        .lock()
        .unwrap()
        .push_back(DamiaoCanFrame::standard(
            0x17,
            encode_damiao_mode(7, DamiaoControlMode::ForcePosition)
                .unwrap()
                .data(),
        ));
    fault
        .reads
        .lock()
        .unwrap()
        .push_back(DamiaoCanFrame::standard(
            0x11,
            [0x81, 0, 0, 0, 0, 0, 40, 40],
        ));
    let mut adapter = DmAdapter::new(fault);
    adapter.connect().await.unwrap();
    adapter.accept_calibration("b601-dm-v1").unwrap();
    adapter.enable().await.unwrap();
    assert!(adapter.observe(42).await.is_err());
    assert_eq!(adapter.state(), DmState::ConnectedDisabled);
}

#[tokio::test]
async fn dm_adapter_disables_every_motor_on_connect_and_safe_stop() {
    // This catches connecting or disconnecting with one B601 actuator left energized.
    let transport = FakeTransport::default();
    let writes = transport.writes.clone();
    let mut adapter = DmAdapter::new(transport);

    adapter.connect().await.unwrap();
    assert_eq!(adapter.state(), DmState::ConnectedUncalibrated);
    adapter.accept_calibration("b601-dm-v1").unwrap();
    adapter.safe_stop().await.unwrap();

    assert_eq!(adapter.state(), DmState::ConnectedDisabled);
    let ids: Vec<_> = writes
        .lock()
        .unwrap()
        .iter()
        .filter(|frame| frame.data()[7] == 0xfd)
        .map(|frame| frame.arbitration_id())
        .collect();
    assert_eq!(ids, vec![1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7]);
}
