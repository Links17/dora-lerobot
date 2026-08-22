use async_trait::async_trait;
use dora_lerobot_hal::{
    RsAdapter, RsCanFrame, RsMitError, RsMitTransport, RsMotorLimits, RsState,
    encode_rs_mit_lifecycle,
};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

#[derive(Clone, Default)]
struct Fake {
    writes: Arc<Mutex<Vec<RsCanFrame>>>,
    reads: Arc<Mutex<VecDeque<RsCanFrame>>>,
}
#[async_trait]
impl RsMitTransport for Fake {
    async fn send_frame(&mut self, frame: RsCanFrame) -> Result<(), RsMitError> {
        self.writes.lock().unwrap().push(frame);
        Ok(())
    }
    async fn receive_frame(&mut self) -> Result<Option<RsCanFrame>, RsMitError> {
        Ok(self.reads.lock().unwrap().pop_front())
    }
}

const LIMITS: [RsMotorLimits; 7] = [RsMotorLimits::new(12.57, 33.0, 14.0); 7];

#[tokio::test]
async fn rs_adapter_requires_calibration_and_enable_before_action() {
    let transport = Fake::default();
    let mut adapter = RsAdapter::new(transport, LIMITS, 0.2);
    assert_eq!(adapter.state(), RsState::Disconnected);
    adapter.connect().await.unwrap();
    assert_eq!(adapter.state(), RsState::ConnectedUncalibrated);
    assert!(adapter.apply_action([0.0; 7], 1).await.is_err());
    adapter.accept_calibration("rs-v1").unwrap();
    assert!(adapter.apply_action([0.0; 7], 2).await.is_err());
}

#[tokio::test]
async fn rs_adapter_clamps_relative_targets_and_disables_all_on_stop() {
    let transport = Fake::default();
    let writes = transport.writes.clone();
    let mut adapter = RsAdapter::new(transport, LIMITS, 0.2);
    adapter.connect().await.unwrap();
    adapter.accept_calibration("rs-v1").unwrap();
    adapter.enable().await.unwrap();
    adapter.apply_action([1.0; 7], 1).await.unwrap();
    {
        let frames = writes.lock().unwrap();
        assert_eq!(
            frames[7].data(),
            encode_rs_mit_lifecycle(1, true).unwrap().data()
        );
    }
    adapter.safe_stop().await.unwrap();
    assert_eq!(adapter.state(), RsState::ConnectedDisabled);
    assert!(
        writes
            .lock()
            .unwrap()
            .iter()
            .any(|frame| frame.data() == encode_rs_mit_lifecycle(7, false).unwrap().data())
    );
}

#[tokio::test]
async fn rs_adapter_fails_closed_when_feedback_times_out() {
    let transport = Fake::default();
    let writes = transport.writes.clone();
    let mut adapter = RsAdapter::new(transport, LIMITS, 0.2);
    adapter.connect().await.unwrap();
    adapter.accept_calibration("rs-v1").unwrap();
    adapter.enable().await.unwrap();
    assert!(adapter.observe(2).await.is_err());
    assert_eq!(adapter.state(), RsState::ConnectedDisabled);
    assert!(
        writes
            .lock()
            .unwrap()
            .iter()
            .any(|frame| frame.data() == encode_rs_mit_lifecycle(1, false).unwrap().data())
    );
}
