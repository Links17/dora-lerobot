use async_trait::async_trait;
use dora_lerobot_hal::{DamiaoCanFrame, DamiaoError, DamiaoTransport, DmAdapter, DmState};
use std::sync::{Arc, Mutex};

#[derive(Clone, Default)]
struct FakeTransport(Arc<Mutex<Vec<DamiaoCanFrame>>>);

#[async_trait]
impl DamiaoTransport for FakeTransport {
    async fn send_frame(&mut self, frame: DamiaoCanFrame) -> Result<(), DamiaoError> {
        self.0.lock().unwrap().push(frame);
        Ok(())
    }
}

#[tokio::test]
async fn dm_adapter_disables_every_motor_on_connect_and_safe_stop() {
    // This catches connecting or disconnecting with one B601 actuator left energized.
    let transport = FakeTransport::default();
    let writes = transport.0.clone();
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
