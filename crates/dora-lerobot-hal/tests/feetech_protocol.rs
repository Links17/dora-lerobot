use async_trait::async_trait;
use dora_lerobot_hal::{FeetechBus, FeetechError, SerialIo};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

#[derive(Clone, Default)]
struct FakeSerial {
    writes: Arc<Mutex<Vec<Vec<u8>>>>,
    reads: Arc<Mutex<VecDeque<Vec<u8>>>>,
}

#[async_trait]
impl SerialIo for FakeSerial {
    async fn write_all(&mut self, bytes: &[u8]) -> Result<(), FeetechError> {
        self.writes.lock().unwrap().push(bytes.to_vec());
        Ok(())
    }

    async fn read_some(&mut self, _max_bytes: usize) -> Result<Vec<u8>, FeetechError> {
        Ok(self.reads.lock().unwrap().pop_front().unwrap_or_default())
    }
}

#[tokio::test]
async fn torque_disable_uses_protocol_zero_write_packet() {
    let serial = FakeSerial::default();
    let writes = serial.writes.clone();
    serial
        .reads
        .lock()
        .unwrap()
        .push_back(vec![0xff, 0xff, 1, 2, 0, 252]);
    let mut bus = FeetechBus::new(serial, vec![("shoulder_pan".into(), 1)]).unwrap();

    bus.set_torque(false).await.unwrap();

    assert_eq!(
        writes.lock().unwrap().as_slice(),
        &[vec![0xff, 0xff, 1, 4, 3, 40, 0, 207]]
    );
}

#[tokio::test]
async fn goal_positions_use_one_broadcast_sync_write() {
    let serial = FakeSerial::default();
    let writes = serial.writes.clone();
    let mut bus = FeetechBus::new(
        serial,
        vec![("shoulder_pan".into(), 1), ("elbow_flex".into(), 3)],
    )
    .unwrap();

    bus.write_goal_ticks(&[("shoulder_pan", 100), ("elbow_flex", 1024)])
        .await
        .unwrap();

    assert_eq!(
        writes.lock().unwrap().as_slice(),
        &[vec![
            0xff, 0xff, 0xfe, 10, 131, 42, 2, 1, 100, 0, 3, 0, 4, 220
        ]]
    );
}

#[tokio::test]
async fn read_position_reassembles_fragmented_status_packet() {
    let serial = FakeSerial::default();
    serial
        .reads
        .lock()
        .unwrap()
        .extend([vec![0xff, 0xff, 1], vec![4, 0, 52, 18, 180]]);
    let mut bus = FeetechBus::new(serial, vec![("shoulder_pan".into(), 1)]).unwrap();

    assert_eq!(
        bus.read_position_ticks("shoulder_pan").await.unwrap(),
        0x1234
    );
}
