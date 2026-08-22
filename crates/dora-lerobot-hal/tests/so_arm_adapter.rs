use async_trait::async_trait;
use dora_lerobot_hal::{
    FeetechBus, FeetechError, JointCalibration, JointLimit, SO_ARM_JOINTS, SerialIo, SoArmAction,
    SoArmAdapter, SoArmConfig, SoArmError, SoArmState,
};
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

fn status(id: u8, parameters: &[u8]) -> Vec<u8> {
    let length = u8::try_from(parameters.len() + 2).unwrap();
    let mut packet = vec![0xff, 0xff, id, length, 0];
    packet.extend_from_slice(parameters);
    let checksum = !packet[2..]
        .iter()
        .fold(0u8, |sum, value| sum.wrapping_add(*value));
    packet.push(checksum);
    packet
}

fn configured_serial() -> FakeSerial {
    let serial = FakeSerial::default();
    let mut reads = serial.reads.lock().unwrap();
    // Connect disables torque. Enable reads the current position, then enables torque.
    for id in [3, 6, 2, 1, 4, 5] {
        reads.push_back(status(id, &[]));
    }
    for id in [1, 2, 3, 4, 5, 6] {
        reads.push_back(status(id, &2048u16.to_le_bytes()));
    }
    for _ in 0..2 {
        for id in [3, 6, 2, 1, 4, 5] {
            reads.push_back(status(id, &[]));
        }
    }
    drop(reads);
    serial
}

fn config() -> SoArmConfig {
    SoArmConfig::new(
        "so-arm-test",
        [
            JointLimit::new(-1.0, 1.0, 1.0),
            JointLimit::new(-1.0, 1.0, 1.0),
            JointLimit::new(-1.0, 1.0, 1.0),
            JointLimit::new(-1.0, 1.0, 1.0),
            JointLimit::new(-1.0, 1.0, 1.0),
            JointLimit::new(0.0, 1.0, 1.0),
        ],
        [JointCalibration::new(2048, 1); 6],
    )
    .unwrap()
}

fn bus(serial: FakeSerial) -> FeetechBus<FakeSerial> {
    FeetechBus::new(
        serial,
        SO_ARM_JOINTS
            .iter()
            .enumerate()
            .map(|(index, name)| (String::from(*name), u8::try_from(index + 1).unwrap()))
            .collect(),
    )
    .unwrap()
}

#[tokio::test]
async fn adapter_is_disabled_until_calibrated_and_explicitly_enabled() {
    let serial = configured_serial();
    let mut adapter = SoArmAdapter::new(bus(serial), config()).unwrap();

    adapter.connect().await.unwrap();
    assert_eq!(adapter.state(), SoArmState::ConnectedUncalibrated);
    assert!(matches!(
        adapter.enable(1_000_000_000).await,
        Err(SoArmError::InvalidState { .. })
    ));

    adapter
        .accept_calibration("so-arm-test-profile-v1")
        .unwrap();
    adapter.enable(1_000_000_000).await.unwrap();
    assert_eq!(adapter.state(), SoArmState::Enabled);
}

#[tokio::test]
async fn adapter_rate_limits_action_and_safe_stop_disables_torque() {
    let serial = configured_serial();
    let writes = serial.writes.clone();
    let mut adapter = SoArmAdapter::new(bus(serial), config()).unwrap();

    adapter.connect().await.unwrap();
    adapter
        .accept_calibration("so-arm-test-profile-v1")
        .unwrap();
    adapter.enable(1_000_000_000).await.unwrap();
    let applied = adapter
        .apply_action(SoArmAction::new(
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            1_100_000_000,
        ))
        .await
        .unwrap();

    assert_eq!(applied.positions_rad, [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]);
    adapter.safe_stop().await.unwrap();
    assert_eq!(adapter.state(), SoArmState::ConnectedDisabled);

    let all_writes = writes.lock().unwrap();
    assert!(all_writes.iter().any(|packet| packet[4] == 0x83));
    let torque_disables = all_writes
        .iter()
        .filter(|packet| packet[4] == 0x03 && packet[5] == 40 && packet[6] == 0)
        .count();
    assert_eq!(torque_disables, 12);
}
