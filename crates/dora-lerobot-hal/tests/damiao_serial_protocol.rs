use async_trait::async_trait;
use dora_lerobot_hal::{
    DamiaoCanFrame, DamiaoCommand, DamiaoSerialBus, DamiaoSerialIo, encode_damiao_command,
};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

#[derive(Clone, Default)]
struct FakeSerial {
    writes: Arc<Mutex<Vec<Vec<u8>>>>,
    reads: Arc<Mutex<VecDeque<Vec<u8>>>>,
}

#[async_trait]
impl DamiaoSerialIo for FakeSerial {
    async fn write_all(&mut self, bytes: &[u8]) -> Result<(), dora_lerobot_hal::DamiaoError> {
        self.writes.lock().unwrap().push(bytes.to_vec());
        Ok(())
    }

    async fn read_some(
        &mut self,
        _max_bytes: usize,
    ) -> Result<Vec<u8>, dora_lerobot_hal::DamiaoError> {
        Ok(self.reads.lock().unwrap().pop_front().unwrap_or_default())
    }
}

#[tokio::test]
async fn dm_serial_encapsulates_a_classic_can_command_in_the_documented_30_byte_frame() {
    // This catches an incorrect serial bridge header, CAN-ID byte order, or payload offset.
    let serial = FakeSerial::default();
    let writes = serial.writes.clone();
    let mut bus = DamiaoSerialBus::new(serial);

    bus.send(DamiaoCanFrame::standard(0x07, [1, 2, 3, 4, 5, 6, 7, 8]))
        .await
        .unwrap();

    assert_eq!(
        writes.lock().unwrap().as_slice(),
        &[vec![
            0x55, 0xaa, 0x1e, 0x03, 1, 0, 0, 0, 10, 0, 0, 0, 0, 7, 0, 0, 0, 0, 8, 0, 0, 1, 2, 3, 4,
            5, 6, 7, 8, 0,
        ]]
    );
}

#[tokio::test]
async fn dm_serial_reassembles_a_fragmented_16_byte_feedback_frame() {
    // This catches a parser that loses feedback across serial read boundaries.
    let serial = FakeSerial::default();
    serial.reads.lock().unwrap().extend([
        vec![0x00, 0xaa, 0x11, 0x08, 0x17, 0x00, 0x00],
        vec![0x00, 9, 8, 7, 6, 5, 4, 3, 2, 0x55],
    ]);
    let mut bus = DamiaoSerialBus::new(serial);

    let frame = bus.receive().await.unwrap().unwrap();

    assert_eq!(frame.arbitration_id(), 0x17);
    assert_eq!(frame.data(), [9, 8, 7, 6, 5, 4, 3, 2]);
}

#[tokio::test]
async fn dm_serial_discards_noise_and_invalid_envelopes_before_feedback() {
    // This catches accepting corrupted bridge traffic as a motor feedback frame.
    let serial = FakeSerial::default();
    serial.reads.lock().unwrap().push_back(vec![
        0x12, 0xaa, 0x10, 0x08, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x55, 0xaa, 0x11, 0x08, 0x11, 0,
        0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0x55,
    ]);
    let mut bus = DamiaoSerialBus::new(serial);

    let frame = bus.receive().await.unwrap().unwrap();

    assert_eq!(frame.arbitration_id(), 0x11);
    assert_eq!(frame.data(), [1, 2, 3, 4, 5, 6, 7, 8]);
}

#[test]
fn dm_arm_position_velocity_command_uses_little_endian_float_payloads() {
    // This catches accidentally encoding the B601 DM arm as an MIT command.
    let frame = encode_damiao_command(
        0x03,
        DamiaoCommand::PositionVelocity {
            position_rad: 1.5,
            velocity_limit_rad_s: 2.25,
        },
    )
    .unwrap();

    assert_eq!(frame.arbitration_id(), 3);
    assert_eq!(frame.data(), [0, 0, 0xc0, 0x3f, 0, 0, 0x10, 0x40]);
}

#[test]
fn dm_gripper_force_position_command_scales_and_caps_velocity_and_torque() {
    // This catches sending a raw float or an out-of-range gripper current value.
    let frame = encode_damiao_command(
        0x07,
        DamiaoCommand::ForcePosition {
            position_rad: -0.5,
            velocity_limit_rad_s: 200.0,
            torque_limit_ratio: 2.0,
        },
    )
    .unwrap();

    assert_eq!(frame.data(), [0, 0, 0, 0xbf, 0x10, 0x27, 0x10, 0x27]);
}
