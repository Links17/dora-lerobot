use dora_lerobot_hal::{
    RsMitCommand, RsMotorLimits, decode_rs_mit_feedback, encode_rs_mit_command,
    encode_rs_mit_lifecycle,
};

const O0: RsMotorLimits = RsMotorLimits::new(12.57, 33.0, 14.0);

#[test]
fn rs_mit_command_uses_documented_packed_bit_layout() {
    let frame = encode_rs_mit_command(
        3,
        RsMitCommand {
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            kp: 250.0,
            kd: 2.5,
            torque_nm: 0.0,
        },
        O0,
    )
    .unwrap();
    assert_eq!(frame.arbitration_id(), 3);
    assert_eq!(
        frame.data(),
        [0x7f, 0xff, 0x7f, 0xf7, 0xff, 0x7f, 0xf7, 0xff]
    );
}

#[test]
fn rs_mit_lifecycle_uses_vendor_enable_and_disable_frames() {
    assert_eq!(
        encode_rs_mit_lifecycle(1, true).unwrap().data(),
        [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc]
    );
    assert_eq!(
        encode_rs_mit_lifecycle(1, false).unwrap().data(),
        [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd]
    );
}

#[test]
fn rs_mit_feedback_decodes_vendor_layout_and_temperature() {
    let feedback =
        decode_rs_mit_feedback(0x101, [3, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 0x02, 0x58], O0).unwrap();
    assert_eq!(feedback.motor_id, 3);
    assert!((feedback.position_rad - 0.0).abs() < 0.001);
    assert!((feedback.velocity_rad_s - 0.0).abs() < 0.02);
    assert!((feedback.torque_nm - 0.0).abs() < 0.02);
    assert_eq!(feedback.status_code, 0);
    assert_eq!(feedback.mos_temperature_c, 60.0);
}
