use dora_lerobot_hal::RsTorqueFeedforward;

#[test]
fn torque_feedforward_is_bounded_locally_before_mit_encoding() {
    let bounded = RsTorqueFeedforward {
        torque_nm: [100.0; 7],
    }
    .bounded([14.0; 7], 3.5)
    .unwrap();
    assert_eq!(bounded.torque_nm[..6], [14.0; 6]);
    assert_eq!(bounded.torque_nm[6], 3.5);
}

#[test]
fn torque_feedforward_rejects_non_finite_values() {
    assert!(
        RsTorqueFeedforward {
            torque_nm: [f32::NAN; 7]
        }
        .bounded([14.0; 7], 3.5)
        .is_err()
    );
}
