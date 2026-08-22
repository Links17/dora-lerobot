use dora_lerobot_hal::{SoArmError, SoArmState};
use so_arm_hal_node::is_recoverable_action_error;

#[test]
fn disabled_or_stale_actions_are_rejected_without_terminating_the_node() {
    assert!(is_recoverable_action_error(&SoArmError::InvalidState {
        operation: "apply action",
        state: SoArmState::ConnectedDisabled,
    }));
    assert!(is_recoverable_action_error(&SoArmError::InvalidTimestamp {
        timestamp_ns: 0,
    }));
}
