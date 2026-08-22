//! Safety boundary for optional RS torque feed-forward (for example gravity compensation).
//! Dynamics are deliberately outside the device driver; this module only validates and bounds
//! the torque vector before it can reach MIT frames.

use crate::RsMitError;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsTorqueFeedforward {
    pub torque_nm: [f32; 7],
}

impl RsTorqueFeedforward {
    pub const fn zero() -> Self {
        Self {
            torque_nm: [0.0; 7],
        }
    }

    pub fn bounded(
        self,
        motor_limits: [f32; 7],
        gripper_limit_nm: f32,
    ) -> Result<Self, RsMitError> {
        if motor_limits.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || !gripper_limit_nm.is_finite()
            || gripper_limit_nm < 0.0
            || self.torque_nm.iter().any(|v| !v.is_finite())
        {
            return Err(RsMitError::Frame(
                "torque feed-forward contains invalid values",
            ));
        }
        let mut bounded = self.torque_nm;
        for (index, value) in bounded.iter_mut().enumerate() {
            let limit = if index == 6 {
                gripper_limit_nm
            } else {
                motor_limits[index]
            };
            *value = value.clamp(-limit, limit);
        }
        Ok(Self { torque_nm: bounded })
    }
}
