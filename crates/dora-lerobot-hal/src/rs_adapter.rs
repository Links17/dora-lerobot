use crate::{
    RsCanFrame, RsMitCommand, RsMitError, RsMotorLimits, decode_rs_mit_feedback,
    encode_rs_mit_command, encode_rs_mit_lifecycle,
};
use async_trait::async_trait;

pub const B601_RS_MOTOR_IDS: [u16; 7] = [1, 2, 3, 4, 5, 6, 7];

#[async_trait]
pub trait RsMitTransport: Send {
    async fn send_frame(&mut self, frame: RsCanFrame) -> Result<(), RsMitError>;
    async fn receive_frame(&mut self) -> Result<Option<RsCanFrame>, RsMitError>;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RsState {
    Disconnected,
    ConnectedUncalibrated,
    ConnectedDisabled,
    Enabled,
    Faulted,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsObservation {
    pub timestamp_ns: u64,
    pub feedback: [crate::RsMitFeedback; 7],
}

pub struct RsAdapter<T> {
    transport: T,
    limits: [RsMotorLimits; 7],
    state: RsState,
    calibration_id: Option<String>,
    max_relative_target_rad: f32,
    last_target: [f32; 7],
    last_action_timestamp_ns: Option<u64>,
    kp: [f32; 7],
    kd: [f32; 7],
}

impl<T: RsMitTransport> RsAdapter<T> {
    pub fn new(transport: T, limits: [RsMotorLimits; 7], max_relative_target_rad: f32) -> Self {
        Self::new_with_gains(
            transport,
            limits,
            max_relative_target_rad,
            [10.0; 7],
            [0.5; 7],
        )
    }
    pub fn new_with_gains(
        transport: T,
        limits: [RsMotorLimits; 7],
        max_relative_target_rad: f32,
        kp: [f32; 7],
        kd: [f32; 7],
    ) -> Self {
        Self {
            transport,
            limits,
            state: RsState::Disconnected,
            calibration_id: None,
            max_relative_target_rad,
            last_target: [0.0; 7],
            last_action_timestamp_ns: None,
            kp,
            kd,
        }
    }
    pub const fn state(&self) -> RsState {
        self.state
    }
    pub async fn connect(&mut self) -> Result<(), RsMitError> {
        if self.state != RsState::Disconnected {
            return Err(RsMitError::Frame("adapter is already connected"));
        }
        self.disable_all().await?;
        self.state = RsState::ConnectedUncalibrated;
        Ok(())
    }
    pub fn accept_calibration(&mut self, id: impl Into<String>) -> Result<(), RsMitError> {
        if self.state != RsState::ConnectedUncalibrated {
            return Err(RsMitError::Frame(
                "calibration requires uncalibrated connected state",
            ));
        }
        let id = id.into();
        if id.trim().is_empty() {
            return Err(RsMitError::Frame("calibration id is empty"));
        }
        self.calibration_id = Some(id);
        self.state = RsState::ConnectedDisabled;
        Ok(())
    }
    pub async fn enable(&mut self) -> Result<(), RsMitError> {
        if self.state != RsState::ConnectedDisabled {
            return Err(RsMitError::Frame(
                "enable requires calibrated disabled state",
            ));
        }
        if self
            .kp
            .iter()
            .any(|v| !v.is_finite() || !(0.0..=500.0).contains(v))
            || self
                .kd
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=5.0).contains(v))
        {
            return Err(RsMitError::Frame("configured MIT gains are invalid"));
        }
        for id in B601_RS_MOTOR_IDS {
            self.transport
                .send_frame(encode_rs_mit_lifecycle(id, true)?)
                .await?;
        }
        self.state = RsState::Enabled;
        Ok(())
    }
    pub async fn safe_stop(&mut self) -> Result<(), RsMitError> {
        if self.state == RsState::Disconnected {
            return Ok(());
        }
        self.disable_all().await?;
        self.state = if self.calibration_id.is_some() {
            RsState::ConnectedDisabled
        } else {
            RsState::ConnectedUncalibrated
        };
        Ok(())
    }
    pub async fn apply_action(
        &mut self,
        positions_rad: [f32; 7],
        timestamp_ns: u64,
    ) -> Result<(), RsMitError> {
        if self.state != RsState::Enabled {
            return Err(RsMitError::Frame("action requires enabled state"));
        }
        if self
            .last_action_timestamp_ns
            .is_some_and(|last| timestamp_ns <= last)
        {
            return Err(RsMitError::Frame("action timestamp is not monotonic"));
        }
        if positions_rad.iter().any(|v| !v.is_finite()) {
            return Err(RsMitError::Frame("action contains non-finite position"));
        }
        for (index, raw) in positions_rad.into_iter().enumerate() {
            let limit = self.limits[index].position_rad;
            let target = raw.clamp(-limit, limit);
            let delta = target - self.last_target[index];
            let bounded =
                if self.max_relative_target_rad.is_finite() && self.max_relative_target_rad > 0.0 {
                    self.last_target[index]
                        + delta.clamp(-self.max_relative_target_rad, self.max_relative_target_rad)
                } else {
                    target
                };
            let command = RsMitCommand {
                position_rad: bounded,
                velocity_rad_s: 0.0,
                kp: self.kp[index],
                kd: self.kd[index],
                torque_nm: 0.0,
            };
            self.transport
                .send_frame(encode_rs_mit_command(
                    (index + 1) as u16,
                    command,
                    self.limits[index],
                )?)
                .await?;
            self.last_target[index] = bounded;
        }
        self.last_action_timestamp_ns = Some(timestamp_ns);
        Ok(())
    }
    pub async fn observe(&mut self, timestamp_ns: u64) -> Result<RsObservation, RsMitError> {
        if self.state != RsState::Enabled {
            return Err(RsMitError::Frame("observation requires enabled state"));
        }
        let mut feedback = [crate::RsMitFeedback {
            motor_id: 0,
            status_code: 0,
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            mos_temperature_c: 0.0,
        }; 7];
        for expected in B601_RS_MOTOR_IDS {
            let frame = match self.transport.receive_frame().await {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    let _ = self.safe_stop().await;
                    return Err(RsMitError::Frame("missing motor feedback"));
                }
                Err(error) => {
                    let _ = self.safe_stop().await;
                    return Err(error);
                }
            };
            let state = match decode_rs_mit_feedback(
                frame.arbitration_id(),
                frame.data(),
                self.limits[(expected - 1) as usize],
            ) {
                Ok(state) => state,
                Err(error) => {
                    let _ = self.safe_stop().await;
                    return Err(error);
                }
            };
            if u16::from(state.motor_id) != expected || state.mos_temperature_c >= 80.0 {
                let _ = self.safe_stop().await;
                return Err(RsMitError::Frame(
                    "motor feedback mismatch or thermal limit",
                ));
            }
            feedback[(expected - 1) as usize] = state;
        }
        Ok(RsObservation {
            timestamp_ns,
            feedback,
        })
    }
    async fn disable_all(&mut self) -> Result<(), RsMitError> {
        for id in B601_RS_MOTOR_IDS {
            self.transport
                .send_frame(encode_rs_mit_lifecycle(id, false)?)
                .await?;
        }
        Ok(())
    }
}
