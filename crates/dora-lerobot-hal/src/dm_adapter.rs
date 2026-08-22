use crate::{
    DamiaoCanFrame, DamiaoControlMode, DamiaoError, DamiaoSerialBus, DamiaoSerialIo, DamiaoStatus,
    encode_damiao_lifecycle, encode_damiao_mode, is_damiao_mode_ack,
};
use async_trait::async_trait;

pub const B601_DM_MOTOR_IDS: [u16; 7] = [1, 2, 3, 4, 5, 6, 7];

#[async_trait]
pub trait DamiaoTransport: Send {
    async fn send_frame(&mut self, frame: DamiaoCanFrame) -> Result<(), DamiaoError>;
    async fn receive_frame(&mut self) -> Result<Option<DamiaoCanFrame>, DamiaoError>;
}

#[async_trait]
impl<T: DamiaoSerialIo> DamiaoTransport for DamiaoSerialBus<T> {
    async fn send_frame(&mut self, frame: DamiaoCanFrame) -> Result<(), DamiaoError> {
        self.send(frame).await
    }

    async fn receive_frame(&mut self) -> Result<Option<DamiaoCanFrame>, DamiaoError> {
        self.receive().await
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DmState {
    Disconnected,
    ConnectedUncalibrated,
    ConnectedDisabled,
    Enabled,
}

pub struct DmAdapter<T> {
    transport: T,
    state: DmState,
    calibration_id: Option<String>,
    last_action_timestamp_ns: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DamiaoAction {
    pub positions_rad: [f32; 7],
    pub timestamp_ns: u64,
    pub velocity_limit_rad_s: f32,
    pub torque_limit_ratio: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DamiaoObservation {
    pub timestamp_ns: u64,
    pub feedback: [crate::DamiaoFeedback; 7],
}

impl DamiaoAction {
    pub fn new(
        positions_rad: [f32; 7],
        timestamp_ns: u64,
        velocity_limit_rad_s: f32,
        torque_limit_ratio: f32,
    ) -> Self {
        Self {
            positions_rad,
            timestamp_ns,
            velocity_limit_rad_s,
            torque_limit_ratio,
        }
    }
}

impl<T: DamiaoTransport> DmAdapter<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            state: DmState::Disconnected,
            calibration_id: None,
            last_action_timestamp_ns: None,
        }
    }
    pub const fn state(&self) -> DmState {
        self.state
    }
    pub async fn connect(&mut self) -> Result<(), DamiaoError> {
        if self.state != DmState::Disconnected {
            return Err(DamiaoError::Frame("adapter is already connected"));
        }
        self.disable_all().await?;
        self.state = DmState::ConnectedUncalibrated;
        Ok(())
    }
    pub fn accept_calibration(&mut self, id: impl Into<String>) -> Result<(), DamiaoError> {
        if self.state != DmState::ConnectedUncalibrated {
            return Err(DamiaoError::Frame(
                "calibration requires uncalibrated connected state",
            ));
        }
        let id = id.into();
        if id.trim().is_empty() {
            return Err(DamiaoError::Frame("calibration id is empty"));
        }
        self.calibration_id = Some(id);
        self.state = DmState::ConnectedDisabled;
        Ok(())
    }
    pub async fn safe_stop(&mut self) -> Result<(), DamiaoError> {
        if self.state == DmState::Disconnected {
            return Ok(());
        }
        self.disable_all().await?;
        self.state = if self.calibration_id.is_some() {
            DmState::ConnectedDisabled
        } else {
            DmState::ConnectedUncalibrated
        };
        Ok(())
    }
    pub async fn enable(&mut self) -> Result<(), DamiaoError> {
        if self.state != DmState::ConnectedDisabled {
            return Err(DamiaoError::Frame(
                "enable requires calibrated disabled state",
            ));
        }
        for id in B601_DM_MOTOR_IDS {
            let mode = if id == 7 {
                DamiaoControlMode::ForcePosition
            } else {
                DamiaoControlMode::PositionVelocity
            };
            self.transport
                .send_frame(encode_damiao_mode(id, mode)?)
                .await?;
            let acknowledged = self.transport.receive_frame().await?;
            if !acknowledged.is_some_and(|frame| is_damiao_mode_ack(frame, id, mode)) {
                self.disable_all().await?;
                return Err(DamiaoError::Frame(
                    "control mode acknowledgement missing or mismatched",
                ));
            }
        }
        for id in B601_DM_MOTOR_IDS {
            self.transport
                .send_frame(encode_damiao_lifecycle(id, DamiaoStatus::Enable)?)
                .await?;
        }
        self.state = DmState::Enabled;
        Ok(())
    }
    pub async fn apply_action(&mut self, action: DamiaoAction) -> Result<(), DamiaoError> {
        if self.state != DmState::Enabled {
            return Err(DamiaoError::Frame("action requires enabled state"));
        }
        if self
            .last_action_timestamp_ns
            .is_some_and(|last| action.timestamp_ns <= last)
        {
            return Err(DamiaoError::Frame("action timestamp is not monotonic"));
        }
        if !action.velocity_limit_rad_s.is_finite()
            || action.velocity_limit_rad_s <= 0.0
            || !action.torque_limit_ratio.is_finite()
            || !(0.0..=1.0).contains(&action.torque_limit_ratio)
            || action
                .positions_rad
                .iter()
                .any(|position| !position.is_finite())
        {
            return Err(DamiaoError::Frame("action contains invalid values"));
        }
        for (index, position_rad) in action.positions_rad.into_iter().enumerate() {
            let id = (index + 1) as u16;
            let command = if id == 7 {
                crate::DamiaoCommand::ForcePosition {
                    position_rad,
                    velocity_limit_rad_s: action.velocity_limit_rad_s,
                    torque_limit_ratio: action.torque_limit_ratio,
                }
            } else {
                crate::DamiaoCommand::PositionVelocity {
                    position_rad,
                    velocity_limit_rad_s: action.velocity_limit_rad_s,
                }
            };
            self.transport
                .send_frame(crate::encode_damiao_command(id, command)?)
                .await?;
        }
        self.last_action_timestamp_ns = Some(action.timestamp_ns);
        Ok(())
    }
    pub async fn observe(&mut self, timestamp_ns: u64) -> Result<DamiaoObservation, DamiaoError> {
        if self.state != DmState::Enabled {
            return Err(DamiaoError::Frame("observation requires enabled state"));
        }
        let mut feedback = [crate::DamiaoFeedback {
            motor_id: 0,
            status_code: 0,
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            mos_temperature_c: 0,
            rotor_temperature_c: 0,
        }; 7];
        for expected_id in B601_DM_MOTOR_IDS {
            let frame = self
                .transport
                .receive_frame()
                .await?
                .ok_or(DamiaoError::Frame("missing motor feedback"))?;
            if frame.arbitration_id() != expected_id + 0x10 {
                self.disable_all().await?;
                self.state = DmState::ConnectedDisabled;
                return Err(DamiaoError::Frame("motor feedback id mismatch"));
            }
            let (position_limit, velocity_limit, torque_limit) = if expected_id <= 3 {
                (12.5, 10.0, 28.0)
            } else {
                (12.5, 30.0, 10.0)
            };
            let state = crate::decode_damiao_feedback(
                frame.data(),
                position_limit,
                velocity_limit,
                torque_limit,
            )?;
            if state.status_code != 1
                || state.mos_temperature_c >= 80
                || state.rotor_temperature_c >= 80
            {
                self.disable_all().await?;
                self.state = DmState::ConnectedDisabled;
                return Err(DamiaoError::Frame("motor fault or thermal limit"));
            }
            feedback[(expected_id - 1) as usize] = state;
        }
        Ok(DamiaoObservation {
            timestamp_ns,
            feedback,
        })
    }
    async fn disable_all(&mut self) -> Result<(), DamiaoError> {
        for id in B601_DM_MOTOR_IDS {
            self.transport
                .send_frame(encode_damiao_lifecycle(id, DamiaoStatus::Disable)?)
                .await?;
        }
        Ok(())
    }
}
