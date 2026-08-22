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
}

impl<T: DamiaoTransport> DmAdapter<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            state: DmState::Disconnected,
            calibration_id: None,
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
    async fn disable_all(&mut self) -> Result<(), DamiaoError> {
        for id in B601_DM_MOTOR_IDS {
            self.transport
                .send_frame(encode_damiao_lifecycle(id, DamiaoStatus::Disable)?)
                .await?;
        }
        Ok(())
    }
}
