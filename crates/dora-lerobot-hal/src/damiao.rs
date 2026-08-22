//! Damiao USB-to-CAN serial bridge framing.
//!
//! The bridge is transport-only. Damiao motor modes and safety policy belong in
//! a device driver above this module.

use async_trait::async_trait;
use std::collections::VecDeque;
use thiserror::Error;

const TX_FRAME_LEN: usize = 30;
const RX_FRAME_LEN: usize = 16;
const RX_HEADER: u8 = 0xaa;
const RX_TRAILER: u8 = 0x55;
const RX_CAN_FORWARDING: u8 = 0x11;

#[derive(Debug, Error)]
pub enum DamiaoError {
    #[error("serial transport failed: {0}")]
    Transport(String),
    #[error("Damiao serial bridge frame is invalid: {0}")]
    Frame(&'static str),
}

/// The byte-stream boundary supplied by HAL or a deterministic test transport.
#[async_trait]
pub trait DamiaoSerialIo: Send {
    async fn write_all(&mut self, bytes: &[u8]) -> Result<(), DamiaoError>;
    async fn read_some(&mut self, max_bytes: usize) -> Result<Vec<u8>, DamiaoError>;
}

/// A standard classic CAN data frame as carried by the Damiao serial bridge.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DamiaoCanFrame {
    arbitration_id: u16,
    data: [u8; 8],
}

/// The B601 DM commands used by the robot adapter.
///
/// Six arm joints use position/velocity mode; the gripper uses force/position
/// mode. MIT remains available for other Damiao products but is intentionally
/// not a B601 DM action path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DamiaoCommand {
    PositionVelocity {
        position_rad: f32,
        velocity_limit_rad_s: f32,
    },
    ForcePosition {
        position_rad: f32,
        velocity_limit_rad_s: f32,
        torque_limit_ratio: f32,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DamiaoStatus {
    Enable,
    Disable,
    SetZero,
    ClearFault,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DamiaoControlMode {
    Mit = 1,
    PositionVelocity = 2,
    Velocity = 3,
    ForcePosition = 4,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DamiaoFeedback {
    pub motor_id: u8,
    pub status_code: u8,
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
    pub mos_temperature_c: u8,
    pub rotor_temperature_c: u8,
}

pub fn encode_damiao_lifecycle(
    motor_id: u16,
    status: DamiaoStatus,
) -> Result<DamiaoCanFrame, DamiaoError> {
    let last = match status {
        DamiaoStatus::Enable => 0xfc,
        DamiaoStatus::Disable => 0xfd,
        DamiaoStatus::SetZero => 0xfe,
        DamiaoStatus::ClearFault => 0xfb,
    };
    command_frame(motor_id, [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, last])
}

pub fn encode_damiao_mode(
    motor_id: u16,
    mode: DamiaoControlMode,
) -> Result<DamiaoCanFrame, DamiaoError> {
    command_frame(
        motor_id,
        [
            motor_id as u8,
            (motor_id >> 8) as u8,
            0x55,
            10,
            mode as u8,
            0,
            0,
            0,
        ],
    )
}

/// Returns true only for the acknowledged CTRL_MODE (register 10) write for
/// the requested actuator and target mode.
pub fn is_damiao_mode_ack(frame: DamiaoCanFrame, motor_id: u16, mode: DamiaoControlMode) -> bool {
    let data = frame.data;
    frame.arbitration_id == motor_id.saturating_add(0x10)
        && data[0] == motor_id as u8
        && data[1] == (motor_id >> 8) as u8
        && data[2] == 0x55
        && data[3] == 10
        && data[4] == mode as u8
        && data[5..] == [0, 0, 0]
}

pub fn decode_damiao_feedback(
    data: [u8; 8],
    position_limit_rad: f32,
    velocity_limit_rad_s: f32,
    torque_limit_nm: f32,
) -> Result<DamiaoFeedback, DamiaoError> {
    if !position_limit_rad.is_finite()
        || !velocity_limit_rad_s.is_finite()
        || !torque_limit_nm.is_finite()
        || position_limit_rad <= 0.0
        || velocity_limit_rad_s <= 0.0
        || torque_limit_nm <= 0.0
    {
        return Err(DamiaoError::Frame(
            "feedback limits must be finite and positive",
        ));
    }
    let position = u16::from_be_bytes([data[1], data[2]]) as u32;
    let velocity = (u32::from(data[3]) << 4) | (u32::from(data[4]) >> 4);
    let torque = ((u32::from(data[4]) & 0x0f) << 8) | u32::from(data[5]);
    Ok(DamiaoFeedback {
        motor_id: data[0] & 0x0f,
        status_code: data[0] >> 4,
        position_rad: uint_to_range(position, -position_limit_rad, position_limit_rad, 16),
        velocity_rad_s: uint_to_range(velocity, -velocity_limit_rad_s, velocity_limit_rad_s, 12),
        torque_nm: uint_to_range(torque, -torque_limit_nm, torque_limit_nm, 12),
        mos_temperature_c: data[6],
        rotor_temperature_c: data[7],
    })
}

fn command_frame(motor_id: u16, data: [u8; 8]) -> Result<DamiaoCanFrame, DamiaoError> {
    if motor_id > 0x7ff {
        return Err(DamiaoError::Frame(
            "motor CAN ID exceeds 11-bit standard range",
        ));
    }
    Ok(DamiaoCanFrame::standard(motor_id, data))
}

fn uint_to_range(value: u32, minimum: f32, maximum: f32, bits: u8) -> f32 {
    value as f32 * (maximum - minimum) / ((1u32 << bits) - 1) as f32 + minimum
}

/// Encodes a B601 DM control command into its standard CAN payload.
pub fn encode_damiao_command(
    motor_id: u16,
    command: DamiaoCommand,
) -> Result<DamiaoCanFrame, DamiaoError> {
    let data = match command {
        DamiaoCommand::PositionVelocity {
            position_rad,
            velocity_limit_rad_s,
        } => {
            if !position_rad.is_finite() || !velocity_limit_rad_s.is_finite() {
                return Err(DamiaoError::Frame(
                    "position/velocity command is not finite",
                ));
            }
            let mut data = [0u8; 8];
            data[..4].copy_from_slice(&position_rad.to_le_bytes());
            data[4..].copy_from_slice(&velocity_limit_rad_s.to_le_bytes());
            data
        }
        DamiaoCommand::ForcePosition {
            position_rad,
            velocity_limit_rad_s,
            torque_limit_ratio,
        } => {
            if !position_rad.is_finite()
                || !velocity_limit_rad_s.is_finite()
                || !torque_limit_ratio.is_finite()
            {
                return Err(DamiaoError::Frame("force/position command is not finite"));
            }
            let velocity = (velocity_limit_rad_s.clamp(0.0, 100.0) * 100.0) as u16;
            let torque = (torque_limit_ratio.clamp(0.0, 1.0) * 10_000.0) as u16;
            let mut data = [0u8; 8];
            data[..4].copy_from_slice(&position_rad.to_le_bytes());
            data[4..6].copy_from_slice(&velocity.min(10_000).to_le_bytes());
            data[6..].copy_from_slice(&torque.min(10_000).to_le_bytes());
            data
        }
    };
    command_frame(motor_id, data)
}

impl DamiaoCanFrame {
    pub fn standard(arbitration_id: u16, data: [u8; 8]) -> Self {
        assert!(
            arbitration_id <= 0x7ff,
            "classic standard CAN ID must fit in 11 bits"
        );
        Self {
            arbitration_id,
            data,
        }
    }

    pub const fn arbitration_id(self) -> u16 {
        self.arbitration_id
    }

    pub const fn data(self) -> [u8; 8] {
        self.data
    }
}

/// Encodes and decodes the documented 30-byte TX / 16-byte RX serial bridge envelopes.
pub struct DamiaoSerialBus<T> {
    serial: T,
    rx: VecDeque<u8>,
}

impl<T: DamiaoSerialIo> DamiaoSerialBus<T> {
    pub fn new(serial: T) -> Self {
        Self {
            serial,
            rx: VecDeque::with_capacity(RX_FRAME_LEN * 2),
        }
    }

    pub async fn send(&mut self, frame: DamiaoCanFrame) -> Result<(), DamiaoError> {
        self.serial.write_all(&encode_tx(frame)).await
    }

    /// Reads until one valid feedback frame is available, or returns `None` when
    /// the HAL serial session reports no bytes.
    pub async fn receive(&mut self) -> Result<Option<DamiaoCanFrame>, DamiaoError> {
        loop {
            if let Some(frame) = parse_rx(&mut self.rx) {
                return Ok(Some(frame));
            }
            let bytes = self.serial.read_some(256).await?;
            if bytes.is_empty() {
                return Ok(None);
            }
            self.rx.extend(bytes);
        }
    }

    pub fn into_serial(self) -> T {
        self.serial
    }
}

fn encode_tx(frame: DamiaoCanFrame) -> [u8; TX_FRAME_LEN] {
    let mut out = [0u8; TX_FRAME_LEN];
    out[0] = 0x55;
    out[1] = 0xaa;
    out[2] = 0x1e;
    out[3] = 0x03;
    out[4..8].copy_from_slice(&1u32.to_le_bytes());
    out[8..12].copy_from_slice(&10u32.to_le_bytes());
    out[12] = 0;
    out[13..17].copy_from_slice(&u32::from(frame.arbitration_id).to_le_bytes());
    out[17] = 0;
    out[18] = 8;
    out[21..29].copy_from_slice(&frame.data);
    out
}

fn parse_rx(rx: &mut VecDeque<u8>) -> Option<DamiaoCanFrame> {
    while rx.front().copied().is_some_and(|byte| byte != RX_HEADER) {
        rx.pop_front();
    }
    while rx.len() >= RX_FRAME_LEN {
        if rx.front().copied() != Some(RX_HEADER) {
            rx.pop_front();
            continue;
        }
        let mut raw = [0u8; RX_FRAME_LEN];
        for (index, byte) in rx.iter().take(RX_FRAME_LEN).enumerate() {
            raw[index] = *byte;
        }
        let dlc = raw[2] & 0x3f;
        let is_extended = raw[2] & 0x40 != 0;
        let is_remote = raw[2] & 0x80 != 0;
        let arbitration_id = u32::from_le_bytes([raw[3], raw[4], raw[5], raw[6]]);
        if raw[1] != RX_CAN_FORWARDING
            || raw[15] != RX_TRAILER
            || dlc != 8
            || is_extended
            || is_remote
            || arbitration_id > 0x7ff
        {
            // A corrupt prefix can contain a valid next-frame header. Discard
            // only the rejected header, then resynchronise byte-by-byte.
            rx.pop_front();
            while rx.front().copied().is_some_and(|byte| byte != RX_HEADER) {
                rx.pop_front();
            }
            continue;
        }
        let mut data = [0u8; 8];
        data.copy_from_slice(&raw[7..15]);
        rx.drain(..RX_FRAME_LEN);
        return Some(DamiaoCanFrame::standard(arbitration_id as u16, data));
    }
    None
}
