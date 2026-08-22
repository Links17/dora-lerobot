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

/// Encodes a B601 DM control command into its standard CAN payload.
pub fn encode_damiao_command(
    motor_id: u16,
    command: DamiaoCommand,
) -> Result<DamiaoCanFrame, DamiaoError> {
    if motor_id > 0x7ff {
        return Err(DamiaoError::Frame(
            "motor CAN ID exceeds 11-bit standard range",
        ));
    }
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
    Ok(DamiaoCanFrame::standard(motor_id, data))
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
