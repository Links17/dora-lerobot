//! RobStride MIT protocol for the B601-RS SocketCAN transport.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum RsMitError {
    #[error("RS MIT frame is invalid: {0}")]
    Frame(&'static str),
    #[error("RS MIT HAL transport failure: {0}")]
    Transport(String),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RsCanFrame {
    arbitration_id: u16,
    data: [u8; 8],
}

impl RsCanFrame {
    pub fn standard(arbitration_id: u16, data: [u8; 8]) -> Self {
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsMotorLimits {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
}
impl RsMotorLimits {
    pub const fn new(position_rad: f32, velocity_rad_s: f32, torque_nm: f32) -> Self {
        Self {
            position_rad,
            velocity_rad_s,
            torque_nm,
        }
    }
    fn valid(self) -> bool {
        self.position_rad.is_finite()
            && self.velocity_rad_s.is_finite()
            && self.torque_nm.is_finite()
            && self.position_rad > 0.0
            && self.velocity_rad_s > 0.0
            && self.torque_nm > 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsMitCommand {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque_nm: f32,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsMitFeedback {
    pub motor_id: u8,
    pub status_code: u8,
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
    pub mos_temperature_c: f32,
}

pub fn encode_rs_mit_lifecycle(motor_id: u16, enabled: bool) -> Result<RsCanFrame, RsMitError> {
    frame(
        motor_id,
        [
            0xff,
            0xff,
            0xff,
            0xff,
            0xff,
            0xff,
            0xff,
            if enabled { 0xfc } else { 0xfd },
        ],
    )
}

pub fn encode_rs_mit_command(
    motor_id: u16,
    command: RsMitCommand,
    limits: RsMotorLimits,
) -> Result<RsCanFrame, RsMitError> {
    if !limits.valid()
        || [
            command.position_rad,
            command.velocity_rad_s,
            command.kp,
            command.kd,
            command.torque_nm,
        ]
        .iter()
        .any(|v| !v.is_finite())
    {
        return Err(RsMitError::Frame("command or limits are not finite"));
    }
    if !(0.0..=500.0).contains(&command.kp) || !(0.0..=5.0).contains(&command.kd) {
        return Err(RsMitError::Frame("MIT gains exceed vendor range"));
    }
    let q = to_uint(
        command.position_rad,
        -limits.position_rad,
        limits.position_rad,
        16,
    );
    let dq = to_uint(
        command.velocity_rad_s,
        -limits.velocity_rad_s,
        limits.velocity_rad_s,
        12,
    );
    let kp = to_uint(command.kp, 0.0, 500.0, 12);
    let kd = to_uint(command.kd, 0.0, 5.0, 12);
    let tau = to_uint(command.torque_nm, -limits.torque_nm, limits.torque_nm, 12);
    frame(
        motor_id,
        [
            (q >> 8) as u8,
            q as u8,
            (dq >> 4) as u8,
            ((dq as u8 & 0x0f) << 4) | ((kp >> 8) as u8 & 0x0f),
            kp as u8,
            (kd >> 4) as u8,
            ((kd as u8 & 0x0f) << 4) | ((tau >> 8) as u8 & 0x0f),
            tau as u8,
        ],
    )
}

pub fn decode_rs_mit_feedback(
    arbitration_id: u16,
    data: [u8; 8],
    limits: RsMotorLimits,
) -> Result<RsMitFeedback, RsMitError> {
    if !limits.valid() {
        return Err(RsMitError::Frame("feedback limits are invalid"));
    }
    let q = u32::from(u16::from_be_bytes([data[1], data[2]]));
    let dq = (u32::from(data[3]) << 4) | u32::from(data[4] >> 4);
    let tau = (u32::from(data[4] & 0x0f) << 8) | u32::from(data[5]);
    let _ = arbitration_id;
    Ok(RsMitFeedback {
        motor_id: data[0],
        status_code: 0,
        position_rad: from_uint(q, -limits.position_rad, limits.position_rad, 16),
        velocity_rad_s: from_uint(dq, -limits.velocity_rad_s, limits.velocity_rad_s, 12),
        torque_nm: from_uint(tau, -limits.torque_nm, limits.torque_nm, 12),
        mos_temperature_c: f32::from(u16::from_be_bytes([data[6], data[7]])) / 10.0,
    })
}

fn frame(motor_id: u16, data: [u8; 8]) -> Result<RsCanFrame, RsMitError> {
    if motor_id > 0x7ff {
        Err(RsMitError::Frame(
            "motor CAN ID exceeds 11-bit standard range",
        ))
    } else {
        Ok(RsCanFrame::standard(motor_id, data))
    }
}
fn to_uint(value: f32, min: f32, max: f32, bits: u8) -> u32 {
    let bounded = value.clamp(min, max);
    (((bounded - min) / (max - min)) * ((1_u32 << bits) - 1) as f32) as u32
}
fn from_uint(value: u32, min: f32, max: f32, bits: u8) -> f32 {
    value as f32 * (max - min) / ((1_u32 << bits) - 1) as f32 + min
}
