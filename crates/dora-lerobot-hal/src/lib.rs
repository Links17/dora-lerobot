//! HAL-backed robot protocol drivers.
//!
//! This crate deliberately contains device protocol semantics above Seeed HAL.
//! HAL remains responsible only for transport resources and their leases.

mod damiao;

pub use damiao::{
    DamiaoCanFrame, DamiaoCommand, DamiaoControlMode, DamiaoError, DamiaoFeedback, DamiaoSerialBus,
    DamiaoSerialIo, DamiaoStatus, decode_damiao_feedback, encode_damiao_command,
    encode_damiao_lifecycle, encode_damiao_mode, is_damiao_mode_ack,
};

use async_trait::async_trait;
use bytes::Bytes;
use seeed_hal_core::{HalError, OwnerId, ResourceSelector};
use seeed_hal_runtime::{HalRuntime, SerialHandle};
use seeed_hal_serial::SerialConfig;
use std::collections::BTreeMap;
use std::f64::consts::TAU;
use thiserror::Error;

const HEADER: [u8; 2] = [0xff, 0xff];
const BROADCAST_ID: u8 = 0xfe;
const READ: u8 = 0x02;
const WRITE: u8 = 0x03;
const SYNC_WRITE: u8 = 0x83;
const TORQUE_ENABLE: u8 = 40;
const GOAL_POSITION: u8 = 42;
const PRESENT_POSITION: u8 = 56;

#[derive(Debug, Error)]
pub enum FeetechError {
    #[error("HAL transport failure: {0}")]
    Hal(Box<HalError>),
    #[error("duplicate motor id or name")]
    DuplicateMotor,
    #[error("unknown motor {0}")]
    UnknownMotor(String),
    #[error("invalid Feetech packet: {0}")]
    Packet(&'static str),
    #[error("servo {id} returned status error {error:#04x}")]
    ServoStatus { id: u8, error: u8 },
}

impl From<HalError> for FeetechError {
    fn from(error: HalError) -> Self {
        Self::Hal(Box::new(error))
    }
}

#[async_trait]
pub trait SerialIo: Send {
    async fn write_all(&mut self, bytes: &[u8]) -> Result<(), FeetechError>;
    async fn read_some(&mut self, max_bytes: usize) -> Result<Vec<u8>, FeetechError>;
}

/// A HAL-owned Serial session exposed to a device protocol driver.
pub struct HalSerialIo {
    handle: SerialHandle,
}

impl HalSerialIo {
    pub async fn open(
        runtime: &HalRuntime,
        owner: OwnerId,
        selector: ResourceSelector,
        config: SerialConfig,
    ) -> Result<Self, FeetechError> {
        Ok(Self {
            handle: runtime.open_serial(owner, selector, config).await?,
        })
    }

    pub async fn close(self) -> Result<(), FeetechError> {
        self.handle.close().await?;
        Ok(())
    }
}

#[async_trait]
impl SerialIo for HalSerialIo {
    async fn write_all(&mut self, bytes: &[u8]) -> Result<(), FeetechError> {
        self.handle.write(Bytes::copy_from_slice(bytes)).await?;
        self.handle.flush().await?;
        Ok(())
    }

    async fn read_some(&mut self, max_bytes: usize) -> Result<Vec<u8>, FeetechError> {
        Ok(self.handle.read(max_bytes).await?.to_vec())
    }
}

#[async_trait]
impl DamiaoSerialIo for HalSerialIo {
    async fn write_all(&mut self, bytes: &[u8]) -> Result<(), DamiaoError> {
        SerialIo::write_all(self, bytes)
            .await
            .map_err(|error| DamiaoError::Transport(error.to_string()))
    }

    async fn read_some(&mut self, max_bytes: usize) -> Result<Vec<u8>, DamiaoError> {
        SerialIo::read_some(self, max_bytes)
            .await
            .map_err(|error| DamiaoError::Transport(error.to_string()))
    }
}

/// Opens the Damiao USB-to-CAN serial bridge through a HAL-owned serial lease.
///
/// The returned transport has no motor semantics: it only translates serial
/// bridge envelopes into classic CAN frames for a Damiao driver.
pub async fn open_damiao_serial(
    runtime: &HalRuntime,
    owner: OwnerId,
    selector: ResourceSelector,
    config: SerialConfig,
) -> Result<DamiaoSerialBus<HalSerialIo>, DamiaoError> {
    let serial = HalSerialIo::open(runtime, owner, selector, config)
        .await
        .map_err(|error| DamiaoError::Transport(error.to_string()))?;
    Ok(DamiaoSerialBus::new(serial))
}

/// Feetech protocol-0 bus used by the SO-ARM STS3215 family.
pub struct FeetechBus<T> {
    serial: T,
    motors: BTreeMap<String, u8>,
}

impl<T: SerialIo> FeetechBus<T> {
    pub fn new(serial: T, motors: Vec<(String, u8)>) -> Result<Self, FeetechError> {
        let mut indexed = BTreeMap::new();
        for (name, id) in motors {
            if indexed.insert(name, id).is_some()
                || indexed.values().filter(|value| **value == id).count() > 1
            {
                return Err(FeetechError::DuplicateMotor);
            }
        }
        if indexed.is_empty() {
            return Err(FeetechError::Packet("a bus needs at least one motor"));
        }
        Ok(Self {
            serial,
            motors: indexed,
        })
    }

    pub async fn set_torque(&mut self, enabled: bool) -> Result<(), FeetechError> {
        let ids: Vec<_> = self.motors.values().copied().collect();
        for id in ids {
            self.write_register(id, TORQUE_ENABLE, &[u8::from(enabled)])
                .await?;
        }
        Ok(())
    }

    pub async fn read_position_ticks(&mut self, motor: &str) -> Result<u16, FeetechError> {
        let id = self.motor_id(motor)?;
        self.send_packet(id, READ, &[PRESENT_POSITION, 2]).await?;
        let (_, parameters) = self.read_status(id).await?;
        if parameters.len() != 2 {
            return Err(FeetechError::Packet(
                "position status has unexpected data length",
            ));
        }
        Ok(u16::from_le_bytes([parameters[0], parameters[1]]))
    }

    pub async fn write_goal_ticks(
        &mut self,
        positions: &[(&str, u16)],
    ) -> Result<(), FeetechError> {
        if positions.is_empty() {
            return Ok(());
        }
        let mut parameters = vec![GOAL_POSITION, 2];
        for (name, ticks) in positions {
            parameters.push(self.motor_id(name)?);
            parameters.extend_from_slice(&ticks.to_le_bytes());
        }
        self.send_packet(BROADCAST_ID, SYNC_WRITE, &parameters)
            .await
    }

    pub fn into_serial(self) -> T {
        self.serial
    }

    fn motor_id(&self, motor: &str) -> Result<u8, FeetechError> {
        self.motors
            .get(motor)
            .copied()
            .ok_or_else(|| FeetechError::UnknownMotor(motor.to_owned()))
    }

    async fn write_register(
        &mut self,
        id: u8,
        address: u8,
        value: &[u8],
    ) -> Result<(), FeetechError> {
        let mut parameters = vec![address];
        parameters.extend_from_slice(value);
        self.send_packet(id, WRITE, &parameters).await?;
        let _ = self.read_status(id).await?;
        Ok(())
    }

    async fn send_packet(
        &mut self,
        id: u8,
        instruction: u8,
        parameters: &[u8],
    ) -> Result<(), FeetechError> {
        self.serial
            .write_all(&instruction_packet(id, instruction, parameters))
            .await
    }

    async fn read_status(&mut self, expected_id: u8) -> Result<(u8, Vec<u8>), FeetechError> {
        let mut raw = Vec::new();
        loop {
            let chunk = self.serial.read_some(64).await?;
            if chunk.is_empty() {
                return Err(FeetechError::Packet(
                    "serial read ended before a status packet",
                ));
            }
            raw.extend_from_slice(&chunk);
            if raw.len() >= 4 {
                let length = raw[3] as usize;
                let total = length + 4;
                if raw.len() >= total {
                    return parse_status(&raw[..total], expected_id);
                }
            }
            if raw.len() > 260 {
                return Err(FeetechError::Packet("status packet exceeds protocol limit"));
            }
        }
    }
}

fn instruction_packet(id: u8, instruction: u8, parameters: &[u8]) -> Vec<u8> {
    let length = u8::try_from(parameters.len() + 2).expect("Feetech packet length fits in u8");
    let mut packet = Vec::with_capacity(parameters.len() + 6);
    packet.extend_from_slice(&HEADER);
    packet.extend_from_slice(&[id, length, instruction]);
    packet.extend_from_slice(parameters);
    packet.push(checksum(&packet[2..]));
    packet
}

fn parse_status(raw: &[u8], expected_id: u8) -> Result<(u8, Vec<u8>), FeetechError> {
    if raw.len() < 6 || raw[..2] != HEADER {
        return Err(FeetechError::Packet("missing status header"));
    }
    if raw[2] != expected_id {
        return Err(FeetechError::Packet(
            "status packet has an unexpected motor id",
        ));
    }
    let length = raw[3] as usize;
    if raw.len() != length + 4 || length < 2 {
        return Err(FeetechError::Packet("status packet has invalid length"));
    }
    if checksum(&raw[2..raw.len() - 1]) != raw[raw.len() - 1] {
        return Err(FeetechError::Packet("status checksum mismatch"));
    }
    let error = raw[4];
    if error != 0 {
        return Err(FeetechError::ServoStatus {
            id: expected_id,
            error,
        });
    }
    Ok((error, raw[5..raw.len() - 1].to_vec()))
}

fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

/// Stable joint ordering used by SO100 and SO101 adapters and LeRobot bridges.
pub const SO_ARM_JOINTS: [&str; 6] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
];

/// Factory for the standard SO-ARM controller-board wiring (servo IDs 1 through 6).
///
/// The selector and serial configuration are supplied by the operator-side runtime
/// configuration. Dora graphs never need a device path or serial parameter.
pub async fn open_so_arm(
    runtime: &HalRuntime,
    owner: OwnerId,
    selector: ResourceSelector,
    serial_config: SerialConfig,
    config: SoArmConfig,
) -> Result<SoArmAdapter<HalSerialIo>, SoArmError> {
    let serial = HalSerialIo::open(runtime, owner, selector, serial_config).await?;
    let motors = SO_ARM_JOINTS
        .iter()
        .enumerate()
        .map(|(index, name)| {
            (
                (*name).to_owned(),
                u8::try_from(index + 1).expect("SO-ARM IDs fit into u8"),
            )
        })
        .collect();
    let bus = FeetechBus::new(serial, motors)?;
    SoArmAdapter::new(bus, config)
}

const SO_ARM_TICKS_PER_TURN: f64 = 4096.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimit {
    pub minimum_rad: f64,
    pub maximum_rad: f64,
    pub max_velocity_rad_s: f64,
}

impl JointLimit {
    pub const fn new(minimum_rad: f64, maximum_rad: f64, max_velocity_rad_s: f64) -> Self {
        Self {
            minimum_rad,
            maximum_rad,
            max_velocity_rad_s,
        }
    }

    fn validate(self) -> bool {
        self.minimum_rad.is_finite()
            && self.maximum_rad.is_finite()
            && self.max_velocity_rad_s.is_finite()
            && self.minimum_rad < self.maximum_rad
            && self.max_velocity_rad_s > 0.0
    }
}

/// Maps a robot-space joint angle to a calibrated Feetech position tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JointCalibration {
    pub zero_tick: u16,
    pub direction: i8,
}

impl JointCalibration {
    pub const fn new(zero_tick: u16, direction: i8) -> Self {
        Self {
            zero_tick,
            direction,
        }
    }

    fn validate(self) -> bool {
        self.zero_tick < SO_ARM_TICKS_PER_TURN as u16 && matches!(self.direction, -1 | 1)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SoArmConfig {
    pub robot_id: String,
    pub limits: [JointLimit; 6],
    pub calibration: [JointCalibration; 6],
}

impl SoArmConfig {
    pub fn new(
        robot_id: impl Into<String>,
        limits: [JointLimit; 6],
        calibration: [JointCalibration; 6],
    ) -> Result<Self, SoArmError> {
        let robot_id = robot_id.into();
        if robot_id.trim().is_empty() {
            return Err(SoArmError::InvalidConfiguration("robot id is empty"));
        }
        if !limits.iter().copied().all(JointLimit::validate) {
            return Err(SoArmError::InvalidConfiguration("invalid joint limit"));
        }
        if !calibration.iter().copied().all(JointCalibration::validate) {
            return Err(SoArmError::InvalidConfiguration(
                "invalid joint calibration",
            ));
        }
        Ok(Self {
            robot_id,
            limits,
            calibration,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoArmAction {
    pub positions_rad: [f64; 6],
    pub timestamp_ns: u64,
}

impl SoArmAction {
    pub const fn new(positions_rad: [f64; 6], timestamp_ns: u64) -> Self {
        Self {
            positions_rad,
            timestamp_ns,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoArmObservation {
    pub positions_rad: [f64; 6],
    pub timestamp_ns: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoArmState {
    Disconnected,
    ConnectedUncalibrated,
    ConnectedDisabled,
    Enabled,
}

#[derive(Debug, Error)]
pub enum SoArmError {
    #[error(transparent)]
    Feetech(Box<FeetechError>),
    #[error("invalid SO-ARM configuration: {0}")]
    InvalidConfiguration(&'static str),
    #[error("invalid lifecycle state for {operation}: {state:?}")]
    InvalidState {
        operation: &'static str,
        state: SoArmState,
    },
    #[error("invalid action timestamp {timestamp_ns}; it must increase monotonically")]
    InvalidTimestamp { timestamp_ns: u64 },
    #[error("joint {joint} maps outside the Feetech single-turn range")]
    CalibrationRange { joint: &'static str },
}

impl From<FeetechError> for SoArmError {
    fn from(error: FeetechError) -> Self {
        Self::Feetech(Box::new(error))
    }
}

/// Robot-level SO-ARM adapter above the raw Feetech protocol driver.
///
/// It is intentionally generic over serial I/O so protocol and safety behavior
/// can be tested without hardware. `HalSerialIo` is the production transport.
pub struct SoArmAdapter<T> {
    bus: FeetechBus<T>,
    config: SoArmConfig,
    state: SoArmState,
    calibration_id: Option<String>,
    last_action: Option<SoArmObservation>,
}

impl<T: SerialIo> SoArmAdapter<T> {
    pub fn new(bus: FeetechBus<T>, config: SoArmConfig) -> Result<Self, SoArmError> {
        Ok(Self {
            bus,
            config,
            state: SoArmState::Disconnected,
            calibration_id: None,
            last_action: None,
        })
    }

    pub const fn state(&self) -> SoArmState {
        self.state
    }

    pub fn calibration_id(&self) -> Option<&str> {
        self.calibration_id.as_deref()
    }

    /// Connects in a safe state. Torque is disabled before the adapter becomes usable.
    pub async fn connect(&mut self) -> Result<(), SoArmError> {
        self.require_state("connect", SoArmState::Disconnected)?;
        self.bus.set_torque(false).await?;
        self.state = SoArmState::ConnectedUncalibrated;
        Ok(())
    }

    /// Accepts a calibration profile that was persisted by the calibration workflow.
    pub fn accept_calibration(
        &mut self,
        calibration_id: impl Into<String>,
    ) -> Result<(), SoArmError> {
        self.require_state("accept calibration", SoArmState::ConnectedUncalibrated)?;
        let calibration_id = calibration_id.into();
        if calibration_id.trim().is_empty() {
            return Err(SoArmError::InvalidConfiguration("calibration id is empty"));
        }
        self.calibration_id = Some(calibration_id);
        self.state = SoArmState::ConnectedDisabled;
        Ok(())
    }

    /// Reads the current pose before torque enable, making rate limiting begin from reality.
    pub async fn enable(&mut self, timestamp_ns: u64) -> Result<(), SoArmError> {
        self.require_state("enable", SoArmState::ConnectedDisabled)?;
        if timestamp_ns == 0 {
            return Err(SoArmError::InvalidTimestamp { timestamp_ns });
        }
        let observation = self.read_observation(timestamp_ns).await?;
        self.bus.set_torque(true).await?;
        self.last_action = Some(observation);
        self.state = SoArmState::Enabled;
        Ok(())
    }

    pub async fn observe(&mut self, timestamp_ns: u64) -> Result<SoArmObservation, SoArmError> {
        if self.state == SoArmState::Disconnected {
            return Err(self.invalid_state("observe"));
        }
        self.read_observation(timestamp_ns).await
    }

    /// Applies a clamped, rate-limited joint target only while torque is explicitly enabled.
    pub async fn apply_action(&mut self, action: SoArmAction) -> Result<SoArmAction, SoArmError> {
        self.require_state("apply action", SoArmState::Enabled)?;
        let previous = self
            .last_action
            .expect("enabled adapter has an observed pose");
        if action.timestamp_ns <= previous.timestamp_ns {
            return Err(SoArmError::InvalidTimestamp {
                timestamp_ns: action.timestamp_ns,
            });
        }
        let elapsed_s = (action.timestamp_ns - previous.timestamp_ns) as f64 / 1_000_000_000.0;
        let mut positions_rad = [0.0; 6];
        let mut goal_ticks = Vec::with_capacity(6);
        for index in 0..SO_ARM_JOINTS.len() {
            let limit = self.config.limits[index];
            let limited = action.positions_rad[index].clamp(limit.minimum_rad, limit.maximum_rad);
            let maximum_delta = limit.max_velocity_rad_s * elapsed_s;
            let applied = limited.clamp(
                previous.positions_rad[index] - maximum_delta,
                previous.positions_rad[index] + maximum_delta,
            );
            positions_rad[index] = applied;
            goal_ticks.push((SO_ARM_JOINTS[index], self.radians_to_ticks(index, applied)?));
        }
        self.bus.write_goal_ticks(&goal_ticks).await?;
        let applied = SoArmAction::new(positions_rad, action.timestamp_ns);
        self.last_action = Some(SoArmObservation {
            positions_rad,
            timestamp_ns: action.timestamp_ns,
        });
        Ok(applied)
    }

    /// Mandatory local stop. It never depends on cloud or workflow connectivity.
    pub async fn safe_stop(&mut self) -> Result<(), SoArmError> {
        if self.state == SoArmState::Disconnected {
            return Ok(());
        }
        self.bus.set_torque(false).await?;
        self.last_action = None;
        self.state = if self.calibration_id.is_some() {
            SoArmState::ConnectedDisabled
        } else {
            SoArmState::ConnectedUncalibrated
        };
        Ok(())
    }

    /// Safe-disables the physical arm; transport ownership is released by the caller.
    pub async fn disconnect(&mut self) -> Result<(), SoArmError> {
        self.safe_stop().await?;
        self.state = SoArmState::Disconnected;
        Ok(())
    }

    pub fn into_bus(self) -> FeetechBus<T> {
        self.bus
    }

    fn require_state(
        &self,
        operation: &'static str,
        expected: SoArmState,
    ) -> Result<(), SoArmError> {
        if self.state == expected {
            Ok(())
        } else {
            Err(self.invalid_state(operation))
        }
    }

    fn invalid_state(&self, operation: &'static str) -> SoArmError {
        SoArmError::InvalidState {
            operation,
            state: self.state,
        }
    }

    async fn read_observation(
        &mut self,
        timestamp_ns: u64,
    ) -> Result<SoArmObservation, SoArmError> {
        let mut positions_rad = [0.0; 6];
        for (index, name) in SO_ARM_JOINTS.iter().enumerate() {
            let ticks = self.bus.read_position_ticks(name).await?;
            positions_rad[index] = self.ticks_to_radians(index, ticks);
        }
        Ok(SoArmObservation {
            positions_rad,
            timestamp_ns,
        })
    }

    fn radians_to_ticks(&self, index: usize, radians: f64) -> Result<u16, SoArmError> {
        let calibration = self.config.calibration[index];
        let offset = (radians * SO_ARM_TICKS_PER_TURN / TAU).round() as i32;
        let tick = i32::from(calibration.zero_tick) + i32::from(calibration.direction) * offset;
        u16::try_from(tick)
            .ok()
            .filter(|tick| *tick < SO_ARM_TICKS_PER_TURN as u16)
            .ok_or(SoArmError::CalibrationRange {
                joint: SO_ARM_JOINTS[index],
            })
    }

    fn ticks_to_radians(&self, index: usize, ticks: u16) -> f64 {
        let calibration = self.config.calibration[index];
        let mut delta = i32::from(ticks) - i32::from(calibration.zero_tick);
        if delta > 2048 {
            delta -= 4096;
        } else if delta < -2048 {
            delta += 4096;
        }
        f64::from(delta) * TAU / SO_ARM_TICKS_PER_TURN * f64::from(calibration.direction)
    }
}

impl SoArmAdapter<HalSerialIo> {
    /// Safe-stops before releasing the HAL serial lease.
    pub async fn close(mut self) -> Result<(), SoArmError> {
        self.disconnect().await?;
        self.bus.into_serial().close().await?;
        Ok(())
    }
}
