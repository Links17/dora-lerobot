//! Operator-side configuration for the HAL-backed SO-ARM Dora node.

use dora_lerobot_hal::{JointCalibration, JointLimit, SO_ARM_JOINTS, SoArmConfig, SoArmError};
use seeed_hal_core::{IdentityQuality, OwnerId, ResourceId, ResourceSelector, TransportKind};
use seeed_hal_serial::SerialConfig;
use serde::Deserialize;
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("invalid YAML configuration: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("invalid HAL identifier: {0}")]
    Hal(Box<seeed_hal_core::HalError>),
    #[error(transparent)]
    Robot(#[from] SoArmError),
    #[error("SO-ARM configuration must define joints in the canonical six-joint order")]
    JointOrder,
    #[error("lifecycle parameter must be calibrate, enable, or disable")]
    InvalidLifecycle,
}

impl From<seeed_hal_core::HalError> for ConfigError {
    fn from(error: seeed_hal_core::HalError) -> Self {
        Self::Hal(Box::new(error))
    }
}

pub struct RuntimeConfig {
    pub owner: OwnerId,
    pub resource: ResourceSelector,
    pub serial: SerialConfig,
    pub robot: RobotRuntimeConfig,
}

pub struct RobotRuntimeConfig {
    pub calibration_id: String,
    pub config: SoArmConfig,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleCommand {
    Calibrate,
    Enable,
    Disable,
}

pub fn parse_lifecycle(value: &serde_json::Value) -> Result<LifecycleCommand, ConfigError> {
    match value.as_str() {
        Some("calibrate") => Ok(LifecycleCommand::Calibrate),
        Some("enable") => Ok(LifecycleCommand::Enable),
        Some("disable") => Ok(LifecycleCommand::Disable),
        _ => Err(ConfigError::InvalidLifecycle),
    }
}

/// Safety rejections are reported to the workflow but do not crash the local node.
pub fn is_recoverable_action_error(error: &SoArmError) -> bool {
    matches!(
        error,
        SoArmError::InvalidState { .. } | SoArmError::InvalidTimestamp { .. }
    )
}

pub fn is_discovery_request(args: impl IntoIterator<Item = impl AsRef<str>>) -> bool {
    args.into_iter()
        .skip(1)
        .any(|arg| arg.as_ref() == "--discover")
}

#[derive(Deserialize)]
struct RawConfig {
    owner: String,
    resource: RawResource,
    serial: RawSerial,
    robot: RawRobot,
}

#[derive(Deserialize)]
struct RawResource {
    id: String,
    minimum_identity_quality: IdentityQuality,
    transport: TransportKind,
}

#[derive(Deserialize)]
struct RawSerial {
    baud_rate: u32,
    read_timeout_ms: u64,
}

#[derive(Deserialize)]
struct RawRobot {
    id: String,
    calibration_id: String,
    joints: Vec<RawJoint>,
}

#[derive(Debug, Deserialize)]
struct RawJoint {
    name: String,
    minimum_rad: f64,
    maximum_rad: f64,
    max_velocity_rad_s: f64,
    zero_tick: u16,
    direction: i8,
}

impl RuntimeConfig {
    pub fn from_yaml(source: &str) -> Result<Self, ConfigError> {
        let raw: RawConfig = serde_yaml::from_str(source)?;
        if raw.resource.transport != TransportKind::Serial
            || raw.serial.baud_rate == 0
            || raw.serial.read_timeout_ms == 0
            || raw.robot.calibration_id.trim().is_empty()
            || raw.robot.joints.len() != SO_ARM_JOINTS.len()
            || raw
                .robot
                .joints
                .iter()
                .map(|joint| joint.name.as_str())
                .ne(SO_ARM_JOINTS.iter().copied())
        {
            return Err(ConfigError::JointOrder);
        }
        let joints: [RawJoint; 6] = raw
            .robot
            .joints
            .try_into()
            .expect("joint count was checked before conversion");
        let limits = joints.each_ref().map(|joint| {
            JointLimit::new(
                joint.minimum_rad,
                joint.maximum_rad,
                joint.max_velocity_rad_s,
            )
        });
        let calibration = joints
            .each_ref()
            .map(|joint| JointCalibration::new(joint.zero_tick, joint.direction));
        let mut serial = SerialConfig::default();
        serial.baud_rate = raw.serial.baud_rate;
        serial.read_timeout = Duration::from_millis(raw.serial.read_timeout_ms);
        Ok(Self {
            owner: OwnerId::parse(raw.owner)?,
            resource: ResourceSelector::exact(
                ResourceId::parse(raw.resource.id)?,
                raw.resource.minimum_identity_quality,
                raw.resource.transport,
            ),
            serial,
            robot: RobotRuntimeConfig {
                calibration_id: raw.robot.calibration_id,
                config: SoArmConfig::new(raw.robot.id, limits, calibration)?,
            },
        })
    }
}
