use dora_lerobot_hal::B601_DM_MOTOR_IDS;
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
    #[error("DM configuration must use a serial HAL resource and seven canonical motor ids")]
    Invalid,
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
    pub calibration_id: String,
}

#[derive(Debug, Deserialize)]
struct RawConfig {
    owner: String,
    resource: RawResource,
    serial: RawSerial,
    robot: RawRobot,
}
#[derive(Debug, Deserialize)]
struct RawResource {
    id: String,
    minimum_identity_quality: IdentityQuality,
    transport: TransportKind,
}
#[derive(Debug, Deserialize)]
struct RawSerial {
    baud_rate: u32,
    read_timeout_ms: u64,
}
#[derive(Debug, Deserialize)]
struct RawRobot {
    calibration_id: String,
    motor_ids: Vec<u16>,
}

impl RuntimeConfig {
    pub fn from_yaml(source: &str) -> Result<Self, ConfigError> {
        let raw: RawConfig = serde_yaml::from_str(source)?;
        if raw.resource.transport != TransportKind::Serial
            || raw.serial.baud_rate == 0
            || raw.serial.read_timeout_ms == 0
            || raw.robot.calibration_id.trim().is_empty()
            || raw.robot.motor_ids != B601_DM_MOTOR_IDS
        {
            return Err(ConfigError::Invalid);
        }
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
            calibration_id: raw.robot.calibration_id,
        })
    }
}

pub fn is_discovery_request(args: impl IntoIterator<Item = impl AsRef<str>>) -> bool {
    args.into_iter()
        .skip(1)
        .any(|arg| arg.as_ref() == "--discover")
}

pub fn parse_lifecycle(value: &serde_json::Value) -> Result<&'static str, ConfigError> {
    match value.as_str() {
        Some("calibrate") => Ok("calibrate"),
        Some("enable") => Ok("enable"),
        Some("disable") => Ok("disable"),
        _ => Err(ConfigError::Invalid),
    }
}
