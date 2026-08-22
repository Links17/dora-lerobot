use dora_lerobot_hal::B601_RS_MOTOR_IDS;
use seeed_hal_core::{IdentityQuality, OwnerId, ResourceId, ResourceSelector, TransportKind};
use serde::Deserialize;
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("invalid YAML configuration: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("invalid HAL identifier: {0}")]
    Hal(Box<seeed_hal_core::HalError>),
    #[error("RS configuration must use CAN and seven canonical motor ids")]
    Invalid,
}
impl From<seeed_hal_core::HalError> for ConfigError {
    fn from(e: seeed_hal_core::HalError) -> Self {
        Self::Hal(Box::new(e))
    }
}
pub struct RuntimeConfig {
    pub owner: OwnerId,
    pub resource: ResourceSelector,
    pub receive_timeout: Duration,
    pub calibration_id: String,
    pub max_relative_target_rad: f32,
    pub limits: [dora_lerobot_hal::RsMotorLimits; 7],
    pub kp: [f32; 7],
    pub kd: [f32; 7],
}
#[derive(Debug, Deserialize)]
struct RawConfig {
    owner: String,
    resource: RawResource,
    can: RawCan,
    robot: RawRobot,
    rs_control: RawControl,
}
#[derive(Debug, Deserialize)]
struct RawResource {
    id: String,
    minimum_identity_quality: IdentityQuality,
    transport: TransportKind,
}
#[derive(Debug, Deserialize)]
struct RawCan {
    receive_timeout_ms: u64,
}
#[derive(Debug, Deserialize)]
struct RawRobot {
    calibration_id: String,
    motor_ids: Vec<u16>,
    max_relative_target_deg: f32,
}
#[derive(Debug, Deserialize)]
struct RawControl {
    mit_kp: [f32; 7],
    mit_kd: [f32; 7],
}
impl RuntimeConfig {
    pub fn from_yaml(source: &str) -> Result<Self, ConfigError> {
        let raw: RawConfig = serde_yaml::from_str(source)?;
        if raw.resource.transport != TransportKind::Can
            || raw.can.receive_timeout_ms == 0
            || raw.robot.calibration_id.trim().is_empty()
            || raw.robot.motor_ids != B601_RS_MOTOR_IDS
            || !raw.robot.max_relative_target_deg.is_finite()
            || raw.robot.max_relative_target_deg <= 0.0
            || raw
                .rs_control
                .mit_kp
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=500.0).contains(v))
            || raw
                .rs_control
                .mit_kd
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=5.0).contains(v))
        {
            return Err(ConfigError::Invalid);
        }
        Ok(Self {
            owner: OwnerId::parse(raw.owner)?,
            resource: ResourceSelector::exact(
                ResourceId::parse(raw.resource.id)?,
                raw.resource.minimum_identity_quality,
                raw.resource.transport,
            ),
            receive_timeout: Duration::from_millis(raw.can.receive_timeout_ms),
            calibration_id: raw.robot.calibration_id,
            max_relative_target_rad: raw.robot.max_relative_target_deg.to_radians(),
            limits: [
                dora_lerobot_hal::RsMotorLimits::new(112.5, 50.0, 36.0),
                dora_lerobot_hal::RsMotorLimits::new(112.5, 50.0, 36.0),
                dora_lerobot_hal::RsMotorLimits::new(112.5, 50.0, 36.0),
                dora_lerobot_hal::RsMotorLimits::new(12.57, 33.0, 14.0),
                dora_lerobot_hal::RsMotorLimits::new(12.57, 33.0, 14.0),
                dora_lerobot_hal::RsMotorLimits::new(12.57, 33.0, 14.0),
                dora_lerobot_hal::RsMotorLimits::new(12.57, 33.0, 14.0),
            ],
            kp: raw.rs_control.mit_kp,
            kd: raw.rs_control.mit_kd,
        })
    }
}
pub fn is_discovery_request(args: impl IntoIterator<Item = impl AsRef<str>>) -> bool {
    args.into_iter().skip(1).any(|a| a.as_ref() == "--discover")
}
pub fn parse_lifecycle(value: &serde_json::Value) -> Result<&'static str, ConfigError> {
    match value.as_str() {
        Some("calibrate") => Ok("calibrate"),
        Some("enable") => Ok("enable"),
        Some("disable") => Ok("disable"),
        _ => Err(ConfigError::Invalid),
    }
}
