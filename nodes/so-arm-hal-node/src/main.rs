//! Dora 1.0 node for the HAL-backed SO-ARM control path.

use dora_lerobot_hal::{SO_ARM_JOINTS, SoArmAction, open_so_arm};
use dora_node_api::{DoraNode, Event, arrow::array::StringArray, dora_core::config::DataId};
use eyre::{Context, Result, bail};
use seeed_hal_adapter_serialport::SerialPortAdapter;
use seeed_hal_runtime::HalRuntime;
use serde::Deserialize;
use so_arm_hal_node::RuntimeConfig;
use std::fs;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Deserialize)]
struct ActionMessage {
    joint_names: Vec<String>,
    positions_rad: Vec<f64>,
    timestamp_ns: u64,
}

fn main() -> Result<()> {
    let config_path = std::env::var("DORA_LEROBOT_SO_ARM_HAL_CONFIG")
        .wrap_err("DORA_LEROBOT_SO_ARM_HAL_CONFIG must name an operator configuration")?;
    let config = RuntimeConfig::from_yaml(&fs::read_to_string(config_path)?)?;
    let runtime = HalRuntime::builder()
        .serial_adapter(SerialPortAdapter::new())
        .build();
    let tokio = tokio::runtime::Runtime::new()?;
    let mut adapter = tokio.block_on(open_so_arm(
        &runtime,
        config.owner,
        config.resource,
        config.serial,
        config.robot.config,
    ))?;
    let outcome = run(&mut adapter, &tokio, &config.robot.calibration_id);
    let close_result = tokio.block_on(adapter.close());
    outcome?;
    close_result?;
    Ok(())
}

fn run(
    adapter: &mut dora_lerobot_hal::SoArmAdapter<dora_lerobot_hal::HalSerialIo>,
    tokio: &tokio::runtime::Runtime,
    calibration_id: &str,
) -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    tokio.block_on(adapter.connect())?;
    while let Some(event) = events.recv() {
        let Event::Input { id, data, .. } = event else {
            continue;
        };
        match id.as_str() {
            "calibrate" => adapter.accept_calibration(calibration_id)?,
            "enable" => tokio.block_on(adapter.enable(now_ns()))?,
            "disable" => tokio.block_on(adapter.safe_stop())?,
            "tick" => {
                let observation = tokio.block_on(adapter.observe(now_ns()))?;
                let joints_rad = SO_ARM_JOINTS
                    .into_iter()
                    .zip(observation.positions_rad)
                    .map(|(name, position)| (name.to_owned(), serde_json::json!(position)))
                    .collect::<serde_json::Map<_, _>>();
                let message = serde_json::json!({
                    "schema_version": "v1",
                    "timestamp_ns": observation.timestamp_ns,
                    "joints_rad": joints_rad,
                    "fault": serde_json::Value::Null,
                })
                .to_string();
                node.send_output(
                    DataId::from("observation".to_owned()),
                    Default::default(),
                    StringArray::from(vec![message.as_str()]),
                )?;
            }
            "action" => {
                let input: StringArray = data.to_data().into();
                let message: ActionMessage = serde_json::from_str(input.value(0))?;
                if message
                    .joint_names
                    .iter()
                    .map(String::as_str)
                    .ne(SO_ARM_JOINTS)
                {
                    bail!("SO-ARM action joint order does not match the versioned contract");
                }
                let positions_rad: [f64; 6] = message
                    .positions_rad
                    .try_into()
                    .map_err(|_| eyre::eyre!("SO-ARM action must contain six positions"))?;
                let applied = tokio.block_on(
                    adapter.apply_action(SoArmAction::new(positions_rad, message.timestamp_ns)),
                )?;
                let output = serde_json::json!({
                    "schema_version": "v1",
                    "timestamp_ns": applied.timestamp_ns,
                    "joint_names": SO_ARM_JOINTS,
                    "positions_rad": applied.positions_rad,
                    "control_mode": "position",
                })
                .to_string();
                node.send_output(
                    DataId::from("safe_action".to_owned()),
                    Default::default(),
                    StringArray::from(vec![output.as_str()]),
                )?;
            }
            _ => {}
        }
    }
    Ok(())
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock must not precede the Unix epoch")
        .as_nanos()
        .try_into()
        .expect("nanoseconds since epoch fit into u64")
}
