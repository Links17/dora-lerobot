use b601_dm_hal_node::{RuntimeConfig, is_discovery_request, parse_lifecycle};
use dora_lerobot_hal::{DamiaoAction, DmAdapter, open_damiao_serial};
use dora_node_api::{DoraNode, Event, arrow::array::StringArray, dora_core::config::DataId};
use eyre::{Context, Result};
use seeed_hal_adapter_serialport::SerialPortAdapter;
use seeed_hal_runtime::HalRuntime;
use serde::Deserialize;
use std::{
    fs,
    time::{SystemTime, UNIX_EPOCH},
};

#[derive(Debug, Deserialize)]
struct ActionMessage {
    positions_rad: [f32; 7],
    timestamp_ns: u64,
    velocity_limit_rad_s: f32,
    torque_limit_ratio: f32,
}

fn main() -> Result<()> {
    if is_discovery_request(std::env::args()) {
        return discover();
    }
    let path = std::env::var("DORA_LEROBOT_B601_DM_HAL_CONFIG")
        .wrap_err("DORA_LEROBOT_B601_DM_HAL_CONFIG must name an operator configuration")?;
    let config = RuntimeConfig::from_yaml(&fs::read_to_string(path)?)?;
    let runtime = HalRuntime::builder()
        .serial_adapter(SerialPortAdapter::new())
        .build();
    let tokio = tokio::runtime::Runtime::new()?;
    let transport = tokio.block_on(open_damiao_serial(
        &runtime,
        config.owner,
        config.resource,
        config.serial,
    ))?;
    let mut adapter = DmAdapter::new(transport);
    tokio.block_on(adapter.connect())?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    while let Some(event) = events.recv() {
        match event {
            Event::ParamUpdate { key, value } if key == "lifecycle" => {
                lifecycle(&mut adapter, &tokio, &config.calibration_id, &value)?
            }
            Event::Input { id, data, .. } if id.as_str() == "lifecycle" => {
                let values: StringArray = data.to_data().into();
                lifecycle(
                    &mut adapter,
                    &tokio,
                    &config.calibration_id,
                    &serde_json::from_str(values.value(0))?,
                )?;
            }
            Event::Input { id, data, .. } if id.as_str() == "action" => {
                let values: StringArray = data.to_data().into();
                let action: ActionMessage = serde_json::from_str(values.value(0))?;
                match tokio.block_on(adapter.apply_action(DamiaoAction::new(
                    action.positions_rad,
                    action.timestamp_ns,
                    action.velocity_limit_rad_s,
                    action.torque_limit_ratio,
                ))) {
                    Ok(()) => node.send_output(
                        DataId::from("safe_action".to_owned()),
                        Default::default(),
                        StringArray::from(vec![values.value(0)]),
                    )?,
                    Err(error) => send_status(&mut node, &adapter, &error.to_string())?,
                }
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                match tokio.block_on(adapter.observe(now_ns())) {
                    Ok(observation) => {
                        let message = serde_json::json!({
                            "schema_version": "v1",
                            "timestamp_ns": observation.timestamp_ns,
                            "joint_ids": [1, 2, 3, 4, 5, 6, 7],
                            "positions_rad": observation.feedback.map(|feedback| feedback.position_rad),
                            "velocities_rad_s": observation.feedback.map(|feedback| feedback.velocity_rad_s),
                            "torques_nm": observation.feedback.map(|feedback| feedback.torque_nm),
                        }).to_string();
                        node.send_output(
                            DataId::from("observation".to_owned()),
                            Default::default(),
                            StringArray::from(vec![message.as_str()]),
                        )?
                    }
                    Err(error) => send_status(&mut node, &adapter, &error.to_string())?,
                }
            }
            _ => {}
        }
    }
    Ok(())
}

fn lifecycle<T: dora_lerobot_hal::DamiaoTransport>(
    adapter: &mut DmAdapter<T>,
    tokio: &tokio::runtime::Runtime,
    calibration_id: &str,
    value: &serde_json::Value,
) -> Result<()> {
    match parse_lifecycle(value)? {
        "calibrate" => adapter.accept_calibration(calibration_id)?,
        "enable" => tokio.block_on(adapter.enable())?,
        "disable" => tokio.block_on(adapter.safe_stop())?,
        _ => unreachable!(),
    }
    Ok(())
}
fn send_status<T: dora_lerobot_hal::DamiaoTransport>(
    node: &mut DoraNode,
    adapter: &DmAdapter<T>,
    fault: &str,
) -> Result<()> {
    let status = serde_json::json!({"schema_version":"v1","state":format!("{:?}", adapter.state()),"fault":fault}).to_string();
    node.send_output(
        DataId::from("status".to_owned()),
        Default::default(),
        StringArray::from(vec![status.as_str()]),
    )?;
    Ok(())
}
fn discover() -> Result<()> {
    let runtime = HalRuntime::builder()
        .serial_adapter(SerialPortAdapter::new())
        .build();
    let tokio = tokio::runtime::Runtime::new()?;
    for descriptor in tokio.block_on(runtime.enumerate_serial())? {
        println!(
            "resource: {} endpoint: {}",
            descriptor.id().as_str(),
            descriptor.endpoint().as_str()
        );
    }
    Ok(())
}
fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock")
        .as_nanos()
        .try_into()
        .expect("timestamp")
}
