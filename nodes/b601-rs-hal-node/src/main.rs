use b601_rs_hal_node::{RuntimeConfig, is_discovery_request, parse_lifecycle};
use dora_lerobot_hal::{HalRsCanTransport, RsAdapter};
use dora_node_api::{DoraNode, Event, arrow::array::StringArray, dora_core::config::DataId};
use eyre::{Context, Result};
use seeed_hal_adapter_socketcan::SocketCanAdapter;
use seeed_hal_runtime::HalRuntime;
use serde::Deserialize;
use std::{
    fs,
    time::{SystemTime, UNIX_EPOCH},
};

#[derive(Debug, Deserialize)]
struct Action {
    schema_version: String,
    joint_names: [String; 7],
    positions_rad: [f32; 7],
    timestamp_ns: u64,
    control_mode: String,
}
const JOINTS: [&str; 7] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
];
fn main() -> Result<()> {
    if is_discovery_request(std::env::args()) {
        return discover();
    }
    let path = std::env::var("DORA_LEROBOT_B601_RS_HAL_CONFIG")
        .wrap_err("DORA_LEROBOT_B601_RS_HAL_CONFIG must name an operator configuration")?;
    let config = RuntimeConfig::from_yaml(&fs::read_to_string(path)?)?;
    let runtime = HalRuntime::builder()
        .can_adapter(SocketCanAdapter::new())
        .build();
    let tokio = tokio::runtime::Runtime::new()?;
    let transport = tokio.block_on(HalRsCanTransport::open(
        &runtime,
        config.owner,
        config.resource,
        config.receive_timeout,
    ))?;
    let mut adapter = RsAdapter::new_with_gains(
        transport,
        config.limits,
        config.max_relative_target_rad,
        config.kp,
        config.kd,
    );
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
                let action: Action = serde_json::from_str(values.value(0))?;
                if action.schema_version != "v1"
                    || action.control_mode != "position"
                    || action.joint_names.iter().map(String::as_str).ne(JOINTS)
                {
                    send_status(
                        &mut node,
                        &adapter,
                        "action does not match B601-RS v1 position contract",
                    )?;
                    continue;
                }
                match tokio
                    .block_on(adapter.apply_action(action.positions_rad, action.timestamp_ns))
                {
                    Ok(()) => node.send_output(
                        DataId::from("safe_action".to_owned()),
                        Default::default(),
                        StringArray::from(vec![values.value(0)]),
                    )?,
                    Err(e) => send_status(&mut node, &adapter, &e.to_string())?,
                }
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                match tokio.block_on(adapter.observe(now_ns())) {
                    Ok(o) => {
                        let msg = serde_json::json!({"schema_version":"v1","timestamp_ns":o.timestamp_ns,"joints_rad":JOINTS.into_iter().zip(o.feedback.map(|f| f.position_rad)).map(|(n,p)|(n.to_owned(),serde_json::json!(p))).collect::<serde_json::Map<_,_>>(),"control_mode":"position"}).to_string();
                        node.send_output(
                            DataId::from("observation".to_owned()),
                            Default::default(),
                            StringArray::from(vec![msg.as_str()]),
                        )?;
                    }
                    Err(e) => send_status(&mut node, &adapter, &e.to_string())?,
                }
            }
            _ => {}
        }
    }
    Ok(())
}
fn lifecycle<T: dora_lerobot_hal::RsMitTransport>(
    a: &mut RsAdapter<T>,
    rt: &tokio::runtime::Runtime,
    id: &str,
    v: &serde_json::Value,
) -> Result<()> {
    match parse_lifecycle(v)? {
        "calibrate" => a.accept_calibration(id)?,
        "enable" => rt.block_on(a.enable())?,
        "disable" => rt.block_on(a.safe_stop())?,
        _ => unreachable!(),
    }
    Ok(())
}
fn send_status<T: dora_lerobot_hal::RsMitTransport>(
    n: &mut DoraNode,
    a: &RsAdapter<T>,
    fault: &str,
) -> Result<()> {
    let s =
        serde_json::json!({"schema_version":"v1","state":format!("{:?}",a.state()),"fault":fault})
            .to_string();
    n.send_output(
        DataId::from("status".to_owned()),
        Default::default(),
        StringArray::from(vec![s.as_str()]),
    )?;
    Ok(())
}
fn discover() -> Result<()> {
    let runtime = HalRuntime::builder()
        .can_adapter(SocketCanAdapter::new())
        .build();
    let rt = tokio::runtime::Runtime::new()?;
    for d in rt.block_on(runtime.enumerate_can())? {
        println!(
            "resource: {} endpoint: {}",
            d.id().as_str(),
            d.endpoint().as_str()
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
