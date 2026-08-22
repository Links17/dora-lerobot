use crate::{RsCanFrame, RsMitError, RsMitTransport};
use async_trait::async_trait;
use seeed_hal_can::{CanFilterSet, CanFrame, CanId, CanLinkExpectation, CanMode, CanOpenConfig};
use seeed_hal_core::{LeaseMode, OwnerId, ResourceSelector};
use seeed_hal_runtime::{CanHandle, HalRuntime};
use std::time::Duration;

/// HAL-owned SocketCAN session for B601-RS. The RS driver only sees standard
/// eight-byte CAN payloads; resource leases and SocketCAN remain below it.
pub struct HalRsCanTransport {
    handle: CanHandle,
    receive_timeout: Duration,
}

impl HalRsCanTransport {
    pub async fn open(
        runtime: &HalRuntime,
        owner: OwnerId,
        selector: ResourceSelector,
        receive_timeout: Duration,
    ) -> Result<Self, RsMitError> {
        let expectation = CanLinkExpectation::new(
            Some(CanMode::Classic),
            Some(1_000_000),
            None,
            Some(false),
            Some(false),
        )
        .map_err(hal)?;
        let handle = runtime
            .open_can(
                owner,
                selector,
                LeaseMode::Control,
                CanOpenConfig::Attach(expectation),
                CanFilterSet::new(vec![]).map_err(hal)?,
            )
            .await
            .map_err(hal)?;
        Ok(Self {
            handle,
            receive_timeout,
        })
    }
    pub async fn close(mut self) -> Result<(), RsMitError> {
        self.handle.close().await.map_err(hal)
    }
}

#[async_trait]
impl RsMitTransport for HalRsCanTransport {
    async fn send_frame(&mut self, frame: RsCanFrame) -> Result<(), RsMitError> {
        self.handle
            .send(
                CanFrame::classic_data(
                    CanId::standard(frame.arbitration_id()).map_err(hal)?,
                    frame.data().to_vec(),
                )
                .map_err(hal)?,
            )
            .await
            .map_err(|error| RsMitError::Transport(error.to_string()))
    }
    async fn receive_frame(&mut self) -> Result<Option<RsCanFrame>, RsMitError> {
        let frames = self
            .handle
            .receive(1, self.receive_timeout)
            .await
            .map_err(hal)?;
        let Some(frame) = frames.into_iter().next() else {
            return Ok(None);
        };
        let CanFrame::ClassicData {
            id: CanId::Standard(id),
            data,
        } = frame.into_frame()
        else {
            return Err(RsMitError::Frame("expected classic standard CAN feedback"));
        };
        let bytes: [u8; 8] = data
            .as_ref()
            .try_into()
            .map_err(|_| RsMitError::Frame("RS feedback payload must be eight bytes"))?;
        Ok(Some(RsCanFrame::standard(id, bytes)))
    }
}
fn hal(error: seeed_hal_core::HalError) -> RsMitError {
    RsMitError::Transport(error.to_string())
}
