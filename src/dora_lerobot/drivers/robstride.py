"""RobStride CAN/MIT driver."""

from __future__ import annotations

from collections.abc import Mapping

from .can_mit import CanTransport
from .damiao import DamiaoMitDriver


class RobStrideMitDriver(DamiaoMitDriver):
    """RobStride uses the same public JointDriver surface with a distinct identity.

    The implementation intentionally shares the standardized MIT packet transport.
    Hardware-specific mode setup belongs in the transport supplied by the integration.
    """

    def __init__(self, transport: CanTransport, motor_ids: Mapping[str, int]) -> None:
        super().__init__(transport, motor_ids)
