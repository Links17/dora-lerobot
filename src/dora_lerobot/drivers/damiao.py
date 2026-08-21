"""Damiao CAN/MIT driver."""

from __future__ import annotations

from collections.abc import Mapping

from .can_mit import CanTransport, decode_mit_position, encode_mit_position


class DamiaoMitDriver:
    def __init__(self, transport: CanTransport, motor_ids: Mapping[str, int]) -> None:
        if not motor_ids or len(set(motor_ids.values())) != len(motor_ids):
            raise ValueError("motor_ids must be non-empty and unique")
        self.transport = transport
        self.motor_ids = dict(motor_ids)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(self.motor_ids)

    def connect(self) -> None:
        self.transport.connect()

    def disconnect(self) -> None:
        self.transport.disconnect()

    def enable_torque(self, enabled: bool) -> None:
        command = (
            b"\xff\xff\xff\xff\xff\xff\xff\xfc" if enabled else b"\xff\xff\xff\xff\xff\xff\xff\xfd"
        )
        for motor_id in self.motor_ids.values():
            from .can_mit import CanFrame

            self.transport.send(CanFrame(motor_id, command))

    def read_positions_rad(self) -> Mapping[str, float]:
        positions: dict[str, float] = {}
        id_to_joint = {motor_id: joint for joint, motor_id in self.motor_ids.items()}
        for _ in self.motor_ids:
            frame = self.transport.receive(0.02)
            if frame is not None and frame.arbitration_id in id_to_joint:
                positions[id_to_joint[frame.arbitration_id]] = decode_mit_position(frame.data)
        if set(positions) != set(self.motor_ids):
            missing = set(self.motor_ids) - set(positions)
            raise RuntimeError(f"missing Damiao feedback for {sorted(missing)}")
        return positions

    def write_positions_rad(self, positions_rad: Mapping[str, float]) -> None:
        if set(positions_rad) != set(self.motor_ids):
            raise ValueError("driver action joint set mismatch")
        from .can_mit import CanFrame

        for joint, position in positions_rad.items():
            self.transport.send(CanFrame(self.motor_ids[joint], encode_mit_position(position)))
