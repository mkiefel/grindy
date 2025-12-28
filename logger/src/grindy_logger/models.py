"""Data models matching Rust structs for WebSocket message deserialization."""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, List, Optional, TypeVar, Union

T = TypeVar('T')


class UserEvent(IntEnum):
    """Grinder state events matching Rust UserEvent enum."""
    Initializing = 0
    Idle = 1
    Stabilizing = 2
    Grinding = 3
    WaitingForRemoval = 4


@dataclass
class ScaleSetting:
    """Scale calibration settings."""
    offset: float
    inv_variance: float
    factor: float


@dataclass
class WeightReading:
    """Individual weight measurement with metadata."""
    timestamp_ms: int
    weight: float
    state: UserEvent
    coffee_weight: Optional[float]


@dataclass
class ConnectedMessage:
    """Initial connection message with current state."""
    state: UserEvent
    scale_setting: ScaleSetting
    timestamp_ms: int


@dataclass
class StateChangeMessage:
    """State transition notification."""
    state: UserEvent
    timestamp_ms: int


@dataclass
class WeightBatchMessage:
    """Batch of weight readings."""
    readings: List[WeightReading]


# Union type for all possible WebSocket messages
WsMessage = Union[ConnectedMessage, StateChangeMessage, WeightBatchMessage]

class PostcardDecoder:
    def __init__(self, buffer: bytes):
        self.buffer = buffer
        self.offset = 0

    def read_varint(self) -> int:
        """Decode variable-length integer using continuation bit encoding."""
        value = 0
        shift = 0
        while True:
            byte = self.buffer[self.offset]
            self.offset += 1
            value |= (byte & 0x7F) << shift
            if (byte & 0x80) == 0:
                break
            shift += 7
        return value

    def read_f32(self) -> float:
        """Decode 32-bit float (little-endian IEEE 754)."""
        value = struct.unpack_from('<f', self.buffer, self.offset)[0]
        self.offset += 4
        return value

    def read_option(self, read_fn: Callable[[], T]) -> Optional[T]:
        """Decode Option<T> - 0x00 for None, 0x01 for Some(value)."""
        tag = self.buffer[self.offset]
        self.offset += 1
        if tag == 0x00:
            return None
        return read_fn()

    def read_user_event(self) -> UserEvent:
        """Decode UserEvent enum."""
        variant = self.read_varint()
        return UserEvent(variant)

    def read_scale_setting(self) -> ScaleSetting:
        """Decode ScaleSetting struct."""
        return ScaleSetting(
            offset=self.read_f32(),
            inv_variance=self.read_f32(),
            factor=self.read_f32()
        )

    def read_weight_reading(self) -> WeightReading:
        """Decode WeightReading struct."""
        return WeightReading(
            timestamp_ms=self.read_varint(),
            weight=self.read_f32(),
            state=self.read_user_event(),
            coffee_weight=self.read_option(self.read_f32)
        )

    def read_vec(self, read_fn: Callable[[], T]) -> List[T]:
        """Decode Vec<T> - varint length followed by elements."""
        length = self.read_varint()
        return [read_fn() for _ in range(length)]

    def read_ws_message(self) -> WsMessage:
        """Decode WsMessage enum."""
        variant = self.read_varint()

        if variant == 0:
            return ConnectedMessage(
                state=self.read_user_event(),
                scale_setting=self.read_scale_setting(),
                timestamp_ms=self.read_varint(),
            )
        elif variant == 1:
            return StateChangeMessage(
                state=self.read_user_event(),
                timestamp_ms=self.read_varint()
            )
        elif variant == 2:
            return WeightBatchMessage(
                readings=self.read_vec(self.read_weight_reading)
            )
        else:
            raise ValueError(f"Unknown WsMessage variant: {variant}")

def parse_message(data: bytes) -> WsMessage:
    """
    Parse an encoded WebSocket message.

    Args:
        data: Binary postcard-encoded message

    Returns:
        Parsed message (Connected, StateChange, or WeightBatch)
    """
    return PostcardDecoder(data).read_ws_message()
