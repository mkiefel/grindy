"""Data models matching Rust structs for WebSocket message deserialization."""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional, TypeVar, Union

T = TypeVar('T')


class UserEvent(IntEnum):
    """Grinder state events matching Rust UserEvent enum."""
    Initializing = 0
    Idle = 1
    Stabilizing = 2
    Grinding = 3
    WaitingForRemoval = 4
    WaitingForCalibration = 5
    Calibrating = 6


class StopReason(IntEnum):
    """Why the grinder stopped, matching the Rust StopReason enum."""
    Prediction = 0
    RawWeight = 1
    Timeout = 2


@dataclass
class Eta:
    """Seconds until the grinder is expected to stop (50/10/90 % quantiles)."""
    median: float
    lo: float
    hi: float


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
    filtered_weight: Optional[float]
    eta: Optional[Eta]


@dataclass
class ConnectedMessage:
    """Initial connection message with current state."""
    state: UserEvent
    scale_setting: ScaleSetting
    target_weight: float
    timestamp_ms: int
    lead_time: float


@dataclass
class StateChangeMessage:
    """State transition notification."""
    state: UserEvent
    scale_setting: ScaleSetting
    target_weight: float
    timestamp_ms: int
    lead_time: float


@dataclass
class WeightMessage:
    """A single weight reading, sent for every scale sample."""
    reading: WeightReading


@dataclass
class TargetWeightChangedMessage:
    """Target coffee weight was changed via the web page."""
    target_weight: float


@dataclass
class GrindFinishedMessage:
    """Summary of a finished grind, sent once the weight settled."""
    stop_reason: StopReason
    stop_weight: float
    settled_weight: Optional[float]
    lead_time_observed: Optional[float]
    lead_time: float


# Union type for all possible WebSocket messages
WsMessage = Union[
    ConnectedMessage, StateChangeMessage, WeightMessage, TargetWeightChangedMessage, GrindFinishedMessage
]

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

    def read_eta(self) -> Eta:
        """Decode EtaReading struct."""
        return Eta(median=self.read_f32(), lo=self.read_f32(), hi=self.read_f32())

    def read_weight_reading(self) -> WeightReading:
        """Decode WeightReading struct."""
        return WeightReading(
            timestamp_ms=self.read_varint(),
            weight=self.read_f32(),
            state=self.read_user_event(),
            coffee_weight=self.read_option(self.read_f32),
            filtered_weight=self.read_option(self.read_f32),
            eta=self.read_option(self.read_eta)
        )

    def read_ws_message(self) -> WsMessage:
        """Decode WsMessage enum."""
        variant = self.read_varint()

        if variant == 0:
            return ConnectedMessage(
                state=self.read_user_event(),
                scale_setting=self.read_scale_setting(),
                target_weight=self.read_f32(),
                timestamp_ms=self.read_varint(),
                lead_time=self.read_f32(),
            )
        elif variant == 1:
            return StateChangeMessage(
                state=self.read_user_event(),
                scale_setting=self.read_scale_setting(),
                target_weight=self.read_f32(),
                timestamp_ms=self.read_varint(),
                lead_time=self.read_f32(),
            )
        elif variant == 2:
            return WeightMessage(reading=self.read_weight_reading())
        elif variant == 3:
            return TargetWeightChangedMessage(target_weight=self.read_f32())
        elif variant == 4:
            return GrindFinishedMessage(
                stop_reason=StopReason(self.read_varint()),
                stop_weight=self.read_f32(),
                settled_weight=self.read_option(self.read_f32),
                lead_time_observed=self.read_option(self.read_f32),
                lead_time=self.read_f32(),
            )
        else:
            raise ValueError(f"Unknown WsMessage variant: {variant}")

def parse_message(data: bytes) -> WsMessage:
    """
    Parse an encoded WebSocket message.

    Args:
        data: Binary postcard-encoded message

    Returns:
        Parsed message (Connected, StateChange, Weight, TargetWeightChanged, or GrindFinished)
    """
    return PostcardDecoder(data).read_ws_message()
