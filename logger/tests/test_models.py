import struct

import pyarrow.parquet as pq
import pytest

from grindy_logger.models import (
    GrindFinishedMessage,
    StateChangeMessage,
    StopReason,
    UserEvent,
    WeightMessage,
    parse_message,
)
from grindy_logger.writer import ArrowWriter


def varint(n: int) -> bytes:
    out = bytearray()
    while True:
        byte = n & 0x7F
        n >>= 7
        if n:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def f32(x: float) -> bytes:
    return struct.pack("<f", x)


def some_f32(x: float) -> bytes:
    return b"\x01" + f32(x)


NONE = b"\x00"


def weight_with_estimate() -> bytes:
    return (
        varint(2) + varint(123456) + f32(650.5) + varint(UserEvent.Grinding)
        + some_f32(10.25) + some_f32(10.0) + b"\x01" + f32(3.0) + f32(2.5) + f32(3.5)
    )


def grind_finished() -> bytes:
    return (
        varint(4) + varint(StopReason.Prediction) + f32(17.75)
        + some_f32(18.0) + some_f32(0.5) + f32(0.48)
    )


def test_weight_reading_with_estimate():
    msg = parse_message(weight_with_estimate())
    assert isinstance(msg, WeightMessage)
    r = msg.reading
    assert r.timestamp_ms == 123456
    assert r.state == UserEvent.Grinding
    assert r.coffee_weight == pytest.approx(10.25)
    assert r.filtered_weight == pytest.approx(10.0)
    assert (r.eta.median, r.eta.lo, r.eta.hi) == pytest.approx((3.0, 2.5, 3.5))


def test_weight_reading_without_estimate():
    data = varint(2) + varint(1) + f32(1.0) + varint(UserEvent.Idle) + NONE + NONE + NONE
    r = parse_message(data).reading
    assert r.coffee_weight is None and r.filtered_weight is None and r.eta is None


def test_state_change_with_lead_time():
    data = (
        varint(1) + varint(UserEvent.Grinding) + f32(1.0) + f32(2.0) + f32(3.0)
        + f32(18.0) + varint(99) + f32(0.45)
    )
    msg = parse_message(data)
    assert isinstance(msg, StateChangeMessage)
    assert msg.timestamp_ms == 99
    assert msg.lead_time == pytest.approx(0.45)


def test_grind_finished():
    msg = parse_message(grind_finished())
    assert isinstance(msg, GrindFinishedMessage)
    assert msg.stop_reason == StopReason.Prediction
    assert msg.stop_weight == pytest.approx(17.75)
    assert msg.settled_weight == pytest.approx(18.0)
    assert msg.lead_time_observed == pytest.approx(0.5)
    assert msg.lead_time == pytest.approx(0.48)


def test_grind_finished_without_settled_weight():
    data = varint(4) + varint(StopReason.Timeout) + f32(12.0) + NONE + NONE + f32(0.5)
    msg = parse_message(data)
    assert msg.stop_reason == StopReason.Timeout
    assert msg.settled_weight is None and msg.lead_time_observed is None


def test_writer_stores_new_columns(tmp_path):
    path = tmp_path / "out.parquet"
    with ArrowWriter(str(path)) as writer:
        writer.add_message(parse_message(weight_with_estimate()))
        writer.add_message(parse_message(grind_finished()))
    rows = pq.read_table(path).to_pylist()
    reading, finished = rows
    assert reading["filtered_weight"] == pytest.approx(10.0)
    assert (reading["eta_median"], reading["eta_lo"], reading["eta_hi"]) == pytest.approx((3.0, 2.5, 3.5))
    assert finished["message_type"] == "GrindFinished"
    assert finished["timestamp_ms"] == 123456
    assert finished["stop_reason"] == "Prediction"
    assert finished["stop_weight"] == pytest.approx(17.75)
    assert finished["settled_weight"] == pytest.approx(18.0)
    assert finished["lead_time_observed"] == pytest.approx(0.5)
    assert finished["lead_time"] == pytest.approx(0.48)
