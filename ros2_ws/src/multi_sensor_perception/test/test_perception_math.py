"""Functional tests for fusion calculations independent of ROS middleware."""

import math

import pytest

from multi_sensor_perception.perception_math import (
    parse_perception,
    quaternion_yaw,
    sync_spread_ms,
    valid_ranges,
)


def test_range_filter_rejects_nan_infinity_and_out_of_window():
    assert valid_ranges([math.nan, math.inf, -1, 0.3, 7.0, 7.1], 7.0) == [0.3, 7.0]


def test_quaternion_yaw_quarter_turn():
    angle = quaternion_yaw(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    assert angle == pytest.approx(math.pi / 2)


def test_sync_spread_and_invalid_stamp():
    assert sync_spread_ms(1_000_000_000, 1_120_000_000, 1_090_000_000) == 120.0
    with pytest.raises(ValueError):
        sync_spread_ms(0, 100)


def test_camera_telemetry_validation():
    data = parse_perception('stamp_ns=100; detected=true; label=person; confidence=0.8; mode=ai')
    assert data == {
        'stamp_ns': 100,
        'detected': True,
        'label': 'person',
        'confidence': 0.8,
        'mode': 'ai',
    }
    assert parse_perception('stamp_ns=0; confidence=0.8') is None
    assert parse_perception('stamp_ns=10; confidence=nan') is None
