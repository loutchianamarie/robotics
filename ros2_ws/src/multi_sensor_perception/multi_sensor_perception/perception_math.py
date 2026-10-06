"""Pure helpers used by the ROS fusion node and tested without ROS installed."""

import math
from typing import Any, Iterable


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw in radians from a unit quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def valid_ranges(ranges: Iterable[float], max_detection_m: float) -> list[float]:
    """Keep finite ranges inside the configured detection window."""
    return [d for d in ranges if math.isfinite(d) and 0.0 <= d <= max_detection_m]


def sync_spread_ms(*timestamps_ns: int) -> float:
    """Largest timestamp difference in milliseconds."""
    if not timestamps_ns or any(stamp <= 0 for stamp in timestamps_ns):
        raise ValueError('All sensor timestamps must be positive')
    return (max(timestamps_ns) - min(timestamps_ns)) / 1_000_000.0


def parse_perception(raw: str) -> dict[str, Any] | None:
    """Parse the camera's key-value telemetry without executing its contents."""
    try:
        data = {}
        for part in raw.split(';'):
            if '=' in part:
                key, value = part.split('=', 1)
                data[key.strip()] = value.strip()
        stamp = int(data['stamp_ns'])
        confidence = float(data.get('confidence', '0'))
        if stamp <= 0 or not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            return None
        return {
            'stamp_ns': stamp,
            'detected': data.get('detected', 'false').lower() == 'true',
            'label': data.get('label', 'none'),
            'confidence': confidence,
            'mode': data.get('mode', 'simple'),
        }
    except (KeyError, ValueError):
        return None
