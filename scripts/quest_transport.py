"""Shared wire protocol for the Step 3 diagnostic Quest -> lab link.

DIAGNOSTIC ONLY. Nothing in this module (or the sender/receiver built on it)
talks to a robot or gripper, and there is deliberately no option to do so.

Wire format: newline-delimited JSON (one UTF-8 JSON object per line, max
MAX_MESSAGE_BYTES including the newline). Non-finite numbers (NaN, Infinity,
overflowing literals such as 1e999) are rejected on both encode and decode.
Nothing received is ever evaluated or unpickled.

Every packet carries ``v`` (PROTOCOL_VERSION) and ``type``. See
docs/step03_transport.md for the full field list and semantics.

Timestamps named ``*_mono_ns`` are time.monotonic_ns() on the host that wrote
them. They are only ever compared with timestamps from the SAME host.

Stdlib only; must stay compatible with the lab computer's Python.
"""

from __future__ import annotations

import json
import math
import secrets
import statistics
from dataclasses import dataclass
from typing import Any

from quest_input_monitor import CONTROLLER_KEYS, PoseCheck, check_pose

PROTOCOL_VERSION = 1
DEFAULT_PORT = 8765  # receiver port on the lab computer (loopback only)
LOOPBACK = "127.0.0.1"
MAX_MESSAGE_BYTES = 16 * 1024  # a full sample is ~2 KB
FRAME = "headset_relative_observed"
TRACKING_VALIDITY = "unknown"
# Input is PRESENT only when all of these controller poses are present.
REQUIRED_CONTROLLERS: tuple[str, ...] = ("r",)
DIAGNOSTIC_BANNER = "DIAGNOSTIC ONLY: ROBOT NOT CONNECTED"

PACKET_TYPES = frozenset({"hello", "hello_ack", "sample", "invalidate", "heartbeat", "bye", "ack", "error"})
INPUT_PRESENT = "PRESENT"
INPUT_MISSING = "MISSING"
INPUT_STALE = "STALE"
INPUT_SOURCE_DISCONNECTED = "SOURCE_DISCONNECTED"

_MAX_BUTTONS = 64
_MAX_ANALOG_LEN = 4
_MAX_KEY_LEN = 32
_MAX_TEXT_LEN = 200


class ProtocolError(ValueError):
    """A received frame or packet that violates the protocol."""


class FrameTooLarge(ProtocolError):
    """A frame exceeding MAX_MESSAGE_BYTES (the connection must be dropped)."""


def new_id() -> str:
    return secrets.token_hex(8)


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------


class LineFramer:
    """Reassembles newline-delimited frames from arbitrary TCP read chunks.

    Handles partial reads and several frames per read. The internal buffer
    never exceeds max_bytes: an over-long frame raises FrameTooLarge.
    """

    def __init__(self, max_bytes: int = MAX_MESSAGE_BYTES):
        self.max_bytes = max_bytes
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buf += data
        frames: list[bytes] = []
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                break
            if i + 1 > self.max_bytes:
                self._buf.clear()
                raise FrameTooLarge(f"frame of {i + 1} bytes exceeds {self.max_bytes}")
            frame = bytes(self._buf[:i])
            del self._buf[: i + 1]
            if frame.strip():
                frames.append(frame)
        if len(self._buf) >= self.max_bytes:
            n = len(self._buf)
            self._buf.clear()
            raise FrameTooLarge(f"unterminated frame of {n}+ bytes exceeds {self.max_bytes}")
        return frames


def encode(packet: dict[str, Any], max_bytes: int = MAX_MESSAGE_BYTES) -> bytes:
    try:
        data = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
    except ValueError as e:  # non-finite float
        raise ProtocolError(f"cannot encode: {e}") from None
    if len(data) > max_bytes:
        raise FrameTooLarge(f"encoded packet of {len(data)} bytes exceeds {max_bytes}")
    return data


def _reject_constant(name: str) -> Any:
    raise ProtocolError(f"non-finite constant {name}")


def _finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise ProtocolError(f"non-finite number {text[:20]}")
    return value


def decode(frame: bytes) -> dict[str, Any]:
    """Strictly decode one frame into a packet dict with a known version/type."""
    try:
        obj = json.loads(frame.decode("utf-8"), parse_constant=_reject_constant, parse_float=_finite_float)
    except ProtocolError:
        raise
    except (UnicodeDecodeError, ValueError) as e:
        raise ProtocolError(f"invalid JSON: {str(e)[:80]}") from None
    if not isinstance(obj, dict):
        raise ProtocolError("packet is not a JSON object")
    if obj.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {obj.get('v')!r}")
    if obj.get("type") not in PACKET_TYPES:
        raise ProtocolError(f"unknown packet type {str(obj.get('type'))[:20]!r}")
    return obj


# --------------------------------------------------------------------------
# Field validation
# --------------------------------------------------------------------------


def _is_number(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def require_int(pkt: dict[str, Any], key: str, minimum: int = 0) -> int:
    v = pkt.get(key)
    if not isinstance(v, int) or isinstance(v, bool) or v < minimum:
        raise ProtocolError(f"{key} must be an integer >= {minimum}")
    return v


def require_str(pkt: dict[str, Any], key: str, max_len: int = _MAX_TEXT_LEN) -> str:
    v = pkt.get(key)
    if not isinstance(v, str) or not v or len(v) > max_len:
        raise ProtocolError(f"{key} must be a non-empty string <= {max_len} chars")
    return v


def validate_matrix(m: Any, key: str) -> list[list[float]]:
    if not isinstance(m, list) or len(m) != 4:
        raise ProtocolError(f"pose {key!r} must be a 4x4 list")
    rows: list[list[float]] = []
    for row in m:
        if not isinstance(row, list) or len(row) != 4 or not all(_is_number(x) for x in row):
            raise ProtocolError(f"pose {key!r} must be a 4x4 list of finite numbers")
        rows.append([float(x) for x in row])
    return rows


def validate_buttons(b: Any) -> dict[str, bool | tuple[float, ...]]:
    """Booleans stay booleans, analog values stay float tuples; absent stays absent."""
    if not isinstance(b, dict) or len(b) > _MAX_BUTTONS:
        raise ProtocolError("buttons must be an object with <= 64 fields")
    out: dict[str, bool | tuple[float, ...]] = {}
    for k, v in b.items():
        if not isinstance(k, str) or not k or len(k) > _MAX_KEY_LEN:
            raise ProtocolError("bad button key")
        if isinstance(v, bool):
            out[k] = v
        elif isinstance(v, list) and 1 <= len(v) <= _MAX_ANALOG_LEN and all(_is_number(x) for x in v):
            out[k] = tuple(float(x) for x in v)
        else:
            raise ProtocolError(f"button {k[:16]!r} must be a boolean or 1-4 finite numbers")
    return out


def compute_input_state(poses: dict[str, Any]) -> str:
    """PRESENT iff every required controller pose is present (structurally).

    PRESENT does not mean physical tracking is verified (tracking validity is
    unknown) and does not authorise any robot action.
    """
    return INPUT_PRESENT if all(poses.get(k) is not None for k in REQUIRED_CONTROLLERS) else INPUT_MISSING


def missing_reason(poses: dict[str, Any], buttons: dict[str, Any]) -> str | None:
    if compute_input_state(poses) == INPUT_PRESENT:
        return None
    if all(poses.get(k) is None for k in CONTROLLER_KEYS) and not buttons:
        return "empty_message"
    return "required_pose_missing:" + ",".join(k for k in REQUIRED_CONTROLLERS if poses.get(k) is None)


@dataclass
class SampleView:
    """A validated sample packet, fields exactly as sent (missing stays None)."""

    source_session: str
    source_seq: int
    poses: dict[str, list[list[float]] | None]
    checks: dict[str, PoseCheck | None]
    buttons: dict[str, bool | tuple[float, ...]]
    input_state: str
    source_up: bool

    def translation(self, key: str) -> tuple[float, float, float] | None:
        m = self.poses.get(key)
        return None if m is None else (m[0][3], m[1][3], m[2][3])


def validate_sample(pkt: dict[str, Any]) -> SampleView:
    source_session = require_str(pkt, "source_session", 64)
    source_seq = require_int(pkt, "source_seq", 1)
    require_int(pkt, "source_recv_mono_ns")
    require_int(pkt, "send_mono_ns")
    if pkt.get("frame") != FRAME:
        raise ProtocolError(f"frame must be {FRAME!r}")
    if pkt.get("tracking_validity") != TRACKING_VALIDITY:
        raise ProtocolError(f"tracking_validity must be {TRACKING_VALIDITY!r}")
    raw_poses = pkt.get("poses")
    if not isinstance(raw_poses, dict) or set(raw_poses) != set(CONTROLLER_KEYS):
        raise ProtocolError("poses must be an object with exactly keys r and l (null when missing)")
    poses: dict[str, list[list[float]] | None] = {}
    checks: dict[str, PoseCheck | None] = {}
    for k in CONTROLLER_KEYS:
        m = raw_poses[k]
        poses[k] = None if m is None else validate_matrix(m, k)
        checks[k] = None if poses[k] is None else check_pose(poses[k])  # type: ignore[arg-type]
    present = pkt.get("pose_present")
    if present != {k: poses[k] is not None for k in CONTROLLER_KEYS}:
        raise ProtocolError("pose_present disagrees with poses")
    buttons = validate_buttons(pkt.get("buttons"))
    state = compute_input_state(poses)
    if pkt.get("input_state") != state:
        raise ProtocolError(f"input_state {str(pkt.get('input_state'))[:20]!r} disagrees with poses ({state})")
    source = pkt.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("up"), bool):
        raise ProtocolError("source.up must be a boolean")
    return SampleView(source_session, source_seq, poses, checks, buttons, state, source["up"])


# --------------------------------------------------------------------------
# Stats
# --------------------------------------------------------------------------


def stats_ms(values_ns: list[int]) -> dict[str, Any]:
    """count / median / p95 (nearest rank) / max, in milliseconds."""
    if not values_ns:
        return {"count": 0}
    ms = sorted(v / 1e6 for v in values_ns)
    p95 = ms[min(len(ms) - 1, max(0, math.ceil(0.95 * len(ms)) - 1))]
    return {
        "count": len(ms),
        "median_ms": round(statistics.median(ms), 3),
        "p95_ms": round(p95, 3),
        "max_ms": round(ms[-1], 3),
    }
