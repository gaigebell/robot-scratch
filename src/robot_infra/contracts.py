"""Explicit M1 joint-position messages, with no SDK or runtime dependencies."""

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from typing import Any

from . import CONTRACT_VERSION


def wire(value: Any) -> Any:
    """Represent rejected IEEE values as tagged objects, never invalid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"non_finite": "nan" if math.isnan(value) else "inf" if value > 0 else "-inf"}
    if hasattr(value, "__dataclass_fields__"):
        return wire(asdict(value))
    if isinstance(value, dict):
        return {k: wire(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [wire(v) for v in value]
    return value


def canonical(value: Any) -> bytes:
    return json.dumps(wire(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def digest(value: Any) -> str:
    return hashlib.sha256(canonical(value)).hexdigest()


@dataclass(frozen=True)
class Lease:
    token: str
    session_id: str
    source: str
    epoch: int
    expires_ns: int


@dataclass(frozen=True)
class JointIntent:
    session_id: str
    source: str
    seq: int
    epoch: int
    lease_token: str | None
    positions: tuple[float, ...]
    joint_names: tuple[str, ...]
    unit: str
    frame_id: str
    clock_domain: str
    created_ns: int
    target_ns: int
    expires_ns: int
    request_id: str | None = None
    chunk_id: str | None = None
    action_index: int | None = None
    schema_version: str = CONTRACT_VERSION


@dataclass(frozen=True)
class PolicyRequest:
    request_id: str
    snapshot_id: str
    lease: Lease
    created_ns: int
    ready_ns: int
    deadline_ns: int
    positions: tuple[tuple[float, ...], ...]
    dt_ns: int
