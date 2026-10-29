"""Passive device evidence and explicit role binding; never opens a device.

This module uses only the standard library so it can also run before installing
the runtime or vendor dependencies on a device host.
"""

import argparse
import hashlib
import json
import platform
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol

DISCOVERY_VERSION = "device-discovery/1"
BINDING_VERSION = "device-bindings/1"


class Collector(Protocol):
    name: str

    def collect(self) -> tuple[list[dict], list[dict]]: ...


def candidate(kind, locator, resource_id, identity, state, evidence):
    key = (kind + "\0" + locator).encode("utf-8")
    return {
        "candidate_id": hashlib.sha256(key).hexdigest()[:24],
        "kind": kind,
        "locator": locator,
        "resource_id": resource_id,
        "identity": identity,
        "state": state,
        "evidence": evidence,
        "device_io": "not_opened",
    }


class LinuxSysfsCollector:
    """Enumerate USB, CAN, serial and video metadata without vendor imports.

    Roots are injectable for fixtures/mounted sysfs. Candidate IDs identify an
    observation; persistent role matching uses explicit identity selectors.
    """

    name = "linux-sysfs"

    def __init__(self, sys_root=Path("/sys"), dev_root=Path("/dev"), max_entries=1024):
        self.sys = Path(sys_root)
        self.dev = Path(dev_root)
        if type(max_entries) is not int or max_entries < 1:
            raise ValueError("max_entries must be a positive integer")
        self.max_entries = max_entries
        self.diagnostics = []

    def _error(self, path, status):
        self.diagnostics.append({"collector": self.name, "path": str(path), "status": status})

    def _read(self, path):
        try:
            with path.open("r", encoding="utf-8", errors="replace") as stream:
                value = stream.read(4097)
            if len(value) > 4096:
                self._error(path, "attribute_truncated")
            return value[:4096].strip() or None
        except FileNotFoundError:
            return None
        except OSError as exc:
            self._error(path, type(exc).__name__)
            return None

    def _entries(self, path):
        try:
            entries = sorted(path.iterdir())
            if len(entries) > self.max_entries:
                self._error(path, "enumeration_truncated")
            return entries[: self.max_entries]
        except FileNotFoundError:
            return []
        except OSError as exc:
            self._error(path, type(exc).__name__)
            return []

    def _usb_parent(self, path):
        resolved = path.resolve()
        # Only traverse this collector's sysfs tree, including fixture roots.
        for parent in (resolved, *resolved.parents):
            if not parent.is_relative_to(self.sys.resolve()):
                break
            if (parent / "idVendor").is_file():
                return parent
        return None

    def _usb_identity(self, path):
        if path is None:
            return {}
        serial = self._read(path / "serial")
        return {
            "usb_vendor_id": self._read(path / "idVendor"),
            "usb_product_id": self._read(path / "idProduct"),
            "serial": serial,
            "usb_serial": serial,
            "serial_namespace": "usb_descriptor",
            "product": self._read(path / "product"),
            "manufacturer": self._read(path / "manufacturer"),
            "usb_path": path.name,
        }

    def collect(self):
        self.diagnostics = []
        found = []
        if not self.sys.is_dir():
            self._error(self.sys, "not_available")
            return found, self.diagnostics
        for p in self._entries(self.sys / "bus/usb/devices"):
            if ":" in p.name or p.name.startswith("usb") or not (p / "idVendor").is_file():
                continue
            found.append(
                candidate(
                    "usb_device",
                    str(p),
                    "usb:" + p.name,
                    self._usb_identity(p),
                    {"link_speed_mbps": self._read(p / "speed")},
                    [str(p)],
                )
            )
        for p in self._entries(self.sys / "class/net"):
            if self._read(p / "type") != "280":  # Linux ARPHRD_CAN
                continue
            parent = self._usb_parent(p / "device")
            identity = self._usb_identity(parent)
            identity["interface"] = p.name
            identity["usb_interface"] = (p / "device").resolve().name if parent else None
            # This names a CAN channel, not the robot attached to that channel.
            resource = (
                "can:"
                + (identity.get("usb_path", "") or "")
                + ":"
                + (identity["usb_interface"] or p.name)
            )
            found.append(
                candidate(
                    "can_interface",
                    str(p),
                    resource,
                    identity,
                    {
                        "operstate": self._read(p / "operstate"),
                        "rx_packets": self._read(p / "statistics/rx_packets"),
                        "tx_packets": self._read(p / "statistics/tx_packets"),
                    },
                    [str(p)],
                )
            )
        for folder, kind in [("tty", "serial_port"), ("video4linux", "video_interface")]:
            for p in self._entries(self.sys / "class" / folder):
                if not (p / "device").exists():
                    continue
                parent = self._usb_parent(p / "device")
                identity = self._usb_identity(parent)
                identity["port"] = str(self.dev / p.name)
                if kind == "serial_port":
                    aliases = {}
                    for directory in ("serial/by-id", "serial/by-path"):
                        for alias in self._entries(self.dev / directory):
                            if alias.resolve() == (self.dev / p.name).resolve():
                                aliases.setdefault(directory, str(alias))
                    identity["by_id"] = aliases.get("serial/by-id")
                    identity["by_path"] = aliases.get("serial/by-path")
                    resource = "serial:" + str((p / "device").resolve())
                else:
                    # Multiple /dev/video nodes can belong to one physical camera.
                    identity["name"] = self._read(p / "name")
                    resource = "usb:" + parent.name if parent else "video:" + str(p.resolve())
                found.append(
                    candidate(
                        kind,
                        str(p),
                        resource,
                        identity,
                        {"protocol_status": "not_queried"},
                        [str(p)],
                    )
                )
        return found, self.diagnostics


def discover(collectors=None):
    if collectors is None:
        collectors = [LinuxSysfsCollector()] if platform.system() == "Linux" else []
    devices, diagnostics = [], []
    for collector in collectors:
        try:
            new, notes = collector.collect()
            devices.extend(new)
            diagnostics.extend(notes)
        except OSError as exc:
            diagnostics.append({"collector": collector.name, "status": type(exc).__name__})
    if not collectors:
        diagnostics.append({"collector": "platform", "status": "passive_collector_unavailable"})
    report = {
        "schema_version": DISCOVERY_VERSION,
        "collected_at_utc": datetime.now(timezone.utc).isoformat(),
        "system": platform.system(),
        "candidates": sorted(devices, key=lambda x: x["candidate_id"]),
        "diagnostics": diagnostics,
        "scope": "passive_metadata_only",
    }
    validate_snapshot(report)
    return report


def _keys(value, required, optional=()):
    if (
        not isinstance(value, dict)
        or not set(required) <= value.keys()
        or value.keys() - set(required) - set(optional)
    ):
        raise ValueError("missing or unknown fields; expected " + ", ".join(required))


def validate_snapshot(snapshot):
    _keys(
        snapshot,
        ("schema_version", "collected_at_utc", "system", "candidates", "diagnostics", "scope"),
    )
    if (
        snapshot["schema_version"] != DISCOVERY_VERSION
        or snapshot["scope"] != "passive_metadata_only"
    ):
        raise ValueError("unsupported discovery version or scope")
    if not all(
        isinstance(snapshot[x], str) and snapshot[x] for x in ("collected_at_utc", "system")
    ):
        raise ValueError("discovery provenance must be nonempty strings")
    if not isinstance(snapshot["candidates"], list) or not isinstance(
        snapshot["diagnostics"], list
    ):
        raise ValueError("discovery candidates and diagnostics must be lists")
    ids = set()
    for c in snapshot["candidates"]:
        _keys(
            c,
            (
                "candidate_id",
                "kind",
                "locator",
                "resource_id",
                "identity",
                "state",
                "evidence",
                "device_io",
            ),
        )
        if not all(
            isinstance(c[x], str) and c[x]
            for x in ("candidate_id", "kind", "locator", "resource_id")
        ):
            raise ValueError("candidate identity fields must be nonempty strings")
        if c["candidate_id"] in ids:
            raise ValueError("duplicate candidate_id")
        ids.add(c["candidate_id"])
        if (
            c["device_io"] != "not_opened"
            or not isinstance(c["identity"], dict)
            or not isinstance(c["state"], dict)
        ):
            raise ValueError("candidate must describe unopened metadata")
        if not all(
            isinstance(k, str) and (v is None or isinstance(v, str))
            for k, v in c["identity"].items()
        ):
            raise ValueError("identity values must be strings or null")
        if not isinstance(c["evidence"], list) or not all(
            isinstance(x, str) for x in c["evidence"]
        ):
            raise ValueError("candidate evidence must be a path list")


def resolve_bindings(snapshot, profile):
    validate_snapshot(snapshot)
    _keys(profile, ("schema_version", "roles"))
    if (
        profile["schema_version"] != BINDING_VERSION
        or not isinstance(profile["roles"], list)
        or not profile["roles"]
    ):
        raise ValueError("unsupported or empty device binding profile")
    roles, seen = [], set()
    for spec in profile["roles"]:
        _keys(spec, ("role", "kind", "match", "confirmation"), ("exclusive",))
        if (
            not all(isinstance(spec[x], str) and spec[x] for x in ("role", "kind"))
            or spec["role"] in seen
        ):
            raise ValueError("role names must be nonempty and unique")
        seen.add(spec["role"])
        match = spec["match"]
        if (
            not isinstance(match, dict)
            or not match
            or not all(
                isinstance(k, str) and k and isinstance(v, str) and v.strip()
                for k, v in match.items()
            )
        ):
            raise ValueError("match requires nonempty exact identity selectors")
        if (
            spec["confirmation"] not in ("unconfirmed", "confirmed")
            or type(spec.get("exclusive", True)) is not bool
        ):
            raise ValueError("invalid confirmation or exclusivity")
        matches = [
            c
            for c in snapshot["candidates"]
            if c["kind"] == spec["kind"]
            and all(c["identity"].get(k) == v for k, v in match.items())
        ]
        status = (
            "missing"
            if not matches
            else "ambiguous"
            if len(matches) > 1
            else "resolved"
            if spec["confirmation"] == "confirmed"
            else "matched_unconfirmed"
        )
        roles.append(
            {
                "role": spec["role"],
                "status": status,
                "exclusive": spec.get("exclusive", True),
                "candidates": [
                    {
                        "candidate_id": c["candidate_id"],
                        "locator": c["locator"],
                        "resource_id": c["resource_id"],
                    }
                    for c in matches
                ],
            }
        )
    for i, a in enumerate(roles):
        for b in roles[i + 1 :]:
            if (
                len(a["candidates"]) == len(b["candidates"]) == 1
                and (a["exclusive"] or b["exclusive"])
                and a["candidates"][0]["resource_id"] == b["candidates"][0]["resource_id"]
            ):
                a["status"] = b["status"] = "resource_conflict"
    return {
        "schema_version": BINDING_VERSION,
        "ready": not snapshot["diagnostics"] and all(r["status"] == "resolved" for r in roles),
        "discovery_complete": not snapshot["diagnostics"],
        "meaning": "identity_binding_only_not_control_admission",
        "roles": roles,
    }


def load_json(path):
    def constant(value):
        raise ValueError("non-finite JSON value: " + value)

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result

    return json.loads(
        Path(path).read_text(encoding="utf-8-sig"),
        parse_constant=constant,
        object_pairs_hook=unique,
    )


def add_arguments(parser):
    parser.add_argument(
        "--snapshot", help="resolve an existing discovery report without enumeration"
    )
    parser.add_argument("--bindings", help="explicit identity-to-role profile")
    parser.add_argument("--output", help="new JSON output file; refuses overwrite")


def execute(args):
    report = load_json(args.snapshot) if args.snapshot else discover()
    if isinstance(report, dict) and "discovery" in report:
        _keys(report, ("discovery",), ("bindings",))
        report = report["discovery"]
    validate_snapshot(report)
    result = {"discovery": report}
    if args.bindings:
        result["bindings"] = resolve_bindings(report, load_json(args.bindings))
    # --snapshot also accepts this envelope, so output is directly reusable.
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=True, indent=2, allow_nan=False)
            stream.write("\n")
    return result, 3 if "bindings" in result and not result["bindings"]["ready"] else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_arguments(parser)
    try:
        result, code = execute(parser.parse_args(argv))
        print(json.dumps(result, ensure_ascii=True, allow_nan=False))
        return code
    except (ValueError, OSError) as exc:
        print(json.dumps({"error": str(exc)}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
