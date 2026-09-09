"""Prepare an AXS1 Flash parameter patch offline. Never connects to a probe.

Uses offsets exported from the matching firmware's DWARF; preserves all other
parameter/calibration bytes. The caller must verify firmware, board and idle
state before programming, then compare the complete Flash and reboot readback.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib

MAGIC = 0x31535841


def encode(name, minimum_deg, maximum_deg, speed_deg_s):
    if name not in ("roll", "pitch"):
        raise ValueError("Axis name must be roll or pitch")
    if (not all(math.isfinite(v) for v in (minimum_deg, maximum_deg, speed_deg_s)) or
            minimum_deg >= maximum_deg or speed_deg_s <= 0):
        raise ValueError("Invalid axis range/speed")
    payload = struct.pack("<II8sfff", MAGIC, 1, name.encode("ascii"),
                          math.radians(minimum_deg), math.radians(maximum_deg), math.radians(speed_deg_s))
    return payload + struct.pack("<I", zlib.crc32(payload))


def decode(record):
    if len(record) != 32:
        raise ValueError("AXS1 must have 32 bytes")
    magic, version, name, low, high, speed, crc = struct.unpack("<II8sfffI", record)
    if (magic != MAGIC or version != 1 or crc != zlib.crc32(record[:28]) or
            name not in (b"roll\0\0\0\0", b"pitch\0\0\0") or
            not all(math.isfinite(v) for v in (low, high, speed)) or low >= high or speed <= 0):
        raise ValueError("Invalid AXS1 record")
    return {"name": name.rstrip(b"\0").decode(), "minimum_deg": math.degrees(low),
            "maximum_deg": math.degrees(high), "maximum_speed_deg_s": math.degrees(speed), "crc32": crc}


def prepare(parameters, offsets, profile):
    """Return patched copy. Offsets refer to the start of the parameter region."""
    record = encode(profile["name"], profile["motion"]["minimum_deg"],
                    profile["motion"]["maximum_deg"], profile["motion"]["cruise_deg_s"])
    changed = bytearray(parameters)
    axis, speed = offsets["axis_profile"], offsets["pos_maxspeed"]
    if (not isinstance(axis, int) or not isinstance(speed, int) or
            axis < 0 or speed < 0 or axis + 32 > len(changed) or speed + 4 > axis):
        raise ValueError("Invalid parameter offsets")
    previous = parameters[axis:axis+32]
    if previous not in (bytes(32), b"\xff" * 32):
        decode(previous)  # Never silently overwrite unexplained/corrupt data.
    changed[axis:axis+32] = record
    changed[speed:speed+4] = record[24:28]
    return bytes(changed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--offsets", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    before = args.parameters.read_bytes()
    offsets = json.loads(args.offsets.read_text())
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    after = prepare(before, offsets, profile)
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out / "parameters.bin").write_bytes(after)
    report = {"saved_to_flash": False, "axis": decode(after[offsets["axis_profile"]:offsets["axis_profile"]+32]),
              "source_sha256": hashlib.sha256(before).hexdigest(),
              "result_sha256": hashlib.sha256(after).hexdigest(), "offsets": offsets,
              "changed_offsets": [i for i, (a, b) in enumerate(zip(before, after)) if a != b]}
    (args.out / "manifest.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["axis"], indent=2))


if __name__ == "__main__":
    main()
