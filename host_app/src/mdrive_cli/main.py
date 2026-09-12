"""Motor-drive parameter, status, discovery, and Loader update CLI."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
import math
import os
from pathlib import Path
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mdrive_core.device.session import DeviceSession
from mdrive_core.protocol.loader_client import LoaderClient
from mdrive_core.protocol.param import layout, raw_get
from mdrive_core.schema.flash_params import lut_angle_error_degrees, lut_raw_angle_degrees, lut_statistics, parse_record
from mdrive_core.schema.params import PARAMS, ParamDef
from mdrive_core.services.telemetry import CsvRecorder, collect, configure_stream
from mdrive_core.transport.controlcanfd import DEFAULT_DLL, ControlCanFdConfig, ControlCanFdTransport


class CliUsageError(ValueError):
    """A parameter name or requested operation is invalid."""


class WriteVerificationError(RuntimeError):
    """A write read-back differs from the requested value."""


def _integer(value: str) -> int:
    return int(value, 0)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dll", type=Path, default=Path(os.environ.get("MDRIVE_CAN_DLL", DEFAULT_DLL)))
    parser.add_argument("--node", type=int, default=0)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--json", action="store_true", dest="json_output", help="emit machine-readable output")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("scan", help="find application nodes and protocol revisions")
    commands.add_parser("readall", help="read all schema-readable parameters serially")
    read = commands.add_parser("read", help="read one SET ID/name or read-only GET ID/name")
    read.add_argument("param")
    write = commands.add_parser("write", help="write and verify one writable parameter")
    write.add_argument("param")
    write.add_argument("value", type=float)
    commands.add_parser("save", help="persist parameters to flash (0x68)")
    upgrade = commands.add_parser("upgrade", help="enter Loader and program an application image")
    upgrade.add_argument("--image", type=Path, required=True)
    upgrade.add_argument("--version", type=_integer, required=True)
    commands.add_parser("status", help="show Loader state or report that the application is running")
    commands.add_parser("flashinfo", help="read the flash parameter record header (0x69)")
    flashdump = commands.add_parser("flashdump", help="read and CRC-verify the whole flash parameter record")
    flashdump.add_argument("--out", type=Path, required=True, help="write the raw record blob here")
    lut = commands.add_parser("lut", help="read the encoder LUT; summarize and optionally export the angle-error curve")
    lut.add_argument("--out", type=Path, help="write CSV (index, raw_deg, err_deg)")
    record = commands.add_parser("record", help="record the CAN status stream to CSV")
    record.add_argument("--seconds", type=float, default=5.0)
    record.add_argument("--rate", type=int, default=100, help="stream rate 10..200 Hz")
    record.add_argument("--out", type=Path, required=True)
    return parser


def _parameter(value: str) -> ParamDef:
    try:
        identifier = int(value, 0)
    except ValueError:
        name = value.upper()
        if name.startswith("CAN_"):
            name = name[4:]
        matches = [definition for definition in PARAMS.values() if definition.name == name]
        if not matches:
            raise CliUsageError(f"unknown parameter name: {value}") from None
        return matches[0]
    definition = PARAMS.get(identifier)
    if definition is None:
        raise CliUsageError(f"unknown parameter ID: {identifier:#x}")
    return definition


def _print_json(value) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))


def _run_command(args: argparse.Namespace, transport: ControlCanFdTransport, session: DeviceSession) -> None:
    command = args.command
    if command == "scan":
        nodes = DeviceSession.scan(transport, args.channel)
        if args.json_output:
            _print_json(nodes)
        else:
            print("NODE  PROTOCOL_REVISION")
            for node, revision in nodes.items():
                print(f"{node:>4}  {revision:>17}")
        return
    if command == "readall":
        values = session.read_all()
        _print_json({f"0x{identifier:02X}": value for identifier, value in values.items()})
        return
    if command == "read":
        definition = _parameter(args.param)
        if definition.access == "cmd":
            raise CliUsageError(f"{definition.name} is a command, not a readable parameter")
        value = session.read(definition.id) if definition.access == "rw" else session.read_raw(definition.id)
        if args.json_output:
            _print_json({"id": definition.id, "name": definition.name, "value": value, "unit": definition.unit})
        else:
            suffix = f" {definition.unit}" if definition.unit else ""
            print(f"{definition.name} = {value:g}{suffix}")
        return
    if command == "write":
        definition = _parameter(args.param)
        if definition.access != "rw":
            raise CliUsageError(f"{definition.name} is not writable")
        expected = args.value
        if definition.minimum is not None and expected < definition.minimum:
            raise CliUsageError(f"{definition.name} is below minimum {definition.minimum}")
        if definition.maximum is not None and expected > definition.maximum:
            raise CliUsageError(f"{definition.name} is above maximum {definition.maximum}")
        session.write(definition.id, expected)
        actual = session.read(definition.id)
        wire_format, scale = layout(definition.get_id or definition.id + 1, reply=True)
        tolerance = 1e-5 if wire_format == ">f" else 1 / scale + 1e-6
        if abs(actual - expected) > tolerance:
            raise WriteVerificationError(f"write verification failed: expected {expected}, read {actual}")
        result = {"id": definition.id, "name": definition.name, "written": expected, "readback": actual}
        _print_json(result) if args.json_output else print(f"{definition.name}: wrote {expected:g}, read {actual:g}")
        return
    if command == "save":
        session.save_params()
        _print_json({"ok": True, "status": "saved"}) if args.json_output else print("Parameters saved.")
        return
    if command == "flashinfo":
        info = session.flash_info()
        payload = info.to_dict()
        payload["magic_hex"] = f"0x{info.magic:08X}"
        payload["crc32_hex"] = f"0x{info.crc32:08X}"
        _print_json(payload)
        return
    if command == "flashdump":
        blob = session.flash_read()
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_bytes(blob)
        _print_json({"ok": True, "bytes": len(blob), "out": str(args.out)})
        return
    if command == "lut":
        blob = session.flash_read()
        parsed = parse_record(blob)
        stats = lut_statistics(blob)
        if args.out:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            rows = ["index,raw_deg,err_deg"]
            errors = lut_angle_error_degrees(blob)
            for index, (angle, error) in enumerate(zip(lut_raw_angle_degrees(), errors)):
                rows.append(f"{index},{angle:.6f},{error:.6f}")
            args.out.write_text("\n".join(rows) + "\n", encoding="utf-8")
        stats["encoder_reverse"] = parsed["encoder_reverse"]
        stats["encoder_calib_flag"] = parsed["encoder_calib_flag"]
        stats["out"] = str(args.out) if args.out else None
        _print_json(stats)
        return
    if command == "record":
        if not 10 <= args.rate <= 200:
            raise CliUsageError("--rate must be 10..200 Hz")
        reported = configure_stream(transport, args.channel, args.node, float(args.rate))
        if reported is None:
            raise RuntimeError("status stream does not acknowledge 0x64/0x65")
        args.out.parent.mkdir(parents=True, exist_ok=True)
        try:
            with CsvRecorder(args.out) as recorder:
                frames, elapsed = collect(transport, args.channel, args.node, args.seconds, on_sample=recorder.write)
        finally:
            configure_stream(transport, args.channel, args.node, 0.0)
        summary = {
            "frames": frames,
            "elapsed_s": round(elapsed, 3),
            "actual_hz": round(frames / elapsed, 1) if elapsed > 0 else 0.0,
            "requested_hz": args.rate,
            "out": str(args.out),
        }
        _print_json(summary)
        return
    if command == "upgrade":
        def progress(done: int, total: int) -> None:
            stream = sys.stderr if args.json_output else sys.stdout
            print(f"program {done}/{total} bytes ({done * 100 / total:.1f}%)", file=stream)

        summary = session.upgrade(args.image, args.version, progress)
        _print_json(summary)
        return
    if command == "status":
        try:
            revision = raw_get(transport, args.channel, args.node, 0x67, "f32", 0.08)
        except TimeoutError:
            status = LoaderClient(transport, node=args.node, timeout=0.2, channel=args.channel).get_status()
            _print_json(status) if args.json_output else print(
                f"Loader state={status['state']} next_offset={status['next_offset']} last_error={status['last_error']}"
            )
        else:
            result = {"application": "running", "protocol_revision": int(revision)}
            _print_json(result) if args.json_output else print(f"Application running (protocol revision {int(revision)}).")
        return
    raise CliUsageError(f"unsupported command: {command}")


def main(argv: Sequence[str] | None = None) -> int:  # noqa: BROAD_EXCEPT_OK
    """Parse arguments, own adapter lifetime, and map failures to stable exits."""
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if not 0 <= args.node <= 7:
            raise CliUsageError("--node must be 0..7")
        if not 0 <= args.channel:
            raise CliUsageError("--channel cannot be negative")
        if args.command == "read":
            definition = _parameter(args.param)
            if definition.access == "cmd":
                raise CliUsageError(f"{definition.name} is a command, not a readable parameter")
        if args.command == "write":
            definition = _parameter(args.param)
            if definition.access != "rw":
                raise CliUsageError(f"{definition.name} is not writable")
            if not math.isfinite(args.value):
                raise CliUsageError("write value must be finite")
            if definition.minimum is not None and args.value < definition.minimum:
                raise CliUsageError(f"{definition.name} is below minimum {definition.minimum}")
            if definition.maximum is not None and args.value > definition.maximum:
                raise CliUsageError(f"{definition.name} is above maximum {definition.maximum}")
    except CliUsageError as exc:
        parser.error(str(exc))

    transport = ControlCanFdTransport(ControlCanFdConfig(dll_path=args.dll, channel=args.channel))
    exit_code = 0
    try:
        transport.open()
        session = DeviceSession(transport, node=args.node, channel=args.channel)
        _run_command(args, transport, session)
    except CliUsageError as exc:
        print(f"error: {exc}", file=sys.stderr)
        exit_code = 2
    except Exception as exc:  # noqa: BROAD_EXCEPT_OK
        if args.json_output:
            print(json.dumps({"ok": False, "error": str(exc)}))
        else:
            print(f"error: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        try:
            transport.close()
        except OSError as exc:
            print(f"error closing CAN transport: {exc}", file=sys.stderr)
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
