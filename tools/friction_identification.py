#!/usr/bin/env python3
"""Run and review the Vector Mini ST no-load friction identification."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


FRICTION_MODE = 19
STATE_COMPLETE = 4
STATE_FAILED = 5
EXPECTED_POINTS = 8

STATE_NAMES = {
    0: "idle",
    1: "tracking",
    2: "sampling",
    3: "stopping",
    4: "complete",
    5: "failed",
}

REASON_NAMES = {
    0: "none",
    1: "cancelled",
    2: "invalid configuration",
    3: "speed tracking timeout",
    4: "sampling timeout",
    5: "stop timeout",
    6: "current saturation",
    7: "invalid sample",
    8: "fit rejected",
    9: "controller safety fault",
}


class ProtocolError(RuntimeError):
    pass


@dataclass
class Sample:
    index: int
    target_rps: float
    speed_rps: float
    iq_a: float
    sample_count: int


@dataclass
class Fit:
    coulomb_a: float
    viscous_a_per_rad_s: float
    rmse_a: float


class VectorUsb:
    def __init__(self, port: str, baudrate: int, timeout: float, verbose: bool):
        self.verbose = verbose
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
        )

    def close(self) -> None:
        self.serial.close()

    def _read_line(self) -> str:
        response = self.serial.read_until(b"\n").decode("ascii", errors="replace").strip()
        if self.verbose and response:
            print(f"< {response}")
        if not response:
            raise ProtocolError("Timed out waiting for the controller")
        return response

    def command(self, command: str) -> str:
        wire = f"\\{command}\r\n"
        self.serial.reset_input_buffer()
        if self.verbose:
            print(f"> {wire!r}")
        self.serial.write(wire.encode("ascii"))
        self.serial.flush()
        response = self._read_line()
        if response in {
            "Syntax error!",
            "Write invalid!",
            "Unknowned parameter!",
            "Data invalid!",
            "Data out of range!",
        }:
            raise ProtocolError(f"Command {command!r} failed: {response}")
        return response

    def read(self, parameter: str) -> str:
        return self.command(f"r_{parameter}")

    def write(self, parameter: str, value: int | float) -> None:
        response = self.command(f"w_{parameter}={value}")
        if response != "Write Success!":
            raise ProtocolError(f"Unexpected write response for {parameter}: {response}")

    def read_export(self) -> list[Sample]:
        begin = self.read("fdt")
        match = re.fullmatch(r"friction_begin,count=(\d+)", begin)
        if not match:
            raise ProtocolError(f"Unexpected friction export header: {begin}")
        count = int(match.group(1))
        samples: list[Sample] = []
        pattern = re.compile(
            r"friction=(\d+),target=(-?\d+(?:\.\d+)?),"
            r"speed=(-?\d+(?:\.\d+)?),iq=(-?\d+(?:\.\d+)?),n=(\d+)"
        )
        while True:
            line = self._read_line()
            if line == "friction_end":
                break
            item = pattern.fullmatch(line)
            if not item:
                raise ProtocolError(f"Malformed friction export line: {line}")
            samples.append(
                Sample(
                    index=int(item.group(1)),
                    target_rps=float(item.group(2)),
                    speed_rps=float(item.group(3)),
                    iq_a=float(item.group(4)),
                    sample_count=int(item.group(5)),
                )
            )
        if len(samples) != count:
            raise ProtocolError(f"Expected {count} samples, received {len(samples)}")
        indices = sorted(sample.index for sample in samples)
        if count != EXPECTED_POINTS or indices != list(range(count)):
            raise ProtocolError(
                "Friction export has missing, duplicate, or unexpected sample indices"
            )
        if any(sample.sample_count <= 0 for sample in samples):
            raise ProtocolError("Friction export contains an empty sample")
        return samples


def extract_number(response: str, field: str) -> float:
    match = re.search(rf"{re.escape(field)}=(-?\d+(?:\.\d+)?)", response)
    if not match:
        raise ProtocolError(f"Cannot parse {field!r} from: {response}")
    return float(match.group(1))


def parse_status(response: str) -> dict[str, float]:
    return {
        field: extract_number(response, field)
        for field in ("friction_state", "reason", "point", "progress", "candidate")
    }


def constrained_fit(samples: list[Sample], positive: bool) -> Fit:
    selected = [sample for sample in samples if (sample.speed_rps > 0.0) == positive]
    if len(selected) < 2:
        raise ProtocolError("Not enough directional samples for regression")
    x = [abs(sample.speed_rps) * 2.0 * math.pi for sample in selected]
    y = [sample.iq_a if positive else -sample.iq_a for sample in selected]
    if any(value <= 0.0 or not math.isfinite(value) for value in x + y):
        raise ProtocolError("The no-load current does not oppose motion in every sample")
    count = float(len(x))
    sum_x, sum_y = sum(x), sum(y)
    sum_xx = sum(value * value for value in x)
    sum_xy = sum(xv * yv for xv, yv in zip(x, y))
    denominator = count * sum_xx - sum_x * sum_x
    if denominator <= 1e-12:
        raise ProtocolError("Speed points are degenerate")
    slope = (count * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_y - slope * sum_x) / count
    if slope < 0.0:
        slope = 0.0
        intercept = sum_y / count
    if intercept < 0.0:
        intercept = 0.0
        slope = sum_xy / sum_xx
    rmse = math.sqrt(
        sum((yv - (intercept + slope * xv)) ** 2 for xv, yv in zip(x, y)) / count
    )
    return Fit(intercept, slope, rmse)


def preflight(device: VectorUsb) -> None:
    mode = int(extract_number(device.read("mod"), "mode"))
    error = int(extract_number(device.read("err"), "error"))
    encoder = device.read("e_s")
    speed_limit = extract_number(device.read("slm"), "spd_lim")
    if mode != 0:
        raise ProtocolError(f"Motor must be disabled; current mode is {mode}")
    if error != 0:
        raise ProtocolError(f"Controller has active error {error}")
    if "Online" not in encoder:
        raise ProtocolError("Encoder is not online")
    if speed_limit < 0.8:
        raise ProtocolError(f"Speed limit {speed_limit:.3f} rev/s is below required 0.8 rev/s")


def candidate_values(device: VectorUsb) -> dict[str, float]:
    fields = {
        "coulomb_pos_a": ("fcp", "friction_coulomb_pos"),
        "coulomb_neg_a": ("fcn", "friction_coulomb_neg"),
        "viscous_pos_a_per_rad_s": ("fvp", "friction_viscous_pos"),
        "viscous_neg_a_per_rad_s": ("fvn", "friction_viscous_neg"),
        "rmse_pos_a": ("frp", "friction_rmse_pos"),
        "rmse_neg_a": ("frn", "friction_rmse_neg"),
    }
    return {
        name: extract_number(device.read(command), response_field)
        for name, (command, response_field) in fields.items()
    }


def run(args: argparse.Namespace) -> int:
    if serial is None:
        raise SystemExit("pyserial is required; install it with: python -m pip install pyserial")
    device: VectorUsb | None = None
    running = False
    try:
        device = VectorUsb(args.port, args.baudrate, args.timeout, args.verbose)
        preflight(device)
        if not args.yes:
            answer = input(
                "Confirm the motor is unloaded and can rotate freely in both directions "
                "at up to 0.8 rev/s. Type RUN to continue: "
            )
            if answer != "RUN":
                print("Cancelled without moving the motor.")
                return 2

        device.write("mod", FRICTION_MODE)
        running = True
        deadline = time.monotonic() + args.max_duration
        last_summary = ""
        while time.monotonic() < deadline:
            status = parse_status(device.read("fst"))
            state = int(status["friction_state"])
            reason = int(status["reason"])
            summary = (
                f"state={STATE_NAMES.get(state, state)}, point="
                f"{min(int(status['point']) + 1, EXPECTED_POINTS)}/"
                f"{EXPECTED_POINTS}, progress={status['progress']:.1f}%"
            )
            if summary != last_summary:
                print(summary)
                last_summary = summary
            if state == STATE_COMPLETE:
                running = False
                break
            if state == STATE_FAILED:
                running = False
                raise ProtocolError(
                    f"Identification failed: {REASON_NAMES.get(reason, f'reason {reason}')}"
                )
            time.sleep(args.poll_interval)
        else:
            raise ProtocolError("Host timeout while waiting for identification")

        samples = device.read_export()
        firmware = candidate_values(device)
        pole_pairs = int(extract_number(device.read("pol"), "pol"))
        flux_wb = extract_number(device.read("mfx"), "Flux") / 1000.0
        torque_constant_nm_per_a = 1.5 * pole_pairs * flux_wb
        positive_fit = constrained_fit(samples, True)
        negative_fit = constrained_fit(samples, False)
        host = {
            "coulomb_pos_a": positive_fit.coulomb_a,
            "coulomb_neg_a": negative_fit.coulomb_a,
            "viscous_pos_a_per_rad_s": positive_fit.viscous_a_per_rad_s,
            "viscous_neg_a_per_rad_s": negative_fit.viscous_a_per_rad_s,
            "rmse_pos_a": positive_fit.rmse_a,
            "rmse_neg_a": negative_fit.rmse_a,
        }
        for name, value in host.items():
            if not math.isclose(value, firmware[name], rel_tol=2e-3, abs_tol=2e-5):
                raise ProtocolError(
                    f"Host/firmware fit mismatch for {name}: {value:.8g} vs {firmware[name]:.8g}"
                )

        print("\nCandidate current-domain model:")
        print(
            f"  positive: Iq = {host['coulomb_pos_a']:.6f} + "
            f"{host['viscous_pos_a_per_rad_s']:.6f} * speed(rad/s) A"
        )
        print(
            f"  negative: Iq = -{host['coulomb_neg_a']:.6f} + "
            f"{host['viscous_neg_a_per_rad_s']:.6f} * speed(rad/s) A"
        )
        print(
            f"  RMSE: +{host['rmse_pos_a']:.6f} A, -{host['rmse_neg_a']:.6f} A"
        )
        print(
            f"  torque estimate (Kt={torque_constant_nm_per_a:.6f} N*m/A): "
            f"Coulomb +{host['coulomb_pos_a'] * torque_constant_nm_per_a:.6f}/"
            f"-{host['coulomb_neg_a'] * torque_constant_nm_per_a:.6f} N*m"
        )

        payload = {
            "created_unix_s": time.time(),
            "firmware_fit": firmware,
            "host_fit": host,
            "torque_conversion": {
                "pole_pairs": pole_pairs,
                "flux_wb": flux_wb,
                "torque_constant_nm_per_a": torque_constant_nm_per_a,
                "coulomb_pos_nm": host["coulomb_pos_a"] * torque_constant_nm_per_a,
                "coulomb_neg_nm": host["coulomb_neg_a"] * torque_constant_nm_per_a,
                "viscous_pos_nm_per_rad_s": host["viscous_pos_a_per_rad_s"]
                * torque_constant_nm_per_a,
                "viscous_neg_nm_per_rad_s": host["viscous_neg_a_per_rad_s"]
                * torque_constant_nm_per_a,
            },
            "samples": [asdict(sample) for sample in samples],
            "applied": False,
            "saved": False,
        }

        if args.apply:
            if not args.yes:
                answer = input("Type APPLY to copy this candidate into controller RAM: ")
                if answer != "APPLY":
                    print("Candidate left unapplied.")
                    args.save = False
                else:
                    device.write("fap", 1)
                    payload["applied"] = True
            else:
                device.write("fap", 1)
                payload["applied"] = True
            if args.save and payload["applied"]:
                device.write("mod", 9)
                save_deadline = time.monotonic() + 3.0
                while time.monotonic() < save_deadline:
                    if int(extract_number(device.read("mod"), "mode")) == 0:
                        break
                    time.sleep(0.1)
                else:
                    raise ProtocolError("Controller did not return to Disable after saving")
                if int(extract_number(device.read("fva"), "friction_model_valid")) != 1:
                    raise ProtocolError("Applied friction model is not valid after saving")
                payload["saved"] = True

        if args.output:
            output_path = Path(args.output).resolve()
            output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            print(f"Result written to {output_path}")
        return 0
    except (KeyboardInterrupt, ProtocolError, serial.SerialException) as exc:
        if running:
            try:
                device.write("mod", 0)
            except Exception:
                pass
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if device is not None:
            device.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the unloaded bidirectional Coulomb-viscous friction identification."
    )
    parser.add_argument("port", help="USB CDC serial port, for example COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.5)
    parser.add_argument("--max-duration", type=float, default=150.0)
    parser.add_argument("--output", default="friction_identification_result.json")
    parser.add_argument("--apply", action="store_true", help="Apply the reviewed candidate to RAM")
    parser.add_argument("--save", action="store_true", help="Save after --apply")
    parser.add_argument("--yes", action="store_true", help="Skip interactive confirmations")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.save and not args.apply:
        parser.error("--save requires --apply")
    if args.poll_interval <= 0.0 or args.max_duration <= 0.0 or args.timeout <= 0.0:
        parser.error("timeout values must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
