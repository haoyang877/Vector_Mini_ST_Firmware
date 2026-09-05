#!/usr/bin/env python3
"""USB CDC smoke and low-speed position-motion test for Vector Mini ST."""

from __future__ import annotations

import argparse
import math
import re
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required; install it with: python -m pip install pyserial"
    ) from exc


class ProtocolError(RuntimeError):
    pass


MAX_TEST_CURRENT_A = 2.0
MAX_TEST_SPEED_REV_S = 0.125
MAX_TEST_ACCEL_REV_S2 = 2.0
MAX_TEST_DELTA_REV = 0.1
MAX_TEST_DURATION_S = 30.0
MAX_POLL_INTERVAL_S = 0.5
MAX_SERIAL_TIMEOUT_S = 5.0
USB_COMMAND_SETTLE_S = 0.1


def encode_decimal(value: float) -> str:
    """Format a value for the firmware parser, which does not support exponents."""
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    integer_part, _, decimal_part = text.lstrip("-").partition(".")
    if len(integer_part) > 9 or len(decimal_part) > 9:
        raise ValueError(f"Value cannot be represented safely by the USB parser: {value}")
    if value != 0.0 and float(text) == 0.0:
        raise ValueError(f"Value is below the USB protocol resolution: {value}")
    return text


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

    def command(self, command: str) -> str:
        wire_command = f"\\{command}\r\n"
        self.serial.reset_input_buffer()
        if self.verbose:
            print(f"> {wire_command!r}")
        self.serial.write(wire_command.encode("ascii"))
        self.serial.flush()
        response = self.serial.read_until(b"\n").decode("ascii", errors="replace").strip()
        if self.verbose:
            print(f"< {response}")
        if not response:
            raise ProtocolError(f"No response to {wire_command!r}")
        if response in {
            "Syntax error!",
            "Write invalid!",
            "Unknowned parameter!",
            "Data invalid!",
            "Data out of range!",
        }:
            raise ProtocolError(f"Command {wire_command!r} failed: {response}")
        # Let the CDC IN completion callback release the endpoint before the
        # next request. Back-to-back host requests can otherwise starve the
        # single-response firmware queue on some Windows USB CDC drivers.
        time.sleep(USB_COMMAND_SETTLE_S)
        return response

    def read(self, parameter: str) -> str:
        return self.command(f"r_{parameter}")

    def write(self, parameter: str, value: float | int) -> None:
        if isinstance(value, float):
            if not math.isfinite(value):
                raise ValueError(f"Non-finite value for {parameter}: {value}")
            encoded_value = encode_decimal(value)
        else:
            encoded_value = str(value)
        response = self.command(f"w_{parameter}={encoded_value}")
        if response != "Write Success!":
            raise ProtocolError(f"Unexpected write response for {parameter}: {response}")


def extract_number(response: str, field: str) -> float:
    match = re.search(rf"{re.escape(field)}=(-?\d+(?:\.\d+)?)", response)
    if not match:
        raise ProtocolError(f"Cannot parse {field!r} from response: {response}")
    return float(match.group(1))


def preflight(device: VectorUsb) -> tuple[int, int, float]:
    mode = int(extract_number(device.read("mod"), "mode"))
    error = int(extract_number(device.read("err"), "error"))
    encoder = device.read("e_s")
    position_rad = extract_number(device.read("p2f"), "pos2_filt")

    print(f"Mode: {mode}")
    print(f"Error: {error}")
    print(f"Encoder: {encoder}")
    print(f"Mechanical position: {position_rad:.3f} rad (current firmware p2f unit)")

    if "Online" not in encoder:
        raise ProtocolError("Encoder is not online")
    return mode, error, position_rad


def verify_setting(device: VectorUsb, parameter: str, field: str, expected: float) -> None:
    actual = extract_number(device.read(parameter), field)
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=0.011):
        raise ProtocolError(
            f"Readback mismatch for {parameter}: expected {expected}, received {actual}"
        )


def wait_for_mode(device: VectorUsb, expected: int, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    observed = -1
    while time.monotonic() < deadline:
        observed = int(extract_number(device.read("mod"), "mode"))
        if observed == expected:
            return
        time.sleep(0.05)
    raise ProtocolError(
        f"Mode transition timed out: expected {expected}, observed {observed}"
    )


def run_motion_test(
    device: VectorUsb,
    args: argparse.Namespace,
    original_settings: dict[str, float],
) -> None:
    mode, error, start_position_rad = preflight(device)
    if mode != 0:
        raise ProtocolError(f"Motor must be disabled before the test; current mode is {mode}")
    if error != 0:
        raise ProtocolError(f"Motor has an active error; current error is {error}")

    speed_limit = extract_number(device.read("slm"), "spd_lim")
    if args.max_speed > speed_limit:
        raise ProtocolError(
            f"Test max speed {args.max_speed} rev/s exceeds speed limit {speed_limit} rev/s"
        )

    original_settings.update(
        {
            "ilm": extract_number(device.read("ilm"), "i_lim"),
            "pac": extract_number(device.read("pac"), "pos_acc"),
            "pde": extract_number(device.read("pde"), "pos_dec"),
            "pms": extract_number(device.read("pms"), "pos_maxspd"),
            "p_p": extract_number(device.read("p_p"), "pos_kp"),
            "p_d": extract_number(device.read("p_d"), "pos_kd"),
            "p_i": extract_number(device.read("p_i"), "pos_ki"),
        }
    )

    device.write("ilm", args.current_limit)
    device.write("pac", args.acceleration)
    device.write("pde", args.deceleration)
    device.write("pms", args.max_speed)
    device.write("p_p", args.position_kp)
    device.write("p_d", args.position_kd)
    device.write("p_i", args.position_ki)

    verify_setting(device, "ilm", "i_lim", args.current_limit)
    verify_setting(device, "pac", "pos_acc", args.acceleration)
    verify_setting(device, "pde", "pos_dec", args.deceleration)
    verify_setting(device, "pms", "pos_maxspd", args.max_speed)
    verify_setting(device, "p_p", "pos_kp", args.position_kp)
    verify_setting(device, "p_d", "pos_kd", args.position_kd)
    verify_setting(device, "p_i", "pos_ki", args.position_ki)

    target_rev = start_position_rad / (2.0 * math.pi) + args.delta_rev
    target_rad = target_rev * 2.0 * math.pi
    print(
        f"Moving by {args.delta_rev:.4f} rev: target={target_rev:.4f} rev, "
        f"current_limit={args.current_limit:.3f} A"
    )

    device.write("mod", 0)
    wait_for_mode(device, 0)
    device.write("mod", 18)
    # Mode requests are consumed asynchronously by the 1 kHz supervisor. Wait
    # before writing p_s, otherwise p_s can observe standby and request mode 3.
    wait_for_mode(device, 18)
    device.write("p_s", target_rev)
    mode = int(extract_number(device.read("mod"), "mode"))
    if mode != 18:
        raise ProtocolError(f"Expected Position_Impedance_Mode (18), received mode {mode}")

    deadline = time.monotonic() + args.duration
    final_position_rad = start_position_rad
    while time.monotonic() < deadline:
        mode = int(extract_number(device.read("mod"), "mode"))
        error = int(extract_number(device.read("err"), "error"))
        if error != 0:
            raise ProtocolError(f"Motor reported error {error} during motion")
        if mode != 18:
            raise ProtocolError(f"Motor left Position_Impedance_Mode during motion; mode={mode}")
        if args.live_position:
            final_position_rad = extract_number(device.read("p2f"), "pos2_filt")
            error_rad = target_rad - final_position_rad
            print(
                f"position={final_position_rad:+.4f} rad, "
                f"target={target_rad:+.4f} rad, error={error_rad:+.4f} rad"
            )
            if abs(error_rad) <= args.tolerance_rad:
                print("Position target reached within tolerance")
                return
        else:
            print(f"motion active: mode={mode}, error={error}")
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(min(args.poll_interval, remaining))

    if not args.live_position:
        # The firmware has one USB CDC response buffer. On some Windows hosts,
        # repeated position reads while the loaded motor is moving can time out
        # even though the control loop remains healthy. Stop first, then verify
        # the final encoder position using a single guarded read.
        device.write("mod", 0)
        wait_for_mode(device, 0)
        final_position_rad = extract_number(device.read("p2f"), "pos2_filt")
        error_rad = target_rad - final_position_rad
        print(
            f"final position={final_position_rad:+.4f} rad, "
            f"target={target_rad:+.4f} rad, error={error_rad:+.4f} rad"
        )
        if abs(error_rad) <= args.tolerance_rad:
            print("Position target reached within tolerance")
            return

    raise ProtocolError(
        f"Position did not reach target: final error={target_rad - final_position_rad:+.4f} rad"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only USB protocol smoke test by default. Add --run-motion to "
            "perform a guarded low-speed relative position move."
        )
    )
    parser.add_argument("port", help="USB CDC serial port, for example COM7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--run-motion", action="store_true")
    parser.add_argument("--delta-rev", type=float, default=0.02)
    parser.add_argument("--current-limit", type=float, default=0.5)
    parser.add_argument("--max-speed", type=float, default=0.1)
    parser.add_argument("--acceleration", type=float, default=0.125)
    parser.add_argument("--deceleration", type=float, default=0.125)
    parser.add_argument("--position-kp", type=float, default=8.0)
    parser.add_argument("--position-kd", type=float, default=0.5)
    parser.add_argument("--position-ki", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--poll-interval", type=float, default=0.1)
    parser.add_argument("--tolerance-rad", type=float, default=0.02)
    parser.add_argument(
        "--live-position",
        action="store_true",
        help="poll position while moving instead of verifying it after Motor_Disable",
    )
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    positive_values = {
        "timeout": args.timeout,
        "current-limit": args.current_limit,
        "max-speed": args.max_speed,
        "acceleration": args.acceleration,
        "deceleration": args.deceleration,
        "duration": args.duration,
        "poll-interval": args.poll_interval,
        "tolerance-rad": args.tolerance_rad,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name} must be finite and greater than zero")
    if args.timeout > MAX_SERIAL_TIMEOUT_S:
        raise ValueError(f"--timeout must not exceed {MAX_SERIAL_TIMEOUT_S} s")
    if args.duration > MAX_TEST_DURATION_S:
        raise ValueError(f"--duration must not exceed {MAX_TEST_DURATION_S} s")
    if args.poll_interval > MAX_POLL_INTERVAL_S:
        raise ValueError(f"--poll-interval must not exceed {MAX_POLL_INTERVAL_S} s")
    if args.poll_interval > args.duration:
        raise ValueError("--poll-interval must not exceed --duration")
    if not math.isfinite(args.delta_rev) or args.delta_rev == 0.0:
        raise ValueError("--delta-rev must be finite and non-zero")
    impedance_gains = {
        "position-kp": (args.position_kp, 50.0),
        "position-kd": (args.position_kd, 10.0),
        "position-ki": (args.position_ki, 10.0),
    }
    for name, (value, maximum) in impedance_gains.items():
        if not math.isfinite(value) or value < 0.0 or value > maximum:
            raise ValueError(f"--{name} must be in the firmware range [0, {maximum}]")
    if args.position_kp == 0.0:
        raise ValueError("--position-kp must be greater than zero for a motion test")
    if args.current_limit > MAX_TEST_CURRENT_A:
        raise ValueError(f"--current-limit must not exceed {MAX_TEST_CURRENT_A} A")
    if args.max_speed > MAX_TEST_SPEED_REV_S:
        raise ValueError(f"--max-speed must not exceed {MAX_TEST_SPEED_REV_S} rev/s")
    if args.acceleration > MAX_TEST_ACCEL_REV_S2:
        raise ValueError(
            f"--acceleration must not exceed {MAX_TEST_ACCEL_REV_S2} rev/s^2"
        )
    if args.deceleration > MAX_TEST_ACCEL_REV_S2:
        raise ValueError(
            f"--deceleration must not exceed {MAX_TEST_ACCEL_REV_S2} rev/s^2"
        )
    if abs(args.delta_rev) > MAX_TEST_DELTA_REV:
        raise ValueError(f"absolute --delta-rev must not exceed {MAX_TEST_DELTA_REV} rev")
    if args.current_limit < 0.01:
        raise ValueError("--current-limit must be at least 0.01 A")
    if args.max_speed < 0.01:
        raise ValueError("--max-speed must be at least 0.01 rev/s")
    if args.acceleration < 0.01 or args.deceleration < 0.01:
        raise ValueError("--acceleration and --deceleration must be at least 0.01 rev/s^2")
    move_rad = abs(args.delta_rev) * 2.0 * math.pi
    if args.tolerance_rad >= move_rad * 0.5:
        raise ValueError("--tolerance-rad must be less than half of the commanded move")


def main() -> int:
    args = parse_args()
    device: VectorUsb | None = None
    motion_requested = args.run_motion
    exit_code = 0
    original_settings: dict[str, float] = {}
    try:
        validate_args(args)
        device = VectorUsb(args.port, args.baudrate, args.timeout, args.verbose)
        time.sleep(0.2)
        if motion_requested:
            run_motion_test(device, args, original_settings)
        else:
            preflight(device)
            print("Read-only USB protocol smoke test passed")
            print("Re-run with --run-motion only after securing the motor")
    except (OSError, ValueError, ProtocolError, serial.SerialException) as exc:
        print(f"TEST FAILED: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if device is not None:
            if motion_requested:
                motor_disabled = False
                try:
                    device.write("mod", 0)
                    wait_for_mode(device, 0)
                except (OSError, ProtocolError, serial.SerialException) as exc:
                    print(f"WARNING: Disable command failed: {exc}", file=sys.stderr)
                try:
                    mode = int(extract_number(device.read("mod"), "mode"))
                    if mode != 0:
                        raise ProtocolError(f"Motor is not disabled; mode={mode}")
                    print("Motor disabled")
                    motor_disabled = True
                    error = int(extract_number(device.read("err"), "error"))
                    if error != 0:
                        raise ProtocolError(f"Motor stopped with error={error}")
                except (OSError, ProtocolError, serial.SerialException) as exc:
                    print(f"TEST FAILED: cannot confirm Motor_Disable: {exc}", file=sys.stderr)
                    exit_code = 1
                if motor_disabled and original_settings:
                    try:
                        readback_fields = {
                            "ilm": "i_lim",
                            "pac": "pos_acc",
                            "pde": "pos_dec",
                            "pms": "pos_maxspd",
                            "p_p": "pos_kp",
                            "p_d": "pos_kd",
                            "p_i": "pos_ki",
                        }
                        for parameter, value in original_settings.items():
                            device.write(parameter, value)
                            verify_setting(
                                device,
                                parameter,
                                readback_fields[parameter],
                                value,
                            )
                        print("Control settings restored from USB readback values")
                    except (OSError, ProtocolError, serial.SerialException) as exc:
                        print(f"TEST FAILED: cannot restore settings: {exc}", file=sys.stderr)
                        exit_code = 1
            try:
                device.close()
            except (OSError, serial.SerialException) as exc:
                print(f"TEST FAILED: cannot close serial port: {exc}", file=sys.stderr)
                exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
