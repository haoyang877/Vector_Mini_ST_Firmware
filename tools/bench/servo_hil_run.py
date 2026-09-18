"""J-Link 台架运行器：校验固件邮箱、读取实时 RTT，退出时始终停机。

本工具会操作硬件：仅在明确授权的台架流程中运行，不参与默认验证。
需要专用 Vector_Mini_ST_HIL 镜像与匹配的 symbols.json；运行期间只使用命令
邮箱，不直接写电机/PI 状态；STOP 失败时先尝试硬件断电再停机诊断。
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
import argparse
import ctypes
import hashlib
import json
import math
import os
import struct
import time
from pathlib import Path

from motor_axis_record import decode as decode_axis_record
from project_paths import ROOT
from servo_hil_emergency import disable_outputs

OUT = ROOT / "outputs/servo_hil_20260908"
try:
    import pylink
except ImportError:
    pylink = None  # Offline tests can import the module without the bench dependency.


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def wait_for_rtt(probe, timeout_s=2.0):
    """Wait for J-Link's asynchronous RTT discovery while the drive is disabled."""
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            if probe.rtt_get_num_up_buffers() >= 2:
                return
        except Exception as exc:
            last_error = exc
        time.sleep(0.05)
    raise RuntimeError(f"RTT discovery timed out before ARM: {last_error}")


def read_phase_guard(probe, symbols):
    """Read the board-owned burst contract and measured per-ARM exposure."""
    address = symbols.get("servo_hil_current_guard")
    if address is None:
        return None
    x = struct.unpack("<IffIIfIIIfff", bytes(probe.memory_read8(address, 48)))
    return dict(
        magic=x[0],
        maximum_phase_A=x[1],
        exposure_threshold_A=x[2],
        exposure_limit_us=x[3],
        exposure_ticks=x[4],
        observed_peak_A=x[5],
        trip=x[6],
        frequency_hz=x[7],
        sample_count=x[8],
        trip_ia=x[9],
        trip_ib=x[10],
        trip_ic=x[11],
    )


def main():
    if not __debug__:
        raise RuntimeError("Bench safety checks require Python without -O/-OO")
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", required=True)
    p.add_argument("--bench-id", default=os.environ.get("VECTOR_BENCH_ID"))
    p.add_argument("--scenario", required=True, choices=("motion", "hold", "timing"))
    p.add_argument(
        "--operator-confirmation",
        required=True,
        choices=("POWER_LIMITS_VERIFIED",),
        help="Confirms physical isolation, limits, and emergency-stop readiness",
    )
    p.add_argument(
        "--expected-firmware-sha256", required=True, help="Expected SHA-256 of the active HIL AXF"
    )
    p.add_argument(
        "--jlink-dll",
        type=Path,
        default=Path(os.environ["JLINK_DLL"]) if os.environ.get("JLINK_DLL") else None,
    )
    p.add_argument(
        "--probe-serial",
        type=int,
        default=int(os.environ["JLINK_PROBE_SERIAL"])
        if os.environ.get("JLINK_PROBE_SERIAL")
        else None,
    )
    p.add_argument("--kp", type=float, default=8.0)
    p.add_argument("--kd", type=float, default=2.0)
    p.add_argument("--speed-kp", type=float, default=0.5)
    p.add_argument("--speed-ki", type=float, default=1.0)
    p.add_argument("--max-speed-deg", type=float, default=45)
    p.add_argument("--targets", default="0,5,0,-5,0")
    p.add_argument("--seconds", type=float, default=6)
    p.add_argument(
        "--axis-profile",
        type=Path,
        required=True,
        help="Verify persisted roll/pitch record and enforce its narrower host travel limits",
    )
    p.add_argument(
        "--session-dir",
        type=Path,
        required=True,
        help="Separate image/symbol/parameter metadata and captures for this motor",
    )
    p.add_argument(
        "--keep-parameters",
        action="store_true",
        help="Verify expected RAM gains and run without writing tuning parameters",
    )
    p.add_argument(
        "--phase-burst",
        action="store_true",
        help="Allow up to 6 A Iq only with verified board 6 A phase / 30 s exposure guard",
    )
    p.add_argument(
        "--arm-only",
        action="store_true",
        help="Hold the current position for diagnostics; send no POSITION commands",
    )
    p.add_argument(
        "--hold-filter-sequence",
        help="HIL opcode 9: 0=bypass, 1=default cutoff, 2=half cutoff; --seconds per state, requires --arm-only",
    )
    p.add_argument(
        "--hold-filter-setting",
        type=int,
        choices=(0, 1, 2),
        help="Select one HOLD filter for the whole motion trial, using the verified command before ARM",
    )
    p.add_argument(
        "--keep-final-hold-filter",
        action="store_true",
        help="Retain the final RAM filter selection after normal STOP; motor outputs still disabled",
    )
    p.add_argument(
        "--base-filter-setting",
        type=int,
        choices=(0, 1),
        help="Before ARM: 0=default base velocity cutoff, 1=half; HOLD extra filter is separate",
    )
    p.add_argument(
        "--keep-final-base-filter",
        action="store_true",
        help="Retain base-filter RAM selection after successful STOP; failure restores default",
    )
    p.add_argument(
        "--profile-timing",
        action="store_true",
        help="Verify the archived image and measure cumulative IRQ cycles while running",
    )
    p.add_argument(
        "--profile-stage",
        type=int,
        choices=range(1, 15),
        help="Select one stage in the optional diagnostic image; requires --profile-timing",
    )
    p.add_argument(
        "--swd-speed-khz",
        type=int,
        choices=(1000, 2000, 4000),
        default=4000,
        help="J-Link debug clock only; does not change firmware or encoder SPI clocks",
    )
    args = p.parse_args()
    if not args.bench_id:
        p.error("--bench-id or VECTOR_BENCH_ID is required; no probe was opened")
    if not args.jlink_dll or not args.jlink_dll.is_file():
        p.error("--jlink-dll or JLINK_DLL must identify JLink_x64.dll; no probe was opened")
    if args.probe_serial is None:
        p.error("--probe-serial or JLINK_PROBE_SERIAL is required; no probe was opened")
    if args.scenario == "hold" and not args.arm_only:
        p.error("--scenario hold requires --arm-only")
    if args.scenario == "timing" and not args.profile_timing:
        p.error("--scenario timing requires --profile-timing")
    if args.scenario == "motion" and args.arm_only:
        p.error("--scenario motion cannot use --arm-only")
    hold_filter_sequence = (
        []
        if args.hold_filter_sequence is None
        else [int(x) for x in args.hold_filter_sequence.split(",")]
    )
    require(
        args.hold_filter_sequence is None
        or (
            args.arm_only
            and 1 <= len(hold_filter_sequence) <= 12
            and all(x in (0, 1, 2) for x in hold_filter_sequence)
        ),
        "hold filter sequence requires ARM-only and 1..12 values from 0,1,2",
    )
    require(
        not hold_filter_sequence or args.hold_filter_setting is None,
        "sequence and single hold-filter setting are mutually exclusive",
    )
    filter_requests = hold_filter_sequence or (
        [] if args.hold_filter_setting is None else [args.hold_filter_setting]
    )
    require(
        not args.keep_final_hold_filter or filter_requests,
        "keeping HOLD filter requires a requested setting",
    )
    require(
        not args.keep_final_base_filter or args.base_filter_setting is not None,
        "keeping base filter requires a requested setting",
    )
    require(
        args.profile_stage is None or args.profile_timing, "profile stage requires profile timing"
    )
    if pylink is None:
        p.error("Install pylink-square in the bench Python environment; no probe was opened")
    session = args.session_dir.resolve()
    targets = [] if args.arm_only else [float(x) for x in args.targets.split(",")]
    axis_profile = json.loads(args.axis_profile.read_text(encoding="utf-8"))
    target_low, target_high, stop_low, stop_high, stop_current = -85.0, 85.0, -88.0, 88.0, 4.0
    if axis_profile:
        motion = axis_profile["motion"]
        margin = motion["target_margin_deg"]
        require(
            math.isfinite(margin) and margin > 0, "axis target margin must be finite and positive"
        )
        target_low = motion["minimum_deg"] + margin
        target_high = motion["maximum_deg"] - margin
        require(
            -85 <= target_low < target_high <= 85, "axis target envelope exceeds host safety range"
        )
        stop_low = max(-88, motion["minimum_deg"] + min(2, margin / 2))
        stop_high = min(88, motion["maximum_deg"] - min(2, margin / 2))
        require(
            stop_low < target_low < target_high < stop_high, "invalid target/stop envelope ordering"
        )
        stop_current = min(
            6.0 if args.phase_burst else 4.0, axis_profile["acceptance"]["maximum_iq_A"]
        )
        require(math.isfinite(stop_current) and stop_current > 0, "invalid current stop threshold")
        require(
            args.max_speed_deg <= motion["cruise_deg_s"] <= 45,
            "requested speed exceeds axis profile",
        )
        if "test_cruise_deg_s" in motion:
            require(
                math.isclose(args.max_speed_deg, motion["test_cruise_deg_s"], abs_tol=1e-6),
                "requested speed differs from the profile test speed",
            )
    require(
        all(target_low <= x <= target_high for x in targets), "target exceeds axis profile envelope"
    )
    require(
        not args.phase_burst or axis_profile is not None, "phase burst requires an axis profile"
    )
    require(
        all(math.isfinite(x) and abs(x) <= 85 for x in targets),
        "target is nonfinite or exceeds host range",
    )
    require(
        math.isfinite(args.max_speed_deg) and 0 < args.max_speed_deg <= 45,
        "invalid host speed limit",
    )
    require(1 <= args.seconds <= 20, "segment duration must be 1..20 seconds")
    destination = session / args.name
    destination.mkdir(exist_ok=False)
    image_metadata = json.loads((session / "active_image.json").read_text())
    authorized_axf = session / image_metadata["directory"] / "Vector_Mini_ST.axf"
    authorized_sha256 = hashlib.sha256(authorized_axf.read_bytes()).hexdigest()
    if (
        authorized_sha256 != image_metadata["axf_sha256"]
        or authorized_sha256 != args.expected_firmware_sha256.lower()
    ):
        message = (
            "active image metadata hash is stale"
            if authorized_sha256 != image_metadata["axf_sha256"]
            else "active HIL image differs from explicit firmware authorization"
        )
        (destination / "preflight_failure.json").write_text(
            json.dumps(
                dict(
                    schema_version=1,
                    bench_id=args.bench_id,
                    scenario=args.scenario,
                    operator_confirmation=args.operator_confirmation,
                    hardware_contacted=False,
                    failure=message,
                    expected_firmware_sha256=args.expected_firmware_sha256.lower(),
                    actual_firmware_sha256=authorized_sha256,
                ),
                indent=2,
            )
        )
        raise RuntimeError(message)
    symbols = json.loads((session / "symbols.json").read_text())
    base = symbols["servo_hil_mailbox"]
    desc = symbols["_SEGGER_RTT"] + 24 + 24
    events, polls = [], []
    j = pylink.JLink(lib=pylink.library.Library(dllpath=str(args.jlink_dll.resolve())))
    seq = 0
    heartbeat = 100
    beginning = time.monotonic()
    last_hb = -1
    raw = bytearray()
    runtime_parameters = {}
    phase_guard = None
    last_tick = None
    last_tick_time = time.monotonic()
    verified_flash = None
    command_channel_ready = not args.profile_timing
    hold_filter_supported = False
    base_filter_supported = False

    def verify_profile_image():
        metadata = json.loads((session / "active_image.json").read_text())
        image_dir = session / metadata["directory"]
        axf = image_dir / "Vector_Mini_ST.axf"
        require(
            hashlib.sha256(axf.read_bytes()).hexdigest() == metadata["axf_sha256"],
            "profile image hash differs from active metadata",
        )
        flash = bytes(j.memory_read8(0x08000000, 0x20000))
        address_base, verified = 0, 0
        for line in axf.with_suffix(".hex").read_text().splitlines():
            record = bytes.fromhex(line[1:])
            require(sum(record) % 256 == 0, "invalid profile-image HEX checksum")
            size, offset, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
            if kind == 4:
                address_base = int.from_bytes(record[4:6], "big") << 16
            elif kind == 0:
                start = address_base + offset - 0x08000000
                require(
                    0 <= start and start + size <= 0x1C000,
                    "profile image overlaps parameters or target bounds",
                )
                require(
                    flash[start : start + size] == record[4 : 4 + size], "installed image differs"
                )
                verified += size
        require(verified > 0, "profile image contains no verified load bytes")
        require(
            flash[0x1C000:] == (session / "expected_parameters.bin").read_bytes(),
            "live parameters differ from authorized backup",
        )
        return flash

    def profile_snapshot(stage_address=None):
        count_address = symbols["hil_profile_count"] if stage_address is None else stage_address + 4
        before = j.memory_read32(count_address, 1)[0]
        address = (
            symbols["hil_profile_total_cycles"] if stage_address is None else stage_address + 8
        )
        for _ in range(10):
            high = j.memory_read32(address + 4, 1)[0]
            low = j.memory_read32(address, 1)[0]
            if high == j.memory_read32(address + 4, 1)[0]:
                break
        else:
            raise RuntimeError("inconsistent cycle accumulator")
        after = j.memory_read32(count_address, 1)[0]
        return dict(count_before=before, count_after=after, total_cycles=(high << 32) | low)

    def snapshot():
        b = struct.pack("<14I", *j.memory_read32(base, 14))
        x = struct.unpack("<III f IIIII fff II", b)
        require(x[0] == 0x48494C31, "wrong firmware mailbox")
        return dict(
            ack=x[5],
            result=x[6],
            active=x[7],
            tick=x[8],
            position=x[9],
            speed=x[10],
            iq=x[11],
            mode=x[12],
            error=x[13],
        )

    def beat():
        nonlocal heartbeat, last_hb
        if time.monotonic() - last_hb >= 0.08:
            heartbeat += 1
            j.memory_write32(base + 16, [heartbeat])
            last_hb = time.monotonic()

    def drain():
        raw.extend(j.rtt_read(1, 8192))

    def command(op, value=0):
        nonlocal seq
        beat()
        seq += 1
        j.memory_write32(base + 8, [op, struct.unpack("<I", struct.pack("<f", value))[0]])
        j.memory_write32(base + 4, [seq])
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            x = snapshot()
            if x["ack"] == seq:
                events.append(
                    dict(
                        time=time.monotonic() - beginning,
                        frame=len(raw) // 8,
                        opcode=op,
                        value=value,
                        **x,
                    )
                )
                require(x["result"] == 0, f"command rejected: opcode={op}, snapshot={x}")
                return x
            time.sleep(0.002)
        raise RuntimeError("command acknowledgement timeout")

    def collect(duration):
        nonlocal last_tick, last_tick_time
        end = time.monotonic() + duration
        while time.monotonic() < end:
            beat()
            drain()
            x = snapshot()
            polls.append(dict(time=time.monotonic() - beginning, frame=len(raw) // 8, **x))
            now = time.monotonic()
            if x["tick"] != last_tick:
                last_tick, last_tick_time = x["tick"], now
            require(now - last_tick_time < 0.1, f"control tick stalled for 100 ms: {x}")
            require(not x["error"] and x["active"] and x["mode"] == 3, f"stopped/fault: {x}")
            require(
                math.radians(stop_low) < x["position"] < math.radians(stop_high),
                f"host position stop: {x}",
            )
            require(abs(x["speed"]) < math.radians(85), f"host speed stop: {x}")
            require(abs(x["iq"]) < stop_current, f"host current stop: {x}")
            time.sleep(0.006)

    failure = None
    timing = {}
    shutdown_error = None
    shutdown_verified = False
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
        j.open(args.probe_serial)
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.connect("STM32G431CB", speed=args.swd_speed_khz)
        require(not j.halted(), "CPU must be running before HIL preflight")
        if args.profile_timing:
            verified_flash = verify_profile_image()
        command_channel_ready = True
        seq = j.memory_read32(base + 4, 1)[0]
        command(1)
        time.sleep(0.02)
        x = snapshot()
        require(
            x["mode"] == 0 and not x["error"], "firmware must be disabled and fault-free before ARM"
        )
        phase_guard = read_phase_guard(j, symbols)
        if args.phase_burst:
            require(
                phase_guard is not None and phase_guard["magic"] == 0x48494331,
                "board phase guard missing",
            )
            require(
                phase_guard["maximum_phase_A"] == 6.0
                and phase_guard["exposure_threshold_A"] == 4.0,
                "wrong board phase-current guard",
            )
            require(phase_guard["exposure_limit_us"] == 30000000, "wrong board burst-time guard")
        require(abs(x["position"]) <= math.radians(85), "initial position exceeds host range")
        specs = [
            ("cascade_pos_Kp", 4, args.kp),
            ("cascade_pos_Kd", 8, args.kd),
            ("speed_Kp", 5, args.speed_kp),
            ("speed_Ki", 6, args.speed_ki),
            ("pos_maxspeed", 7, math.radians(args.max_speed_deg)),
        ]
        if not args.keep_parameters:
            for _, op, value in specs:
                command(op, value)
        offsets = json.loads((session / "member_offsets.json").read_text())["MotorControl_TypeDef"]
        if args.base_filter_setting is not None:
            require(
                "position_velocity_filter_half_cutoff" in offsets,
                "image does not support base filter comparison",
            )
            base_filter_supported = True
        if filter_requests:
            require(
                "position_hold_filter_bypass" in offsets, "image does not support live HOLD A/B"
            )
            if 2 in filter_requests:
                require(
                    "position_hold_filter_half_cutoff" in offsets,
                    "image does not support half-cutoff comparison",
                )
            hold_filter_supported = True
        if axis_profile:
            motor_address = symbols["MotorControl"]
            require(
                j.memory_read8(motor_address + offsets["axis_profile_valid"], 1)[0] == 1,
                "persisted axis profile is not valid",
            )
            axis = decode_axis_record(
                bytes(j.memory_read8(motor_address + offsets["axis_profile"], 32))
            )
            require(axis["name"] == axis_profile["name"], f"wrong persisted motor axis: {axis}")
            for actual_key, expected_key in [
                ("minimum_deg", "minimum_deg"),
                ("maximum_deg", "maximum_deg"),
                ("maximum_speed_deg_s", "cruise_deg_s"),
            ]:
                require(
                    math.isclose(axis[actual_key], motion[expected_key], abs_tol=1e-4),
                    f"wrong axis envelope: {axis}",
                )
            require(
                math.radians(target_low) <= x["position"] <= math.radians(target_high),
                "initial position is outside the profiled target envelope",
            )
            for key, expected in [
                ("current_limit", axis_profile["runtime"]["current_limit_A"]),
                ("posAcc", math.radians(motion["acceleration_deg_s2"])),
                ("posDec", math.radians(motion["stored_deceleration_deg_s2"])),
            ]:
                actual = struct.unpack(
                    "<f", bytes(j.memory_read8(motor_address + offsets[key], 4))
                )[0]
                require(
                    math.isclose(actual, expected, abs_tol=1e-6),
                    f"unexpected axis parameter: {key}={actual}, expected {expected}",
                )
        for key, _, value in specs:
            bits = j.memory_read32(symbols["MotorControl"] + offsets[key], 1)[0]
            actual = struct.unpack("<f", struct.pack("<I", bits))[0]
            require(
                math.isclose(actual, value, abs_tol=1e-6),
                f"unexpected gain: {key}={actual}, expected {value}",
            )
            runtime_parameters[key] = actual
        # Flush stale bytes to the producer's complete-frame boundary.
        wr = j.memory_read32(desc + 12, 1)[0]
        j.memory_write32(desc + 16, [wr])
        j.rtt_start(symbols["_SEGGER_RTT"])
        wait_for_rtt(j)
        for _ in range(50):
            j.rtt_read(1, 8192)
            time.sleep(0.005)
        beginning = time.monotonic()
        if "hil_irq_histogram" in symbols:
            j.memory_write32(symbols["hil_irq_histogram"], [0, 0, 0, 0])
            j.memory_write32(symbols["hil_irq_max_cycles"], [0])
        if args.profile_stage:
            stage_address = symbols["fast_loop_stage_profile"]
            j.memory_write32(stage_address, [0])
            j.memory_write32(stage_address + 4, [0, 0, 0, 0xFFFFFFFF, 0, 0])
            j.memory_write32(stage_address, [args.profile_stage])
        if filter_requests:
            selection = filter_requests[0]
            command(9, float(selection))
            require(
                j.memory_read8(symbols["MotorControl"] + offsets["position_hold_filter_bypass"], 1)[
                    0
                ]
                == int(selection == 0),
                "HOLD filter selection readback differs",
            )
            if "position_hold_filter_half_cutoff" in offsets:
                require(
                    j.memory_read8(
                        symbols["MotorControl"] + offsets["position_hold_filter_half_cutoff"], 1
                    )[0]
                    == int(selection == 2),
                    "HOLD half-cutoff selection readback differs",
                )
            runtime_parameters["hold_filter_setting"] = selection
        if base_filter_supported:
            command(10, float(args.base_filter_setting))
            require(
                j.memory_read8(
                    symbols["MotorControl"] + offsets["position_velocity_filter_half_cutoff"], 1
                )[0]
                == args.base_filter_setting,
                "base-filter selection readback differs",
            )
            runtime_parameters["base_filter_setting"] = args.base_filter_setting
        command(2)
        collect(0.5)
        if "hil_irq_histogram" in symbols:
            timing["startup_max_cycles"] = j.memory_read32(symbols["hil_irq_max_cycles"], 1)[0]
            j.memory_write32(symbols["hil_irq_histogram"], [0, 0, 0, 0])
            j.memory_write32(symbols["hil_irq_max_cycles"], [0])
        if args.profile_timing:
            for key, value in [
                ("hil_profile_min_cycles", 0xFFFFFFFF),
                ("hil_profile_interval_min_cycles", 0xFFFFFFFF),
                ("hil_profile_interval_max_cycles", 0),
            ]:
                j.memory_write32(symbols[key], [value])
            timing["profile_before"] = profile_snapshot()
        if args.profile_stage:
            j.memory_write32(stage_address + 16, [0xFFFFFFFF, 0])
            timing["stage_before"] = profile_snapshot(stage_address)
        for target in targets:
            command(3, math.radians(target))
            print("target", target, "deg", flush=True)
            collect(args.seconds)
        if args.arm_only:
            if hold_filter_sequence:
                for enabled in hold_filter_sequence:
                    command(9, float(enabled))
                    actual = j.memory_read8(
                        symbols["MotorControl"] + offsets["position_hold_filter_bypass"], 1
                    )[0]
                    require(actual == int(enabled == 0), "HOLD filter request readback differs")
                    if "position_hold_filter_half_cutoff" in offsets:
                        actual_half = j.memory_read8(
                            symbols["MotorControl"] + offsets["position_hold_filter_half_cutoff"], 1
                        )[0]
                        require(
                            actual_half == int(enabled == 2), "HOLD cutoff request readback differs"
                        )
                    print(
                        {
                            0: "A: HOLD FILTER OFF",
                            1: "B: DEFAULT HOLD CUTOFF",
                            2: "C: HALF HOLD CUTOFF",
                        }[enabled]
                        + " for "
                        + str(args.seconds)
                        + " seconds",
                        flush=True,
                    )
                    collect(args.seconds)
            else:
                collect(args.seconds)
        if args.profile_timing:
            beat()
            if args.profile_stage:
                timing["stage_after"] = profile_snapshot(stage_address)
                a, b = timing["stage_before"], timing["stage_after"]
                count_low = b["count_before"] - a["count_after"]
                count_high = b["count_after"] - a["count_before"]
                cycles = b["total_cycles"] - a["total_cycles"]
                require(0 < count_low <= count_high and cycles > 0, "invalid stage timing counters")
                timing["stage_average_us_bounds"] = [
                    cycles / count_high / 170,
                    cycles / count_low / 170,
                ]
                timing["stage_min_max_cycles"] = j.memory_read32(stage_address + 16, 2)
            timing["profile_after"] = profile_snapshot()
            a, b = timing["profile_before"], timing["profile_after"]
            count_low = b["count_before"] - a["count_after"]
            count_high = b["count_after"] - a["count_before"]
            cycles = b["total_cycles"] - a["total_cycles"]
            require(0 < count_low <= count_high and cycles > 0, "counter reset or wrap")
            timing["average_us_bounds"] = [cycles / count_high / 170, cycles / count_low / 170]
            for key in [
                "hil_profile_min_cycles",
                "hil_profile_interval_min_cycles",
                "hil_profile_interval_max_cycles",
            ]:
                timing[key] = j.memory_read32(symbols[key], 1)[0]
        if "hil_irq_histogram" in symbols:
            timing.update(
                max_cycles=j.memory_read32(symbols["hil_irq_max_cycles"], 1)[0],
                duration_bins_50us=j.memory_read32(symbols["hil_irq_histogram"], 4),
                scope="commanded moves and hold, read before STOP; ARM tracked separately",
            )
        print("trial complete", len(raw) // 8, "frames", flush=True)
    except BaseException as exc:
        failure = repr(exc)
        print("STOP:", failure, flush=True)
        raise
    finally:
        try:
            if j.opened() and command_channel_ready:
                command(1)
                time.sleep(0.03)
                final = snapshot()
                require(final["mode"] == 0 and not final["active"], "STOP did not disable HIL mode")
                require(
                    j.memory_read32(0x40012C20, 1)[0] & 0x555 == 0,
                    "phase PWM remains enabled after STOP",
                )
                if base_filter_supported and not (args.keep_final_base_filter and failure is None):
                    command(10, 0.0)
                    require(
                        j.memory_read8(
                            symbols["MotorControl"]
                            + offsets["position_velocity_filter_half_cutoff"],
                            1,
                        )[0]
                        == 0,
                        "base filter failed to restore after STOP",
                    )
                if hold_filter_supported and not (args.keep_final_hold_filter and failure is None):
                    command(9, 1.0)  # Restore configured default only after confirmed STOP.
                    require(
                        j.memory_read8(
                            symbols["MotorControl"] + offsets["position_hold_filter_bypass"], 1
                        )[0]
                        == 0,
                        "HOLD filter failed to restore after STOP",
                    )
                    if "position_hold_filter_half_cutoff" in offsets:
                        require(
                            j.memory_read8(
                                symbols["MotorControl"]
                                + offsets["position_hold_filter_half_cutoff"],
                                1,
                            )[0]
                            == 0,
                            "HOLD half-cutoff failed to restore after STOP",
                        )
                phase_guard = read_phase_guard(j, symbols)
                shutdown_verified = True
                if "hil_irq_histogram" in symbols:
                    timing["including_stop_max_cycles"] = j.memory_read32(
                        symbols["hil_irq_max_cycles"], 1
                    )[0]
                print(
                    "verified disabled at",
                    round(math.degrees(final["position"]), 3),
                    "deg",
                    flush=True,
                )
                if verified_flash is not None:
                    timing["flash_and_parameters_unchanged"] = (
                        bytes(j.memory_read8(0x08000000, 0x20000)) == verified_flash
                    )
                    require(timing["flash_and_parameters_unchanged"], "Flash changed during trial")
        except BaseException as exc:
            shutdown_error = repr(exc)
            print("STOP NOT VERIFIED: disconnect motor power; " + shutdown_error, flush=True)
            try:
                emergency = disable_outputs(j)
                (destination / "emergency.json").write_text(json.dumps(emergency, indent=2))
                print("Emergency phase disable verified; CPU halted for diagnosis", flush=True)
                (destination / "fault_ram.bin").write_bytes(
                    bytes(j.memory_read8(0x20000000, 0x8000))
                )
                (destination / "fault_scb.json").write_text(
                    json.dumps(j.memory_read32(0xE000ED00, 16))
                )
            except BaseException as emergency_exc:
                shutdown_error += "; emergency: " + repr(emergency_exc)
        finally:
            try:
                j.rtt_stop()
            except BaseException as exc:
                shutdown_error = shutdown_error or repr(exc)
            try:
                j.close()
            except BaseException as exc:
                shutdown_error = shutdown_error or repr(exc)
        ctypes.windll.winmm.timeEndPeriod(1)
        (destination / "capture.bin").write_bytes(raw)
        (destination / "trial.json").write_text(
            json.dumps(
                dict(
                    schema_version=1,
                    bench_id=args.bench_id,
                    scenario=args.scenario,
                    operator_confirmation=args.operator_confirmation,
                    authorized_firmware_sha256=args.expected_firmware_sha256.lower(),
                    motor_profile_sha256=hashlib.sha256(args.axis_profile.read_bytes()).hexdigest(),
                    arguments={
                        k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()
                    },
                    events=events,
                    polls=polls,
                    failure=failure,
                    shutdown_error=shutdown_error,
                    shutdown_verified=shutdown_verified,
                    timing=timing,
                    runtime_parameters=runtime_parameters,
                    phase_guard=phase_guard,
                    axis_profile_snapshot=axis_profile,
                    image=json.loads((session / "active_image.json").read_text())
                    if (session / "active_image.json").exists()
                    else None,
                    frame_bytes=8,
                    nominal_sample_rate_hz=2000,
                ),
                indent=2,
            )
        )
        if len(raw) % 8 == 0:
            with (destination / "capture.tsv").open("w") as f:
                f.write("\t".join(f"rtt_channel1.data{i}" for i in range(4)) + "\n")
                for row in struct.iter_unpack("<4h", raw):
                    f.write("\t".join(map(str, row)) + "\n")
        if shutdown_error and failure is None:
            raise RuntimeError("Shutdown/connection cleanup failed: " + shutdown_error)


if __name__ == "__main__":
    main()
