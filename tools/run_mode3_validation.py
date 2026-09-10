"""Mode-3 host regression, motion plans and recorded-data acceptance (no probe IO).

Only the explicitly generated run_bench.ps1 uses the existing local HIL backend.
This module uses Python's standard library and never downloads or enables firmware.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / "tests/mode3/bench_20260908.json"
CASES = ("hold", "micro", "small", "medium", "range")
# Pin the known backend's motor/motion/runtime/core contract, not the mutable
# default JSON file. Editing that file must not silently authorize a new bench.
LOCAL_BENCH_CONTRACT_SHA256 = "dff45d613ef09df00e911916ab0c6dd8aa5c7710fda072dd01bef90e04048295"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def number(obj, key, lower=0, strictly=True):
    value = obj.get(key)
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or (value <= lower if strictly else value < lower)):
        raise ValueError(f"{key}: requires a finite {'>' if strictly else '>='} {lower} value, got {value}")
    return value


def validate_profile(p):
    if p.get("format_version") != 1:
        raise ValueError("Unsupported profile format_version")
    m, r, c, a = (p[k] for k in ("motion", "runtime", "core", "acceptance"))
    lo = number(m, "minimum_deg", -1e6)
    hi = number(m, "maximum_deg", -1e6)
    margin = number(m, "target_margin_deg")
    if hi <= lo + 2 * margin:
        raise ValueError("Travel interval must exceed twice the target margin")
    for key in ("cruise_deg_s", "acceleration_deg_s2", "stored_deceleration_deg_s2"):
        number(m, key)
    if "test_cruise_deg_s" in m:
        if number(m, "test_cruise_deg_s") > m["cruise_deg_s"]:
            raise ValueError("Test cruise must not exceed the persisted axis speed ceiling")
    for key in ("cascade_pos_Kp", "cascade_pos_Kd", "speed_Kp", "speed_Ki"):
        number(r, key, strictly=False)
    if r["cascade_pos_Kp"] > 50 or r["cascade_pos_Kd"] > 10:
        raise ValueError("Position gains exceed firmware validation limits")
    number(r, "current_limit_A")
    for key in ("deceleration_cap_deg_s2", "jerk_ramp_s", "hold_enter_deg", "hold_exit_deg"):
        number(c, key)
    number(a, "position_tolerance_deg")
    if not c["hold_enter_deg"] < c["hold_exit_deg"] < a["position_tolerance_deg"]:
        raise ValueError("Require hold enter < hold exit < acceptance tolerance")
    number(a, "tail_seconds")
    if number(a, "minimum_hold_fraction") > 1:
        raise ValueError("minimum_hold_fraction must be <= 1")
    number(a, "maximum_iq_A")
    if "maximum_post_hold_error_deg" in a:
        number(a, "maximum_post_hold_error_deg")
    for key in ("maximum_drop_samples", "maximum_saturation_samples"):
        value = number(a, key, strictly=False)
        if int(value) != value:
            raise ValueError(f"{key} must be an integer")
    for case in CASES:
        if number(p["dwell_seconds"], case) < a["tail_seconds"] + 1:
            raise ValueError(f"{case}: dwell must leave time before the acceptance tail")
    return p


def motion_cases(p):
    validate_profile(p)
    m = p["motion"]
    lo, hi = m["minimum_deg"] + m["target_margin_deg"], m["maximum_deg"] - m["target_margin_deg"]
    center, radius = (lo + hi) / 2, (hi - lo) / 2
    small, medium = min(5, radius / 4), min(20, radius / 2)
    micro = min(p["core"]["hold_enter_deg"] / 2, radius / 8)
    targets = {
        "hold": [center],
        "micro": [center, center + micro, center - micro, center],
        "small": [center + small, center, center - small, center],
        "medium": [center + medium, center, center - medium, center + medium, center],
        "range": [hi, lo, center, center + small, center - small, center],
    }
    return [{"id": key, "targets_deg": values, "seconds_per_target": p["dwell_seconds"][key]}
            for key, values in targets.items()]


def metadata():
    def git(*args):
        r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else None
    files = ["Foc/position_cascade.c", "Foc/position_cascade_config.h",
             "Foc/position_smooth_trajectory.c", "Foc/foc_run.c", "Foc/foc_param.h",
             "tools/run_mode3_validation.py", "tools/run_position_servo_tests.py",
             "tools/test_mode3_validation.py", "tests/unit/position_servo_test.c",
             "tests/unit/position_smooth_trajectory_test.c"]
    return {"utc": datetime.now(timezone.utc).isoformat(), "commit": git("rev-parse", "HEAD"),
            "working_tree_status": git("status", "--short", "--untracked-files=no"),
            "source_sha256": {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest() for f in files}}


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def make_plan(profile, out, python_executable=sys.executable):
    p = validate_profile(read_json(profile))
    cases = motion_cases(p)
    m, c = p["motion"], p["core"]
    decel = min(m["stored_deceleration_deg_s2"], c["deceleration_cap_deg_s2"])
    report = {"profile": p, "cases": cases, "metadata": metadata(), "hardware_executed": False,
              "effective_deceleration_deg_s2": decel,
              "jerk_limit_deg_s3": max(m["acceleration_deg_s2"], decel) / c["jerk_ramp_s"],
              "actual_speed_pi_kp": p["runtime"]["speed_Kp"] * p["runtime"]["current_limit_A"],
              "actual_speed_pi_ki": p["runtime"]["speed_Ki"] * p["runtime"]["current_limit_A"]}
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    # The existing runner is bound to one known bench, including fixed stop limits.
    # Do not generate a deceptively portable motion script for a different motor.
    contract = {key: p[key] for key in ("motor", "motion", "runtime", "core")}
    # Naming the existing bench roll does not change its validated hardware.
    if p.get("name") == "roll" and contract["motor"].get("identity") == "roll":
        contract = dict(contract, motor=dict(contract["motor"], identity="existing direct-drive bench"))
    digest = hashlib.sha256(json.dumps(contract, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    compatible = digest == LOCAL_BENCH_CONTRACT_SHA256
    compatible = compatible and all(case["seconds_per_target"] <= 20 for case in cases)
    report["local_bench_script_available"] = compatible
    write_json(out / "plan.json", report)
    write_json(out / "profile.json", p)
    lines = ["# Mode 3 test plan", "", "This file describes commands; plan generation does not move the motor.", "",
             "| Case | Motor targets / deg | Seconds per target |", "| --- | --- | --- |"]
    lines += [f"| {x['id']} | {', '.join(f'{v:g}' for v in x['targets_deg'])} | {x['seconds_per_target']:g} |" for x in cases]
    lines += ["", "Verify motor zero, installed HIL image/symbols, parameters and travel before motion.",
              "Mechanical vibration is a separate manual measurement, not an automatic PASS."]
    (out / "plan.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if compatible:
        quote = lambda s: "'" + str(s).replace("'", "''") + "'"
        commands = ["# Explicit invocation moves the existing bench motor. No flashing or parameter writes.",
                    "param([ValidateSet('hold','micro','small','medium','range')][string]$Case = 'small')",
                    "$ErrorActionPreference = 'Stop'", f"$taskPython = {quote(python_executable)}",
                    f"$taskBackend = {quote(ROOT / 'tools/servo_hil_run.py')}",
                    "if (!(Test-Path -LiteralPath $taskBackend)) { throw 'Local HIL backend is missing; see docs/mode3_test_plan.md' }",
                    "$taskName = 'mode3_' + $Case + '_' + [Guid]::NewGuid().ToString('N')", "switch ($Case) {"]
        r = p["runtime"]
        for case in cases:
            target = ",".join(f"{x:g}" for x in case["targets_deg"])
            axis_arg = f" --axis-profile {quote(out / 'profile.json')}" if p.get('name') == 'roll' else ""
            commands += [f"  '{case['id']}' {{ & $taskPython $taskBackend --name $taskName --keep-parameters "
                         f"--kp {r['cascade_pos_Kp']} --kd {r['cascade_pos_Kd']} --speed-kp {r['speed_Kp']} "
                         f"--speed-ki {r['speed_Ki']} --max-speed-deg {m['cruise_deg_s']} "
                         f"'--targets={target}' --seconds {case['seconds_per_target']}{axis_arg} }}"]
        commands += ["}", "if ($LASTEXITCODE -ne 0) { throw 'HIL run failed; inspect the trial and shutdown state before continuing' }",
                     f"$taskTrial = Join-Path {quote(ROOT / 'outputs/servo_hil_20260908')} $taskName",
                     f"& $taskPython {quote(Path(__file__).resolve())} analyze --profile {quote(out / 'profile.json')} --trial $taskTrial --case $Case",
                     "if ($LASTEXITCODE -ne 0) { throw 'Recorded-data acceptance failed' }"]
        (out / "run_bench.ps1").write_text("\n".join(commands) + "\n", encoding="utf-8-sig")
    return report


def analyze_trial(profile, trial_path, case_id=None):
    """Fail closed for missing/invalid samples; use host frame anchors as approximate time."""
    p = validate_profile(read_json(profile))
    path = Path(trial_path)
    trial = read_json(path / "trial.json")
    with (path / "capture.tsv").open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream, delimiter="\t")
        header = next(reader)
        if header != [f"rtt_channel1.data{i}" for i in range(12)]:
            raise ValueError("Expected mode-3 12-channel RTT capture")
        rows = [[int(v) for v in row] for row in reader]
    if not rows or any(len(row) != 12 or any(v < -32768 or v > 32767 for v in row) for row in rows):
        raise ValueError("Empty/malformed int16 RTT capture")
    from rtt_control_frame import decode
    decoded = decode(rows)
    count = len(rows)
    events = [e for e in trial["events"] if e["opcode"] == 3]
    expected = [float(v) for v in trial["arguments"]["targets"].split(",")]
    expected_case = None
    if case_id:
        expected_case = next(x for x in motion_cases(p) if x["id"] == case_id)
        expected = expected_case["targets_deg"]
    errors = []
    def require(ok, why):
        if not ok:
            errors.append(why)
    require(not trial.get("failure"), "trial failure")
    require(trial.get("shutdown_verified") is True and not trial.get("shutdown_error"), "shutdown not verified")
    require(bool(trial.get("image")), "image identity missing")
    if "maximum_irq_cycles" in p["acceptance"]:
        deadline = p["acceptance"]["maximum_irq_cycles"]
        for key in ("startup_max_cycles", "max_cycles", "including_stop_max_cycles"):
            cycles = (trial.get("timing") or {}).get(key)
            require(isinstance(cycles, int) and 0 <= cycles < deadline, "IRQ deadline/missing: " + key)
    if "burst_current_guard" in p:
        guard = trial.get("phase_guard") or {}
        require(trial.get("arguments", {}).get("phase_burst") is True, "phase burst not explicitly enabled")
        require(guard.get("magic") == 0x48494331, "board phase guard missing")
        for key in ("maximum_phase_A", "exposure_threshold_A", "exposure_limit_us"):
            require(guard.get(key) == p["burst_current_guard"][key], "phase guard contract: " + key)
        peak = guard.get("observed_peak_A")
        require(isinstance(peak, (int, float)) and math.isfinite(peak) and
                0 <= peak < p["burst_current_guard"]["maximum_phase_A"], "phase peak threshold")
        freq, ticks = guard.get("frequency_hz", 0), guard.get("exposure_ticks", -1)
        require(isinstance(freq, int) and freq > 0 and isinstance(ticks, int) and ticks >= 0 and
                ticks * 1000000 < freq * p["burst_current_guard"]["exposure_limit_us"], "phase exposure duration")
        require(guard.get("trip") == 0, "board phase guard tripped")
    expected_runtime = {key: p["runtime"][key] for key in
                        ("cascade_pos_Kp", "cascade_pos_Kd", "speed_Kp", "speed_Ki")}
    expected_runtime["pos_maxspeed"] = math.radians(
        p["motion"].get("test_cruise_deg_s", p["motion"]["cruise_deg_s"]))
    actual_runtime = trial.get("runtime_parameters", {})
    for key, value in expected_runtime.items():
        actual = actual_runtime.get(key)
        require(isinstance(actual, (int, float)) and not isinstance(actual, bool) and
                math.isfinite(actual) and math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-6),
                f"runtime parameter missing/mismatch: {key}")
    if expected_case:
        require(trial["arguments"]["seconds"] >= expected_case["seconds_per_target"], "case dwell shorter than planned")
    require(len(events) == len(expected) and len(events) > 0, "missing/unexpected target commands")
    m = p["motion"]
    require(all(m["minimum_deg"] + m["target_margin_deg"] <= v <= m["maximum_deg"] - m["target_margin_deg"] for v in expected), "targets outside profile travel")
    for i, e in enumerate(events):
        if i < len(expected):
            require(abs(math.degrees(e["value"]) - expected[i]) < .001, "target sequence mismatch")
        require(e.get("result") == 0 and e.get("error") == 0 and e.get("mode") == 3, "target command rejected/fault")
    polls = trial["polls"]
    if len(polls) < 2:
        raise ValueError("Need host timing anchors")
    anchors = []
    for poll in polls:
        frame, time_s = poll["frame"], poll["time"]
        if not isinstance(frame, int) or not 0 <= frame <= count or not math.isfinite(time_s):
            raise ValueError("Invalid host timing anchor")
        if anchors and (frame < anchors[-1][0] or time_s < anchors[-1][1]):
            raise ValueError("Non-monotonic host timing anchors")
        if anchors and frame == anchors[-1][0]:
            anchors[-1] = (frame, time_s)
        else:
            anchors.append((frame, time_s))
    if len(anchors) < 2:
        raise ValueError("No sample progress")
    q, times, flags = decoded['position_deg'].tolist(), [], []
    k = 0
    for i, row in enumerate(rows):
        while k + 1 < len(anchors) and anchors[k+1][0] <= i: k += 1
        if i < anchors[0][0]: t = anchors[0][1]
        elif k + 1 == len(anchors): t = anchors[k][1]
        else:
            f0, t0 = anchors[k]; f1, t1 = anchors[k+1]
            t = t0 + (t1-t0) * (i-f0) / (f1-f0)
        times.append(t)
        flags.append(row[11] & 65535)
    a = p["acceptance"]
    moves = []
    for i, e in enumerate(events):
        start = e["frame"]
        end = events[i+1]["frame"] if i+1 < len(events) else count
        if not isinstance(start, int) or not 0 <= start < end <= count:
            raise ValueError("Missing/out-of-order target samples")
        indices = range(start, end)
        target = math.degrees(e["value"])
        require(all(decoded['valid'][n] for n in indices), f"move {i+1}: invalid telemetry")
        require(not any(decoded['encoding_saturated'][n] for n in indices), f'move {i+1}: RTT encoding saturated')
        tail = [n for n in indices if times[n] >= times[end-1] - a["tail_seconds"]]
        require(times[tail[-1]] - times[tail[0]] >= a["tail_seconds"] - .05, f"move {i+1}: incomplete acceptance tail")
        dwell = trial["arguments"]["seconds"]
        require(times[end-1] - e["time"] >= dwell - .1, f"move {i+1}: truncated dwell")
        error = max(abs(target - q[n]) for n in tail)
        hold = sum((flags[n] & 3) == 2 for n in tail) / len(tail)
        peak = max(abs(decoded['iq_feedback_A'][n]) for n in indices)
        drops = sum(bool(flags[n] & 64) for n in indices)
        saturation = sum(bool(flags[n] & 8) for n in indices)
        # Require 50 ms continuously in HOLD after command dispatch before
        # checking all subsequent samples, including a later exit/recovery.
        # A good final tail must not hide a delayed slip earlier in the dwell.
        post_hold_error = None
        hold_start = None
        for n in indices:
            if times[n] < e["time"] + .05 or (flags[n] & 3) != 2:
                hold_start = None
            else:
                if hold_start is None: hold_start = n
                if times[n] - times[hold_start] >= .05:
                    post_hold_error = max(abs(target - q[k]) for k in range(hold_start, end))
                    break
        if "maximum_post_hold_error_deg" in a:
            require(post_hold_error is not None and post_hold_error <= a["maximum_post_hold_error_deg"],
                    f"move {i+1}: post-HOLD excursion/missing sustained HOLD")
        require(error <= a["position_tolerance_deg"], f"move {i+1}: position tolerance")
        require(hold >= a["minimum_hold_fraction"], f"move {i+1}: HOLD fraction")
        require(peak < a["maximum_iq_A"], f"move {i+1}: current threshold")
        require(drops <= a["maximum_drop_samples"], f"move {i+1}: RTT drops")
        require(saturation <= a["maximum_saturation_samples"], f"move {i+1}: current saturation")
        direction = 1 if target > q[start] else -1 if target < q[start] else 0
        moves.append({"target_deg": target, "tail_max_abs_error_deg": error, "tail_hold_fraction": hold,
                      "post_hold_max_abs_error_deg": post_hold_error,
                      "tail_motion_peak_to_peak_deg": max(q[n] for n in tail) - min(q[n] for n in tail),
                      "iq_peak_A": peak, "drop_samples": drops, "saturation_samples": saturation,
                      "overshoot_deg": max(0, max(direction * (q[n]-target) for n in indices))})
    result = {"automatic_pass": not errors, "errors": errors, "moves": moves, "profile": p,
              "recorded_image": trial.get("image"), "recorded_runtime_parameters": trial.get("runtime_parameters"),
              "timing": "Host receipt anchors only; batching delay, not precise device timestamps",
              "mechanical_vibration": "NOT_EVALUATED; requires user/structure measurement",
              "hardware_executed_by_analyzer": False}
    write_json(path / "mode3_acceptance.json", result)
    return result


def host_tests(cc, out):
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = {"metadata": metadata(), "hardware_executed": False, "steps": []}
    commands = [[sys.executable, str(ROOT / "tools/run_position_servo_tests.py"), "--out", str(out / "native")]]
    if cc: commands[0] += ["--cc", cc]
    # Select repository-owned offline checks, avoiding optional local HIL imports.
    commands += [[sys.executable, "-m", "unittest", "discover", "-s", "tools", "-p", pattern]
                 for pattern in ("test_position_servo_boundary.py", "test_mode3_validation.py", "test_motor_axis_record.py")]
    for i, cmd in enumerate(commands):
        result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        (out / f"step_{i+1}.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout + result.stderr, end="")
        report["steps"].append({"command": cmd, "returncode": result.returncode})
        report["passed"] = all(x["returncode"] == 0 for x in report["steps"])
        write_json(out / "result.json", report)
        if result.returncode: return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    host = sub.add_parser("host"); host.add_argument("--cc"); host.add_argument("--out", type=Path)
    plan = sub.add_parser("plan"); plan.add_argument("--out", type=Path)
    plan.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    analyze = sub.add_parser("analyze"); analyze.add_argument("--trial", type=Path, required=True)
    analyze.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    analyze.add_argument("--case", choices=CASES)
    args = parser.parse_args()
    try:
        if args.action == "analyze":
            result = analyze_trial(args.profile, args.trial, args.case)
            print(json.dumps({"automatic_pass": result["automatic_pass"], "errors": result["errors"],
                              "report": str(args.trial / "mode3_acceptance.json")}, ensure_ascii=False, indent=2))
            return 0 if result["automatic_pass"] else 1
        out = args.out or ROOT / "outputs/mode3_validation" / (args.action + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        if args.action == "host": return 0 if host_tests(args.cc, out) else 1
        make_plan(args.profile, out)
        print(f"Plan written to {out}; no hardware accessed.")
        return 0
    except (ValueError, KeyError, OSError, StopIteration) as exc:
        parser.exit(2, f"ERROR: {exc}\n")


if __name__ == "__main__":
    sys.exit(main())
