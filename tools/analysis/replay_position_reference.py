"""使用已有 CAN/参数备份回放生产位置控制器；不访问任何设备。"""

import argparse
import bisect
import hashlib
import json
import math
import re
import struct
import subprocess
import sys
import zlib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def axis_bounds(raw):
    """按固件AXS记录加载语义解码边界；空记录表示未配置，不能解释成零宽度限位。"""
    record = raw[0x1C000 + 2208:0x1C000 + 2240]
    if len(record) != 32:
        raise ValueError("Flash中的32字节AXS记录不完整")
    if record == bytes(32) or record == bytes([255]) * 32:
        maximum = struct.unpack("<f", bytes.fromhex("ffff7f7f"))[0]
        return -maximum, maximum, {"configured": False, "record_hex": record.hex(),
                                    "interpretation": "unconfigured; firmware uses +/-FLT_MAX"}
    magic, version, identity, minimum, maximum, speed, crc = struct.unpack("<II8sfffI", record)
    if zlib.crc32(record[:28]) != crc:
        raise ValueError("AXS记录CRC不正确，不能将无效轴配置用于回放")
    legacy = magic == 0x31535841 and version == 1 and identity in (b"roll\0\0\0\0", b"pitch\0\0\0")
    joint, revision = struct.unpack("<II", identity)
    numeric = magic == 0x32535841 and version == 2 and joint in (1, 2, 3) and revision == 1
    if not (legacy or numeric) or not all(map(math.isfinite, (minimum, maximum, speed))) or not minimum < maximum or speed <= 0:
        raise ValueError("AXS轴记录不支持或边界无效，禁止猜测回放范围")
    return minimum, maximum, {"configured": True, "record_hex": record.hex(),
                              "maximum_speed_rad_s": speed, "interpretation": "CRC-validated AXS bounds"}


def main():
    """编译冻结基线和候选，输入同一记录反馈并输出可复现指标和曲线。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-plot", action="store_true", help="只生成CSV/JSON，不要求matplotlib")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    capture = args.capture_dir
    log = capture / "csp-v0452-five-cycles-can.log"
    metadata = capture / "csp-v0452-five-cycles.json"
    dynamics = capture / "roll-dynamics-v0452.json"
    flash = capture / "pre-streaming-roll/roll-flash-readonly.bin"
    rows = []
    for line in log.read_text().splitlines():
        match = re.match(r"\(([0-9.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)", line)
        if match:
            rows.append((float(match[1]), int(match[2], 16), bytes.fromhex(match[3])))
    commands = [(t, int.from_bytes(data,"big",signed=True)/1000.0) for t, identifier, data in rows if identifier==0x306 and len(data)==4]
    feedback = {0x341:[], 0x33F:[], 0x339:[]}
    pending = {}
    for time, identifier, data in rows:
        if identifier not in feedback:
            continue
        if identifier in pending and time-pending[identifier] < 0.015:
            feedback[identifier].append((time,int.from_bytes(data,"big",signed=True)/(100.0 if identifier==0x33F else 1000.0)))
            del pending[identifier]
        elif data == bytes(4):
            pending[identifier]=time
    if not commands or any(not items for items in feedback.values()):
        raise ValueError("CAN记录缺少目标或真实反馈")
    origin = feedback[0x341][0][0]
    duration = rows[-1][0]-origin
    command_times=[x[0] for x in commands]

    def held(items, now):
        index=bisect.bisect_right([item[0] for item in items],now)-1
        return items[max(index,0)][1]

    with (out/"input.txt").open("w",encoding="utf-8") as stream:
        for tick in range(int(duration/.0005)+1):
            time=tick*.0005
            now=origin+time
            sequence=bisect.bisect_right(command_times,now)
            receipt=max(0,int((command_times[sequence-1]-origin)*1000)) if sequence else 0
            stream.write(f"{time:.6f} {held(commands,now):.9g} {held(feedback[0x341],now):.9g} {held(feedback[0x33F],now):.9g} {sequence} {receipt}\n")
    raw=flash.read_bytes()
    if len(raw)!=131072:
        raise ValueError("需要完整128KiB只读Flash备份")
    values=json.loads(dynamics.read_text(encoding="utf-8"))["values"]
    params={"ACC":values["position_acceleration"]["value"],"DEC":values["position_deceleration"]["value"],
            "MAX_SPEED":values["position_max_speed"]["value"],"SPEED_LIMIT":values["global_speed_limit"]["value"],
            "KP":values["cascade_position_kp"]["value"],"KD":values["cascade_position_kd"]["value"],
            "SPEED_KP":values["position_speed_kp"]["value"],"SPEED_KI":values["position_speed_ki"]["value"]}
    for name,offset in {"CURRENT":2148,"FRICTION_POS":2188,"FRICTION_NEG":2192,"ATTACK":2300,"RELEASE":2304}.items():
        params[name]=struct.unpack_from("<f",raw,0x1C000+offset)[0]
    params["MIN"], params["MAX"], axis_record = axis_bounds(raw)
    if axis_record["configured"]:
        params["MAX_SPEED"] = min(params["MAX_SPEED"], axis_record["maximum_speed_rad_s"])
    def literal(value):
        result=format(value,".9g")
        return result+(".0" if "." not in result and "e" not in result else "")+"f"
    (out/"runtime_params.h").write_text("\n".join(f"#define REPLAY_{key} {literal(value)}" for key,value in params.items())+"\n",encoding="utf-8")
    config=function_source((ROOT/"tests/unit/position_servo_test.c").read_text(encoding="utf-8"),"config")
    (out/"config.inc").write_text(config,encoding="utf-8")
    frozen=out/"baseline-source"
    frozen.mkdir(exist_ok=True)
    files=["firmware/motor/position/position_cascade.c","firmware/motor/position/position_cascade.h","firmware/motor/position/position_cascade_config.h",
           "firmware/motor/trajectory/position_smooth_trajectory.c","firmware/motor/trajectory/position_smooth_trajectory.h","firmware/motor/foc/foc_pid.c","firmware/motor/foc/foc_pid.h"]
    baseline_hash={}
    for relative in files:
        content=subprocess.check_output(["git","show",f"ef6bb874:{relative}"],cwd=ROOT)
        (frozen/Path(relative).name).write_bytes(content)
        baseline_hash[relative]=hashlib.sha256(content).hexdigest()
    compiler=[args.cc]+(["cc"] if Path(args.cc).stem=="zig" else [])
    traces={}
    logs=[]
    for name,baseline,external in (("v062-baseline",True,False),("v064-default",False,False),("v064-external",False,True)):
        exe=out/(name+".exe")
        sources=[frozen/Path(p).name if baseline else ROOT/p for p in files if p.endswith(".c")]
        includes=(["-I",str(frozen)] if baseline else [])+NATIVE_INCLUDE_FLAGS
        command=compiler+["-std=c99","-O2","-ffp-contract=off","-UNDEBUG","-Wall","-Wextra","-Werror",f"-DREPLAY_HAS_EXTERNAL={int(not baseline)}"]+includes+["-I",str(out),str(ROOT/"tools/analysis/position_reference_replay.c"),*map(str,sources),"-o",str(exe)]
        for invocation in (command,[str(exe),str(out/"input.txt"),str(out/(name+".csv")),str(int(external))]):
            result=subprocess.run(invocation,cwd=ROOT,capture_output=True,text=True)
            logs.append(result.stdout+result.stderr)
            (out/"replay.log").write_text("".join(logs),encoding="utf-8")
            result.check_returncode()
        traces[name]=np.genfromtxt(out/(name+".csv"),delimiter=",",names=True)
    baseline=traces["v062-baseline"]
    default=traces["v064-default"]
    if not np.array_equal(baseline,default):
        raise RuntimeError("默认点到点回放与冻结基线不一致")
    metrics={}
    for name,trace in traces.items():
        error=trace["target"]-trace["reference"]
        index=int(np.flatnonzero(trace["target"]>=.371999)[0])
        metrics[name]={"max_reference_error_deg":float(np.max(abs(error))*180/math.pi),
                       "rms_reference_error_deg":float(np.sqrt(np.mean(error**2))*180/math.pi),
                       "max_speed_command_rad_s":float(np.max(abs(trace["speed_command"]))),
                       "max_iq_command_a":float(np.max(abs(trace["iq_command"]))),
                       "first_target_0372":{field:float(trace[field][index]) for field in trace.dtype.names}}
        if "external_rejected_points" in trace.dtype.names:
            metrics[name]["external_rejected_points"] = int(np.max(trace["external_rejected_points"]))
            metrics[name]["external_stale_ticks"] = int(np.count_nonzero(trace["external_stale"]))
    info=json.loads(metadata.read_text(encoding="utf-8"))
    summary={"hardware_accessed":False,"fixed_recorded_feedback_not_a_plant_prediction":True,
             "recorded_duration_s":duration,"commands":len(commands),"feedback_pairs":{hex(k):len(v) for k,v in feedback.items()},
             "capture_declared_success":info.get("overall_success"),"parameters":params,"axis_record":axis_record,"default_equals_frozen_baseline":True,"metrics":metrics,
             "plot_requested":not args.no_plot,
             "baseline_source_sha256":baseline_hash,"input_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (log,metadata,dynamics,flash)}}
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    if args.no_plot:
        print(json.dumps(summary["metrics"],indent=2))
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(11,12),sharex=True)
    current=traces["v064-external"]
    for column,label,color in (("target","Received target","black"),("measured","Recorded actual (v0.6.2)","gray")):
        axes[0].plot(baseline["time"],np.rad2deg(baseline[column]),label=label,color=color)
    for trace,label,color in ((baseline,"Point-to-point replay","tab:red"),(current,"External candidate replay","tab:blue")):
        axes[0].plot(trace["time"],np.rad2deg(trace["reference"]),label=label,color=color)
        axes[1].plot(trace["time"],np.rad2deg(trace["target"]-trace["reference"]),label=label,color=color)
        axes[2].plot(trace["time"],np.rad2deg(trace["speed_command"]),label=label,color=color)
        axes[3].plot(trace["time"],trace["iq_command"],label=label,color=color)
    axes[2].axhline(45,color="black",ls="--",label="45 deg/s acceptance")
    iq=feedback[0x339]
    axes[3].plot([t-origin for t,v in iq],[v for t,v in iq],color="gray",label="Recorded actual Iq")
    for axis,label in zip(axes,("Position (deg)","Target - internal ref (deg)","Speed command (deg/s)","Iq (A)")):
        axis.set_ylabel(label);axis.grid(True,alpha=.25);axis.legend(fontsize=8)
    axes[-1].set_xlabel("Seconds from first recorded position reply")
    fig.suptitle("Fixed-sensor replay; candidate curves are NOT new motor measurements")
    fig.tight_layout();fig.savefig(out/"replay.png",dpi=160);plt.close(fig)
    print(json.dumps(summary["metrics"],indent=2))


if __name__=="__main__":
    main()
