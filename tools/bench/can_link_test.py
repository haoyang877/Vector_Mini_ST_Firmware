"""六轴台架 CAN FD 只读通信测试。固定笔数或阶梯负载，按序号记录应答。"""

from __future__ import annotations

import argparse
import csv
import ctypes as C
import json
import os
import secrets
import struct
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "host_app"))
from ports.controlcanfd import _RXFD, _TXFD, ControlCanFd  # noqa: E402
from upgrade_client import decode_reply, encode_frames  # noqa: E402

TOOL_VERSION = "1.0.1"
AXES = {3: "左轮", 4: "右轮", 5: "ROLL", 6: "PITCH", 7: "YAW", 8: "Lift"}
ARB_RATES = {"500K": 500_000, "1M": 1_000_000}
DATA_RATES = {**ARB_RATES, "2M": 2_000_000, "3M": 3_000_000, "4M": 4_000_000, "5M": 5_000_000}

# 名称、总查询速率（笔/秒）、发送时间（秒）
STRESS = [
    ("baseline", 5, 20),
    ("medium", 50, 30),
    ("high", 200, 30),
    ("peak", 500, 60),
    ("recovery", 5, 20),
]


@dataclass
class Query:
    seq: int
    node: int
    kind: int
    stage: str
    sent_s: float
    accepted: bool = False
    clean_ms: float | None = None
    flagged_ms: float | None = None


def match_reply(queries, identifier, data, now, *, extended, brs, remote, flagged):
    """按目标、类型、双 CRC、序号和负载校验；同一序号每类只记首次应答。"""
    seq = int.from_bytes(data[8:10], "little")
    q = queries.get(seq)
    if not (extended and brs and not remote and q and q.accepted):
        return "unmatched"
    try:
        body = decode_reply(identifier, data, q.node, q.kind, seq)
    except ValueError:
        return "unmatched"
    if q.kind == 108 and (len(body) != 45 or body[-3] != q.node):
        return "unmatched"
    if q.kind == 1 and len(body) != 32:
        return "unmatched"
    latency = (now - q.sent_s) * 1000
    if flagged:
        if q.flagged_ms is None:
            q.flagged_ms = latency
        return "flagged"
    if q.clean_ms is None:
        q.clean_ms = latency
    return "clean"


def summarize(rows):
    rows = list(rows)
    accepted = sum(q.accepted for q in rows)
    clean = sum(q.clean_ms is not None for q in rows)
    flagged = sum(q.flagged_ms is not None and q.clean_ms is None for q in rows)
    return dict(
        attempted=len(rows),
        accepted=accepted,
        rejected=len(rows) - accepted,
        clean=clean,
        ontime_100ms=sum(q.clean_ms is not None and q.clean_ms <= 100 for q in rows),
        flagged_only=flagged,
        no_match=accepted - clean - flagged,
        response_pct=round(100 * clean / accepted, 3) if accepted else None,
    )


def receive(bus, buffer, queries, trace, start):
    count = bus.dll.ZCAN_ReceiveFD(bus.channel, C.byref(buffer), len(buffer), 0)
    if not 0 <= count <= len(buffer):
        raise OSError(f"ZCAN_ReceiveFD 返回 {count}")
    now = time.perf_counter() - start
    for item in buffer[:count]:
        frame = item.frame
        if frame.length > 64:
            raise OSError(f"CAN FD 长度异常 {frame.length}")
        data = bytes(frame.data[: frame.length])
        label = match_reply(
            queries,
            frame.can_id,
            data,
            now,
            extended=frame.eff,
            brs=frame.brs,
            remote=frame.rtr,
            flagged=frame.err or frame.esi,
        )
        trace.writerow(
            (
                now,
                item.timestamp,
                hex(frame.can_id),
                frame.eff,
                frame.brs,
                frame.rtr,
                frame.err,
                frame.esi,
                data.hex(),
                label,
            )
        )
    return count


def send_fd_once(bus, identifier, data, *, extended=True):
    """用 SDK 提交一帧 CAN FD + BRS 数据，保留调用方给定的字节。"""
    tx = _TXFD()
    tx.transmit_type = 1  # 单次发送，复用台架的无重试口径
    tx.frame.can_id = identifier
    tx.frame.eff = extended
    tx.frame.brs = 1
    tx.frame.length = len(data)
    tx.frame.data[: len(data)] = data
    return bus.dll.ZCAN_TransmitFD(bus.channel, C.byref(tx), 1) == 1


def transmit(bus, q):
    payload = struct.pack("<IIHH", 0, q.seq, 0, 0) if q.kind == 1 else b""
    ((identifier, data),) = encode_frames(q.node, q.kind, payload, q.seq)
    q.accepted = send_fd_once(bus, identifier, data)


def run(args, report, queries, trace, log, stop, progress=None):
    start = time.perf_counter()
    seq = secrets.randbelow(65535)
    buffer = (_RXFD * 256)()
    with (
        os.add_dll_directory(str(args.dll.parent)),
        ControlCanFd(
            args.dll,
            channel_index=args.channel - 1,
            arb_bps=report["arb_bps"],
            data_bps=report["data_bps"],
        ) as bus,
    ):

        def pump(until):
            if stop is not None and stop.is_set():
                raise InterruptedError("测试已停止")
            while time.perf_counter() < until:
                if stop is not None and stop.is_set():
                    raise InterruptedError("测试已停止")
                report["raw_rx"] += receive(bus, buffer, queries, trace, start)
                time.sleep(min(0.0006, max(0, until - time.perf_counter())))

        pump(time.perf_counter() + 0.3)
        schedule = STRESS if args.stress else [("fixed", args.rate, None)]
        total_seconds = sum(seconds for _, _, seconds in STRESS)
        completed_seconds = 0
        for stage_index, (name, rate, duration) in enumerate(schedule, start=1):
            begin = due = time.perf_counter()
            next_update = begin
            count = round(rate * duration) if duration else args.count * len(args.nodes)
            stage = dict(name=name, target_hz=rate, planned_count=count)
            report["stages"].append(stage)
            for i in range(count):
                pump(due)
                now = time.perf_counter()
                if duration and now >= begin + duration:
                    break
                seq = seq % 65535 + 1
                node = args.nodes[i % len(args.nodes)]
                kind = 1 if args.stress and i // len(args.nodes) % 20 == 0 else 108
                q = Query(seq, node, kind, name, now - start)
                queries[seq] = q
                transmit(bus, q)
                if progress and (now >= next_update or i + 1 == count):
                    if duration:
                        fraction = (completed_seconds + min(duration, now - begin)) / total_seconds
                        label = f"阶段 {stage_index}/{len(schedule)} · {rate} 笔/秒 · 已查询 {len(queries)} 笔"
                    else:
                        fraction = (i + 1) / count
                        label = f"已查询 {i + 1}/{count} 笔"
                    progress(fraction, label)
                    next_update = now + 0.15
                report["raw_rx"] += receive(bus, buffer, queries, trace, start)
                due = max(due + 1 / rate, time.perf_counter())
            pump(begin + duration if duration else due)
            stage["send_duration_s"] = round(time.perf_counter() - begin, 3)
            completed_seconds += duration or 0
            if progress:
                progress(
                    completed_seconds / total_seconds if duration else 1,
                    f"等待应答 · 已查询 {len(queries)} 笔",
                )
            pump(time.perf_counter() + (1.2 if args.stress else 2))
            stats = summarize(q for q in queries.values() if q.stage == name)
            log(
                f"{name}  有效应答 {stats['clean']}/{stats['accepted']}，"
                f"SDK 拒收 {stats['rejected']}"
            )
        if args.stress:
            pump(time.perf_counter() + 4)


def node_id(value):
    try:
        node = int(value, 0)
    except ValueError:
        node = next((n for n, axis in AXES.items() if axis.casefold() == value.casefold()), None)
    if node not in AXES:
        raise argparse.ArgumentTypeError(f"未知节点 {value}，可选 0x03 到 0x08 或轴名")
    return node


def main(argv=None, *, output_dir=None, log=print, stop=None, progress=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("nodes", nargs="*", type=node_id, help="节点地址或轴名，默认六轴")
    parser.add_argument("--stress", action="store_true", help="5/50/200/500/5 笔每秒阶梯压测")
    parser.add_argument("--count", type=int, help="常规测试每节点查询数，默认 1000")
    parser.add_argument("--rate", type=int, help="常规测试总速率，默认每秒 50 笔")
    parser.add_argument("--channel", type=int, choices=(1, 2), default=1, help="USB-CAN 物理通道")
    parser.add_argument(
        "--arb-rate", type=str.upper, choices=ARB_RATES, default="1M", help="仲裁段波特率"
    )
    parser.add_argument(
        "--data-rate", type=str.upper, choices=DATA_RATES, default="5M", help="数据段波特率"
    )
    sdk_root = Path(os.environ.get("MOTOR_CTRL_APP_ROOT", ROOT.parent / "motor_ctrl_app"))
    parser.add_argument(
        "--dll",
        type=Path,
        default=sdk_root / ".local/canfd-sdk/ControlCANFD.dll",
        help="厂商 ControlCANFD.dll 路径",
    )
    args = parser.parse_args(argv)
    args.nodes = args.nodes or list(AXES)
    if len(set(args.nodes)) != len(args.nodes):
        parser.error("节点重复")
    if args.stress and (args.count is not None or args.rate is not None):
        parser.error("--stress 使用固定阶梯，请单独使用 --count 和 --rate 进行常规测试")
    args.count = 1000 if args.count is None else args.count
    args.rate = 50 if args.rate is None else args.rate
    if args.count < 1 or args.count * len(args.nodes) > 65535 or args.rate < 1:
        parser.error("笔数和速率须为正数，一轮最多使用 65535 个唯一序号")
    args.dll = args.dll.resolve()
    if not args.dll.is_file():
        parser.error(f"找不到 {args.dll}，请通过 --dll 指定厂商 DLL")

    out = output_dir or ROOT / "outputs/can-link-test" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    out.mkdir(parents=True)
    queries = {}
    report = dict(
        complete=False,
        tool_version=TOOL_VERSION,
        nodes=args.nodes,
        channel=args.channel,
        arb_bps=ARB_RATES[args.arb_rate],
        data_bps=DATA_RATES[args.data_rate],
        single_shot=True,
        raw_rx=0,
        stages=[],
    )
    try:
        with (out / "rx.csv").open("w", newline="", encoding="utf-8") as f:
            trace = csv.writer(f)
            trace.writerow(
                (
                    "host_s",
                    "adapter_us",
                    "can_id",
                    "extended",
                    "brs",
                    "rtr",
                    "err",
                    "esi",
                    "data_hex",
                    "match",
                )
            )
            timer_enabled = C.windll.winmm.timeBeginPeriod(1) == 0
            try:
                run(args, report, queries, trace, log, stop, progress)
                report["complete"] = True
            finally:
                if timer_enabled:
                    C.windll.winmm.timeEndPeriod(1)
    except (OSError, KeyboardInterrupt) as exc:
        report["error"] = str(exc) or "测试中断"
        log(report["error"])
    finally:
        report["overall"] = summarize(queries.values())
        report["by_node"] = {
            hex(n): summarize(q for q in queries.values() if q.node == n) for n in args.nodes
        }
        for stage in report["stages"]:
            stage.update(summarize(q for q in queries.values() if q.stage == stage["name"]))
        with (out / "requests.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(Query.__dataclass_fields__))
            writer.writeheader()
            writer.writerows(asdict(q) for q in queries.values())
        (out / "summary.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        for n in args.nodes:
            s = report["by_node"][hex(n)]
            pct = "无受理查询" if s["response_pct"] is None else f"{s['response_pct']}%"
            log(
                f"0x{n:02X} {AXES[n]}  {s['clean']}/{s['accepted']}  {pct}  "
                f"仅带标志 {s['flagged_only']}  无匹配 {s['no_match']}  SDK拒收 {s['rejected']}"
            )
        log(f"原始接收 {report['raw_rx']} 帧，记录 {out}")
    if not report["complete"]:
        return 2
    return int(report["overall"]["clean"] != report["overall"]["attempted"])


if __name__ == "__main__":
    raise SystemExit(main())
