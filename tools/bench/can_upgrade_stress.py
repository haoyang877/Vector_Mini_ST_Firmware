"""五轴 CAN APP 升级压力测试，每轮按节点依次完整升级并记录。"""

import hashlib
import json
import os
import secrets
import struct
import sys
import time
from pathlib import Path

from five_axis_can_stability import ControlCanFd, controlcan_dll_path
from upgrade_client import UpgradeClient, UpgradePackage, decode_reply, encode_frames


def read_node_uids(bus, node):
    """单播读取身份首页，保留同一请求窗口内所有通过帧校验的 UID。"""
    sequence = secrets.randbits(16)
    payload = struct.pack("<IIHH", 0, sequence, 0, 0)
    bus.send(*encode_frames(node, 1, payload, sequence)[0])
    deadline = time.monotonic() + 0.35
    uids = set()
    while (remaining := deadline - time.monotonic()) > 0:
        frame = bus.receive(remaining)
        if frame is None:
            break
        try:
            page = decode_reply(*frame, node, 1, sequence)
        except ValueError:
            continue
        if len(page) == 32 and page[:12] not in (bytes(12), b"\xff" * 12):
            uids.add(page[:12])
    return uids


def scan_upgrade_motors(bus, stopped, stage, nodes=(3, 4, 5, 6, 7)):
    """读取本批五轴的实时身份，APP 另读状态和支持的活动型号配置。"""
    motors, issues = [], []
    for node in nodes:
        if stopped():
            break
        stage(f"扫描节点 {node}")
        uids = read_node_uids(bus, node)
        if len(uids) > 1:
            issues.append(f"节点 {node} 有多个 UID，请检查重复节点地址。")
            continue
        if not uids:
            continue
        uid = uids.pop()
        client = UpgradeClient(bus, node, uid)
        client.sequence = secrets.randbits(16)
        try:
            page0, page1, _ = client.identify()
            motor = {
                "node": node,
                "uid": uid.hex(),
                "role": page1[22],
                "boot_state": page1[23],
                "app_version": struct.unpack_from("<I", page0, 20)[0],
                "loader_version": struct.unpack_from("<I", page0, 24)[0],
            }
            if motor["role"] == 1:
                state = client.request(108)
                if len(state) != 45 or state[:2] != bytes(2) or state[42] != node:
                    raise ValueError("状态回复无效")
                motor.update(state=state[43], faults=struct.unpack_from("<I", state, 4)[0])
            motors.append(motor)
        except (TimeoutError, ValueError) as exc:
            issues.append(f"节点 {node}（UID {uid.hex()}）未完成识别：{exc}")
            continue
        if motor["role"] == 1:
            try:
                caps = client.request(2, bytes(12), timeout_s=0.08)
                if len(caps) == 16 and int.from_bytes(caps[:4], "little") & (1 << 17):
                    config = client.request(190, bytes(4), timeout_s=0.08)
                    if len(config) == 44 and config[:5] == bytes(4) + b"\x01" and config[6] == node:
                        motor["model"] = {1: "HT8115-4", 2: "YQ-5708", 3: "YQ-5740"}.get(config[5])
            except TimeoutError:
                pass  # 旧板仍可凭身份与状态识别，型号来源在界面标注。
    return motors, issues


def check_upgrade_bus(bus):
    """交接或擦除前被动监听，检查升级使用的主站 2 是否正向电机发送协议请求。"""
    deadline = time.monotonic() + 1.5
    while (remaining := deadline - time.monotonic()) > 0:
        frame = bus.receive(remaining)
        if frame is None:
            return
        identifier, data = frame
        node = (identifier >> 8) & 0xFF
        if (
            identifier & 0x03FF00FF == 0x00EF0002
            and len(data) >= 18
            and data[:3] == b"\x5a\xa5\x01"
            and not data[3] & 0x10
            and data[4] == 2
            and data[5] == node
        ):
            kind = int.from_bytes(data[6:8], "little")
            raise RuntimeError(
                f"主站 2 正向节点 {node} 发送 Type{kind}。请暂停其他主站轮询后重试；本轮尚未擦写。"
            )


def run_upgrade_test(settings, out, events, stop):
    rounds = settings["rounds"]
    # 46 B/片，扣除 18 B 事务字段并按 8 B 对齐，两片可传 72 B。
    block_limit = 24 if settings.get("single_frame_writes") else 72
    report = {
        "profile": "can_upgrade",
        "planned": 0,
        "rounds_per_motor": rounds,
        "motors": {},
        "write_block_limit": block_limit,
        "bus": settings["bus"],
        "attempted": 0,
        "passed": 0,
        "failed": 0,
        "status": "进行中",
    }
    start = time.monotonic()

    def save():
        report["elapsed_s"] = round(time.monotonic() - start, 3)
        tmp = out / "summary.tmp"
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(out / "summary.json")

    def stage(name):
        report["stage"] = name
        motor["stage"] = name
        save()
        events.put(("axis_status", (node, f"第 {number}/{rounds} 轮 · {name}")))
        events.put(("stage", f"第 {number}/{rounds} 轮 · 节点 {node} · {name}"))

    try:
        save()
        if type(rounds) is not int or rounds < 1:
            raise ValueError("每台升级次数须为正整数")
        targets = settings.get("targets")
        if targets is None:
            targets = [{"node": settings["node"], "uid": settings["uid"]}]
        if not 1 <= len(targets) <= 5:
            raise ValueError("请勾选要升级的在线电机")
        selected = {}
        for target in targets:
            node = target["node"]
            uid = bytes.fromhex(target["uid"])
            if type(node) is not int or node not in range(3, 8) or node in selected:
                raise ValueError("目标节点必须为互不重复的五轴节点")
            if len(uid) != 12 or uid in (bytes(12), b"\xff" * 12) or uid in selected.values():
                raise ValueError("目标 UID 无效或重复，请重新扫描")
            selected[node] = uid
            report["motors"][str(node)] = {"uid": uid.hex(), "passed": 0, "failed": 0}
        if len(selected) == 1:
            report.update(node=node, uid=uid.hex())
        planned = report["planned"] = rounds * len(selected)
        resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        contract = (
            resource / "yg_esc_v1_at32f456.json"
            if getattr(sys, "frozen", False)
            else "yg_esc_v1_at32f456"
        )
        package = UpgradePackage.load(Path(settings["package"]), contract)
        report.update(
            image_bytes=len(package.image),
            image_sha256=hashlib.sha256(package.image).hexdigest(),
            crc32=f"{package.crc32:08x}",
            version=package.version,
        )
        dll = controlcan_dll_path(resource)
        with (
            os.add_dll_directory(str(dll.parent)),
            ControlCanFd(dll, **settings["bus"]) as bus,
            (out / "rounds.jsonl").open("x", encoding="utf-8") as log,
        ):
            schedule = ((n, node) for n in range(1, rounds + 1) for node in sorted(selected))
            for number, node in schedule:
                if stop.is_set():
                    break
                uid = selected[node]
                motor = report["motors"][str(node)]
                motor["round"] = number
                report["attempted"] += 1
                row = {
                    "node": node,
                    "uid": uid.hex(),
                    "round": number,
                    "success": False,
                    "bytes_written": 0,
                }
                round_start = time.monotonic()
                last_percent = -1

                def progress(written, total, *, row=row, node=node, number=number):
                    nonlocal last_percent
                    row["bytes_written"] = written
                    percent = written * 100 // total
                    if percent != last_percent:
                        events.put(("upgrade_progress", (report["attempted"], planned, percent)))
                        events.put(
                            ("axis_status", (node, f"第 {number}/{rounds} 轮 · 下载 {percent}%"))
                        )
                        last_percent = percent

                client = UpgradeClient(
                    bus,
                    node,
                    uid,
                    progress=progress,
                    on_stage=stage,
                    write_block_limit=block_limit,
                )
                try:
                    stage("监听其他主站轮询（1.5 秒）")
                    check_upgrade_bus(bus)
                    if read_node_uids(bus, node) != {uid}:
                        motor["identity_changed_or_missing"] = True
                        raise ValueError(
                            "节点未响应、UID 已变或地址冲突，请重新扫描。本轮尚未擦写。"
                        )
                    result = client.run(package)
                    row.update(
                        success=result["success"],
                        uid=result["uid"],
                        version=result["version"],
                        crc32=result["crc32"],
                    )
                    report["passed"] += 1
                    motor["passed"] += 1
                    stage("APP 已持久确认")
                except Exception as exc:
                    report["failed"] += 1
                    motor.update(failed=1, error=str(exc))
                    row.update(error=str(exc), stage=report.get("stage"), trace=client.trace)
                    raise
                finally:
                    row["seconds"] = round(time.monotonic() - round_start, 3)
                    report["last_round_bytes"] = row["bytes_written"]
                    row["timing"] = client.timing_summary()
                    log.write(json.dumps(row, ensure_ascii=False) + "\n")
                    log.flush()
                    save()
        report["status"] = "通过" if report["passed"] == planned else "已停止"
    except Exception as exc:
        report.update(status="失败", fatal=str(exc) or type(exc).__name__)
    finally:
        save()
        events.put(("done", report))
