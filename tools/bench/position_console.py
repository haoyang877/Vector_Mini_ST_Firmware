"""AT32F456 单轴位置台架手动控制台：使用普通 CAN，不占用 RTT 调试器。"""

import argparse
import json
import math
import os
import queue
import struct
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "host_app"))
from ports.controlcanfd import ControlCanFd  # noqa: E402
from upgrade_client import decode_reply, encode_frames  # noqa: E402


class Console:
    """在主线程串行处理 CAN；输入线程只收集用户命令。"""

    def __init__(self, bus, config):
        self.bus = bus
        self.node = config["node"]
        self.uid = bytes.fromhex(config["uid"])
        self.zero = config["zero_motor_mrad"]
        self.ratio = config["ratio"]
        self.limit = config["max_output_deg"]
        self.sequence = int(time.monotonic() * 1000) % 65535
        self.active = False
        self.goal = None

    def request(self, kind, make_payload):
        """发送一个关联请求，严格匹配回复序号和设备。"""
        self.sequence = self.sequence % 65535 + 1
        for identifier, data in encode_frames(
            self.node, kind, make_payload(self.sequence), self.sequence
        ):
            self.bus.send(identifier, data)
        deadline = time.monotonic() + 0.22
        while time.monotonic() < deadline:
            frame = self.bus.receive(max(0.001, deadline - time.monotonic()))
            if frame:
                try:
                    return decode_reply(*frame, self.node, kind, self.sequence)
                except ValueError:
                    continue
        raise TimeoutError(f"CAN {kind} 应答超时")

    def status(self):
        """读取状态、故障及机械位置；位置单位 mrad。"""
        p = self.request(108, lambda _: b"")
        fields = struct.unpack("<HHI4iH8h3B", p)
        if fields[0] or fields[-3] != self.node:
            raise RuntimeError("位置状态无效")
        return fields[-2], fields[-1], fields[2], fields[4] if fields[7] & 2 else None

    def show(self):
        state, mode, fault, position = self.status()
        angle = (
            "未知"
            if position is None
            else f"{math.degrees((position - self.zero) * 0.001) / self.ratio:+.3f}°"
        )
        print(f"state={state} mode={mode} fault=0x{fault:x} 输出侧={angle}")

    def accepted(self, kind, payload):
        reply = self.request(kind, lambda _: payload)
        result, detail = struct.unpack("<HH", reply)
        if result not in (0, 1):
            raise RuntimeError(f"CAN {kind} 拒绝：result={result} detail=0x{detail:04x}")

    def start(self):
        state, mode, fault, position = self.status()
        if (state, mode, fault) != (5, 0, 0) or self.active:
            raise RuntimeError("启动需要 IDLE、零故障")
        if position is None:
            raise RuntimeError("编码器尚无有效位置")
        if abs(math.degrees((position - self.zero) * 0.001) / self.ratio) > self.limit:
            raise RuntimeError("当前位置在本次台架行程之外")
        self.goal = position
        self.active = True
        self.accepted(117, bytes([3, 0, 0, 0]))
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            state, mode, fault, _ = self.status()
            if fault:
                break
            if (state, mode) == (6, 3):
                self.renew()
                print("位置模式已运行，保持启动位置")
                return
            time.sleep(0.003)
        raise RuntimeError("未进入位置模式")

    def renew(self):
        self.accepted(116, struct.pack("<B3xiiii", 3, self.goal, 0, 0, 0))
        state, mode, fault, _ = self.status()
        if (state, mode, fault) != (6, 3, 0):
            raise RuntimeError(f"位置模式已退出：state={state} mode={mode} fault=0x{fault:x}")

    def move(self, angle):
        if not self.active or not math.isfinite(angle) or abs(angle) > self.limit:
            raise ValueError(f"先 start，目标须在 ±{self.limit:g}° 内")
        self.goal = self.zero + round(math.radians(angle) * self.ratio * 1000)
        self.renew()
        print(f"目标已受理：输出侧 {angle:+g}°")

    def stop(self):
        self.accepted(110, b"")
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            state, mode, fault, _ = self.status()
            if state in (5, 8) and mode == 0:
                self.active = False
                print(f"已停机：state={state} fault=0x{fault:x}")
                return
            time.sleep(0.01)
        raise RuntimeError("STOP 未确认，请现场核查")

    def serve(self):
        info = self.request(1, lambda seq: struct.pack("<IIHH", 0, seq, 0, 0))
        if info[:12] != self.uid or len(info) != 32:
            raise RuntimeError("设备 UID 与台架配置不符")
        self.show()
        print("命令：start | move <输出侧目标角度> | status | stop | quit。连接后保持 idle。")
        commands = queue.Queue()

        def read_commands():
            for line in sys.stdin:
                commands.put(line.strip())
            commands.put("quit")

        threading.Thread(target=read_commands, daemon=True).start()
        try:
            while True:
                try:
                    line = commands.get(timeout=0.08)
                except queue.Empty:
                    if self.active:
                        self.renew()
                    continue
                parts = line.split()
                if parts == ["quit"]:
                    return
                if parts == ["start"]:
                    self.start()
                elif parts == ["stop"]:
                    self.stop()
                elif parts == ["status"]:
                    self.show()
                elif len(parts) == 2 and parts[0] == "move":
                    self.move(float(parts[1]))
                elif parts:
                    print("可用命令：start / move <角度> / status / stop / quit")
        finally:
            if self.active:
                self.stop()


def main():
    """显式加载已核验台架配置；仅 start 指令使能。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bench", required=True, type=Path)
    parser.add_argument("--dll", type=Path, help="覆盖台架配置中的 USB-CAN SDK 路径")
    args = parser.parse_args()
    config = json.loads(args.bench.read_text(encoding="utf-8-sig"))
    dll = (args.dll or Path(config["can_dll"])).resolve()
    with os.add_dll_directory(str(dll.parent)), ControlCanFd(dll) as bus:
        Console(bus, config).serve()


if __name__ == "__main__":
    main()
