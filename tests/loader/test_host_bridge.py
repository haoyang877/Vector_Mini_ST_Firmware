"""真实Python客户端连接真实C核心的stdio替身，不打开任何硬件。"""

from pathlib import Path
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from loader_updater import Client  # noqa: E402 - 按仓库目录加载离线客户端


class NativeBus:
    """只将CAN载荷交给离线C进程，模拟一次丢ACK和错误关联回复。"""

    def __init__(self, exe):
        self.process = subprocess.Popen(
            [str(exe), "server"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.pending = []
        self.dropped = False

    def send(self, channel, identifier, raw):
        if channel != 0 or identifier != 0x7D4:
            raise RuntimeError("Wrong host target")
        self.process.stdin.write(raw.hex() + "\n")
        self.process.stdin.flush()
        reply = bytes.fromhex(self.process.stdout.readline().strip())
        if raw[2] == 0x22 and not self.dropped:
            self.dropped = True
            return
        self.pending.append((0x7E4, reply))

    def receive(self, channel, timeout):
        if channel != 0:
            raise RuntimeError("Wrong channel")
        if self.pending:
            result, self.pending = self.pending, []
            return result
        time.sleep(min(timeout, 0.001))
        return []

    def valid(self):
        self.process.stdin.write("VALID\n")
        self.process.stdin.flush()
        return self.process.stdout.readline().strip() == "1"

    def close(self):
        self.process.stdin.close()
        if self.process.wait(timeout=5) != 0:
            raise RuntimeError("C bridge failed")


def main():
    """校验跨语言清单、偏移、重试和真实C提交结果。"""
    exe = Path(sys.argv[1]).resolve()
    bus = NativeBus(exe)
    try:
        client = Client(bus, 0, 4)
        image = struct.pack("<II", 0x20007000, 0x08004011) + bytes(range(56))
        result = client.update(image, 0x00050000, bytes(range(12)))
        if not result["activation_acknowledged"] or not bus.dropped or not bus.valid():
            raise RuntimeError("C/Python update or retry did not complete")
        print(
            "PASS actual Python/C update: identity, manifest, dropped PROGRAM ACK, retry and valid boot record"
        )
    finally:
        bus.close()


if __name__ == "__main__":
    main()
