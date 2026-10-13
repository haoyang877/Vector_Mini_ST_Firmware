"""单目标CAN-FD升级；只在显式CLI执行时打开硬件。"""

import argparse
import ctypes as C
import hashlib
import json
from pathlib import Path
import secrets
import struct
import time

from loader_proto import frame_bytes, parse_reply, ProtocolFrameError, crc32
from usbcan_adapter import CanFD, RxCAN, RxFD, TxFD

LAYOUT_ID = 0x4F564C31
HARDWARE_ID = 1
PARAM_ABI = 0x4F563130
APP_BASE = 0x08004000
CAPACITY = 0x18000


class LoaderBus(CanFD):
    """拥有显式选定的适配器和通道，关闭时只释放自己打开的资源。"""

    def send(self, channel, identifier, data):
        """发送标准FD数据帧；SDK成功只表示提交，不表示业务成功。"""
        if (
            channel not in self.channels
            or not 0 <= identifier <= 0x7FF
            or len(data) > 64
        ):
            raise ValueError("Invalid CAN target/frame")
        msg = TxFD()
        msg.frame.can_id = identifier
        msg.frame.length = len(data)
        msg.frame.flags = 1
        msg.frame.data[: len(data)] = data
        self._ok(
            self.dll.ZCAN_TransmitFD(self.channels[channel], C.byref(msg), 1),
            "transmit",
        )

    def receive(self, channel, timeout):
        """返回原始标准帧，扩展/远程帧不参与升级应答。"""
        rows = []
        for kind, method, wait in (
            (RxFD, "ZCAN_ReceiveFD", round(timeout * 1000)),
            (RxCAN, "ZCAN_Receive", 0),
        ):
            buffer = (kind * 64)()
            count = getattr(self.dll, method)(self.channels[channel], buffer, 64, wait)
            if count > 64:
                raise OSError("SDK receive failed")
            for entry in buffer[:count]:
                if entry.frame.can_id <= 0x7FF:
                    rows.append(
                        (
                            entry.frame.can_id,
                            bytes(entry.frame.data[: entry.frame.length]),
                        )
                    )
        return rows


class Client:
    """停止等待客户端；响应必须匹配本次ID/op/seq/session。"""

    def __init__(self, bus, channel, node):
        self.bus, self.channel, self.node = bus, channel, node
        self.seq = 0
        self.session = 0
        self.trace = []

    def request(self, op, payload=b"", timeout=1, retries=3, session=None):
        """有界重试相同字节，不能通过创建新请求重试擦写。"""
        self.seq += 1
        if self.seq > 65535:
            raise RuntimeError("Sequence exhausted")
        sid = self.session if session is None else session
        raw = frame_bytes(op, self.seq, sid, payload)
        for attempt in range(retries):
            self.bus.send(self.channel, 0x7D0 + self.node, raw)
            self.trace.append(
                {"op": op, "seq": self.seq, "session": sid, "attempt": attempt}
            )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                for identifier, data in self.bus.receive(self.channel, 0.01):
                    if identifier != 0x7E0 + self.node:
                        continue
                    try:
                        reply = parse_reply(data)
                    except ProtocolFrameError:
                        continue
                    if (reply["opcode"], reply["seq"], reply["session"]) != (
                        op,
                        self.seq,
                        sid,
                    ):
                        continue
                    if reply["result"] != 0:
                        raise RuntimeError(
                            f"Loader rejected op {op:#x}: {reply['result']}"
                        )
                    return reply["payload"][2:]
        raise TimeoutError(f"Loader op {op:#x} timed out")

    def query(self, op, expected_length):
        """用新nonce关联只读结果，不采信旧队列中的应答。"""
        nonce = secrets.token_bytes(4)
        payload = self.request(op, nonce, session=0)
        if len(payload) != expected_length or payload[:4] != nonce:
            raise RuntimeError("Malformed/stale query response")
        return payload[4:]

    def check_target(self, uid):
        """副作用前同时核对UID、节点、布局和兼容族。"""
        info = self.query(1, 36)
        version, base, capacity, layout, hardware, abi = struct.unpack(">6I", info[:24])
        if (base, capacity, layout, hardware, abi, info[24]) != (
            APP_BASE,
            CAPACITY,
            LAYOUT_ID,
            HARDWARE_ID,
            PARAM_ABI,
            self.node,
        ):
            raise RuntimeError("Loader target/layout mismatch")
        if self.query(3, 16) != uid:
            raise RuntimeError("Selected UID mismatch")
        self.parameter_crc = struct.unpack(">I", info[28:32])[0]
        return version

    def update(self, image, version, uid):
        """完整下载并提交；不自动重擦恢复，也不启动电机。"""
        validate_image(image)
        self.check_target(uid)
        self.session = secrets.randbelow(0xFFFFFFFF) + 1
        manifest = (
            struct.pack(
                ">IIIHHIII",
                len(image),
                crc32(image),
                version,
                1,
                0,
                LAYOUT_ID,
                HARDWARE_ID,
                PARAM_ABI,
            )
            + uid
        )
        if self.request(0x20, manifest) != struct.pack(">I", APP_BASE):
            raise RuntimeError("BEGIN response mismatch")
        self.request(0x21, timeout=20)
        offset = 0
        while offset < len(image):
            n = min(40, len(image) - offset, 2048 - offset % 2048)
            ack = self.request(
                0x22, struct.pack(">I", offset) + image[offset : offset + n]
            )
            if ack != struct.pack(">I", offset + n):
                raise RuntimeError("Offset ACK mismatch")
            offset += n
        if self.request(0x23, timeout=10) != struct.pack(">I", crc32(image)):
            raise RuntimeError("VERIFY CRC mismatch")
        activate_ack = True
        try:
            self.request(0x24)
        except TimeoutError:
            activate_ack = False
        return {
            "image_bytes": len(image),
            "image_crc32": crc32(image),
            "session": self.session,
            "activation_acknowledged": activate_ack,
        }


def validate_image(image):
    """在任何硬件操作前检查重定位镜像长度和Cortex-M向量。"""
    if len(image) < 8 or len(image) > CAPACITY or len(image) % 8:
        raise ValueError("Image size/alignment")
    sp, reset = struct.unpack("<II", image[:8])
    if sp % 8 or not 0x20000028 <= sp <= 0x20008000 or not reset & 1:
        raise ValueError("Invalid stack/Thumb vector")
    if not APP_BASE <= reset & ~1 < APP_BASE + len(image):
        raise ValueError("Image was not linked for this APP address")


def app_query(bus, channel, node, op, payload, predicate, timeout=1):
    """只接受当前请求的nonce/版本；函数本身不发送运动命令。"""
    deadline = time.monotonic() + timeout
    last_send = 0
    while time.monotonic() < deadline:
        if time.monotonic() - last_send > 0.1:
            bus.send(channel, (node << 8) | op, payload)
            last_send = time.monotonic()
        for identifier, data in bus.receive(channel, 0.01):
            if identifier == (node << 8) | op and predicate(data):
                return data
    raise TimeoutError(f"APP query {op:#x} timed out")


def app_ready(bus, channel, node, version, uid):
    """核对新版本和同板UID；不将GET_MODE=0单独当作升级成功。"""
    app_query(
        bus,
        channel,
        node,
        0x6F,
        b"\0" * 4,
        lambda data: data == struct.pack(">I", version),
        timeout=5,
    )
    nonce = secrets.token_bytes(4)
    app_query(
        bus, channel, node, 0x63, nonce, lambda data: data == nonce + uid, timeout=2
    )


def app_parameter_crc(bus, channel, node):
    """用nonce读取整个16 KiB参数区CRC，供独立上位机核对。"""
    nonce = secrets.token_bytes(4)
    reply = app_query(
        bus,
        channel,
        node,
        0x7C,
        nonce,
        lambda data: len(data) == 8 and data[:4] == nonce,
        timeout=2,
    )
    return struct.unpack(">I", reply[4:])[0]


def main():
    """显式单设备升级CLI；多个设备由序列号与通道选择。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dll", type=Path, required=True)
    parser.add_argument("--adapter-serial", required=True)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--channel", type=int, choices=(0, 1), required=True)
    parser.add_argument("--node", type=int, required=True)
    parser.add_argument("--uid", required=True)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--enter-app", action="store_true")
    parser.add_argument("--out", type=Path, default=Path("outputs/loader_update"))
    args = parser.parse_args()
    uid = bytes.fromhex(args.uid)
    if len(uid) != 12 or not 0 <= args.node <= 7:
        raise ValueError("UID/node invalid")
    manifest = json.loads((args.package / "manifest.json").read_text(encoding="utf-8"))
    image = (args.package / "app.bin").read_bytes()
    validate_image(image)
    if (
        manifest["app_base"],
        manifest["layout_id"],
        manifest["hardware_id"],
        manifest["param_abi"],
        manifest["image_bytes"],
        manifest["image_crc32"],
        manifest["image_sha256"],
    ) != (
        APP_BASE,
        LAYOUT_ID,
        HARDWARE_ID,
        PARAM_ABI,
        len(image),
        crc32(image),
        hashlib.sha256(image).hexdigest(),
    ):
        raise ValueError("Package manifest mismatch")
    bus = LoaderBus(args.dll, (args.channel,), args.device_index, args.adapter_serial)
    client = Client(bus, args.channel, args.node)
    args.out.mkdir(parents=True, exist_ok=True)
    report = {
        "status": "FAIL",
        "node": args.node,
        "uid": args.uid,
        "adapter": bus.info,
        "channel": args.channel,
    }
    try:
        if args.enter_app:
            nonce = secrets.token_bytes(4)
            app_query(
                bus,
                args.channel,
                args.node,
                0x63,
                nonce,
                lambda data: data == nonce + uid,
            )
            # 此处进入命令由APP停机/关相后处理，缺少回复即中止。
            app_query(
                bus,
                args.channel,
                args.node,
                0x66,
                b"BOOT",
                lambda data: data == b"\0" * 4,
            )
            time.sleep(0.1)
        report.update(client.update(image, manifest["app_version"], uid))
        app_ready(bus, args.channel, args.node, manifest["app_version"], uid)
        after_crc = app_parameter_crc(bus, args.channel, args.node)
        report["parameter_crc_before"] = client.parameter_crc
        report["parameter_crc_after"] = after_crc
        if after_crc != client.parameter_crc:
            raise RuntimeError("Protected parameter region changed")
        report["status"] = "PASS"
    finally:
        report["requests"] = client.trace
        (args.out / "summary.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        bus.close()
    print(
        json.dumps(
            {key: value for key, value in report.items() if key != "requests"}, indent=2
        )
    )


if __name__ == "__main__":
    main()
