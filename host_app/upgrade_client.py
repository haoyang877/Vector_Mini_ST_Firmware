"""独立CAN FD升级客户端；导入与包检查不访问硬件，硬件入口须显式提供台架条件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import secrets
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

ROOT = Path(__file__).resolve().parents[1]
TARGET_CONTRACT = Path(__file__).resolve().parent / "contracts/vector_mini_st_g431.json"
if not TARGET_CONTRACT.is_file():
    TARGET_CONTRACT = ROOT / "shared/targets/vector_mini_st_g431.json"
PROFILE = 0x53593549


def crc8(data: bytes) -> int:
    """公司帧CRC8，初值0、多项式0x9B。"""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x9B if value & 0x80 else 0)) & 0xFF
    return value


def crc16(data: bytes) -> int:
    """公司帧CRC16，初值FFFF、多项式BAAD。"""
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0xBAAD if value & 0x8000 else 0)) & 0xFFFF
    return value


def encode_frames(node: int, kind: int, payload: bytes, sequence: int) -> list[tuple[int, bytes]]:
    """主站2发送公司帧；长消息片号从0开始，只在尾片请求回复。"""
    if not 0 <= node < 255 or len(payload) > 146:
        raise ValueError("非法目标节点或超出升级逻辑消息容量")
    chunks = [payload[i : i + 46] for i in range(0, len(payload), 46)] or [b""]
    frames = []
    for index, chunk in enumerate(chunks):
        if len(chunks) == 1:
            flags, seq = 0x20, sequence
        else:
            flags = 0x40 if index == 0 else 0xE0 if index == len(chunks) - 1 else 0x80
            seq = index
        header = struct.pack("<HBBBBHHIB", 0xA55A, 1, flags, 2, node, kind, seq, len(chunk), 0)
        data = header + bytes([crc8(header)]) + chunk
        data += struct.pack("<H", crc16(data))
        padded = next(n for n in (20, 24, 32, 48, 64) if n >= len(data))
        frames.append(((3 << 26) | (0xEF << 16) | (node << 8) | 2, data.ljust(padded, b"\x00")))
    return frames


def decode_reply(identifier: int, data: bytes, node: int, kind: int, sequence: int) -> bytes:
    """严格检查CAN ID、来源、序号、长度、补零与两层CRC，拒绝不关联回复。"""
    if len(data) not in (20, 24, 32, 48, 64):
        raise ValueError("非法CAN FD长度")
    magic, version, flags, src, dst, actual_kind, seq, length, reserved = struct.unpack_from(
        "<HBBBBHHIB", data
    )
    end = 16 + length
    if (
        identifier != (3 << 26) | (0xEF << 16) | (2 << 8) | node
        or (magic, version, flags, src, dst, actual_kind, seq, reserved)
        != (0xA55A, 1, 0x10, node, 2, kind, sequence, 0)
        or length > 46
        or end + 2 > len(data)
        or len(data) != next(n for n in (20, 24, 32, 48, 64) if n >= end + 2)
        or crc8(data[:15]) != data[15]
        or crc16(data[:end]) != struct.unpack_from("<H", data, end)[0]
        or any(data[end + 2 :])
    ):
        raise ValueError("回复帧身份、长度或CRC不匹配")
    return data[16:end]


@dataclass(frozen=True)
class UpgradePackage:
    """已完整核对的部署输入，不能以JSON元数据代替镜像和清单校验。"""

    image: bytes
    manifest: bytes
    version: int
    crc32: int
    token: int
    target: tuple[int, ...]
    loader_version: int

    @classmethod
    def load(cls, directory: Path) -> UpgradePackage:
        """严格检查冻结目标、描述符、向量、CRC和配套文件SHA256。"""
        image = (directory / "image.bin").read_bytes()
        manifest = (directory / "manifest.bin").read_bytes()
        metadata = json.loads((directory / "package.json").read_text(encoding="utf-8"))
        constants = json.loads(TARGET_CONTRACT.read_text(encoding="utf-8"))["constants"]

        def value(key: str) -> int:
            return int(constants[key].removesuffix("U"), 0)

        if len(manifest) != 96 or len(image) < 608 or len(image) % 8:
            raise ValueError("镜像或清单长度非法")
        fields = struct.unpack(">11I2H", manifest[:48])
        magic, profile, layout, mcu, product, hw, version, token, length, role, abi, low, high = (
            fields
        )
        target = (
            mcu,
            layout,
            abi,
            value("TARGET_SCHEMA"),
            value("TARGET_APP_BASE"),
            value("TARGET_APP_BYTES"),
            value("TARGET_LOADER_BYTES"),
            value("TARGET_IDENTITY_BASE"),
        )
        if (
            (magic, profile, layout, mcu, product, hw, abi, role)
            != (
                0x494D3331,
                PROFILE,
                value("TARGET_LAYOUT"),
                value("TARGET_MCU"),
                value("TARGET_PRODUCT"),
                value("TARGET_HARDWARE"),
                value("TARGET_PARAM_ABI"),
                1,
            )
            or version == 0
            or token == 0
            or length != len(image)
            or length > target[5]
            or not low <= target[3] <= high
            or any(manifest[52:])
        ):
            raise ValueError("升级包目标、版本、参数ABI或角色与冻结配置不匹配")
        offset = value("TARGET_DESCRIPTOR_OFFSET")
        desc = struct.unpack(">10I2HII", image[offset : offset + 52])
        crc = struct.unpack_from(">I", manifest, 48)[0]
        expected = (
            0x59494431,
            3,
            profile,
            layout,
            mcu,
            product,
            hw,
            version,
            role,
            abi,
            low,
            high,
            length,
            crc,
        )
        normalized = bytearray(image)
        normalized[offset + 48 : offset + 52] = bytes(4)
        sp, pc = struct.unpack_from("<II", image)
        if (
            desc != expected
            or any(image[offset + 52 : offset + 64])
            or image[offset + 64 : offset + 96] != b"\xff" * 24 + bytes(8)
            or zlib.crc32(normalized) != crc
            or not value("TARGET_RAM_BASE") < sp <= value("TARGET_RAM_END")
            or sp % 8
            or not pc & 1
            or not target[4] + offset + 96 <= (pc & ~1) < target[4] + length
            or metadata.get("format") != "loader-image-v3-crc32"
            or metadata.get("image_sha256") != hashlib.sha256(image).hexdigest()
            or metadata.get("manifest_sha256") != hashlib.sha256(manifest).hexdigest()
            or metadata.get("loader_version") != value("TARGET_LOADER_VERSION")
        ):
            raise ValueError("描述符、镜像CRC、向量或文件哈希不匹配")
        return cls(image, manifest, version, crc, token, target, value("TARGET_LOADER_VERSION"))


class CanTransport(Protocol):
    """真实CAN适配器与离线模型共用的有界I/O边界。"""

    def send(self, identifier: int, data: bytes) -> None:
        """发送一帧；入队失败抛出异常，不宣称设备已处理。"""
        ...

    def receive(self, timeout_s: float) -> tuple[int, bytes] | None:
        """在指定秒数内返回一帧，超时返回None。"""
        ...


class UpgradeClient:
    """单目标升级所有者，有限重试、不续传旧会话、不自动恢复电机运行。"""

    def __init__(
        self,
        transport: CanTransport,
        node: int,
        uid: bytes,
        progress: Callable[[int, int], None] | None = None,
    ) -> None:
        if not 0 <= node < 255 or len(uid) != 12:
            raise ValueError("升级须指定0..254节点与12B实际UID")
        self.transport, self.node, self.uid = transport, node, uid
        self.sequence = 0
        self.trace: list[dict] = []
        self.binding = b""
        self.progress = progress
        self.request_timings: list[dict] = []

    def request(
        self,
        kind: int,
        payload: bytes = b"",
        *,
        sequence: int | None = None,
        timeout_s: float = 0.25,
    ) -> bytes:
        """整条逻辑消息至多重发四次；重试保持序号、nonce和内容一致。"""
        if sequence is None:
            self.sequence = (self.sequence + 1) & 0xFFFF
            sequence = self.sequence
        frames = encode_frames(self.node, kind, payload, sequence)
        reply_sequence = len(frames) - 1 if len(frames) > 1 else sequence
        started = time.perf_counter()
        send_s = 0.0
        attempts = 0
        for attempt in range(4):
            attempts += 1
            send_started = time.perf_counter()
            for identifier, data in frames:
                self.transport.send(identifier, data)
            send_s += time.perf_counter() - send_started
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                frame = self.transport.receive(max(0.0, deadline - time.monotonic()))
                if frame is None:
                    break
                try:
                    response = decode_reply(*frame, self.node, kind, reply_sequence)
                except ValueError:
                    continue
                self.trace.append({"type": kind, "attempt": attempt, "reply": response.hex()})
                self._record_timing(kind, started, send_s, attempts, len(frames), True)
                return response
        self._record_timing(kind, started, send_s, attempts, len(frames), False)
        raise TimeoutError(f"Type{kind}回复超时；停止升级，检查CAN与Loader驻留状态")

    def _record_timing(
        self,
        kind: int,
        started: float,
        send_s: float,
        attempts: int,
        frames_per_attempt: int,
        success: bool,
    ) -> None:
        """记录主机观察耗时；send包含适配器或外层检查，不能解释为总线占用。"""
        elapsed_s = time.perf_counter() - started
        self.request_timings.append(
            {
                "type": kind,
                "success": success,
                "attempts": attempts,
                "frames_sent": attempts * frames_per_attempt,
                "elapsed_s": elapsed_s,
                "transport_send_s": send_s,
                "reply_wait_s": max(0.0, elapsed_s - send_s),
            }
        )

    def timing_summary(self) -> dict:
        """按升级命令汇总成功和超时请求；等待含协议重试与Python调度开销。"""
        by_type: dict[str, dict] = {}
        for item in self.request_timings:
            group = by_type.setdefault(
                str(item["type"]),
                {
                    "requests": 0,
                    "failed_requests": 0,
                    "retries": 0,
                    "frames_sent": 0,
                    "elapsed_s": 0.0,
                    "transport_send_s": 0.0,
                    "reply_wait_s": 0.0,
                },
            )
            group["requests"] += 1
            group["failed_requests"] += int(not item["success"])
            group["retries"] += item["attempts"] - 1
            for key in ("frames_sent", "elapsed_s", "transport_send_s", "reply_wait_s"):
                group[key] += item[key]
        return {
            "clock": "host_perf_counter",
            "by_type": by_type,
            "note": "send包含传输适配器和外层检查；不是纯CAN总线耗时",
        }

    def identify(self) -> tuple[bytes, bytes, tuple[int, ...]]:
        """查询三页并核对实际UID与合法角色，拒绝混入旧设备回复。"""
        pages = [self.request(1, struct.pack("<IIHH", 0, page, page, 0)) for page in range(3)]
        if any(len(page) != 32 for page in pages) or pages[0][:12] != self.uid:
            raise ValueError("设备UID不匹配或维护页不完整")
        if pages[1][21] != self.node or pages[1][22] not in (1, 2):
            raise ValueError("设备角色或CAN ID不匹配")
        return pages[0], pages[1], struct.unpack("<8I", pages[2])

    def hold_rescue_window(self) -> None:
        """10s内反复发空STATUS抓住50ms复位窗口；只驻留、不擦写，随后仍须严格身份检查。"""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                reply = self.request(53, timeout_s=0.01)
                if len(reply) == 40 and reply[:4] == bytes(4) and reply[17] & 6 == 6:
                    return
            except TimeoutError:
                pass
            time.sleep(0.002)
        raise TimeoutError("未抓到Loader启动窗口；停止操作，检查复位、节点与CAN连接")

    def status(self, kind: int = 53, extra: bytes = b"") -> tuple[int, int]:
        """校验升级回复的精确会话和镜像身份，再返回阶段与持久写入偏移。"""
        payload = self.request(kind, self.binding + extra)
        if len(payload) != 40:
            raise ValueError("升级回复长度错误")
        result, detail = struct.unpack_from("<HH", payload)
        if result not in (0, 1):
            raise RuntimeError(f"升级被拒绝：Type{kind} result={result} detail={detail}")
        nonce, session = struct.unpack_from("<QI", payload, 4)
        if self.binding and struct.pack("<QI", nonce, session) != self.binding:
            raise ValueError("升级回复会话不匹配")
        if payload[17] & 6 != 6 or payload[17] & ~7:
            raise ValueError("设备未声明当前nonce/完整性协议能力")
        if self.binding and (
            struct.unpack_from("<I", payload, 28)[0] != self.package.token
            or struct.unpack_from("<I", payload, 32)[0] != self.package.version
            or struct.unpack_from("<I", payload, 24)[0] != len(self.package.image)
        ):
            raise ValueError("升级回复镜像身份不匹配")
        return payload[16], struct.unpack_from("<I", payload, 20)[0]

    def run(self, package: UpgradePackage) -> dict:
        """核对设备后全擦下载，激活后核对新APP并显式确认；成功不发送电机使能。"""
        self.package = package
        # 先用无副作用STATUS固定Loader驻留，避免三页查询跨越50ms窗口后角色发生变化。
        # APP未开放53时只等待短窗口，然后走56停机交接。
        try:
            self.request(53, timeout_s=0.025)
        except TimeoutError:
            pass
        page0, page1, target = self.identify()
        self.check_target(page0, page1, target)
        nonce = secrets.randbits(64) or 1
        if page1[22] == 1:
            # 56相同seq与nonce轮询唯一停机事务，READY精确TX完成由APP自己复核。
            self.sequence = (self.sequence + 1) & 0xFFFF
            enter_sequence = self.sequence
            deadline = time.monotonic() + 1.0
            while True:
                try:
                    response = self.request(56, struct.pack("<Q", nonce), sequence=enter_sequence)
                except TimeoutError:
                    # READY已经物理发送并复位，但主机丢失回复；只凭相同UID的配套Loader恢复。
                    page0, page1, target = self.identify()
                    self.check_target(page0, page1, target)
                    if page1[22] != 2:
                        raise
                    break
                if len(response) != 4 or struct.unpack_from("<H", response)[0] != 1:
                    raise RuntimeError("APP拒绝Loader交接；保持停机，禁止继续")
                if struct.unpack_from("<H", response, 2)[0] == 0:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("APP未确认停止与关相，必须人工检查")
                time.sleep(0.01)
            time.sleep(0.1)
            page0, page1, target = self.identify()
            self.check_target(page0, page1, target)
            if page1[22] != 2:
                raise RuntimeError("交接后未识别到Loader")
        response = self.request(50, struct.pack("<Q", nonce) + package.manifest)
        if len(response) != 40 or struct.unpack_from("<H", response)[0] not in (0, 1):
            raise RuntimeError("Loader拒绝BEGIN，未进入下载")
        actual_nonce, session = struct.unpack_from("<QI", response, 4)
        block = struct.unpack_from("<H", response, 18)[0]
        if actual_nonce != nonce or session == 0 or not 8 <= block <= 128 or block % 8:
            raise ValueError("BEGIN回复绑定或写入能力非法")
        self.binding = struct.pack("<QI", nonce, session)
        deadline = time.monotonic() + 5.0
        while self.status()[0] != 3:
            if time.monotonic() >= deadline:
                raise TimeoutError("整槽擦除未完成")
            time.sleep(0.01)
        for offset in range(0, len(package.image), block):
            data = package.image[offset : offset + block]
            phase, written = self.status(51, struct.pack("<IH", offset, len(data)) + data)
            if phase != 3 or written != offset + len(data):
                raise ValueError("写入回复偏移异常，不继续下载")
            if self.progress is not None:
                self.progress(written, len(package.image))
        self.status(52)
        deadline = time.monotonic() + 5.0
        while self.status()[0] != 5:
            if time.monotonic() >= deadline:
                raise TimeoutError("完整CRC校验未完成")
            time.sleep(0.01)
        try:
            self.status(54)
        except TimeoutError:
            # 激活回复可能已发出但主机未收到；只能以相同UID的新APP版本/CRC和确认判成功。
            pass
        self.binding = b""
        time.sleep(0.1)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            page0, page1, target = self.identify()
            self.check_target(page0, page1, target)
            if (
                page1[22] != 1
                or struct.unpack_from("<I", page0, 20)[0] != package.version
                or struct.unpack_from("<I", page1, 24)[0] != package.crc32
            ):
                raise RuntimeError("激活后未读到目标APP；留Loader救援，不宣称成功")
            reply = self.request(57, struct.pack("<II", package.version, package.crc32))
            if len(reply) == 4 and reply == bytes(4):
                _, confirmed, _ = self.identify()
                if confirmed[22:24] == bytes((1, 1)):
                    return {
                        "success": True,
                        "uid": self.uid.hex(),
                        "version": package.version,
                        "crc32": f"{package.crc32:08x}",
                        "trace": self.trace,
                        "timing": self.timing_summary(),
                    }
            time.sleep(0.05)
        raise TimeoutError("新APP未完成健康与持久确认，等待看门狗退回Loader")

    def check_target(self, page0: bytes, page1: bytes, target: tuple[int, ...]) -> None:
        """每次跳转后核对三端配套版本与几何；读不到能力即拒绝擦写。"""
        product, hw, api = struct.unpack_from("<IHH", page0, 12)
        expected_product, expected_hw = struct.unpack_from(">II", self.package.manifest, 16)
        if (
            target != self.package.target
            or (product, hw, api) != (expected_product, expected_hw, 3)
            or struct.unpack_from("<I", page0, 24)[0] != self.package.loader_version
            or struct.unpack_from("<I", page1, 28)[0] != PROFILE
        ):
            raise ValueError("设备与升级包的MCU、版本、布局或参数ABI不配套")


def main() -> int:
    """包检查默认离线；upgrade须显式独占台架且逻辑电源供电。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("inspect", "upgrade"))
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--sdk", type=Path)
    parser.add_argument("--node", type=int)
    parser.add_argument("--uid")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--exclusive-bench", action="store_true")
    parser.add_argument(
        "--rescue-window", action="store_true", help="先抓住复位启动窗口，再检查身份"
    )
    args = parser.parse_args()
    try:
        package = UpgradePackage.load(args.package)
    except (OSError, ValueError, KeyError, struct.error) as exc:
        parser.error(f"升级包或目标契约无法读取/校验：{exc}")
    if args.action == "inspect":
        print(
            json.dumps(
                {
                    "version": package.version,
                    "bytes": len(package.image),
                    "crc32": f"{package.crc32:08x}",
                    "target": package.target,
                },
                indent=2,
            )
        )
        return 0
    if (
        not args.exclusive_bench
        or args.sdk is None
        or args.node is None
        or args.uid is None
        or args.output is None
    ):
        parser.error("upgrade必须提供SDK、节点、UID、outputs证据目录及独占台架条件")
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("证据目录必须在当前工作树outputs下")
    output.mkdir(parents=True, exist_ok=False)
    from ports.controlcanfd import ControlCanFd

    report = {"success": False, "package_sha256": hashlib.sha256(package.image).hexdigest()}
    client = None
    try:
        uid = bytes.fromhex(args.uid)
        if len(uid) != 12 or not 0 <= args.node < 255:
            raise ValueError("UID须为12B十六进制且node须为0..254")
        last_percent = -1

        def progress(written: int, total: int) -> None:
            nonlocal last_percent
            percent = written * 100 // total
            if percent != last_percent:
                print(f"\r下载 {percent}%", end="\n" if written == total else "", flush=True)
                last_percent = percent

        with ControlCanFd(args.sdk) as transport:
            client = UpgradeClient(transport, args.node, uid, progress)
            if args.rescue_window:
                client.hold_rescue_window()
            report.update(client.run(package))
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        report["failure"] = str(exc)
        if client is not None:
            report["trace"] = client.trace
            report["timing"] = client.timing_summary()
    finally:
        (output / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    print("升级已确认" if report["success"] else f"升级失败：{report.get('failure', '未知错误')}")
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
