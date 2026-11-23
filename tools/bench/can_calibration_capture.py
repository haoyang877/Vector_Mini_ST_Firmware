"""按固件能力采集 CAN Hall 原码快照，兼容已验证的旧 RTT 格式。"""

import csv
import struct
import threading
import time
from pathlib import Path

from can_calibration_curves import calibration_probe
from hardware_check import Link

FRAME = struct.Struct("<8h")
CHANNEL_NAME = "JScope_i2i2i2i2i2i2i2i2"


def decode_frames(data: bytes) -> tuple[int, list[tuple[int, ...]], int]:
    """RTT主机读取可从半帧开始；以保留零槽及数值范围唯一同步，原BIN不裁剪。"""

    def valid(row):
        return (
            row[:5] == (0,) * 5
            and 0 <= row[5] <= 6284
            and all(0 <= value <= 4095 for value in row[6:])
        )

    candidates = []
    for offset in range(FRAME.size):
        count = min(32, (len(data) - offset) // FRAME.size)
        if count >= 2 and all(
            valid(FRAME.unpack_from(data, offset + i * FRAME.size)) for i in range(count)
        ):
            candidates.append(offset)
    if len(candidates) != 1:
        raise ValueError("RTT原码无法唯一定位帧边界，保留BIN但不猜测波形")
    offset = candidates[0]
    end = offset + (len(data) - offset) // FRAME.size * FRAME.size
    frames = list(FRAME.iter_unpack(data[offset:end]))
    if not all(valid(row) for row in frames):
        raise ValueError("RTT原码帧格式异常，保留BIN但不绘制")
    return offset, frames, len(data) - end


class HallRawCapture:
    """CAN由标定线程串行轮询；旧RTT由独立线程独占已选探针。"""

    def __init__(self, directory: Path, *, probe_serial: int | None = None):
        self.directory = directory
        self.probe_serial = probe_serial
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.thread = None
        self.can_mode = False
        self.first_read_ms = None
        self.last_counter = None
        self.result = dict(
            source="RTT channel 1",
            nominal_hz=2000,
            loss_detection_available=False,
            samples=0,
        )

    def start(self, report: dict) -> None:
        if report.get("capabilities", 0) & (1 << 25):
            self.can_mode = True
            self.result.update(
                source="CAN Hall snapshots",
                nominal_hz=100,
                sample_counter_available=True,
                csv="hall-raw.csv",
            )
            (self.directory / "hall-raw.csv").write_text(
                "sample_index,nominal_elapsed_s,sample_counter,device_read_ms,hall_sin_counts,hall_cos_counts\n",
                encoding="utf-8",
            )
            return
        version = report["app_version"]
        if version in (0x01080E, 0x01080F):
            self.result["error"] = "当前 APP 的 RTT1 提供母线及三相 ADC 原码，没有 Hall 原码"
            return
        if version not in (
            0x010803,
            0x010804,
            0x010805,
            0x010806,
            0x010807,
            0x010808,
            0x010809,
            0x01080A,
            0x01080B,
            0x01080C,
            0x01080D,
        ):
            self.result["error"] = "当前APP未验证RTT原码格式，未采集原始波形"
            return
        self.result["app_version"] = version
        self.thread = threading.Thread(target=self._read, args=(report["uid"],), daemon=True)
        self.thread.start()
        if not self.ready.wait(10):
            self.result["error"] = "等待Hall原码采集启动超时"
            self.stop.set()

    def sample(self, link: Link) -> None:
        """由标定CAN线程串行轮询真实原码，不与命令争用端口。"""
        if not self.can_mode or self.result.get("error"):
            return
        try:
            data = link.request(195)
            if len(data) != 20:
                raise ValueError("Hall原码快照长度错误")
            result, detail, version, flags, counter, millis, sin, cos = struct.unpack(
                "<4H2I2H", data
            )
            if (result, detail, version, flags) != (0, 0, 1, 1) or max(sin, cos) > 4095:
                raise ValueError("Hall原码快照无效")
            if counter == self.last_counter:
                return
            if (
                self.last_counter is not None
                and ((counter - self.last_counter) & 0xFFFFFFFF) > 0x7FFFFFFF
            ):
                raise ValueError("Hall原码采样计数回退，停止采集")
            if self.first_read_ms is None:
                self.first_read_ms = millis
            elapsed = ((millis - self.first_read_ms) & 0xFFFFFFFF) / 1000
            with (self.directory / "hall-raw.csv").open(
                "a", newline="", encoding="utf-8"
            ) as stream:
                csv.writer(stream).writerow(
                    (self.result["samples"], elapsed, counter, millis, sin, cos)
                )
            self.last_counter = counter
            self.result["samples"] += 1
        except (TimeoutError, ValueError, OSError) as error:
            self.result["error"] = str(error)
            self.result["complete"] = False

    def _read(self, uid: str) -> None:
        try:
            with calibration_probe(uid, self.probe_serial) as (probe, _):
                probe.rtt_start()
                try:
                    deadline = time.monotonic() + 2
                    while True:
                        try:
                            descriptor = probe.rtt_get_buf_descriptor(1, True)
                            if descriptor.name:
                                break
                        except Exception:
                            pass
                        if time.monotonic() >= deadline or self.stop.wait(0.01):
                            raise ValueError("未发现Hall RTT通道") from None
                    if descriptor.name != CHANNEL_NAME:
                        raise ValueError("RTT通道格式与Hall原码契约不一致")
                    probe.rtt_read(1, 65536)  # 丢弃启动前的旧缓冲。
                    started = time.monotonic()
                    with (
                        (self.directory / "hall-raw.bin").open("wb") as binary,
                        (self.directory / "hall-raw-receive.csv").open(
                            "w", newline="", encoding="utf-8"
                        ) as stream,
                    ):
                        writer = csv.writer(stream)
                        writer.writerow(["receive_elapsed_s", "byte_offset", "byte_count"])
                        self.ready.set()
                        while not self.stop.is_set():
                            data = bytes(probe.rtt_read(1, 65536))
                            if data:
                                writer.writerow(
                                    [time.monotonic() - started, binary.tell(), len(data)]
                                )
                                binary.write(data)
                            self.stop.wait(0.005)
                        self.result["elapsed_s"] = time.monotonic() - started
                finally:
                    probe.rtt_stop()
        except Exception as exc:
            self.result["error"] = str(exc)
        finally:
            self.ready.set()

    def finish(self) -> dict:
        self.stop.set()
        if self.can_mode:
            return self.result
        if self.thread is not None:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                self.result["error"] = "Hall采集线程未退出，跳过探针回读"
                return self.result
        path = self.directory / "hall-raw.bin"
        if path.exists():
            data = path.read_bytes()
            self.result["bytes"] = len(data)
            try:
                offset, frames, tail = decode_frames(data)
            except ValueError as exc:
                self.result["error"] = str(exc)
                return self.result
            self.result.update(prefix_bytes=offset, suffix_bytes=tail)
            with (self.directory / "hall-raw.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "sample_index",
                        "nominal_elapsed_s",
                        "hall_angle_mrad",
                        "hall_sin_counts",
                        "hall_cos_counts",
                    ]
                )
                for index, row in enumerate(frames):
                    writer.writerow([index, index / 2000, *row[5:]])
            self.result.update(
                samples=len(frames),
                csv="hall-raw.csv",
                binary="hall-raw.bin",
                receive_csv="hall-raw-receive.csv",
            )
            self.result["observed_extrema_counts"] = {
                key: {
                    "min": min(row[column] for row in frames),
                    "max": max(row[column] for row in frames),
                }
                for key, column in (("sin", 6), ("cos", 7))
            }
        return self.result
