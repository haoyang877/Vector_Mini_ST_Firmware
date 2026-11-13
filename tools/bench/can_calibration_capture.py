"""采集 APP1.8.3～1.8.6 的 RTT Hall 原码；不修改控制参数或固件。"""

import csv
import struct
import threading
import time
from pathlib import Path

from can_calibration_curves import calibration_probe

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
    """一个工作线程独占探针；启动确认完成后，CAN才允许发标定命令。"""

    def __init__(self, directory: Path):
        self.directory = directory
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.thread = None
        self.result = dict(
            source="RTT channel 1",
            nominal_hz=2000,
            loss_detection_available=False,
            samples=0,
        )

    def start(self, report: dict) -> None:
        version = report["app_version"]
        if version not in (0x010803, 0x010804, 0x010805, 0x010806):
            self.result["error"] = "当前APP未验证RTT原码格式，未采集原始波形"
            return
        self.result["app_version"] = version
        self.thread = threading.Thread(target=self._read, args=(report["uid"],), daemon=True)
        self.thread.start()
        if not self.ready.wait(10):
            self.result["error"] = "等待Hall原码采集启动超时"
            self.stop.set()

    def _read(self, uid: str) -> None:
        try:
            with calibration_probe(uid) as (probe, _):
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
