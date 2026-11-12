"""CAN FD 单帧手动发送与原始收帧记录。"""

import csv
import ctypes as C
import os
import re
import time

from can_link_test import _RXFD, ControlCanFd, send_fd_once


def parse_frame(id_text, hex_text):
    digits = id_text.strip().lower().removeprefix("0x")
    if not re.fullmatch(r"[0-9a-f]{1,8}", digits) or int(digits, 16) > 0x1FFFFFFF:
        raise ValueError("CAN ID 须为 0 至 1FFFFFFF 的十六进制数。")
    identifier = int(digits, 16)
    extended = len(digits) > 3 or identifier > 0x7FF
    try:
        data = bytes.fromhex(hex_text)
    except ValueError as exc:
        raise ValueError("每个字节请输入两位十六进制数，例如 01 02 FF。") from exc
    if len(data) not in (*range(9), 12, 16, 20, 24, 32, 48, 64):
        raise ValueError("CAN FD 数据长度须为 0–8、12、16、20、24、32、48 或 64 字节。")
    return identifier, extended, data


def send_and_capture(
    dll,
    channel,
    frame,
    directory,
    log,
    stop,
    *,
    arb_bps=1_000_000,
    data_bps=5_000_000,
    progress=None,
):
    identifier, extended, data = frame
    directory.mkdir(parents=True)
    with (directory / "frames.csv").open("w", encoding="utf-8", newline="") as file:
        trace = csv.writer(file)
        trace.writerow(
            (
                "event",
                "channel",
                "elapsed_ms",
                "adapter_us",
                "can_id",
                "extended",
                "brs",
                "rtr",
                "err",
                "esi",
                "data_hex",
                "detail",
                "arb_bps",
                "data_bps",
            )
        )
        start = time.perf_counter()

        def record(event, detail=""):
            trace.writerow(
                (
                    event,
                    channel,
                    round((time.perf_counter() - start) * 1000, 3),
                    "",
                    hex(identifier),
                    int(extended),
                    1,
                    0,
                    0,
                    0,
                    data.hex(),
                    detail,
                    arb_bps,
                    data_bps,
                )
            )

        record("requested")
        try:
            with (
                os.add_dll_directory(str(dll.parent)),
                ControlCanFd(
                    dll, channel_index=channel - 1, arb_bps=arb_bps, data_bps=data_bps
                ) as bus,
            ):
                if stop.is_set():
                    record("cancelled")
                    return "已停止"
                if not send_fd_once(bus, identifier, data, extended=extended):
                    record("tx_rejected")
                    return "SDK 拒收"
                record("tx_accepted")
                log(
                    f"TX  0x{identifier:X}  {'扩展' if extended else '标准'}  {len(data)} 字节  {data.hex(' ').upper()}  [SDK 已受理]"
                )
                deadline = time.perf_counter() + 1
                next_update = deadline - 1
                received = 0
                batch = (_RXFD * 128)()
                while time.perf_counter() < deadline and not stop.is_set():
                    now = time.perf_counter()
                    if progress and now >= next_update:
                        progress(min(1, now - deadline + 1), f"接收窗口 · 已收到 {received} 帧")
                        next_update = now + 0.1
                    n = bus.dll.ZCAN_ReceiveFD(bus.channel, C.byref(batch), len(batch), 0)
                    if not 0 <= n <= len(batch):
                        raise OSError(f"CAN FD 接收失败，SDK 返回 {n}")
                    for item in batch[:n]:
                        f = item.frame
                        if f.length > 64:
                            raise OSError(f"SDK 返回非法帧长度 {f.length}")
                        raw = bytes(f.data[: f.length])
                        ms = round((time.perf_counter() - start) * 1000, 3)
                        trace.writerow(
                            (
                                "rx",
                                channel,
                                ms,
                                item.timestamp,
                                hex(f.can_id),
                                f.eff,
                                f.brs,
                                f.rtr,
                                f.err,
                                f.esi,
                                raw.hex(),
                                "",
                                arb_bps,
                                data_bps,
                            )
                        )
                        received += 1
                        if received <= 50:
                            flags = " ".join(
                                k for k, v in [("ERR", f.err), ("ESI", f.esi), ("RTR", f.rtr)] if v
                            )
                            log(
                                f"RX  0x{f.can_id:X}  {'扩展' if f.eff else '标准'}  {len(raw)} 字节  {raw.hex(' ').upper()}  {flags}".rstrip()
                            )
                    if n < len(batch):
                        time.sleep(0.001)
                if received > 50:
                    log("界面显示前 50 帧，全部原始帧已写入 frames.csv。")
                return f"{'已停止接收' if stop.is_set() else '已结束'}，收到 {received} 帧"
        except OSError as exc:
            record("error", str(exc))
            raise
