"""在原生平台编译 AT32 检测、独立霍尔角算法并注入 ADC/DMA 和运动事件。"""

from _bsp_test_support import ROOT, arguments, run_c
from hardware_check import Link, encode_frames


def main():
    args = arguments(__doc__)
    _, start = encode_frames(6, 127, bytes.fromhex("0201010001000000"), 0x42)[0]
    _, poll = encode_frames(6, 127, b"", 0x43)[0]
    source = (ROOT / "tests/unit/native/hardware_check_test.c").read_text(encoding="utf-8")
    for name, data in (("host_start", start), ("host_read", poll)):
        source += f"\nconst uint8_t {name}[] = {{" + ",".join(map(str, data)) + "};\n"
    run_c(
        args,
        "hardware_check",
        source,
        2,
        [
            "-DFIRMWARE_TARGET_AT32_OFF",
            "-I",
            ROOT / "projects/at32f456/application",
            ROOT / "projects/at32f456/application/hardware_check.c",
            ROOT / "firmware/motor/feedback/linear_hall.c",
            ROOT / "firmware/motor/math/control_math.c",
            ROOT / "firmware/communication/yg_protocol.c",
            ROOT / "firmware/communication/yg_protocol_boot.c",
            ROOT / "shared/protocol/yg_protocol_crc.c",
            ROOT / "shared/protocol/yg_protocol_frame_codec.c",
            ROOT / "shared/protocol/yg_protocol_can_id.c",
            ROOT / "tests/unit/yg_protocol_bsp_stubs.c",
        ],
    )
    log = (args.out / "hardware_check.log").read_text(encoding="utf-8")
    response = bytes.fromhex(
        next(line.split()[1] for line in log.splitlines() if line.startswith("CAN_HW_REPORT "))
    )

    class Replay:
        def send(self, identifier, data):
            assert (identifier, data) == (0x0CEF0602, poll)

        def receive(self, timeout):
            return 0x0CEF0206, response

    result = Link(Replay(), 6).board(sequence=0x43)
    assert result == dict(
        run_id=2, stage=4, board_items=[2, 1, 1, 1, 1, 1, 1, 1, 0, 0], directions=0
    )
    print(
        "PASS host to AT32 RX IRQ to physical-window detector to TX to host, including corrupt and repeated START"
    )


if __name__ == "__main__":
    main()
