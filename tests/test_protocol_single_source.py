"""Cross-check every copy of the Loader/APP handoff contract against its source.

Offline only.  Parses the shared single-source headers and asserts that:

* ``loader/tools/loader_proto.py`` mirrors every shared value,
* ``loader/firmware/loader_cfg.h`` aliases (never re-declares) the shared values,
* the Keil scatter/project files and ``Bsp/flash.h`` agree with ``shared/flash_layout``,
* ``Communication/interface_can.h`` consumes the boot-handoff macros,
* ``shared/boot_mailbox.h`` agrees with the RAM window.

A failure names the exact file and value pair that must change together.
"""

from pathlib import Path
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "loader" / "tools"))

from loader_proto import (  # noqa: E402
    APP_BASE_ADDRESS,
    APP_CAPACITY,
    APP_REQUEST_ENTER_BOOT,
    APP_REQUEST_GET_MODE,
    BOOT_WINDOW_MS,
    CRC_SIZE,
    FLASH_PAGE_SIZE,
    HEADER_SIZE,
    MAGIC,
    MAX_PAYLOAD_SIZE,
    MAX_PROGRAM_CHUNK,
    OP_ABORT,
    OP_ACTIVATE,
    OP_BEGIN_UPDATE,
    OP_ERASE,
    OP_GET_INFO,
    OP_GET_STATUS,
    OP_PROGRAM,
    OP_VERIFY,
    PROTOCOL_VERSION,
    PROGRAM_ALIGNMENT,
    REQUEST_ID_BASE,
    RESPONSE_ID_BASE,
    RESULT_BAD_CRC,
    RESULT_BAD_FRAME,
    RESULT_BAD_OFFSET,
    RESULT_BAD_OP,
    RESULT_BAD_SIZE,
    RESULT_BAD_STATE,
    RESULT_FLASH_ERR,
    RESULT_OK,
    RESULT_VERIFY_FAIL,
)

LAYOUT_H = ROOT / "shared" / "flash_layout" / "flash_layout.h"
PROTOCOL_H = ROOT / "shared" / "protocol" / "loader_can_v1.h"
HANDOFF_H = ROOT / "shared" / "protocol" / "boot_handoff.h"
MAILBOX_H = ROOT / "shared" / "boot_mailbox.h"
LOADER_CFG_H = ROOT / "loader" / "firmware" / "loader_cfg.h"
LOADER_SCT = ROOT / "loader" / "mdk" / "Vector_Mini_ST_Loader" / "Vector_Mini_ST_Loader.sct"
APP_SCT = ROOT / "MDK-ARM" / "Vector_Mini_ST" / "Vector_Mini_ST.sct"
APP_UVPROJX = ROOT / "MDK-ARM" / "Vector_Mini_ST.uvprojx"
INTERFACE_CAN_H = ROOT / "Communication" / "interface_can.h"
BSP_FLASH_H = ROOT / "Bsp" / "flash.h"

# shared/protocol/loader_can_v1.h  <->  loader/tools/loader_proto.py
PROTOCOL_VALUE_MAP = {
    "LOADER_CAN_REQ_ID_BASE": REQUEST_ID_BASE,
    "LOADER_CAN_RSP_ID_BASE": RESPONSE_ID_BASE,
    "LOADER_CAN_MAGIC": MAGIC,
    "LOADER_CAN_PROTOCOL_VERSION": PROTOCOL_VERSION,
    "LOADER_CAN_HEADER_SIZE": HEADER_SIZE,
    "LOADER_CAN_CRC_SIZE": CRC_SIZE,
    "LOADER_CAN_MAX_PAYLOAD": MAX_PAYLOAD_SIZE,
    "LOADER_CAN_MAX_CHUNK": MAX_PROGRAM_CHUNK,
    "LOADER_CAN_CHUNK_ALIGN": PROGRAM_ALIGNMENT,
    "LOADER_CAN_BOOT_WINDOW_MS": BOOT_WINDOW_MS,
    "LOADER_CAN_OP_GET_INFO": OP_GET_INFO,
    "LOADER_CAN_OP_GET_STATUS": OP_GET_STATUS,
    "LOADER_CAN_OP_BEGIN": OP_BEGIN_UPDATE,
    "LOADER_CAN_OP_ERASE": OP_ERASE,
    "LOADER_CAN_OP_PROGRAM": OP_PROGRAM,
    "LOADER_CAN_OP_VERIFY": OP_VERIFY,
    "LOADER_CAN_OP_ACTIVATE": OP_ACTIVATE,
    "LOADER_CAN_OP_ABORT": OP_ABORT,
    "LOADER_CAN_RES_OK": RESULT_OK,
    "LOADER_CAN_RES_BAD_FRAME": RESULT_BAD_FRAME,
    "LOADER_CAN_RES_BAD_CRC": RESULT_BAD_CRC,
    "LOADER_CAN_RES_BAD_STATE": RESULT_BAD_STATE,
    "LOADER_CAN_RES_BAD_OFFSET": RESULT_BAD_OFFSET,
    "LOADER_CAN_RES_BAD_SIZE": RESULT_BAD_SIZE,
    "LOADER_CAN_RES_FLASH": RESULT_FLASH_ERR,
    "LOADER_CAN_RES_VERIFY": RESULT_VERIFY_FAIL,
    "LOADER_CAN_RES_BAD_OP": RESULT_BAD_OP,
}

# shared/flash_layout/flash_layout.h  <->  loader/tools/loader_proto.py
LAYOUT_VALUE_MAP = {
    "FLASH_LAYOUT_PAGE_SIZE": FLASH_PAGE_SIZE,
    "FLASH_LAYOUT_APP_BASE": APP_BASE_ADDRESS,
    "FLASH_LAYOUT_APP_SIZE": APP_CAPACITY,
}

# shared/protocol/boot_handoff.h  <->  loader/tools/loader_proto.py
HANDOFF_VALUE_MAP = {
    "BOOT_HANDOFF_CMD_ENTER_BOOT": APP_REQUEST_ENTER_BOOT,
    "BOOT_HANDOFF_CMD_GET_MODE": APP_REQUEST_GET_MODE,
}

# loader/firmware/loader_cfg.h must alias the shared macros, never redefine them
LOADER_CFG_ALIASES = {
    "LDR_FLASH_BASE": "FLASH_LAYOUT_BASE",
    "LDR_PAGE_SIZE": "FLASH_LAYOUT_PAGE_SIZE",
    "LDR_APP_BASE": "FLASH_LAYOUT_APP_BASE",
    "LDR_APP_SIZE": "FLASH_LAYOUT_APP_SIZE",
    "LDR_APP_FIRST_PAGE": "FLASH_LAYOUT_APP_FIRST_PAGE",
    "LDR_APP_PAGES": "FLASH_LAYOUT_APP_PAGES",
    "LDR_RECORD_ADDR": "FLASH_LAYOUT_RECORD_BASE",
    "LDR_RECORD_PAGE": "FLASH_LAYOUT_RECORD_PAGE",
    "LDR_RECORD_MAGIC": "FLASH_LAYOUT_RECORD_MAGIC",
    "LDR_REQ_BASE": "LOADER_CAN_REQ_ID_BASE",
    "LDR_RSP_BASE": "LOADER_CAN_RSP_ID_BASE",
    "LDR_MAGIC": "LOADER_CAN_MAGIC",
    "LDR_VERSION": "LOADER_CAN_PROTOCOL_VERSION",
    "LDR_HEADER_SIZE": "LOADER_CAN_HEADER_SIZE",
    "LDR_CRC_SIZE": "LOADER_CAN_CRC_SIZE",
    "LDR_MAX_PAYLOAD": "LOADER_CAN_MAX_PAYLOAD",
    "LDR_OP_INFO": "LOADER_CAN_OP_GET_INFO",
    "LDR_OP_STATUS": "LOADER_CAN_OP_GET_STATUS",
    "LDR_OP_BEGIN": "LOADER_CAN_OP_BEGIN",
    "LDR_OP_ERASE": "LOADER_CAN_OP_ERASE",
    "LDR_OP_PROGRAM": "LOADER_CAN_OP_PROGRAM",
    "LDR_OP_VERIFY": "LOADER_CAN_OP_VERIFY",
    "LDR_OP_ACTIVATE": "LOADER_CAN_OP_ACTIVATE",
    "LDR_OP_ABORT": "LOADER_CAN_OP_ABORT",
    "LDR_RES_OK": "LOADER_CAN_RES_OK",
    "LDR_RES_BAD_FRAME": "LOADER_CAN_RES_BAD_FRAME",
    "LDR_RES_BAD_CRC": "LOADER_CAN_RES_BAD_CRC",
    "LDR_RES_BAD_STATE": "LOADER_CAN_RES_BAD_STATE",
    "LDR_RES_BAD_OFFSET": "LOADER_CAN_RES_BAD_OFFSET",
    "LDR_RES_BAD_SIZE": "LOADER_CAN_RES_BAD_SIZE",
    "LDR_RES_FLASH": "LOADER_CAN_RES_FLASH",
    "LDR_RES_VERIFY": "LOADER_CAN_RES_VERIFY",
    "LDR_RES_BAD_OP": "LOADER_CAN_RES_BAD_OP",
    "LDR_MAX_CHUNK": "LOADER_CAN_MAX_CHUNK",
    "LDR_BOOT_WINDOW_MS": "LOADER_CAN_BOOT_WINDOW_MS",
}


def parse_defines(path: Path) -> dict[str, str]:
    """Return ``{NAME: value-text}`` for every plain object-like #define."""
    defines: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("/*", 1)[0].strip()
        if not line.startswith("#define"):
            continue
        parts = line.split(None, 2)
        if len(parts) < 3 or "(" in parts[1]:
            continue
        defines[parts[1]] = parts[2].strip()
    return defines


def numeric(defines: dict[str, str], name: str, seen: frozenset[str] = frozenset()) -> int:
    """Resolve a plain or derived numeric #define into an int."""
    if name in seen:
        raise AssertionError(f"circular #define chain at {name}")
    if name not in defines:
        raise AssertionError(f"missing #define {name}")
    text = re.sub(r"[uUlL]+$", "", defines[name].strip())
    if re.fullmatch(r"0[xX][0-9A-Fa-f]+|\d+", text):
        return int(text, 0)

    def substitute(match: re.Match[str]) -> str:
        return str(numeric(defines, match.group(0), seen | {name}))

    expression = re.sub(r"\b[A-Za-z_]\w*\b", substitute, text)
    return int(eval(expression, {"__builtins__": {}}, {}))  # noqa: S307 - arithmetic over parsed defines


class ProtocolSingleSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = parse_defines(PROTOCOL_H)
        cls.layout = parse_defines(LAYOUT_H)
        cls.handoff = parse_defines(HANDOFF_H)
        cls.loader_cfg = parse_defines(LOADER_CFG_H)
        cls.mailbox = parse_defines(MAILBOX_H)

    def test_loader_protocol_mirror_matches_shared(self) -> None:
        for shared_name, python_value in PROTOCOL_VALUE_MAP.items():
            with self.subTest(shared=shared_name):
                self.assertIn(shared_name, self.protocol, "shared/protocol/loader_can_v1.h changed")
                self.assertEqual(
                    numeric(self.protocol, shared_name), python_value,
                    f"{shared_name} drifted; update loader/tools/loader_proto.py to match",
                )

    def test_flash_layout_mirror_matches_shared(self) -> None:
        for shared_name, python_value in LAYOUT_VALUE_MAP.items():
            with self.subTest(shared=shared_name):
                self.assertEqual(
                    numeric(self.layout, shared_name), python_value,
                    f"{shared_name} drifted; update loader/tools/loader_proto.py to match",
                )

    def test_boot_handoff_mirror_matches_shared(self) -> None:
        for shared_name, python_value in HANDOFF_VALUE_MAP.items():
            with self.subTest(shared=shared_name):
                self.assertEqual(
                    numeric(self.handoff, shared_name), python_value,
                    f"{shared_name} drifted; update loader/tools/loader_proto.py to match",
                )

    def test_loader_cfg_aliases_shared_without_literals(self) -> None:
        text = LOADER_CFG_H.read_text(encoding="utf-8")
        self.assertIn('#include "flash_layout/flash_layout.h"', text)
        self.assertIn('#include "protocol/loader_can_v1.h"', text)
        for loader_name, shared_name in LOADER_CFG_ALIASES.items():
            with self.subTest(alias=loader_name):
                self.assertIn(loader_name, self.loader_cfg, "loader_cfg.h dropped an alias")
                self.assertEqual(
                    self.loader_cfg[loader_name], shared_name,
                    f"loader_cfg.h: {loader_name} must alias {shared_name}, "
                    f"found {self.loader_cfg[loader_name]!r}",
                )

    def test_app_scatter_matches_flash_layout(self) -> None:
        text = APP_SCT.read_text(encoding="utf-8")
        app_base = numeric(self.layout, "FLASH_LAYOUT_APP_BASE")
        app_size = numeric(self.layout, "FLASH_LAYOUT_APP_SIZE")
        ram_fw_base = numeric(self.layout, "FLASH_LAYOUT_RAM_FW_BASE")
        ram_fw_size = numeric(self.layout, "FLASH_LAYOUT_RAM_FW_SIZE")
        self.assertIn(f"LR_IROM1 0x{app_base:08X} 0x{app_size:08X}", text)
        self.assertIn(f"RW_IRAM1 0x{ram_fw_base:08X} 0x{ram_fw_size:08X}", text)

    def test_loader_scatter_matches_flash_layout(self) -> None:
        text = LOADER_SCT.read_text(encoding="utf-8")
        loader_base = numeric(self.layout, "FLASH_LAYOUT_LOADER_BASE")
        loader_size = numeric(self.layout, "FLASH_LAYOUT_LOADER_SIZE")
        ram_fw_base = numeric(self.layout, "FLASH_LAYOUT_RAM_FW_BASE")
        ram_fw_size = numeric(self.layout, "FLASH_LAYOUT_RAM_FW_SIZE")
        self.assertIn(f"LR_IROM1 0x{loader_base:08X} 0x{loader_size:08X}", text)
        self.assertIn(f"RW_IRAM1 0x{ram_fw_base:08X} 0x{ram_fw_size:08X}", text)

    def test_keil_app_project_matches_flash_layout(self) -> None:
        text = APP_UVPROJX.read_text(encoding="utf-8", errors="replace")
        app_base = numeric(self.layout, "FLASH_LAYOUT_APP_BASE")
        app_size = numeric(self.layout, "FLASH_LAYOUT_APP_SIZE")
        flash_base = numeric(self.layout, "FLASH_LAYOUT_BASE")
        ram_fw_base = numeric(self.layout, "FLASH_LAYOUT_RAM_FW_BASE")
        ram_fw_size = numeric(self.layout, "FLASH_LAYOUT_RAM_FW_SIZE")
        self.assertIn(f"APP_VTOR_OFFSET=0x{app_base - flash_base:X}", text)
        self.assertIn(f"IROM(0x{app_base:X}-0x{app_base + app_size - 1:X})", text)
        self.assertIn(f"IRAM(0x{ram_fw_base:X}-0x{ram_fw_base + ram_fw_size - 1:X})", text)

    def test_interface_can_consumes_boot_handoff_macros(self) -> None:
        text = INTERFACE_CAN_H.read_text(encoding="utf-8")
        self.assertIn('#include "protocol/boot_handoff.h"', text)
        for line_pattern in (
            r"CAN_ENTER_BOOT\s*=\s*BOOT_HANDOFF_CMD_ENTER_BOOT\b",
            r"CAN_GET_MODE\s*=\s*BOOT_HANDOFF_CMD_GET_MODE\b",
        ):
            with self.subTest(pattern=line_pattern):
                self.assertRegex(text, line_pattern)
        self.assertNotRegex(text, r"CAN_ENTER_BOOT\s*=\s*0x")
        self.assertNotRegex(text, r"CAN_GET_MODE\s*=\s*0x")

    def test_boot_mailbox_matches_ram_window(self) -> None:
        self.assertEqual(
            numeric(self.mailbox, "BOOT_MAILBOX_ADDR"),
            numeric(self.layout, "FLASH_LAYOUT_RAM_BASE"),
            "boot_mailbox.h and flash_layout.h disagree on the mailbox base",
        )

    def test_app_param_page_matches_flash_layout(self) -> None:
        text = BSP_FLASH_H.read_text(encoding="utf-8")
        param_base = numeric(self.layout, "FLASH_LAYOUT_PARAM_BASE")
        self.assertRegex(text, rf"ADDR_FLASH_PAGE_56\b[^\n]*0x{param_base:08X}")


if __name__ == "__main__":
    unittest.main()
