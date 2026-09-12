"""Fault-injection HIL cases for the resident Loader: bad images, misuse, rescue.

Runs against the real bench (USBCANFD adapter, node 0) with an optional JLink for
SWD resets:

* protocol-misuse cases assert the Loader rejects bad BEGIN/PROGRAM/VERIFY/ACTIVATE
  requests without ever activating anything,
* synthetic defect images (``fault_images.py``) are pushed end-to-end; the Loader
  must either refuse them at boot validation after a reset, or the host must be
  able to rescue the device through the 50 ms boot window,
* every case must leave the device recoverable and the suite restores the good
  APP image as its last step.

A run writes per-case evidence (images + summary.json) into --logdir.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time
from typing import Callable

from fault_images import FaultImage, build_all
from loader_proto import (
    APP_BASE_ADDRESS,
    FLASH_PAGE_SIZE,
    MAX_PROGRAM_CHUNK,
    OP_GET_STATUS,
    PROGRAM_ALIGNMENT,
    RESULT_BAD_SIZE,
    RESULT_BAD_STATE,
    RESULT_VERIFY_FAIL,
    RESULT_NAMES,
    crc32,
)
from loader_updater import (
    DEFAULT_DLL,
    LoaderBus,
    LoaderClient,
    LoaderResultError,
    update_firmware,
)

DEFAULT_JLINK_DLL = Path("C:/Program Files/SEGGER/JLink_V964/JLink_x64.dll")
STUB_DEPS = Path(__file__).resolve().parents[2] / "outputs" / "servo_hil_20260908" / ".deps"

STATE_NAMES = {0: "IDLE", 1: "MANIFEST", 2: "RECEIVING", 3: "VERIFIED", 4: "ERROR"}


class FaultClient(LoaderClient):
    """LoaderClient plus GET_STATUS parsing used by the fault suite."""

    def status(self) -> dict[str, int | str]:
        reply = self._request(OP_GET_STATUS)
        payload = self._payload(reply, 22)[2:]
        next_offset, size, crc, session = struct.unpack(">IIII", payload[4:20])
        return {
            "state": payload[0],
            "state_name": STATE_NAMES.get(payload[0], "UNKNOWN"),
            "last_error": payload[1],
            "last_error_name": RESULT_NAMES.get(payload[1], "NONE"),
            "next_offset": next_offset,
            "size": size,
            "crc": crc,
            "session": session,
        }


class JLinkResetter:
    """SWD reset helper; reuses the pylink copy under outputs/servo_hil_20260908/.deps."""

    def __init__(self, dll: Path, serial: int | None = None) -> None:
        if str(STUB_DEPS) not in sys.path:
            sys.path.insert(0, str(STUB_DEPS))
        import pylink  # noqa: PLC0415

        self._jlink = pylink.JLink(lib=pylink.library.Library(dllpath=str(dll)))
        if serial is None:
            self._jlink.open()
        else:
            self._jlink.open(serial)
        self._jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
        self._jlink.connect("STM32G431CB", speed=4000)
        self.resets = 0

    def reset(self) -> None:
        self._jlink.reset(halt=False)
        self.resets += 1

    def close(self) -> None:
        try:
            self._jlink.close()
        except Exception:  # noqa: BLE001
            pass


CASES: tuple[tuple[str, str, bool, str], ...] = (
    ("begin_zero_size", "BEGIN(size=0) must be rejected", False, "case_begin_zero_size"),
    ("begin_unaligned_size", "BEGIN(size%8!=0) must be rejected", False, "case_begin_unaligned_size"),
    ("begin_oversize", "BEGIN(size>96KiB) must be rejected", False, "case_begin_oversize"),
    ("program_requires_erase", "PROGRAM before ERASE must be rejected", False, "case_program_requires_erase"),
    ("verify_requires_complete_image", "VERIFY with missing data must be rejected", False,
     "case_verify_requires_complete_image"),
    ("activate_requires_verified", "ACTIVATE before VERIFY must be rejected", False, "case_activate_requires_verified"),
    ("verify_crc_mismatch", "VERIFY must reject a bad declared CRC", False, "case_verify_crc_mismatch"),
    ("bad_image_sp_out_of_sram", "out-of-SRAM SP image must never boot (recovery after reset)", True,
     "case_bad_image_sp_out_of_sram"),
    ("bad_image_sp_unaligned", "unaligned SP image must never boot (recovery after reset)", True,
     "case_bad_image_sp_unaligned"),
    ("bad_image_reset_thumb_bit", "even reset vector image must never boot (recovery after reset)", True,
     "case_bad_image_reset_thumb_bit"),
    ("bad_image_reset_outside", "out-of-range reset image must never boot (recovery after reset)", True,
     "case_bad_image_reset_outside"),
    ("hang_image_window_rescue", "valid-but-dead image rescued via 50 ms window + SWD reset", True,
     "case_hang_image_window_rescue"),
    ("return_image_reset_loop_rescue", "reset-loop image rescued by plain CAN polling", False,
     "case_return_image_reset_loop_rescue"),
    ("interrupted_update_recovery", "power loss mid-update -> Loader recovery -> re-upgrade", True,
     "case_interrupted_update_recovery"),
)


class FaultSuite:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.good_image = args.image.read_bytes()
        if not self.good_image or len(self.good_image) % 8:
            raise SystemExit("--image must be a non-empty, 8-byte aligned raw APP image")
        self.images: dict[str, FaultImage] = {image.name: image for image in build_all()}
        self.logdir: Path = args.logdir or Path("outputs") / f"loader_fault_{datetime.now():%Y%m%d_%H%M%S}"
        self.bus: LoaderBus | None = None
        self.client: FaultClient | None = None
        self.jlink: JLinkResetter | None = None

    # ---------------------------------------------------------------- helpers

    def _open(self) -> None:
        self.bus = LoaderBus(self.args.dll)
        self.client = FaultClient(self.bus, node=self.args.node)
        if self.args.no_jlink:
            print("note: --no-jlink; reset-dependent cases will be skipped")
            return
        try:
            self.jlink = JLinkResetter(self.args.jlink_dll, serial=self.args.jlink_serial)
        except Exception as exc:  # noqa: BLE001
            print(f"warning: JLink unavailable ({exc!r}); reset-dependent cases will be skipped")

    def _close(self) -> None:
        if self.jlink is not None:
            self.jlink.close()
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError as exc:
                print(f"warning: bus close: {exc!r}")

    def _expect_error(self, call: Callable[[], object], result: int, what: str) -> None:
        try:
            call()
        except LoaderResultError as exc:
            if exc.result != result:
                raise AssertionError(
                    f"{what}: expected {RESULT_NAMES.get(result, result)}, got {exc}") from exc
            return
        raise AssertionError(f"{what}: expected LoaderResultError({RESULT_NAMES.get(result, result)}), call succeeded")

    def _push_image(self, image: bytes, *, declared_crc: int | None = None,
                    truncate: int | None = None, version: int = 0) -> int:
        """BEGIN + ERASE + PROGRAM; optionally stop early or declare a wrong CRC."""
        assert self.client is not None
        declared = crc32(image) if declared_crc is None else declared_crc
        self.client.begin(len(image), declared, version)
        self.client.erase()
        body = image if truncate is None else image[:truncate]
        offset = 0
        while offset < len(body):
            step = min(MAX_PROGRAM_CHUNK, FLASH_PAGE_SIZE - offset % FLASH_PAGE_SIZE)
            chunk = body[offset:offset + step]
            aligned = (len(chunk) + PROGRAM_ALIGNMENT - 1) // PROGRAM_ALIGNMENT * PROGRAM_ALIGNMENT
            padded = chunk.ljust(aligned, b"\xFF")
            if self.client.program(offset, padded) != offset + len(padded):
                raise AssertionError(f"next_offset mismatch at {offset:#x}")
            offset += len(padded)
        return offset

    def _expect_app_silent(self, timeout: float = 1.5) -> None:
        assert self.client is not None
        try:
            self.client.wait_app_alive(timeout)
        except TimeoutError:
            return
        raise AssertionError("APP answered but must be silent (the fault image appears to run)")

    def _rescue_to_loader(self, timeout: float, *, allow_swd: bool = True) -> int:
        """Catch the Loader's 50 ms boot window; returns the number of SWD resets used."""
        assert self.client is not None
        deadline = time.perf_counter() + timeout
        resets = 0
        while time.perf_counter() < deadline:
            try:
                self.client.wait_loader_alive(0.25)
                return resets
            except TimeoutError:
                pass
            if allow_swd and self.jlink is not None:
                self.jlink.reset()
                resets += 1
        raise TimeoutError(f"Loader not reached within {timeout:.0f}s (SWD resets used: {resets})")

    def _ensure_loader(self) -> None:
        assert self.client is not None
        try:
            self.client.wait_loader_alive(0.4)
            return
        except TimeoutError:
            pass
        self.client.enter_boot()
        try:
            self.client.wait_loader_alive(2.0)
            return
        except TimeoutError:
            pass
        self._rescue_to_loader(timeout=10.0)

    def _restore_good_image(self) -> None:
        assert self.client is not None
        self._ensure_loader()
        update_firmware(self.client, self.good_image, self.args.version)

    # ------------------------------------------------------------ test cases

    def case_begin_zero_size(self) -> dict[str, object]:
        assert self.client is not None
        self._expect_error(lambda: self.client.begin(0, 0, self.args.version), RESULT_BAD_SIZE, "BEGIN size=0")
        return {"rejected": "size=0"}

    def case_begin_unaligned_size(self) -> dict[str, object]:
        assert self.client is not None
        self._expect_error(lambda: self.client.begin(123, 0, self.args.version), RESULT_BAD_SIZE, "BEGIN size=123")
        return {"rejected": "size%8!=0"}

    def case_begin_oversize(self) -> dict[str, object]:
        assert self.client is not None
        self._expect_error(lambda: self.client.begin(0x18008, 0, self.args.version), RESULT_BAD_SIZE,
                           "BEGIN size>96KiB")
        return {"rejected": "size>APP_SIZE"}

    def case_program_requires_erase(self) -> dict[str, object]:
        assert self.client is not None
        image = self.images["hang_no_comms"]
        self.client.begin(image.size, image.crc32, self.args.version)
        self._expect_error(lambda: self.client.program(0, bytes(8)), RESULT_BAD_STATE, "PROGRAM in MANIFEST")
        self.client.abort()
        return {"rejected": "PROGRAM before ERASE"}

    def case_verify_requires_complete_image(self) -> dict[str, object]:
        assert self.client is not None
        image = self.images["hang_no_comms"]
        self._push_image(image.data, truncate=40, version=self.args.version)
        self._expect_error(self.client.verify, RESULT_BAD_SIZE, "VERIFY with next_offset<size")
        self.client.abort()
        return {"rejected": "VERIFY with missing data"}

    def case_activate_requires_verified(self) -> dict[str, object]:
        assert self.client is not None
        image = self.images["hang_no_comms"]
        self.client.begin(image.size, image.crc32, self.args.version)
        self.client.erase()
        self._expect_error(self.client.activate, RESULT_BAD_STATE, "ACTIVATE in RECEIVING")
        self.client.abort()
        return {"rejected": "ACTIVATE before VERIFY"}

    def case_verify_crc_mismatch(self) -> dict[str, object]:
        assert self.client is not None
        image = self.images["hang_no_comms"]
        self._push_image(image.data, declared_crc=image.crc32 ^ 1, version=self.args.version)
        self._expect_error(self.client.verify, RESULT_VERIFY_FAIL, "VERIFY CRC mismatch")
        status = self.client.status()
        if status["state_name"] != "RECEIVING" or status["last_error"] != RESULT_VERIFY_FAIL:
            raise AssertionError(f"state after failed VERIFY: {status}")
        self.client.abort()
        return {"rejected": "declared CRC mismatch", "state_after_fail": status["state_name"]}

    def _case_bad_image(self, name: str) -> dict[str, object]:
        assert self.client is not None and self.jlink is not None
        image = self.images[name]
        if image.boot_valid:
            raise AssertionError(f"{name} is declared boot-valid")
        self._push_image(image.data, version=self.args.version)
        computed = self.client.verify()
        if computed != image.crc32:
            raise AssertionError(f"Loader CRC {computed:#010x} != image CRC {image.crc32:#010x}")
        self.client.activate()
        self._expect_app_silent(1.5)
        self.jlink.reset()
        self.client.wait_loader_alive(2.0)
        return {"silent_after_activate": True, "loader_stayed_in_recovery": True}

    def case_bad_image_sp_out_of_sram(self) -> dict[str, object]:
        return self._case_bad_image("bad_sp_out_of_sram")

    def case_bad_image_sp_unaligned(self) -> dict[str, object]:
        return self._case_bad_image("bad_sp_unaligned")

    def case_bad_image_reset_thumb_bit(self) -> dict[str, object]:
        return self._case_bad_image("bad_reset_thumb_bit")

    def case_bad_image_reset_outside(self) -> dict[str, object]:
        return self._case_bad_image("bad_reset_outside")

    def case_hang_image_window_rescue(self) -> dict[str, object]:
        assert self.client is not None
        image = self.images["hang_no_comms"]
        self._push_image(image.data, version=self.args.version)
        if self.client.verify() != image.crc32:
            raise AssertionError("Loader CRC mismatch for the hang image")
        self.client.activate()
        self._expect_app_silent(1.5)
        resets = self._rescue_to_loader(timeout=10.0)
        self._restore_good_image()
        return {"silent_after_activate": True, "window_rescued": True, "swd_resets_used": resets}

    def case_return_image_reset_loop_rescue(self) -> dict[str, object]:
        assert self.client is not None
        image = self.images["return_from_reset"]
        self._push_image(image.data, version=self.args.version)
        if self.client.verify() != image.crc32:
            raise AssertionError("Loader CRC mismatch for the return image")
        self.client.activate()
        self._expect_app_silent(1.5)
        # The device re-enters the Loader window on its own; plain CAN polling must catch it.
        self._rescue_to_loader(timeout=8.0, allow_swd=False)
        self._restore_good_image()
        return {"silent_after_activate": True, "polling_rescued": True}

    def case_interrupted_update_recovery(self) -> dict[str, object]:
        assert self.client is not None and self.jlink is not None
        self._push_image(self.good_image, truncate=0x4000, version=self.args.version)
        # ERASE cleared the record before programming; simulate power loss mid-update.
        self.jlink.reset()
        self.client.wait_loader_alive(2.0)
        self._restore_good_image()
        return {"record_invalidated_by_erase": True, "loader_recovery": True, "app_restored": True}

    # ----------------------------------------------------------------- runner

    def run(self) -> int:
        selected = {item.strip() for item in self.args.cases.split(",") if item.strip()} if self.args.cases else None
        if selected is not None:
            unknown = selected - {case[0] for case in CASES}
            if unknown:
                raise SystemExit(f"unknown cases: {sorted(unknown)}")

        self.logdir.mkdir(parents=True, exist_ok=True)
        images_dir = self.logdir / "images"
        images_dir.mkdir(exist_ok=True)
        manifest = []
        for image in self.images.values():
            (images_dir / f"{image.name}.bin").write_bytes(image.data)
            manifest.append({"name": image.name, "size": image.size, "crc32": f"0x{image.crc32:08X}",
                             "boot_valid": image.boot_valid, "note": image.note})
        (images_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

        records: list[dict[str, object]] = []
        overall_started = time.perf_counter()
        self._open()
        try:
            for ident, title, needs_reset, method_name in CASES:
                if selected is not None and ident not in selected:
                    continue
                record: dict[str, object] = {"case": ident, "title": title}
                started = time.perf_counter()
                if needs_reset and self.jlink is None:
                    record.update(ok=None, status="skipped", detail="requires SWD reset (use JLink or power-cycle)")
                else:
                    try:
                        self._ensure_loader()
                        observations = getattr(self, method_name)()
                        record.update(ok=True, status="passed", observations=observations)
                    except Exception as exc:  # noqa: BLE001
                        record.update(
                            ok=False, status="failed", detail=repr(exc),
                            communication_errors=(self.client.errors[-8:] if self.client else []),
                        )
                record["elapsed_ms"] = round((time.perf_counter() - started) * 1000, 1)
                records.append(record)
                print(f"[{record['status']:>7}] {ident} ({record['elapsed_ms']} ms)")

            restore: dict[str, object] = {"ok": True, "status": "restored"}
            restore_started = time.perf_counter()
            try:
                self._restore_good_image()
            except Exception as exc:  # noqa: BLE001
                restore = {"ok": False, "status": "failed", "detail": repr(exc)}
            restore["elapsed_ms"] = round((time.perf_counter() - restore_started) * 1000, 1)
        finally:
            self._close()

        summary = {
            "ok": all(record["ok"] is not False for record in records) and bool(restore["ok"]),
            "total": len(records),
            "passed": sum(record["ok"] is True for record in records),
            "failed": sum(record["ok"] is False for record in records),
            "skipped": sum(record["status"] == "skipped" for record in records),
            "elapsed_s": round(time.perf_counter() - overall_started, 1),
            "restore": restore,
            "cases": records,
        }
        (self.logdir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({key: value for key, value in summary.items() if key != "cases"}, indent=2))
        return 0 if summary["ok"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True, help="raw APP bin used to restore the device")
    parser.add_argument("--node", type=int, default=0)
    parser.add_argument("--dll", type=Path, default=DEFAULT_DLL)
    parser.add_argument("--logdir", type=Path)
    parser.add_argument("--cases", help="comma-separated case ids (default: all)")
    parser.add_argument("--no-jlink", action="store_true", help="skip cases that need an SWD reset")
    parser.add_argument("--jlink-dll", type=Path, default=DEFAULT_JLINK_DLL)
    parser.add_argument("--jlink-serial", type=int)
    parser.add_argument("--version", type=int, default=600000, help="record version for restored images")
    args = parser.parse_args()
    return FaultSuite(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
