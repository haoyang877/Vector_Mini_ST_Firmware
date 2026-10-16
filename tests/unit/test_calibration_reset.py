"""离线核对清标定工具、页保留及真实APP装载；不连接设备。"""

from __future__ import annotations

import hashlib
import struct
import subprocess
import sys
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/build"))
from harness.common import zig_path  # noqa: E402
from prepare_calibration_reset import (  # noqa: E402
    FEEDFORWARD_DEFAULT,
    IDENTITY_OFFSET,
    PARAM_BYTES,
    RECORD_BYTES,
    THERMAL_DEFAULT,
    prepare,
    record_crc,
)
from project_paths import NATIVE_INCLUDE_FLAGS  # noqa: E402
from run_position_servo_tests import function_source  # noqa: E402


def source_page(schema: int) -> bytearray:
    """构造含有效模型、限值、AXS1和独立身份页的完整参数快照。"""
    raw = bytearray(PARAM_BYTES)
    raw[RECORD_BYTES:] = bytes(
        (index * 17 + 3) % 256 for index in range(PARAM_BYTES - RECORD_BYTES)
    )
    struct.pack_into("<9f", raw, 0, 7, 2048, 2049, 2047, 21, 1.2, 0.0011, 0.0012, 0.01)
    struct.pack_into("<HHBB", raw, 36, 1234, 4567, 3, 1)
    for index in range(1024):
        struct.pack_into("<h", raw, 44 + index * 2, index % 128 - 64)
    struct.pack_into("<11f", raw, 2092, 0.2, 20, 0.25, 21, 3, 4, 0.023, 0.04, 5, 6, 1.2)
    struct.pack_into("<I", raw, 2136, 1)
    struct.pack_into("<3f", raw, 2144, 2, 4, 1.5)
    struct.pack_into("<II", raw, 2160, schema, 0x454E4332)
    struct.pack_into("<I", raw, 2172, 6)
    struct.pack_into("<2f", raw, 2180, 0.1, 0.3)
    axis = struct.pack("<II8s3f", 0x31535841, 1, b"roll", -1, 1, 2)
    raw[2208:2240] = axis + struct.pack("<I", zlib.crc32(axis))
    if schema == 13:
        struct.pack_into("<7f", raw, 4304, 3.2, 6.8, 70, 0.85, 0.65, 0.95, 0.45)
        struct.pack_into("<I4f", raw, 4332, 1, 0.04, 15, 120, 1.2)
    elif schema == 12:
        raw[4304:RECORD_BYTES] = b"\xff" * (RECORD_BYTES - 4304)
    else:
        put_extension(raw, 2344, b"MTH1", struct.pack("<7f", 3.2, 6.8, 70, 0.85, 0.65, 0.95, 0.45))
        put_extension(raw, 2312, b"SFF1", struct.pack("<I4f", 1, 0.04, 15, 120, 1.2))
    refresh_crc(raw)
    return raw


def refresh_crc(raw: bytearray) -> None:
    """只有schema12/13有整条CRC；schema10不伪造旧记录完整性。"""
    schema = struct.unpack_from("<I", raw, 2160)[0]
    if schema in (12, 13):
        size = 4304 if schema == 12 else RECORD_BYTES
        struct.pack_into("<I", raw, 2140, zlib.crc32(raw[:2140] + raw[2144:size]))


def put_extension(raw: bytearray, offset: int, magic: bytes, payload: bytes) -> None:
    """写入旧交付的独立版本化扩展及其真实CRC。"""
    block = magic + struct.pack("<I", 1) + payload
    raw[offset : offset + len(block) + 4] = block + struct.pack("<I", zlib.crc32(block))


def prepare_page(raw: bytes, node: int | None = None) -> tuple[bytes, dict]:
    """以本次输入的哈希调用真实工具。"""
    return prepare(raw, hashlib.sha256(raw).hexdigest(), node)


class CalibrationResetTests(unittest.TestCase):
    """固定页边界，清除资格但不更换设备、模型或应用配置。"""

    def test_preserves_all_owned_configuration_and_identity(self) -> None:
        for schema in (10, 12, 13):
            with self.subTest(schema=schema):
                raw = source_page(schema)
                before = bytes(raw)
                result, report = prepare_page(raw)
                self.assertEqual(bytes(raw), before)
                self.assertEqual(len(result), PARAM_BYTES)
                self.assertEqual(result[RECORD_BYTES:], before[RECORD_BYTES:])
                self.assertEqual(result[IDENTITY_OFFSET:], before[IDENTITY_OFFSET:])
                for start, end in (
                    (0, 36),
                    (41, 44),
                    (2092, 2136),
                    (2144, 2156),
                    (2172, 2176),
                    (2180, 2188),
                    (2208, 2240),
                ):
                    self.assertEqual(result[start:end], before[start:end])
                self.assertEqual(result[36:41], bytes(5))
                self.assertEqual(result[44:2092], bytes(2048))
                self.assertEqual(result[2136:2140], bytes(4))
                self.assertEqual(struct.unpack_from("<II", result, 2160), (13, 0x454E4332))
                crc = zlib.crc32(result[:2140] + result[2144:RECORD_BYTES])
                self.assertEqual(struct.unpack_from("<I", result, 2140)[0], crc)
                self.assertEqual(record_crc(result, RECORD_BYTES), crc)
                self.assertEqual(report["output_sha256"], hashlib.sha256(result).hexdigest())
                self.assertEqual(report["source_base_has_crc"], schema != 10)
                self.assertFalse(report["configuration_confirmed"])
                self.assertFalse(report["hardware_actions"])
                if schema == 12:
                    self.assertEqual(result[4304:4332], struct.pack("<7f", *THERMAL_DEFAULT))
                    self.assertEqual(result[4332:4352], struct.pack("<I4f", *FEEDFORWARD_DEFAULT))
                else:
                    self.assertEqual(
                        result[4304:4332], before[2352:2380] if schema == 10 else before[4304:4332]
                    )
                    self.assertEqual(
                        result[4332:4352], before[2320:2340] if schema == 10 else before[4332:4352]
                    )

    def test_invalid_hash_length_crc_and_node_are_rejected(self) -> None:
        for schema in (10, 12, 13):
            raw = source_page(schema)
            with self.assertRaises(ValueError):
                prepare(raw, "0" * 64)
            with self.assertRaises(ValueError):
                prepare_page(raw[:-1])
            if schema != 10:
                raw[2180] ^= 1
                with self.assertRaises(ValueError):
                    prepare_page(raw)
            raw = source_page(schema)
            struct.pack_into("<f", raw, 0, 0)
            refresh_crc(raw)
            with self.assertRaises(ValueError):
                prepare_page(raw)
            result, _ = prepare_page(raw, 1)
            self.assertEqual(struct.unpack_from("<f", result)[0], 1)
            self.assertEqual(result[IDENTITY_OFFSET:], raw[IDENTITY_OFFSET:])
            with self.assertRaises(ValueError):
                prepare_page(raw, 255)

    def test_legacy_extensions_must_be_valid_or_explicitly_blank(self) -> None:
        for offset, size in ((2312, 32), (2344, 40)):
            for fill in (0, 255):
                raw = source_page(10)
                raw[offset : offset + size] = bytes([fill]) * size
                prepare_page(raw)
            raw = source_page(10)
            raw[offset + size - 1] ^= 1
            with self.assertRaises(ValueError):
                prepare_page(raw)
        for schema in (10, 13):
            for thermal in (False, True):
                raw = source_page(schema)
                if schema == 10:
                    if thermal:
                        put_extension(
                            raw,
                            2344,
                            b"MTH1",
                            struct.pack("<7f", float("nan"), 7, 70, 0.8, 0.7, 1, 0.5),
                        )
                    else:
                        put_extension(
                            raw, 2312, b"SFF1", struct.pack("<I4f", 1, float("nan"), 20, 200, 2)
                        )
                else:
                    struct.pack_into("<f", raw, 4304 if thermal else 4336, float("nan"))
                    refresh_crc(raw)
                with self.assertRaises(ValueError):
                    prepare_page(raw)

    def test_float32_valid_endpoints_are_not_rejected(self) -> None:
        raw = source_page(13)
        struct.pack_into("<f", raw, 4304, 0.01)
        struct.pack_into("<f", raw, 4336, 0.2)
        struct.pack_into("<f", raw, 4340, 0.1)
        refresh_crc(raw)
        result, _ = prepare_page(raw)
        self.assertEqual(result[4304:4352], raw[4304:4352])

    def test_schema13_outputs_load_in_real_app_without_fallback(self) -> None:
        compiler = zig_path()
        if compiler is None:
            self.skipTest("未安装native C编译器")
        output = (
            ROOT / "outputs/coordination/delivery-feature-integration-20261017/calibration_reset"
        )
        output.mkdir(parents=True, exist_ok=True)
        sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
        source = output / "load_prepared.c"
        source.write_text(
            '#include <math.h>\n#include <stdio.h>\n#include <string.h>\n#include "foc_param.h"\n'
            '#include "parameter_value_service.h"\n#include "foc_sensing.h"\n'
            "#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
            + function_source(sensing, "MotorCurrentOffsets_Valid")
            + r"""
uint8_t Encoder_GetCalibFlag(const EncoderState *encoder) { return encoder->calib_flag; }
#define CHECK(condition) do { if (!(condition)) { printf("FAIL %u: %s\n", __LINE__, #condition); return 1; } } while (0)
int main(int argc, char **argv) {
    CHECK(argc == 2);
    FILE *input = fopen(argv[1], "rb"); CHECK(input != NULL);
    InterfaceParam param;
    CHECK(fread(&param, 1, sizeof(param), input) == sizeof(param)); fclose(input);
    MotorControlState motor = {0}; EncoderState encoder = {0};
    Param_Download(&param, &motor, &encoder);
    CHECK(motor.configuration_source == MOTOR_CONFIGURATION_STORED);
    CHECK(!motor.configuration_confirmed && encoder.calib_flag == 0);
    CHECK(encoder.electrical_zero_q15 == 0 && encoder.mechanical_zero_q15 == 0);
    CHECK(encoder.reverse == param.encoder_reverse);
    for (unsigned i = 0; i < ENCODER_OFFSET_LUT_SIZE; ++i) { CHECK(encoder.linearization_lut_q15[i] == 0); }
    CHECK(motor.device.node_id == (uint8_t)param.node_id);
    CHECK(motor.model.motor_pole_pairs == (int)param.motor_pole_pairs);
    CHECK(motor.model.motor_phase_resistance_ohm == param.motor_phase_resistance_ohm);
    CHECK(motor.model.motor_d_inductance_h == param.motor_d_inductance_h);
    CHECK(motor.model.motor_q_inductance_h == param.motor_q_inductance_h);
    CHECK(motor.model.motor_flux_wb == param.motor_flux_wb);
    CHECK(motor.application.current_limit_a == param.current_limit_a);
    CHECK(motor.application.speed_limit_rad_s == param.speed_limit_rad_s);
    CHECK(motor.application.speed_kp == param.speed_kp && motor.application.speed_ki == param.speed_ki);
    CHECK(motor.application.axis_profile_valid);
    CHECK(memcmp(&motor.application.axis_profile, &param.axis_profile, sizeof(param.axis_profile)) == 0);
    CHECK(memcmp(&motor.model.thermal, &param.thermal, sizeof(param.thermal)) == 0);
    CHECK(motor.application.speed_feedforward.enabled == (param.speed_feedforward_enabled == 1));
    CHECK(motor.application.speed_feedforward.gain_a_per_rad_s2 == param.speed_feedforward_gain_a_per_rad_s2);
    CHECK(motor.application.speed_feedforward.acceleration_limit_rad_s2 == param.speed_feedforward_acceleration_rad_s2);
    CHECK(motor.application.speed_feedforward.jerk_limit_rad_s3 == param.speed_feedforward_jerk_rad_s3);
    CHECK(motor.application.speed_feedforward.current_limit_a == param.speed_feedforward_current_limit_a);
    puts("PASS prepared schema13 real APP load, configuration preserved and calibration cleared");
    return 0;
}
""",
            encoding="utf-8",
        )
        executable = output / "load_prepared.exe"
        command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
        command += NATIVE_INCLUDE_FLAGS + [
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DCURRENT_SENSE_SHUNT_MILLIOHM=6U",
            str(source),
        ]
        command += [
            str(ROOT / path)
            for path in (
                "firmware/services/parameters/foc_param.c",
                "firmware/services/parameters/parameter_value_service.c",
                "firmware/services/parameters/motor_axis_profile.c",
                "firmware/motor/math/control_math.c",
                "firmware/motor/protection/motor_thermal.c",
                "firmware/motor/protection/motor_thermal_config.c",
                "firmware/motor/foc/speed_feedforward.c",
                "shared/checksum/crc32.c",
            )
        ] + ["-lm", "-o", str(executable)]
        log = []
        compiled = subprocess.run(command, capture_output=True, text=True, cwd=ROOT)
        log.append(compiled.stdout + compiled.stderr)
        self.assertEqual(compiled.returncode, 0, log[-1])
        for schema in (10, 12, 13):
            raw = source_page(schema)
            result, report = prepare_page(raw)
            path = output / f"from_schema{schema}.bin"
            path.write_bytes(result)
            loaded = subprocess.run(
                [str(executable), str(path)], capture_output=True, text=True, cwd=ROOT
            )
            log.append(f"schema{schema}: " + loaded.stdout + loaded.stderr)
            self.assertEqual(loaded.returncode, 0, log[-1])
            self.assertEqual(report["output_sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
        (output / "cross_language.log").write_text("\n".join(log), encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
