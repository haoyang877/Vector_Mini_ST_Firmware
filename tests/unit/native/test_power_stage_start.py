"""对照本地官方 HAL 原函数，验证真实功率级端口的六步 MMIO/软件状态序列。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

from project_paths import ROOT
from run_position_servo_tests import function_source

HAL = ROOT / "firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver"
CMSIS = ROOT / "firmware/platform/stm32g4/cubemx/Drivers/CMSIS/Device/ST/STM32G4xx/Include"
PORT = ROOT / "firmware/platform/stm32g4/ports/motor/power_stage_stm32g4.c"
NATIVE = ROOT / "tests/unit/native"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def macro_source(source: str, name: str) -> str:
    """保留官方宏原文及续行；不把待测实现作为参考。"""
    lines = source.splitlines(keepends=True)
    for index, line in enumerate(lines):
        if re.match(r"#define\s+" + re.escape(name) + r"(?=\s|\(|$)", line):
            end = index + 1
            while lines[end - 1].rstrip().endswith("\\"):
                end += 1
            return "".join(lines[index:end]).rstrip()
    raise ValueError("Official macro not found: " + name)


def enum_source(source: str, name: str) -> str:
    match = re.search(r"typedef\s+enum\s*\{[^}]*\}\s*" + name + r"\s*;", source)
    if match is None:
        raise ValueError("Official enum not found: " + name)
    return match.group()


def prepare_reference(out: Path) -> dict:
    inputs = {}

    def read(path: Path) -> str:
        data = path.read_bytes()
        inputs[path.relative_to(ROOT).as_posix()] = sha256(data)
        return data.decode("utf-8")

    sources = {
        "tim": read(HAL / "Src/stm32g4xx_hal_tim.c"),
        "ex": read(HAL / "Src/stm32g4xx_hal_tim_ex.c"),
    }
    functions = (
        ("tim", "TIM_CCxChannelCmd"),
        ("ex", "TIM_CCxNChannelCmd"),
        ("tim", "HAL_TIM_PWM_Start"),
        ("ex", "HAL_TIMEx_OCN_Start"),
        ("tim", "HAL_TIM_PWM_Stop"),
        ("ex", "HAL_TIMEx_OCN_Stop"),
    )
    extracted = [(name, function_source(sources[key], name)) for key, name in functions]
    reference = "\n\n".join(body for _, body in extracted)
    for _, name in functions:
        reference = re.sub(r"\b" + name + r"\b", "Ref_" + name, reference)
    reference = "/* Extracted from local ST HAL; only symbol names are prefixed. */\n" + reference
    reference_path = out / "official_hal_reference.inc"
    reference_path.write_text(reference + "\n", encoding="utf-8")

    tim = read(HAL / "Inc/stm32g4xx_hal_tim.h")
    device = read(CMSIS / "stm32g431xx.h")
    definitions = read(HAL / "Inc/stm32g4xx_hal_def.h")
    macros = [
        *(f"TIM_CHANNEL_{number}" for number in range(1, 7)),
        "TIM_CCx_ENABLE",
        "TIM_CCx_DISABLE",
        "TIM_CCxN_ENABLE",
        "TIM_CCxN_DISABLE",
        "__HAL_TIM_ENABLE",
        "__HAL_TIM_MOE_ENABLE",
        "__HAL_TIM_DISABLE",
        "__HAL_TIM_MOE_DISABLE",
        "TIM_CCER_CCxE_MASK",
        "TIM_CCER_CCxNE_MASK",
        "IS_TIM_SLAVEMODE_TRIGGER_ENABLED",
        "TIM_SLAVEMODE_TRIGGER",
        "TIM_SLAVEMODE_COMBINED_RESETTRIGGER",
        "TIM_CHANNEL_STATE_GET",
        "TIM_CHANNEL_STATE_SET",
        "TIM_CHANNEL_N_STATE_GET",
        "TIM_CHANNEL_N_STATE_SET",
    ]
    header = [
        "/* Unmodified definitions extracted from local ST headers. */",
        enum_source(definitions, "HAL_StatusTypeDef"),
        enum_source(tim, "HAL_TIM_ChannelStateTypeDef"),
        *(macro_source(tim, name) for name in macros),
    ]
    bits = re.findall(
        r"^#define\s+(TIM_(?:CR1_CEN\w*|BDTR_MOE\w*|SMCR_SMS\w*|CCER_CC[1-4]N?E\w*))\b",
        device,
        re.M,
    )
    header.extend(macro_source(device, name) for name in bits)
    selectors = [
        macro_source(device, "IS_TIM_BREAK_INSTANCE"),
        macro_source(device, "IS_TIM_SLAVE_INSTANCE"),
    ]
    # Other instance addresses are identity-only stubs, never dereferenced.
    other_instances = sorted(set(re.findall(r"\bTIM\d+\b", "\n".join(selectors))) - {"TIM1", "TIM8"})
    for index, name in enumerate(other_instances):
        header.append(f"#define {name} ((TIM_TypeDef *)(uintptr_t){0x1000 + index * 0x100}U)")
    header.extend(selectors)
    header_path = out / "official_tim_defs.h"
    header_path.write_text("\n\n".join(header) + "\n", encoding="utf-8")
    return {
        "official_input_sha256": inputs,
        "extracted_function_sha256": {name: sha256(body.encode()) for name, body in extracted},
        "generated_reference_sha256": {
            path.name: sha256(path.read_bytes()) for path in (reference_path, header_path)
        },
        "reference_transform": "Extract complete original functions/macros; prefix function symbols only.",
        "model_limits": [
            "Native register-access and software-state model; no MCU timing or electrical waveform claim.",
            "Break cases force MOE clear after writes; they test repeated MOE writes, not full Break hardware.",
            "Caller serialization is required; no concurrent CPU mutation of HAL channel states is modeled.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = prepare_reference(out)
    files = [PORT, NATIVE / "power_stage_start_test.cpp", NATIVE / "power_stage_fake/tim.h", Path(__file__)]
    manifest["tested_source_sha256"] = {
        path.relative_to(ROOT).as_posix(): sha256(path.read_bytes()) for path in files
    }
    (out / "reference-manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    exe = out / "power_stage_start.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command = compiler + [
        "-x", "c++", "-std=c++17", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror",
        "-fno-exceptions", "-fno-rtti",
        "-I", str(out), "-I", str(NATIVE / "power_stage_fake"),
        "-I", str(ROOT / "firmware/platform/api"),
        str(NATIVE / "power_stage_start_test.cpp"), str(PORT), "-o", str(exe),
    ]
    build = subprocess.run(command, check=True, capture_output=True, text=True)
    (out / "compile.log").write_text(build.stdout + build.stderr, encoding="utf-8")
    result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    (out / "run.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    summary = {
        "status": "PASS", "compile_command": command, "stdout": result.stdout.strip(),
        "reference_manifest_sha256": sha256((out / "reference-manifest.json").read_bytes()),
        "executable_sha256": sha256(exe.read_bytes()), "hardware_access": False,
    }
    (out / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(result.stdout, end="")


if __name__ == "__main__":
    main()
