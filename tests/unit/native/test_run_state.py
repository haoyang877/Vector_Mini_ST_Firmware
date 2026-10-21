"""使用真实运行所有者验证命令、维护和取消；可与固定提交逐事件对照，不访问硬件。"""

import argparse
import difflib
import hashlib
import io
import json
import subprocess
import tarfile
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source

REFERENCE_PATHS = (
    "firmware/app",
    "firmware/motor",
    "firmware/services",
    "firmware/communication",
    "firmware/platform/api",
    "firmware/platform/stm32g4/bsp",
    "shared",
)
OWNER_SOURCES = (
    "firmware/app/motor_state_machine/motor_state_machine.c",
    "firmware/app/motor_state_machine/motor_state_rules.c",
    "firmware/motor/protection/foc_errhandle.c",
    "firmware/motor/foc/speed_feedforward.c",
    "firmware/motor/protection/motor_thermal.c",
    "firmware/motor/protection/motor_thermal_config.c",
    "firmware/app/runtime/motor_thermal_control.c",
    "firmware/services/parameters/motor_axis_profile.c",
    "firmware/services/parameters/parameter_value_service.c",
    "shared/checksum/crc32.c",
)


def git_text(*arguments: str) -> str:
    """读取版本身份；不修改工作区或索引。"""
    return subprocess.check_output(["git", *arguments], cwd=ROOT, text=True).strip()


def file_hash(path: Path) -> str:
    """对实际编译输入记录 SHA256。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export_reference(reference: str, output: Path) -> tuple[str, Path]:
    """只导出固定提交的生产源和头文件；基线不读取当前生产代码。"""
    revision = git_text("rev-parse", "--verify", reference + "^{commit}")
    destination = output / "reference" / revision
    if destination.exists():
        raise ValueError(f"基线导出目录已存在，请为本轮选择新的 --out：{destination}")
    payload = subprocess.check_output(
        ["git", "archive", "--format=tar", revision, *REFERENCE_PATHS], cwd=ROOT
    )
    destination.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:") as archive:
        archive.extractall(destination, filter="data")
    return revision, destination


def owner_fixture(tree: Path) -> str:
    """夹具一致，准入计算从各自生产版本读取。"""
    fixture = (ROOT / "tests/unit/native/run_state_owner_test.c").read_text(encoding="utf-8")
    algorithm = (tree / "firmware/motor/foc/foc_algorithm.c").read_text(encoding="utf-8")
    sensing = (tree / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    fixture += "\n" + function_source(algorithm, "FOC_MaximumVoltage")
    fixture += (
        "\n#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
    )
    fixture += function_source(sensing, "MotorCurrentOffsets_Valid")
    execution = (tree / "firmware/app/runtime/motor_execution.c").read_text(encoding="utf-8")
    fixture += "\n" + function_source(execution, "MotorExecution_ValidateTarget")
    adapter = (tree / "firmware/app/composition/yg_protocol_status_adapter.c").read_text(
        encoding="utf-8"
    )
    fixture += "\n" + function_source(adapter, "protocol_mode")
    fixture += "\n" + function_source(adapter, "protocol_state")
    return fixture


def invoke(command: list[str], log: Path) -> str:
    """保留完整命令和结果；失败时只向终端输出末尾。"""
    result = subprocess.run(command, capture_output=True, text=True)
    text = result.stdout + result.stderr
    log.write_text(json.dumps(command, ensure_ascii=True) + "\n" + text, encoding="utf-8")
    if result.returncode:
        print("\n".join(text.splitlines()[-25:]))
        raise subprocess.CalledProcessError(result.returncode, command)
    return text


def run_variant(tree: Path, output: Path, compiler: list[str], maintenance: Path) -> dict:
    """各版本分别编译 owner 和 rules，保留所有旧断言。"""
    output.mkdir(parents=True, exist_ok=True)
    source = output / "run_state_owner.c"
    owner_path = tree / OWNER_SOURCES[0]
    preemptible = "read_locked_state_conditions" in owner_path.read_text(encoding="utf-8")
    source.write_text(
        f"#define STATE_VALIDATION_PREEMPTIBLE {int(preemptible)}\n" + owner_fixture(tree),
        encoding="utf-8",
    )
    # 仅替换实际校验调用名用于观测屏蔽状态；校验主体仍链接该版本的真实实现。
    instrumented_owner = output / "instrumented_owner.c"
    instrumented_owner.write_text(
        "#define ParameterValue_ConfigurationValid Test_ConfigurationValid\n"
        f'#include "{owner_path.as_posix()}"\n',
        encoding="utf-8",
    )
    includes = [
        flag if flag == "-I" else str(tree / Path(flag).relative_to(ROOT))
        for flag in NATIVE_INCLUDE_FLAGS
    ]
    executable = output / "run_state_owner.exe"
    production = [tree / path for path in OWNER_SOURCES]
    production[0] = instrumented_owner
    production += [maintenance, maintenance.parent / "maintenance_workspace.c"]
    command = (
        compiler
        + ["-I", str(maintenance.parent)]
        + includes
        + [
            "-std=c99",
            "-O1",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DCURRENT_SENSE_SHUNT_MILLIOHM=6U",
            str(source),
        ]
        + [str(path) for path in production]
        + ["-o", str(executable)]
    )
    invoke(command, output / "owner_compile.log")
    owner = invoke([str(executable)], output / "owner_run.log")
    rules_executable = output / "motor_state_rules.exe"
    rules_source = ROOT / "tests/unit/native/motor_state_rules_test.c"
    command = (
        compiler
        + includes
        + [
            "-std=c99",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(rules_source),
            str(tree / "firmware/app/motor_state_machine/motor_state_rules.c"),
            "-o",
            str(rules_executable),
        ]
    )
    invoke(command, output / "rules_compile.log")
    rules = invoke([str(rules_executable)], output / "rules_run.log")
    # 导出的所有头文件也记录在内，避免只凭 owner SHA 推断实际输入。
    input_hashes = {
        path.relative_to(tree).as_posix(): file_hash(path)
        for relative in REFERENCE_PATHS
        for path in sorted((tree / relative).rglob("*"))
        if path.suffix in (".c", ".h")
    }
    (output / "input_hashes.json").write_text(
        json.dumps(input_hashes, indent=2, sort_keys=True), encoding="utf-8"
    )
    return {"owner": owner, "rules": rules}


def main() -> None:
    """运行定向夹具，选择固定基线时逐字比较观察轨迹。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/run_state_tests")
    parser.add_argument("--reference-ref", help="固定生产基线提交；夹具仍使用本轮版本")
    default_maintenance = ROOT / "firmware/app/runtime/motor_maintenance.c"
    parser.add_argument("--maintenance-source", type=Path, default=default_maintenance)
    args = parser.parse_args()
    maintenance = args.maintenance_source.resolve()
    if args.reference_ref and maintenance != default_maintenance.resolve():
        parser.error("--reference-ref 不允许使用外部 --maintenance-source")
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    summary = {
        "current_head": git_text("rev-parse", "HEAD"),
        "source_dirty": bool(git_text("status", "--porcelain")),
        "fixture_sha256": {
            path: file_hash(ROOT / "tests/unit/native" / path)
            for path in ("run_state_owner_test.c", "motor_state_rules_test.c", "test_run_state.py")
        },
        "status": "running",
        "hardware_access": False,
    }
    try:
        reference = None
        if args.reference_ref:
            revision, tree = export_reference(args.reference_ref, output)
            summary["reference_commit"] = revision
            reference = run_variant(
                tree,
                output / "baseline",
                compiler,
                tree / "firmware/app/runtime/motor_maintenance.c",
            )
        current = run_variant(ROOT, output / "current", compiler, maintenance)
        if reference is not None:
            differences = []
            for name in ("owner", "rules"):
                matched = reference[name] == current[name]
                summary[name + "_equal"] = matched
                if not matched:
                    differences.extend(
                        difflib.unified_diff(
                            reference[name].splitlines(True),
                            current[name].splitlines(True),
                            fromfile="baseline/" + name,
                            tofile="current/" + name,
                        )
                    )
            if differences:
                (output / "behavior.diff").write_text("".join(differences), encoding="utf-8")
                raise ValueError("固定基线与当前观察结果不一致，见 behavior.diff")
        summary["status"] = "passed"
        print("PASS run-state owner assertions and 262144 rules cases")
        if reference is not None:
            print("PASS fixed-reference owner/power/event traces and rules output match")
    except (subprocess.CalledProcessError, ValueError) as error:
        summary["status"] = "failed"
        summary["error"] = str(error)
        raise
    finally:
        (output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=True), encoding="utf-8"
        )


if __name__ == "__main__":
    main()
