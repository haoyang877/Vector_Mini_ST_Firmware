"""使用真实运行所有者验证命令、维护和取消；可与固定提交逐事件对照，不访问硬件。"""

import argparse
import difflib
import hashlib
import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, native_include_flags, resolve_source_path
from run_position_servo_tests import function_source

REFERENCE_PATHS = (
    "firmware/app",
    "firmware/motor",
    "firmware/parameters",
    "firmware/services",
    "firmware/communication",
    "firmware/platform/bsp/include",
    "firmware/config",
    "firmware/platform/config",
    "firmware/platform/bsp/mcu/stm32g4",
    "shared",
)
OWNER_SOURCES = (
    "firmware/app/motor_state_machine/motor_state_machine.c",
    "firmware/app/motor_state_machine/motor_state_rules.c",
    "firmware/motor/protection/motor_fault.c",
    "firmware/motor/control/speed_feedforward.c",
    "firmware/motor/protection/motor_thermal.c",
    "firmware/motor/protection/motor_thermal_config.c",
    "firmware/motor/protection/motor_thermal_control.c",
    "firmware/parameters/motor_axis_profile.c",
    "firmware/parameters/parameter_values.c",
    "shared/checksum/crc32.c",
)


LEGACY_SOURCES = {
    "firmware/motor/protection/motor_fault.c": ("firmware/motor/protection/foc_errhandle.c",),
    "firmware/motor/protection/motor_thermal_control.c": (
        "firmware/app/runtime/motor_thermal_control.c",
    ),
    "firmware/motor/control/speed_feedforward.c": ("firmware/motor/foc/speed_feedforward.c",),
    "firmware/parameters/motor_axis_profile.c": (
        "firmware/services/parameters/motor_axis_profile.c",
    ),
    "firmware/parameters/parameter_values.c": (
        "firmware/services/parameters/parameter_value_service.c",
    ),
}


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
    paths = [
        path
        for path in REFERENCE_PATHS
        if subprocess.run(
            ["git", "cat-file", "-e", revision + ":" + path],
            cwd=ROOT,
            capture_output=True,
            check=False,
        ).returncode
        == 0
    ]
    payload = subprocess.check_output(
        ["git", "archive", "--format=tar", revision, *paths], cwd=ROOT
    )
    destination.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:") as archive:
        archive.extractall(destination, filter="data")
    return revision, destination


def adapt_measurement_fixture(fixture: str, declare: bool = True) -> str:
    """将相同测试输入投影到新测量对象；冻结参考仍使用自己的旧结构。"""
    if declare:
        fixture = fixture.replace(
            "static FocState foc;", "static FocState foc;\nstatic MotorMeasurement measurement;"
        )
        fixture = fixture.replace(
            "#define s_foc_state foc\n",
            "#define s_foc_state foc\n#define s_motor_measurement measurement\n",
        )
        fixture = fixture.replace(
            "memset(&foc, 0, sizeof(foc));",
            "memset(&foc, 0, sizeof(foc));\n    memset(&measurement, 0, sizeof(measurement));",
        )
        fixture = fixture.replace(
            "void McuTemperature_Update(FocState *state,",
            "void McuTemperature_Update(MotorMeasurement *state,",
        ).replace(
            "state == &foc && control == &motor", "state == &measurement && control == &motor"
        )
        fixture = fixture.replace(
            "FocState *f,\n                                               MotorControlState *m,",
            "FocState *f,\n                                               const MotorMeasurement *measurement_input,\n                                               MotorControlState *m,",
        ).replace(
            "FocState *f,\n                                                          MotorControlState *m,",
            "FocState *f,\n                                                          const MotorMeasurement *measurement_input,\n                                                          MotorControlState *m,",
        )
        fixture = fixture.replace(
            "    (void)f;\n    (void)m;", "    (void)f;\n    (void)measurement_input;\n    (void)m;"
        )
    for field in (
        "bus_voltage_v",
        "bus_voltage_filtered_v",
        "phase_a_current_a",
        "phase_b_current_a",
        "phase_c_current_a",
        "temperature_c",
    ):
        fixture = fixture.replace("foc." + field, "measurement." + field)
    fixture = fixture.replace(
        "MotorStateMachine_Init(&cfg, &motor, &foc,",
        "MotorStateMachine_Init(&cfg, &motor, &foc, &measurement,",
    )
    fixture = fixture.replace(
        "&foc, &motor, &pi, &encoder, NULL, &startup",
        "&foc, &measurement, &motor, &pi, &encoder, NULL, &startup",
    )
    import re

    fixture = re.sub(
        r"(MotorCommand_ValidateTarget\([^,]+,\s*[^,]+,)\s*&foc,", r"\1 &measurement,", fixture
    )
    return fixture


def owner_fixture(tree: Path) -> str:
    """夹具一致，准入计算从各自生产版本读取。"""
    fixture = (ROOT / "tests/unit/native/run_state_owner_test.c").read_text(encoding="utf-8")
    if not (tree / "firmware/motor/sensing/motor_sensing.h").is_file():
        fixture = fixture.replace('"motor_sensing.h"', '"foc_sensing.h"')
    if not (tree / "firmware/motor/feedback/encoder_feedback.h").is_file():
        fixture = "#define Foc_MaxVoltage FOC_MaximumVoltage\n" + fixture
        fixture = "#define Foc_ConfigureControllers FOC_ConfigureControllers\n" + fixture
        fixture = "#define Foc_UpdateCurrentLimits FOC_UpdateCurrentControllerLimits\n" + fixture
        fixture = "#define Foc_Step FOC_Step\n" + fixture
        fixture = "#define Foc_RunCurrentRef FOC_CurrentWithReference\n" + fixture
        fixture = "#define Foc_ResetCurrent FOC_CurrentController_Reset\n" + fixture
        fixture = "#define Foc_RunCurrent FOC_Current\n" + fixture
        fixture = "#define Foc_RunVq FOC_Vq_Mode\n" + fixture
        fixture = "#define Svm_ZeroInjection SVM_ZeroInjection\n" + fixture
        fixture = "#define Svm_SectorJudge SVM_SectorJudge\n" + fixture
        fixture = "#define Clarke_Inverse Inverse_Clarke_Transform\n" + fixture
        fixture = "#define Park_Inverse Inverse_Park_Transform\n" + fixture
        fixture = "#define Encoder_GetCprRatio Encoder_GetCountInCPR_Ratio\n" + fixture
        fixture = "#define Encoder_GetCalibrationFlag Encoder_GetCalibFlag\n" + fixture
        fixture = "#define SpeedControl_UpdateRamp MotorAxis_UpdateSpeedRamp\n" + fixture
        fixture = "#define MotorCurrentOffsets_IsValid MotorCurrentOffsets_Valid\n" + fixture
        fixture = "#define Encoder_InitFeedback Encoder_ParamInit\n" + fixture
        fixture = "#define FluxObserver_Init FluxObserver_ParamInit\n" + fixture
        fixture = "#define MotorCurrent_Update Current_Cal\n" + fixture
        fixture = "#define MotorVoltage_Update Vbus_Update\n" + fixture
        fixture = "#define McuTemperature_Update Temperature_Update\n" + fixture
        fixture = "#define McuTemperature_IsFresh MotorTemperature_IsFresh\n" + fixture
        fixture = "#define Encoder_GetElecVel Encoder_GetEleVel\n" + fixture
        fixture = "#define Encoder_GetMechVel Encoder_GetMecVel\n" + fixture
        fixture = "#define Encoder_GetMechVelContinuous Encoder_GetMecVelContinuous\n" + fixture
        fixture = "#define Encoder_GetElecPhase Encoder_GetElePhase\n" + fixture
        fixture = "#define Encoder_GetMechPos Encoder_GetMecPos\n" + fixture
        fixture = "#define FluxObserver_GetElecPhase Observer_GetElePhase\n" + fixture
        fixture = "#define FluxObserver_GetElecVel Observer_GetEleVel\n" + fixture
        fixture = "#define FluxObserver_GetElecPos Observer_GetElePosition\n" + fixture
        fixture = "#define FluxObserver_GetPositionEpoch Observer_GetPositionEpoch\n" + fixture
        fixture = fixture.replace('"encoder_feedback.h"', '"angle_feedback.h"')
        fixture = fixture.replace('"encoder_calibration.h"', '"encoder_calibration_task.h"')
    if not (tree / "firmware/motor/control/foc.h").is_file():
        fixture = fixture.replace('"foc.h"', '"foc_algorithm.h"')
        fixture = fixture.replace('"pi_controller.h"', '"foc_pid.h"')
        fixture = fixture.replace('"position_control.h"', '"position_cascade.h"')
    algorithm = resolve_source_path(
        tree, "firmware/motor/control/foc.c", "firmware/motor/foc/foc_algorithm.c"
    ).read_text(encoding="utf-8")
    sensing = resolve_source_path(
        tree,
        "firmware/motor/sensing/motor_current.c",
        "firmware/motor/sensing/motor_sensing.c",
        "firmware/motor/foc/foc_sensing.c",
    ).read_text(encoding="utf-8")
    fixture += "\n" + function_source(
        algorithm, "Foc_MaxVoltage" if "Foc_MaxVoltage(" in algorithm else "FOC_MaximumVoltage"
    )
    fixture += (
        "\n#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
    )
    fixture += function_source(
        sensing,
        "MotorCurrentOffsets_IsValid"
        if "MotorCurrentOffsets_IsValid(" in sensing
        else "MotorCurrentOffsets_Valid",
    )
    command_path = resolve_source_path(
        tree, "firmware/app/runtime/motor_command.c", "firmware/app/runtime/motor_execution.c"
    )
    execution = command_path.read_text(encoding="utf-8")
    command_name = (
        "MotorCommand_ValidateTarget"
        if "MotorCommand_ValidateTarget(" in execution
        else "MotorExecution_ValidateTarget"
    )
    fixture += "\n" + function_source(execution, command_name)
    if "MotorStateMachine_PrepareSupervisor" in (tree / OWNER_SOURCES[0]).read_text(
        encoding="utf-8"
    ):
        axis = (tree / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
        fixture += "\n#define s_motor_configuration cfg\n#define s_motor_state motor\n"
        fixture += (
            "#define s_foc_state foc\n#define s_speed_pi pi\n#define s_rotor_encoder encoder\n"
        )
        if "finalize_current_offsets();" in axis:
            # 此夹具直接提供零偏条件，采样/延后发布由真实startup_current_offsets用例覆盖。
            fixture += "static void finalize_current_offsets(void) {}\n"
        if "s_status_revision" in axis:
            fixture += "static volatile uint32_t s_status_revision;\n"
        fixture += function_source(axis, "MotorAxis_SupervisorTick")
    adapter = (tree / "firmware/app/composition/yg_protocol_status_adapter.c").read_text(
        encoding="utf-8"
    )
    fixture += "\n" + function_source(adapter, "protocol_mode")
    fixture += "\n" + function_source(adapter, "protocol_state")
    # 夹具只改头名和接口别名；旧生产函数体、配置及常量仍来自同一冻结树。
    headers = {
        "motor_command.h": ("firmware/app/runtime/motor_command.h", "motor_execution.h"),
        "motor_control.h": (
            "firmware/motor/control/motor_control.h",
            "motor_control_execution.h",
        ),
        "motor_fault.h": ("firmware/motor/protection/motor_fault.h", "foc_errhandle.h"),
        "sensorless_startup.h": (
            "firmware/motor/control/sensorless_startup.h",
            "foc_sensorless_run.h",
        ),
        "parameter_values.h": (
            "firmware/parameters/parameter_values.h",
            "parameter_value_service.h",
        ),
    }
    for current, (relative, legacy) in headers.items():
        if (
            not (tree / relative).is_file()
            and not (
                tree / relative.replace("/interface/", "/foc/").replace("/control/", "/foc/")
            ).is_file()
        ):
            if (
                current == "motor_control.h"
                and not (tree / "firmware/motor/foc/motor_control_execution.h").is_file()
            ):
                legacy = "motor_execution.h"
            fixture = fixture.replace(f'#include "{current}"', f'#include "{legacy}"')
    if (tree / "firmware/motor/sensing/motor_measurement.h").is_file():
        fixture = adapt_measurement_fixture(fixture)
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


def run_variant(
    tree: Path,
    output: Path,
    compiler: list[str],
    maintenance: Path,
    optional_temperature: bool = False,
) -> dict:
    """各版本分别编译 owner 和 rules，保留所有旧断言。"""
    output.mkdir(parents=True, exist_ok=True)
    source = output / "run_state_owner.c"
    owner_path = tree / OWNER_SOURCES[0]
    owner_text = owner_path.read_text(encoding="utf-8")
    fast_outcome_unlocked = "BspSystem_EnterControlCritical" not in function_source(
        owner_text, "MotorStateMachine_PostOutcome"
    )
    take_result_source = (
        function_source(owner_text, "take_current_task_result")
        if "take_current_task_result(" in owner_text
        else ""
    )
    take_result_unlocked = bool(take_result_source) and (
        "BspSystem_EnterControlCritical" not in take_result_source
    )
    shared_mailbox_source = (
        function_source(owner_text, "take_pending_outcome")
        if "take_pending_outcome(" in owner_text
        else ""
    )
    mailbox_source = shared_mailbox_source or take_result_source
    mailbox_atomic = "BspSystem_EnterCritical()" in mailbox_source
    discard_source = (
        function_source(owner_text, "discard_pending_outcome")
        if "discard_pending_outcome(" in owner_text
        else ""
    )
    discard_atomic = mailbox_atomic and (
        bool(shared_mailbox_source) or "BspSystem_EnterCritical()" in discard_source
    )
    start_commit_guarded = "start_commit_mask" in owner_text
    preemptible = "read_locked_state_conditions" in owner_text
    health_inputs_only = "state_inputs_valid()" in function_source(
        owner_text, "MotorStateMachine_ReadBootHealth"
    )
    fixture = owner_fixture(tree)
    if optional_temperature:
        # 新板使用真实默认值，防止夹具手填3A掩盖默认配置超限。
        fixture = fixture.replace(
            "cfg.application.current_limit_a = 6.0f;",
            "cfg.application.current_limit_a = PARAM_APP_DEFAULT_CURRENT_LIMIT_A;",
        )
        case = ROOT / "tests/unit/native/mcu_temperature_owner_case.c"
        case_text = case.read_text(encoding="utf-8")
        if (tree / "firmware/motor/sensing/motor_measurement.h").is_file():
            case_text = adapt_measurement_fixture(case_text, declare=False)
        fixture = (
            "#define main required_temperature_main\n" + fixture + "\n#undef main\n" + case_text
        )

    source.write_text(
        f"#define STATE_NO_NESTED_CONDITION_LOCKS {int('read_state_conditions_locked' in owner_text)}\n"
        + f"#define STATE_FAST_OUTCOME_UNLOCKED {int(fast_outcome_unlocked)}\n"
        + f"#define STATE_TAKE_RESULT_UNLOCKED {int(take_result_unlocked)}\n"
        + f"#define STATE_MAILBOX_ATOMIC {int(mailbox_atomic)}\n"
        + f"#define STATE_DISCARD_ATOMIC {int(discard_atomic)}\n"
        + f"#define STATE_START_COMMIT_GUARDED {int(start_commit_guarded)}\n"
        + f"#define STATE_TARGET_COMMIT_GUARDED {int('target_commit_mask' in owner_text)}\n"
        + f"#define STATE_FAST_POWER_OWNER {int('MotorStateMachine_BeginFastTick' in owner_text)}\n"
        + f"#define STATE_VALIDATION_PREEMPTIBLE {int(preemptible)}\n"
        + f"#define STATE_SUPERVISOR_STAGED {int('MotorStateMachine_PrepareSupervisor' in owner_text)}\n"
        + f"#define STATE_PUBLICATION_PREVALIDATED {int('step->configuration_valid' in owner_text)}\n"
        + f"#define STATE_BOOT_HEALTH_INPUTS_ONLY {int(health_inputs_only)}\n"
        + f"#define STATE_START_ADMISSION_PREVALIDATED {int('prepare_start_configuration' in owner_text)}\n"
        + f"#define STATE_START_SUPERVISOR_PREVALIDATED {int('startup_configuration_stable' in owner_text)}\n"
        + f"#define STATE_CALIBRATION_GUARD_PREVALIDATED {int('encoder_calibration_poll' in owner_text)}\n"
        + f"#define STATE_MAINTENANCE_ADMISSION_PREVALIDATED {int('request_maintenance_locked' in owner_text)}\n"
        + fixture,
        encoding="utf-8",
    )
    # 仅替换实际校验调用名用于观测屏蔽状态；校验主体仍链接该版本的真实实现。
    instrumented_owner = output / "instrumented_owner.c"
    tested_owner = owner_path
    owner_probe = owner_text
    if mailbox_atomic:
        marker = "        s_context.has_latched_outcome = false;"
        if mailbox_source.count(marker) != 1:
            raise ValueError("无法定位结果邮箱消费点")
        instrumented_take = mailbox_source.replace(
            marker, "        Test_MailboxInterrupt();\n" + marker
        )
        owner_probe = owner_probe.replace(mailbox_source, instrumented_take, 1)
    if discard_atomic and not shared_mailbox_source:
        if discard_source.count(marker) != 1:
            raise ValueError("无法定位结果邮箱撤销点")
        instrumented_discard = discard_source.replace(
            marker, "        Test_MailboxInterrupt();\n" + marker
        )
        owner_probe = owner_probe.replace(discard_source, instrumented_discard, 1)
    if start_commit_guarded:
        start_source = function_source(owner_text, "update_start_progress")
        before = "    uint32_t start_commit_mask = BspSystem_EnterCritical();"
        publish = (
            "            finish_pending_start(MOTOR_CONTROL_ACCEPTED);"
            if "finish_pending_start(MOTOR_CONTROL_ACCEPTED)" in start_source
            else "            s_context.active_start.state = MOTOR_CONTROL_START_STARTED;"
        )
        if start_source.count(before) != 1 or start_source.count(publish) != 1:
            raise ValueError("无法定位启动进度提交点")
        instrumented_start = start_source.replace(
            before, "    Test_StartProgressInterrupt(1U);\n" + before
        ).replace(publish, "            Test_StartProgressInterrupt(2U);\n" + publish)
        owner_probe = owner_probe.replace(start_source, instrumented_start, 1)
    if mailbox_atomic or start_commit_guarded:
        tested_owner = output / "owner_preemption_probe.c"
        tested_owner.write_text(owner_probe, encoding="utf-8")
    instrumented_owner.write_text(
        "#define ParameterValue_ConfigurationValid Test_ConfigurationValid\n"
        + ("void Test_MailboxInterrupt(void);\n" if mailbox_atomic else "")
        + ("void Test_StartProgressInterrupt(unsigned stage);\n" if start_commit_guarded else "")
        + f'#include "{tested_owner.as_posix()}"\n'
        + (
            "MotorTaskResult Test_TakeCurrentTaskResult(void) "
            "{ return take_current_task_result(); }\n"
            if take_result_unlocked
            else ""
        )
        + (
            "bool Test_MailboxHasPending(void) { return s_context.has_latched_outcome; }\n"
            if mailbox_atomic
            else ""
        ),
        encoding="utf-8",
    )
    includes = native_include_flags(tree)
    # 冻结提交使用重构前的配置目录；基线始终包含该提交自己的头文件。
    for relative in (
        "firmware/app/config",
        "firmware/app/config/targets",
        "firmware/platform/config",
        "firmware/platform/config/boards",
        "firmware/motor/profiles",
    ):
        if (tree / relative).is_dir():
            includes += ["-I", str(tree / relative)]
    executable = output / "run_state_owner.exe"
    production = [
        resolve_source_path(tree, path, *LEGACY_SOURCES.get(path, ())) for path in OWNER_SOURCES
    ]
    production[0] = instrumented_owner
    production += [
        maintenance,
        maintenance.parent / "maintenance_workspace.c",
        ROOT / "tests/unit/bsp_preemption_stubs.c",
    ]
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
            str(source),
        ]
        + (
            [
                "-DFOC_ACTIVE_HW_PROFILE=2",
                "-DR40_APP_PHASE_LOCK=1",
                "-DBSP_PWM_HARD_LOCK=1",
                "-DBSP_REQUIRED_MCU_TEMPERATURE=0",
            ]
            if optional_temperature
            else ["-DCURRENT_SENSE_SHUNT_MILLIOHM=6U"]
        )
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
        + (
            ["-DMOTOR_RULES_BASELINE"]
            if "request_epoch"
            in (tree / "firmware/app/motor_state_machine/motor_state_internal.h").read_text(
                encoding="utf-8"
            )
            else []
        )
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
        optional = run_variant(
            ROOT,
            output / "r40_optional_temperature",
            compiler,
            maintenance,
            optional_temperature=True,
        )
        summary["r40_optional_temperature"] = optional["owner"]
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
