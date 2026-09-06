param(
    [string]$Compiler,
    [switch]$BuildOnly,
    [ValidateSet('Damped', 'NoDamper')]
    [string]$ProductVariant = 'Damped'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repositoryRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$buildDirectory = Join-Path $PSScriptRoot '.build'
$variantSlug = $ProductVariant.ToLowerInvariant()
$binaryName = if ([System.Environment]::OSVersion.Platform -eq
    [System.PlatformID]::Win32NT) { "firmware_host_tests_$variantSlug.exe" } else {
    "firmware_host_tests_$variantSlug"
}
$binaryPath = Join-Path $buildDirectory $binaryName
$variantDefine = switch ($ProductVariant) {
    'Damped' { 'PRODUCT_CATALOG_VARIANT_DAMPED' }
    'NoDamper' { 'PRODUCT_CATALOG_VARIANT_NO_DAMPER' }
}

# Keep this list explicit. A new production dependency or test suite must be
# reviewed here instead of being pulled into a host build by a recursive glob.
$sourcePaths = @(
    'tests\host\host_test_runner.c',
    'tests\host\angle_serial_stm32g431_config_tests.c',
    'tests\host\can_protocol_v1_tests.c',
	'tests\host\can_command_router_tests.c',
	'tests\host\usb_command_router_tests.c',
	'tests\host\usb_interface_tests.c',
	'tests\host\communication_interface_fault_tests.c',
    'tests\host\context_isolation_tests.c',
    'tests\host\control_authority_service_tests.c',
    'tests\host\device_lifecycle_tests.c',
	'tests\host\encoder_tests.c',
	'tests\host\fast_math_tests.c',
	'tests\host\feedback_router_tests.c',
	'tests\host\rotor_feedback_runtime_tests.c',
    'tests\host\fault_manager_tests.c',
    'tests\host\friction_identification_tests.c',
    'tests\host\measurement_model_tests.c',
	'tests\host\measurement_runtime_tests.c',
	'tests\host\motor_drive_service_tests.c',
	'tests\host\phase_current_strategy_tests.c',
	'tests\host\temperature_monitor_tests.c',
	'tests\host\temperature_supervision_tests.c',
    'tests\host\text_writer_tests.c',
    'tests\host\tle5012b_driver_tests.c',
	'Firmware\Core\Communication\Protocol\can_protocol_v1.c',
	'Firmware\Core\Communication\Interfaces\interface_can.c',
	'Firmware\Core\Communication\Protocol\usb_protocol_v1.c',
	'Firmware\Core\Communication\Transport\byte_ring_buffer.c',
	'Firmware\Platform\Stm32G431\angle_serial_stm32g431_config.c',
    'tests\host\product_catalog_variant_tests.c',
    'tests\host\motor_commissioning_workflow_tests.c',
    'tests\host\parameter_manager_tests.c',
    'tests\host\parameter_persistence_adapter_tests.c',
    'tests\host\parameter_service_tests.c',
    'tests\host\parameter_transaction_service_tests.c',
    'tests\host\service_result_validation_tests.c',
    'tests\host\update_service_tests.c',
    'Firmware\Core\Config\Tests\product_config_tests.c',
    'Firmware\Bsp\Boards\Tests\test_bsp_board.c',
	'tests\host\product_config_bridge_tests.c',
    'Firmware\Core\Application\Commissioning\calibration_service.c',
	'Firmware\Core\Application\Communication\can_configuration_service.c',
    'Firmware\Core\Application\communication_watchdog_service.c',
    'Firmware\Core\Application\control_authority_service.c',
    'Firmware\Core\Application\device_lifecycle.c',
    'Firmware\Core\Services\Safety\fault_manager.c',
    'Firmware\Core\Application\friction_identification_service.c',
    'Firmware\Core\Application\Commissioning\identification_service.c',
    'Firmware\Core\Application\motor_command_service.c',
    'Firmware\Core\Application\motor_commissioning_workflow.c',
	'Firmware\Core\Application\MotorControl\motor_drive_service.c',
	'Firmware\Core\Application\Supervision\temperature_supervision.c',
    'Firmware\Core\Infrastructure\Parameters\parameter_manager.c',
    'Firmware\Core\Application\MotorControl\parameter_persistence_adapter.c',
    'Firmware\Core\Application\Parameters\parameter_service.c',
    'Firmware\Core\Application\parameter_transaction_service.c',
    'Firmware\Core\Application\rotor_calibration_service.c',
    'Firmware\Core\Infrastructure\Telemetry\telemetry_service.c',
    'Firmware\Core\Communication\Can\can_response_service.c',
    'Firmware\Core\Application\Update\update_service.c',
	'Firmware\Core\Services\RotorFeedback\encoder.c',
	'Firmware\Core\Services\Math\fast_math.c',
	'Firmware\Core\Services\RotorFeedback\feedback_router.c',
	'Firmware\Core\Services\RotorFeedback\secondary_angle_tracker.c',
	'Firmware\Core\Application\MotorControl\rotor_feedback_runtime.c',
	'Firmware\Drivers\Angle\Tle5012b\tle5012b_angle_sensor_adapter.c',
    'Firmware\Core\Communication\Formatting\text_writer.c',
    'Firmware\Core\Services\Identification\friction_identification.c',
    'Firmware\Core\Services\Measurement\measurement_model.c',
	'Firmware\Core\Services\Measurement\phase_current_strategy.c',
	'Firmware\Core\Services\Measurement\temperature_monitor.c',
	'Firmware\Core\Application\MotorControl\measurement_runtime.c',
	'Firmware\Core\Application\MotorControl\current_offset_calibration_runtime.c',
    'Firmware\Core\Config\product_manifest.c',
    'Firmware\Core\Config\product_config.c',
    'Firmware\Core\Config\product_capabilities.c',
    'Firmware\Core\Config\product_config_validator.c',
    'Firmware\Core\Config\product_config_runtime.c',
    'Firmware\Core\Config\product_catalog.c',
    'Firmware\Bsp\Boards\bsp_board.c',
    'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_bsp.c',
	'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_bsp_identity.c',
	'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_bsp_validation.c',
	'Firmware\Bsp\Boards\VectorMiniSt\Bootstrap\product_config_bridge.c',
	'Firmware\Bsp\Boards\bsp_product_binding.c'
)

$includePaths = @(
    'Bootloader',
    'Firmware',
    'Firmware\Core\Application\Api',
    'Firmware\Core\Application\Commissioning',
    'Firmware\Core\Application\Diagnostics',
    'Firmware\Core\Application\Parameters',
    'Firmware\Core\Application\Update',
	'Firmware\Core\Application\MotorControl',
	'Firmware\Core\Application\Supervision',
    'Firmware\Core\Services\Identification',
	'Firmware\Core\Services\Measurement',
	'Firmware\Core\Services\CurrentControl',
	'Firmware\Core\Services\Math',
	'Firmware\Core\Services\MotionControl',
	'Firmware\Core\Services\RotorFeedback',
    'Firmware\Core\Config',
	'Firmware\Core\Services\Measurement',
    'Firmware\Core\Infrastructure\Parameters',
    'Firmware\Drivers\Angle\Tle5012b',
    'Firmware\Bsp\Api',
    'Firmware\Bsp\Boards',
    'Firmware\Bsp\Boards\VectorMiniSt',
	'Firmware\Platform\Stm32G431',
	'Firmware\Bsp\Boards\VectorMiniSt\Bootstrap',
	'Firmware\Core\Communication\Protocol',
	'Firmware\Core\Communication\Router',
	'Firmware\Core\Communication\Interfaces'
)

function Resolve-CompilerCommand {
    param([string]$RequestedCompiler)

    $candidateNames = if ([string]::IsNullOrWhiteSpace($RequestedCompiler)) {
        @('clang', 'gcc', 'cc', 'clang-cl', 'cl', 'zig')
    } else {
        @($RequestedCompiler)
    }

    foreach ($candidate in $candidateNames) {
        $command = $null
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            $command = Get-Item -LiteralPath $candidate
        } else {
            $command = Get-Command $candidate -CommandType Application `
                -ErrorAction SilentlyContinue | Select-Object -First 1
        }
        if ($null -eq $command) {
            continue
        }

        $path = if ($command.PSObject.Properties.Name -contains 'Source') {
            $command.Source
        } else {
            $command.FullName
        }
        $leaf = [System.IO.Path]::GetFileNameWithoutExtension($path).ToLowerInvariant()
        $kind = switch ($leaf) {
            'cl' { 'msvc' }
            'clang-cl' { 'msvc' }
            'zig' { 'zig' }
            default { 'gnu' }
        }
        return [pscustomobject]@{
            Path = $path
            Kind = $kind
        }
    }
    return $null
}

$missingInputs = @()
$absoluteSources = foreach ($relativePath in $sourcePaths) {
    $absolutePath = Join-Path $repositoryRoot $relativePath
    if (-not (Test-Path -LiteralPath $absolutePath -PathType Leaf)) {
        $missingInputs += $relativePath
    }
    $absolutePath
}
$absoluteIncludes = foreach ($relativePath in $includePaths) {
    $absolutePath = Join-Path $repositoryRoot $relativePath
    if (-not (Test-Path -LiteralPath $absolutePath -PathType Container)) {
        $missingInputs += $relativePath
    }
    $absolutePath
}

if ($missingInputs.Count -ne 0) {
    foreach ($missingInput in $missingInputs) {
        Write-Output "ERROR: Required host-test input does not exist: $missingInput"
    }
    exit 3
}

$compilerCommand = Resolve-CompilerCommand -RequestedCompiler $Compiler
if ($null -eq $compilerCommand) {
    Write-Output 'HOST_TEST_RESULT=NOT_RUN reason=no-supported-host-c-compiler'
    Write-Output 'Install clang, GCC, MSVC, or Zig, or pass -Compiler with an explicit path.'
    exit 2
}

New-Item -ItemType Directory -Path $buildDirectory -Force | Out-Null
if (Test-Path -LiteralPath $binaryPath -PathType Leaf) {
    Remove-Item -LiteralPath $binaryPath -Force
}

Write-Output "HOST_TEST_COMPILER=$($compilerCommand.Path)"
Write-Output "HOST_TEST_PRODUCT_VARIANT=$ProductVariant"
Write-Output "HOST_TEST_SOURCE_COUNT=$($absoluteSources.Count)"

if ($compilerCommand.Kind -eq 'msvc') {
    $arguments = @('/nologo', '/std:c11', '/W4', '/WX', '/TC')
	$arguments += '/DPRODUCT_CATALOG_INCLUDE_ALL=1'
    $arguments += "/DPRODUCT_CATALOG_ACTIVE_VARIANT=$variantDefine"
    $arguments += $absoluteIncludes | ForEach-Object { "/I$_" }
    $arguments += $absoluteSources
    $arguments += "/Fe:$binaryPath"
    & $compilerCommand.Path @arguments
} else {
    $arguments = @('-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
        '-fno-common', '-DPRODUCT_CATALOG_INCLUDE_ALL=1',
        "-DPRODUCT_CATALOG_ACTIVE_VARIANT=$variantDefine")
    $arguments += $absoluteIncludes | ForEach-Object { '-I'; $_ }
    $arguments += $absoluteSources
    $arguments += @('-o', $binaryPath, '-lm')
    if ($compilerCommand.Kind -eq 'zig') {
        & $compilerCommand.Path 'cc' @arguments
    } else {
        & $compilerCommand.Path @arguments
    }
}

$buildExitCode = $LASTEXITCODE
if ($buildExitCode -ne 0) {
    Write-Output "HOST_TEST_RESULT=BUILD_FAILED exit_code=$buildExitCode"
    exit $buildExitCode
}

if ($BuildOnly) {
    Write-Output "HOST_TEST_RESULT=BUILT binary=$binaryPath"
    exit 0
}

& $binaryPath
$testExitCode = $LASTEXITCODE
if ($testExitCode -eq 0) {
    Write-Output 'HOST_TEST_RESULT=PASSED'
} else {
    Write-Output "HOST_TEST_RESULT=FAILED exit_code=$testExitCode"
}
exit $testExitCode
