param(
    [string]$Compiler,
    [switch]$BuildOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repositoryRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$buildDirectory = Join-Path $PSScriptRoot '.build'
$binaryName = if ([System.Environment]::OSVersion.Platform -eq
    [System.PlatformID]::Win32NT) { 'firmware_host_tests.exe' } else {
    'firmware_host_tests'
}
$binaryPath = Join-Path $buildDirectory $binaryName

# Keep this list explicit. A new production dependency or test suite must be
# reviewed here instead of being pulled into a host build by a recursive glob.
$sourcePaths = @(
    'tests\host\host_test_runner.c',
    'tests\host\angle_serial_stm32g431_config_tests.c',
    'tests\host\context_isolation_tests.c',
    'tests\host\control_authority_service_tests.c',
    'tests\host\device_lifecycle_tests.c',
    'tests\host\fault_manager_tests.c',
    'tests\host\friction_identification_tests.c',
    'tests\host\measurement_model_tests.c',
	'tests\host\phase_current_strategy_tests.c',
    'tests\host\tle5012b_driver_tests.c',
	'Firmware\Composition\tle5012b_rotor_sensor_adapter.c',
	'Firmware\Platform\Stm32G431\angle_serial_stm32g431_config.c',
    'tests\host\mechanical_load_profile_tests.c',
    'tests\host\motor_commissioning_workflow_tests.c',
    'tests\host\parameter_manager_tests.c',
    'tests\host\parameter_service_tests.c',
    'tests\host\parameter_transaction_service_tests.c',
    'tests\host\product_variant_tests.c',
    'tests\host\service_result_validation_tests.c',
    'tests\host\update_service_tests.c',
    'Firmware\Core\Config\Tests\product_config_tests.c',
    'Firmware\Bsp\Boards\Tests\test_bsp_board.c',
	'tests\host\product_config_bridge_tests.c',
    'Firmware\Application\calibration_service.c',
    'Firmware\Application\control_authority_service.c',
    'Firmware\Application\device_lifecycle.c',
    'Firmware\Application\fault_manager.c',
    'Firmware\Application\identification_service.c',
    'Firmware\Application\motor_commissioning_workflow.c',
    'Firmware\Application\parameter_manager.c',
    'Firmware\Application\parameter_service.c',
    'Firmware\Application\parameter_transaction_service.c',
    'Firmware\Application\power_stage.c',
    'Firmware\Application\telemetry_service.c',
    'Firmware\Application\update_service.c',
    'Firmware\Core\Services\Identification\friction_identification.c',
    'Firmware\Core\Services\Measurement\measurement_model.c',
	'Firmware\Core\Services\Measurement\phase_current_strategy.c',
    'Firmware\Product\board_profile.c',
    'Firmware\Product\control_tuning_profile.c',
    'Firmware\Product\encoder_profiles.c',
    'Firmware\Product\mechanical_load_profiles.c',
    'Firmware\Product\memory_layout_profile.c',
    'Firmware\Product\motor_profiles.c',
    'Firmware\Product\product_manifest.c',
    'Firmware\Product\product_variant.c',
    'Firmware\Core\Config\product_config.c',
    'Firmware\Core\Config\product_capabilities.c',
    'Firmware\Core\Config\product_config_validator.c',
    'Firmware\Core\Config\product_config_runtime.c',
    'Firmware\Core\Config\product_catalog.c',
	'Firmware\Core\Config\product_runtime_selection.c',
    'Firmware\Bsp\Boards\bsp_board.c',
    'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_bsp.c',
	'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_bsp_identity.c',
	'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_bsp_validation.c',
	'Firmware\Composition\product_config_bridge.c',
	'Firmware\Bsp\Boards\bsp_product_binding.c'
)

$includePaths = @(
    'Bootloader',
    'Firmware',
    'Firmware\Application',
    'Firmware\Core\Services\Identification',
    'Firmware\Core\Services\Measurement',
    'Firmware\Ports',
    'Firmware\Product',
    'Firmware\Core\Config',
	'Firmware\Core\Services\Measurement',
    'Firmware\Drivers\Angle\Tle5012b',
    'Firmware\Bsp\Api',
    'Firmware\Bsp\Boards',
    'Firmware\Bsp\Boards\VectorMiniSt',
	'Firmware\Platform\Stm32G431',
	'Firmware\Composition'
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
Write-Output "HOST_TEST_SOURCE_COUNT=$($absoluteSources.Count)"

if ($compilerCommand.Kind -eq 'msvc') {
    $arguments = @('/nologo', '/std:c11', '/W4', '/WX', '/TC')
	$arguments += '/DMECHANICAL_LOAD_PROFILE_INCLUDE_CATALOG=1'
    $arguments += $absoluteIncludes | ForEach-Object { "/I$_" }
    $arguments += $absoluteSources
    $arguments += "/Fe:$binaryPath"
    & $compilerCommand.Path @arguments
} else {
    $arguments = @('-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
        '-fno-common', '-DMECHANICAL_LOAD_PROFILE_INCLUDE_CATALOG=1')
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
