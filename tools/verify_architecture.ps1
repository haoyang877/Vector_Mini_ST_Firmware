param(
    [switch]$StrictCommunication
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$projectPath = Join-Path $repositoryRoot 'MDK-ARM\Vector_Mini_ST.uvprojx'
$failures = [System.Collections.Generic.List[string]]::new()
$warnings = [System.Collections.Generic.List[string]]::new()

function Get-RepositoryRelativePath {
    param([string]$Path)
    $rootWithSeparator = $repositoryRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    if ($Path.StartsWith($rootWithSeparator, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $Path.Substring($rootWithSeparator.Length)
    }
    return $Path
}

function Add-Matches {
    param(
        [string[]]$Files,
        [string]$Pattern,
        [string]$Description,
        [bool]$IsFailure = $true
    )

    foreach ($file in $Files) {
        $matches = Select-String -LiteralPath $file -Pattern $Pattern
        foreach ($match in $matches) {
            $relativePath = Get-RepositoryRelativePath $file
            $message = "$Description at ${relativePath}:$($match.LineNumber)"
            if ($IsFailure) {
                $failures.Add($message)
            } else {
                $warnings.Add($message)
            }
        }
    }
}

foreach ($requiredDocument in @(
    'docs\architecture\firmware_architecture.md',
    'docs\architecture\refactoring_plan.md',
    'docs\product_configuration_quick_guide.md'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $repositoryRoot $requiredDocument))) {
        $failures.Add("Missing architecture document: $requiredDocument")
    }
}

foreach ($requiredTest in @(
    'tests\host\device_lifecycle_tests.c',
    'tests\host\fault_manager_tests.c',
    'tests\host\measurement_model_tests.c',
	'tests\host\mechanical_load_profile_tests.c',
    'tests\host\parameter_transaction_service_tests.c',
    'tests\host\parameter_service_tests.c',
    'tests\host\service_result_validation_tests.c',
    'tests\host\update_service_tests.c'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $repositoryRoot $requiredTest))) {
        $failures.Add("Missing host architecture test: $requiredTest")
    }
}

foreach ($requiredImplementation in @(
    'Firmware\Composition\firmware_composition.c',
    'Firmware\Communication\Protocol\can_protocol_v1.c',
    'Firmware\Communication\Protocol\usb_protocol_v1.c',
    'Firmware\Communication\Router\can_command_router.c',
    'Firmware\Communication\Router\usb_command_router.c',
    'Firmware\Communication\Transport\byte_ring_buffer.c',
    'Firmware\Application\can_configuration_service.c',
    'Firmware\Application\can_response_service.c',
    'Firmware\Application\diagnostic_service.c',
    'Firmware\Application\update_service.c',
    'Firmware\Platform\Stm32G431\device_identity_stm32g431.c',
    'Firmware\Platform\Stm32G431\execution_timer_stm32g431.c',
    'Firmware\Platform\Stm32G431\reset_reason_stm32g431.c',
    'Firmware\Product\board_profile.c',
    'Firmware\Product\motor_profiles.c',
	'Firmware\Product\mechanical_load_profiles.c',
    'Firmware\Product\encoder_profiles.c',
    'Firmware\Domain\Measurement\measurement_model.c',
    'Firmware\Domain\Math\fast_math.c',
    'Firmware\Domain\CurrentControl\current_control_math.c',
    'Firmware\Domain\MotionControl\trapezoidal_trajectory.c',
    'Firmware\Runtime\MotorControl\motor_control_runtime.c',
    'Firmware\Runtime\MotorControl\parameter_persistence_adapter.c',
    'Firmware\Runtime\Supervisor\supervisor_task.c'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $repositoryRoot $requiredImplementation))) {
        $failures.Add("Missing required layered implementation: $requiredImplementation")
    }
}

$parameterStorePort = Join-Path $repositoryRoot 'Firmware\Ports\parameter_store_port.h'
if (-not (Test-Path -LiteralPath $parameterStorePort)) {
    $failures.Add('Missing parameter storage Port: Firmware\Ports\parameter_store_port.h')
}

[xml]$project = Get-Content -Raw -LiteralPath $projectPath
$projectGroupNodes = @($project.Project.Targets.Target.Groups.Group)
$projectFiles = @($projectGroupNodes.Files.File)
$projectGroups = @($projectGroupNodes.GroupName)
foreach ($requiredGroup in @('Firmware/Application', 'Firmware/Product',
    'Firmware/Domain/Modulation', 'Firmware/Domain/Math',
    'Firmware/Domain/Measurement', 'Firmware/Domain/Identification',
    'Firmware/Domain/CurrentControl', 'Firmware/Domain/MotionControl',
    'Firmware/Domain/RotorFeedback', 'Firmware/Runtime/MotorControl',
    'Firmware/Runtime/Supervisor', 'Firmware/Communication/Transport',
    'Firmware/Communication/Protocol', 'Firmware/Communication/Router',
    'Firmware/Communication/Interfaces', 'Firmware/Application/Indicators',
    'Firmware/Platform/Stm32G431', 'Firmware/Composition')) {
    if ($projectGroups -notcontains $requiredGroup) {
        $failures.Add("Missing Keil architecture group: $requiredGroup")
    }
}

$groupPathPrefixes = [ordered]@{
    'Firmware/Application' = '..\Firmware\Application\'
    'Firmware/Application/Indicators' = '..\Firmware\Application\Indicators\'
    'Firmware/Product' = '..\Firmware\Product\'
    'Firmware/Domain/Math' = '..\Firmware\Domain\Math\'
    'Firmware/Domain/Measurement' = '..\Firmware\Domain\Measurement\'
    'Firmware/Domain/Identification' = '..\Firmware\Domain\Identification\'
    'Firmware/Domain/CurrentControl' = '..\Firmware\Domain\CurrentControl\'
    'Firmware/Domain/MotionControl' = '..\Firmware\Domain\MotionControl\'
    'Firmware/Domain/Modulation' = '..\Firmware\Domain\Modulation\'
    'Firmware/Domain/RotorFeedback' = '..\Firmware\Domain\RotorFeedback\'
    'Firmware/Runtime/MotorControl' = '..\Firmware\Runtime\MotorControl\'
    'Firmware/Runtime/Supervisor' = '..\Firmware\Runtime\Supervisor\'
    'Firmware/Communication/Transport' = '..\Firmware\Communication\Transport\'
    'Firmware/Communication/Protocol' = '..\Firmware\Communication\Protocol\'
    'Firmware/Communication/Router' = '..\Firmware\Communication\Router\'
    'Firmware/Communication/Interfaces' = '..\Firmware\Communication\'
    'Firmware/Platform/Stm32G431' = '..\Firmware\Platform\Stm32G431\'
    'Firmware/Composition' = '..\Firmware\Composition\'
}
foreach ($groupNode in $projectGroupNodes) {
    $groupName = [string]$groupNode.GroupName
    if (-not $groupPathPrefixes.Contains($groupName)) {
        continue
    }
    $expectedPrefix = $groupPathPrefixes[$groupName]
    foreach ($file in @($groupNode.Files.File)) {
        $normalizedPath = ([string]$file.FilePath).Replace('/', '\')
        if ($normalizedPath -and -not $normalizedPath.StartsWith($expectedPrefix,
            [System.StringComparison]::OrdinalIgnoreCase)) {
            $failures.Add("Keil group '$groupName' contains a file outside '$expectedPrefix': $normalizedPath")
        }
    }
}
foreach ($projectFile in $projectFiles) {
    if (-not $projectFile.FilePath) {
        continue
    }
    $sourcePath = Join-Path (Split-Path $projectPath) ([string]$projectFile.FilePath)
    if (-not (Test-Path -LiteralPath $sourcePath)) {
        $failures.Add("Project source does not exist: $($projectFile.FilePath)")
    }
}

$projectSourcePaths = @($projectFiles | Where-Object FilePath | ForEach-Object {
    [System.IO.Path]::GetFullPath((Join-Path (Split-Path $projectPath) ([string]$_.FilePath)))
})
foreach ($sourceDirectory in @('Application', 'Communication', 'Composition',
    'Domain', 'Platform', 'Product', 'Runtime')) {
    $directoryPath = Join-Path $repositoryRoot "Firmware\$sourceDirectory"
    if (-not (Test-Path -LiteralPath $directoryPath)) {
        continue
    }
    foreach ($source in Get-ChildItem -LiteralPath $directoryPath -Recurse -File -Filter *.c) {
        if ($projectSourcePaths -notcontains $source.FullName) {
            $relativePath = Get-RepositoryRelativePath $source.FullName
            $failures.Add("Layered source is not compiled by the target project: $relativePath")
        }
    }
}

$powerStageAdapter = (Resolve-Path (Join-Path $repositoryRoot 'Firmware\Platform\Stm32G431\power_stage_tim1.c')).Path
$productSources = Get-ChildItem -LiteralPath $repositoryRoot -Recurse -File -Include *.c,*.h |
    Where-Object {
        $_.Extension -in @('.c', '.h') -and
        $_.FullName -notlike "$repositoryRoot\Drivers\*" -and
        $_.FullName -notlike "$repositoryRoot\Middlewares\*" -and
        $_.FullName -ne $powerStageAdapter
    } | ForEach-Object FullName
Add-Matches -Files $productSources -Pattern 'TIM1->CCR[123]|HAL_TIM_PWM_(Start|Stop)\([^\r\n]*TIM_CHANNEL_[123]|HAL_TIMEx_OCN_(Start|Stop)\([^\r\n]*TIM_CHANNEL_[123]|__HAL_TIM_SET_COMPARE\([^\r\n]*TIM_CHANNEL_[123]' -Description 'TIM1 three-phase output bypasses the PowerStage adapter'

$applicationSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Application') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $applicationSources -Pattern '#include\s+["<](stm32|main\.h|tim\.h|adc\.h|fdcan\.h)|\bHAL_|\bTIM1->|\bADC[12]->|\bFDCAN1->' -Description 'Application layer depends on STM32 HAL or registers'
Add-Matches -Files $applicationSources -Pattern '^\s*extern\s+' -Description 'Application layer uses a hidden external object instead of an injected Port'
Add-Matches -Files $applicationSources -Pattern '#include\s+"(foc_[^"]*|encoder|hw_conf|data_type|interface_[^"]*)\.h"' -Description 'Application layer depends on a legacy control, BSP, or communication header'

$layeredSources = @('Application', 'Communication', 'Composition', 'Domain',
    'Ports', 'Product', 'Runtime') | ForEach-Object {
        $layerPath = Join-Path $repositoryRoot "Firmware\$_"
        if (Test-Path -LiteralPath $layerPath) {
            Get-ChildItem -LiteralPath $layerPath -Recurse -File -Include *.c,*.h
        }
    } | ForEach-Object FullName
Add-Matches -Files $layeredSources -Pattern '#include\s+["<](main\.h|adc\.h|tim\.h|spi\.h|fdcan\.h|gpio\.h|stm32[^">]*)[">]|\bHAL_|\b(ADC[12]|TIM[0-9]+|SPI[12]|FDCAN1)->|\b__(disable_irq|enable_irq|get_PRIMASK|set_PRIMASK)\b' -Description 'HAL, CMSIS, or peripheral access escaped the Platform/Core boundary'

$generatedCallbackSources = @(
    (Join-Path $repositoryRoot 'Core\Src\stm32g4xx_it.c'),
    (Join-Path $repositoryRoot 'USB_Device\App\usbd_cdc_if.c')
)
Add-Matches -Files $generatedCallbackSources -Pattern '#include\s+"(?:supervisor_task|motor_control_runtime|interface_can|interface_usb)\.h"' -Description 'A generated callback bypasses FirmwareComposition and depends directly on Runtime or Communication'

$profileConsumerSources = @('Application', 'Communication', 'Composition',
    'Domain', 'Runtime') | ForEach-Object {
        Get-ChildItem -LiteralPath (Join-Path $repositoryRoot "Firmware\$_") -Recurse -File -Include *.c,*.h
    } | ForEach-Object FullName
Add-Matches -Files $profileConsumerSources -Pattern '#include\s+"(?:vector_mini_st_profile|current_sense_profile|motor_safety_limits)\.h"|\b(?:PARAM_(?:MOTOR|HW|APP)|CURRENT_SENSE_PROFILE|MOTOR_LIMIT_)' -Description 'A consumer selects Product macros instead of using an injected typed profile'

$portSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Ports') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $portSources -Pattern '#include\s+"' -Description 'Port interface depends on a concrete project header'

$runtimeSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Runtime') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $runtimeSources -Pattern '\bHAL_|\bTIM1->|\bADC[12]->|\bFDCAN1->|\bSPI[12]->' -Description 'Runtime module depends directly on HAL or peripheral registers'
Add-Matches -Files $runtimeSources -Pattern '#include\s+"parameter_store_flash\.h"|\bParameterStoreFlash_' -Description 'Runtime module depends on the concrete STM32 parameter store instead of an injected Port'
Add-Matches -Files $runtimeSources -Pattern '\bMotorExecutionAction\b|\bMOTOR_ACTION_|runtime\.action\b' -Description 'Runtime reintroduces a combined protocol action/state model'

$applicationImplementationSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Application') -Recurse -File -Filter *.c |
    ForEach-Object FullName
Add-Matches -Files $applicationImplementationSources -Pattern '^\s*static\s+(?!const\b)(?![A-Za-z_][A-Za-z0-9_\s\*]*\()(?![A-Za-z_][A-Za-z0-9_\s]*\*\s*Active)' -Description 'Application owns hidden mutable state instead of a Composition-injected Context'

$controlLoopConfig = Join-Path $repositoryRoot 'Firmware\Product\control_loop_config.h'
Add-Matches -Files @($controlLoopConfig) -Pattern '^\s*#define\s+(SENSORLESS_|ENCODER_)' -Description 'Product tuning remains a compile-time macro instead of a typed injected Profile'

$domainSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Domain') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $domainSources -Pattern '#include\s+["<](stm32|main\.h|tim\.h|adc\.h|fdcan\.h|power_stage\.h)|\bHAL_|\bTIM1->|\bADC[12]->|\bFDCAN1->|\bSPI[12]->' -Description 'Domain module depends on platform or application output APIs'
Add-Matches -Files $domainSources -Pattern '#include\s+"(?:foc_|interface_|telemetry_service|parameter_service|motor_command_service|hw_conf|data_type|vector_mini_st_profile)[^"]*\.h"' -Description 'Domain module depends on a legacy, communication, Application, or Product implementation header'
$domainImplementationSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Domain') -Recurse -File -Filter *.c |
    ForEach-Object FullName
Add-Matches -Files $domainImplementationSources -Pattern '^\s*static\s+(?!const\b)(?![A-Za-z_][A-Za-z0-9_\s\*]*\()' -Description 'Domain module owns hidden mutable file-scope state'

$ownedPublicSources = @('Application', 'Communication', 'Composition', 'Domain',
    'Ports', 'Product', 'Runtime') | ForEach-Object {
        Get-ChildItem -LiteralPath (Join-Path $repositoryRoot "Firmware\$_") -Recurse -File -Include *.c,*.h
    } | ForEach-Object FullName
Add-Matches -Files $ownedPublicSources -Pattern '\b[A-Za-z][A-Za-z0-9_]*_(?:TypeDef|TyepeDef)\b' -Description 'Product-owned type retains migration-era TypeDef naming'
Add-Matches -Files $ownedPublicSources -Pattern '#include\s+"(?:hw_conf|data_type|utils|flash|board_config|bsp_task|foc_[^"]*|legacy_[^"]*)\.h"' -Description 'Source includes a removed architecture-era header'

$realTimeSources = @(
    'Firmware\Runtime\MotorControl\motor_control_runtime.c',
    'Firmware\Runtime\MotorControl\measurement_runtime.c',
    'Firmware\Runtime\MotorControl\control_mode_runtime.c',
    'Firmware\Runtime\MotorControl\current_control_runtime.c',
    'Firmware\Runtime\MotorControl\current_offset_calibration_runtime.c',
    'Firmware\Runtime\MotorControl\electrical_zero_calibration_runtime.c',
    'Firmware\Runtime\MotorControl\encoder_calibration_runtime.c',
    'Firmware\Runtime\MotorControl\phase_resistance_runtime.c',
    'Firmware\Domain\Identification\phase_resistance.c',
    'Firmware\Domain\RotorFeedback\encoder.c',
    'Firmware\Domain\Modulation\svpwm.c'
) | ForEach-Object { Join-Path $repositoryRoot $_ }
Add-Matches -Files $realTimeSources -Pattern '\b(HEAP_malloc|HEAP_free|malloc|calloc|realloc|free|flash_write|flash_erase|sprintf|snprintf|printf)\s*\(' -Description 'Prohibited operation appears in the 20 kHz call graph'

$controlAlgorithmSources = @(
    'Firmware\Runtime\MotorControl\control_mode_runtime.c',
    'Firmware\Runtime\MotorControl\current_control_runtime.c',
    'Firmware\Runtime\MotorControl\current_offset_calibration_runtime.c',
    'Firmware\Runtime\MotorControl\electrical_zero_calibration_runtime.c',
    'Firmware\Runtime\MotorControl\encoder_calibration_runtime.c',
    'Firmware\Runtime\MotorControl\phase_resistance_runtime.c'
) | ForEach-Object { Join-Path $repositoryRoot $_ }
Add-Matches -Files $controlAlgorithmSources -Pattern '(?:MotorControl|motor)->command\.[A-Za-z_][A-Za-z0-9_]*\s*=(?!=)' -Description 'A control algorithm rewrites the accepted Application command instead of its internal targets'
Add-Matches -Files $controlAlgorithmSources -Pattern '(?:MotorControl|motor)->configuration\.[A-Za-z_][A-Za-z0-9_]*\s*=(?!=)' -Description 'A control algorithm mutates active configuration instead of staging a validated candidate'

$communicationSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Communication') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
$directCommunicationPattern = 'MotorControl\.[A-Za-z_][A-Za-z0-9_]*\s*=(?!=)|\.addr\s*=\s*&(?:MotorControl|CurrentControl|OnBoard_Encoder)\.'
$communicationMatches = @($communicationSources | ForEach-Object {
    Select-String -LiteralPath $_ -Pattern $directCommunicationPattern
})
foreach ($match in $communicationMatches) {
    $relativePath = Get-RepositoryRelativePath $match.Path
    $failures.Add("Communication directly writes or retains addresses of control internals at ${relativePath}:$($match.LineNumber)")
}
Add-Matches -Files $communicationSources -Pattern 'extern\s+[^;]*\b(MotorControl|CurrentControl|OnBoard_Encoder)\b|#include\s+"(encoder|current_control_runtime|parameter_snapshot|hw_conf)\.h"' -Description 'Communication bypasses an Application service boundary'

$protocolSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Communication\Protocol') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $protocolSources -Pattern '#include\s+"(?:interface_|.*command_router|.*_service|current_control_runtime|hw_conf|motor_control_types|product_)[^"]*\.h"|\bHAL_' -Description 'Protocol codec depends on a Router, Service, control runtime, or platform implementation'

$routerSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Communication\Router') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $routerSources -Pattern '#include\s+"(?:interface_|foc_|hw_conf|data_type|stm32)[^"]*\.h"|\bHAL_|\bCAN_SendMessage_Update\b' -Description 'Router depends directly on a Transport or control/platform implementation'

$transportSources = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'Firmware\Communication\Transport') -Recurse -File -Include *.c,*.h |
    ForEach-Object FullName
Add-Matches -Files $transportSources -Pattern '#include\s+"(?:(?:can|usb)_protocol|.*command_router|.*_service|interface_|current_control_runtime|hw_conf|motor_control_types)[^"]*\.h"|\bHAL_' -Description 'Transport utility depends on Protocol, Router, Service, or platform implementation'

Add-Matches -Files $communicationSources -Pattern '\bUSBMsg_TypeDef\b|\bCANMsg_TypeDef\b|\bUSBMsg\b|\bCANMsg\b' -Description 'Communication exposes or retains a legacy shared message object'

foreach ($obsoletePath in @('System\common_inc.h', 'System\heap.c', 'System\heap.h',
    'System', 'Foc', 'Bsp', 'Firmware\Product\hw_conf.h',
    'System\board_config.c', 'System\bsp_task.c', 'System\flash.c',
    'System\utils.c', 'System\data_type.h',
    'Bsp\delay.c', 'Bsp\delay.h', 'Firmware\Communication\ring_buffer.c',
    'Firmware\Communication\ring_buffer.h', 'Foc\foc_pid.c', 'Foc\foc_pid.h',
    'Firmware\Communication\Protocol\legacy_can_protocol.c',
    'Firmware\Communication\Protocol\legacy_usb_protocol.c',
    'Firmware\Communication\Router\legacy_can_router.c',
    'Firmware\Communication\Router\legacy_usb_router.c',
    'Foc\foc_traptraj.c', 'Foc\foc_traptraj.h',
    'Foc\position_cascade.c', 'Foc\position_cascade.h',
    'Foc\position_impedance.c', 'Foc\position_impedance.h')) {
    if (Test-Path -LiteralPath (Join-Path $repositoryRoot $obsoletePath)) {
        $failures.Add("Obsolete infrastructure is still present: $obsoletePath")
    }
}

foreach ($obsoleteTopLevelLayer in @('Application', 'Communication', 'Composition',
    'Domain', 'Platform', 'Ports', 'Product', 'Runtime')) {
    if (Test-Path -LiteralPath (Join-Path $repositoryRoot $obsoleteTopLevelLayer)) {
        $failures.Add("Layer must be located below Firmware/: $obsoleteTopLevelLayer")
    }
}

Write-Output "Architecture verification: project_entries=$($projectFiles.Count) failures=$($failures.Count) warnings=$($warnings.Count)"
foreach ($warning in $warnings) {
    Write-Warning $warning
}
foreach ($failure in $failures) {
    Write-Output "ERROR: $failure"
}

if ($failures.Count -ne 0) {
    exit 1
}

exit 0
