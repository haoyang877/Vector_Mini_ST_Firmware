param(
    [switch]$FailOnLegacyExceptions
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$repositoryRootPrefix = $repositoryRoot.TrimEnd(
    [System.IO.Path]::DirectorySeparatorChar,
    [System.IO.Path]::AltDirectorySeparatorChar) +
    [System.IO.Path]::DirectorySeparatorChar
$firmwareRoot = Join-Path $repositoryRoot 'Firmware'
$failures = [System.Collections.Generic.List[string]]::new()
$warnings = [System.Collections.Generic.List[string]]::new()
$legacyExceptions = [System.Collections.Generic.List[string]]::new()

function Get-RepositoryRelativePath {
    param([string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if (-not $fullPath.StartsWith($repositoryRootPrefix,
        [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Path is outside the repository: $fullPath"
    }
    return $fullPath.Substring($repositoryRootPrefix.Length).Replace('\', '/')
}

function Add-ContentMatches {
    param(
        [System.IO.FileInfo[]]$Files,
        [string]$Pattern,
        [string]$Description,
        [hashtable]$LegacyFileAllowlist
    )

    $matchesByFile = @{}
    foreach ($file in $Files) {
        foreach ($match in Select-String -LiteralPath $file.FullName -Pattern $Pattern) {
            $relativePath = Get-RepositoryRelativePath $file.FullName
            if (-not $matchesByFile.ContainsKey($relativePath)) {
                $matchesByFile[$relativePath] = [System.Collections.Generic.List[object]]::new()
            }
            $matchesByFile[$relativePath].Add($match)
        }
    }

    foreach ($relativePath in ($matchesByFile.Keys | Sort-Object)) {
        $matches = $matchesByFile[$relativePath]
        if ($LegacyFileAllowlist.ContainsKey($relativePath)) {
            $allowance = $LegacyFileAllowlist[$relativePath]
            if ($matches.Count -le $allowance.MaximumOccurrences) {
                $legacyExceptions.Add(
                    "$Description in ${relativePath}: $($matches.Count)/" +
                    "$($allowance.MaximumOccurrences) allowed; $($allowance.Reason)")
                continue
            }
            $failures.Add(
                "$Description expanded in ${relativePath}: $($matches.Count) occurrences " +
                "exceed legacy maximum $($allowance.MaximumOccurrences)")
            continue
        }

        foreach ($match in $matches) {
            $failures.Add("$Description at ${relativePath}:$($match.LineNumber)")
        }
    }
}

function Get-ArchitectureLayer {
    param([string]$RelativePath)

    $path = $RelativePath.Replace('\', '/')
    switch -Regex ($path) {
        '^Firmware/Core/Application/' { return 'CoreApplication' }
        '^Firmware/Core/Services/' { return 'CoreServices' }
        '^Firmware/Core/Communication/' { return 'CoreCommunication' }
        '^Firmware/Core/Config/' { return 'CoreConfig' }
        '^Firmware/Core/Infrastructure/' { return 'CoreInfrastructure' }
        '^Firmware/Drivers/' { return 'Drivers' }
        '^Firmware/Bsp/Api/' { return 'BspApi' }
        '^Firmware/Bsp/Boards/' { return 'BspBoards' }
        '^Firmware/Platform/' { return 'Platform' }
        '^Firmware/Application/' { return 'LegacyApplication' }
        '^Firmware/Communication/' { return 'LegacyCommunication' }
        '^Firmware/Composition/' { return 'LegacyComposition' }
        '^Firmware/Domain/' { return 'LegacyDomain' }
        '^Firmware/Ports/' { return 'LegacyPorts' }
        '^Firmware/Runtime/' { return 'LegacyRuntime' }
        '^Firmware/Product/' { return 'LegacyProduct' }
        default { return $null }
    }
}

if (-not (Test-Path -LiteralPath $firmwareRoot -PathType Container)) {
    Write-Output 'ERROR: Firmware directory does not exist.'
    exit 1
}

# These are the P0/P1 boundaries introduced before source migration. Their
# temporary absence is explicit debt; -FailOnLegacyExceptions turns that debt
# into a release gate. Later phases may extend this list after their directory
# decisions are recorded in an architecture decision record.
$requiredTargetDirectories = @(
    'Firmware/Core/Application',
    'Firmware/Core/Services',
    'Firmware/Core/Communication',
    'Firmware/Core/Config',
    'Firmware/Core/Infrastructure',
    'Firmware/Drivers',
    'Firmware/Bsp/Api',
    'Firmware/Bsp/Boards',
    'Firmware/Platform'
)
$legacyMissingDirectoryAllowlist = @{
    'Firmware/Core/Application' = 'application entry and use cases remain in Firmware/Application'
    'Firmware/Core/Services' = 'control and algorithm services remain split across Domain and Runtime'
    'Firmware/Core/Communication' = 'communication has not moved below Core yet'
    'Firmware/Core/Config' = 'generic product configuration contracts have not been introduced yet'
    'Firmware/Core/Infrastructure' = 'persistence and diagnostics remain spread across legacy layers'
    'Firmware/Drivers' = 'portable component drivers have not been extracted yet'
    'Firmware/Bsp/Api' = 'BSP semantic contracts have not been introduced yet'
    'Firmware/Bsp/Boards' = 'board mapping is still combined with STM32G431 adapters'
}
foreach ($relativePath in $requiredTargetDirectories) {
    $absolutePath = Join-Path $repositoryRoot $relativePath
    if (Test-Path -LiteralPath $absolutePath -PathType Container) {
        continue
    }
    if ($legacyMissingDirectoryAllowlist.ContainsKey($relativePath)) {
        $legacyExceptions.Add(
            "Missing target directory $relativePath; " +
            $legacyMissingDirectoryAllowlist[$relativePath])
    } else {
        $failures.Add("Missing target directory: $relativePath")
    }
}

$legacyDirectoryAllowlist = @{
    'application' = 'moves to Firmware/Core/Application'
    'communication' = 'moves to Firmware/Core/Communication'
    'composition' = 'remains temporary until the final composition-root location is selected'
    'domain' = 'moves to Firmware/Core/Services'
    'ports' = 'splits by ownership between Core services and Bsp/Api'
    'product' = 'moves to the unified Firmware/Core/Config model'
    'runtime' = 'splits between Core/Application and Core/Services'
}
$allowedTopLevelDirectories = @('core', 'drivers', 'bsp', 'platform')
foreach ($directory in Get-ChildItem -LiteralPath $firmwareRoot -Directory) {
    $name = $directory.Name.ToLowerInvariant()
    if ($allowedTopLevelDirectories -contains $name) {
        continue
    }
    if ($legacyDirectoryAllowlist.ContainsKey($name)) {
        $legacyExceptions.Add(
            "Legacy directory Firmware/$($directory.Name); " +
            $legacyDirectoryAllowlist[$name])
    } else {
        $failures.Add("Unexpected Firmware top-level directory: Firmware/$($directory.Name)")
    }
}

$allowedCoreDirectories = @(
    'application', 'services', 'communication', 'config', 'infrastructure'
)
$coreRoot = Join-Path $firmwareRoot 'Core'
if (Test-Path -LiteralPath $coreRoot -PathType Container) {
    foreach ($directory in Get-ChildItem -LiteralPath $coreRoot -Directory) {
        if ($allowedCoreDirectories -notcontains $directory.Name.ToLowerInvariant()) {
            $failures.Add(
                "Unexpected Firmware/Core directory: Firmware/Core/$($directory.Name)")
        }
    }
}

$allowedBspDirectories = @('api', 'boards')
$bspRoot = Join-Path $firmwareRoot 'Bsp'
if (Test-Path -LiteralPath $bspRoot -PathType Container) {
    foreach ($directory in Get-ChildItem -LiteralPath $bspRoot -Directory) {
        if ($allowedBspDirectories -notcontains $directory.Name.ToLowerInvariant()) {
            $failures.Add(
                "Unexpected Firmware/Bsp directory: Firmware/Bsp/$($directory.Name)")
        }
    }
}

$firmwareFiles = @(Get-ChildItem -LiteralPath $firmwareRoot -Recurse -File |
    Where-Object { $_.Extension -in @('.c', '.h') })

$platformLeakPattern = '(?i)(#include\s+["<](?:stm32[^">]*|cmsis[^">]*|' +
    'main\.h|adc\.h|tim\.h|spi\.h|fdcan\.h|gpio\.h|usbd[^">]*)[">]|' +
    '\bHAL_[A-Za-z0-9_]+|\bLL_[A-Za-z0-9_]+|' +
    '\b(?:ADC|TIM|SPI|FDCAN|GPIO|DMA|UART|USB)_HandleTypeDef\b|' +
    '\b(?:ADC|TIM|SPI|FDCAN|GPIO)[A-Z0-9_]*->|' +
    '\b__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK)\b|' +
    '\b(?:STM32[A-Z0-9_]*|CMSIS|VECTOR_?MINI(?:_?ST)?|TLE5012B|' +
    'AS5047|MT6701|MT6835|DRV83[0-9A-Z]*)\b)'

$coreDirectory = Join-Path $firmwareRoot 'Core'
if (Test-Path -LiteralPath $coreDirectory -PathType Container) {
    $coreFiles = @(Get-ChildItem -LiteralPath $coreDirectory -Recurse -File |
        Where-Object { $_.Extension -in @('.c', '.h') })
    Add-ContentMatches -Files $coreFiles -Pattern $platformLeakPattern `
        -Description 'Core contains a platform or concrete-device dependency' `
        -LegacyFileAllowlist @{}
}

$bspApiDirectory = Join-Path $firmwareRoot 'Bsp\Api'
if (Test-Path -LiteralPath $bspApiDirectory -PathType Container) {
    $bspApiFiles = @(Get-ChildItem -LiteralPath $bspApiDirectory -Recurse -File |
        Where-Object { $_.Extension -in @('.c', '.h') })
    Add-ContentMatches -Files $bspApiFiles -Pattern $platformLeakPattern `
        -Description 'Bsp/Api contains a platform or concrete-device dependency' `
        -LegacyFileAllowlist @{}
}

# Portable drivers may name the device family that they implement, but they
# must not bind directly to an MCU SDK, generated peripheral header, HAL call,
# or memory-mapped peripheral register. Those bindings belong in Platform.
$mcuLeakPattern = '(?i)(#include\s+["<](?:stm32[^">]*|cmsis[^">]*|' +
    'main\.h|adc\.h|tim\.h|spi\.h|fdcan\.h|gpio\.h|usbd[^">]*)[">]|' +
    '\bHAL_[A-Za-z0-9_]+|\bLL_[A-Za-z0-9_]+|' +
    '\b(?:ADC|TIM|SPI|FDCAN|GPIO|DMA|UART|USB)_HandleTypeDef\b|' +
    '\b(?:ADC|TIM|SPI|FDCAN|GPIO)[A-Z0-9_]*->|' +
    '\b__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK)\b|' +
    '\b(?:STM32[A-Z0-9_]*|CMSIS)\b)'
$driversDirectory = Join-Path $firmwareRoot 'Drivers'
if (Test-Path -LiteralPath $driversDirectory -PathType Container) {
    $driverFiles = @(Get-ChildItem -LiteralPath $driversDirectory -Recurse -File |
        Where-Object { $_.Extension -in @('.c', '.h') })
    Add-ContentMatches -Files $driverFiles -Pattern $mcuLeakPattern `
        -Description 'Drivers contains an MCU-platform dependency' `
        -LegacyFileAllowlist @{}
}

# Apply the future Core rule to the current Domain too. Only the already-known
# TLE5012B leakage is tolerated, with a frozen per-file occurrence ceiling.
$legacyDomainDirectory = Join-Path $firmwareRoot 'Domain'
if (Test-Path -LiteralPath $legacyDomainDirectory -PathType Container) {
    $legacyDomainFiles = @(Get-ChildItem -LiteralPath $legacyDomainDirectory `
        -Recurse -File | Where-Object { $_.Extension -in @('.c', '.h') })
    $legacyDomainContentAllowlist = @{
        'Firmware/Domain/RotorFeedback/encoder.h' = [pscustomobject]@{
            MaximumOccurrences = 3
            Reason = 'TLE5012B diagnostics must leave the domain contract'
        }
    }
    Add-ContentMatches -Files $legacyDomainFiles -Pattern $platformLeakPattern `
        -Description 'Legacy Domain contains a platform or concrete-device dependency' `
        -LegacyFileAllowlist $legacyDomainContentAllowlist
}

# Firmware/Ports is the migration-era equivalent of part of Bsp/Api. It must
# already remain free of MCU headers, HAL calls, registers, and concrete parts.
$legacyPortsDirectory = Join-Path $firmwareRoot 'Ports'
if (Test-Path -LiteralPath $legacyPortsDirectory -PathType Container) {
    $legacyPortFiles = @(Get-ChildItem -LiteralPath $legacyPortsDirectory `
        -Recurse -File | Where-Object { $_.Extension -in @('.c', '.h') })
    Add-ContentMatches -Files $legacyPortFiles -Pattern $platformLeakPattern `
        -Description 'Legacy Ports API contains a platform or concrete-device dependency' `
        -LegacyFileAllowlist @{}
}

$allowedDependencies = @{
    # Services and Config are the hardware-independent center. Services may
    # read immutable product policy, but neither may reach outward.
    'CoreServices' = @('CoreServices', 'CoreConfig')
    'CoreConfig' = @('CoreConfig')

    # Application orchestrates use cases through BSP semantic contracts.
    # Communication and Infrastructure are outer Core services and may call
    # Application, but Application must not depend back on either one.
    'CoreApplication' = @('CoreApplication', 'CoreServices', 'CoreConfig',
        'CoreInfrastructure', 'BspApi')
    'CoreCommunication' = @('CoreCommunication', 'CoreApplication',
        'CoreServices', 'CoreConfig', 'CoreInfrastructure', 'BspApi')
    'CoreInfrastructure' = @('CoreInfrastructure', 'CoreConfig', 'BspApi')

    # Drivers implement reusable components against BSP contracts. Platform
    # owns vendor SDK details. Boards are the only target-layer composition of
    # a product's physical capabilities/endpoints.
    'Drivers' = @('Drivers', 'CoreConfig', 'BspApi')
    'BspApi' = @('BspApi')
    'Platform' = @('Platform', 'BspApi')
    'BspBoards' = @('BspBoards', 'BspApi', 'CoreConfig', 'Platform', 'Drivers')

    # Legacy layers are frozen migration sources. Each cross-layer exception
    # that remains outside these narrow rules is counted below.
    'LegacyApplication' = @('LegacyApplication', 'LegacyDomain')
    'LegacyCommunication' = @('LegacyCommunication', 'LegacyApplication')
    'LegacyComposition' = @('LegacyApplication', 'LegacyCommunication',
        'LegacyComposition', 'LegacyDomain', 'LegacyPorts', 'LegacyProduct',
        'LegacyRuntime', 'CoreApplication', 'CoreServices', 'CoreCommunication',
        'CoreConfig', 'CoreInfrastructure', 'Drivers', 'BspApi', 'BspBoards',
        'Platform')
    'LegacyDomain' = @('LegacyDomain')
    'LegacyPorts' = @('LegacyPorts')
    'LegacyProduct' = @('LegacyProduct')
    'LegacyRuntime' = @('LegacyRuntime', 'LegacyDomain')
}

# Frozen ceilings prevent an allowlisted legacy dependency direction from
# silently spreading to more files/includes during migration.
$legacyDependencyAllowlist = @{
    'LegacyApplication->LegacyProduct' = [pscustomobject]@{
        MaximumOccurrences = 5
        Reason = 'Application still consumes broad Product profiles'
    }
    'LegacyApplication->LegacyPorts' = [pscustomobject]@{
        MaximumOccurrences = 14
        Reason = 'Application port contracts have not moved to their final owners'
    }
    'LegacyCommunication->LegacyDomain' = [pscustomobject]@{
        MaximumOccurrences = 1
        Reason = 'USB formatting still uses fast_math directly'
    }
    'LegacyCommunication->LegacyPorts' = [pscustomobject]@{
        MaximumOccurrences = 6
        Reason = 'communication transports still consume legacy port contracts'
    }
    'LegacyDomain->LegacyPorts' = [pscustomobject]@{
        MaximumOccurrences = 2
        Reason = 'legacy encoder owns sensor and critical-section ports'
    }
    'Platform->LegacyApplication' = [pscustomobject]@{
        MaximumOccurrences = 1
        Reason = 'PowerStagePort is still declared beside Application policy'
    }
    'Platform->LegacyPorts' = [pscustomobject]@{
        MaximumOccurrences = 12
        Reason = 'STM32 adapters still implement legacy hardware port contracts'
    }
    'Platform->LegacyProduct' = [pscustomobject]@{
        MaximumOccurrences = 1
        Reason = 'Flash adapter still reads the memory-layout profile directly'
    }
    'LegacyRuntime->LegacyApplication' = [pscustomobject]@{
        MaximumOccurrences = 19
        Reason = 'runtime aggregate still owns/calls Application services'
    }
    'LegacyRuntime->LegacyCommunication' = [pscustomobject]@{
        MaximumOccurrences = 2
        Reason = 'supervisor still depends on concrete CAN and USB interfaces'
    }
    'LegacyRuntime->LegacyPorts' = [pscustomobject]@{
        MaximumOccurrences = 21
        Reason = 'runtime still implements and consumes legacy port contracts'
    }
    'LegacyRuntime->LegacyProduct' = [pscustomobject]@{
        MaximumOccurrences = 32
        Reason = 'runtime modules still consume broad Product profiles'
    }
}

$headersByName = @{}
foreach ($header in $firmwareFiles | Where-Object { $_.Extension -eq '.h' }) {
    $key = $header.Name.ToLowerInvariant()
    if (-not $headersByName.ContainsKey($key)) {
        $headersByName[$key] = [System.Collections.Generic.List[System.IO.FileInfo]]::new()
    }
    $headersByName[$key].Add($header)
}

$dependencyEdges = [System.Collections.Generic.List[object]]::new()
$unqualifiedAmbiguousIncludes = [System.Collections.Generic.HashSet[string]]::new()
foreach ($source in $firmwareFiles) {
    $sourceRelative = Get-RepositoryRelativePath $source.FullName
    $sourceLayer = Get-ArchitectureLayer $sourceRelative
    if ($null -eq $sourceLayer) {
        $failures.Add("Source is outside a recognized architecture layer: $sourceRelative")
        continue
    }

    foreach ($line in Get-Content -LiteralPath $source.FullName) {
        if ($line -notmatch '^\s*#include\s+["<]([^">]+)[">]') {
            continue
        }
        $includeText = $Matches[1].Replace('\', '/')
        $includeName = [System.IO.Path]::GetFileName($includeText).ToLowerInvariant()
        if (-not $headersByName.ContainsKey($includeName)) {
            continue
        }

        $candidates = @($headersByName[$includeName])
        if ($includeText.Contains('/')) {
            $suffix = '/' + $includeText.ToLowerInvariant()
            $qualifiedCandidates = @($candidates | Where-Object {
                ('/' + (Get-RepositoryRelativePath $_.FullName).ToLowerInvariant()).
                    EndsWith($suffix)
            })
            if ($qualifiedCandidates.Count -ne 0) {
                $candidates = $qualifiedCandidates
            }
        }
        if ($candidates.Count -ne 1) {
            $ambiguousKey = "$sourceRelative -> $includeText"
            if ($unqualifiedAmbiguousIncludes.Add($ambiguousKey)) {
                $failures.Add(
                    "Ambiguous internal include '$includeText' in $sourceRelative; " +
                    'use a qualified include path')
            }
            continue
        }

        $targetRelative = Get-RepositoryRelativePath $candidates[0].FullName
        $targetLayer = Get-ArchitectureLayer $targetRelative
        if ($null -eq $targetLayer) {
            $failures.Add("Included header is outside a recognized layer: $targetRelative")
            continue
        }
        $dependencyEdges.Add([pscustomobject]@{
            SourceLayer = $sourceLayer
            TargetLayer = $targetLayer
            Source = $sourceRelative
            Target = $targetRelative
            Include = $includeText
        })
    }
}

$dependencyViolations = @($dependencyEdges | Where-Object {
    -not $allowedDependencies.ContainsKey($_.SourceLayer) -or
    $allowedDependencies[$_.SourceLayer] -notcontains $_.TargetLayer
})
$violationGroups = @($dependencyViolations | Group-Object -Property {
    "$($_.SourceLayer)->$($_.TargetLayer)"
})
foreach ($group in $violationGroups | Sort-Object Name) {
    $key = $group.Name
    if ($legacyDependencyAllowlist.ContainsKey($key)) {
        $allowance = $legacyDependencyAllowlist[$key]
        if ($group.Count -le $allowance.MaximumOccurrences) {
            $legacyExceptions.Add(
                "Dependency ${key}: $($group.Count)/$($allowance.MaximumOccurrences) " +
                "allowed; $($allowance.Reason)")
            continue
        }
        $failures.Add(
            "Dependency $key expanded to $($group.Count) includes; legacy maximum " +
            "is $($allowance.MaximumOccurrences)")
        continue
    }

    $examples = @($group.Group | Select-Object -First 3 | ForEach-Object {
        "$($_.Source) includes $($_.Include)"
    }) -join '; '
    $failures.Add("Forbidden dependency $key ($($group.Count) includes): $examples")
}

if ($FailOnLegacyExceptions -and $legacyExceptions.Count -ne 0) {
    $failures.Add(
        "$($legacyExceptions.Count) explicitly allowlisted migration exceptions remain")
}

Write-Output (
    "Architecture v2 verification: files=$($firmwareFiles.Count) " +
    "dependency_edges=$($dependencyEdges.Count) failures=$($failures.Count) " +
    "warnings=$($warnings.Count) legacy_exceptions=$($legacyExceptions.Count)")
foreach ($legacyException in $legacyExceptions) {
    Write-Output "LEGACY: $legacyException"
}
foreach ($warning in $warnings) {
    Write-Output "WARNING: $warning"
}
foreach ($failure in $failures) {
    Write-Output "ERROR: $failure"
}

if ($failures.Count -ne 0) {
    exit 1
}
exit 0
