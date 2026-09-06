Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$repositoryPrefix = $repositoryRoot.TrimEnd('\', '/') +
    [System.IO.Path]::DirectorySeparatorChar
$firmwareRoot = Join-Path $repositoryRoot 'Firmware'
$baseGatePath = Join-Path $PSScriptRoot 'verify_architecture.ps1'
$failures = [System.Collections.Generic.List[string]]::new()

function Get-RepositoryRelativePath {
    param([string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if (-not $fullPath.StartsWith($repositoryPrefix,
        [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Path is outside the repository: $fullPath"
    }
    return $fullPath.Substring($repositoryPrefix.Length).Replace('\', '/')
}

function Get-ArchitectureLayer {
    param([string]$RelativePath)

    $path = $RelativePath.Replace('\', '/')
    switch -Regex ($path) {
        '^Firmware/Bsp/Boards/[^/]+/Bootstrap/' { return 'CompositionRoot' }
        '^Firmware/Core/Application/' { return 'CoreApplication' }
        '^Firmware/Core/Services/' { return 'CoreServices' }
        '^Firmware/Core/Config/' { return 'CoreConfig' }
        '^Firmware/Core/Infrastructure/' { return 'CoreInfrastructure' }
        '^Firmware/Core/Communication/Contracts/' { return 'CommContracts' }
        '^Firmware/Core/Communication/Protocol/' { return 'CommProtocol' }
        '^Firmware/Core/Communication/Formatting/' { return 'CommFormatting' }
        '^Firmware/Core/Communication/Transport/' { return 'CommTransport' }
        '^Firmware/Core/Communication/Can/' { return 'CommCan' }
        '^Firmware/Core/Communication/Router/' { return 'CommRouter' }
        '^Firmware/Core/Communication/Interfaces/' { return 'CommInterfaces' }
        '^Firmware/Core/Communication/' { return 'CoreCommunication' }
        '^Firmware/Drivers/' { return 'Drivers' }
        '^Firmware/Bsp/Api/' { return 'BspApi' }
        '^Firmware/Bsp/Boards/' { return 'BspBoards' }
        '^Firmware/Platform/' { return 'Platform' }
        default { return $null }
    }
}

function Add-ContentMatches {
    param(
        [System.IO.FileInfo[]]$Files,
        [string]$Pattern,
        [string]$Description
    )

    foreach ($file in $Files) {
        foreach ($match in Select-String -LiteralPath $file.FullName `
            -Pattern $Pattern) {
            $relativePath = Get-RepositoryRelativePath $file.FullName
            $failures.Add("$Description at ${relativePath}:$($match.LineNumber)")
        }
    }
}

# V2 is the dependency-depth extension of the structural/build gate. Run the
# base gate first so both entry points enforce one physical architecture.
$powershellPath = (Get-Process -Id $PID).Path
$baseOutput = @(& $powershellPath -NoProfile -File $baseGatePath 2>&1)
$baseExitCode = $LASTEXITCODE
$baseOutput | ForEach-Object { Write-Output $_ }
if ($baseExitCode -ne 0) {
    Write-Output 'Architecture v2 verification: base_gate=FAILED'
    exit 1
}

$protectedProductPath = [System.IO.Path]::GetFullPath(
    (Join-Path $repositoryRoot 'Firmware\Product\control_loop_config.h'))
$firmwareFiles = @(Get-ChildItem -LiteralPath $firmwareRoot -Recurse -File |
    Where-Object {
        $_.Extension -in @('.c', '.h') -and
        -not $_.FullName.Equals($protectedProductPath,
            [System.StringComparison]::OrdinalIgnoreCase)
    })

# Portable code cannot acquire MCU SDK, generated-peripheral, register, or
# concrete board/device dependencies. Config may name product models as data,
# and Drivers may name the component they implement, but neither may use MCU
# SDK APIs.
$mcuDependencyTerms = '#include\s+["<](?:stm32[^">]*|cmsis[^">]*|' +
    'main\.h|adc\.h|tim\.h|spi\.h|fdcan\.h|gpio\.h|usbd[^">]*)[">]|' +
    '\bHAL_[A-Za-z0-9_]+|\bLL_[A-Za-z0-9_]+|' +
    '\b(?:ADC|TIM|SPI|FDCAN|GPIO|DMA|UART|USB)_HandleTypeDef\b|' +
    '\b(?:ADC|TIM|SPI|FDCAN|GPIO)[A-Z0-9_]*->|' +
    '\b__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK)\b|' +
    '\b(?:STM32[A-Z0-9_]*)\b'
$mcuDependencyPattern = '(?i)(' + $mcuDependencyTerms + ')'
$concreteDependencyPattern = '(?i)(' + $mcuDependencyTerms +
    '|\b(?:CMSIS|VECTOR_?MINI(?:_?ST)?|TLE5012B|AS5047|MT6701|MT6835|' +
    'DRV83[0-9A-Z]*)\b)'

foreach ($relativeDirectory in @(
    'Firmware/Core/Application',
    'Firmware/Core/Services',
    'Firmware/Core/Communication',
    'Firmware/Core/Infrastructure')) {
    $directory = Join-Path $repositoryRoot $relativeDirectory
    $files = @(Get-ChildItem -LiteralPath $directory -Recurse -File |
        Where-Object { $_.Extension -in @('.c', '.h') })
    Add-ContentMatches -Files $files -Pattern $concreteDependencyPattern `
        -Description "$relativeDirectory contains a concrete platform dependency"
}

$coreConfigFiles = @(Get-ChildItem -LiteralPath `
    (Join-Path $firmwareRoot 'Core\Config') -Recurse -File |
    Where-Object { $_.Extension -in @('.c', '.h') })
Add-ContentMatches -Files $coreConfigFiles -Pattern $mcuDependencyPattern `
    -Description 'Core/Config contains an MCU implementation dependency'

$bspApiFiles = @(Get-ChildItem -LiteralPath `
    (Join-Path $firmwareRoot 'Bsp\Api') -Recurse -File |
    Where-Object { $_.Extension -in @('.c', '.h') })
Add-ContentMatches -Files $bspApiFiles -Pattern $concreteDependencyPattern `
    -Description 'Bsp/Api contains a concrete platform dependency'

$driverFiles = @(Get-ChildItem -LiteralPath `
    (Join-Path $firmwareRoot 'Drivers') -Recurse -File |
    Where-Object { $_.Extension -in @('.c', '.h') })
Add-ContentMatches -Files $driverFiles -Pattern $mcuDependencyPattern `
    -Description 'Drivers contains an MCU implementation dependency'

# Allowed arrows are exhaustive. Any layer absent from a row is forbidden.
# Communication sublayers are ordered so Protocol/Transport remain reusable
# and cannot depend back on Router or Interfaces.
$applicationTargets = @(
    'CoreApplication', 'CoreServices', 'CoreConfig',
    'CoreInfrastructure', 'BspApi')
$communicationOuterTargets = @(
    'CoreApplication', 'CoreServices', 'CoreConfig',
    'CoreInfrastructure', 'BspApi')
$allowedDependencies = @{
    'CoreServices' = @('CoreServices', 'CoreConfig')
    'CoreConfig' = @('CoreConfig')
    'CoreInfrastructure' = @('CoreInfrastructure', 'CoreConfig', 'BspApi')
    'CoreApplication' = $applicationTargets

    'CommContracts' = @('CommContracts')
    'CommProtocol' = @('CommProtocol', 'CommContracts', 'BspApi')
    'CommFormatting' = @('CommFormatting')
    'CommTransport' = @('CommTransport', 'CommContracts', 'BspApi')
    'CommCan' = @('CommCan', 'CommContracts')
    'CommRouter' = @(
        'CommRouter', 'CommProtocol', 'CommFormatting', 'CommCan',
        'CommContracts') + $communicationOuterTargets
    'CommInterfaces' = @(
        'CommInterfaces', 'CommRouter', 'CommProtocol', 'CommFormatting',
        'CommTransport', 'CommCan', 'CommContracts') +
        $communicationOuterTargets
    'CoreCommunication' = @(
        'CoreCommunication', 'CommContracts', 'CommProtocol',
        'CommFormatting', 'CommTransport', 'CommCan', 'CommRouter') +
        $communicationOuterTargets

    'Drivers' = @('Drivers', 'BspApi')
    'BspApi' = @('BspApi')
    'Platform' = @('Platform', 'BspApi')
    'BspBoards' = @(
        'BspBoards', 'BspApi', 'CoreConfig', 'Platform', 'Drivers')
    'CompositionRoot' = @(
        'CompositionRoot', 'CoreApplication', 'CoreServices', 'CoreConfig',
        'CoreInfrastructure', 'CoreCommunication', 'CommContracts',
        'CommProtocol', 'CommFormatting', 'CommTransport', 'CommCan',
        'CommRouter', 'CommInterfaces', 'Drivers', 'BspApi', 'BspBoards',
        'Platform')
}

$headersByName = @{}
foreach ($header in $firmwareFiles | Where-Object { $_.Extension -eq '.h' }) {
    $key = $header.Name.ToLowerInvariant()
    if (-not $headersByName.ContainsKey($key)) {
        $headersByName[$key] =
            [System.Collections.Generic.List[System.IO.FileInfo]]::new()
    }
    $headersByName[$key].Add($header)
}

$dependencyEdges = [System.Collections.Generic.List[object]]::new()
foreach ($source in $firmwareFiles) {
    $sourceRelative = Get-RepositoryRelativePath $source.FullName
    $sourceLayer = Get-ArchitectureLayer $sourceRelative
    if ($null -eq $sourceLayer) {
        $failures.Add("Source is outside a final architecture layer: $sourceRelative")
        continue
    }

    foreach ($line in Get-Content -LiteralPath $source.FullName) {
        if ($line -notmatch '^\s*#include\s+(["<])([^">]+)[">]') {
            continue
        }
        $delimiter = $Matches[1]
        $includeText = $Matches[2].Replace('\', '/')
        $includeName = [System.IO.Path]::GetFileName($includeText).
            ToLowerInvariant()
        if (-not $headersByName.ContainsKey($includeName)) {
            continue
        }

        $candidates = @()
        if ($delimiter -eq '"') {
            $relativeCandidate = [System.IO.Path]::GetFullPath(
                (Join-Path $source.DirectoryName $includeText))
            $candidates = @($headersByName[$includeName] | Where-Object {
                $_.FullName.Equals($relativeCandidate,
                    [System.StringComparison]::OrdinalIgnoreCase)
            })
        }
        if ($candidates.Count -eq 0 -and $includeText.Contains('/')) {
            $suffix = '/' + $includeText.ToLowerInvariant().TrimStart('.', '/')
            $candidates = @($headersByName[$includeName] | Where-Object {
                ('/' + (Get-RepositoryRelativePath $_.FullName).
                    ToLowerInvariant()).EndsWith($suffix)
            })
        }
        if ($candidates.Count -eq 0 -and -not $includeText.Contains('/')) {
            $candidates = @($headersByName[$includeName])
        }
        if ($candidates.Count -ne 1) {
            $failures.Add(
                "Internal include '$includeText' in $sourceRelative resolves to " +
                "$($candidates.Count) headers")
            continue
        }

        $targetRelative = Get-RepositoryRelativePath $candidates[0].FullName
        $targetLayer = Get-ArchitectureLayer $targetRelative
        if ($null -eq $targetLayer) {
            $failures.Add("Included header is outside a final layer: $targetRelative")
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
foreach ($group in $dependencyViolations | Group-Object -Property {
    "$($_.SourceLayer)->$($_.TargetLayer)"
} | Sort-Object Name) {
    $examples = @($group.Group | Select-Object -First 3 | ForEach-Object {
        "$($_.Source) includes $($_.Include)"
    }) -join '; '
    $failures.Add(
        "Forbidden dependency $($group.Name) ($($group.Count) includes): $examples")
}

Write-Output (
    "Architecture v2 verification: files=$($firmwareFiles.Count) " +
    "dependency_edges=$($dependencyEdges.Count) failures=$($failures.Count)")
foreach ($failure in $failures) {
    Write-Output "ERROR: $failure"
}

if ($failures.Count -ne 0) {
    exit 1
}
exit 0
