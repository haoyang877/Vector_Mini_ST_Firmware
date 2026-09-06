Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$repositoryPrefix = $repositoryRoot.TrimEnd('\', '/') +
    [System.IO.Path]::DirectorySeparatorChar
$firmwareRoot = Join-Path $repositoryRoot 'Firmware'
$projectPath = Join-Path $repositoryRoot 'MDK-ARM\Vector_Mini_ST.uvprojx'
$projectDirectory = Split-Path -Parent $projectPath
$failures = [System.Collections.Generic.List[string]]::new()

function Get-RepositoryRelativePath {
    param([string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($fullPath.StartsWith($repositoryPrefix,
        [System.StringComparison]::OrdinalIgnoreCase)) {
        return $fullPath.Substring($repositoryPrefix.Length).Replace('\', '/')
    }
    return $fullPath.Replace('\', '/')
}

function Test-PathBelow {
    param(
        [string]$Path,
        [string]$Directory
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $directoryPrefix = [System.IO.Path]::GetFullPath($Directory).
        TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    return $fullPath.StartsWith($directoryPrefix,
        [System.StringComparison]::OrdinalIgnoreCase)
}

function Add-TextMatches {
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

if (-not (Test-Path -LiteralPath $firmwareRoot -PathType Container)) {
    Write-Output 'ERROR: Firmware directory does not exist.'
    exit 1
}
if (-not (Test-Path -LiteralPath $projectPath -PathType Leaf)) {
    Write-Output 'ERROR: Keil project does not exist: MDK-ARM/Vector_Mini_ST.uvprojx'
    exit 1
}

# The production firmware has four physical roots. Product is not a fifth
# layer: its sole retained header is an intentionally uncompiled archive.
$requiredTopLevelDirectories = @('Core', 'Drivers', 'Bsp', 'Platform')
$permittedTopLevelDirectories = @(
    $requiredTopLevelDirectories + @('Product'))
foreach ($requiredDirectory in $requiredTopLevelDirectories) {
    $path = Join-Path $firmwareRoot $requiredDirectory
    if (-not (Test-Path -LiteralPath $path -PathType Container)) {
        $failures.Add("Missing Firmware top-level directory: Firmware/$requiredDirectory")
    }
}
foreach ($directory in Get-ChildItem -LiteralPath $firmwareRoot -Directory) {
    if ($permittedTopLevelDirectories -notcontains $directory.Name) {
        $failures.Add("Unexpected Firmware top-level directory: Firmware/$($directory.Name)")
    }
}
foreach ($file in Get-ChildItem -LiteralPath $firmwareRoot -File) {
    $failures.Add(
        'Files must be owned by a final Firmware layer: ' +
        (Get-RepositoryRelativePath $file.FullName))
}

$coreRoot = Join-Path $firmwareRoot 'Core'
$requiredCoreDirectories = @(
    'Application', 'Services', 'Communication', 'Config', 'Infrastructure')
foreach ($requiredDirectory in $requiredCoreDirectories) {
    $path = Join-Path $coreRoot $requiredDirectory
    if (-not (Test-Path -LiteralPath $path -PathType Container)) {
        $failures.Add("Missing Core layer: Firmware/Core/$requiredDirectory")
    }
}
if (Test-Path -LiteralPath $coreRoot -PathType Container) {
    foreach ($directory in Get-ChildItem -LiteralPath $coreRoot -Directory) {
        if ($requiredCoreDirectories -notcontains $directory.Name) {
            $failures.Add("Unexpected Core layer: Firmware/Core/$($directory.Name)")
        }
    }
}

$bspRoot = Join-Path $firmwareRoot 'Bsp'
$requiredBspDirectories = @('Api', 'Boards')
foreach ($requiredDirectory in $requiredBspDirectories) {
    $path = Join-Path $bspRoot $requiredDirectory
    if (-not (Test-Path -LiteralPath $path -PathType Container)) {
        $failures.Add("Missing BSP layer: Firmware/Bsp/$requiredDirectory")
    }
}
if (Test-Path -LiteralPath $bspRoot -PathType Container) {
    foreach ($directory in Get-ChildItem -LiteralPath $bspRoot -Directory) {
        if ($requiredBspDirectories -notcontains $directory.Name) {
            $failures.Add("Unexpected BSP layer: Firmware/Bsp/$($directory.Name)")
        }
    }
}

# Every concrete board owns its only composition root below Bootstrap. Tests
# and common Bsp/Boards files are not concrete boards.
$boardsRoot = Join-Path $bspRoot 'Boards'
$bootstrapDirectories = @()
if (Test-Path -LiteralPath $boardsRoot -PathType Container) {
    foreach ($boardDirectory in Get-ChildItem -LiteralPath $boardsRoot -Directory) {
        if ($boardDirectory.Name -eq 'Tests') {
            continue
        }
        $bootstrapDirectory = Join-Path $boardDirectory.FullName 'Bootstrap'
        if (-not (Test-Path -LiteralPath $bootstrapDirectory -PathType Container)) {
            $failures.Add(
                "Board has no Bootstrap composition root: Firmware/Bsp/Boards/$($boardDirectory.Name)")
            continue
        }
        $bootstrapDirectories += Get-Item -LiteralPath $bootstrapDirectory
        if (-not (Test-Path -LiteralPath (Join-Path $bootstrapDirectory `
            'firmware_composition.c') -PathType Leaf)) {
            $failures.Add(
                "Board Bootstrap has no firmware_composition.c: $($boardDirectory.Name)")
        }
    }
}
if ($bootstrapDirectories.Count -eq 0) {
    $failures.Add(
        'No composition root exists below Firmware/Bsp/Boards/<board>/Bootstrap')
}

$firmwareCodeFiles = @(Get-ChildItem -LiteralPath $firmwareRoot -Recurse -File |
    Where-Object { $_.Extension -in @('.c', '.h') })
$compositionOwnedNames = @(
    'firmware_composition.c', 'firmware_composition.h',
    'product_config_bridge.c', 'product_config_bridge.h')
foreach ($file in $firmwareCodeFiles | Where-Object {
    $compositionOwnedNames -contains $_.Name }) {
    $relativePath = Get-RepositoryRelativePath $file.FullName
    if ($relativePath -notmatch
        '^Firmware/Bsp/Boards/[^/]+/Bootstrap/[^/]+$') {
        $failures.Add("Composition-root file is outside board Bootstrap: $relativePath")
    }
}
$compositionSymbolPattern = '\b(?:FirmwareComposition|ProductConfigBridge)_'
foreach ($file in $firmwareCodeFiles) {
    $relativePath = Get-RepositoryRelativePath $file.FullName
    if ($relativePath -match '^Firmware/Bsp/Boards/[^/]+/Bootstrap/') {
        continue
    }
    foreach ($match in Select-String -LiteralPath $file.FullName `
        -Pattern $compositionSymbolPattern) {
        $failures.Add(
            "CompositionRoot symbol is outside board Bootstrap at ${relativePath}:$($match.LineNumber)")
    }
}

# This exact archived header may remain physically present, but it must be
# isolated from every production source and the build manifest.
$protectedProductRelativePath = 'Firmware/Product/control_loop_config.h'
$protectedProductPath = Join-Path $repositoryRoot `
    $protectedProductRelativePath.Replace('/', '\')
$productRoot = Join-Path $firmwareRoot 'Product'
$productFiles = @()
if (Test-Path -LiteralPath $productRoot -PathType Container) {
    $productFiles = @(Get-ChildItem -LiteralPath $productRoot -Recurse -File)
}
if (-not (Test-Path -LiteralPath $protectedProductPath -PathType Leaf)) {
    $failures.Add("Missing protected isolated file: $protectedProductRelativePath")
}
foreach ($file in $productFiles) {
    if (-not $file.FullName.Equals($protectedProductPath,
        [System.StringComparison]::OrdinalIgnoreCase)) {
        $failures.Add(
            'Firmware/Product may contain only control_loop_config.h: ' +
            (Get-RepositoryRelativePath $file.FullName))
    }
}

$productionSourceRoots = @(
    'Firmware', 'Core', 'USB_Device', 'Bootloader', 'tests') |
    ForEach-Object { Join-Path $repositoryRoot $_ } |
    Where-Object { Test-Path -LiteralPath $_ -PathType Container }
$productionCodeFiles = @($productionSourceRoots | ForEach-Object {
    Get-ChildItem -LiteralPath $_ -Recurse -File |
        Where-Object { $_.Extension -in @('.c', '.h') }
} | Where-Object {
    -not $_.FullName.Equals($protectedProductPath,
        [System.StringComparison]::OrdinalIgnoreCase)
})
Add-TextMatches -Files $productionCodeFiles `
    -Pattern '(?i)control_loop_config(?:\.h)?' `
    -Description 'Production source references the isolated control-loop header'

$oldLayerIncludePattern = '(?i)^\s*#include\s+["<]' +
    '(?:(?:\.\.[/\\])+)?(?:Firmware[/\\])?' +
    '(?:Application|Communication|Composition|Domain|Ports|Product|Runtime)' +
    '(?:[/\\]|[">])'
Add-TextMatches -Files @($firmwareCodeFiles | Where-Object {
    -not $_.FullName.Equals($protectedProductPath,
        [System.StringComparison]::OrdinalIgnoreCase)
}) -Pattern $oldLayerIncludePattern `
    -Description 'Source references a removed Firmware layer'

[xml]$project = Get-Content -Raw -LiteralPath $projectPath
$projectGroups = @($project.Project.Targets.Target.Groups.Group)
$projectFiles = @($projectGroups.Files.File | Where-Object FilePath)
$projectGroupNames = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$projectPathSet = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$projectFirmwareFiles = [System.Collections.Generic.List[string]]::new()
$projectBootstrapRoots = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)

$removedLayerPathPattern = '(?i)(?:^|[/\\])Firmware[/\\]' +
    '(?:Application|Communication|Composition|Domain|Ports|Product|Runtime)' +
    '(?:[/\\]|$)'
foreach ($projectFile in $projectFiles) {
    $filePath = ([string]$projectFile.FilePath).Replace('/', '\')
    $fullPath = [System.IO.Path]::GetFullPath(
        (Join-Path $projectDirectory $filePath))
    if (-not $projectPathSet.Add($fullPath)) {
        $failures.Add("Duplicate Keil project entry: $filePath")
    }
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        $failures.Add("Keil project source does not exist: $filePath")
        continue
    }
    if ($filePath -match $removedLayerPathPattern -or
        $filePath -match '(?i)control_loop_config(?:\.h)?') {
        $failures.Add("Keil project references a removed or isolated path: $filePath")
    }
    if (Test-PathBelow -Path $fullPath -Directory $firmwareRoot) {
        $relativePath = Get-RepositoryRelativePath $fullPath
        $projectFirmwareFiles.Add($relativePath)
        if ($relativePath -notmatch
            '^Firmware/(?:Core|Drivers|Bsp|Platform)/') {
            $failures.Add("Keil firmware entry is outside the final roots: $relativePath")
        }
        if ($relativePath -match
            '^Firmware/Bsp/Boards/([^/]+)/Bootstrap/') {
            [void]$projectBootstrapRoots.Add($Matches[1])
        }
    }
}

foreach ($group in $projectGroups) {
    $groupName = ([string]$group.GroupName).Replace('\', '/')
    if (-not $projectGroupNames.Add($groupName)) {
        $failures.Add("Duplicate Keil project group: $groupName")
    }
    if ($groupName -match '^Firmware/' -and
        $groupName -notmatch '^Firmware/(?:Core|Drivers|Bsp|Platform)(?:/|$)') {
        $failures.Add("Keil group references a removed Firmware layer: $groupName")
    }
}

$projectIncludePaths = @($project.SelectNodes('//IncludePath') |
    ForEach-Object { ([string]$_.InnerText) -split ';' } |
    Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
foreach ($includePath in $projectIncludePaths) {
    $normalizedPath = $includePath.Trim().Replace('/', '\')
    if ($normalizedPath -match $removedLayerPathPattern -or
        $normalizedPath -match '(?i)control_loop_config(?:\.h)?') {
        $failures.Add(
            "Keil include path references a removed or isolated layer: $includePath")
    }
}

$compiledCompositionRoots = @($projectFirmwareFiles | Where-Object {
    $_ -match '^Firmware/Bsp/Boards/[^/]+/Bootstrap/firmware_composition\.c$'
})
if ($compiledCompositionRoots.Count -ne 1) {
    $failures.Add(
        "Keil target must compile exactly one board CompositionRoot; found $($compiledCompositionRoots.Count)")
}
if ($projectBootstrapRoots.Count -ne 1) {
    $failures.Add(
        "Keil target must select exactly one board Bootstrap; found $($projectBootstrapRoots.Count)")
}

Write-Output (
    "Architecture verification: roots=4 firmware_files=$($firmwareCodeFiles.Count) " +
    "project_entries=$($projectFiles.Count) failures=$($failures.Count)")
foreach ($failure in $failures) {
    Write-Output "ERROR: $failure"
}

if ($failures.Count -ne 0) {
    exit 1
}
exit 0
