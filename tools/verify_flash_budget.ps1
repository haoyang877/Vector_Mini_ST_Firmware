param(
    [string]$MapPath,
    [string]$Region = 'LR_IROM1',
    [string]$MinimumRemainingBytes = '2048',
    [string]$BoardMemoryMapPath,
    [string]$ScatterPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Stop-FlashBudgetCheck {
    param(
        [string]$Reason,
        [string]$Detail
    )

    Write-Output 'FLASH_BUDGET_RESULT=ERROR'
    Write-Output "FLASH_BUDGET_REASON=$Reason"
    if (-not [string]::IsNullOrWhiteSpace($Detail)) {
        $singleLineDetail = $Detail -replace '[\r\n]+', ' '
        Write-Output "FLASH_BUDGET_DETAIL=$singleLineDetail"
    }
    exit 2
}

function Convert-HexToUInt64 {
    param(
        [string]$HexText,
        [string]$FieldName
    )

    try {
        return [Convert]::ToUInt64($HexText, 16)
    }
    catch {
        throw "Invalid hexadecimal $FieldName value '0x$HexText'."
    }
}

function Resolve-RepositoryPath {
    param(
        [string]$RequestedPath,
        [string]$DefaultRelativePath
    )

    $candidate = if ([string]::IsNullOrWhiteSpace($RequestedPath)) {
        Join-Path $repositoryRoot $DefaultRelativePath
    }
    elseif ([System.IO.Path]::IsPathRooted($RequestedPath)) {
        $RequestedPath
    }
    else {
        Join-Path (Get-Location).Path $RequestedPath
    }
    return [System.IO.Path]::GetFullPath($candidate)
}

function Read-NumericDefine {
    param(
        [string]$Path,
        [string]$Name
    )

    $escapedName = [System.Text.RegularExpressions.Regex]::Escape($Name)
    $pattern = '^\s*#define\s+' + $escapedName +
        '\s+(?<Value>0[xX][0-9A-Fa-f]+|[0-9]+)(?:[uUlL]+)?\s*$'
    $matches = @(Select-String -LiteralPath $Path -Pattern $pattern)
    if ($matches.Count -ne 1) {
        throw "Expected exactly one numeric #define for $Name in $Path."
    }
    $textValue = $matches[0].Matches[0].Groups['Value'].Value
    if ($textValue.StartsWith('0x',
            [System.StringComparison]::OrdinalIgnoreCase)) {
        return [Convert]::ToUInt64($textValue.Substring(2), 16)
    }
    return [Convert]::ToUInt64($textValue,
        [System.Globalization.CultureInfo]::InvariantCulture)
}

$minimumBytes = [uint64]0
if (-not [uint64]::TryParse(
        $MinimumRemainingBytes,
        [System.Globalization.NumberStyles]::None,
        [System.Globalization.CultureInfo]::InvariantCulture,
        [ref]$minimumBytes)) {
    Stop-FlashBudgetCheck -Reason 'INVALID_MINIMUM_REMAINING_BYTES' `
        -Detail "Expected a non-negative decimal byte count, got '$MinimumRemainingBytes'."
}

if ([string]::IsNullOrWhiteSpace($Region)) {
    Stop-FlashBudgetCheck -Reason 'INVALID_REGION' `
        -Detail 'Region must be a non-empty Keil load or execution region name.'
}

$repositoryRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if ([string]::IsNullOrWhiteSpace($MapPath)) {
    $candidateMapPath = Join-Path $repositoryRoot `
        'MDK-ARM\Vector_Mini_ST\Vector_Mini_ST.map'
}
elseif ([System.IO.Path]::IsPathRooted($MapPath)) {
    $candidateMapPath = $MapPath
}
else {
    $candidateMapPath = Join-Path (Get-Location).Path $MapPath
}

try {
    $resolvedMapPath = [System.IO.Path]::GetFullPath($candidateMapPath)
}
catch {
    Stop-FlashBudgetCheck -Reason 'INVALID_MAP_PATH' -Detail $_.Exception.Message
}

if (-not (Test-Path -LiteralPath $resolvedMapPath -PathType Leaf)) {
    Stop-FlashBudgetCheck -Reason 'MAP_NOT_FOUND' `
        -Detail "Map file does not exist: $resolvedMapPath"
}

$regionMatches = [System.Collections.Generic.List[object]]::new()
try {
    foreach ($line in [System.IO.File]::ReadLines($resolvedMapPath)) {
        if ($line -notmatch '^\s*(?<Kind>Load|Execution)\s+Region\s+' +
                '(?<Name>\S+)\s+\((?<Details>.*)\)\s*$') {
            continue
        }

        $kind = $Matches['Kind']
        $name = $Matches['Name']
        $details = $Matches['Details']
        if (-not $name.Equals($Region,
                [System.StringComparison]::OrdinalIgnoreCase)) {
            continue
        }

        if ($details -notmatch '(?:^|,\s*)Size:\s*0x(?<Value>[0-9A-Fa-f]+)') {
            throw "Region '$name' has no parseable Size field."
        }
        $sizeBytes = Convert-HexToUInt64 -HexText $Matches['Value'] `
            -FieldName 'Size'

        if ($details -notmatch '(?:^|,\s*)Base:\s*0x(?<Value>[0-9A-Fa-f]+)') {
            throw "Region '$name' has no parseable Base field."
        }
        $baseAddress = Convert-HexToUInt64 -HexText $Matches['Value'] `
            -FieldName 'Base'

        if ($details -notmatch '(?:^|,\s*)Max:\s*0x(?<Value>[0-9A-Fa-f]+)') {
            throw "Region '$name' has no parseable Max field."
        }
        $maximumBytes = Convert-HexToUInt64 -HexText $Matches['Value'] `
            -FieldName 'Max'

        $usedBytes = $sizeBytes
        $usedSource = 'Size'
        if ($kind -eq 'Load' -and
                $details -match 'COMPRESSED\[0x(?<Value>[0-9A-Fa-f]+)\]') {
            $usedBytes = Convert-HexToUInt64 -HexText $Matches['Value'] `
                -FieldName 'COMPRESSED'
            $usedSource = 'COMPRESSED'
        }

        $regionMatches.Add([pscustomobject]@{
            Kind = $kind
            Name = $name
            BaseAddress = $baseAddress
            SizeBytes = $sizeBytes
            MaximumBytes = $maximumBytes
            UsedBytes = $usedBytes
            UsedSource = $usedSource
        })
    }
}
catch {
    Stop-FlashBudgetCheck -Reason 'MAP_READ_OR_PARSE_FAILED' `
        -Detail $_.Exception.Message
}

if ($regionMatches.Count -eq 0) {
    Stop-FlashBudgetCheck -Reason 'REGION_NOT_FOUND' `
        -Detail "Region '$Region' was not found in map file: $resolvedMapPath"
}
if ($regionMatches.Count -gt 1) {
    Stop-FlashBudgetCheck -Reason 'REGION_AMBIGUOUS' `
        -Detail "Region '$Region' occurs $($regionMatches.Count) times in map file."
}

$selectedRegion = $regionMatches[0]
if ($selectedRegion.MaximumBytes -eq 0) {
    Stop-FlashBudgetCheck -Reason 'INVALID_REGION_MAXIMUM' `
        -Detail "Region '$Region' reports Max=0."
}

try {
    $resolvedBoardMemoryMapPath = Resolve-RepositoryPath `
        -RequestedPath $BoardMemoryMapPath `
        -DefaultRelativePath 'Firmware\Bsp\Boards\VectorMiniSt\vector_mini_st_memory_map.h'
    $resolvedScatterPath = Resolve-RepositoryPath -RequestedPath $ScatterPath `
        -DefaultRelativePath 'MDK-ARM\Vector_Mini_ST\Vector_Mini_ST.sct'
    if (-not (Test-Path -LiteralPath $resolvedBoardMemoryMapPath -PathType Leaf)) {
        throw "Board memory map does not exist: $resolvedBoardMemoryMapPath"
    }
    if (-not (Test-Path -LiteralPath $resolvedScatterPath -PathType Leaf)) {
        throw "Scatter file does not exist: $resolvedScatterPath"
    }

    $boardFlashBase = Read-NumericDefine -Path $resolvedBoardMemoryMapPath `
        -Name 'BSP_VECTOR_MINI_ST_FLASH_BASE_ADDRESS'
    $boardFlashCapacity = Read-NumericDefine -Path $resolvedBoardMemoryMapPath `
        -Name 'BSP_VECTOR_MINI_ST_FLASH_CAPACITY_BYTES'
    $boardApplicationCapacity = Read-NumericDefine `
        -Path $resolvedBoardMemoryMapPath `
        -Name 'BSP_VECTOR_MINI_ST_APPLICATION_CAPACITY_BYTES'
    $boardStorageBase = Read-NumericDefine -Path $resolvedBoardMemoryMapPath `
        -Name 'BSP_VECTOR_MINI_ST_PARAMETER_STORAGE_BASE_ADDRESS'
    $boardStorageCapacity = Read-NumericDefine `
        -Path $resolvedBoardMemoryMapPath `
        -Name 'BSP_VECTOR_MINI_ST_PARAMETER_STORAGE_CAPACITY_BYTES'

    if ($boardStorageBase -ne $boardFlashBase + $boardApplicationCapacity -or
            $boardApplicationCapacity + $boardStorageCapacity -ne
                $boardFlashCapacity) {
        throw 'Board memory-map regions are not contiguous or do not cover Flash.'
    }

    $escapedRegion = [System.Text.RegularExpressions.Regex]::Escape($Region)
    $scatterMatches = @(Select-String -LiteralPath $resolvedScatterPath `
        -Pattern ('^\s*' + $escapedRegion +
            '\s+0[xX](?<Base>[0-9A-Fa-f]+)\s+0[xX](?<Size>[0-9A-Fa-f]+)'))
    if ($scatterMatches.Count -ne 1) {
        throw "Expected exactly one $Region declaration in $resolvedScatterPath."
    }
    $scatterBase = [Convert]::ToUInt64(
        $scatterMatches[0].Matches[0].Groups['Base'].Value, 16)
    $scatterSize = [Convert]::ToUInt64(
        $scatterMatches[0].Matches[0].Groups['Size'].Value, 16)

    if ($scatterBase -ne $boardFlashBase -or
            $scatterSize -ne $boardApplicationCapacity -or
            $selectedRegion.BaseAddress -ne $boardFlashBase -or
            $selectedRegion.MaximumBytes -ne $boardApplicationCapacity) {
        throw ('Board memory map, scatter file, and linked map disagree: ' +
            "board=0x$($boardFlashBase.ToString('X8'))/0x$($boardApplicationCapacity.ToString('X')), " +
            "scatter=0x$($scatterBase.ToString('X8'))/0x$($scatterSize.ToString('X')), " +
            "map=0x$($selectedRegion.BaseAddress.ToString('X8'))/0x$($selectedRegion.MaximumBytes.ToString('X')).")
    }
}
catch {
    Stop-FlashBudgetCheck -Reason 'LAYOUT_MISMATCH' -Detail $_.Exception.Message
}

Write-Output "FLASH_BUDGET_MAP=$resolvedMapPath"
Write-Output "FLASH_BUDGET_BOARD_MEMORY_MAP=$resolvedBoardMemoryMapPath"
Write-Output "FLASH_BUDGET_SCATTER=$resolvedScatterPath"
Write-Output "FLASH_BUDGET_REGION=$($selectedRegion.Name)"
Write-Output "FLASH_BUDGET_REGION_KIND=$($selectedRegion.Kind)"
Write-Output "FLASH_BUDGET_USED_SOURCE=$($selectedRegion.UsedSource)"
Write-Output "FLASH_BUDGET_USED_BYTES=$($selectedRegion.UsedBytes)"
Write-Output "FLASH_BUDGET_MAX_BYTES=$($selectedRegion.MaximumBytes)"
Write-Output "FLASH_BUDGET_MINIMUM_REMAINING_BYTES=$minimumBytes"

if ($selectedRegion.UsedBytes -gt $selectedRegion.MaximumBytes) {
    $overBudgetBytes = $selectedRegion.UsedBytes - $selectedRegion.MaximumBytes
    Write-Output "FLASH_BUDGET_REMAINING_BYTES=-$overBudgetBytes"
    Write-Output 'FLASH_BUDGET_RESULT=FAIL'
    Write-Output 'FLASH_BUDGET_REASON=REGION_OVERFLOW'
    exit 1
}

$remainingBytes = $selectedRegion.MaximumBytes - $selectedRegion.UsedBytes
Write-Output "FLASH_BUDGET_REMAINING_BYTES=$remainingBytes"
if ($remainingBytes -lt $minimumBytes) {
    Write-Output 'FLASH_BUDGET_RESULT=FAIL'
    Write-Output 'FLASH_BUDGET_REASON=INSUFFICIENT_HEADROOM'
    exit 1
}

Write-Output 'FLASH_BUDGET_RESULT=PASS'
exit 0
