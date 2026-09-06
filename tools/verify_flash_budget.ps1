param(
    [string]$MapPath,
    [string]$Region = 'LR_IROM1',
    [string]$MinimumRemainingBytes = '2048'
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

Write-Output "FLASH_BUDGET_MAP=$resolvedMapPath"
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
