param(
    [string]$Port = "COM4",
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [string]$Prefix = "mode5"
)

$ErrorActionPreference = "Stop"
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, "None", 8, "One")
$serial.ReadTimeout = 2500
$serial.WriteTimeout = 2500
$serial.NewLine = "`n"

function Invoke-VectorCommand {
    param([Parameter(Mandatory = $true)][string]$Command)

    $serial.DiscardInBuffer()
    $serial.Write("\$Command`r`n")
    return $serial.ReadLine().Trim()
}

try {
    New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
    $serial.Open()
    Start-Sleep -Milliseconds 250

    $metadata = [ordered]@{}
    foreach ($command in @(
        "r_mod", "r_err", "r_e_s", "r_erv", "r_pol", "r_ica", "r_ilm",
        "r_slm", "r_vbs", "r_p2f", "r_mrs", "r_mld", "r_mlq", "r_mfx"
    )) {
        $metadata[$command] = Invoke-VectorCommand -Command $command
    }

    if ($metadata["r_mod"] -ne "mode=0") {
        throw "LUT export requires mode 0; received '$($metadata['r_mod'])'."
    }
    if ($metadata["r_err"] -ne "error=0") {
        throw "LUT export requires error 0; received '$($metadata['r_err'])'."
    }

    $serial.DiscardInBuffer()
    $serial.Write("\r_lut`r`n")
    $deadline = [DateTime]::UtcNow.AddSeconds(30)
    $rows = [System.Collections.Generic.List[object]]::new()
    $rawLines = [System.Collections.Generic.List[string]]::new()
    $sawBegin = $false

    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $line = $serial.ReadLine().Trim()
        } catch [System.TimeoutException] {
            continue
        }
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        $rawLines.Add($line)
        if ($line -match '^lut_begin,count=(\d+),reverse=(\d+)$') {
            $metadata["lut_count"] = [int]$Matches[1]
            $metadata["lut_reverse"] = [int]$Matches[2]
            $sawBegin = $true
            continue
        }
        if ($line -eq "lut_end") {
            break
        }
        if ($line -match '^lut=(\d+),raw_deg=(-?\d+(?:\.\d+)?),err_deg=(-?\d+(?:\.\d+)?)$') {
            $rows.Add([pscustomobject]@{
                index = [int]$Matches[1]
                raw_deg = [double]$Matches[2]
                error_deg = [double]$Matches[3]
            })
        }
    }

    if (-not $sawBegin) {
        throw "The target did not start LUT export."
    }
    if ($rows.Count -ne $metadata["lut_count"]) {
        throw "Incomplete LUT export: expected $($metadata['lut_count']) rows, received $($rows.Count)."
    }

    $metadata["captured_at"] = [DateTimeOffset]::Now.ToString("o")
    $metadata["port"] = $Port
    $metadata["row_count"] = $rows.Count
    $metadata | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutputDirectory "${Prefix}_metadata.json") -Encoding UTF8
    $rows | Export-Csv -LiteralPath (Join-Path $OutputDirectory "${Prefix}_encoder_lut.csv") -NoTypeInformation -Encoding UTF8
    $rawLines | Set-Content -LiteralPath (Join-Path $OutputDirectory "${Prefix}_encoder_lut_raw.txt") -Encoding UTF8

    Write-Output "Captured $($rows.Count) LUT entries to $OutputDirectory"
    $metadata.GetEnumerator() | ForEach-Object { Write-Output "$($_.Key)=$($_.Value)" }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
