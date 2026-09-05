param(
    [string]$Port = "COM4",
    [string]$OutputDirectory = "validation/mode15",
    [double]$MaximumRunSeconds = 8.0,
    [double]$MinimumBusVoltage = 22.0
)

$ErrorActionPreference = "Stop"
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, "None", 8, "One")
$serial.ReadTimeout = 1500
$serial.WriteTimeout = 1500
$serial.NewLine = "`n"
$samples = [System.Collections.Generic.List[object]]::new()
$stopwatch = [System.Diagnostics.Stopwatch]::new()

function Invoke-VectorCommand {
    param([Parameter(Mandatory = $true)][string]$Command)
    $serial.DiscardInBuffer()
    $serial.Write("\$Command`r`n")
    return $serial.ReadLine().Trim()
}

function Read-Number {
    param([Parameter(Mandatory = $true)][string]$Response)
    if ($Response -notmatch '^[^=]+=(-?\d+(?:\.\d+)?)(?:[^\d].*)?$') {
        throw "Unexpected numeric response '$Response'."
    }
    return [double]$Matches[1]
}

try {
    New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
    $serial.Open()
    Start-Sleep -Milliseconds 250
    $mode = Invoke-VectorCommand "r_mod"
    $errorCode = Invoke-VectorCommand "r_err"
    $initialBusVoltage = Read-Number (Invoke-VectorCommand "r_vbs")
    if ($mode -ne "mode=0" -or $errorCode -ne "error=0" -or
        $initialBusVoltage -lt $MinimumBusVoltage) {
        throw "Mode 15 preflight failed: $mode, $errorCode, vbus=$initialBusVoltage V."
    }

    $response = Invoke-VectorCommand "w_mod=15"
    if ($response -notmatch '^(OK|Write Success!|mode=15)') {
        throw "Mode 15 command was rejected: '$response'."
    }

    $stopwatch.Start()
    $terminalReason = "maximum_run_time"
    while ($stopwatch.Elapsed.TotalSeconds -lt $MaximumRunSeconds) {
        $modeValue = Read-Number (Invoke-VectorCommand "r_mod")
        $faultValue = Read-Number (Invoke-VectorCommand "r_err")
        $busVoltage = Read-Number (Invoke-VectorCommand "r_vbs")
        $id = Read-Number (Invoke-VectorCommand "r_i_d")
        $iq = Read-Number (Invoke-VectorCommand "r_i_q")
        $samples.Add([pscustomobject]@{
            time_s = [math]::Round($stopwatch.Elapsed.TotalSeconds, 4)
            mode = [int]$modeValue
            error = [int]$faultValue
            bus_voltage_v = $busVoltage
            id_a = $id
            iq_a = $iq
        })
        if ($busVoltage -lt $MinimumBusVoltage) {
            $terminalReason = "bus_undervoltage_guard"
            break
        }
        if ($faultValue -ne 0) {
            $terminalReason = "firmware_fault_$([int]$faultValue)"
            break
        }
        if ($modeValue -eq 0) {
            $terminalReason = "mode15_completed"
            break
        }
        Start-Sleep -Milliseconds 40
    }

    $summary = [ordered]@{
        terminal_reason = $terminalReason
        elapsed_s = [math]::Round($stopwatch.Elapsed.TotalSeconds, 4)
        sample_count = $samples.Count
        final_mode = if ($samples.Count) { $samples[-1].mode } else { $null }
        final_error = if ($samples.Count) { $samples[-1].error } else { $null }
        minimum_bus_voltage_v = if ($samples.Count) { ($samples.bus_voltage_v | Measure-Object -Minimum).Minimum } else { $null }
        maximum_id_a = if ($samples.Count) { ($samples.id_a | Measure-Object -Maximum).Maximum } else { $null }
        maximum_abs_iq_a = if ($samples.Count) { ($samples.iq_a | ForEach-Object { [math]::Abs($_) } | Measure-Object -Maximum).Maximum } else { $null }
    }
    $samples | Export-Csv -LiteralPath (Join-Path $OutputDirectory "mode15_usb.csv") -NoTypeInformation -Encoding UTF8
    $summary | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutputDirectory "mode15_summary.json") -Encoding UTF8
    Write-Output ($summary | ConvertTo-Json -Compress)
    if ($terminalReason -ne "mode15_completed") {
        throw "Mode 15 did not complete: $terminalReason."
    }
} finally {
    if ($serial.IsOpen) {
        try { Invoke-VectorCommand "w_mod=0" | Out-Null } catch { }
        $serial.Close()
    }
    $serial.Dispose()
}
