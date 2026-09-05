param(
    [string]$Port = "COM4",
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [double]$MaximumRunSeconds = 35.0,
    [double]$MinimumBusVoltage = 22.0
)

$ErrorActionPreference = "Stop"
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, "None", 8, "One")
$serial.ReadTimeout = 1200
$serial.WriteTimeout = 1200
$serial.NewLine = "`n"
$rttProcess = $null
$rows = [System.Collections.Generic.List[object]]::new()
$eventLog = [System.Collections.Generic.List[string]]::new()
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

function Add-Event {
    param([Parameter(Mandatory = $true)][string]$Message)
    $line = "{0:F3}s {1}" -f $stopwatch.Elapsed.TotalSeconds, $Message
    $eventLog.Add($line)
    Write-Output $line
}

try {
    New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
    $outputPath = (Resolve-Path -LiteralPath $OutputDirectory).Path
    $rttFile = Join-Path $outputPath "mode13_rtt.bin"
    $serial.Open()
    Start-Sleep -Milliseconds 250

    $preflight = [ordered]@{}
    foreach ($command in @("r_mod", "r_err", "r_e_s", "r_pol", "r_ilm", "r_vbs", "r_mrs", "r_mld", "r_mlq", "r_mfx")) {
        $preflight[$command] = Invoke-VectorCommand -Command $command
    }
    if ($preflight["r_mod"] -ne "mode=0") {
        throw "Mode 13 preflight requires mode 0; received '$($preflight['r_mod'])'."
    }
    if ($preflight["r_err"] -ne "error=0") {
        throw "Mode 13 preflight requires error 0; received '$($preflight['r_err'])'."
    }
    if ($preflight["r_e_s"] -notmatch 'Online') {
        throw "Mode 13 preflight requires an online encoder; received '$($preflight['r_e_s'])'."
    }
    $initialBusVoltage = Read-Number -Response $preflight["r_vbs"]
    if ($initialBusVoltage -lt $MinimumBusVoltage) {
        throw "Bus voltage $initialBusVoltage V is below the guarded minimum $MinimumBusVoltage V."
    }

    $preflight["captured_at"] = [DateTimeOffset]::Now.ToString("o")
    $preflight["port"] = $Port
    $preflight["minimum_bus_voltage_v"] = $MinimumBusVoltage
    $preflight["maximum_run_seconds"] = $MaximumRunSeconds
    $preflight | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $outputPath "mode13_preflight.json") -Encoding UTF8

    $jlinkLogger = "C:\Program Files\SEGGER\JLink_V964\JLinkRTTLogger.exe"
    $loggerArguments = @(
        "-Device", "STM32G431CB", "-If", "SWD", "-Speed", "4000",
        "-USB", "602722271", "-RTTChannel", "1", "`"$rttFile`""
    )
    $rttProcess = Start-Process -FilePath $jlinkLogger -ArgumentList $loggerArguments -WindowStyle Hidden -PassThru
    Start-Sleep -Seconds 2
    if ($rttProcess.HasExited) {
        throw "J-Link RTT logger exited before the diagnostic run."
    }

    $stopwatch.Start()
    Add-Event -Message "preflight_ok vbus=$initialBusVoltage"
    $modeResponse = Invoke-VectorCommand -Command "w_mod=13"
    Add-Event -Message "mode13_command $modeResponse"
    if ($modeResponse -notmatch '^(OK|Write Success!|mode=13)') {
        throw "Mode 13 command was rejected: '$modeResponse'."
    }

    $terminalReason = "maximum_run_time"
    while ($stopwatch.Elapsed.TotalSeconds -lt $MaximumRunSeconds) {
        $mode = Read-Number -Response (Invoke-VectorCommand -Command "r_mod")
        $errorCode = Read-Number -Response (Invoke-VectorCommand -Command "r_err")
        $busVoltage = Read-Number -Response (Invoke-VectorCommand -Command "r_vbs")
        $busCurrent = Read-Number -Response (Invoke-VectorCommand -Command "r_ibs")
        $speed = Read-Number -Response (Invoke-VectorCommand -Command "r_s2f")
        $id = Read-Number -Response (Invoke-VectorCommand -Command "r_i_d")
        $iq = Read-Number -Response (Invoke-VectorCommand -Command "r_i_q")
        $rows.Add([pscustomobject]@{
            time_s = [math]::Round($stopwatch.Elapsed.TotalSeconds, 4)
            mode = [int]$mode
            error = [int]$errorCode
            bus_voltage_v = $busVoltage
            bus_current_a = $busCurrent
            speed_rad_s = $speed
            id_a = $id
            iq_a = $iq
        })

        if ($busVoltage -lt $MinimumBusVoltage) {
            $terminalReason = "bus_undervoltage_guard"
            Add-Event -Message "$terminalReason vbus=$busVoltage"
            break
        }
        if ($errorCode -ne 0) {
            $terminalReason = "firmware_fault_$([int]$errorCode)"
            Add-Event -Message $terminalReason
            break
        }
        if ($mode -eq 0) {
            $terminalReason = "mode13_completed"
            Add-Event -Message $terminalReason
            break
        }
        Start-Sleep -Milliseconds 40
    }

    $summary = [ordered]@{
        terminal_reason = $terminalReason
        elapsed_s = [math]::Round($stopwatch.Elapsed.TotalSeconds, 4)
        sample_count = $rows.Count
        final_mode = if ($rows.Count -gt 0) { $rows[$rows.Count - 1].mode } else { $null }
        final_error = if ($rows.Count -gt 0) { $rows[$rows.Count - 1].error } else { $null }
        minimum_bus_voltage_v = if ($rows.Count -gt 0) { ($rows.bus_voltage_v | Measure-Object -Minimum).Minimum } else { $null }
        maximum_bus_current_a = if ($rows.Count -gt 0) { ($rows.bus_current_a | Measure-Object -Maximum).Maximum } else { $null }
        maximum_speed_rad_s = if ($rows.Count -gt 0) { ($rows.speed_rad_s | Measure-Object -Maximum).Maximum } else { $null }
    }
    $summary | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $outputPath "mode13_summary.json") -Encoding UTF8
    $rows | Export-Csv -LiteralPath (Join-Path $outputPath "mode13_usb.csv") -NoTypeInformation -Encoding UTF8
    $eventLog | Set-Content -LiteralPath (Join-Path $outputPath "mode13_events.log") -Encoding UTF8
    Write-Output ($summary | ConvertTo-Json -Compress)
} finally {
    if ($serial.IsOpen) {
        try {
            $stopResponse = Invoke-VectorCommand -Command "w_mod=0"
            Add-Event -Message "stop_command $stopResponse"
        } catch {
            Write-Warning "Emergency Mode 0 command failed: $($_.Exception.Message)"
        }
        $serial.Close()
    }
    $serial.Dispose()
    if ($null -ne $rttProcess -and -not $rttProcess.HasExited) {
        Stop-Process -Id $rttProcess.Id
        $rttProcess.WaitForExit()
    }
}
