param(
    [int]$Channel = 0,
    [int]$Node = 0,
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [double]$MaximumRunSeconds = 300.0,
    [double]$MinimumBusVoltage = 22.0,
    [double]$MaximumBusVoltage = 31.0,
    [double]$MaximumPhaseCurrent = 8.0,
    [double]$MaximumTemperature = 80.0,
    [double]$CommunicationRecoverySeconds = 8.0,
    [string]$ControlCanDll = "C:/Program Files (x86)/USB_CAN TOOL/ControlCAN.dll"
)

$ErrorActionPreference = "Stop"

if ($Channel -lt 0 -or $Channel -gt 1) {
    throw "Channel must be 0 or 1."
}
if ($Node -lt 0 -or $Node -gt 7) {
    throw "Node must be in [0, 7]."
}
if (!(Test-Path -LiteralPath $ControlCanDll)) {
    throw "ControlCAN.dll was not found: $ControlCanDll"
}

$absoluteOutputDirectory = [IO.Path]::GetFullPath($OutputDirectory)
if ([Environment]::Is64BitProcess) {
    $powershell32 = "$env:WINDIR/SysWOW64/WindowsPowerShell/v1.0/powershell.exe"
    & $powershell32 -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath `
        -Channel $Channel -Node $Node -OutputDirectory $absoluteOutputDirectory `
        -MaximumRunSeconds $MaximumRunSeconds `
        -MinimumBusVoltage $MinimumBusVoltage `
        -MaximumBusVoltage $MaximumBusVoltage `
        -MaximumPhaseCurrent $MaximumPhaseCurrent `
        -MaximumTemperature $MaximumTemperature `
        -CommunicationRecoverySeconds $CommunicationRecoverySeconds `
        -ControlCanDll $ControlCanDll
    exit $LASTEXITCODE
}

$escapedDll = $ControlCanDll.Replace('\', '\\')
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;

public static class CommissioningCan
{
    [StructLayout(LayoutKind.Sequential)]
    public struct InitConfig
    {
        public UInt32 AccCode, AccMask, Reserved;
        public byte Filter, Timing0, Timing1, Mode;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Frame
    {
        public UInt32 ID, TimeStamp;
        public byte TimeFlag, SendType, RemoteFlag, ExternFlag, DataLen;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst=8)] public byte[] Data;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst=3)] public byte[] Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Status
    {
        public byte ErrInterrupt, RegMode, RegStatus, RegALCapture;
        public byte RegECCapture, RegEWLimit, ReceiveErrorCount, TransmitErrorCount;
        public UInt32 Reserved;
    }

    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_OpenDevice(UInt32 type, UInt32 index, UInt32 reserved);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_CloseDevice(UInt32 type, UInt32 index);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_InitCAN(UInt32 type, UInt32 index, UInt32 channel, ref InitConfig config);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_StartCAN(UInt32 type, UInt32 index, UInt32 channel);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_ResetCAN(UInt32 type, UInt32 index, UInt32 channel);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_ClearBuffer(UInt32 type, UInt32 index, UInt32 channel);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_Transmit(UInt32 type, UInt32 index, UInt32 channel, ref Frame frame, UInt32 count);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_Receive(UInt32 type, UInt32 index, UInt32 channel, [In, Out] ref Frame frame, UInt32 count, Int32 waitMs);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_ReadCANStatus(UInt32 type, UInt32 index, UInt32 channel, ref Status status);
}
"@

$samples = New-Object System.Collections.ArrayList
$events = New-Object System.Collections.ArrayList
$stopwatch = New-Object System.Diagnostics.Stopwatch
$terminalReason = "not_started"
$workflowStarted = $false
$success = $false
$lastStage = -1
$lastHeartbeatSecond = -1
$lastDetailedSecond = -1
$lastBusCurrent = 0.0
$lastPhaseA = 0.0
$lastPhaseB = 0.0
$lastPhaseC = 0.0
$lastSpeed = 0.0
$lastPosition = 0.0
$lastResistanceSpread = 0.0
$lastResistanceDesignError = 0.0
$deviceOpened = $false
$channelStarted = $false
$preflight = [ordered]@{}

function New-CanFrame([uint32]$Identifier, [float]$Value = 0.0) {
    $frame = New-Object CommissioningCan+Frame
    $frame.ID = $Identifier
    $frame.DataLen = 4
    $frame.Data = New-Object byte[] 8
    $frame.Reserved = New-Object byte[] 3
    $bytes = [BitConverter]::GetBytes($Value)
    $frame.Data[0] = $bytes[3]
    $frame.Data[1] = $bytes[2]
    $frame.Data[2] = $bytes[1]
    $frame.Data[3] = $bytes[0]
    return $frame
}

function ConvertFrom-BigEndianFloat([byte[]]$Data) {
    return [BitConverter]::ToSingle(
        [byte[]]@($Data[3], $Data[2], $Data[1], $Data[0]), 0)
}

function Send-Value([byte]$Parameter, [float]$Value) {
    $identifier = ([uint32]$Node -shl 8) -bor $Parameter
    $frame = New-CanFrame $identifier $Value
    if ([CommissioningCan]::VCI_Transmit(
        4, 0, $Channel, [ref]$frame, 1) -ne 1) {
        throw ("CAN transmit failed for ID 0x{0:X3}." -f $identifier)
    }
}

function Read-Value([byte]$Parameter, [int]$TimeoutMs = 300) {
    $wanted = ([uint32]$Node -shl 8) -bor $Parameter
    for ($attempt = 0; $attempt -lt 2; ++$attempt) {
        Send-Value $Parameter 0.0
        $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)
        while ([DateTime]::UtcNow -lt $deadline) {
            $response = New-CanFrame 0
            if ([CommissioningCan]::VCI_Receive(
                4, 0, $Channel, [ref]$response, 1, 20) -eq 1 -and
                $response.ID -eq $wanted -and $response.DataLen -eq 4) {
                $value = ConvertFrom-BigEndianFloat $response.Data
                Start-Sleep -Milliseconds 15
                return $value
            }
        }
        Start-Sleep -Milliseconds 30
    }
    throw ("No CAN response for ID 0x{0:X3} after two attempts." -f $wanted)
}

function Add-Event([string]$Message) {
    $line = "{0:F3}s {1}" -f $stopwatch.Elapsed.TotalSeconds, $Message
    [void]$events.Add($line)
    Write-Output $line
}

function Write-RunArtifacts([string]$OutputPath) {
    $samples | Export-Csv -LiteralPath (Join-Path $OutputPath "commissioning_can_samples.csv") -NoTypeInformation -Encoding UTF8
    $events | Set-Content -LiteralPath (Join-Path $OutputPath "commissioning_can_events.log") -Encoding UTF8
    $status = New-Object CommissioningCan+Status
    $statusRead = if ($deviceOpened -and $channelStarted) {
        [CommissioningCan]::VCI_ReadCANStatus(4, 0, $Channel, [ref]$status)
    } else { 0 }
    [ordered]@{
        captured_at = [DateTimeOffset]::Now.ToString("o")
        channel = $Channel
        node = $Node
        success = $success
        terminal_reason = $terminalReason
        elapsed_seconds = [math]::Round($stopwatch.Elapsed.TotalSeconds, 3)
        sample_count = $samples.Count
        minimum_bus_voltage_v = $MinimumBusVoltage
        maximum_bus_voltage_v = $MaximumBusVoltage
        maximum_phase_current_a = $MaximumPhaseCurrent
        maximum_temperature_c = $MaximumTemperature
        can_status_read_ok = $statusRead -eq 1
        can_receive_error_count = $status.ReceiveErrorCount
        can_transmit_error_count = $status.TransmitErrorCount
        preflight = $preflight
    } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $OutputPath "commissioning_can_summary.json") -Encoding UTF8
}

[void][IO.Directory]::CreateDirectory($absoluteOutputDirectory)

try {
    if ([CommissioningCan]::VCI_OpenDevice(4, 0, 0) -ne 1) {
        throw "CANalyst-II could not be opened. Close USB_CAN_Tool/CANPro first."
    }
    $deviceOpened = $true

    $config = New-Object CommissioningCan+InitConfig
    $config.AccMask = [uint32]::MaxValue
    $config.Filter = 1
    $config.Timing0 = 0x00
    $config.Timing1 = 0x14
    $config.Mode = 0
    if ([CommissioningCan]::VCI_InitCAN(4, 0, $Channel, [ref]$config) -ne 1 -or
        [CommissioningCan]::VCI_StartCAN(4, 0, $Channel) -ne 1) {
        throw "CANalyst-II initialization failed at 1 Mbit/s."
    }
    $channelStarted = $true
    [void][CommissioningCan]::VCI_ClearBuffer(4, 0, $Channel)

    $preflight["mode"] = Read-Value 0x01
    $preflight["error"] = Read-Value 0x4D
    $preflight["encoder_online"] = Read-Value 0x0D
    $preflight["bus_voltage_v"] = Read-Value 0x2D
    $preflight["bus_current_a"] = Read-Value 0x2F
    $preflight["phase_a_current_a"] = Read-Value 0x31
    $preflight["phase_b_current_a"] = Read-Value 0x33
    $preflight["phase_c_current_a"] = Read-Value 0x35
    $preflight["temperature_c"] = Read-Value 0x43
    $preflight["current_limit_a"] = Read-Value 0x11
    $preflight["calibration_current_a"] = Read-Value 0x0F
    $preflight["commissioning_stage"] = Read-Value 0x63
    $preflight["commissioning_progress_pct"] = Read-Value 0x64
    $preflight["captured_at"] = [DateTimeOffset]::Now.ToString("o")

    if ($preflight["mode"] -ne 0.0 -or $preflight["error"] -ne 0.0) {
        throw "Preflight requires mode=0 and error=0."
    }
    if ($preflight["encoder_online"] -ne 1.0) {
        throw "Preflight requires the encoder to be online."
    }
    if ($preflight["bus_voltage_v"] -lt $MinimumBusVoltage -or
        $preflight["bus_voltage_v"] -gt $MaximumBusVoltage) {
        throw "Preflight bus voltage is outside the guarded range."
    }
    if ($preflight["temperature_c"] -gt $MaximumTemperature) {
        throw "Preflight temperature exceeds the guarded maximum."
    }
    $preflight | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $absoluteOutputDirectory "commissioning_can_preflight.json") -Encoding UTF8

    $stopwatch.Start()
    Add-Event ("preflight_ok vbus={0}V temp={1}C" -f $preflight["bus_voltage_v"], $preflight["temperature_c"])
    Send-Value 0x00 21.0
    $workflowStarted = $true
    $terminalReason = "maximum_run_time"
    Add-Event "mode21_command_sent"
    Start-Sleep -Milliseconds 20

    while ($stopwatch.Elapsed.TotalSeconds -lt $MaximumRunSeconds) {
        try {
            $mode = Read-Value 0x01
            $errorCode = Read-Value 0x4D
            $stage = Read-Value 0x63
            $progress = Read-Value 0x64
            $busVoltage = Read-Value 0x2D
            $temperature = Read-Value 0x43
        }
        catch {
            $initialCommunicationError = $_.Exception.Message
            Add-Event ("communication_gap_begin " + $initialCommunicationError)
            $recoveryWatch = [System.Diagnostics.Stopwatch]::StartNew()
            $recovered = $false
            while ($recoveryWatch.Elapsed.TotalSeconds -lt $CommunicationRecoverySeconds) {
                try {
                    # These requests also keep the CAN command watchdog alive.
                    # Avoid detailed telemetry while a service transition owns
                    # most of the foreground execution budget.
                    $mode = Read-Value 0x01 200
                    $errorCode = Read-Value 0x4D 200
                    $stage = Read-Value 0x63 200
                    $progress = Read-Value 0x64 200
                    $busVoltage = Read-Value 0x2D 200
                    $temperature = Read-Value 0x43 200
                    $recovered = $true
                    break
                }
                catch {
                    Start-Sleep -Milliseconds 100
                }
            }
            $recoveryWatch.Stop()
            if (-not $recovered) {
                throw ("CAN response did not recover within {0:F1}s after: {1}" -f `
                    $CommunicationRecoverySeconds, $initialCommunicationError)
            }
            Add-Event ("communication_gap_end duration={0:F3}s stage={1} mode={2}" -f `
                $recoveryWatch.Elapsed.TotalSeconds, [int]$stage, [int]$mode)
        }
        $elapsed = $stopwatch.Elapsed.TotalSeconds

        # Core state is sampled at about 1 Hz. Detailed motion/electrical
        # telemetry is sampled every five seconds to avoid stressing the
        # firmware's single-response CAN queue during real-time calibration.
        $detailedSecond = [int][math]::Floor($elapsed / 5.0)
        if ($detailedSecond -ne $lastDetailedSecond) {
            try {
                $lastBusCurrent = Read-Value 0x2F
                $lastPhaseA = Read-Value 0x31
                $lastPhaseB = Read-Value 0x33
                $lastPhaseC = Read-Value 0x35
                $lastSpeed = Read-Value 0x3F
                $lastPosition = Read-Value 0x41
                $lastResistanceSpread = Read-Value 0x65
                $lastResistanceDesignError = Read-Value 0x66
                $lastDetailedSecond = $detailedSecond
            }
            catch {
                # Detailed telemetry must never terminate commissioning. The
                # next loop's guarded core read performs bounded recovery and
                # still enforces all host-side safety limits.
                Add-Event ("detailed_telemetry_skipped " + $_.Exception.Message)
            }
        }

        [void]$samples.Add([pscustomobject]@{
            time_s = [math]::Round($elapsed, 4)
            mode = [int]$mode
            error = [int]$errorCode
            commissioning_stage = [int]$stage
            commissioning_progress_pct = $progress
            bus_voltage_v = $busVoltage
            bus_current_a = $lastBusCurrent
            phase_a_current_a = $lastPhaseA
            phase_b_current_a = $lastPhaseB
            phase_c_current_a = $lastPhaseC
            temperature_c = $temperature
            speed_rad_s = $lastSpeed
            position_rad = $lastPosition
            phase_resistance_spread_pct = $lastResistanceSpread
            phase_resistance_design_error_pct = $lastResistanceDesignError
        })

        if ([int]$stage -ne $lastStage) {
            Add-Event ("stage_change stage={0} progress={1}% mode={2}" -f [int]$stage, $progress, [int]$mode)
            $lastStage = [int]$stage
        }
        $heartbeatSecond = [int][math]::Floor($elapsed / 5.0)
        if ($heartbeatSecond -ne $lastHeartbeatSecond) {
            Add-Event ("status stage={0} progress={1}% mode={2} err={3} vbus={4}V ibus={5}A speed={6}rad/s temp={7}C" -f [int]$stage, $progress, [int]$mode, [int]$errorCode, $busVoltage, $lastBusCurrent, $lastSpeed, $temperature)
            $lastHeartbeatSecond = $heartbeatSecond
        }

        if (([int]$stage -band 0x80) -ne 0) {
            $terminalReason = "workflow_failed_stage_$([int]$stage -band 0x7F)_fault_$([int]$errorCode)"
            Add-Event $terminalReason
            break
        }
        if ([int]$stage -eq 9 -and $progress -ge 100.0) {
            $terminalReason = "workflow_complete"
            $success = $true
            Add-Event $terminalReason
            break
        }
        if ($errorCode -ne 0.0) {
            $terminalReason = "firmware_fault_$([int]$errorCode)"
            Add-Event $terminalReason
            break
        }
        if ($busVoltage -lt $MinimumBusVoltage -or $busVoltage -gt $MaximumBusVoltage) {
            $terminalReason = "host_bus_voltage_guard"
            Add-Event ("$terminalReason vbus=$busVoltage")
            break
        }
        $peakPhaseCurrent = [math]::Max([math]::Abs($lastPhaseA), [math]::Max([math]::Abs($lastPhaseB), [math]::Abs($lastPhaseC)))
        if ($peakPhaseCurrent -gt $MaximumPhaseCurrent) {
            $terminalReason = "host_overcurrent_guard"
            Add-Event ("$terminalReason peak_phase_current=$peakPhaseCurrent")
            break
        }
        if ($temperature -gt $MaximumTemperature) {
            $terminalReason = "host_overtemperature_guard"
            Add-Event ("$terminalReason temperature=$temperature")
            break
        }
        Start-Sleep -Milliseconds 750
    }

    if (-not $success) {
        Send-Value 0x00 0.0
        Add-Event "safety_stop_sent"
    }
}
catch {
    $terminalReason = "exception: $($_.Exception.Message)"
    Add-Event $terminalReason
    if ($workflowStarted -and $deviceOpened -and $channelStarted) {
        try {
            Send-Value 0x00 0.0
            Add-Event "exception_safety_stop_sent"
        }
        catch {
            Add-Event ("exception_safety_stop_failed " + $_.Exception.Message)
        }
    }
}
finally {
    Write-RunArtifacts $absoluteOutputDirectory
    if ($channelStarted) {
        [void][CommissioningCan]::VCI_ResetCAN(4, 0, $Channel)
    }
    if ($deviceOpened) {
        [void][CommissioningCan]::VCI_CloseDevice(4, 0)
    }
}

if (-not $success) {
    Write-Error "Unified commissioning did not complete: $terminalReason"
    exit 2
}

Write-Output "Unified commissioning completed successfully. Artifacts: $absoluteOutputDirectory"
