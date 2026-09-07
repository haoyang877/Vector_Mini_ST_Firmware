param(
    [int]$Channel = 0,
    [int]$Node = 0,
    [double]$TargetRps = 0.10,
    [double]$DirectionDurationSeconds = 8.0,
    [double]$SettleSeconds = 2.0,
    [double]$CurrentLimitA = 4.5,
    [double]$MaximumPhaseCurrentA = 5.0,
    [int]$InterRequestDelayMilliseconds = 10,
    [switch]$ResetTargetAtStart,
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [string]$ControlCanDll = "C:/Program Files (x86)/USB_CAN TOOL/ControlCAN.dll"
)

$ErrorActionPreference = "Stop"
$absoluteOutputDirectory = [IO.Path]::GetFullPath($OutputDirectory)
if ([Environment]::Is64BitProcess) {
    $powershell32 = "$env:WINDIR/SysWOW64/WindowsPowerShell/v1.0/powershell.exe"
    $forwardArguments = @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath,
        '-Channel', $Channel, '-Node', $Node, '-TargetRps', $TargetRps,
        '-DirectionDurationSeconds', $DirectionDurationSeconds,
        '-SettleSeconds', $SettleSeconds, '-CurrentLimitA', $CurrentLimitA,
        '-MaximumPhaseCurrentA', $MaximumPhaseCurrentA,
        '-InterRequestDelayMilliseconds', $InterRequestDelayMilliseconds,
        '-OutputDirectory', $absoluteOutputDirectory,
        '-ControlCanDll', $ControlCanDll
    )
    if ($ResetTargetAtStart.IsPresent) {
        $forwardArguments += '-ResetTargetAtStart'
    }
    & $powershell32 @forwardArguments
    exit $LASTEXITCODE
}

$escapedDll = $ControlCanDll.Replace('\', '\\')
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class SpeedLoopCan {
    [StructLayout(LayoutKind.Sequential)]
    public struct InitConfig {
        public UInt32 AccCode, AccMask, Reserved;
        public byte Filter, Timing0, Timing1, Mode;
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct Frame {
        public UInt32 ID, TimeStamp;
        public byte TimeFlag, SendType, RemoteFlag, ExternFlag, DataLen;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst=8)] public byte[] Data;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst=3)] public byte[] Reserved;
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct Status {
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

function New-Frame([uint32]$Identifier, [float]$Value) {
    $frame = New-Object SpeedLoopCan+Frame
    $frame.ID = $Identifier
    $frame.DataLen = 4
    $frame.Data = New-Object byte[] 8
    $frame.Reserved = New-Object byte[] 3
    $bytes = [BitConverter]::GetBytes($Value)
    $frame.Data[0] = $bytes[3]; $frame.Data[1] = $bytes[2]
    $frame.Data[2] = $bytes[1]; $frame.Data[3] = $bytes[0]
    return $frame
}

function Send-Value([byte]$Parameter, [float]$Value) {
    $identifier = ([uint32]$Node -shl 8) -bor $Parameter
    $frame = New-Frame $identifier $Value
    if ([SpeedLoopCan]::VCI_Transmit(4, 0, $Channel, [ref]$frame, 1) -ne 1) {
        throw ("CAN transmit failed for 0x{0:X3}." -f $identifier)
    }
}

function Read-Value([byte]$Parameter) {
    $identifier = ([uint32]$Node -shl 8) -bor $Parameter
    for ($attempt = 1; $attempt -le 2; ++$attempt) {
        Send-Value $Parameter 0.0
        $deadline = [DateTime]::UtcNow.AddMilliseconds(500)
        while ($deadline -gt [DateTime]::UtcNow) {
            $response = New-Frame 0 0.0
            if ([SpeedLoopCan]::VCI_Receive(4, 0, $Channel, [ref]$response, 1, 30) -eq 1 -and
                $response.ID -eq $identifier -and $response.DataLen -eq 4) {
                $value = [BitConverter]::ToSingle(
                    [byte[]]@($response.Data[3], $response.Data[2],
                        $response.Data[1], $response.Data[0]), 0)
                if ($InterRequestDelayMilliseconds -gt 0) {
                    Start-Sleep -Milliseconds $InterRequestDelayMilliseconds
                }
                return $value
            }
        }
        Start-Sleep -Milliseconds 25
    }
    throw ("No CAN response for 0x{0:X3}." -f $identifier)
}

function Set-ModeAndConfirm([int]$ExpectedMode) {
    $observedMode = -1
    for ($attempt = 1; $attempt -le 3; ++$attempt) {
        Send-Value 0x00 ([float]$ExpectedMode)
        Start-Sleep -Milliseconds 250
        $observedMode = [int](Read-Value 0x01)
        if ($observedMode -eq $ExpectedMode) {
            return
        }
    }
    $observedError = [int](Read-Value 0x4D)
    throw "Mode transition failed: expected=$ExpectedMode observed=$observedMode error=$observedError."
}

function Capture-Direction([string]$Direction, [double]$CommandRps) {
    # Start each direction with a disabled control state.  A direct reversal
    # leaves the speed PI integral from the previous loaded direction in place,
    # so the second half would measure integrator unwind instead of a clean
    # negative-direction response.
    Set-ModeAndConfirm 0
    Start-Sleep -Milliseconds 200
    $disabledMode = [int](Read-Value 0x01)
    $disabledError = [int](Read-Value 0x4D)
    if ($disabledMode -ne 0 -or $disabledError -ne 0) {
        throw "Direction preflight failed: mode=$disabledMode, error=$disabledError."
    }
    Set-ModeAndConfirm 2
    Send-Value 0x04 ([float]$CommandRps)
    # The firmware exposes a single command mailbox. Give the supervisor one
    # cycle to consume the write before telemetry reads start arriving.
    Start-Sleep -Milliseconds 80
    $watch = [Diagnostics.Stopwatch]::StartNew()
    while ($watch.Elapsed.TotalSeconds -lt $DirectionDurationSeconds) {
        $sample = [pscustomobject]@{
            direction = $Direction
            time_s = [math]::Round($totalWatch.Elapsed.TotalSeconds, 4)
            direction_time_s = [math]::Round($watch.Elapsed.TotalSeconds, 4)
            command_rps = $CommandRps
            mode = [int](Read-Value 0x01)
            error = [int](Read-Value 0x4D)
            speed_rad_s = Read-Value 0x3F
            bus_voltage_v = Read-Value 0x2D
            bus_current_a = Read-Value 0x2F
            phase_a_current_a = Read-Value 0x31
            phase_b_current_a = Read-Value 0x33
            phase_c_current_a = Read-Value 0x35
            temperature_c = Read-Value 0x43
        }
        [void]$samples.Add($sample)
        if ($sample.mode -ne 2 -or $sample.error -ne 0) {
            throw "Closed-loop state violation: mode=$($sample.mode), error=$($sample.error)."
        }
        if ($sample.bus_voltage_v -lt 22.0 -or $sample.bus_voltage_v -gt 31.0) {
            throw "Bus voltage guard tripped: $($sample.bus_voltage_v) V."
        }
        $peak = [math]::Max([math]::Abs($sample.phase_a_current_a),
            [math]::Max([math]::Abs($sample.phase_b_current_a),
                [math]::Abs($sample.phase_c_current_a)))
        if ($peak -gt $MaximumPhaseCurrentA) {
            throw "Phase-current guard tripped: $peak A."
        }
        if ($sample.temperature_c -gt 80.0) {
            throw "Temperature guard tripped: $($sample.temperature_c) C."
        }
        Start-Sleep -Milliseconds 150
    }
}

[void][IO.Directory]::CreateDirectory($absoluteOutputDirectory)
$samples = New-Object System.Collections.ArrayList
$totalWatch = [Diagnostics.Stopwatch]::StartNew()
$opened = $false
$started = $false
$success = $false
$terminalReason = "not_started"
$originalCurrentLimit = $null

try {
    if ([SpeedLoopCan]::VCI_OpenDevice(4, 0, 0) -ne 1) {
        throw "CANalyst-II could not be opened."
    }
    $opened = $true
    $config = New-Object SpeedLoopCan+InitConfig
    $config.AccMask = [uint32]::MaxValue; $config.Filter = 1
    $config.Timing0 = 0x00; $config.Timing1 = 0x14; $config.Mode = 0
    if ([SpeedLoopCan]::VCI_InitCAN(4, 0, $Channel, [ref]$config) -ne 1 -or
        [SpeedLoopCan]::VCI_StartCAN(4, 0, $Channel) -ne 1) {
        throw "CAN initialization failed."
    }
    $started = $true
    [void][SpeedLoopCan]::VCI_ClearBuffer(4, 0, $Channel)
    if ($ResetTargetAtStart) {
        $startupJlinkExe = 'C:\Program Files\SEGGER\JLink_V964\JLink.exe'
        $startupResetCommand = Join-Path $PSScriptRoot 'jlink_reset_run.jlink'
        & $startupJlinkExe -NoGui 1 -CommanderScript $startupResetCommand | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'J-Link startup reset failed.' }
        Start-Sleep -Milliseconds 700
        [void][SpeedLoopCan]::VCI_ClearBuffer(4, 0, $Channel)
    }
    $preflight = [ordered]@{
        mode = Read-Value 0x01; error = Read-Value 0x4D
        encoder_online = Read-Value 0x0D; friction_model_valid = Read-Value 0x62
        bus_voltage_v = Read-Value 0x2D; temperature_c = Read-Value 0x43
        current_limit_a = Read-Value 0x11
    }
    if ($preflight.mode -ne 0 -or $preflight.error -ne 0 -or
        $preflight.encoder_online -ne 1 -or $preflight.friction_model_valid -ne 1) {
        throw "Closed-loop preflight failed."
    }
    # Protocol mode 0 represents both STANDBY and FAULTED.  An idempotent clear
    # request guarantees the lifecycle is in STANDBY after a prior watchdog
    # trip, while a healthy standby target simply rejects it without mutation.
    Send-Value 0x00 10.0
    Start-Sleep -Milliseconds 150
    if ([int](Read-Value 0x01) -ne 0 -or [int](Read-Value 0x4D) -ne 0) {
        throw "Lifecycle recovery preflight failed."
    }
    $originalCurrentLimit = [double]$preflight.current_limit_a
    Send-Value 0x10 ([float]$CurrentLimitA)
    Start-Sleep -Milliseconds 100
    $appliedCurrentLimit = Read-Value 0x11
    if ([math]::Abs($appliedCurrentLimit - $CurrentLimitA) -gt 0.01) {
        throw "Current-limit staging failed: requested=$CurrentLimitA applied=$appliedCurrentLimit."
    }

    Capture-Direction "positive" $TargetRps
    Capture-Direction "negative" (-$TargetRps)
    Set-ModeAndConfirm 0
    Start-Sleep -Milliseconds 300

    $positive = @($samples | Where-Object {
        $_.direction -eq "positive" -and $_.direction_time_s -ge $SettleSeconds })
    $negative = @($samples | Where-Object {
        $_.direction -eq "negative" -and $_.direction_time_s -ge $SettleSeconds })
    $targetRadS = $TargetRps * 2.0 * [math]::PI
    $positiveMean = ($positive | Measure-Object speed_rad_s -Average).Average
    $negativeMean = ($negative | Measure-Object speed_rad_s -Average).Average
    $allowedError = [math]::Max(0.20, [math]::Abs($targetRadS) * 0.35)
    if ($positive.Count -eq 0 -or $negative.Count -eq 0 -or
        [math]::Abs($positiveMean - $targetRadS) -gt $allowedError -or
        [math]::Abs($negativeMean + $targetRadS) -gt $allowedError) {
        throw "Steady-state speed tracking did not meet tolerance."
    }
    $finalMode = Read-Value 0x01
    $finalError = Read-Value 0x4D
    if ($finalMode -ne 0 -or $finalError -ne 0) {
        throw "Final disabled-state check failed."
    }
    $success = $true
    $terminalReason = "completed"
}
catch {
    $terminalReason = $_.Exception.Message
}
finally {
    if ($opened -and $started) {
        try {
            Send-Value 0x00 0.0
            Start-Sleep -Milliseconds 100
            if ($null -ne $originalCurrentLimit) {
                Send-Value 0x10 ([float]$originalCurrentLimit)
                Start-Sleep -Milliseconds 100
            }
        } catch { }
    }
    $samples | Export-Csv -LiteralPath (Join-Path $absoluteOutputDirectory "speed_closed_loop_samples.csv") -NoTypeInformation -Encoding UTF8
    $status = New-Object SpeedLoopCan+Status
    $statusOk = if ($opened -and $started) {
        [SpeedLoopCan]::VCI_ReadCANStatus(4, 0, $Channel, [ref]$status)
    } else { 0 }
    $positive = @($samples | Where-Object {
        $_.direction -eq "positive" -and $_.direction_time_s -ge $SettleSeconds })
    $negative = @($samples | Where-Object {
        $_.direction -eq "negative" -and $_.direction_time_s -ge $SettleSeconds })
    [ordered]@{
        captured_at = [DateTimeOffset]::Now.ToString("o")
        success = $success; terminal_reason = $terminalReason
        target_rps = $TargetRps; target_rad_s = $TargetRps * 2.0 * [math]::PI
        requested_current_limit_a = $CurrentLimitA
        original_current_limit_a = $originalCurrentLimit
        positive_mean_speed_rad_s = ($positive | Measure-Object speed_rad_s -Average).Average
        negative_mean_speed_rad_s = ($negative | Measure-Object speed_rad_s -Average).Average
        sample_count = $samples.Count
        maximum_abs_phase_current_a = ($samples | ForEach-Object {
            [math]::Max([math]::Abs($_.phase_a_current_a),
                [math]::Max([math]::Abs($_.phase_b_current_a), [math]::Abs($_.phase_c_current_a)))
        } | Measure-Object -Maximum).Maximum
        maximum_bus_current_a = ($samples | Measure-Object bus_current_a -Maximum).Maximum
        minimum_bus_voltage_v = ($samples | Measure-Object bus_voltage_v -Minimum).Minimum
        maximum_temperature_c = ($samples | Measure-Object temperature_c -Maximum).Maximum
        can_status_read_ok = $statusOk -eq 1
        can_receive_error_count = $status.ReceiveErrorCount
        can_transmit_error_count = $status.TransmitErrorCount
    } | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $absoluteOutputDirectory "speed_closed_loop_summary.json") -Encoding UTF8
    if ($started) { [void][SpeedLoopCan]::VCI_ResetCAN(4, 0, $Channel) }
    if ($opened) { [void][SpeedLoopCan]::VCI_CloseDevice(4, 0) }
}

if (-not $success) {
    Write-Error "Speed closed-loop validation failed: $terminalReason"
    exit 2
}
Write-Output "Speed closed-loop validation passed: $absoluteOutputDirectory"
