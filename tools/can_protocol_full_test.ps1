param(
    [int]$Channel = 0,
    [string]$OutputDirectory = "validation/can_protocol_2026-09-05",
    [string]$ControlCanDll = "C:/Program Files (x86)/USB_CAN TOOL/ControlCAN.dll"
)

$ErrorActionPreference = 'Stop'

if ([Environment]::Is64BitProcess) {
    $powershell32 = "$env:WINDIR/SysWOW64/WindowsPowerShell/v1.0/powershell.exe"
    & $powershell32 -NoProfile -File $PSCommandPath -Channel $Channel `
        -OutputDirectory $OutputDirectory -ControlCanDll $ControlCanDll
    exit $LASTEXITCODE
}

$outputPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../$OutputDirectory"))
[void][IO.Directory]::CreateDirectory($outputPath)
$escapedDll = $ControlCanDll.Replace('\', '\\')

Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class FullCanTest {
    [StructLayout(LayoutKind.Sequential)] public struct InitConfig {
        public UInt32 AccCode, AccMask, Reserved;
        public byte Filter, Timing0, Timing1, Mode;
    }
    [StructLayout(LayoutKind.Sequential)] public struct Frame {
        public UInt32 ID, TimeStamp;
        public byte TimeFlag, SendType, RemoteFlag, ExternFlag, DataLen;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst=8)] public byte[] Data;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst=3)] public byte[] Reserved;
    }
    [StructLayout(LayoutKind.Sequential)] public struct Status {
        public byte ErrInterrupt, RegMode, RegStatus, RegALCapture;
        public byte RegECCapture, RegEWLimit, ReceiveErrorCount, TransmitErrorCount;
        public UInt32 Reserved;
    }
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_OpenDevice(UInt32 t, UInt32 i, UInt32 r);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_CloseDevice(UInt32 t, UInt32 i);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_InitCAN(UInt32 t, UInt32 i, UInt32 c, ref InitConfig cfg);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_StartCAN(UInt32 t, UInt32 i, UInt32 c);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_ResetCAN(UInt32 t, UInt32 i, UInt32 c);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_ClearBuffer(UInt32 t, UInt32 i, UInt32 c);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_Transmit(UInt32 t, UInt32 i, UInt32 c, ref Frame f, UInt32 n);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_Receive(UInt32 t, UInt32 i, UInt32 c, [In, Out] ref Frame f, UInt32 n, Int32 wait);
    [DllImport(@"$escapedDll", CallingConvention=CallingConvention.StdCall)]
    public static extern UInt32 VCI_ReadCANStatus(UInt32 t, UInt32 i, UInt32 c, ref Status s);
}
"@

$script:Results = New-Object System.Collections.ArrayList
$script:Node = 0
$script:Bitrate = 1000

function Add-Result($group, $name, $id, $result, $expected, $actual, $detail) {
    [void]$script:Results.Add([pscustomobject]@{
        Group=$group; Command=$name; Id=('{0:X3}' -f $id); Result=$result
        Expected=$expected; Actual=$actual; Detail=$detail
    })
}

function New-Frame([uint32]$id, [float]$value=0.0, [byte]$length=4) {
    $frame = New-Object FullCanTest+Frame
    $frame.ID = $id; $frame.DataLen = $length
    $frame.Data = New-Object byte[] 8; $frame.Reserved = New-Object byte[] 3
    $bytes = [BitConverter]::GetBytes($value)
    $frame.Data[0]=$bytes[3]; $frame.Data[1]=$bytes[2]
    $frame.Data[2]=$bytes[1]; $frame.Data[3]=$bytes[0]
    return $frame
}

function ConvertFrom-BigEndianFloat([byte[]]$data) {
    $bytes = [byte[]]@($data[3],$data[2],$data[1],$data[0])
    return [BitConverter]::ToSingle($bytes, 0)
}

function Start-Channel([int]$kbps) {
    $timings = @{
        1000=@(0x00,0x14); 500=@(0x00,0x1C); 250=@(0x01,0x1C)
        125=@(0x03,0x1C); 100=@(0x04,0x1C); 200=@(0x81,0xFA)
    }
    if (!$timings.ContainsKey($kbps)) { throw "Unsupported adapter bitrate $kbps" }
    [void][FullCanTest]::VCI_ResetCAN(4,0,$Channel)
    $cfg = New-Object FullCanTest+InitConfig
    $cfg.AccMask=[uint32]::MaxValue; $cfg.Filter=1
    $cfg.Timing0=$timings[$kbps][0]; $cfg.Timing1=$timings[$kbps][1]; $cfg.Mode=0
    if ([FullCanTest]::VCI_InitCAN(4,0,$Channel,[ref]$cfg) -ne 1 -or
        [FullCanTest]::VCI_StartCAN(4,0,$Channel) -ne 1) { throw "Adapter init failed at $kbps kbps" }
    [void][FullCanTest]::VCI_ClearBuffer(4,0,$Channel)
    $script:Bitrate=$kbps
}

function Send-Value([byte]$parameter, [float]$value, [byte]$length=4, [int]$node=$script:Node) {
    $frame=New-Frame (([uint32]$node -shl 8) -bor $parameter) $value $length
    if ([FullCanTest]::VCI_Transmit(4,0,$Channel,[ref]$frame,1) -ne 1) { throw ('TX failed: {0:X3}' -f $frame.ID) }
}

function Read-Value([byte]$parameter, [int]$timeoutMs=400, [int]$node=$script:Node) {
    Send-Value $parameter 0.0 4 $node
    $wanted=([uint32]$node -shl 8) -bor $parameter
    $deadline=[DateTime]::UtcNow.AddMilliseconds($timeoutMs)
    while ([DateTime]::UtcNow -lt $deadline) {
        $response=New-Frame 0
        if ([FullCanTest]::VCI_Receive(4,0,$Channel,[ref]$response,1,20) -eq 1 -and
            $response.ID -eq $wanted -and $response.DataLen -eq 4) {
            return ConvertFrom-BigEndianFloat $response.Data
        }
    }
    throw ('No response: {0:X3}' -f $wanted)
}

function Nearly-Equal([float]$a,[float]$b,[float]$rel=0.0001,[float]$abs=0.000001) {
    return [Math]::Abs($a-$b) -le [Math]::Max($abs,$rel*[Math]::Max([Math]::Abs($a),[Math]::Abs($b)))
}

function Test-Read($name,[byte]$id) {
    try { $v=Read-Value $id; Add-Result 'GET' $name $id 'PASS' 'finite float32' $v '' }
    catch { Add-Result 'GET' $name $id 'FAIL' 'response' '' $_.Exception.Message }
}

function Test-ReadWrite($name,[byte]$setId,[byte]$getId,[float]$testValue) {
    $original=$null
    try {
        $original=Read-Value $getId
        Send-Value $setId $testValue; Start-Sleep -Milliseconds 25
        $actual=Read-Value $getId
        if (Nearly-Equal $actual $testValue) { Add-Result 'SET/GET' $name $setId 'PASS' $testValue $actual ('restores '+$original) }
        else { Add-Result 'SET/GET' $name $setId 'FAIL' $testValue $actual 'readback mismatch' }
    } catch { Add-Result 'SET/GET' $name $setId 'FAIL' $testValue '' $_.Exception.Message }
    finally {
        if ($null -ne $original) { Send-Value $setId ([float]$original); Start-Sleep -Milliseconds 20 }
    }
}

function Enter-CurrentModeAtZero {
    $deadline=[DateTime]::UtcNow.AddSeconds(2)
    $requested=$false
    $lastMode=-1
    while ([DateTime]::UtcNow -lt $deadline) {
        if (!$requested) { Send-Value 0x02 0.0; $requested=$true }
        Start-Sleep -Milliseconds 50
        $lastMode=Read-Value 0x01
        if ($lastMode -eq 1) { return [float]$lastMode }
        # A fresh boot first performs action 11 (current-offset calibration),
        # returns to disabled, and then accepts the current-mode request.
        if ($lastMode -eq 0) { $requested=$false }
    }
    return [float]$lastMode
}

function Expect-NoResponse($group,$name,[byte]$id,[float]$value=0.0,[byte]$length=4,[int]$node=$script:Node,[bool]$remote=$false,[bool]$extended=$false) {
    try {
        [void][FullCanTest]::VCI_ClearBuffer(4,0,$Channel)
        $frame=New-Frame (([uint32]$node -shl 8) -bor $id) $value $length
        if ($remote) { $frame.RemoteFlag=1 }
        if ($extended) { $frame.ExternFlag=1 }
        if ([FullCanTest]::VCI_Transmit(4,0,$Channel,[ref]$frame,1) -ne 1) { throw 'TX failed' }
        $deadline=[DateTime]::UtcNow.AddMilliseconds(120); $received=$false
        while ([DateTime]::UtcNow -lt $deadline) {
            $rx=New-Frame 0
            if ([FullCanTest]::VCI_Receive(4,0,$Channel,[ref]$rx,1,20) -eq 1) { $received=$true; break }
        }
        if (!$received) { Add-Result $group $name $frame.ID 'PASS' 'no response' 'no response' '' }
        else { Add-Result $group $name $frame.ID 'FAIL' 'no response' ('response '+('{0:X3}' -f $rx.ID)) '' }
    } catch { Add-Result $group $name $id 'FAIL' 'no response' '' $_.Exception.Message }
}

if (!(Test-Path -LiteralPath $ControlCanDll)) { throw "Missing $ControlCanDll" }
if ([FullCanTest]::VCI_OpenDevice(4,0,0) -ne 1) { throw 'CANalyst-II open failed' }

try {
    Start-Channel 1000
    $mode=Read-Value 0x01
    if ($mode -ne 0.0) { Send-Value 0x00 0.0; Start-Sleep -Milliseconds 100 }

    $reads=@(
        @('GET_MODE',0x01),@('GET_CURRENT_SET',0x03),@('GET_SPEED_SET',0x05),@('GET_POS_SET',0x07),
        @('GET_NODE_ID',0x09),@('GET_POLE_PAIRS',0x0B),@('GET_ENCODER_STATE',0x0D),@('GET_CURRENT_CAL',0x0F),
        @('GET_CURRENT_LIMIT',0x11),@('GET_SPEED_LIMIT',0x13),@('GET_SPEED_ACC',0x15),@('GET_SPEED_DEC',0x17),
        @('GET_SPEED_KP',0x19),@('GET_SPEED_KI',0x1B),@('GET_POS_ACC',0x1D),@('GET_POS_DEC',0x1F),
        @('GET_POS_MAXSPEED',0x21),@('GET_POS_KP',0x23),@('GET_POS_KD',0x25),@('GET_CAN_BR',0x29),
        @('GET_CAN_HB',0x2B),@('GET_VBUS',0x2D),@('GET_IBUS',0x2F),@('GET_IA',0x31),@('GET_IB',0x33),
        @('GET_IC',0x35),@('GET_ID',0x37),@('GET_IQ',0x39),@('GET_SPEED2_FILT',0x3F),@('GET_POS2_FILT',0x41),
        @('GET_TEMP',0x43),@('GET_RS',0x45),@('GET_LD',0x47),@('GET_LQ',0x49),@('GET_FLUX',0x4B),
        @('GET_ERROR',0x4D),@('GET_ENCODER_REVERSE',0x4F),@('GET_POS_KI',0x51),@('GET_POS_INTEGRAL_LIMIT',0x53),
        @('GET_CASCADE_POS_KP',0x55),@('GET_CASCADE_POS_KD',0x57),@('GET_FRICTION_STATE',0x59),
        @('GET_FRICTION_REASON',0x5A),@('GET_FRICTION_COULOMB_POS',0x5B),@('GET_FRICTION_COULOMB_NEG',0x5C),
        @('GET_FRICTION_VISCOUS_POS',0x5D),@('GET_FRICTION_VISCOUS_NEG',0x5E),@('GET_FRICTION_RMSE_POS',0x5F),
        @('GET_FRICTION_RMSE_NEG',0x60),@('GET_FRICTION_CANDIDATE_VALID',0x61),@('GET_FRICTION_MODEL_VALID',0x62)
    )
    foreach($t in $reads) { Test-Read $t[0] ([byte]$t[1]) }

    # Configuration and tuning writes are changed, read back, and restored.
    $rw=@(
        @('POLE_PAIRS',0x0A,0x0B,20.0),@('CURRENT_CAL',0x0E,0x0F,3.5),@('CURRENT_LIMIT',0x10,0x11,5.5),
        @('SPEED_LIMIT_RPS',0x12,0x13,0.4),@('SPEED_ACC_RPS2',0x14,0x15,40.0),@('SPEED_DEC_RPS2',0x16,0x17,40.0),
        @('SPEED_KP',0x18,0x19,0.06),@('SPEED_KI',0x1A,0x1B,0.6),@('POS_ACC_RPS2',0x1C,0x1D,0.1),
        @('POS_DEC_RPS2',0x1E,0x1F,0.1),@('POS_MAX_SPEED_RPS',0x20,0x21,0.1),@('POS_KP',0x22,0x23,7.5),
        @('POS_KD',0x24,0x25,0.4),@('RS_OHM',0x44,0x45,1.8),@('LD_H',0x46,0x47,0.0015),
        @('LQ_H',0x48,0x49,0.0015),@('FLUX_WB',0x4A,0x4B,0.016),@('POS_KI',0x50,0x51,9.0),
        @('POS_INTEGRAL_LIMIT_A',0x52,0x53,4.5),@('CASCADE_POS_KP',0x54,0x55,0.04),@('CASCADE_POS_KD',0x56,0x57,0.4)
    )
    foreach($t in $rw) { Test-ReadWrite $t[0] ([byte]$t[1]) ([byte]$t[2]) ([float]$t[3]) }

    # Encoder reverse is a discrete read/write field.
    $encOriginal=Read-Value 0x4F; $encTest=if($encOriginal -eq 0){1.0}else{0.0}
    Test-ReadWrite 'ENCODER_REVERSE' 0x4E 0x4F $encTest

    # Pole-pair and direction writes deliberately invalidate encoder calibration.
    # Reset reloads the saved calibration before any control-mode or watchdog test.
    $jlinkExe='C:\Program Files\SEGGER\JLink_V964\JLink.exe'
    $resetCommand=Join-Path $PSScriptRoot 'jlink_reset_run.jlink'
    $resetLog=Join-Path $outputPath 'jlink_reset_during_full_test.log'
    & $jlinkExe -USB 602722271 -CommandFile $resetCommand 2>&1 | Out-File -Encoding ascii $resetLog
    if ($LASTEXITCODE -ne 0) { throw 'J-Link reset failed during full CAN test' }
    Start-Sleep -Milliseconds 600
    [void][FullCanTest]::VCI_ClearBuffer(4,0,$Channel)

    # Command references implicitly enter control modes; use zero/same-position and immediately disable.
    try { $m=Enter-CurrentModeAtZero; $v=Read-Value 0x03; Send-Value 0x00 0.0
        Add-Result 'CONTROL' 'SET_CURRENT_ZERO' 0x02 $(if((Nearly-Equal $v 0)-and $m -eq 1){'PASS'}else{'FAIL'}) 'set=0, mode=1' "set=$v mode=$m" 'disabled after test' } catch { Add-Result 'CONTROL' 'SET_CURRENT_ZERO' 0x02 'FAIL' '' '' $_.Exception.Message }
    Start-Sleep -Milliseconds 80
    try { Send-Value 0x04 0.0; Start-Sleep -Milliseconds 50; $v=Read-Value 0x05; $m=Read-Value 0x01; Send-Value 0x00 0.0
        Add-Result 'CONTROL' 'SET_SPEED_ZERO' 0x04 $(if((Nearly-Equal $v 0)-and $m -eq 2){'PASS'}else{'FAIL'}) 'set=0, mode=2' "set=$v mode=$m" 'disabled after test' } catch { Add-Result 'CONTROL' 'SET_SPEED_ZERO' 0x04 'FAIL' '' '' $_.Exception.Message }
    Start-Sleep -Milliseconds 80
    try { $posRad=Read-Value 0x41; $posRev=[float]($posRad/(2*[Math]::PI)); Send-Value 0x06 $posRev; Start-Sleep -Milliseconds 50; $v=Read-Value 0x07; $m=Read-Value 0x01; Send-Value 0x00 0.0
        Add-Result 'CONTROL' 'SET_POSITION_HOLD' 0x06 $(if((Nearly-Equal $v $posRev)-and $m -eq 3){'PASS'}else{'FAIL'}) $posRev "set=$v mode=$m" 'same-position command; disabled after test' } catch { Add-Result 'CONTROL' 'SET_POSITION_HOLD' 0x06 'FAIL' '' '' $_.Exception.Message }
    Start-Sleep -Milliseconds 100
    Send-Value 0x00 0.0; Start-Sleep -Milliseconds 50
    $m=Read-Value 0x01; Add-Result 'CONTROL' 'SET_MODE_DISABLED' 0x00 $(if($m -eq 0){'PASS'}else{'FAIL'}) 0 $m ''

    # Node-ID change also verifies that the old hardware filter is rejected.
    try {
        Send-Value 0x08 1.0; Start-Sleep -Milliseconds 100; $script:Node=1
        $v=Read-Value 0x09
        $oldRejected=$false; try { [void](Read-Value 0x09 120 0) } catch { $oldRejected=$true }
        Add-Result 'CONFIG' 'SET_NODE_ID_1' 0x08 $(if($v -eq 1 -and $oldRejected){'PASS'}else{'FAIL'}) 'new=1, old filtered' "new=$v oldRejected=$oldRejected" ''
        Send-Value 0x08 0.0; Start-Sleep -Milliseconds 100; $script:Node=0; $v=Read-Value 0x09
        Add-Result 'CONFIG' 'RESTORE_NODE_ID_0' 0x08 $(if($v -eq 0){'PASS'}else{'FAIL'}) 0 $v ''
    } catch { Add-Result 'CONFIG' 'NODE_ID_SWITCH' 0x08 'FAIL' '' '' $_.Exception.Message; $script:Node=0 }

    # Real bitrate transition 1000 -> 500 -> 1000 kbit/s.
    try {
        Send-Value 0x28 500.0; Start-Sleep -Milliseconds 180; Start-Channel 500
        $v=Read-Value 0x29
        Add-Result 'CONFIG' 'SET_BITRATE_500' 0x28 $(if($v -eq 500){'PASS'}else{'FAIL'}) 500 $v ''
        Send-Value 0x28 1000.0; Start-Sleep -Milliseconds 180; Start-Channel 1000
        $v=Read-Value 0x29
        Add-Result 'CONFIG' 'RESTORE_BITRATE_1000' 0x28 $(if($v -eq 1000){'PASS'}else{'FAIL'}) 1000 $v ''
    } catch { Add-Result 'CONFIG' 'BITRATE_SWITCH' 0x28 'FAIL' '500 then 1000' '' $_.Exception.Message; try { Start-Channel 1000 } catch {} }

    Test-ReadWrite 'HEARTBEAT_MS' 0x2A 0x2B 600.0

    # Defined but intentionally ignored/placeholder SET commands.
    $ignored=@(@('SET_ENCODER_STATE',0x0C),@('SET_COGGING',0x26),@('SET_VBUS',0x2C),@('SET_IBUS',0x2E),
        @('SET_IA',0x30),@('SET_IB',0x32),@('SET_IC',0x34),@('SET_ID',0x36),@('SET_IQ',0x38),
        @('SET_SPEED2_FILT',0x3E),@('SET_POS2_FILT',0x40),@('SET_TEMP',0x42),@('SET_ERROR',0x4C))
    foreach($t in $ignored) { Expect-NoResponse 'IGNORED' $t[0] ([byte]$t[1]) 0.0 }
    Expect-NoResponse 'NOT_IMPLEMENTED' 'GET_COGGING' 0x27 0.0
    Expect-NoResponse 'GUARDED' 'APPLY_FRICTION_MODEL_VALUE_0' 0x58 0.0

    # Decoder/filter rejection cases.
    Expect-NoResponse 'NEGATIVE' 'WRONG_NODE' 0x01 0.0 4 1
    Expect-NoResponse 'NEGATIVE' 'WRONG_DLC_3' 0x01 0.0 3
    Expect-NoResponse 'NEGATIVE' 'REMOTE_FRAME' 0x01 0.0 4 $script:Node $true
    Expect-NoResponse 'NEGATIVE' 'EXTENDED_FRAME' 0x01 0.0 4 $script:Node $false $true
    Expect-NoResponse 'NEGATIVE' 'UNKNOWN_PARAMETER' 0x7F 0.0
    Send-Value 0x00 ([float]::NaN); Start-Sleep -Milliseconds 50
    $m=Read-Value 0x01; Add-Result 'NEGATIVE' 'NAN_REJECTED' 0x00 $(if($m -eq 0){'PASS'}else{'FAIL'}) 'mode remains 0' $m ''

    # Heartbeat timeout protection: zero-current mode, then silence > 500 ms.
    try {
        Send-Value 0x2A 500.0; Start-Sleep -Milliseconds 30; $enteredMode=Enter-CurrentModeAtZero
        Start-Sleep -Milliseconds 650
        $m=Read-Value 0x01; $e=Read-Value 0x4D
        # The first CAN frame after reconnect clears the communication fault
        # before its GET command is routed, so the observable recovery state is
        # disabled mode with error 0.
        Add-Result 'SAFETY' 'HEARTBEAT_TIMEOUT' 0x2A $(if($enteredMode -eq 1 -and $m -eq 0 -and $e -eq 0){'PASS'}else{'FAIL'}) 'entered=1,mode=0,error cleared on RX' "entered=$enteredMode mode=$m error=$e" ''
        Send-Value 0x00 10.0; Start-Sleep -Milliseconds 100
        Send-Value 0x2A 500.0
    } catch { Add-Result 'SAFETY' 'HEARTBEAT_TIMEOUT' 0x2A 'FAIL' '' '' $_.Exception.Message }

    # Reliability/latency test.
    $latencies=New-Object System.Collections.ArrayList; $ok=0
    for($i=0;$i -lt 200;$i++) { try { $sw=[Diagnostics.Stopwatch]::StartNew(); $v=Read-Value 0x01; $sw.Stop(); [void]$latencies.Add($sw.Elapsed.TotalMilliseconds); if($v -eq 0){$ok++} } catch {} }
    $avg=if($latencies.Count){($latencies|Measure-Object -Average).Average}else{0}; $max=if($latencies.Count){($latencies|Measure-Object -Maximum).Maximum}else{0}
    Add-Result 'STRESS' 'GET_MODE_X200' 0x01 $(if($ok -eq 200){'PASS'}else{'FAIL'}) '200/200' "$ok/200" (('avg={0:F2}ms max={1:F2}ms' -f $avg,$max))

    $status=New-Object FullCanTest+Status; $statusOk=[FullCanTest]::VCI_ReadCANStatus(4,0,$Channel,[ref]$status)
    $busResult=if($statusOk -eq 1 -and $status.ReceiveErrorCount -eq 0 -and $status.TransmitErrorCount -eq 0){'PASS'}else{'FAIL'}
    Add-Result 'BUS' 'ADAPTER_ERROR_COUNTERS' 0 $busResult 'status readable,rx=0,tx=0' "rx=$($status.ReceiveErrorCount) tx=$($status.TransmitErrorCount)" "statusOk=$statusOk"
}
finally {
    try { Send-Value 0x00 0.0 } catch {}
    [void][FullCanTest]::VCI_ResetCAN(4,0,$Channel)
    [void][FullCanTest]::VCI_CloseDevice(4,0)
    $csv=Join-Path $outputPath 'can_protocol_results.csv'
    $json=Join-Path $outputPath 'can_protocol_results.json'
    $script:Results | Export-Csv -NoTypeInformation -Encoding UTF8 -Path $csv
    $summary=[pscustomobject]@{
        timestamp=(Get-Date).ToString('o'); total=$script:Results.Count
        passed=@($script:Results|Where-Object Result -eq 'PASS').Count
        failed=@($script:Results|Where-Object Result -eq 'FAIL').Count
        results=$script:Results
    }
    $summary | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -Path $json
    $summary | Select-Object timestamp,total,passed,failed
    $script:Results | Where-Object Result -eq 'FAIL' | Format-Table -AutoSize
    if ($summary.failed -ne 0) { exit 2 }
}
