param(
    [int]$Channel = 0,
    [int]$RepeatCount = 100,
    [int]$InterRequestDelayMilliseconds = 10,
    [switch]$VerboseFrames,
    [string]$ControlCanDll = "C:/Program Files (x86)/USB_CAN TOOL/ControlCAN.dll"
)

$ErrorActionPreference = 'Stop'

if ($Channel -lt 0 -or $Channel -gt 1) {
    throw "Channel must be 0 or 1."
}
if ($RepeatCount -lt 1 -or $RepeatCount -gt 10000) {
    throw "RepeatCount must be in [1, 10000]."
}
if (!(Test-Path -LiteralPath $ControlCanDll)) {
    throw "ControlCAN.dll was not found: $ControlCanDll"
}

# CANalyst-II's vendor DLL is 32-bit. Relaunch this script under 32-bit
# Windows PowerShell when invoked from the normal 64-bit shell.
if ([Environment]::Is64BitProcess) {
    $powershell32 = "$env:WINDIR/SysWOW64/WindowsPowerShell/v1.0/powershell.exe"
    $forwardArguments = @(
        '-NoProfile', '-File', $PSCommandPath,
        '-Channel', $Channel, '-RepeatCount', $RepeatCount,
        '-InterRequestDelayMilliseconds', $InterRequestDelayMilliseconds,
        '-ControlCanDll', $ControlCanDll
    )
    if ($VerboseFrames.IsPresent) {
        $forwardArguments += '-VerboseFrames'
    }
    & $powershell32 @forwardArguments
    exit $LASTEXITCODE
}

$escapedDll = $ControlCanDll.Replace('\', '\\')
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;

public static class VectorControlCan
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

function New-CanFrame([uint32]$Identifier) {
    $frame = New-Object VectorControlCan+Frame
    $frame.ID = $Identifier
    $frame.DataLen = 4
    $frame.Data = New-Object byte[] 8
    $frame.Reserved = New-Object byte[] 3
    return $frame
}

function ConvertFrom-BigEndianFloat([byte[]]$Data) {
    [uint32]$bits = ([uint32]$Data[0] -shl 24) -bor
        ([uint32]$Data[1] -shl 16) -bor
        ([uint32]$Data[2] -shl 8) -bor [uint32]$Data[3]
    return [BitConverter]::ToSingle([BitConverter]::GetBytes($bits), 0)
}

function Invoke-CanRead([uint32]$Identifier) {
    for ($attempt = 1; $attempt -le 2; ++$attempt) {
        $request = New-CanFrame $Identifier
        if ([VectorControlCan]::VCI_Transmit(4, 0, $Channel, [ref]$request, 1) -ne 1) {
            throw ("Transmit failed for ID 0x{0:X3}." -f $Identifier)
        }
        $deadline = [DateTime]::UtcNow.AddMilliseconds(500)
        while ([DateTime]::UtcNow -lt $deadline) {
            $response = New-CanFrame 0
            $received = [VectorControlCan]::VCI_Receive(
                4, 0, $Channel, [ref]$response, 1, 50)
            if ($received -eq 1 -and $VerboseFrames) {
                Write-Host ("RX id=0x{0:X3} len={1} ext={2} remote={3} data={4}" -f `
                    $response.ID, $response.DataLen, $response.ExternFlag,
                    $response.RemoteFlag,
                    [BitConverter]::ToString($response.Data, 0, $response.DataLen))
            }
            if ($received -eq 1 -and $response.ID -eq $Identifier -and
                $response.DataLen -eq 4) {
                $value = ConvertFrom-BigEndianFloat $response.Data
                if ($InterRequestDelayMilliseconds -gt 0) {
                    Start-Sleep -Milliseconds $InterRequestDelayMilliseconds
                }
                return $value
            }
        }
        Start-Sleep -Milliseconds 25
    }
    throw ("No matching response for ID 0x{0:X3}." -f $Identifier)
}

$opened = [VectorControlCan]::VCI_OpenDevice(4, 0, 0)
if ($opened -ne 1) {
    throw "CANalyst-II could not be opened. Close USB_CAN_Tool/CANPro first."
}

try {
    $config = New-Object VectorControlCan+InitConfig
    $config.AccMask = [uint32]::MaxValue
    $config.Filter = 1
    $config.Timing0 = 0x00
    $config.Timing1 = 0x14
    $config.Mode = 0
    if ([VectorControlCan]::VCI_InitCAN(4, 0, $Channel, [ref]$config) -ne 1 -or
        [VectorControlCan]::VCI_StartCAN(4, 0, $Channel) -ne 1) {
        throw "CANalyst-II channel initialization failed."
    }
    [void][VectorControlCan]::VCI_ClearBuffer(4, 0, $Channel)

    $mode = Invoke-CanRead 0x01
    $errorCode = Invoke-CanRead 0x4D
    $bitrate = Invoke-CanRead 0x29
    $frictionValid = Invoke-CanRead 0x62

    $passed = 0
    for ($index = 0; $index -lt $RepeatCount; ++$index) {
        if ((Invoke-CanRead 0x01) -eq 0.0) {
            ++$passed
        }
    }

    $status = New-Object VectorControlCan+Status
    $statusOk = [VectorControlCan]::VCI_ReadCANStatus(
        4, 0, $Channel, [ref]$status)
    [pscustomobject]@{
        channel = $Channel
        bitrate_kbps = $bitrate
        mode = $mode
        error = $errorCode
        friction_model_valid = $frictionValid
        repeat_passed = $passed
        repeat_expected = $RepeatCount
        receive_error_count = $status.ReceiveErrorCount
        transmit_error_count = $status.TransmitErrorCount
        status_read_ok = $statusOk -eq 1
    }

    if ($mode -ne 0.0 -or $errorCode -ne 0.0 -or $bitrate -ne 1000.0 -or
        $passed -ne $RepeatCount -or $statusOk -ne 1 -or
        $status.ReceiveErrorCount -ne 0 -or $status.TransmitErrorCount -ne 0) {
        throw "Classic CAN smoke test failed."
    }
}
catch {
    $diagnosticStatus = New-Object VectorControlCan+Status
    $diagnosticStatusOk = [VectorControlCan]::VCI_ReadCANStatus(
        4, 0, $Channel, [ref]$diagnosticStatus)
    Write-Host (("CAN_STATUS ok={0} interrupt=0x{1:X2} mode=0x{2:X2} " +
        "status=0x{3:X2} ecc=0x{4:X2} rxerr={5} txerr={6}") -f
        $diagnosticStatusOk, $diagnosticStatus.ErrInterrupt,
        $diagnosticStatus.RegMode, $diagnosticStatus.RegStatus,
        $diagnosticStatus.RegECCapture, $diagnosticStatus.ReceiveErrorCount,
        $diagnosticStatus.TransmitErrorCount)
    throw
}
finally {
    [void][VectorControlCan]::VCI_ResetCAN(4, 0, $Channel)
    [void][VectorControlCan]::VCI_CloseDevice(4, 0)
}
