param(
    [string]$Axf = "MDK-ARM/Vector_Mini_ST/Vector_Mini_ST.axf",
    [string]$Map = "MDK-ARM/Vector_Mini_ST/Vector_Mini_ST.map",
    [string]$JLinkExe = "C:/Program Files/SEGGER/JLink_V964/JLink.exe",
    [string]$FromElfExe = "C:/Keil_v5/ARM/ARMCLANG/bin/fromelf.exe",
    [string]$JLinkSerialNumber = "602722271"
)

$ErrorActionPreference = "Stop"

if (!(Test-Path -LiteralPath $Axf) -or !(Test-Path -LiteralPath $Map)) {
    throw "Build AXF and map files are required before reading fast-loop metrics."
}

$fieldOffsets = & $FromElfExe --fieldoffsets $Axf
$offsetMatch = $fieldOffsets | Select-String -Pattern '^\|MotorControlRuntimeContext\.fast_loop_metrics\|\s+EQU\s+(0x[0-9a-fA-F]+)' | Select-Object -First 1
if ($null -eq $offsetMatch) {
    throw "MotorControlRuntimeContext.fast_loop_metrics was not found in AXF debug data."
}
$baseMatch = Select-String -LiteralPath $Map -Pattern '^\s+MotorControlRuntime\s+(0x[0-9a-fA-F]+)\s+Data' | Select-Object -First 1
if ($null -eq $baseMatch) {
    throw "MotorControlRuntime base address was not found in the map file."
}

$offset = [Convert]::ToUInt32($offsetMatch.Matches[0].Groups[1].Value.Substring(2), 16)
$base = [Convert]::ToUInt32($baseMatch.Matches[0].Groups[1].Value.Substring(2), 16)
$address = $base + $offset
$commandPath = Join-Path ([IO.Path]::GetTempPath()) ("vector-fast-loop-{0}.jlink" -f [Guid]::NewGuid().ToString("N"))

try {
    [IO.File]::WriteAllLines($commandPath, @(
        "device STM32G431CB",
        "si SWD",
        "speed 4000",
        "connect",
        ("mem32 0x{0:X8} 6" -f $address),
        "exit"
    ))
    $jlinkOutput = & $JLinkExe -USB $JLinkSerialNumber -CommandFile $commandPath
    $words = [System.Collections.Generic.List[uint32]]::new()
    foreach ($line in $jlinkOutput) {
        if ($line -match '^[0-9A-Fa-f]{8}\s+=\s+(.+)$') {
            foreach ($word in ($Matches[1] -split '\s+')) {
                if ($word -match '^[0-9A-Fa-f]{8}$') {
                    $words.Add([Convert]::ToUInt32($word, 16))
                }
            }
        }
    }
    if ($words.Count -ne 6) {
        throw "Expected six metric words from J-Link, received $($words.Count)."
    }
    [pscustomobject]@{
        address = ("0x{0:X8}" -f $address)
        invocation_count = $words[0]
        maximum_cycles = $words[1]
        deadline_cycles = $words[2]
        deadline_overrun_count = $words[3]
        latest_cycles = $words[4]
        filtered_cycles = $words[5]
    }
}
finally {
    if (Test-Path -LiteralPath $commandPath) {
        Remove-Item -LiteralPath $commandPath -Force
    }
}
