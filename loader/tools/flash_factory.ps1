# 显式选择探针和芯片，调用同目录的初装包烧录工具。
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Package,
    [Parameter(Mandatory=$true)][long]$JLinkSerial,
    [Parameter(Mandatory=$true)][string]$Uid,
    [Parameter(Mandatory=$true)][string]$Out,
    [string]$Python = 'python',
    [string]$JLinkDll,
    [string]$PylinkDir,
    [switch]$Run
)
$ErrorActionPreference = 'Stop'
$factoryArguments = @((Join-Path $PSScriptRoot 'loader_factory.py'), 'flash',
    '--package', $Package, '--jlink-serial', "$JLinkSerial", '--uid', $Uid, '--out', $Out)
if ($JLinkDll) { $factoryArguments += @('--jlink-dll', $JLinkDll) }
if ($PylinkDir) { $factoryArguments += @('--pylink-dir', $PylinkDir) }
if ($Run) { $factoryArguments += '--run' }
& $Python @factoryArguments
if ($LASTEXITCODE -ne 0) { throw "初装烧录失败，退出码 $LASTEXITCODE；请查看 $Out/summary.json" }
