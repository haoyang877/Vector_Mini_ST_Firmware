param(
    [string]$Compiler
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$hostTestScript = Join-Path $PSScriptRoot 'build_and_run_host_tests.ps1'
$variants = @('Damped', 'NoDamper')

foreach ($variant in $variants) {
    Write-Output "HOST_TEST_MATRIX_VARIANT=$variant"
    $parameters = @{ ProductVariant = $variant }
    if (-not [string]::IsNullOrWhiteSpace($Compiler)) {
        $parameters.Compiler = $Compiler
    }
    & $hostTestScript @parameters
    if ($LASTEXITCODE -ne 0) {
        Write-Output "HOST_TEST_MATRIX_RESULT=FAILED variant=$variant exit_code=$LASTEXITCODE"
        exit $LASTEXITCODE
    }
}

Write-Output 'HOST_TEST_MATRIX_RESULT=PASSED variants=2'
