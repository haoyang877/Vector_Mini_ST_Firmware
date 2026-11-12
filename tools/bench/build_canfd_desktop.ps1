param([Parameter(Mandatory = $true)][string]$Dll, [string]$FirmwareHex,
      [string]$CanPackage = "outputs/AT32F456-FULL-v1.7.0/app-package",
      [string[]]$PitchThermal = @('3','3','3600','0.8','0.7','1','0.5'),
      [string[]]$YawThermal = @('3','3','3600','0.8','0.7','1','0.5'))
$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$sdk = (Resolve-Path -LiteralPath $Dll).Path
if ((Split-Path -Leaf $sdk) -ne "ControlCANFD.dll") { throw "请选择厂商 x64 ControlCANFD.dll" }

Push-Location $root
try {
    $output = "outputs/canfd-desktop"
    New-Item -ItemType Directory -Path $output -Force | Out-Null
    $batchFile = Join-Path $root "tools/bench/five_axis_can_stability.json"
    $batch = Get-Content -Encoding utf8 -Raw -LiteralPath $batchFile | ConvertFrom-Json
    $canDir = (Resolve-Path -LiteralPath $CanPackage).Path
    if ((Get-FileHash -LiteralPath (Join-Path $canDir 'image.bin')).Hash -ne $batch.can_image_sha256 -or
        (Get-FileHash -LiteralPath (Join-Path $canDir 'manifest.bin')).Hash -ne $batch.can_manifest_sha256) {
        throw "CAN 镜像或清单与本批 J-Link 固件不配套"
    }
    uv run python host_app/upgrade_client.py inspect --target yg_esc_v1_at32f456 --package $canDir
    if ($LASTEXITCODE -ne 0) { throw "CAN 升级包校验失败" }
    if (-not $FirmwareHex) {
        $match = Get-ChildItem -Path "outputs/AT32F456-FULL-*/*.hex" -File |
            Where-Object { (Get-FileHash -LiteralPath $_.FullName).Hash -eq $batch.hex_sha256 } |
            Select-Object -First 1
        if (-not $match) { throw "未找到本批统一烧录包，请指定 -FirmwareHex" }
        $FirmwareHex = $match.FullName
    }
    if ((Get-FileHash -LiteralPath $FirmwareHex).Hash -ne $batch.hex_sha256) {
        throw "HEX 与五轴固件清单的 SHA-256 不符"
    }
    $firmware = New-Item -ItemType Directory -Path "$output/resources" -Force
    Copy-Item -LiteralPath $FirmwareHex -Destination (Join-Path $firmware.FullName "five-axis.hex")
    if ($PitchThermal.Count -ne 7 -or $YawThermal.Count -ne 7) {
        throw "请为 PITCH 和 YAW 分别提供七项已核实的热参数"
    }
    $records = New-Item -ItemType Directory -Path (Join-Path $firmware.FullName 'first-fit') -Force
    foreach ($node in 3..7) {
        $speed = @('20', '20', '3.14159265', '2.0943951', '2.0943951')[$node - 3]
        $model = @('HT8115-4','HT8115-4','HT8115-4','YQ-5740','YQ-5708')[$node - 3]
        $thermal = @()
        if ($node -eq 6) { $thermal = @('--thermal') + $PitchThermal }
        if ($node -eq 7) { $thermal = @('--thermal') + $YawThermal }
        $temp = Join-Path $records.FullName ("$node-" + [Guid]::NewGuid().ToString('N') + '.tmp')
        uv run --extra dev python tools/build/make_axis_record.py --node $node --model $model --speed $speed --output-shaft --confirm-model @thermal --output $temp
        if ($LASTEXITCODE -ne 0) { throw "节点 $node 参数生成失败" }
        Move-Item -LiteralPath $temp -Destination (Join-Path $records.FullName "$node.bin") -Force
    }
    uv run --extra maintenance --with pyinstaller==6.22.3 python -m PyInstaller --noconfirm --onefile --windowed `
        --name CAN-Link-Test-v1.6.3 --distpath "$output/dist" --workpath "$output/build" `
        --specpath $output --paths loader/tools --paths host_app --paths shared/targets --paths tools/bench `
        --add-binary "${sdk};." --add-data "$root/tools/bench/five_axis_can_stability.json;." `
        --add-data "$root/shared/targets/yg_esc_v1_at32f456.json;." `
        --add-data "$($firmware.FullName)/five-axis.hex;firmware" `
        --add-data "$($records.FullName)/*.bin;first-fit" `
        --add-data "$canDir/image.bin;can-app" --add-data "$canDir/manifest.bin;can-app" --add-data "$canDir/package.json;can-app" `
        tools/bench/canfd_desktop.py
    if ($LASTEXITCODE -ne 0) { throw "PyInstaller 打包失败 ($LASTEXITCODE)" }
    Get-FileHash -LiteralPath "$output/dist/CAN-Link-Test-v1.6.3.exe" -Algorithm SHA256
} finally {
    Pop-Location
}
