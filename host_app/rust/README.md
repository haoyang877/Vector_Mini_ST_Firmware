# Rust工作台构建与部署

当前源码版本由Cargo.toml维护，产品行为见[软件设计](../../docs/architecture/yg_mc_host_design_v1.md)。
本页只维护可重现的构建、部署和离线验证方法，不记录版本调试过程。

## 构建

在仓库根目录执行，生成物保存在outputs：

```powershell
$env:CARGO_TARGET_DIR = "$PWD/outputs/host-rust-cargo"
cargo fmt --manifest-path host_app/rust/Cargo.toml --check
cargo clippy --manifest-path host_app/rust/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path host_app/rust/Cargo.toml --locked
cargo build --manifest-path host_app/rust/Cargo.toml --release --locked
```

Windows GNU工具链需要兼容的dlltool与链接器。必要时使用Rust自带LLD：

```powershell
$taskRustSysroot = (& rustc --print sysroot).Trim()
$taskRustLld = Join-Path $taskRustSysroot 'lib/rustlib/x86_64-pc-windows-gnu/bin/rust-lld.exe'
$env:CARGO_ENCODED_RUSTFLAGS = @('-C', 'linker-flavor=ld.lld', '-C', "linker=$taskRustLld", '-C', 'link-self-contained=yes') -join [char]31
cargo build --manifest-path host_app/rust/Cargo.toml --release --locked
```

## 部署契约

目标身份、Flash布局、schema与ABI来自shared/targets，不能在UI手工复制一套。
Windows包须包含维护脚本、隔离Python、J-Link器件库、目标契约和ControlCANFD.dll。
只有EXE不足以进行SWD维护。打包入口为：

```text
python tools/build/package_rust_host.py --exe <Rust EXE> --can-dll <ControlCANFD.dll> --jlink-dir <J-Link SDK目录> --firmware <整机构建目录>
```

打包器核对版本、固件哈希和器件支持，清单与文件名必须一致。
版本更新后同步Cargo.lock，窗口显示从构建版本读取。

## 离线验证与设备边界

`--preview-dir <目录>`生成未连接界面；`--studio-preview`提供明确标识的离线样机。
离线示例不能进入真实SDK或被用作设备身份。正常会话的扫描、UID复核、
启动前续租预留、参数/标定事务与SWD互斥遵循设计文档。
未知设备能力、未接入功能和缺失数据明确显示，不采用样例值补齐。

构建、界面预览、协议替身和真实设备验收分开；运行数据由各自作业写入用户目录或outputs，
不回填为历史过程文档。设备操作必须使用实际目标、UID和配套包。
