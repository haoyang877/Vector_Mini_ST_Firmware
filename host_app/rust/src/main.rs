//! YG_MC_HOST Rust 入口；离线 inspect 不启动 GUI，也不打开 CAN 设备。

mod calibration;
mod can;
mod capture;
mod control;
mod device;
mod diagnostics;
mod discovery;
mod maintenance;
mod motor_configuration;
mod package;
mod parameters;
mod target;
mod ui;
mod upgrade;
mod wire;

use anyhow::Result;
use std::path::Path;

fn main() -> Result<()> {
    let mut args: Vec<String> = std::env::args().collect();
    if matches!(args.as_slice(), [_, option] if option == "--version") {
        println!("YG_MC_HOST v{}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }
    let target = if let Some(at) = args.iter().position(|value| value == "--target") {
        let name = args
            .get(at + 1)
            .ok_or_else(|| anyhow::anyhow!("--target缺少目标名称"))?
            .clone();
        args.drain(at..=at + 1);
        target::Target::named(&name)?
    } else {
        target::Target::bundled()?
    };
    if args.len() == 3 && args[1] == "--inspect" {
        let package = package::UpgradePackage::load(Path::new(&args[2]), &target)?;
        println!(
            "APP v{}.{}.{} · {} B · CRC32 {:08x} · layout {}",
            package.version >> 16,
            (package.version >> 8) & 255,
            package.version & 255,
            package.image.len(),
            package.crc32,
            target.layout
        );
        return Ok(());
    }
    let bench = if args.len() == 5 && args[1] == "--bench-observe" && args[3] == "--uid" {
        let uid = args[4].to_ascii_uppercase();
        anyhow::ensure!(
            uid.len() == 24 && uid.bytes().all(|b| b.is_ascii_hexdigit()),
            "实机测试需要明确的 24 位 UID"
        );
        let output = std::path::PathBuf::from(&args[2]);
        std::fs::create_dir_all(&output)?;
        args.truncate(1);
        Some(ui::BenchRequest { uid, output })
    } else {
        None
    };
    let preview = if args.len() == 2 && args[1] == "--studio-preview" {
        Some(ui::PreviewRequest::Interactive)
    } else if args.len() == 3 && args[1] == "--preview" {
        let path = Path::new(&args[2]).to_path_buf();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        Some(ui::PreviewRequest::Single(path))
    } else if args.len() == 3 && args[1] == "--preview-dir" {
        let path = Path::new(&args[2]).to_path_buf();
        std::fs::create_dir_all(&path)?;
        Some(ui::PreviewRequest::Gallery(path))
    } else if args.len() == 1 {
        None
    } else {
        anyhow::bail!(
            "用法: YG_MC_HOST [--studio-preview | --inspect <APP 包目录> | --preview <PNG> | --preview-dir <目录>]"
        );
    };
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1080.0, 720.0])
            .with_min_inner_size([1024.0, 700.0]),
        ..Default::default()
    };
    eframe::run_native(
        concat!("YG_MC_HOST v", env!("CARGO_PKG_VERSION")),
        options,
        Box::new(move |cc| Ok(Box::new(ui::HostApp::new(cc, target, preview, bench)))),
    )
    .map_err(|error| anyhow::anyhow!("Rust 工作台启动失败: {error}"))?;
    Ok(())
}
