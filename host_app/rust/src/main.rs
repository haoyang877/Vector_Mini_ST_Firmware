//! YG_MC_HOST Rust 入口；离线 inspect 不启动 GUI，也不打开 CAN 设备。

mod can;
mod capture;
mod device;
mod package;
mod target;
mod ui;
mod upgrade;
mod wire;

use anyhow::Result;
use std::path::Path;

fn main() -> Result<()> {
    let target = target::Target::bundled()?;
    let args: Vec<String> = std::env::args().collect();
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
    let preview = if args.len() == 3 && args[1] == "--preview" {
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
            "用法: YG_MC_HOST [--inspect <APP 包目录> | --preview <PNG> | --preview-dir <目录>]"
        );
    };
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1366.0, 768.0])
            .with_min_inner_size([1024.0, 700.0]),
        ..Default::default()
    };
    eframe::run_native(
        "YG_MC_HOST",
        options,
        Box::new(move |cc| Ok(Box::new(ui::HostApp::new(cc, target, preview)))),
    )
    .map_err(|error| anyhow::anyhow!("Rust 工作台启动失败: {error}"))?;
    Ok(())
}
