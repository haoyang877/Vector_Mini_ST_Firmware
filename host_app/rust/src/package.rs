//! 离线 APP 包检查。任何目标、描述符、CRC 或文件哈希不匹配均禁止升级。

use crate::target::Target;
use anyhow::{Context, Result, ensure};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::Path;

const PROFILE: u32 = 0x5359_3549;

fn be32(data: &[u8], offset: usize) -> Result<u32> {
    Ok(u32::from_be_bytes(data[offset..offset + 4].try_into()?))
}
fn be16(data: &[u8], offset: usize) -> Result<u16> {
    Ok(u16::from_be_bytes(data[offset..offset + 2].try_into()?))
}

#[derive(Deserialize)]
struct Metadata {
    format: String,
    image_sha256: String,
    manifest_sha256: String,
    loader_version: u32,
}

pub struct UpgradePackage {
    pub image: Vec<u8>,
    pub manifest: Vec<u8>,
    pub version: u32,
    pub crc32: u32,
    pub token: u32,
    pub target: [u32; 8],
    pub loader_version: u32,
}

impl UpgradePackage {
    pub fn load(directory: &Path, target: &Target) -> Result<Self> {
        let image = fs::read(directory.join("image.bin")).context("无法读取 image.bin")?;
        let manifest = fs::read(directory.join("manifest.bin")).context("无法读取 manifest.bin")?;
        let metadata: Metadata = serde_json::from_slice(
            &fs::read(directory.join("package.json")).context("无法读取 package.json")?,
        )?;
        ensure!(
            manifest.len() == 96 && image.len() >= 608 && image.len() % 8 == 0,
            "镜像或清单长度非法"
        );
        let expected = [
            0x494d_3331,
            PROFILE,
            target.layout,
            target.mcu,
            target.product,
            target.hardware,
        ];
        for (index, wanted) in expected.into_iter().enumerate() {
            ensure!(be32(&manifest, index * 4)? == wanted, "清单目标不匹配");
        }
        let version = be32(&manifest, 24)?;
        let token = be32(&manifest, 28)?;
        let length = be32(&manifest, 32)?;
        let role = be32(&manifest, 36)?;
        let abi = be32(&manifest, 40)?;
        let low = be16(&manifest, 44)?;
        let high = be16(&manifest, 46)?;
        let crc32 = be32(&manifest, 48)?;
        ensure!(
            version != 0
                && token != 0
                && length as usize == image.len()
                && length <= target.app_bytes
                && role == 1
                && abi == target.abi
                && low <= target.schema as u16
                && target.schema as u16 <= high
                && manifest[52..].iter().all(|&value| value == 0),
            "升级包版本、角色、ABI 或 schema 不匹配"
        );
        let offset = target.descriptor_offset;
        ensure!(offset + 96 <= image.len(), "描述符越界");
        let descriptor = &image[offset..offset + 96];
        let expected = [
            0x5949_4431,
            3,
            PROFILE,
            target.layout,
            target.mcu,
            target.product,
            target.hardware,
            version,
            role,
            abi,
        ];
        for (index, wanted) in expected.into_iter().enumerate() {
            ensure!(
                be32(descriptor, index * 4)? == wanted,
                "镜像描述符与清单不匹配"
            );
        }
        ensure!(
            be16(descriptor, 40)? == low
                && be16(descriptor, 42)? == high
                && be32(descriptor, 44)? == length
                && be32(descriptor, 48)? == crc32
                && descriptor[52..64].iter().all(|&value| value == 0)
                && descriptor[64..88].iter().all(|&value| value == 0xff)
                && descriptor[88..96].iter().all(|&value| value == 0),
            "镜像长度、CRC 或激活槽格式错误"
        );
        let mut normalized = image.clone();
        normalized[offset + 48..offset + 52].fill(0);
        ensure!(crc32fast::hash(&normalized) == crc32, "镜像 CRC32 不匹配");
        let sp = u32::from_le_bytes(image[0..4].try_into()?);
        let pc = u32::from_le_bytes(image[4..8].try_into()?);
        ensure!(
            target.ram_base < sp
                && sp <= target.ram_end
                && sp % 8 == 0
                && pc & 1 != 0
                && target.app_base + offset as u32 + 96 <= (pc & !1)
                && (pc & !1) < target.app_base + length,
            "镜像向量不在配套 APP/RAM 范围"
        );
        ensure!(
            metadata.format == "loader-image-v3-crc32"
                && metadata.loader_version == target.loader_version
                && metadata
                    .image_sha256
                    .eq_ignore_ascii_case(&hex::encode(Sha256::digest(&image)))
                && metadata
                    .manifest_sha256
                    .eq_ignore_ascii_case(&hex::encode(Sha256::digest(&manifest))),
            "包元数据或 SHA-256 不匹配"
        );
        Ok(Self {
            image,
            manifest,
            version,
            crc32,
            token,
            target: target.upgrade_tuple(),
            loader_version: target.loader_version,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put32(data: &mut [u8], offset: usize, value: u32) {
        data[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    #[test]
    fn bundled_target_rejects_old_layout_fixture() {
        let target = Target::bundled().unwrap();
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("../../../tests/fixtures/loader_image_v3.json"))
                .unwrap();
        let manifest = hex::decode(fixture["manifest_hex"].as_str().unwrap()).unwrap();
        assert_ne!(be32(&manifest, 8).unwrap(), target.layout);
    }

    #[test]
    fn current_layout_package_accepts_only_matching_bytes() {
        let target = Target::bundled().unwrap();
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("../../../tests/fixtures/loader_image_v3.json"))
                .unwrap();
        let mut image = hex::decode(fixture["image_hex"].as_str().unwrap()).unwrap();
        let mut manifest = hex::decode(fixture["manifest_hex"].as_str().unwrap()).unwrap();
        image[4..8].copy_from_slice(&(target.app_base + 0x261).to_le_bytes());
        put32(&mut image, target.descriptor_offset + 12, target.layout);
        put32(&mut image, target.descriptor_offset + 36, target.abi);
        image[target.descriptor_offset + 40..target.descriptor_offset + 42]
            .copy_from_slice(&(target.schema as u16).to_be_bytes());
        image[target.descriptor_offset + 42..target.descriptor_offset + 44]
            .copy_from_slice(&(target.schema as u16).to_be_bytes());
        put32(&mut manifest, 8, target.layout);
        put32(&mut manifest, 40, target.abi);
        manifest[44..46].copy_from_slice(&(target.schema as u16).to_be_bytes());
        manifest[46..48].copy_from_slice(&(target.schema as u16).to_be_bytes());
        put32(&mut image, target.descriptor_offset + 48, 0);
        let crc = crc32fast::hash(&image);
        put32(&mut image, target.descriptor_offset + 48, crc);
        put32(&mut manifest, 48, crc);
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../outputs/rust-host-tests");
        fs::create_dir_all(&root).unwrap();
        let dir = root.join(format!("package-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("image.bin"), &image).unwrap();
        fs::write(dir.join("manifest.bin"), &manifest).unwrap();
        fs::write(
            dir.join("package.json"),
            serde_json::to_vec(&serde_json::json!({
                "format": "loader-image-v3-crc32",
                "loader_version": target.loader_version,
                "image_sha256": hex::encode(Sha256::digest(&image)),
                "manifest_sha256": hex::encode(Sha256::digest(&manifest)),
            }))
            .unwrap(),
        )
        .unwrap();
        assert!(UpgradePackage::load(&dir, &target).is_ok());
        let last = image.len() - 1;
        image[last] ^= 1;
        fs::write(dir.join("image.bin"), &image).unwrap();
        assert!(UpgradePackage::load(&dir, &target).is_err());
        fs::remove_dir_all(dir).unwrap();
    }
}
