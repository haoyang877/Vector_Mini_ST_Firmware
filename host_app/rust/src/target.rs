//! 共享目标清单是主机的唯一产品、分区和版本来源。

use anyhow::{Context, Result, bail};
use serde::Deserialize;
use std::collections::HashMap;

#[derive(Deserialize)]
struct Contract {
    target: String,
    constants: HashMap<String, String>,
}

#[derive(Clone, Debug)]
pub struct Target {
    pub name: String,
    pub mcu: u32,
    pub product: u32,
    pub hardware: u32,
    pub layout: u32,
    pub schema: u32,
    pub abi: u32,
    pub app_base: u32,
    pub app_bytes: u32,
    pub loader_bytes: u32,
    pub identity_base: u32,
    pub descriptor_offset: usize,
    pub ram_base: u32,
    pub ram_end: u32,
    pub loader_version: u32,
}

impl Target {
    pub fn bundled() -> Result<Self> {
        Self::named("vector_mini_st_g431")
    }

    pub fn named(name: &str) -> Result<Self> {
        let raw = match name {
            "vector_mini_st_g431" => {
                include_str!("../../../shared/targets/vector_mini_st_g431.json")
            }
            "yg_esc_v1_at32f456" => include_str!("../../../shared/targets/yg_esc_v1_at32f456.json"),
            _ => bail!("未知目标，不能回退到默认板卡"),
        };
        let document: Contract = serde_json::from_str(raw)?;
        let value = |name: &str| -> Result<u32> {
            let text = document
                .constants
                .get(name)
                .with_context(|| format!("目标常量缺失: {name}"))?
                .trim_end_matches('U');
            if let Some(hex) = text.strip_prefix("0x") {
                Ok(u32::from_str_radix(hex, 16)?)
            } else {
                Ok(text.parse()?)
            }
        };
        let target = Self {
            name: document.target.clone(),
            mcu: value("TARGET_MCU")?,
            product: value("TARGET_PRODUCT")?,
            hardware: value("TARGET_HARDWARE")?,
            layout: value("TARGET_LAYOUT")?,
            schema: value("TARGET_SCHEMA")?,
            abi: value("TARGET_PARAM_ABI")?,
            app_base: value("TARGET_APP_BASE")?,
            app_bytes: value("TARGET_APP_BYTES")?,
            loader_bytes: value("TARGET_LOADER_BYTES")?,
            identity_base: value("TARGET_IDENTITY_BASE")?,
            descriptor_offset: value("TARGET_DESCRIPTOR_OFFSET")? as usize,
            ram_base: value("TARGET_RAM_BASE")?,
            ram_end: value("TARGET_RAM_END")?,
            loader_version: value("TARGET_LOADER_VERSION")?,
        };
        if target.app_base != 0x0800_0000 + target.loader_bytes
            || target.app_base + target.app_bytes > target.identity_base
            || target.layout == 0
            || target.loader_bytes != 16384
            || target.app_bytes != 98304
            || target.identity_base != 0x0801F800
        {
            bail!("目标清单的 Flash 分区不连续或重叠");
        }
        Ok(target)
    }

    pub fn upgrade_tuple(&self) -> [u32; 8] {
        [
            self.mcu,
            self.layout,
            self.abi,
            self.schema,
            self.app_base,
            self.app_bytes,
            self.loader_bytes,
            self.identity_base,
        ]
    }
}
