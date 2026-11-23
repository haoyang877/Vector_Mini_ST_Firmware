//! 参数目录、展示元数据与参数服务 v1 编解码；不持有设备会话。
use anyhow::{Result, ensure};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::sync::OnceLock;

pub const FEATURE: u32 = 1 << 14;
const CAN_BITRATE_ID: u8 = 0x71;

#[derive(Deserialize)]
pub struct Scalar {
    pub id: u8,
    pub value_type: u8,
    pub scale: f64,
    pub unit: String,
    pub access: u8,
    pub minimum: Option<f64>,
    pub maximum: Option<f64>,
}

#[derive(Deserialize)]
pub struct Row {
    pub key: String,
    pub id: Option<u8>,
    pub name: String,
    pub category: String,
    pub group: String,
    pub field: String,
    pub unit: String,
    pub origin: String,
    pub access: String,
    pub source: String,
}

#[derive(Deserialize)]
pub struct Presentation {
    pub rows: Vec<Row>,
    pub groups: BTreeMap<String, BTreeMap<String, String>>,
}

pub fn catalog() -> &'static [Scalar] {
    #[derive(Deserialize)]
    struct Catalog {
        parameters: Vec<Scalar>,
    }
    static CATALOG: OnceLock<Vec<Scalar>> = OnceLock::new();
    CATALOG.get_or_init(|| {
        serde_json::from_str::<Catalog>(include_str!(
            "../../../shared/protocol/parameter_catalog_v1.json"
        ))
        .expect("内嵌参数目录有效")
        .parameters
    })
}

pub fn presentation() -> &'static Presentation {
    static PRESENTATION: OnceLock<Presentation> = OnceLock::new();
    PRESENTATION.get_or_init(|| {
        serde_json::from_str(include_str!("../resources/parameter_presentation.json"))
            .expect("内嵌参数展示目录有效")
    })
}

pub fn scalar(id: u8) -> Option<&'static Scalar> {
    catalog().iter().find(|entry| entry.id == id)
}

impl Scalar {
    pub fn physical(&self, raw: u32) -> f64 {
        let value = if self.value_type == 1 {
            f64::from(raw as i32)
        } else {
            f64::from(raw)
        };
        value / self.scale
    }

    pub fn format(&self, raw: u32) -> String {
        if self.id == CAN_BITRATE_ID {
            return format!("{}/{}", raw >> 16, raw & 0xFFFF);
        }
        if self.unit.contains("crc32") || self.id == 4 {
            return format!("0x{raw:08X}");
        }
        let decimals = self.scale.log10().ceil().clamp(0.0, 9.0) as usize;
        format!("{:.*}", decimals, self.physical(raw))
    }

    pub fn encode(&self, text: &str) -> Result<u32> {
        ensure!(self.access == 3, "参数只读或由标定生成");
        if self.id == CAN_BITRATE_ID {
            let (nominal, data) = text
                .trim()
                .split_once('/')
                .ok_or_else(|| anyhow::anyhow!("请按仲裁/数据输入 kbit/s，如 1000/5000"))?;
            let nominal: u32 = nominal.trim().parse()?;
            let data: u32 = data.trim().parse()?;
            ensure!(
                matches!(nominal, 500 | 1000)
                    && matches!(data, 500 | 1000 | 2000 | 3000 | 4000 | 5000),
                "不支持该 CAN FD 波特率组合"
            );
            return Ok((nominal << 16) | data);
        }
        let physical: f64 = text.trim().parse()?;
        ensure!(physical.is_finite(), "参数必须为有限数值");
        ensure!(
            self.minimum.is_none_or(|v| physical >= v)
                && self.maximum.is_none_or(|v| physical <= v),
            "超出参数目录范围"
        );
        let value = (physical * self.scale).round();
        if self.value_type == 1 {
            ensure!(
                value >= f64::from(i32::MIN) && value <= f64::from(i32::MAX),
                "定点值超界"
            );
            Ok(value as i32 as u32)
        } else {
            ensure!(value >= 0.0 && value <= f64::from(u32::MAX), "无符号值超界");
            Ok(value as u32)
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Value {
    pub raw: u32,
    pub revision: u32,
    pub saved_revision: u32,
    pub flags: u8,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReplyError {
    Rejected(u16, u16),
    Malformed,
}

pub fn parse(data: &[u8], id: Option<u8>) -> std::result::Result<Value, ReplyError> {
    if data.len() == 4 {
        let result = u16::from_le_bytes([data[0], data[1]]);
        let detail = u16::from_le_bytes([data[2], data[3]]);
        return if result != 0 {
            Err(ReplyError::Rejected(result, detail))
        } else {
            Err(ReplyError::Malformed)
        };
    }
    if data.len() != 20 || data[..4] != [0; 4] || data[7] & !7 != 0 {
        return Err(ReplyError::Malformed);
    }
    if let Some(id) = id {
        let entry = scalar(id).ok_or(ReplyError::Malformed)?;
        if data[4] != id || data[5] != entry.value_type || data[6] != entry.access {
            return Err(ReplyError::Malformed);
        }
    } else if data[4..7] != [0; 3] || data[8..12] != [0; 4] {
        return Err(ReplyError::Malformed);
    }
    let read = |offset| u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
    let value = Value {
        raw: read(8),
        revision: read(12),
        saved_revision: read(16),
        flags: data[7],
    };
    if value.revision == 0 || (value.flags & 1 != 0) != (value.saved_revision != 0) {
        return Err(ReplyError::Malformed);
    }
    Ok(value)
}

pub fn write_payload(id: u8, raw: u32, revision: u32) -> Result<[u8; 12]> {
    let entry = scalar(id).ok_or_else(|| anyhow::anyhow!("参数未登记"))?;
    ensure!(entry.access == 3 && revision != 0, "参数权限或版本无效");
    let mut data = [id, entry.value_type, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    data[4..8].copy_from_slice(&revision.to_le_bytes());
    data[8..12].copy_from_slice(&raw.to_le_bytes());
    Ok(data)
}

#[derive(Clone)]
pub struct Report {
    pub uid: String,
    pub saved: bool,
    pub values: BTreeMap<u8, Value>,
    pub unsupported: Vec<u8>,
    pub state: Option<Value>,
}

pub enum Action {
    Read { ids: Vec<u8>, saved: bool },
    Write { id: u8, raw: u32, revision: u32 },
    Save { revision: u32 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_presentation_matches_protocol_without_design_examples() {
        let rows = &presentation().rows;
        assert_eq!(rows.len(), 135);
        assert_eq!(catalog().len(), 49);
        for entry in catalog() {
            assert_eq!(
                rows.iter().filter(|row| row.id == Some(entry.id)).count(),
                1
            );
            assert!(entry.scale > 0.0);
        }
        assert_eq!(
            rows.iter().filter(|row| row.category == "control").count(),
            19
        );
        assert!(!include_str!("../resources/parameter_presentation.json").contains("\"example\""));
    }

    #[test]
    fn can_bitrate_editor_matches_wire_vectors() {
        let entry = scalar(CAN_BITRATE_ID).unwrap();
        for nominal in [500u32, 1000] {
            for data in [500u32, 1000, 2000, 3000, 4000, 5000] {
                let packed = (nominal << 16) | data;
                assert_eq!(entry.format(packed), format!("{nominal}/{data}"));
                assert_eq!(entry.encode(&entry.format(packed)).unwrap(), packed);
            }
        }
        for invalid in ["500", "250/2000", "500/1500", "1000/8000"] {
            assert!(entry.encode(invalid).is_err());
        }
    }

    #[test]
    fn fixed_point_rejects_nonfinite_readonly_overflow_and_bad_boolean() {
        let kp = scalar(11).unwrap();
        assert_eq!(kp.encode("0.123").unwrap(), 123);
        assert_eq!(kp.format(123), "0.123");
        for text in ["NaN", "inf", "1e100"] {
            assert!(kp.encode(text).is_err());
        }
        assert!(scalar(5).unwrap().encode("10").is_err());
        assert!(scalar(104).unwrap().encode("2").is_err());
        assert_eq!(kp.physical((-500i32) as u32), -0.5);
    }

    #[test]
    fn reply_offsets_flags_and_business_errors_are_strict() {
        let mut reply = [0u8; 20];
        reply[4..8].copy_from_slice(&[11, 1, 3, 3]);
        reply[8..12].copy_from_slice(&123456u32.to_le_bytes());
        reply[12..16].copy_from_slice(&4u32.to_le_bytes());
        reply[16..20].copy_from_slice(&2u32.to_le_bytes());
        let value = parse(&reply, Some(11)).unwrap();
        assert_eq!(
            (value.raw, value.revision, value.saved_revision),
            (123456, 4, 2)
        );
        assert!(parse(&reply, Some(12)).is_err());
        reply[7] = 8;
        assert_eq!(parse(&reply, Some(11)).unwrap_err(), ReplyError::Malformed);
        assert_eq!(
            parse(&[5, 0, 6, 0], Some(11)).unwrap_err(),
            ReplyError::Rejected(5, 6)
        );
        assert!(parse(&[0; 4], None).is_err());
        assert_eq!(
            &write_payload(11, 12, 34).unwrap()[4..],
            &[34, 0, 0, 0, 12, 0, 0, 0]
        );
    }
}
