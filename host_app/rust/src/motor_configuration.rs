//! CAN 型号配置 v1 的值对象；实际电气参数由固件产品预设构造。
use crate::wire::{u16le, u32le};
use anyhow::{Result, ensure};

pub const FEATURE: u32 = 1 << 17;
pub const READ: u16 = 190;
pub const PREPARE: u16 = 191;
pub const COMMIT: u16 = 192;
pub const ACTIVATE: u16 = 193;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    Ht8115,
    Yq5708,
    Yq5740,
}
impl Model {
    pub const ALL: [Self; 3] = [Self::Ht8115, Self::Yq5708, Self::Yq5740];
    pub fn id(self) -> u8 {
        match self {
            Self::Ht8115 => 1,
            Self::Yq5708 => 2,
            Self::Yq5740 => 3,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Self::Ht8115 => "HT8115-4",
            Self::Yq5708 => "YQ-5708",
            Self::Yq5740 => "YQ-5740",
        }
    }
    pub fn nodes(self) -> &'static [u8] {
        match self {
            Self::Ht8115 => &[3, 4, 5],
            Self::Yq5708 => &[7],
            Self::Yq5740 => &[6],
        }
    }
}
pub fn axis_name(node: u8) -> &'static str {
    match node {
        3 => "底盘左轮",
        4 => "底盘右轮",
        5 => "ROLL",
        6 => "PITCH",
        7 => "YAW",
        _ => "未指定轴",
    }
}
pub fn model_name(id: u8) -> &'static str {
    Model::ALL
        .into_iter()
        .find(|model| model.id() == id)
        .map_or("未配置", Model::name)
}

#[derive(Clone, Debug)]
pub struct Info {
    pub model: u8,
    pub node: u8,
    pub flags: u8,
    pub revision: u32,
    pub speed_rad_s: f64,
    pub thermal: [f64; 7],
}
impl Info {
    pub fn matches_request(&self, payload: &[u8; 40]) -> bool {
        if self.model != payload[1] || self.node != payload[2] || self.flags & 6 != 0 {
            return false;
        }
        let close = |actual: f64, expected: f64, scale: f64| {
            (actual - expected).abs()
                <= (1.5 / scale).max(expected.abs() * 2.0 * f64::from(f32::EPSILON))
        };
        let speed = i32::from_le_bytes(payload[8..12].try_into().unwrap());
        if !close(self.speed_rad_s, f64::from(speed) / 1000.0, 1000.0) {
            return false;
        }
        if payload[12..].iter().all(|v| *v == 0) {
            return self.model == Model::Ht8115.id();
        }
        self.thermal.iter().enumerate().all(|(i, actual)| {
            let scale = if i < 3 { 1000.0 } else { 1_000_000.0 };
            let value = i32::from_le_bytes(payload[12 + i * 4..16 + i * 4].try_into().unwrap());
            close(*actual, f64::from(value) / scale, scale)
        })
    }
    pub fn parse(data: &[u8]) -> Result<Self> {
        ensure!(
            data.len() == 44
                && u16le(data, 0)? == 0
                && u16le(data, 2)? == 0
                && data[4] == 1
                && data[5] <= 3
                && data[6] > 0
                && data[6] < 255
                && data[7] & !63 == 0,
            "型号配置回复格式错误"
        );
        if let Some(model) = Model::ALL.into_iter().find(|model| model.id() == data[5]) {
            ensure!(model.nodes().contains(&data[6]), "设备型号与轴节点不匹配");
        }
        let decode = |offset, scale: f64| -> Result<f64> {
            let value = u32le(data, offset)? as i32;
            ensure!(value >= 0, "型号参数超出可表示范围");
            Ok(f64::from(value) / scale)
        };
        let mut thermal = [0.0; 7];
        for (i, value) in thermal.iter_mut().enumerate() {
            *value = decode(16 + i * 4, if i < 3 { 1000.0 } else { 1_000_000.0 })?;
        }
        Ok(Self {
            model: data[5],
            node: data[6],
            flags: data[7],
            revision: u32le(data, 8)?,
            speed_rad_s: decode(12, 1000.0)?,
            thermal,
        })
    }
    pub fn blocked(&self) -> bool {
        self.flags & 24 != 0
    }
    pub fn calibrated(&self) -> bool {
        self.flags & 6 == 6
    }
}

#[derive(Clone)]
pub struct Draft {
    pub model: Model,
    pub node: u8,
    pub speed_rad_s: f64,
}
impl Default for Draft {
    fn default() -> Self {
        Self {
            model: Model::Ht8115,
            node: 5,
            speed_rad_s: 1.0,
        }
    }
}
impl Draft {
    pub fn payload(&self, revision: u32, thermal_text: &str) -> Result<[u8; 40]> {
        ensure!(
            self.model.nodes().contains(&self.node),
            "型号与轴节点不匹配"
        );
        let heat = if thermal_text.trim().is_empty() {
            ensure!(self.model == Model::Ht8115, "YQ 型号需要七项已核实的热参数");
            vec![0.0; 7]
        } else {
            let values: Vec<f64> = thermal_text
                .split_whitespace()
                .map(str::parse)
                .collect::<std::result::Result<_, _>>()?;
            ensure!(
                values.len() == 7,
                "请填写七项热参数：持续A 峰值A 时间s 进入比 退出比 跳闸比 清除比"
            );
            ensure!(
                values.iter().all(|v| v.is_finite() && *v > 0.0)
                    && values[1] >= values[0]
                    && values[6] < values[4]
                    && values[4] < values[3]
                    && values[3] < values[5],
                "热参数无效，请核对电流、时间和门限顺序"
            );
            values
        };
        let encode = |value: f64, scale: f64| -> Result<i32> {
            let number = (value * scale).round();
            ensure!(
                value.is_finite() && number >= 0.0 && number <= f64::from(i32::MAX) - 256.0,
                "参数超出线路范围"
            );
            Ok(number as i32)
        };
        ensure!(self.speed_rad_s > 0.0, "速度上限必须大于零");
        let speed = encode(self.speed_rad_s, 1000.0)?;
        ensure!(speed > 0, "速度上限小于线路最小分辨率");
        let mut data = [0u8; 40];
        data[..4].copy_from_slice(&[1, self.model.id(), self.node, 1]);
        data[4..8].copy_from_slice(&revision.to_le_bytes());
        data[8..12].copy_from_slice(&speed.to_le_bytes());
        for (i, value) in heat.into_iter().enumerate() {
            data[12 + i * 4..16 + i * 4].copy_from_slice(
                &encode(value, if i < 3 { 1000.0 } else { 1_000_000.0 })?.to_le_bytes(),
            );
        }
        Ok(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wire_contract_matches_frozen_configuration_vectors() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../../tests/fixtures/protocol/yg_protocol_motor_configuration_v1_vectors.json"
        ))
        .unwrap();
        for (vector, id) in fixture["vectors"].as_array().unwrap().iter().zip([PREPARE, READ, COMMIT, ACTIVATE]) {
            assert_eq!(vector["type"].as_u64(), Some(u64::from(id)));
        }
        let write = fixture["vectors"][0]["request_payload_hex"]
            .as_str()
            .unwrap();
        let draft = Draft {
            model: Model::Yq5740,
            node: 6,
            speed_rad_s: 1.0,
        };
        assert_eq!(
            hex::encode(draft.payload(0x12345678, "3 7 65 0.8 0.7 1 0.5").unwrap()),
            write
        );
        let mut read = hex::decode(
            fixture["vectors"][1]["response_payload_hex"]
                .as_str()
                .unwrap(),
        )
        .unwrap();
        let info = Info::parse(&read).unwrap();
        assert_eq!((info.model, info.node, info.revision), (3, 6, 0x12345678));
        assert_eq!(info.thermal, [3.0, 7.0, 65.0, 0.8, 0.7, 1.0, 0.5]);
        assert!(info.blocked() && !info.calibrated());
        let payload = draft.payload(0x12345678, "3 7 65 0.8 0.7 1 0.5").unwrap();
        assert!(info.matches_request(&payload));
        let mut wrong = payload;
        wrong[8] ^= 16;
        assert!(!info.matches_request(&wrong));
        wrong = payload;
        wrong[12] ^= 16;
        assert!(!info.matches_request(&wrong));
        assert!(Info::parse(&read[..43]).is_err());
        read[6] = 5;
        assert!(Info::parse(&read).is_err());
        read[6] = 6;
        read[7] = 32;
        assert!(Info::parse(&read).is_ok());
        read[7] = 64;
        assert!(Info::parse(&read).is_err());
    }
    #[test]
    fn preset_request_rejects_mismatch_missing_heat_and_unrepresentable_values() {
        let mut draft = Draft::default();
        let data = draft.payload(0x12345678, "").unwrap();
        assert_eq!(&data[..8], &[1, 1, 5, 1, 0x78, 0x56, 0x34, 0x12]);
        assert_eq!(u32le(&data, 8).unwrap(), 1000);
        assert!(data[12..].iter().all(|v| *v == 0));
        draft.model = Model::Yq5740;
        assert!(draft.payload(1, "3 7 65 0.8 0.7 1 0.5").is_err());
        draft.node = 6;
        assert!(draft.payload(1, "").is_err());
        assert!(draft.payload(1, "3 7 65 0.8 0.7 1 0.5").is_ok());
        assert!(draft.payload(1, "3 7 NaN 0.8 0.7 1 0.5").is_err());
        assert!(draft.payload(1, "3 7 65 0.8 0.9 1 0.5").is_err());
        draft.speed_rad_s = f64::INFINITY;
        assert!(draft.payload(1, "3 7 65 0.8 0.7 1 0.5").is_err());
    }
}
