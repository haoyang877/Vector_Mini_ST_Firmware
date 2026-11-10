//! 四种调试模式的线路映射、单位与主机调试包络；不访问设备。
use anyhow::{Result, ensure};

pub const MAX_SPEED_RAD_S: f64 = 10.0 * std::f64::consts::PI;
pub const MAX_IQ_A: f64 = 6.0;
pub const MAX_VQ_V: f64 = 6.0;
pub const HOST_POSITION_SESSION_FEATURE: u32 = 1 << 18;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ControlMode {
    Voltage,
    Torque,
    #[default]
    Speed,
    Position,
}

impl ControlMode {
    pub const ALL: [Self; 4] = [Self::Voltage, Self::Torque, Self::Speed, Self::Position];
    pub fn wire(self) -> u8 {
        match self {
            Self::Voltage => 6,
            Self::Torque => 1,
            Self::Speed => 2,
            Self::Position => 3,
        }
    }
    pub fn index(self) -> usize {
        match self {
            Self::Voltage => 0,
            Self::Torque => 1,
            Self::Speed => 2,
            Self::Position => 3,
        }
    }
    pub fn start_payload(self) -> [u8; 4] {
        [self.wire(), u8::from(self == Self::Position), 0, 0]
    }
    pub fn name(self) -> &'static str {
        match self {
            Self::Voltage => "电压 Vq",
            Self::Torque => "力矩 Iq",
            Self::Speed => "速度",
            Self::Position => "位置",
        }
    }
    pub fn unit(self) -> &'static str {
        match self {
            Self::Voltage => "V",
            Self::Torque => "A",
            Self::Speed => "rad/s",
            Self::Position => "rad",
        }
    }
    pub fn description(self) -> &'static str {
        match self {
            Self::Voltage => "直接给定 q 轴电压，使用编码器角度。",
            Self::Torque => "给定 q 轴电流 Iq，电流单位为 A。",
            Self::Speed => "给定速度，设备执行速度闭环。",
            Self::Position => "给定绝对位置；启动时保持当前位置。",
        }
    }
    pub fn range(self) -> std::ops::RangeInclusive<f64> {
        match self {
            Self::Voltage => -MAX_VQ_V..=MAX_VQ_V,
            Self::Torque => -MAX_IQ_A..=MAX_IQ_A,
            Self::Speed => 0.0..=MAX_SPEED_RAD_S,
            Self::Position => -2_147_483.647..=2_147_483.647,
        }
    }
    pub fn limit_text(self) -> &'static str {
        match self {
            Self::Voltage => "Vq：−6…6 V；设备电压限值仍然生效。",
            Self::Torque => "Iq：−6…6 A；设备电流与热限值仍然生效。",
            Self::Speed => "速度：0…31.416 rad/s（0…300 rpm）。",
            Self::Position => "上位机位置会话不限制位置步长与行程；需要配套固件支持。",
        }
    }
    pub fn channels(self) -> [usize; 3] {
        match self {
            Self::Voltage => [3, 1, 2],
            Self::Torque => [2, 1, 0],
            Self::Speed => [1, 2, 0],
            Self::Position => [0, 1, 2],
        }
    }
    pub fn target_payload(self, value: f64, current_position: Option<f64>) -> Result<[u8; 20]> {
        ensure!(
            value.is_finite() && self.range().contains(&value),
            "{}目标超出主机调试范围",
            self.name()
        );
        let offset = match self {
            Self::Position => 4,
            Self::Speed => 8,
            Self::Torque => 12,
            Self::Voltage => 16,
        };
        let limit = (*self.range().end() * 1000.0).floor();
        let encoded = (value * 1000.0).round().clamp(-limit, limit) as i32;
        if self == Self::Position {
            current_position
                .filter(|value| value.is_finite())
                .ok_or_else(|| anyhow::anyhow!("位置控制需要有效的实测位置"))?;
        }
        let mut payload = [0u8; 20];
        payload[0] = self.wire();
        payload[offset..offset + 4].copy_from_slice(&encoded.to_le_bytes());
        Ok(payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn position_start_uses_explicit_scoped_wire_flag() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../../tests/fixtures/protocol/yg_protocol_host_position_session_v1_vectors.json"
        ))
        .unwrap();
        assert_eq!(
            hex::encode(ControlMode::Position.start_payload()),
            fixture["host_start_payload_hex"]
        );
        assert_eq!(
            HOST_POSITION_SESSION_FEATURE,
            fixture["feature_mask"].as_u64().unwrap() as u32
        );
        for mode in [
            ControlMode::Voltage,
            ControlMode::Torque,
            ControlMode::Speed,
        ] {
            assert_eq!(mode.start_payload()[1], 0);
        }
    }
    #[test]
    fn targets_use_only_the_selected_mode_field() {
        for (mode, offset, value, position) in [
            (ControlMode::Voltage, 16, -1.25, None),
            (ControlMode::Torque, 12, -0.5, None),
            (ControlMode::Speed, 8, 12.5, None),
            (ControlMode::Position, 4, 1.25, Some(1.2)),
        ] {
            let payload = mode.target_payload(value, position).unwrap();
            assert_eq!(payload[0], mode.wire());
            assert_eq!(
                i32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()),
                (value * 1000.0).round() as i32
            );
            assert!(
                payload
                    .iter()
                    .enumerate()
                    .skip(1)
                    .all(|(i, value)| (offset..offset + 4).contains(&i) || *value == 0)
            );
        }
    }
    #[test]
    fn invalid_values_are_rejected_and_large_position_steps_are_allowed() {
        for mode in ControlMode::ALL {
            assert!(mode.target_payload(f64::NAN, Some(0.0)).is_err());
        }
        assert!(ControlMode::Voltage.target_payload(6.001, None).is_err());
        assert!(ControlMode::Torque.target_payload(-6.001, None).is_err());
        assert!(ControlMode::Position.target_payload(0.0, None).is_err());
        // 不再按相对当前位置的10°步长拒绝；保留有限反馈与线路整数范围。
        assert!(
            ControlMode::Position
                .target_payload(0.17451, Some(0.0))
                .is_ok()
        );
        assert!(
            ControlMode::Position
                .target_payload(0.174, Some(0.0))
                .is_ok()
        );
        assert!(ControlMode::Position.target_payload(1.0, Some(0.0)).is_ok());
        for target in [-100.0, 100.0, -2_147_483.647, 2_147_483.647] {
            assert!(
                ControlMode::Position
                    .target_payload(target, Some(0.0))
                    .is_ok()
            );
        }
        assert!(
            ControlMode::Position
                .target_payload(2_147_483.648, Some(0.0))
                .is_err()
        );
        assert!(
            ControlMode::Position
                .target_payload(1.0, Some(f64::NAN))
                .is_err()
        );
    }
}
