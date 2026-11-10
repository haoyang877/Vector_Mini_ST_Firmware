//! 标定票据与设备阶段；不伪造百分比或质量评分。
use crate::wire::{u16le, u32le};
use anyhow::{Result, ensure};
use std::time::Instant;

pub const FEATURE: u32 = 1 << 15;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Lut = 1,
    ElectricalZero = 2,
    MechanicalZero = 6,
}

impl Kind {
    pub const ALL: [Self; 3] = [Self::Lut, Self::ElectricalZero, Self::MechanicalZero];
    pub fn name(self) -> &'static str {
        match self {
            Self::Lut => "角度传感器 LUT",
            Self::ElectricalZero => "电气零位",
            Self::MechanicalZero => "机械零位",
        }
    }
    pub fn description(self) -> &'static str {
        match self {
            Self::Lut => "设备执行角度传感器线性化；带 Hall 的装配会继续联合标定，完成后自动保存。",
            Self::ElectricalZero => "设备执行电气零位对齐，完成后自动保存；与机械零位置不同。",
            Self::MechanicalZero => "以当前机械位置置零并保存；设备须满足输出轴与编码器标定要求。",
        }
    }
    pub fn start_payload(self) -> [u8; 4] {
        [self as u8, 0, 0, 0]
    }
}

#[derive(Clone, Debug)]
pub struct Job {
    pub ticket: u32,
    pub kind: Kind,
    pub phase: u8,
    pub effects: u8,
    pub detail: u16,
    pub started: Instant,
    pub updated: Instant,
}

impl Job {
    pub fn followup(&self, data: &[u8]) -> Result<Self> {
        let mut next = Self::parse(data)?;
        ensure!(
            next.ticket == self.ticket && next.kind == self.kind,
            "标定票据或任务种类变化"
        );
        ensure!(self.effects != 2 || next.effects == 2, "标定保存事实回退");
        ensure!(self.phase != 3 || next.phase != 1, "标定取消状态回退");
        next.started = self.started;
        Ok(next)
    }
    pub fn parse(data: &[u8]) -> Result<Self> {
        ensure!(data.len() == 12, "标定回复长度错误");
        ensure!(
            u16le(data, 0)? <= 1,
            "标定请求被拒：{}/{}",
            u16le(data, 0)?,
            u16le(data, 2)?
        );
        let kind = match u16le(data, 8)? {
            1 => Kind::Lut,
            2 => Kind::ElectricalZero,
            6 => Kind::MechanicalZero,
            value => anyhow::bail!("标定种类非法：{value}"),
        };
        let ticket = u32le(data, 4)?;
        ensure!(
            ticket != 0 && (1..=5).contains(&data[10]) && data[11] <= 2,
            "标定票据或状态非法"
        );
        Ok(Self {
            ticket,
            kind,
            phase: data[10],
            effects: data[11],
            detail: u16le(data, 2)?,
            started: Instant::now(),
            updated: Instant::now(),
        })
    }
    pub fn active(&self) -> bool {
        matches!(self.phase, 1 | 3)
    }
    pub fn state(&self) -> &'static str {
        match self.phase {
            1 => "标定进行中",
            2 => "标定完成",
            3 => "正在请求取消",
            4 => "已取消",
            _ => "标定失败",
        }
    }
    pub fn saved(&self) -> &'static str {
        match self.effects {
            2 => "设备已确认保存",
            1 => "尚未保存",
            _ => "保存状态未知",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn replies_preserve_real_terminal_state_and_saved_effects() {
        for kind in Kind::ALL {
            for phase in 1..=5 {
                let bytes = [0, 0, 0, 0, 1, 0, 0, 0, kind as u8, 0, phase, 2];
                let job = Job::parse(&bytes).unwrap();
                assert_eq!(job.kind, kind);
                assert_eq!(job.active(), matches!(phase, 1 | 3));
                assert_eq!(job.effects, 2);
            }
        }
        for bytes in [
            vec![0; 4],
            vec![0; 12],
            vec![2, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1],
        ] {
            assert!(Job::parse(&bytes).is_err());
        }
    }
}
