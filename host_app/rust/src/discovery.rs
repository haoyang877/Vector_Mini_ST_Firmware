//! 显式扫描：仅发送 GET_INFO；不发送运动、STOP、参数写入或升级命令。
use crate::can::{Bitrates, ControlCanFd, Transport};
use crate::target::Target;
use crate::wire::{self, u16le, u32le};
use anyhow::{Result, ensure};
use std::path::Path;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc::Sender,
};
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct Device {
    pub adapter_index: u32,
    pub adapter: String,
    pub node: u8,
    pub uid: String,
    pub role: u8,
    pub target_name: Option<String>,
}

pub enum ScanEvent {
    Progress(String),
    Adapter(u32, String),
    Device(Device),
    Finished(String),
}

fn query(port: &mut impl Transport, node: u8, page: u16, sequence: u16) -> Result<Option<Vec<u8>>> {
    let mut request = [0u8; 12];
    request[4..8].copy_from_slice(&u32::from(sequence).to_le_bytes());
    request[8..10].copy_from_slice(&page.to_le_bytes());
    let frames = wire::encode(node, 1, sequence, &request)?;
    port.send(frames[0].0, &frames[0].1)?;
    let deadline = Instant::now() + Duration::from_millis(100);
    while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        let Some((id, data)) = port.receive(remaining)? else {
            break;
        };
        if let Ok(reply) = wire::decode(node, 1, sequence, id, &data) {
            return Ok(Some(reply));
        }
    }
    Ok(None)
}

fn identify(
    port: &mut impl Transport,
    index: u32,
    adapter: &str,
    node: u8,
) -> Result<Option<Device>> {
    let Some(page0) = query(port, node, 0, u16::from(node) * 3)? else {
        return Ok(None);
    };
    if page0.len() != 32 || page0[..12].iter().all(|b| *b == 0) {
        return Ok(None);
    }
    let Some(page1) = query(port, node, 1, u16::from(node) * 3 + 1)? else {
        return Ok(None);
    };
    if page1.len() != 32 || page1[21] != node || ![1, 2].contains(&page1[22]) {
        return Ok(None);
    }
    let Some(page2) = query(port, node, 2, u16::from(node) * 3 + 2)? else {
        return Ok(None);
    };
    if page2.len() != 32 {
        return Ok(None);
    }
    let product = u32le(&page0, 12)?;
    let hardware = u32::from(u16le(&page0, 16)?);
    let mut target_name = None;
    for name in ["vector_mini_st_g431", "yg_esc_v1_at32f456"] {
        let target = Target::named(name)?;
        if target.product == product
            && target.hardware == hardware
            && target.mcu == u32le(&page2, 0)?
            && target.layout == u32le(&page2, 4)?
            && target.abi == u32le(&page2, 8)?
            && target.schema == u32le(&page2, 12)?
        {
            target_name = Some(name.to_owned());
        }
    }
    Ok(Some(Device {
        adapter_index: index,
        adapter: adapter.into(),
        node,
        uid: hex::encode_upper(&page0[..12]),
        role: page1[22],
        target_name,
    }))
}

pub fn scan(path: &Path, bitrates: Bitrates, cancel: Arc<AtomicBool>, events: Sender<ScanEvent>) {
    let result = (|| -> Result<String> {
        bitrates.validate()?;
        ControlCanFd::check_sdk(path)?;
        let mut adapters = 0;
        let mut devices = 0;
        for index in 0..=15 {
            if cancel.load(Ordering::Relaxed) {
                return Ok("扫描已取消。".into());
            }
            let Ok(mut port) = ControlCanFd::open(path, index, bitrates) else {
                continue;
            };
            adapters += 1;
            let adapter = port.adapter_name();
            let _ = events.send(ScanEvent::Adapter(index, adapter.clone()));
            for node in 1..=254 {
                if cancel.load(Ordering::Relaxed) {
                    return Ok("扫描已取消。".into());
                }
                if node == 1 || node % 10 == 0 {
                    let _ = events.send(ScanEvent::Progress(format!(
                        "适配器 {index} · 扫描节点 {node}/254"
                    )));
                }
                if let Some(device) = identify(&mut port, index, &adapter, node)? {
                    devices += 1;
                    let _ = events.send(ScanEvent::Device(device));
                }
            }
        }
        ensure!(
            adapters > 0,
            "未找到可打开的 USBCANFD-200U；检查连接、驱动、所选波特率或端口占用"
        );
        Ok(format!(
            "扫描完成：{adapters} 个可用适配器，{devices} 个设备；{}。超时设备可能未被发现。",
            bitrates.label()
        ))
    })();
    let _ = events.send(ScanEvent::Finished(match result {
        Ok(message) => message,
        Err(error) => format!("扫描失败：{error:#}"),
    }));
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    struct Fake {
        replies: VecDeque<(u32, Vec<u8>)>,
        requests: Vec<(u8, u16)>,
        zero_uid: bool,
    }
    impl Transport for Fake {
        fn send(&mut self, id: u32, data: &[u8]) -> Result<()> {
            let node = ((id >> 8) & 255) as u8;
            let kind = u16::from_le_bytes([data[6], data[7]]);
            let seq = u16::from_le_bytes([data[8], data[9]]);
            let page = u16::from_le_bytes([data[24], data[25]]);
            self.requests.push((node, kind));
            if node == 7 && kind == 1 {
                let t = Target::named("yg_esc_v1_at32f456")?;
                let mut payload = [0u8; 32];
                match page {
                    0 => {
                        if !self.zero_uid {
                            payload[..12].fill(0x12);
                        }
                        payload[12..16].copy_from_slice(&t.product.to_le_bytes());
                        payload[16..18].copy_from_slice(&(t.hardware as u16).to_le_bytes());
                    }
                    1 => {
                        payload[21] = node;
                        payload[22] = 1;
                    }
                    2 => {
                        for (i, value) in [t.mcu, t.layout, t.abi, t.schema].iter().enumerate() {
                            payload[i * 4..i * 4 + 4].copy_from_slice(&value.to_le_bytes());
                        }
                    }
                    _ => unreachable!(),
                }
                self.replies
                    .push_back(wire::test_reply(node, kind, seq, &payload));
            }
            Ok(())
        }
        fn receive(&mut self, _timeout: Duration) -> Result<Option<(u32, Vec<u8>)>> {
            Ok(self.replies.pop_front())
        }
    }
    #[test]
    fn identifies_matching_target_using_only_get_info() {
        let mut fake = Fake {
            replies: VecDeque::new(),
            requests: vec![],
            zero_uid: false,
        };
        assert!(identify(&mut fake, 0, "adapter", 6).unwrap().is_none());
        let device = identify(&mut fake, 0, "adapter", 7).unwrap().unwrap();
        assert_eq!(device.target_name.as_deref(), Some("yg_esc_v1_at32f456"));
        assert_eq!(device.uid, "12".repeat(12));
        assert_eq!(device.node, 7);
        assert!(fake.requests.iter().all(|(_, kind)| *kind == 1));
    }
    #[test]
    fn rejects_zero_uid() {
        let mut fake = Fake {
            replies: VecDeque::new(),
            requests: vec![],
            zero_uid: true,
        };
        assert!(identify(&mut fake, 0, "adapter", 7).unwrap().is_none());
        assert_eq!(fake.requests.len(), 1);
    }
}
