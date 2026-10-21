//! v3 APP 升级事务：同一 UID、目标几何、nonce/session、写入偏移和确认均须核对。

use crate::can::Transport;
use crate::package::UpgradePackage;
use crate::wire::{self, u16le, u32le, u64le};
use anyhow::{Result, bail, ensure};
use std::time::{Duration, Instant};

pub struct UpgradeClient<'a> {
    port: &'a mut dyn Transport,
    node: u8,
    uid: [u8; 12],
    sequence: u16,
    binding: Option<(u64, u32)>,
}

impl<'a> UpgradeClient<'a> {
    pub fn new(port: &'a mut dyn Transport, node: u8, uid: [u8; 12]) -> Self {
        Self {
            port,
            node,
            uid,
            sequence: 0,
            binding: None,
        }
    }

    fn request(&mut self, kind: u16, payload: &[u8], timeout: Duration) -> Result<Vec<u8>> {
        self.sequence = self.sequence.wrapping_add(1).max(1);
        self.request_sequence(kind, payload, self.sequence, timeout)
    }

    fn request_sequence(
        &mut self,
        kind: u16,
        payload: &[u8],
        sequence: u16,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let frames = wire::encode(self.node, kind, sequence, payload)?;
        let reply_sequence = if frames.len() == 1 {
            sequence
        } else {
            (frames.len() - 1) as u16
        };
        for _ in 0..4 {
            for (id, frame) in &frames {
                self.port.send(*id, frame)?;
            }
            let deadline = Instant::now() + timeout;
            while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
                let Some((id, frame)) = self.port.receive(remaining)? else {
                    break;
                };
                if let Ok(response) = wire::decode(self.node, kind, reply_sequence, id, &frame) {
                    return Ok(response);
                }
            }
        }
        bail!("Type{kind} 回复超时；停止升级并检查 Loader 状态")
    }

    fn identify(&mut self) -> Result<([u8; 32], [u8; 32], [u32; 8])> {
        let mut pages = [[0u8; 32]; 3];
        for (page, buffer) in pages.iter_mut().enumerate() {
            let mut request = [0u8; 12];
            request[4..8].copy_from_slice(&(page as u32).to_le_bytes());
            request[8..10].copy_from_slice(&(page as u16).to_le_bytes());
            let reply = self.request(1, &request, Duration::from_millis(250))?;
            ensure!(reply.len() == 32, "设备身份页长度错误");
            buffer.copy_from_slice(&reply);
        }
        ensure!(pages[0][..12] == self.uid, "设备 UID 变化或不匹配");
        ensure!(
            pages[1][21] == self.node && [1, 2].contains(&pages[1][22]),
            "节点或角色不匹配"
        );
        let mut target = [0u32; 8];
        for (index, value) in target.iter_mut().enumerate() {
            *value = u32le(&pages[2], index * 4)?;
        }
        Ok((pages[0], pages[1], target))
    }

    fn check_target(
        &self,
        package: &UpgradePackage,
        page0: &[u8; 32],
        page1: &[u8; 32],
        actual: &[u32; 8],
    ) -> Result<()> {
        let product = u32le(page0, 12)?;
        let hardware = u16le(page0, 16)? as u32;
        let api = u16le(page0, 18)?;
        let expected_product = u32::from_be_bytes(package.manifest[16..20].try_into()?);
        let expected_hardware = u32::from_be_bytes(package.manifest[20..24].try_into()?);
        ensure!(
            *actual == package.target
                && (product, hardware, api) == (expected_product, expected_hardware, 3)
                && u32le(page0, 24)? == package.loader_version
                && u32le(page1, 28)? == 0x5359_3549,
            "设备与包的产品、Loader、布局或参数 ABI 不配套"
        );
        Ok(())
    }

    fn status(&mut self, package: &UpgradePackage, kind: u16, extra: &[u8]) -> Result<(u8, u32)> {
        let mut payload = Vec::with_capacity(12 + extra.len());
        if let Some((nonce, session)) = self.binding {
            payload.extend_from_slice(&nonce.to_le_bytes());
            payload.extend_from_slice(&session.to_le_bytes());
        }
        payload.extend_from_slice(extra);
        let response = self.request(kind, &payload, Duration::from_millis(250))?;
        ensure!(response.len() == 40, "升级状态长度错误");
        let result = u16le(&response, 0)?;
        let detail = u16le(&response, 2)?;
        ensure!(
            result <= 1,
            "升级被拒绝：Type{kind} result={result} detail={detail}"
        );
        ensure!(
            response[17] & 6 == 6 && response[17] & !7 == 0,
            "升级能力缺失"
        );
        if let Some((nonce, session)) = self.binding {
            ensure!(
                u64le(&response, 4)? == nonce
                    && u32le(&response, 12)? == session
                    && u32le(&response, 24)? == package.image.len() as u32
                    && u32le(&response, 28)? == package.token
                    && u32le(&response, 32)? == package.version,
                "升级会话、镜像或 offset 绑定不匹配"
            );
        }
        Ok((response[16], u32le(&response, 20)?))
    }

    pub fn run(
        &mut self,
        package: &UpgradePackage,
        mut progress: impl FnMut(usize, usize),
    ) -> Result<()> {
        // 无副作用 STATUS 可在 Loader 50ms 窗口先驻留；旧 APP 超时后走 Type56。
        let _ = self.request(53, &[], Duration::from_millis(25));
        let (mut page0, mut page1, mut target) = self.identify()?;
        self.check_target(package, &page0, &page1, &target)?;
        let nonce = rand::random::<u64>().max(1);
        if page1[22] == 1 {
            self.sequence = self.sequence.wrapping_add(1).max(1);
            let enter_sequence = self.sequence;
            let deadline = Instant::now() + Duration::from_secs(1);
            loop {
                match self.request_sequence(
                    56,
                    &nonce.to_le_bytes(),
                    enter_sequence,
                    Duration::from_millis(250),
                ) {
                    Ok(response) => {
                        ensure!(
                            response.len() == 4 && u16le(&response, 0)? == 1,
                            "APP 拒绝停机交接"
                        );
                        if u16le(&response, 2)? == 0 {
                            break;
                        }
                        ensure!(Instant::now() < deadline, "STOP/关相交接未完成；需现场检查");
                        std::thread::sleep(Duration::from_millis(10));
                    }
                    Err(error) => {
                        (page0, page1, target) = self.identify()?;
                        self.check_target(package, &page0, &page1, &target)?;
                        if page1[22] != 2 {
                            return Err(error);
                        }
                        break;
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(100));
            (page0, page1, target) = self.identify()?;
            self.check_target(package, &page0, &page1, &target)?;
            ensure!(page1[22] == 2, "交接后未识别到 Loader");
        }
        let mut begin = Vec::with_capacity(104);
        begin.extend_from_slice(&nonce.to_le_bytes());
        begin.extend_from_slice(&package.manifest);
        let response = self.request(50, &begin, Duration::from_millis(250))?;
        ensure!(
            response.len() == 40 && u16le(&response, 0)? <= 1,
            "Loader 拒绝 BEGIN"
        );
        let session = u32le(&response, 12)?;
        let block = u16le(&response, 18)? as usize;
        ensure!(
            u64le(&response, 4)? == nonce
                && session != 0
                && (8..=128).contains(&block)
                && block.is_multiple_of(8),
            "BEGIN 会话或块大小非法"
        );
        self.binding = Some((nonce, session));
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.status(package, 53, &[])?.0 != 3 {
            ensure!(Instant::now() < deadline, "APP 槽擦除未完成");
            std::thread::sleep(Duration::from_millis(10));
        }
        for offset in (0..package.image.len()).step_by(block) {
            let data = &package.image[offset..(offset + block).min(package.image.len())];
            let mut payload = Vec::with_capacity(6 + data.len());
            payload.extend_from_slice(&(offset as u32).to_le_bytes());
            payload.extend_from_slice(&(data.len() as u16).to_le_bytes());
            payload.extend_from_slice(data);
            let (phase, written) = self.status(package, 51, &payload)?;
            ensure!(
                phase == 3 && written == (offset + data.len()) as u32,
                "写入偏移异常；停止下载"
            );
            progress(written as usize, package.image.len());
        }
        self.status(package, 52, &[])?;
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.status(package, 53, &[])?.0 != 5 {
            ensure!(Instant::now() < deadline, "完整镜像 CRC 校验未完成");
            std::thread::sleep(Duration::from_millis(10));
        }
        let _ = self.status(package, 54, &[]); // 只允许随后同 UID 的新 APP 证明激活成功。
        self.binding = None;
        std::thread::sleep(Duration::from_millis(100));
        let deadline = Instant::now() + Duration::from_secs(4);
        while Instant::now() < deadline {
            let (page0, page1, target) = self.identify()?;
            self.check_target(package, &page0, &page1, &target)?;
            ensure!(
                page1[22] == 1
                    && u32le(&page0, 20)? == package.version
                    && u32le(&page1, 24)? == package.crc32,
                "激活后未读到配套新 APP"
            );
            let mut confirm = Vec::with_capacity(8);
            confirm.extend_from_slice(&package.version.to_le_bytes());
            confirm.extend_from_slice(&package.crc32.to_le_bytes());
            if self.request(57, &confirm, Duration::from_millis(250))? == [0; 4] {
                let (_, confirmed, target) = self.identify()?;
                self.check_target(package, &page0, &confirmed, &target)?;
                if confirmed[22..24] == [1, 1] {
                    return Ok(());
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        bail!("新 APP 未完成持久确认，保持停机")
    }
}
