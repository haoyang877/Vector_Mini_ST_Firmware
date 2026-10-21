//! ControlCANFD 的唯一动态 SDK 边界；仅用户显式连接时打开指定设备 / 通道 0。

use anyhow::{Context, Result, bail, ensure};
use libloading::Library;
use std::ffi::c_void;
use std::path::Path;
use std::ptr::NonNull;
use std::time::{Duration, Instant};

#[repr(C)]
#[derive(Default)]
struct ChannelConfig {
    can_type: u32,
    init: [u8; 28],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CanFdFrame {
    flags: u32,
    length: u8,
    brs_esi: u8,
    reserved0: u8,
    reserved1: u8,
    data: [u8; 64],
}
impl Default for CanFdFrame {
    fn default() -> Self {
        Self {
            flags: 0,
            length: 0,
            brs_esi: 0,
            reserved0: 0,
            reserved1: 0,
            data: [0; 64],
        }
    }
}

#[repr(C)]
#[derive(Default)]
struct TxFd {
    frame: CanFdFrame,
    transmit_type: u32,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct RxFd {
    frame: CanFdFrame,
    timestamp: u64,
}

type Open = unsafe extern "system" fn(u32, u32, u32) -> *mut c_void;
type Close = unsafe extern "system" fn(*mut c_void) -> u32;
type Set = unsafe extern "system" fn(*mut c_void, u32, u32) -> u32;
type Init = unsafe extern "system" fn(*mut c_void, u32, *const ChannelConfig) -> *mut c_void;
type Start = unsafe extern "system" fn(*mut c_void) -> u32;
type Tx = unsafe extern "system" fn(*mut c_void, *const TxFd, u32) -> i32;
type Rx = unsafe extern "system" fn(*mut c_void, *mut RxFd, u32, i32) -> i32;

struct Api {
    _library: Library,
    open: Open,
    close: Close,
    set_abit: Set,
    set_dbit: Set,
    set_standard: Set,
    init: Init,
    start: Start,
    reset: Start,
    transmit: Tx,
    receive: Rx,
}

impl Api {
    fn load(path: &Path) -> Result<Self> {
        ensure!(path.is_file(), "ControlCANFD.dll 不存在");
        // 函数指针由 Library 生命周期覆盖，SDK 仅在本线程独占使用。
        let library = unsafe { Library::new(path) }.context("无法加载 ControlCANFD.dll")?;
        unsafe fn get<T: Copy>(library: &Library, name: &[u8]) -> Result<T> {
            Ok(*unsafe { library.get::<T>(name) }?)
        }
        Ok(Self {
            open: unsafe { get(&library, b"ZCAN_OpenDevice\0")? },
            close: unsafe { get(&library, b"ZCAN_CloseDevice\0")? },
            set_abit: unsafe { get(&library, b"ZCAN_SetAbitBaud\0")? },
            set_dbit: unsafe { get(&library, b"ZCAN_SetDbitBaud\0")? },
            set_standard: unsafe { get(&library, b"ZCAN_SetCANFDStandard\0")? },
            init: unsafe { get(&library, b"ZCAN_InitCAN\0")? },
            start: unsafe { get(&library, b"ZCAN_StartCAN\0")? },
            reset: unsafe { get(&library, b"ZCAN_ResetCAN\0")? },
            transmit: unsafe { get(&library, b"ZCAN_TransmitFD\0")? },
            receive: unsafe { get(&library, b"ZCAN_ReceiveFD\0")? },
            _library: library,
        })
    }
}

pub trait Transport {
    fn send(&mut self, id: u32, data: &[u8]) -> Result<()>;
    fn receive(&mut self, timeout: Duration) -> Result<Option<(u32, Vec<u8>)>>;
}

pub struct ControlCanFd {
    api: Api,
    device: NonNull<c_void>,
    channel: Option<NonNull<c_void>>,
    pending: Vec<(u32, Vec<u8>)>,
}

impl ControlCanFd {
    pub fn open(path: &Path, device_index: u32) -> Result<Self> {
        ensure!(device_index <= 15, "CAN 设备索引必须为 0..15");
        let api = Api::load(path)?;
        let device = NonNull::new(unsafe { (api.open)(41, device_index, 0) })
            .context("CAN 设备打开失败；检查驱动和端口独占")?;
        let mut port = Self {
            api,
            device,
            channel: None,
            pending: Vec::new(),
        };
        for (set, value, name) in [
            (port.api.set_abit, 1_000_000, "仲裁速率"),
            (port.api.set_dbit, 5_000_000, "数据速率"),
            (port.api.set_standard, 0, "ISO CAN FD"),
        ] {
            ensure!(
                unsafe { set(port.device.as_ptr(), 0, value) } == 1,
                "{name}配置失败"
            );
        }
        let config = ChannelConfig {
            can_type: 1,
            ..Default::default()
        };
        port.channel = NonNull::new(unsafe { (port.api.init)(port.device.as_ptr(), 0, &config) });
        ensure!(port.channel.is_some(), "CAN 通道初始化失败");
        ensure!(
            unsafe { (port.api.start)(port.channel.unwrap().as_ptr()) } == 1,
            "CAN 通道启动失败"
        );
        Ok(port)
    }

    fn channel(&self) -> Result<NonNull<c_void>> {
        self.channel.context("CAN 通道未打开")
    }
}

impl Transport for ControlCanFd {
    fn send(&mut self, id: u32, data: &[u8]) -> Result<()> {
        ensure!(
            [20, 24, 32, 48, 64].contains(&data.len()),
            "CAN FD 帧长度非法"
        );
        let mut tx = TxFd::default();
        tx.frame.flags = (id & 0x1fff_ffff) | (1 << 31);
        tx.frame.length = data.len() as u8;
        tx.frame.brs_esi = 1;
        tx.frame.data[..data.len()].copy_from_slice(data);
        ensure!(
            unsafe { (self.api.transmit)(self.channel()?.as_ptr(), &tx, 1) } == 1,
            "CAN FD 发送失败；停止事务"
        );
        Ok(())
    }

    fn receive(&mut self, timeout: Duration) -> Result<Option<(u32, Vec<u8>)>> {
        let deadline = Instant::now() + timeout;
        loop {
            if !self.pending.is_empty() {
                return Ok(Some(self.pending.remove(0)));
            }
            let mut batch = [RxFd::default(); 32];
            let count =
                unsafe { (self.api.receive)(self.channel()?.as_ptr(), batch.as_mut_ptr(), 32, 0) };
            if !(0..=32).contains(&count) {
                bail!("CAN FD 接收失败");
            }
            for item in batch.into_iter().take(count as usize) {
                let frame = item.frame;
                if frame.flags & (1 << 29) != 0 || frame.brs_esi & 2 != 0 {
                    bail!("CAN 总线错误或 error-passive；停止升级");
                }
                if frame.length > 64 {
                    bail!("SDK 返回非法帧长度");
                }
                if frame.flags & (1 << 31) != 0
                    && frame.flags & (1 << 30) == 0
                    && frame.brs_esi & 1 != 0
                {
                    self.pending.push((
                        frame.flags & 0x1fff_ffff,
                        frame.data[..frame.length as usize].to_vec(),
                    ));
                }
            }
            if Instant::now() >= deadline && self.pending.is_empty() {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

impl Drop for ControlCanFd {
    fn drop(&mut self) {
        if let Some(channel) = self.channel.take() {
            unsafe { (self.api.reset)(channel.as_ptr()) };
        }
        unsafe { (self.api.close)(self.device.as_ptr()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sdk_layout_matches_reference_adapter() {
        assert_eq!(std::mem::size_of::<ChannelConfig>(), 32);
        assert_eq!(std::mem::size_of::<CanFdFrame>(), 72);
        assert_eq!(std::mem::size_of::<TxFd>(), 76);
        assert_eq!(std::mem::size_of::<RxFd>(), 80);
    }
    #[test]
    fn invalid_device_index_is_rejected_before_loading_sdk() {
        let error = ControlCanFd::open(Path::new("no-sdk.dll"), 16)
            .err()
            .unwrap();
        assert!(error.to_string().contains("索引"));
    }
}
