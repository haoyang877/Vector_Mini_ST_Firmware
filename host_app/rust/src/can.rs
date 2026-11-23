//! ControlCANFD 的唯一动态 SDK 边界；显式扫描或连接时按所选波特率打开通道 0。

use anyhow::{Context, Result, bail, ensure};
use libloading::Library;
use std::ffi::c_void;
use std::path::Path;
use std::ptr::NonNull;
use std::time::{Duration, Instant};

pub const ARBITRATION_RATES: [u32; 4] = [125_000, 250_000, 500_000, 1_000_000];
pub const DATA_RATES: [u32; 7] = [
    500_000, 1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000, 8_000_000,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bitrates {
    pub arbitration: u32,
    pub data: u32,
}
impl Default for Bitrates {
    fn default() -> Self {
        Self {
            arbitration: 1_000_000,
            data: 5_000_000,
        }
    }
}
impl Bitrates {
    pub fn validate(self) -> Result<()> {
        ensure!(
            ARBITRATION_RATES.contains(&self.arbitration),
            "仲裁段波特率不在支持列表内"
        );
        ensure!(
            DATA_RATES.contains(&self.data),
            "数据段波特率不在支持列表内"
        );
        Ok(())
    }
    pub fn label(self) -> String {
        format!(
            "{} / {}",
            bitrate_label(self.arbitration),
            bitrate_label(self.data)
        )
    }
}
pub fn bitrate_label(rate: u32) -> String {
    if rate >= 1_000_000 {
        format!("{} Mbit/s", rate / 1_000_000)
    } else {
        format!("{} kbit/s", rate / 1000)
    }
}

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
type DeviceInfo = unsafe extern "system" fn(*mut c_void, *mut SdkDeviceInfo) -> u32;

#[repr(C)]
#[derive(Clone, Copy)]
struct SdkDeviceInfo {
    versions: [u16; 5],
    channels: u8,
    serial: [u8; 21],
    hardware: [u8; 40],
    reserved: [u16; 4],
}

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
    info: DeviceInfo,
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
            info: unsafe { get(&library, b"ZCAN_GetDeviceInf\0")? },
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
    pub fn check_sdk(path: &Path) -> Result<()> {
        Api::load(path).map(|_| ())
    }

    pub fn adapter_name(&self) -> String {
        let mut info: SdkDeviceInfo = unsafe { std::mem::zeroed() };
        if unsafe { (self.api.info)(self.device.as_ptr(), &mut info) } != 1 {
            return "USBCANFD-200U（信息不可用）".into();
        }
        let string = |bytes: &[u8]| {
            let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
            String::from_utf8_lossy(&bytes[..end]).trim().to_owned()
        };
        let serial = string(&info.serial);
        if serial.is_empty() {
            "USBCANFD-200U".into()
        } else {
            format!("USBCANFD-200U · {serial}")
        }
    }
    pub fn open(path: &Path, device_index: u32, bitrates: Bitrates) -> Result<Self> {
        ensure!(device_index <= 15, "CAN 设备索引必须为 0..15");
        bitrates.validate()?;
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
            (port.api.set_abit, bitrates.arbitration, "仲裁速率"),
            (port.api.set_dbit, bitrates.data, "数据速率"),
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
                bail!("CAN FD 接收失败：SDK count={count}");
            }
            for item in batch.into_iter().take(count as usize) {
                let frame = item.frame;
                if frame.flags & (1 << 29) != 0 || frame.brs_esi & 2 != 0 {
                    bail!(
                        "CAN 总线错误或 error-passive：flags={:08X}, fd_flags={:02X}, length={}, data={}；停止事务",
                        frame.flags,
                        frame.brs_esi,
                        frame.length,
                        hex::encode(&frame.data[..usize::from(frame.length).min(64)])
                    );
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
        assert_eq!(std::mem::size_of::<SdkDeviceInfo>(), 80);
    }
    #[test]
    fn invalid_device_index_is_rejected_before_loading_sdk() {
        let error = ControlCanFd::open(Path::new("no-sdk.dll"), 16, Bitrates::default())
            .err()
            .unwrap();
        assert!(error.to_string().contains("索引"));
    }
    #[test]
    fn invalid_rates_are_rejected_before_loading_sdk() {
        assert!(Bitrates::default().validate().is_ok());
        assert_eq!(Bitrates::default().arbitration, 1_000_000);
        assert_eq!(Bitrates::default().data, 5_000_000);
        for rates in [
            Bitrates {
                arbitration: 123,
                data: 5_000_000,
            },
            Bitrates {
                arbitration: 500_000,
                data: 123,
            },
        ] {
            let error = ControlCanFd::open(Path::new("no-sdk.dll"), 0, rates)
                .err()
                .unwrap();
            assert!(error.to_string().contains("波特率"));
        }
        for arbitration in [500_000, 1_000_000] {
            for data in [500_000, 1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000] {
                assert!(Bitrates { arbitration, data }.validate().is_ok());
            }
        }
    }
}
