//! 公司 CAN FD 帧；严格校验来源、长度、填充和双 CRC。

use anyhow::{Result, bail, ensure};

pub const SOURCE: u8 = 2;
pub const DLC: [usize; 5] = [20, 24, 32, 48, 64];

fn crc8(data: &[u8]) -> u8 {
    let mut value = 0u8;
    for &byte in data {
        value ^= byte;
        for _ in 0..8 {
            value = (value << 1) ^ if value & 0x80 != 0 { 0x9b } else { 0 };
        }
    }
    value
}

fn crc16(data: &[u8]) -> u16 {
    let mut value = 0xffffu16;
    for &byte in data {
        value ^= (byte as u16) << 8;
        for _ in 0..8 {
            value = (value << 1) ^ if value & 0x8000 != 0 { 0xbaad } else { 0 };
        }
    }
    value
}

pub fn can_id(source: u8, destination: u8) -> u32 {
    (3 << 26) | (0xef << 16) | ((destination as u32) << 8) | source as u32
}

pub fn encode(node: u8, kind: u16, sequence: u16, payload: &[u8]) -> Result<Vec<(u32, Vec<u8>)>> {
    ensure!(
        node != 0 && node != 255 && payload.len() <= 146,
        "目标或消息长度非法"
    );
    let chunks: Vec<&[u8]> = if payload.is_empty() {
        vec![&[]]
    } else {
        payload.chunks(46).collect()
    };
    chunks
        .iter()
        .enumerate()
        .map(|(index, chunk)| {
            let flags = if chunks.len() == 1 {
                0x20
            } else if index == 0 {
                0x40
            } else if index + 1 == chunks.len() {
                0xe0
            } else {
                0x80
            };
            let seq = if chunks.len() == 1 {
                sequence
            } else {
                index as u16
            };
            let mut frame = Vec::with_capacity(64);
            frame.extend_from_slice(&0xa55au16.to_le_bytes());
            frame.extend_from_slice(&[1, flags, SOURCE, node]);
            frame.extend_from_slice(&kind.to_le_bytes());
            frame.extend_from_slice(&seq.to_le_bytes());
            frame.extend_from_slice(&(chunk.len() as u32).to_le_bytes());
            frame.push(0);
            frame.push(crc8(&frame));
            frame.extend_from_slice(chunk);
            frame.extend_from_slice(&crc16(&frame).to_le_bytes());
            let dlc = DLC
                .iter()
                .copied()
                .find(|&size| size >= frame.len())
                .ok_or_else(|| anyhow::anyhow!("帧长度超过 CAN FD 上限"))?;
            frame.resize(dlc, 0);
            Ok((can_id(SOURCE, node), frame))
        })
        .collect()
}

pub fn decode(node: u8, kind: u16, sequence: u16, id: u32, data: &[u8]) -> Result<Vec<u8>> {
    ensure!(id == can_id(node, SOURCE), "回复 CAN ID 不匹配");
    ensure!(DLC.contains(&data.len()), "非法 CAN FD 长度");
    ensure!(data.len() >= 18, "帧过短");
    ensure!(
        u16::from_le_bytes([data[0], data[1]]) == 0xa55a,
        "帧魔数错误"
    );
    ensure!(
        data[2..6] == [1, 0x10, node, SOURCE]
            && u16::from_le_bytes([data[6], data[7]]) == kind
            && u16::from_le_bytes([data[8], data[9]]) == sequence
            && data[14] == 0
            && data[15] == crc8(&data[..15]),
        "回复身份、序号或头部 CRC 不匹配"
    );
    let length = u32::from_le_bytes(data[10..14].try_into()?) as usize;
    let end = 16 + length;
    if length > 46 || end + 2 > data.len() {
        bail!("回复载荷长度非法");
    }
    let expected = DLC.iter().copied().find(|&size| size >= end + 2);
    ensure!(expected == Some(data.len()), "CAN FD DLC 与载荷不匹配");
    ensure!(
        u16::from_le_bytes(data[end..end + 2].try_into()?) == crc16(&data[..end])
            && data[end + 2..].iter().all(|&value| value == 0),
        "回复 CRC 或填充错误"
    );
    Ok(data[16..end].to_vec())
}

pub fn u16le(data: &[u8], offset: usize) -> Result<u16> {
    Ok(u16::from_le_bytes(data[offset..offset + 2].try_into()?))
}
pub fn u32le(data: &[u8], offset: usize) -> Result<u32> {
    Ok(u32::from_le_bytes(data[offset..offset + 4].try_into()?))
}
pub fn u64le(data: &[u8], offset: usize) -> Result<u64> {
    Ok(u64::from_le_bytes(data[offset..offset + 8].try_into()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_and_rejection() {
        let mut frame = encode(1, 53, 7, &[0; 40]).unwrap()[0].1.clone();
        frame[3] = 0x10;
        frame[4] = 1;
        frame[5] = SOURCE;
        frame[15] = crc8(&frame[..15]);
        let crc = crc16(&frame[..56]);
        frame[56..58].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            decode(1, 53, 7, can_id(1, SOURCE), &frame).unwrap(),
            vec![0; 40]
        );
        frame[20] = 1;
        assert!(decode(1, 53, 7, can_id(1, SOURCE), &frame).is_err());
    }

    #[test]
    fn begin_fragments_are_bounded() {
        let frames = encode(1, 50, 42, &[0; 104]).unwrap();
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].1[3], 0x40);
        assert_eq!(frames[2].1[3], 0xe0);
    }
}
