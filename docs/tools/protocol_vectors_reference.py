"""Reference wire vectors for the company CAN FD protocol, not MCU firmware.

Run with Python 3. The two CRC computations use different formulations.
Outputs the adjacent JSON fixture and refreshes Appendix A in the design spec.
"""
from pathlib import Path
import json
import struct

DOCS = Path(__file__).resolve().parent.parent


def crc_shift(data, width, poly, initial):
    value = initial
    mask = (1 << width) - 1
    for byte in data:
        value ^= byte << (width - 8)
        for _ in range(8):
            value = ((value << 1) ^ (poly if value & (1 << (width - 1)) else 0)) & mask
    return value


def crc_division(data, width, poly, initial):
    dividend = (initial << (8 * len(data))) ^ (int.from_bytes(data, 'big') << width)
    divisor = (1 << width) | poly
    while dividend.bit_length() > width:
        dividend ^= divisor << (dividend.bit_length() - divisor.bit_length())
    return dividend


def crc(data, width, poly, initial):
    result = crc_shift(data, width, poly, initial)
    assert result == crc_division(data, width, poly, initial)
    return result


def can_id(priority, pf, dst, src):
    return (priority << 26) | (pf << 16) | (dst << 8) | src


def frame(msgtype, seq, payload, flags=0x20, src=2, dst=3):
    header = struct.pack('<HBBBBHHIB', 0xA55A, 1, flags, src, dst, msgtype, seq, len(payload), 0)
    assert len(header) == 15
    header += bytes([crc(header, 8, 0x9B, 0)])
    data = header + payload
    return data + struct.pack('<H', crc(data, 16, 0xBAAD, 0xFFFF))


def validate(data):
    if len(data) < 18:
        return False
    length = struct.unpack_from('<I', data, 10)[0]
    return (data[:3] == bytes.fromhex('5A A5 01')
            and length + 18 == len(data)
            and crc(data[:15], 8, 0x9B, 0) == data[15]
            and crc(data[:-2], 16, 0xBAAD, 0xFFFF) == struct.unpack_from('<H', data, len(data) - 2)[0])


def hexbytes(data):
    return data.hex(' ').upper()


def fd_frames(msgtype, seq, payload, flags=0x20, src=2, dst=3, priority=6):
    if msgtype in {120, 121, 122, 123}:
        raise ValueError('retired message type')
    if len(payload) > 46 and msgtype in {3, 100, 103, 104, 105, 113, 124, 131, 132, 183, 184}:
        raise ValueError('real-time message cannot be fragmented')
    chunks = [payload[i:i+46] for i in range(0, len(payload), 46)] or [b'']
    records = []
    for index, chunk in enumerate(chunks):
        fragment = len(chunks) > 1
        wireflags = flags
        if fragment:
            wireflags |= 0x40 if index == 0 else (0xC0 if index == len(chunks)-1 else 0x80)
            if index != len(chunks)-1:
                wireflags &= ~0x20
        encoded = frame(msgtype, index if fragment else seq, chunk, wireflags, src, dst)
        physical_len = next(n for n in [20, 24, 32, 48, 64] if n >= len(encoded))
        padded = encoded.ljust(physical_len, b'\x00')
        records.append({'can_id': f'0x{can_id(priority, 0xEF, dst, src):08X}',
                        'extended_id': True, 'fdf': True, 'brs': True,
                        'logical_frame_bytes': len(encoded), 'fd_data_bytes': physical_len,
                        'data': hexbytes(padded)})
    reconstructed = bytearray()
    for index, rec in enumerate(records):
        raw = bytes.fromhex(rec['data'])
        valid = raw[:rec['logical_frame_bytes']]
        assert validate(valid)
        assert ((int(rec['can_id'], 16) >> 16) & 0xFF) == 0xEF
        assert (int(rec['can_id'], 16) & 0xFFFF) == (valid[5] << 8) | valid[4]
        assert all(b == 0 for b in raw[len(valid):])
        if len(records) > 1:
            assert struct.unpack_from('<H', valid, 8)[0] == index
            assert valid[3] >> 6 == (1 if index == 0 else (3 if index == len(records)-1 else 2))
            assert bool(valid[3] & 0x20) == bool((flags & 0x20) and index == len(records)-1)
        reconstructed.extend(valid[16:-2])
    assert bytes(reconstructed) == payload
    return records


def write_catalog():
    groups = [
        ('通用管理', 0, ['TEST', 'GET_INFO', 'GET_CAPS', 'HEARTBEAT', 'ACQUIRE_CONTROL', 'RELEASE_CONTROL']),
        ('固件升级', 50, ['UPDATE_OPEN', 'UPDATE_MANIFEST', 'UPDATE_BEGIN', 'UPDATE_WRITE', 'UPDATE_QUERY', 'UPDATE_FINALIZE', 'UPDATE_ACTIVATE', 'UPDATE_ABORT', 'GET_BOOT_RESULT', 'ENTER_BOOT']),
        ('基础电机控制', 100, ['MOTOR_STOP', 'MOTOR_ENABLE', 'MOTOR_DISABLE', 'SET_IQ', 'SET_SPEED', 'SET_POSITION', 'SET_MODE', 'CLEAR_FAULT', 'GET_MOTOR_STATE', 'SET_REPORT', 'READ_PARAM', 'WRITE_PARAM', 'SAVE_PARAM', 'MOTION_KEEPALIVE', 'MOTION_ARM']),
        ('电机状态上报', 120, ['旧版位置反馈', '旧版速度反馈', '旧版电流反馈', '经典 CAN 心跳/状态事件', 'MOTION_FEEDBACK']),
        ('阻抗控制', 130, ['IMPEDANCE_CONFIG', 'SET_IMPEDANCE_TARGET', 'SET_IMPEDANCE_TARGET_FULL', 'GET_IMPEDANCE_STATE']),
        ('轨迹', 140, ['TRAJ_BEGIN', 'TRAJ_APPEND', 'TRAJ_COMMIT', 'TRAJ_START', 'TRAJ_ABORT', 'GET_TRAJ_STATE']),
        ('回零', 150, ['HOME_CONFIG', 'HOME_START', 'HOME_ABORT', 'GET_HOME_STATE', 'SET_POSITION_ORIGIN']),
        ('标定与结果应用', 160, ['CALIB_CONFIG', 'CALIB_START', 'CALIB_ABORT', 'GET_CALIB_STATE', 'READ_CALIB_RESULT', 'APPLY_TASK_RESULT']),
        ('参数辨识', 170, ['IDENT_CONFIG', 'IDENT_START', 'IDENT_ABORT', 'GET_IDENT_STATE', 'READ_IDENT_RESULT']),
        ('时钟与同步', 180, ['CLOCK_EXCHANGE', 'CLOCK_ADJUST', 'SYNC_ARM', 'SYNC_COMMIT', 'SYNC_ABORT', 'GET_SYNC_STATE', 'GET_CLOCK_STATE']),
    ]
    retired = {120, 121, 122, 123}
    reports = {3, 120, 121, 122, 123, 124}
    no_ack = {100, 103, 104, 105, 113, 131, 132, 183, 184} | reports
    cyclic = {103, 104, 105, 113, 124, 131, 132}
    notes = {
        3: '1 Hz 心跳；状态/故障变化额外触发同格式事件',
        100: '立即功率禁止，允许广播，不等待周期',
        103: '电流模式，200 Hz；每轴只发送当前模式的一种目标',
        104: '速度模式，200 Hz；每轴只发送当前模式的一种目标',
        105: '位置模式，200 Hz；每轴只发送当前模式的一种目标',
        108: '按需查询位置、速度、Iq、完整状态、故障、母线电压、温度和执行序号',
        109: '三个周期字段必须相等，默认全 5 ms；全 0 关闭，否则步进 5 ms',
        113: '仅高级任务 mode 5～8，200 Hz，不能替代流式目标',
        124: '同一快照 position:i32 + speed:i16 + iq:i16；8 B payload，32 B FD 数据区',
        131: '基础阻抗位置目标；与 132 互斥',
        132: '位置/速度/Iq 前馈联合目标，16 B payload，48 B FD 数据区',
        183: '允许广播，须满足提前量，不替代单轴保活',
        184: '取消同步组并禁止输出，允许广播',
    }
    entries = []
    for group, start, names in groups:
        for offset, name in enumerate(names):
            code = start + offset
            period = 5 if code in cyclic else (1000 if code == 3 else None)
            entries.append({'type': code, 'hex': f'0x{code:04X}', 'name': name, 'group': group,
                            'role': 'report' if code in reports else 'request',
                            'business_ack': code not in no_ack,
                            'new_in_1_2': code in {113, 114} or code >= 130,
                            'new_in_1_2_1': code == 124,
                            'status': 'retired' if code in retired else 'active',
                            'supported_profiles': [] if code in retired else ['canfd'],
                            'default_period_ms': period,
                            'default_frequency_hz': 1000 // period if period else None,
                            'trigger': 'none' if code in retired else ('periodic' if period else ('event' if code in {100,184} else 'on_demand')),
                            'schedule_note': '历史编号；本版不实现、不发送、不复用' if code in retired else notes.get(code, '按需请求；业务应答不等于异步操作完成')})
    active = [e for e in entries if e['status'] == 'active']
    assert len(entries) == len({e['type'] for e in entries}) == 69
    assert len(active) == 65
    assert sum(e['role'] == 'request' for e in active) == 63
    assert sum(e['role'] == 'report' for e in active) == 2
    assert sum(e['new_in_1_2'] for e in entries) == 35
    merged = next(e for e in active if e['type'] == 124)
    merged.update({'payload_bytes': 8, 'byte_order': 'little', 'can_pgn': '0x00EF00',
                   'type_location': 'company header offset 6, uint16',
                   'fd_data_bytes': 32,
                   'payload_fields': [
                       {'name': 'position', 'offset': 0, 'type': 'int32', 'scale': 0.001, 'unit': 'rad', 'invalid': -2147483648},
                       {'name': 'speed', 'offset': 4, 'type': 'int16', 'scale': 0.001, 'unit': 'rad/s', 'invalid': -32768},
                       {'name': 'iq', 'offset': 6, 'type': 'int16', 'scale': 0.001, 'unit': 'A', 'invalid': -32768}],
                   'overflow_policy': 'invalid sentinel for affected field; never wrap or saturate'})
    spec = (DOCS / 'motor_protocol_v1.md').read_text(encoding='utf-8')
    for entry in active:
        assert entry['name'] in spec, entry['name']
    lines = ['# CAN FD 通信协议命令总表', '',
             '版本 1.2.3 评审稿，2026-09-08。当前仅 CAN FD：65 种有效消息，63 条请求、2 种上报。另有 120～123 四个历史退出编号，含历史记录的登记总数为 69，不代表本版实现 69 种消息。', '',
             '29 位扩展 ID；仲裁 500 kbit/s，数据 5 Mbit/s，BRS=1。全部报文使用公司 16 B 帧头 + payload + 2 B CRC16；完整格式、会话、错误码和分片见[协议设计规范](motor_protocol_v1.md)。', '',
             '目标和合并运动反馈各 200 Hz，8 B payload 各封装为一个 32 B FD 帧；心跳 1 Hz，状态事件额外发送。无裸 8 B 业务帧、无 J1939 TP。具体可选功能以 GET_CAPS 为准；本表未表示固件已实现。', '',
             '命令简化仍见[独立建议稿](motor_command_consolidation_review.md)，尚未替换本表编号。', '',
             '| type | 十六进制 | 类别 | 名称 | 方向 | 业务 ACK | 周期 | 说明 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for e in active:
        period = f"{e['default_frequency_hz']} Hz / {e['default_period_ms']} ms" if e['default_period_ms'] else ('事件立即' if e['trigger'] == 'event' else '按需')
        lines.append(f"| {e['type']} | {e['hex']} | {e['group']} | {e['name']} | {'上报' if e['role']=='report' else '请求'} | {'是' if e['business_ack'] else '否'} | {period} | {e['schedule_note']} |")
    lines.extend(['', '历史编号仅作防复用登记；当前收发和能力声明均不支持：', '', '| type | 历史含义 | 当前替代 |', '| --- | --- | --- |',
                  '| 120 | 旧位置反馈 | 124 |', '| 121 | 旧速度反馈 | 124 |', '| 122 | 旧电流反馈 | 124 |', '| 123 | 经典 CAN 心跳/状态事件 | 3 |', '',
                  'JSON 的 status=active 表示本版消息，retired 表示历史退出；active 的 supported_profiles 仅为 canfd，retired 为空。default_period_ms 为 null 表示无固定周期。', '',
                  '默认运动看门狗 50 ms，只由合法新目标或对应高级任务保活刷新；重复报文、查询、反馈和心跳均不刷新。实时业务单帧禁止分片；长管理/升级消息按公司 flags/seq 分片，每片独立 CRC。', '',
                  '示例见 [CAN FD 帧示例](motor_control_frame_example.md)，机器可读登记见 [motor_command_catalog.json](motor_command_catalog.json)。', ''])
    (DOCS / 'motor_command_catalog.md').write_text('\n'.join(lines), encoding='utf-8')
    (DOCS / 'motor_command_catalog.json').write_text(json.dumps(entries, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return entries


def main():
    # Cross-check CRC mechanics with a named known check value as well as custom parameters.
    assert crc(b'123456789', 16, 0x1021, 0xFFFF) == 0x29B1
    checks = {'crc8_123456789': f'{crc(b"123456789", 8, 0x9B, 0):02X}',
              'crc16_123456789': f'{crc(b"123456789", 16, 0xBAAD, 0xFFFF):04X}'}
    for length in [0, 1, 7, 8, 15, 16, 46, 128, 144, 1767]:
        payload = bytes((i * 37 + 11) % 256 for i in range(length))
        data = frame(0, 1, payload)
        assert validate(data)
        for index in [0, 3, 10, 15, len(data) - 1]:
            damaged = bytearray(data)
            damaged[index] ^= 1
            assert not validate(bytes(damaged))
        assert not validate(data[:-1])
        assert not validate(data + b'\x00')

    test = frame(0, 1, struct.pack('<III', 0, 1, 0x11223344))
    ack = frame(0, 1, struct.pack('<IIHHI', 0, 1, 0, 0, 0x11223344), flags=0x10, src=3, dst=2)
    speed = struct.pack('<BBHi', 104, 1, 0x1234, 10000)
    feedback = struct.pack('<ihh', 1570, 9500, 1200)
    assert feedback == bytes.fromhex('22 06 00 00 1C 25 B0 04')
    assert struct.unpack('<ihh', feedback) == (1570, 9500, 1200)
    for values in [(-2147483647, -32767, -32767), (2147483647, 32767, 32767), (-2147483648, -32768, -32768)]:
        assert struct.unpack('<ihh', struct.pack('<ihh', *values)) == values
    stop = struct.pack('<BBHI', 100, 1, 1, 0x53544F50)
    write = frame(53, 5, struct.pack('<IIIHH', 0x10203040, 5, 0, 16, 0) + bytes(range(16)))
    assert len(test) == 30 and len(ack) == 34 and len(write) == 50
    # Verify sizes that are especially easy to miscount in the normative tables.
    layouts = {'Q': ('<II', 8), 'R': ('<IIHH', 12), 'motor_state': ('<IIiiiIHhBBBB', 32), 'motion_feedback': ('<ihh', 8),
               'info_page0': ('<12sIHHIII', 32), 'caps_page0': ('<IHHHHHH', 16),
               'update_query': ('<BBHIIIHH', 20), 'manifest': ('<4sHHIHHIIIHBB32sHH12s64s', 144),
               'advanced_caps': ('<IIIHHIIHHI', 32), 'motion_arm': ('<IIBBHIII', 24),
               'impedance_config': ('<IIiiiii', 28), 'impedance_full': ('<BBHiii', 16),
               'task_state': ('<IBBHHHIiHH', 24), 'home_config': ('<IIBbHiiiiiIHH', 40),
               'calib_config': ('<IIIiiiI', 28), 'apply_result': ('<IIIIB3s', 20),
               'ident_config': ('<IIIiiiIHH4i', 48), 'clock_exchange': ('<IIIQ', 20),
               'clock_adjust': ('<IIIQHH', 24), 'sync_arm': ('<IIHHIQI', 28),
               'sync_state_response': ('<IIHHHHIBBHQI', 36),
               'clock_state_response': ('<IIHHHBBQqII', 40), 'result_entry': ('<HBBiHH', 12)}
    for name, (fmt, expected) in layouts.items():
        assert struct.calcsize(fmt) == expected, name
    for length in [0, 8, 45, 46, 47, 92, 93, 144, 1767]:
        fd_frames(53, 8, bytes((i * 37) % 256 for i in range(length)))
        fd_frames(53, 8, bytes((i * 37) % 256 for i in range(length)), flags=0x10)
    fd_speed = fd_frames(104, 1, speed, flags=0, priority=2)
    fd_write = fd_frames(53, 6, struct.pack('<IIIHH', 0x10203040, 6, 0, 128, 0) + bytes(range(128)))
    assert len(fd_speed) == 1 and fd_speed[0]['fd_data_bytes'] == 32
    assert len(fd_write) == 4
    for rejected_type, payload in [(124, bytes(47)), (103, bytes(47)), (123, bytes(8))]:
        try:
            fd_frames(rejected_type, 1, payload)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid frame accepted')
    encoded_speed = frame(104, 1, speed, flags=0)
    assert bytes.fromhex(fd_speed[0]['data'])[:26] == encoded_speed
    advanced = {
        'motion_keepalive': hexbytes(struct.pack('<BBHI', 113, 1, 0x1234, 7)),
        'motion_arm': hexbytes(struct.pack('<IIBBHIII', 0x10203040, 12, 1, 0, 0, 7, 42, 3)),
        'impedance_full': hexbytes(struct.pack('<BBHiii', 132, 2, 0x1234, 1000, 2000, 100)),
        'sync_arm': hexbytes(struct.pack('<IIHHIQI', 0x10203040, 30, 1, 0x2222, 7, 0x1000186A0, 20000)),
        'sync_commit': hexbytes(struct.pack('<BBHI', 183, 1, 0x2222, 0x186A0)),
    }
    segments = struct.pack('<iiiiHH', 1000, 5000, 10000, 10000, 20, 0) + struct.pack('<iiiiHH', 2000, 5000, 10000, 10000, 0, 0)
    advanced['trajectory_segments'] = hexbytes(segments)
    advanced['trajectory_crc16'] = f'{crc(segments, 16, 0xBAAD, 0xFFFF):04X}'
    advanced['trajectory_append'] = hexbytes(struct.pack('<IIIHH', 0x10203040, 20, 42, 0, 2) + segments)
    assert len(fd_frames(132, 2, bytes.fromhex(advanced['impedance_full']), flags=0, priority=2)) == 1
    assert len(fd_frames(141, 20, bytes.fromhex(advanced['trajectory_append']))) == 2
    # Timestamp signs and delay-asymmetry bounds, with a known 5 ms master offset.
    t1, t2, t3, t4 = 100000, 96100, 96300, 102000
    rtt = (t4-t1)-(t3-t2)
    estimated = ((t1-t2)+(t4-t3)) // 2
    assert rtt == 1800 and estimated == 4800
    assert abs(estimated - 5000) <= (rtt + 1) // 2
    catalog = write_catalog()
    fixtures = {'version': '1.2.3-review', 'crc_checks': checks,
                'link_configuration': {'profile': 'canfd', 'extended_id': True, 'nominal_bps': 500000, 'data_bps': 5000000, 'brs': True, 'control_period_ms': 5, 'feedback_period_ms': 5},
                'full_frames': [{'name': 'TEST request', 'data': hexbytes(test)},
                                {'name': 'TEST response', 'data': hexbytes(ack)},
                                {'name': 'UPDATE_WRITE 16 bytes', 'data': hexbytes(write)}],
                'fd_speed': fd_speed, 'fd_write_128': fd_write,
                'fd_feedback': fd_frames(124, 1, feedback, flags=0, src=3, dst=2, priority=3),
                'fd_stop': fd_frames(100, 1, stop, flags=0, dst=255, priority=0),
                'fd_test_request': fd_frames(0, 1, struct.pack('<III', 0, 1, 0x11223344)),
                'fd_test_response': fd_frames(0, 1, struct.pack('<IIHHI', 0, 1, 0, 0, 0x11223344), flags=0x10, src=3, dst=2),
                'advanced_payloads': advanced}
    (DOCS / 'motor_protocol_vectors.json').write_text(json.dumps(fixtures, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    sections = [f"CRC 校验串 ASCII `123456789`：CRC8=0x{checks['crc8_123456789']}，CRC16=0x{checks['crc16_123456789']}，CRC16 低字节在前。以下全部线上帧使用 CAN FD、29 位 ID、BRS=1，500 kbit/s 仲裁、5 Mbit/s 数据。"]
    for name in ['fd_test_request', 'fd_test_response', 'fd_speed', 'fd_feedback', 'fd_stop']:
        for rec in fixtures[name]:
            sections.append(f"{name}：CAN ID `{rec['can_id']}`，公司有效报文 {rec['logical_frame_bytes']} B，FD 数据区 {rec['fd_data_bytes']} B，CRC 后为 00 填充。\n\n```text\n{rec['data']}\n```")
    sections.append('速度目标为 10 rad/s，lease=0x1234，command_seq=1。合并反馈同一快照：位置 1.570 rad、速度 9.500 rad/s、Iq=1.200 A；8 B payload 不含模式或执行序号，由状态消息提供。')
    sections.append('CAN FD 的 128 B 写块示例：业务 payload 共 144 B，分成 46+46+46+6 B。以下四帧每帧都含独立公司帧头与 CRC；物理填充位于各帧 CRC 之后。\n\n| 片 | CAN ID | FD 数据区长度 | 公司帧有效长度 |\n| --- | --- | --- | --- |\n' + '\n'.join(f"| {i} | {r['can_id']} | {r['fd_data_bytes']} | {r['logical_frame_bytes']} |" for i, r in enumerate(fd_write)))
    for i, rec in enumerate(fd_write):
        sections.append(f"CAN FD 写块片 {i}：\n\n```text\n{rec['data']}\n```")
    sections.append('1.2 高级命令 payload 向量如下，仅验证字节编码，不表示设备已满足执行前置条件；公司完整帧仍需封装后放入 CAN FD 数据区。')
    for name, encoded in advanced.items():
        if name != 'trajectory_crc16':
            sections.append(f'{name}：\n\n```text\n{encoded}\n```')
    sections.append(f"上述两段轨迹的 content_crc16 数值为 `0x{advanced['trajectory_crc16']}`，TRAJ_COMMIT 中按小端 u16 编码。")
    path = DOCS / 'motor_protocol_v1.md'
    content = path.read_text(encoding='utf-8').split('<!-- VECTORS -->')[0]
    path.write_text(content + '<!-- VECTORS -->\n\n' + '\n\n'.join(sections) + '\n', encoding='utf-8')
    print(json.dumps({'result': 'PASS', 'crc_checks': checks, 'verified_layouts': len(layouts), 'catalog_messages': len(catalog), 'active_messages': 65, 'fd_write_fragments': len(fd_write), 'document': str(path)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
