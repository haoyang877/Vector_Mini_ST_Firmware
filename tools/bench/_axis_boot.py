"""从所选 Keil 大包的 AXF 解析字段，复位后通过 SWD 查看实际运行配置。"""

import struct
import time


def boot_layout(axf):
    """读取此构建的 g_app 和成员布局，不沿用其他版本的 RAM 地址。"""
    from elftools.dwarf.dwarf_expr import DWARFExprParser
    from elftools.elf.elffile import ELFFile

    with axf.open("rb") as stream:
        dwarf = ELFFile(stream).get_dwarf_info()
        expressions = DWARFExprParser(dwarf.structs)
        for unit in dwarf.iter_CUs():
            for die in unit.iter_DIEs():
                name = die.attributes.get("DW_AT_name")
                if (
                    die.tag == "DW_TAG_variable"
                    and name
                    and name.value == b"g_app"
                    and "DW_AT_location" in die.attributes
                ):
                    break
            else:
                continue
            break
        else:
            raise ValueError("配套 AXF 缺少 g_app 调试符号")

        def operand(data, operation):
            ops = expressions.parse_expr(data)
            if len(ops) != 1 or ops[0].op_name != operation:
                raise ValueError("AXF 字段位置无法解析")
            return ops[0].args[0]

        def field(owner, path):
            offset = 0
            for part in path.split("."):
                while owner.tag != "DW_TAG_structure_type":
                    owner = owner.get_DIE_from_attribute("DW_AT_type")
                member = next(
                    (
                        child
                        for child in owner.iter_children()
                        if child.tag == "DW_TAG_member"
                        and child.attributes["DW_AT_name"].value == part.encode()
                    ),
                    None,
                )
                if member is None:
                    raise ValueError(f"AXF 缺少成员 {path}")
                location = member.attributes["DW_AT_data_member_location"].value
                offset += (
                    location
                    if isinstance(location, int)
                    else operand(location, "DW_OP_plus_uconst")
                )
                owner = member.get_DIE_from_attribute("DW_AT_type")
            return offset, owner

        base = operand(die.attributes["DW_AT_location"].value, "DW_OP_addr")
        context = die.get_DIE_from_attribute("DW_AT_type")
        paths = {
            "node": "axis.configuration.device.node_id",
            "speed": "axis.configuration.application.speed_limit_rad_s",
            "ready": "communication_ready",
            "initialized": "initialized",
            "preset": "axis.configuration.preset",
        }
        fields = {key: base + field(context, path)[0] for key, path in paths.items()}
        _, pointer = field(context, paths["preset"])
        fields["joint_offset"] = field(pointer, "joint")[0]
        fields["model_offset"] = field(pointer, "model_name")[0]
        return fields


def verify_boot(probe, fields, app_base, app_size, node, joint, model, speed):
    """连续读取已运行 APP 的活动配置，Flash 中有记录本身不代表装载成功。"""

    def byte(address):
        return probe.memory_read8(address, 1)[0]

    def word(address):
        return probe.memory_read32(address, 1)[0]

    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if not probe.halted() and word(0xE000ED08) == app_base and byte(fields["initialized"]) == 1:
            preset = word(fields["preset"])
            if app_base <= preset < app_base + app_size - fields["model_offset"] - 4:
                name = word(preset + fields["model_offset"])
                if app_base <= name <= app_base + app_size - len(model) - 1:
                    actual = bytes(probe.memory_read8(name, len(model) + 1))
                    active_speed = struct.unpack(
                        "<f", bytes(probe.memory_read8(fields["speed"], 4))
                    )[0]
                    if (
                        actual == model.encode() + b"\0"
                        and byte(fields["node"]) == node
                        and byte(preset + fields["joint_offset"]) == joint
                        and byte(fields["ready"]) == 1
                        and active_speed == speed
                    ):
                        return
        time.sleep(0.05)
    raise RuntimeError("APP 启动或活动节点、型号、限速、通信就绪回读未通过")
