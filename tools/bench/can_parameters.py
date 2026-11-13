"""参数服务 v1 的目录和停机事务；调用者独占 CAN 端口，文件不访问设备。"""

import json
import struct
import sys
from collections.abc import Callable
from decimal import ROUND_HALF_UP, Decimal, InvalidOperation
from pathlib import Path

from hardware_check import Link, Rejected

GROUPS = {
    "motor": (
        "电机参数",
        (
            (5, "极对数"),
            (6, "磁链"),
            (7, "相电阻"),
            (8, "D轴电感"),
            (9, "Q轴电感"),
            (2, "电流上限"),
            (64, "电角零位"),
            (65, "机械零位"),
            (66, "编码器方向"),
            (67, "完整标定"),
        ),
    ),
    "current": ("电流环控制参数", ((11, "Id Kp"), (12, "Id Ki"), (13, "Iq Kp"), (14, "Iq Ki"))),
    "speed": (
        "速度环控制参数",
        (
            (1, "速度上限"),
            (16, "速度 Kp"),
            (17, "速度 Ki"),
            (19, "加速度"),
            (20, "减速度"),
            (104, "前馈启用"),
            (105, "前馈增益"),
            (106, "前馈加速度"),
            (107, "前馈加加速度"),
            (108, "前馈电流上限"),
        ),
    ),
    "position": (
        "位置环控制参数",
        (
            (21, "位置 Kp"),
            (22, "位置 Kd"),
            (23, "速度上限"),
            (24, "加速度"),
            (25, "减速度"),
            (26, "误差窗"),
        ),
    ),
}
_catalog_path = (
    Path(sys._MEIPASS) / "parameter_catalog_v1.json"
    if getattr(sys, "frozen", False)
    else Path(__file__).resolve().parents[2] / "shared/protocol/parameter_catalog_v1.json"
)
CATALOG = {item["id"]: item for item in json.loads(_catalog_path.read_text("utf-8"))["parameters"]}
DETAILS = {
    1: "未登记",
    2: "只读",
    3: "类型错误",
    4: "超出板端范围或耦合约束",
    5: "当前状态不允许",
    6: "修订号冲突，请重新读取",
    7: "Flash保存失败",
}


def parameter_ids(group: str) -> list[int]:
    """返回页面组包含的协议 ID，保持显示和事务顺序一致。"""
    return [identifier for identifier, _ in GROUPS[group][1]]


def encode_value(identifier: int, text: str) -> int:
    """按目录缩放并四舍五入；拒绝非有限值、只读项及已知越界输入。"""
    spec = CATALOG[identifier]
    if not spec["access"] & 2:
        raise ValueError(f"参数 {identifier} 为只读")
    try:
        value = Decimal(text.strip())
    except InvalidOperation:
        raise ValueError(f"参数 {identifier} 须为有效数值") from None
    if not value.is_finite():
        raise ValueError(f"参数 {identifier} 须为有限数值")
    if spec["value_type"] == 2 and value != value.to_integral_value():
        raise ValueError(f"参数 {identifier} 须为整数")
    if ("minimum" in spec and value < Decimal(str(spec["minimum"]))) or (
        "maximum" in spec and value > Decimal(str(spec["maximum"]))
    ):
        raise ValueError(f"参数 {identifier} 超出目录范围")
    low, high = (-2147483648, 2147483647) if spec["value_type"] == 1 else (0, 4294967295)
    scaled = (value * spec["scale"]).to_integral_value(rounding=ROUND_HALF_UP)
    if not low <= scaled <= high:
        raise ValueError(f"参数 {identifier} 超出线路整数范围")
    return int(scaled)


def value_text(identifier: int, raw: int) -> str:
    """无浮点截断地显示目录中的物理量。"""
    return format(Decimal(raw) / CATALOG[identifier]["scale"], "f")


def _reply(data: bytes, identifier: int, kind: int) -> dict:
    if len(data) < 4:
        raise Rejected(f"Type {kind} 回复截断")
    result, detail = struct.unpack_from("<HH", data)
    if result:
        raise Rejected(f"Type {kind} 拒绝：{DETAILS.get(detail, '协议错误')} ({result}, {detail})")
    if len(data) != 20 or data[4] != identifier:
        raise Rejected(f"Type {kind} 参数 ID 或回复长度不匹配")
    expected_type = CATALOG[identifier]["value_type"] if kind != 122 else 0
    if data[5] != expected_type:
        raise Rejected(f"Type {kind} 参数类型不匹配")
    return dict(
        raw=int.from_bytes(data[8:12], "little", signed=data[5] == 1),
        access=data[6],
        flags=data[7],
        revision=int.from_bytes(data[12:16], "little"),
        saved_revision=int.from_bytes(data[16:20], "little"),
    )


def _read(link: Link, ids: list[int], *, saved: bool = False) -> dict[int, dict]:
    rows = {}
    for identifier in ids:
        kind = 123 if saved else 120
        rows[identifier] = _reply(
            link.request(kind, bytes((identifier, 0, 0, 0))), identifier, kind
        )
    if len({row["revision"] for row in rows.values()}) != 1:
        raise Rejected("读取期间参数发生变化，请重新读取")
    return rows


def run_parameters(
    bus,
    node: int,
    uid: str,
    group: str,
    *,
    changes: dict[int, int] | None = None,
    snapshot: dict | None = None,
    stopped: Callable[[], bool] = lambda: False,
) -> dict:
    """读取或显式写入并保存；失败不回放写命令，保留已确认 RAM 修改及持久化状态。"""
    link = Link(bus, node, timeout_s=0.5)
    report = dict(
        profile="parameters",
        node=node,
        uid=uid,
        group=group,
        passed=False,
        action="read" if changes is None else "save",
        applied={},
        save_sent=False,
        persisted=False,
        write_attempted=[],
    )
    try:
        info = link.request(1, page=0)
        identity = link.request(1, page=1)
        caps = link.request(2, page=0)
        if len(info) != 32 or info[:12].hex() != uid:
            raise Rejected("设备UID已改变，请重新识别并读取")
        if len(identity) != 32 or identity[21:23] != bytes((node, 1)):
            raise Rejected("当前电机未运行APP")
        if len(caps) != 16 or not int.from_bytes(caps[:4], "little") & (1 << 14):
            raise Rejected("固件未声明参数服务 v1")
        status = link.status()
        if status[43:] != bytes((5, 0)) or status[4:8] != bytes(4):
            raise Rejected("请先停机并清除故障，再读取或保存参数")
        ids = parameter_ids(group)
        rows = _read(link, ids)
        report.update(rows=rows, revision=next(iter(rows.values()))["revision"])
        if stopped():
            raise Rejected("操作已停止，未发送保存")
        if changes is not None:
            if not snapshot or snapshot.get("uid") != uid or snapshot.get("node") != node:
                raise Rejected("请先读取当前电机参数")
            if snapshot.get("revision") != report["revision"]:
                raise Rejected("参数修订号已变化，请重新读取后再保存")
            if any(
                snapshot.get("rows", {}).get(i, {}).get("raw") != row["raw"]
                for i, row in rows.items()
            ):
                raise Rejected("参数内容已变化，请重新读取后再保存")
            for identifier, raw in changes.items():
                if identifier not in ids or not CATALOG[identifier]["access"] & 2:
                    raise Rejected(f"参数 {identifier} 不属于本组可写项")
                encode_value(identifier, value_text(identifier, raw))
                if not rows[identifier]["access"] & 2:
                    raise Rejected(f"板端参数 {identifier} 为只读")
            for identifier, raw in changes.items():
                if stopped():
                    raise Rejected("操作已停止，已确认的RAM修改尚未保存")
                if raw == rows[identifier]["raw"]:
                    continue
                spec = CATALOG[identifier]
                data = struct.pack(
                    "<4BII",
                    identifier,
                    spec["value_type"],
                    4,
                    0,
                    report["revision"],
                    raw & 0xFFFFFFFF,
                )
                report["write_attempted"].append(identifier)
                row = _reply(link.request(121, data), identifier, 121)
                report["applied"][identifier] = row
                report["revision"] = row["revision"]
            if stopped():
                raise Rejected("操作已停止，已确认的RAM修改尚未保存")
            # 122 保存整个活动配置；不重试，以免把超时误认成未写入。
            report["save_sent"] = True
            saved = _reply(link.request(122, struct.pack("<I", report["revision"])), 0, 122)
            if (
                saved["revision"] != report["revision"]
                or saved["saved_revision"] != report["revision"]
                or saved["flags"] & 6
                or not saved["flags"] & 1
            ):
                raise Rejected("Flash保存修订未确认")
            report["rows"] = rows = _read(link, ids)
            stored = report["saved_rows"] = _read(link, ids, saved=True)
            if any(
                rows[i]["raw"] != stored[i]["raw"]
                or stored[i]["saved_revision"] != report["revision"]
                or rows[i]["revision"] != report["revision"]
                or (i in report["applied"] and rows[i]["raw"] != report["applied"][i]["raw"])
                for i in ids
            ):
                raise Rejected("Flash与活动参数回读不一致")
            report["persisted"] = True
        report["passed"] = True
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        report["fatal"] = str(exc)
    report["communication_timeouts"] = link.timeouts
    return report
