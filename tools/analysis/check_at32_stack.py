"""核算AT32 APP静态栈及中断嵌套预算，不连接硬件。"""

import argparse
import json
import re
from pathlib import Path


def check(callgraph: Path, memory_map: Path) -> dict:
    """读取当前ARMCC产物，结合已审计的前台回调及三级IRQ嵌套保守计算。"""
    html = callgraph.read_text(errors="replace")
    functions = {}
    callbacks = []
    for block in re.split(r"(?=<P><STRONG>)", html):
        name = re.search(r"</a>([^<]+)</STRONG>", block)
        own = re.search(r"Stack size (\d+) bytes", block)
        depth = re.search(r"Max Depth = (\d+)", block)
        if name and own:
            functions[name[1]] = (int(own[1]), int(depth[1]) if depth else int(own[1]))
            if "[Address Reference Count" in block and "IRQHandler" not in name[1]:
                if name[1] not in {"main", "PendSV_Handler"}:
                    if name[1].startswith("Bsp") and functions[name[1]][1] > 64:
                        raise ValueError("BSP函数指针叶调用超过64B，需重新审计栈预算")
                    callbacks.append(functions[name[1]][1])
    # CAN/SysTick同优先级3，PendSV/监督拍同优先级2，ADC优先级0。
    resources = (
        Path(__file__).resolve().parents[2] / "firmware/platform/bsp/mcu/at32f456/mcu_resources.h"
    )
    source = resources.read_text(encoding="utf-8")
    expected = {"ADC": 0, "CONTROL": 2, "CAN": 3, "SYSTICK": 3}
    for name, priority in expected.items():
        match = re.search(rf"#define MCU_{name}_IRQ_PRIORITY (\d+)U", source)
        if match is None or int(match[1]) != priority:
            raise ValueError("IRQ优先级变化，需要重新审计嵌套预算")
    # 协议分发到一个业务回调，业务中的BSP函数指针额外预留64B。
    protocol = sum(
        functions[name][0] for name in ("main", "App_Process", "Protocol_Process", "handle_command")
    )
    foreground = max(functions["main"][1], protocol + max(callbacks) + 64)
    interrupts = [
        max(functions["CAN1_RX_IRQHandler"][1], functions["SysTick_Handler"][1]),
        max(functions["PendSV_Handler"][1], functions["TMR6_DAC_GLOBAL_IRQHandler"][1]),
        functions["ADC1_2_IRQHandler"][1],
    ]
    # 每层额外64B覆盖功率等叶级指针调用；每层112B含FP扩展硬件帧和对齐。
    # 另留256B覆盖汇编运行库及终止故障入口，不能将链接器Unknown项解释为零。
    required = foreground + sum(interrupts) + 3 * (64 + 112) + 256
    text = memory_map.read_text(errors="replace")
    allocated = re.search(r"Stack_Mem\s+0x[0-9a-f]+\s+Data\s+(\d+)", text)
    if allocated is None:
        raise ValueError("MAP缺少APP栈边界")
    capacity = int(allocated[1])
    result = dict(
        foreground=foreground,
        interrupt_depths=interrupts,
        exception_frames=336,
        indirect_leaves=192,
        assembly_fault_margin=256,
        required=required,
        allocated=capacity,
        margin=capacity - required,
        hardware_measured=False,
    )
    if required > capacity:
        raise ValueError(f"APP栈预算超限：{result}")
    return result


def main() -> int:
    """拒绝无法满足既定中断/回调预算的构建，并保留可复查证据。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--callgraph", type=Path, required=True)
    parser.add_argument("--map", dest="memory_map", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = check(args.callgraph, args.memory_map)
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"APP stack: {result['required']} / {result['allocated']} B; margin {result['margin']} B")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
