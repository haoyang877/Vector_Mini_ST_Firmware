# APP1.8.5 / CAN-Link-Test1.11.1候选

ROLL及左右轮的LUT采样由100圈减为10圈，PITCH/YAW/Lift保持100圈。
15 rad/s、PI、电流、稳速、覆盖、参考检查和保存条件不变，参数ABI兼容。
固件实现08ae7e59，工具配套7fd6e8f5；完整Release通过。仅完成离线与构建验证，
约8秒为估计，实机耗时、噪声与重复性待对照；候选保持debug，不作硬件验收声明。
交付、哈希及当前边界见[报告](../reports/2026-11/ht_calibration_ten_turns_20261113.md)。
