# Pitch上下限方向修正 v1

用户要求当前Pitch从−0.3～+0.9 rad改为−0.9～+0.3 rad；确认已停止其他调试操作并停稳，授权更新Flash。
采用本次实时完整备份，仅更改AXS1的minimum_position_rad、maximum_position_rad和其CRC；不运行mode9全参数保存，避免把手动调试RAM值一并持久化。
先核对Pitch节点4、AXS1/版本/原CRC、schema10、原始范围及mode0/关相；只写包含轴记录的2 KiB参数页。写后及复位后完整Flash必须等于仅12字节差异的候选，随后验证RAM范围和CRC生效、mode0/无故障/关相。
执行结果通过；新范围−51.5662°～+17.1887°，机械零位12747、标定标志7保留。RAM保持原6 A，Flash其他字节全部保留，未运动。
备份、候选、执行脚本及结果在outputs/pitch_range_negative_20261010/。回退仅可在停机且确认板卡身份后使用本次page_before.bin，不能用其他电机备份。
本轮没有改固件逻辑或ABI；通过实际Flash差异和复位回读验证此项参数操作，不表示新全运动范围已实机验收。
