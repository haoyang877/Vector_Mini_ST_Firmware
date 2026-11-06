# Checked Code Templates

These examples are the starting point for repository-owned modules and tools. Copy the nearest
example, rename the public prefix, remove unused fields, and add focused tests. Do not copy product
limits or hardware behavior from another module.

编程规则唯一来源为 [STYLE.md](../STYLE.md)。本目录展示外部输入校验和工具入口，
不要求所有模块都增加初始化状态、配置对象或错误枚举。已有测试覆盖时优先复用。
可信输入的确定性计算参考真实生产样例
[pi_controller.h](../firmware/motor/control/pi_controller.h) 与对应实现；其边界说明见 STYLE.md。

- `c_module/vector_module.h`: public API, units, ownership, and explicit status values.
- `c_module/vector_module.c`: validation, initialization, and bounded state updates.
- `python_tool.py`: import-safe CLI structure, typed paths, and actionable errors.

The harness formats and lints these files alongside production code. A template change therefore
requires `uv run python tools/run.py verify --profile pr`.

仓库自有代码的注释和 docstring 使用简洁中文，重点描述接口契约、单位、所有权、失败行为
和硬件风险；不要逐句翻译代码。协议关键字、命令行选项和外部 API 名称保留规范原文。
公共 C/C++ 接口按 `vector_module.h` 使用 `@brief`、逐参数 `@param`、非 `void` 的
`@return`，并在存在调用上下文或安全限制时补充 `@note`。
