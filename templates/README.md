# Checked Code Templates

These examples are the starting point for repository-owned modules and tools. Copy the nearest
example, rename the public prefix, remove unused fields, and add focused tests. Do not copy product
limits or hardware behavior from another module.

- `c_module/vector_module.h`: public API, units, ownership, and explicit status values.
- `c_module/vector_module.c`: validation, initialization, and bounded state updates.
- `python_tool.py`: import-safe CLI structure, typed paths, and actionable errors.

The harness formats and lints these files alongside production code. A template change therefore
requires `uv run python tools/run.py verify --profile pr`.

仓库自有代码的注释和 docstring 使用简洁中文，重点描述接口契约、单位、所有权、失败行为
和硬件风险；不要逐句翻译代码。协议关键字、命令行选项和外部 API 名称保留规范原文。
公共 C/C++ 接口按 `vector_module.h` 使用 `@brief`、逐参数 `@param`、非 `void` 的
`@return`，并在存在调用上下文或安全限制时补充 `@note`。
