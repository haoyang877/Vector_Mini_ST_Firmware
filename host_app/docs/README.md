# 上位机文档

本目录旧版 Python / PySide6 方案保留在本目录中，作为设计沿革和参考稿；其中的页面、参数协议决策和里程碑不代表当前产品能力。

当前 Rust 上位机以以下文档为准：

- [功能需求清单](../../docs/guides/yg_mc_host_requirements_v1.md)
- [软件设计说明](../../docs/architecture/yg_mc_host_design_v1.md)

本目录原始设计文件来自提交 `48332788`（2026-09-13）：
[UI_DESIGN.md](UI_DESIGN.md)、[IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)、
[LONG_TERM_DESIGN.md](LONG_TERM_DESIGN.md) 和 [界面参考稿](mockups/ui_reference.html)。
示例中的 DLL 路径已改为相对写法。涉及当前支持状态、界面流程或实现边界时，以 Rust 文档和 `host_app/rust/` 源码为准。


当前生产测试工具另由[AT32五轴指南](../../docs/guides/at32f456_five_axes.md)维护，
Qt入口为`tools/bench/qt_production_desktop.py`；与本目录历史草案及Rust候选分别维护。
