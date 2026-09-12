# mdrive-host — PC 上位机（CAN 升级 / 参数 / 曲线）

长期可维护的产品级上位机工程：**无 GUI 的核心库 + 薄 GUI + CLI** 三件套。

- 核心库 `mdrive_core`：协议编解码、传输层、参数 schema、设备会话（纯 Python，无 Qt，可离线测试）
- CLI `mdrive_cli`：产线 / CI 无人值守（scan / readall / write / save / upgrade）
- GUI `mdrive_gui`：PySide6 + pyqtgraph（连接 / 升级 / 参数 / 曲线 / 日志）

设计与规范文档：

| 文档 | 内容 |
|---|---|
| `docs/IMPLEMENTATION_PLAN.md` | 功能范围 / 技术选型 / 里程碑 / 风险 |
| `docs/UI_DESIGN.md` | 页面规范（唯一权威）+ 组件 + 术语 |
| `docs/LONG_TERM_DESIGN.md` | 长期架构 / 测试体系 / 维护机制 |
| `docs/mockups/ui_reference.html` | UI 视觉参考稿（浏览器直接打开）|

## 快速开始（Windows）

```powershell
# 1) 虚拟环境（仓库内已创建 host_app/.venv）
python -m venv host_app\.venv
host_app\.venv\Scripts\python.exe -m pip install -e host_app            # 核心 + CLI
host_app\.venv\Scripts\python.exe -m pip install -e "host_app[gui]"     # 追加 GUI 依赖

# 2) CLI（需连接 USBCANFD 与设备）
host_app\.venv\Scripts\mdrive.exe scan
host_app\.venv\Scripts\mdrive.exe readall
host_app\.venv\Scripts\mdrive.exe read 0x04
host_app\.venv\Scripts\mdrive.exe write 0x02 1.5
host_app\.venv\Scripts\mdrive.exe save
host_app\.venv\Scripts\mdrive.exe flashinfo
host_app\.venv\Scripts\mdrive.exe flashdump --out flash_record.bin
host_app\.venv\Scripts\mdrive.exe lut --out encoder_lut.csv
host_app\.venv\Scripts\mdrive.exe record --seconds 5 --rate 100 --out stream.csv
host_app\.venv\Scripts\mdrive.exe upgrade --image <app_image.bin> --version 123

# 3) GUI
host_app\.venv\Scripts\python.exe -m mdrive_gui
```

CAN 适配器 DLL 默认路径可通过环境变量覆盖：`MDRIVE_CAN_DLL=D:\...\ControlCANFD.dll`。

## 固件镜像构建（含命名规范）

```powershell
python host_app\tools\make_image.py --variant product            # Keil 构建 + fromelf + 规范命名 + 清单
python host_app\tools\make_image.py --no-build --version 1.2.3   # 仅转换当前 axf，指定发布版本
python host_app\tools\make_image.py --image-type loader --project loader\mdk\Vector_Mini_ST_Loader.uvprojx --target Vector_Mini_ST_Loader
```

- 命名与清单规范：`docs/firmware/FIRMWARE_NAMING.md`（型如
  `app_product_vector_mini_st_stm32g4_v1.2.3_<git>_<CRC32>.bin` + 同名 `.json` 清单）
- 产物目录：`outputs/firmware_images/`；清单含 `version_u32`，升级时传入
  `mdrive upgrade --image <bin> --version 0x01020337`

## 工具链

```powershell
host_app\.venv\Scripts\python.exe -m pytest host_app\tests -q      # 单元测试（无硬件）
host_app\.venv\Scripts\python.exe -m ruff check host_app\src
```

## 打包（PyInstaller）

```powershell
cd host_app
.venv\Scripts\python.exe -m PyInstaller --onefile --windowed --name mdrive-host --paths src --collect-submodules pyqtgraph --noconfirm src\mdrive_gui\__main__.py
.venv\Scripts\python.exe -m PyInstaller --onefile --console --name mdrive-cli --paths src --noconfirm src\mdrive_cli\main.py
python packaging\verify_exe.py        # exe 验收：GUI 离线/实机自测 + CLI 冒烟
```

产物：`dist/mdrive-host.exe`（GUI，含 `--selftest` / `--selftest-live` 自测）、
`dist/mdrive-cli.exe`（CLI，11 个命令）。验收脚本输出 `5/5 exe checks passed` 方为通过。

## 目录

```
host_app/
├── docs/                  设计与参考稿（见上表）
├── src/
│   ├── mdrive_core/       transport / protocol / schema / device
│   ├── mdrive_cli/        命令行工具（console script: mdrive）
│   └── mdrive_gui/        PySide6 界面
├── tests/                 离线单元测试
├── pyproject.toml         PEP 621 工程定义（核心零依赖；GUI 为可选 extra）
└── .venv/                 本地虚拟环境（已被 .gitignore 忽略）
```

## 协议速览

- 参数：标准 ID `(node<<8)|param`，0x00–0x67；编码 `f32 / mA16 / mrad32 / crad32`；协议版本 `0x67 = 2`
- 升级：Loader 协议 v1（`0x7D0/0x7E0 + node`，CAN-FD、BRS、CRC32；`UPDATE_FLOW.md`）
- 状态流：48 B 帧 `0x7F0 + node`，10–200 Hz（`0x64` 配置）
- 计划新增（固件配合包 B）：`0x68 SAVE_PARAM`、`0x69 FLASH_PARAM_INFO`、`0x6A FLASH_PARAM_CHUNK`

## 开发约定

- 协议逻辑禁止写进 GUI；GUI 只做绑定与呈现
- 所有总线访问经由**单工作线程**独占传输层（无事务序号，必须串行）
- 新增设备/产品 → 增加 `profiles/` 数据，不改代码
- 失败必须可诊断：结构化日志 + 诊断包（帧日志 + 参数快照 + 配置）
