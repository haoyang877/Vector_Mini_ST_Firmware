# 固件命名规范（Firmware Naming Specification）

> 适用范围：Loader 与 APP（PRODUCT / FACTORY / DEBUG）全部固件镜像、以及随镜像发布的清单文件。
> 工具实现：`host_app/tools/make_image.py` 按本规范自动生成文件名与 `.json` 清单。

## 1. 原则

1. **文件名可读、清单权威**：文件名编码关键身份信息（人眼识别、防选错）；同一路径下的 `.json`
   清单是机器可读的权威来源（CRC/大小/git/构建信息）。
2. **可追溯**：任何镜像都能追到 git 提交与构建时刻；dirty 工作区构建必须可识别。
3. **小写 + 下划线分隔**：跨平台安全（无空格/无中文/无大小写歧义），`_` 分隔字段，`-` 仅用于 SemVer 预发布段。
4. **一经发布不改名**：发布后重命名会破坏与清单/记录页的对应关系；需要新名字就重新构建。
5. **与协议对齐**：文件名中的 `v<semver>` 必须与升级时 `BEGIN_UPDATE` 传入的 `version u32` 一致
   （编码规则见 §3.4）。

## 2. 文件名格式

```
<type>_<variant>_<board>_<mcu>_v<version>_<git>_<crc32>.<ext>
```

| 字段 | 取值 | 说明 |
|---|---|---|
| `type` | `app` / `loader` | Loader 无 variant 段（见下方 Loader 规则） |
| `variant` | `product` / `factory` / `debug` | APP 变体；三种必须共享同一 APP Flash 区 |
| `board` | 板卡 slug（小写，如 `vector_mini_st`、`lz_v3_lcm`） | 来自 BSP 目录名 |
| `mcu` | HAL Port 族名（如 `stm32g4`、`domestic_x`） | 与 `hal/ports/` 命名一致 |
| `version` | SemVer 2.0.0（`v` 前缀；预发布用 `-`，开发版可带 `+build`） | 见 §3 |
| `git` | `git rev-parse --short HEAD`（7–12 位小写） | dirty 状态不进文件名、进清单 |
| `crc32` | 镜像 CRC32 的 8 位大写十六进制（无前缀） | 与升级校验值完全一致 |
| `ext` | `bin`（CAN 升级对象）；`hex`/`elf` 仅调试用，不用于升级 | |

**示例**

```
app_product_vector_mini_st_stm32g4_v1.2.3_a1b2c3d_9CC04CB4.bin
app_factory_vector_mini_st_stm32g4_v1.2.3_a1b2c3d_12AB34CD.bin
app_debug_vector_mini_st_stm32g4_v0.0.7+20260913.0038_a1b2c3d_99887766.bin
loader_vector_mini_st_stm32g4_v1.0.0_a1b2c3d_ABCDEF12.bin
```

Loader 规则：`loader_<board>_<mcu>_v<version>_<git>_<crc32>.bin`（省略 variant，地址固定在
`0x08000000`，仅经 JLink/SWD 烧录，不参与 CAN 升级）。

**禁止**：空格、中文、混合大小写、`final/new/latest/test` 等无版本含义的词、手改 CRC 字段。

## 3. 版本与编码

### 3.1 SemVer 规则
- 正式发布：`vMAJOR.MINOR.PATCH`（如 `v1.2.3`）；接口/布局变更升 MAJOR，功能升 MINOR，修复升 PATCH。
- 预发布：`v1.3.0-rc.1`。
- 开发构建：`v0.0.<commit_count>+<yyyymmdd.HHMMSS>`（脚本自动生成）。

### 3.2 单调性
同一产品的发布版本必须单调递增（未来防降级策略依赖该性质）；不允许回退或复用版本号。

### 3.3 构建号
`build = git rev-list --count HEAD & 0xFF`，随每次提交单调增长（发布前打 tag 时以 SemVer 为准）。

### 3.4 `version u32` 编码（升级协议用）
```
u32 = (MAJOR & 0xFF) << 24 | (MINOR & 0xFF) << 16 | (PATCH & 0xFF) << 8 | (build & 0xFF)
例：v1.2.3（build=7）→ 0x01020307
```
`make_image.py` 输出该值；GUI/CLI 升级时直接传入。

## 4. 清单文件（sidecar）

与镜像同目录、同基名：`<image-name>.json`

```json
{
  "schema": 1,
  "image_type": "app",
  "variant": "product",
  "board": "vector_mini_st",
  "mcu": "stm32g4",
  "version": "1.2.3",
  "version_u32": "0x01020307",
  "git_commit": "a1b2c3d",
  "git_dirty": false,
  "build_time_utc": "2026-09-13T00:38:23Z",
  "image_size": 83424,
  "crc32": "0x9CC04CB4",
  "raw_size": 83424,
  "toolchain": "ARMCC 5.06u7",
  "source": { "project": "MDK-ARM/Vector_Mini_ST.uvprojx", "target": "Vector_Mini_ST" },
  "notes": ""
}
```

要求：
- `crc32`/`image_size` 与升级对象完全一致；`git_dirty=true` 的镜像**禁止发布**（仅限内部调试）。
- 未来引入 64B 镜像描述符（见 `loader/docs/` 升级方案）后，清单字段与其一一对应，清单继续保留为交付层。

## 5. 存放与流转

| 目录 | 用途 |
|---|---|
| `outputs/firmware_images/` | 构建产物（按本规范命名，附清单） |
| 发布归档（服务器/发布包） | 同规范；一版一目录或直接平铺，清单随行 |
| `dist/` | 上位机可执行文件，不放固件 |

## 6. 工具

```powershell
# 一键：Keil 构建 → fromelf → 去尾对齐 → 规范命名 + 清单
python host_app\tools\make_image.py --variant product
python host_app\tools\make_image.py --variant factory --version 1.2.3
python host_app\tools\make_image.py --no-build          # 只转换当前 axf

# 升级（文件名中的版本应与 --version 一致，u32 由清单给出）
mdrive upgrade --image outputs\firmware_images\app_product_vector_mini_st_stm32g4_v1.2.3_a1b2c3d_9CC04CB4.bin --version 0x01020307
```

## 7. 例外与变更

- 本规范由 `docs/firmware/FIRMWARE_NAMING.md` 唯一维护；修改需同步 `make_image.py` 与清单 schema 版本。
- 现有历史镜像（如 `app_pkgb.bin`）不强制改名，重新构建即自动符合规范。
