# Vector Mini ST Code Style

Formatting is mechanical; readability and embedded-safety rules are design constraints. The
repository harness enforces both for new files and for a legacy file as soon as that file changes.
Vendor and generated sources retain their upstream formatting.

## Scope and commands

- Managed Python: Python sources under `tools`, `tests`, `loader`, `host_app`, and `shared`.
- Managed C/C++: repository-owned sources under `firmware`, `loader`, `host_app`, `shared`, and
  `tests/hil/firmware`.
- Excluded: `firmware/third_party/**` and all `firmware/platform/**/cubemx/**` generated content.
- Check only: `uv run python tools/run.py format --check` and
  `uv run python tools/run.py lint`.
- Format new or changed managed files: `uv run python tools/run.py format`.
- Explicitly migrate all legacy formatting debt: `uv run python tools/run.py format --all`.
- Limit a local operation with
  `--module firmware|loader|host_app|shared|tooling|templates`; CI always checks all modules against
  the same root configuration.

`tools/harness/style_debt.json` records exact hashes of unchanged legacy files that do not yet
meet the rules. Changing one of those files invalidates its waiver and requires cleanup. Resolved
debt never needs to be restored.

## C and C++

- Use four spaces, no tabs, Allman braces, and a 100-column target.
- Manually wrap Chinese prose near the 100-column target. The formatter does not reflow comments,
  because byte-based wrapping can split Chinese contracts at unreadable positions.
- Every `if`, `else`, `for`, `while`, and `do` body uses braces.
- Public functions use `Module_Action`; private functions and variables use `snake_case`; types use
  `PascalCase`; macros and compile-time constants use `UPPER_SNAKE_CASE`.
- Put the unit or representation in physical-value names, such as `_rad`, `_rad_s`, `_a`, `_v`,
  `_us`, `_q15`, or `_counts`. Convert units at boundaries rather than in the control core.
- Prefer small, single-purpose functions, early validation, explicit failure paths, and bounded
  loops. Avoid nesting deeper than four levels.
- Never allocate, block, log without a bound, or perform storage/CRC work in an ISR or fast loop.
- Public headers must include their own prerequisites and remain usable on the host test target.
- Protocol and persistent structures keep explicit-width fields and compile-time size/offset checks.
  Reformatting must never silently change ABI layout.
- Repository-owned comments and docstrings use concise Chinese. They explain invariants, units,
  ownership, hardware risks, or a non-obvious decision; they do not narrate syntax. Public APIs and
  non-trivial modules require a Chinese contract comment.
- Every public C/C++ function declaration uses a Doxygen contract immediately above it: Chinese
  `@brief`, one `@param` for every named parameter, and `@return` for every non-`void` result.
  Add `@note` when the caller must know ISR/thread context, ownership, blocking behavior, side
  effects, hardware state, or a safety precondition.

## Python

- Ruff is the formatter and linter; configuration and versions live in `pyproject.toml` and
  `harness.toml`.
- Public helpers and non-trivial data structures use type annotations. Prefer `pathlib.Path` for
  paths and explicit subprocess return-code handling.
- Put executable behavior in `main()` and keep imports free of hardware access or side effects.
- Never use `assert` for hardware safety, input validation, STOP verification, or shutdown checks.
- CLI failures state what failed and provide an actionable remediation.
- Python module and public API docstrings use Chinese; command-line option names and externally
  defined protocol terms retain their canonical spelling.

## Generated and third-party code

Do not reformat vendor, SEGGER, CMSIS, HAL, or CubeMX-generated files. Edits to CubeMX files remain
inside `USER CODE` regions and are reviewed for generator survival. Repository adapters around
generated code follow this style normally.

## Module consistency

All buildable modules inherit the root `.editorconfig`, `.clang-format`, Ruff configuration, and
this document. A module may exclude generated or externally owned files, but it must not introduce
a competing formatter configuration. Cross-module interfaces use the `shared` naming, units,
error semantics, and ABI vocabulary. New module build commands must run the root `format --check`
and `lint` gates rather than defining weaker local equivalents.

## Golden examples

Copy and rename the examples under `templates/` when creating a module or command. The examples are
part of the managed style scope, so CI verifies them with the same rules as production code. They
demonstrate structure and error handling, not product-specific behavior or default parameters.

## Public interface example

```c
/**
 * @brief 尝试发布一帧状态；队列忙时立即返回，不等待也不重试。
 * @param identifier 标准 CAN 标识符，只使用低 11 位。
 * @param data 只读负载，所有权始终属于调用方。
 * @param length 负载字节数，范围为 0..64。
 * @return 成功入队返回 true；参数非法或队列忙返回 false。
 * @note 可从前台调用，不可在电机快速中断中调用。
 */
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length);
```
