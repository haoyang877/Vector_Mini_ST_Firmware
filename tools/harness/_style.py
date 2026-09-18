"""集中实现源码发现、格式化、可读性检查和历史债务收敛。"""

from __future__ import annotations

import ast
import hashlib
import io
import json
import re
import shutil
import subprocess
import sys
import tokenize
import tomllib
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, config

DEBT_PATH = ROOT / "tools/harness/style_debt.json"
CHINESE = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff]")


def style_config() -> dict:
    return config()["style"]


def module_names() -> tuple[str, ...]:
    return tuple(style_config()["modules"])


def _is_excluded(relative: str) -> bool:
    path = Path(relative)
    if any(part in {".git", ".venv", "__pycache__", ".ruff_cache"} for part in path.parts):
        return True
    return any(
        relative == prefix or relative.startswith(f"{prefix}/")
        for prefix in style_config()["exclude_prefixes"]
    )


def managed_files(module: str | None = None) -> list[Path]:
    cfg = style_config()
    modules = cfg["modules"]
    selected = (module,) if module else tuple(modules)
    extensions = set(cfg["python_extensions"]) | set(cfg["native_extensions"])
    paths: set[Path] = set()
    for name in selected:
        for root_name in modules[name]["paths"]:
            root = ROOT / root_name
            if not root.exists():
                continue
            for path in root.rglob("*"):
                if path.is_file() and path.suffix.lower() in extensions:
                    relative = path.relative_to(ROOT).as_posix()
                    if not _is_excluded(relative):
                        paths.add(path)
    return sorted(paths, key=lambda item: item.relative_to(ROOT).as_posix())


def relative(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def debt() -> dict:
    if not DEBT_PATH.is_file():
        return {"schema_version": 1, "files": {}}
    return json.loads(DEBT_PATH.read_text(encoding="utf-8"))


def debt_allows(path: Path, check: str, baseline: dict | None = None) -> bool:
    entry = (baseline or debt())["files"].get(relative(path))
    if not entry or entry["sha256"] != digest(path):
        return False
    # 中文注释规则后于初始基线加入；原样未动的历史文件沿用同一哈希豁免。
    return check == "comment-language" or check in entry["checks"]


def executable(name: str) -> Path | None:
    found = shutil.which(name)
    if found:
        return Path(found).resolve()
    candidate = Path(sys.executable).resolve().parent / (
        f"{name}.exe" if sys.platform == "win32" else name
    )
    return candidate if candidate.is_file() else None


def _chunks(values: list[Path], size: int = 60):
    for index in range(0, len(values), size):
        yield values[index : index + size]


def _run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )


def collect_format_issues(paths: list[Path]) -> list[dict]:
    issues: list[dict] = []
    python_files = [path for path in paths if path.suffix.lower() == ".py"]
    native_extensions = set(style_config()["native_extensions"])
    native_files = [path for path in paths if path.suffix.lower() in native_extensions]

    if python_files:
        for chunk in _chunks(python_files):
            command = [
                sys.executable,
                "-m",
                "ruff",
                "format",
                "--check",
                *[relative(path) for path in chunk],
            ]
            result = _run(command)
            found = []
            for line in (result.stdout + result.stderr).splitlines():
                match = re.match(r"Would reformat:\s+(.+)$", line.strip(), re.IGNORECASE)
                if match:
                    found.append((ROOT / match.group(1)).resolve())
            if result.returncode and not found:
                found = chunk
            issues.extend(
                {
                    "path": path,
                    "check": "python-format",
                    "line": 1,
                    "message": "Ruff formatting differs",
                }
                for path in found
            )

    formatter = executable("clang-format")
    if native_files and formatter is None:
        issues.append(
            {
                "path": ROOT / ".clang-format",
                "check": "c-format-tool",
                "line": 1,
                "message": "clang-format is unavailable; run uv sync --locked --extra dev",
            }
        )
    elif formatter:
        pattern = re.compile(r"^(.+?\.(?:c|h|cc|cpp|hh|hpp)):(\d+):(\d+):")
        for chunk in _chunks(native_files):
            command = [str(formatter), "--dry-run", "--Werror", *[relative(path) for path in chunk]]
            result = _run(command)
            found: dict[Path, int] = {}
            for line in (result.stdout + result.stderr).splitlines():
                match = pattern.match(line.strip())
                if match:
                    found[(ROOT / match.group(1)).resolve()] = int(match.group(2))
            if result.returncode and not found:
                found = {path: 1 for path in chunk}
            issues.extend(
                {
                    "path": path,
                    "check": "c-format",
                    "line": line,
                    "message": "clang-format formatting differs",
                }
                for path, line in found.items()
            )
    return issues


def apply_format(paths: list[Path]) -> None:
    python_files = [path for path in paths if path.suffix.lower() == ".py"]
    native_extensions = set(style_config()["native_extensions"])
    native_files = [path for path in paths if path.suffix.lower() in native_extensions]
    for chunk in _chunks(python_files):
        result = subprocess.run(
            [sys.executable, "-m", "ruff", "format", *[relative(path) for path in chunk]],
            cwd=ROOT,
            check=False,
        )
        if result.returncode:
            raise RuntimeError("Ruff formatting failed")
    formatter = executable("clang-format")
    if native_files and formatter is None:
        raise RuntimeError("clang-format is unavailable; run uv sync --locked --extra dev")
    for chunk in _chunks(native_files):
        result = subprocess.run(
            [str(formatter), "-i", *[relative(path) for path in chunk]], cwd=ROOT, check=False
        )
        if result.returncode:
            raise RuntimeError("clang-format failed")


def _ruff_issues(paths: list[Path]) -> list[dict]:
    python_files = [path for path in paths if path.suffix.lower() == ".py"]
    issues = []
    for chunk in _chunks(python_files):
        result = _run(
            [
                sys.executable,
                "-m",
                "ruff",
                "check",
                "--output-format",
                "json",
                *[relative(path) for path in chunk],
            ]
        )
        try:
            records = json.loads(result.stdout or "[]")
        except json.JSONDecodeError:
            records = []
        if result.returncode not in (0, 1) and not records:
            records = [
                {
                    "filename": relative(path),
                    "location": {"row": 1},
                    "code": "TOOL",
                    "message": result.stderr.strip() or "Ruff failed",
                }
                for path in chunk
            ]
        for record in records:
            filename = Path(record["filename"])
            path = filename if filename.is_absolute() else ROOT / filename
            issues.append(
                {
                    "path": path.resolve(),
                    "check": "python-lint",
                    "line": record["location"]["row"],
                    "message": f"{record.get('code', 'RUFF')} {record['message']}",
                }
            )
    return issues


def _control_without_braces(path: Path) -> list[dict]:
    issues = []
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    direct = re.compile(
        r"^\s*(?:if|for|while)\s*\([^;{}]*\)\s+(?!\{)(?:return\b|break\s*;|continue\s*;|[^/].*;)\s*$"
    )
    direct_else = re.compile(
        r"^\s*else\s+(?!if\b|\{)(?:return\b|break\s*;|continue\s*;|[^/].*;)\s*$"
    )
    for number, line in enumerate(lines, 1):
        if direct.match(line) or direct_else.match(line):
            issues.append(
                {
                    "path": path,
                    "check": "c-readability",
                    "line": number,
                    "message": "control-flow body must use braces",
                }
            )
        if "\t" in line:
            issues.append(
                {
                    "path": path,
                    "check": "c-readability",
                    "line": number,
                    "message": "tab indentation is not allowed",
                }
            )
    return issues


def _has_chinese_comment(path: Path) -> bool:
    source = path.read_text(encoding="utf-8", errors="replace")
    if path.suffix.lower() == ".py":
        candidates = []
        try:
            tokens = tokenize.generate_tokens(io.StringIO(source).readline)
            candidates.extend(token.string for token in tokens if token.type == tokenize.COMMENT)
            tree = ast.parse(source)
            candidates.extend(
                docstring
                for node in ast.walk(tree)
                if isinstance(
                    node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)
                )
                and (docstring := ast.get_docstring(node, clean=False))
            )
        except (SyntaxError, tokenize.TokenError):
            return False
        return any(CHINESE.search(candidate) for candidate in candidates)
    comments = re.findall(r"//[^\n]*|/\*.*?\*/", source, flags=re.DOTALL)
    return any(CHINESE.search(comment) for comment in comments)


def _comment_issues(path: Path) -> list[dict]:
    source = path.read_text(encoding="utf-8", errors="replace")
    meaningful_lines = [line for line in source.splitlines() if line.strip()]
    if len(meaningful_lines) < 10 or _has_chinese_comment(path):
        return []
    return [
        {
            "path": path,
            "check": "comment-language",
            "line": 1,
            "message": "非简单源码必须包含说明意图、约束或风险的中文注释",
        }
    ]


def collect_lint_issues(paths: list[Path]) -> list[dict]:
    issues = _ruff_issues(paths)
    native_extensions = set(style_config()["native_extensions"])
    for path in paths:
        issues.extend(_comment_issues(path))
        if path.suffix.lower() in native_extensions:
            issues.extend(_control_without_braces(path))
    return issues


def collect_style_config_issues(module: str | None = None) -> list[dict]:
    """拒绝模块用局部配置静默覆盖仓库统一风格。"""
    modules = style_config()["modules"]
    selected = (module,) if module else tuple(modules)
    forbidden = {".clang-format", ".clang-tidy", ".editorconfig", ".ruff.toml", "ruff.toml"}
    issues = []
    roots = {ROOT / root_name for name in selected for root_name in modules[name]["paths"]}
    for root in sorted(roots):
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file() or _is_excluded(path.relative_to(ROOT).as_posix()):
                continue
            if path.name in forbidden:
                issues.append(
                    {
                        "path": path,
                        "check": "style-config",
                        "line": 1,
                        "message": "模块不得覆盖根目录代码风格配置",
                    }
                )
            if path.name == "pyproject.toml":
                try:
                    local_config = tomllib.loads(path.read_text(encoding="utf-8"))
                except (OSError, tomllib.TOMLDecodeError):
                    continue
                if local_config.get("tool", {}).get("ruff"):
                    issues.append(
                        {
                            "path": path,
                            "check": "style-config",
                            "line": 1,
                            "message": "模块 pyproject.toml 不得定义局部 Ruff 规则",
                        }
                    )
    return issues


def partition_debt(issues: list[dict]) -> tuple[list[dict], list[dict]]:
    baseline = debt()
    new = []
    allowed = []
    for issue in issues:
        (allowed if debt_allows(issue["path"], issue["check"], baseline) else new).append(issue)
    return new, allowed


def debt_records(issues: list[dict]) -> dict:
    grouped: dict[str, set[str]] = defaultdict(set)
    paths: dict[str, Path] = {}
    for issue in issues:
        name = relative(issue["path"])
        grouped[name].add(issue["check"])
        paths[name] = issue["path"]
    return {
        "schema_version": 1,
        "files": {
            name: {"sha256": digest(paths[name]), "checks": sorted(checks)}
            for name, checks in sorted(grouped.items())
        },
    }
