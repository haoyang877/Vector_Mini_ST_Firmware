"""仅对离线冻结源码投影标识符；保留字符串、数值和计算次序，不链接进固件。"""

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

_MAPPING = json.loads((ROOT / "tests/fixtures/app_naming_v1.json").read_text(encoding="utf-8"))
_TOKENS = re.compile(
    r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[A-Za-z_]\w*|[^\s]',
    re.S,
)


def project_reference(source: str, owner: str | None = None) -> str:
    """将历史定义映射到当前测试头；私有符号只按其原所有者映射。"""
    private = _MAPPING["private_identifiers"].get(owner, {})
    public = _MAPPING["identifiers"]
    result = []
    end = 0
    previous = ""
    before_previous = ""
    for match in _TOKENS.finditer(source):
        token = match.group()
        replacement = token
        if re.fullmatch(r"[A-Za-z_]\w*", token):
            member = previous == "." or (previous == ">" and before_previous == "-")
            replacement = public.get(token if member else private.get(token, token), token)
            if not member and token in private:
                replacement = private[token]
        result.extend((source[end : match.start()], replacement))
        end = match.end()
        if not token.startswith(("/*", "//")):
            before_previous, previous = previous, token
    result.append(source[end:])
    text = "".join(result)
    for old, new in _MAPPING["files"].items():
        text = re.sub(re.escape(old.split("/")[-1]) + r"(?![A-Za-z0-9_])", new.split("/")[-1], text)
    return text
