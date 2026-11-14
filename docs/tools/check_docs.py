#!/usr/bin/env python3
"""文档一致性检查。

两件事：
1. 项目自有文档里的相对链接必须可解析（外部链接、锚点、mailto 跳过）；
2. docs/bugs|adr|changelog 下每个条目文件都必须出现在对应 README 索引里。

范围：根 README、CLAUDE.md、docs/（不含 teach、tools）、包级 README。
不检查 third_party 与 easy_handeye2。

用法：python3 docs/tools/check_docs.py
退出码：0 = 通过；1 = 有 error。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
SKIP_DIRS = {(DOCS / "teach").resolve(), (DOCS / "tools").resolve()}
LINK = re.compile(r"\]\(([^)]+)\)")
INDEXED_KINDS = ("bugs", "adr", "changelog")

errors: list[str] = []


def iter_docs():
    yield ROOT / "README.md"
    yield ROOT / "CLAUDE.md"
    yield ROOT / "src" / "parameter_identify" / "README.md"
    for path in DOCS.rglob("*.md"):
        if any(skip in path.resolve().parents for skip in SKIP_DIRS):
            continue
        yield path


def check_links(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    for match in LINK.finditer(text):
        target = match.group(1).strip()
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        target = target.split("#", 1)[0]
        if not target:
            continue
        resolved = (path.parent / target).resolve()
        if not resolved.exists():
            errors.append(f"{path.relative_to(ROOT)}: 链接不存在 -> {target}")


def check_index(kind: str) -> None:
    index = DOCS / kind / "README.md"
    if not index.exists():
        errors.append(f"docs/{kind}/README.md 缺失")
        return
    text = index.read_text(encoding="utf-8")
    for entry in (DOCS / kind).rglob("*.md"):
        if entry.name == "README.md":
            continue
        if entry.name not in text:
            rel = entry.relative_to(DOCS / kind)
            errors.append(f"docs/{kind}/README.md 未索引 -> {rel}")


def main() -> int:
    for path in iter_docs():
        if not path.exists():
            errors.append(f"缺失: {path.relative_to(ROOT)}")
            continue
        check_links(path)

    for kind in INDEXED_KINDS:
        check_index(kind)

    if errors:
        print(f"FAIL: {len(errors)} 个问题")
        for item in errors:
            print(" -", item)
        return 1

    print("OK: 链接与索引检查通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
