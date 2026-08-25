#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def markdown_cells(line: str) -> list[str]:
    stripped = line.strip()
    if not stripped.startswith("|") or not stripped.endswith("|"):
        return []
    return [cell.strip() for cell in stripped.strip("|").split("|")]


def is_separator_row(cells: list[str]) -> bool:
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell) for cell in cells)


def table_after_heading(text: str, heading: str) -> list[str]:
    lines = text.splitlines()
    try:
        start = next(index for index, line in enumerate(lines) if line.strip() == heading)
    except StopIteration:
        return []
    table: list[str] = []
    in_table = False
    for line in lines[start + 1 :]:
        if line.startswith("## ") and in_table:
            break
        cells = markdown_cells(line)
        if cells:
            table.append(line)
            in_table = True
            continue
        if in_table and line.strip() == "":
            break
    return table


def parse_backbone_semantics_cells(text: str) -> tuple[set[str], list[str]]:
    source = "docs/wire/backbone_operation_semantics.md"
    table = table_after_heading(text, "## 操作×状態")
    if len(table) < 2:
        return set(), [f"{source}: missing operation-state table"]

    errors: list[str] = []
    header = markdown_cells(table[0])
    states = [
        match.group(1)
        for cell in header[1:]
        if (match := re.fullmatch(r"`([^`]+)`", cell))
    ]
    if len(states) != len(header) - 1:
        errors.append(
            f"{source}: operation-state table state headers must be backtick ids"
        )

    required: set[str] = set()
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) != len(header):
            errors.append(f"{source}: malformed operation-state row: {line.strip()}")
            continue
        op_match = re.search(r"`([^`]+)`", cells[0])
        if op_match is None:
            errors.append(f"{source}: operation row lacks backtick id: {cells[0]}")
            continue
        operation = op_match.group(1)
        for state, value in zip(states, cells[1:]):
            code_values = set(re.findall(r"`([^`]+)`", value))
            bare = re.sub(r"`[^`]+`", "", value).strip()
            if "-" in code_values or bare == "-":
                continue
            if any(re.fullmatch(r"D\d+", item) for item in code_values):
                continue
            if code_values.intersection({"C", "O", "K", "U"}):
                required.add(f"BOS:{operation}:{state}")
    return required, errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate required runtime cells from Backbone operation semantics."
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        text = args.input.read_text(encoding="utf-8")
    except OSError as error:
        print(f"backbone-semantics: cannot read {args.input}: {error}", file=sys.stderr)
        return 1

    required, errors = parse_backbone_semantics_cells(text)
    if not required:
        errors.append(
            "docs/wire/backbone_operation_semantics.md: operation-state table has no required runtime cells"
        )
    if errors:
        for error in errors:
            print(f"backbone-semantics: {error}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(f"{cell}\n" for cell in sorted(required)), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
