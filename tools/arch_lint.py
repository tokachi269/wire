#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path, PurePosixPath

from road_arch_lint import check_road_architecture


def rel(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def matches(path: str, pattern: str) -> bool:
    return PurePosixPath(path).match(pattern)


def word_present(text: str, symbol: str) -> bool:
    if symbol.isidentifier():
        return re.search(rf"\b{re.escape(symbol)}\b", text) is not None
    return symbol in text


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
    for line in lines[start + 1:]:
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
    errors: list[str] = []
    table = table_after_heading(text, "## 操作×状態")
    if len(table) < 2:
        return set(), ["docs/wire/backbone_operation_semantics.md: missing operation-state table"]
    header = markdown_cells(table[0])
    states = [match.group(1) for cell in header[1:] if (match := re.fullmatch(r"`([^`]+)`", cell))]
    if len(states) != len(header) - 1:
        errors.append("docs/wire/backbone_operation_semantics.md: operation-state table state headers must be backtick ids")
    required: set[str] = set()
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) != len(header):
            errors.append(f"docs/wire/backbone_operation_semantics.md: malformed operation-state row: {line.strip()}")
            continue
        op_match = re.search(r"`([^`]+)`", cells[0])
        if op_match is None:
            errors.append(f"docs/wire/backbone_operation_semantics.md: operation row lacks backtick id: {cells[0]}")
            continue
        operation = op_match.group(1)
        for state, value in zip(states, cells[1:]):
            code_values = set(re.findall(r"`([^`]+)`", value))
            bare = re.sub(r"`[^`]+`", "", value).strip()
            if "-" in code_values or bare == "-":
                continue
            if any(re.fullmatch(r"D\d+", item) for item in code_values):
                continue
            if code_values.intersection({"C", "O", "K", "U"}) or "O" in value or "U" in value or "K" in value:
                required.add(f"BOS:{operation}:{state}")
    return required, errors


def parse_backbone_semantics_coverage(text: str) -> tuple[dict[str, set[str]], set[str], list[str]]:
    errors: list[str] = []
    case_ids: set[str] = set()
    coverage: dict[str, set[str]] = {}
    case_row = re.compile(r"\|\s*(C\d+)\s*\|")
    for line in text.splitlines():
        if match := case_row.match(line):
            case_ids.add(match.group(1))
    table = table_after_heading(text, "## Backbone Operation Semantics Coverage")
    if len(table) < 2:
        return coverage, case_ids, ["domains/wire/tests/spec_ledger.md: missing Backbone Operation Semantics Coverage table"]
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) < 2:
            errors.append(f"domains/wire/tests/spec_ledger.md: malformed semantics coverage row: {line.strip()}")
            continue
        cell_id = cells[0]
        if not re.fullmatch(r"BOS:[a-z0-9_]+:[A-Z0-9]+", cell_id):
            errors.append(f"domains/wire/tests/spec_ledger.md: invalid semantics coverage cell id: {cell_id}")
            continue
        if cell_id in coverage:
            errors.append(f"domains/wire/tests/spec_ledger.md: duplicate semantics coverage cell: {cell_id}")
            continue
        cases = set(re.findall(r"C\d+", cells[1]))
        if not cases:
            errors.append(f"domains/wire/tests/spec_ledger.md: semantics coverage has no case ids: {cell_id}")
        coverage[cell_id] = cases
    return coverage, case_ids, errors


def parse_aspect_list(value: str) -> set[str]:
    return set(re.findall(r"`([a-z0-9_]+)`", value))


def parse_backbone_aspect_requirements(text: str) -> tuple[dict[str, set[str]], list[str]]:
    errors: list[str] = []
    requirements: dict[str, set[str]] = {}
    table = table_after_heading(text, "## セル必須観点")
    if len(table) < 2:
        return requirements, ["docs/wire/backbone_operation_semantics.md: missing cell aspect requirements table"]
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) < 2:
            errors.append(f"docs/wire/backbone_operation_semantics.md: malformed aspect requirement row: {line.strip()}")
            continue
        cell_id = cells[0]
        if not re.fullmatch(r"BOS:[a-z0-9_]+:[A-Z0-9]+", cell_id):
            errors.append(f"docs/wire/backbone_operation_semantics.md: invalid aspect requirement cell id: {cell_id}")
            continue
        if cell_id in requirements:
            errors.append(f"docs/wire/backbone_operation_semantics.md: duplicate aspect requirement cell: {cell_id}")
            continue
        aspects = parse_aspect_list(cells[1])
        if not aspects:
            errors.append(f"docs/wire/backbone_operation_semantics.md: aspect requirement has no aspects: {cell_id}")
        requirements[cell_id] = aspects
    return requirements, errors


def parse_backbone_aspect_coverage(text: str) -> tuple[dict[str, tuple[set[str], set[str]]], list[str]]:
    errors: list[str] = []
    coverage: dict[str, tuple[set[str], set[str]]] = {}
    table = table_after_heading(text, "## Backbone Operation Aspect Coverage")
    if len(table) < 2:
        return coverage, ["domains/wire/tests/spec_ledger.md: missing Backbone Operation Aspect Coverage table"]
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) < 3:
            errors.append(f"domains/wire/tests/spec_ledger.md: malformed semantics aspect row: {line.strip()}")
            continue
        cell_id = cells[0]
        if not re.fullmatch(r"BOS:[a-z0-9_]+:[A-Z0-9]+", cell_id):
            errors.append(f"domains/wire/tests/spec_ledger.md: invalid semantics aspect cell id: {cell_id}")
            continue
        if cell_id in coverage:
            errors.append(f"domains/wire/tests/spec_ledger.md: duplicate semantics aspect cell: {cell_id}")
            continue
        cases = set(re.findall(r"C\d+", cells[1]))
        aspects = parse_aspect_list(cells[2])
        if not cases:
            errors.append(f"domains/wire/tests/spec_ledger.md: semantics aspect coverage has no case ids: {cell_id}")
        if not aspects:
            errors.append(f"domains/wire/tests/spec_ledger.md: semantics aspect coverage has no aspects: {cell_id}")
        coverage[cell_id] = (cases, aspects)
    return coverage, errors


def parse_backbone_representative_requirements(text: str) -> tuple[dict[str, set[str]], list[str]]:
    errors: list[str] = []
    requirements: dict[str, set[str]] = {}
    table = table_after_heading(text, "## セル必須入力代表")
    if len(table) < 2:
        return requirements, ["docs/wire/backbone_operation_semantics.md: missing cell representative requirements table"]
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) < 2:
            errors.append(f"docs/wire/backbone_operation_semantics.md: malformed representative requirement row: {line.strip()}")
            continue
        cell_id = cells[0]
        if not re.fullmatch(r"BOS:[a-z0-9_]+:[A-Z0-9]+", cell_id):
            errors.append(f"docs/wire/backbone_operation_semantics.md: invalid representative requirement cell id: {cell_id}")
            continue
        if cell_id in requirements:
            errors.append(f"docs/wire/backbone_operation_semantics.md: duplicate representative requirement cell: {cell_id}")
            continue
        representatives = parse_aspect_list(cells[1])
        if not representatives:
            errors.append(f"docs/wire/backbone_operation_semantics.md: representative requirement has no representatives: {cell_id}")
        requirements[cell_id] = representatives
    return requirements, errors


def parse_backbone_representative_coverage(text: str) -> tuple[dict[str, tuple[set[str], set[str]]], list[str]]:
    errors: list[str] = []
    coverage: dict[str, tuple[set[str], set[str]]] = {}
    table = table_after_heading(text, "## Backbone Operation Representative Coverage")
    if len(table) < 2:
        return coverage, ["domains/wire/tests/spec_ledger.md: missing Backbone Operation Representative Coverage table"]
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) < 3:
            errors.append(f"domains/wire/tests/spec_ledger.md: malformed semantics representative row: {line.strip()}")
            continue
        cell_id = cells[0]
        if not re.fullmatch(r"BOS:[a-z0-9_]+:[A-Z0-9]+", cell_id):
            errors.append(f"domains/wire/tests/spec_ledger.md: invalid semantics representative cell id: {cell_id}")
            continue
        if cell_id in coverage:
            errors.append(f"domains/wire/tests/spec_ledger.md: duplicate semantics representative cell: {cell_id}")
            continue
        cases = set(re.findall(r"C\d+", cells[1]))
        representatives = parse_aspect_list(cells[2])
        if not cases:
            errors.append(f"domains/wire/tests/spec_ledger.md: semantics representative coverage has no case ids: {cell_id}")
        if not representatives:
            errors.append(f"domains/wire/tests/spec_ledger.md: semantics representative coverage has no representatives: {cell_id}")
        coverage[cell_id] = (cases, representatives)
    return coverage, errors


def parse_backbone_authority_guards(text: str) -> tuple[list[dict[str, object]], list[str]]:
    errors: list[str] = []
    guards: list[dict[str, object]] = []
    table = table_after_heading(text, "## Backbone Authority Guard Coverage")
    if len(table) < 2:
        return guards, ["domains/wire/tests/spec_ledger.md: missing Backbone Authority Guard Coverage table"]
    for line in table[1:]:
        cells = markdown_cells(line)
        if not cells or is_separator_row(cells):
            continue
        if len(cells) < 4:
            errors.append(f"domains/wire/tests/spec_ledger.md: malformed authority guard row: {line.strip()}")
            continue
        name = cells[0]
        owner = cells[1].strip("`")
        if not re.fullmatch(r"[a-z0-9_]+", name):
            errors.append(f"domains/wire/tests/spec_ledger.md: invalid authority guard name: {name}")
            continue
        if not (owner.startswith("domains/wire/src/") or owner.startswith("domains/wire/include/")):
            errors.append(f"domains/wire/tests/spec_ledger.md: authority guard owner must be wire production code: {owner}")
            continue
        guards.append(
            {
                "name": name,
                "owner": owner,
                "required": parse_aspect_list(cells[2]),
                "unique": parse_aspect_list(cells[3]),
                "forbidden": parse_aspect_list(cells[4]) if len(cells) >= 5 else set(),
            }
        )
    return guards, errors


def registered_core_case_ids(root: Path) -> set[str]:
    ids: set[str] = set()
    tests_root = root / "domains" / "wire" / "tests"
    if not tests_root.exists():
        return ids
    for path in tests_root.rglob("*.cpp"):
        text = path.read_text(encoding="utf-8", errors="replace")
        ids.update(re.findall(r'AddTest\(\s*tests,\s*"(C\d+)_', text))
    return ids


def check_backbone_semantics_coverage(root: Path) -> list[str]:
    docs_path = root / "docs" / "wire" / "backbone_operation_semantics.md"
    ledger_path = root / "domains" / "wire" / "tests" / "spec_ledger.md"
    if not docs_path.exists() or not ledger_path.exists():
        return ["backbone semantics coverage: docs or spec ledger file is missing"]
    required, parse_errors = parse_backbone_semantics_cells(docs_path.read_text(encoding="utf-8"))
    coverage, ledger_case_ids, coverage_errors = parse_backbone_semantics_coverage(
        ledger_path.read_text(encoding="utf-8")
    )
    requirements, requirement_errors = parse_backbone_aspect_requirements(docs_path.read_text(encoding="utf-8"))
    aspect_coverage, aspect_coverage_errors = parse_backbone_aspect_coverage(ledger_path.read_text(encoding="utf-8"))
    representative_requirements, representative_requirement_errors = parse_backbone_representative_requirements(
        docs_path.read_text(encoding="utf-8")
    )
    representative_coverage, representative_coverage_errors = parse_backbone_representative_coverage(
        ledger_path.read_text(encoding="utf-8")
    )
    authority_guards, authority_errors = parse_backbone_authority_guards(ledger_path.read_text(encoding="utf-8"))
    registered_ids = registered_core_case_ids(root)
    errors = (
        parse_errors
        + coverage_errors
        + requirement_errors
        + aspect_coverage_errors
        + representative_requirement_errors
        + representative_coverage_errors
        + authority_errors
    )
    covered = set(coverage.keys())
    for cell in sorted(required - covered):
        errors.append(f"domains/wire/tests/spec_ledger.md: missing semantics coverage for {cell}")
    for cell in sorted(covered - required):
        errors.append(f"domains/wire/tests/spec_ledger.md: semantics coverage for non-required cell {cell}")
    for cell, cases in sorted(coverage.items()):
        missing_ledger_cases = sorted(case for case in cases if case not in ledger_case_ids)
        if missing_ledger_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: {cell} references cases absent from ledger {', '.join(missing_ledger_cases)}"
            )
        missing_registered_cases = sorted(case for case in cases if case not in registered_ids)
        if missing_registered_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: {cell} references unregistered cases {', '.join(missing_registered_cases)}"
            )
    required_aspect_cells = set(requirements.keys())
    aspect_covered_cells = set(aspect_coverage.keys())
    for cell in sorted(required - required_aspect_cells):
        errors.append(f"docs/wire/backbone_operation_semantics.md: missing aspect requirements for {cell}")
    for cell in sorted(required_aspect_cells - required):
        errors.append(f"docs/wire/backbone_operation_semantics.md: aspect requirements for non-required cell {cell}")
    for cell in sorted(required - aspect_covered_cells):
        errors.append(f"domains/wire/tests/spec_ledger.md: missing aspect coverage for {cell}")
    for cell in sorted(aspect_covered_cells - required):
        errors.append(f"domains/wire/tests/spec_ledger.md: aspect coverage for non-required cell {cell}")
    for cell, (cases, aspects) in sorted(aspect_coverage.items()):
        expected_cases = coverage.get(cell, set())
        if cases != expected_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: aspect coverage cases for {cell} do not match semantics coverage"
            )
        missing_ledger_cases = sorted(case for case in cases if case not in ledger_case_ids)
        if missing_ledger_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: aspect coverage {cell} references cases absent from ledger {', '.join(missing_ledger_cases)}"
            )
        missing_registered_cases = sorted(case for case in cases if case not in registered_ids)
        if missing_registered_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: aspect coverage {cell} references unregistered cases {', '.join(missing_registered_cases)}"
            )
        missing_aspects = sorted(requirements.get(cell, set()) - aspects)
        if missing_aspects:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: {cell} missing required aspects {', '.join(missing_aspects)}"
            )
    required_representative_cells = set(representative_requirements.keys())
    representative_covered_cells = set(representative_coverage.keys())
    for cell in sorted(required - required_representative_cells):
        errors.append(f"docs/wire/backbone_operation_semantics.md: missing representative requirements for {cell}")
    for cell in sorted(required_representative_cells - required):
        errors.append(f"docs/wire/backbone_operation_semantics.md: representative requirements for non-required cell {cell}")
    for cell in sorted(required - representative_covered_cells):
        errors.append(f"domains/wire/tests/spec_ledger.md: missing representative coverage for {cell}")
    for cell in sorted(representative_covered_cells - required):
        errors.append(f"domains/wire/tests/spec_ledger.md: representative coverage for non-required cell {cell}")
    for cell, (cases, representatives) in sorted(representative_coverage.items()):
        expected_cases = coverage.get(cell, set())
        if cases != expected_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: representative coverage cases for {cell} do not match semantics coverage"
            )
        missing_ledger_cases = sorted(case for case in cases if case not in ledger_case_ids)
        if missing_ledger_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: representative coverage {cell} references cases absent from ledger {', '.join(missing_ledger_cases)}"
            )
        missing_registered_cases = sorted(case for case in cases if case not in registered_ids)
        if missing_registered_cases:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: representative coverage {cell} references unregistered cases {', '.join(missing_registered_cases)}"
            )
        missing_representatives = sorted(representative_requirements.get(cell, set()) - representatives)
        if missing_representatives:
            errors.append(
                f"domains/wire/tests/spec_ledger.md: {cell} missing required representatives {', '.join(missing_representatives)}"
            )
    production_roots = [root / "domains" / "wire" / "src", root / "domains" / "wire" / "include"]
    production_files = [
        path
        for production_root in production_roots
        for path in production_root.rglob("*")
        if path.is_file() and path.suffix in {".cpp", ".hpp"}
    ]
    production_text: dict[str, str] = {
        rel(path, root): path.read_text(encoding="utf-8", errors="replace") for path in production_files
    }
    for guard in authority_guards:
        name = str(guard["name"])
        owner = str(guard["owner"])
        owner_text = production_text.get(owner)
        if owner_text is None:
            errors.append(f"domains/wire/tests/spec_ledger.md: authority guard {name} owner is missing: {owner}")
            continue
        for token in sorted(guard["required"]):
            if token not in owner_text:
                errors.append(f"domains/wire/tests/spec_ledger.md: authority guard {name} owner missing token {token}")
        for token in sorted(guard["forbidden"]):
            if token in owner_text:
                errors.append(f"domains/wire/tests/spec_ledger.md: authority guard {name} owner contains forbidden token {token}")
        for token in sorted(guard["unique"]):
            owners = sorted(path for path, text in production_text.items() if token in text)
            if owners != [owner]:
                errors.append(
                    f"domains/wire/tests/spec_ledger.md: authority guard {name} token {token} appears outside owner: {', '.join(owners) or 'none'}"
                )
    return errors


def check_architecture_documents(root: Path) -> list[str]:
    required_tokens = {
        "docs/architecture.md": (
            "# Repository architecture",
            "## Domain着手条件",
            "## State layers",
            "## Operations and build",
            "## Dependency direction",
        ),
        "docs/wire/architecture.md": (
            "# Wire architecture",
            "backbone_operation_semantics.md",
        ),
        "docs/wire/backbone_operation_semantics.md": (
            "## 操作×状態",
        ),
        "docs/road/architecture.md": (
            "# Road architecture",
            "## Naming",
            "## State ownership",
            "## Regenerate",
        ),
        "docs/road/operation_semantics.md": (
            "## 操作 x 状態",
        ),
    }
    errors: list[str] = []
    for source, tokens in required_tokens.items():
        path = root / source
        if not path.exists():
            errors.append(f"{source}: required architecture document is missing")
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for token in tokens:
            if token not in text:
                errors.append(f"{source}: required architecture contract is missing {token!r}")
    return errors


def excluded(path: str, patterns: list[str]) -> bool:
    return any(
        matches(path, pattern)
        or (pattern.endswith("/**") and path.startswith(pattern[:-2]))
        for pattern in patterns
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Fail-closed wire architecture lint.")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--manifest", type=Path, default=Path("tools/arch_manifest.json"))
    args = parser.parse_args()
    root = args.root.resolve()
    manifest_path = args.manifest if args.manifest.is_absolute() else root / args.manifest

    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)

    extensions = set(manifest["scan"]["extensions"])
    exclude_patterns = manifest["scan"].get("exclude_patterns", [])
    files: list[Path] = []
    for scan_root in manifest["scan"]["roots"]:
        base = root / scan_root
        if not base.exists():
            continue
        files.extend(path for path in base.rglob("*") if path.is_file() and path.suffix in extensions)
    files = [
        path
        for path in files
        if not excluded(rel(path, root), exclude_patterns)
    ]

    errors: list[str] = []
    classified: dict[str, str] = {}
    layers = manifest.get("layers", [])
    for path in sorted(set(files)):
        source = rel(path, root)
        layer_matches = [
            layer
            for layer in layers
            if any(matches(source, pattern) for pattern in layer.get("patterns", []))
        ]
        if len(layer_matches) != 1:
            names = [layer["name"] for layer in layer_matches]
            errors.append(f"{source}: expected exactly one layer, found {names or 'none'}")
            continue
        classified[source] = layer_matches[0]["name"]

    forbidden_symbols = manifest["guards"].get("forbidden_symbols", [])
    domain_symbols = manifest["guards"].get("forbidden_core_domain_symbols", [])
    for path in sorted(set(files)):
        source = rel(path, root)
        if source not in classified:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        layer = next(item for item in layers if item["name"] == classified[source])
        for token in layer.get("forbidden_tokens", []):
            if token in text:
                errors.append(f"{source}: {classified[source]} forbids {token!r}")
        if source.startswith(("domains/wire/include/", "domains/wire/src/", "viewer/src/")):
            for symbol in forbidden_symbols:
                if word_present(text, symbol):
                    errors.append(f"{source}: forbidden resurrection symbol {symbol!r}")
        if source.startswith(("domains/wire/include/", "domains/wire/src/")):
            for symbol in domain_symbols:
                if word_present(text, symbol):
                    errors.append(f"{source}: wire core forbids city-domain symbol {symbol!r}")

    errors.extend(check_architecture_documents(root))
    errors.extend(check_backbone_semantics_coverage(root))
    errors.extend(check_road_architecture(root))

    if errors:
        for error in errors:
            print(f"arch-lint: {error}", file=sys.stderr)
        return 1

    print(f"arch-lint: pass ({len(classified)} files, {len(layers)} layers)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
