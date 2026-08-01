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
            if code_values.intersection({"C", "O", "K", "U"}):
                required.add(f"BOS:{operation}:{state}")
    return required, errors


def parse_aspect_list(value: str) -> set[str]:
    return set(re.findall(r"`([a-z0-9_]+)`", value))


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


def check_backbone_semantics_coverage(root: Path) -> list[str]:
    docs_path = root / "docs" / "wire" / "backbone_operation_semantics.md"
    ledger_path = root / "domains" / "wire" / "tests" / "spec_ledger.md"
    if not docs_path.exists() or not ledger_path.exists():
        return ["backbone semantics coverage: docs or spec ledger file is missing"]

    docs_text = docs_path.read_text(encoding="utf-8")
    ledger_text = ledger_path.read_text(encoding="utf-8")
    required, parse_errors = parse_backbone_semantics_cells(docs_text)
    authority_guards, authority_errors = parse_backbone_authority_guards(ledger_text)
    errors = parse_errors + authority_errors
    if not required:
        errors.append("docs/wire/backbone_operation_semantics.md: operation-state table has no required runtime cells")

    for heading in ("## セル必須観点", "## セル必須入力代表"):
        if heading in docs_text:
            errors.append(f"docs/wire/backbone_operation_semantics.md: obsolete self-referential table remains: {heading}")
    for heading in (
        "## Backbone Operation Semantics Coverage",
        "## Backbone Operation Aspect Coverage",
        "## Backbone Operation Representative Coverage",
    ):
        if heading in ledger_text:
            errors.append(f"domains/wire/tests/spec_ledger.md: obsolete self-referential table remains: {heading}")

    required_runtime_tokens = {
        "domains/wire/tests/backbone/semantics_coverage.cpp": (
            "ValidateRuntimeCoverage",
            "required_cells",
            "classify",
            "ObserveMidspan",
            "TestCaseHasIndependentAssertion",
            "CurrentTestFamily",
            "TestFamily::kSourceGuard",
        ),
        "domains/wire/tests/backbone/semantics_matrix.cpp": (
            "C836_backbone_operation_matrix_executes_declared_states",
            "Observe(",
            "ObserveEmpty",
            "ObserveMidspan",
            "WIRE_TEST_EXPECT_PRESENCE",
            "WIRE_TEST_EXPECT_ANCHOR",
        ),
        "domains/wire/tests/runner.cpp": (
            "ResetRuntimeCoverage",
            "ValidateRuntimeCoverage",
        ),
        "domains/wire/CMakeLists.txt": (
            "WIRE_TEST_BACKBONE_SEMANTICS_PATH",
            "tests/backbone/semantics_coverage.cpp",
            "tests/backbone/semantics_matrix.cpp",
        ),
        "web/tests/backbone_semantics_contract.ts": (
            "requiredBackboneEntryCells",
            "missingBackboneEntryCells",
            "## 入口境界",
        ),
        "web/tests/wasm.test.ts": (
            'missingBackboneEntryCells("wasm_adapter"',
            "BOS:add_one_edge:S1",
            "BOS:add_one_edge:SM",
        ),
        "web/tests/actions.test.ts": (
            'missingBackboneEntryCells("viewer_action"',
            "BOS:add_one_edge:S1",
            "BOS:add_one_edge:SM",
        ),
    }
    for source, tokens in required_runtime_tokens.items():
        path = root / source
        if not path.exists():
            errors.append(f"{source}: required runtime semantics coverage source is missing")
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for token in tokens:
            if token not in text:
                errors.append(f"{source}: runtime semantics coverage contract is missing {token!r}")

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
            "## Generate",
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


def check_draw_interaction_contract(root: Path) -> list[str]:
    errors: list[str] = []
    required = {
        "docs/editor/draw_interaction.md": (
            "## Click",
            "## Pointer move",
            "## Enter",
            "## Escape",
            "## Undo",
            "## Tool switch",
        ),
        "web/src/actions/viewer.ts": (
            "this.draw.primaryViewportPoint",
            "this.draw.previewViewportPoint",
            "this.draw.finishSession",
            "this.draw.cancelSession",
            "this.draw.undoCommitted",
            "this.road.commitPath",
            "this.road.cancelSession",
            "this.road.undoCommitted",
        ),
        "web/wasm/bindings.cpp": (
            "preview_placements",
            "CoreState trial = *state_",
            "GenerateFromBackboneSpec(spec)",
            'result.set("failureCategory"',
            'result.set("reasonCode"',
        ),
        "docs/editor/commit_failure_categories.md": (
            "RequirementConstraint",
            "InvalidInput",
            "NotImplemented",
            "StateConflict",
            "InternalError",
        ),
        "docs/road/supported_operations.md": (
            "## Normal drawing fixtures",
            "## Requirement constraints",
            "## Failure ownership",
            "road_path_self_intersection",
        ),
        "docs/wire/supported_operations.md": (
            "## Normal drawing fixtures",
            "## Requirement constraints",
            "## Failure ownership",
            "twelve independent model-aware routes",
        ),
        "web/src/store/viewer.ts": (
            "DrawActionResult",
            'kind: "commit-rejected"',
            'kind: "ignored"',
            "lastDrawActionResult",
        ),
        "web/src/main.ts": (
            "buildIdentitiesMatch",
            "buildMismatch",
        ),
        "web/wasm/bindings.cpp": (
            "wireBuildCommit",
            "wireBuildVersion",
        ),
    }
    for source, tokens in required.items():
        path = root / source
        if not path.exists():
            errors.append(f"{source}: shared draw interaction source is missing")
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for token in tokens:
            if token not in text:
                errors.append(f"{source}: shared draw interaction contract is missing {token!r}")

    forbidden = {
        "web/src/App.svelte": ("Generate Path", "actions.undoPathPoint()"),
        "web/src/actions/draw_actions.ts": ("this.ctx.bridge.previewWireInterval",),
        "web/src/actions/road_actions.ts": ("this.ctx.bridge.roadPreviewSegment",),
        "web/src/actions/viewer.ts": (
            "preview: () => undefined",
            "enter: () => this.draw.generatePath()",
            "escape: () => this.draw.clearPath()",
        ),
        "web/src/road.ts": ('"bend"', "draftBend", "draftSpans", "withRoadBend"),
        "domains/road/src/generation/connections.cpp": (
            "connected approach angle is outside supported range",
            "adjacent junction approach angle is outside supported range",
        ),
        "domains/road/include/city/road/common_types.hpp": ("enum class ErrorKind", "error_kind"),
        "domains/wire/include/city/wire/core_edit_types.hpp": ("enum class EditErrorKind", "error_kind"),
        "web/src/model.ts": ("enum EditErrorKind", "errorKind"),
        "web/src/bridge/wire.ts": ("regenerateBuildIdentity",),
    }
    for source, tokens in forbidden.items():
        path = root / source
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for token in tokens:
            if token in text:
                errors.append(f"{source}: obsolete draw interaction path remains: {token!r}")
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
    errors.extend(check_draw_interaction_contract(root))
    errors.extend(check_road_architecture(root))

    if errors:
        for error in errors:
            print(f"arch-lint: {error}", file=sys.stderr)
        return 1

    print(f"arch-lint: pass ({len(classified)} files, {len(layers)} layers)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
