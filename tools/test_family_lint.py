#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def rel(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def registered_cases(text: str) -> list[str]:
    cases: list[str] = []
    start = 0
    while True:
        add_test = text.find("AddTest", start)
        if add_test < 0:
            break
        line_start = text.rfind("\n", 0, add_test) + 1
        if text[line_start:add_test].strip().startswith("void"):
            start = add_test + len("AddTest")
            continue
        open_paren = text.find("(", add_test)
        first_comma = text.find(",", open_paren)
        first_quote = text.find('"', first_comma)
        second_quote = text.find('"', first_quote + 1)
        if open_paren < 0 or first_comma < 0 or first_quote < 0 or second_quote < 0:
            start = add_test + len("AddTest")
            continue
        cases.append(text[first_quote + 1 : second_quote])
        start = second_quote + 1
    return cases


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate core test family ownership.")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("domains/wire/tests/test_family_manifest.json"),
    )
    args = parser.parse_args()
    root = args.root.resolve()
    manifest_path = args.manifest if args.manifest.is_absolute() else root / args.manifest

    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)

    errors: list[str] = []
    owners: dict[str, list[str]] = {}
    valid_status = {"keep", "migrated", "isolated", "retired"}
    for family in manifest.get("families", []):
        name = family.get("name", "")
        status = family.get("status", "")
        if not name or status not in valid_status or not family.get("owner"):
            errors.append(f"invalid family metadata: {name or '<unnamed>'}")
        for source in family.get("sources", []):
            owners.setdefault(source, []).append(name)
            if not (root / source).is_file():
                errors.append(f"{name}: missing source {source}")

    registered = 0
    for test_root in (root / "domains/wire/tests", root / "viewer/tests"):
        for source_path in sorted(test_root.rglob("*.cpp")):
            source = rel(source_path, root)
            text = source_path.read_text(encoding="utf-8")
            cases = registered_cases(text)
            if cases:
                registered += len(cases)
                source_owners = owners.get(source, [])
                if len(source_owners) != 1:
                    errors.append(
                        f"{source}: {len(cases)} registered cases need exactly one family, "
                        f"found {source_owners or 'none'}"
                    )

    for source, source_owners in sorted(owners.items()):
        if len(source_owners) != 1:
            errors.append(f"{source}: classified by multiple families {source_owners}")

    if errors:
        for error in errors:
            print(f"test-family-lint: {error}", file=sys.stderr)
        return 1

    print(f"test-family-lint: pass ({registered} registered cases, {len(owners)} source files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
