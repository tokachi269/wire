#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


CASE_RE = re.compile(r'AddTest\s*\(\s*[^,]+,\s*"(?P<case>[^"]+)"', re.MULTILINE)


def rel(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate core test family ownership.")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("core/tests/test_family_manifest.json"),
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
    for source_path in sorted((root / "core/tests").glob("*.cpp")):
        source = rel(source_path, root)
        text = source_path.read_text(encoding="utf-8")
        cases = CASE_RE.findall(text)
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
