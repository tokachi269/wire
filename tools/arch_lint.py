#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path, PurePosixPath


def rel(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def matches(path: str, pattern: str) -> bool:
    return PurePosixPath(path).match(pattern)


def word_present(text: str, symbol: str) -> bool:
    if symbol.isidentifier():
        return re.search(rf"\b{re.escape(symbol)}\b", text) is not None
    return symbol in text


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
    files: list[Path] = []
    for scan_root in manifest["scan"]["roots"]:
        base = root / scan_root
        if not base.exists():
            continue
        files.extend(path for path in base.rglob("*") if path.is_file() and path.suffix in extensions)

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
        if source.startswith(("core/include/", "core/src/", "viewer/src/")):
            for symbol in forbidden_symbols:
                if word_present(text, symbol):
                    errors.append(f"{source}: forbidden resurrection symbol {symbol!r}")
        if source.startswith(("core/include/", "core/src/")):
            for symbol in domain_symbols:
                if word_present(text, symbol):
                    errors.append(f"{source}: wire core forbids city-domain symbol {symbol!r}")

    if errors:
        for error in errors:
            print(f"arch-lint: {error}", file=sys.stderr)
        return 1

    print(f"arch-lint: pass ({len(classified)} files, {len(layers)} layers)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
