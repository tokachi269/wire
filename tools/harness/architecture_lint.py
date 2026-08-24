#!/usr/bin/env python3
"""Repository-independent source layer classification and token guards."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


@dataclass
class ArchitectureLintResult:
    errors: list[str]
    classified: dict[str, str]
    files: list[str]


def relative_path(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def path_matches(path: str, pattern: str) -> bool:
    return PurePosixPath(path).match(pattern)


def is_excluded(path: str, patterns: list[str]) -> bool:
    return any(
        path_matches(path, pattern)
        or (pattern.endswith("/**") and path.startswith(pattern[:-2]))
        for pattern in patterns
    )


def lint_architecture(
    root: Path,
    manifest: Mapping[str, Any],
) -> ArchitectureLintResult:
    """Classify every scanned source once and enforce layer token boundaries."""
    root = root.resolve()
    scan = manifest["scan"]
    extensions = set(scan["extensions"])
    exclude_patterns = list(scan.get("exclude_patterns", []))
    paths: set[Path] = set()
    for scan_root in scan["roots"]:
        base = root / scan_root
        if not base.exists():
            continue
        paths.update(
            path
            for path in base.rglob("*")
            if path.is_file() and path.suffix in extensions
        )

    files = sorted(
        relative_path(path, root)
        for path in paths
        if not is_excluded(relative_path(path, root), exclude_patterns)
    )
    errors: list[str] = []
    classified: dict[str, str] = {}
    layers = list(manifest.get("layers", []))
    layers_by_name = {layer["name"]: layer for layer in layers}

    for source in files:
        layer_matches = [
            layer
            for layer in layers
            if any(
                path_matches(source, pattern)
                for pattern in layer.get("patterns", [])
            )
        ]
        if len(layer_matches) != 1:
            names = [layer["name"] for layer in layer_matches]
            errors.append(
                f"{source}: expected exactly one layer, found {names or 'none'}"
            )
            continue
        classified[source] = layer_matches[0]["name"]

    for source in files:
        layer_name = classified.get(source)
        if layer_name is None:
            continue
        text = (root / source).read_text(encoding="utf-8", errors="replace")
        for token in layers_by_name[layer_name].get("forbidden_tokens", []):
            if token in text:
                errors.append(f"{source}: {layer_name} forbids {token!r}")

    return ArchitectureLintResult(
        errors=errors,
        classified=classified,
        files=files,
    )
