#!/usr/bin/env python3
"""Repository-independent source layer classification and token guards."""

from __future__ import annotations

from dataclasses import dataclass
import re
from pathlib import Path
from typing import Any, Mapping


@dataclass
class ArchitectureLintResult:
    errors: list[str]
    classified: dict[str, str]
    files: list[str]


def relative_path(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def path_matches(path: str, pattern: str) -> bool:
    if pattern == "**":
        return True
    expression = re.escape(pattern)
    expression = expression.replace(r"\*\*/", r"(?:[^/]+/)*")
    expression = re.sub(r"/\\\*\\\*$", r"(?:/.*)?", expression)
    expression = expression.replace(r"\*", r"[^/]*")
    expression = expression.replace(r"\?", r"[^/]")
    return re.fullmatch(expression, path) is not None


def is_excluded(path: str, patterns: list[str]) -> bool:
    return any(path_matches(path, pattern) for pattern in patterns)


def _string_list(
    value: object,
    label: str,
    *,
    allow_empty: bool,
) -> tuple[list[str], list[str]]:
    if not isinstance(value, list) or (not allow_empty and not value):
        qualifier = "a list" if allow_empty else "a non-empty list"
        return [], [f"{label} must be {qualifier}"]
    errors: list[str] = []
    strings: list[str] = []
    for index, item in enumerate(value):
        if not isinstance(item, str) or not item.strip():
            errors.append(f"{label}[{index}] must be a non-empty string")
        else:
            strings.append(item)
    return strings, errors


def _validated_manifest(
    manifest: object,
) -> tuple[dict[str, object] | None, list[str]]:
    if not isinstance(manifest, Mapping):
        return None, ["manifest must be an object"]
    scan = manifest.get("scan")
    if not isinstance(scan, Mapping):
        return None, ["manifest.scan must be an object"]

    roots, errors = _string_list(
        scan.get("roots"), "manifest.scan.roots", allow_empty=False
    )
    extensions, extension_errors = _string_list(
        scan.get("extensions"), "manifest.scan.extensions", allow_empty=False
    )
    errors.extend(extension_errors)
    exclude_patterns, exclude_errors = _string_list(
        scan.get("exclude_patterns", []),
        "manifest.scan.exclude_patterns",
        allow_empty=True,
    )
    errors.extend(exclude_errors)

    raw_layers = manifest.get("layers")
    if not isinstance(raw_layers, list) or not raw_layers:
        errors.append("manifest.layers must be a non-empty list")
        return None, errors

    layers: list[dict[str, object]] = []
    names: set[str] = set()
    for index, raw_layer in enumerate(raw_layers):
        label = f"manifest.layers[{index}]"
        if not isinstance(raw_layer, Mapping):
            errors.append(f"{label} must be an object")
            continue
        name = raw_layer.get("name")
        if not isinstance(name, str) or not name.strip():
            errors.append(f"{label}.name must be a non-empty string")
            continue
        if name in names:
            errors.append(f"duplicate layer name: {name}")
        names.add(name)
        patterns, pattern_errors = _string_list(
            raw_layer.get("patterns"), f"{label}.patterns", allow_empty=True
        )
        forbidden_tokens, token_errors = _string_list(
            raw_layer.get("forbidden_tokens"),
            f"{label}.forbidden_tokens",
            allow_empty=True,
        )
        errors.extend(pattern_errors)
        errors.extend(token_errors)
        layers.append(
            {
                "name": name,
                "patterns": patterns,
                "forbidden_tokens": forbidden_tokens,
            }
        )

    if errors:
        return None, errors
    return {
        "scan": {
            "roots": roots,
            "extensions": extensions,
            "exclude_patterns": exclude_patterns,
        },
        "layers": layers,
    }, []


def lint_architecture(
    root: Path,
    manifest: Mapping[str, Any] | object,
) -> ArchitectureLintResult:
    """Classify every scanned source once and enforce layer token boundaries."""
    root = root.resolve()
    validated, validation_errors = _validated_manifest(manifest)
    if validated is None:
        return ArchitectureLintResult(
            errors=validation_errors,
            classified={},
            files=[],
        )

    scan = validated["scan"]
    extensions = set(scan["extensions"])
    exclude_patterns = list(scan.get("exclude_patterns", []))
    paths: set[Path] = set()
    errors: list[str] = []
    for scan_root in scan["roots"]:
        base = root / scan_root
        if not base.exists():
            errors.append(f"scan root does not exist: {scan_root}")
            continue
        if not base.is_dir():
            errors.append(f"scan root is not a directory: {scan_root}")
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
    if not files:
        errors.append("scan matched zero files")
    classified: dict[str, str] = {}
    layers = list(validated["layers"])
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
