#!/usr/bin/env python3
"""Fail-closed road architecture lint.

Guards dependency direction and the things road must never do. It does not
prescribe how many files, stages or tables the generation code is split into.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def source_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""


def road_sources(root: Path) -> list[Path]:
    source_root = root / "domains/road/src"
    return [
        path
        for path in source_root.rglob("*")
        if path.is_file() and path.suffix in {".cpp", ".hpp"}
    ]


def check_road_architecture(root: Path) -> list[str]:
    errors: list[str] = []
    sources = road_sources(root)
    if not sources:
        return ["domains/road/src: no road sources were found"]

    def relative(path: Path) -> str:
        return path.relative_to(root).as_posix()

    # 1. Authority layers exist and stay separated.
    for boundary in (
        "domains/road/include/city/road/input_types",
        "domains/road/include/city/road/authoritative_types",
        "domains/road/include/city/road/derived_types",
        "domains/road/src/generation",
        "domains/road/src/geometry",
        "domains/road/src/operations",
        "domains/road/src/persistence",
    ):
        if not (root / boundary).exists():
            errors.append(f"{boundary}: required road boundary is missing")

    # 2. The old build/stage structure must not come back.
    for banned in ("domains/road/src/build", "domains/road/src/draw"):
        if (root / banned).exists():
            errors.append(f"{banned}: superseded road generation directory still exists")

    banned_tokens = (
        "BuildDerived",
        "BuildContext",
        "BuildPipeline",
        "StageOutput",
        "MaterializeSegment",
        "MaterializeJunction",
        "MaterializeMarkings",
    )
    stage_type = re.compile(r"\b(?:struct|class|enum(?:\s+class)?)\s+\w*Stage\b")
    for path in sources:
        text = source_text(path)
        for token in banned_tokens:
            if token in text:
                errors.append(f"{relative(path)}: superseded generation vocabulary: {token!r}")
        if stage_type.search(text):
            errors.append(f"{relative(path)}: Stage must not be a road design unit")

    # 3. One regenerate entry, and generation never writes the authority.
    road_source = source_text(root / "domains/road/src/road.cpp")
    if "generation::regenerate_road(graph_)" not in road_source:
        errors.append("domains/road/src/road.cpp: RoadState must regenerate through one entry")
    generation_sources = [path for path in sources if "/generation/" in relative(path)]
    if not any("regenerate_road" in source_text(path) for path in generation_sources):
        errors.append("domains/road/src/generation: regenerate entry is missing")
    for path in generation_sources + [path for path in sources if "/geometry/" in relative(path)]:
        text = source_text(path)
        if re.search(r"\bSavedRoadGraph\s*&(?!\s*const)", text.replace("const SavedRoadGraph &", "")):
            errors.append(f"{relative(path)}: generation must not take a mutable authoritative graph")

    # 4. Emit consumes resolved geometry only.
    emit_text = source_text(root / "domains/road/src/generation/emit.cpp")
    for token in (
        "SavedRoadGraph",
        "AutoMarkingPolicy",
        "marking.enabled",
        "NodeConnectionPolicy",
        "find_template",
        "section_at",
    ):
        if token in emit_text:
            errors.append(f"domains/road/src/generation/emit.cpp: emit must not read {token!r}")
    for policy in ("0.12", "0.16", "0.35", "kStopLineCenterM", "kCrosswalkStripeStepM"):
        if policy in emit_text:
            errors.append(
                f"domains/road/src/generation/emit.cpp: marking policy constant must not live in emit: {policy!r}"
            )

    # 5. Identity never comes from position.
    marking_text = source_text(root / "domains/road/src/generation/markings.cpp")
    connection_text = source_text(root / "domains/road/src/generation/connections.cpp")
    if "side_marking" in marking_text:
        errors.append(
            "domains/road/src/generation/markings.cpp: lane side policies must resolve during section evaluation"
        )
    section_text = source_text(root / "domains/road/src/geometry/section.cpp")
    if "merge_boundary_policies" not in section_text:
        errors.append(
            "domains/road/src/geometry/section.cpp: lane side requests must merge into one boundary policy here"
        )
    if "kDegenerateBandWidthM" not in marking_text:
        errors.append(
            "domains/road/src/generation/markings.cpp: degenerate run activation must be decided here"
        )
    if marking_text.count("kDegenerateBandWidthM") > 2:
        errors.append(
            "domains/road/src/generation/markings.cpp: degenerate run activation must stay a single decision site"
        )
    for path in sources:
        text = source_text(path)
        for pattern in ("nearest", "closest", "snap_to_nearest"):
            if pattern in text.lower() and "/persistence/" not in relative(path):
                errors.append(f"{relative(path)}: identity must not come from proximity: {pattern!r}")

    # 6. Connection resolution owns auto values, overrides and their policy.
    if "find_approach_override" not in connection_text:
        errors.append(
            "domains/road/src/generation/connections.cpp: approach overrides must resolve with the auto values"
        )
    for path in sources:
        relative_path = relative(path)
        if relative_path in {
            "domains/road/src/generation/connections.cpp",
            "domains/road/src/persistence/road_archive.cpp",
            "domains/road/src/road.cpp",
        }:
            continue
        text = source_text(path)
        if "setback_m.has_value" in text or "lateral_shift_m.has_value" in text:
            errors.append(
                f"{relative_path}: manual approach override values must only be consumed by connection resolution"
            )

    # 7. Public operations do not call one another.
    operation_names = re.findall(r"Result<[^>]+> RoadState::(\w+)\(", road_source)
    public_operations = {
        name for name in operation_names if name not in {"regenerate", "Execute", "Save", "Load"}
    }
    for name in sorted(public_operations):
        body_start = road_source.find(f"RoadState::{name}(")
        if body_start < 0:
            continue
        body_end = road_source.find("\n}\n", body_start)
        body = road_source[body_start:body_end if body_end > 0 else len(road_source)]
        for other in sorted(public_operations - {name}):
            if re.search(rf"\b(?:this->)?{other}\(", body) and f"RoadState::{other}(" not in body:
                if other in {"SetBoundaryMarkingPolicy", "SetLaneSideMarkingPolicy"} and name.startswith("Reset"):
                    continue
                errors.append(
                    f"domains/road/src/road.cpp: public operation {name} calls public operation {other}"
                )

    # 8. Persistence stays in its own layer.
    save_start = road_source.find("Result<std::string> RoadState::Save()")
    save_region = road_source[save_start : save_start + 2000] if save_start >= 0 else ""
    for token in ("surface_band=", "manual_line=", "manual_area=", "std::ostringstream out"):
        if token in save_region:
            errors.append(f"domains/road/src/road.cpp: Save/Load body must not remain in road.cpp: {token!r}")

    # 9. road never reaches into wire.
    for path in sources + list((root / "domains/road/include").rglob("*.hpp")):
        text = source_text(path)
        if "wire/core" in text or "city::wire" in text:
            errors.append(f"{relative(path)}: road must not depend on wire")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Fail-closed road architecture lint.")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    errors = check_road_architecture(args.root.resolve())
    if errors:
        for error in errors:
            print(f"road-arch-lint: {error}", file=sys.stderr)
        return 1
    print("road-arch-lint: pass")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
