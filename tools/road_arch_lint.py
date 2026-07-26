#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def source_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""


def check_road_architecture(root: Path) -> list[str]:
    errors: list[str] = []
    required_boundaries = (
        "domains/road/include/city/road/input_types",
        "domains/road/include/city/road/authoritative_types",
        "domains/road/include/city/road/derived_types",
        "domains/road/src/operations",
        "domains/road/src/build",
        "domains/road/src/materialization",
        "domains/road/src/persistence",
    )
    for source in required_boundaries:
        if not (root / source).exists():
            errors.append(f"{source}: required road architecture boundary is missing")

    road_header = source_text(root / "domains/road/include/city/road/road.hpp")
    road_source = source_text(root / "domains/road/src/road.cpp")
    bindings = source_text(root / "web/wasm/bindings.cpp")

    forbidden_legacy_authority = {
        "JunctionDefinition": "automatic junction existence must not be authoritative",
        "Path alignment{}": "RoadSegment must store SegmentShape without endpoint coordinates",
    }
    for token, reason in forbidden_legacy_authority.items():
        if token in road_header:
            errors.append(f"domains/road/include/city/road/road.hpp: {reason}: {token!r}")

    nested_public_calls = (
        "trial.AddSegment(",
        "trial.AddSegmentConnectedTo(",
        "trial.AddTransition(",
        "trial.AttachSectionTransition(",
    )
    for token in nested_public_calls:
        if token in road_source:
            errors.append(f"domains/road/src/road.cpp: public operation nesting is forbidden: {token!r}")

    operation_begin = road_source.find("Result<RoadSegmentId> RoadState::AddSegment(")
    operation_end = road_source.find("Result<bool> RoadState::BuildDerived()")
    if operation_begin < 0 or operation_end <= operation_begin:
        errors.append("domains/road/src/road.cpp: road public operation region could not be identified")
    else:
        operation_region = road_source[operation_begin:operation_end]
        direct_mutations = (
            "graph_.nodes.push_back",
            "graph_.segments.push_back",
            "graph_.section_templates.push_back",
            "graph_.transitions.push_back",
            "graph_.junctions.push_back",
            "graph_.manual_lines.push_back",
            "graph_.manual_areas.push_back",
            "graph_.nodes.erase",
            "graph_.segments.erase",
            "graph_.section_templates.erase",
            "graph_.transitions.erase",
            "graph_.junctions.erase",
            "graph_.manual_lines.erase",
            "graph_.manual_areas.erase",
            "next_id_++",
            ".BuildDerived()",
        )
        for token in direct_mutations:
            if token in operation_region:
                errors.append(f"domains/road/src/road.cpp: public operation bypasses plan/Execute: {token!r}")

    identity_inference = (
        "almost_same(path_start(alignment), node->position)",
        "almost_same(path_start(alignment), node_a->position)",
        "almost_same(path_end(alignment), node_b->position)",
    )
    for token in identity_inference:
        if token in road_source:
            errors.append(f"domains/road/src/road.cpp: endpoint ownership/identity uses position epsilon: {token!r}")

    adapter_fallbacks = (
        'input["connectToFirstNode"]',
        "graph().nodes.front()",
    )
    for token in adapter_fallbacks:
        if token in bindings:
            errors.append(f"web/wasm/bindings.cpp: road adapter connection fallback is forbidden: {token!r}")

    adapter_authority = (
        "city::road::SectionTransition transition",
        "city::road::ManualLineMarking marking",
        "city::road::ManualAreaMarking marking",
    )
    for token in adapter_authority:
        if token in bindings:
            errors.append(f"web/wasm/bindings.cpp: adapter constructs road authoritative type: {token!r}")

    materialization_root = root / "domains/road/src/materialization"
    if materialization_root.exists():
        forbidden_materialization = ("SavedRoadGraph", "evaluate_section", "evaluate_segment_section", "node_degree")
        for path in materialization_root.rglob("*"):
            if not path.is_file() or path.suffix not in {".cpp", ".hpp"}:
                continue
            text = source_text(path)
            for token in forbidden_materialization:
                if token in text:
                    source = path.relative_to(root).as_posix()
                    errors.append(f"{source}: materialization must not use {token!r}")
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
