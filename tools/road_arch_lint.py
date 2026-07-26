#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
    required_stage_owners = (
        "domains/road/src/build/topology.cpp",
        "domains/road/src/build/canonical_alignment.cpp",
        "domains/road/src/build/node_connection_decision.cpp",
        "domains/road/src/build/sampling_plan.cpp",
        "domains/road/src/build/section_evaluation.cpp",
        "domains/road/src/build/connection_gate.cpp",
        "domains/road/src/build/junction_geometry.cpp",
        "domains/road/src/build/build_pipeline.cpp",
    )
    for source in required_stage_owners:
        if not (root / source).is_file():
            errors.append(f"{source}: required road build-stage owner is missing")

    road_header = source_text(root / "domains/road/include/city/road/road.hpp")
    road_source = source_text(root / "domains/road/src/road.cpp")
    derived_header = source_text(
        root / "domains/road/include/city/road/derived_types/derived_road.hpp"
    )
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
        geometry_policy = (
            "angle_deg(",
            "ResolveTemplateAt(",
            "node_gate_setback",
            "existing_degree",
        )
        for token in geometry_policy:
            if token in operation_region:
                errors.append(
                    "domains/road/src/road.cpp: operation preflight duplicates trial build "
                    f"geometry policy: {token!r}"
                )

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
        forbidden_materialization = (
            "SavedRoadGraph",
            "CrossSectionTemplate",
            "SectionTransition",
            "NodeConnectionPolicyOverride",
            "NodeConnectionDecision",
            "ConnectionArea",
            "JunctionArea",
            "node_degree",
            'city/road/road.hpp',
            "authoritative_types",
        )
        policy_materialization = {
            "BoundaryRole::kCurb": "materialization must not infer section meaning by curb-role search",
            "* 0.55": "materialization must not choose connection corner control",
            "* 0.45": "materialization must not choose junction corner control",
            "std::sort(sides": "materialization must not sort approaches",
            "append_gate_quad": "materialization must not place junction markings",
        }
        for path in materialization_root.rglob("*"):
            if not path.is_file() or path.suffix not in {".cpp", ".hpp"}:
                continue
            text = source_text(path)
            for token in forbidden_materialization:
                if token in text:
                    source = path.relative_to(root).as_posix()
                    errors.append(f"{source}: materialization must not use {token!r}")
            for token, reason in policy_materialization.items():
                if token in text:
                    source = path.relative_to(root).as_posix()
                    errors.append(f"{source}: {reason}: {token!r}")
        materialization_header = source_text(
            materialization_root / "materialize.hpp"
        )
        required_resolved_inputs = (
            "MaterializeConnection(const ConnectionGeometry&",
            "MaterializeJunction(const JunctionGeometry&",
        )
        for token in required_resolved_inputs:
            if token not in materialization_header:
                errors.append(
                    "domains/road/src/materialization/materialize.hpp: "
                    f"resolved geometry input is missing: {token!r}"
                )

    if "ApproachKey approach" not in derived_header:
        errors.append(
            "domains/road/include/city/road/derived_types/derived_road.hpp: "
            "ConnectionGate must own ApproachKey identity"
        )

    for path in road_sources(root):
        source = path.relative_to(root).as_posix()
        if "node_gate_setback" in source_text(path):
            errors.append(
                f"{source}: legacy node_gate_setback must be removed"
            )

    for path in road_sources(root):
        source = path.relative_to(root).as_posix()
        text = source_text(path)
        for token in ("evaluate_segment_section", "evaluate_segment_template"):
            if token in text:
                errors.append(
                    f"{source}: legacy distributed section evaluator must be removed: {token}"
                )
        for token in ("connection_control_factor", "junction_control_factor"):
            if token in text and source != "domains/road/src/build/node_connection_decision.cpp":
                errors.append(
                    f"{source}: connection geometry policy must be owned by "
                    f"node_connection_decision.cpp: {token!r}"
                )

    build_begin = road_source.find("Result<bool> RoadState::BuildDerived()")
    save_begin = road_source.find("Result<std::string> RoadState::Save()")
    if build_begin < 0 or save_begin <= build_begin:
        errors.append("domains/road/src/road.cpp: BuildDerived region could not be identified")
    else:
        build_region = road_source[build_begin:save_begin]
        for token in (
            "build::Stage::",
            "NodeConnectionDecision",
            "evaluate_segment_section",
            "MaterializeConnection",
            "MaterializeJunction",
        ):
            if token in build_region:
                errors.append(
                    "domains/road/src/road.cpp: BuildDerived must remain a thin orchestrator; "
                    f"stage implementation token remains: {token!r}"
                )
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
