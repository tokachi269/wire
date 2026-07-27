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
        "domains/road/src/draw",
        "domains/road/src/persistence",
    )
    for source in required_boundaries:
        if not (root / source).exists():
            errors.append(f"{source}: required road architecture boundary is missing")
    required_stage_owners = (
        "domains/road/src/build/topology.cpp",
        "domains/road/src/build/alignment.cpp",
        "domains/road/src/build/connection.cpp",
        "domains/road/src/build/sampling.cpp",
        "domains/road/src/build/section.cpp",
        "domains/road/src/build/gate.cpp",
        "domains/road/src/build/junction.cpp",
        "domains/road/src/build/marking.cpp",
        "domains/road/src/build/pipeline.cpp",
    )
    for source in required_stage_owners:
        if not (root / source).is_file():
            errors.append(f"{source}: required road build-stage owner is missing")

    road_header = source_text(root / "domains/road/include/city/road/road.hpp")
    road_source = source_text(root / "domains/road/src/road.cpp")
    derived_header = source_text(
        root / "domains/road/include/city/road/derived_types/derived_road.hpp"
    )
    common_header = source_text(root / "domains/road/include/city/road/common_types.hpp")
    authoritative_header = source_text(
        root / "domains/road/include/city/road/authoritative_types/road_graph.hpp"
    )
    request_header = source_text(
        root / "domains/road/include/city/road/input_types/requests.hpp"
    )
    bindings = source_text(root / "web/wasm/bindings.cpp")

    forbidden_legacy_authority = {
        "JunctionDefinition": "automatic junction existence must not be authoritative",
        "Path alignment{}": "RoadSegment must store SegmentShape without endpoint coordinates",
    }
    for token, reason in forbidden_legacy_authority.items():
        if token in road_header:
            errors.append(f"domains/road/include/city/road/road.hpp: {reason}: {token!r}")

    for source, text in (
        ("domains/road/include/city/road/common_types.hpp", common_header),
        ("domains/road/include/city/road/authoritative_types/road_graph.hpp", authoritative_header),
        ("domains/road/include/city/road/input_types/requests.hpp", request_header),
    ):
        for token in ("std::string style", "string style"):
            if token in text:
                errors.append(f"{source}: road authoritative/request style must be typed ID, not {token!r}")
    if "SurfaceStyleId" not in common_header or "MarkingStyleId" not in common_header:
        errors.append("domains/road/include/city/road/common_types.hpp: typed road style IDs are missing")
    if "std::string material" in derived_header or "surface_materials" in derived_header:
        errors.append(
            "domains/road/include/city/road/derived_types/derived_road.hpp: "
            "derived road material must use RenderStyleRef, not display strings"
        )
    if "RenderStyleRef" not in derived_header:
        errors.append(
            "domains/road/include/city/road/derived_types/derived_road.hpp: "
            "RenderStyleRef is missing"
        )

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
            "template_at(",
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
    if "road_material_key(city::road::RenderStyleRef" not in bindings:
        errors.append("web/wasm/bindings.cpp: road viewer material mapping must be a single RenderStyleRef adapter")
    if "mesh.material" in bindings:
        errors.append("web/wasm/bindings.cpp: road adapter must not read core mesh material strings")

    draw_root = root / "domains/road/src/draw"
    if draw_root.exists():
        forbidden_draw = (
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
        policy_draw = {
            "BoundaryRole::kCurb": "draw must not infer section meaning by curb-role search",
            "* 0.55": "draw must not choose connection corner control",
            "* 0.45": "draw must not choose junction corner control",
            "std::sort(sides": "draw must not sort approaches",
            "append_gate_quad": "draw must not place junction markings",
        }
        for path in draw_root.rglob("*"):
            if not path.is_file() or path.suffix not in {".cpp", ".hpp"}:
                continue
            text = source_text(path)
            for token in forbidden_draw:
                if token in text:
                    source = path.relative_to(root).as_posix()
                    errors.append(f"{source}: draw must not use {token!r}")
            for token, reason in policy_draw.items():
                if token in text:
                    source = path.relative_to(root).as_posix()
                    errors.append(f"{source}: {reason}: {token!r}")
            if "ApproachGeometryOverride" in text:
                source = path.relative_to(root).as_posix()
                errors.append(f"{source}: draw must not read ApproachGeometryOverride")
            for token in ('"asphalt"', '"sidewalk"', '"curb"', '"road_marking"', '"marking"'):
                if token in text:
                    source = path.relative_to(root).as_posix()
                    errors.append(f"{source}: draw must not use display material string {token}")
            if ".material" in text:
                source = path.relative_to(root).as_posix()
                errors.append(f"{source}: draw must consume RenderStyleRef, not Mesh.material")
        draw_header = source_text(
            draw_root / "mesh.hpp"
        )
        required_resolved_inputs = (
            ("make_connection(", "ConnectionGeometry"),
            ("make_junction(", "JunctionGeometry"),
            ("make_markings(", "ResolvedMarkingGraph"),
        )
        for function_token, type_token in required_resolved_inputs:
            if function_token not in draw_header or type_token not in draw_header:
                errors.append(
                    "domains/road/src/draw/mesh.hpp: "
                    f"resolved geometry input is missing: {function_token!r} / {type_token!r}"
                )

    if "ApproachKey approach" not in derived_header:
        errors.append(
            "domains/road/include/city/road/derived_types/derived_road.hpp: "
            "ConnectionGate must own ApproachKey identity"
        )
    for token in ("AutoNodeLayout", "ResolvedNodeLayout", "MarkingAnchor"):
        if token not in derived_header:
            errors.append(
                "domains/road/include/city/road/derived_types/derived_road.hpp: "
                f"missing road layout read model: {token}"
            )
    authoritative_header = source_text(
        root / "domains/road/include/city/road/authoritative_types/road_graph.hpp"
    )
    if "ApproachGeometryOverride" not in authoritative_header:
      errors.append(
          "domains/road/include/city/road/authoritative_types/road_graph.hpp: "
          "manual approach override authority is missing"
      )

    retired_build_names = (
        "build_context.hpp",
        "build_pipeline.cpp",
        "stages.hpp",
        "stage_support.hpp",
        "stage_support.cpp",
        "canonical_alignment.cpp",
        "node_connection_decision.cpp",
        "auto_node_layout.cpp",
        "resolved_node_layout.cpp",
        "sampling_plan.cpp",
        "section_evaluation.cpp",
        "connection_gate.cpp",
        "junction_geometry.cpp",
    )
    for name in retired_build_names:
        if (root / "domains/road/src/build" / name).exists():
            errors.append(
                f"domains/road/src/build/{name}: retired verbose build owner returned"
            )

    build_root = root / "domains/road/src/build"
    forbidden_build_names = (
        "BuildContext",
        "BuildTopologyIndex",
        "BuildCanonicalAlignments",
        "BuildNodeConnectionDecisions",
        "BuildAutoNodeLayouts",
        "BuildResolvedNodeLayouts",
        "BuildSamplingPlans",
        "BuildSectionEvaluations",
        "BuildConnectionGates",
        "BuildJunctionGeometries",
        "BuildMarkingAnchors",
        "BuildMarkingIntents",
        "BuildMarkingContinuations",
        "BuildResolvedMarkingGraph",
    )
    for path in build_root.rglob("*"):
        if not path.is_file() or path.suffix not in {".cpp", ".hpp"}:
            continue
        text = source_text(path)
        for token in forbidden_build_names:
            if token in text:
                source = path.relative_to(root).as_posix()
                errors.append(f"{source}: verbose road build name returned: {token}")
    for path in road_sources(root):
        source = path.relative_to(root).as_posix()
        if "node_gate_setback" in source_text(path):
            errors.append(
                f"{source}: legacy node_gate_setback must be removed"
            )

    for path in road_sources(root):
        source = path.relative_to(root).as_posix()
        text = source_text(path)
        if source != "domains/road/src/build/section.cpp":
            for token in ("SurfaceStyleForBoundaryRole",):
                if token in text:
                    errors.append(
                        f"{source}: BoundaryRole to SurfaceStyleId mapping must stay in section.cpp"
                    )
        if source in {
            "domains/road/src/build/gate.cpp",
            "domains/road/src/build/junction.cpp",
        } and "ApproachGeometryOverride" in text:
            errors.append(f"{source}: override authority must be consumed only by layout.cpp")
        if "find_approach_override" in text and source not in {
            "domains/road/src/build/layout.cpp",
            "domains/road/src/build/read.cpp",
            "domains/road/src/build/read.hpp",
        }:
            errors.append(f"{source}: auto/manual approach layout composition must stay in layout.cpp")
        if "setback_m.value" in text and source not in {
            "domains/road/src/build/layout.cpp",
            "domains/road/src/persistence/road_archive.cpp",
            "domains/road/src/road.cpp",
        }:
            errors.append(f"{source}: manual setback value must not be consumed outside resolved layout/persistence/operation")
        if "lateral_shift_m.value" in text and source not in {
            "domains/road/src/build/layout.cpp",
            "domains/road/src/persistence/road_archive.cpp",
            "domains/road/src/road.cpp",
        }:
                errors.append(f"{source}: manual lateral shift value must not be consumed outside resolved layout/persistence/operation")
        if "MarkingRule" in text or "marking_rule" in text:
            errors.append(f"{source}: legacy MarkingRule/marking_rule must not be used")
        if source.startswith("domains/road/src/draw/"):
            for token in ("AutoMarkingPolicy", "SectionBoundarySample.marking", "MarkingRole::"):
                if token in text:
                    errors.append(f"{source}: marking policy/role decisions must be resolved before draw: {token!r}")
            for token in ("kCenterLine ? 0.06", "kStopLineCenter", "kCrosswalk"):
                if token in text:
                    errors.append(f"{source}: marking width/placement policy must not live in draw: {token!r}")
        if source == "domains/road/src/build/junction.cpp":
            for token in ("ResolvedAutoMarking", "auto_markings", "marking_quad", "zebra"):
                if token in text:
                    errors.append(f"{source}: junction geometry must not generate marking quads: {token!r}")
        if source == "domains/road/src/build/marking.cpp":
            for token in ("MarkingIntent", "ResolvedMarkingGraph", "AutoMarkingPolicy"):
                if token not in text:
                    errors.append(f"{source}: marking resolution stage must own {token}")
        if source.startswith("domains/road/src/persistence/"):
            for token in ("find(',')", "getline(in, line)", "std::stringstream"):
                if token in text:
                    errors.append(f"{source}: road persistence must not use position-dependent comma row parsing: {token!r}")
        for token in ("evaluate_segment_section", "evaluate_segment_template"):
            if token in text:
                errors.append(
                    f"{source}: legacy distributed section evaluator must be removed: {token}"
                )
        for token in ("connection_control_factor", "junction_control_factor"):
            if token in text and source != "domains/road/src/build/connection.cpp":
                errors.append(
                    f"{source}: connection geometry policy must be owned by "
                    f"connection.cpp: {token!r}"
                )

    build_begin = road_source.find("Result<bool> RoadState::BuildDerived()")
    save_begin = road_source.find("Result<std::string> RoadState::Save()")
    if build_begin < 0 or save_begin <= build_begin:
        errors.append("domains/road/src/road.cpp: BuildDerived region could not be identified")
    else:
        build_region = road_source[build_begin:save_begin]
        for token in (
            "build::stage::",
            "NodeConnectionDecision",
            "evaluate_segment_section",
            "make_connection",
            "make_junction",
        ):
            if token in build_region:
                errors.append(
                    "domains/road/src/road.cpp: BuildDerived must remain a thin orchestrator; "
                    f"stage implementation token remains: {token!r}"
                )
    save_region = road_source[save_begin:] if save_begin >= 0 else ""
    if "return persistence::SaveRoad(graph_, next_id_);" not in save_region:
        errors.append("domains/road/src/road.cpp: RoadState::Save must delegate to road persistence")
    if "persistence::LoadRoad(text)" not in save_region:
        errors.append("domains/road/src/road.cpp: RoadState::Load must delegate to road persistence")
    for token in ("surface_band=", "manual_line=", "manual_area=", "std::ostringstream out"):
        if token in save_region:
            errors.append(f"domains/road/src/road.cpp: Save/Load body must not remain in road.cpp: {token!r}")
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
