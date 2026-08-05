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
    source_roots = (
        root / "domains/road/src",
        root / "domains/road/include",
    )
    return [
        path
        for source_root in source_roots
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
        "regenerate_road",
        "BuildDerived",
        "BuildContext",
        "BuildPipeline",
        "BuildCanonicalAlignment",
        "StageOutput",
        "MaterializeSegment",
        "MaterializeJunction",
        "MaterializeMarkings",
        "AutoNodeLayout",
        "ResolvedNodeLayout",
        "MarkingAnchor",
        "MarkingIntent",
        "ResolvedMarkingGraph",
        "setback_calculation_count",
        "section_evaluation_count",
        "NormalizeRoadPath",
        "SurfaceBand",
        "SurfaceRole",
        "LaneBandId",
        "HighwayConnectionType",
        "HighwayExit",
        "RampType",
        "RampAsset",
    )
    stage_type = re.compile(r"\b(?:struct|class|enum(?:\s+class)?)\s+\w*Stage\b")
    for path in sources:
        text = source_text(path)
        for token in banned_tokens:
            if token in text:
                errors.append(f"{relative(path)}: superseded generation vocabulary: {token!r}")
        if stage_type.search(text):
            errors.append(f"{relative(path)}: Stage must not be a road design unit")

    if (root / "domains/road/src/generation/regenerate.cpp").exists():
        errors.append("domains/road/src/generation/regenerate.cpp: retired generation entry file exists")
    # 3. One generate entry, and generation never writes the authority.
    road_source = source_text(root / "domains/road/src/road.cpp")
    if "RoadState::regenerate" in road_source:
        errors.append("domains/road/src/road.cpp: RoadState::regenerate must not exist")
    if "generation::generate_road(trial.graph_)" not in road_source:
        errors.append("domains/road/src/road.cpp: RoadState must generate through one trial entry")
    generation_sources = [path for path in sources if "/generation/" in relative(path)]
    if not any("generate_road" in source_text(path) for path in generation_sources):
        errors.append("domains/road/src/generation: generate entry is missing")
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
    if "junction_control_factor" in connection_text or "connection_control_factor" in connection_text:
        errors.append(
            "domains/road/src/generation/connections.cpp: corner and junction curves must share one control factor"
        )
    if "resolve_junction_movement_path" not in connection_text:
        errors.append(
            "domains/road/src/generation/connections.cpp: junction lane movements need their own resolver"
        )
    if "topology.kind == LaneConnectionKind::kJunctionMovement" not in connection_text:
        errors.append(
            "domains/road/src/generation/connections.cpp: junction movement topology is not dispatched explicitly"
        )
    if "side_marking" in marking_text:
        errors.append(
            "domains/road/src/generation/markings.cpp: lane side policies must resolve during section evaluation"
        )
    section_text = source_text(root / "domains/road/src/geometry/section.cpp")
    if "merge_boundary_policies" not in section_text:
        errors.append(
            "domains/road/src/geometry/section.cpp: lane side requests must merge into one boundary policy here"
        )
    if "kDegenerateStripWidthM" not in marking_text:
        errors.append(
            "domains/road/src/generation/markings.cpp: degenerate run activation must be decided here"
        )
    if "derive_boundary_continuation_markings" not in marking_text or "DerivedBoundaryPath" not in marking_text:
        errors.append(
            "domains/road/src/generation/markings.cpp: connected markings must consume resolved boundary paths"
        )
    if marking_text.count("kDegenerateStripWidthM") > 2:
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
        name for name in operation_names if name not in {"Execute", "Save", "Load"}
    }
    for name in sorted(public_operations):
        body_start = road_source.find(f"RoadState::{name}(")
        if body_start < 0:
            continue
        body_end = road_source.find("\n}\n", body_start)
        body = road_source[body_start:body_end if body_end > 0 else len(road_source)]
        for other in sorted(public_operations - {name}):
            if re.search(rf"\b(?:this->)?{other}\(", body) and f"RoadState::{other}(" not in body:
                errors.append(
                    f"domains/road/src/road.cpp: public operation {name} calls public operation {other}"
                )

    # 7c. The product catalogue of cross sections belongs to whoever presents
    # it. Core keeps the type, the validation and the IDs, and Add Lane still
    # derives the section a widened road needs; what Core must not hold again is
    # a named list of finished roads, or a constructor that registers one.
    core_section_text = "\n".join(
        (
            road_source,
            source_text(root / "domains/road/include/city/road/road.hpp"),
        )
    )
    for preset in (
        "JapaneseUrbanTwoLaneTemplate",
        "ThreeLaneTemplate",
        "NoLeftSidewalkTemplate",
        "MedianTwoLaneTemplate",
        "ShoulderedTwoLaneTemplate",
    ):
        if preset in core_section_text:
            errors.append(
                "domains/road: product road preset is back in core: " + repr(preset)
            )
    constructor_start = road_source.find("RoadState::RoadState()")
    constructor_end = (
        road_source.find("\n}\n", constructor_start) if constructor_start >= 0 else -1
    )
    constructor_body = (
        road_source[constructor_start:constructor_end]
        if constructor_start >= 0 and constructor_end > constructor_start
        else road_source[constructor_start : constructor_start + 200]
        if constructor_start >= 0
        else ""
    )
    if "section_templates" in constructor_body:
        errors.append(
            "domains/road/src/road.cpp: RoadState construction must not register a section template"
        )
    if not (root / "web/src/road_templates.ts").exists():
        errors.append(
            "web/src/road_templates.ts: the road section catalogue is missing"
        )

    # 7b. The retired low-level operations must not return to the public API.
    # They edited saved rows directly; the product surface is the drawing,
    # editing, section template and Add Lane operations.
    public_api_text = "\n".join(
        (
            source_text(root / "domains/road/include/city/road/road.hpp"),
            source_text(
                root / "domains/road/include/city/road/input_types/requests.hpp"
            ),
        )
    )
    for retired in (
        "DeleteSegmentRange",
        "SetApproachSetbackOverride",
        "SetApproachLateralShiftOverride",
        "ResetApproachOverrideField",
        "ResetAllApproachOverrides",
        "SetBoundaryMarkingPolicy",
        "ResetBoundaryMarkingPolicy",
        "SetLaneSideMarkingPolicy",
        "ResetLaneSideMarkingPolicy",
        "AddTransition",
        "AttachSectionTransition",
        "AddLaneConnection",
        "AddBoundaryContinuation",
        "AddManualLine",
        "AddManualArea",
        "SuppressAutoMarking",
        "ResetAutoMarkingSuppression",
        "SetJunctionMarkingOverride",
        "DeleteJunctionMarkingOverride",
    ):
        if retired in public_api_text:
            errors.append(
                "domains/road: retired low-level road operation is public again: "
                + repr(retired)
            )

    # 7a. Local segment and corridor operations stay local.
    if re.search(r"\bExtendSegment(?:Request)?\b", road_source):
        errors.append(
            "domains/road/src/road.cpp: retired giant-segment extension API remains"
        )
    extension_start = road_source.find("RoadState::ExtendCorridorFromEnd(")
    extension_end = (
        road_source.find("\n}\n", extension_start)
        if extension_start >= 0
        else -1
    )
    extension_body = (
        road_source[extension_start:extension_end]
        if extension_start >= 0 and extension_end >= 0
        else ""
    )
    for token in ("replace_segments", "replace_nodes", "internal_knots.push_back"):
        if token in extension_body:
            errors.append(
                "domains/road/src/road.cpp: corridor extension mutates an existing segment: "
                + repr(token)
            )
    for required in ("add_segments", "replace_corridors"):
        if required not in extension_body:
            errors.append(
                "domains/road/src/road.cpp: corridor extension does not express local append: "
                + repr(required)
            )

    authoritative_text = source_text(
        root / "domains/road/include/city/road/authoritative_types/road_graph.hpp"
    )
    common_text = source_text(
        root / "domains/road/include/city/road/common_types.hpp"
    )
    for token in ("struct RoadCorridor", "std::vector<DirectedSegmentRef> segments"):
        if token not in authoritative_text:
            errors.append(
                "domains/road authoritative graph: corridor contract is missing: "
                + repr(token)
            )
    for token in ("enum class SegmentShapeIntent", "SegmentShapeIntent intent"):
        if token not in authoritative_text:
            errors.append(
                "domains/road authoritative graph: straight shape intent contract is missing: "
                + repr(token)
            )
    for token in (
        "struct SectionStrip",
        "enum class StripFunction",
        "struct LaneBand",
        "kShoulder",
    ):
        if token not in common_text:
            errors.append(
                "domains/road common types: section-axis contract is missing: "
                + repr(token)
            )

    adapter_text = source_text(root / "web/wasm/bindings.cpp")
    # Add Lane previews with a local guide. The retired path copied RoadState and
    # regenerated every road per pointer move.
    for web_source in (
        "web/wasm/bindings.cpp",
        "web/src/bridge/wire.ts",
        "web/src/bridge/wasm.ts",
        "web/src/actions/road_actions.ts",
    ):
        text = source_text(root / web_source)
        for token in ("previewAddLane", "preview_add_lane", "roadPreviewAddLane"):
            if token in text:
                errors.append(f"{web_source}: retired Add Lane preview path: {token!r}")
        # Branch/Merge asked for a raw BoundaryId and auto-picked boundaries[0],
        # so the product operation is gone until it can be selected by meaning.
        for token in ("ConnectedLaneSegment", "branch-lane", "merge-lane"):
            if token in text:
                errors.append(f"{web_source}: retired branch/merge operation: {token!r}")
    for token in ("extensionSegmentId", "bandElementId"):
        if token in adapter_text:
            errors.append(
                "web/wasm/bindings.cpp: retired road adapter identity remains: "
                + repr(token)
            )
    if (
        "segment.shape.intent == city::road::SegmentShapeIntent::kStraight"
        not in adapter_text
    ):
        errors.append(
            "web/wasm/bindings.cpp: road editable kind must come from SegmentShapeIntent"
        )
    if 'input["kind"].as<std::string>()' not in adapter_text:
        errors.append(
            "web/wasm/bindings.cpp: road adapter must pass the explicit tool kind into core path conversion"
        )
    if "align_first_span_start" not in road_source:
        errors.append(
            "domains/road/src/road.cpp: snapped road spans must use one start-alignment helper"
        )
    if "first_span.p0 =" in road_source.replace("first_span.p0 = start;", ""):
        errors.append(
            "domains/road/src/road.cpp: road snap correction must not patch only the Bezier start point"
        )
    if "segment.shape.intent != SegmentShapeIntent::kStraight" not in road_source:
        errors.append(
            "domains/road/src/road.cpp: MoveNode must preserve straight segment intent"
        )
    retired_delete_api_text = "\n".join(
        (
            road_source,
            source_text(root / "domains/road/include/city/road/road.hpp"),
            source_text(
                root
                / "domains/road/include/city/road/input_types/requests.hpp"
            ),
        )
    )
    if "DeleteRoadSection" in retired_delete_api_text:
        errors.append(
            "domains/road: retired topology-section deletion API must not return"
        )
    if re.search(r"(?:style_id|style)\s*==[^;\n]*StripFunction", road_source):
        errors.append(
            "domains/road/src/road.cpp: strip function must not be inferred from style"
        )

    road_actions_text = source_text(root / "web/src/actions/road_actions.ts")
    for token in (
        "deleteDraftSegmentId",
        "deleteDraftDistanceM",
        "roadDeleteSegmentRange(",
        "roadDeleteSection(",
    ):
        if token in road_actions_text:
            errors.append(
                "web/src/actions/road_actions.ts: retired two-point road deletion remains: "
                + repr(token)
            )
    if "roadDeleteSegment(snap.segmentId)" not in road_actions_text:
        errors.append(
            "web/src/actions/road_actions.ts: standard road deletion must delete the picked RoadSegment"
        )
    if "hoveredDeleteSegmentId" not in road_actions_text:
        errors.append(
            "web/src/actions/road_actions.ts: standard road deletion must retain the hovered RoadSegment ID"
        )
    for token in (
        "selectedLaneSegmentId",
        "selectedLaneEndpointRole",
    ):
        if token not in road_actions_text:
            errors.append(
                "web/src/actions/road_actions.ts: lane editing contract is missing: "
                + repr(token)
            )
    for token in (
        "laneStartCorridorDistanceM",
        "laneFullWidthCorridorDistanceM",
        "startCorridorDistanceM",
        "fullWidthCorridorDistanceM",
    ):
        if token in road_actions_text:
            errors.append(
                "web/src/actions/road_actions.ts: Add Lane must cross the Core boundary as segment-local t: "
                + repr(token)
            )
    for token in (
        'laneEditStage: "transition-complete"',
        'laneEditStage: "continuation-end"',
        "laneContinuationEndNodeId",
    ):
        if token not in road_actions_text:
            errors.append(
                "web/src/actions/road_actions.ts: Add Lane selection stages are incomplete: "
                + repr(token)
            )
    for token in ("nearestLane", "closestLane", "inferLaneConnection"):
        if token in road_actions_text:
            errors.append(
                "web/src/actions/road_actions.ts: adapter must not infer lane topology: "
                + repr(token)
            )
    if "laneAnchorBoundaryId: snapshot.road.laneAnchorBoundaryId || section?.boundaries[0]" in road_actions_text:
        errors.append(
            "web/src/actions/road_actions.ts: Add lane must not infer its transition anchor from boundary order"
        )
    app_text = source_text(root / "web/src/App.svelte")
    for token in ('setRoadOperation("branch-lane")',
                  'setRoadOperation("merge-lane")'):
        if token in app_text:
            errors.append(
                "web/src/App.svelte: incomplete lane topology tool remains in the standard toolbar: "
                + repr(token)
            )
    road_tool_text = source_text(root / "web/src/road.ts")
    if "pointerDelta" in road_tool_text or "previousPointer" in road_tool_text:
        errors.append(
            "web/src/road.ts: curve tangent must not depend on pointer motion history"
        )
    if "anchor->width_m > kEpsilon" in road_source:
        errors.append(
            "domains/road/src/road.cpp: lane transition must reuse the single-position boundary decision"
        )
    road_tests_text = source_text(root / "domains/road/tests/road_tests.cpp")
    if re.search(r'bool LAN\d+_|\{"LAN\d+_', road_tests_text):
        errors.append(
            "domains/road/tests/road_tests.cpp: lane behavior tests must not expose implementation phase names"
        )
    add_lane_start = road_source.find("Result<LaneId> RoadState::AddLane(")
    add_lane_end = road_source.find(
        "Result<std::string> RoadState::Save()",
        add_lane_start,
    )
    add_lane_region = (
        road_source[add_lane_start:add_lane_end]
        if add_lane_start >= 0 and add_lane_end > add_lane_start
        else ""
    )
    for token in ("split_path_at_distance", "SplitSegmentAtDistance"):
        if token in add_lane_region:
            errors.append(
                "domains/road/src/road.cpp: Add Lane must not split RoadSegments: "
                + repr(token)
            )
    request_text = source_text(
        root / "domains/road/include/city/road/input_types/requests.hpp"
    )
    for token in ("SegmentPosition transition_start", "SegmentPosition transition_complete"):
        if token not in request_text:
            errors.append(
                "domains/road: Add Lane request must use segment-local t: "
                + repr(token)
            )
    for token in (
        'result.set("lanePaths", lane_paths)',
        'builtin_marking_styles::kWhiteDashed.value',
        '.function("addLane"',
    ):
        if token not in adapter_text:
            errors.append(
                "web/wasm/bindings.cpp: lane editing boundary is missing: "
                + repr(token)
            )

    add_segment_start = road_source.find(
        "Result<RoadSegmentId> RoadState::AddSegment("
    )
    add_segment_end = road_source.find(
        "Result<RoadSegmentId> RoadState::ExtendCorridorFromEnd(",
        add_segment_start,
    )
    add_segment_region = (
        road_source[add_segment_start:add_segment_end]
        if add_segment_start >= 0 and add_segment_end > add_segment_start
        else ""
    )
    if "SegmentShapeFromPath(alignment)" not in add_segment_region:
        errors.append(
            "domains/road/src/road.cpp: one confirmed path must remain one multi-span RoadSegment"
        )
    if "alignment.spans[index]" in add_segment_region:
        errors.append(
            "domains/road/src/road.cpp: Bezier span boundaries must not create RoadSegment identities"
        )

    road_distance_paths = [
        root / "domains/road",
        root / "docs/road",
        root / "web/src/road.ts",
        root / "web/src/actions/road_actions.ts",
        root / "web/src/bridge/wasm.ts",
        root / "web/src/bridge/wire.ts",
        root / "web/wasm/bindings.cpp",
    ]
    for path in road_distance_paths:
        candidates = path.rglob("*") if path.is_dir() else [path]
        for candidate in candidates:
            if not candidate.is_file() or candidate.suffix not in {
                ".cpp", ".hpp", ".ts", ".md"
            }:
                continue
            text = source_text(candidate)
            if re.search(r"Station|station_|_station|\bstationM\b|\bstation\b", text):
                errors.append(
                    f"{candidate.relative_to(root)}: road path distance uses retired Station terminology"
                )

    # 8. Persistence stays in its own layer.
    save_start = road_source.find("Result<std::string> RoadState::Save()")
    save_region = road_source[save_start : save_start + 2000] if save_start >= 0 else ""
    for token in ("surface_band=", "manual_line=", "manual_area=", "std::ostringstream out"):
        if token in save_region:
            errors.append(f"domains/road/src/road.cpp: Save/Load body must not remain in road.cpp: {token!r}")


    # 10. Road docs describe the current generate structure, not retired internals.
    doc_root = root / "docs/road"
    doc_tokens = (
        "regenerate_road",
        "BuildDerived",
        "BuildContext",
        "BuildPipeline",
        "BuildCanonicalAlignment",
        "AutoNodeLayout",
        "ResolvedNodeLayout",
        "MarkingAnchor",
        "MarkingIntent",
        "ResolvedMarkingGraph",
        "SectionEvaluationTable",
        "SamplingPlanTable",
    )
    for path in doc_root.rglob("*.md"):
        text = source_text(path)
        for token in doc_tokens:
            if token in text:
                errors.append(f"{relative(path)}: retired road design vocabulary remains in docs: {token!r}")
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
