#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

#include <cstdint>

namespace city::road::build {
namespace {

std::uint64_t mix(std::uint64_t seed, std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
  return seed;
}

MarkingAnchorId anchor_id(MarkingOwner owner, ApproachKey approach,
                          std::uint64_t boundary_id, MarkingAnchorKind kind,
                          MarkingRole role) {
  std::uint64_t seed = 0x8f7c5d1e6a4b3920ull;
  seed = mix(seed, static_cast<std::uint64_t>(owner.kind));
  seed = mix(seed, owner.segment_id);
  seed = mix(seed, owner.node_id);
  seed = mix(seed, owner.manual_id);
  seed = mix(seed, approach.node_id);
  seed = mix(seed, approach.segment_id);
  seed = mix(seed, static_cast<std::uint64_t>(approach.endpoint_role));
  seed = mix(seed, boundary_id);
  seed = mix(seed, static_cast<std::uint64_t>(kind));
  seed = mix(seed, static_cast<std::uint64_t>(role));
  return seed == 0 ? 1 : seed;
}

MarkingRole role_for_boundary(const SectionBoundarySample &boundary) {
  if (boundary.marking.enabled)
    return boundary.marking.role;
  if (boundary.role == BoundaryRole::kLaneDivider)
    return MarkingRole::kLaneSeparator;
  return MarkingRole::kFree;
}

Result<MarkingAnchor>
segment_anchor(const DerivedRoad &out, const RoadSegment &segment,
               const Path &alignment, const SectionEvaluation &section,
               const SectionBoundarySample &boundary, MarkingAnchorKind kind) {
  const Result<Vec2d> center = EvaluatePath(alignment, section.station_m);
  const Result<Vec2d> tangent2 = tangent_at(alignment, section.station_m);
  if (!center.ok || !tangent2.ok) {
    return Result<MarkingAnchor>::Fail(
        ErrorKind::kInternal, "marking anchor segment frame is missing");
  }
  const Vec2d lateral2{-tangent2.value.y, tangent2.value.x};
  const Vec2d point = add(center.value, scale(lateral2, boundary.lateral_m));
  const MarkingOwner owner{MarkingOwner::Kind::kRoadSegment, segment.id, 0, 0};
  const MarkingRole role = role_for_boundary(boundary);
  MarkingAnchor anchor{};
  anchor.id = anchor_id(owner, {}, boundary.boundary_id, kind, role);
  anchor.kind = kind;
  anchor.owner = owner;
  anchor.boundary_id = boundary.boundary_id;
  anchor.role = role;
  anchor.world_position = {point.x, point.y, boundary.height_m};
  anchor.tangent = {tangent2.value.x, tangent2.value.y, 0.0};
  anchor.lateral = {lateral2.x, lateral2.y, 0.0};
  anchor.normal = {0.0, 0.0, 1.0};
  (void)out;
  return Result<MarkingAnchor>::Ok(anchor);
}

} // namespace

Result<bool> make_marking_anchors(pipeline &pipe) {
  pipe.out.marking_anchors.clear();
  for (const RoadSegment &segment : pipe.source.segments) {
    const Path *alignment = find_alignment(pipe.out, segment.id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "marking anchor alignment is missing");
    }
    for (const SectionEvaluation &section : pipe.out.sections) {
      if (section.segment_id != segment.id)
        continue;
      for (const SectionBoundarySample &boundary : section.boundaries) {
        Result<MarkingAnchor> anchor =
            segment_anchor(pipe.out, segment, *alignment, section, boundary,
                           MarkingAnchorKind::kSectionBoundary);
        if (!anchor.ok)
          return Result<bool>::Fail(anchor.error_kind, anchor.error);
        pipe.out.marking_anchors.push_back(anchor.value);
      }
    }
  }
  for (const ResolvedNodeLayout &layout : pipe.out.layouts) {
    for (const ResolvedApproachLayout &approach : layout.approaches) {
      const MarkingOwner owner{MarkingOwner::Kind::kJunction, 0, layout.node_id,
                               0};
      MarkingAnchor gate{};
      gate.id = anchor_id(owner, approach.key, 0,
                          MarkingAnchorKind::kApproachGate, MarkingRole::kFree);
      gate.kind = MarkingAnchorKind::kApproachGate;
      gate.owner = owner;
      gate.approach = approach.key;
      gate.role = MarkingRole::kFree;
      gate.world_position = approach.position;
      gate.tangent = approach.tangent;
      gate.lateral = approach.lateral;
      gate.normal = approach.normal;
      pipe.out.marking_anchors.push_back(gate);

      MarkingAnchor center = gate;
      center.id =
          anchor_id(owner, approach.key, 0, MarkingAnchorKind::kApproachCenter,
                    MarkingRole::kFree);
      center.kind = MarkingAnchorKind::kApproachCenter;
      pipe.out.marking_anchors.push_back(center);
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
