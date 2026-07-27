#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {

Result<bool> BuildMarkingAnchors(BuildContext& context) {
  context.derived.marking_anchors.clear();
  for (const RoadSegment& segment : context.authoritative.segments) {
    for (const SectionEvaluation& section : context.derived.section_evaluations) {
      if (section.segment_id != segment.id) continue;
      for (const SectionBoundarySample& boundary : section.boundaries) {
        context.derived.marking_anchors.push_back(MarkingAnchor{
            MarkingAnchorKind::kSectionBoundary,
            0,
            segment.id,
            {},
            boundary.boundary_id,
            Vec3d{section.station_m, boundary.lateral_m, boundary.height_m},
        });
        if (boundary.role == BoundaryRole::kLaneDivider) {
          context.derived.marking_anchors.push_back(MarkingAnchor{
              MarkingAnchorKind::kLaneSide,
              0,
              segment.id,
              {},
              boundary.boundary_id,
              Vec3d{section.station_m, boundary.lateral_m, boundary.height_m},
          });
        }
        if (boundary.role == BoundaryRole::kCurb || boundary.role == BoundaryRole::kOuterEdge) {
          context.derived.marking_anchors.push_back(MarkingAnchor{
              MarkingAnchorKind::kCarriagewayEdge,
              0,
              segment.id,
              {},
              boundary.boundary_id,
              Vec3d{section.station_m, boundary.lateral_m, boundary.height_m},
          });
        }
      }
    }
  }
  for (const ResolvedNodeLayout& layout : context.derived.resolved_node_layouts) {
    for (const ResolvedApproachLayout& approach : layout.approaches) {
      context.derived.marking_anchors.push_back(MarkingAnchor{
          MarkingAnchorKind::kApproachGate,
          layout.node_id,
          approach.key.segment_id,
          approach.key,
          0,
          approach.position,
      });
      context.derived.marking_anchors.push_back(MarkingAnchor{
          MarkingAnchorKind::kApproachCenter,
          layout.node_id,
          approach.key.segment_id,
          approach.key,
          0,
          approach.position,
      });
    }
    if (layout.kind == NodeConnectionKind::kJunction) {
      for (const ApproachKey& key : layout.ordered_approaches) {
        const ResolvedApproachLayout* approach = FindResolvedApproachLayout(layout, key);
        if (approach == nullptr) {
          return Result<bool>::Fail(ErrorKind::kInternal, "road marking anchor resolved approach is missing");
        }
        context.derived.marking_anchors.push_back(MarkingAnchor{
            MarkingAnchorKind::kJunctionCorner,
            layout.node_id,
            key.segment_id,
            key,
            0,
            approach->position,
        });
      }
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
