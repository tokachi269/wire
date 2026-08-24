#pragma once

#include "city/road/common_types.hpp"

#include <array>
#include <optional>
#include <tuple>

namespace city::road {

struct SectionBoundarySample {
  std::uint64_t boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  double lateral_m = 0.0;
  double height_m = 0.0;
  // Effective marking policy: the boundary policy and the adjacent lane side
  // policies merged into one during section evaluation.
  AutoMarkingPolicy marking{};
  bool hard_edge = false;
  // 0 where the sample does not meet the carriageway.
  double carriageway_side = 0.0;
  // lateral_m with the boundary's marking placement applied.
  double marking_lateral_m = 0.0;
  // Strips adjacent in template order; used to decide marking begin and end.
  RoadLayoutStripId left_strip_id = 0;
  RoadLayoutStripId right_strip_id = 0;
  double left_strip_width_m = 0.0;
  double right_strip_width_m = 0.0;
  double profile_v_m = 0.0;
};
enum class RenderStyleDomain {
  kSurface,
  kMarking,
};
struct RenderStyleRef {
  RenderStyleDomain domain = RenderStyleDomain::kSurface;
  std::uint64_t value = 0;

  bool operator==(const RenderStyleRef&) const = default;
  bool operator<(const RenderStyleRef& other) const {
    return std::tie(domain, value) < std::tie(other.domain, other.value);
  }
};
[[nodiscard]] inline RenderStyleRef RenderStyleFromSurface(SurfaceStyleId id) {
  return RenderStyleRef{RenderStyleDomain::kSurface, id.value};
}
[[nodiscard]] inline RenderStyleRef RenderStyleFromMarking(MarkingStyleId id) {
  return RenderStyleRef{RenderStyleDomain::kMarking, id.value};
}
enum class MeshUvMapping {
  kWorld,
  kPatchQuantized,
};
struct PatchUvSettings {
  double reference_length_m = 64.0;
  int seam_divisions = 4;
};
struct MeshMaterialGroup {
  RenderStyleRef style{};
  std::uint32_t index_start = 0;
  std::uint32_t index_count = 0;
};
struct SectionEvaluation {
  RoadSegmentId segment_id = 0;
  double segment_distance_m = 0.0;
  RoadLayoutTemplateId resolved_template_id = 0;
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<RenderStyleRef> surface_styles{};
};
struct Mesh {
  RoadSegmentId owner_segment_id = 0;
  RenderStyleRef style{};
  MeshUvMapping uv_mapping = MeshUvMapping::kWorld;
  std::vector<Vec3d> vertices{};
  std::vector<std::uint32_t> indices{};
  std::vector<Vec3d> normals{};
  std::vector<Vec2d> uv0{};
  std::vector<MeshMaterialGroup> material_groups{};
};
struct TerrainMaskPolygon { RoadSegmentId segment_id = 0; std::vector<Vec2d> points{}; };

enum class NodeConnectionKind {
  kPassThrough,
  kCorner,
  kJunction,
  kUnsupported,
};

struct ConnectionGate {
  ApproachKey approach{};
  RoadSegmentId segment_id = 0;
  RoadNodeId node_id = 0;
  Vec3d position{};
  Vec3d tangent{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
  std::vector<SectionBoundarySample> boundaries{};
};

struct ResolvedBoundaryCurve {
  std::uint64_t source_boundary_id = 0;
  std::uint64_t target_boundary_id = 0;
  BoundaryRole role = BoundaryRole::kOuterEdge;
  bool carries_marking = false;
  std::vector<Vec3d> points{};
  std::vector<Vec3d> marking_points{};
  ApproachKey source_approach{};
  ApproachKey target_approach{};
};

enum class SurfaceWinding {
  kLeftToRight,
  kRightToLeft,
};

struct ResolvedSurfaceStrip {
  RenderStyleRef style{};
  std::uint64_t left_boundary_id = 0;
  std::uint64_t right_boundary_id = 0;
  // The owner orders these rails so path direction crossed with left-to-right
  // is the front face. Emit preserves that winding for horizontal, sloped and
  // vertical faces instead of reinterpreting it against world up.
  std::vector<Vec3d> left{};
  std::vector<Vec3d> right{};
  SurfaceWinding winding = SurfaceWinding::kLeftToRight;
};

struct ResolvedSurfaceRegion {
  RenderStyleRef style{};
  std::vector<Vec3d> perimeter{};
};

struct ConnectionGeometry {
  RoadNodeId node_id = 0;
  std::array<ApproachKey, 2> approaches{};
  std::vector<ResolvedBoundaryCurve> boundary_curves{};
  std::vector<ResolvedSurfaceStrip> surface_strips{};
};

struct JunctionGeometry {
  RoadNodeId node_id = 0;
  std::vector<ApproachKey> ordered_approaches{};
  std::vector<ResolvedBoundaryCurve> perimeter_curves{};
  std::vector<ResolvedSurfaceRegion> surface_regions{};
  std::vector<ResolvedSurfaceStrip> surface_strips{};
};

// One segment resolved from the authoritative graph: canonical alignment, the
// distances every consumer shares, and the section evaluated at those distances.
struct DerivedSegment {
  RoadSegmentId id = 0;
  Path alignment{};
  double length_m = 0.0;
  double start_elevation_m = 0.0;
  double end_elevation_m = 0.0;
  double surface_start_m = 0.0;
  double surface_end_m = 0.0;
  std::vector<double> semantic_segment_distances_m{};
  std::vector<double> surface_segment_distances_m{};
  std::vector<SectionEvaluation> sections{};
};

// One approach of a connected node. Auto values and the values after user
// overrides live together so no second table is needed to compare them.
struct ResolvedApproach {
  ApproachKey key{};
  RoadLayoutTemplateId endpoint_template_id = 0;
  Vec3d position{};
  Vec3d tangent{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
  double auto_setback_m = 0.0;
  double resolved_setback_m = 0.0;
  double auto_lateral_shift_m = 0.0;
  double resolved_lateral_shift_m = 0.0;
  double gate_segment_distance_m = 0.0;
  ConnectionGate gate{};
};

struct ResolvedJunctionCorner {
  ApproachKey first_approach{};
  ApproachKey second_approach{};
  double radius_m = 0.0;
};

// One connected node: what it is, the approaches after override resolution and
// the geometry that follows from them.
struct ResolvedConnection {
  RoadNodeId node_id = 0;
  NodeConnectionKind kind = NodeConnectionKind::kUnsupported;
  NodeConnectionPolicyOverrideId applied_policy_override_id = 0;
  std::string reason{};
  // Degree-two corners have one pair. Junction radii are resolved per adjacent
  // approach pair and never collapsed into a connection-wide value.
  double corner_radius_m = 0.0;
  double corner_control_m = 0.0;
  std::vector<ResolvedJunctionCorner> junction_corners{};
  std::vector<ApproachKey> ordered_approaches{};
  std::vector<ResolvedApproach> approaches{};
  ConnectionGeometry connection_geometry{};
  JunctionGeometry junction_geometry{};
};

struct DerivedLanePath {
  LaneConnectionId connection_id = 0;
  Path centerline{};
  std::vector<Vec3d> points{};
  double length_m = 0.0;
  double minimum_radius_m = 0.0;
};

// A segment-owned lane centerline for inspection and editing. It reuses the
// segment section samples; it is not a render mesh or an authoritative path.
struct DerivedSegmentLanePath {
  RoadSegmentId segment_id = 0;
  LaneId lane_id = 0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
  RoadLayoutTemplateId start_template_id = 0;
  RoadLayoutTemplateId end_template_id = 0;
  double start_segment_distance_m = 0.0;
  double end_segment_distance_m = 0.0;
  std::vector<Vec3d> points{};
};

struct DerivedBoundaryPath {
  BoundaryContinuationId continuation_id = 0;
  Path path{};
  double length_m = 0.0;
};

struct DerivedSeparationArea {
  LaneConnectionId connection_id = 0;
  std::vector<Vec3d> perimeter{};
};

// One marking line or area, already placed in world space. The owner keeps
// identity explicit; emit only turns points into meshes.
struct DerivedMarking {
  MarkingOwner owner{};
  std::uint64_t boundary_id = 0;
  MarkingRole role = MarkingRole::kLaneSeparator;
  MarkingStyleId style_id{};
  double width_m = 0.05;
  std::vector<Vec3d> points{};
  std::vector<Vec3d> polygon{};
};

struct DerivedRoad {
  std::vector<DerivedSegment> segments{};
  std::vector<ResolvedConnection> connections{};
  std::vector<DerivedSegmentLanePath> segment_lane_paths{};
  std::vector<DerivedLanePath> lane_paths{};
  std::vector<DerivedBoundaryPath> boundary_paths{};
  std::vector<DerivedSeparationArea> separation_areas{};
  std::vector<DerivedMarking> markings{};
  std::vector<Mesh> segment_meshes{};
  std::vector<Mesh> connection_meshes{};
  std::vector<Mesh> junction_meshes{};
  std::vector<Mesh> marking_meshes{};
  std::vector<TerrainMaskPolygon> terrain_masks{};
};

[[nodiscard]] const DerivedSegment* FindDerivedSegment(const DerivedRoad& derived,
                                                       RoadSegmentId segment_id);
[[nodiscard]] const ResolvedConnection* FindResolvedConnection(const DerivedRoad& derived,
                                                               RoadNodeId node_id);
[[nodiscard]] const ResolvedApproach* FindResolvedApproach(const DerivedRoad& derived,
                                                           const ApproachKey& key);
[[nodiscard]] const SectionEvaluation* FindSectionAt(const DerivedSegment& segment,
                                                     double segment_distance_m);

} // namespace city::road
