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
  // Bands adjacent in element order; used to decide marking begin and end.
  std::uint64_t left_element_id = 0;
  std::uint64_t right_element_id = 0;
  double left_band_width_m = 0.0;
  double right_band_width_m = 0.0;
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
struct SectionEvaluation {
  RoadSegmentId segment_id = 0;
  double station_m = 0.0;
  CrossSectionTemplateId resolved_template_id = 0;
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<RenderStyleRef> surface_styles{};
};
struct Mesh {
  RoadSegmentId owner_segment_id = 0;
  RenderStyleRef style{};
  std::vector<Vec3d> vertices{};
  std::vector<std::uint32_t> indices{};
};
struct TerrainMaskPolygon { RoadSegmentId segment_id = 0; std::vector<Vec2d> points{}; };

enum class NodeConnectionKind {
  kPassThrough,
  kCorner,
  kJunction,
  kUnsupported,
};

struct ApproachConnectionDecision {
  ApproachKey key{};
  CrossSectionTemplateId endpoint_template_id = 0;
  double setback_m = 0.0;
  double gate_station_m = 0.0;
};

struct NodeConnectionDecision {
  RoadNodeId node_id = 0;
  NodeConnectionKind kind = NodeConnectionKind::kUnsupported;
  std::vector<ApproachKey> ordered_approaches{};
  std::vector<ApproachConnectionDecision> approaches{};
  double corner_radius_m = 0.0;
  double corner_control_m = 0.0;
  double junction_corner_control_m = 0.0;
  NodeConnectionPolicyOverrideId applied_policy_override_id = 0;
  std::string reason{};
};

struct SegmentSamplingPlan {
  RoadSegmentId segment_id = 0;
  std::vector<double> semantic_stations_m{};
  std::vector<double> surface_stations_m{};
  std::vector<double> marking_stations_m{};
  std::vector<double> mask_stations_m{};
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
struct AutoApproachLayout {
  ApproachKey key{};
  Vec3d base_position{};
  Vec3d tangent{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
  double setback_m = 0.0;
  double lateral_shift_m = 0.0;
  double gate_station_m = 0.0;
  ConnectionGate auto_gate{};
};
struct AutoNodeLayout {
  RoadNodeId node_id = 0;
  NodeConnectionKind kind = NodeConnectionKind::kUnsupported;
  std::vector<ApproachKey> ordered_approaches{};
  std::vector<AutoApproachLayout> approaches{};
};
struct ResolvedApproachLayout {
  ApproachKey key{};
  Vec3d position{};
  Vec3d tangent{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
  double setback_m = 0.0;
  double lateral_shift_m = 0.0;
  double gate_station_m = 0.0;
  ConnectionGate gate{};
};
struct ResolvedNodeLayout {
  RoadNodeId node_id = 0;
  NodeConnectionKind kind = NodeConnectionKind::kUnsupported;
  std::vector<ApproachKey> ordered_approaches{};
  std::vector<ResolvedApproachLayout> approaches{};
};
enum class MarkingAnchorKind {
  kSectionBoundary,
  kLaneSide,
  kCarriagewayEdge,
  kApproachGate,
  kApproachCenter,
  kJunctionCorner,
};
using MarkingAnchorId = std::uint64_t;
struct MarkingAnchor {
  MarkingAnchorId id = 0;
  MarkingAnchorKind kind = MarkingAnchorKind::kApproachGate;
  MarkingOwner owner{};
  ApproachKey approach{};
  std::uint64_t boundary_id = 0;
  MarkingRole role = MarkingRole::kFree;
  Vec3d world_position{};
  Vec3d tangent{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
};
struct JunctionArea {
  NodeConnectionPolicyOverrideId policy_override_id = 0;
  RoadNodeId node_id = 0;
  std::vector<ConnectionGate> gates{};
};
struct ConnectionArea { RoadNodeId node_id = 0; std::vector<ConnectionGate> gates{}; };
struct CanonicalAlignment { RoadSegmentId segment_id = 0; Path path{}; };

struct ResolvedBoundaryCurve {
  std::uint64_t source_boundary_id = 0;
  std::uint64_t target_boundary_id = 0;
  BoundaryRole role = BoundaryRole::kOuterEdge;
  std::vector<Vec3d> points{};
};

struct ResolvedSurfaceStrip {
  RenderStyleRef style{};
  std::uint64_t left_boundary_id = 0;
  std::uint64_t right_boundary_id = 0;
  std::vector<Vec3d> left{};
  std::vector<Vec3d> right{};
};

struct ResolvedSurfaceRegion {
  RenderStyleRef style{};
  std::vector<Vec3d> perimeter{};
};

using MarkingIntentId = std::uint64_t;
using ResolvedMarkingId = std::uint64_t;
enum class MarkingGeometryRule {
  kFollowBoundary,
  kConnectAnchors,
  kOwnerLocalPath,
  kOwnerLocalArea,
};
enum class MarkingContinuationAction {
  kContinue,
  kBegin,
  kTerminate,
  kConnectAcrossJunction,
  kSuppress,
  kUnsupported,
};
struct MarkingStyleDefinition {
  MarkingStyleId style_id{};
  RenderStyleRef render_style{};
  double width_m = 0.05;
};
// kFollowBoundary requests are collected as per-station samples; the extent of
// the run is decided by MarkingContinuation.
struct MarkingBoundarySample {
  double station_m = 0.0;
  double lateral_m = 0.0;
  double min_adjacent_band_width_m = 0.0;
  Vec3d position{};
};
struct MarkingIntent {
  MarkingIntentId id = 0;
  MarkingOwner owner{};
  MarkingRole role = MarkingRole::kLaneSeparator;
  MarkingStyleId style_id{};
  MarkingGeometryRule geometry = MarkingGeometryRule::kFollowBoundary;
  std::optional<MarkingTrackKey> track{};
  std::vector<MarkingAnchorId> anchors{};
  std::vector<MarkingBoundarySample> boundary_samples{};
  double range_start_m = 0.0;
  double range_end_m = 0.0;
  MarkingContinuationAction source_action = MarkingContinuationAction::kContinue;
  MarkingContinuationAction end_action = MarkingContinuationAction::kContinue;
  std::vector<Vec3d> world_path{};
  std::vector<Vec3d> world_polygon{};
};
struct ResolvedMarkingPath {
  ResolvedMarkingId id = 0;
  MarkingOwner owner{};
  MarkingRole role = MarkingRole::kLaneSeparator;
  MarkingStyleId style_id{};
  MarkingContinuationAction source_action = MarkingContinuationAction::kContinue;
  MarkingContinuationAction end_action = MarkingContinuationAction::kContinue;
  std::vector<Vec3d> centerline{};
  double width_m = 0.05;
};
struct ResolvedMarkingArea {
  ResolvedMarkingId id = 0;
  MarkingOwner owner{};
  MarkingRole role = MarkingRole::kFree;
  MarkingStyleId style_id{};
  Vec3d origin{};
  Vec3d forward{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
  std::vector<std::vector<Vec3d>> polygons{};
};
struct ResolvedMarkingGraph {
  std::vector<ResolvedMarkingPath> paths{};
  std::vector<ResolvedMarkingArea> areas{};
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

struct DerivedRoad {
  std::array<std::size_t, 15> build_stage_runs{};
  std::size_t setback_calculation_count = 0;
  std::size_t section_evaluation_count = 0;
  std::vector<CanonicalAlignment> alignments{};
  std::vector<NodeConnectionDecision> decisions{};
  std::vector<SegmentSamplingPlan> sampling{};
  std::vector<SectionEvaluation> sections{};
  std::vector<AutoNodeLayout> auto_layouts{};
  std::vector<ResolvedNodeLayout> layouts{};
  std::vector<MarkingAnchor> marking_anchors{};
  std::vector<MarkingIntent> marking_intents{};
  ResolvedMarkingGraph markings{};
  std::vector<Mesh> segment_meshes{};
  std::vector<Mesh> marking_meshes{};
  std::vector<TerrainMaskPolygon> terrain_masks{};
  std::vector<ConnectionGate> gates{};
  std::vector<ConnectionArea> connection_areas{};
  std::vector<Mesh> connection_meshes{};
  std::vector<JunctionArea> junction_areas{};
  std::vector<ConnectionGeometry> connections{};
  std::vector<JunctionGeometry> junctions{};
  std::vector<Mesh> junction_meshes{};
};

} // namespace city::road
