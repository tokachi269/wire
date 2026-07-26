#pragma once

#include "city/road/common_types.hpp"

#include <array>
#include <string>

namespace city::road {

enum class EndpointRole {
  kStart,
  kEnd,
};

struct ApproachKey {
  RoadNodeId node_id = 0;
  RoadSegmentId segment_id = 0;
  EndpointRole endpoint_role = EndpointRole::kStart;

  bool operator==(const ApproachKey&) const = default;
};

struct SectionBoundarySample {
  std::uint64_t boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  double lateral_m = 0.0;
  double height_m = 0.0;
  MarkingRule marking_rule = MarkingRule::kNone;
};
struct SectionEvaluation {
  RoadSegmentId segment_id = 0;
  double station_m = 0.0;
  CrossSectionTemplateId resolved_template_id = 0;
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<std::string> surface_materials{};
};
struct Mesh {
  RoadSegmentId owner_segment_id = 0;
  std::string material{};
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
  std::string material{};
  std::uint64_t left_boundary_id = 0;
  std::uint64_t right_boundary_id = 0;
  std::vector<Vec3d> left{};
  std::vector<Vec3d> right{};
};

struct ResolvedSurfaceRegion {
  std::string material{};
  std::vector<Vec3d> perimeter{};
};

struct ResolvedAutoMarking {
  ApproachKey approach{};
  std::string material{};
  std::vector<std::array<Vec3d, 4>> quads{};
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
  std::vector<ResolvedAutoMarking> auto_markings{};
};

struct DerivedRoad {
  std::array<std::size_t, 9> build_stage_runs{};
  std::size_t setback_calculation_count = 0;
  std::size_t section_evaluation_count = 0;
  std::vector<CanonicalAlignment> canonical_alignments{};
  std::vector<NodeConnectionDecision> node_connection_decisions{};
  std::vector<SegmentSamplingPlan> sampling_plans{};
  std::vector<SectionEvaluation> section_evaluations{};
  std::vector<Mesh> segment_meshes{};
  std::vector<Mesh> marking_meshes{};
  std::vector<TerrainMaskPolygon> terrain_masks{};
  std::vector<ConnectionGate> connection_gates{};
  std::vector<ConnectionArea> connection_areas{};
  std::vector<Mesh> connection_meshes{};
  std::vector<JunctionArea> junction_areas{};
  std::vector<ConnectionGeometry> connection_geometries{};
  std::vector<JunctionGeometry> junction_geometries{};
  std::vector<Mesh> junction_meshes{};
  std::vector<Mesh> junction_marking_meshes{};
  std::vector<Mesh> manual_marking_meshes{};
};

} // namespace city::road
