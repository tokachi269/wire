#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace city::road {

struct Vec2d {
  double x = 0.0;
  double y = 0.0;
};

struct Vec3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

enum class ErrorKind {
  kNone,
  kValidation,
  kUnsupported,
  kInternal,
};

template <typename T>
struct Result {
  bool ok = false;
  T value{};
  ErrorKind error_kind = ErrorKind::kNone;
  std::string error{};

  [[nodiscard]] static Result<T> Ok(T value) {
    Result<T> result{};
    result.ok = true;
    result.value = std::move(value);
    return result;
  }

  [[nodiscard]] static Result<T> Fail(ErrorKind kind, std::string error) {
    Result<T> result{};
    result.ok = false;
    result.error_kind = kind;
    result.error = std::move(error);
    return result;
  }
};

struct Primitive {
  enum class Kind {
    kLine,
    kArc,
    kBezier,
  };

  Kind kind = Kind::kLine;
  Vec2d p0{};
  Vec2d p1{};
  Vec2d p2{};
  Vec2d p3{};
  Vec2d center{};
  double radius = 0.0;
  double start_angle_rad = 0.0;
  double sweep_angle_rad = 0.0;
};

struct Path {
  std::vector<Primitive> primitives{};
};

using RoadNodeId = std::uint64_t;
using RoadSegmentId = std::uint64_t;
using CrossSectionTemplateId = std::uint64_t;
using SectionTransitionId = std::uint64_t;
using ManualMarkingId = std::uint64_t;
using JunctionDefinitionId = std::uint64_t;

enum class SurfaceRole {
  kCarriageway,
  kSidewalk,
  kMedian,
};

enum class BoundaryRole {
  kOuterEdge,
  kCurb,
  kLaneDivider,
  kMedianEdge,
};

enum class MarkingRule {
  kNone,
  kCenterLine,
  kOuterLine,
};

struct SurfaceBand {
  std::uint64_t element_id = 0;
  SurfaceRole role = SurfaceRole::kCarriageway;
  double width_m = 0.0;
  double cross_slope = 0.0;
  std::string style{};
};

struct BoundaryProfile {
  std::uint64_t boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  double width_m = 0.0;
  double height_m = 0.0;
  MarkingRule marking_rule = MarkingRule::kNone;
};

struct CrossSectionTemplate {
  CrossSectionTemplateId id = 0;
  std::vector<SurfaceBand> bands{};
  std::vector<BoundaryProfile> boundaries{};
};

enum class StationRefKind {
  kFromStart,
  kFromEnd,
  kRatio,
};

struct StationRef {
  StationRefKind kind = StationRefKind::kFromStart;
  double value = 0.0;
};

enum class TransitionAction {
  kContinue,
  kChangeWidthHeightOffset,
  kTaperOut,
  kTaperIn,
  kEndCap,
  kUnsupported,
};

enum class TransitionAnchor {
  kCenter,
  kLeftEdge,
  kRightEdge,
};

struct SectionTransitionRule {
  std::uint64_t element_id = 0;
  TransitionAction action = TransitionAction::kContinue;
};

struct SectionTransition {
  SectionTransitionId id = 0;
  CrossSectionTemplateId from_template = 0;
  CrossSectionTemplateId to_template = 0;
  StationRef start{};
  StationRef end{};
  TransitionAnchor anchor = TransitionAnchor::kCenter;
  std::vector<SectionTransitionRule> rules{};
};

struct RoadNode {
  RoadNodeId id = 0;
  Vec2d position{};
};

struct RoadSegment {
  RoadSegmentId id = 0;
  RoadNodeId node_a = 0;
  RoadNodeId node_b = 0;
  Path alignment{};
  CrossSectionTemplateId section_template = 0;
  std::optional<SectionTransitionId> transition{};
};

struct JunctionDefinition {
  JunctionDefinitionId id = 0;
  RoadNodeId node_id = 0;
  double corner_radius_m = 4.0;
};

struct ManualLineMarking {
  ManualMarkingId id = 0;
  RoadSegmentId owner_segment_id = 0;
  Path path{};
  std::string style{};
};

struct ManualAreaMarking {
  ManualMarkingId id = 0;
  RoadSegmentId owner_segment_id = 0;
  Vec2d frame_origin{};
  double width_m = 0.0;
  double length_m = 0.0;
  std::string style{};
};

struct SavedRoadGraph {
  std::vector<RoadNode> nodes{};
  std::vector<RoadSegment> segments{};
  std::vector<CrossSectionTemplate> section_templates{};
  std::vector<SectionTransition> transitions{};
  std::vector<JunctionDefinition> junctions{};
  std::vector<ManualLineMarking> manual_lines{};
  std::vector<ManualAreaMarking> manual_areas{};
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
  std::vector<SectionBoundarySample> boundaries{};
};

struct Mesh {
  RoadSegmentId owner_segment_id = 0;
  std::string material{};
  std::vector<Vec3d> vertices{};
  std::vector<std::uint32_t> indices{};
};

struct TerrainMaskPolygon {
  RoadSegmentId segment_id = 0;
  std::vector<Vec2d> points{};
};

struct ConnectionGate {
  RoadSegmentId segment_id = 0;
  RoadNodeId node_id = 0;
  Vec3d position{};
  Vec3d tangent{};
  std::vector<SectionBoundarySample> boundaries{};
};

struct JunctionArea {
  JunctionDefinitionId definition_id = 0;
  RoadNodeId node_id = 0;
  std::vector<ConnectionGate> gates{};
};

struct DerivedRoad {
  std::vector<SectionEvaluation> section_evaluations{};
  std::vector<Mesh> segment_meshes{};
  std::vector<Mesh> marking_meshes{};
  std::vector<TerrainMaskPolygon> terrain_masks{};
  std::vector<ConnectionGate> connection_gates{};
  std::vector<JunctionArea> junction_areas{};
  std::vector<Mesh> junction_meshes{};
  std::vector<Mesh> junction_marking_meshes{};
  std::vector<Mesh> manual_marking_meshes{};
};

struct RoadToolDraft {
  Path preview_path{};
  bool has_live_preview = false;
  bool supports_bezier_handles = false;
};

[[nodiscard]] Primitive MakeLine(Vec2d a, Vec2d b);
[[nodiscard]] Primitive MakeArc(Vec2d center, double radius, double start_angle_rad, double sweep_angle_rad);
[[nodiscard]] Result<Primitive> MakeArcThroughPoints(Vec2d start, Vec2d through, Vec2d end);
[[nodiscard]] Primitive MakeBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3);
[[nodiscard]] Path MakePath(std::vector<Primitive> primitives);
[[nodiscard]] RoadToolDraft PreviewRoadToolPath(Vec2d start, Vec2d end, std::optional<Vec2d> handle_a,
                                                std::optional<Vec2d> handle_b);

[[nodiscard]] CrossSectionTemplate JapaneseUrbanTwoLaneTemplate(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate ThreeLaneTemplate(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate NoLeftSidewalkTemplate(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate MedianTwoLaneTemplate(CrossSectionTemplateId id);

class RoadState {
public:
  RoadState();

  [[nodiscard]] const SavedRoadGraph& graph() const;
  [[nodiscard]] const DerivedRoad& derived() const;

  [[nodiscard]] Result<RoadSegmentId> AddSegment(Path alignment, CrossSectionTemplateId section_template);
  [[nodiscard]] Result<RoadSegmentId> AddSegmentConnectedTo(Path alignment, CrossSectionTemplateId section_template,
                                                            RoadNodeId start_node);
  [[nodiscard]] Result<RoadSegmentId> AddSegmentConnectedToSegment(Path alignment,
                                                                   CrossSectionTemplateId section_template,
                                                                   RoadSegmentId start_segment);
  [[nodiscard]] Result<bool> EditSegmentPath(RoadSegmentId segment_id, Path alignment);
  [[nodiscard]] Result<bool> DeleteSegment(RoadSegmentId segment_id);
  [[nodiscard]] Result<CrossSectionTemplateId> AddSectionTemplate(CrossSectionTemplate section_template);
  [[nodiscard]] Result<bool> EditSectionTemplate(CrossSectionTemplate section_template);
  [[nodiscard]] Result<SectionTransitionId> AddTransition(SectionTransition transition);
  [[nodiscard]] Result<SectionTransitionId> AddTransitionToSegment(RoadSegmentId segment_id,
                                                                   SectionTransition transition);
  [[nodiscard]] Result<bool> AttachSectionTransition(RoadSegmentId segment_id,
                                                     SectionTransitionId transition_id);
  [[nodiscard]] Result<ManualMarkingId> AddManualLine(ManualLineMarking marking);
  [[nodiscard]] Result<ManualMarkingId> AddManualArea(ManualAreaMarking marking);
  [[nodiscard]] Result<bool> RebuildDerived();
  [[nodiscard]] Result<std::string> Save() const;
  [[nodiscard]] static Result<RoadState> Load(const std::string& text);

private:
  SavedRoadGraph graph_{};
  DerivedRoad derived_{};
  std::uint64_t next_id_ = 1;
};

[[nodiscard]] Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived);
[[nodiscard]] Result<bool> ValidatePath(const Path& path);
[[nodiscard]] Result<double> PathLength(const Path& path);
[[nodiscard]] Result<Vec2d> EvaluatePath(const Path& path, double station_m);
[[nodiscard]] std::vector<Vec2d> FlattenPath(const Path& path);

} // namespace city::road
