#pragma once

#include "city/road/authoritative_types/road_graph.hpp"
#include "city/road/derived_types/derived_road.hpp"
#include "city/road/input_types/requests.hpp"

#include <optional>
#include <string>
#include <vector>

namespace city::road {

namespace operations {
struct OperationPlan;
}

[[nodiscard]] BezierSpan MakeLine(Vec2d a, Vec2d b);
[[nodiscard]] BezierSpan MakeBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3);
[[nodiscard]] Path MakePath(std::vector<BezierSpan> spans);
[[nodiscard]] bool IsLinearSpan(const BezierSpan& span);
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

  [[nodiscard]] Result<RoadSegmentId> AddSegment(AddSegmentRequest request);
  [[nodiscard]] Result<RoadSegmentId> AddSegmentConnectedTo(AddSegmentConnectedToRequest request);
  [[nodiscard]] Result<RoadSegmentId> AddSegmentConnectedToSegment(AddSegmentConnectedToSegmentRequest request);
  [[nodiscard]] Result<bool> EditSegmentShape(EditSegmentShapeRequest request);
  [[nodiscard]] Result<bool> MoveNode(MoveNodeRequest request);
  [[nodiscard]] Result<bool> DeleteSegment(DeleteSegmentRequest request);
  [[nodiscard]] Result<CrossSectionTemplateId> AddSectionTemplate(AddSectionTemplateRequest request);
  [[nodiscard]] Result<bool> EditSectionTemplate(EditSectionTemplateRequest request);
  [[nodiscard]] Result<SectionTransitionId> AddTransition(SectionTransitionRequest request);
  [[nodiscard]] Result<SectionTransitionId> AddTransitionToSegment(AddTransitionToSegmentRequest request);
  [[nodiscard]] Result<bool> AttachSectionTransition(AttachSectionTransitionRequest request);
  [[nodiscard]] Result<ManualMarkingId> AddManualLine(ManualLineRequest request);
  [[nodiscard]] Result<ManualMarkingId> AddManualArea(ManualAreaRequest request);
  [[nodiscard]] Result<std::string> Save() const;
  [[nodiscard]] static Result<RoadState> Load(const std::string& text);

private:
  [[nodiscard]] Result<bool> Execute(const operations::OperationPlan& plan);
  [[nodiscard]] Result<bool> BuildDerived();

  SavedRoadGraph graph_{};
  DerivedRoad derived_{};
  std::uint64_t next_id_ = 1;
};

[[nodiscard]] Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived);
[[nodiscard]] Result<SegmentShape> SegmentShapeFromPath(const Path& path);
[[nodiscard]] Result<Path> BuildCanonicalAlignment(Vec2d start, Vec2d end, const SegmentShape& shape);
[[nodiscard]] const Path* FindCanonicalAlignment(const DerivedRoad& derived, RoadSegmentId segment_id);
[[nodiscard]] Result<bool> ValidatePath(const Path& path);
[[nodiscard]] Result<double> PathLength(const Path& path);
[[nodiscard]] Result<Vec2d> EvaluatePath(const Path& path, double station_m);
[[nodiscard]] std::vector<Vec2d> FlattenPath(const Path& path);

} // namespace city::road
