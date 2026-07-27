#include "stages.hpp"

#include "stage_support.hpp"
#include "../materialization/materialize.hpp"

#include <algorithm>
#include <array>
#include <iterator>

namespace city::road::build {
namespace {

double section_height(const std::vector<SectionBoundarySample>& boundaries,
                      double lateral_m) {
  if (boundaries.empty()) return 0.0;
  if (lateral_m <= boundaries.front().lateral_m) {
    return boundaries.front().height_m;
  }
  for (std::size_t index = 1; index < boundaries.size(); ++index) {
    const SectionBoundarySample& a = boundaries[index - 1];
    const SectionBoundarySample& b = boundaries[index];
    if (lateral_m > b.lateral_m) continue;
    const double width = b.lateral_m - a.lateral_m;
    const double t =
        width <= kStationEpsilon ? 0.0 : (lateral_m - a.lateral_m) / width;
    return a.height_m + (b.height_m - a.height_m) * t;
  }
  return boundaries.back().height_m;
}

Result<Vec3d> owner_local_point(const DerivedRoad& derived,
                                const Path& alignment,
                                RoadSegmentId segment_id,
                                Vec2d local,
                                double elevation_m) {
  const Result<Vec2d> center = EvaluatePath(alignment, local.x);
  const Result<Vec2d> tangent = TangentAt(alignment, local.x);
  const SectionEvaluation* section =
      FindSectionEvaluation(derived, segment_id, local.x);
  if (!center.ok || !tangent.ok || section == nullptr) {
    return Result<Vec3d>::Fail(
        ErrorKind::kInternal, "manual marking resolved frame is missing");
  }
  const Vec2d lateral{-tangent.value.y, tangent.value.x};
  const Vec2d point = Add(center.value, Scale(lateral, local.y));
  return Result<Vec3d>::Ok(
      Vec3d{point.x, point.y,
            section_height(section->boundaries, local.y) + elevation_m});
}

Result<bool> materialize(BuildContext& context) {
  for (const CanonicalAlignment& canonical :
       context.derived.canonical_alignments) {
    const SegmentSamplingPlan* plan =
        FindSamplingPlan(context.derived, canonical.segment_id);
    if (plan == nullptr || plan->surface_stations_m.empty()) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road materialization sampling plan is missing");
    }
    materialization::SegmentInput input{};
    input.segment_id = canonical.segment_id;
    for (const double station : plan->surface_stations_m) {
      const Result<Vec2d> center = EvaluatePath(canonical.path, station);
      const Result<Vec2d> tangent = TangentAt(canonical.path, station);
      const SectionEvaluation* section = FindSectionEvaluation(
          context.derived, canonical.segment_id, station);
      if (!center.ok || !tangent.ok || section == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal, "road materialization sample is missing");
      }
      input.samples.push_back(materialization::SegmentSample{
          center.value, tangent.value, section->boundaries,
          section->surface_styles});
    }
    Result<materialization::SegmentOutput> output =
        materialization::MaterializeSegment(input);
    if (!output.ok) return Result<bool>::Fail(output.error_kind, output.error);
    context.derived.segment_meshes.insert(
        context.derived.segment_meshes.end(),
        std::make_move_iterator(output.value.surface_meshes.begin()),
        std::make_move_iterator(output.value.surface_meshes.end()));
    context.derived.marking_meshes.insert(
        context.derived.marking_meshes.end(),
        std::make_move_iterator(output.value.marking_meshes.begin()),
        std::make_move_iterator(output.value.marking_meshes.end()));
    context.derived.terrain_masks.push_back(
        std::move(output.value.terrain_mask));
  }

  for (const ConnectionGeometry& geometry :
       context.derived.connection_geometries) {
    Result<std::vector<Mesh>> meshes =
        materialization::MaterializeConnection(geometry);
    if (!meshes.ok) return Result<bool>::Fail(meshes.error_kind, meshes.error);
    context.derived.connection_meshes.insert(
        context.derived.connection_meshes.end(),
        std::make_move_iterator(meshes.value.begin()),
        std::make_move_iterator(meshes.value.end()));
  }
  for (const JunctionGeometry& geometry :
       context.derived.junction_geometries) {
    Result<materialization::JunctionOutput> output =
        materialization::MaterializeJunction(geometry);
    if (!output.ok) return Result<bool>::Fail(output.error_kind, output.error);
    context.derived.junction_meshes.insert(
        context.derived.junction_meshes.end(),
        std::make_move_iterator(output.value.surface_meshes.begin()),
        std::make_move_iterator(output.value.surface_meshes.end()));
    context.derived.junction_marking_meshes.insert(
        context.derived.junction_marking_meshes.end(),
        std::make_move_iterator(output.value.marking_meshes.begin()),
        std::make_move_iterator(output.value.marking_meshes.end()));
  }

  for (const ManualLineMarking& marking :
       context.authoritative.manual_lines) {
    const Path* alignment =
        FindAlignment(context.derived, marking.owner_segment_id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "manual line owner alignment is missing");
    }
    materialization::ManualLineInput input{};
    input.marking_id = marking.id;
    input.style = RenderStyleFromMarking(marking.style_id);
    const std::vector<Vec2d> points = FlattenPath(marking.path);
    constexpr double half_width = 0.05;
    for (std::size_t index = 0; index < points.size(); ++index) {
      Vec2d direction =
          index + 1 < points.size()
              ? Subtract(points[index + 1], points[index])
              : Subtract(points[index], points[index - 1]);
      direction = Normalize(direction);
      const Vec2d normal{-direction.y, direction.x};
      Result<Vec3d> left = owner_local_point(
          context.derived, *alignment, marking.owner_segment_id,
          Add(points[index], Scale(normal, -half_width)), 0.02);
      Result<Vec3d> right = owner_local_point(
          context.derived, *alignment, marking.owner_segment_id,
          Add(points[index], Scale(normal, half_width)), 0.02);
      if (!left.ok || !right.ok) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "manual line resolved geometry is invalid");
      }
      input.left.push_back(left.value);
      input.right.push_back(right.value);
    }
    Result<Mesh> mesh = materialization::MaterializeManualLine(input);
    if (!mesh.ok) return Result<bool>::Fail(mesh.error_kind, mesh.error);
    context.derived.manual_marking_meshes.push_back(std::move(mesh.value));
  }

  for (const ManualAreaMarking& marking :
       context.authoritative.manual_areas) {
    const Path* alignment =
        FindAlignment(context.derived, marking.owner_segment_id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "manual area owner alignment is missing");
    }
    materialization::ManualAreaInput input{};
    input.marking_id = marking.id;
    input.style = RenderStyleFromMarking(marking.style_id);
    const double half_width = marking.width_m * 0.5;
    const double half_length = marking.length_m * 0.5;
    const std::array<Vec2d, 4> locals{
        Vec2d{marking.frame_origin.x - half_length,
              marking.frame_origin.y - half_width},
        Vec2d{marking.frame_origin.x + half_length,
              marking.frame_origin.y - half_width},
        Vec2d{marking.frame_origin.x - half_length,
              marking.frame_origin.y + half_width},
        Vec2d{marking.frame_origin.x + half_length,
              marking.frame_origin.y + half_width},
    };
    for (std::size_t index = 0; index < locals.size(); ++index) {
      Result<Vec3d> point = owner_local_point(
          context.derived, *alignment, marking.owner_segment_id, locals[index],
          0.025);
      if (!point.ok) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "manual area resolved geometry is invalid");
      }
      input.corners[index] = point.value;
    }
    Result<Mesh> mesh = materialization::MaterializeManualArea(input);
    if (!mesh.ok) return Result<bool>::Fail(mesh.error_kind, mesh.error);
    context.derived.manual_marking_meshes.push_back(std::move(mesh.value));
  }
  return Result<bool>::Ok(true);
}

} // namespace

Result<DerivedRoad> BuildRoad(const SavedRoadGraph& authoritative) {
  BuildContext context(authoritative);
  const auto run = [&context](Stage stage, auto&& function) -> Result<bool> {
    Result<bool> begun = context.Begin(stage);
    if (!begun.ok) return begun;
    return function(context);
  };

  Result<bool> result =
      run(Stage::kTopologyIndex, BuildTopologyIndex);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kCanonicalAlignment, BuildCanonicalAlignments);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kNodeConnectionDecision, BuildNodeConnectionDecisions);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kAutoNodeLayout, BuildAutoNodeLayouts);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kResolvedNodeLayout, BuildResolvedNodeLayouts);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kSamplingPlan, BuildSamplingPlans);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kSectionEvaluation, BuildSectionEvaluations);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kConnectionGate, BuildConnectionGates);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kJunctionGeometry, BuildJunctionGeometries);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kMarkingAnchor, BuildMarkingAnchors);
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(Stage::kMaterialization,
               [](BuildContext& value) { return materialize(value); });
  if (!result.ok) return Result<DerivedRoad>::Fail(result.error_kind, result.error);

  Result<bool> begun = context.Begin(Stage::kDerivedInvariant);
  if (!begun.ok) return Result<DerivedRoad>::Fail(begun.error_kind, begun.error);
  context.derived.build_stage_runs = context.stage_runs;
  Result<bool> invariant =
      ValidateGraphInvariants(authoritative, context.derived);
  if (!invariant.ok) {
    return Result<DerivedRoad>::Fail(invariant.error_kind, invariant.error);
  }
  return Result<DerivedRoad>::Ok(std::move(context.derived));
}

} // namespace city::road::build
