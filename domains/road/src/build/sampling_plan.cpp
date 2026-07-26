#include "stages.hpp"

#include "stage_support.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::build {
namespace {

double station_value(StationRef ref, double total_m) {
  if (ref.kind == StationRefKind::kFromEnd) return total_m - ref.value;
  if (ref.kind == StationRefKind::kRatio) return total_m * ref.value;
  return ref.value;
}

void append_if_in_range(std::vector<double>& stations, double station_m, double total_m) {
  if (station_m >= -kStationEpsilon && station_m <= total_m + kStationEpsilon) {
    stations.push_back(std::clamp(station_m, 0.0, total_m));
  }
}

} // namespace

Result<bool> BuildSamplingPlans(BuildContext& context) {
  context.derived.sampling_plans.clear();
  context.derived.sampling_plans.reserve(context.authoritative.segments.size());

  for (const RoadSegment& segment : context.authoritative.segments) {
    const Path* alignment = FindAlignment(context.derived, segment.id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road segment canonical alignment is missing");
    }
    const Result<double> length = PathLength(*alignment);
    if (!length.ok) return Result<bool>::Fail(length.error_kind, length.error);

    SegmentSamplingPlan plan{};
    plan.segment_id = segment.id;
    plan.semantic_stations_m = {0.0, length.value};

    double span_boundary = 0.0;
    for (std::size_t index = 0; index + 1 < alignment->spans.size(); ++index) {
      const Result<double> span_length =
          PathLength(MakePath({alignment->spans[index]}));
      if (!span_length.ok) {
        return Result<bool>::Fail(span_length.error_kind, span_length.error);
      }
      span_boundary += span_length.value;
      plan.semantic_stations_m.push_back(span_boundary);
    }

    if (segment.transition.has_value()) {
      const SectionTransition* transition =
          FindTransition(context.authoritative, *segment.transition);
      if (transition == nullptr) {
        return Result<bool>::Fail(ErrorKind::kValidation,
                                  "road segment transition is missing");
      }
      append_if_in_range(plan.semantic_stations_m,
                         station_value(transition->start, length.value), length.value);
      append_if_in_range(plan.semantic_stations_m,
                         station_value(transition->end, length.value), length.value);
    }

    const ApproachKey start_key =
        ApproachKey{segment.node_a, segment.id, EndpointRole::kStart};
    const ApproachKey end_key =
        ApproachKey{segment.node_b, segment.id, EndpointRole::kEnd};
    const NodeConnectionDecision* start_decision =
        FindDecision(context.derived, segment.node_a);
    const NodeConnectionDecision* end_decision =
        FindDecision(context.derived, segment.node_b);
    const ApproachConnectionDecision* start =
        start_decision == nullptr
            ? nullptr
            : FindApproachDecision(*start_decision, start_key);
    const ApproachConnectionDecision* end =
        end_decision == nullptr ? nullptr
                                : FindApproachDecision(*end_decision, end_key);
    if ((start_decision != nullptr && start == nullptr) ||
        (end_decision != nullptr && end == nullptr)) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road segment approach decision is missing");
    }
    const double surface_start =
        start == nullptr ? 0.0 : start->gate_station_m;
    const double surface_end =
        end == nullptr ? length.value : end->gate_station_m;
    plan.semantic_stations_m.push_back(surface_start);
    plan.semantic_stations_m.push_back(surface_end);

    for (const ManualLineMarking& marking : context.authoritative.manual_lines) {
      if (marking.owner_segment_id != segment.id) continue;
      const std::vector<Vec2d> points = FlattenPath(marking.path);
      constexpr double half_width = 0.05;
      for (std::size_t index = 0; index < points.size(); ++index) {
        const Vec2d point = points[index];
        append_if_in_range(plan.semantic_stations_m, point.x, length.value);
        Vec2d direction =
            index + 1 < points.size()
                ? Subtract(points[index + 1], points[index])
                : Subtract(points[index], points[index - 1]);
        direction = Normalize(direction);
        const Vec2d normal{-direction.y, direction.x};
        append_if_in_range(plan.semantic_stations_m,
                           point.x - normal.x * half_width, length.value);
        append_if_in_range(plan.semantic_stations_m,
                           point.x + normal.x * half_width, length.value);
      }
    }
    for (const ManualAreaMarking& marking : context.authoritative.manual_areas) {
      if (marking.owner_segment_id != segment.id) continue;
      append_if_in_range(plan.semantic_stations_m,
                         marking.frame_origin.x - marking.length_m * 0.5, length.value);
      append_if_in_range(plan.semantic_stations_m,
                         marking.frame_origin.x + marking.length_m * 0.5, length.value);
    }
    SortUniqueStations(plan.semantic_stations_m);

    if (surface_end < surface_start - kStationEpsilon) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "road segment connection gates overlap");
    }
    for (const double station : plan.semantic_stations_m) {
      if (station >= surface_start - kStationEpsilon &&
          station <= surface_end + kStationEpsilon) {
        plan.surface_stations_m.push_back(station);
      }
    }
    const double surface_length = std::max(0.0, surface_end - surface_start);
    const int intervals =
        std::max(1, static_cast<int>(std::ceil(surface_length / kSurfaceSampleStepM)));
    for (int i = 0; i <= intervals; ++i) {
      plan.surface_stations_m.push_back(
          surface_start + surface_length * static_cast<double>(i) / intervals);
    }
    SortUniqueStations(plan.surface_stations_m);
    plan.marking_stations_m = plan.surface_stations_m;
    plan.mask_stations_m = plan.surface_stations_m;
    context.derived.sampling_plans.push_back(std::move(plan));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
