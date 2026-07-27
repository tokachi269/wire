#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::build {
namespace {

double station_value(StationRef ref, double total_m) {
  if (ref.kind == StationRefKind::kFromEnd)
    return total_m - ref.value;
  if (ref.kind == StationRefKind::kRatio)
    return total_m * ref.value;
  return ref.value;
}

void append_if_in_range(std::vector<double> &stations, double station_m,
                        double total_m) {
  if (station_m >= -station_epsilon && station_m <= total_m + station_epsilon) {
    stations.push_back(std::clamp(station_m, 0.0, total_m));
  }
}

} // namespace

Result<bool> make_sampling(pipeline &pipe) {
  pipe.out.sampling.clear();
  pipe.out.sampling.reserve(pipe.source.segments.size());

  for (const RoadSegment &segment : pipe.source.segments) {
    const Path *alignment = find_alignment(pipe.out, segment.id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road segment canonical alignment is missing");
    }
    const Result<double> length = PathLength(*alignment);
    if (!length.ok)
      return Result<bool>::Fail(length.error_kind, length.error);

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
      const SectionTransition *transition =
          find_transition(pipe.source, *segment.transition);
      if (transition == nullptr) {
        return Result<bool>::Fail(ErrorKind::kValidation,
                                  "road segment transition is missing");
      }
      append_if_in_range(plan.semantic_stations_m,
                         station_value(transition->start, length.value),
                         length.value);
      append_if_in_range(plan.semantic_stations_m,
                         station_value(transition->end, length.value),
                         length.value);
    }

    const ApproachKey start_key =
        ApproachKey{segment.node_a, segment.id, EndpointRole::kStart};
    const ApproachKey end_key =
        ApproachKey{segment.node_b, segment.id, EndpointRole::kEnd};
    const ResolvedNodeLayout *start_layout =
        find_layout(pipe.out, segment.node_a);
    const ResolvedNodeLayout *end_layout =
        find_layout(pipe.out, segment.node_b);
    const ResolvedApproachLayout *start =
        start_layout == nullptr
            ? nullptr
            : find_approach_layout(*start_layout, start_key);
    const ResolvedApproachLayout *end =
        end_layout == nullptr ? nullptr
                              : find_approach_layout(*end_layout, end_key);
    if ((start_layout != nullptr && start == nullptr) ||
        (end_layout != nullptr && end == nullptr)) {
      return Result<bool>::Fail(
          ErrorKind::kInternal,
          "road segment resolved approach layout is missing");
    }
    const double surface_start = start == nullptr ? 0.0 : start->gate_station_m;
    const double surface_end =
        end == nullptr ? length.value : end->gate_station_m;
    plan.semantic_stations_m.push_back(surface_start);
    plan.semantic_stations_m.push_back(surface_end);

    for (const ManualLineMarking &marking : pipe.source.manual_lines) {
      if (marking.owner_segment_id != segment.id)
        continue;
      const std::vector<Vec2d> points = FlattenPath(marking.path);
      constexpr double half_width = 0.05;
      for (std::size_t index = 0; index < points.size(); ++index) {
        const Vec2d point = points[index];
        append_if_in_range(plan.semantic_stations_m, point.x, length.value);
        Vec2d direction = index + 1 < points.size()
                              ? subtract(points[index + 1], points[index])
                              : subtract(points[index], points[index - 1]);
        direction = normalize(direction);
        const Vec2d normal{-direction.y, direction.x};
        append_if_in_range(plan.semantic_stations_m,
                           point.x - normal.x * half_width, length.value);
        append_if_in_range(plan.semantic_stations_m,
                           point.x + normal.x * half_width, length.value);
      }
    }
    for (const ManualAreaMarking &marking : pipe.source.manual_areas) {
      if (marking.owner_segment_id != segment.id)
        continue;
      append_if_in_range(plan.semantic_stations_m,
                         marking.frame_origin.x - marking.length_m * 0.5,
                         length.value);
      append_if_in_range(plan.semantic_stations_m,
                         marking.frame_origin.x + marking.length_m * 0.5,
                         length.value);
    }
    sort_unique_stations(plan.semantic_stations_m);

    if (surface_end < surface_start - station_epsilon) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "road segment connection gates overlap");
    }
    for (const double station : plan.semantic_stations_m) {
      if (station >= surface_start - station_epsilon &&
          station <= surface_end + station_epsilon) {
        plan.surface_stations_m.push_back(station);
      }
    }
    const double surface_length = std::max(0.0, surface_end - surface_start);
    const int intervals = std::max(
        1, static_cast<int>(std::ceil(surface_length / surface_sample_step_m)));
    for (int i = 0; i <= intervals; ++i) {
      plan.surface_stations_m.push_back(
          surface_start + surface_length * static_cast<double>(i) / intervals);
    }
    sort_unique_stations(plan.surface_stations_m);
    plan.marking_stations_m = plan.surface_stations_m;
    plan.mask_stations_m = plan.surface_stations_m;
    pipe.out.sampling.push_back(std::move(plan));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
