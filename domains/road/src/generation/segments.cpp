#include "generation.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/section.hpp"
#include "../lookup.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::generation {
namespace {

using internal::find_node;
using internal::find_transition;
using internal::normalize;
using internal::sort_unique_distances;
using internal::distance_epsilon;
using internal::subtract;
using internal::surface_sample_step_m;

double distance_value(DistanceRef ref, double total_m) {
  if (ref.kind == DistanceRefKind::kFromEnd)
    return total_m - ref.value;
  if (ref.kind == DistanceRefKind::kRatio)
    return total_m * ref.value;
  return ref.value;
}

void append_if_in_range(std::vector<double> &distances,
                        double segment_distance_m,
                        double total_m) {
  if (segment_distance_m >= -distance_epsilon &&
      segment_distance_m <= total_m + distance_epsilon) {
    distances.push_back(std::clamp(segment_distance_m, 0.0, total_m));
  }
}

// Distances that carry meaning: endpoints, span boundaries, transition limits
// and the distances manual markings need.
Result<bool> append_semantic_distances(const SavedRoadGraph &graph,
                                      const RoadSegment &segment,
                                      DerivedSegment &derived) {
  const double length = derived.length_m;
  derived.semantic_segment_distances_m = {0.0, length};

  double span_boundary = 0.0;
  for (std::size_t index = 0; index + 1 < derived.alignment.spans.size();
       ++index) {
    const Result<double> span_length =
        PathLength(MakePath({derived.alignment.spans[index]}));
    if (!span_length.ok) {
      return Result<bool>::Fail(span_length.error_kind, span_length.error);
    }
    span_boundary += span_length.value;
    derived.semantic_segment_distances_m.push_back(span_boundary);
  }

  if (segment.transition.has_value()) {
    const SectionTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "road segment transition is missing");
    }
    append_if_in_range(derived.semantic_segment_distances_m,
                       distance_value(transition->start, length), length);
    append_if_in_range(derived.semantic_segment_distances_m,
                       distance_value(transition->end, length), length);
  }

  for (const ManualLineMarking &marking : graph.manual_lines) {
    if (marking.owner_segment_id != segment.id)
      continue;
    const std::vector<Vec2d> points = FlattenPath(marking.path);
    constexpr double half_width = 0.05;
    for (std::size_t index = 0; index < points.size(); ++index) {
      const Vec2d point = points[index];
      append_if_in_range(derived.semantic_segment_distances_m, point.x, length);
      Vec2d direction = index + 1 < points.size()
                            ? subtract(points[index + 1], points[index])
                            : subtract(points[index], points[index - 1]);
      direction = normalize(direction);
      const Vec2d normal{-direction.y, direction.x};
      append_if_in_range(derived.semantic_segment_distances_m,
                         point.x - normal.x * half_width, length);
      append_if_in_range(derived.semantic_segment_distances_m,
                         point.x + normal.x * half_width, length);
    }
  }
  for (const ManualAreaMarking &marking : graph.manual_areas) {
    if (marking.owner_segment_id != segment.id)
      continue;
    append_if_in_range(derived.semantic_segment_distances_m,
                       marking.frame_origin.x - marking.length_m * 0.5, length);
    append_if_in_range(derived.semantic_segment_distances_m,
                       marking.frame_origin.x + marking.length_m * 0.5, length);
  }
  return Result<bool>::Ok(true);
}

} // namespace

std::vector<NodeIncidence> derive_node_incidence(const SavedRoadGraph &graph) {
  std::vector<NodeIncidence> incidence{};
  incidence.reserve(graph.nodes.size());
  for (const RoadNode &node : graph.nodes) {
    NodeIncidence item{};
    item.node_id = node.id;
    for (const RoadSegment &segment : graph.segments) {
      if (segment.node_a == node.id || segment.node_b == node.id) {
        item.endpoints.push_back(NodeEndpoint{
            segment.id,
            segment.node_a == node.id ? EndpointRole::kStart
                                      : EndpointRole::kEnd,
        });
      }
    }
    incidence.push_back(std::move(item));
  }
  return incidence;
}

Result<std::vector<DerivedSegment>>
derive_segment_shapes(const SavedRoadGraph &graph) {
  std::vector<DerivedSegment> segments{};
  segments.reserve(graph.segments.size());
  for (const RoadSegment &segment : graph.segments) {
    const RoadNode *node_a = find_node(graph, segment.node_a);
    const RoadNode *node_b = find_node(graph, segment.node_b);
    if (node_a == nullptr || node_b == nullptr) {
      return Result<std::vector<DerivedSegment>>::Fail(
          ErrorKind::kValidation, "road segment endpoint node is missing");
    }
    Result<Path> alignment = DeriveCanonicalAlignment(
        node_a->position, node_b->position, segment.shape);
    if (!alignment.ok) {
      return Result<std::vector<DerivedSegment>>::Fail(alignment.error_kind,
                                                       alignment.error);
    }
    const Result<double> length = PathLength(alignment.value);
    if (!length.ok) {
      return Result<std::vector<DerivedSegment>>::Fail(length.error_kind,
                                                       length.error);
    }
    DerivedSegment derived{};
    derived.id = segment.id;
    derived.alignment = std::move(alignment.value);
    derived.length_m = length.value;
    derived.surface_start_m = 0.0;
    derived.surface_end_m = length.value;
    segments.push_back(std::move(derived));
  }
  return Result<std::vector<DerivedSegment>>::Ok(std::move(segments));
}

Result<bool> derive_segment_sections(const SavedRoadGraph &graph,
                                     std::vector<DerivedSegment> &segments,
                                     const std::vector<ResolvedConnection> &connections) {
  for (DerivedSegment &derived : segments) {
    const RoadSegment *segment = internal::find_segment(graph, derived.id);
    if (segment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road segment source is missing");
    }
    const Result<bool> semantic =
        append_semantic_distances(graph, *segment, derived);
    if (!semantic.ok)
      return semantic;

    double surface_start = 0.0;
    double surface_end = derived.length_m;
    for (const ResolvedConnection &connection : connections) {
      for (const ResolvedApproach &approach : connection.approaches) {
        if (approach.key.segment_id != derived.id)
          continue;
        if (approach.key.endpoint_role == EndpointRole::kStart) {
          surface_start = std::max(surface_start, approach.gate_segment_distance_m);
        } else {
          surface_end = std::min(surface_end, approach.gate_segment_distance_m);
        }
      }
    }
    if (surface_end < surface_start - distance_epsilon) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "road segment connection gates overlap");
    }
    derived.surface_start_m = surface_start;
    derived.surface_end_m = surface_end;
    derived.semantic_segment_distances_m.push_back(surface_start);
    derived.semantic_segment_distances_m.push_back(surface_end);
    sort_unique_distances(derived.semantic_segment_distances_m);

    derived.surface_segment_distances_m.clear();
    for (const double distance : derived.semantic_segment_distances_m) {
      if (distance >= surface_start - distance_epsilon &&
          distance <= surface_end + distance_epsilon) {
        derived.surface_segment_distances_m.push_back(distance);
      }
    }
    const double surface_length = std::max(0.0, surface_end - surface_start);
    const int intervals = std::max(
        1, static_cast<int>(std::ceil(surface_length / surface_sample_step_m)));
    for (int i = 0; i <= intervals; ++i) {
      derived.surface_segment_distances_m.push_back(
          surface_start + surface_length * static_cast<double>(i) / intervals);
    }
    sort_unique_distances(derived.surface_segment_distances_m);

    std::vector<double> distances = derived.semantic_segment_distances_m;
    distances.insert(distances.end(), derived.surface_segment_distances_m.begin(),
                    derived.surface_segment_distances_m.end());
    sort_unique_distances(distances);
    derived.sections.clear();
    derived.sections.reserve(distances.size());
    for (const double distance : distances) {
      Result<SectionEvaluation> evaluated =
          internal::section_at(graph, *segment, distance, derived.length_m);
      if (!evaluated.ok) {
        return Result<bool>::Fail(evaluated.error_kind, evaluated.error);
      }
      derived.sections.push_back(std::move(evaluated.value));
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::generation
