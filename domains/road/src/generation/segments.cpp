#include "generation.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/section.hpp"
#include "../lookup.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::generation {
namespace {

using internal::find_node;
using internal::find_template;
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
      return Result<bool>::Fail(span_length.failure_category, span_length.error);
    }
    span_boundary += span_length.value;
    derived.semantic_segment_distances_m.push_back(span_boundary);
  }

  if (segment.transition.has_value()) {
    const SectionTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
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

// A strip that appears or disappears across a transition must say how. The
// rules travel with the saved transition, so generation is where they are read:
// a lane tapers, a median gets an end cap, and nothing changes silently.
Result<bool> validate_transition_strip_actions(const SavedRoadGraph &graph) {
  const auto has_strip = [](const CrossSectionTemplate &section,
                            SectionStripId id) {
    return std::any_of(section.strips.begin(), section.strips.end(),
                       [id](const SectionStrip &strip) {
                         return strip.id == id;
                       });
  };
  for (const RoadSegment &segment : graph.segments) {
    if (!segment.transition.has_value())
      continue;
    const SectionTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road segment transition is missing");
    }
    const CrossSectionTemplate *from =
        find_template(graph, transition->from_template);
    const CrossSectionTemplate *to =
        find_template(graph, transition->to_template);
    if (from == nullptr || to == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road transition template is missing");
    }
    if (transition->rules.empty()) {
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
                                "section transition must define element actions");
    }
    const auto action_for =
        [transition](SectionStripId id) -> std::optional<TransitionAction> {
      const auto rule = std::find_if(
          transition->rules.begin(), transition->rules.end(),
          [id](const SectionTransitionRule &item) { return item.strip_id == id; });
      return rule == transition->rules.end()
                 ? std::nullopt
                 : std::optional<TransitionAction>{rule->action};
    };
    for (const SectionTransitionRule &rule : transition->rules) {
      if (rule.action == TransitionAction::kUnsupported) {
        return Result<bool>::Fail(
            CommitFailureCategory::kNotImplemented,
            "section transition contains unsupported element action");
      }
    }
    for (const SectionStrip &strip : to->strips) {
      if (!has_strip(*from, strip.id) &&
          action_for(strip.id) != TransitionAction::kTaperIn) {
        return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
                                  "appearing section strip requires TaperIn");
      }
    }
    for (const SectionStrip &strip : from->strips) {
      if (has_strip(*to, strip.id))
        continue;
      const TransitionAction required =
          strip.function == StripFunction::kMedian ? TransitionAction::kEndCap
                                                   : TransitionAction::kTaperOut;
      if (action_for(strip.id) != required) {
        return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
                                  strip.function == StripFunction::kMedian
                                      ? "disappearing median requires EndCap"
                                      : "disappearing section strip requires TaperOut");
      }
    }
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
          CommitFailureCategory::kInvalidInput, "road segment endpoint node is missing");
    }
    Result<Path> alignment = DeriveCanonicalAlignment(
        node_a->position, node_b->position, segment.shape);
    if (!alignment.ok) {
      return Result<std::vector<DerivedSegment>>::Fail(alignment.failure_category,
                                                       alignment.error);
    }
    const Result<double> length = PathLength(alignment.value);
    if (!length.ok) {
      return Result<std::vector<DerivedSegment>>::Fail(length.failure_category,
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
  const Result<bool> transition_actions = validate_transition_strip_actions(graph);
  if (!transition_actions.ok)
    return transition_actions;

  for (DerivedSegment &derived : segments) {
    const RoadSegment *segment = internal::find_segment(graph, derived.id);
    if (segment == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
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
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
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
        return Result<bool>::Fail(evaluated.failure_category, evaluated.error);
      }
      derived.sections.push_back(std::move(evaluated.value));
    }
  }

  // A boundary-anchored section transition changes the section's lateral
  // origin for the remainder of its corridor. Carry that resolved origin
  // across semantic segment splits so a completed lane addition does not
  // recenter lanes that existed before the transition.
  for (const RoadCorridor &corridor : graph.corridors) {
    const SectionEvaluation *previous_exit = nullptr;
    std::optional<TransitionAnchor> carried_anchor{};
    BoundaryId carried_boundary_id = 0;
    for (const DirectedSegmentRef &ref : corridor.segments) {
      auto derived_it = std::find_if(
          segments.begin(), segments.end(), [&ref](const DerivedSegment &item) {
            return item.id == ref.segment_id;
          });
      const RoadSegment *source = internal::find_segment(graph, ref.segment_id);
      if (derived_it == segments.end() || source == nullptr ||
          derived_it->sections.empty()) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInternalError,
            "road corridor section continuity source is missing");
      }
      if (source->transition.has_value()) {
        const SectionTransition *transition =
            find_transition(graph, *source->transition);
        if (transition == nullptr) {
          return Result<bool>::Fail(
              CommitFailureCategory::kInternalError,
              "road corridor section transition is missing");
        }
        if (transition->anchor == TransitionAnchor::kBoundary ||
            transition->anchor == TransitionAnchor::kLeftEdge ||
            transition->anchor == TransitionAnchor::kRightEdge) {
          carried_anchor = transition->anchor;
          carried_boundary_id = transition->anchor_boundary_id;
        }
      }
      SectionEvaluation &entry =
          ref.reversed ? derived_it->sections.back()
                       : derived_it->sections.front();
      if (previous_exit != nullptr && carried_anchor.has_value()) {
        const auto anchor_lateral =
            [carried_anchor, carried_boundary_id](
                const SectionEvaluation &section) -> std::optional<double> {
          if (*carried_anchor == TransitionAnchor::kLeftEdge)
            return section.boundaries.front().lateral_m;
          if (*carried_anchor == TransitionAnchor::kRightEdge)
            return section.boundaries.back().lateral_m;
          const auto boundary = std::find_if(
              section.boundaries.begin(), section.boundaries.end(),
              [carried_boundary_id](const SectionBoundarySample &item) {
                return item.boundary_id == carried_boundary_id;
              });
          return boundary == section.boundaries.end()
                     ? std::nullopt
                     : std::optional<double>{boundary->lateral_m};
        };
        const std::optional<double> previous_lateral =
            anchor_lateral(*previous_exit);
        const std::optional<double> current_lateral = anchor_lateral(entry);
        if (!previous_lateral.has_value() || !current_lateral.has_value()) {
          return Result<bool>::Fail(
              CommitFailureCategory::kInternalError,
              "road corridor carried section anchor is missing");
        }
        const double shift = *previous_lateral - *current_lateral;
        for (SectionEvaluation &section : derived_it->sections) {
          for (SectionBoundarySample &boundary : section.boundaries) {
            boundary.lateral_m += shift;
          }
        }
      }
      previous_exit = ref.reversed ? &derived_it->sections.front()
                                   : &derived_it->sections.back();
    }
  }
  return Result<bool>::Ok(true);
}

Result<std::vector<DerivedSegmentLanePath>>
derive_segment_lane_paths(const SavedRoadGraph &graph,
                          const std::vector<DerivedSegment> &segments) {
  using Out = Result<std::vector<DerivedSegmentLanePath>>;
  std::vector<DerivedSegmentLanePath> paths{};
  for (const DerivedSegment &segment : segments) {
    const RoadSegment *source = internal::find_segment(graph, segment.id);
    if (source == nullptr) {
      return Out::Fail(CommitFailureCategory::kInternalError,
                       "lane inspection source segment is missing");
    }
    for (const SectionEvaluation &evaluation : segment.sections) {
      Result<CrossSectionTemplate> section = internal::template_at(
          graph, *source, evaluation.segment_distance_m, segment.length_m);
      if (!section.ok) {
        return Out::Fail(section.failure_category, section.error);
      }
      const Result<Vec2d> center =
          EvaluatePath(segment.alignment, evaluation.segment_distance_m);
      const Result<Vec2d> tangent =
          internal::tangent_at(segment.alignment,
                               evaluation.segment_distance_m);
      if (!center.ok || !tangent.ok) {
        return Out::Fail(CommitFailureCategory::kInternalError,
                         "lane inspection alignment sample is missing");
      }
      const Vec2d lateral{-tangent.value.y, tangent.value.x};
      for (const LaneBand &lane : section.value.lane_bands) {
        const Result<internal::LaneSectionPosition> position =
            internal::lane_position(section.value, lane, evaluation);
        if (!position.ok) {
          return Out::Fail(position.failure_category, position.error);
        }
        auto found = std::find_if(
            paths.begin(), paths.end(), [&segment, &lane](const auto &path) {
              return path.segment_id == segment.id && path.lane_id == lane.id;
            });
        if (found == paths.end()) {
          paths.push_back(DerivedSegmentLanePath{
              segment.id, lane.id, lane.direction,
              section.value.id, section.value.id,
              evaluation.segment_distance_m, evaluation.segment_distance_m,
              {}});
          found = paths.end() - 1;
        }
        found->end_segment_distance_m = evaluation.segment_distance_m;
        found->end_template_id = section.value.id;
        found->points.push_back(Vec3d{
            center.value.x + lateral.x * position.value.lateral_m,
            center.value.y + lateral.y * position.value.lateral_m,
            position.value.height_m});
      }
    }
  }
  return Out::Ok(std::move(paths));
}

} // namespace city::road::generation
