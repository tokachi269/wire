#include "generation.hpp"

#include <algorithm>
#include <cmath>

#include "../geometry/geometry.hpp"

namespace city::road {

const DerivedSegment *FindDerivedSegment(const DerivedRoad &derived,
                                         RoadSegmentId segment_id) {
  const auto found = std::find_if(derived.segments.begin(),
                                  derived.segments.end(),
                                  [segment_id](const DerivedSegment &segment) {
                                    return segment.id == segment_id;
                                  });
  return found == derived.segments.end() ? nullptr : &*found;
}

const ResolvedConnection *FindResolvedConnection(const DerivedRoad &derived,
                                                 RoadNodeId node_id) {
  const auto found =
      std::find_if(derived.connections.begin(), derived.connections.end(),
                   [node_id](const ResolvedConnection &connection) {
                     return connection.node_id == node_id;
                   });
  return found == derived.connections.end() ? nullptr : &*found;
}

const ResolvedApproach *FindResolvedApproach(const DerivedRoad &derived,
                                             const ApproachKey &key) {
  for (const ResolvedConnection &connection : derived.connections) {
    for (const ResolvedApproach &approach : connection.approaches) {
      if (approach.key == key)
        return &approach;
    }
  }
  return nullptr;
}

const SectionEvaluation *FindSectionAt(const DerivedSegment &segment,
                                       double station_m) {
  const SectionEvaluation *match = nullptr;
  for (const SectionEvaluation &section : segment.sections) {
    if (std::abs(section.station_m - station_m) > internal::station_epsilon)
      continue;
    if (match != nullptr)
      return nullptr;
    match = &section;
  }
  return match;
}

const Path *FindCanonicalAlignment(const DerivedRoad &derived,
                                   RoadSegmentId segment_id) {
  const DerivedSegment *segment = FindDerivedSegment(derived, segment_id);
  return segment == nullptr ? nullptr : &segment->alignment;
}

} // namespace city::road

namespace city::road::generation {

// The only entry point that turns the authoritative graph into derived state.
// Nothing is published until every step succeeded.
Result<DerivedRoad> regenerate_road(const SavedRoadGraph &graph) {
  const std::vector<NodeIncidence> incidence = derive_node_incidence(graph);

  Result<std::vector<DerivedSegment>> segments = derive_segment_shapes(graph);
  if (!segments.ok) {
    return Result<DerivedRoad>::Fail(segments.error_kind, segments.error);
  }

  DerivedRoad derived{};
  Result<std::vector<ResolvedConnection>> connections = resolve_connections(
      graph, incidence, segments.value, derived.setback_calculation_count);
  if (!connections.ok) {
    return Result<DerivedRoad>::Fail(connections.error_kind, connections.error);
  }

  const Result<bool> sections =
      derive_segment_sections(graph, segments.value, connections.value,
                              derived.section_evaluation_count);
  if (!sections.ok) {
    return Result<DerivedRoad>::Fail(sections.error_kind, sections.error);
  }

  const Result<bool> geometry =
      resolve_connection_geometry(connections.value, segments.value);
  if (!geometry.ok) {
    return Result<DerivedRoad>::Fail(geometry.error_kind, geometry.error);
  }

  Result<std::vector<DerivedMarking>> markings =
      derive_markings(graph, segments.value, connections.value);
  if (!markings.ok) {
    return Result<DerivedRoad>::Fail(markings.error_kind, markings.error);
  }

  derived.segments = std::move(segments.value);
  derived.connections = std::move(connections.value);
  derived.markings = std::move(markings.value);

  const Result<bool> emitted = emit_geometry(derived);
  if (!emitted.ok) {
    return Result<DerivedRoad>::Fail(emitted.error_kind, emitted.error);
  }

  const Result<bool> valid = ValidateGraphInvariants(graph, derived);
  if (!valid.ok) {
    return Result<DerivedRoad>::Fail(valid.error_kind, valid.error);
  }
  return Result<DerivedRoad>::Ok(std::move(derived));
}

} // namespace city::road::generation
