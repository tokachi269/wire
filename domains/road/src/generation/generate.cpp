#include "generation.hpp"

#include <algorithm>
#include <cmath>

#include "../geometry/geometry.hpp"

namespace city::road::generation {

// The only entry point that turns the authoritative graph into derived state.
// Nothing is published until every step succeeded.
Result<DerivedRoad> generate_road(const SavedRoadGraph &graph) {
  const std::vector<NodeIncidence> incidence = derive_node_incidence(graph);

  Result<std::vector<DerivedSegment>> segments = derive_segment_shapes(graph);
  if (!segments.ok) {
    return Result<DerivedRoad>::Fail(segments.error_kind, segments.error);
  }

  DerivedRoad derived{};
  Result<std::vector<ResolvedConnection>> connections = resolve_connections(graph, incidence, segments.value);
  if (!connections.ok) {
    return Result<DerivedRoad>::Fail(connections.error_kind, connections.error);
  }

  const Result<bool> sections =
      derive_segment_sections(graph, segments.value, connections.value);
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
  return Result<DerivedRoad>::Ok(std::move(derived));
}

} // namespace city::road::generation
