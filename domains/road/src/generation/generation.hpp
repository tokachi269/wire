#pragma once

#include "city/road/road.hpp"

namespace city::road::generation {

// Which segment endpoints meet at a node. Derived once per generation and read by
// connection resolution; it is not part of the derived output.
struct NodeEndpoint {
  RoadSegmentId segment_id = 0;
  EndpointRole role = EndpointRole::kStart;
};
struct NodeIncidence {
  RoadNodeId node_id = 0;
  std::vector<NodeEndpoint> endpoints{};
};

[[nodiscard]] std::vector<NodeIncidence> derive_node_incidence(const SavedRoadGraph &graph);

// Canonical alignment, the distances every consumer shares, and the section at
// those distances. Connection resolution needs the alignments first, so segments
// are derived in two passes: shapes, then the sections that depend on gates.
[[nodiscard]] Result<std::vector<DerivedSegment>>
derive_segment_shapes(const SavedRoadGraph &graph);

[[nodiscard]] Result<bool> derive_segment_sections(const SavedRoadGraph &graph,
                                                   std::vector<DerivedSegment> &segments,
                                                   const std::vector<ResolvedConnection> &connections);

[[nodiscard]] Result<std::vector<ResolvedConnection>>
resolve_connections(const SavedRoadGraph &graph,
                    const std::vector<NodeIncidence> &incidence,
                    const std::vector<DerivedSegment> &segments);

[[nodiscard]] Result<bool> resolve_connection_geometry(std::vector<ResolvedConnection> &connections,
                                                       const std::vector<DerivedSegment> &segments);

[[nodiscard]] Result<std::vector<DerivedMarking>>
derive_markings(const SavedRoadGraph &graph, const std::vector<DerivedSegment> &segments,
                const std::vector<ResolvedConnection> &connections);

[[nodiscard]] Result<bool> emit_geometry(DerivedRoad &derived);

// Generates every derived value from the authoritative graph. Pure: it does
// not touch RoadState and publishes nothing on failure.
[[nodiscard]] Result<DerivedRoad> generate_road(const SavedRoadGraph &graph);

} // namespace city::road::generation
