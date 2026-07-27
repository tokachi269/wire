#pragma once

#include "city/road/road.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace city::road::build {

enum class stage : std::size_t {
  topology,
  alignments,
  connections,
  auto_layout,
  layout,
  sampling,
  sections,
  gates,
  junctions,
  marking_anchors,
  marking_intents,
  marking_continuity,
  markings,
  draw,
  invariant,
  count,
};

struct endpoint {
  RoadSegmentId segment_id = 0;
  EndpointRole role = EndpointRole::kStart;
};

struct topology {
  RoadNodeId node_id = 0;
  std::vector<endpoint> endpoints{};
};

struct pipeline {
  explicit pipeline(const SavedRoadGraph &graph) : source(graph) {}

  [[nodiscard]] Result<bool> begin(stage step) {
    const std::size_t index = static_cast<std::size_t>(step);
    if (index != next || index >= runs.size() || runs[index] != 0) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road build stage order invariant failed");
    }
    ++runs[index];
    ++next;
    return Result<bool>::Ok(true);
  }

  const SavedRoadGraph &source;
  std::vector<topology> nodes{};
  DerivedRoad out{};
  std::array<std::size_t, static_cast<std::size_t>(stage::count)> runs{};
  std::size_t next = 0;
};

[[nodiscard]] Result<bool> make_topology(pipeline &pipe);
[[nodiscard]] Result<bool> make_alignments(pipeline &pipe);
[[nodiscard]] Result<bool> make_connections(pipeline &pipe);
[[nodiscard]] Result<bool> make_auto_layouts(pipeline &pipe);
[[nodiscard]] Result<bool> resolve_layouts(pipeline &pipe);
[[nodiscard]] Result<bool> make_sampling(pipeline &pipe);
[[nodiscard]] Result<bool> make_sections(pipeline &pipe);
[[nodiscard]] Result<bool> make_gates(pipeline &pipe);
[[nodiscard]] Result<bool> make_junctions(pipeline &pipe);
[[nodiscard]] Result<bool> make_marking_anchors(pipeline &pipe);
[[nodiscard]] Result<bool> make_marking_intents(pipeline &pipe);
[[nodiscard]] Result<bool> make_marking_continuity(pipeline &pipe);
[[nodiscard]] Result<bool> resolve_markings(pipeline &pipe);
[[nodiscard]] Result<DerivedRoad> make(const SavedRoadGraph &source);

[[nodiscard]] Result<SectionEvaluation> section_at(const SavedRoadGraph &graph,
                                                   const RoadSegment &segment,
                                                   double station_m,
                                                   double total_m);
[[nodiscard]] Result<CrossSectionTemplate>
template_at(const SavedRoadGraph &graph, const RoadSegment &segment,
            double station_m, double total_m);

} // namespace city::road::build