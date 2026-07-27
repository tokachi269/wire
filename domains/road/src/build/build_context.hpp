#pragma once

#include "city/road/road.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace city::road::build {

enum class Stage : std::size_t {
  kTopologyIndex,
  kCanonicalAlignment,
  kNodeConnectionDecision,
  kAutoNodeLayout,
  kResolvedNodeLayout,
  kSamplingPlan,
  kSectionEvaluation,
  kConnectionGate,
  kJunctionGeometry,
  kMarkingAnchor,
  kMaterialization,
  kDerivedInvariant,
  kCount,
};

struct TopologyEndpoint {
  RoadSegmentId segment_id = 0;
  EndpointRole endpoint_role = EndpointRole::kStart;
};

struct NodeTopology {
  RoadNodeId node_id = 0;
  std::vector<TopologyEndpoint> endpoints{};
};

struct BuildContext {
  explicit BuildContext(const SavedRoadGraph& authoritative_graph) : authoritative(authoritative_graph) {}

  [[nodiscard]] Result<bool> Begin(Stage stage) {
    const std::size_t index = static_cast<std::size_t>(stage);
    if (index != next_stage || index >= stage_runs.size() || stage_runs[index] != 0) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road build stage order invariant failed");
    }
    ++stage_runs[index];
    ++next_stage;
    return Result<bool>::Ok(true);
  }

  const SavedRoadGraph& authoritative;
  std::vector<NodeTopology> topology{};
  DerivedRoad derived{};
  std::vector<std::string> diagnostics{};
  std::array<std::size_t, static_cast<std::size_t>(Stage::kCount)> stage_runs{};
  std::size_t next_stage = 0;
};

} // namespace city::road::build
