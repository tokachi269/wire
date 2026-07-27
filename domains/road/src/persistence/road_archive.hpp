#pragma once

#include "city/road/authoritative_types/road_graph.hpp"

#include <cstdint>
#include <string>

namespace city::road::persistence {

struct LoadedRoad {
  SavedRoadGraph graph{};
  std::uint64_t next_id = 1;
};

[[nodiscard]] Result<std::string> SaveRoad(const SavedRoadGraph& graph,
                                           std::uint64_t next_id);
[[nodiscard]] Result<LoadedRoad> LoadRoad(const std::string& text);
[[nodiscard]] Result<bool> ValidateAuthoritativeGraph(
    const SavedRoadGraph& graph, std::uint64_t next_id);

} // namespace city::road::persistence
