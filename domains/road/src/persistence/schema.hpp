#pragma once

#include <string_view>

namespace city::road::persistence {

inline constexpr int kVersion = 16;
inline constexpr std::string_view kHeader = "road_graph_version=16\n";

// Version 15 had per-road corner radius but no node elevation.
inline constexpr int kPreviousVersion = 15;
inline constexpr std::string_view kPreviousHeader = "road_graph_version=15\n";

[[nodiscard]] inline bool HasReadableHeader(std::string_view text) {
  return text.starts_with(kHeader) || text.starts_with(kPreviousHeader) ||
         text.starts_with("road_graph_version=14\n");
}

[[nodiscard]] inline bool HasRoadHeader(std::string_view text) {
  return text.starts_with("road_graph_version=");
}

} // namespace city::road::persistence
