#pragma once

#include <string_view>

namespace city::road::persistence {

inline constexpr int kVersion = 15;
inline constexpr std::string_view kHeader = "road_graph_version=15\n";

// Version 14 had marking placement but no per-road corner radius.
inline constexpr int kPreviousVersion = 14;
inline constexpr std::string_view kPreviousHeader = "road_graph_version=14\n";

[[nodiscard]] inline bool HasReadableHeader(std::string_view text) {
  return text.starts_with(kHeader) || text.starts_with(kPreviousHeader);
}

[[nodiscard]] inline bool HasRoadHeader(std::string_view text) {
  return text.starts_with("road_graph_version=");
}

} // namespace city::road::persistence
