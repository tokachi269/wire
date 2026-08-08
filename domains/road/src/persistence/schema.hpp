#pragma once

#include <string_view>

namespace city::road::persistence {

inline constexpr int kVersion = 14;
inline constexpr std::string_view kHeader = "road_graph_version=14\n";

// Version 13 had no placement on a line and centred every one of them on its
// boundary, so that is the placement they come back with.
inline constexpr int kPreviousVersion = 13;
inline constexpr std::string_view kPreviousHeader = "road_graph_version=13\n";

[[nodiscard]] inline bool HasReadableHeader(std::string_view text) {
  return text.starts_with(kHeader) || text.starts_with(kPreviousHeader);
}

[[nodiscard]] inline bool HasRoadHeader(std::string_view text) {
  return text.starts_with("road_graph_version=");
}

} // namespace city::road::persistence
