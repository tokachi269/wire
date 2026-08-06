#pragma once

#include <string_view>

namespace city::road::persistence {

inline constexpr int kVersion = 12;
inline constexpr std::string_view kHeader = "road_graph_version=12\n";

// Version 11 predates the layout's explicit alignment offset and is still
// readable: the loader resolves that offset once from the widths v11 saved.
inline constexpr int kPreviousVersion = 11;
inline constexpr std::string_view kPreviousHeader = "road_graph_version=11\n";

[[nodiscard]] inline bool HasReadableHeader(std::string_view text) {
  return text.starts_with(kHeader) || text.starts_with(kPreviousHeader);
}

[[nodiscard]] inline bool HasRoadHeader(std::string_view text) {
  return text.starts_with("road_graph_version=");
}

} // namespace city::road::persistence
