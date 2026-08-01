#pragma once

#include <string_view>

namespace city::road::persistence {

inline constexpr int kVersion = 11;
inline constexpr std::string_view kHeader = "road_graph_version=11\n";

[[nodiscard]] inline bool HasCurrentHeader(std::string_view text) {
  return text.starts_with(kHeader);
}

[[nodiscard]] inline bool HasRoadHeader(std::string_view text) {
  return text.starts_with("road_graph_version=");
}

} // namespace city::road::persistence
