#pragma once

#include <string_view>

namespace city::road::persistence {

inline constexpr int kVersion = 13;
inline constexpr std::string_view kHeader = "road_graph_version=13\n";

// Version 12 stored a boundary as one width and one height, which only
// describes a ramp. The loader turns that into the two-point profile it always
// was and hands the width it consumed to the strip on its left, which leaves
// every position in the section exactly where version 12 put it.
inline constexpr int kPreviousVersion = 12;
inline constexpr std::string_view kPreviousHeader = "road_graph_version=12\n";

[[nodiscard]] inline bool HasReadableHeader(std::string_view text) {
  return text.starts_with(kHeader) || text.starts_with(kPreviousHeader);
}

[[nodiscard]] inline bool HasRoadHeader(std::string_view text) {
  return text.starts_with("road_graph_version=");
}

} // namespace city::road::persistence
