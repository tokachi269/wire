#pragma once

#include "city/road/road.hpp"

#include <algorithm>
#include <vector>

// Test-side views over DerivedRoad. Tests state behaviour in terms of
// connections, gates, sections and markings without depending on how the
// generation code lays those out internally.
namespace road_test_view {

using city::road::ConnectionGate;
using city::road::DerivedMarking;
using city::road::DerivedRoad;
using city::road::DerivedSegment;
using city::road::NodeConnectionKind;
using city::road::ResolvedApproach;
using city::road::ResolvedConnection;
using city::road::RoadNodeId;
using city::road::RoadSegmentId;
using city::road::SectionEvaluation;

[[nodiscard]] inline std::vector<const ResolvedConnection*> connections_of_kind(
    const DerivedRoad& derived, NodeConnectionKind kind) {
  std::vector<const ResolvedConnection*> out{};
  for (const ResolvedConnection& connection : derived.connections) {
    if (connection.kind == kind) out.push_back(&connection);
  }
  return out;
}

[[nodiscard]] inline std::vector<const ResolvedConnection*> junctions(const DerivedRoad& derived) {
  return connections_of_kind(derived, NodeConnectionKind::kJunction);
}

[[nodiscard]] inline std::vector<const ResolvedConnection*> corners(const DerivedRoad& derived) {
  return connections_of_kind(derived, NodeConnectionKind::kCorner);
}

[[nodiscard]] inline std::vector<ConnectionGate> gates(const DerivedRoad& derived) {
  std::vector<ConnectionGate> out{};
  for (const ResolvedConnection& connection : derived.connections) {
    for (const ResolvedApproach& approach : connection.approaches) {
      out.push_back(approach.gate);
    }
  }
  return out;
}

[[nodiscard]] inline std::vector<ConnectionGate> gates_of(const ResolvedConnection& connection) {
  std::vector<ConnectionGate> out{};
  for (const ResolvedApproach& approach : connection.approaches) {
    out.push_back(approach.gate);
  }
  return out;
}

[[nodiscard]] inline std::vector<const ResolvedApproach*> approaches(const DerivedRoad& derived) {
  std::vector<const ResolvedApproach*> out{};
  for (const ResolvedConnection& connection : derived.connections) {
    for (const ResolvedApproach& approach : connection.approaches) {
      out.push_back(&approach);
    }
  }
  return out;
}

[[nodiscard]] inline std::vector<const SectionEvaluation*> sections(const DerivedRoad& derived) {
  std::vector<const SectionEvaluation*> out{};
  for (const DerivedSegment& segment : derived.segments) {
    for (const SectionEvaluation& section : segment.sections) {
      out.push_back(&section);
    }
  }
  return out;
}

[[nodiscard]] inline std::vector<const DerivedMarking*> marking_lines(const DerivedRoad& derived) {
  std::vector<const DerivedMarking*> out{};
  for (const DerivedMarking& marking : derived.markings) {
    if (!marking.points.empty()) out.push_back(&marking);
  }
  return out;
}

[[nodiscard]] inline std::vector<const DerivedMarking*> marking_areas(const DerivedRoad& derived) {
  std::vector<const DerivedMarking*> out{};
  for (const DerivedMarking& marking : derived.markings) {
    if (!marking.polygon.empty()) out.push_back(&marking);
  }
  return out;
}

template <typename Predicate>
[[nodiscard]] std::size_t count_marking_lines(const DerivedRoad& derived, Predicate match) {
  std::size_t count = 0;
  for (const DerivedMarking* marking : marking_lines(derived)) {
    if (match(*marking)) ++count;
  }
  return count;
}

template <typename Predicate>
[[nodiscard]] std::size_t count_marking_areas(const DerivedRoad& derived, Predicate match) {
  std::size_t count = 0;
  for (const DerivedMarking* marking : marking_areas(derived)) {
    if (match(*marking)) ++count;
  }
  return count;
}

template <typename Predicate>
[[nodiscard]] const DerivedMarking* find_marking_line(const DerivedRoad& derived, Predicate match) {
  for (const DerivedMarking* marking : marking_lines(derived)) {
    if (match(*marking)) {
      return marking;
    }
  }
  return nullptr;
}

template <typename Predicate>
[[nodiscard]] const DerivedMarking* find_marking_area(const DerivedRoad& derived, Predicate match) {
  for (const DerivedMarking* marking : marking_areas(derived)) {
    if (match(*marking)) return marking;
  }
  return nullptr;
}

} // namespace road_test_view
