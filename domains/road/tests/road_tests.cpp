#include "city/road/road.hpp"

#include "derived_view.hpp"
#include "fixtures/layouts.hpp"
#include "../src/generation/generation.hpp"
#include "../src/generation/emit.hpp"
#include "../src/geometry/geometry.hpp"
#include "../src/geometry/section.hpp"
#include "../src/persistence/archive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <numbers>
#include <optional>
#include <sstream>
#include <set>
#include <string>

namespace {

#define ROAD_TEST_EXPECT(condition, message) \
  do {                                       \
    if (!(condition)) {                      \
      failure = (message);                   \
      return false;                          \
    }                                        \
  } while (false)

using city::road::CommitFailureCategory;
using city::road::BoundaryRole;
using city::road::BoundaryId;
using city::road::AutoMarkingKey;
using city::road::AutoMarkingPolicy;
using city::road::EndpointRole;
using city::road::JunctionMarkingAction;
using city::road::JunctionMarkingEndpoint;
using city::road::JunctionMarkingOverride;
using city::road::LaneId;
using city::road::MarkingPlacement;
using city::road::MarkingRole;
using city::road::MarkingOwner;
using city::road::MarkingTrackKey;
using city::road::MakeBezier;
using city::road::MakeLine;
using city::road::MakePath;
using city::road::Mesh;
using city::road::MeshUvMapping;
using city::road::EvaluatePath;
using city::road::Path;
using city::road::PathLength;
using city::road::RenderStyleFromMarking;
using city::road::RenderStyleFromSurface;
using city::road::RenderStyleRef;
using city::road::RoadState;
using city::road::RoadCorridorId;
using city::road::RoadCorridor;
using city::road::RoadNodeId;
using city::road::RoadLayoutTemplateId;
using city::road::RoadSegment;
using city::road::RoadSegmentId;
using city::road::RoadLayoutTransitionRule;
using city::road::DistanceRef;
using city::road::DistanceRefKind;
using city::road::DerivedMarking;
using city::road::StripFunction;
using city::road::TransitionAnchor;
using city::road::TransitionAction;
using city::road::ValidateGraphInvariants;
using city::road::Vec2d;
using city::road::Vec3d;
namespace builtin_marking_styles = city::road::builtin_marking_styles;
namespace builtin_surface_styles = city::road::builtin_surface_styles;

// Section transitions, saved lane topology and manual markings are authoritative
// data that no public operation authors. Tests that need such a graph build it
// directly and enter through the same generate and load entries RoadState uses.
constexpr std::uint64_t kFixtureNextId = 100000;

using city::road::SavedRoadGraph;
using city::road::RoadLayoutTransition;

void attach_transition(SavedRoadGraph& graph, RoadSegmentId segment_id,
                       RoadLayoutTransition transition) {
  for (RoadSegment& segment : graph.segments) {
    if (segment.id == segment_id) segment.transition = transition.id;
  }
  graph.transitions.push_back(std::move(transition));
}

[[nodiscard]] city::road::Result<RoadState> load_fixture(const SavedRoadGraph& graph) {
  const auto text = city::road::persistence::SaveRoad(graph, kFixtureNextId);
  if (!text.ok) {
    return city::road::Result<RoadState>::Fail(text.failure_category, text.error);
  }
  return RoadState::Load(text.value);
}

bool SetAddLaneRange(RoadState& state, city::road::AddLaneRequest& request,
                     double start_distance_m, double complete_distance_m) {
  const auto start = city::road::ResolveCorridorDistance(
      state.graph(), state.derived(),
      city::road::CorridorDistanceRef{request.corridor_id, start_distance_m});
  const auto complete = city::road::ResolveCorridorDistance(
      state.graph(), state.derived(), city::road::CorridorDistanceRef{
                                          request.corridor_id,
                                          complete_distance_m});
  const auto* start_segment = start.ok
                                  ? city::road::FindDerivedSegment(
                                        state.derived(), start.value.segment_id)
                                  : nullptr;
  const auto* complete_segment = complete.ok
                                     ? city::road::FindDerivedSegment(
                                           state.derived(),
                                           complete.value.segment_id)
                                     : nullptr;
  const auto* corridor =
      city::road::FindRoadCorridor(state.graph(), request.corridor_id);
  if (!start.ok || !complete.ok || start_segment == nullptr ||
      complete_segment == nullptr || corridor == nullptr ||
      corridor->segments.empty()) return false;
  request.transition_start = {
      start.value.segment_id,
      start.value.segment_distance_m / start_segment->length_m};
  request.transition_complete = {
      complete.value.segment_id,
      complete.value.segment_distance_m / complete_segment->length_m};
  return true;
}

bool SetAddLanePositions(RoadState& state, city::road::AddLaneRequest& request,
                         double start_distance_m, double complete_distance_m) {
  const auto set_position = [&](double corridor_distance_m,
                                city::road::SegmentPosition& out) {
    const auto resolved = city::road::ResolveCorridorDistance(
        state.graph(), state.derived(),
        city::road::CorridorDistanceRef{request.corridor_id,
                                        corridor_distance_m});
    const auto* segment =
        resolved.ok ? city::road::FindDerivedSegment(state.derived(),
                                                     resolved.value.segment_id)
                    : nullptr;
    if (!resolved.ok || segment == nullptr || segment->length_m <= 1e-9) {
      return false;
    }
    out = {resolved.value.segment_id,
           resolved.value.segment_distance_m / segment->length_m};
    return true;
  };
  return set_position(start_distance_m, request.transition_start) &&
         set_position(complete_distance_m, request.transition_complete);
}

bool mesh_faces_up(const Mesh& mesh) {
  for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    const auto& a = mesh.vertices[mesh.indices[i]];
    const auto& b = mesh.vertices[mesh.indices[i + 1]];
    const auto& c = mesh.vertices[mesh.indices[i + 2]];
    const double ux = b.x - a.x;
    const double uy = b.y - a.y;
    const double uz = b.z - a.z;
    const double vx = c.x - a.x;
    const double vy = c.y - a.y;
    const double vz = c.z - a.z;
    const double nx = uy * vz - uz * vy;
    const double ny = uz * vx - ux * vz;
    const double nz = ux * vy - uy * vx;
    if (nz < -1e-9 && std::abs(nz) >= std::hypot(nx, ny)) {
      return false;
    }
  }
  return true;
}

double orient_2d(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
  return (b.x - a.x) * (c.y - a.y) -
         (b.y - a.y) * (c.x - a.x);
}

bool point_inside_triangle_2d(const Vec3d& point, const Vec3d& a,
                              const Vec3d& b, const Vec3d& c) {
  constexpr double epsilon = 1e-9;
  const double ab = orient_2d(a, b, point);
  const double bc = orient_2d(b, c, point);
  const double ca = orient_2d(c, a, point);
  return (ab > epsilon && bc > epsilon && ca > epsilon) ||
         (ab < -epsilon && bc < -epsilon && ca < -epsilon);
}

bool point_inside_or_on_polygon_2d(const Vec3d& point,
                                   const std::vector<Vec3d>& polygon) {
  constexpr double epsilon = 1e-9;
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1;
       index < polygon.size(); previous = index++) {
    const Vec3d& a = polygon[previous];
    const Vec3d& b = polygon[index];
    const double edge = orient_2d(a, b, point);
    if (std::abs(edge) <= epsilon &&
        point.x >= std::min(a.x, b.x) - epsilon &&
        point.x <= std::max(a.x, b.x) + epsilon &&
        point.y >= std::min(a.y, b.y) - epsilon &&
        point.y <= std::max(a.y, b.y) + epsilon) {
      return true;
    }
    const bool crosses =
        ((a.y > point.y) != (b.y > point.y)) &&
        (point.x < (b.x - a.x) * (point.y - a.y) /
                           ((b.y - a.y) == 0.0 ? 1.0 : b.y - a.y) +
                       a.x);
    if (crosses)
      inside = !inside;
  }
  return inside;
}

bool segments_cross_2d(const Vec3d& a, const Vec3d& b, const Vec3d& c,
                       const Vec3d& d) {
  constexpr double epsilon = 1e-9;
  const double ab_c = orient_2d(a, b, c);
  const double ab_d = orient_2d(a, b, d);
  const double cd_a = orient_2d(c, d, a);
  const double cd_b = orient_2d(c, d, b);
  return ab_c * ab_d < -epsilon && cd_a * cd_b < -epsilon;
}

std::optional<std::pair<std::size_t, std::size_t>>
polygon_self_intersection_2d(const std::vector<Vec3d>& polygon) {
  for (std::size_t first = 0; first < polygon.size(); ++first) {
    const std::size_t first_next = (first + 1) % polygon.size();
    for (std::size_t second = first + 1; second < polygon.size(); ++second) {
      const std::size_t second_next = (second + 1) % polygon.size();
      if (first == second || first_next == second || second_next == first)
        continue;
      if (segments_cross_2d(polygon[first], polygon[first_next],
                            polygon[second], polygon[second_next]))
        return std::pair{first, second};
    }
  }
  return std::nullopt;
}

bool triangles_overlap_2d(const Vec3d& a, const Vec3d& b, const Vec3d& c,
                          const Vec3d& d, const Vec3d& e, const Vec3d& f) {
  const std::array<std::pair<Vec3d, Vec3d>, 3> first_edges{
      std::pair{a, b}, std::pair{b, c}, std::pair{c, a}};
  const std::array<std::pair<Vec3d, Vec3d>, 3> second_edges{
      std::pair{d, e}, std::pair{e, f}, std::pair{f, d}};
  for (const auto& first : first_edges) {
    for (const auto& second : second_edges) {
      if (segments_cross_2d(first.first, first.second, second.first,
                            second.second)) {
        return true;
      }
    }
  }
  const Vec3d first_center{(a.x + b.x + c.x) / 3.0,
                           (a.y + b.y + c.y) / 3.0, 0.0};
  const Vec3d second_center{(d.x + e.x + f.x) / 3.0,
                            (d.y + e.y + f.y) / 3.0, 0.0};
  return point_inside_triangle_2d(first_center, d, e, f) ||
         point_inside_triangle_2d(second_center, a, b, c);
}

std::optional<std::pair<std::size_t, std::size_t>>
overlapping_face_pair(const Mesh& mesh) {
  for (std::size_t first = 0; first + 2 < mesh.indices.size(); first += 3) {
    const Vec3d& a = mesh.vertices[mesh.indices[first]];
    const Vec3d& b = mesh.vertices[mesh.indices[first + 1]];
    const Vec3d& c = mesh.vertices[mesh.indices[first + 2]];
    if (std::abs(orient_2d(a, b, c)) <= 1e-9)
      continue;
    for (std::size_t second = first + 3; second + 2 < mesh.indices.size();
         second += 3) {
      const Vec3d& d = mesh.vertices[mesh.indices[second]];
      const Vec3d& e = mesh.vertices[mesh.indices[second + 1]];
      const Vec3d& f = mesh.vertices[mesh.indices[second + 2]];
      if (std::abs(orient_2d(d, e, f)) <= 1e-9)
        continue;
      if (triangles_overlap_2d(a, b, c, d, e, f))
        return std::pair{first / 3, second / 3};
    }
  }
  return std::nullopt;
}

std::string face_points(const Mesh& mesh, std::size_t face) {
  std::ostringstream text;
  for (std::size_t corner = 0; corner < 3; ++corner) {
    const Vec3d& point = mesh.vertices[mesh.indices[face * 3 + corner]];
    if (corner != 0)
      text << ';';
    text << point.x << ',' << point.y;
  }
  return text.str();
}

bool junction_fan_is_valid(const RoadState& state, const std::string& label,
                           std::string& failure) {
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1 &&
                       junctions.front()->junction_geometry.surface_regions.size() == 1,
                   label + " junction geometry is missing");
  const auto& road_region =
      junctions.front()->junction_geometry.surface_regions.front().perimeter;
  const auto perimeter_crossing = polygon_self_intersection_2d(road_region);
  ROAD_TEST_EXPECT(
      !perimeter_crossing.has_value(),
      label + " central road perimeter self-intersects between edges " +
          (perimeter_crossing.has_value()
               ? std::to_string(perimeter_crossing->first) + " and " +
                     std::to_string(perimeter_crossing->second)
               : "unknown"));
  double perimeter_area = 0.0;
  for (std::size_t index = 0; index < road_region.size(); ++index) {
    const Vec3d& a = road_region[index];
    const Vec3d& b = road_region[(index + 1) % road_region.size()];
    perimeter_area += a.x * b.y - b.x * a.y;
  }
  perimeter_area = std::abs(perimeter_area * 0.5);
  bool checked_fan = false;
  for (const Mesh& mesh : state.derived().junction_meshes) {
    if (mesh.style == RenderStyleFromSurface(builtin_surface_styles::kAsphalt) &&
        mesh.uv_mapping == MeshUvMapping::kWorld &&
        mesh.vertices.size() == road_region.size() + 1 &&
        mesh.indices.size() == road_region.size() * 3) {
      double triangle_area_sum = 0.0;
      for (std::size_t face = 0; face + 2 < mesh.indices.size(); face += 3) {
        const Vec3d& a = mesh.vertices[mesh.indices[face]];
        const Vec3d& b = mesh.vertices[mesh.indices[face + 1]];
        const Vec3d& c = mesh.vertices[mesh.indices[face + 2]];
        const double area = orient_2d(a, b, c) * 0.5;
        ROAD_TEST_EXPECT(area > 1e-9,
                         label + " junction fan triangle has inconsistent winding");
        const Vec2d& uva = mesh.uv0[mesh.indices[face]];
        const Vec2d& uvb = mesh.uv0[mesh.indices[face + 1]];
        const Vec2d& uvc = mesh.uv0[mesh.indices[face + 2]];
        const double uv_area =
            std::abs(((uvb.x - uva.x) * (uvc.y - uva.y) -
                      (uvc.x - uva.x) * (uvb.y - uva.y)) *
                     0.5);
        ROAD_TEST_EXPECT(uv_area > 1e-12,
                         label + " junction fan triangle has collapsed UV");
        const Vec3d centroid{(a.x + b.x + c.x) / 3.0,
                             (a.y + b.y + c.y) / 3.0,
                             (a.z + b.z + c.z) / 3.0};
        ROAD_TEST_EXPECT(point_inside_or_on_polygon_2d(centroid, road_region),
                         label + " junction fan triangle centroid is outside the perimeter");
        triangle_area_sum += area;
      }
      ROAD_TEST_EXPECT(
          std::abs(triangle_area_sum - perimeter_area) <=
              std::max(1e-6, perimeter_area * 1e-6),
          label + " junction fan triangle area does not match the perimeter");
      checked_fan = true;
    }
  }
  ROAD_TEST_EXPECT(checked_fan, label + " junction fan mesh was not found");
  return true;
}

bool junction_mesh_is_non_overlapping(const RoadState& state,
                                      const std::string& label,
                                      std::string& failure) {
  ROAD_TEST_EXPECT(junction_fan_is_valid(state, label, failure), failure);
  for (const Mesh& mesh : state.derived().junction_meshes) {
    const auto overlap = overlapping_face_pair(mesh);
    ROAD_TEST_EXPECT(
        !overlap.has_value(),
        label + " junction mesh overlaps in style " +
            std::to_string(mesh.style.value) +
            (overlap.has_value()
                 ? " between triangles " + std::to_string(overlap->first) +
                       " [" + face_points(mesh, overlap->first) + "] and " +
                       std::to_string(overlap->second) + " [" +
                       face_points(mesh, overlap->second) + "]"
                 : ""));
  }
  return true;
}

city::road::RoadLayoutTemplate OneWayLaneTemplate(
    city::road::RoadLayoutTemplateId id, std::size_t lane_count,
    city::road::LaneTravelDirection direction =
        city::road::LaneTravelDirection::kAlongSegment) {
  const AutoMarkingPolicy edge{
      true, MarkingRole::kCarriagewayEdge,
      builtin_marking_styles::kWhiteSolid, MarkingPlacement::kInside};
  const AutoMarkingPolicy divider{
      true, MarkingRole::kLaneSeparator,
      builtin_marking_styles::kWhiteDashed, MarkingPlacement::kCenter};
  city::road::RoadLayoutTemplate section{};
  section.id = id;
  section.strips.push_back({10, StripFunction::kShoulder, 0.75, 0.0,
                            builtin_surface_styles::kAsphalt});
  for (std::size_t index = 0; index < lane_count; ++index) {
    const auto strip_id = static_cast<city::road::RoadLayoutStripId>(20 + index * 10);
    const auto lane_id = static_cast<city::road::LaneId>(1000 + index * 10);
    section.strips.push_back({strip_id, StripFunction::kCarriageway, 3.0,
                              0.0, builtin_surface_styles::kAsphalt});
    section.lane_bands.push_back(
        {lane_id, strip_id, 0.0, 3.0,
         direction});
  }
  section.strips.push_back({90, StripFunction::kShoulder, 0.75, 0.0,
                            builtin_surface_styles::kAsphalt});
  section.boundaries.push_back(
      road_fixture::PaintedLineBoundary(100, BoundaryRole::kOuterEdge, edge));
  for (std::size_t index = 1; index < lane_count; ++index) {
    section.boundaries.push_back(road_fixture::PaintedLineBoundary(
        static_cast<city::road::BoundaryId>(100 + index * 100),
        BoundaryRole::kLaneDivider, divider));
  }
  section.boundaries.push_back(
      road_fixture::PaintedLineBoundary(900, BoundaryRole::kOuterEdge, edge));
  section.alignment_offset_from_left_m =
      road_fixture::CentredAlignmentOffset(section);
  return section;
}

city::road::RoadLayoutTemplate SharedCarriagewayLaneTemplate(
    city::road::RoadLayoutTemplateId id) {
  city::road::RoadLayoutTemplate section{};
  section.id = id;
  section.strips = {
      {10, StripFunction::kShoulder, 0.75, 0.0,
       builtin_surface_styles::kAsphalt},
      {20, StripFunction::kCarriageway, 6.0, 0.0,
       builtin_surface_styles::kAsphalt},
      {30, StripFunction::kShoulder, 0.75, 0.0,
       builtin_surface_styles::kAsphalt},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0,
       city::road::LaneTravelDirection::kAlongSegment},
      {1010, 20, 3.0, 6.0,
       city::road::LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      road_fixture::PaintedLineBoundary(100, BoundaryRole::kOuterEdge, {}),
      road_fixture::PaintedLineBoundary(900, BoundaryRole::kOuterEdge, {}),
  };
  section.alignment_offset_from_left_m =
      road_fixture::CentredAlignmentOffset(section);
  return section;
}

city::road::RoadLayoutTemplate OpposingFourLaneTemplate(
    city::road::RoadLayoutTemplateId id) {
  const AutoMarkingPolicy edge{
      true, MarkingRole::kCarriagewayEdge,
      builtin_marking_styles::kWhiteSolid, MarkingPlacement::kInside};
  const AutoMarkingPolicy divider{
      true, MarkingRole::kLaneSeparator,
      builtin_marking_styles::kWhiteDashed, MarkingPlacement::kCenter};
  city::road::RoadLayoutTemplate section{};
  section.id = id;
  section.strips = {
      {10, StripFunction::kShoulder, 0.75, 0.0,
       builtin_surface_styles::kAsphalt},
      {20, StripFunction::kCarriageway, 3.0, 0.0,
       builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, 0.0,
       builtin_surface_styles::kAsphalt},
      {40, StripFunction::kMedian, 2.0, 0.0,
       builtin_surface_styles::kMedian},
      {50, StripFunction::kCarriageway, 3.0, 0.0,
       builtin_surface_styles::kAsphalt},
      {60, StripFunction::kCarriageway, 3.0, 0.0,
       builtin_surface_styles::kAsphalt},
      {70, StripFunction::kShoulder, 0.75, 0.0,
       builtin_surface_styles::kAsphalt},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0,
       city::road::LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0,
       city::road::LaneTravelDirection::kAgainstSegment},
      {2000, 50, 0.0, 3.0,
       city::road::LaneTravelDirection::kAlongSegment},
      {2010, 60, 0.0, 3.0,
       city::road::LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      road_fixture::PaintedLineBoundary(100, BoundaryRole::kOuterEdge, edge),
      road_fixture::PaintedLineBoundary(150, BoundaryRole::kLaneDivider, divider),
      road_fixture::CurbBoundary(200, 0.0, 0.0, {}),
      road_fixture::CurbBoundary(210, 0.0, 0.0, {}),
      road_fixture::PaintedLineBoundary(250, BoundaryRole::kLaneDivider, divider),
      road_fixture::PaintedLineBoundary(300, BoundaryRole::kOuterEdge, edge),
  };
  return section;
}

bool P0_generates_two_lane_segment(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1, "P0 did not save one authoritative segment");
  ROAD_TEST_EXPECT(road_test_view::sections(state.derived()).size() > 2, "P0 did not derive section samples");
  ROAD_TEST_EXPECT(!state.derived().segment_meshes.empty(), "P0 did not derive a surface mesh");
  ROAD_TEST_EXPECT(!state.derived().terrain_masks.empty(), "P0 did not derive a terrain mask");
  const auto registered = std::find_if(
      state.graph().layout_templates.begin(), state.graph().layout_templates.end(),
      [section](const auto& item) { return item.id == section; });
  ROAD_TEST_EXPECT(registered != state.graph().layout_templates.end(),
                   "P0 did not save the registered section template");
  ROAD_TEST_EXPECT(registered->strips.size() == 4, "bidirectional layout should have sidewalk/lane/lane/sidewalk");
  ROAD_TEST_EXPECT(std::abs(registered->strips[1].width_m - 3.0) < 1e-9, "lane width is not 3.0m");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok, "P0 invariants failed");
  return true;
}

bool P0_two_lane_mesh_shows_sidewalks_curbs_and_markings(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(!road_test_view::sections(state.derived()).empty(), "P0 did not evaluate the section");
  const auto& boundaries = road_test_view::sections(state.derived()).front()->boundaries;
  ROAD_TEST_EXPECT(boundaries.size() >= 7, "P0 section does not expose sidewalk/curb/carriageway edges");
  ROAD_TEST_EXPECT(std::abs(boundaries.front().height_m - 0.132) < 1e-9,
                   "left sidewalk outer edge does not slope across the 1.8m its curb leaves it");
  ROAD_TEST_EXPECT(std::abs(boundaries[1].height_m - 0.15) < 1e-9,
                   "left curb top is not 0.15m above the carriageway edge");
  ROAD_TEST_EXPECT(std::abs(boundaries.back().height_m - 0.132) < 1e-9,
                   "right sidewalk outer edge does not slope across the 1.8m its curb leaves it");
  ROAD_TEST_EXPECT(std::abs(boundaries[5].height_m - 0.15) < 1e-9,
                   "right curb top is not 0.15m above the carriageway edge");
  ROAD_TEST_EXPECT(std::abs(boundaries[2].height_m) < 1e-9, "left carriageway edge is not at road height");
  ROAD_TEST_EXPECT(std::abs(boundaries[4].height_m) < 1e-9, "right carriageway edge is not at road height");
  ROAD_TEST_EXPECT(std::abs(boundaries[3].height_m - 0.06) < 1e-9,
                   "two-lane carriageway does not have a 2% center crown");
  ROAD_TEST_EXPECT(std::abs((boundaries[2].lateral_m - boundaries[1].lateral_m) - 0.2) < 1e-9,
                   "left curb width is not 0.2m");
  ROAD_TEST_EXPECT(std::abs((boundaries[5].lateral_m - boundaries[4].lateral_m) - 0.2) < 1e-9,
                   "right curb width is not 0.2m");
  ROAD_TEST_EXPECT(!state.derived().marking_meshes.empty(), "P0 did not derive marking meshes");
  std::vector<double> outer_line_laterals{};
  for (const auto& boundary : boundaries) {
    if (boundary.marking.enabled &&
        boundary.marking.role == MarkingRole::kCarriagewayEdge) {
      outer_line_laterals.push_back(boundary.lateral_m);
    }
  }
  ROAD_TEST_EXPECT(
      outer_line_laterals.size() == 2 &&
          std::abs(outer_line_laterals[0] + 3.0) < 1e-9 &&
          std::abs(outer_line_laterals[1] - 3.0) < 1e-9,
      "outer lines are not on both carriageway-adjacent curb edges");
  const auto& marking = state.derived().marking_meshes.front();
  ROAD_TEST_EXPECT(!marking.vertices.empty(), "P0 marking mesh has no vertices");
  ROAD_TEST_EXPECT(!marking.indices.empty(), "P0 marking mesh has no triangles");
  std::set<RenderStyleRef> styles{};
  for (const auto& mesh : state.derived().segment_meshes) styles.insert(mesh.style);
  ROAD_TEST_EXPECT(styles.contains(RenderStyleFromSurface(builtin_surface_styles::kAsphalt)) &&
                       styles.contains(RenderStyleFromSurface(builtin_surface_styles::kSidewalk)) &&
                       styles.contains(RenderStyleFromSurface(builtin_surface_styles::kCurb)),
                   "P0 surface meshes are not separated by core style semantics");
  return true;
}

bool mesh_attributes_are_complete(const Mesh& mesh, std::string& failure) {
  ROAD_TEST_EXPECT(mesh.normals.size() == mesh.vertices.size(),
                   "mesh normal count does not match vertex count");
  ROAD_TEST_EXPECT(mesh.uv0.size() == mesh.vertices.size(),
                   "mesh uv count does not match vertex count");
  ROAD_TEST_EXPECT(mesh.material_groups.size() == 1,
                   "mesh does not expose its material group");
  ROAD_TEST_EXPECT(mesh.material_groups.front().style == mesh.style,
                   "mesh material group style does not match mesh style");
  ROAD_TEST_EXPECT(mesh.material_groups.front().index_start == 0,
                   "mesh material group starts after the first index");
  ROAD_TEST_EXPECT(mesh.material_groups.front().index_count == mesh.indices.size(),
                   "mesh material group does not cover every index");
  for (const Vec3d& normal : mesh.normals) {
    ROAD_TEST_EXPECT(std::isfinite(normal.x) && std::isfinite(normal.y) &&
                         std::isfinite(normal.z),
                     "mesh normal is not finite");
  }
  for (const Vec2d& uv : mesh.uv0) {
    ROAD_TEST_EXPECT(std::isfinite(uv.x) && std::isfinite(uv.y),
                     "mesh uv is not finite");
  }
  return true;
}

const Mesh* first_asphalt_mesh_for_segment(const RoadState& state,
                                           RoadSegmentId segment_id) {
  const RenderStyleRef asphalt =
      RenderStyleFromSurface(builtin_surface_styles::kAsphalt);
  const auto found = std::find_if(
      state.derived().segment_meshes.begin(),
      state.derived().segment_meshes.end(),
      [&](const Mesh& mesh) {
        return mesh.owner_segment_id == segment_id && mesh.style == asphalt;
      });
  return found == state.derived().segment_meshes.end() ? nullptr : &*found;
}

double max_u(const Mesh& mesh) {
  double value = -std::numeric_limits<double>::infinity();
  for (const Vec2d& uv : mesh.uv0)
    value = std::max(value, uv.x);
  return value;
}

double indexed_uv_v_span(const Mesh& mesh) {
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (const std::uint32_t index : mesh.indices) {
    if (index >= mesh.uv0.size()) continue;
    min_v = std::min(min_v, mesh.uv0[index].y);
    max_v = std::max(max_v, mesh.uv0[index].y);
  }
  return std::isfinite(min_v) && std::isfinite(max_v) ? max_v - min_v : 0.0;
}

std::vector<Vec2d> uvs_at_yz(const Mesh& mesh, double y, double z) {
  std::vector<Vec2d> out{};
  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    const Vec3d& vertex = mesh.vertices[index];
    if (std::abs(vertex.y - y) <= 1e-9 &&
        std::abs(vertex.z - z) <= 1e-9) {
      out.push_back(mesh.uv0[index]);
    }
  }
  return out;
}

bool has_u_at_x(const Mesh& mesh, double x, double expected_u) {
  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    if (std::abs(mesh.vertices[index].x - x) <= 1e-9 &&
        std::abs(mesh.uv0[index].x - expected_u) <= 1e-9) {
      return true;
    }
  }
  return false;
}

bool has_u_near_segment_distance(const Mesh& mesh, const Path& path,
                                 double segment_distance_m,
                                 double expected_u) {
  const auto center = EvaluatePath(path, segment_distance_m);
  if (!center.ok)
    return false;
  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    const Vec3d& vertex = mesh.vertices[index];
    if (std::hypot(vertex.x - center.value.x, vertex.y - center.value.y) <= 8.0 &&
        std::abs(mesh.uv0[index].x - expected_u) <= 1e-9) {
      return true;
    }
  }
  return false;
}

std::string u_values_at_x(const Mesh& mesh, double x) {
  std::ostringstream out{};
  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    if (std::abs(mesh.vertices[index].x - x) <= 1e-9)
      out << mesh.uv0[index].x << ",";
  }
  return out.str();
}

std::optional<double> nearest_z(const std::vector<Vec3d>& points, Vec2d xy,
                                double max_distance_m) {
  std::optional<double> z{};
  double best = max_distance_m;
  for (const Vec3d& point : points) {
    const double distance = std::hypot(point.x - xy.x, point.y - xy.y);
    if (distance <= best) {
      best = distance;
      z = point.z;
    }
  }
  return z;
}

bool mesh_uv_normals_and_material_groups_are_local(std::string& failure) {
  RoadState short_state{};
  const auto section =
      road_fixture::AddLayout(short_state, road_fixture::BidirectionalLayout(0));
  const auto short_segment = short_state.AddSegment(
      city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {2.0, 0.0})}), section});
  ROAD_TEST_EXPECT(short_segment.ok, short_segment.error);
  const Mesh* short_mesh =
      first_asphalt_mesh_for_segment(short_state, short_segment.value);
  ROAD_TEST_EXPECT(short_mesh != nullptr, "short segment asphalt mesh is missing");
  ROAD_TEST_EXPECT(mesh_attributes_are_complete(*short_mesh, failure), failure);
  ROAD_TEST_EXPECT(short_mesh->uv_mapping == MeshUvMapping::kWorld,
                   "planar asphalt surface does not use metric UV");
  ROAD_TEST_EXPECT(std::abs(max_u(*short_mesh) - 2.0) < 1e-9,
                   "short asphalt segment U is not measured in meters");
  ROAD_TEST_EXPECT(!short_state.derived().marking_meshes.empty(),
                   "short segment marking mesh is missing");
  ROAD_TEST_EXPECT(mesh_attributes_are_complete(
                        short_state.derived().marking_meshes.front(), failure),
                   failure);
  ROAD_TEST_EXPECT(short_state.derived().marking_meshes.front().uv_mapping ==
                       MeshUvMapping::kWorld,
                   "marking ribbon does not use metric local UV");
  ROAD_TEST_EXPECT(max_u(short_state.derived().marking_meshes.front()) > 0.0,
                   "marking ribbon U does not progress along the line");
  ROAD_TEST_EXPECT(indexed_uv_v_span(short_state.derived().marking_meshes.front()) >
                       0.0,
                   "marking ribbon V does not span the line width");

  RoadState long_state{};
  const auto long_section =
      road_fixture::AddLayout(long_state, road_fixture::BidirectionalLayout(0));
  const auto long_segment = long_state.AddSegment(
      city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {78.0, 0.0})}), long_section});
  ROAD_TEST_EXPECT(long_segment.ok, long_segment.error);
  const Mesh* long_mesh =
      first_asphalt_mesh_for_segment(long_state, long_segment.value);
  ROAD_TEST_EXPECT(long_mesh != nullptr, "long segment asphalt mesh is missing");
  ROAD_TEST_EXPECT(mesh_attributes_are_complete(*long_mesh, failure), failure);
  ROAD_TEST_EXPECT(std::abs(max_u(*long_mesh) - 78.0) < 1e-9,
                   "long asphalt segment U is not measured in meters");
  ROAD_TEST_EXPECT(std::abs(indexed_uv_v_span(*short_mesh) - 6.0) < 1e-9,
                   "two-lane asphalt V is not measured in lateral meters");
  RoadState wide_state{};
  const auto wide_section =
      road_fixture::AddLayout(wide_state, road_fixture::ExtraLaneLayout(0));
  const auto wide_segment = wide_state.AddSegment(
      city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {2.0, 0.0})}), wide_section});
  ROAD_TEST_EXPECT(wide_segment.ok, wide_segment.error);
  const Mesh* wide_mesh =
      first_asphalt_mesh_for_segment(wide_state, wide_segment.value);
  ROAD_TEST_EXPECT(wide_mesh != nullptr, "wide asphalt UV fixture is missing");
  ROAD_TEST_EXPECT(indexed_uv_v_span(*wide_mesh) >
                       indexed_uv_v_span(*short_mesh),
                   "wider asphalt section was normalized to the same V span");

  auto branch_segment_u = [](double upstream_length_m) {
    RoadState state{};
    const auto layout =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto upstream = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {upstream_length_m, 0.0})}), layout});
    const auto branch = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 40.0}, {25.0, 40.0})}), layout});
    if (!upstream.ok || !branch.ok) return -1.0;
    const Mesh* mesh = first_asphalt_mesh_for_segment(state, branch.value);
    return mesh == nullptr ? -1.0 : max_u(*mesh);
  };
  const double short_upstream = branch_segment_u(20.0);
  const double long_upstream = branch_segment_u(40.0);
  ROAD_TEST_EXPECT(short_upstream >= 0.0 && long_upstream >= 0.0,
                   "branch UV fixture did not generate asphalt meshes");
  ROAD_TEST_EXPECT(std::abs(short_upstream - long_upstream) < 1e-9,
                   "unrelated upstream segment length changed another segment UV");

  RoadState junction_state{};
  const auto junction_section = road_fixture::AddLayout(
      junction_state, road_fixture::BidirectionalLayout(0));
  const auto base = junction_state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-20.0, 0.0}, {20.0, 0.0})}), junction_section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = junction_state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 20.0})}), junction_section,
          base.value, 20.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto junctions = road_test_view::junctions(junction_state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1, "junction UV fixture did not create one junction");
  ROAD_TEST_EXPECT(
      !junctions.front()->junction_geometry.surface_regions.empty(),
      "junction UV fixture has no interior surface region");
  const auto& perimeter =
      junctions.front()->junction_geometry.surface_regions.front().perimeter;
  const Mesh* junction_interior = nullptr;
  for (const Mesh& mesh : junction_state.derived().junction_meshes) {
    ROAD_TEST_EXPECT(mesh_attributes_are_complete(mesh, failure), failure);
    if (mesh.style == RenderStyleFromSurface(builtin_surface_styles::kAsphalt) &&
        mesh.uv_mapping == MeshUvMapping::kWorld &&
        mesh.vertices.size() == perimeter.size() + 1) {
      junction_interior = &mesh;
    }
  }
  ROAD_TEST_EXPECT(junction_interior != nullptr,
                   "junction interior is not emitted as a center fan over the resolved perimeter");
  return true;
}

bool junction_fan_uv_covers_basic_and_asymmetric_cases(
    std::string& failure) {
  {
    RoadState state{};
    const auto section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-20.0, 0.0}, {20.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto branch = state.AddSegmentConnectedToSegment(
        city::road::AddSegmentConnectedToSegmentRequest{
            MakePath({MakeLine({0.0, 0.0}, {0.0, 20.0})}), section,
            base.value, 20.0});
    ROAD_TEST_EXPECT(branch.ok, branch.error);
    ROAD_TEST_EXPECT(junction_fan_is_valid(state, "normal T", failure),
                     failure);
  }
  {
    RoadState state{};
    const auto section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto north = state.AddSegmentConnectedToSegment(
        city::road::AddSegmentConnectedToSegmentRequest{
            MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), section,
            base.value, 20.0});
    ROAD_TEST_EXPECT(north.ok, north.error);
    const auto junctions = road_test_view::junctions(state.derived());
    ROAD_TEST_EXPECT(junctions.size() == 1,
                     "normal cross fixture did not create a T junction");
    const auto south = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({20.0, 0.0}, {20.0, -24.0})}), section,
            junctions.front()->node_id});
    ROAD_TEST_EXPECT(south.ok, south.error);
    ROAD_TEST_EXPECT(junction_fan_is_valid(state, "normal cross", failure),
                     failure);
  }
  {
    RoadState state{};
    auto asymmetric = road_fixture::BidirectionalLayout(0);
    asymmetric.alignment_offset_from_left_m = 2.0;
    const auto base_section = road_fixture::AddLayout(state, asymmetric);
    const auto branch_section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), base_section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto branch = state.AddSegmentConnectedToSegment(
        city::road::AddSegmentConnectedToSegmentRequest{
            MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), branch_section,
            base.value, 40.0});
    ROAD_TEST_EXPECT(branch.ok, branch.error);
    ROAD_TEST_EXPECT(junction_fan_is_valid(state, "asymmetric width T", failure),
                     failure);
  }
  return true;
}

bool gutter_profile_uv_uses_contour_v_and_patch_u(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto sections = road_test_view::sections(state.derived());
  ROAD_TEST_EXPECT(!sections.empty(), "gutter UV fixture has no section");
  const auto& boundaries = sections.front()->boundaries;
  ROAD_TEST_EXPECT(boundaries.size() >= 4, "gutter UV fixture has no vertical gutter face");
  ROAD_TEST_EXPECT(std::abs(boundaries[2].lateral_m - boundaries[3].lateral_m) < 1e-9 &&
                       std::abs(boundaries[2].height_m - boundaries[3].height_m) > 1e-9,
                   "gutter UV fixture no longer has a vertical profile face");

  const auto curb_mesh = std::find_if(
      state.derived().segment_meshes.begin(), state.derived().segment_meshes.end(),
      [](const Mesh& mesh) {
        return mesh.style == RenderStyleFromSurface(builtin_surface_styles::kCurb);
      });
  ROAD_TEST_EXPECT(curb_mesh != state.derived().segment_meshes.end(),
                   "gutter UV fixture has no curb mesh");
  ROAD_TEST_EXPECT(curb_mesh->uv_mapping == MeshUvMapping::kPatchQuantized,
                   "gutter sweep does not use patch-quantized UV");
  ROAD_TEST_EXPECT(std::abs(max_u(*curb_mesh) - 1.25) < 1e-9,
                   "long gutter sweep did not quantize past U=1 for repeating texture");
  const std::vector<Vec2d> top =
      uvs_at_yz(*curb_mesh, boundaries[2].lateral_m, boundaries[2].height_m);
  const std::vector<Vec2d> bottom =
      uvs_at_yz(*curb_mesh, boundaries[3].lateral_m, boundaries[3].height_m);
  ROAD_TEST_EXPECT(!top.empty() && !bottom.empty(),
                   "gutter vertical face vertices were not emitted");
  ROAD_TEST_EXPECT(std::abs(top.front().y - bottom.front().y) > 1e-9,
                   "gutter vertical face collapsed to one V coordinate");
  ROAD_TEST_EXPECT(std::abs(top.front().y - boundaries[2].profile_v_m) < 1e-9 &&
                       std::abs(bottom.front().y - boundaries[3].profile_v_m) < 1e-9,
                   "gutter V is not the profile-local contour distance");
  ROAD_TEST_EXPECT(boundaries[1].profile_v_m < boundaries[2].profile_v_m &&
                       boundaries[2].profile_v_m < boundaries[3].profile_v_m &&
                       boundaries[3].profile_v_m < boundaries[4].profile_v_m,
                   "default profile V does not progress along the gutter contour");

  const auto* corridor =
      city::road::FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "gutter transition UV fixture has no corridor");
  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.lane_width_m = 3.0;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "gutter transition UV range could not be resolved");
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const Mesh* asphalt =
      first_asphalt_mesh_for_segment(state, segment.value);
  ROAD_TEST_EXPECT(asphalt != nullptr, "transition UV fixture has no asphalt mesh");
  ROAD_TEST_EXPECT(asphalt->uv_mapping == MeshUvMapping::kWorld,
                   "transition asphalt did not keep metric planar UV");
  ROAD_TEST_EXPECT(has_u_at_x(*asphalt, 20.0, 20.0) &&
                       has_u_at_x(*asphalt, 20.0, 0.0),
                   "transition start did not keep metric U while splitting the derived patch");
  ROAD_TEST_EXPECT(has_u_at_x(*asphalt, 60.0, 40.0) &&
                       has_u_at_x(*asphalt, 60.0, 0.0),
                   "transition completion did not keep metric U while splitting the derived patch: " +
                       u_values_at_x(*asphalt, 60.0));

  RoadState short_state{};
  const auto short_layout =
      road_fixture::AddLayout(short_state, road_fixture::BidirectionalLayout(0));
  const auto short_segment = short_state.AddSegment(
      city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {10.0, 0.0})}), short_layout});
  ROAD_TEST_EXPECT(short_segment.ok, short_segment.error);
  const auto* short_corridor =
      city::road::FindCorridorForSegment(short_state.graph(), short_segment.value);
  ROAD_TEST_EXPECT(short_corridor != nullptr, "short transition UV fixture has no corridor");
  city::road::AddLaneRequest short_request{};
  short_request.corridor_id = short_corridor->id;
  short_request.direction = city::road::LaneTravelDirection::kAlongSegment;
  short_request.side = city::road::RoadSide::kRight;
  short_request.lane_width_m = 3.0;
  ROAD_TEST_EXPECT(SetAddLaneRange(short_state, short_request, 2.0, 4.0),
                   "short transition UV range could not be resolved");
  const auto short_added = short_state.AddLane(short_request);
  ROAD_TEST_EXPECT(short_added.ok, short_added.error);
  const Mesh* short_asphalt =
      first_asphalt_mesh_for_segment(short_state, short_segment.value);
  ROAD_TEST_EXPECT(short_asphalt != nullptr, "short transition UV fixture has no asphalt mesh");
  ROAD_TEST_EXPECT(has_u_at_x(*short_asphalt, 2.0, 2.0) &&
                       has_u_at_x(*short_asphalt, 2.0, 0.0) &&
                       has_u_at_x(*short_asphalt, 4.0, 2.0),
                   "short transition asphalt did not keep meter-scaled local U");

  RoadState curve_state{};
  const auto curve_layout =
      road_fixture::AddLayout(curve_state, road_fixture::BidirectionalLayout(0));
  const Path curve_path = MakePath({MakeBezier({0.0, 0.0}, {30.0, 0.0},
                                               {30.0, 40.0}, {60.0, 40.0})});
  const auto curve_segment = curve_state.AddSegment(
      city::road::AddSegmentRequest{curve_path, curve_layout});
  ROAD_TEST_EXPECT(curve_segment.ok, curve_segment.error);
  const auto* curve_corridor =
      city::road::FindCorridorForSegment(curve_state.graph(), curve_segment.value);
  ROAD_TEST_EXPECT(curve_corridor != nullptr, "curve transition UV fixture has no corridor");
  city::road::AddLaneRequest curve_request{};
  curve_request.corridor_id = curve_corridor->id;
  curve_request.direction = city::road::LaneTravelDirection::kAgainstSegment;
  curve_request.side = city::road::RoadSide::kLeft;
  curve_request.lane_width_m = 3.0;
  ROAD_TEST_EXPECT(SetAddLaneRange(curve_state, curve_request, 20.0, 40.0),
                   "curve transition UV range could not be resolved");
  const auto curve_added = curve_state.AddLane(curve_request);
  ROAD_TEST_EXPECT(curve_added.ok, curve_added.error);
  const Mesh* curve_asphalt =
      first_asphalt_mesh_for_segment(curve_state, curve_segment.value);
  ROAD_TEST_EXPECT(curve_asphalt != nullptr, "curve transition UV fixture has no asphalt mesh");
  ROAD_TEST_EXPECT(has_u_near_segment_distance(*curve_asphalt, curve_path, 20.0, 20.0) &&
                       has_u_near_segment_distance(*curve_asphalt, curve_path, 40.0, 20.0),
                   "curve transition asphalt did not keep meter-scaled local U");

  const auto saved = short_state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  std::string reversed_archive = saved.value;
  const std::string old_value = "corridor.0.segment.0.reversed=0";
  const std::string new_value = "corridor.0.segment.0.reversed=1";
  const std::size_t position = reversed_archive.find(old_value);
  ROAD_TEST_EXPECT(position != std::string::npos,
                   "reversed UV fixture archive field is missing");
  reversed_archive.replace(position, old_value.size(), new_value);
  const auto reversed = RoadState::Load(reversed_archive);
  ROAD_TEST_EXPECT(reversed.ok, reversed.error);
  const Mesh* reversed_asphalt =
      first_asphalt_mesh_for_segment(reversed.value, short_segment.value);
  ROAD_TEST_EXPECT(reversed_asphalt != nullptr,
                   "reversed transition UV fixture has no asphalt mesh");
  ROAD_TEST_EXPECT(has_u_at_x(*reversed_asphalt, 2.0, 2.0) &&
                       has_u_at_x(*reversed_asphalt, 4.0, 2.0),
                   "reversed corridor changed segment-local metric UV lengths");
  return true;
}

bool road_elevation_profile_drives_derived_geometry(std::string& failure) {
  RoadState state{};
  const auto layout =
      road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  city::road::AddSegmentRequest request{};
  request.alignment = MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})});
  request.layout_template = layout;
  request.start_elevation_m = 0.0;
  request.end_elevation_m = 10.0;
  const auto segment = state.AddSegment(request);
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  ROAD_TEST_EXPECT(state.graph().nodes.size() == 2 &&
                       state.graph().nodes.front().elevation_m == 0.0 &&
                       state.graph().nodes.back().elevation_m == 10.0,
                   "road node elevation is not authoritative");
  const auto derived_segment = std::find_if(
      state.derived().segments.begin(), state.derived().segments.end(),
      [id = segment.value](const auto& item) { return item.id == id; });
  ROAD_TEST_EXPECT(derived_segment != state.derived().segments.end(),
                   "elevated segment was not derived");
  ROAD_TEST_EXPECT(derived_segment->start_elevation_m == 0.0 &&
                       derived_segment->end_elevation_m == 10.0,
                   "derived segment did not keep endpoint elevations");

  const Mesh* asphalt = first_asphalt_mesh_for_segment(state, segment.value);
  ROAD_TEST_EXPECT(asphalt != nullptr, "elevated asphalt mesh is missing");
  const auto asphalt_mid = nearest_z(asphalt->vertices, {50.0, 0.0}, 4.0);
  ROAD_TEST_EXPECT(asphalt_mid.has_value() && *asphalt_mid > 4.9 &&
                       *asphalt_mid < 5.2,
                   "asphalt surface does not follow the vertical profile");
  const auto curb = std::find_if(
      state.derived().segment_meshes.begin(), state.derived().segment_meshes.end(),
      [](const Mesh& mesh) {
        return mesh.style == RenderStyleFromSurface(builtin_surface_styles::kCurb);
      });
  ROAD_TEST_EXPECT(curb != state.derived().segment_meshes.end(),
                   "elevated gutter mesh is missing");
  double gutter_min = std::numeric_limits<double>::infinity();
  double gutter_max = -std::numeric_limits<double>::infinity();
  for (const Vec3d& vertex : curb->vertices) {
    if (std::abs(vertex.x - 50.0) <= 0.1) {
      gutter_min = std::min(gutter_min, vertex.z);
      gutter_max = std::max(gutter_max, vertex.z);
    }
  }
  ROAD_TEST_EXPECT(gutter_max - gutter_min > 0.05,
                   "gutter profile local height collapsed on a grade");

  bool marking_on_grade = false;
  for (const DerivedMarking& marking : state.derived().markings) {
    if (nearest_z(marking.points, {50.0, 0.0}, 4.0).value_or(0.0) > 4.9) {
      marking_on_grade = true;
      break;
    }
  }
  ROAD_TEST_EXPECT(marking_on_grade, "road marking does not follow the grade");
  ROAD_TEST_EXPECT(!state.derived().segment_lane_paths.empty(),
                   "elevated lane path is missing");
  bool lane_path_on_grade = false;
  for (const auto& lane_path : state.derived().segment_lane_paths) {
    if (nearest_z(lane_path.points, {50.0, 0.0}, 5.0).value_or(0.0) > 4.9) {
      lane_path_on_grade = true;
      break;
    }
  }
  ROAD_TEST_EXPECT(lane_path_on_grade,
                   "segment lane path does not follow the grade");

  RoadState curve_state{};
  const auto curve_layout =
      road_fixture::AddLayout(curve_state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest curve_request{};
  curve_request.alignment =
      MakePath({MakeBezier({0.0, 0.0}, {30.0, 40.0}, {70.0, -40.0},
                           {100.0, 0.0})});
  curve_request.layout_template = curve_layout;
  curve_request.start_elevation_m = 0.0;
  curve_request.end_elevation_m = 10.0;
  const auto curve = curve_state.AddSegment(curve_request);
  ROAD_TEST_EXPECT(curve.ok, curve.error);
  const auto& curve_segment = curve_state.derived().segments.front();
  const double mid_distance = curve_segment.length_m * 0.5;
  const auto mid_xy = EvaluatePath(curve_segment.alignment, mid_distance);
  ROAD_TEST_EXPECT(mid_xy.ok, mid_xy.error);
  const Mesh* curve_asphalt =
      first_asphalt_mesh_for_segment(curve_state, curve.value);
  ROAD_TEST_EXPECT(curve_asphalt != nullptr, "elevated curve mesh is missing");
  const auto curve_mid_z = nearest_z(curve_asphalt->vertices, mid_xy.value, 6.0);
  ROAD_TEST_EXPECT(curve_mid_z.has_value() && *curve_mid_z > 4.8 &&
                       *curve_mid_z < 5.3,
                   "curved horizontal alignment did not use distance-based elevation");

  RoadState add_lane_state{};
  const auto add_lane_layout = road_fixture::AddLayout(
      add_lane_state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest add_lane_base{};
  add_lane_base.alignment =
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})});
  add_lane_base.layout_template = add_lane_layout;
  add_lane_base.start_elevation_m = 0.0;
  add_lane_base.end_elevation_m = 10.0;
  const auto add_lane_segment = add_lane_state.AddSegment(add_lane_base);
  ROAD_TEST_EXPECT(add_lane_segment.ok, add_lane_segment.error);
  city::road::AddLaneRequest add_lane{};
  add_lane.corridor_id = add_lane_state.graph().corridors.front().id;
  add_lane.direction = city::road::LaneTravelDirection::kAlongSegment;
  add_lane.side = city::road::RoadSide::kRight;
  add_lane.lane_width_m = 3.0;
  ROAD_TEST_EXPECT(SetAddLanePositions(add_lane_state, add_lane, 20.0, 40.0),
                   "elevated ADD LANE positions could not be resolved");
  const auto added_lane = add_lane_state.AddLane(add_lane);
  ROAD_TEST_EXPECT(added_lane.ok, added_lane.error);
  ROAD_TEST_EXPECT(add_lane_state.graph().nodes.front().elevation_m == 0.0 &&
                       add_lane_state.graph().nodes.back().elevation_m == 10.0,
                   "ADD LANE changed endpoint elevations");
  bool added_lane_path_on_grade = false;
  for (const auto& lane_path : add_lane_state.derived().segment_lane_paths) {
    if (nearest_z(lane_path.points, {50.0, 0.0}, 8.0).value_or(0.0) > 4.9) {
      added_lane_path_on_grade = true;
      break;
    }
  }
  ROAD_TEST_EXPECT(added_lane_path_on_grade,
                   "ADD LANE lane paths do not follow the grade");

  const auto split = state.SplitSegmentAtDistance(
      city::road::SplitSegmentAtDistanceRequest{segment.value, 50.0});
  ROAD_TEST_EXPECT(split.ok, split.error);
  const auto split_node = std::find_if(
      state.graph().nodes.begin(), state.graph().nodes.end(),
      [](const auto& node) {
        return std::abs(node.position.x - 50.0) <= 1e-9 &&
               std::abs(node.position.y) <= 1e-9;
      });
  ROAD_TEST_EXPECT(split_node != state.graph().nodes.end() &&
                       std::abs(split_node->elevation_m - 5.0) <= 1e-9,
                   "split node did not inherit evaluated elevation");

  RoadState branch_state{};
  const auto branch_layout =
      road_fixture::AddLayout(branch_state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest base_request{};
  base_request.alignment = MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})});
  base_request.layout_template = branch_layout;
  base_request.start_elevation_m = 0.0;
  base_request.end_elevation_m = 10.0;
  const auto base = branch_state.AddSegment(base_request);
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = branch_state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({50.0, 0.0}, {50.0, 30.0})}), branch_layout,
          base.value, 50.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  std::size_t elevated_branch_nodes = 0;
  for (const auto& node : branch_state.graph().nodes) {
    if (std::abs(node.elevation_m - 5.0) <= 1e-9) ++elevated_branch_nodes;
  }
  ROAD_TEST_EXPECT(elevated_branch_nodes >= 2,
                   "branch split/free nodes did not inherit source elevation");
  bool junction_surface_on_grade = false;
  for (const Mesh& mesh : branch_state.derived().junction_meshes) {
    for (const Vec3d& vertex : mesh.vertices) {
      if (vertex.z > 4.5) {
        junction_surface_on_grade = true;
        break;
      }
    }
  }
  ROAD_TEST_EXPECT(junction_surface_on_grade,
                   "junction mesh fell back to a world-Z plane");

  RoadNodeId split_node_id = 0;
  RoadSegmentId west_segment = 0;
  RoadSegmentId east_segment = 0;
  for (const auto& node : branch_state.graph().nodes) {
    if (std::abs(node.position.x - 50.0) <= 1e-9 &&
        std::abs(node.position.y) <= 1e-9) {
      split_node_id = node.id;
    }
  }
  ROAD_TEST_EXPECT(split_node_id != 0,
                   "elevated branch split node is missing");
  for (const auto& candidate : branch_state.graph().segments) {
    if (candidate.id == branch.value)
      continue;
    if (candidate.node_a != split_node_id && candidate.node_b != split_node_id)
      continue;
    const RoadNodeId other_id = candidate.node_a == split_node_id
                                    ? candidate.node_b
                                    : candidate.node_a;
    const auto other = std::find_if(
        branch_state.graph().nodes.begin(), branch_state.graph().nodes.end(),
        [other_id](const auto& node) { return node.id == other_id; });
    ROAD_TEST_EXPECT(other != branch_state.graph().nodes.end(),
                     "elevated lane connection endpoint is missing");
    if (other->position.x < 50.0)
      west_segment = candidate.id;
    if (other->position.x > 50.0)
      east_segment = candidate.id;
  }
  ROAD_TEST_EXPECT(west_segment != 0 && east_segment != 0,
                   "elevated lane connection fixture did not find base approaches");
  const auto endpoint_role_at = [&](RoadSegmentId id) {
    const auto found = std::find_if(
        branch_state.graph().segments.begin(), branch_state.graph().segments.end(),
        [id](const auto& candidate) { return candidate.id == id; });
    return found->node_a == split_node_id ? EndpointRole::kStart
                                          : EndpointRole::kEnd;
  };
  SavedRoadGraph lane_connection_seed = branch_state.graph();
  lane_connection_seed.lane_connections.push_back(city::road::LaneConnection{
      9601,
      city::road::LaneEndpointKey{west_segment, 1010,
                                  endpoint_role_at(west_segment)},
      city::road::LaneEndpointKey{east_segment, 1010,
                                  endpoint_role_at(east_segment)},
      city::road::LaneConnectionKind::kJunctionMovement});
  const auto lane_archive =
      city::road::persistence::SaveRoad(lane_connection_seed, kFixtureNextId);
  ROAD_TEST_EXPECT(lane_archive.ok, lane_archive.error);
  const auto lane_loaded = RoadState::Load(lane_archive.value);
  ROAD_TEST_EXPECT(lane_loaded.ok, lane_loaded.error);
  ROAD_TEST_EXPECT(!lane_loaded.value.derived().lane_paths.empty(),
                   "elevated connection lane path was not derived");
  bool connection_lane_path_on_grade = false;
  for (const auto& lane_path : lane_loaded.value.derived().lane_paths) {
    for (const Vec3d& point : lane_path.points) {
      if (point.z > 4.5) {
        connection_lane_path_on_grade = true;
        break;
      }
    }
  }
  ROAD_TEST_EXPECT(connection_lane_path_on_grade,
                   "connection lane path remained at world Z zero");

  const auto corridor_id = branch_state.graph().corridors.front().id;
  const auto terminal_ref = branch_state.graph().corridors.front().segments.back();
  const auto terminal_segment = std::find_if(
      branch_state.graph().segments.begin(), branch_state.graph().segments.end(),
      [&terminal_ref](const auto& candidate) {
        return candidate.id == terminal_ref.segment_id;
      });
  ROAD_TEST_EXPECT(terminal_segment != branch_state.graph().segments.end(),
                   "extension fixture terminal segment is missing");
  const RoadNodeId endpoint = terminal_ref.reversed ? terminal_segment->node_a
                                                    : terminal_segment->node_b;
  const auto extension = branch_state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, endpoint,
          MakePath({MakeLine({100.0, 0.0}, {130.0, 0.0})}), branch_layout});
  ROAD_TEST_EXPECT(extension.ok, extension.error);
  const auto extension_node = std::find_if(
      branch_state.graph().nodes.begin(), branch_state.graph().nodes.end(),
      [](const auto& node) {
        return std::abs(node.position.x - 130.0) <= 1e-9 &&
               std::abs(node.position.y) <= 1e-9;
      });
  ROAD_TEST_EXPECT(extension_node != branch_state.graph().nodes.end() &&
                       std::abs(extension_node->elevation_m - 10.0) <= 1e-9,
                   "corridor extension did not keep endpoint elevation");

  const auto saved = branch_state.Save();
  ROAD_TEST_EXPECT(saved.ok && saved.value.find("node.0.elevation_m=") !=
                                  std::string::npos,
                   "node elevation is not saved");
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto saved_again = loaded.value.Save();
  ROAD_TEST_EXPECT(saved_again.ok && saved_again.value == saved.value,
                   "elevated road save/load is not bit stable");

  RoadState crossing{};
  const auto crossing_layout =
      road_fixture::AddLayout(crossing, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest low{};
  low.alignment = MakePath({MakeLine({-20.0, 0.0}, {20.0, 0.0})});
  low.layout_template = crossing_layout;
  low.start_elevation_m = 0.0;
  low.end_elevation_m = 0.0;
  city::road::AddSegmentRequest high{};
  high.alignment = MakePath({MakeLine({0.0, -20.0}, {0.0, 20.0})});
  high.layout_template = crossing_layout;
  high.start_elevation_m = 5.0;
  high.end_elevation_m = 5.0;
  ROAD_TEST_EXPECT(crossing.AddSegment(low).ok, "low road could not be drawn");
  ROAD_TEST_EXPECT(crossing.AddSegment(high).ok, "high road could not be drawn");
  ROAD_TEST_EXPECT(crossing.graph().nodes.size() == 4 &&
                       road_test_view::junctions(crossing.derived()).empty(),
                   "XY crossing with different elevations became a junction");
  return true;
}

bool P0_odd_lane_carriageway_mesh_has_drainage_crown(std::string& failure) {
  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::ExtraLaneLayout(0));
  const auto added = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto sections = road_test_view::sections(state.derived());
  ROAD_TEST_EXPECT(!sections.empty(), "three-lane section was not evaluated");
  const auto center_line = std::find_if(
      sections.front()->boundaries.begin(), sections.front()->boundaries.end(),
      [](const auto& boundary) { return boundary.boundary_id == 200; });
  ROAD_TEST_EXPECT(center_line != sections.front()->boundaries.end(),
                   "three-lane center line boundary is missing");

  const Mesh* asphalt = nullptr;
  for (const Mesh& mesh : state.derived().segment_meshes) {
    if (mesh.style == RenderStyleFromSurface(builtin_surface_styles::kAsphalt)) {
      asphalt = &mesh;
      break;
    }
  }
  ROAD_TEST_EXPECT(asphalt != nullptr, "three-lane asphalt mesh is missing");
  double crown_z = -1.0;
  for (const Vec3d& vertex : asphalt->vertices) {
    if (std::abs(vertex.y) <= 1e-9) {
      crown_z = std::max(crown_z, vertex.z);
    }
  }
  ROAD_TEST_EXPECT(crown_z > center_line->height_m + 0.02,
                   "odd-lane carriageway mesh kept the painted center line as "
                   "the highest road point");
  return true;
}

bool P0_angled_segment_keeps_final_section_perpendicular(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const Vec2d start{0.0, 0.0};
  const Vec2d end{24.0, 12.0};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine(start, end)}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto& vertices = state.derived().segment_meshes.front().vertices;
  const std::size_t row_count = static_cast<std::size_t>(std::ceil(std::hypot(24.0, 12.0) / 2.0)) + 1;
  ROAD_TEST_EXPECT(vertices.size() % row_count == 0, "surface mesh rows are not stable");
  const std::size_t row_width = vertices.size() / row_count;
  const auto& left = vertices[(row_count - 1) * row_width];
  const auto& right = vertices[row_count * row_width - 1];
  const double tangent_length = std::hypot(end.x - start.x, end.y - start.y);
  const double dot = (right.x - left.x) * (end.x - start.x) / tangent_length +
                     (right.y - left.y) * (end.y - start.y) / tangent_length;
  ROAD_TEST_EXPECT(std::abs(dot) < 1e-9, "final cross section is not perpendicular to the road tangent");
  return true;
}

bool P0_rejects_self_intersection_without_mutation(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const std::size_t before_segments = state.graph().segments.size();
  const auto bad = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {10.0, 10.0}), MakeLine({10.0, 10.0}, {0.0, 10.0}),
                MakeLine({0.0, 10.0}, {10.0, 0.0})}),
      1});
  ROAD_TEST_EXPECT(!bad.ok &&
                       bad.failure_category == CommitFailureCategory::kRequirementConstraint &&
                       bad.reason_code == "road_path_self_intersection",
                   "self-intersection was not classified as a documented requirement constraint");
  ROAD_TEST_EXPECT(state.graph().segments.size() == before_segments, "failed preflight mutated authoritative graph");
  return true;
}

bool P0_save_load_is_authoritative_and_bit_stable(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeBezier({0.0, 0.0}, {10.0, 12.0}, {30.0, -12.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  ROAD_TEST_EXPECT(saved.value.starts_with("road_graph_version=16\n") &&
                       saved.value.find("primitive=") == std::string::npos &&
                       saved.value.find("section_template.0.alignment_offset_from_left_m=") !=
                           std::string::npos &&
                       saved.value.find("section_template.0.lane_band.0.direction=") != std::string::npos &&
                       saved.value.find("lane_connection.count=0") != std::string::npos &&
                       saved.value.find("boundary_continuation.count=0") != std::string::npos &&
                       saved.value.find("segment.0.shape.intent=0") != std::string::npos &&
                       saved.value.find("segment.0.shape.start_handle.x=") != std::string::npos &&
                       saved.value.find("corridor.0.segment.0.segment_id=") != std::string::npos,
                   "road save did not use named canonical Bezier shape authority");
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto saved_again = loaded.value.Save();
  ROAD_TEST_EXPECT(saved_again.ok, saved_again.error);
  ROAD_TEST_EXPECT(saved.value == saved_again.value, "road save/load was not bit stable");
  auto archive_with = [&failure](std::string archive, const std::string& key, const std::string& value) {
    const std::string prefix = key + "=";
    const auto pos = archive.find(prefix);
    if (pos == std::string::npos) {
      failure = "test archive key is missing: " + key;
      return archive;
    }
    const auto end = archive.find('\n', pos);
    archive.replace(pos, end == std::string::npos ? std::string::npos : end - pos, prefix + value);
    return archive;
  };
  std::string version4 = saved.value;
  version4.replace(0, std::string("road_graph_version=16").size(), "road_graph_version=4");
  const auto rejected = RoadState::Load(version4);
  ROAD_TEST_EXPECT(!rejected.ok && rejected.failure_category == CommitFailureCategory::kInvalidInput,
                   "legacy road archive was not rejected");
  std::string truncated = saved.value;
  const std::size_t shape_row = truncated.find("segment.0.shape.start_handle.x=");
  ROAD_TEST_EXPECT(shape_row != std::string::npos, "road archive has no segment shape field");
  truncated.erase(shape_row, truncated.find('\n', shape_row) - shape_row + 1);
  ROAD_TEST_EXPECT(!RoadState::Load(truncated).ok, "truncated road archive was accepted");
  std::string duplicate = saved.value;
  const std::size_t node_row = duplicate.find("node.0.id=");
  const std::size_t node_end = duplicate.find('\n', node_row);
  duplicate.append(duplicate.substr(node_row, node_end - node_row + 1));
  ROAD_TEST_EXPECT(!RoadState::Load(duplicate).ok, "duplicate road archive key was accepted");
  std::string unknown = saved.value;
  unknown += "unknown.field=1\n";
  ROAD_TEST_EXPECT(!RoadState::Load(unknown).ok, "unknown road archive key was accepted");
  ROAD_TEST_EXPECT(!RoadState::Load(archive_with(saved.value, "node.0.position.x", "nan")).ok,
                   "non-finite road archive double was accepted");
  ROAD_TEST_EXPECT(!RoadState::Load(archive_with(saved.value, "section_template.0.strip.0.style_id", "999")).ok,
                   "unknown road archive surface style was accepted");
  ROAD_TEST_EXPECT(!RoadState::Load(archive_with(saved.value, "road_graph_version", "17")).ok,
                   "future road archive version was accepted");
  ROAD_TEST_EXPECT(failure.empty(), failure);
  for (int old_version = 1; old_version <= 13; ++old_version) {
    std::string legacy = saved.value;
    legacy.replace(0, std::string("road_graph_version=16").size(),
                   "road_graph_version=" + std::to_string(old_version));
    ROAD_TEST_EXPECT(!RoadState::Load(legacy).ok, "legacy road archive version was accepted");
  }
  return true;
}

bool lane_topology_validation_and_round_trip(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const RoadCorridorId corridor = state.graph().corridors.front().id;
  const RoadNodeId endpoint = state.graph().segments.front().node_b;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor, endpoint,
          MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(second.ok, second.error);

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  std::uint64_t maximum_id = 0;
  for (const auto& node : state.graph().nodes) maximum_id = std::max(maximum_id, node.id);
  for (const auto& segment : state.graph().segments) maximum_id = std::max(maximum_id, segment.id);
  for (const auto& road_corridor : state.graph().corridors) {
    maximum_id = std::max(maximum_id, road_corridor.id);
  }
  for (const auto& section : state.graph().layout_templates) {
    maximum_id = std::max(maximum_id, section.id);
  }

  auto replace_value = [&failure](std::string* archive, const std::string& key,
                                  const std::string& value) {
    const std::string prefix = key + "=";
    const std::size_t begin = archive->find(prefix);
    if (begin == std::string::npos) {
      failure = "test archive key is missing: " + key;
      return;
    }
    const std::size_t end = archive->find('\n', begin);
    archive->replace(begin, end - begin, prefix + value);
  };

  std::string with_topology = saved.value;
  replace_value(&with_topology, "next_id", std::to_string(maximum_id + 3));
  replace_value(&with_topology, "lane_connection.count", "1");
  replace_value(&with_topology, "boundary_continuation.count", "1");
  ROAD_TEST_EXPECT(failure.empty(), failure);
  with_topology +=
      "lane_connection.0.id=" + std::to_string(maximum_id + 1) + "\n" +
      "lane_connection.0.source.segment_id=" + std::to_string(first.value) + "\n" +
      "lane_connection.0.source.lane_id=1010\n"
      "lane_connection.0.source.endpoint_role=1\n" +
      "lane_connection.0.target.segment_id=" + std::to_string(second.value) + "\n" +
      "lane_connection.0.target.lane_id=1010\n"
      "lane_connection.0.target.endpoint_role=0\n"
      "lane_connection.0.kind=0\n" +
      "boundary_continuation.0.id=" + std::to_string(maximum_id + 2) + "\n" +
      "boundary_continuation.0.source.segment_id=" + std::to_string(first.value) + "\n" +
      "boundary_continuation.0.source.boundary_id=200\n"
      "boundary_continuation.0.source.endpoint_role=1\n" +
      "boundary_continuation.0.target.segment_id=" + std::to_string(second.value) + "\n" +
      "boundary_continuation.0.target.boundary_id=200\n"
      "boundary_continuation.0.target.endpoint_role=0\n"
      "boundary_continuation.0.kind=0\n";

  const auto loaded = RoadState::Load(with_topology);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  ROAD_TEST_EXPECT(loaded.value.graph().lane_connections.size() == 1,
                   "lane connection was not restored");
  ROAD_TEST_EXPECT(loaded.value.graph().boundary_continuations.size() == 1,
                   "boundary continuation was not restored");
  const auto& lane = loaded.value.graph().lane_connections.front();
  ROAD_TEST_EXPECT(lane.source.segment_id == first.value &&
                       lane.source.lane_id == 1010 &&
                       lane.target.segment_id == second.value &&
                       lane.target.lane_id == 1010 &&
                       lane.kind == city::road::LaneConnectionKind::kContinuation,
                   "lane connection identity changed during load");
  const auto& boundary = loaded.value.graph().boundary_continuations.front();
  ROAD_TEST_EXPECT(boundary.source.boundary_id == 200 &&
                       boundary.target.boundary_id == 200,
                   "boundary continuation identity changed during load");
  const auto canonical = loaded.value.Save();
  ROAD_TEST_EXPECT(canonical.ok, canonical.error);
  const auto loaded_again = RoadState::Load(canonical.value);
  ROAD_TEST_EXPECT(loaded_again.ok, loaded_again.error);
  const auto canonical_again = loaded_again.value.Save();
  ROAD_TEST_EXPECT(canonical_again.ok, canonical_again.error);
  ROAD_TEST_EXPECT(canonical.value == canonical_again.value,
                   "lane topology archive was not canonical");

  std::string missing_lane = canonical.value;
  replace_value(&missing_lane, "lane_connection.0.target.lane_id", "999999");
  ROAD_TEST_EXPECT(!RoadState::Load(missing_lane).ok,
                   "lane connection accepted a missing target lane");

  std::string wrong_direction = canonical.value;
  replace_value(&wrong_direction,
                "lane_connection.0.source.endpoint_role", "0");
  ROAD_TEST_EXPECT(!RoadState::Load(wrong_direction).ok,
                   "lane connection accepted a non-exit source endpoint");

  std::string missing_boundary = canonical.value;
  replace_value(&missing_boundary,
                "boundary_continuation.0.target.boundary_id", "999999");
  ROAD_TEST_EXPECT(!RoadState::Load(missing_boundary).ok,
                   "boundary continuation accepted a missing target boundary");

  std::string duplicate_connection = canonical.value;
  replace_value(&duplicate_connection, "next_id",
                std::to_string(maximum_id + 4));
  replace_value(&duplicate_connection, "lane_connection.count", "2");
  duplicate_connection +=
      "lane_connection.1.id=" + std::to_string(maximum_id + 3) + "\n" +
      "lane_connection.1.source.segment_id=" + std::to_string(first.value) + "\n" +
      "lane_connection.1.source.lane_id=1010\n"
      "lane_connection.1.source.endpoint_role=1\n" +
      "lane_connection.1.target.segment_id=" + std::to_string(second.value) + "\n" +
      "lane_connection.1.target.lane_id=1010\n"
      "lane_connection.1.target.endpoint_role=0\n"
      "lane_connection.1.kind=0\n";
  ROAD_TEST_EXPECT(!RoadState::Load(duplicate_connection).ok,
                   "duplicate lane connection was accepted");

  std::string overlapping_lanes = saved.value;
  replace_value(&overlapping_lanes,
                "section_template.0.lane_band.1.surface_strip_id", "20");
  ROAD_TEST_EXPECT(!RoadState::Load(overlapping_lanes).ok,
                   "overlapping lane allocations were accepted");
  return true;
}

bool P0_tool_preview_matches_the_committed_interval(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest first{};
  first.alignment = MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})});
  first.layout_template = section;
  first.intent = city::road::SegmentShapeIntent::kCurve;
  const auto added = state.AddSegment(first);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), added.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "preview corridor is missing");
  const auto segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&added](const auto& item) { return item.id == added.value; });
  ROAD_TEST_EXPECT(segment != state.graph().segments.end(), "preview segment is missing");

  const Vec2d start{40.0, 0.0};
  const Vec2d end{70.0, 22.0};
  const Path preview = city::road::PreviewDrawnInterval(
      state.graph(), corridor->id, segment->node_b, start, end,
      city::road::SegmentShapeIntent::kCurve);
  ROAD_TEST_EXPECT(preview.spans.size() == 1, "preview did not produce one interval");
  ROAD_TEST_EXPECT(!IsLinearSpan(preview.spans.front()),
                   "curve preview stayed straight");

  city::road::ExtendCorridorFromEndRequest extension{};
  extension.corridor_id = corridor->id;
  extension.endpoint_node_id = segment->node_b;
  extension.extension = preview;
  extension.layout_template = section;
  extension.intent = city::road::SegmentShapeIntent::kCurve;
  const auto extended = state.ExtendCorridorFromEnd(extension);
  ROAD_TEST_EXPECT(extended.ok, extended.error);
  const Path* committed = FindCanonicalAlignment(state.derived(), extended.value);
  ROAD_TEST_EXPECT(committed != nullptr && committed->spans.size() == 1,
                   "committed interval is missing");
  const auto& a = preview.spans.front();
  const auto& b = committed->spans.front();
  ROAD_TEST_EXPECT(std::abs(a.p1.x - b.p1.x) < 1e-9 && std::abs(a.p1.y - b.p1.y) < 1e-9 &&
                       std::abs(a.p2.x - b.p2.x) < 1e-9 && std::abs(a.p2.y - b.p2.y) < 1e-9,
                   "preview and committed interval disagree");
  return true;
}

bool P0_straight_segments_stay_linear_after_snap_and_move(std::string& failure) {
  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadSegmentId source = base.value;
    const RoadNodeId endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({20.2, 0.0}, {20.0, 20.0})}), section, endpoint});
    ROAD_TEST_EXPECT(connected.ok, connected.error);
    const Path* source_alignment = FindCanonicalAlignment(state.derived(), source);
    const Path* connected_alignment = FindCanonicalAlignment(state.derived(), connected.value);
    ROAD_TEST_EXPECT(source_alignment != nullptr && connected_alignment != nullptr,
                     "connected straight alignment is missing");
    ROAD_TEST_EXPECT(IsLinearSpan(source_alignment->spans.front()),
                     "connecting a straight segment changed the previous segment");
    ROAD_TEST_EXPECT(IsLinearSpan(connected_alignment->spans.front()),
                     "snapped straight segment is no longer a linear Bezier");
  }

  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto first = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
    ROAD_TEST_EXPECT(first.ok, first.error);
    const RoadSegment& segment = state.graph().segments.front();
    const RoadNodeId endpoint = segment.node_b;
    const RoadCorridorId corridor = state.graph().corridors.front().id;
    const auto extended = state.ExtendCorridorFromEnd(
        city::road::ExtendCorridorFromEndRequest{
            corridor, endpoint,
            MakePath({MakeLine({20.2, 0.0}, {20.0, 20.0})}), section});
    ROAD_TEST_EXPECT(extended.ok, extended.error);
    const Path* extension_alignment = FindCanonicalAlignment(state.derived(), extended.value);
    ROAD_TEST_EXPECT(extension_alignment != nullptr,
                     "extended straight alignment is missing");
    ROAD_TEST_EXPECT(IsLinearSpan(extension_alignment->spans.front()),
                     "extended straight segment is no longer a linear Bezier");
  }

  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto added = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {30.0, 0.0})}), section});
    ROAD_TEST_EXPECT(added.ok, added.error);
    const auto saved = state.Save();
    ROAD_TEST_EXPECT(saved.ok, saved.error);
    ROAD_TEST_EXPECT(saved.value.find("segment.0.shape.intent=1") != std::string::npos,
                     "straight segment intent was not saved");
    const RoadNodeId end_node = state.graph().segments.front().node_b;
    const auto moved = state.MoveNode(city::road::MoveNodeRequest{end_node, {30.0, 10.0}});
    ROAD_TEST_EXPECT(moved.ok, moved.error);
    const Path* alignment = FindCanonicalAlignment(state.derived(), added.value);
    ROAD_TEST_EXPECT(alignment != nullptr, "moved straight alignment is missing");
    ROAD_TEST_EXPECT(IsLinearSpan(alignment->spans.front()),
                     "moving a straight segment endpoint did not rederive linear Bezier controls");
  }
  return true;
}

bool P0_edit_and_delete_preserve_graph_ownership(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto isolated = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
  ROAD_TEST_EXPECT(isolated.ok, isolated.error);
  const auto moved = state.MoveNode(city::road::MoveNodeRequest{state.graph().segments.front().node_a, {2.0, 3.0}});
  ROAD_TEST_EXPECT(moved.ok, moved.error);
  ROAD_TEST_EXPECT(std::abs(state.graph().nodes[0].position.x - 2.0) < 1e-9,
                   "MoveNode did not move endpoint authority");

  const auto shared_node = state.graph().segments.front().node_a;
  const auto branch = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({2.0, 3.0}, {2.0, 23.0})}), section, shared_node});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto shape = SegmentShapeFromPath(MakePath({MakeBezier({2.0, 3.0}, {8.0, 8.0}, {16.0, -2.0}, {20.0, 0.0})}));
  ROAD_TEST_EXPECT(shape.ok, shape.error);
  const auto edited = state.EditSegmentShape(city::road::EditSegmentShapeRequest{isolated.value, shape.value});
  ROAD_TEST_EXPECT(edited.ok, edited.error);
  ROAD_TEST_EXPECT(state.graph().nodes[0].position.x == 2.0 && state.graph().nodes[0].position.y == 3.0,
                   "shape edit changed shared endpoint authority");

  const auto deleted = state.DeleteSegment(city::road::DeleteSegmentRequest{branch.value});
  ROAD_TEST_EXPECT(deleted.ok, deleted.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1, "road delete removed the wrong segment");
  ROAD_TEST_EXPECT(state.graph().nodes.size() == 2, "road delete left an orphan endpoint node");
  ROAD_TEST_EXPECT(state.graph().connection_policy_overrides.empty(), "road delete left a connection policy override");
  return true;
}

bool P0_multispan_segment_is_one_user_deletion_unit(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const Path input = MakePath({
      MakeBezier({0.0, 0.0}, {5.0, 0.0}, {10.0, 5.0}, {15.0, 5.0}),
      MakeBezier({15.0, 5.0}, {20.0, 5.0}, {25.0, 0.0}, {30.0, 0.0}),
  });
  const auto added =
      state.AddSegment(city::road::AddSegmentRequest{input, section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(
      state.graph().segments.size() == 1 &&
          state.graph().corridors.size() == 1 &&
          state.graph().corridors.front().segments.size() == 1,
      "one confirmed multi-span path was split into multiple RoadSegments");
  const Path* alignment = FindCanonicalAlignment(state.derived(), added.value);
  ROAD_TEST_EXPECT(alignment != nullptr && alignment->spans.size() == 2,
                   "multi-span RoadSegment did not preserve both Bezier spans");

  const auto deleted =
      state.DeleteSegment(city::road::DeleteSegmentRequest{added.value});
  ROAD_TEST_EXPECT(deleted.ok, deleted.error);
  ROAD_TEST_EXPECT(state.graph().segments.empty() &&
                       state.graph().corridors.empty() &&
                       state.graph().nodes.empty(),
                   "deleting a multi-span RoadSegment left part of the user unit");
  return true;
}

bool P1_degree_two_corner_uses_a_curve_without_a_junction(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_a;
  const auto branch = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({0.0, 0.0}, {0.0, 30.0})}), section, node});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().connection_policy_overrides.empty(), "degree-two connection saved an automatic decision");
  ROAD_TEST_EXPECT(road_test_view::junctions(state.derived()).empty(), "degree-two connection derived a JunctionArea");
  ROAD_TEST_EXPECT(0 == road_test_view::count_marking_areas(state.derived(), [](const auto& area) {
                                  return area.owner.kind == city::road::MarkingOwner::Kind::kJunction;
                                }),
                   "degree-two connection derived stop or crosswalk markings");
  ROAD_TEST_EXPECT(road_test_view::corners(state.derived()).size() == 1,
                   "degree-two corner did not derive a separate connection area");
  const auto& corner = *road_test_view::corners(state.derived()).front();
  ROAD_TEST_EXPECT(corner.corner_control_m > 0.0,
                   "degree-two corner did not derive a curve control");
  std::set<RenderStyleRef> connection_styles{};
  bool has_curved_vertex = false;
  for (const auto& mesh : state.derived().connection_meshes) {
    connection_styles.insert(mesh.style);
    has_curved_vertex = has_curved_vertex || std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [](const auto& p) {
      return p.x > 0.1 && p.y > 0.1;
    });
  }
  ROAD_TEST_EXPECT(connection_styles.contains(RenderStyleFromSurface(builtin_surface_styles::kAsphalt)) &&
                       connection_styles.contains(RenderStyleFromSurface(builtin_surface_styles::kSidewalk)) &&
                       connection_styles.contains(RenderStyleFromSurface(builtin_surface_styles::kCurb)),
                   "degree-two corner does not preserve the road section styles");
  ROAD_TEST_EXPECT(has_curved_vertex, "degree-two corner connector is not curved between its gates");
  return true;
}

bool P1_common_skew_angles_are_resolved_from_geometry(std::string& failure) {
  for (const double degrees : {30.0, 45.0, 60.0}) {
    RoadState corner_state{};
    const auto corner_state_section = road_fixture::AddLayout(corner_state, road_fixture::BidirectionalLayout(0));
    const auto base = corner_state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), corner_state_section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto node = corner_state.graph().segments.front().node_a;
    const double radians = degrees * std::numbers::pi / 180.0;
    const auto branch = corner_state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({0.0, 0.0},
                               {40.0 * std::cos(radians),
                                40.0 * std::sin(radians)})}),
            1, node});
    ROAD_TEST_EXPECT(branch.ok,
                     "degree-two skew angle was rejected: " + branch.error);
    ROAD_TEST_EXPECT(road_test_view::corners(corner_state.derived()).size() == 1,
                     "degree-two skew angle did not produce one corner");
    ROAD_TEST_EXPECT(ValidateGraphInvariants(corner_state.graph(),
                                             corner_state.derived()).ok,
                     "degree-two skew angle violated derived invariants");
  }

  RoadState junction_state{};
  const auto junction_state_section = road_fixture::AddLayout(junction_state, road_fixture::BidirectionalLayout(0));
  const auto base = junction_state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), junction_state_section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const double radians = 30.0 * std::numbers::pi / 180.0;
  const auto branch = junction_state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0},
                             {40.0 * std::cos(radians),
                              40.0 * std::sin(radians)})}),
          junction_state_section, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, "skew T junction was rejected: " + branch.error);
  ROAD_TEST_EXPECT(road_test_view::junctions(junction_state.derived()).size() == 1,
                   "skew T junction did not produce one junction");
  const auto junction_node =
      road_test_view::junctions(junction_state.derived()).front()->node_id;
  const auto opposite = junction_state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0},
                             {-40.0 * std::cos(radians),
                              -40.0 * std::sin(radians)})}),
          1, junction_node});
  ROAD_TEST_EXPECT(opposite.ok,
                   "skew cross junction was rejected: " + opposite.error);
  ROAD_TEST_EXPECT(
      road_test_view::gates_of(
          *road_test_view::junctions(junction_state.derived()).front())
              .size() == 4,
      "skew cross junction does not have four approaches");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(junction_state.graph(),
                                           junction_state.derived()).ok,
                   "skew cross junction violated derived invariants");
  return true;
}

bool P1_corner_preserves_endpoint_section_sides(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto branch = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({40.0, 0.0}, {40.0, 30.0})}), section, node});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto corners = road_test_view::corners(state.derived());
  ROAD_TEST_EXPECT(corners.size() == 1, "end-start corner did not derive one connection");
  const auto& corner = *corners.front();
  const auto base_gate = std::find_if(
      corner.approaches.begin(), corner.approaches.end(), [base](const auto& approach) {
        return approach.key.segment_id == base.value &&
               approach.key.endpoint_role == EndpointRole::kEnd;
      });
  ROAD_TEST_EXPECT(base_gate != corner.approaches.end(),
                   "corner source end approach is missing");
  const auto curb = std::find_if(
      corner.connection_geometry.boundary_curves.begin(),
      corner.connection_geometry.boundary_curves.end(), [](const auto& curve) {
        return curve.source_boundary_id == 100 && curve.target_boundary_id == 100 &&
               !curve.points.empty();
      });
  ROAD_TEST_EXPECT(curb != corner.connection_geometry.boundary_curves.end(),
                   "corner left curb boundary curve is missing");
  const bool base_is_source =
      corner.connection_geometry.approaches[0] == base_gate->key;
  const auto base_point = base_is_source ? curb->points.front() : curb->points.back();
  ROAD_TEST_EXPECT(base_point.y < base_gate->gate.position.y,
                   "segment end approach mirrored the left curb to the opposite side");
  for (const auto& strip : corner.connection_geometry.surface_strips) {
    ROAD_TEST_EXPECT(strip.left.size() == strip.right.size() &&
                         strip.left.size() >= 2,
                     "corner strip boundaries are incomplete");
    const double expected_width =
        std::hypot(strip.left.front().x - strip.right.front().x,
                   strip.left.front().y - strip.right.front().y);
    for (std::size_t index = 1; index < strip.left.size(); ++index) {
      const double width =
          std::hypot(strip.left[index].x - strip.right[index].x,
                     strip.left[index].y - strip.right[index].y);
      ROAD_TEST_EXPECT(std::abs(width - expected_width) <= 1e-6,
                       "corner section width changed along its curve");
    }
  }
  ROAD_TEST_EXPECT(
      std::any_of(
          state.derived().markings.begin(), state.derived().markings.end(),
          [node](const auto& marking) {
            return marking.owner.kind ==
                       city::road::MarkingOwner::Kind::kJunction &&
                   marking.owner.node_id == node &&
                   marking.role == city::road::MarkingRole::kCenterLine &&
                   marking.points.size() >= 2;
          }),
      "corner center line does not continue through the connection");
  for (const auto& mesh : state.derived().connection_meshes) {
    ROAD_TEST_EXPECT(mesh_faces_up(mesh),
                     "corner connection mesh has downward-facing triangles");
  }
  return true;
}

bool P1_straight_connection_has_no_junction_area(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto continued = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), section, node});
  ROAD_TEST_EXPECT(continued.ok, continued.error);
  ROAD_TEST_EXPECT(state.graph().connection_policy_overrides.empty(), "straight connection saved an automatic decision");
  ROAD_TEST_EXPECT(road_test_view::corners(state.derived()).empty() && road_test_view::junctions(state.derived()).empty(),
                   "straight connection derived an area with artificial width");
  ROAD_TEST_EXPECT(0 == road_test_view::count_marking_areas(state.derived(), [](const auto& area) {
                                  return area.owner.kind == city::road::MarkingOwner::Kind::kJunction;
                                }),
                   "straight connection derived stop or crosswalk markings");
  return true;
}

bool P1_short_connections_use_required_setback(std::string& failure) {
  {
    RoadState state{};
    const auto section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadNodeId endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({20.0, 0.0}, {24.0, 0.0})}), section,
            endpoint});
    ROAD_TEST_EXPECT(
        connected.ok,
        "a 4m aligned connection with zero required setback was rejected: " +
            connected.error);
    const auto* derived = FindDerivedSegment(state.derived(), connected.value);
    ROAD_TEST_EXPECT(derived != nullptr &&
                         std::abs(derived->surface_end_m -
                                  derived->surface_start_m - 4.0) < 1e-6,
                     "the short aligned connection lost usable road surface");
  }

  {
    RoadState state{};
    const auto section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadNodeId endpoint = state.graph().segments.front().node_b;
    const auto before = state.Save();
    ROAD_TEST_EXPECT(before.ok, before.error);
    const auto connected = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({20.0, 0.0}, {20.0, 2.0})}), section,
            endpoint});
    ROAD_TEST_EXPECT(!connected.ok &&
                         connected.failure_category ==
                             CommitFailureCategory::kNotImplemented &&
                         connected.error.find("setback exceeds") !=
                             std::string::npos,
                     "an unfit short corner was not rejected by its required setback");
    const auto after = state.Save();
    ROAD_TEST_EXPECT(after.ok && before.value == after.value,
                     "a rejected short corner changed authoritative state");
  }
  return true;
}

bool P1_short_junctions_shrink_auto_gate_setbacks(std::string& failure) {
  {
    RoadState state{};
    const auto section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-20.0, 0.0}, {20.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto branch = state.AddSegmentConnectedToSegment(
        city::road::AddSegmentConnectedToSegmentRequest{
            MakePath({MakeLine({0.0, 0.0}, {0.0, 6.0})}), section,
            base.value, 20.0});
    ROAD_TEST_EXPECT(
        branch.ok,
        "a 6m T branch rejected a corner that fits by shrinking radius: " +
            branch.error);
    const auto junctions = road_test_view::junctions(state.derived());
    ROAD_TEST_EXPECT(junctions.size() == 1,
                     "short T branch did not form one junction");
    const auto branch_approach = std::find_if(
        junctions.front()->approaches.begin(), junctions.front()->approaches.end(),
        [id = branch.value](const auto& approach) {
          return approach.key.segment_id == id;
        });
    ROAD_TEST_EXPECT(branch_approach != junctions.front()->approaches.end(),
                     "short T branch approach is missing");
    ROAD_TEST_EXPECT(
        branch_approach->resolved_setback_m <= 6.0 + 1e-6,
        "short T branch gate remained beyond the segment length");
  }

  {
    RoadState state{};
    const auto section =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto lower = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-20.0, 0.0}, {20.0, 0.0})}), section});
    const auto upper = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-20.0, 10.0}, {20.0, 10.0})}), section});
    ROAD_TEST_EXPECT(lower.ok && upper.ok,
                     "short two-junction fixture could not be created");
    const auto connected = state.AddSegmentBetween(
        city::road::AddSegmentBetweenRequest{
            MakePath({MakeLine({0.0, 0.0}, {0.0, 10.0})}), section,
            city::road::RoadConnectionTarget{0, lower.value, 20.0},
            city::road::RoadConnectionTarget{0, upper.value, 20.0}});
    ROAD_TEST_EXPECT(
        connected.ok,
        "a short two-ended connection rejected overlapping auto gates: " +
            connected.error);
    const auto* derived = FindDerivedSegment(state.derived(), connected.value);
    ROAD_TEST_EXPECT(derived != nullptr, "short connector was not derived");
    ROAD_TEST_EXPECT(derived->surface_end_m + 1e-6 >= derived->surface_start_m,
                     "short connector gates still overlap after fitting");
  }
  return true;
}

bool P1_segment_snap_splits_straight_road_for_t_junction(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{MakePath({MakeLine({20.0, 0.0}, {32.0, 24.0})}), section,
                                                         base.value, 20.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3, "T junction did not split base road and add branch");
  ROAD_TEST_EXPECT(state.graph().nodes.size() == 4, "T junction did not create one shared middle node");
  ROAD_TEST_EXPECT(state.graph().connection_policy_overrides.empty(), "T junction saved an automatic decision");
  ROAD_TEST_EXPECT(road_test_view::junctions(state.derived()).size() == 1, "T junction did not derive one JunctionArea");
  const auto& junction = *road_test_view::junctions(state.derived()).front();
  ROAD_TEST_EXPECT(junction.junction_corners.size() == 3,
                   "T junction did not resolve one radius per adjacent side pair");
  ROAD_TEST_EXPECT(road_test_view::gates_of(junction).size() == 3, "T junction does not have three gates");
  ROAD_TEST_EXPECT(state.derived().junction_meshes.size() >= 3,
                   "T junction did not derive material-separated junction surface meshes");
  ROAD_TEST_EXPECT(!state.derived().junction_meshes.front().indices.empty(),
                   "T junction surface mesh has no triangles");
  const auto junction_area_count = road_test_view::count_marking_areas(state.derived(), [](const auto& area) {
        return area.owner.kind == city::road::MarkingOwner::Kind::kJunction;
      });
  ROAD_TEST_EXPECT(junction_area_count >= 6,
                   "T junction did not resolve stop line and crosswalk markings for every approach");
  std::set<RenderStyleRef> junction_styles{};
  for (const auto& mesh : state.derived().junction_meshes) junction_styles.insert(mesh.style);
  ROAD_TEST_EXPECT(junction_styles.contains(RenderStyleFromSurface(builtin_surface_styles::kAsphalt)) &&
                       junction_styles.contains(RenderStyleFromSurface(builtin_surface_styles::kSidewalk)) &&
                       junction_styles.contains(RenderStyleFromSurface(builtin_surface_styles::kCurb)),
                   "T junction does not connect carriageway, sidewalks, and curbs by style authority");
  for (const auto& strip : junction.junction_geometry.surface_strips) {
    ROAD_TEST_EXPECT(strip.left.size() == strip.right.size(),
                     "T junction strip boundaries have different sample counts");
    for (std::size_t index = 0; index < strip.left.size(); ++index) {
      const double width = std::hypot(strip.left[index].x - strip.right[index].x,
                                      strip.left[index].y - strip.right[index].y);
      ROAD_TEST_EXPECT(width <= 2.2,
                       "T junction connected a strip to a non-adjacent boundary");
    }
  }
  for (const auto& mesh : state.derived().junction_meshes) {
    ROAD_TEST_EXPECT(mesh_faces_up(mesh),
                     "T junction mesh has downward-facing triangles");
  }
  for (const auto& gate : road_test_view::gates_of(junction)) {
    ROAD_TEST_EXPECT(std::hypot(gate.position.x - 20.0, gate.position.y) >= 5.2 - 1e-6,
                     "T junction gate setback did not adapt to the approach width");
  }
  const auto& gate = road_test_view::gates_of(junction).front();
  const auto stop_line = std::find_if(
      state.derived().markings.begin(), state.derived().markings.end(),
      [](const auto& marking) {
        return marking.owner.kind == MarkingOwner::Kind::kJunction &&
               marking.role == MarkingRole::kStopLine;
      });
  ROAD_TEST_EXPECT(stop_line != state.derived().markings.end(),
                   "T junction has no stop line");
  ROAD_TEST_EXPECT(stop_line->points.size() >= 3,
                   "stop line does not sample the carriageway cross slope");
  const double stop_end_height =
      std::max(stop_line->points.front().z, stop_line->points.back().z);
  ROAD_TEST_EXPECT(
      std::any_of(stop_line->points.begin() + 1, stop_line->points.end() - 1,
                  [stop_end_height](const Vec3d& point) {
                    return point.z > stop_end_height + 1e-6;
                  }),
      "stop line passes through the crowned road surface");
  const auto zebra_areas = road_test_view::marking_areas(state.derived());
  for (const auto& mesh : state.derived().marking_meshes) {
    if (mesh.style ==
        RenderStyleFromMarking(builtin_marking_styles::kCrosswalk)) {
      ROAD_TEST_EXPECT(mesh_faces_up(mesh),
                       "T junction crosswalk mesh faces downward");
    }
  }
  for (const auto& approach_gate : road_test_view::gates_of(junction)) {
    const bool has_crosswalk = std::any_of(
        zebra_areas.begin(), zebra_areas.end(),
        [&approach_gate](const auto* area) {
          if (area->owner.kind !=
                  city::road::MarkingOwner::Kind::kJunction ||
              area->role != MarkingRole::kCrosswalk ||
              area->polygon.empty()) {
            return false;
          }
          Vec2d center{};
          for (const auto& point : area->polygon) {
            center.x += point.x;
            center.y += point.y;
          }
          center.x /= static_cast<double>(area->polygon.size());
          center.y /= static_cast<double>(area->polygon.size());
          const double expected_x =
              approach_gate.position.x + approach_gate.tangent.x * 2.0;
          const double expected_y =
              approach_gate.position.y + approach_gate.tangent.y * 2.0;
          return std::hypot(center.x - expected_x, center.y - expected_y) <
                 0.5;
        });
    ROAD_TEST_EXPECT(has_crosswalk,
                     "T junction approach has no crosswalk geometry");
    const bool has_crosswalk_mesh = std::any_of(
        state.derived().marking_meshes.begin(),
        state.derived().marking_meshes.end(),
        [&approach_gate](const auto& mesh) {
          if (mesh.style != RenderStyleFromMarking(
                                builtin_marking_styles::kCrosswalk) ||
              mesh.vertices.empty()) {
            return false;
          }
          Vec2d center{};
          for (const auto& point : mesh.vertices) {
            center.x += point.x;
            center.y += point.y;
          }
          center.x /= static_cast<double>(mesh.vertices.size());
          center.y /= static_cast<double>(mesh.vertices.size());
          const double expected_x =
              approach_gate.position.x + approach_gate.tangent.x * 2.0;
          const double expected_y =
              approach_gate.position.y + approach_gate.tangent.y * 2.0;
          return std::hypot(center.x - expected_x, center.y - expected_y) <
                 0.5;
        });
    ROAD_TEST_EXPECT(has_crosswalk_mesh,
                     "T junction approach crosswalk was not emitted");
  }
  const auto zebra_it = std::find_if(
      zebra_areas.begin(), zebra_areas.end(), [](const auto* area) {
        return area->owner.kind == city::road::MarkingOwner::Kind::kJunction &&
               area->role == MarkingRole::kCrosswalk && !area->polygon.empty();
      });
  ROAD_TEST_EXPECT(zebra_it != zebra_areas.end(),
                   "T junction crosswalk has no resolved geometry");
  const auto& zebra = (*zebra_it)->polygon;
  ROAD_TEST_EXPECT(zebra.size() >= 4, "T junction crosswalk has no stripe geometry");
  const Vec2d tangent{gate.tangent.x, gate.tangent.y};
  const Vec2d lateral{-tangent.y, tangent.x};
  const Vec2d stripe_edge_a{zebra[1].x - zebra[0].x,
                            zebra[1].y - zebra[0].y};
  const Vec2d stripe_edge_b{zebra[2].x - zebra[0].x,
                            zebra[2].y - zebra[0].y};
  const double tangent_extent = std::max(std::abs(stripe_edge_a.x * tangent.x + stripe_edge_a.y * tangent.y),
                                         std::abs(stripe_edge_b.x * tangent.x + stripe_edge_b.y * tangent.y));
  const double lateral_extent = std::max(std::abs(stripe_edge_a.x * lateral.x + stripe_edge_a.y * lateral.y),
                                         std::abs(stripe_edge_b.x * lateral.x + stripe_edge_b.y * lateral.y));
  ROAD_TEST_EXPECT(tangent_extent > lateral_extent, "T junction zebra stripes are rotated by 90 degrees");
  const auto [min_z, max_z] = std::minmax_element(
      zebra.begin(), zebra.end(), [](const auto& a, const auto& b) { return a.z < b.z; });
  // 0.35 wide at the carriageway's 2%.
  ROAD_TEST_EXPECT(std::abs((max_z->z - min_z->z) - 0.35 * 0.02) < 1e-6,
                   "T junction zebra does not follow the road cross slope");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok, "T junction invariants failed");
  return true;
}

bool P1_junction_setback_uses_the_facing_section_side(std::string& failure) {
  RoadState state{};
  auto asymmetric = road_fixture::BidirectionalLayout(0);
  asymmetric.alignment_offset_from_left_m = 2.0;
  const auto section = road_fixture::AddLayout(state, asymmetric);
  const auto branch_section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), branch_section,
          base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);

  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1,
                   "asymmetric T did not produce one junction");
  const auto approach = std::find_if(
      junctions.front()->approaches.begin(),
      junctions.front()->approaches.end(),
      [id = branch.value](const auto& candidate) {
        return candidate.key.segment_id == id;
      });
  ROAD_TEST_EXPECT(approach != junctions.front()->approaches.end(),
                   "asymmetric T branch approach is missing");
  // The horizontal approaches present their 2m side to this branch. At 90
  // degrees the required clearance is 2/sin(90) + 4/tan(45) = 6m.
  ROAD_TEST_EXPECT(
      std::abs(approach->resolved_setback_m - 6.0) <= 1e-6,
      "junction used the far section side; resolved setback=" +
          std::to_string(approach->resolved_setback_m));
  ROAD_TEST_EXPECT(
      std::abs(std::hypot(approach->gate.position.x,
                          approach->gate.position.y) -
               approach->resolved_setback_m) <= 1e-6,
      "junction gate does not match its resolved setback");

  const auto zebra = road_test_view::find_marking_area(
      state.derived(), [&approach](const auto& marking) {
        if (marking.owner.kind != MarkingOwner::Kind::kJunction ||
            marking.role != MarkingRole::kCrosswalk ||
            marking.polygon.empty()) {
          return false;
        }
        Vec2d center{};
        for (const Vec3d& point : marking.polygon) {
          center.x += point.x;
          center.y += point.y;
        }
        center.x /= static_cast<double>(marking.polygon.size());
        center.y /= static_cast<double>(marking.polygon.size());
        const double expected_x =
            approach->gate.position.x + approach->gate.tangent.x * 2.0;
        const double expected_y =
            approach->gate.position.y + approach->gate.tangent.y * 2.0;
        return std::hypot(center.x - expected_x, center.y - expected_y) < 0.5;
      });
  ROAD_TEST_EXPECT(zebra != nullptr,
                   "asymmetric T crosswalk did not follow its resolved gate");

  for (const double degrees : {60.0, 15.0}) {
    RoadState angled{};
    auto angled_layout = road_fixture::BidirectionalLayout(0);
    angled_layout.alignment_offset_from_left_m = 2.0;
    const auto angled_section =
        road_fixture::AddLayout(angled, angled_layout);
    const auto angled_branch_section =
        road_fixture::AddLayout(angled, road_fixture::BidirectionalLayout(0));
    const auto angled_base = angled.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-120.0, 0.0}, {120.0, 0.0})}),
        angled_section});
    ROAD_TEST_EXPECT(angled_base.ok, angled_base.error);
    const double radians = degrees * std::numbers::pi / 180.0;
    const auto angled_branch = angled.AddSegmentConnectedToSegment(
        city::road::AddSegmentConnectedToSegmentRequest{
            MakePath({MakeLine({0.0, 0.0},
                               {120.0 * std::cos(radians),
                                120.0 * std::sin(radians)})}),
            angled_branch_section, angled_base.value, 120.0});
    ROAD_TEST_EXPECT(angled_branch.ok, angled_branch.error);
    const auto angled_junctions = road_test_view::junctions(angled.derived());
    ROAD_TEST_EXPECT(angled_junctions.size() == 1,
                     "angled asymmetric T did not produce one junction");
    const auto angled_approach = std::find_if(
        angled_junctions.front()->approaches.begin(),
        angled_junctions.front()->approaches.end(),
        [id = angled_branch.value](const auto& candidate) {
          return candidate.key.segment_id == id;
        });
    ROAD_TEST_EXPECT(
        angled_approach != angled_junctions.front()->approaches.end(),
        "angled asymmetric T branch approach is missing");
    // Intersect the actual side lines. The branch carries a 5m half-width, so
    // its own offset contributes when the approach is not perpendicular.
    const double side_clearance =
        (2.0 + 5.0 * std::cos(radians)) / std::sin(radians);
    const double radius_clearance = 4.0 / std::tan(radians * 0.5);
    const double expected = side_clearance + radius_clearance;
    ROAD_TEST_EXPECT(
        std::abs(angled_approach->resolved_setback_m - expected) <= 1e-6,
        "angled junction setback does not equal facing-side clearance " +
            std::to_string(side_clearance) + " + radius clearance " +
            std::to_string(radius_clearance) + "; actual=" +
            std::to_string(angled_approach->resolved_setback_m));
    if (degrees == 15.0) {
      ROAD_TEST_EXPECT(side_clearance > 20.0 && radius_clearance > 30.0,
                       "acute diagnostic did not expose both long-distance terms");
    }
  }

  RoadState reordered{};
  auto reordered_layout = road_fixture::BidirectionalLayout(0);
  reordered_layout.alignment_offset_from_left_m = 2.0;
  const auto reordered_section =
      road_fixture::AddLayout(reordered, reordered_layout);
  const auto reordered_branch_section = road_fixture::AddLayout(
      reordered, road_fixture::BidirectionalLayout(0));
  const auto north = reordered.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}),
      reordered_branch_section});
  ROAD_TEST_EXPECT(north.ok, north.error);
  const RoadNodeId reordered_node = reordered.graph().segments.front().node_a;
  const auto west = reordered.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({-40.0, 0.0}, {0.0, 0.0})}),
          reordered_section, reordered_node, EndpointRole::kEnd});
  ROAD_TEST_EXPECT(west.ok, west.error);
  const auto east = reordered.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}),
          reordered_section, reordered_node, EndpointRole::kStart});
  ROAD_TEST_EXPECT(east.ok, east.error);
  const auto reordered_junctions =
      road_test_view::junctions(reordered.derived());
  ROAD_TEST_EXPECT(reordered_junctions.size() == 1,
                   "reordered asymmetric T did not produce one junction");
  const auto reordered_north = std::find_if(
      reordered_junctions.front()->approaches.begin(),
      reordered_junctions.front()->approaches.end(),
      [id = north.value](const auto& candidate) {
        return candidate.key.segment_id == id;
      });
  ROAD_TEST_EXPECT(
      reordered_north != reordered_junctions.front()->approaches.end() &&
          std::abs(reordered_north->resolved_setback_m -
                   approach->resolved_setback_m) <= 1e-6,
      "junction setback changed with approach creation order");
  return true;
}

bool P1_junction_setback_uses_only_adjacent_approaches(
    std::string& failure) {
  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto ray = [](double degrees) {
    const double radians = degrees * std::numbers::pi / 180.0;
    return MakePath({MakeLine({0.0, 0.0},
                              {100.0 * std::cos(radians),
                               100.0 * std::sin(radians)})});
  };
  const auto first = state.AddSegment(
      city::road::AddSegmentRequest{ray(0.0), section});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const RoadNodeId node = state.graph().segments.front().node_a;
  for (const double degrees : {80.0, 170.0, 260.0}) {
    const auto added = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            ray(degrees), section, node, EndpointRole::kStart});
    ROAD_TEST_EXPECT(added.ok,
                     "skew cross approach was rejected: " + added.error);
  }
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1,
                   "skew cross did not produce one junction");
  const auto approach = std::find_if(
      junctions.front()->approaches.begin(),
      junctions.front()->approaches.end(),
      [id = first.value](const auto& candidate) {
        return candidate.key.segment_id == id;
      });
  ROAD_TEST_EXPECT(approach != junctions.front()->approaches.end(),
                   "skew cross reference approach is missing");
  const double adjacent_angle = 80.0 * std::numbers::pi / 180.0;
  const double expected =
      (5.0 + 4.0) / std::tan(adjacent_angle * 0.5);
  ROAD_TEST_EXPECT(
      std::abs(approach->resolved_setback_m - expected) <= 1e-6,
      "non-adjacent approach extended the junction; expected=" +
          std::to_string(expected) + " actual=" +
          std::to_string(approach->resolved_setback_m));
  return true;
}

bool P1_one_interval_connects_two_existing_roads_atomically(
    std::string& failure) {
  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto lower = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-20.0, 0.0}, {20.0, 0.0})}), section});
  const auto upper = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-20.0, 40.0}, {20.0, 40.0})}), section});
  ROAD_TEST_EXPECT(lower.ok && upper.ok,
                   "two-road fixture could not be created");

  const auto connected = state.AddSegmentBetween(
      city::road::AddSegmentBetweenRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), section,
          city::road::RoadConnectionTarget{0, lower.value, 20.0},
          city::road::RoadConnectionTarget{0, upper.value, 20.0}});
  ROAD_TEST_EXPECT(connected.ok,
                   "one interval did not connect two roads: " + connected.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 5 &&
                       state.graph().nodes.size() == 6 &&
                       state.graph().corridors.size() == 3,
                   "two-ended connection did not atomically split both roads");
  ROAD_TEST_EXPECT(road_test_view::junctions(state.derived()).size() == 2,
                   "two-ended connection did not resolve both junctions");

  const auto before = state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto rejected = state.AddSegmentBetween(
      city::road::AddSegmentBetweenRequest{
          MakePath({MakeLine({-10.0, 0.0}, {-10.0, 40.0})}), section,
          city::road::RoadConnectionTarget{0, lower.value, 10.0},
          city::road::RoadConnectionTarget{0, upper.value, 1000.0}});
  ROAD_TEST_EXPECT(!rejected.ok, "an invalid second endpoint was accepted");
  const auto after = state.Save();
  ROAD_TEST_EXPECT(after.ok && before.value == after.value,
                   "a failed second endpoint left the first road split");
  return true;
}

bool P1_endpoint_to_segment_middle_connection_does_not_twist(
    std::string& failure) {
  const auto strip_rails_keep_direction = [&failure](
                                             const auto& strip,
                                             const char* label) {
    ROAD_TEST_EXPECT(strip.left.size() == strip.right.size() &&
                         strip.left.size() >= 2,
                     std::string(label) + " strip rails are incomplete");
    const Vec3d first{
        strip.right.front().x - strip.left.front().x,
        strip.right.front().y - strip.left.front().y,
        strip.right.front().z - strip.left.front().z};
    const double first_length =
        std::hypot(std::hypot(first.x, first.y), first.z);
    ROAD_TEST_EXPECT(first_length > 1e-6,
                     std::string(label) + " strip starts with zero width");
    for (std::size_t index = 1; index < strip.left.size(); ++index) {
      const Vec3d current{
          strip.right[index].x - strip.left[index].x,
          strip.right[index].y - strip.left[index].y,
          strip.right[index].z - strip.left[index].z};
      const double current_length =
          std::hypot(std::hypot(current.x, current.y), current.z);
      ROAD_TEST_EXPECT(current_length > 1e-6,
                       std::string(label) + " strip collapses to zero width");
      ROAD_TEST_EXPECT(
          first.x * current.x + first.y * current.y + first.z * current.z >
              0.0,
          std::string(label) + " strip left/right rails swapped along the curve");
    }
    return true;
  };

  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto source = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section});
  const auto target = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 24.0}, {40.0, 24.0})}), section});
  ROAD_TEST_EXPECT(source.ok && target.ok,
                   "endpoint-to-middle fixture could not be created");
  const RoadNodeId source_endpoint = state.graph().segments.front().node_b;
  const auto connected = state.AddSegmentBetween(
      city::road::AddSegmentBetweenRequest{
          MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), section,
          city::road::RoadConnectionTarget{source_endpoint, 0, 0.0},
          city::road::RoadConnectionTarget{0, target.value, 20.0}});
  ROAD_TEST_EXPECT(connected.ok,
                   "endpoint-to-middle connection was rejected: " +
                       connected.error);
  ROAD_TEST_EXPECT(road_test_view::corners(state.derived()).size() == 1,
                   "endpoint-to-middle connection did not create one corner");
  ROAD_TEST_EXPECT(road_test_view::junctions(state.derived()).size() == 1,
                   "endpoint-to-middle connection did not create one junction");
  for (const auto* corner : road_test_view::corners(state.derived())) {
    for (const auto& strip : corner->connection_geometry.surface_strips) {
      ROAD_TEST_EXPECT(strip_rails_keep_direction(strip, "corner"), failure);
    }
  }
  for (const auto* junction : road_test_view::junctions(state.derived())) {
    for (const auto& strip : junction->junction_geometry.surface_strips) {
      ROAD_TEST_EXPECT(strip_rails_keep_direction(strip, "junction"), failure);
    }
  }
  for (const auto* corner : road_test_view::corners(state.derived())) {
    for (const auto& strip : corner->connection_geometry.surface_strips) {
      city::road::ConnectionGeometry one{};
      one.surface_strips.push_back(strip);
      const auto emitted = city::road::generation::emit_connection(one);
      ROAD_TEST_EXPECT(emitted.ok && emitted.value.size() == 1,
                       "endpoint-to-middle strip could not be emitted");
      if (!mesh_faces_up(emitted.value.front())) {
        double ltr_normal_z = 0.0;
        if (strip.left.size() >= 2 && strip.left.size() == strip.right.size()) {
          const Vec3d tangent{strip.left[1].x - strip.left[0].x,
                              strip.left[1].y - strip.left[0].y,
                              strip.left[1].z - strip.left[0].z};
          const Vec3d lateral{strip.right[0].x - strip.left[0].x,
                              strip.right[0].y - strip.left[0].y,
                              strip.right[0].z - strip.left[0].z};
          ltr_normal_z = tangent.x * lateral.y - tangent.y * lateral.x;
        }
        failure = "endpoint-to-middle inverted strip " +
                  std::to_string(strip.left_boundary_id) + "/" +
                  std::to_string(strip.right_boundary_id) + " style " +
                  std::to_string(strip.style.value) + " winding " +
                  std::to_string(static_cast<int>(strip.winding)) +
                  " ltr_normal_z " + std::to_string(ltr_normal_z);
        return false;
      }
    }
  }
  for (const auto& mesh : state.derived().connection_meshes) {
    ROAD_TEST_EXPECT(mesh_faces_up(mesh),
                     "endpoint-to-middle corner mesh has downward triangles");
  }
  for (const auto& mesh : state.derived().junction_meshes) {
    ROAD_TEST_EXPECT(mesh_faces_up(mesh),
                     "endpoint-to-middle junction mesh has downward triangles");
  }

  RoadState short_state{};
  const auto short_section =
      road_fixture::AddLayout(short_state, road_fixture::BidirectionalLayout(0));
  const auto short_source = short_state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), short_section});
  const auto short_target = short_state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 12.0}, {40.0, 12.0})}), short_section});
  ROAD_TEST_EXPECT(short_source.ok && short_target.ok,
                   "short endpoint-to-middle fixture could not be created");
  const auto saved = short_state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const RoadNodeId short_endpoint = short_state.graph().segments.front().node_b;
  const auto short_connected = short_state.AddSegmentBetween(
      city::road::AddSegmentBetweenRequest{
          MakePath({MakeLine({20.0, 0.0}, {20.0, 12.0})}), short_section,
          city::road::RoadConnectionTarget{short_endpoint, 0, 0.0},
          city::road::RoadConnectionTarget{0, short_target.value, 20.0}});
  ROAD_TEST_EXPECT(
      !short_connected.ok &&
          short_connected.error.find("connection gates overlap") !=
              std::string::npos,
      "a short corner-to-junction connector was accepted and can twist");
  const auto after = short_state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value == saved.value,
                   "rejected short corner-to-junction connector mutated state");
  return true;
}

bool P1_end_segment_snap_splits_straight_road_for_t_junction(
    std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 24.0}, {20.0, 0.0})}), section,
          base.value, 20.0, EndpointRole::kEnd});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3,
                   "end-snapped T junction did not split the base road");
  const auto branch_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [branch](const auto& segment) { return segment.id == branch.value; });
  ROAD_TEST_EXPECT(branch_segment != state.graph().segments.end(),
                   "end-snapped branch segment is missing");
  ROAD_TEST_EXPECT(
      state.graph().nodes.end() != std::find_if(
          state.graph().nodes.begin(), state.graph().nodes.end(),
          [&branch_segment](const auto& node) {
            return node.id == branch_segment->node_b &&
                   std::hypot(node.position.x - 20.0, node.position.y) < 1e-9;
          }),
      "end-snapped branch did not preserve its input direction to the junction");
  ROAD_TEST_EXPECT(road_test_view::junctions(state.derived()).size() == 1,
                   "end-snapped T junction did not derive one junction");
  ROAD_TEST_EXPECT(
      road_test_view::gates_of(*road_test_view::junctions(state.derived()).front()).size() == 3,
      "end-snapped T junction does not have three approaches");
  return true;
}

bool P1_every_edge_section_forms_a_t_junction(std::string& failure) {
  const std::array<city::road::RoadLayoutTemplate, 5> sections{
      road_fixture::BidirectionalLayout(0), road_fixture::ExtraLaneLayout(0),
      road_fixture::AsymmetricLayout(0), road_fixture::MedianLayout(0),
      road_fixture::ShoulderedLayout(0)};
  for (const city::road::RoadLayoutTemplate& section : sections) {
    RoadState state{};
    const auto template_id = road_fixture::AddLayout(state, section);
    ROAD_TEST_EXPECT(template_id != 0, "edge-section fixture was rejected");
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-30.0, 0.0}, {30.0, 0.0})}), template_id});
    ROAD_TEST_EXPECT(base.ok,
                     "edge-section base road could not be created");
    const auto branch = state.AddSegmentConnectedToSegment(
        city::road::AddSegmentConnectedToSegmentRequest{
            MakePath({MakeLine({0.0, 0.0}, {0.0, 30.0})}), template_id,
            base.value, 30.0});
    ROAD_TEST_EXPECT(
        branch.ok,
        "edge-section T junction was rejected: " + branch.error);
    ROAD_TEST_EXPECT(
        road_test_view::junctions(state.derived()).size() == 1 &&
            !state.derived().junction_meshes.empty(),
        "edge-section T junction geometry is missing");
  }
  return true;
}

bool P1_skew_shoulder_junction_connects_outer_lines(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const double radians = 30.0 * std::numbers::pi / 180.0;
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0},
                             {40.0 * std::cos(radians),
                              40.0 * std::sin(radians)})}),
          shouldered, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const std::size_t junction_outer_lines = std::count_if(
      state.derived().markings.begin(), state.derived().markings.end(),
      [](const DerivedMarking& marking) {
        return marking.owner.kind == MarkingOwner::Kind::kJunction &&
               marking.role == MarkingRole::kCarriagewayEdge &&
               marking.points.size() >= 2;
      });
  ROAD_TEST_EXPECT(junction_outer_lines == 3,
                   "skew shoulder T junction does not connect all three "
                   "adjacent outer-line pairs");
  return true;
}

bool P1_skew_shoulder_junction_mesh_does_not_overlap(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const double radians = 30.0 * std::numbers::pi / 180.0;
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0},
                             {40.0 * std::cos(radians),
                              40.0 * std::sin(radians)})}),
          shouldered, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  return junction_mesh_is_non_overlapping(state, "skew shoulder T", failure);
}

bool P1_segment_snap_splits_bezier_road_for_t_junction(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const Path curve = MakePath({MakeBezier({0.0, 0.0}, {20.0, 10.0}, {60.0, 10.0}, {80.0, 0.0})});
  const auto base = state.AddSegment(city::road::AddSegmentRequest{curve, section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto curve_length = PathLength(curve);
  ROAD_TEST_EXPECT(curve_length.ok, curve_length.error);
  const auto split_point = EvaluatePath(curve, curve_length.value * 0.5);
  ROAD_TEST_EXPECT(split_point.ok, split_point.error);
  const auto branch = state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{
      MakePath({MakeLine(split_point.value, {split_point.value.x, split_point.value.y + 30.0})}), section, base.value,
      curve_length.value * 0.5});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3, "Bezier T junction did not split base road and add branch");
  ROAD_TEST_EXPECT(state.graph().connection_policy_overrides.empty(), "Bezier T junction saved an automatic decision");
  ROAD_TEST_EXPECT(road_test_view::junctions(state.derived()).size() == 1 &&
                       road_test_view::gates_of(*road_test_view::junctions(state.derived()).front()).size() == 3,
                   "Bezier T junction did not derive three connection gates");
  const auto first = std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                                  [base](const auto& segment) { return segment.id == base.value; });
  const auto second = std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                                   [base, branch](const auto& segment) {
                                     return segment.id != base.value && segment.id != branch.value;
                                   });
  ROAD_TEST_EXPECT(first != state.graph().segments.end() && second != state.graph().segments.end(),
                   "Bezier split segments are missing");
  const Path* first_path = FindCanonicalAlignment(state.derived(), first->id);
  const Path* second_path = FindCanonicalAlignment(state.derived(), second->id);
  ROAD_TEST_EXPECT(first_path != nullptr && second_path != nullptr, "Bezier split alignments are missing");
  const auto& left = first_path->spans.back();
  const auto& right = second_path->spans.front();
  const Vec2d left_tangent{left.p3.x - left.p2.x, left.p3.y - left.p2.y};
  const Vec2d right_tangent{right.p1.x - right.p0.x, right.p1.y - right.p0.y};
  ROAD_TEST_EXPECT(std::hypot(left.p3.x - right.p0.x, left.p3.y - right.p0.y) < 1e-9,
                   "Bezier split produced a positional gap");
  ROAD_TEST_EXPECT(std::abs(left_tangent.x * right_tangent.y - left_tangent.y * right_tangent.x) < 1e-9 &&
                       left_tangent.x * right_tangent.x + left_tangent.y * right_tangent.y > 0.0,
                   "Bezier split did not preserve tangent continuity");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok,
                   "Bezier T junction invariants failed");
  return true;
}

bool P1_segment_snap_splits_at_internal_bezier_span_boundary(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first_span = MakeBezier({0.0, 0.0}, {10.0, 10.0}, {30.0, 10.0}, {40.0, 0.0});
  const auto second_span = MakeBezier({40.0, 0.0}, {50.0, -10.0}, {70.0, -10.0}, {80.0, 0.0});
  const Path alignment = MakePath({first_span, second_span});
  const auto base = state.AddSegment(city::road::AddSegmentRequest{alignment, section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1,
                   "one multi-span draw was split before an explicit edit");
  const auto first_length = PathLength(MakePath({first_span}));
  ROAD_TEST_EXPECT(first_length.ok, first_length.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({40.0, 0.0}, {40.0, 30.0})}), section,
          base.value, first_length.value});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3,
                   "explicit connection at an internal span boundary did not split once and add a branch");
  for (const auto& segment : state.graph().segments) {
    const Path* path = FindCanonicalAlignment(state.derived(), segment.id);
    ROAD_TEST_EXPECT(path != nullptr, "span-boundary split alignment is missing");
    const auto length = PathLength(*path);
    ROAD_TEST_EXPECT(length.ok && length.value > 0.0, "span-boundary split created a zero-length span");
  }
  return true;
}

bool P1_cross_junction_accepts_opposite_approaches(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), section,
                                                        base.value, 20.0});
  ROAD_TEST_EXPECT(north.ok, north.error);
  ROAD_TEST_EXPECT(!road_test_view::junctions(state.derived()).empty(), "T junction area is missing");
  const auto junction_node = road_test_view::junctions(state.derived()).front()->node_id;
  const auto south = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({20.0, 0.0}, {20.0, -24.0})}), section,
                                                 junction_node});
  ROAD_TEST_EXPECT(south.ok, south.error);
  ROAD_TEST_EXPECT(road_test_view::gates_of(*road_test_view::junctions(state.derived()).front()).size() == 4,
                   "cross junction does not have four approaches");
  ROAD_TEST_EXPECT(state.derived().junction_meshes.size() >= 3,
                   "cross junction did not derive material-separated shared surfaces");
  return true;
}

bool P1_incremental_skew_cross_accepts_ordered_approaches(
    std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {25.0, 24.0})}), section, base.value,
          20.0});
  ROAD_TEST_EXPECT(north.ok, north.error);
  const auto junction_node =
      road_test_view::junctions(state.derived()).front()->node_id;
  const auto south = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({20.0, 0.0}, {25.0, -24.0})}), section,
          junction_node});
  ROAD_TEST_EXPECT(south.ok, south.error);
  ROAD_TEST_EXPECT(
      road_test_view::gates_of(
          *road_test_view::junctions(state.derived()).front())
              .size() == 4,
      "incremental skew cross does not have four approaches");
  return true;
}

bool P1_skew_shoulder_cross_has_connected_lines_without_overlap(
    std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto first_diagonal = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {35.0, 20.0})}), shouldered, base.value,
          40.0});
  ROAD_TEST_EXPECT(first_diagonal.ok, first_diagonal.error);
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1,
                   "skew shoulder cross T state has no junction");
  const auto second_diagonal = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {-35.0, -20.0})}), shouldered,
          junctions.front()->node_id});
  ROAD_TEST_EXPECT(second_diagonal.ok, second_diagonal.error);
  const std::size_t junction_outer_lines = std::count_if(
      state.derived().markings.begin(), state.derived().markings.end(),
      [](const DerivedMarking& marking) {
        return marking.owner.kind == MarkingOwner::Kind::kJunction &&
               marking.role == MarkingRole::kCarriagewayEdge &&
               marking.points.size() >= 2;
      });
  ROAD_TEST_EXPECT(junction_outer_lines == 4,
                   "skew shoulder cross does not connect all four adjacent "
                   "outer-line pairs");
  return junction_mesh_is_non_overlapping(state, "skew shoulder cross",
                                          failure);
}

bool section_transition_widens_one_side_from_its_anchor(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({100.0, 50.0}, {160.0, 50.0})}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto template_id = state.AddRoadLayoutTemplate(city::road::AddRoadLayoutTemplateRequest{road_fixture::ExtraLaneLayout(0)});
  ROAD_TEST_EXPECT(template_id.ok, template_id.error);
  SavedRoadGraph graph = state.graph();
  attach_transition(
      graph, added.value,
      RoadLayoutTransition{9001, section, template_id.value,
                        DistanceRef{DistanceRefKind::kFromEnd, 25.0},
                        DistanceRef{DistanceRefKind::kFromEnd, 5.0},
                        TransitionAnchor::kLeftEdge, 0,
                        {RoadLayoutTransitionRule{35, TransitionAction::kTaperIn},
                         RoadLayoutTransitionRule{10, TransitionAction::kContinue}}});
  const auto generated = city::road::generation::generate_road(graph);
  ROAD_TEST_EXPECT(generated.ok, generated.error);

  const auto sections = road_test_view::sections(generated.value);
  const auto at_distance = [&sections](double distance) -> const city::road::SectionEvaluation* {
    const auto it = std::find_if(sections.begin(), sections.end(), [distance](const auto* item) {
      return std::abs(item->segment_distance_m - distance) < 1e-6;
    });
    return it == sections.end() ? nullptr : *it;
  };
  const auto* before = at_distance(0.0);
  const auto* after = at_distance(60.0);
  ROAD_TEST_EXPECT(before != nullptr && after != nullptr, "P2 transition endpoints were not evaluated");
  const double before_width = before->boundaries.back().lateral_m - before->boundaries.front().lateral_m;
  const double after_width = after->boundaries.back().lateral_m - after->boundaries.front().lateral_m;
  ROAD_TEST_EXPECT(std::abs(before_width - 10.0) < 1e-6, "P2 transition changed the from-section before its start");
  ROAD_TEST_EXPECT(std::abs(after_width - 13.0) < 1e-6, "P2 transition did not reach the extra-lane layout");
  ROAD_TEST_EXPECT(std::abs(before->boundaries.front().lateral_m - after->boundaries.front().lateral_m) < 1e-6,
                   "P2 left-edge anchor moved during one-sided widening");

  const auto loaded = load_fixture(graph);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  ROAD_TEST_EXPECT(loaded.value.graph().transitions.size() == 1 &&
                       loaded.value.graph().segments.front().transition == 9001,
                   "P2 transition authority did not survive save/load");
  return true;
}

bool lane_allocation_transition_uses_lane_identity(std::string& failure) {
  RoadState state{};
  city::road::RoadLayoutTemplate from{};
  from.alignment_offset_from_left_m = 2.0;
  from.strips = {
      {10, StripFunction::kShoulder, 2.0, 0.0,
       builtin_surface_styles::kAsphalt},
      {20, StripFunction::kCarriageway, 6.0, 0.0,
       builtin_surface_styles::kAsphalt},
  };
  from.boundaries = {
      road_fixture::PaintedLineBoundary(100, BoundaryRole::kOuterEdge, {})};
  from.lane_bands = {
      {10, 20, 0.0, 3.0, city::road::LaneTravelDirection::kAlongSegment},
      {20, 20, 3.0, 6.0, city::road::LaneTravelDirection::kAlongSegment},
  };
  const auto from_id = road_fixture::AddLayout(state, from);
  ROAD_TEST_EXPECT(from_id != 0, "source lane allocation was rejected");

  city::road::RoadLayoutTemplate to = from;
  to.strips[0].width_m = 0.5;
  to.strips[1].width_m = 7.5;
  to.lane_bands = {
      {10, 20, 0.0, 2.5, city::road::LaneTravelDirection::kAlongSegment},
      {20, 20, 2.5, 5.0, city::road::LaneTravelDirection::kAlongSegment},
      {30, 20, 5.0, 7.5, city::road::LaneTravelDirection::kAlongSegment},
  };
  const auto to_id = road_fixture::AddLayout(state, to);
  ROAD_TEST_EXPECT(to_id != 0, "target lane allocation was rejected");
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), from_id});
  ROAD_TEST_EXPECT(segment.ok, segment.error);

  const auto transition_graph = [&](city::road::RoadLayoutTemplateId source,
                                    city::road::RoadLayoutTemplateId target) {
    SavedRoadGraph graph = state.graph();
    graph.segments.front().layout_template = source;
    attach_transition(
        graph, segment.value,
        RoadLayoutTransition{
            9001, source, target,
            DistanceRef{DistanceRefKind::kFromStart, 10.0},
            DistanceRef{DistanceRefKind::kFromStart, 50.0},
            TransitionAnchor::kLeftEdge, 0,
            {RoadLayoutTransitionRule{
                 10, TransitionAction::kChangeWidthHeightOffset},
             RoadLayoutTransitionRule{
                 20, TransitionAction::kChangeWidthHeightOffset}}});
    return graph;
  };
  const SavedRoadGraph graph = transition_graph(from_id, to_id);
  const auto halfway = city::road::internal::template_at(
      graph, graph.segments.front(), 30.0, 60.0);
  ROAD_TEST_EXPECT(halfway.ok, halfway.error);
  const double width = std::accumulate(
      halfway.value.strips.begin(), halfway.value.strips.end(), 0.0,
      [](double sum, const city::road::RoadLayoutStrip& strip) {
        return sum + strip.width_m;
      });
  ROAD_TEST_EXPECT(std::abs(width - 8.0) < 1e-9,
                   "lane birth changed the fixed road outer width");
  const auto lane = [](const city::road::RoadLayoutTemplate& layout,
                       LaneId id) -> const city::road::LaneBand* {
    const auto found = std::find_if(
        layout.lane_bands.begin(), layout.lane_bands.end(),
        [id](const city::road::LaneBand& item) { return item.id == id; });
    return found == layout.lane_bands.end() ? nullptr : &*found;
  };
  const auto* first = lane(halfway.value, 10);
  const auto* second = lane(halfway.value, 20);
  const auto* born = lane(halfway.value, 30);
  ROAD_TEST_EXPECT(first != nullptr && second != nullptr && born != nullptr,
                   "lane identity transition lost a lane");
  ROAD_TEST_EXPECT(std::abs(first->lateral_end_m - 2.75) < 1e-9 &&
                       std::abs(second->lateral_start_m - 2.75) < 1e-9 &&
                       std::abs(second->lateral_end_m - 5.5) < 1e-9 &&
                       std::abs(born->lateral_start_m - 5.5) < 1e-9 &&
                       std::abs(born->lateral_end_m - 6.75) < 1e-9,
                   "lane ranges did not interpolate by LaneId");

  const SavedRoadGraph reverse = transition_graph(to_id, from_id);
  const auto reverse_halfway = city::road::internal::template_at(
      reverse, reverse.segments.front(), 30.0, 60.0);
  ROAD_TEST_EXPECT(reverse_halfway.ok, reverse_halfway.error);
  for (const auto& expected : halfway.value.lane_bands) {
    const auto* actual = lane(reverse_halfway.value, expected.id);
    ROAD_TEST_EXPECT(actual != nullptr &&
                         actual->lateral_start_m == expected.lateral_start_m &&
                         actual->lateral_end_m == expected.lateral_end_m,
                     "lane birth and death differ when reversed");
  }

  auto changed_marking = from;
  changed_marking.boundaries.front().marking = {
      true, MarkingRole::kCarriagewayEdge,
      builtin_marking_styles::kCrosswalk, MarkingPlacement::kInside};
  const auto base_center = city::road::internal::lane_template_lateral(
      from, from.lane_bands.front());
  const auto marked_center = city::road::internal::lane_template_lateral(
      changed_marking, changed_marking.lane_bands.front());
  ROAD_TEST_EXPECT(base_center.ok && marked_center.ok &&
                       base_center.value == marked_center.value,
                   "marking width changed lane allocation center");

  const auto archived = city::road::persistence::SaveRoad(graph, kFixtureNextId);
  ROAD_TEST_EXPECT(archived.ok, archived.error);
  const auto loaded = RoadState::Load(archived.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto resaved = loaded.value.Save();
  ROAD_TEST_EXPECT(resaved.ok && resaved.value == archived.value,
                   "lane transition was not byte-stable after load");

  city::road::RoadLayoutTemplate crossing = from;
  crossing.lane_bands = {
      {10, 20, 3.0, 6.0, city::road::LaneTravelDirection::kAlongSegment},
      {20, 20, 0.0, 3.0, city::road::LaneTravelDirection::kAlongSegment},
  };
  const auto crossing_id = road_fixture::AddLayout(state, crossing);
  ROAD_TEST_EXPECT(crossing_id != 0, "crossing target was rejected early");
  const auto crossing_result = city::road::generation::generate_road(
      transition_graph(from_id, crossing_id));
  ROAD_TEST_EXPECT(!crossing_result.ok &&
                       crossing_result.failure_category ==
                           CommitFailureCategory::kNotImplemented,
                   "crossing LaneIds were rebound by lateral order");
  return true;
}

bool saved_manual_markings_load_and_draw(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({100.0, 50.0}, {160.0, 50.0})}), section});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const double rotation = std::acos(-1.0) * 0.5;
  SavedRoadGraph graph = state.graph();
  graph.manual_lines.push_back(city::road::ManualLineMarking{
      9001, added.value, MakePath({MakeLine({5.0, 0.5}, {20.0, 0.5})}),
      builtin_marking_styles::kWhiteSolid});
  graph.manual_areas.push_back(city::road::ManualAreaMarking{
      9002, added.value, {30.0, 0.0}, rotation, 4.0, 8.0,
      builtin_marking_styles::kCrosswalk});

  const auto archived = city::road::persistence::SaveRoad(graph, kFixtureNextId);
  ROAD_TEST_EXPECT(archived.ok, archived.error);
  const auto loaded = RoadState::Load(archived.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const RoadState& restored = loaded.value;
  ROAD_TEST_EXPECT(restored.graph().manual_lines.size() == 1 &&
                       restored.graph().manual_areas.size() == 1,
                   "manual marking authority did not survive load");
  ROAD_TEST_EXPECT(restored.graph().manual_areas.front().rotation_rad == rotation,
                   "manual area rotation did not survive load");
  const auto round_trip = restored.Save();
  ROAD_TEST_EXPECT(round_trip.ok && round_trip.value == archived.value,
                   "loaded manual markings did not re-save bit-identically");

  const auto manual_path_count = road_test_view::count_marking_lines(restored.derived(), [](const auto& path) {
        return path.owner.kind == city::road::MarkingOwner::Kind::kManual;
      });
  const auto manual_area_count = road_test_view::count_marking_areas(restored.derived(), [](const auto& area) {
        return area.owner.kind == city::road::MarkingOwner::Kind::kManual;
      });
  ROAD_TEST_EXPECT(manual_path_count == 1 && manual_area_count == 1,
                   "loaded manual markings were not resolved");

  const auto manual_lines = road_test_view::marking_lines(restored.derived());
  const auto manual_line_it = std::find_if(
      manual_lines.begin(), manual_lines.end(), [](const auto* path) {
        return path->owner.kind == city::road::MarkingOwner::Kind::kManual &&
               path->style_id == builtin_marking_styles::kWhiteSolid;
      });
  ROAD_TEST_EXPECT(manual_line_it != manual_lines.end(),
                   "loaded manual line was not resolved");
  ROAD_TEST_EXPECT((*manual_line_it)->points.size() >= 2,
                   "loaded manual line is not a drawable path");
  ROAD_TEST_EXPECT(std::abs((*manual_line_it)->points.front().x - 105.0) < 1e-6 &&
                       std::abs((*manual_line_it)->points.front().y - 50.5) < 0.1,
                   "loaded manual line was not transformed from owner-local coordinates");
  ROAD_TEST_EXPECT((*manual_line_it)->points.front().z > 0.02,
                   "loaded manual line does not follow the owner section cross slope");
  const auto area_mesh_it = std::find_if(
      restored.derived().marking_meshes.begin(), restored.derived().marking_meshes.end(),
      [](const auto& mesh) {
        return mesh.style == RenderStyleFromMarking(builtin_marking_styles::kCrosswalk);
      });
  ROAD_TEST_EXPECT(area_mesh_it != restored.derived().marking_meshes.end(),
                   "loaded manual area mesh was not materialized");
  ROAD_TEST_EXPECT(std::abs(area_mesh_it->vertices.front().x - 132.0) < 1e-6 &&
                       std::abs(area_mesh_it->vertices.front().y - 46.0) < 1e-6,
                   "loaded manual area rotation was not transformed from owner-local coordinates");
  ROAD_TEST_EXPECT((*manual_line_it)->style_id != builtin_marking_styles::kCrosswalk,
                   "distinct manual marking styles collapsed to one mesh style");

  SavedRoadGraph unknown_style = graph;
  unknown_style.manual_lines.front().style_id = city::road::MarkingStyleId{999};
  const auto rejected = city::road::persistence::SaveRoad(unknown_style, kFixtureNextId);
  ROAD_TEST_EXPECT(!rejected.ok && rejected.failure_category == CommitFailureCategory::kInvalidInput,
                   "an unknown manual line style was accepted as authoritative data");
  return true;
}

bool add_lane_preserves_existing_lanes(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto corridor = city::road::FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "LAN3 fixture corridor is missing");

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added_lane = state.AddLane(request);
  ROAD_TEST_EXPECT(added_lane.ok, added_lane.error);

  const auto sections = road_test_view::sections(state.derived());
  const auto lateral = [](const city::road::SectionEvaluation& section,
                          city::road::BoundaryId id) -> std::optional<double> {
    const auto found = std::find_if(
        section.boundaries.begin(), section.boundaries.end(),
        [id](const auto& boundary) { return boundary.boundary_id == id; });
    return found == section.boundaries.end()
               ? std::nullopt
               : std::optional<double>(found->lateral_m);
  };
  ROAD_TEST_EXPECT(sections.size() >= 3, "LAN3 transition samples are missing");
  const auto* before = sections.front();
  const auto* after = sections.back();
  for (const city::road::BoundaryId id : {150ULL, 200ULL}) {
    const auto a = lateral(*before, id);
    const auto c = lateral(*after, id);
    ROAD_TEST_EXPECT(a.has_value() && c.has_value(),
                     "LAN3 existing boundary is missing");
    for (const auto* section : sections) {
      const auto current = lateral(*section, id);
      ROAD_TEST_EXPECT(
          current.has_value() && *current == *a,
          "LAN3 moved boundary " + std::to_string(id) + " from " +
              std::to_string(*a) + " to " +
              (current.has_value() ? std::to_string(*current) : "missing") +
              " on segment " + std::to_string(section->segment_id));
    }
  }
  const auto outer_before = lateral(*before, 250);
  const auto outer_after = lateral(*after, 250);
  ROAD_TEST_EXPECT(outer_before && outer_after,
                   "LAN3 shoulder boundary is missing");
  double previous = *outer_before;
  for (const auto* section : sections) {
    const auto current = lateral(*section, 250);
    ROAD_TEST_EXPECT(current.has_value() && *current + 1e-9 >= previous,
                     "LAN3 added lane width is not monotonic");
    previous = *current;
  }
  ROAD_TEST_EXPECT(std::abs((*outer_after - *outer_before) - 3.0) < 1e-6,
                   "LAN3 added lane did not reach its requested width");
  const auto outer_line = std::find_if(
      state.derived().markings.begin(), state.derived().markings.end(),
      [segment](const city::road::DerivedMarking& marking) {
        return marking.owner.kind ==
                   city::road::MarkingOwner::Kind::kRoadSegment &&
               marking.owner.segment_id == segment.value &&
               marking.boundary_id == 250 &&
               marking.role == city::road::MarkingRole::kCarriagewayEdge;
      });
  ROAD_TEST_EXPECT(outer_line != state.derived().markings.end(),
                   "LAN3 removed the existing shoulder edge line");
  ROAD_TEST_EXPECT(!outer_line->points.empty() &&
                       outer_line->points.front().x < 1.0,
                   "LAN3 shoulder edge line does not start before the added lane");
  const auto target_template = std::max_element(
      state.graph().layout_templates.begin(),
      state.graph().layout_templates.end(), [](const auto& a, const auto& b) {
        return a.id < b.id;
      });
  ROAD_TEST_EXPECT(target_template != state.graph().layout_templates.end(),
                   "LAN3 target template is missing");
  const auto added = std::find_if(
      target_template->lane_bands.begin(), target_template->lane_bands.end(),
      [&added_lane](const auto& lane) { return lane.id == added_lane.value; });
  ROAD_TEST_EXPECT(added != target_template->lane_bands.end() &&
                       added->direction ==
                           city::road::LaneTravelDirection::kAlongSegment,
                   "LAN3 added lane lost its identity or direction");
  const auto shoulder = std::find_if(
      target_template->strips.begin(), target_template->strips.end(),
      [](const auto& strip) { return strip.id == 35; });
  ROAD_TEST_EXPECT(shoulder != target_template->strips.end() &&
                       shoulder->function == city::road::StripFunction::kShoulder &&
                       shoulder->width_m == 0.75,
                   "LAN3 changed the existing shoulder width");

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  request.lane_width_m = 0.0;
  const auto rejected = state.AddLane(request);
  ROAD_TEST_EXPECT(!rejected.ok &&
                       rejected.failure_category == city::road::CommitFailureCategory::kInvalidInput,
                   "LAN3 accepted an invalid lane width");
  const auto after_reject = state.Save();
  ROAD_TEST_EXPECT(after_reject.ok && after_reject.value == saved.value,
                   "LAN3 invalid request mutated authoritative state");
  return true;
}

bool add_lane_stores_one_segment_local_transition(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest segment_request{
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), shouldered};
  segment_request.corner_radius_m = 6.0;
  const auto segment = state.AddSegment(std::move(segment_request));
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto* corridor =
      city::road::FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr,
                   "LANE0 fixture corridor is missing");
  const auto source = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = segment.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(source != state.graph().segments.end(),
                   "LANE0 source segment is missing");
  const std::size_t segment_count = state.graph().segments.size();
  const std::size_t section_count = state.graph().layout_templates.size();
  const RoadNodeId segment_end_node = source->node_b;

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.transition_start = {segment.value, 0.2};
  request.transition_complete = {segment.value, 0.6};
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == segment_count,
                   "LANE0 Add Lane split the selected RoadSegment");
  const auto updated = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = segment.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(updated != state.graph().segments.end() &&
                       updated->transition.has_value() &&
                       updated->corner_radius_m == 6.0,
                   "LANE0 segment identity or transition was lost");
  const auto transition = std::find_if(
      state.graph().transitions.begin(), state.graph().transitions.end(),
      [id = *updated->transition](const auto& item) { return item.id == id; });
  ROAD_TEST_EXPECT(
      transition != state.graph().transitions.end() &&
          transition->start.kind == DistanceRefKind::kRatio &&
          transition->start.value == 0.2 &&
          transition->end.kind == DistanceRefKind::kRatio &&
          transition->end.value == 0.6,
      "LANE0 transition did not preserve segment-local t");
  const auto transition_id = transition->id;
  ROAD_TEST_EXPECT(state.graph().layout_templates.size() == section_count + 1,
                   "LANE0 stored an intermediate section template");
  const auto moved = state.MoveNode(
      city::road::MoveNodeRequest{segment_end_node, {150.0, 0.0}});
  ROAD_TEST_EXPECT(moved.ok, moved.error);
  const auto moved_transition = std::find_if(
      state.graph().transitions.begin(), state.graph().transitions.end(),
      [id = transition_id](const auto& item) { return item.id == id; });
  ROAD_TEST_EXPECT(moved_transition != state.graph().transitions.end() &&
                       moved_transition->start.value == 0.2 &&
                       moved_transition->end.value == 0.6,
                   "segment resize rewrote segment-local transition t");
  const auto curved_shape = city::road::SegmentShapeFromPath(MakePath({
      MakeBezier({0.0, 0.0}, {35.0, 30.0}, {110.0, 30.0},
                 {150.0, 0.0})}));
  ROAD_TEST_EXPECT(curved_shape.ok, curved_shape.error);
  const auto edited = state.EditSegmentShape(
      city::road::EditSegmentShapeRequest{segment.value, curved_shape.value});
  ROAD_TEST_EXPECT(edited.ok, edited.error);
  const auto edited_transition = std::find_if(
      state.graph().transitions.begin(), state.graph().transitions.end(),
      [id = transition_id](const auto& item) { return item.id == id; });
  ROAD_TEST_EXPECT(edited_transition != state.graph().transitions.end() &&
                       edited_transition->start.value == 0.2 &&
                       edited_transition->end.value == 0.6,
                   "segment shape editing rewrote segment-local transition t");
  const auto* edited_segment =
      city::road::FindDerivedSegment(state.derived(), segment.value);
  ROAD_TEST_EXPECT(edited_segment != nullptr,
                   "edited transition segment is missing from derived road");
  const auto has_semantic_distance = [edited_segment](double expected) {
    return std::any_of(
        edited_segment->semantic_segment_distances_m.begin(),
        edited_segment->semantic_segment_distances_m.end(),
        [expected](double actual) { return std::abs(actual - expected) < 1e-8; });
  };
  ROAD_TEST_EXPECT(
      has_semantic_distance(edited_segment->length_m * 0.2) &&
          has_semantic_distance(edited_segment->length_m * 0.6),
      "segment shape editing did not re-evaluate transition t on the new curve");
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto loaded_transition = std::find_if(
      loaded.value.graph().transitions.begin(),
      loaded.value.graph().transitions.end(),
      [id = transition_id](const auto& item) { return item.id == id; });
  ROAD_TEST_EXPECT(loaded_transition != loaded.value.graph().transitions.end() &&
                       loaded_transition->start.value == 0.2 &&
                       loaded_transition->end.value == 0.6,
                   "save/load rewrote segment-local transition t");
  return true;
}

bool add_lane_preserves_unrelated_corridor_geometry(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto affected = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), shouldered});
  const auto unrelated = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeBezier({0.0, 80.0}, {30.0, 95.0}, {70.0, 95.0},
                           {100.0, 80.0})}), shouldered});
  ROAD_TEST_EXPECT(affected.ok && unrelated.ok,
                   affected.ok ? unrelated.error : affected.error);
  const auto* corridor =
      FindCorridorForSegment(state.graph(), affected.value);
  const auto affected_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = affected.value](const RoadSegment& item) { return item.id == id; });
  const auto unrelated_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = unrelated.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(corridor != nullptr &&
                       affected_segment != state.graph().segments.end() &&
                       unrelated_segment != state.graph().segments.end(),
                   "locality fixture is incomplete");
  const RoadSegment unrelated_authoritative = *unrelated_segment;
  std::vector<Mesh> unrelated_meshes{};
  for (const Mesh& mesh : state.derived().segment_meshes) {
    if (mesh.owner_segment_id == unrelated.value) unrelated_meshes.push_back(mesh);
  }
  std::vector<city::road::DerivedMarking> unrelated_markings{};
  for (const auto& marking : state.derived().markings) {
    if (marking.owner.kind == MarkingOwner::Kind::kRoadSegment &&
        marking.owner.segment_id == unrelated.value) {
      unrelated_markings.push_back(marking);
    }
  }

  city::road::AddLaneRequest request{
      corridor->id, city::road::LaneTravelDirection::kAlongSegment,
      city::road::RoadSide::kRight, {affected.value, 0.3},
      {affected.value, 0.7}, 3.0};
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto unrelated_after = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = unrelated.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(unrelated_after != state.graph().segments.end() &&
                       unrelated_after->node_a == unrelated_authoritative.node_a &&
                       unrelated_after->node_b == unrelated_authoritative.node_b &&
                       unrelated_after->layout_template ==
                           unrelated_authoritative.layout_template &&
                       unrelated_after->transition == unrelated_authoritative.transition &&
                       unrelated_after->shape.start_handle.x ==
                           unrelated_authoritative.shape.start_handle.x &&
                       unrelated_after->shape.start_handle.y ==
                           unrelated_authoritative.shape.start_handle.y &&
                       unrelated_after->shape.end_handle.x ==
                           unrelated_authoritative.shape.end_handle.x &&
                       unrelated_after->shape.end_handle.y ==
                           unrelated_authoritative.shape.end_handle.y,
                   "ADD LANE changed unrelated authoritative segment data");

  std::vector<Mesh> meshes_after{};
  for (const Mesh& mesh : state.derived().segment_meshes) {
    if (mesh.owner_segment_id == unrelated.value) meshes_after.push_back(mesh);
  }
  ROAD_TEST_EXPECT(meshes_after.size() == unrelated_meshes.size(),
                   "ADD LANE changed unrelated surface mesh count");
  for (std::size_t index = 0; index < unrelated_meshes.size(); ++index) {
    const Mesh& before = unrelated_meshes[index];
    const Mesh& after = meshes_after[index];
    ROAD_TEST_EXPECT(before.style == after.style &&
                         before.indices == after.indices &&
                         before.vertices.size() == after.vertices.size(),
                     "ADD LANE changed unrelated surface mesh structure");
    for (std::size_t vertex = 0; vertex < before.vertices.size(); ++vertex) {
      ROAD_TEST_EXPECT(before.vertices[vertex].x == after.vertices[vertex].x &&
                           before.vertices[vertex].y == after.vertices[vertex].y &&
                           before.vertices[vertex].z == after.vertices[vertex].z,
                       "ADD LANE changed unrelated surface mesh geometry");
    }
  }
  std::vector<city::road::DerivedMarking> markings_after{};
  for (const auto& marking : state.derived().markings) {
    if (marking.owner.kind == MarkingOwner::Kind::kRoadSegment &&
        marking.owner.segment_id == unrelated.value) {
      markings_after.push_back(marking);
    }
  }
  ROAD_TEST_EXPECT(markings_after.size() == unrelated_markings.size(),
                   "ADD LANE changed unrelated marking count");
  for (std::size_t index = 0; index < unrelated_markings.size(); ++index) {
    const auto& before = unrelated_markings[index];
    const auto& after = markings_after[index];
    ROAD_TEST_EXPECT(before.owner == after.owner &&
                         before.boundary_id == after.boundary_id &&
                         before.role == after.role &&
                         before.style_id == after.style_id &&
                         before.width_m == after.width_m &&
                         before.points.size() == after.points.size(),
                     "ADD LANE changed unrelated marking semantics");
    for (std::size_t point = 0; point < before.points.size(); ++point) {
      ROAD_TEST_EXPECT(before.points[point].x == after.points[point].x &&
                           before.points[point].y == after.points[point].y &&
                           before.points[point].z == after.points[point].z,
                       "ADD LANE changed unrelated marking geometry");
    }
  }
  return true;
}

bool add_lane_continues_to_the_operation_direction_terminal(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), first.value);
  const RoadSegment* first_segment = corridor == nullptr
      ? nullptr
      : &state.graph().segments.front();
  ROAD_TEST_EXPECT(corridor != nullptr && first_segment != nullptr,
                   "explicit endpoint fixture corridor is missing");
  const RoadCorridorId corridor_id = corridor->id;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, first_segment->node_b,
          MakePath({MakeLine({60.0, 0.0}, {120.0, 0.0})}), section});
  ROAD_TEST_EXPECT(second.ok, second.error);
  const auto second_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = second.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(second_segment != state.graph().segments.end(),
                   "explicit endpoint fixture second segment is missing");
  const RoadNodeId terminal_node = second_segment->node_b;
  const auto third = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, terminal_node,
          MakePath({MakeLine({120.0, 0.0}, {180.0, 0.0})}), section});
  ROAD_TEST_EXPECT(third.ok, third.error);
  const auto branch = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({120.0, 0.0}, {120.0, 60.0})}), section,
          terminal_node});
  ROAD_TEST_EXPECT(branch.ok, branch.error);

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor_id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.transition_start = {first.value, 0.25};
  request.transition_complete = {first.value, 0.75};
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto find_by_id = [&state](RoadSegmentId id) {
    return std::find_if(state.graph().segments.begin(),
                        state.graph().segments.end(),
                        [id](const RoadSegment& item) { return item.id == id; });
  };
  const auto updated_first = find_by_id(first.value);
  const auto updated_second = find_by_id(second.value);
  const auto updated_third = find_by_id(third.value);
  ROAD_TEST_EXPECT(updated_first->transition.has_value() &&
                       updated_second->layout_template != 1 &&
                       updated_third->layout_template != 1,
                   "ADD LANE did not continue the completed section to the corridor terminal");
  return true;
}

bool add_lane_rejects_taper_across_segment_boundaries(std::string& failure) {
  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), first.value);
  const auto find_segment_by_id = [&state](RoadSegmentId id) {
    const auto item = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [id](const RoadSegment& segment) { return segment.id == id; });
    return item == state.graph().segments.end() ? nullptr : &*item;
  };
  const RoadSegment* first_segment = find_segment_by_id(first.value);
  ROAD_TEST_EXPECT(corridor != nullptr && first_segment != nullptr,
                   "cross-segment lane fixture is incomplete");
  const RoadCorridorId corridor_id = corridor->id;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, first_segment->node_b,
          MakePath({MakeLine({60.0, 0.0}, {120.0, 0.0})}), section});
  ROAD_TEST_EXPECT(second.ok, second.error);
  const RoadSegment* second_segment = find_segment_by_id(second.value);
  ROAD_TEST_EXPECT(second_segment != nullptr,
                   "cross-segment lane second segment is missing");
  city::road::AddLaneRequest request{};
  request.corridor_id = corridor_id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.transition_start = {first.value, 0.5};
  request.transition_complete = {second.value, 0.5};
  request.lane_width_m = 3.0;
  const auto before = state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(!added.ok &&
                       added.failure_category ==
                           CommitFailureCategory::kNotImplemented &&
                       added.reason_code ==
                           "road_add_lane_taper_crosses_segment_boundary" &&
                       added.error.find("taper must stay within one road segment") !=
                           std::string::npos,
                   "ADD LANE accepted a cross-segment taper");
  const auto after = state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                   "rejected cross-segment taper mutated authoritative state");
  return true;
}

bool add_lane_conflict_is_specific_and_atomic(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), segment.value);
  const auto source = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = segment.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(corridor != nullptr && source != state.graph().segments.end(),
                   "conflict fixture is incomplete");
  city::road::AddLaneRequest request{
      corridor->id, city::road::LaneTravelDirection::kAlongSegment,
      city::road::RoadSide::kRight, {segment.value, 0.2},
      {segment.value, 0.6}, 3.0};
  const auto first = state.AddLane(request);
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto before = state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto rejected = state.AddLane(request);
  ROAD_TEST_EXPECT(!rejected.ok &&
                       rejected.failure_category ==
                           CommitFailureCategory::kInvalidInput &&
                       rejected.reason_code ==
                           "road_add_lane_transition_conflict" &&
                       rejected.error.find("already has a section transition") !=
                           std::string::npos,
                   "overlapping transition did not return a specific input error");
  const auto after = state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                   "rejected overlapping transition mutated state");
  return true;
}

bool transitioning_segment_split_respects_transition_bounds(std::string& failure) {
  const auto make_state = [](double split_distance, bool expect_success,
                             bool expect_transition_on_second,
                             std::string& error) {
    RoadState state{};
    const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), shouldered});
    if (!segment.ok) { error = segment.error; return false; }
    const auto* corridor = FindCorridorForSegment(state.graph(), segment.value);
    const auto source = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [id = segment.value](const RoadSegment& item) { return item.id == id; });
    if (corridor == nullptr || source == state.graph().segments.end()) {
      error = "split fixture is incomplete";
      return false;
    }
    city::road::AddLaneRequest request{
        corridor->id, city::road::LaneTravelDirection::kAlongSegment,
        city::road::RoadSide::kRight, {segment.value, 0.2},
        {segment.value, 0.6}, 3.0};
    const auto added = state.AddLane(request);
    if (!added.ok) { error = added.error; return false; }
    const auto before = state.Save();
    const auto split = state.SplitSegmentAtDistance({segment.value, split_distance});
    if (split.ok != expect_success) {
      error = split.ok ? "split unexpectedly succeeded" : split.error;
      return false;
    }
    if (!expect_success) {
      const auto after = state.Save();
      if (!before.ok || !after.ok || before.value != after.value ||
          split.error.find("inside its section transition") == std::string::npos) {
        error = "inside-transition split was not specific and atomic";
        return false;
      }
      return true;
    }
    const auto first = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [id = segment.value](const RoadSegment& item) { return item.id == id; });
    const auto second = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [id = split.value](const RoadSegment& item) { return item.id == id; });
    if (first == state.graph().segments.end() ||
        second == state.graph().segments.end() ||
        first->transition.has_value() == expect_transition_on_second ||
        second->transition.has_value() != expect_transition_on_second) {
      error = "transition was assigned to the wrong split segment";
      return false;
    }
    const city::road::RoadLayoutTransitionId transition_id =
        expect_transition_on_second ? *second->transition
                                    : *first->transition;
    const auto transition = std::find_if(
        state.graph().transitions.begin(), state.graph().transitions.end(),
        [transition_id](const auto& item) { return item.id == transition_id; });
    if (transition == state.graph().transitions.end() ||
        transition->start.kind != DistanceRefKind::kRatio ||
        transition->end.kind != DistanceRefKind::kRatio) {
      error = "split transition lost ratio authority";
      return false;
    }
    const double expected_start = expect_transition_on_second
        ? (0.2 - 0.1) / 0.9 : 0.2 / 0.8;
    const double expected_end = expect_transition_on_second
        ? (0.6 - 0.1) / 0.9 : 0.6 / 0.8;
    if (std::abs(transition->start.value - expected_start) > 1e-9 ||
        std::abs(transition->end.value - expected_end) > 1e-9) {
      error = "split transition t was not re-normalized";
      return false;
    }
    return true;
  };
  ROAD_TEST_EXPECT(make_state(10.0, true, true, failure), failure);
  ROAD_TEST_EXPECT(make_state(80.0, true, false, failure), failure);
  ROAD_TEST_EXPECT(make_state(40.0, false, false, failure), failure);
  return true;
}

bool segment_split_remaps_saved_lane_topology(std::string& failure) {
  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest first_request{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), section};
  first_request.corner_radius_m = 7.0;
  const auto first = state.AddSegment(first_request);
  ROAD_TEST_EXPECT(first.ok, first.error);
  const RoadCorridorId corridor = state.graph().corridors.front().id;
  const RoadNodeId endpoint = state.graph().segments.front().node_b;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor, endpoint,
          MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(second.ok, second.error);
  SavedRoadGraph seeded = state.graph();
  seeded.lane_connections.push_back(city::road::LaneConnection{
      9001, {first.value, 1010, EndpointRole::kEnd},
      {second.value, 1010, EndpointRole::kStart},
      city::road::LaneConnectionKind::kContinuation});
  seeded.boundary_continuations.push_back(city::road::BoundaryContinuation{
      9002, {first.value, 200, EndpointRole::kEnd},
      {second.value, 200, EndpointRole::kStart},
      city::road::BoundaryContinuationKind::kContinuation});
  const auto loaded = load_fixture(seeded);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  RoadState split_state = loaded.value;
  const auto split = split_state.SplitSegmentAtDistance({first.value, 10.0});
  ROAD_TEST_EXPECT(split.ok, split.error);
  const auto end_side = std::find_if(
      split_state.graph().segments.begin(), split_state.graph().segments.end(),
      [id = split.value](const RoadSegment& segment) {
        return segment.id == id;
      });
  ROAD_TEST_EXPECT(end_side != split_state.graph().segments.end(),
                   "split endpoint segment is missing");
  ROAD_TEST_EXPECT(std::abs(end_side->corner_radius_m - 7.0) < 1e-9,
                   "split endpoint segment did not inherit corner radius");
  ROAD_TEST_EXPECT(split_state.graph().lane_connections.size() == 1 &&
                       split_state.graph().lane_connections.front().source.segment_id ==
                           split.value &&
                       split_state.graph().lane_connections.front().source.endpoint_role ==
                           EndpointRole::kEnd &&
                       split_state.graph().lane_connections.front().target.segment_id ==
                           second.value,
                   "split did not remap saved lane topology to the end-side segment");
  ROAD_TEST_EXPECT(
      split_state.graph().boundary_continuations.size() == 1 &&
          split_state.graph().boundary_continuations.front().source.segment_id ==
              split.value &&
          split_state.graph().boundary_continuations.front().source.endpoint_role ==
              EndpointRole::kEnd &&
          split_state.graph().boundary_continuations.front().target.segment_id ==
              second.value,
      "split did not remap saved boundary topology to the end-side segment");

  RoadState delete_state = loaded.value;
  const auto before = delete_state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto deleted = delete_state.DeleteSegment({first.value});
  ROAD_TEST_EXPECT(deleted.ok, deleted.error);
  ROAD_TEST_EXPECT(delete_state.graph().lane_connections.empty() &&
                       delete_state.graph().boundary_continuations.empty(),
                   "segment delete left saved lane or boundary topology orphaned");
  const auto saved = delete_state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  return true;
}

bool add_lane_position_rejections_are_specific_and_atomic(std::string& failure) {
  RoadState state{};
  const auto section =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto selected = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
  const auto other = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 20.0}, {60.0, 20.0})}), section});
  ROAD_TEST_EXPECT(selected.ok && other.ok, "ADD LANE position fixture failed");
  const auto* corridor = FindCorridorForSegment(state.graph(), selected.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "selected ADD LANE corridor is missing");
  city::road::AddLaneRequest request{
      corridor->id, city::road::LaneTravelDirection::kAlongSegment,
      city::road::RoadSide::kRight, {selected.value, 0.25},
      {selected.value, 0.25}, 3.0};
  const auto before = state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto identical = state.AddLane(request);
  ROAD_TEST_EXPECT(!identical.ok &&
                       identical.reason_code ==
                           "road_add_lane_positions_identical",
                   "identical ADD LANE positions lacked the Core reason");
  request.transition_complete = {other.value, 0.5};
  const auto outside = state.AddLane(request);
  ROAD_TEST_EXPECT(!outside.ok &&
                       outside.reason_code ==
                           "road_add_lane_position_not_on_selected_corridor",
                   "outside-corridor ADD LANE position lacked the Core reason");
  const auto after = state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                   "rejected ADD LANE positions mutated authoritative state");
  return true;
}

struct SplitSemanticsFixture {
  RoadState state{};
  RoadSegmentId source_id = 0;
  RoadNodeId old_end_node_id = 0;
  RoadNodeId connector_target_node_id = 0;
};

city::road::Result<SplitSemanticsFixture> make_split_semantics_fixture(
    bool create_connector_target_first, bool crossing_owner = false) {
  RoadState authored{};
  const auto section = road_fixture::AddLayout(
      authored, road_fixture::BidirectionalLayout(0));
  if (section == 0) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics fixture layout is missing");
  }
  city::road::Result<RoadSegmentId> connector_target{};
  if (create_connector_target_first) {
    connector_target = authored.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({40.0, 50.0}, {80.0, 50.0})}), section});
  }
  city::road::AddSegmentRequest source_request{
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), section};
  source_request.start_elevation_m = 3.0;
  source_request.end_elevation_m = 13.0;
  source_request.corner_radius_m = 7.0;
  const auto source = authored.AddSegment(source_request);
  if (!source.ok) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        source.failure_category, source.error);
  }
  if (!create_connector_target_first) {
    connector_target = authored.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({40.0, 50.0}, {80.0, 50.0})}), section});
  }
  if (!connector_target.ok) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        connector_target.failure_category, connector_target.error);
  }
  const auto source_segment_it = std::find_if(
      authored.graph().segments.begin(), authored.graph().segments.end(),
      [id = source.value](const RoadSegment& item) { return item.id == id; });
  const RoadSegment* source_segment =
      source_segment_it == authored.graph().segments.end()
          ? nullptr
          : &*source_segment_it;
  const auto* source_corridor =
      FindCorridorForSegment(authored.graph(), source.value);
  if (source_segment == nullptr || source_corridor == nullptr) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics fixture source is incomplete");
  }
  const RoadNodeId old_end_node_id = source_segment->node_b;
  const RoadCorridorId corridor_id = source_corridor->id;
  const auto tail = authored.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, old_end_node_id,
          MakePath({MakeLine({100.0, 0.0}, {140.0, 0.0})}), section});
  if (!tail.ok) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        tail.failure_category, tail.error);
  }
  const auto old_end_branch = authored.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({100.0, 0.0}, {100.0, 40.0})}), section,
          old_end_node_id});
  if (!old_end_branch.ok) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        old_end_branch.failure_category, old_end_branch.error);
  }
  const auto connector_segment_it = std::find_if(
      authored.graph().segments.begin(), authored.graph().segments.end(),
      [id = connector_target.value](const RoadSegment& item) {
        return item.id == id;
      });
  const RoadSegment* connector_segment =
      connector_segment_it == authored.graph().segments.end()
          ? nullptr
          : &*connector_segment_it;
  if (connector_segment == nullptr) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics connector target is missing");
  }

  SavedRoadGraph graph = authored.graph();
  auto corridor = std::find_if(
      graph.corridors.begin(), graph.corridors.end(),
      [corridor_id](const auto& value) { return value.id == corridor_id; });
  if (corridor == graph.corridors.end()) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics corridor is missing");
  }
  corridor->segments = {{tail.value, true}, {source.value, true}};

  graph.manual_lines.push_back(city::road::ManualLineMarking{
      9001, source.value,
      MakePath({MakeLine(crossing_owner ? Vec2d{35.0, 0.4}
                                         : Vec2d{5.0, 0.4},
                         crossing_owner ? Vec2d{45.0, 0.4}
                                         : Vec2d{15.0, 0.4})}),
      builtin_marking_styles::kWhiteSolid});
  graph.manual_lines.push_back(city::road::ManualLineMarking{
      9002, source.value, MakePath({MakeLine({60.0, 0.6}, {70.0, 0.6})}),
      builtin_marking_styles::kWhiteSolid});
  graph.manual_areas.push_back(city::road::ManualAreaMarking{
      9003, source.value, {20.0, 0.0}, 0.0, 2.0, 5.0,
      builtin_marking_styles::kCrosswalk});
  graph.manual_areas.push_back(city::road::ManualAreaMarking{
      9004, source.value, crossing_owner ? Vec2d{38.0, 0.0}
                                          : Vec2d{75.0, 0.0},
      0.0, 2.0, 5.0, builtin_marking_styles::kCrosswalk});
  graph.auto_marking_overrides.push_back(city::road::AutoMarkingOverride{
      AutoMarkingKey{
          MarkingOwner{MarkingOwner::Kind::kRoadSegment, source.value, 0, 0},
          MarkingRole::kCenterLine,
          MarkingTrackKey{source.value, 200, MarkingRole::kCenterLine},
          std::nullopt},
      true});
  graph.approach_geometry_overrides.push_back(
      city::road::ApproachGeometryOverride{
          {old_end_node_id, source.value, EndpointRole::kEnd},
          city::road::ManualDoubleOverride{true, 1.0}, {}});
  graph.lane_connections.push_back(city::road::LaneConnection{
      9005, {source.value, 1010, EndpointRole::kEnd},
      {tail.value, 1010, EndpointRole::kStart},
      city::road::LaneConnectionKind::kContinuation});
  graph.boundary_continuations.push_back(city::road::BoundaryContinuation{
      9006, {source.value, 200, EndpointRole::kEnd},
      {tail.value, 200, EndpointRole::kStart},
      city::road::BoundaryContinuationKind::kContinuation});
  graph.junction_marking_overrides.push_back(JunctionMarkingOverride{
      9007, old_end_node_id,
      {{old_end_node_id, source.value, EndpointRole::kEnd}, 200,
       MarkingRole::kCenterLine},
      JunctionMarkingAction::kConnectToApproach,
      JunctionMarkingEndpoint{
          {old_end_node_id, tail.value, EndpointRole::kStart}, 200,
          MarkingRole::kCenterLine}});

  const auto loaded = load_fixture(graph);
  if (!loaded.ok) {
    return city::road::Result<SplitSemanticsFixture>::Fail(
        loaded.failure_category, loaded.error);
  }
  return city::road::Result<SplitSemanticsFixture>::Ok(
      SplitSemanticsFixture{loaded.value, source.value, old_end_node_id,
                            connector_segment->node_a});
}

city::road::Result<std::string> normalized_split_semantics(
    const SplitSemanticsFixture& fixture) {
  const SavedRoadGraph& graph = fixture.state.graph();
  const auto first_it = std::find_if(
      graph.segments.begin(), graph.segments.end(),
      [id = fixture.source_id](const RoadSegment& item) {
        return item.id == id;
      });
  const RoadSegment* first =
      first_it == graph.segments.end() ? nullptr : &*first_it;
  if (first == nullptr) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics retained segment is missing");
  }
  const auto second = std::find_if(
      graph.segments.begin(), graph.segments.end(), [&](const RoadSegment& item) {
        return item.id != fixture.source_id &&
               item.node_b == fixture.old_end_node_id;
      });
  if (second == graph.segments.end() || first->node_b != second->node_a) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics end-side segment is missing");
  }
  const auto split_node = std::find_if(
      graph.nodes.begin(), graph.nodes.end(),
      [id = first->node_b](const auto& item) { return item.id == id; });
  const Path* second_path =
      FindCanonicalAlignment(fixture.state.derived(), second->id);
  const RoadCorridor* corridor =
      FindCorridorForSegment(graph, fixture.source_id);
  if (split_node == graph.nodes.end() || second_path == nullptr ||
      corridor == nullptr) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics derived state is incomplete");
  }
  if (std::abs(split_node->position.x - 40.0) > 1e-6 ||
      std::abs(split_node->position.y) > 1e-9 ||
      std::abs(split_node->elevation_m - 7.0) > 1e-9 ||
      std::abs(second_path->spans.front().p0.x - 40.0) > 1e-6 ||
      std::abs(second_path->spans.front().p0.y) > 1e-9 ||
      std::abs(second_path->spans.back().p3.x - 100.0) > 1e-9 ||
      std::abs(second_path->spans.back().p3.y) > 1e-9) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "split semantics position, elevation, or second alignment drifted");
  }
  const auto segment_label = [&](RoadSegmentId id) {
    if (id == fixture.source_id) return std::string{"first"};
    if (id == second->id) return std::string{"second"};
    return std::string{"other"};
  };
  std::ostringstream out;
  out << std::setprecision(17) << "split=" << split_node->position.x << ','
      << split_node->position.y << ',' << split_node->elevation_m
      << ";second_path=" << second_path->spans.front().p0.x << ','
      << second_path->spans.front().p0.y << '>'
      << second_path->spans.back().p3.x << ','
      << second_path->spans.back().p3.y << ";layout=" << first->layout_template << ','
      << second->layout_template << ";radius=" << first->corner_radius_m << ','
      << second->corner_radius_m << ";corridor=";
  for (const auto& ref : corridor->segments) {
    out << segment_label(ref.segment_id) << ':' << ref.reversed << ',';
  }
  out << ";manual_lines=";
  for (const auto& marking : graph.manual_lines) {
    if (marking.id == 9001 || marking.id == 9002) {
      out << marking.id << ':' << segment_label(marking.owner_segment_id)
          << ':' << marking.path.spans.front().p0.x << ',';
    }
  }
  out << ";manual_areas=";
  for (const auto& marking : graph.manual_areas) {
    if (marking.id == 9003 || marking.id == 9004) {
      out << marking.id << ':' << segment_label(marking.owner_segment_id)
          << ':' << marking.frame_origin.x << ',';
    }
  }
  out << ";approach=";
  for (const auto& value : graph.approach_geometry_overrides) {
    if (value.key.node_id == fixture.old_end_node_id) {
      out << segment_label(value.key.segment_id) << ':'
          << static_cast<int>(value.key.endpoint_role) << ',';
    }
  }
  out << ";auto=";
  for (const auto& value : graph.auto_marking_overrides) {
    if (value.key.owner.kind == MarkingOwner::Kind::kRoadSegment &&
        (value.key.owner.segment_id == fixture.source_id ||
         value.key.owner.segment_id == second->id)) {
      out << segment_label(value.key.owner.segment_id) << ':'
          << segment_label(value.key.track->segment_id) << ',';
    }
  }
  out << ";junction=";
  for (const auto& value : graph.junction_marking_overrides) {
    if (value.id == 9007) {
      out << segment_label(value.source.approach.segment_id) << ':'
          << (value.target.has_value()
                  ? segment_label(value.target->approach.segment_id)
                  : "none");
    }
  }
  out << ";lane=";
  for (const auto& value : graph.lane_connections) {
    if (value.id == 9005) {
      out << segment_label(value.source.segment_id) << ':'
          << static_cast<int>(value.source.endpoint_role) << ">"
          << segment_label(value.target.segment_id) << ':'
          << static_cast<int>(value.target.endpoint_role);
    }
  }
  out << ";boundary=";
  for (const auto& value : graph.boundary_continuations) {
    if (value.id == 9006) {
      out << segment_label(value.source.segment_id) << ':'
          << static_cast<int>(value.source.endpoint_role) << ">"
          << segment_label(value.target.segment_id) << ':'
          << static_cast<int>(value.target.endpoint_role);
    }
  }
  return city::road::Result<std::string>::Ok(out.str());
}

bool split_public_operations_share_normalized_semantics(std::string& failure) {
  const auto run = [](int operation, bool target_first) {
    auto fixture = make_split_semantics_fixture(target_first);
    if (!fixture.ok) {
      return city::road::Result<std::string>::Fail(
          fixture.failure_category, fixture.error);
    }
    city::road::Result<RoadSegmentId> result{};
    if (operation == 0) {
      result = fixture.value.state.SplitSegmentAtDistance(
          {fixture.value.source_id, 40.0});
    } else if (operation == 1) {
      result = fixture.value.state.AddSegmentConnectedToSegment(
          city::road::AddSegmentConnectedToSegmentRequest{
              MakePath({MakeLine({40.0, 0.0}, {40.0, -50.0})}),
              fixture.value.state.graph().layout_templates.front().id,
              fixture.value.source_id, 40.0});
    } else {
      result = fixture.value.state.AddSegmentBetween(
          city::road::AddSegmentBetweenRequest{
              MakePath({MakeLine({40.0, 0.0}, {40.0, 50.0})}),
              fixture.value.state.graph().layout_templates.front().id,
              {0, fixture.value.source_id, 40.0},
              {fixture.value.connector_target_node_id, 0, 0.0}});
    }
    if (!result.ok) {
      return city::road::Result<std::string>::Fail(
          result.failure_category, result.error);
    }
    return normalized_split_semantics(fixture.value);
  };

  const auto direct = run(0, false);
  const auto branch = run(1, false);
  const auto between = run(2, false);
  const auto reordered = run(0, true);
  ROAD_TEST_EXPECT(direct.ok, direct.error);
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(between.ok, between.error);
  ROAD_TEST_EXPECT(reordered.ok, reordered.error);
  ROAD_TEST_EXPECT(direct.value == branch.value &&
                       direct.value == between.value,
                   "public operations drifted in normalized split semantics\n" +
                       direct.value + "\n" + branch.value + "\n" +
                       between.value);
  ROAD_TEST_EXPECT(direct.value == reordered.value,
                   "split semantics depend on creation order or source IDs\n" +
                       direct.value + "\n" + reordered.value);
  const std::array<std::string_view, 11> required{
      "layout=1,1", "radius=7,7",
      "corridor=other:1,second:1,first:1", "9001:first:5",
      "9002:second:20", "9003:first:20", "9004:second:35",
      "approach=second:1", "auto=first:first,second:second",
      "junction=second:other", "lane=second:1>other:0;boundary=second:1>other:0"};
  for (const std::string_view expected : required) {
    ROAD_TEST_EXPECT(direct.value.find(expected) != std::string::npos,
                     "normalized split snapshot is missing " +
                         std::string(expected) + "\n" + direct.value);
  }
  return true;
}

bool split_crossing_owners_are_rejected_atomically(std::string& failure) {
  for (int operation = 0; operation != 3; ++operation) {
    auto fixture = make_split_semantics_fixture(false, true);
    ROAD_TEST_EXPECT(fixture.ok, fixture.error);
    const auto before = fixture.value.state.Save();
    ROAD_TEST_EXPECT(before.ok, before.error);
    city::road::Result<RoadSegmentId> result{};
    if (operation == 0) {
      result = fixture.value.state.SplitSegmentAtDistance(
          {fixture.value.source_id, 40.0});
    } else if (operation == 1) {
      result = fixture.value.state.AddSegmentConnectedToSegment(
          {MakePath({MakeLine({40.0, 0.0}, {40.0, -50.0})}),
           fixture.value.state.graph().layout_templates.front().id,
           fixture.value.source_id, 40.0});
    } else {
      result = fixture.value.state.AddSegmentBetween(
          {MakePath({MakeLine({40.0, 0.0}, {40.0, 50.0})}),
           fixture.value.state.graph().layout_templates.front().id,
           {0, fixture.value.source_id, 40.0},
           {fixture.value.connector_target_node_id, 0, 0.0}});
    }
    ROAD_TEST_EXPECT(!result.ok &&
                         result.failure_category ==
                             CommitFailureCategory::kNotImplemented,
                     "crossing split owner was not explicitly rejected");
    const auto after = fixture.value.state.Save();
    ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                     "rejected crossing split mutated authoritative state");
  }
  return true;
}

bool split_semantics_have_one_internal_planner(std::string& failure) {
  const std::filesystem::path source =
      std::filesystem::current_path() / "domains" / "road" / "src" /
      "operations" / "create.cpp";
  if (!std::filesystem::exists(source)) return true;
  std::ifstream in(source);
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
  const auto occurrences = [&text](std::string_view needle) {
    std::size_t count = 0;
    for (std::size_t at = text.find(needle); at != std::string::npos;
         at = text.find(needle, at + needle.size())) {
      ++count;
    }
    return count;
  };
  ROAD_TEST_EXPECT(occurrences("split_path_at_distance(") == 1,
                   "Road split primitive has more than one operation owner");
  ROAD_TEST_EXPECT(occurrences("plan_segment_split(") == 4,
                   "the three public split operations do not share one planner");
  return true;
}

bool connection_splits_reject_transitioning_sources(std::string& failure) {
  for (int operation = 0; operation != 2; ++operation) {
    RoadState state{};
    const auto section = road_fixture::AddLayout(
        state, road_fixture::ShoulderedLayout(0));
    const auto source = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), section});
    const auto target = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({10.0, 40.0}, {50.0, 40.0})}), section});
    ROAD_TEST_EXPECT(source.ok && target.ok,
                     source.ok ? target.error : source.error);
    const RoadCorridor* corridor =
        FindCorridorForSegment(state.graph(), source.value);
    const auto target_segment = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [id = target.value](const RoadSegment& item) { return item.id == id; });
    ROAD_TEST_EXPECT(corridor != nullptr &&
                         target_segment != state.graph().segments.end(),
                     "transition rejection fixture is incomplete");
    const RoadCorridorId corridor_id = corridor->id;
    const RoadNodeId target_node_id = target_segment->node_a;
    const auto lane_added = state.AddLane(city::road::AddLaneRequest{
        corridor_id, city::road::LaneTravelDirection::kAlongSegment,
        city::road::RoadSide::kRight, {source.value, 0.2},
        {source.value, 0.6}, 3.0});
    ROAD_TEST_EXPECT(lane_added.ok, lane_added.error);
    const auto before = state.Save();
    ROAD_TEST_EXPECT(before.ok, before.error);
    city::road::Result<RoadSegmentId> result{};
    if (operation == 0) {
      result = state.AddSegmentConnectedToSegment(
          {MakePath({MakeLine({10.0, 0.0}, {10.0, -40.0})}), section,
           source.value, 10.0});
    } else {
      result = state.AddSegmentBetween(
          {MakePath({MakeLine({10.0, 0.0}, {10.0, 40.0})}), section,
           {0, source.value, 10.0}, {target_node_id, 0, 0.0}});
    }
    ROAD_TEST_EXPECT(!result.ok &&
                         result.failure_category ==
                             CommitFailureCategory::kNotImplemented &&
                         result.error.find("transitioning") != std::string::npos,
                     "connection split did not preserve transition rejection policy");
    const auto after = state.Save();
    ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                     "rejected transition connection mutated authoritative state");
  }
  return true;
}

struct MultiSplitRelationFixture {
  RoadState state{};
  RoadLayoutTemplateId section_id = 0;
  RoadSegmentId segment_a = 0;
  RoadSegmentId segment_b = 0;
  RoadNodeId common_end_node = 0;
};

city::road::Result<MultiSplitRelationFixture>
make_multi_split_relation_fixture() {
  RoadState authored{};
  const RoadLayoutTemplateId section = road_fixture::AddLayout(
      authored, road_fixture::BidirectionalLayout(0));
  const auto a = authored.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-100.0, 0.0}, {0.0, 0.0})}), section});
  const auto b = authored.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, -100.0}, {0.0, 0.0})}), section});
  if (!a.ok || !b.ok) {
    return city::road::Result<MultiSplitRelationFixture>::Fail(
        a.ok ? b.failure_category : a.failure_category,
        a.ok ? b.error : a.error);
  }
  const auto a_segment = std::find_if(
      authored.graph().segments.begin(), authored.graph().segments.end(),
      [id = a.value](const RoadSegment& item) { return item.id == id; });
  const auto b_segment = std::find_if(
      authored.graph().segments.begin(), authored.graph().segments.end(),
      [id = b.value](const RoadSegment& item) { return item.id == id; });
  if (a_segment == authored.graph().segments.end() ||
      b_segment == authored.graph().segments.end()) {
    return city::road::Result<MultiSplitRelationFixture>::Fail(
        CommitFailureCategory::kInternalError,
        "multi-split relation fixture segments are missing");
  }
  const RoadNodeId common_end = a_segment->node_b;
  const RoadNodeId duplicate_b_end = b_segment->node_b;
  const auto third = authored.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {70.0, 70.0})}), section,
          common_end});
  if (!third.ok) {
    return city::road::Result<MultiSplitRelationFixture>::Fail(
        third.failure_category, third.error);
  }

  SavedRoadGraph graph = authored.graph();
  const auto graph_b = std::find_if(
      graph.segments.begin(), graph.segments.end(),
      [id = b.value](const RoadSegment& item) { return item.id == id; });
  if (graph_b == graph.segments.end()) {
    return city::road::Result<MultiSplitRelationFixture>::Fail(
        CommitFailureCategory::kInternalError,
        "multi-split relation fixture B is missing");
  }
  graph_b->node_b = common_end;
  graph.nodes.erase(
      std::remove_if(graph.nodes.begin(), graph.nodes.end(),
                     [duplicate_b_end](const auto& node) {
                       return node.id == duplicate_b_end;
                     }),
      graph.nodes.end());
  graph.lane_connections.push_back(city::road::LaneConnection{
      9101, {a.value, 1010, EndpointRole::kEnd},
      {b.value, 1000, EndpointRole::kEnd},
      city::road::LaneConnectionKind::kJunctionMovement});
  graph.boundary_continuations.push_back(city::road::BoundaryContinuation{
      9102, {a.value, 200, EndpointRole::kEnd},
      {b.value, 200, EndpointRole::kEnd},
      city::road::BoundaryContinuationKind::kContinuation});
  graph.junction_marking_overrides.push_back(JunctionMarkingOverride{
      9103, common_end,
      {{common_end, a.value, EndpointRole::kEnd}, 200,
       MarkingRole::kCenterLine},
      JunctionMarkingAction::kConnectToApproach,
      JunctionMarkingEndpoint{
          {common_end, b.value, EndpointRole::kEnd}, 200,
          MarkingRole::kCenterLine}});
  const auto loaded = load_fixture(graph);
  if (!loaded.ok) {
    return city::road::Result<MultiSplitRelationFixture>::Fail(
        loaded.failure_category, loaded.error);
  }
  return city::road::Result<MultiSplitRelationFixture>::Ok(
      {loaded.value, section, a.value, b.value, common_end});
}

city::road::Result<std::string> observe_multi_split_relations(
    const MultiSplitRelationFixture& fixture) {
  const SavedRoadGraph& graph = fixture.state.graph();
  if (graph.lane_connections.size() != 1 ||
      graph.boundary_continuations.size() != 1 ||
      graph.junction_marking_overrides.size() != 1) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "multi-split relations are not exactly one each");
  }
  const auto& lane = graph.lane_connections.front();
  const auto& boundary = graph.boundary_continuations.front();
  const auto& marking = graph.junction_marking_overrides.front();
  const RoadSegmentId a2_id = lane.source.segment_id;
  const RoadSegmentId b2_id = lane.target.segment_id;
  if (lane.id != 9101 || boundary.id != 9102 || marking.id != 9103 ||
      a2_id == fixture.segment_a || b2_id == fixture.segment_b ||
      lane.source != city::road::LaneEndpointKey{
                         a2_id, 1010, EndpointRole::kEnd} ||
      lane.target != city::road::LaneEndpointKey{
                         b2_id, 1000, EndpointRole::kEnd} ||
      boundary.source != city::road::BoundaryEndpointKey{
                              a2_id, 200, EndpointRole::kEnd} ||
      boundary.target != city::road::BoundaryEndpointKey{
                              b2_id, 200, EndpointRole::kEnd} ||
      marking.source.approach != city::road::ApproachKey{
                                     fixture.common_end_node, a2_id,
                                     EndpointRole::kEnd} ||
      !marking.target.has_value() ||
      marking.target->approach != city::road::ApproachKey{
                                      fixture.common_end_node, b2_id,
                                      EndpointRole::kEnd}) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "multi-split endpoint substitutions were not composed");
  }
  const auto segment = [&](RoadSegmentId id) {
    return std::find_if(graph.segments.begin(), graph.segments.end(),
                        [id](const RoadSegment& item) {
                          return item.id == id;
                        });
  };
  const auto node = [&](RoadNodeId id) {
    return std::find_if(graph.nodes.begin(), graph.nodes.end(),
                        [id](const auto& item) { return item.id == id; });
  };
  const auto a1 = segment(fixture.segment_a);
  const auto b1 = segment(fixture.segment_b);
  const auto a2 = segment(a2_id);
  const auto b2 = segment(b2_id);
  if (a1 == graph.segments.end() || b1 == graph.segments.end() ||
      a2 == graph.segments.end() || b2 == graph.segments.end() ||
      a1->node_b == fixture.common_end_node ||
      b1->node_b == fixture.common_end_node ||
      a2->node_a != a1->node_b || b2->node_a != b1->node_b ||
      a2->node_b != fixture.common_end_node ||
      b2->node_b != fixture.common_end_node) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "multi-split segment identity or half ownership drifted");
  }
  const auto a_split = node(a1->node_b);
  const auto b_split = node(b1->node_b);
  if (a_split == graph.nodes.end() || b_split == graph.nodes.end() ||
      std::abs(a_split->position.x + 50.0) > 1e-6 ||
      std::abs(a_split->position.y) > 1e-9 ||
      std::abs(b_split->position.x) > 1e-9 ||
      std::abs(b_split->position.y + 50.0) > 1e-6) {
    return city::road::Result<std::string>::Fail(
        CommitFailureCategory::kInternalError,
        "multi-split positions drifted");
  }
  const auto invariant =
      ValidateGraphInvariants(graph, fixture.state.derived());
  const auto lane_path = std::find_if(
      fixture.state.derived().lane_paths.begin(),
      fixture.state.derived().lane_paths.end(),
      [](const auto& path) { return path.connection_id == 9101; });
  const auto boundary_path = std::find_if(
      fixture.state.derived().boundary_paths.begin(),
      fixture.state.derived().boundary_paths.end(),
      [](const auto& path) { return path.continuation_id == 9102; });
  const bool junction_marking_derived =
      0 < road_test_view::count_marking_lines(
              fixture.state.derived(), [&fixture](const auto& path) {
                return path.owner.kind == MarkingOwner::Kind::kJunction &&
                       path.owner.node_id == fixture.common_end_node &&
                       path.role == MarkingRole::kCenterLine;
              });
  if (!invariant.ok || lane_path == fixture.state.derived().lane_paths.end() ||
      boundary_path == fixture.state.derived().boundary_paths.end() ||
      !junction_marking_derived) {
    return city::road::Result<std::string>::Fail(
        invariant.ok ? CommitFailureCategory::kInternalError
                     : invariant.failure_category,
        invariant.ok ? "multi-split derived topology is missing"
                     : invariant.error);
  }
  return city::road::Result<std::string>::Ok(
      "lane=A2.End>B2.End;boundary=A2.End>B2.End;marking=A2.End>B2.End");
}

bool multi_split_composes_cross_source_relations(std::string& failure) {
  const auto run = [](bool reverse_request) {
    auto fixture = make_multi_split_relation_fixture();
    if (!fixture.ok) {
      return city::road::Result<std::string>::Fail(
          fixture.failure_category, fixture.error);
    }
    const auto connected = fixture.value.state.AddSegmentBetween(
        city::road::AddSegmentBetweenRequest{
            reverse_request
                ? MakePath({MakeLine({0.0, -50.0}, {-50.0, 0.0})})
                : MakePath({MakeLine({-50.0, 0.0}, {0.0, -50.0})}),
            fixture.value.section_id,
            reverse_request
                ? city::road::RoadConnectionTarget{
                      0, fixture.value.segment_b, 50.0}
                : city::road::RoadConnectionTarget{
                      0, fixture.value.segment_a, 50.0},
            reverse_request
                ? city::road::RoadConnectionTarget{
                      0, fixture.value.segment_a, 50.0}
                : city::road::RoadConnectionTarget{
                      0, fixture.value.segment_b, 50.0}});
    if (!connected.ok) {
      return city::road::Result<std::string>::Fail(
          connected.failure_category, connected.error);
    }
    const auto observed = observe_multi_split_relations(fixture.value);
    if (!observed.ok) return observed;
    const auto saved = fixture.value.state.Save();
    if (!saved.ok) {
      return city::road::Result<std::string>::Fail(
          saved.failure_category, saved.error);
    }
    const auto loaded = RoadState::Load(saved.value);
    if (!loaded.ok) {
      return city::road::Result<std::string>::Fail(
          loaded.failure_category, loaded.error);
    }
    MultiSplitRelationFixture reloaded{
        loaded.value, fixture.value.section_id, fixture.value.segment_a,
        fixture.value.segment_b, fixture.value.common_end_node};
    const auto round_trip = observe_multi_split_relations(reloaded);
    if (!round_trip.ok) return round_trip;
    if (round_trip.value != observed.value) {
      return city::road::Result<std::string>::Fail(
          CommitFailureCategory::kInternalError,
          "multi-split relation semantics changed after save/load");
    }
    return observed;
  };

  const auto forward = run(false);
  const auto reverse = run(true);
  ROAD_TEST_EXPECT(forward.ok, forward.error);
  ROAD_TEST_EXPECT(reverse.ok, reverse.error);
  ROAD_TEST_EXPECT(forward.value == reverse.value,
                   "multi-split relation semantics depend on split order");
  return true;
}

bool multi_split_second_target_failure_is_atomic(std::string& failure) {
  auto fixture = make_multi_split_relation_fixture();
  ROAD_TEST_EXPECT(fixture.ok, fixture.error);
  const auto before = fixture.value.state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto rejected = fixture.value.state.AddSegmentBetween(
      city::road::AddSegmentBetweenRequest{
          MakePath({MakeLine({-50.0, 0.0}, {0.0, -50.0})}),
          fixture.value.section_id,
          {0, fixture.value.segment_a, 50.0},
          {0, fixture.value.segment_b, 1000.0}});
  ROAD_TEST_EXPECT(!rejected.ok,
                   "multi-split invalid second target was accepted");
  const auto after = fixture.value.state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                   "multi-split failure retained a split or relation remap");
  return true;
}

bool add_lane_propagates_from_middle_corridor_segment(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto first_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&first](const auto& segment) { return segment.id == first.value; });
  ROAD_TEST_EXPECT(first_segment != state.graph().segments.end(),
                   "ADD1 first segment is missing");
  const auto* corridor = FindCorridorForSegment(state.graph(), first.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "ADD1 corridor is missing");
  const city::road::RoadCorridorId corridor_id = corridor->id;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, first_segment->node_b,
          MakePath({MakeLine({80.0, 0.0}, {160.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(second.ok, second.error);
  const auto second_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&second](const auto& segment) { return segment.id == second.value; });
  ROAD_TEST_EXPECT(second_segment != state.graph().segments.end(),
                   "ADD1 second segment is missing");
  const auto third = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, second_segment->node_b,
          MakePath({MakeLine({160.0, 0.0}, {240.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(third.ok, third.error);

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor_id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);

  const auto segment_by_id = [&state](city::road::RoadSegmentId id) {
    return std::find_if(state.graph().segments.begin(),
                        state.graph().segments.end(),
                        [id](const auto& segment) { return segment.id == id; });
  };
  const auto updated_first = segment_by_id(first.value);
  const auto updated_second = segment_by_id(second.value);
  const auto updated_third = segment_by_id(third.value);
  const auto* updated_corridor =
      FindCorridorForSegment(state.graph(), first.value);
  ROAD_TEST_EXPECT(
      updated_corridor != nullptr &&
          std::any_of(updated_corridor->segments.begin(),
                      updated_corridor->segments.end(),
                      [&state](const auto& ref) {
                        const auto segment = std::find_if(
                            state.graph().segments.begin(),
                            state.graph().segments.end(),
                            [&ref](const auto& item) {
                              return item.id == ref.segment_id;
                            });
                        return segment != state.graph().segments.end() &&
                               segment->transition.has_value();
                      }),
      "ADD1 taper segment has no transition");
  ROAD_TEST_EXPECT(updated_second->layout_template ==
                       updated_third->layout_template &&
                       updated_second->layout_template != 5,
                   "ADD1 added section did not propagate to following segments");
  ROAD_TEST_EXPECT(updated_corridor != nullptr &&
                       updated_corridor->layout_template_id ==
                           updated_third->layout_template,
                   "ADD1 corridor terminal section was not updated");
  const auto target = std::find_if(
      state.graph().layout_templates.begin(),
      state.graph().layout_templates.end(),
      [updated_second](const auto& section) {
        return section.id == updated_second->layout_template;
      });
  ROAD_TEST_EXPECT(target != state.graph().layout_templates.end() &&
                       target->lane_bands.size() == 3 &&
                       std::any_of(target->lane_bands.begin(),
                                   target->lane_bands.end(),
                                   [](const auto& lane) {
                                     return lane.id == 1000;
                                   }) &&
                       std::any_of(target->lane_bands.begin(),
                                   target->lane_bands.end(),
                                   [](const auto& lane) {
                                     return lane.id == 1010;
                                   }),
                   "ADD1 did not preserve existing lane identity");
  return true;
}

bool add_lane_degree_two_terminal_preserves_existing_identity(
    std::string& failure) {
  struct Scenario {
    const char* name;
    bool operation_forward;
    bool reversed_ref;
    city::road::RoadSide side;
    bool seed_claimed_relations;
  };
  const std::array scenarios{
      Scenario{"forward", true, false, city::road::RoadSide::kRight, false},
      Scenario{"backward", false, false, city::road::RoadSide::kRight, false},
      Scenario{"reversed_ref", true, true, city::road::RoadSide::kRight,
               false},
      Scenario{"left_side", true, false, city::road::RoadSide::kLeft, false},
      Scenario{"claimed", true, false, city::road::RoadSide::kRight, true},
  };
  const auto topology_observation = [](const SavedRoadGraph& graph) {
    std::vector<std::tuple<std::uint64_t, RoadSegmentId, LaneId, int,
                           RoadSegmentId, LaneId, int, int>> lanes{};
    for (const auto& relation : graph.lane_connections) {
      lanes.emplace_back(
          relation.id, relation.source.segment_id, relation.source.lane_id,
          static_cast<int>(relation.source.endpoint_role),
          relation.target.segment_id, relation.target.lane_id,
          static_cast<int>(relation.target.endpoint_role),
          static_cast<int>(relation.kind));
    }
    std::vector<std::tuple<std::uint64_t, RoadSegmentId, BoundaryId, int,
                           RoadSegmentId, BoundaryId, int, int>> boundaries{};
    for (const auto& relation : graph.boundary_continuations) {
      boundaries.emplace_back(
          relation.id, relation.source.segment_id,
          relation.source.boundary_id,
          static_cast<int>(relation.source.endpoint_role),
          relation.target.segment_id, relation.target.boundary_id,
          static_cast<int>(relation.target.endpoint_role),
          static_cast<int>(relation.kind));
    }
    std::sort(lanes.begin(), lanes.end());
    std::sort(boundaries.begin(), boundaries.end());
    return std::pair{std::move(lanes), std::move(boundaries)};
  };

  for (const Scenario& scenario : scenarios) {
    RoadState authored{};
    const auto section = road_fixture::AddLayout(
        authored, road_fixture::BidirectionalLayout(0));
    const auto source = authored.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), section});
    ROAD_TEST_EXPECT(source.ok,
                     std::string(scenario.name) + ": " + source.error);
    const auto source_segment = std::find_if(
        authored.graph().segments.begin(), authored.graph().segments.end(),
        [&source](const auto& segment) { return segment.id == source.value; });
    ROAD_TEST_EXPECT(source_segment != authored.graph().segments.end(),
                     std::string(scenario.name) +
                         ": source segment is missing");
    const EndpointRole source_role =
        scenario.operation_forward
            ? (scenario.reversed_ref ? EndpointRole::kStart
                                     : EndpointRole::kEnd)
            : (scenario.reversed_ref ? EndpointRole::kEnd
                                     : EndpointRole::kStart);
    const EndpointRole target_role = source_role == EndpointRole::kStart
                                         ? EndpointRole::kEnd
                                         : EndpointRole::kStart;
    const RoadNodeId terminal_node = source_role == EndpointRole::kStart
                                         ? source_segment->node_a
                                         : source_segment->node_b;
    const auto node = std::find_if(
        authored.graph().nodes.begin(), authored.graph().nodes.end(),
        [terminal_node](const auto& item) { return item.id == terminal_node; });
    ROAD_TEST_EXPECT(node != authored.graph().nodes.end(),
                     std::string(scenario.name) +
                         ": terminal node is missing");
    const Vec2d before = {node->position.x - 80.0, node->position.y};
    const Vec2d after = {node->position.x + 80.0, node->position.y};
    const auto target = authored.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine(target_role == EndpointRole::kStart
                                   ? node->position
                                   : before,
                               target_role == EndpointRole::kStart
                                   ? after
                                   : node->position)}),
            section, terminal_node, target_role});
    ROAD_TEST_EXPECT(target.ok,
                     std::string(scenario.name) + ": " + target.error);
    const auto* authored_corridor =
        FindCorridorForSegment(authored.graph(), source.value);
    ROAD_TEST_EXPECT(authored_corridor != nullptr &&
                         authored_corridor->segments.size() == 1,
                     std::string(scenario.name) +
                         ": source corridor is missing");

    SavedRoadGraph seeded = authored.graph();
    auto corridor = std::find_if(
        seeded.corridors.begin(), seeded.corridors.end(),
        [id = authored_corridor->id](const auto& item) {
          return item.id == id;
        });
    ROAD_TEST_EXPECT(corridor != seeded.corridors.end(),
                     std::string(scenario.name) +
                         ": saved source corridor is missing");
    corridor->segments.front().reversed = scenario.reversed_ref;
    const LaneId crossing_lane =
        source_role == EndpointRole::kEnd ? 1010 : 1000;
    if (scenario.seed_claimed_relations) {
      seeded.lane_connections.push_back(city::road::LaneConnection{
          9001,
          {source.value, crossing_lane, source_role},
          {target.value, crossing_lane, target_role},
          city::road::LaneConnectionKind::kContinuation});
      seeded.boundary_continuations.push_back(
          city::road::BoundaryContinuation{
              9002,
              {source.value, 100, source_role},
              {target.value, 100, target_role},
              city::road::BoundaryContinuationKind::kContinuation});
    }
    auto loaded = load_fixture(seeded);
    ROAD_TEST_EXPECT(loaded.ok,
                     std::string(scenario.name) +
                         ": fixture load failed: " + loaded.error);
    RoadState& state = loaded.value;
    city::road::AddLaneRequest request{};
    request.corridor_id = authored_corridor->id;
    request.direction = city::road::LaneTravelDirection::kAlongSegment;
    request.side = scenario.side;
    ROAD_TEST_EXPECT(
        SetAddLaneRange(state, request,
                        scenario.operation_forward ? 20.0 : 60.0,
                        scenario.operation_forward ? 60.0 : 20.0),
        std::string(scenario.name) + ": lane range could not be resolved");
    request.lane_width_m = 3.0;
    const auto added = state.AddLane(request);
    ROAD_TEST_EXPECT(added.ok,
                     std::string(scenario.name) +
                         ": AddLane failed: " + added.error);

    const auto lane_pair = [&](const auto& relation) {
      return relation.source == city::road::LaneEndpointKey{
                                    source.value, crossing_lane, source_role} &&
             relation.target == city::road::LaneEndpointKey{
                                    target.value, crossing_lane, target_role};
    };
    ROAD_TEST_EXPECT(
        std::count_if(state.graph().lane_connections.begin(),
                      state.graph().lane_connections.end(), lane_pair) == 1,
        std::string(scenario.name) +
            ": existing lane identity did not cross the terminal exactly once");
    ROAD_TEST_EXPECT(
        std::none_of(state.graph().lane_connections.begin(),
                     state.graph().lane_connections.end(),
                     [&added, &source](const auto& relation) {
                       return relation.source.segment_id == source.value &&
                              relation.source.lane_id == added.value;
                     }),
        std::string(scenario.name) +
            ": newly added lane crossed the terminal");

    for (const BoundaryId boundary_id :
         std::array<BoundaryId, 3>{100, 200, 300}) {
      const auto boundary_pair = [&](const auto& relation) {
        return relation.source == city::road::BoundaryEndpointKey{
                                      source.value, boundary_id, source_role} &&
               relation.target == city::road::BoundaryEndpointKey{
                                      target.value, boundary_id, target_role};
      };
      ROAD_TEST_EXPECT(
          std::count_if(state.graph().boundary_continuations.begin(),
                        state.graph().boundary_continuations.end(),
                        boundary_pair) == 1,
          std::string(scenario.name) + ": existing boundary " +
              std::to_string(boundary_id) +
              " did not cross the terminal exactly once");
    }
    std::set<BoundaryId> new_boundaries{};
    for (const auto& layout : state.graph().layout_templates) {
      const bool owns_added_lane = std::any_of(
          layout.lane_bands.begin(), layout.lane_bands.end(),
          [&added](const auto& lane) { return lane.id == added.value; });
      if (!owns_added_lane) continue;
      for (const auto& boundary : layout.boundaries) {
        if (boundary.boundary_id != 100 && boundary.boundary_id != 200 &&
            boundary.boundary_id != 300) {
          new_boundaries.insert(boundary.boundary_id);
        }
      }
    }
    ROAD_TEST_EXPECT(new_boundaries.size() == 1,
                     std::string(scenario.name) +
                         ": added divider identity is ambiguous");
    ROAD_TEST_EXPECT(
        std::none_of(state.graph().boundary_continuations.begin(),
                     state.graph().boundary_continuations.end(),
                     [&source, source_role,
                      divider = *new_boundaries.begin()](const auto& relation) {
                       return relation.source ==
                              city::road::BoundaryEndpointKey{
                                  source.value, divider, source_role};
                     }),
        std::string(scenario.name) +
            ": newly added divider crossed the terminal");

    std::set<std::uint64_t> relation_ids{};
    for (const auto& relation : state.graph().lane_connections) {
      ROAD_TEST_EXPECT(relation_ids.insert(relation.id).second,
                       std::string(scenario.name) +
                           ": duplicate relation ID");
    }
    for (const auto& relation : state.graph().boundary_continuations) {
      ROAD_TEST_EXPECT(relation_ids.insert(relation.id).second,
                       std::string(scenario.name) +
                           ": duplicate relation ID");
    }
    std::set<std::pair<city::road::LaneEndpointKey,
                       city::road::LaneEndpointKey>> lane_pairs{};
    for (const auto& relation : state.graph().lane_connections) {
      ROAD_TEST_EXPECT(
          lane_pairs.insert({relation.source, relation.target}).second,
          std::string(scenario.name) + ": duplicate lane relation");
    }
    std::set<std::pair<city::road::BoundaryEndpointKey,
                       city::road::BoundaryEndpointKey>> boundary_pairs{};
    for (const auto& relation : state.graph().boundary_continuations) {
      ROAD_TEST_EXPECT(
          boundary_pairs.insert({relation.source, relation.target}).second,
          std::string(scenario.name) + ": duplicate boundary relation");
    }
    if (scenario.seed_claimed_relations) {
      ROAD_TEST_EXPECT(
          std::any_of(state.graph().lane_connections.begin(),
                      state.graph().lane_connections.end(),
                      [](const auto& relation) { return relation.id == 9001; }) &&
              std::any_of(state.graph().boundary_continuations.begin(),
                          state.graph().boundary_continuations.end(),
                          [](const auto& relation) {
                            return relation.id == 9002;
                          }),
          "claimed relation identities were replaced");
    }
    const auto invariant =
        ValidateGraphInvariants(state.graph(), state.derived());
    ROAD_TEST_EXPECT(invariant.ok,
                     std::string(scenario.name) +
                         ": graph invariant failed: " + invariant.error);
    const auto before_round_trip = topology_observation(state.graph());
    const auto saved = state.Save();
    ROAD_TEST_EXPECT(saved.ok,
                     std::string(scenario.name) + ": " + saved.error);
    const auto restored = RoadState::Load(saved.value);
    ROAD_TEST_EXPECT(restored.ok,
                     std::string(scenario.name) +
                         ": round-trip load failed: " + restored.error);
    ROAD_TEST_EXPECT(topology_observation(restored.value.graph()) ==
                         before_round_trip,
                     std::string(scenario.name) +
                         ": terminal topology changed after save/load");
  }
  return true;
}

bool add_lane_normalizes_reversed_corridor_direction(std::string& failure) {
  RoadState forward{};
  const auto shouldered = road_fixture::AddLayout(forward, road_fixture::ShoulderedLayout(0));
  const auto segment = forward.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto saved = forward.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  std::string reversed_archive = saved.value;
  const std::string old_value = "corridor.0.segment.0.reversed=0";
  const std::string new_value = "corridor.0.segment.0.reversed=1";
  const std::size_t position = reversed_archive.find(old_value);
  ROAD_TEST_EXPECT(position != std::string::npos,
                   "ADD3 reversed corridor archive field is missing");
  reversed_archive.replace(position, old_value.size(), new_value);
  auto loaded = RoadState::Load(reversed_archive);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto* corridor =
      FindCorridorForSegment(loaded.value.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr &&
                       corridor->segments.front().reversed,
                   "ADD3 reversed corridor did not load");

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(loaded.value, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = loaded.value.AddLane(request);
  ROAD_TEST_EXPECT(added.ok,
                   "ADD3 reversed corridor lane addition failed: " +
                       added.error);
  const auto lane = std::find_if(
      loaded.value.graph().layout_templates.begin(),
      loaded.value.graph().layout_templates.end(),
      [&added](const auto& section) {
        return std::any_of(section.lane_bands.begin(),
                           section.lane_bands.end(),
                           [&added](const auto& candidate) {
                             return candidate.id == added.value;
                           });
      });
  ROAD_TEST_EXPECT(lane != loaded.value.graph().layout_templates.end(),
                   "ADD3 generated lane section is missing");
  ROAD_TEST_EXPECT(loaded.value.graph().segments.size() == 1,
                   "ADD3 Add Lane split a segment to create a maintained end");
  const auto added_lane = std::find_if(
      lane->lane_bands.begin(), lane->lane_bands.end(),
      [&added](const auto& candidate) { return candidate.id == added.value; });
  ROAD_TEST_EXPECT(
      added_lane != lane->lane_bands.end() &&
          added_lane->direction ==
              city::road::LaneTravelDirection::kAgainstSegment,
      "ADD3 corridor direction was not normalized to segment direction");
  return true;
}

bool add_lane_reaches_mixed_section_junction(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto incoming = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-80.0, 0.0}, {0.0, 0.0})}), section});
  ROAD_TEST_EXPECT(incoming.ok, incoming.error);
  const auto incoming_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&incoming](const auto& segment) { return segment.id == incoming.value; });
  ROAD_TEST_EXPECT(incoming_segment != state.graph().segments.end(),
                   "ADD4 incoming segment is missing");
  const RoadNodeId junction_node = incoming_segment->node_b;
  const auto straight = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), section,
          junction_node});
  const auto branch = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 80.0})}), section,
          junction_node});
  ROAD_TEST_EXPECT(straight.ok && branch.ok,
                   straight.ok ? branch.error : straight.error);
  const auto* corridor =
      FindCorridorForSegment(state.graph(), incoming.value);
  ROAD_TEST_EXPECT(corridor != nullptr,
                   "ADD4 incoming corridor is missing");

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok,
                   "ADD4 lane could not reach a mixed-section junction: " +
                       added.error);
  const auto* junction =
      city::road::FindResolvedConnection(state.derived(), junction_node);
  ROAD_TEST_EXPECT(junction != nullptr &&
                       junction->kind ==
                           city::road::NodeConnectionKind::kJunction &&
                       junction->approaches.size() == 3,
                   "ADD4 junction approaches were not regenerated");
  ROAD_TEST_EXPECT(!junction->junction_geometry.surface_regions.empty(),
                   "ADD4 mixed-section junction surface is missing");
  const auto* updated_corridor =
      FindCorridorForSegment(state.graph(), incoming.value);
  ROAD_TEST_EXPECT(updated_corridor != nullptr &&
                       !updated_corridor->segments.empty(),
                   "ADD4 updated corridor is missing");
  const RoadSegmentId terminal_segment_id =
      updated_corridor->segments.back().segment_id;
  const auto incoming_approach = std::find_if(
      junction->approaches.begin(), junction->approaches.end(),
      [terminal_segment_id](const auto& approach) {
        return approach.key.segment_id == terminal_segment_id;
      });
  ROAD_TEST_EXPECT(incoming_approach != junction->approaches.end() &&
                       incoming_approach->endpoint_template_id != 1,
                   "ADD4 junction did not consume the added-lane endpoint section");
  ROAD_TEST_EXPECT(state.graph().lane_connections.empty() &&
                       state.graph().boundary_continuations.empty(),
                   "ADD4 inferred a junction destination where no unique lane-count match exists");
  return true;
}

bool add_lane_connects_the_only_matching_junction_approach(
    std::string& failure) {
  RoadState authored{};
  const auto two_lane = authored.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{OneWayLaneTemplate(0, 2)});
  const auto three_lane = authored.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{OneWayLaneTemplate(0, 3)});
  ROAD_TEST_EXPECT(two_lane.ok && three_lane.ok,
                   "ADD9 fixture templates could not be created");
  const auto incoming = authored.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-80.0, 0.0}, {0.0, 0.0})}), two_lane.value});
  ROAD_TEST_EXPECT(incoming.ok, incoming.error);
  const auto incoming_segment = std::find_if(
      authored.graph().segments.begin(), authored.graph().segments.end(),
      [&incoming](const auto& segment) { return segment.id == incoming.value; });
  ROAD_TEST_EXPECT(incoming_segment != authored.graph().segments.end(),
                   "ADD9 incoming segment is missing");
  const RoadNodeId junction_node = incoming_segment->node_b;
  const auto branch = authored.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 80.0})}), two_lane.value,
          junction_node});
  const auto straight = authored.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), three_lane.value,
          junction_node});
  ROAD_TEST_EXPECT(branch.ok && straight.ok,
                   branch.ok ? straight.error : branch.error);
  // The junction already carries the movements a two-lane approach resolves to.
  // Only a load can produce that, so the fixture goes in as saved topology.
  SavedRoadGraph seeded = authored.graph();
  std::uint64_t topology_id = 9001;
  for (const auto& pair :
       std::array<std::pair<LaneId, LaneId>, 2>{{{1000, 1020},
                                                 {1010, 1010}}}) {
    seeded.lane_connections.push_back(city::road::LaneConnection{
        topology_id++,
        {incoming.value, pair.first, EndpointRole::kEnd},
        {straight.value, pair.second, EndpointRole::kStart},
        city::road::LaneConnectionKind::kJunctionMovement});
  }
  for (const auto& pair :
       std::array<std::pair<BoundaryId, BoundaryId>, 3>{{{900, 100},
                                                         {200, 300},
                                                         {100, 900}}}) {
    seeded.boundary_continuations.push_back(city::road::BoundaryContinuation{
        topology_id++,
        {incoming.value, pair.first, EndpointRole::kEnd},
        {straight.value, pair.second, EndpointRole::kStart},
        city::road::BoundaryContinuationKind::kContinuation});
  }
  auto loaded = load_fixture(seeded);
  ROAD_TEST_EXPECT(loaded.ok, "ADD9 saved junction topology failed: " + loaded.error);
  RoadState& state = loaded.value;
  ROAD_TEST_EXPECT(state.graph().lane_connections.size() == 2 &&
                       state.graph().boundary_continuations.size() == 3,
                   "ADD9 saved junction topology did not survive load");
  const auto* corridor = FindCorridorForSegment(state.graph(), incoming.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "ADD9 incoming corridor is missing");

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, "ADD9 lane addition failed: " + added.error);

  const auto* updated_corridor =
      FindRoadCorridor(state.graph(), request.corridor_id);
  ROAD_TEST_EXPECT(updated_corridor != nullptr &&
                       !updated_corridor->segments.empty(),
                   "ADD9 updated corridor is missing");
  const RoadSegmentId source_segment =
      updated_corridor->segments.back().segment_id;
  const auto added_connection = std::find_if(
      state.graph().lane_connections.begin(),
      state.graph().lane_connections.end(),
      [source_segment, &added, &straight](const auto& connection) {
        return connection.source.segment_id == source_segment &&
               connection.source.lane_id == added.value &&
               connection.target.segment_id == straight.value &&
               connection.kind ==
                   city::road::LaneConnectionKind::kJunctionMovement;
      });
  ROAD_TEST_EXPECT(added_connection == state.graph().lane_connections.end(),
                   "ADD9 inferred a junction destination for the added lane");
  ROAD_TEST_EXPECT(state.graph().lane_connections.size() == 2,
                   "ADD9 changed explicit saved junction lane topology");
  ROAD_TEST_EXPECT(state.graph().boundary_continuations.size() == 3,
                   "ADD9 changed explicit saved junction boundary topology");
  ROAD_TEST_EXPECT(state.derived().lane_paths.size() == 2 &&
                       state.derived().boundary_paths.size() == 3,
                   "ADD9 did not preserve explicit junction topology");
  return true;
}

bool add_lane_terminates_at_ambiguous_junction_destinations(
    std::string& failure) {
  RoadState state{};
  const auto two_lane = state.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{OneWayLaneTemplate(0, 2)});
  const auto three_lane = state.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{OneWayLaneTemplate(0, 3)});
  ROAD_TEST_EXPECT(two_lane.ok && three_lane.ok,
                   "ADD10 fixture templates could not be created");
  const auto incoming = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-80.0, 0.0}, {0.0, 0.0})}), two_lane.value});
  ROAD_TEST_EXPECT(incoming.ok, incoming.error);
  const auto incoming_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&incoming](const auto& segment) { return segment.id == incoming.value; });
  ROAD_TEST_EXPECT(incoming_segment != state.graph().segments.end(),
                   "ADD10 incoming segment is missing");
  const RoadNodeId node = incoming_segment->node_b;
  const auto narrow = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 80.0})}), two_lane.value,
          node});
  const auto first_wide = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), three_lane.value,
          node});
  const auto second_wide = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, -80.0})}), three_lane.value,
          node});
  ROAD_TEST_EXPECT(narrow.ok && first_wide.ok && second_wide.ok,
                   !narrow.ok ? narrow.error
                              : (!first_wide.ok ? first_wide.error
                                                : second_wide.error));
  const auto* corridor = FindCorridorForSegment(state.graph(), incoming.value);
  ROAD_TEST_EXPECT(corridor != nullptr,
                   "ADD10 preflight state is missing");
  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto before = state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok,
                   "ADD10 ambiguous junction destination should terminate the added lane: " +
                       added.error);
  ROAD_TEST_EXPECT(state.graph().lane_connections.empty() &&
                       state.graph().boundary_continuations.empty(),
                   "ADD10 inferred topology for an ambiguous junction destination");
  const auto after = state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value != before.value,
                   "ADD10 successful Add Lane did not change authoritative state");
  return true;
}

bool add_lane_allows_a_later_non_overlapping_addition(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {160.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "ADD5 corridor is missing");

  city::road::AddLaneRequest first{};
  first.corridor_id = corridor->id;
  first.direction = city::road::LaneTravelDirection::kAlongSegment;
  first.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, first, 20.0, 50.0),
                   "ADD LANE test range could not be resolved");
  first.lane_width_m = 3.0;
  const auto first_lane = state.AddLane(first);
  ROAD_TEST_EXPECT(first_lane.ok, first_lane.error);

  city::road::AddLaneRequest second = first;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, second, 80.0, 110.0),
                   "ADD LANE test range could not be resolved");
  const auto second_lane = state.AddLane(second);
  ROAD_TEST_EXPECT(
      second_lane.ok,
      "ADD5 non-overlapping lane addition was rejected: " + second_lane.error);
  ROAD_TEST_EXPECT(first_lane.value != second_lane.value,
                   "ADD5 reused the first lane identity");
  const auto terminal = std::find_if(
      state.graph().layout_templates.begin(),
      state.graph().layout_templates.end(),
      [&state, &second_lane](const auto& section) {
        return std::any_of(section.lane_bands.begin(),
                           section.lane_bands.end(),
                           [&second_lane](const auto& lane) {
                             return lane.id == second_lane.value;
                           });
      });
  ROAD_TEST_EXPECT(terminal != state.graph().layout_templates.end() &&
                       terminal->lane_bands.size() == 4,
                   "ADD5 second lane did not reach the terminal section");
  return true;
}

bool add_lane_allows_an_earlier_non_overlapping_addition(
    std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto first_segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(first_segment.ok, first_segment.error);
  const auto first = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&first_segment](const auto& segment) {
        return segment.id == first_segment.value;
      });
  const auto* initial_corridor =
      FindCorridorForSegment(state.graph(), first_segment.value);
  ROAD_TEST_EXPECT(first != state.graph().segments.end() &&
                       initial_corridor != nullptr,
                   "ADD7 initial corridor is missing");
  const RoadCorridorId corridor_id = initial_corridor->id;
  const auto second_segment = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, first->node_b,
          MakePath({MakeLine({80.0, 0.0}, {180.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(second_segment.ok, second_segment.error);

  city::road::AddLaneRequest first_add{};
  first_add.corridor_id = corridor_id;
  first_add.direction = city::road::LaneTravelDirection::kAlongSegment;
  first_add.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, first_add, 60.0, 100.0),
                   "ADD LANE test range could not be resolved");
  first_add.lane_width_m = 3.0;
  const auto first_lane = state.AddLane(first_add);
  ROAD_TEST_EXPECT(first_lane.ok, first_lane.error);

  city::road::AddLaneRequest second_add = first_add;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, second_add, 10.0, 40.0),
                   "ADD LANE test range could not be resolved");
  const auto second_lane = state.AddLane(second_add);
  ROAD_TEST_EXPECT(
      second_lane.ok,
      "ADD8 earlier non-overlapping lane addition was rejected: " +
          second_lane.error);
  return true;
}

bool add_lane_accepts_multiple_lanes_on_one_carriageway_strip(
    std::string& failure) {
  RoadState state{};
  const auto section = state.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{
          SharedCarriagewayLaneTemplate(0)});
  ROAD_TEST_EXPECT(section.ok, section.error);
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), section.value});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "ADD7 corridor is missing");

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(
      added.ok,
      "ADD7 shared carriageway lane addition was rejected: " + added.error);
  return true;
}

bool add_lane_supports_every_section_outer_side(
    std::string& failure) {
  const std::array<city::road::RoadLayoutTemplate, 5> sections{
      road_fixture::BidirectionalLayout(0), road_fixture::ExtraLaneLayout(0),
      road_fixture::AsymmetricLayout(0), road_fixture::MedianLayout(0),
      road_fixture::ShoulderedLayout(0)};
  for (std::size_t index = 0; index < sections.size(); ++index) {
    for (const auto [direction, side] : {
             std::pair{city::road::LaneTravelDirection::kAgainstSegment,
                       city::road::RoadSide::kLeft},
             std::pair{city::road::LaneTravelDirection::kAlongSegment,
                       city::road::RoadSide::kRight}}) {
      RoadState state{};
      const auto template_id = road_fixture::AddLayout(state, sections[index]);
      ROAD_TEST_EXPECT(template_id != 0, "ADD11 section fixture was rejected");
      const auto segment = state.AddSegment(city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), template_id});
      ROAD_TEST_EXPECT(segment.ok, segment.error);
      const auto* corridor =
          city::road::FindCorridorForSegment(state.graph(), segment.value);
      ROAD_TEST_EXPECT(corridor != nullptr,
                       "ADD11 corridor is missing");

      city::road::AddLaneRequest request{};
      request.corridor_id = corridor->id;
      request.direction = direction;
      request.side = side;
      ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                       "ADD LANE test range could not be resolved");
      request.lane_width_m = 3.0;
      const auto added = state.AddLane(request);
      ROAD_TEST_EXPECT(
          added.ok,
          "ADD11 section " + std::to_string(index) +
              " rejected an outer lane: " + added.error);
    }
  }

  RoadState single_strip_state{};
  city::road::RoadLayoutTemplate single_strip{};
  single_strip.strips = {
      {10, StripFunction::kCarriageway, 3.0, 0.0,
       builtin_surface_styles::kAsphalt}};
  single_strip.lane_bands = {
      {1000, 10, 0.0, 3.0,
       city::road::LaneTravelDirection::kAlongSegment}};
  single_strip.alignment_offset_from_left_m =
      road_fixture::CentredAlignmentOffset(single_strip);
  const auto section = single_strip_state.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{single_strip});
  ROAD_TEST_EXPECT(section.ok, section.error);
  const auto segment = single_strip_state.AddSegment(
      city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), section.value});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto* corridor = city::road::FindCorridorForSegment(
      single_strip_state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr,
                   "ADD11 single-strip corridor is missing");
  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(single_strip_state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = single_strip_state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok,
                   "ADD11 single-strip lane addition was rejected: " +
                       added.error);
  return true;
}

bool add_lane_uses_user_pick_direction_on_a_corridor(std::string& failure) {
  for (const bool reverse_operation : {false, true}) {
    for (const auto direction : {city::road::LaneTravelDirection::kAlongSegment,
                                 city::road::LaneTravelDirection::kAgainstSegment}) {
      for (const auto side : {city::road::RoadSide::kLeft,
                              city::road::RoadSide::kRight}) {
        RoadState state{};
        const auto section =
            road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
        ROAD_TEST_EXPECT(section != 0,
                         "ADD12 section fixture could not be registered");
        const auto segment = state.AddSegment(city::road::AddSegmentRequest{
            MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), section});
        ROAD_TEST_EXPECT(segment.ok, segment.error);
        const auto* corridor =
            city::road::FindCorridorForSegment(state.graph(), segment.value);
        ROAD_TEST_EXPECT(corridor != nullptr, "ADD12 corridor is missing");

        city::road::AddLaneRequest request{};
        request.corridor_id = corridor->id;
        request.direction = direction;
        request.side = side;
        request.lane_width_m = 3.0;
        const bool positioned =
            reverse_operation
                ? SetAddLanePositions(state, request, 80.0, 60.0)
                : SetAddLanePositions(state, request, 20.0, 40.0);
        ROAD_TEST_EXPECT(positioned,
                         "ADD12 corridor positions could not be resolved");
        const auto before = state.Save();
        ROAD_TEST_EXPECT(before.ok, before.error);
        const auto added = state.AddLane(request);
        ROAD_TEST_EXPECT(
            added.ok,
            std::string("ADD12 ") + (reverse_operation ? "reverse" : "forward") +
                " Add Lane was rejected: " + added.error);
        const auto saved = state.Save();
        ROAD_TEST_EXPECT(saved.ok, saved.error);
        const auto loaded = RoadState::Load(saved.value);
        ROAD_TEST_EXPECT(loaded.ok,
                         "ADD12 saved lane transition failed to load: " +
                             loaded.error);

        city::road::AddLaneRequest invalid = request;
        ROAD_TEST_EXPECT(
            SetAddLanePositions(state, invalid,
                                reverse_operation ? 80.0 : 20.0,
                                reverse_operation ? 80.0 : 20.0),
            "ADD12 invalid positions could not resolve");
        const auto after_valid = state.Save();
        ROAD_TEST_EXPECT(after_valid.ok, after_valid.error);
        const auto rejected = state.AddLane(invalid);
        ROAD_TEST_EXPECT(!rejected.ok &&
                             rejected.failure_category ==
                                 CommitFailureCategory::kInvalidInput,
                         "ADD12 accepted identical start and completion points");
        const auto after_reject = state.Save();
        ROAD_TEST_EXPECT(after_reject.ok &&
                             after_reject.value == after_valid.value,
                         "ADD12 invalid request mutated authoritative state");
        ROAD_TEST_EXPECT(after_valid.value != before.value,
                         "ADD12 valid Add Lane did not change the graph");
      }
    }
  }
  return true;
}




bool saved_junction_movements_derive_lane_paths(std::string& failure) {
  RoadState authored{};
  const auto authored_section = road_fixture::AddLayout(authored, road_fixture::BidirectionalLayout(0));
  const auto base = authored.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), authored_section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = authored.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({30.0, 0.0}, {30.0, 30.0})}), authored_section,
          base.value, 30.0});
  ROAD_TEST_EXPECT(north.ok, north.error);
  ROAD_TEST_EXPECT(!road_test_view::junctions(authored.derived()).empty(),
                   "LAN7 T junction was not generated");
  const RoadNodeId node =
      road_test_view::junctions(authored.derived()).front()->node_id;
  const auto south = authored.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({30.0, 0.0}, {30.0, -30.0})}), authored_section, node});
  ROAD_TEST_EXPECT(south.ok, south.error);

  RoadSegmentId west_segment = 0;
  RoadSegmentId east_segment = 0;
  for (const auto& segment : authored.graph().segments) {
    if (segment.id == north.value || segment.id == south.value)
      continue;
    const RoadNodeId other_id = segment.node_a == node ? segment.node_b
                                                       : segment.node_a;
    const auto other = std::find_if(
        authored.graph().nodes.begin(), authored.graph().nodes.end(),
        [other_id](const auto& candidate) { return candidate.id == other_id; });
    ROAD_TEST_EXPECT(other != authored.graph().nodes.end(),
                     "LAN7 cross approach endpoint is missing");
    if (other->position.x < 30.0)
      west_segment = segment.id;
    if (other->position.x > 30.0)
      east_segment = segment.id;
  }
  ROAD_TEST_EXPECT(west_segment != 0 && east_segment != 0,
                   "LAN7 could not identify west/east approaches by fixture IDs");
  const auto role_at = [&](RoadSegmentId id) {
    const auto segment = std::find_if(
        authored.graph().segments.begin(), authored.graph().segments.end(),
        [id](const auto& candidate) { return candidate.id == id; });
    return segment->node_a == node ? EndpointRole::kStart
                                   : EndpointRole::kEnd;
  };
  const city::road::LaneEndpointKey source{
      west_segment, 1010, role_at(west_segment)};
  const std::array<city::road::LaneEndpointKey, 3> targets{
      city::road::LaneEndpointKey{east_segment, 1010,
                                  role_at(east_segment)},
      city::road::LaneEndpointKey{north.value, 1010,
                                  role_at(north.value)},
      city::road::LaneEndpointKey{south.value, 1010,
                                  role_at(south.value)},
  };
  // Junction movements are saved lane topology. Loading a workspace that
  // already holds them must reproduce one drivable path per movement.
  SavedRoadGraph seeded = authored.graph();
  std::vector<city::road::LaneConnectionId> movement_ids{};
  for (const auto& target : targets) {
    const city::road::LaneConnectionId id = 9001 + movement_ids.size();
    seeded.lane_connections.push_back(city::road::LaneConnection{
        id, source, target,
        city::road::LaneConnectionKind::kJunctionMovement});
    movement_ids.push_back(id);
  }
  const auto archived = city::road::persistence::SaveRoad(seeded, kFixtureNextId);
  ROAD_TEST_EXPECT(archived.ok, archived.error);
  const auto loaded = RoadState::Load(archived.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const RoadState& state = loaded.value;
  ROAD_TEST_EXPECT(state.derived().lane_paths.size() == 3,
                   "LAN7 did not derive all saved junction movements");
  for (const auto id : movement_ids) {
    const auto movement = std::find_if(
        state.graph().lane_connections.begin(),
        state.graph().lane_connections.end(),
        [id](const auto& candidate) { return candidate.id == id; });
    const auto path = std::find_if(
        state.derived().lane_paths.begin(), state.derived().lane_paths.end(),
        [id](const auto& candidate) { return candidate.connection_id == id; });
    ROAD_TEST_EXPECT(movement != state.graph().lane_connections.end() &&
                         movement->kind ==
                             city::road::LaneConnectionKind::kJunctionMovement &&
                         path != state.derived().lane_paths.end() &&
                         path->centerline.spans.size() == 1 &&
                         std::isfinite(path->length_m) && path->length_m > 0.0 &&
                         !std::isnan(path->minimum_radius_m) &&
                         path->minimum_radius_m > 0.0,
                     "LAN7 junction movement is missing or degenerate");
    const auto& span = path->centerline.spans.front();
    const Vec2d start_tangent{span.p1.x - span.p0.x,
                              span.p1.y - span.p0.y};
    ROAD_TEST_EXPECT(start_tangent.x > 0.0 &&
                         std::abs(start_tangent.y) < 1e-9,
                     "LAN7 movement is not G1 with the incoming west lane");
  }
  const auto round_trip = state.Save();
  ROAD_TEST_EXPECT(round_trip.ok && round_trip.value == archived.value,
                   "LAN7 junction movements did not re-save bit-identically");

  SavedRoadGraph wrong_way = seeded;
  wrong_way.lane_connections.front().source.lane_id = 1000;
  const auto rejected =
      city::road::persistence::SaveRoad(wrong_way, kFixtureNextId);
  ROAD_TEST_EXPECT(!rejected.ok && rejected.failure_category ==
                                       CommitFailureCategory::kInvalidInput,
                   "LAN7 accepted a wrong-way junction movement");

  RoadState pass_through{};
  const auto pass_through_section = road_fixture::AddLayout(pass_through, road_fixture::BidirectionalLayout(0));
  const auto first = pass_through.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {30.0, 0.0})}), pass_through_section});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const RoadNodeId pass_node = pass_through.graph().segments.front().node_b;
  const auto second = pass_through.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({30.0, 0.0}, {60.0, 0.0})}), pass_through_section, pass_node});
  ROAD_TEST_EXPECT(second.ok, second.error);
  SavedRoadGraph outside_junction = pass_through.graph();
  outside_junction.lane_connections.push_back(city::road::LaneConnection{
      9001,
      {first.value, 1010, EndpointRole::kEnd},
      {second.value, 1010, EndpointRole::kStart},
      city::road::LaneConnectionKind::kJunctionMovement});
  const auto generated =
      city::road::generation::generate_road(outside_junction);
  ROAD_TEST_EXPECT(!generated.ok && generated.failure_category ==
                                        CommitFailureCategory::kNotImplemented,
                   "LAN7 accepted junction movement topology outside a junction");
  return true;
}


bool P2_supports_taper_lane_reduction_and_median_end(std::string& failure) {
  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    auto no_left_sidewalk = road_fixture::BidirectionalLayout(0);
    no_left_sidewalk.strips.erase(no_left_sidewalk.strips.begin());
    no_left_sidewalk.boundaries.erase(no_left_sidewalk.boundaries.begin());
    const auto target = state.AddRoadLayoutTemplate(city::road::AddRoadLayoutTemplateRequest{no_left_sidewalk});
    ROAD_TEST_EXPECT(target.ok, target.error);
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SavedRoadGraph graph = state.graph();
    attach_transition(
        graph, segment.value,
        RoadLayoutTransition{9001, section, target.value,
                          DistanceRef{DistanceRefKind::kFromStart, 10.0},
                          DistanceRef{DistanceRefKind::kRatio, 0.5},
                          TransitionAnchor::kRightEdge, 0,
                          {RoadLayoutTransitionRule{10, TransitionAction::kTaperOut}}});
    const auto generated = city::road::generation::generate_road(graph);
    ROAD_TEST_EXPECT(generated.ok, "sidewalk taper was not generated: " + generated.error);
    ROAD_TEST_EXPECT(!generated.value.segment_meshes.empty(), "sidewalk taper produced no material meshes");
  }

  {
    RoadState state{};
    const auto two_lane = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto three_lane = state.AddRoadLayoutTemplate(city::road::AddRoadLayoutTemplateRequest{road_fixture::ExtraLaneLayout(0)});
    ROAD_TEST_EXPECT(three_lane.ok, three_lane.error);
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), three_lane.value});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SavedRoadGraph graph = state.graph();
    attach_transition(
        graph, segment.value,
        RoadLayoutTransition{9001, three_lane.value, two_lane,
                          DistanceRef{DistanceRefKind::kFromEnd, 30.0},
                          DistanceRef{DistanceRefKind::kFromEnd, 5.0},
                          TransitionAnchor::kCenter, 0,
                          {RoadLayoutTransitionRule{35, TransitionAction::kTaperOut}}});
    const auto generated = city::road::generation::generate_road(graph);
    ROAD_TEST_EXPECT(generated.ok, "lane reduction was not generated: " + generated.error);
  }

  {
    RoadState state{};
    const auto two_lane = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    auto median = road_fixture::BidirectionalLayout(0);
    const city::road::AutoMarkingPolicy outer_line{
        true, MarkingRole::kCarriagewayEdge, builtin_marking_styles::kWhiteSolid,
        MarkingPlacement::kInside};
    median.strips.insert(median.strips.begin() + 2,
                        {25, StripFunction::kMedian, 2.0, 0.0, builtin_surface_styles::kMedian});
    median.boundaries = {
        road_fixture::CurbBoundary(100, -0.2, 0.15, outer_line),
        road_fixture::CurbBoundary(210, 0.2, 0.12, {}),
        road_fixture::CurbBoundary(220, -0.2, 0.12, {}),
        road_fixture::CurbBoundary(300, 0.2, 0.15, outer_line),
    };
    const auto median_id = state.AddRoadLayoutTemplate(city::road::AddRoadLayoutTemplateRequest{median});
    ROAD_TEST_EXPECT(median_id.ok, median_id.error);
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), median_id.value});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    const auto with_median_rule = [&](TransitionAction action) {
      SavedRoadGraph graph = state.graph();
      attach_transition(
          graph, segment.value,
          RoadLayoutTransition{9001, median_id.value, two_lane,
                            DistanceRef{DistanceRefKind::kFromStart, 10.0},
                            DistanceRef{DistanceRefKind::kFromStart, 30.0},
                            TransitionAnchor::kCenter, 0,
                            {RoadLayoutTransitionRule{25, action}}});
      return city::road::generation::generate_road(graph);
    };
    const auto tapered = with_median_rule(TransitionAction::kTaperOut);
    ROAD_TEST_EXPECT(!tapered.ok && tapered.failure_category == CommitFailureCategory::kNotImplemented,
                     "median disappearance accepted TaperOut instead of EndCap");
    const auto capped = with_median_rule(TransitionAction::kEndCap);
    ROAD_TEST_EXPECT(capped.ok, "median end cap was not generated: " + capped.error);
  }
  return true;
}

bool P2_degree_two_accepts_different_lane_counts(std::string& failure) {
  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto three_lane = road_fixture::AddLayout(state, road_fixture::ExtraLaneLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto endpoint = state.graph().segments.front().node_b;
    const auto direct = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), three_lane, endpoint});
    ROAD_TEST_EXPECT(direct.ok,
                     "degree-two connection rejected a different lane count: " +
                         direct.error);
    const auto connections = road_test_view::corners(state.derived());
    ROAD_TEST_EXPECT(connections.size() == 1,
                     "different lane counts did not form one degree-two corner");
    ROAD_TEST_EXPECT(
        !connections.front()->connection_geometry.surface_strips.empty(),
        "different lane counts produced no degree-two connector surfaces");
  }

  {
    RoadState authored{};
    const auto authored_section = road_fixture::AddLayout(authored, road_fixture::BidirectionalLayout(0));
    const auto three_lane = road_fixture::AddLayout(authored, road_fixture::ExtraLaneLayout(0));
    const auto base = authored.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), authored_section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    SavedRoadGraph graph = authored.graph();
    attach_transition(
        graph, base.value,
        RoadLayoutTransition{9001, authored_section, three_lane,
                          DistanceRef{DistanceRefKind::kFromEnd, 20.0},
                          DistanceRef{DistanceRefKind::kFromEnd, 0.0},
                          TransitionAnchor::kCenter, 0,
                          {RoadLayoutTransitionRule{35, TransitionAction::kTaperIn}}});
    auto loaded = load_fixture(graph);
    ROAD_TEST_EXPECT(loaded.ok, loaded.error);
    RoadState& state = loaded.value;
    const auto endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), three_lane, endpoint});
    ROAD_TEST_EXPECT(connected.ok, connected.error);
  }
  return true;
}

bool equivalent_endpoint_sections_ignore_template_identity(
    std::string& failure) {
  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto equivalent = state.AddRoadLayoutTemplate(
        city::road::AddRoadLayoutTemplateRequest{
            road_fixture::BidirectionalLayout(0)});
    ROAD_TEST_EXPECT(equivalent.ok, equivalent.error);
    ROAD_TEST_EXPECT(equivalent.value != section,
                     "equivalent section did not receive a distinct ID");
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadNodeId endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({60.0, 0.0}, {60.0, 30.0})}),
            equivalent.value, endpoint});
    ROAD_TEST_EXPECT(
        connected.ok,
        "equivalent endpoint sections were rejected by template identity: " +
            connected.error);
  }

  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto equivalent = state.AddRoadLayoutTemplate(
        city::road::AddRoadLayoutTemplateRequest{
            road_fixture::BidirectionalLayout(0)});
    ROAD_TEST_EXPECT(equivalent.ok, equivalent.error);
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadSegment source = state.graph().segments.front();
    const auto* corridor =
        FindCorridorForSegment(state.graph(), source.id);
    ROAD_TEST_EXPECT(corridor != nullptr,
                     "equivalent section extension corridor is missing");
    const auto extended = state.ExtendCorridorFromEnd(
        city::road::ExtendCorridorFromEndRequest{
            corridor->id, source.node_b,
            MakePath({MakeLine({40.0, 0.0}, {80.0, 0.0})}),
            equivalent.value});
    ROAD_TEST_EXPECT(
        extended.ok,
        "equivalent endpoint section extension was rejected by template "
        "identity: " +
            extended.error);
  }
  return true;
}

bool P2_marking_policy_suppression_and_junction_override(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto base = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto branch_a = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({40.0, 0.0}, {40.0, 30.0})}), section, node});
  ROAD_TEST_EXPECT(branch_a.ok, branch_a.error);
  const auto branch_b = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({40.0, 0.0}, {70.0, 0.0})}), section, node});
  ROAD_TEST_EXPECT(branch_b.ok, branch_b.error);
  ROAD_TEST_EXPECT(!road_test_view::junctions(state.derived()).empty(), "junction area was not derived");

  const auto center_paths = [&]() {
    return static_cast<std::size_t>(road_test_view::count_marking_lines(state.derived(), [](const auto& path) {
          return path.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                 path.role == MarkingRole::kCenterLine;
        }));
  };
  const std::size_t center_before = center_paths();
  ROAD_TEST_EXPECT(center_before >= 3, "center lines were not resolved per segment");

  const auto center_paths_of = [](const city::road::DerivedRoad& derived) {
    return static_cast<std::size_t>(road_test_view::count_marking_lines(derived, [](const auto& path) {
          return path.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                 path.role == MarkingRole::kCenterLine;
        }));
  };

  // Suppression and junction overrides are saved rows, not operations. Both are
  // read while generating, so the fixture puts them straight on the graph.
  AutoMarkingKey suppress_key{
      MarkingOwner{MarkingOwner::Kind::kRoadSegment, base.value, 0, 0},
      MarkingRole::kCenterLine,
      MarkingTrackKey{base.value, 200, MarkingRole::kCenterLine},
      std::nullopt};
  SavedRoadGraph suppressed_graph = state.graph();
  suppressed_graph.auto_marking_overrides.push_back(
      city::road::AutoMarkingOverride{suppress_key, true});
  const auto suppressed = city::road::generation::generate_road(suppressed_graph);
  ROAD_TEST_EXPECT(suppressed.ok, suppressed.error);
  ROAD_TEST_EXPECT(center_paths_of(suppressed.value) + 1 == center_before,
                   "semantic suppression did not remove exactly one segment marking");
  const auto archived =
      city::road::persistence::SaveRoad(suppressed_graph, kFixtureNextId);
  ROAD_TEST_EXPECT(archived.ok, archived.error);
  ROAD_TEST_EXPECT(archived.value.find("auto_marking_override.count=1") != std::string::npos,
                   "auto marking suppression was not persisted");
  const auto loaded = RoadState::Load(archived.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  ROAD_TEST_EXPECT(center_paths_of(loaded.value.derived()) + 1 == center_before,
                   "auto marking suppression was not restored");

  const auto& area = *road_test_view::junctions(state.derived()).front();
  ROAD_TEST_EXPECT(road_test_view::gates_of(area).size() >= 2, "junction override test needs two gates");
  JunctionMarkingOverride override{};
  override.id = 9001;
  override.node_id = area.node_id;
  override.source = JunctionMarkingEndpoint{area.approaches[0].gate.approach, 200, MarkingRole::kCenterLine};
  override.action = JunctionMarkingAction::kConnectToApproach;
  override.target = JunctionMarkingEndpoint{area.approaches[1].gate.approach, 200, MarkingRole::kCenterLine};
  SavedRoadGraph override_graph = state.graph();
  override_graph.junction_marking_overrides.push_back(override);
  const auto overridden = city::road::generation::generate_road(override_graph);
  ROAD_TEST_EXPECT(overridden.ok, overridden.error);
  ROAD_TEST_EXPECT(0 < road_test_view::count_marking_lines(overridden.value, [](const auto& path) {
                                 return path.owner.kind == MarkingOwner::Kind::kJunction &&
                                        path.role == MarkingRole::kCenterLine &&
                                        path.points.size() == 2;
                               }),
                   "explicit junction marking override did not create a junction-owned path");
  ROAD_TEST_EXPECT(0 == road_test_view::count_marking_lines(state.derived(), [](const auto& path) {
                                  return path.owner.kind == MarkingOwner::Kind::kJunction &&
                                         path.role == MarkingRole::kCenterLine;
                                }),
                   "a junction-owned center path appeared without an override");

  // The boundary policy lives on the shared section template, so editing the
  // template is what turns a center line off and on.
  auto without_center = road_fixture::BidirectionalLayout(section);
  without_center.boundaries[1].marking = AutoMarkingPolicy{};
  const auto disabled =
      state.EditRoadLayoutTemplate(city::road::EditRoadLayoutTemplateRequest{without_center});
  ROAD_TEST_EXPECT(disabled.ok, disabled.error);
  ROAD_TEST_EXPECT(0 == road_test_view::count_marking_lines(state.derived(), [](const auto& path) {
                                  return path.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                                         path.role == MarkingRole::kCenterLine;
                                }),
                   "disabling the boundary policy did not remove center line tracks");
  const auto reenabled = state.EditRoadLayoutTemplate(
      city::road::EditRoadLayoutTemplateRequest{road_fixture::BidirectionalLayout(section)});
  ROAD_TEST_EXPECT(reenabled.ok, reenabled.error);
  ROAD_TEST_EXPECT(center_paths() == center_before,
                   "restoring the boundary policy did not restore center line tracks");
  auto unknown_style_template = road_fixture::BidirectionalLayout(section);
  unknown_style_template.boundaries[1].marking =
      AutoMarkingPolicy{true, MarkingRole::kCenterLine,
                        city::road::MarkingStyleId{9999},
                        MarkingPlacement::kCenter};
  const auto unknown_style = state.EditRoadLayoutTemplate(
      city::road::EditRoadLayoutTemplateRequest{unknown_style_template});
  ROAD_TEST_EXPECT(!unknown_style.ok && unknown_style.failure_category == CommitFailureCategory::kInvalidInput,
                   "unknown boundary marking style was not validation rejected");
  return true;
}

// A boundary line is identified by its owner segment and boundary ID.
const city::road::DerivedMarking* find_boundary_line(const city::road::DerivedRoad& derived,
                                                     city::road::RoadSegmentId segment_id,
                                                     std::uint64_t boundary_id) {
  for (const auto* marking : road_test_view::marking_lines(derived)) {
    if (marking->owner.kind == city::road::MarkingOwner::Kind::kRoadSegment &&
        marking->owner.segment_id == segment_id &&
        marking->boundary_id == boundary_id) {
      return marking;
    }
  }
  return nullptr;
}

bool M1_lane_side_requests_share_one_boundary_line(std::string& failure) {
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto segment = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), section});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const std::size_t baseline = road_test_view::marking_lines(state.derived()).size();
  const AutoMarkingPolicy center{true, MarkingRole::kCenterLine,
                                 builtin_marking_styles::kCenterLine,
                                 MarkingPlacement::kCenter};
  // Both lanes request the same line on the boundary they share.
  auto shared_request = road_fixture::BidirectionalLayout(section);
  shared_request.strips[1].side_marking.right = center;
  shared_request.strips[2].side_marking.left = center;
  const auto merged =
      state.EditRoadLayoutTemplate(city::road::EditRoadLayoutTemplateRequest{shared_request});
  ROAD_TEST_EXPECT(merged.ok, merged.error);
  ROAD_TEST_EXPECT(road_test_view::marking_lines(state.derived()).size() == baseline,
                   "duplicate lane side requests did not merge into one line");

  const auto saved_before = state.Save();
  ROAD_TEST_EXPECT(saved_before.ok, saved_before.error);
  auto conflicting_request = shared_request;
  conflicting_request.strips[2].side_marking.left =
      AutoMarkingPolicy{true, MarkingRole::kLaneSeparator,
                        builtin_marking_styles::kWhiteDashed,
                        MarkingPlacement::kCenter};
  const auto conflict = state.EditRoadLayoutTemplate(
      city::road::EditRoadLayoutTemplateRequest{conflicting_request});
  ROAD_TEST_EXPECT(!conflict.ok && conflict.failure_category == CommitFailureCategory::kNotImplemented,
                   "conflicting lane side requests were not unsupported");
  const auto saved_after = state.Save();
  ROAD_TEST_EXPECT(saved_after.ok && saved_after.value == saved_before.value,
                   "conflicting request mutated authoritative state");

  const auto reloaded = RoadState::Load(saved_before.value);
  ROAD_TEST_EXPECT(reloaded.ok, reloaded.error);
  ROAD_TEST_EXPECT(road_test_view::marking_lines(reloaded.value.derived()).size() == baseline,
                   "lane side policy did not survive save and load");
  return true;
}

bool M1_lane_side_without_adjacent_boundary_is_unsupported(std::string& failure) {
  RoadState state{};
  const auto no_left_sidewalk = road_fixture::AddLayout(state, road_fixture::AsymmetricLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  // Template 3 starts with a carriageway band, so its left side has no boundary.
  const auto segment = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), no_left_sidewalk});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto saved_before = state.Save();
  ROAD_TEST_EXPECT(saved_before.ok, saved_before.error);
  const AutoMarkingPolicy edge{true, MarkingRole::kCarriagewayEdge,
                               builtin_marking_styles::kWhiteSolid,
                               MarkingPlacement::kInside};
  auto outermost = road_fixture::AsymmetricLayout(no_left_sidewalk);
  outermost.strips.front().side_marking.left = edge;
  const auto request =
      state.EditRoadLayoutTemplate(city::road::EditRoadLayoutTemplateRequest{outermost});
  ROAD_TEST_EXPECT(!request.ok && request.failure_category == CommitFailureCategory::kNotImplemented,
                   "outermost lane side request was not unsupported");
  const auto saved_after = state.Save();
  ROAD_TEST_EXPECT(saved_after.ok && saved_after.value == saved_before.value,
                   "unsupported lane side request mutated authoritative state");

  auto sidewalk_request = road_fixture::BidirectionalLayout(section);
  sidewalk_request.strips.front().side_marking.right = edge;
  const auto non_carriageway = state.EditRoadLayoutTemplate(
      city::road::EditRoadLayoutTemplateRequest{sidewalk_request});
  ROAD_TEST_EXPECT(!non_carriageway.ok && non_carriageway.failure_category == CommitFailureCategory::kInvalidInput,
                   "lane side policy on a sidewalk band was not rejected");
  return true;
}

bool M3_lane_count_change_begins_and_terminates_lines(std::string& failure) {
  // The roads run along +X from the origin, so a point's x is its distance.
  {
    RoadState state{};
    const auto three_lane = road_fixture::AddLayout(state, road_fixture::ExtraLaneLayout(0));
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto segment = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), section});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    const auto* corridor = FindCorridorForSegment(state.graph(), segment.value);
    ROAD_TEST_EXPECT(corridor != nullptr, "lane addition corridor is missing");
    city::road::AddLaneRequest request{};
    request.corridor_id = corridor->id;
    request.direction = city::road::LaneTravelDirection::kAlongSegment;
    request.side = city::road::RoadSide::kRight;
    ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 50.0),
                     "lane addition range could not be resolved");
    request.lane_width_m = 3.0;
    const auto added = state.AddLane(request);
    ROAD_TEST_EXPECT(added.ok, added.error);

    // Template 1 has no lane separator, so the dashed line is the divider the
    // added lane brought with it.
    const auto* divider = road_test_view::find_marking_line(
        state.derived(), [&segment](const auto& marking) {
          return marking.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                 marking.owner.segment_id == segment.value &&
                 marking.role == MarkingRole::kLaneSeparator;
        });
    ROAD_TEST_EXPECT(divider != nullptr, "added lane divider produced no marking line");
    ROAD_TEST_EXPECT(divider->points.front().x > 20.0,
                     "added lane divider began before its lane existed");
    ROAD_TEST_EXPECT(divider->points.back().x > 70.0,
                     "added lane divider did not continue to the segment end");

    const auto* kept = find_boundary_line(state.derived(), segment.value, 200);
    ROAD_TEST_EXPECT(kept != nullptr, "existing lane divider lost its marking line");
    ROAD_TEST_EXPECT(kept->points.front().x < 1e-6 && kept->points.back().x > 79.0,
                     "existing lane divider did not continue across the transition");
  }

  {
    // A lane reduction has no public operation, so it is generated from the
    // saved transition that expresses one.
    RoadState authored{};
    const auto three_lane = road_fixture::AddLayout(authored, road_fixture::ExtraLaneLayout(0));
    const auto two_lane = road_fixture::AddLayout(authored, road_fixture::BidirectionalLayout(0));
    const auto segment = authored.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), three_lane});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SavedRoadGraph graph = authored.graph();
    attach_transition(
        graph, segment.value,
        RoadLayoutTransition{9001, three_lane, two_lane,
                          DistanceRef{DistanceRefKind::kFromStart, 30.0},
                          DistanceRef{DistanceRefKind::kFromStart, 60.0},
                          TransitionAnchor::kCenter, 0,
                          {RoadLayoutTransitionRule{35, TransitionAction::kTaperOut}}});
    const auto generated = city::road::generation::generate_road(graph);
    ROAD_TEST_EXPECT(generated.ok, generated.error);

    const auto* removed = find_boundary_line(generated.value, segment.value, 250);
    ROAD_TEST_EXPECT(removed != nullptr, "removed lane divider produced no marking line");
    ROAD_TEST_EXPECT(removed->points.back().x < 60.0,
                     "removed lane divider survived past its lane");
    ROAD_TEST_EXPECT(removed->points.front().x < 1e-6,
                     "removed lane divider did not start at the segment start");

    const auto* kept = find_boundary_line(generated.value, segment.value, 200);
    ROAD_TEST_EXPECT(kept != nullptr, "existing lane divider lost its marking line");
    ROAD_TEST_EXPECT(kept->points.back().x > 79.0,
                     "existing lane divider did not continue across the reduction");
  }
  return true;
}

bool M6_transition_without_boundary_mapping_is_unsupported(std::string& failure) {
  RoadState authored{};
  const auto authored_section = road_fixture::AddLayout(authored, road_fixture::BidirectionalLayout(0));
  auto role_changed = road_fixture::BidirectionalLayout(0);
  role_changed.boundaries[1].role = BoundaryRole::kMedianEdge;
  const auto target = authored.AddRoadLayoutTemplate(city::road::AddRoadLayoutTemplateRequest{role_changed});
  ROAD_TEST_EXPECT(target.ok, target.error);
  const auto segment = authored.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), authored_section});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const SavedRoadGraph without_transition = authored.graph();
  ROAD_TEST_EXPECT(city::road::generation::generate_road(without_transition).ok,
                   "M6 fixture did not generate before the transition was added");

  SavedRoadGraph graph = without_transition;
  attach_transition(
      graph, segment.value,
      RoadLayoutTransition{9001, authored_section, target.value,
                        DistanceRef{DistanceRefKind::kFromStart, 10.0},
                        DistanceRef{DistanceRefKind::kFromStart, 40.0},
                        TransitionAnchor::kCenter, 0,
                        {RoadLayoutTransitionRule{20, TransitionAction::kContinue}}});
  const auto generated = city::road::generation::generate_road(graph);
  ROAD_TEST_EXPECT(!generated.ok && generated.failure_category == CommitFailureCategory::kNotImplemented,
                   "boundary role change was not reported as unsupported");
  return true;
}

bool RSL_section_axes_and_shoulder_are_independent(std::string& failure) {
  const auto shoulder = road_fixture::ShoulderedLayout(0);
  ROAD_TEST_EXPECT(shoulder.strips.size() == 6,
                   "shouldered template does not contain independent shoulder strips");
  ROAD_TEST_EXPECT(
      shoulder.strips[1].function == StripFunction::kShoulder &&
          shoulder.strips[1].style_id == builtin_surface_styles::kAsphalt &&
          shoulder.strips[2].function == StripFunction::kCarriageway &&
          shoulder.strips[2].style_id == builtin_surface_styles::kAsphalt,
      "strip function is still inferred from surface style");
  ROAD_TEST_EXPECT(shoulder.lane_bands.size() == 2 &&
                       shoulder.lane_bands[0].surface_strip_id == 20 &&
                       shoulder.lane_bands[1].surface_strip_id == 30,
                   "lane allocation is not independent from physical strips");

  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto marking_count = [&](std::uint64_t boundary_id) {
    return std::count_if(
        state.derived().markings.begin(), state.derived().markings.end(),
        [segment_id = segment.value, boundary_id](const auto& marking) {
          return marking.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                 marking.owner.segment_id == segment_id &&
                 marking.boundary_id == boundary_id;
        });
  };
  ROAD_TEST_EXPECT(marking_count(150) == 1 && marking_count(250) == 1,
                   "outer lines are not anchored to carriageway/shoulder boundaries");
  ROAD_TEST_EXPECT(marking_count(100) == 0 && marking_count(300) == 0,
                   "curb boundaries are still used as shoulder spacing");

  city::road::RoadLayoutTemplate continuous{};
  continuous.strips = {
      {10, StripFunction::kCarriageway, 6.0, 0.0,
       builtin_surface_styles::kMedian},
  };
  continuous.lane_bands = {
      {100, 10, 0.0, 3.0},
      {200, 10, 3.0, 6.0},
  };
  continuous.alignment_offset_from_left_m =
      road_fixture::CentredAlignmentOffset(continuous);
  const auto template_id =
      state.AddRoadLayoutTemplate(city::road::AddRoadLayoutTemplateRequest{
          std::move(continuous)});
  ROAD_TEST_EXPECT(template_id.ok, template_id.error);
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto restored = std::find_if(
      loaded.value.graph().layout_templates.begin(),
      loaded.value.graph().layout_templates.end(),
      [id = template_id.value](const auto& section) {
        return section.id == id;
      });
  ROAD_TEST_EXPECT(
      restored != loaded.value.graph().layout_templates.end() &&
          restored->strips.size() == 1 &&
          restored->lane_bands.size() == 2 &&
          restored->strips.front().function == StripFunction::kCarriageway &&
          restored->strips.front().style_id ==
              builtin_surface_styles::kMedian,
      "one physical strip with multiple lane bands did not round-trip");
  return true;
}

// A drawn interval must bend one way only. An interval that leaves in one
// heading and arrives along its own chord reverses its bend in the middle,
// which is the S the curve tool used to draw.
bool CRV_drawn_curve_bends_one_way_per_interval(std::string& failure) {
  const auto turn_sign_flips = [](const city::road::BezierSpan& span) {
    const auto first_derivative = [&span](double t) {
      const double u = 1.0 - t;
      return Vec2d{3.0 * u * u * (span.p1.x - span.p0.x) + 6.0 * u * t * (span.p2.x - span.p1.x) +
                       3.0 * t * t * (span.p3.x - span.p2.x),
                   3.0 * u * u * (span.p1.y - span.p0.y) + 6.0 * u * t * (span.p2.y - span.p1.y) +
                       3.0 * t * t * (span.p3.y - span.p2.y)};
    };
    const auto second_derivative = [&span](double t) {
      const double u = 1.0 - t;
      return Vec2d{6.0 * u * (span.p2.x - 2.0 * span.p1.x + span.p0.x) +
                       6.0 * t * (span.p3.x - 2.0 * span.p2.x + span.p1.x),
                   6.0 * u * (span.p2.y - 2.0 * span.p1.y + span.p0.y) +
                       6.0 * t * (span.p3.y - 2.0 * span.p2.y + span.p1.y)};
    };
    int flips = 0;
    int previous = 0;
    for (int sample = 0; sample <= 20; ++sample) {
      const double t = sample / 20.0;
      const Vec2d d1 = first_derivative(t);
      const Vec2d d2 = second_derivative(t);
      const double turn = d1.x * d2.y - d1.y * d2.x;
      if (std::abs(turn) < 1e-9) continue;
      const int sign = turn > 0.0 ? 1 : -1;
      if (previous != 0 && sign != previous) ++flips;
      previous = sign;
    }
    return flips;
  };

  // A hand-drawn arc: every point turns the same way.
  const std::array<Vec2d, 5> points{Vec2d{0.0, 0.0}, Vec2d{40.0, 0.0}, Vec2d{70.0, 22.0},
                                    Vec2d{86.0, 54.0}, Vec2d{88.0, 90.0}};
  RoadState state{};
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest first{};
  first.alignment = MakePath({MakeLine(points[0], points[1])});
  first.layout_template = section;
  first.intent = city::road::SegmentShapeIntent::kCurve;
  const auto added = state.AddSegment(first);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), added.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "drawn curve corridor is missing");
  const city::road::RoadCorridorId corridor_id = corridor->id;

  city::road::RoadSegmentId last_segment = added.value;
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    const auto previous = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [last_segment](const auto& segment) { return segment.id == last_segment; });
    ROAD_TEST_EXPECT(previous != state.graph().segments.end(), "drawn curve terminal segment is missing");
    city::road::ExtendCorridorFromEndRequest extension{};
    extension.corridor_id = corridor_id;
    extension.endpoint_node_id = previous->node_b;
    extension.extension = city::road::PreviewDrawnInterval(
        state.graph(), corridor_id, previous->node_b, points[index],
        points[index + 1], city::road::SegmentShapeIntent::kCurve);
    extension.layout_template = section;
    extension.intent = city::road::SegmentShapeIntent::kCurve;
    const auto extended = state.ExtendCorridorFromEnd(extension);
    ROAD_TEST_EXPECT(extended.ok, extended.error);
    last_segment = extended.value;
  }

  for (const auto& segment : state.graph().segments) {
    const Path* alignment = FindCanonicalAlignment(state.derived(), segment.id);
    ROAD_TEST_EXPECT(alignment != nullptr, "drawn curve alignment is missing");
    for (const auto& span : alignment->spans) {
      ROAD_TEST_EXPECT(turn_sign_flips(span) == 0,
                       "drawn interval reverses its bend: segment " + std::to_string(segment.id));
    }
  }

  // Consecutive intervals meet without a corner, so the drawn road reads as one
  // continuous curve rather than a chain of hinges.
  for (const auto& connection : state.derived().connections) {
    ROAD_TEST_EXPECT(connection.kind != city::road::NodeConnectionKind::kCorner,
                     "drawn curve produced a corner between two intervals");
  }
  return true;
}

bool CRV_control_handles_are_the_tangent_authority(std::string& failure) {
  const Path straight = MakePath({MakeLine({0.0, 0.0}, {10.0, 0.0})});
  const Path curve = MakePath(
      {MakeBezier({10.0, 0.0}, {14.0, 0.0}, {18.0, 4.0}, {20.0, 10.0})});
  const auto straight_length = PathLength(straight);
  ROAD_TEST_EXPECT(straight_length.ok, straight_length.error);
  const auto straight_tangent = city::road::internal::tangent_at(
      straight, straight_length.value);
  const auto curve_tangent = city::road::internal::tangent_at(curve, 0.0);
  ROAD_TEST_EXPECT(straight_tangent.ok && curve_tangent.ok,
                   "straight-to-curve endpoint tangent is missing");
  ROAD_TEST_EXPECT(
      std::abs(straight_tangent.value.x - 1.0) <= 1e-12 &&
          std::abs(straight_tangent.value.y) <= 1e-12 &&
          std::abs(curve_tangent.value.x - 1.0) <= 1e-12 &&
          std::abs(curve_tangent.value.y) <= 1e-12,
      "endpoint tangent does not follow its cubic control handle");

  const auto offset_point = [](Vec2d point, Vec2d tangent, double lateral_m) {
    return Vec2d{point.x - tangent.y * lateral_m,
                 point.y + tangent.x * lateral_m};
  };
  const Vec2d straight_left =
      offset_point({10.0, 0.0}, straight_tangent.value, 4.0);
  const Vec2d curve_left =
      offset_point({10.0, 0.0}, curve_tangent.value, 4.0);
  ROAD_TEST_EXPECT(std::hypot(straight_left.x - curve_left.x,
                              straight_left.y - curve_left.y) <= 1e-12,
                   "straight-to-curve left edge is discontinuous");

  const Path reversed_curve = MakePath(
      {MakeBezier({20.0, 10.0}, {18.0, 4.0}, {14.0, 0.0}, {10.0, 0.0})});
  const auto reversed_length = PathLength(reversed_curve);
  ROAD_TEST_EXPECT(reversed_length.ok, reversed_length.error);
  const auto reversed_tangent = city::road::internal::tangent_at(
      reversed_curve, reversed_length.value);
  ROAD_TEST_EXPECT(
      reversed_tangent.ok &&
          std::abs(reversed_tangent.value.x + curve_tangent.value.x) <= 1e-12 &&
          std::abs(reversed_tangent.value.y + curve_tangent.value.y) <= 1e-12,
      "reversing the same cubic changed its endpoint tangent geometry");

  const Path internal = MakePath(
      {MakeLine({0.0, 0.0}, {10.0, 0.0}),
       MakeBezier({10.0, 0.0}, {14.0, 0.0}, {18.0, 4.0}, {20.0, 10.0})});
  const auto internal_tangent =
      city::road::internal::tangent_at(internal, straight_length.value);
  ROAD_TEST_EXPECT(
      internal_tangent.ok && std::abs(internal_tangent.value.x - 1.0) <= 1e-12 &&
          std::abs(internal_tangent.value.y) <= 1e-12,
      "a G1 internal span knot was averaged across both spans");
  return true;
}

bool CRV_explicit_extension_controls_are_not_rewritten(std::string& failure) {
  RoadState state{};
  const auto layout =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeBezier({0.0, 0.0}, {5.0, 0.0}, {10.0, 5.0},
                           {10.0, 10.0})}),
      layout, city::road::SegmentShapeIntent::kCurve});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto* corridor = FindCorridorForSegment(state.graph(), first.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "curve corridor is missing");
  const RoadNodeId endpoint = state.graph().segments.front().node_b;
  const Path explicit_curve = MakePath(
      {MakeBezier({10.0, 10.0}, {15.0, 10.0}, {20.0, 15.0}, {20.0, 20.0})});
  const auto extended = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor->id, endpoint, explicit_curve, layout,
          city::road::SegmentShapeIntent::kCurve});
  ROAD_TEST_EXPECT(extended.ok, extended.error);
  const RoadSegment* stored = nullptr;
  for (const RoadSegment& segment : state.graph().segments) {
    if (segment.id == extended.value) stored = &segment;
  }
  ROAD_TEST_EXPECT(stored != nullptr, "explicit curve extension is missing");
  const auto alignment = city::road::DeriveCanonicalAlignment(
      {10.0, 10.0}, {20.0, 20.0}, stored->shape);
  ROAD_TEST_EXPECT(alignment.ok && alignment.value.spans.size() == 1,
                   "explicit curve extension has no canonical span");
  const auto same_point = [](Vec2d a, Vec2d b) {
    return a.x == b.x && a.y == b.y;
  };
  const auto& actual = alignment.value.spans.front();
  const auto& expected = explicit_curve.spans.front();
  ROAD_TEST_EXPECT(same_point(actual.p0, expected.p0) &&
                       same_point(actual.p1, expected.p1) &&
                       same_point(actual.p2, expected.p2) &&
                       same_point(actual.p3, expected.p3),
                   "curve extension rewrote user control points");

  RoadState straight_state{};
  const auto straight_layout = road_fixture::AddLayout(
      straight_state, road_fixture::BidirectionalLayout(0));
  const auto straight = straight_state.AddSegment(
      city::road::AddSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {10.0, 0.0})}), straight_layout});
  ROAD_TEST_EXPECT(straight.ok, straight.error);
  const auto* straight_corridor =
      FindCorridorForSegment(straight_state.graph(), straight.value);
  ROAD_TEST_EXPECT(straight_corridor != nullptr,
                   "straight predecessor corridor is missing");
  const RoadNodeId straight_endpoint =
      straight_state.graph().segments.front().node_b;
  const Path preview = city::road::PreviewDrawnInterval(
      straight_state.graph(), straight_corridor->id, straight_endpoint,
      {10.0, 0.0}, {20.0, 10.0},
      city::road::SegmentShapeIntent::kCurve);
  const Vec2d preview_tangent{
      preview.spans.front().p1.x - preview.spans.front().p0.x,
      preview.spans.front().p1.y - preview.spans.front().p0.y};
  ROAD_TEST_EXPECT(preview_tangent.x > 0.0 &&
                       std::abs(preview_tangent.y) <= 1e-12,
                   "curve preview did not inherit a straight predecessor tangent");
  return true;
}

// --- Layout alignment origin -------------------------------------------------
// The layout says how far the alignment sits from its left outer end, so a
// layout that gains width on one side moves only that side, and the alignment
// itself stays on the segment path.

city::road::RoadLayoutTemplate OffCentreLayout(double alignment_offset_m) {
  city::road::RoadLayoutTemplate layout = road_fixture::BidirectionalLayout(0);
  layout.alignment_offset_from_left_m = alignment_offset_m;
  return layout;
}

std::optional<double> boundary_lateral(const city::road::SectionEvaluation& section,
                                       BoundaryId id) {
  std::optional<double> found{};
  for (const auto& boundary : section.boundaries) {
    if (boundary.boundary_id != id) continue;
    if (!found.has_value() || boundary.lateral_m < *found) found = boundary.lateral_m;
  }
  return found;
}

std::vector<double> section_laterals(const RoadState& state) {
  std::vector<double> out{};
  for (const auto* section : road_test_view::sections(state.derived())) {
    for (const auto& boundary : section->boundaries) out.push_back(boundary.lateral_m);
  }
  return out;
}

bool draw_straight_road(RoadState& state, city::road::RoadLayoutTemplateId layout,
                        double length_m) {
  const auto added = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {length_m, 0.0})}), layout});
  return added.ok;
}

bool layout_alignment_origin_measures_from_the_left_outer_end(std::string& failure) {
  // Bidirectional: 2 + 0.2 + 3 + 0 + 3 + 0.2 + 2 = 10.4m wide.
  RoadState centred{};
  const auto centred_layout =
      road_fixture::AddLayout(centred, road_fixture::BidirectionalLayout(0));
  ROAD_TEST_EXPECT(centred_layout != 0, "centred layout was rejected");
  ROAD_TEST_EXPECT(draw_straight_road(centred, centred_layout, 60.0),
                   "centred road could not be drawn");
  const auto centred_sections = road_test_view::sections(centred.derived());
  ROAD_TEST_EXPECT(!centred_sections.empty(), "centred road has no sections");
  for (const auto* section : centred_sections) {
    ROAD_TEST_EXPECT(std::abs(section->boundaries.front().lateral_m + 5.0) < 1e-12 &&
                         std::abs(section->boundaries.back().lateral_m - 5.0) < 1e-12,
                     "a symmetric layout no longer reaches 5.0m each way");
    const auto divider = boundary_lateral(*section, 200);
    ROAD_TEST_EXPECT(divider.has_value() && std::abs(*divider) < 1e-12,
                     "a symmetric layout no longer puts its divider on the alignment");
  }

  // The same layout with the alignment 3m from its left outer end: the road is
  // the same width, it just sits differently against the path it was drawn on.
  RoadState off_centre{};
  const auto off_centre_layout =
      road_fixture::AddLayout(off_centre, OffCentreLayout(3.0));
  ROAD_TEST_EXPECT(off_centre_layout != 0, "off-centre layout was rejected");
  ROAD_TEST_EXPECT(draw_straight_road(off_centre, off_centre_layout, 60.0),
                   "off-centre road could not be drawn");
  const auto off_centre_sections = road_test_view::sections(off_centre.derived());
  ROAD_TEST_EXPECT(!off_centre_sections.empty(), "off-centre road has no sections");
  for (const auto* section : off_centre_sections) {
    ROAD_TEST_EXPECT(std::abs(section->boundaries.front().lateral_m + 3.0) < 1e-12 &&
                         std::abs(section->boundaries.back().lateral_m - 7.0) < 1e-12,
                     "the alignment offset did not decide where the layout starts");
  }

  // The alignment itself is the drawn path either way.
  const Path* centred_alignment = city::road::FindCanonicalAlignment(
      centred.derived(), centred.graph().segments.front().id);
  const Path* off_centre_alignment = city::road::FindCanonicalAlignment(
      off_centre.derived(), off_centre.graph().segments.front().id);
  ROAD_TEST_EXPECT(centred_alignment != nullptr && off_centre_alignment != nullptr,
                   "a drawn road has no canonical alignment");
  ROAD_TEST_EXPECT(
      centred_alignment->spans.size() == off_centre_alignment->spans.size(),
      "the alignment offset changed the alignment span count");
  for (std::size_t index = 0; index < centred_alignment->spans.size(); ++index) {
    const auto& a = centred_alignment->spans[index];
    const auto& b = off_centre_alignment->spans[index];
    ROAD_TEST_EXPECT(a.p0.x == b.p0.x && a.p0.y == b.p0.y && a.p3.x == b.p3.x &&
                         a.p3.y == b.p3.y,
                     "the alignment moved when the layout offset changed");
  }
  return true;
}

bool layout_widened_on_one_side_keeps_the_other_side_still(std::string& failure) {
  const auto laterals_for = [](const city::road::RoadLayoutTemplate& layout,
                               std::vector<double>& out) {
    RoadState state{};
    const auto id = road_fixture::AddLayout(state, layout);
    if (id == 0 || !draw_straight_road(state, id, 60.0)) return false;
    const auto sections = road_test_view::sections(state.derived());
    if (sections.empty()) return false;
    out.clear();
    for (const auto& boundary : sections.front()->boundaries)
      out.push_back(boundary.lateral_m);
    return true;
  };

  std::vector<double> base{};
  ROAD_TEST_EXPECT(laterals_for(road_fixture::BidirectionalLayout(0), base),
                   "base layout could not be drawn");
  ROAD_TEST_EXPECT(base.size() >= 4, "base layout has too few boundary samples");

  // The left walkway grows by 1m and the alignment keeps its distance to the
  // right end, so only the left outer end moves.
  city::road::RoadLayoutTemplate wider_left = road_fixture::BidirectionalLayout(0);
  wider_left.strips.front().width_m += 1.0;
  wider_left.alignment_offset_from_left_m += 1.0;
  std::vector<double> left{};
  ROAD_TEST_EXPECT(laterals_for(wider_left, left), "left-widened layout could not be drawn");
  ROAD_TEST_EXPECT(left.size() == base.size(),
                   "widening one side changed the boundary sample count");
  ROAD_TEST_EXPECT(std::abs((left.front() - base.front()) + 1.0) < 1e-12,
                   "the widened left end did not move outward by its own width");
  for (std::size_t index = 1; index < base.size(); ++index) {
    ROAD_TEST_EXPECT(std::abs(left[index] - base[index]) < 1e-12,
                     "widening the left side moved a boundary on the other side");
  }

  // The right walkway grows by 1m and the alignment keeps its distance to the
  // left end, so only the right outer end moves.
  city::road::RoadLayoutTemplate wider_right = road_fixture::BidirectionalLayout(0);
  wider_right.strips.back().width_m += 1.0;
  std::vector<double> right{};
  ROAD_TEST_EXPECT(laterals_for(wider_right, right), "right-widened layout could not be drawn");
  ROAD_TEST_EXPECT(right.size() == base.size(),
                   "widening one side changed the boundary sample count");
  ROAD_TEST_EXPECT(std::abs((right.back() - base.back()) - 1.0) < 1e-12,
                   "the widened right end did not move outward by its own width");
  for (std::size_t index = 0; index + 1 < base.size(); ++index) {
    ROAD_TEST_EXPECT(std::abs(right[index] - base[index]) < 1e-12,
                     "widening the right side moved a boundary on the other side");
  }
  return true;
}

bool add_lane_on_either_side_keeps_existing_lanes_still(std::string& failure) {
  for (const city::road::RoadSide side :
       {city::road::RoadSide::kLeft, city::road::RoadSide::kRight}) {
    RoadState state{};
    const auto layout = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), layout});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    const auto* corridor = city::road::FindCorridorForSegment(state.graph(), segment.value);
    ROAD_TEST_EXPECT(corridor != nullptr, "added lane fixture corridor is missing");

    // The new lane goes next to the outermost lane on the chosen side, so
    // everything from there to the far outer end keeps its lateral position.
    const std::vector<BoundaryId> held =
        side == city::road::RoadSide::kLeft
            ? std::vector<BoundaryId>{250ULL, 300ULL}
            : std::vector<BoundaryId>{100ULL, 150ULL, 200ULL};
    const auto before_sections = road_test_view::sections(state.derived());
    ROAD_TEST_EXPECT(!before_sections.empty(), "added lane fixture has no sections");
    std::vector<double> before{};
    for (const BoundaryId id : held) {
      const auto lateral = boundary_lateral(*before_sections.front(), id);
      ROAD_TEST_EXPECT(lateral.has_value(), "added lane fixture boundary is missing");
      before.push_back(*lateral);
    }

    city::road::AddLaneRequest request{};
    request.corridor_id = corridor->id;
    request.direction = city::road::LaneTravelDirection::kAlongSegment;
    request.side = side;
    ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                     "added lane range could not be resolved");
    request.lane_width_m = 3.0;
    const auto added = state.AddLane(request);
    ROAD_TEST_EXPECT(added.ok, added.error);

    for (const auto* section : road_test_view::sections(state.derived())) {
      for (std::size_t index = 0; index < held.size(); ++index) {
        const auto lateral = boundary_lateral(*section, held[index]);
        ROAD_TEST_EXPECT(lateral.has_value() &&
                             std::abs(*lateral - before[index]) < 1e-9,
                         "adding a lane moved boundary " +
                             std::to_string(held[index]) + " on the other side");
      }
    }
  }
  return true;
}

bool layout_alignment_basis_is_continuous_across_a_transition(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto* corridor = city::road::FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "transition fixture corridor is missing");

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "transition range could not be resolved");
  request.lane_width_m = 3.0;
  ROAD_TEST_EXPECT(state.AddLane(request).ok, "transition lane was not added");

  // The offset in force at a distance is what the left outer end reports. It
  // has to grow with the road rather than jump back to half the total width.
  const auto sections = road_test_view::sections(state.derived());
  ROAD_TEST_EXPECT(sections.size() >= 3, "transition produced too few sections");
  double previous = -sections.front()->boundaries.front().lateral_m;
  double widest = 0.0;
  for (const auto* section : sections) {
    const double offset = -section->boundaries.front().lateral_m;
    const double width = section->boundaries.back().lateral_m -
                         section->boundaries.front().lateral_m;
    ROAD_TEST_EXPECT(std::abs(offset - previous) < 1e-9,
                     "the alignment basis jumped across the transition");
    widest = std::max(widest, width);
    previous = offset;
  }
  ROAD_TEST_EXPECT(widest > sections.front()->boundaries.back().lateral_m -
                                 sections.front()->boundaries.front().lateral_m + 1e-9,
                   "the transition never widened the road");
  ROAD_TEST_EXPECT(std::abs(previous * 2.0 - widest) > 1e-6,
                   "the widened road was recentred on half its total width");
  return true;
}

bool off_centre_layout_keeps_its_alignment_through_a_junction(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, OffCentreLayout(3.0));
  ROAD_TEST_EXPECT(layout != 0, "off-centre junction layout was rejected");
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, first.value,
          40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1, "the three roads did not form one junction");

  for (const auto& gate : road_test_view::gates_of(*junctions.front())) {
    ROAD_TEST_EXPECT(!gate.boundaries.empty(), "a junction gate carries no section");
    const double left = gate.boundaries.front().lateral_m;
    const double right = gate.boundaries.back().lateral_m;
    ROAD_TEST_EXPECT(std::abs(left + 3.0) < 1e-9 && std::abs(right - 7.0) < 1e-9,
                     "a junction gate re-measured the layout from somewhere else");
    ROAD_TEST_EXPECT(std::abs((left + right) * 0.5) > 1e-6,
                     "the off-centre layout was recentred between its outer edges");
  }
  return true;
}

bool saved_layout_alignment_offset_survives_reload(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, OffCentreLayout(3.0));
  ROAD_TEST_EXPECT(layout != 0, "off-centre layout was rejected");
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0), "off-centre road could not be drawn");
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto& restored = loaded.value.graph().layout_templates;
  ROAD_TEST_EXPECT(restored.size() == 1 &&
                       restored.front().alignment_offset_from_left_m == 3.0,
                   "the saved alignment offset did not come back");
  ROAD_TEST_EXPECT(section_laterals(state) == section_laterals(loaded.value),
                   "reloading an off-centre road produced different geometry");
  const auto saved_again = loaded.value.Save();
  ROAD_TEST_EXPECT(saved_again.ok && saved_again.value == saved.value,
                   "an off-centre road did not save back bit-identically");
  return true;
}

bool version_fourteen_archive_defaults_the_corner_radius(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0), "road could not be drawn");
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);

  // Version 14 predates per-road corner radius but already stores marking
  // placement. Loading it must add only the historical 4 m default.
  std::string legacy{};
  std::istringstream lines{saved.value};
  std::string line{};
  std::size_t dropped = 0;
  while (std::getline(lines, line)) {
    if (line.find(".corner_radius_m=") != std::string::npos ||
        line.find(".elevation_m=") != std::string::npos) {
      ++dropped;
      continue;
    }
    if (line == "road_graph_version=16") line = "road_graph_version=14";
    legacy += line;
    legacy += '\n';
  }
  ROAD_TEST_EXPECT(dropped > 0, "the archive under test states no corner radius");

  const auto loaded = RoadState::Load(legacy);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  for (const auto& segment : loaded.value.graph().segments) {
    ROAD_TEST_EXPECT(segment.corner_radius_m == 4.0,
                     "a version 14 road did not receive the historical radius");
  }
  // The migration states a radius the archive never had, so it cannot save
  // back as the bytes it came from.
  const auto resaved = loaded.value.Save();
  ROAD_TEST_EXPECT(resaved.ok, resaved.error);
  ROAD_TEST_EXPECT(resaved.value.starts_with("road_graph_version=16\n"),
                   "a migrated road did not save as the current version");
  const auto reloaded = RoadState::Load(resaved.value);
  ROAD_TEST_EXPECT(reloaded.ok, reloaded.error);
  const auto settled = reloaded.value.Save();
  ROAD_TEST_EXPECT(settled.ok && settled.value == resaved.value,
                   "a migrated road did not settle after one save");
  return true;
}

bool junction_corner_radius_is_authoritative_and_inherited(std::string& failure) {
  RoadState state{};
  const auto layout =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  ROAD_TEST_EXPECT(layout != 0, "corner-radius section was rejected");

  city::road::AddSegmentRequest base_request{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), layout};
  base_request.corner_radius_m = 7.0;
  const auto base = state.AddSegment(std::move(base_request));
  ROAD_TEST_EXPECT(base.ok, base.error);

  city::road::AddSegmentConnectedToSegmentRequest branch_request{
      MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, base.value,
      40.0};
  branch_request.corner_radius_m = 7.0;
  const auto branch =
      state.AddSegmentConnectedToSegment(std::move(branch_request));
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3,
                   "branch split did not produce three local segments");
  for (const auto& segment : state.graph().segments) {
    ROAD_TEST_EXPECT(segment.corner_radius_m == 7.0,
                     "new or split road lost its corner radius");
  }
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1,
                   "corner-radius roads did not form one junction");
  ROAD_TEST_EXPECT(
      junctions.front()->junction_corners.size() == 3 &&
          std::all_of(junctions.front()->junction_corners.begin(),
                      junctions.front()->junction_corners.end(),
                      [](const auto& corner) { return corner.radius_m == 7.0; }),
      "junction corner pairs ignored the saved road radius");

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  for (const auto& segment : loaded.value.graph().segments) {
    ROAD_TEST_EXPECT(segment.corner_radius_m == 7.0,
                     "saved corner radius did not survive load");
  }
  const auto saved_again = loaded.value.Save();
  ROAD_TEST_EXPECT(saved_again.ok && saved_again.value == saved.value,
                   "corner-radius archive was not byte stable");
  return true;
}

bool corner_radius_extension_validation_and_mixed_resolution(std::string& failure) {
  RoadState state{};
  const auto layout =
      road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  ROAD_TEST_EXPECT(layout != 0, "radius extension section was rejected");
  city::road::AddSegmentRequest first_request{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), layout};
  first_request.corner_radius_m = 6.0;
  const auto first = state.AddSegment(std::move(first_request));
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto* corridor = city::road::FindCorridorForSegment(state.graph(), first.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "radius extension corridor is missing");
  const auto first_segment_it = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&first](const RoadSegment& segment) { return segment.id == first.value; });
  ROAD_TEST_EXPECT(first_segment_it != state.graph().segments.end(),
                   "radius extension source is missing");
  const auto extended = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor->id, first_segment_it->node_b,
          MakePath({MakeLine({40.0, 0.0}, {40.0, 40.0})}), layout});
  ROAD_TEST_EXPECT(extended.ok, extended.error);
  const auto extension_it = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&extended](const RoadSegment& segment) {
        return segment.id == extended.value;
      });
  ROAD_TEST_EXPECT(extension_it != state.graph().segments.end() &&
                       extension_it->corner_radius_m == 6.0,
                   "corridor extension did not inherit the terminal radius");

  const auto saved_before_invalid = state.Save();
  ROAD_TEST_EXPECT(saved_before_invalid.ok, saved_before_invalid.error);
  city::road::AddSegmentRequest invalid_request{
      MakePath({MakeLine({80.0, 0.0}, {100.0, 0.0})}), layout};
  invalid_request.corner_radius_m = -1.0;
  const auto invalid = state.AddSegment(std::move(invalid_request));
  ROAD_TEST_EXPECT(!invalid.ok &&
                       invalid.failure_category ==
                           CommitFailureCategory::kInvalidInput,
                   "negative corner radius was not an input error");
  const auto saved_after_invalid = state.Save();
  ROAD_TEST_EXPECT(saved_after_invalid.ok &&
                       saved_after_invalid.value == saved_before_invalid.value,
                   "invalid corner radius mutated the road state");

  // A degree-two corner has one pair, so equal inherited values resolve once.
  const auto corners = road_test_view::corners(state.derived());
  ROAD_TEST_EXPECT(corners.size() == 1, "mixed-radius extension made no corner");
  ROAD_TEST_EXPECT(corners.front()->corner_radius_m == 6.0,
                   "equal inherited radii did not resolve deterministically");

  RoadState mixed{};
  const auto mixed_layout =
      road_fixture::AddLayout(mixed, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest mixed_base_request{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), mixed_layout};
  mixed_base_request.corner_radius_m = 3.0;
  const auto mixed_base = mixed.AddSegment(std::move(mixed_base_request));
  ROAD_TEST_EXPECT(mixed_base.ok, mixed_base.error);
  city::road::AddSegmentConnectedToSegmentRequest mixed_branch_request{
      MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), mixed_layout,
      mixed_base.value, 40.0};
  mixed_branch_request.corner_radius_m = 8.0;
  const auto mixed_branch = mixed.AddSegmentConnectedToSegment(
      std::move(mixed_branch_request));
  ROAD_TEST_EXPECT(mixed_branch.ok, mixed_branch.error);
  const auto mixed_junctions = road_test_view::junctions(mixed.derived());
  ROAD_TEST_EXPECT(
      mixed_junctions.size() == 1 &&
          std::all_of(mixed_junctions.front()->junction_corners.begin(),
                      mixed_junctions.front()->junction_corners.end(),
                      [](const auto& corner) { return corner.radius_m == 3.0; }),
      "mixed approach radii did not resolve per adjacent pair");

  RoadState reordered{};
  const auto reordered_layout = road_fixture::AddLayout(
      reordered, road_fixture::BidirectionalLayout(0));
  city::road::AddSegmentRequest north_request{
      MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), reordered_layout};
  north_request.corner_radius_m = 8.0;
  const auto north = reordered.AddSegment(std::move(north_request));
  ROAD_TEST_EXPECT(north.ok, north.error);
  const RoadNodeId node = reordered.graph().segments.front().node_a;
  for (const Path& path : {
           MakePath({MakeLine({-40.0, 0.0}, {0.0, 0.0})}),
           MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})})}) {
    city::road::AddSegmentConnectedToRequest request{
        path, reordered_layout, node,
        path.spans.front().p3.x == 0.0 ? EndpointRole::kEnd
                                       : EndpointRole::kStart};
    request.corner_radius_m = 3.0;
    const auto added = reordered.AddSegmentConnectedTo(std::move(request));
    ROAD_TEST_EXPECT(added.ok, added.error);
  }
  const auto reordered_junctions =
      road_test_view::junctions(reordered.derived());
  ROAD_TEST_EXPECT(
      reordered_junctions.size() == 1 &&
          std::all_of(reordered_junctions.front()->junction_corners.begin(),
                      reordered_junctions.front()->junction_corners.end(),
                      [](const auto& corner) { return corner.radius_m == 3.0; }),
      "mixed pair radius changed with approach creation order");
  return true;
}

bool junction_zero_radius_keeps_a_t_junction_local(std::string& failure) {
  const auto build_t = [](double base_radius_m, double branch_radius_m,
                          RoadState& state, std::string& failure) {
    const auto layout =
        road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    ROAD_TEST_EXPECT(layout != 0, "zero-radius T section was rejected");
    city::road::AddSegmentRequest base_request{
        MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), layout};
    base_request.corner_radius_m = base_radius_m;
    const auto base = state.AddSegment(std::move(base_request));
    ROAD_TEST_EXPECT(base.ok, base.error);
    city::road::AddSegmentConnectedToSegmentRequest branch_request{
        MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, base.value,
        40.0};
    branch_request.corner_radius_m = branch_radius_m;
    const auto branch =
        state.AddSegmentConnectedToSegment(std::move(branch_request));
    ROAD_TEST_EXPECT(branch.ok, branch.error);
    return true;
  };

  RoadState all_zero{};
  ROAD_TEST_EXPECT(build_t(0.0, 0.0, all_zero, failure), failure);
  const auto all_zero_junctions =
      road_test_view::junctions(all_zero.derived());
  ROAD_TEST_EXPECT(all_zero_junctions.size() == 1,
                   "all-zero T did not produce one junction");
  for (const auto& approach : all_zero_junctions.front()->approaches) {
    ROAD_TEST_EXPECT(std::abs(approach.resolved_setback_m - 5.0) <= 1e-6,
                     "all-zero 10m T gate was not 5m from the node");
  }
  for (const auto& point :
       all_zero_junctions.front()->junction_geometry.surface_regions.front().perimeter) {
    ROAD_TEST_EXPECT(std::hypot(point.x, point.y) <= std::sqrt(50.0) + 1e-6,
                     "all-zero T junction perimeter extends beyond its 5m gates");
  }
  const bool has_curved_curb_boundary = std::any_of(
      all_zero_junctions.front()->junction_geometry.perimeter_curves.begin(),
      all_zero_junctions.front()->junction_geometry.perimeter_curves.end(),
      [](const auto& curve) {
        if (curve.role != BoundaryRole::kCurb ||
            curve.points.size() < 3)
          return false;
        const Vec3d& first = curve.points.front();
        const Vec3d& last = curve.points.back();
        return std::any_of(
            curve.points.begin() + 1, curve.points.end() - 1,
            [&first, &last](const Vec3d& point) {
              return std::abs((last.x - first.x) * (point.y - first.y) -
                              (last.y - first.y) * (point.x - first.x)) >
                     1e-6;
            });
      });
  ROAD_TEST_EXPECT(
      has_curved_curb_boundary,
      "zero-radius T made the road-sidewalk curb boundary a straight chord");

  RoadState mixed{};
  ROAD_TEST_EXPECT(build_t(4.0, 0.0, mixed, failure), failure);
  const auto mixed_junctions = road_test_view::junctions(mixed.derived());
  ROAD_TEST_EXPECT(mixed_junctions.size() == 1,
                   "mixed-radius T did not produce one junction");
  for (const auto& approach : mixed_junctions.front()->approaches) {
    ROAD_TEST_EXPECT(std::abs(approach.resolved_setback_m - 5.0) <= 1e-6,
                     "new zero-radius branch was replaced by a 4m connection default");
  }
  return true;
}

// --- Edge structures ---------------------------------------------------------
// GutteredLayout puts an L-shaped gutter where the curb was: a top face in the
// walkway, a vertical face on the datum, then a channel and a lip reaching back
// into the roadway.

bool l_gutter_comes_out_of_the_widths_beside_it(std::string& failure) {
  RoadState curbed_state{};
  const auto curbed_layout =
      road_fixture::AddLayout(curbed_state, road_fixture::BidirectionalLayout(0));
  ROAD_TEST_EXPECT(draw_straight_road(curbed_state, curbed_layout, 60.0),
                   "the curbed road could not be drawn");
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  ROAD_TEST_EXPECT(layout != 0, "the guttered layout was rejected");
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0),
                   "the guttered road could not be drawn");

  const auto curbed = road_test_view::sections(curbed_state.derived());
  const auto guttered = road_test_view::sections(state.derived());
  ROAD_TEST_EXPECT(!curbed.empty() && !guttered.empty(), "a road produced no sections");
  const auto& before = curbed.front()->boundaries;
  const auto& after = guttered.front()->boundaries;

  // Swapping a 0.2 curb for a gutter that reaches 0.45 across changes nothing
  // about how wide the road is or where it sits.
  ROAD_TEST_EXPECT(std::abs((after.back().lateral_m - after.front().lateral_m) -
                            (before.back().lateral_m - before.front().lateral_m)) < 1e-9,
                   "the gutter changed the width of the road it sits in");
  ROAD_TEST_EXPECT(std::abs(after.front().lateral_m + 5.0) < 1e-9 &&
                       std::abs(after.back().lateral_m - 5.0) < 1e-9,
                   "the gutter moved the layout's outer ends");
  const auto divider = boundary_lateral(*guttered.front(), 200);
  ROAD_TEST_EXPECT(divider.has_value() && std::abs(*divider) < 1e-9,
                   "the gutter moved the centre line");

  // Its top face is inside the 2.0m walkway, its channel and lip inside the
  // 3.0m carriageway: 0.1 + 0.1 of walkway and 0.25 + 0.1 of roadway.
  ROAD_TEST_EXPECT(after.size() == 13, "the guttered section does not expose every face");
  const std::array<double, 5> left_gutter{-3.1, -3.0, -3.0, -2.75, -2.65};
  for (std::size_t index = 0; index < left_gutter.size(); ++index) {
    ROAD_TEST_EXPECT(std::abs(after[index + 1].lateral_m - left_gutter[index]) < 1e-9,
                     "the left gutter is not where its own dimensions put it");
  }
  // The wall stands 0.1 above the channel, and the channel climbs back to the
  // road, leaving the top 0.07 above the road edge.
  ROAD_TEST_EXPECT(std::abs((after[2].height_m - after[3].height_m) - 0.1) < 1e-9,
                   "the gutter's vertical face is not its own height");
  ROAD_TEST_EXPECT(std::abs((after[2].height_m - after[5].height_m) - 0.07) < 1e-9,
                   "the gutter top does not stand where its channel grades put it");
  return true;
}

bool l_gutter_dimensions_leave_the_road_where_it_was(std::string& failure) {
  const auto laterals_of = [](city::road::RoadLayoutTemplate layout,
                              std::vector<double>& out) {
    RoadState state{};
    const auto id = road_fixture::AddLayout(state, std::move(layout));
    if (id == 0 || !draw_straight_road(state, id, 60.0)) return false;
    const auto sections = road_test_view::sections(state.derived());
    if (sections.empty()) return false;
    out.clear();
    for (const auto& boundary : sections.front()->boundaries)
      out.push_back(boundary.lateral_m);
    return true;
  };
  std::vector<double> narrow{};
  ROAD_TEST_EXPECT(laterals_of(road_fixture::GutteredLayout(0), narrow),
                   "the guttered road could not be drawn");

  // A deeper, wider gutter: the same structure with more of everything.
  city::road::RoadLayoutTemplate wider = road_fixture::GutteredLayout(0);
  for (city::road::BoundaryProfile& boundary : wider.boundaries) {
    for (city::road::ProfilePoint& point : boundary.contour) {
      point.lateral_m *= 1.5;
      point.height_m *= 1.5;
    }
  }
  std::vector<double> wide{};
  ROAD_TEST_EXPECT(laterals_of(wider, wide), "the wider guttered road could not be drawn");
  ROAD_TEST_EXPECT(narrow.size() == wide.size(),
                   "resizing the gutter changed the section's shape");
  ROAD_TEST_EXPECT(std::abs(wide.front() - narrow.front()) < 1e-9 &&
                       std::abs(wide.back() - narrow.back()) < 1e-9,
                   "resizing the gutter moved the layout's outer ends");
  // Sample 6 is the centre line, the only thing between the two gutters.
  ROAD_TEST_EXPECT(std::abs(wide[6] - narrow[6]) < 1e-9,
                   "resizing the gutter moved the centre line");
  ROAD_TEST_EXPECT(std::abs(wide[1] - narrow[1]) > 1e-6,
                   "resizing the gutter did not move the gutter");
  return true;
}

bool saved_boundary_profiles_survive_reload(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0),
                   "the guttered road could not be drawn");
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto& restored = loaded.value.graph().layout_templates.front();
  const auto& original = state.graph().layout_templates.front();
  ROAD_TEST_EXPECT(restored.boundaries.size() == original.boundaries.size(),
                   "reloading lost a boundary");
  for (std::size_t index = 0; index < original.boundaries.size(); ++index) {
    const auto& a = original.boundaries[index];
    const auto& b = restored.boundaries[index];
    ROAD_TEST_EXPECT(a.contour.size() == b.contour.size() &&
                         a.segment_styles == b.segment_styles,
                     "reloading changed a boundary profile's shape");
    for (std::size_t point = 0; point < a.contour.size(); ++point) {
      ROAD_TEST_EXPECT(a.contour[point].lateral_m == b.contour[point].lateral_m &&
                           a.contour[point].height_m == b.contour[point].height_m,
                       "reloading moved a boundary profile point");
    }
  }
  ROAD_TEST_EXPECT(section_laterals(state) == section_laterals(loaded.value),
                   "reloading a guttered road produced different geometry");
  const auto resaved = loaded.value.Save();
  ROAD_TEST_EXPECT(resaved.ok && resaved.value == saved.value,
                   "a guttered road did not save back bit-identically");
  return true;
}

bool l_gutter_keeps_its_faces_through_a_junction(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1,
                   "guttered roads did not form one junction");
  for (const auto& gate : road_test_view::gates_of(*junctions.front())) {
    ROAD_TEST_EXPECT(gate.boundaries.size() == 13,
                     "a junction gate dropped part of the gutter profile");
  }
  for (const Mesh& mesh : state.derived().junction_meshes) {
    ROAD_TEST_EXPECT(mesh_faces_up(mesh), "the guttered junction mesh is inverted");
  }
  std::size_t road_facing_walls = 0;
  const auto& geometry = junctions.front()->junction_geometry;
  for (const auto& strip : geometry.surface_strips) {
    const auto& first =
        strip.winding == city::road::SurfaceWinding::kLeftToRight
            ? strip.left
            : strip.right;
    const auto& second =
        strip.winding == city::road::SurfaceWinding::kLeftToRight
            ? strip.right
            : strip.left;
    if (first.size() < 2 || first.size() != second.size()) continue;
    bool standing = true;
    bool separated_in_height = false;
    for (std::size_t point = 0; point < first.size(); ++point) {
      standing = standing &&
                 std::hypot(first[point].x - second[point].x,
                            first[point].y - second[point].y) <= 1e-6;
      separated_in_height =
          separated_in_height ||
          std::abs(first[point].z - second[point].z) > 1e-6;
    }
    if (!standing || !separated_in_height) continue;
    for (std::size_t point = 0; point + 1 < first.size(); ++point) {
      const Vec3d u{first[point + 1].x - first[point].x,
                    first[point + 1].y - first[point].y,
                    first[point + 1].z - first[point].z};
      const Vec3d v{second[point].x - first[point].x,
                    second[point].y - first[point].y,
                    second[point].z - first[point].z};
      const Vec2d normal{u.y * v.z - u.z * v.y,
                         u.z * v.x - u.x * v.z};
      const Vec2d midpoint{(first[point].x + second[point].x) * 0.5,
                           (first[point].y + second[point].y) * 0.5};
      ROAD_TEST_EXPECT(
          normal.x * -midpoint.x + normal.y * -midpoint.y > 0.0,
          "a gutter wall faces away from the junction roadway");
      ++road_facing_walls;
    }
  }
  ROAD_TEST_EXPECT(road_facing_walls > 0,
                   "the guttered junction exposed no road-facing wall");
  return true;
}

bool resolved_strip_winding_is_not_reinterpreted_as_world_up(
    std::string& failure) {
  city::road::ConnectionGeometry geometry{};
  geometry.surface_strips.push_back(city::road::ResolvedSurfaceStrip{
      RenderStyleFromSurface(builtin_surface_styles::kCurb), 1, 2,
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      {{0.0, -0.001, 1.0}, {1.0, -0.001, 1.0}},
  });
  const auto emitted = city::road::generation::emit_connection(geometry);
  ROAD_TEST_EXPECT(emitted.ok && emitted.value.size() == 1,
                   "the resolved vertical strip was not emitted");
  const Mesh& mesh = emitted.value.front();
  ROAD_TEST_EXPECT(mesh.indices.size() >= 3,
                   "the resolved vertical strip has no triangle");
  const Vec3d& a = mesh.vertices[mesh.indices[0]];
  const Vec3d& b = mesh.vertices[mesh.indices[1]];
  const Vec3d& c = mesh.vertices[mesh.indices[2]];
  const Vec3d ab{b.x - a.x, b.y - a.y, b.z - a.z};
  const Vec3d ac{c.x - a.x, c.y - a.y, c.z - a.z};
  const Vec3d normal{ab.y * ac.z - ab.z * ac.y,
                     ab.z * ac.x - ab.x * ac.z,
                     ab.x * ac.y - ab.y * ac.x};
  ROAD_TEST_EXPECT(
      normal.y < -0.9,
      "emit reversed a resolved wall because its small Z normal was negative");
  return true;
}

bool add_lane_beside_a_gutter_tapers_from_nothing(std::string& failure) {
  for (const city::road::RoadSide side :
       {city::road::RoadSide::kLeft, city::road::RoadSide::kRight}) {
    RoadState state{};
    const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), layout});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    const auto* corridor = city::road::FindCorridorForSegment(state.graph(), segment.value);
    ROAD_TEST_EXPECT(corridor != nullptr, "guttered fixture corridor is missing");

    const auto before = road_test_view::sections(state.derived());
    ROAD_TEST_EXPECT(!before.empty(), "guttered road has no sections");
    const auto divider_before = boundary_lateral(*before.front(), 200);
    const double left_end_before = before.front()->boundaries.front().lateral_m;
    ROAD_TEST_EXPECT(divider_before.has_value(), "guttered fixture divider is missing");

    city::road::AddLaneRequest request{};
    request.corridor_id = corridor->id;
    request.direction = city::road::LaneTravelDirection::kAlongSegment;
    request.side = side;
    ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                     "added lane range could not be resolved");
    request.lane_width_m = 3.0;
    const auto added = state.AddLane(request);
    ROAD_TEST_EXPECT(added.ok, added.error);

    const auto sections = road_test_view::sections(state.derived());
    ROAD_TEST_EXPECT(sections.size() >= 3, "the taper produced too few sections");
    double narrowest = std::numeric_limits<double>::infinity();
    for (const auto* section : sections) {
      double previous = -std::numeric_limits<double>::infinity();
      for (const auto& boundary : section->boundaries) {
        ROAD_TEST_EXPECT(boundary.lateral_m + 1e-12 >= previous,
                         "the section surface folded back on itself");
        previous = boundary.lateral_m;
      }
      const double width = section->boundaries.back().lateral_m -
                           section->boundaries.front().lateral_m;
      narrowest = std::min(narrowest, width);
    }
    ROAD_TEST_EXPECT(std::abs(narrowest - 10.0) < 1e-9,
                     "the new lane did not start from zero width");

    for (const auto* section : sections) {
      const auto divider = boundary_lateral(*section, 200);
      ROAD_TEST_EXPECT(divider.has_value(), "the centre line went missing");
      if (side == city::road::RoadSide::kRight) {
        ROAD_TEST_EXPECT(std::abs(*divider - *divider_before) < 1e-9,
                         "adding a lane on the right moved the centre line");
        ROAD_TEST_EXPECT(
            std::abs(section->boundaries.front().lateral_m - left_end_before) < 1e-9,
            "adding a lane on the right moved the left outer end");
      }
    }

    const auto& widest = *sections.back();
    const auto& tail = widest.boundaries;
    ROAD_TEST_EXPECT(tail.size() >= 5, "the widened section lost the gutter");
    const std::array<double, 4> gutter_steps{0.1, 0.0, 0.25, 0.1};
    for (std::size_t index = 0; index < gutter_steps.size(); ++index) {
      ROAD_TEST_EXPECT(
          std::abs((tail[index + 2].lateral_m - tail[index + 1].lateral_m) -
                   gutter_steps[index]) < 1e-9,
          "the gutter profile was reshaped to fit the lane beside it");
    }
  }
  return true;
}

bool gutter_corners_are_edges_and_gentle_ones_are_not(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0), "the guttered road could not be drawn");
  const auto sections = road_test_view::sections(state.derived());
  ROAD_TEST_EXPECT(!sections.empty(), "the guttered road has no sections");
  const auto& boundaries = sections.front()->boundaries;
  ROAD_TEST_EXPECT(boundaries.size() == 13, "the guttered section changed shape");

  // Samples 1..5 are the left gutter: top outer, datum top, datum bottom,
  // channel end, lip end.
  const std::array<bool, 13> expected{false, false, true,  true,  false,
                                      false, false, false, false, true,
                                      true,  false, false};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    ROAD_TEST_EXPECT(boundaries[index].hard_edge == expected[index],
                     "sample " + std::to_string(index) +
                         " was classified as the wrong kind of corner");
  }

  const auto curb_mesh = std::find_if(
      state.derived().segment_meshes.begin(), state.derived().segment_meshes.end(),
      [](const Mesh& mesh) {
        return mesh.style == RenderStyleFromSurface(builtin_surface_styles::kCurb);
      });
  ROAD_TEST_EXPECT(curb_mesh != state.derived().segment_meshes.end(),
                   "the gutter has no surface of its own");
  std::map<std::tuple<double, double, double>, std::set<std::uint32_t>> at_position{};
  for (const std::uint32_t index : curb_mesh->indices) {
    const Vec3d& vertex = curb_mesh->vertices[index];
    at_position[{vertex.x, vertex.y, vertex.z}].insert(index);
  }
  std::size_t split_positions = 0;
  for (const auto& [position, indices] : at_position) {
    (void)position;
    if (indices.size() > 1) ++split_positions;
  }
  ROAD_TEST_EXPECT(split_positions > 0,
                   "no vertex was split, so the gutter still shades as one surface");
  return true;
}

bool a_corner_carries_one_line_per_boundary(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {0.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto branch = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, node});
  ROAD_TEST_EXPECT(branch.ok, branch.error);

  std::map<std::uint64_t, std::size_t> lines_through_the_corner{};
  for (const auto& marking : state.derived().markings) {
    if (marking.owner.kind != city::road::MarkingOwner::Kind::kJunction)
      continue;
    ++lines_through_the_corner[marking.boundary_id];
  }
  ROAD_TEST_EXPECT(!lines_through_the_corner.empty(),
                   "no line was carried through the corner");
  for (const auto& [boundary_id, count] : lines_through_the_corner) {
    ROAD_TEST_EXPECT(count == 1,
                     "boundary " + std::to_string(boundary_id) + " drew " +
                         std::to_string(count) + " lines through one corner");
  }
  return true;
}

bool a_crossing_stays_on_the_roadway(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);

  // The roadway reaches 2.65 either side: the gutter's channel takes the rest
  // of the 3.0 lane and its top belongs to the walkway.
  std::size_t checked = 0;
  for (const auto& marking : state.derived().markings) {
    if (marking.role != city::road::MarkingRole::kCrosswalk &&
        marking.role != city::road::MarkingRole::kStopLine) {
      continue;
    }
    if (marking.polygon.empty()) continue;
    const bool east = std::all_of(
        marking.polygon.begin(), marking.polygon.end(),
        [](const Vec3d& point) { return point.x > 3.0; });
    if (!east) continue;
    ++checked;
    for (const Vec3d& point : marking.polygon) {
      ROAD_TEST_EXPECT(std::abs(point.y) <= 2.65 + 1e-6,
                       "a crossing was painted onto the gutter at y=" +
                           std::to_string(point.y));
    }
  }
  ROAD_TEST_EXPECT(checked > 0, "the east approach carried no crossing");
  return true;
}

bool a_junction_carries_the_edge_profile_it_was_given(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), layout});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), layout, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1, "the guttered roads did not form one junction");

  std::size_t standing = 0;
  for (const auto& strip : junctions.front()->junction_geometry.surface_strips) {
    ROAD_TEST_EXPECT(strip.left.size() == strip.right.size() && !strip.left.empty(),
                     "a junction strip is incomplete");
    bool flat_in_plan = true;
    bool apart_in_height = false;
    for (std::size_t i = 0; i < strip.left.size(); ++i) {
      if (std::hypot(strip.left[i].x - strip.right[i].x,
                     strip.left[i].y - strip.right[i].y) > 1e-6) {
        flat_in_plan = false;
      }
      if (std::abs(strip.left[i].z - strip.right[i].z) > 1e-6) {
        apart_in_height = true;
      }
    }
    if (flat_in_plan && apart_in_height) ++standing;
  }
  ROAD_TEST_EXPECT(standing > 0,
                   "the junction flattened the gutter into a slope");
  return true;
}

bool a_carriageway_edge_line_is_painted_on_the_road(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0),
                   "the guttered road could not be drawn");

  // The roadway runs to 2.65 either side; an edge line clears the gutter by its
  // own half width.
  std::size_t checked = 0;
  for (const auto& marking : state.derived().markings) {
    if (marking.role != city::road::MarkingRole::kCarriagewayEdge) continue;
    ROAD_TEST_EXPECT(!marking.points.empty(), "an edge line has no geometry");
    ++checked;
    const double half = marking.width_m * 0.5;
    for (const Vec3d& point : marking.points) {
      ROAD_TEST_EXPECT(std::abs(point.y) + half <= 2.65 + 1e-9,
                       "an edge line reaches onto the gutter: centre y=" +
                           std::to_string(point.y) + " half width " +
                           std::to_string(half));
    }
  }
  ROAD_TEST_EXPECT(checked == 2, "the road did not carry two edge lines");
  return true;
}

bool a_gutter_keeps_its_width_where_it_meets_another_layout(std::string& failure) {
  RoadState state{};
  const auto guttered = road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  city::road::RoadLayoutTemplate no_left = road_fixture::GutteredLayout(0);
  no_left.strips.erase(no_left.strips.begin());
  no_left.boundaries.erase(no_left.boundaries.begin());
  no_left.alignment_offset_from_left_m = road_fixture::CentredAlignmentOffset(no_left);
  const auto plain = road_fixture::AddLayout(state, no_left);
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), guttered});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), plain, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1, "the two layouts did not form one junction");

  bool found_collapsed_face = false;
  for (const auto& strip : junctions.front()->junction_geometry.surface_strips) {
    const double entered = std::hypot(strip.left.front().x - strip.right.front().x,
                                      strip.left.front().y - strip.right.front().y);
    const double left = std::hypot(strip.left.back().x - strip.right.back().x,
                                   strip.left.back().y - strip.right.back().y);
    if (std::min(entered, left) <= 1e-6 && std::max(entered, left) > 1e-6) {
      found_collapsed_face = true;
    }
    city::road::JunctionGeometry one{};
    one.surface_strips.push_back(strip);
    const auto emitted = city::road::generation::emit_junction(one);
    ROAD_TEST_EXPECT(emitted.ok,
                     "a mixed-layout junction face could not be emitted");
    for (const auto& mesh : emitted.value.surface_meshes) {
      ROAD_TEST_EXPECT(mesh_faces_up(mesh),
                       "a mixed-layout junction face is inverted");
    }
  }
  ROAD_TEST_EXPECT(found_collapsed_face,
                   "a missing side face was not collapsed at the carriageway edge");
  return true;
}

bool unlike_guttered_roads_join_road_outer_to_road_outer(std::string& failure) {
  RoadState state{};
  const auto guttered =
      road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
  city::road::RoadLayoutTemplate shouldered =
      road_fixture::ShoulderedLayout(0);
  const city::road::RoadLayoutTemplate gutter_profile =
      road_fixture::GutteredLayout(0);
  shouldered.boundaries.front() = gutter_profile.boundaries.front();
  shouldered.boundaries.front().marking = {};
  shouldered.boundaries.back() = gutter_profile.boundaries.back();
  shouldered.boundaries.back().marking = {};
  const auto guttered_shoulder =
      road_fixture::AddLayout(state, std::move(shouldered));

  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {40.0, 0.0})}), guttered});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}),
          guttered_shoulder, base.value, 40.0});
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto junctions = road_test_view::junctions(state.derived());
  ROAD_TEST_EXPECT(junctions.size() == 1,
                   "unlike guttered roads did not form one junction");

  const auto world_point = [](const city::road::ConnectionGate& gate,
                              const city::road::SectionBoundarySample& sample) {
    const double lateral = gate.approach.endpoint_role == EndpointRole::kEnd
                               ? -sample.lateral_m
                               : sample.lateral_m;
    return Vec3d{gate.position.x + gate.lateral.x * lateral,
                 gate.position.y + gate.lateral.y * lateral,
                 gate.position.z + sample.height_m};
  };
  const auto horizontal_distance = [](const Vec3d& a, const Vec3d& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  std::vector<Vec3d> expected_outer{};
  for (const auto& gate : road_test_view::gates_of(*junctions.front())) {
    ROAD_TEST_EXPECT(gate.boundaries.size() >= 2,
                     "a mixed-layout gate has no road outer boundaries");
    expected_outer.push_back(world_point(gate, gate.boundaries.front()));
    expected_outer.push_back(world_point(gate, gate.boundaries.back()));
  }
  std::vector<Vec3d> resolved_outer{};
  bool kept_gutter_top = false;
  for (const auto& strip :
       junctions.front()->junction_geometry.surface_strips) {
    if (strip.style == RenderStyleFromSurface(builtin_surface_styles::kCurb) &&
        !strip.left.empty() && !strip.right.empty() &&
        std::abs(horizontal_distance(strip.left.front(), strip.right.front()) -
                 0.1) <= 1e-6 &&
        std::abs(horizontal_distance(strip.left.back(), strip.right.back()) -
                 0.1) <= 1e-6 &&
        std::abs(strip.left.front().z - strip.right.front().z) <= 1e-6 &&
        std::abs(strip.left.back().z - strip.right.back().z) <= 1e-6) {
      kept_gutter_top = true;
    }
    if (strip.style !=
        RenderStyleFromSurface(builtin_surface_styles::kSidewalk)) {
      continue;
    }
    ROAD_TEST_EXPECT(!strip.left.empty() && !strip.right.empty(),
                     "a junction sidewalk strip has no boundary curves");
    ROAD_TEST_EXPECT(
        std::abs(horizontal_distance(strip.left.front(), strip.right.front()) -
                 1.9) <=
                1e-6 &&
            std::abs(horizontal_distance(strip.left.back(), strip.right.back()) -
                     1.9) <=
                1e-6,
        "joining different roads changed the gutter top or sidewalk width");
    resolved_outer.push_back(strip.right.front());
    resolved_outer.push_back(strip.right.back());
  }
  ROAD_TEST_EXPECT(resolved_outer.size() == expected_outer.size(),
                   "junction sidewalk outer edge count does not match its gates");
  for (const Vec3d& expected : expected_outer) {
    const auto found = std::find_if(
        resolved_outer.begin(), resolved_outer.end(),
        [&](const Vec3d& actual) {
          return horizontal_distance(expected, actual) <= 1e-6 &&
                 std::abs(expected.z - actual.z) <= 1e-6;
        });
    ROAD_TEST_EXPECT(
        found != resolved_outer.end(),
        "a junction connected a gutter point instead of the road outer edge at " +
            std::to_string(expected.x) + "," + std::to_string(expected.y));
  }
  ROAD_TEST_EXPECT(kept_gutter_top,
                   "joining different roads changed the 0.1m gutter top");
  return true;
}

bool unlike_side_profiles_form_a_degree_two_corner(std::string& failure) {
  for (const bool reversed : {false, true}) {
    RoadState state{};
    const auto guttered =
        road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
    const auto shouldered =
        road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-40.0, 0.0}, {0.0, 0.0})}),
        reversed ? shouldered : guttered});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadNodeId endpoint = state.graph().segments.front().node_b;
    const auto corner = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}),
            reversed ? guttered : shouldered, endpoint});
    ROAD_TEST_EXPECT(
        corner.ok,
        "degree-two semantic side mapping rejected unlike profiles: " +
            corner.error);
    const auto connections = road_test_view::corners(state.derived());
    ROAD_TEST_EXPECT(connections.size() == 1,
                     "unlike profiles did not form one degree-two connection");
    ROAD_TEST_EXPECT(
        !connections.front()->connection_geometry.surface_strips.empty(),
        "unlike profiles produced no connecting surfaces");
    const auto& curves = connections.front()->connection_geometry.boundary_curves;
    const auto has_pair = [&curves](std::uint64_t a, std::uint64_t b) {
      return std::any_of(curves.begin(), curves.end(), [a, b](const auto& curve) {
        return (curve.source_boundary_id == a && curve.target_boundary_id == b) ||
               (curve.source_boundary_id == b && curve.target_boundary_id == a);
      });
    };
    ROAD_TEST_EXPECT(has_pair(100, 150) && has_pair(300, 250),
                     "degree-two mapping did not join carriageway edges");
    ROAD_TEST_EXPECT(has_pair(100, 100) && has_pair(300, 300),
                     "degree-two mapping did not keep road outer edges fixed");
    for (const auto& strip :
         connections.front()->connection_geometry.surface_strips) {
      city::road::ConnectionGeometry one{};
      one.surface_strips.push_back(strip);
      const auto emitted = city::road::generation::emit_connection(one);
      ROAD_TEST_EXPECT(emitted.ok && emitted.value.size() == 1,
                       "a semantic side strip was not emitted");
      if (!mesh_faces_up(emitted.value.front())) {
        failure = "inverted degree-two strip " +
                  std::to_string(strip.left_boundary_id) + "/" +
                  std::to_string(strip.right_boundary_id) + " style " +
                  std::to_string(strip.style.value) + " winding " +
                  std::to_string(static_cast<int>(strip.winding));
        return false;
      }
    }
  }
  return true;
}

bool asymmetric_mixed_width_degree_two_corner_keeps_edges_inside_gates(
    std::string& failure) {
  RoadState state{};
  auto narrow = road_fixture::GutteredLayout(0);
  narrow.strips.erase(narrow.strips.begin());
  narrow.boundaries.erase(narrow.boundaries.begin());
  narrow.alignment_offset_from_left_m = road_fixture::CentredAlignmentOffset(narrow);
  const auto narrow_id = road_fixture::AddLayout(state, std::move(narrow));
  const auto wide_id =
      road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  ROAD_TEST_EXPECT(narrow_id != 0 && wide_id != 0,
                   "asymmetric mixed-width layouts were rejected");

  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-40.0, 0.0}, {0.0, 0.0})}), narrow_id});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const RoadNodeId endpoint = state.graph().segments.front().node_b;
  const auto corner = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}), wide_id, endpoint});
  ROAD_TEST_EXPECT(corner.ok, corner.error);
  const auto corners = road_test_view::corners(state.derived());
  ROAD_TEST_EXPECT(corners.size() == 1,
                   "asymmetric mixed-width roads did not form one corner");
  const auto& connection = *corners.front();
  const auto node_it = std::find_if(
      state.graph().nodes.begin(), state.graph().nodes.end(),
      [&connection](const city::road::RoadNode& node) {
        return node.id == connection.node_id;
      });
  ROAD_TEST_EXPECT(node_it != state.graph().nodes.end(),
                   "asymmetric mixed-width corner node is missing");

  for (const auto& approach : connection.approaches) {
    const double gate_distance =
        (approach.gate.position.x - node_it->position.x) *
            approach.tangent.x +
        (approach.gate.position.y - node_it->position.y) *
            approach.tangent.y;
    for (const auto& strip : connection.connection_geometry.surface_strips) {
      for (const auto points : {&strip.left, &strip.right}) {
        for (const Vec3d& point : *points) {
          const double node_to_point =
              (point.x - node_it->position.x) * approach.tangent.x +
              (point.y - node_it->position.y) * approach.tangent.y;
          ROAD_TEST_EXPECT(
              node_to_point <= gate_distance + 1e-6,
              "a degree-two mixed-width corner extends past an approach gate: "
                  "projection=" + std::to_string(node_to_point) +
                  " gate=" + std::to_string(gate_distance));
        }
      }
    }
  }
  return true;
}

bool corridor_extension_uses_degree_two_semantic_side_mapping(
    std::string& failure) {
  for (const bool reversed : {false, true}) {
    RoadState state{};
    const auto guttered =
        road_fixture::AddLayout(state, road_fixture::GutteredLayout(0));
    const auto shouldered =
        road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{
        MakePath({MakeLine({-40.0, 0.0}, {0.0, 0.0})}),
        reversed ? shouldered : guttered});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto* corridor = FindCorridorForSegment(state.graph(), base.value);
    const auto segment = std::find_if(
        state.graph().segments.begin(), state.graph().segments.end(),
        [id = base.value](const RoadSegment& item) { return item.id == id; });
    ROAD_TEST_EXPECT(corridor != nullptr && segment != state.graph().segments.end(),
                     "mixed-profile extension source is missing");
    const RoadCorridorId corridor_id = corridor->id;
    const RoadNodeId endpoint_id = segment->node_b;

    const auto extended = state.ExtendCorridorFromEnd(
        city::road::ExtendCorridorFromEndRequest{
            corridor_id, endpoint_id,
            MakePath({MakeLine({0.0, 0.0}, {0.0, 40.0})}),
            reversed ? guttered : shouldered});
    ROAD_TEST_EXPECT(
        extended.ok,
        "corridor extension bypassed the degree-two semantic resolver: " +
            extended.error);

    const auto connections = road_test_view::corners(state.derived());
    ROAD_TEST_EXPECT(connections.size() == 1,
                     "mixed-profile extension did not form one corner");
    const auto& curves =
        connections.front()->connection_geometry.boundary_curves;
    const auto has_pair = [&curves](std::uint64_t a, std::uint64_t b) {
      return std::any_of(curves.begin(), curves.end(), [a, b](const auto& curve) {
        return (curve.source_boundary_id == a && curve.target_boundary_id == b) ||
               (curve.source_boundary_id == b && curve.target_boundary_id == a);
      });
    };
    ROAD_TEST_EXPECT(has_pair(100, 150) && has_pair(300, 250),
                     "extension did not join carriageway edges semantically");
    ROAD_TEST_EXPECT(has_pair(100, 100) && has_pair(300, 300),
                     "extension did not preserve road outer edges");
  }
  return true;
}

bool road_does_not_enter_wire_core(std::string& failure) {
  const std::filesystem::path root = std::filesystem::current_path();
  const std::filesystem::path wire_domain = root / "domains" / "wire";
  if (!std::filesystem::exists(wire_domain)) {
    return true;
  }
  for (const auto& entry : std::filesystem::recursive_directory_iterator(wire_domain)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto ext = entry.path().extension().string();
    if (ext != ".hpp" && ext != ".cpp" && ext != ".txt") {
      continue;
    }
    std::ifstream in(entry.path());
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    ROAD_TEST_EXPECT(text.find("city/road") == std::string::npos,
                     "wire core references road include: " + entry.path().string());
  }
  return true;
}

struct Test {
  const char* name;
  bool (*run)(std::string&);
};

} // namespace

int main() {
  const Test tests[] = {
      {"P0_generates_two_lane_segment", P0_generates_two_lane_segment},
      {"P0_two_lane_mesh_shows_sidewalks_curbs_and_markings", P0_two_lane_mesh_shows_sidewalks_curbs_and_markings},
      {"mesh_uv_normals_and_material_groups_are_local",
       mesh_uv_normals_and_material_groups_are_local},
      {"junction_fan_uv_covers_basic_and_asymmetric_cases",
       junction_fan_uv_covers_basic_and_asymmetric_cases},
      {"gutter_profile_uv_uses_contour_v_and_patch_u",
       gutter_profile_uv_uses_contour_v_and_patch_u},
      {"road_elevation_profile_drives_derived_geometry",
       road_elevation_profile_drives_derived_geometry},
      {"P0_odd_lane_carriageway_mesh_has_drainage_crown",
       P0_odd_lane_carriageway_mesh_has_drainage_crown},
      {"P0_angled_segment_keeps_final_section_perpendicular", P0_angled_segment_keeps_final_section_perpendicular},
      {"P0_rejects_self_intersection_without_mutation", P0_rejects_self_intersection_without_mutation},
      {"P0_save_load_is_authoritative_and_bit_stable", P0_save_load_is_authoritative_and_bit_stable},
      {"lane_topology_validation_and_round_trip", lane_topology_validation_and_round_trip},
      {"P0_tool_preview_matches_the_committed_interval", P0_tool_preview_matches_the_committed_interval},
      {"P0_straight_segments_stay_linear_after_snap_and_move", P0_straight_segments_stay_linear_after_snap_and_move},
      {"P0_edit_and_delete_preserve_graph_ownership", P0_edit_and_delete_preserve_graph_ownership},
      {"P0_multispan_segment_is_one_user_deletion_unit",
       P0_multispan_segment_is_one_user_deletion_unit},
      {"P1_degree_two_corner_uses_a_curve_without_a_junction", P1_degree_two_corner_uses_a_curve_without_a_junction},
      {"P1_common_skew_angles_are_resolved_from_geometry",
       P1_common_skew_angles_are_resolved_from_geometry},
      {"P1_corner_preserves_endpoint_section_sides", P1_corner_preserves_endpoint_section_sides},
      {"P1_straight_connection_has_no_junction_area", P1_straight_connection_has_no_junction_area},
      {"P1_short_connections_use_required_setback",
       P1_short_connections_use_required_setback},
      {"P1_short_junctions_shrink_auto_gate_setbacks",
       P1_short_junctions_shrink_auto_gate_setbacks},
      {"P1_segment_snap_splits_straight_road_for_t_junction", P1_segment_snap_splits_straight_road_for_t_junction},
      {"P1_junction_setback_uses_the_facing_section_side",
       P1_junction_setback_uses_the_facing_section_side},
      {"P1_junction_setback_uses_only_adjacent_approaches",
       P1_junction_setback_uses_only_adjacent_approaches},
      {"P1_one_interval_connects_two_existing_roads_atomically",
       P1_one_interval_connects_two_existing_roads_atomically},
      {"P1_endpoint_to_segment_middle_connection_does_not_twist",
       P1_endpoint_to_segment_middle_connection_does_not_twist},
      {"P1_end_segment_snap_splits_straight_road_for_t_junction",
       P1_end_segment_snap_splits_straight_road_for_t_junction},
      {"P1_every_edge_section_forms_a_t_junction",
       P1_every_edge_section_forms_a_t_junction},
      {"P1_skew_shoulder_junction_connects_outer_lines",
       P1_skew_shoulder_junction_connects_outer_lines},
      {"P1_skew_shoulder_junction_mesh_does_not_overlap",
       P1_skew_shoulder_junction_mesh_does_not_overlap},
      {"P1_segment_snap_splits_bezier_road_for_t_junction", P1_segment_snap_splits_bezier_road_for_t_junction},
      {"P1_segment_snap_splits_at_internal_bezier_span_boundary",
       P1_segment_snap_splits_at_internal_bezier_span_boundary},
      {"P1_cross_junction_accepts_opposite_approaches", P1_cross_junction_accepts_opposite_approaches},
      {"P1_incremental_skew_cross_accepts_ordered_approaches",
       P1_incremental_skew_cross_accepts_ordered_approaches},
      {"P1_skew_shoulder_cross_has_connected_lines_without_overlap",
       P1_skew_shoulder_cross_has_connected_lines_without_overlap},
      {"section_transition_widens_one_side_from_its_anchor",
       section_transition_widens_one_side_from_its_anchor},
      {"lane_allocation_transition_uses_lane_identity",
       lane_allocation_transition_uses_lane_identity},
      {"saved_manual_markings_load_and_draw", saved_manual_markings_load_and_draw},
      {"add_lane_preserves_existing_lanes",
       add_lane_preserves_existing_lanes},
      {"add_lane_stores_one_segment_local_transition",
       add_lane_stores_one_segment_local_transition},
      {"add_lane_preserves_unrelated_corridor_geometry",
       add_lane_preserves_unrelated_corridor_geometry},
      {"add_lane_continues_to_the_operation_direction_terminal",
       add_lane_continues_to_the_operation_direction_terminal},
      {"add_lane_rejects_taper_across_segment_boundaries",
       add_lane_rejects_taper_across_segment_boundaries},
      {"add_lane_conflict_is_specific_and_atomic",
       add_lane_conflict_is_specific_and_atomic},
      {"add_lane_position_rejections_are_specific_and_atomic",
       add_lane_position_rejections_are_specific_and_atomic},
      {"transitioning_segment_split_respects_transition_bounds",
       transitioning_segment_split_respects_transition_bounds},
      {"segment_split_remaps_saved_lane_topology",
       segment_split_remaps_saved_lane_topology},
      {"split_public_operations_share_normalized_semantics",
       split_public_operations_share_normalized_semantics},
      {"split_crossing_owners_are_rejected_atomically",
       split_crossing_owners_are_rejected_atomically},
      {"split_semantics_have_one_internal_planner",
       split_semantics_have_one_internal_planner},
      {"connection_splits_reject_transitioning_sources",
       connection_splits_reject_transitioning_sources},
      {"multi_split_composes_cross_source_relations",
       multi_split_composes_cross_source_relations},
      {"multi_split_second_target_failure_is_atomic",
       multi_split_second_target_failure_is_atomic},
      {"add_lane_propagates_from_middle_corridor_segment",
       add_lane_propagates_from_middle_corridor_segment},
      {"add_lane_degree_two_terminal_preserves_existing_identity",
       add_lane_degree_two_terminal_preserves_existing_identity},
      {"add_lane_normalizes_reversed_corridor_direction",
       add_lane_normalizes_reversed_corridor_direction},
      {"add_lane_reaches_mixed_section_junction",
       add_lane_reaches_mixed_section_junction},
      {"add_lane_connects_the_only_matching_junction_approach",
       add_lane_connects_the_only_matching_junction_approach},
      {"add_lane_terminates_at_ambiguous_junction_destinations",
       add_lane_terminates_at_ambiguous_junction_destinations},
      {"add_lane_accepts_multiple_lanes_on_one_carriageway_strip",
       add_lane_accepts_multiple_lanes_on_one_carriageway_strip},
      {"add_lane_supports_every_section_outer_side",
       add_lane_supports_every_section_outer_side},
      {"saved_junction_movements_derive_lane_paths",
       saved_junction_movements_derive_lane_paths},
      {"P2_supports_taper_lane_reduction_and_median_end", P2_supports_taper_lane_reduction_and_median_end},
      {"P2_degree_two_accepts_different_lane_counts",
       P2_degree_two_accepts_different_lane_counts},
      {"equivalent_endpoint_sections_ignore_template_identity",
       equivalent_endpoint_sections_ignore_template_identity},
      {"P2_marking_policy_suppression_and_junction_override", P2_marking_policy_suppression_and_junction_override},
      {"M1_lane_side_requests_share_one_boundary_line", M1_lane_side_requests_share_one_boundary_line},
      {"M1_lane_side_without_adjacent_boundary_is_unsupported",
       M1_lane_side_without_adjacent_boundary_is_unsupported},
      {"M3_lane_count_change_begins_and_terminates_lines", M3_lane_count_change_begins_and_terminates_lines},
      {"M6_transition_without_boundary_mapping_is_unsupported",
       M6_transition_without_boundary_mapping_is_unsupported},
      {"RSL_section_axes_and_shoulder_are_independent",
       RSL_section_axes_and_shoulder_are_independent},
      {"CRV_drawn_curve_bends_one_way_per_interval", CRV_drawn_curve_bends_one_way_per_interval},
      {"CRV_control_handles_are_the_tangent_authority",
       CRV_control_handles_are_the_tangent_authority},
      {"CRV_explicit_extension_controls_are_not_rewritten",
       CRV_explicit_extension_controls_are_not_rewritten},
      {"layout_alignment_origin_measures_from_the_left_outer_end",
       layout_alignment_origin_measures_from_the_left_outer_end},
      {"layout_widened_on_one_side_keeps_the_other_side_still",
       layout_widened_on_one_side_keeps_the_other_side_still},
      {"add_lane_on_either_side_keeps_existing_lanes_still",
       add_lane_on_either_side_keeps_existing_lanes_still},
      {"add_lane_uses_user_pick_direction_on_a_corridor",
       add_lane_uses_user_pick_direction_on_a_corridor},
      {"layout_alignment_basis_is_continuous_across_a_transition",
       layout_alignment_basis_is_continuous_across_a_transition},
      {"off_centre_layout_keeps_its_alignment_through_a_junction",
       off_centre_layout_keeps_its_alignment_through_a_junction},
      {"saved_layout_alignment_offset_survives_reload",
       saved_layout_alignment_offset_survives_reload},
      {"version_fourteen_archive_defaults_the_corner_radius",
       version_fourteen_archive_defaults_the_corner_radius},
      {"junction_corner_radius_is_authoritative_and_inherited",
       junction_corner_radius_is_authoritative_and_inherited},
      {"corner_radius_extension_validation_and_mixed_resolution",
       corner_radius_extension_validation_and_mixed_resolution},
      {"junction_zero_radius_keeps_a_t_junction_local",
       junction_zero_radius_keeps_a_t_junction_local},
      {"l_gutter_comes_out_of_the_widths_beside_it",
       l_gutter_comes_out_of_the_widths_beside_it},
      {"l_gutter_dimensions_leave_the_road_where_it_was",
       l_gutter_dimensions_leave_the_road_where_it_was},
      {"saved_boundary_profiles_survive_reload",
       saved_boundary_profiles_survive_reload},
      {"l_gutter_keeps_its_faces_through_a_junction",
       l_gutter_keeps_its_faces_through_a_junction},
      {"resolved_strip_winding_is_not_reinterpreted_as_world_up",
       resolved_strip_winding_is_not_reinterpreted_as_world_up},
      {"add_lane_beside_a_gutter_tapers_from_nothing",
       add_lane_beside_a_gutter_tapers_from_nothing},
      {"gutter_corners_are_edges_and_gentle_ones_are_not",
       gutter_corners_are_edges_and_gentle_ones_are_not},
      {"a_corner_carries_one_line_per_boundary",
       a_corner_carries_one_line_per_boundary},
      {"a_crossing_stays_on_the_roadway",
       a_crossing_stays_on_the_roadway},
      {"a_junction_carries_the_edge_profile_it_was_given",
       a_junction_carries_the_edge_profile_it_was_given},
      {"a_carriageway_edge_line_is_painted_on_the_road",
       a_carriageway_edge_line_is_painted_on_the_road},
      {"a_gutter_keeps_its_width_where_it_meets_another_layout",
       a_gutter_keeps_its_width_where_it_meets_another_layout},
      {"unlike_guttered_roads_join_road_outer_to_road_outer",
       unlike_guttered_roads_join_road_outer_to_road_outer},
      {"unlike_side_profiles_form_a_degree_two_corner",
       unlike_side_profiles_form_a_degree_two_corner},
      {"asymmetric_mixed_width_degree_two_corner_keeps_edges_inside_gates",
       asymmetric_mixed_width_degree_two_corner_keeps_edges_inside_gates},
      {"corridor_extension_uses_degree_two_semantic_side_mapping",
       corridor_extension_uses_degree_two_semantic_side_mapping},
      {"road_does_not_enter_wire_core", road_does_not_enter_wire_core},
  };
  int failed = 0;
  for (const Test& test : tests) {
    std::string failure;
    const bool ok = test.run(failure);
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << test.name << std::endl;
    if (!ok) {
      std::cerr << "  reason: " << failure << "\n";
      ++failed;
    }
  }
  if (failed != 0) {
    return 1;
  }
  std::cout << "road tests passed (" << std::size(tests) << " cases)\n";
  return 0;
}
