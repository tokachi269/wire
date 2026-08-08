#include "city/road/road.hpp"

#include "derived_view.hpp"
#include "fixtures/layouts.hpp"
#include "../src/generation/generation.hpp"
#include "../src/generation/emit.hpp"
#include "../src/geometry/geometry.hpp"
#include "../src/persistence/archive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
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
using city::road::EvaluatePath;
using city::road::Path;
using city::road::PathLength;
using city::road::RenderStyleFromMarking;
using city::road::RenderStyleFromSurface;
using city::road::RenderStyleRef;
using city::road::RoadState;
using city::road::RoadCorridorId;
using city::road::RoadNodeId;
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
  const auto& terminal_ref = corridor->segments.back();
  const auto terminal = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&terminal_ref](const RoadSegment& item) {
        return item.id == terminal_ref.segment_id;
      });
  if (terminal == state.graph().segments.end()) return false;
  request.continuation_end = city::road::SegmentPosition{
      terminal_ref.segment_id, terminal_ref.reversed ? 0.0 : 1.0};
  return true;
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

bool junction_mesh_is_non_overlapping(const RoadState& state,
                                      const std::string& label,
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
  ROAD_TEST_EXPECT(saved.value.starts_with("road_graph_version=14\n") &&
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
  version4.replace(0, std::string("road_graph_version=14").size(), "road_graph_version=4");
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
  ROAD_TEST_EXPECT(!RoadState::Load(archive_with(saved.value, "road_graph_version", "15")).ok,
                   "future road archive version was accepted");
  ROAD_TEST_EXPECT(failure.empty(), failure);
  for (int old_version = 1; old_version <= 12; ++old_version) {
    std::string legacy = saved.value;
    legacy.replace(0, std::string("road_graph_version=14").size(),
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
  ROAD_TEST_EXPECT(std::abs(corner.corner_control_m - corner.junction_corner_control_m) < 1e-9,
                   "corner and junction curve derivation use different control factors");
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
  ROAD_TEST_EXPECT(std::abs(junction.corner_control_m - junction.junction_corner_control_m) < 1e-9,
                   "corner and junction curve derivation use different control factors");
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
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {100.0, 0.0})}), shouldered});
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
  request.continuation_end = {segment.value, 1.0};
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == segment_count,
                   "LANE0 Add Lane split the selected RoadSegment");
  const auto updated = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [id = segment.value](const RoadSegment& item) { return item.id == id; });
  ROAD_TEST_EXPECT(updated != state.graph().segments.end() &&
                       updated->transition.has_value(),
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
      {affected.value, 0.7}, {affected.value, 1.0}, 3.0};
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

bool add_lane_stops_at_the_explicit_corridor_endpoint(std::string& failure) {
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
  const RoadNodeId continuation_end_node = second_segment->node_b;
  const auto third = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, continuation_end_node,
          MakePath({MakeLine({120.0, 0.0}, {180.0, 0.0})}), section});
  ROAD_TEST_EXPECT(third.ok, third.error);
  const auto branch = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({120.0, 0.0}, {120.0, 60.0})}), section,
          continuation_end_node});
  ROAD_TEST_EXPECT(branch.ok, branch.error);

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor_id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.transition_start = {first.value, 0.25};
  request.transition_complete = {first.value, 0.75};
  request.continuation_end = {second.value, 1.0};
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
                       updated_third->layout_template == 1,
                   "ADD LANE propagated beyond its explicit continuation endpoint");
  return true;
}

bool add_lane_taper_crosses_segment_boundaries(std::string& failure) {
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
  const RoadNodeId original_end_node = second_segment->node_b;

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor_id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.transition_start = {first.value, 0.5};
  request.transition_complete = {second.value, 0.5};
  request.continuation_end = {second.value, 0.75};
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok,
                   "ADD LANE rejected a taper crossing one segment boundary: " +
                       added.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3,
                   "an interior ADD LANE end did not split its terminal segment");
  const auto suffix = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [second_id = second.value, section,
       original_end_node](const RoadSegment& item) {
        return item.id != second_id && item.layout_template == section &&
               item.node_b == original_end_node;
      });
  ROAD_TEST_EXPECT(suffix != state.graph().segments.end(),
                   "the road after an interior ADD LANE end did not keep its original section");
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
      {segment.value, 0.6}, {segment.value, 1.0}, 3.0};
  const auto first = state.AddLane(request);
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto before = state.Save();
  ROAD_TEST_EXPECT(before.ok, before.error);
  const auto rejected = state.AddLane(request);
  ROAD_TEST_EXPECT(!rejected.ok &&
                       rejected.failure_category ==
                           CommitFailureCategory::kInvalidInput &&
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
        {segment.value, 0.6}, {segment.value, 1.0}, 3.0};
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

bool add_lane_taper_crosses_segment_boundary(std::string& failure) {
  RoadState state{};
  const auto shouldered = road_fixture::AddLayout(state, road_fixture::ShoulderedLayout(0));
  const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const auto first_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&first](const auto& segment) { return segment.id == first.value; });
  const auto* corridor = FindCorridorForSegment(state.graph(), first.value);
  ROAD_TEST_EXPECT(first_segment != state.graph().segments.end() &&
                       corridor != nullptr,
                   "ADD2 initial corridor is missing");
  const auto corridor_id = corridor->id;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor_id, first_segment->node_b,
          MakePath({MakeLine({80.0, 0.0}, {160.0, 0.0})}), shouldered});
  ROAD_TEST_EXPECT(second.ok, second.error);

  city::road::AddLaneRequest request{};
  request.corridor_id = corridor_id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 60.0, 100.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto first_after = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&first](const auto& segment) { return segment.id == first.value; });
  const auto second_after = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&second](const auto& segment) { return segment.id == second.value; });
  const auto* updated_corridor =
      FindCorridorForSegment(state.graph(), first.value);
  const std::size_t transitioning_segments =
      updated_corridor == nullptr
          ? 0
          : static_cast<std::size_t>(std::count_if(
                updated_corridor->segments.begin(),
                updated_corridor->segments.end(), [&state](const auto& ref) {
                  const auto segment = std::find_if(
                      state.graph().segments.begin(),
                      state.graph().segments.end(), [&ref](const auto& item) {
                        return item.id == ref.segment_id;
                      });
                  return segment != state.graph().segments.end() &&
                         segment->transition.has_value();
                }));
  ROAD_TEST_EXPECT(!first_after->transition.has_value() &&
                       second_after->transition.has_value() &&
                       transitioning_segments == 2,
                   "ADD2 did not resolve both sides of the segment boundary");
  const auto sections = road_test_view::sections(state.derived());
  const auto at_boundary = std::find_if(
      sections.begin(), sections.end(), [&second](const auto* section) {
        return section->segment_id == second.value &&
               std::abs(section->segment_distance_m) < 1e-6;
      });
  ROAD_TEST_EXPECT(at_boundary != sections.end(),
                   "ADD2 boundary section sample is missing");
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
  request.continuation_end = {segment.value, 0.25};
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
  ROAD_TEST_EXPECT(loaded.value.graph().segments.size() == 2,
                   "ADD3 reversed interior end did not split its segment");
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
  ROAD_TEST_EXPECT(added_connection != state.graph().lane_connections.end(),
                   "ADD9 added lane was not connected to the only matching approach");
  ROAD_TEST_EXPECT(state.graph().lane_connections.size() == 3,
                   "ADD9 did not resolve every lane in the unique 3-to-3 movement");
  ROAD_TEST_EXPECT(state.graph().boundary_continuations.size() == 4,
                   "ADD9 did not resolve every boundary in the unique 3-to-3 movement");
  ROAD_TEST_EXPECT(state.derived().lane_paths.size() == 3 &&
                       state.derived().boundary_paths.size() == 4,
                   "ADD9 unique junction topology was not derived");
  return true;
}

bool add_lane_rejects_ambiguous_junction_destinations(
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
  const auto before = state.Save();
  const auto* corridor = FindCorridorForSegment(state.graph(), incoming.value);
  ROAD_TEST_EXPECT(before.ok && corridor != nullptr,
                   "ADD10 preflight state is missing");
  city::road::AddLaneRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  ROAD_TEST_EXPECT(SetAddLaneRange(state, request, 20.0, 60.0),
                   "ADD LANE test range could not be resolved");
  request.lane_width_m = 3.0;
  const auto added = state.AddLane(request);
  ROAD_TEST_EXPECT(!added.ok &&
                       added.failure_category ==
                           city::road::CommitFailureCategory::kNotImplemented &&
                       added.error.find("destination is ambiguous") !=
                           std::string::npos,
                   "ADD10 ambiguous junction destination was not reported concretely");
  const auto after = state.Save();
  ROAD_TEST_EXPECT(after.ok && after.value == before.value,
                   "ADD10 ambiguous junction failure mutated authoritative state");
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

bool add_lane_allows_a_later_addition_after_cross_segment_taper(
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
                   "ADD6 initial corridor is missing");
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
  ROAD_TEST_EXPECT(SetAddLaneRange(state, second_add, 130.0, 160.0),
                   "ADD LANE test range could not be resolved");
  const auto second_lane = state.AddLane(second_add);
  ROAD_TEST_EXPECT(
      second_lane.ok,
      "ADD6 lane after cross-segment taper was rejected: " +
          second_lane.error);
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

bool P2_requires_transition_for_mixed_section_connection(std::string& failure) {
  {
    RoadState state{};
    const auto section = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
    const auto three_lane = road_fixture::AddLayout(state, road_fixture::ExtraLaneLayout(0));
    const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), section});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto endpoint = state.graph().segments.front().node_b;
    const auto direct = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), three_lane, endpoint});
    ROAD_TEST_EXPECT(!direct.ok && direct.failure_category == CommitFailureCategory::kNotImplemented,
                     "P2 accepted a mixed-section node connection without a transition");
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

bool version_thirteen_archive_centres_the_lines_it_saved(std::string& failure) {
  RoadState state{};
  const auto layout = road_fixture::AddLayout(state, road_fixture::BidirectionalLayout(0));
  ROAD_TEST_EXPECT(draw_straight_road(state, layout, 60.0), "road could not be drawn");
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);

  // Version 13 said nothing about placement and centred every line.
  std::string legacy{};
  std::istringstream lines{saved.value};
  std::string line{};
  std::size_t dropped = 0;
  while (std::getline(lines, line)) {
    if (line.find(".placement=") != std::string::npos) {
      ++dropped;
      continue;
    }
    if (line == "road_graph_version=14") line = "road_graph_version=13";
    legacy += line;
    legacy += '\n';
  }
  ROAD_TEST_EXPECT(dropped > 0, "the archive under test states no placement");

  const auto loaded = RoadState::Load(legacy);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  for (const auto& boundary : loaded.value.graph().layout_templates.front().boundaries) {
    if (!boundary.marking.enabled) continue;
    ROAD_TEST_EXPECT(boundary.marking.placement == MarkingPlacement::kCenter,
                     "a version 13 line came back with a placement it never had");
  }
  for (const auto* section : road_test_view::sections(loaded.value.derived())) {
    for (const auto& boundary : section->boundaries) {
      ROAD_TEST_EXPECT(boundary.marking_lateral_m == boundary.lateral_m,
                       "a version 13 line moved off its boundary");
    }
  }
  // The migration states a placement the archive never had, so it cannot save
  // back as the bytes it came from.
  const auto resaved = loaded.value.Save();
  ROAD_TEST_EXPECT(resaved.ok, resaved.error);
  ROAD_TEST_EXPECT(resaved.value.starts_with("road_graph_version=14\n"),
                   "a migrated road did not save as the current version");
  const auto reloaded = RoadState::Load(resaved.value);
  ROAD_TEST_EXPECT(reloaded.ok, reloaded.error);
  const auto settled = reloaded.value.Save();
  ROAD_TEST_EXPECT(settled.ok && settled.value == resaved.value,
                   "a migrated road did not settle after one save");
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
      {"P1_segment_snap_splits_straight_road_for_t_junction", P1_segment_snap_splits_straight_road_for_t_junction},
      {"P1_one_interval_connects_two_existing_roads_atomically",
       P1_one_interval_connects_two_existing_roads_atomically},
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
      {"saved_manual_markings_load_and_draw", saved_manual_markings_load_and_draw},
      {"add_lane_preserves_existing_lanes",
       add_lane_preserves_existing_lanes},
      {"add_lane_stores_one_segment_local_transition",
       add_lane_stores_one_segment_local_transition},
      {"add_lane_preserves_unrelated_corridor_geometry",
       add_lane_preserves_unrelated_corridor_geometry},
      {"add_lane_stops_at_the_explicit_corridor_endpoint",
       add_lane_stops_at_the_explicit_corridor_endpoint},
      {"add_lane_taper_crosses_segment_boundaries",
       add_lane_taper_crosses_segment_boundaries},
      {"add_lane_conflict_is_specific_and_atomic",
       add_lane_conflict_is_specific_and_atomic},
      {"transitioning_segment_split_respects_transition_bounds",
       transitioning_segment_split_respects_transition_bounds},
      {"add_lane_propagates_from_middle_corridor_segment",
       add_lane_propagates_from_middle_corridor_segment},
      {"add_lane_normalizes_reversed_corridor_direction",
       add_lane_normalizes_reversed_corridor_direction},
      {"add_lane_reaches_mixed_section_junction",
       add_lane_reaches_mixed_section_junction},
      {"add_lane_connects_the_only_matching_junction_approach",
       add_lane_connects_the_only_matching_junction_approach},
      {"add_lane_rejects_ambiguous_junction_destinations",
       add_lane_rejects_ambiguous_junction_destinations},
      {"add_lane_accepts_multiple_lanes_on_one_carriageway_strip",
       add_lane_accepts_multiple_lanes_on_one_carriageway_strip},
      {"add_lane_supports_every_section_outer_side",
       add_lane_supports_every_section_outer_side},
      {"saved_junction_movements_derive_lane_paths",
       saved_junction_movements_derive_lane_paths},
      {"P2_supports_taper_lane_reduction_and_median_end", P2_supports_taper_lane_reduction_and_median_end},
      {"P2_requires_transition_for_mixed_section_connection", P2_requires_transition_for_mixed_section_connection},
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
      {"layout_alignment_basis_is_continuous_across_a_transition",
       layout_alignment_basis_is_continuous_across_a_transition},
      {"off_centre_layout_keeps_its_alignment_through_a_junction",
       off_centre_layout_keeps_its_alignment_through_a_junction},
      {"saved_layout_alignment_offset_survives_reload",
       saved_layout_alignment_offset_survives_reload},
      {"version_thirteen_archive_centres_the_lines_it_saved",
       version_thirteen_archive_centres_the_lines_it_saved},
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
