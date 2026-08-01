#include "city/road/road.hpp"

#include "derived_view.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
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

using city::road::ErrorKind;
using city::road::BoundaryRole;
using city::road::AutoMarkingKey;
using city::road::AutoMarkingPolicy;
using city::road::EndpointRole;
using city::road::JapaneseUrbanTwoLaneTemplate;
using city::road::ShoulderedTwoLaneTemplate;
using city::road::JunctionMarkingAction;
using city::road::JunctionMarkingEndpoint;
using city::road::JunctionMarkingOverride;
using city::road::MarkingRole;
using city::road::MarkingOwner;
using city::road::MarkingTrackKey;
using city::road::MakeBezier;
using city::road::MakeLine;
using city::road::MakePath;
using city::road::ManualAreaRequest;
using city::road::ManualLineRequest;
using city::road::Mesh;
using city::road::EvaluatePath;
using city::road::Path;
using city::road::PathLength;
using city::road::PreviewRoadToolPath;
using city::road::RenderStyleFromMarking;
using city::road::RenderStyleFromSurface;
using city::road::RenderStyleRef;
using city::road::RoadState;
using city::road::RoadCorridorId;
using city::road::RoadNodeId;
using city::road::RoadSegment;
using city::road::RoadSegmentId;
using city::road::SectionTransitionRequest;
using city::road::SectionTransitionRule;
using city::road::DistanceRef;
using city::road::DistanceRefKind;
using city::road::StripFunction;
using city::road::ThreeLaneTemplate;
using city::road::TransitionAnchor;
using city::road::TransitionAction;
using city::road::ValidateGraphInvariants;
using city::road::Vec2d;
namespace builtin_marking_styles = city::road::builtin_marking_styles;
namespace builtin_surface_styles = city::road::builtin_surface_styles;

bool mesh_faces_up(const Mesh& mesh) {
  for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    const auto& a = mesh.vertices[mesh.indices[i]];
    const auto& b = mesh.vertices[mesh.indices[i + 1]];
    const auto& c = mesh.vertices[mesh.indices[i + 2]];
    const double ux = b.x - a.x;
    const double uy = b.y - a.y;
    const double vx = c.x - a.x;
    const double vy = c.y - a.y;
    if (ux * vy - uy * vx < -1e-9) {
      return false;
    }
  }
  return true;
}

city::road::CrossSectionTemplate OneWayLaneTemplate(
    city::road::CrossSectionTemplateId id, std::size_t lane_count,
    city::road::LaneTravelDirection direction =
        city::road::LaneTravelDirection::kAlongSegment) {
  const AutoMarkingPolicy edge{
      true, MarkingRole::kCarriagewayEdge,
      builtin_marking_styles::kWhiteSolid};
  const AutoMarkingPolicy divider{
      true, MarkingRole::kLaneSeparator,
      builtin_marking_styles::kWhiteDashed};
  city::road::CrossSectionTemplate section{};
  section.id = id;
  section.strips.push_back({10, StripFunction::kShoulder, 0.75, 0.0,
                            builtin_surface_styles::kAsphalt});
  for (std::size_t index = 0; index < lane_count; ++index) {
    const auto strip_id = static_cast<city::road::SectionStripId>(20 + index * 10);
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
      {100, BoundaryRole::kOuterEdge, 0.0, 0.0, edge});
  for (std::size_t index = 1; index < lane_count; ++index) {
    section.boundaries.push_back(
        {static_cast<city::road::BoundaryId>(100 + index * 100),
         BoundaryRole::kLaneDivider, 0.0, 0.0, divider});
  }
  section.boundaries.push_back(
      {900, BoundaryRole::kOuterEdge, 0.0, 0.0, edge});
  return section;
}

city::road::CrossSectionTemplate OpposingFourLaneTemplate(
    city::road::CrossSectionTemplateId id) {
  const AutoMarkingPolicy edge{
      true, MarkingRole::kCarriagewayEdge,
      builtin_marking_styles::kWhiteSolid};
  const AutoMarkingPolicy divider{
      true, MarkingRole::kLaneSeparator,
      builtin_marking_styles::kWhiteDashed};
  city::road::CrossSectionTemplate section{};
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
      {100, BoundaryRole::kOuterEdge, 0.0, 0.0, edge},
      {150, BoundaryRole::kLaneDivider, 0.0, 0.0, divider},
      {200, BoundaryRole::kMedianEdge, 0.0, 0.0, {}},
      {210, BoundaryRole::kMedianEdge, 0.0, 0.0, {}},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, divider},
      {300, BoundaryRole::kOuterEdge, 0.0, 0.0, edge},
  };
  return section;
}

bool P0_generates_two_lane_segment(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1, "P0 did not save one authoritative segment");
  ROAD_TEST_EXPECT(road_test_view::sections(state.derived()).size() > 2, "P0 did not derive section samples");
  ROAD_TEST_EXPECT(!state.derived().segment_meshes.empty(), "P0 did not derive a surface mesh");
  ROAD_TEST_EXPECT(!state.derived().terrain_masks.empty(), "P0 did not derive a terrain mask");
  const auto section = JapaneseUrbanTwoLaneTemplate(1);
  ROAD_TEST_EXPECT(section.strips.size() == 4, "Japanese two-lane template should have sidewalk/car/car/sidewalk");
  ROAD_TEST_EXPECT(std::abs(section.strips[1].width_m - 3.0) < 1e-9, "lane width is not 3.0m");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok, "P0 invariants failed");
  return true;
}

bool P0_two_lane_mesh_shows_sidewalks_curbs_and_markings(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(!road_test_view::sections(state.derived()).empty(), "P0 did not evaluate the section");
  const auto& boundaries = road_test_view::sections(state.derived()).front()->boundaries;
  ROAD_TEST_EXPECT(boundaries.size() >= 7, "P0 section does not expose sidewalk/curb/carriageway edges");
  ROAD_TEST_EXPECT(std::abs(boundaries.front().height_m - 0.13) < 1e-9,
                   "left sidewalk outer edge does not apply the 1% outward slope");
  ROAD_TEST_EXPECT(std::abs(boundaries[1].height_m - 0.15) < 1e-9,
                   "left curb top is not 0.15m above the carriageway edge");
  ROAD_TEST_EXPECT(std::abs(boundaries.back().height_m - 0.13) < 1e-9,
                   "right sidewalk outer edge does not apply the 1% outward slope");
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
  const Vec2d start{0.0, 0.0};
  const Vec2d end{24.0, 12.0};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine(start, end)}), 1});
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
  const auto first = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const std::size_t before_segments = state.graph().segments.size();
  const auto bad = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {10.0, 10.0}), MakeLine({10.0, 10.0}, {0.0, 10.0}),
                MakeLine({0.0, 10.0}, {10.0, 0.0})}),
      1});
  ROAD_TEST_EXPECT(!bad.ok && bad.error_kind == ErrorKind::kUnsupported, "self-intersection was not unsupported");
  ROAD_TEST_EXPECT(state.graph().segments.size() == before_segments, "failed preflight mutated authoritative graph");
  return true;
}

bool P0_save_load_is_authoritative_and_bit_stable(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeBezier({0.0, 0.0}, {10.0, 12.0}, {30.0, -12.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  ROAD_TEST_EXPECT(saved.value.starts_with("road_graph_version=11\n") &&
                       saved.value.find("primitive=") == std::string::npos &&
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
  version4.replace(0, std::string("road_graph_version=11").size(), "road_graph_version=4");
  const auto rejected = RoadState::Load(version4);
  ROAD_TEST_EXPECT(!rejected.ok && rejected.error_kind == ErrorKind::kValidation,
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
  ROAD_TEST_EXPECT(!RoadState::Load(archive_with(saved.value, "road_graph_version", "12")).ok,
                   "future road archive version was accepted");
  ROAD_TEST_EXPECT(failure.empty(), failure);
  for (int old_version = 1; old_version <= 10; ++old_version) {
    std::string legacy = saved.value;
    legacy.replace(0, std::string("road_graph_version=11").size(),
                   "road_graph_version=" + std::to_string(old_version));
    ROAD_TEST_EXPECT(!RoadState::Load(legacy).ok, "legacy road archive version was accepted");
  }
  return true;
}

bool lane_topology_validation_and_round_trip(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const RoadCorridorId corridor = state.graph().corridors.front().id;
  const RoadNodeId endpoint = state.graph().segments.front().node_b;
  const auto second = state.ExtendCorridorFromEnd(
      city::road::ExtendCorridorFromEndRequest{
          corridor, endpoint,
          MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(second.ok, second.error);

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  std::uint64_t maximum_id = 0;
  for (const auto& node : state.graph().nodes) maximum_id = std::max(maximum_id, node.id);
  for (const auto& segment : state.graph().segments) maximum_id = std::max(maximum_id, segment.id);
  for (const auto& road_corridor : state.graph().corridors) {
    maximum_id = std::max(maximum_id, road_corridor.id);
  }
  for (const auto& section : state.graph().section_templates) {
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

bool P0_tool_preview_includes_bezier_handles(std::string& failure) {
  const auto draft = PreviewRoadToolPath({0.0, 0.0}, {40.0, 0.0}, Vec2d{10.0, 15.0}, Vec2d{30.0, -15.0});
  ROAD_TEST_EXPECT(draft.has_live_preview, "road tool lacks live preview");
  ROAD_TEST_EXPECT(draft.supports_bezier_handles, "road tool does not expose Bezier handles");
  ROAD_TEST_EXPECT(draft.preview_path.spans.size() == 1, "road tool did not produce a preview span");
  const auto line = MakeLine({0.0, 0.0}, {30.0, 0.0});
  ROAD_TEST_EXPECT(std::abs(line.p1.x - 10.0) < 1e-9 && std::abs(line.p2.x - 20.0) < 1e-9 &&
                       std::abs(line.p3.x - 30.0) < 1e-9,
                   "straight input was not normalized to a cubic Bezier span");
  return true;
}

bool P0_straight_segments_stay_linear_after_snap_and_move(std::string& failure) {
  {
    RoadState state{};
    const auto base = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const RoadSegmentId source = base.value;
    const RoadNodeId endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(
        city::road::AddSegmentConnectedToRequest{
            MakePath({MakeLine({20.2, 0.0}, {20.0, 20.0})}), 1, endpoint});
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
    const auto first = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
    ROAD_TEST_EXPECT(first.ok, first.error);
    const RoadSegment& segment = state.graph().segments.front();
    const RoadNodeId endpoint = segment.node_b;
    const RoadCorridorId corridor = state.graph().corridors.front().id;
    const auto extended = state.ExtendCorridorFromEnd(
        city::road::ExtendCorridorFromEndRequest{
            corridor, endpoint,
            MakePath({MakeLine({20.2, 0.0}, {20.0, 20.0})}), 1});
    ROAD_TEST_EXPECT(extended.ok, extended.error);
    const Path* extension_alignment = FindCanonicalAlignment(state.derived(), extended.value);
    ROAD_TEST_EXPECT(extension_alignment != nullptr,
                     "extended straight alignment is missing");
    ROAD_TEST_EXPECT(IsLinearSpan(extension_alignment->spans.front()),
                     "extended straight segment is no longer a linear Bezier");
  }

  {
    RoadState state{};
    const auto added = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {30.0, 0.0})}), 1});
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
  const auto isolated = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(isolated.ok, isolated.error);
  const auto moved = state.MoveNode(city::road::MoveNodeRequest{state.graph().segments.front().node_a, {2.0, 3.0}});
  ROAD_TEST_EXPECT(moved.ok, moved.error);
  ROAD_TEST_EXPECT(std::abs(state.graph().nodes[0].position.x - 2.0) < 1e-9,
                   "MoveNode did not move endpoint authority");

  const auto shared_node = state.graph().segments.front().node_a;
  const auto branch = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({2.0, 3.0}, {2.0, 23.0})}), 1, shared_node});
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
  const Path input = MakePath({
      MakeBezier({0.0, 0.0}, {5.0, 0.0}, {10.0, 5.0}, {15.0, 5.0}),
      MakeBezier({15.0, 5.0}, {20.0, 5.0}, {25.0, 0.0}, {30.0, 0.0}),
  });
  const auto added =
      state.AddSegment(city::road::AddSegmentRequest{input, 1});
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
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_a;
  const auto branch = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({0.0, 0.0}, {0.0, 30.0})}), 1, node});
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
  const auto too_sharp = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({0.0, 0.0}, {30.0, 1.0})}), 1, node});
  ROAD_TEST_EXPECT(!too_sharp.ok && too_sharp.error_kind == ErrorKind::kUnsupported,
                   "P1 accepted a connection angle outside the fixed range");
  return true;
}

bool P1_corner_preserves_endpoint_section_sides(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto branch = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({40.0, 0.0}, {40.0, 30.0})}), 1, node});
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
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto continued = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), 1, node});
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

bool P1_segment_snap_splits_straight_road_for_t_junction(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{MakePath({MakeLine({20.0, 0.0}, {32.0, 24.0})}), 1,
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
  ROAD_TEST_EXPECT(max_z->z - min_z->z > 0.01, "T junction zebra does not follow the road cross slope");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok, "T junction invariants failed");
  return true;
}

bool P1_segment_snap_splits_bezier_road_for_t_junction(std::string& failure) {
  RoadState state{};
  const Path curve = MakePath({MakeBezier({0.0, 0.0}, {20.0, 10.0}, {60.0, 10.0}, {80.0, 0.0})});
  const auto base = state.AddSegment(city::road::AddSegmentRequest{curve, 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto curve_length = PathLength(curve);
  ROAD_TEST_EXPECT(curve_length.ok, curve_length.error);
  const auto split_point = EvaluatePath(curve, curve_length.value * 0.5);
  ROAD_TEST_EXPECT(split_point.ok, split_point.error);
  const auto branch = state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{
      MakePath({MakeLine(split_point.value, {split_point.value.x, split_point.value.y + 30.0})}), 1, base.value,
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
  const auto first_span = MakeBezier({0.0, 0.0}, {10.0, 10.0}, {30.0, 10.0}, {40.0, 0.0});
  const auto second_span = MakeBezier({40.0, 0.0}, {50.0, -10.0}, {70.0, -10.0}, {80.0, 0.0});
  const Path alignment = MakePath({first_span, second_span});
  const auto base = state.AddSegment(city::road::AddSegmentRequest{alignment, 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1,
                   "one multi-span draw was split before an explicit edit");
  const auto first_length = PathLength(MakePath({first_span}));
  ROAD_TEST_EXPECT(first_length.ok, first_length.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({40.0, 0.0}, {40.0, 30.0})}), 1,
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
  const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), 1,
                                                        base.value, 20.0});
  ROAD_TEST_EXPECT(north.ok, north.error);
  ROAD_TEST_EXPECT(!road_test_view::junctions(state.derived()).empty(), "T junction area is missing");
  const auto junction_node = road_test_view::junctions(state.derived()).front()->node_id;
  const auto south = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({20.0, 0.0}, {20.0, -24.0})}), 1,
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
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {25.0, 24.0})}), 1, base.value,
          20.0});
  ROAD_TEST_EXPECT(north.ok, north.error);
  const auto junction_node =
      road_test_view::junctions(state.derived()).front()->node_id;
  const auto south = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({20.0, 0.0}, {25.0, -24.0})}), 1,
          junction_node});
  ROAD_TEST_EXPECT(south.ok, south.error);
  ROAD_TEST_EXPECT(
      road_test_view::gates_of(
          *road_test_view::junctions(state.derived()).front())
              .size() == 4,
      "incremental skew cross does not have four approaches");
  return true;
}

bool P2_section_transition_and_manual_markings(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({100.0, 50.0}, {160.0, 50.0})}), 1});
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto template_id = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{ThreeLaneTemplate(0)});
  ROAD_TEST_EXPECT(template_id.ok, template_id.error);
  SectionTransitionRequest transition{};
  transition.from_template = 1;
  transition.to_template = template_id.value;
  transition.start = DistanceRef{DistanceRefKind::kFromEnd, 25.0};
  transition.end = DistanceRef{DistanceRefKind::kFromEnd, 5.0};
  transition.anchor = TransitionAnchor::kLeftEdge;
  transition.rules = {
      SectionTransitionRule{35, TransitionAction::kTaperIn},
      SectionTransitionRule{10, TransitionAction::kContinue},
  };
  const auto transition_id = state.AddTransition(transition);
  ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
  const auto attached = state.AttachSectionTransition(city::road::AttachSectionTransitionRequest{added.value, transition_id.value});
  ROAD_TEST_EXPECT(attached.ok, attached.error);
  ManualLineRequest line{};
  line.owner_segment_id = added.value;
  line.path = MakePath({MakeLine({5.0, 0.5}, {20.0, 0.5})});
  line.style_id = builtin_marking_styles::kWhiteSolid;
  const auto line_id = state.AddManualLine(line);
  ROAD_TEST_EXPECT(line_id.ok, line_id.error);
  ManualAreaRequest area{};
  area.owner_segment_id = added.value;
  area.frame_origin = {30.0, 0.0};
  area.rotation_rad = std::acos(-1.0) * 0.5;
  area.width_m = 4.0;
  area.length_m = 8.0;
  area.style_id = builtin_marking_styles::kCrosswalk;
  const auto area_id = state.AddManualArea(area);
  ROAD_TEST_EXPECT(area_id.ok, area_id.error);
  ROAD_TEST_EXPECT(state.graph().transitions.size() == 1, "P2 did not save transition authority");
  ROAD_TEST_EXPECT(state.graph().manual_lines.size() == 1, "P2 did not save manual line authority");
  ROAD_TEST_EXPECT(state.graph().manual_areas.size() == 1, "P2 did not save manual area authority");
  const auto manual_path_count = road_test_view::count_marking_lines(state.derived(), [](const auto& path) {
        return path.owner.kind == city::road::MarkingOwner::Kind::kManual;
      });
  const auto manual_area_count = road_test_view::count_marking_areas(state.derived(), [](const auto& area) {
        return area.owner.kind == city::road::MarkingOwner::Kind::kManual;
      });
  ROAD_TEST_EXPECT(manual_path_count == 1 && manual_area_count == 1,
                   "P2 did not resolve manual markings");
  ROAD_TEST_EXPECT(state.graph().segments.front().transition == transition_id.value,
                   "P2 transition is not attached to the segment authority");

  const auto sections = road_test_view::sections(state.derived());
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
  ROAD_TEST_EXPECT(std::abs(before_width - 10.4) < 1e-6, "P2 transition changed the from-section before its start");
  ROAD_TEST_EXPECT(std::abs(after_width - 13.4) < 1e-6, "P2 transition did not reach the three-lane section");
  ROAD_TEST_EXPECT(std::abs(before->boundaries.front().lateral_m - after->boundaries.front().lateral_m) < 1e-6,
                   "P2 left-edge anchor moved during one-sided widening");

  const auto manual_lines = road_test_view::marking_lines(state.derived());
  const auto manual_line_it = std::find_if(
      manual_lines.begin(), manual_lines.end(), [](const auto* path) {
        return path->owner.kind == city::road::MarkingOwner::Kind::kManual &&
               path->style_id == builtin_marking_styles::kWhiteSolid;
      });
  ROAD_TEST_EXPECT(manual_line_it != manual_lines.end(),
                   "P2 manual line was not resolved");
  ROAD_TEST_EXPECT((*manual_line_it)->points.size() >= 2,
                   "P2 manual line is not a drawable path");
  ROAD_TEST_EXPECT((*manual_line_it)->style_id == builtin_marking_styles::kWhiteSolid,
                   "P2 manual line style was saved but not consumed by draw");
  ROAD_TEST_EXPECT(std::abs((*manual_line_it)->points.front().x - 105.0) < 1e-6 &&
                       std::abs((*manual_line_it)->points.front().y - 50.5) < 0.1,
                   "P2 manual line was not transformed from owner-local coordinates");
  ROAD_TEST_EXPECT((*manual_line_it)->points.front().z > 0.02,
                   "P2 manual line does not follow the owner section cross slope");
  const auto area_mesh_it = std::find_if(
      state.derived().marking_meshes.begin(), state.derived().marking_meshes.end(),
      [](const auto& mesh) {
        return mesh.style == RenderStyleFromMarking(builtin_marking_styles::kCrosswalk);
      });
  ROAD_TEST_EXPECT(area_mesh_it != state.derived().marking_meshes.end(),
                   "P2 manual area mesh was not materialized");
  const auto& area_mesh = *area_mesh_it;
  ROAD_TEST_EXPECT(std::abs(area_mesh.vertices.front().x - 132.0) < 1e-6 &&
                       std::abs(area_mesh.vertices.front().y - 46.0) < 1e-6,
                   "P2 manual area rotation was not transformed from owner-local coordinates");
  ROAD_TEST_EXPECT(area_mesh.style == RenderStyleFromMarking(builtin_marking_styles::kCrosswalk),
                   "P2 manual area style was saved but not consumed by draw");
  ROAD_TEST_EXPECT((*manual_line_it)->style_id != builtin_marking_styles::kCrosswalk,
                   "P2 distinct manual marking styles collapsed to one mesh style");

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  ROAD_TEST_EXPECT(loaded.value.graph().transitions.size() == 1 &&
                       loaded.value.graph().manual_lines.size() == 1 &&
                       loaded.value.graph().manual_areas.size() == 1,
                   "P2 authority did not survive save/load");
  ROAD_TEST_EXPECT(loaded.value.graph().manual_areas.front().rotation_rad == area.rotation_rad,
                   "P2 manual area rotation did not survive save/load");
  const auto before_bad_style = state.Save();
  ROAD_TEST_EXPECT(before_bad_style.ok, before_bad_style.error);
  ManualLineRequest bad_line = line;
  bad_line.style_id = city::road::MarkingStyleId{999};
  const auto rejected_line = state.AddManualLine(bad_line);
  ROAD_TEST_EXPECT(!rejected_line.ok && rejected_line.error_kind == ErrorKind::kValidation,
                   "P2 accepted an unknown manual line style");
  const auto after_bad_style = state.Save();
  ROAD_TEST_EXPECT(after_bad_style.ok && after_bad_style.value == before_bad_style.value,
                   "P2 unknown manual line style mutated authoritative state");
  return true;
}

bool outer_lane_transition_preserves_existing_lanes(std::string& failure) {
  RoadState state{};
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), 5});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto corridor = city::road::FindCorridorForSegment(state.graph(), segment.value);
  ROAD_TEST_EXPECT(corridor != nullptr, "LAN3 fixture corridor is missing");

  city::road::AddLaneTransitionRequest request{};
  request.corridor_id = corridor->id;
  request.direction = city::road::LaneTravelDirection::kAlongSegment;
  request.side = city::road::RoadSide::kRight;
  request.start_corridor_distance_m = 20.0;
  request.full_width_corridor_distance_m = 60.0;
  request.lane_width_m = 3.0;
  request.anchor_boundary_id = 200;
  const auto added_lane = state.AddLaneTransition(request);
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
      ROAD_TEST_EXPECT(current.has_value() && *current == *a,
                       "LAN3 moved an existing lane boundary inside the anchor");
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
      state.graph().section_templates.begin(),
      state.graph().section_templates.end(), [](const auto& a, const auto& b) {
        return a.id < b.id;
      });
  ROAD_TEST_EXPECT(target_template != state.graph().section_templates.end(),
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
  request.anchor_boundary_id = 999999;
  const auto rejected = state.AddLaneTransition(request);
  ROAD_TEST_EXPECT(!rejected.ok &&
                       rejected.error_kind == city::road::ErrorKind::kValidation,
                   "LAN3 accepted an unknown anchor boundary");
  const auto after_reject = state.Save();
  ROAD_TEST_EXPECT(after_reject.ok && after_reject.value == saved.value,
                   "LAN3 invalid request mutated authoritative state");
  return true;
}

bool selected_outer_lane_branches_to_one_lane_road(
    std::string& failure) {
  RoadState state{};
  const auto three_lane_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 3)});
  const auto two_lane_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 2)});
  const auto one_lane_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 1)});
  ROAD_TEST_EXPECT(three_lane_template.ok && two_lane_template.ok &&
                       one_lane_template.ok,
                   "LAN4 fixture templates could not be created");
  const auto incoming = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-60.0, 0.0}, {0.0, 0.0})}),
      three_lane_template.value});
  ROAD_TEST_EXPECT(incoming.ok, incoming.error);
  const auto incoming_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&incoming](const auto& segment) { return segment.id == incoming.value; });
  ROAD_TEST_EXPECT(incoming_segment != state.graph().segments.end(),
                   "LAN4 incoming segment is missing");
  const RoadNodeId split_node = incoming_segment->node_b;

  city::road::AddConnectedLaneSegmentRequest mainline{};
  mainline.start_node = split_node;
  mainline.alignment = MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})});
  mainline.section_template = two_lane_template.value;
  mainline.lane_connections = {
      {{incoming.value, 1000, EndpointRole::kEnd}, 1000,
       city::road::LaneConnectionKind::kContinuation},
      {{incoming.value, 1010, EndpointRole::kEnd}, 1010,
       city::road::LaneConnectionKind::kContinuation},
  };
  mainline.boundary_continuations = {
      {{incoming.value, 100, EndpointRole::kEnd}, 100},
      {{incoming.value, 200, EndpointRole::kEnd}, 200},
  };
  const auto outgoing = state.AddConnectedLaneSegment(std::move(mainline));
  ROAD_TEST_EXPECT(outgoing.ok, outgoing.error);

  city::road::AddConnectedLaneSegmentRequest branch{};
  branch.start_node = split_node;
  branch.alignment = MakePath({MakeLine({0.0, 0.0}, {42.0, -42.0})});
  branch.section_template = one_lane_template.value;
  branch.lane_connections = {
      {{incoming.value, 1020, EndpointRole::kEnd}, 1000,
       city::road::LaneConnectionKind::kSplit},
  };
  branch.boundary_continuations = {
      {{incoming.value, 300, EndpointRole::kEnd}, 100,
       city::road::BoundaryContinuationKind::kSplit},
      {{incoming.value, 900, EndpointRole::kEnd}, 900,
       city::road::BoundaryContinuationKind::kSplit},
  };
  const auto branch_segment =
      state.AddConnectedLaneSegment(std::move(branch));
  ROAD_TEST_EXPECT(branch_segment.ok, branch_segment.error);

  ROAD_TEST_EXPECT(state.graph().lane_connections.size() == 3,
                   "LAN4 did not save all explicit lane connections");
  ROAD_TEST_EXPECT(state.graph().boundary_continuations.size() == 4,
                   "LAN4 did not save explicit mainline and branch boundaries");
  ROAD_TEST_EXPECT(state.derived().lane_paths.size() == 3,
                   "LAN4 did not derive one path per lane connection");
  ROAD_TEST_EXPECT(state.derived().boundary_paths.size() == 4,
                   "LAN4 did not derive one path per boundary continuation");
  const auto split = std::find_if(
      state.graph().lane_connections.begin(),
      state.graph().lane_connections.end(), [](const auto& connection) {
        return connection.kind == city::road::LaneConnectionKind::kSplit;
      });
  ROAD_TEST_EXPECT(split != state.graph().lane_connections.end() &&
                       split->source.lane_id == 1020 &&
                       split->target.segment_id == branch_segment.value &&
                       split->target.lane_id == 1000,
                   "LAN4 connected a lane other than the selected outer lane");
  const auto split_path = std::find_if(
      state.derived().lane_paths.begin(), state.derived().lane_paths.end(),
      [&split](const auto& path) { return path.connection_id == split->id; });
  ROAD_TEST_EXPECT(split_path != state.derived().lane_paths.end() &&
                       split_path->centerline.spans.size() == 1,
                   "LAN4 split lane path is missing");
  const auto& curve = split_path->centerline.spans.front();
  ROAD_TEST_EXPECT(std::abs(curve.p0.y - 3.0) < 1e-6,
                   "LAN4 branch starts at the road center instead of lane 2");
  ROAD_TEST_EXPECT(split_path->length_m > 0.0 &&
                       split_path->minimum_radius_m > 0.0 &&
                       std::isfinite(split_path->minimum_radius_m),
                   "LAN4 branch lane path is degenerate");
  const Vec2d start_tangent{curve.p1.x - curve.p0.x,
                            curve.p1.y - curve.p0.y};
  const Vec2d end_tangent{curve.p3.x - curve.p2.x,
                          curve.p3.y - curve.p2.y};
  ROAD_TEST_EXPECT(start_tangent.x > 0.0 &&
                       std::abs(start_tangent.y) < 1e-9 &&
                       end_tangent.x > 0.0 && end_tangent.y < 0.0,
                   "LAN4 branch path is not G1 with its source and target lanes");
  const auto samples = city::road::FlattenPath(split_path->centerline);
  for (std::size_t index = 1; index < samples.size(); ++index) {
    ROAD_TEST_EXPECT(samples[index].x + 1e-9 >= samples[index - 1].x &&
                         samples[index].y <= samples[index - 1].y + 1e-9,
                     "LAN4 branch path reverses lateral or longitudinal direction");
  }
  ROAD_TEST_EXPECT(state.derived().separation_areas.size() == 1 &&
                       state.derived().separation_areas.front().perimeter.size() >= 6 &&
                       std::all_of(
                           state.derived().separation_areas.front().perimeter.begin(),
                           state.derived().separation_areas.front().perimeter.end(),
                           [](const auto& point) {
                             return std::isfinite(point.x) &&
                                    std::isfinite(point.y) &&
                                    std::isfinite(point.z);
                           }),
                   "LAN4 separation area is missing or non-finite");
  for (const auto& connection : state.graph().lane_connections) {
    if (connection.kind != city::road::LaneConnectionKind::kContinuation)
      continue;
    const auto path = std::find_if(
        state.derived().lane_paths.begin(), state.derived().lane_paths.end(),
        [&connection](const auto& candidate) {
          return candidate.connection_id == connection.id;
        });
    ROAD_TEST_EXPECT(path != state.derived().lane_paths.end(),
                     "LAN4 mainline lane path is missing");
    const auto& main_curve = path->centerline.spans.front();
    ROAD_TEST_EXPECT(main_curve.p0.y == main_curve.p1.y &&
                         main_curve.p1.y == main_curve.p2.y &&
                         main_curve.p2.y == main_curve.p3.y,
                     "LAN4 bent an unaffected mainline lane");
  }
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto resaved = loaded.value.Save();
  ROAD_TEST_EXPECT(resaved.ok && resaved.value == saved.value &&
                       loaded.value.derived().lane_paths.size() == 3 &&
                       loaded.value.derived().boundary_paths.size() == 4,
                   "LAN4 topology or derived paths changed across save/load");
  return true;
}

bool one_lane_road_merges_into_outer_mainline_lane(
    std::string& failure) {
  RoadState state{};
  const auto outgoing_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 2)});
  const auto incoming_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(
          0, 2, city::road::LaneTravelDirection::kAgainstSegment)});
  const auto branch_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(
          0, 1, city::road::LaneTravelDirection::kAgainstSegment)});
  ROAD_TEST_EXPECT(outgoing_template.ok && incoming_template.ok &&
                       branch_template.ok,
                   "LAN5 fixture templates could not be created");

  const auto outgoing = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}),
      outgoing_template.value});
  ROAD_TEST_EXPECT(outgoing.ok, outgoing.error);
  const auto outgoing_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&outgoing](const auto& segment) { return segment.id == outgoing.value; });
  ROAD_TEST_EXPECT(outgoing_segment != state.graph().segments.end(),
                   "LAN5 outgoing segment is missing");
  const RoadNodeId merge_node = outgoing_segment->node_a;

  city::road::AddConnectedLaneSegmentRequest incoming{};
  incoming.start_node = merge_node;
  incoming.alignment = MakePath({MakeLine({0.0, 0.0}, {-60.0, 0.0})});
  incoming.section_template = incoming_template.value;
  incoming.source_lane_connections = {
      {1000, {outgoing.value, 1010, EndpointRole::kStart},
       city::road::LaneConnectionKind::kMerge},
      {1010, {outgoing.value, 1000, EndpointRole::kStart},
       city::road::LaneConnectionKind::kMerge},
  };
  incoming.source_boundary_continuations = {
      {100, {outgoing.value, 900, EndpointRole::kStart},
       city::road::BoundaryContinuationKind::kMerge},
      {200, {outgoing.value, 200, EndpointRole::kStart},
       city::road::BoundaryContinuationKind::kMerge},
      {900, {outgoing.value, 100, EndpointRole::kStart},
       city::road::BoundaryContinuationKind::kMerge},
  };
  const auto incoming_segment_id =
      state.AddConnectedLaneSegment(std::move(incoming));
  ROAD_TEST_EXPECT(incoming_segment_id.ok, incoming_segment_id.error);

  city::road::AddConnectedLaneSegmentRequest branch{};
  branch.start_node = merge_node;
  branch.alignment = MakePath({MakeLine({0.0, 0.0}, {-42.0, -42.0})});
  branch.section_template = branch_template.value;
  branch.source_lane_connections = {
      {1000, {outgoing.value, 1010, EndpointRole::kStart},
       city::road::LaneConnectionKind::kMerge},
  };
  branch.source_boundary_continuations = {
      {100, {outgoing.value, 200, EndpointRole::kStart},
       city::road::BoundaryContinuationKind::kMerge},
      {900, {outgoing.value, 900, EndpointRole::kStart},
       city::road::BoundaryContinuationKind::kMerge},
  };
  const auto branch_segment_id =
      state.AddConnectedLaneSegment(std::move(branch));
  ROAD_TEST_EXPECT(branch_segment_id.ok, branch_segment_id.error);

  ROAD_TEST_EXPECT(state.graph().lane_connections.size() == 3,
                   "LAN5 did not save all merge lane connections");
  const auto branch_merge = std::find_if(
      state.graph().lane_connections.begin(),
      state.graph().lane_connections.end(), [&branch_segment_id](const auto& c) {
        return c.source.segment_id == branch_segment_id.value;
      });
  ROAD_TEST_EXPECT(
      branch_merge != state.graph().lane_connections.end() &&
          branch_merge->source.lane_id == 1000 &&
          branch_merge->target.segment_id == outgoing.value &&
          branch_merge->target.lane_id == 1010 &&
          branch_merge->kind == city::road::LaneConnectionKind::kMerge,
      "LAN5 branch did not merge into the selected outer mainline lane");
  ROAD_TEST_EXPECT(state.derived().lane_paths.size() == 3 &&
                       state.derived().boundary_paths.size() == 5,
                   "LAN5 did not derive all explicit merge paths");
  const auto branch_path = std::find_if(
      state.derived().lane_paths.begin(), state.derived().lane_paths.end(),
      [&branch_merge](const auto& path) {
        return path.connection_id == branch_merge->id;
      });
  ROAD_TEST_EXPECT(branch_path != state.derived().lane_paths.end() &&
                       !branch_path->centerline.spans.empty() &&
                       std::isfinite(branch_path->minimum_radius_m) &&
                       branch_path->minimum_radius_m > 0.0,
                   "LAN5 branch merge path is missing or degenerate");
  const auto& merge_span = branch_path->centerline.spans.front();
  const Vec2d start_tangent{merge_span.p1.x - merge_span.p0.x,
                            merge_span.p1.y - merge_span.p0.y};
  const Vec2d end_tangent{merge_span.p3.x - merge_span.p2.x,
                          merge_span.p3.y - merge_span.p2.y};
  ROAD_TEST_EXPECT(start_tangent.x > 0.0 && start_tangent.y > 0.0 &&
                       end_tangent.x > 0.0 && std::abs(end_tangent.y) < 1e-9,
                   "LAN5 merge path is not G1 with its source and target lanes");

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  city::road::AddConnectedLaneSegmentRequest wrong_direction{};
  wrong_direction.start_node = merge_node;
  wrong_direction.alignment =
      MakePath({MakeLine({0.0, 0.0}, {-30.0, 30.0})});
  wrong_direction.section_template = outgoing_template.value;
  wrong_direction.source_lane_connections = {
      {1000, {outgoing.value, 1010, EndpointRole::kStart},
       city::road::LaneConnectionKind::kMerge},
  };
  const auto rejected =
      state.AddConnectedLaneSegment(std::move(wrong_direction));
  ROAD_TEST_EXPECT(!rejected.ok &&
                       rejected.error_kind == city::road::ErrorKind::kValidation,
                   "LAN5 accepted a lane whose travel direction does not exit the node");
  const auto after_reject = state.Save();
  ROAD_TEST_EXPECT(after_reject.ok && after_reject.value == saved.value,
                   "LAN5 invalid merge request mutated authoritative state");

  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto resaved = loaded.value.Save();
  ROAD_TEST_EXPECT(resaved.ok && resaved.value == saved.value &&
                       loaded.value.derived().lane_paths.size() == 3 &&
                       loaded.value.derived().boundary_paths.size() == 5,
                   "LAN5 merge topology did not round-trip deterministically");
  return true;
}

bool opposing_lanes_and_median_survive_one_direction_branch(
    std::string& failure) {
  RoadState state{};
  const auto mainline_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OpposingFourLaneTemplate(0)});
  const auto branch_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 1)});
  ROAD_TEST_EXPECT(mainline_template.ok && branch_template.ok,
                   "LAN6 fixture templates could not be created");
  const auto incoming = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-60.0, 0.0}, {0.0, 0.0})}),
      mainline_template.value});
  ROAD_TEST_EXPECT(incoming.ok, incoming.error);
  const auto incoming_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&incoming](const auto& segment) { return segment.id == incoming.value; });
  ROAD_TEST_EXPECT(incoming_segment != state.graph().segments.end(),
                   "LAN6 incoming segment is missing");
  const RoadNodeId split_node = incoming_segment->node_b;

  city::road::AddConnectedLaneSegmentRequest outgoing{};
  outgoing.start_node = split_node;
  outgoing.alignment = MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})});
  outgoing.section_template = mainline_template.value;
  outgoing.lane_connections = {
      {{incoming.value, 2000, EndpointRole::kEnd}, 2000,
       city::road::LaneConnectionKind::kContinuation},
      {{incoming.value, 2010, EndpointRole::kEnd}, 2010,
       city::road::LaneConnectionKind::kSplit},
  };
  outgoing.source_lane_connections = {
      {1000, {incoming.value, 1000, EndpointRole::kEnd},
       city::road::LaneConnectionKind::kContinuation},
      {1010, {incoming.value, 1010, EndpointRole::kEnd},
       city::road::LaneConnectionKind::kContinuation},
  };
  outgoing.boundary_continuations = {
      {{incoming.value, 100, EndpointRole::kEnd}, 100},
      {{incoming.value, 150, EndpointRole::kEnd}, 150},
      {{incoming.value, 200, EndpointRole::kEnd}, 200},
      {{incoming.value, 210, EndpointRole::kEnd}, 210},
      {{incoming.value, 250, EndpointRole::kEnd}, 250,
       city::road::BoundaryContinuationKind::kSplit},
      {{incoming.value, 300, EndpointRole::kEnd}, 300,
       city::road::BoundaryContinuationKind::kSplit},
  };
  const auto outgoing_segment_id =
      state.AddConnectedLaneSegment(std::move(outgoing));
  ROAD_TEST_EXPECT(outgoing_segment_id.ok, outgoing_segment_id.error);

  city::road::AddConnectedLaneSegmentRequest branch{};
  branch.start_node = split_node;
  branch.alignment = MakePath({MakeLine({0.0, 0.0}, {42.0, -42.0})});
  branch.section_template = branch_template.value;
  branch.lane_connections = {
      {{incoming.value, 2010, EndpointRole::kEnd}, 1000,
       city::road::LaneConnectionKind::kSplit},
  };
  branch.boundary_continuations = {
      {{incoming.value, 250, EndpointRole::kEnd}, 100,
       city::road::BoundaryContinuationKind::kSplit},
      {{incoming.value, 300, EndpointRole::kEnd}, 900,
       city::road::BoundaryContinuationKind::kSplit},
  };
  const auto branch_segment_id =
      state.AddConnectedLaneSegment(std::move(branch));
  ROAD_TEST_EXPECT(branch_segment_id.ok, branch_segment_id.error);

  const auto template_after = std::find_if(
      state.graph().section_templates.begin(),
      state.graph().section_templates.end(), [&mainline_template](const auto& t) {
        return t.id == mainline_template.value;
      });
  ROAD_TEST_EXPECT(template_after != state.graph().section_templates.end() &&
                       template_after->lane_bands.size() == 4 &&
                       template_after->boundaries.size() == 6,
                   "LAN6 branch changed the opposing mainline section shape");
  const auto median = std::find_if(
      template_after->strips.begin(), template_after->strips.end(),
      [](const auto& strip) { return strip.id == 40; });
  const auto median_left = std::find_if(
      template_after->boundaries.begin(), template_after->boundaries.end(),
      [](const auto& boundary) { return boundary.boundary_id == 200; });
  const auto median_right = std::find_if(
      template_after->boundaries.begin(), template_after->boundaries.end(),
      [](const auto& boundary) { return boundary.boundary_id == 210; });
  ROAD_TEST_EXPECT(
      median != template_after->strips.end() && median->width_m == 2.0 &&
          median->function == StripFunction::kMedian &&
          median_left != template_after->boundaries.end() &&
          median_left->role == BoundaryRole::kMedianEdge &&
          median_right != template_after->boundaries.end() &&
          median_right->role == BoundaryRole::kMedianEdge,
      "LAN6 branch changed the median strip or its stable boundaries");
  for (const auto& connection : state.graph().lane_connections) {
    const auto is_opposing_mainline = [&](const auto& endpoint) {
      return (endpoint.segment_id == incoming.value ||
              endpoint.segment_id == outgoing_segment_id.value) &&
             (endpoint.lane_id == 1000 || endpoint.lane_id == 1010);
    };
    const bool touches_opposing = is_opposing_mainline(connection.source) ||
                                  is_opposing_mainline(connection.target);
    if (touches_opposing) {
      ROAD_TEST_EXPECT(
          connection.kind == city::road::LaneConnectionKind::kContinuation &&
              connection.source.segment_id == outgoing_segment_id.value &&
              connection.target.segment_id == incoming.value,
          "LAN6 branch consumed or redirected an opposing lane");
    }
  }
  for (const auto& continuation : state.graph().boundary_continuations) {
    const bool touches_median = continuation.source.boundary_id == 200 ||
                                continuation.source.boundary_id == 210 ||
                                continuation.target.boundary_id == 200 ||
                                continuation.target.boundary_id == 210;
    if (touches_median) {
      ROAD_TEST_EXPECT(
          continuation.kind ==
                  city::road::BoundaryContinuationKind::kContinuation &&
              continuation.source.segment_id == incoming.value &&
              continuation.target.segment_id == outgoing_segment_id.value,
          "LAN6 branch consumed or redirected a median boundary");
    }
  }
  const auto split = std::find_if(
      state.graph().lane_connections.begin(),
      state.graph().lane_connections.end(), [&branch_segment_id](const auto& c) {
        return c.target.segment_id == branch_segment_id.value;
      });
  ROAD_TEST_EXPECT(split != state.graph().lane_connections.end() &&
                       split->source.lane_id == 2010 &&
                       split->target.lane_id == 1000 &&
                       split->kind == city::road::LaneConnectionKind::kSplit,
                   "LAN6 did not branch only the selected travel direction");
  const auto branch_path = std::find_if(
      state.derived().lane_paths.begin(), state.derived().lane_paths.end(),
      [&split](const auto& path) {
        return path.connection_id == split->id;
      });
  ROAD_TEST_EXPECT(branch_path != state.derived().lane_paths.end() &&
                       branch_path->centerline.spans.size() == 1 &&
                       std::isfinite(branch_path->minimum_radius_m),
                   "LAN6 selected lane branch path is missing or non-finite");
  return true;
}

bool junction_movements_are_explicit_lane_connections(
    std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = state.AddSegmentConnectedToSegment(
      city::road::AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({30.0, 0.0}, {30.0, 30.0})}), 1,
          base.value, 30.0});
  ROAD_TEST_EXPECT(north.ok, north.error);
  ROAD_TEST_EXPECT(!road_test_view::junctions(state.derived()).empty(),
                   "LAN7 T junction was not generated");
  const RoadNodeId node =
      road_test_view::junctions(state.derived()).front()->node_id;
  const auto south = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({30.0, 0.0}, {30.0, -30.0})}), 1, node});
  ROAD_TEST_EXPECT(south.ok, south.error);

  RoadSegmentId west_segment = 0;
  RoadSegmentId east_segment = 0;
  for (const auto& segment : state.graph().segments) {
    if (segment.id == north.value || segment.id == south.value)
      continue;
    const RoadNodeId other_id = segment.node_a == node ? segment.node_b
                                                       : segment.node_a;
    const auto other = std::find_if(
        state.graph().nodes.begin(), state.graph().nodes.end(),
        [other_id](const auto& candidate) { return candidate.id == other_id; });
    ROAD_TEST_EXPECT(other != state.graph().nodes.end(),
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
        state.graph().segments.begin(), state.graph().segments.end(),
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
  std::vector<city::road::LaneConnectionId> movement_ids{};
  for (const auto& target : targets) {
    const auto movement = state.AddLaneConnection(
        city::road::AddLaneConnectionRequest{
            source, target, city::road::LaneConnectionKind::kJunctionMovement});
    ROAD_TEST_EXPECT(movement.ok, movement.error);
    movement_ids.push_back(movement.value);
  }
  ROAD_TEST_EXPECT(state.derived().lane_paths.size() == 3,
                   "LAN7 did not derive all explicit junction movements");
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

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const city::road::LaneEndpointKey wrong_source{
      west_segment, 1000, role_at(west_segment)};
  const auto rejected = state.AddLaneConnection(
      city::road::AddLaneConnectionRequest{
          wrong_source, targets.front(),
          city::road::LaneConnectionKind::kJunctionMovement});
  ROAD_TEST_EXPECT(!rejected.ok &&
                       rejected.error_kind == city::road::ErrorKind::kValidation,
                   "LAN7 accepted a wrong-way junction movement");
  const auto after_reject = state.Save();
  ROAD_TEST_EXPECT(after_reject.ok && after_reject.value == saved.value,
                   "LAN7 wrong-way movement mutated authoritative state");
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok && loaded.value.derived().lane_paths.size() == 3,
                   "LAN7 junction movements did not survive save/load");

  RoadState non_junction{};
  const auto first = non_junction.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {30.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(first.ok, first.error);
  const RoadNodeId pass_node = non_junction.graph().segments.front().node_b;
  const auto second = non_junction.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({30.0, 0.0}, {60.0, 0.0})}), 1, pass_node});
  ROAD_TEST_EXPECT(second.ok, second.error);
  const auto before_non_junction = non_junction.Save();
  ROAD_TEST_EXPECT(before_non_junction.ok, before_non_junction.error);
  const auto rejected_non_junction = non_junction.AddLaneConnection(
      city::road::AddLaneConnectionRequest{
          {first.value, 1010, EndpointRole::kEnd},
          {second.value, 1010, EndpointRole::kStart},
          city::road::LaneConnectionKind::kJunctionMovement});
  ROAD_TEST_EXPECT(
      !rejected_non_junction.ok &&
          rejected_non_junction.error_kind ==
              city::road::ErrorKind::kUnsupported,
      "LAN7 accepted junction movement topology outside a junction");
  const auto after_non_junction = non_junction.Save();
  ROAD_TEST_EXPECT(after_non_junction.ok &&
                       after_non_junction.value == before_non_junction.value,
                   "LAN7 non-junction movement failure mutated state");
  return true;
}

bool branch_markings_follow_explicit_boundary_paths(
    std::string& failure) {
  RoadState state{};
  const auto source_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 3)});
  const auto mainline_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 2)});
  const auto branch_template = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{OneWayLaneTemplate(0, 1)});
  ROAD_TEST_EXPECT(source_template.ok && mainline_template.ok &&
                       branch_template.ok,
                   "LAN8 fixture templates could not be created");
  const auto source = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({-60.0, 0.0}, {0.0, 0.0})}),
      source_template.value});
  ROAD_TEST_EXPECT(source.ok, source.error);
  const auto source_segment = std::find_if(
      state.graph().segments.begin(), state.graph().segments.end(),
      [&source](const auto& segment) { return segment.id == source.value; });
  ROAD_TEST_EXPECT(source_segment != state.graph().segments.end(),
                   "LAN8 source segment is missing");
  const RoadNodeId split_node = source_segment->node_b;

  city::road::AddConnectedLaneSegmentRequest mainline{};
  mainline.start_node = split_node;
  mainline.alignment = MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})});
  mainline.section_template = mainline_template.value;
  mainline.lane_connections = {
      {{source.value, 1000, EndpointRole::kEnd}, 1000,
       city::road::LaneConnectionKind::kContinuation},
      {{source.value, 1010, EndpointRole::kEnd}, 1010,
       city::road::LaneConnectionKind::kContinuation},
  };
  mainline.boundary_continuations = {
      {{source.value, 100, EndpointRole::kEnd}, 100},
      {{source.value, 200, EndpointRole::kEnd}, 200},
  };
  const auto mainline_segment =
      state.AddConnectedLaneSegment(std::move(mainline));
  ROAD_TEST_EXPECT(mainline_segment.ok, mainline_segment.error);

  city::road::AddConnectedLaneSegmentRequest branch{};
  branch.start_node = split_node;
  branch.alignment = MakePath({MakeLine({0.0, 0.0}, {42.0, -42.0})});
  branch.section_template = branch_template.value;
  branch.lane_connections = {
      {{source.value, 1020, EndpointRole::kEnd}, 1000,
       city::road::LaneConnectionKind::kSplit},
  };
  branch.boundary_continuations = {
      {{source.value, 300, EndpointRole::kEnd}, 100,
       city::road::BoundaryContinuationKind::kSplit},
      {{source.value, 900, EndpointRole::kEnd}, 900,
       city::road::BoundaryContinuationKind::kSplit},
  };
  const auto branch_segment = state.AddConnectedLaneSegment(std::move(branch));
  ROAD_TEST_EXPECT(branch_segment.ok, branch_segment.error);

  for (const auto boundary_id : {300ULL, 900ULL}) {
    const auto topology = std::find_if(
        state.graph().boundary_continuations.begin(),
        state.graph().boundary_continuations.end(),
        [boundary_id, &branch_segment](const auto& continuation) {
          return continuation.source.boundary_id == boundary_id &&
                 continuation.target.segment_id == branch_segment.value;
        });
    ROAD_TEST_EXPECT(topology != state.graph().boundary_continuations.end(),
                     "LAN8 explicit branch boundary topology is missing");
    const auto boundary_path = std::find_if(
        state.derived().boundary_paths.begin(),
        state.derived().boundary_paths.end(), [&topology](const auto& path) {
          return path.continuation_id == topology->id;
        });
    const auto marking = road_test_view::find_marking_line(
        state.derived(), [boundary_id, split_node](const auto& candidate) {
          return candidate.owner.kind == MarkingOwner::Kind::kJunction &&
                 candidate.owner.node_id == split_node &&
                 candidate.boundary_id == boundary_id;
        });
    ROAD_TEST_EXPECT(boundary_path != state.derived().boundary_paths.end() &&
                         boundary_path->path.spans.size() == 1 &&
                         marking != nullptr && marking->points.size() >= 2,
                     "LAN8 branch boundary did not produce a marking path");
    const auto boundary_points = FlattenPath(boundary_path->path);
    ROAD_TEST_EXPECT(
        !boundary_points.empty() &&
            marking->points.front().x == boundary_points.front().x &&
            marking->points.front().y == boundary_points.front().y &&
            marking->points.back().x == boundary_points.back().x &&
            marking->points.back().y == boundary_points.back().y,
        "LAN8 marking does not follow its DerivedBoundaryPath");
  }
  return true;
}

bool P2_supports_taper_lane_reduction_and_median_end(std::string& failure) {
  {
    RoadState state{};
    auto no_left_sidewalk = JapaneseUrbanTwoLaneTemplate(0);
    no_left_sidewalk.strips.erase(no_left_sidewalk.strips.begin());
    no_left_sidewalk.boundaries.erase(no_left_sidewalk.boundaries.begin());
    const auto target = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{no_left_sidewalk});
    ROAD_TEST_EXPECT(target.ok, target.error);
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransitionRequest transition{};
    transition.from_template = 1;
    transition.to_template = target.value;
    transition.start = DistanceRef{DistanceRefKind::kFromStart, 10.0};
    transition.end = DistanceRef{DistanceRefKind::kRatio, 0.5};
    transition.anchor = TransitionAnchor::kRightEdge;
    transition.rules = {SectionTransitionRule{10, TransitionAction::kTaperOut}};
    const auto transition_id = state.AddTransition(transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(state.AttachSectionTransition(city::road::AttachSectionTransitionRequest{segment.value, transition_id.value}).ok,
                     "sidewalk taper could not be attached");
    ROAD_TEST_EXPECT(!state.derived().segment_meshes.empty(), "sidewalk taper produced no material meshes");
  }

  {
    RoadState state{};
    const auto three_lane = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{ThreeLaneTemplate(0)});
    ROAD_TEST_EXPECT(three_lane.ok, three_lane.error);
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), three_lane.value});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransitionRequest transition{};
    transition.from_template = three_lane.value;
    transition.to_template = 1;
    transition.start = DistanceRef{DistanceRefKind::kFromEnd, 30.0};
    transition.end = DistanceRef{DistanceRefKind::kFromEnd, 5.0};
    transition.rules = {SectionTransitionRule{35, TransitionAction::kTaperOut}};
    const auto transition_id = state.AddTransition(transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(state.AttachSectionTransition(city::road::AttachSectionTransitionRequest{segment.value, transition_id.value}).ok,
                     "lane reduction could not be attached");
  }

  {
    RoadState state{};
    auto median = JapaneseUrbanTwoLaneTemplate(0);
    const city::road::AutoMarkingPolicy outer_line{
        true, MarkingRole::kCarriagewayEdge, builtin_marking_styles::kWhiteSolid};
    median.strips.insert(median.strips.begin() + 2,
                        {25, StripFunction::kMedian, 2.0, 0.0, builtin_surface_styles::kMedian});
    median.boundaries = {
        {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
        {210, BoundaryRole::kMedianEdge, 0.2, 0.12, {}},
        {220, BoundaryRole::kMedianEdge, 0.2, -0.12, {}},
        {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
    };
    const auto median_id = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{median});
    ROAD_TEST_EXPECT(median_id.ok, median_id.error);
    const auto segment = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), median_id.value});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransitionRequest invalid{};
    invalid.from_template = median_id.value;
    invalid.to_template = 1;
    invalid.start = DistanceRef{DistanceRefKind::kFromStart, 10.0};
    invalid.end = DistanceRef{DistanceRefKind::kFromStart, 30.0};
    invalid.rules = {SectionTransitionRule{25, TransitionAction::kTaperOut}};
    ROAD_TEST_EXPECT(!state.AddTransition(invalid).ok, "median disappearance accepted TaperOut instead of EndCap");
    invalid.rules = {SectionTransitionRule{25, TransitionAction::kEndCap}};
    const auto transition_id = state.AddTransition(invalid);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(state.AttachSectionTransition(city::road::AttachSectionTransitionRequest{segment.value, transition_id.value}).ok,
                     "median end cap could not be attached");
  }
  return true;
}

bool P2_requires_transition_for_mixed_section_connection(std::string& failure) {
  {
    RoadState state{};
    const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1});
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto endpoint = state.graph().segments.front().node_b;
    const auto direct = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), 2, endpoint});
    ROAD_TEST_EXPECT(!direct.ok && direct.error_kind == ErrorKind::kUnsupported,
                     "P2 accepted a mixed-section node connection without a transition");
  }

  {
    RoadState state{};
    const auto base = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1});
    ROAD_TEST_EXPECT(base.ok, base.error);
    SectionTransitionRequest transition{};
    transition.to_template = 2;
    transition.start = DistanceRef{DistanceRefKind::kFromEnd, 20.0};
    transition.end = DistanceRef{DistanceRefKind::kFromEnd, 0.0};
    transition.rules = {SectionTransitionRule{35, TransitionAction::kTaperIn}};
    const auto transition_id = state.AddTransitionToSegment(city::road::AddTransitionToSegmentRequest{base.value, transition});
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    const auto endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), 2, endpoint});
    ROAD_TEST_EXPECT(connected.ok, connected.error);
  }
  return true;
}

bool P2_marking_policy_suppression_and_junction_override(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_b;
  const auto branch_a = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({40.0, 0.0}, {40.0, 30.0})}), 1, node});
  ROAD_TEST_EXPECT(branch_a.ok, branch_a.error);
  const auto branch_b = state.AddSegmentConnectedTo(
      city::road::AddSegmentConnectedToRequest{
          MakePath({MakeLine({40.0, 0.0}, {70.0, 0.0})}), 1, node});
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

  AutoMarkingKey suppress_key{
      MarkingOwner{MarkingOwner::Kind::kRoadSegment, base.value, 0, 0},
      MarkingRole::kCenterLine,
      MarkingTrackKey{base.value, 200, MarkingRole::kCenterLine},
      std::nullopt};
  const auto suppressed = state.SuppressAutoMarking(city::road::SuppressAutoMarkingRequest{suppress_key});
  ROAD_TEST_EXPECT(suppressed.ok, suppressed.error);
  ROAD_TEST_EXPECT(center_paths() + 1 == center_before,
                   "semantic suppression did not remove exactly one segment marking");
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  ROAD_TEST_EXPECT(saved.value.find("auto_marking_override.count=1") != std::string::npos,
                   "auto marking suppression was not persisted");
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  ROAD_TEST_EXPECT(road_test_view::count_marking_lines(loaded.value.derived(), [](const auto& path) {
                                   return path.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                                          path.role == MarkingRole::kCenterLine;
                                 }) == static_cast<long long>(center_paths()),
                   "auto marking suppression was not restored");

  const auto reset = state.ResetAutoMarkingSuppression(
      city::road::ResetAutoMarkingSuppressionRequest{suppress_key});
  ROAD_TEST_EXPECT(reset.ok, reset.error);
  ROAD_TEST_EXPECT(center_paths() == center_before, "suppression reset did not restore center line");

  const auto& area = *road_test_view::junctions(state.derived()).front();
  ROAD_TEST_EXPECT(road_test_view::gates_of(area).size() >= 2, "junction override test needs two gates");
  JunctionMarkingOverride override{};
  override.node_id = area.node_id;
  override.source = JunctionMarkingEndpoint{area.approaches[0].gate.approach, 200, MarkingRole::kCenterLine};
  override.action = JunctionMarkingAction::kConnectToApproach;
  override.target = JunctionMarkingEndpoint{area.approaches[1].gate.approach, 200, MarkingRole::kCenterLine};
  const auto override_id =
      state.SetJunctionMarkingOverride(city::road::SetJunctionMarkingOverrideRequest{override});
  ROAD_TEST_EXPECT(override_id.ok, override_id.error);
  ROAD_TEST_EXPECT(0 < road_test_view::count_marking_lines(state.derived(), [&](const auto& path) {
                                 return path.owner.kind == MarkingOwner::Kind::kJunction &&
                                        path.role == MarkingRole::kCenterLine &&
                                        path.points.size() == 2;
                               }),
                   "explicit junction marking override did not create a junction-owned path");
  const auto deleted = state.DeleteJunctionMarkingOverride(
      city::road::DeleteJunctionMarkingOverrideRequest{override_id.value});
  ROAD_TEST_EXPECT(deleted.ok, deleted.error);
  ROAD_TEST_EXPECT(0 == road_test_view::count_marking_lines(state.derived(), [](const auto& path) {
                                  return path.owner.kind == MarkingOwner::Kind::kJunction &&
                                         path.role == MarkingRole::kCenterLine;
                                }),
                   "deleting junction marking override did not remove the explicit path");

  const auto disabled = state.ResetBoundaryMarkingPolicy(
      city::road::ResetBoundaryMarkingPolicyRequest{1, 200});
  ROAD_TEST_EXPECT(disabled.ok, disabled.error);
  ROAD_TEST_EXPECT(0 == road_test_view::count_marking_lines(state.derived(), [](const auto& path) {
                                  return path.owner.kind == MarkingOwner::Kind::kRoadSegment &&
                                         path.role == MarkingRole::kCenterLine;
                                }),
                   "reset boundary policy did not remove center line tracks");
  const auto reenabled = state.SetBoundaryMarkingPolicy(
      city::road::SetBoundaryMarkingPolicyRequest{
          1, 200,
          AutoMarkingPolicy{true, MarkingRole::kCenterLine,
                            builtin_marking_styles::kCenterLine}});
  ROAD_TEST_EXPECT(reenabled.ok, reenabled.error);
  ROAD_TEST_EXPECT(center_paths() == center_before,
                   "set boundary policy did not restore center line tracks");
  const auto unknown_style = state.SetBoundaryMarkingPolicy(
      city::road::SetBoundaryMarkingPolicyRequest{
          1, 200, AutoMarkingPolicy{true, MarkingRole::kCenterLine,
                                    city::road::MarkingStyleId{9999}}});
  ROAD_TEST_EXPECT(!unknown_style.ok && unknown_style.error_kind == ErrorKind::kValidation,
                   "unknown boundary marking style was not validation rejected");
  return true;
}

// A boundary line is identified by its owner segment and boundary ID.
const city::road::DerivedMarking* find_boundary_line(const RoadState& state,
                                                     city::road::RoadSegmentId segment_id,
                                                     std::uint64_t boundary_id) {
  for (const auto* marking : road_test_view::marking_lines(state.derived())) {
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
  const auto segment = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const std::size_t baseline = road_test_view::marking_lines(state.derived()).size();
  const AutoMarkingPolicy center{true, MarkingRole::kCenterLine,
                                 builtin_marking_styles::kCenterLine};
  // Both lanes request the same line on the boundary they share.
  const auto left_request = state.SetLaneSideMarkingPolicy(
      city::road::SetLaneSideMarkingPolicyRequest{1, 20, city::road::LaneSide::kRight, center});
  ROAD_TEST_EXPECT(left_request.ok, left_request.error);
  const auto right_request = state.SetLaneSideMarkingPolicy(
      city::road::SetLaneSideMarkingPolicyRequest{1, 30, city::road::LaneSide::kLeft, center});
  ROAD_TEST_EXPECT(right_request.ok, right_request.error);
  ROAD_TEST_EXPECT(road_test_view::marking_lines(state.derived()).size() == baseline,
                   "duplicate lane side requests did not merge into one line");

  const auto saved_before = state.Save();
  ROAD_TEST_EXPECT(saved_before.ok, saved_before.error);
  const AutoMarkingPolicy conflicting{true, MarkingRole::kLaneSeparator,
                                      builtin_marking_styles::kWhiteDashed};
  const auto conflict = state.SetLaneSideMarkingPolicy(
      city::road::SetLaneSideMarkingPolicyRequest{1, 30, city::road::LaneSide::kLeft, conflicting});
  ROAD_TEST_EXPECT(!conflict.ok && conflict.error_kind == ErrorKind::kUnsupported,
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
  // Template 3 starts with a carriageway band, so its left side has no boundary.
  const auto segment = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 3});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  const auto saved_before = state.Save();
  ROAD_TEST_EXPECT(saved_before.ok, saved_before.error);
  const auto request = state.SetLaneSideMarkingPolicy(
      city::road::SetLaneSideMarkingPolicyRequest{
          3, 20, city::road::LaneSide::kLeft,
          AutoMarkingPolicy{true, MarkingRole::kCarriagewayEdge,
                            builtin_marking_styles::kWhiteSolid}});
  ROAD_TEST_EXPECT(!request.ok && request.error_kind == ErrorKind::kUnsupported,
                   "outermost lane side request was not unsupported");
  const auto saved_after = state.Save();
  ROAD_TEST_EXPECT(saved_after.ok && saved_after.value == saved_before.value,
                   "unsupported lane side request mutated authoritative state");

  const auto non_carriageway = state.SetLaneSideMarkingPolicy(
      city::road::SetLaneSideMarkingPolicyRequest{
          1, 10, city::road::LaneSide::kRight,
          AutoMarkingPolicy{true, MarkingRole::kCarriagewayEdge,
                            builtin_marking_styles::kWhiteSolid}});
  ROAD_TEST_EXPECT(!non_carriageway.ok && non_carriageway.error_kind == ErrorKind::kValidation,
                   "lane side policy on a sidewalk band was not rejected");
  return true;
}

bool M3_lane_count_change_begins_and_terminates_lines(std::string& failure) {
  // The roads run along +X from the origin, so a point's x is its distance.
  {
    RoadState state{};
    const auto segment = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), 1});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransitionRequest transition{};
    transition.from_template = 1;
    transition.to_template = 2;
    transition.start = DistanceRef{DistanceRefKind::kFromStart, 20.0};
    transition.end = DistanceRef{DistanceRefKind::kFromStart, 50.0};
    transition.rules = {SectionTransitionRule{35, TransitionAction::kTaperIn}};
    const auto transition_id = state.AddTransition(transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(
        state.AttachSectionTransition(
                 city::road::AttachSectionTransitionRequest{segment.value, transition_id.value})
            .ok,
        "lane addition transition could not be attached");

    const auto* added = find_boundary_line(state, segment.value, 250);
    ROAD_TEST_EXPECT(added != nullptr, "added lane divider produced no marking line");
    ROAD_TEST_EXPECT(added->points.front().x > 20.0,
                     "added lane divider began before its lane existed");
    ROAD_TEST_EXPECT(added->points.back().x > 70.0,
                     "added lane divider did not continue to the segment end");

    const auto* kept = find_boundary_line(state, segment.value, 200);
    ROAD_TEST_EXPECT(kept != nullptr, "existing lane divider lost its marking line");
    ROAD_TEST_EXPECT(kept->points.front().x < 1e-6 && kept->points.back().x > 79.0,
                     "existing lane divider did not continue across the transition");
  }

  {
    RoadState state{};
    const auto segment = state.AddSegment(
        city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {80.0, 0.0})}), 2});
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransitionRequest transition{};
    transition.from_template = 2;
    transition.to_template = 1;
    transition.start = DistanceRef{DistanceRefKind::kFromStart, 30.0};
    transition.end = DistanceRef{DistanceRefKind::kFromStart, 60.0};
    transition.rules = {SectionTransitionRule{35, TransitionAction::kTaperOut}};
    const auto transition_id = state.AddTransition(transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(
        state.AttachSectionTransition(
                 city::road::AttachSectionTransitionRequest{segment.value, transition_id.value})
            .ok,
        "lane reduction transition could not be attached");

    const auto* removed = find_boundary_line(state, segment.value, 250);
    ROAD_TEST_EXPECT(removed != nullptr, "removed lane divider produced no marking line");
    ROAD_TEST_EXPECT(removed->points.back().x < 60.0,
                     "removed lane divider survived past its lane");
    ROAD_TEST_EXPECT(removed->points.front().x < 1e-6,
                     "removed lane divider did not start at the segment start");

    const auto* kept = find_boundary_line(state, segment.value, 200);
    ROAD_TEST_EXPECT(kept != nullptr, "existing lane divider lost its marking line");
    ROAD_TEST_EXPECT(kept->points.back().x > 79.0,
                     "existing lane divider did not continue across the reduction");
  }
  return true;
}

bool M6_transition_without_boundary_mapping_is_unsupported(std::string& failure) {
  RoadState state{};
  auto role_changed = JapaneseUrbanTwoLaneTemplate(0);
  role_changed.boundaries[1].role = BoundaryRole::kMedianEdge;
  const auto target = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{role_changed});
  ROAD_TEST_EXPECT(target.ok, target.error);
  const auto segment = state.AddSegment(
      city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1});
  ROAD_TEST_EXPECT(segment.ok, segment.error);
  SectionTransitionRequest transition{};
  transition.from_template = 1;
  transition.to_template = target.value;
  transition.start = DistanceRef{DistanceRefKind::kFromStart, 10.0};
  transition.end = DistanceRef{DistanceRefKind::kFromStart, 40.0};
  transition.rules = {SectionTransitionRule{20, TransitionAction::kContinue}};
  const auto transition_id = state.AddTransition(transition);
  ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
  const auto saved_before = state.Save();
  ROAD_TEST_EXPECT(saved_before.ok, saved_before.error);
  const auto attached = state.AttachSectionTransition(
      city::road::AttachSectionTransitionRequest{segment.value, transition_id.value});
  ROAD_TEST_EXPECT(!attached.ok && attached.error_kind == ErrorKind::kUnsupported,
                   "boundary role change was not reported as unsupported");
  const auto saved_after = state.Save();
  ROAD_TEST_EXPECT(saved_after.ok && saved_after.value == saved_before.value,
                   "unsupported transition mutated authoritative state");
  return true;
}

bool RSL_section_axes_and_shoulder_are_independent(std::string& failure) {
  const auto shoulder = ShoulderedTwoLaneTemplate(5);
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
  const auto segment = state.AddSegment(city::road::AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 5});
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

  city::road::CrossSectionTemplate continuous{};
  continuous.strips = {
      {10, StripFunction::kCarriageway, 6.0, 0.0,
       builtin_surface_styles::kMedian},
  };
  continuous.lane_bands = {
      {100, 10, 0.0, 3.0},
      {200, 10, 3.0, 6.0},
  };
  const auto template_id =
      state.AddSectionTemplate(city::road::AddSectionTemplateRequest{
          std::move(continuous)});
  ROAD_TEST_EXPECT(template_id.ok, template_id.error);
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto restored = std::find_if(
      loaded.value.graph().section_templates.begin(),
      loaded.value.graph().section_templates.end(),
      [id = template_id.value](const auto& section) {
        return section.id == id;
      });
  ROAD_TEST_EXPECT(
      restored != loaded.value.graph().section_templates.end() &&
          restored->strips.size() == 1 &&
          restored->lane_bands.size() == 2 &&
          restored->strips.front().function == StripFunction::kCarriageway &&
          restored->strips.front().style_id ==
              builtin_surface_styles::kMedian,
      "one physical strip with multiple lane bands did not round-trip");
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
      {"P0_tool_preview_includes_bezier_handles", P0_tool_preview_includes_bezier_handles},
      {"P0_straight_segments_stay_linear_after_snap_and_move", P0_straight_segments_stay_linear_after_snap_and_move},
      {"P0_edit_and_delete_preserve_graph_ownership", P0_edit_and_delete_preserve_graph_ownership},
      {"P0_multispan_segment_is_one_user_deletion_unit",
       P0_multispan_segment_is_one_user_deletion_unit},
      {"P1_degree_two_corner_uses_a_curve_without_a_junction", P1_degree_two_corner_uses_a_curve_without_a_junction},
      {"P1_corner_preserves_endpoint_section_sides", P1_corner_preserves_endpoint_section_sides},
      {"P1_straight_connection_has_no_junction_area", P1_straight_connection_has_no_junction_area},
      {"P1_segment_snap_splits_straight_road_for_t_junction", P1_segment_snap_splits_straight_road_for_t_junction},
      {"P1_segment_snap_splits_bezier_road_for_t_junction", P1_segment_snap_splits_bezier_road_for_t_junction},
      {"P1_segment_snap_splits_at_internal_bezier_span_boundary",
       P1_segment_snap_splits_at_internal_bezier_span_boundary},
      {"P1_cross_junction_accepts_opposite_approaches", P1_cross_junction_accepts_opposite_approaches},
      {"P1_incremental_skew_cross_accepts_ordered_approaches",
       P1_incremental_skew_cross_accepts_ordered_approaches},
      {"P2_section_transition_and_manual_markings", P2_section_transition_and_manual_markings},
      {"outer_lane_transition_preserves_existing_lanes",
       outer_lane_transition_preserves_existing_lanes},
      {"selected_outer_lane_branches_to_one_lane_road",
       selected_outer_lane_branches_to_one_lane_road},
      {"one_lane_road_merges_into_outer_mainline_lane",
       one_lane_road_merges_into_outer_mainline_lane},
      {"opposing_lanes_and_median_survive_one_direction_branch",
       opposing_lanes_and_median_survive_one_direction_branch},
      {"junction_movements_are_explicit_lane_connections",
       junction_movements_are_explicit_lane_connections},
      {"branch_markings_follow_explicit_boundary_paths",
       branch_markings_follow_explicit_boundary_paths},
      {"P2_supports_taper_lane_reduction_and_median_end", P2_supports_taper_lane_reduction_and_median_end},
      {"P2_requires_transition_for_mixed_section_connection", P2_requires_transition_for_mixed_section_connection},
      {"P2_marking_policy_suppression_and_junction_override", P2_marking_policy_suppression_and_junction_override},
      {"M1_lane_side_requests_share_one_boundary_line", M1_lane_side_requests_share_one_boundary_line},
      {"M1_lane_side_without_adjacent_boundary_is_unsupported",
       M1_lane_side_without_adjacent_boundary_is_unsupported},
      {"M3_lane_count_change_begins_and_terminates_lines", M3_lane_count_change_begins_and_terminates_lines},
      {"M6_transition_without_boundary_mapping_is_unsupported",
       M6_transition_without_boundary_mapping_is_unsupported},
      {"RSL_section_axes_and_shoulder_are_independent",
       RSL_section_axes_and_shoulder_are_independent},
      {"road_does_not_enter_wire_core", road_does_not_enter_wire_core},
  };
  int failed = 0;
  for (const Test& test : tests) {
    std::string failure;
    const bool ok = test.run(failure);
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << test.name << "\n";
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
