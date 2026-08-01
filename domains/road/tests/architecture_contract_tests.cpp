#include "city/road/road.hpp"

#include "derived_view.hpp"
#include "../src/generation/generation.hpp"
#include "../src/lookup.hpp"
#include "../src/persistence/road_archive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>

namespace {

#define ROAD_CONTRACT_EXPECT(condition, message) \
  do {                                           \
    if (!(condition)) {                          \
      failure = (message);                       \
      return false;                              \
    }                                            \
  } while (false)

using namespace city::road;

void append_boundary(std::ostringstream& out, const SectionBoundarySample& value) {
  out << value.boundary_id << ',' << static_cast<int>(value.role) << ',' << value.lateral_m << ','
      << value.height_m << ',' << value.marking.enabled << ','
      << static_cast<int>(value.marking.role) << ',' << value.marking.style_id.value << ';';
}

void append_approach(std::ostringstream& out, const ApproachKey& value) {
  out << value.node_id << ',' << value.segment_id << ','
      << static_cast<int>(value.endpoint_role);
}

void append_gate(std::ostringstream& out, const ConnectionGate& value) {
  append_approach(out, value.approach);
  out << ':' << value.segment_id << ',' << value.node_id << ',' << value.position.x << ',' << value.position.y << ','
      << value.position.z << ',' << value.tangent.x << ',' << value.tangent.y << ',' << value.tangent.z << ','
      << value.lateral.x << ',' << value.lateral.y << ',' << value.lateral.z << ','
      << value.normal.x << ',' << value.normal.y << ',' << value.normal.z << ':';
  for (const auto& boundary : value.boundaries) append_boundary(out, boundary);
}

void append_meshes(std::ostringstream& out, const std::vector<Mesh>& meshes) {
  out << meshes.size() << ':';
  for (const auto& mesh : meshes) {
    out << mesh.owner_segment_id << ',' << static_cast<int>(mesh.style.domain)
        << ':' << mesh.style.value << ',' << mesh.vertices.size() << ','
        << mesh.indices.size() << ':';
    for (const auto& vertex : mesh.vertices) out << vertex.x << ',' << vertex.y << ',' << vertex.z << ';';
    for (const auto index : mesh.indices) out << index << ',';
  }
}

std::string path_observation(const Path& path) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::hexfloat;
  for (const BezierSpan& span : path.spans) {
    out << span.p0.x << ',' << span.p0.y << ',' << span.p1.x << ','
        << span.p1.y << ',' << span.p2.x << ',' << span.p2.y << ','
        << span.p3.x << ',' << span.p3.y << ';';
  }
  return out.str();
}

void append_owner(std::ostringstream& out, const MarkingOwner& owner) {
  out << static_cast<int>(owner.kind) << ',' << owner.segment_id << ','
      << owner.node_id << ',' << owner.manual_id;
}

std::string derived_observation(const DerivedRoad& derived) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::hexfloat;
  out << derived.segments.size() << ':';
  for (const DerivedSegment& segment : derived.segments) {
    out << segment.id << ',' << segment.length_m << ',' << segment.surface_start_m << ','
        << segment.surface_end_m << ',' << segment.alignment.spans.size() << ':';
    for (const auto& span : segment.alignment.spans) {
      out << span.p0.x << ',' << span.p0.y << ',' << span.p1.x << ',' << span.p1.y << ','
          << span.p2.x << ',' << span.p2.y << ',' << span.p3.x << ',' << span.p3.y << ';';
    }
    for (double distance : segment.semantic_segment_distances_m) out << distance << ',';
    out << ':';
    for (double distance : segment.surface_segment_distances_m) out << distance << ',';
    out << ':' << segment.sections.size() << ':';
    for (const SectionEvaluation& section : segment.sections) {
      out << section.segment_id << ',' << section.segment_distance_m << ','
          << section.resolved_template_id << ':';
      for (const auto& boundary : section.boundaries) append_boundary(out, boundary);
      for (const auto& style : section.surface_styles) {
        out << static_cast<int>(style.domain) << ':' << style.value << ';';
      }
    }
  }
  out << derived.connections.size() << ':';
  for (const ResolvedConnection& connection : derived.connections) {
    out << connection.node_id << ',' << static_cast<int>(connection.kind) << ','
        << connection.corner_radius_m << ',' << connection.corner_control_m << ','
        << connection.junction_corner_control_m << ','
        << connection.applied_policy_override_id << ':';
    for (const ApproachKey& key : connection.ordered_approaches) {
      append_approach(out, key);
      out << ';';
    }
    for (const ResolvedApproach& approach : connection.approaches) {
      append_approach(out, approach.key);
      out << ',' << approach.endpoint_template_id << ',' << approach.auto_setback_m << ','
          << approach.resolved_setback_m << ',' << approach.resolved_lateral_shift_m << ','
          << approach.gate_segment_distance_m << ';';
      append_gate(out, approach.gate);
    }
    out << connection.connection_geometry.surface_strips.size() << ':';
    for (const ResolvedSurfaceStrip& strip : connection.connection_geometry.surface_strips) {
      out << static_cast<int>(strip.style.domain) << ':' << strip.style.value << ','
          << strip.left_boundary_id << ',' << strip.right_boundary_id << ':';
      for (const Vec3d& point : strip.left) {
        out << point.x << ',' << point.y << ',' << point.z << ';';
      }
      for (const Vec3d& point : strip.right) {
        out << point.x << ',' << point.y << ',' << point.z << ';';
      }
    }
    out << connection.junction_geometry.surface_regions.size() << ','
        << connection.junction_geometry.surface_strips.size() << ':';
    for (const ResolvedSurfaceRegion& region : connection.junction_geometry.surface_regions) {
      out << static_cast<int>(region.style.domain) << ':' << region.style.value << ':';
      for (const Vec3d& point : region.perimeter) {
        out << point.x << ',' << point.y << ',' << point.z << ';';
      }
    }
  }
  out << derived.markings.size() << ':';
  for (const DerivedMarking& marking : derived.markings) {
    append_owner(out, marking.owner);
    out << ':' << marking.boundary_id << ',' << static_cast<int>(marking.role) << ','
        << marking.style_id.value << ',' << marking.width_m << ':';
    for (const Vec3d& point : marking.points) {
      out << point.x << ',' << point.y << ',' << point.z << ';';
    }
    out << ':';
    for (const Vec3d& point : marking.polygon) {
      out << point.x << ',' << point.y << ',' << point.z << ';';
    }
  }
  out << derived.terrain_masks.size() << ':';
  for (const auto& mask : derived.terrain_masks) {
    out << mask.segment_id << ',' << mask.points.size() << ':';
    for (const auto& point : mask.points) out << point.x << ',' << point.y << ';';
  }
  append_meshes(out, derived.segment_meshes);
  append_meshes(out, derived.marking_meshes);
  append_meshes(out, derived.connection_meshes);
  append_meshes(out, derived.junction_meshes);
  return out.str();
}

std::uint64_t next_id_observation(std::string_view authoritative) {
  const std::string_view key = "next_id=";
  const std::size_t begin = authoritative.find(key);
  if (begin == std::string_view::npos) return 0;
  const std::size_t value_begin = begin + key.size();
  const std::size_t end = authoritative.find('\n', value_begin);
  return static_cast<std::uint64_t>(std::stoull(std::string(authoritative.substr(value_begin, end - value_begin))));
}

std::string query_observation(const RoadState& state) {
  std::ostringstream out;
  out << state.graph().nodes.size() << ',' << state.graph().segments.size() << ','
      << state.graph().section_templates.size() << ',' << state.graph().transitions.size() << ','
      << state.graph().connection_policy_overrides.size() << ',' << state.graph().manual_lines.size() << ','
      << state.graph().manual_areas.size() << ':';
  for (const auto& node : state.graph().nodes) out << 'n' << node.id << ';';
  for (const auto& segment : state.graph().segments) out << 's' << segment.id << ';';
  for (const auto& gate : road_test_view::gates(state.derived())) out << 'g' << gate.segment_id << ',' << gate.node_id << ';';
  return out.str();
}

struct StateObservation {
  std::string authoritative{};
  std::string derived{};
  std::uint64_t next_id = 0;
  std::string query{};

  bool operator==(const StateObservation&) const = default;
};

StateObservation observe(const RoadState& state) {
  const auto saved = state.Save();
  if (!saved.ok) return {};
  return StateObservation{saved.value, derived_observation(state.derived()), next_id_observation(saved.value),
                          query_observation(state)};
}

template <typename Operation>
bool expect_failed_unchanged(RoadState& state, Operation&& operation, std::string_view label, std::string& failure) {
  const StateObservation before = observe(state);
  const auto result = operation();
  if (result.ok) {
    failure = std::string(label) + " unexpectedly succeeded";
    return false;
  }
  if (!(observe(state) == before)) {
    failure = std::string(label) + " changed authoritative, derived, next ID, or query state after failure";
    return false;
  }
  return true;
}

bool all_public_operation_validation_failures_are_atomic(std::string& failure) {
  RoadState state{};
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(state, [&] { return state.AddSegment(city::road::AddSegmentRequest{{}, 1}); }, "AddSegment", failure),
                       failure);
  const Path line = MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})});
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state,
                           [&] {
                              return state.ExtendCorridorFromEnd(
                                  ExtendCorridorFromEndRequest{999999, 999998,
                                                               line, 1});
                           },
                           "ExtendCorridorFromEnd", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{line, 1, 999999}); },
                           "AddSegmentConnectedTo", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{line, 1, 999999, 10.0}); },
                           "AddSegmentConnectedToSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          state,
          [&] {
            return state.SplitSegmentAtDistance(
                SplitSegmentAtDistanceRequest{999999, 10.0});
          },
          "SplitSegmentAtDistance", failure),
      failure);
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          state,
          [&] {
            return state.DeleteSegmentRange(
                DeleteSegmentRangeRequest{999999, 1.0, 2.0});
          },
          "DeleteSegmentRange", failure),
      failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.DeleteSegment(city::road::DeleteSegmentRequest{999999}); }, "DeleteSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.EditSegmentShape(city::road::EditSegmentShapeRequest{999999, {}}); }, "EditSegmentShape", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.MoveNode(city::road::MoveNodeRequest{999999, {0.0, 0.0}}); }, "MoveNode", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSectionTemplate(city::road::AddSectionTemplateRequest{{}}); }, "AddSectionTemplate", failure),
                       failure);
  CrossSectionTemplate missing = JapaneseUrbanTwoLaneTemplate(999999);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.EditSectionTemplate(city::road::EditSectionTemplateRequest{missing}); }, "EditSectionTemplate", failure),
                       failure);
  SectionTransitionRequest invalid_transition{};
  invalid_transition.from_template = 1;
  invalid_transition.to_template = 999999;
  invalid_transition.start = {DistanceRefKind::kFromStart, 1.0};
  invalid_transition.end = {DistanceRefKind::kFromStart, 2.0};
  invalid_transition.rules = {{10, TransitionAction::kContinue}};
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddTransition(invalid_transition); }, "AddTransition", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddTransitionToSegment(city::road::AddTransitionToSegmentRequest{999999, invalid_transition}); },
                           "AddTransitionToSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AttachSectionTransition(city::road::AttachSectionTransitionRequest{999999, 999998}); },
                           "AttachSectionTransition", failure), failure);
  ManualLineRequest line_marking{};
  line_marking.owner_segment_id = 999999;
  line_marking.path = line;
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddManualLine(line_marking); }, "AddManualLine", failure),
                       failure);
  ManualAreaRequest area_marking{};
  area_marking.owner_segment_id = 999999;
  area_marking.width_m = 1.0;
  area_marking.length_m = 1.0;
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddManualArea(area_marking); }, "AddManualArea", failure),
                       failure);
  return true;
}

bool confirmation_boundaries_define_segment_units(std::string& failure) {
  const Path first =
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})});
  const Path second =
      MakePath({MakeLine({20.0, 0.0}, {23.0, 4.0})});
  const Path complete =
      MakePath({first.spans.front(), second.spans.front()});

  RoadState one_operation{};
  const auto complete_added =
      one_operation.AddSegment(AddSegmentRequest{complete, 1});
  ROAD_CONTRACT_EXPECT(complete_added.ok, complete_added.error);

  RoadState incremental{};
  const auto first_added =
      incremental.AddSegment(AddSegmentRequest{first, 1});
  ROAD_CONTRACT_EXPECT(first_added.ok, first_added.error);
  const RoadSegment& segment = incremental.graph().segments.front();
  const RoadCorridor* corridor =
      FindCorridorForSegment(incremental.graph(), segment.id);
  ROAD_CONTRACT_EXPECT(corridor != nullptr,
                       "initial segment has no road corridor");
  const auto extended = incremental.ExtendCorridorFromEnd(
      ExtendCorridorFromEndRequest{corridor->id, segment.node_b, second, 1});
  ROAD_CONTRACT_EXPECT(extended.ok, extended.error);
  ROAD_CONTRACT_EXPECT(extended.value != first_added.value,
                       "extension reused the existing segment identity");
  ROAD_CONTRACT_EXPECT(incremental.graph().segments.size() == 2 &&
                           incremental.graph().nodes.size() == 3 &&
                           incremental.graph().corridors.size() == 1,
                       "extension did not create a local segment chain");
  ROAD_CONTRACT_EXPECT(
      one_operation.graph().segments.size() == 1 &&
          one_operation.graph().nodes.size() == 2 &&
          one_operation.graph().segments.front().shape.internal_knots.size() == 1,
      "one confirmed multi-span path was not retained as one user segment");
  ROAD_CONTRACT_EXPECT(
      incremental.graph().segments.size() == 2 &&
          incremental.graph().nodes.size() == 3,
      "a later confirmed extension did not create a new user segment");

  const auto before_first = observe(incremental);
  const Path prepend =
      MakePath({MakeLine({0.0, 0.0}, {-12.0, 16.0})});
  const auto prepended = incremental.ExtendCorridorFromEnd(
      ExtendCorridorFromEndRequest{corridor->id, segment.node_a, prepend, 1});
  ROAD_CONTRACT_EXPECT(!prepended.ok && observe(incremental) == before_first,
                       "corridor start extension was not atomic unsupported");
  return true;
}

bool reverse_input_has_equivalent_geometry(std::string& failure) {
  const Path forward_path = MakePath({
      MakeLine({0.0, 0.0}, {20.0, 0.0}),
      MakeLine({20.0, 0.0}, {32.0, 16.0}),
  });
  Path reverse_path{};
  for (auto it = forward_path.spans.rbegin();
       it != forward_path.spans.rend(); ++it) {
    reverse_path.spans.push_back(
        BezierSpan{it->p3, it->p2, it->p1, it->p0});
  }
  RoadState forward{};
  RoadState reverse{};
  const auto forward_added =
      forward.AddSegment(AddSegmentRequest{forward_path, 1});
  const auto reverse_added =
      reverse.AddSegment(AddSegmentRequest{reverse_path, 1});
  ROAD_CONTRACT_EXPECT(forward_added.ok && reverse_added.ok,
                       "reverse geometry setup failed");
  ROAD_CONTRACT_EXPECT(forward.graph().corridors.size() == 1 &&
                           reverse.graph().corridors.size() == 1,
                       "reverse corridor is missing");
  const RoadCorridor& forward_corridor = forward.graph().corridors.front();
  const RoadCorridor& reverse_corridor = reverse.graph().corridors.front();
  ROAD_CONTRACT_EXPECT(forward_corridor.segments.size() ==
                           reverse_corridor.segments.size(),
                       "reverse corridor segment count differs");
  const auto close = [](Vec2d a, Vec2d b) {
    return std::abs(a.x - b.x) <= 1e-9 &&
           std::abs(a.y - b.y) <= 1e-9;
  };
  const Path* forward_alignment = FindCanonicalAlignment(
      forward.derived(), forward_corridor.segments.front().segment_id);
  const Path* reverse_alignment = FindCanonicalAlignment(
      reverse.derived(), reverse_corridor.segments.front().segment_id);
  ROAD_CONTRACT_EXPECT(forward_alignment != nullptr &&
                           reverse_alignment != nullptr &&
                           forward_alignment->spans.size() ==
                               reverse_alignment->spans.size(),
                       "reverse canonical alignment is missing");
  for (std::size_t index = 0; index < forward_alignment->spans.size();
       ++index) {
    const BezierSpan& a = forward_alignment->spans[index];
    const BezierSpan& b =
        reverse_alignment->spans[reverse_alignment->spans.size() - 1 - index];
    ROAD_CONTRACT_EXPECT(
        close(a.p0, b.p3) && close(a.p1, b.p2) &&
            close(a.p2, b.p1) && close(a.p3, b.p0),
        "reverse canonical Bezier geometry differs");
  }
  const auto vertex_signature = [](const std::vector<Mesh>& meshes) {
    std::vector<std::string> signature{};
    for (const Mesh& mesh : meshes) {
      std::vector<std::array<long long, 3>> vertices{};
      vertices.reserve(mesh.vertices.size());
      for (const Vec3d& vertex : mesh.vertices) {
        vertices.push_back({
            std::llround(vertex.x * 1e6),
            std::llround(vertex.y * 1e6),
            std::llround(vertex.z * 1e6),
        });
      }
      std::sort(vertices.begin(), vertices.end());
      std::ostringstream row;
      row << static_cast<int>(mesh.style.domain) << ':' << mesh.style.value << ':';
      for (const auto& vertex : vertices) {
        row << vertex[0] << ',' << vertex[1] << ',' << vertex[2] << ';';
      }
      signature.push_back(row.str());
    }
    std::sort(signature.begin(), signature.end());
    return signature;
  };
  ROAD_CONTRACT_EXPECT(
      vertex_signature(forward.derived().segment_meshes) ==
          vertex_signature(reverse.derived().segment_meshes),
      "reverse surface mesh geometry differs");
  const auto equivalent_vertices = [](const std::vector<Mesh>& a,
                                      const std::vector<Mesh>& b,
                                      std::string& diagnostic) {
    if (a.size() != b.size()) {
      diagnostic = "mesh count";
      return false;
    }
    for (const Mesh& source : a) {
      const auto target = std::find_if(
          b.begin(), b.end(), [&source](const Mesh& candidate) {
            return candidate.style == source.style &&
                   candidate.vertices.size() == source.vertices.size();
          });
      if (target == b.end()) {
        diagnostic = "style or vertex count";
        return false;
      }
      std::vector<bool> matched(target->vertices.size(), false);
      for (const Vec3d& vertex : source.vertices) {
        bool found = false;
        for (std::size_t index = 0; index < target->vertices.size(); ++index) {
          if (matched[index]) continue;
          const Vec3d& candidate = target->vertices[index];
          if (std::abs(vertex.x - candidate.x) <= 1e-6 &&
              std::abs(vertex.y - candidate.y) <= 1e-6 &&
              std::abs(vertex.z - candidate.z) <= 1e-6) {
            matched[index] = true;
            found = true;
            break;
          }
        }
        if (!found) {
          double nearest = std::numeric_limits<double>::infinity();
          for (const Vec3d& candidate : target->vertices) {
            nearest = std::min(
                nearest,
                std::hypot(vertex.x - candidate.x,
                           vertex.y - candidate.y));
          }
          diagnostic = "unmatched " + std::to_string(vertex.x) + "," +
                       std::to_string(vertex.y) + " nearest=" +
                       std::to_string(nearest);
          return false;
        }
      }
    }
    return true;
  };
  const auto marking_centers = [](const std::vector<DerivedMarking>& markings) {
    std::vector<Mesh> meshes{};
    for (const DerivedMarking& marking : markings) {
      Mesh mesh{};
      mesh.style = RenderStyleFromMarking(marking.style_id);
      mesh.vertices = marking.points;
      mesh.vertices.insert(mesh.vertices.end(), marking.polygon.begin(),
                           marking.polygon.end());
      meshes.push_back(std::move(mesh));
    }
    return meshes;
  };
  const std::vector<Mesh> forward_markings =
      marking_centers(forward.derived().markings);
  const std::vector<Mesh> reverse_markings =
      marking_centers(reverse.derived().markings);
  ROAD_CONTRACT_EXPECT(
      vertex_signature(forward_markings) == vertex_signature(reverse_markings),
      "reverse marking geometry differs");
  return true;
}

bool extension_semantic_boundaries_are_atomic(std::string& failure) {
  const Path base_path =
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})});
  const Path extension =
      MakePath({MakeLine({20.0, 0.0}, {32.0, 16.0})});

  RoadState connected{};
  const auto connected_base =
      connected.AddSegment(AddSegmentRequest{base_path, 1});
  ROAD_CONTRACT_EXPECT(connected_base.ok, connected_base.error);
  const RoadSegment connected_segment = connected.graph().segments.front();
  const auto branch = connected.AddSegmentConnectedTo(
      AddSegmentConnectedToRequest{extension, 1, connected_segment.node_b});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          connected,
          [&] {
            const RoadCorridor* corridor =
                FindCorridorForSegment(connected.graph(),
                                       connected_segment.id);
            return connected.ExtendCorridorFromEnd(
                ExtendCorridorFromEndRequest{
                    corridor == nullptr ? 0 : corridor->id,
                    connected_segment.node_b, extension, 1});
          },
          "degree-two ExtendCorridorFromEnd", failure),
      failure);

  RoadState mixed_section{};
  const auto section =
      mixed_section.AddSectionTemplate(
          AddSectionTemplateRequest{ThreeLaneTemplate(0)});
  ROAD_CONTRACT_EXPECT(section.ok, section.error);
  const auto mixed_base =
      mixed_section.AddSegment(AddSegmentRequest{base_path, 1});
  ROAD_CONTRACT_EXPECT(mixed_base.ok, mixed_base.error);
  const RoadSegment mixed_segment = mixed_section.graph().segments.front();
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          mixed_section,
          [&] {
            const RoadCorridor* corridor =
                FindCorridorForSegment(mixed_section.graph(),
                                       mixed_segment.id);
            return mixed_section.ExtendCorridorFromEnd(
                ExtendCorridorFromEndRequest{
                    corridor == nullptr ? 0 : corridor->id,
                    mixed_segment.node_b, extension, section.value});
          },
          "mixed-section ExtendCorridorFromEnd", failure),
      failure);

  RoadState marked{};
  const auto marked_base =
      marked.AddSegment(AddSegmentRequest{base_path, 1});
  ROAD_CONTRACT_EXPECT(marked_base.ok, marked_base.error);
  const auto marking = marked.AddManualArea(
      ManualAreaRequest{marked_base.value, {10.0, 0.0}, 0.0, 2.0, 2.0,
                        builtin_marking_styles::kCrosswalk});
  ROAD_CONTRACT_EXPECT(marking.ok, marking.error);
  const RoadSegment marked_segment = marked.graph().segments.front();
  const Path prepend =
      MakePath({MakeLine({0.0, 0.0}, {-12.0, 16.0})});
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          marked,
          [&] {
            const RoadCorridor* corridor =
                FindCorridorForSegment(marked.graph(), marked_segment.id);
            return marked.ExtendCorridorFromEnd(
                ExtendCorridorFromEndRequest{
                    corridor == nullptr ? 0 : corridor->id,
                    marked_segment.node_a, prepend, 1});
          },
          "corridor-start ExtendCorridorFromEnd", failure),
      failure);
  return true;
}

bool isolated_segment_uses_simple_path(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {30.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  ROAD_CONTRACT_EXPECT(state.derived().connections.empty(),
                       "isolated segment created a connection entity");
  ROAD_CONTRACT_EXPECT(road_test_view::gates(state.derived()).empty(),
                       "isolated segment created connection gates");
  ROAD_CONTRACT_EXPECT(state.derived().connection_meshes.empty() &&
                           state.derived().junction_meshes.empty(),
                       "isolated segment entered connection geometry");
  return true;
}

bool approach_override_resolves_and_persists_manual_fields(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {32.0, 24.0})}), 1, base.value, 20.0});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);
  ROAD_CONTRACT_EXPECT(!state.derived().connections.empty(),
                       "T branch did not derive a resolved connection");
  const ResolvedConnection& connection = state.derived().connections.front();
  const ResolvedApproach& original = connection.approaches.front();
  const ApproachKey key = original.key;
  const double manual_setback = original.resolved_setback_m + 0.5;
  const auto set_setback = state.SetApproachSetbackOverride(
      SetApproachSetbackOverrideRequest{key, manual_setback});
  ROAD_CONTRACT_EXPECT(set_setback.ok, set_setback.error);
  const auto set_shift = state.SetApproachLateralShiftOverride(
      SetApproachLateralShiftOverrideRequest{key, 0.75});
  ROAD_CONTRACT_EXPECT(set_shift.ok, set_shift.error);
  ROAD_CONTRACT_EXPECT(state.graph().approach_geometry_overrides.size() == 1,
                       "manual approach override was not saved as one authoritative row");
  const ResolvedApproach* resolved = FindResolvedApproach(state.derived(), key);
  ROAD_CONTRACT_EXPECT(resolved != nullptr, "manual override lost the resolved approach");
  ROAD_CONTRACT_EXPECT(std::abs(resolved->resolved_setback_m - manual_setback) < 1e-9 &&
                           std::abs(resolved->resolved_lateral_shift_m - 0.75) < 1e-9,
                       "manual override was not consumed by the resolved approach");
  ROAD_CONTRACT_EXPECT(std::abs(resolved->auto_setback_m - (manual_setback - 0.5)) < 1e-9 &&
                           std::abs(resolved->auto_lateral_shift_m) < 1e-9,
                       "resolved approach lost the automatic values it overrode");
  ROAD_CONTRACT_EXPECT(resolved->gate.approach == key &&
                           resolved->gate.position.x == resolved->position.x &&
                           resolved->gate.position.y == resolved->position.y &&
                           resolved->gate.position.z == resolved->position.z,
                       "connection gate did not follow the resolved approach");
  const auto saved = state.Save();
  ROAD_CONTRACT_EXPECT(saved.ok, saved.error);
  ROAD_CONTRACT_EXPECT(saved.value.find("approach_geometry_override.count=1\n") != std::string::npos &&
                           saved.value.find(".setback.value=") != std::string::npos &&
                           saved.value.find(".lateral_shift.value=") != std::string::npos,
                       "manual approach override fields were not persisted");
  const auto loaded = RoadState::Load(saved.value);
  ROAD_CONTRACT_EXPECT(loaded.ok, loaded.error);
  ROAD_CONTRACT_EXPECT(loaded.value.graph().approach_geometry_overrides.size() == 1,
                       "manual approach override did not survive load");
  const auto reset_field = state.ResetApproachOverrideField(
      ResetApproachOverrideFieldRequest{key, ApproachOverrideField::kSetback});
  ROAD_CONTRACT_EXPECT(reset_field.ok, reset_field.error);
  ROAD_CONTRACT_EXPECT(state.graph().approach_geometry_overrides.size() == 1 &&
                           !state.graph().approach_geometry_overrides.front().setback_m.has_value,
                       "reset field did not remove only setback override");
  const auto reset_all = state.ResetAllApproachOverrides(ResetAllApproachOverridesRequest{key});
  ROAD_CONTRACT_EXPECT(reset_all.ok, reset_all.error);
  ROAD_CONTRACT_EXPECT(state.graph().approach_geometry_overrides.empty(),
                       "reset all did not remove empty override entity");
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state,
                           [&] {
                             return state.SetApproachSetbackOverride(
                                 SetApproachSetbackOverrideRequest{key, -1.0});
                           },
                           "negative setback override", failure),
                       failure);
  return true;
}

bool add_segment_build_failure_is_atomic(std::string& failure) {
  RoadState state{};
  CrossSectionTemplate unusable{};
  unusable.strips = {
      SectionStrip{10, StripFunction::kSidewalk, 1.0e308, 0.0, builtin_surface_styles::kSidewalk},
      SectionStrip{20, StripFunction::kCarriageway, 1.0e308, 0.0, builtin_surface_styles::kAsphalt},
      SectionStrip{30, StripFunction::kSidewalk, 1.0e308, 0.0, builtin_surface_styles::kSidewalk},
  };
  unusable.boundaries = {
      BoundaryProfile{11, BoundaryRole::kCurb, 0.0, 0.15, {}},
      BoundaryProfile{21, BoundaryRole::kCurb, 0.0, -0.15, {}},
  };
  const auto template_id = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{std::move(unusable)});
  ROAD_CONTRACT_EXPECT(template_id.ok, template_id.error);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state,
                           [&] {
                             return state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}),
                                                     template_id.value});
                           },
                           "AddSegment build failure", failure),
                       failure);
  return true;
}

bool node_identity_does_not_come_from_position(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  const auto second = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {0.0, 20.0})}), 1});
  const auto near = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0000001}, {-20.0, 0.0000001})}), 1});
  ROAD_CONTRACT_EXPECT(first.ok && second.ok && near.ok, "independent segments could not be created");
  ROAD_CONTRACT_EXPECT(state.graph().nodes.size() == 6, "equal or near node positions were merged by geometry");
  ROAD_CONTRACT_EXPECT(state.graph().segments[0].node_a != state.graph().segments[1].node_a &&
                           state.graph().segments[0].node_a != state.graph().segments[2].node_a,
                       "node identity was inferred from equal or epsilon-near positions");
  return true;
}

bool segment_shape_edit_does_not_move_endpoint_authority(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const RoadNode before_a = state.graph().nodes[0];
  const RoadNode before_b = state.graph().nodes[1];
  const auto shape = SegmentShapeFromPath(
      MakePath({MakeBezier({3.0, 4.0}, {7.0, 10.0}, {16.0, -5.0}, {24.0, 2.0})}));
  ROAD_CONTRACT_EXPECT(shape.ok, shape.error);
  const auto edited = state.EditSegmentShape(city::road::EditSegmentShapeRequest{added.value, shape.value});
  ROAD_CONTRACT_EXPECT(edited.ok, edited.error);
  ROAD_CONTRACT_EXPECT(state.graph().nodes[0].id == before_a.id && state.graph().nodes[1].id == before_b.id &&
                           state.graph().nodes[0].position.x == before_a.position.x &&
                           state.graph().nodes[0].position.y == before_a.position.y &&
                           state.graph().nodes[1].position.x == before_b.position.x &&
                           state.graph().nodes[1].position.y == before_b.position.y,
                       "segment shape edit changed RoadNode endpoint authority");
  return true;
}

bool move_node_rederives_incident_alignment_endpoints(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(first.ok, first.error);
  const RoadNodeId shared = state.graph().segments.front().node_b;
  const auto second = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), 1, shared});
  ROAD_CONTRACT_EXPECT(second.ok, second.error);
  const auto moved = state.MoveNode(city::road::MoveNodeRequest{shared, {20.0, 15.0}});
  ROAD_CONTRACT_EXPECT(moved.ok, moved.error);
  const Path* first_path = FindCanonicalAlignment(state.derived(), first.value);
  const Path* second_path = FindCanonicalAlignment(state.derived(), second.value);
  ROAD_CONTRACT_EXPECT(first_path != nullptr && second_path != nullptr, "incident canonical alignment is missing");
  ROAD_CONTRACT_EXPECT(first_path->spans.back().p3.x == 20.0 && first_path->spans.back().p3.y == 15.0 &&
                           second_path->spans.front().p0.x == 20.0 && second_path->spans.front().p0.y == 15.0,
                       "MoveNode did not rederive every incident canonical endpoint");
  ROAD_CONTRACT_EXPECT(IsLinearSpan(first_path->spans.back()) &&
                           IsLinearSpan(second_path->spans.front()),
                       "MoveNode did not preserve straight segment intent");
  ROAD_CONTRACT_EXPECT(state.graph().segments[0].node_b == shared && state.graph().segments[1].node_a == shared,
                       "MoveNode changed segment connectivity");
  return true;
}

bool generate_road_is_pure(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);

  const SavedRoadGraph graph = state.graph();
  const auto state_saved = state.Save();
  ROAD_CONTRACT_EXPECT(state_saved.ok, state_saved.error);
  const std::uint64_t graph_next_id = next_id_observation(state_saved.value);
  const auto before = persistence::SaveRoad(graph, graph_next_id);
  ROAD_CONTRACT_EXPECT(before.ok, before.error);
  const auto first = generation::generate_road(graph);
  const auto second = generation::generate_road(graph);
  ROAD_CONTRACT_EXPECT(first.ok && second.ok, "generate_road failed for a valid graph");
  ROAD_CONTRACT_EXPECT(derived_observation(first.value) == derived_observation(second.value),
                       "generate_road is not deterministic for the same graph");
  const auto after = persistence::SaveRoad(graph, graph_next_id);
  ROAD_CONTRACT_EXPECT(after.ok && after.value == before.value,
                       "generate_road changed its authoritative input");

  SavedRoadGraph unsupported = RoadState{}.graph();
  unsupported.nodes = {RoadNode{10, {0.0, 0.0}}, RoadNode{11, {20.0, 0.0}},
                       RoadNode{12, {40.0, 0.0}}};
  unsupported.segments = {
      RoadSegment{20, 10, 11, {}, 1, std::nullopt},
      RoadSegment{21, 11, 12, {}, 2, std::nullopt},
  };
  unsupported.corridors = {
      RoadCorridor{22, 1, {DirectedSegmentRef{20, false}}},
      RoadCorridor{23, 2, {DirectedSegmentRef{21, false}}},
  };
  const auto unsupported_before = persistence::SaveRoad(unsupported, 24);
  ROAD_CONTRACT_EXPECT(unsupported_before.ok, unsupported_before.error);
  const auto failed = generation::generate_road(unsupported);
  ROAD_CONTRACT_EXPECT(!failed.ok, "generate_road unexpectedly accepted an unsupported graph");
  const auto unsupported_after = persistence::SaveRoad(unsupported, 24);
  ROAD_CONTRACT_EXPECT(unsupported_after.ok && unsupported_after.value == unsupported_before.value,
                       "failed generate_road changed its authoritative input");
  return true;
}
bool generate_is_deterministic(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {32.0, 24.0})}), 1, base.value, 20.0});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);
  const std::string first = derived_observation(state.derived());
  const auto saved = state.Save();
  ROAD_CONTRACT_EXPECT(saved.ok, saved.error);
  const auto reloaded = RoadState::Load(saved.value);
  ROAD_CONTRACT_EXPECT(reloaded.ok, reloaded.error);
  ROAD_CONTRACT_EXPECT(derived_observation(reloaded.value.derived()) == first,
                       "generate from the same authoritative state was not deterministic");
  return true;
}

bool same_boundaries(const std::vector<SectionBoundarySample>& a,
                     const std::vector<SectionBoundarySample>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t index = 0; index < a.size(); ++index) {
    if (a[index].boundary_id != b[index].boundary_id ||
        a[index].role != b[index].role ||
        a[index].lateral_m != b[index].lateral_m ||
        a[index].height_m != b[index].height_m ||
        a[index].marking != b[index].marking) {
      return false;
    }
  }
  return true;
}

bool connection_section_and_gate_have_single_owners(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {32.0, 24.0})}), 1, base.value, 20.0});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);

  const DerivedRoad& derived = state.derived();
  for (const ResolvedConnection& connection : derived.connections) {
    ROAD_CONTRACT_EXPECT(connection.approaches.size() == connection.ordered_approaches.size(),
                         "resolved approaches and deterministic order differ in size");
    for (const ResolvedApproach& approach : connection.approaches) {
      ROAD_CONTRACT_EXPECT(
          std::count_if(connection.approaches.begin(), connection.approaches.end(),
                        [&approach](const ResolvedApproach& candidate) {
                          return candidate.key == approach.key;
                        }) == 1,
          "ApproachKey has more than one resolved row");
    }
  }
  for (const ResolvedConnection& connection : derived.connections) {
    for (const ResolvedApproach& approach : connection.approaches) {
      const DerivedSegment* segment = FindDerivedSegment(derived, approach.key.segment_id);
      ROAD_CONTRACT_EXPECT(segment != nullptr, "approach segment is missing");
      ROAD_CONTRACT_EXPECT(
          std::count(segment->semantic_segment_distances_m.begin(),
                     segment->semantic_segment_distances_m.end(),
                     approach.gate_segment_distance_m) == 1,
          "gate distance is not a unique semantic distance");
      ROAD_CONTRACT_EXPECT(
          std::count_if(segment->sections.begin(), segment->sections.end(),
                        [&approach](const SectionEvaluation& section) {
                          return section.segment_distance_m == approach.gate_segment_distance_m;
                        }) == 1,
          "gate distance has no unique section evaluation");
      const SectionEvaluation* section = FindSectionAt(*segment, approach.gate_segment_distance_m);
      ROAD_CONTRACT_EXPECT(section != nullptr, "gate section evaluation is missing");
      ROAD_CONTRACT_EXPECT(same_boundaries(approach.gate.boundaries, section->boundaries),
                           "gate boundaries differ from their section evaluation");
    }
  }
  return true;
}

std::vector<ApproachKey> sorted_gate_keys(const DerivedRoad& derived) {
  std::vector<ApproachKey> keys{};
  for (const ConnectionGate& gate : road_test_view::gates(derived)) keys.push_back(gate.approach);
  std::sort(keys.begin(), keys.end(), [](const ApproachKey& a, const ApproachKey& b) {
    if (a.node_id != b.node_id) return a.node_id < b.node_id;
    if (a.segment_id != b.segment_id) return a.segment_id < b.segment_id;
    return static_cast<int>(a.endpoint_role) < static_cast<int>(b.endpoint_role);
  });
  return keys;
}

bool approach_identity_survives_geometry_changes(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(first.ok, first.error);
  const RoadNodeId shared = state.graph().segments.front().node_b;
  const auto second = state.AddSegmentConnectedTo(
      AddSegmentConnectedToRequest{
          MakePath({MakeLine({20.0, 0.0}, {32.0, 16.0})}), 1, shared});
  ROAD_CONTRACT_EXPECT(second.ok, second.error);
  const std::vector<ApproachKey> before = sorted_gate_keys(state.derived());
  ROAD_CONTRACT_EXPECT(before.size() == 2 && before[0] != before[1],
                       "connected approaches collapsed to one identity");

  const RoadSegment& segment = state.graph().segments.front();
  const auto moved = state.MoveNode(MoveNodeRequest{segment.node_a, {-1.0, 1.0}});
  ROAD_CONTRACT_EXPECT(moved.ok, moved.error);
  ROAD_CONTRACT_EXPECT(sorted_gate_keys(state.derived()) == before,
                       "MoveNode changed ApproachKey identity");
  const Path* alignment = FindCanonicalAlignment(state.derived(), first.value);
  ROAD_CONTRACT_EXPECT(alignment != nullptr, "edited approach alignment is missing");
  const auto shape = SegmentShapeFromPath(*alignment);
  ROAD_CONTRACT_EXPECT(shape.ok, shape.error);
  SegmentShape edited = shape.value;
  edited.start_handle.y += 0.25;
  const auto changed = state.EditSegmentShape(EditSegmentShapeRequest{first.value, edited});
  ROAD_CONTRACT_EXPECT(changed.ok, changed.error);
  ROAD_CONTRACT_EXPECT(sorted_gate_keys(state.derived()) == before,
                       "EditSegmentShape changed ApproachKey identity");
  return true;
}

bool same_angle_approaches_use_id_tie_break(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({-20.0, 0.0}, {0.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(base.ok, base.error);
  const RoadNodeId shared = state.graph().segments.front().node_b;
  const auto upper = state.AddSegmentConnectedTo(AddSegmentConnectedToRequest{
      MakePath({MakeBezier({0.0, 0.0}, {5.0, 0.0}, {15.0, 10.0}, {20.0, 20.0})}),
      1, shared});
  ROAD_CONTRACT_EXPECT(upper.ok, upper.error);
  const auto lower = state.AddSegmentConnectedTo(AddSegmentConnectedToRequest{
      MakePath({MakeBezier({0.0, 0.0}, {5.0, 0.0}, {15.0, -10.0}, {20.0, -20.0})}),
      1, shared});
  ROAD_CONTRACT_EXPECT(lower.ok, lower.error);
  const ResolvedConnection* decision = FindResolvedConnection(state.derived(), shared);
  ROAD_CONTRACT_EXPECT(decision != nullptr && decision->ordered_approaches.size() == 3,
                       "same-angle junction decision is missing");
  const auto upper_position =
      std::find(decision->ordered_approaches.begin(), decision->ordered_approaches.end(),
                ApproachKey{shared, upper.value, EndpointRole::kStart});
  const auto lower_position =
      std::find(decision->ordered_approaches.begin(), decision->ordered_approaches.end(),
                ApproachKey{shared, lower.value, EndpointRole::kStart});
  ROAD_CONTRACT_EXPECT(upper_position != decision->ordered_approaches.end() &&
                           lower_position != decision->ordered_approaches.end(),
                       "same-angle approaches are absent from the order");
  ROAD_CONTRACT_EXPECT((upper.value < lower.value) == (upper_position < lower_position),
                       "same-angle approaches are not tie-broken by stable ID");
  const auto saved = state.Save();
  const auto loaded = saved.ok ? RoadState::Load(saved.value) : Result<RoadState>{};
  ROAD_CONTRACT_EXPECT(loaded.ok, "same-angle state did not round-trip");
  const ResolvedConnection* loaded_decision =
      FindResolvedConnection(loaded.value.derived(), shared);
  ROAD_CONTRACT_EXPECT(loaded_decision != nullptr &&
                           loaded_decision->ordered_approaches ==
                               decision->ordered_approaches,
                       "same-angle approach order is not deterministic");

  std::istringstream archive(saved.value);
  std::vector<std::string> prefix{};
  std::vector<std::vector<std::string>> segment_blocks{};
  std::vector<std::string> suffix{};
  std::string line;
  bool in_segments = false;
  bool after_segments = false;
  while (std::getline(archive, line)) {
    if (line.starts_with("segment=")) {
      in_segments = true;
      segment_blocks.push_back({line});
    } else if (in_segments &&
               (line.starts_with("segment_shape=") ||
                line.starts_with("segment_knot="))) {
      segment_blocks.back().push_back(line);
    } else if (!in_segments) {
      prefix.push_back(line);
    } else {
      after_segments = true;
      suffix.push_back(line);
    }
    if (after_segments) in_segments = true;
  }
  std::ostringstream reordered_archive;
  for (const std::string& value : prefix) reordered_archive << value << '\n';
  for (auto block = segment_blocks.rbegin(); block != segment_blocks.rend(); ++block) {
    for (const std::string& value : *block) reordered_archive << value << '\n';
  }
  for (const std::string& value : suffix) reordered_archive << value << '\n';
  const auto reordered = RoadState::Load(reordered_archive.str());
  ROAD_CONTRACT_EXPECT(reordered.ok, "reordered segment archive did not load");
  const ResolvedConnection* reordered_decision =
      FindResolvedConnection(reordered.value.derived(), shared);
  ROAD_CONTRACT_EXPECT(reordered_decision != nullptr &&
                           reordered_decision->ordered_approaches ==
                               decision->ordered_approaches,
                       "segment storage order changed deterministic approach order");
  return true;
}

bool unsupported_junction_section_is_atomic(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({-20.0, 0.0}, {0.0, 0.0})}), 3});
  ROAD_CONTRACT_EXPECT(base.ok, base.error);
  const RoadNodeId shared = state.graph().segments.front().node_b;
  const auto first = state.AddSegmentConnectedTo(AddSegmentConnectedToRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 20.0})}), 3, shared});
  ROAD_CONTRACT_EXPECT(first.ok, first.error);
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          state,
          [&] {
            return state.AddSegmentConnectedTo(AddSegmentConnectedToRequest{
                MakePath({MakeLine({0.0, 0.0}, {20.0, -20.0})}), 3, shared});
          },
          "unsupported junction section", failure),
      failure);
  return true;
}

bool derived_segment_owns_all_semantic_distances(std::string& failure) {
  RoadState state{};
  const Path alignment = MakePath({
      MakeLine({0.0, 0.0}, {20.0, 0.0}),
      MakeLine({20.0, 0.0}, {50.0, 0.0}),
  });
  const auto segment = state.AddSegment(AddSegmentRequest{alignment, 1});
  ROAD_CONTRACT_EXPECT(segment.ok, segment.error);
  SectionTransitionRequest transition{};
  transition.to_template = 2;
  transition.start = DistanceRef{DistanceRefKind::kFromStart, 5.0};
  transition.end = DistanceRef{DistanceRefKind::kFromStart, 20.0};
  transition.rules = {
      SectionTransitionRule{35, TransitionAction::kTaperIn},
  };
  const auto attached = state.AddTransitionToSegment(
      AddTransitionToSegmentRequest{segment.value, transition});
  ROAD_CONTRACT_EXPECT(attached.ok, attached.error);
  const DerivedSegment* derived_segment =
      FindDerivedSegment(state.derived(), segment.value);
  ROAD_CONTRACT_EXPECT(derived_segment != nullptr, "derived segment is missing");
  for (const double distance : std::array<double, 4>{0.0, 5.0, 20.0, 50.0}) {
    ROAD_CONTRACT_EXPECT(
        std::count(derived_segment->semantic_segment_distances_m.begin(),
                   derived_segment->semantic_segment_distances_m.end(), distance) == 1,
        "required endpoint/span/transition semantic distance is missing or duplicated");
    ROAD_CONTRACT_EXPECT(
        std::count_if(derived_segment->sections.begin(),
                      derived_segment->sections.end(),
                      [distance](const SectionEvaluation& section) {
                        return section.segment_distance_m == distance;
                      }) == 1,
        "semantic distance has no unique section evaluation");
  }
  ROAD_CONTRACT_EXPECT(
      std::is_sorted(derived_segment->semantic_segment_distances_m.begin(),
                     derived_segment->semantic_segment_distances_m.end()),
      "semantic distances are not sorted");
  return true;
}

bool corridor_distance_and_periodic_placement_cross_segment_boundaries(
    std::string& failure) {
  RoadState state{};
  const Path path = MakePath({
      MakeLine({0.0, 0.0}, {15.0, 0.0}),
  });
  const auto added = state.AddSegment(AddSegmentRequest{path, 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const RoadSegment first_segment = state.graph().segments.front();
  const RoadCorridorId corridor_id = state.graph().corridors.front().id;
  const auto extended = state.ExtendCorridorFromEnd(
      ExtendCorridorFromEndRequest{
          corridor_id, first_segment.node_b,
          MakePath({MakeLine({15.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(extended.ok, extended.error);
  ROAD_CONTRACT_EXPECT(state.graph().corridors.size() == 1 &&
                           state.graph().corridors.front().segments.size() == 2,
                       "confirmed extension did not create one two-segment corridor");
  const RoadCorridor& corridor = state.graph().corridors.front();
  const auto boundary = ResolveCorridorDistance(
      state.graph(), state.derived(),
      CorridorDistanceRef{corridor.id, 15.0});
  ROAD_CONTRACT_EXPECT(
      boundary.ok &&
          boundary.value.segment_id == corridor.segments[1].segment_id &&
          std::abs(boundary.value.segment_distance_m) <= 1e-9,
      "internal corridor boundary did not resolve to following local zero");
  const auto periodic = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), corridor.id,
      RepeatingPlacementPolicy{10.0, 0.0});
  ROAD_CONTRACT_EXPECT(
      periodic.ok &&
          periodic.value == std::vector<double>({0.0, 10.0, 20.0, 30.0, 40.0}),
      "periodic placement reset or drifted at a segment boundary");

  SavedRoadGraph reversed_graph = state.graph();
  std::reverse(reversed_graph.corridors.front().segments.begin(),
               reversed_graph.corridors.front().segments.end());
  for (DirectedSegmentRef& ref : reversed_graph.corridors.front().segments) {
    ref.reversed = true;
  }
  const auto reversed_derived = city::road::generation::generate_road(reversed_graph);
  ROAD_CONTRACT_EXPECT(reversed_derived.ok, reversed_derived.error);
  const auto reversed_start = ResolveCorridorDistance(
      reversed_graph, reversed_derived.value,
      CorridorDistanceRef{corridor.id, 0.0});
  const DerivedSegment* original_last =
      FindDerivedSegment(reversed_derived.value,
                         reversed_graph.corridors.front().segments.front().segment_id);
  ROAD_CONTRACT_EXPECT(
      reversed_start.ok && original_last != nullptr &&
          reversed_start.value.reversed &&
          std::abs(reversed_start.value.segment_distance_m -
                   original_last->length_m) <= 1e-9,
      "reversed corridor start did not resolve to segment end");
  const auto reversed_side = ResolveCorridorSideRef(
      reversed_graph, reversed_derived.value,
      CorridorSideRef{corridor.id, RoadSide::kLeft, 0.0, 2.0});
  ROAD_CONTRACT_EXPECT(
      reversed_side.ok && reversed_side.value.side == RoadSide::kRight,
      "corridor left side did not flip on a reversed segment");
  const auto side_position =
      reversed_side.ok
          ? ResolveRoadSidePosition(reversed_derived.value,
                                    reversed_side.value)
          : Result<Vec3d>::Fail(ErrorKind::kInternal, "side resolve failed");
  ROAD_CONTRACT_EXPECT(
      side_position.ok && std::abs(side_position.value.x - 40.0) <= 1e-9 &&
          std::abs(side_position.value.y + 2.0) <= 1e-9,
      "reversed corridor side did not resolve in corridor direction");
  return true;
}

bool branch_and_tail_extension_preserve_existing_corridor(
    std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {15.0, 0.0})}),
      1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const RoadSegment initial_segment = state.graph().segments.front();
  const RoadCorridorId initial_corridor_id =
      state.graph().corridors.front().id;
  const auto initial_extension = state.ExtendCorridorFromEnd(
      ExtendCorridorFromEndRequest{
          initial_corridor_id, initial_segment.node_b,
          MakePath({MakeLine({15.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(initial_extension.ok, initial_extension.error);
  const RoadCorridor main_before = state.graph().corridors.front();
  std::map<RoadSegmentId, std::string> alignments_before{};
  for (const DirectedSegmentRef& ref : main_before.segments) {
    const Path* path = FindCanonicalAlignment(state.derived(), ref.segment_id);
    ROAD_CONTRACT_EXPECT(path != nullptr, "main alignment is missing");
    alignments_before.emplace(ref.segment_id, path_observation(*path));
  }
  const auto periodic_before = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), main_before.id,
      RepeatingPlacementPolicy{10.0, 0.0});
  ROAD_CONTRACT_EXPECT(periodic_before.ok, periodic_before.error);

  const RoadSegment& first =
      *std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                    [id = main_before.segments.front().segment_id](
                        const RoadSegment& segment) { return segment.id == id; });
  const auto branch = state.AddSegmentConnectedTo(AddSegmentConnectedToRequest{
      MakePath({MakeLine({15.0, 0.0}, {15.0, 20.0})}), 1, first.node_b});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);
  const RoadCorridor* main_after_branch =
      FindRoadCorridor(state.graph(), main_before.id);
  ROAD_CONTRACT_EXPECT(
      main_after_branch != nullptr &&
          main_after_branch->segments == main_before.segments &&
          state.graph().corridors.size() == 2,
      "branch changed the main corridor or was inserted into it");
  const auto periodic_after_branch = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), main_before.id,
      RepeatingPlacementPolicy{10.0, 0.0});
  ROAD_CONTRACT_EXPECT(
      periodic_after_branch.ok &&
          periodic_after_branch.value == periodic_before.value,
      "branch changed main corridor periodic placement");

  const DirectedSegmentRef tail_ref = main_after_branch->segments.back();
  const RoadSegment& tail =
      *std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                    [id = tail_ref.segment_id](const RoadSegment& segment) {
                      return segment.id == id;
                    });
  const RoadNodeId tail_node = tail_ref.reversed ? tail.node_a : tail.node_b;
  const auto extended = state.ExtendCorridorFromEnd(
      ExtendCorridorFromEndRequest{
          main_before.id, tail_node,
          MakePath({MakeLine({40.0, 0.0}, {60.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(extended.ok, extended.error);
  for (const auto& [segment_id, observation] : alignments_before) {
    const Path* path = FindCanonicalAlignment(state.derived(), segment_id);
    ROAD_CONTRACT_EXPECT(
        path != nullptr && path_observation(*path) == observation,
        "tail extension changed an existing segment alignment");
  }
  const auto periodic_after_extension = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), main_before.id,
      RepeatingPlacementPolicy{10.0, 0.0});
  ROAD_CONTRACT_EXPECT(
      periodic_after_extension.ok &&
          periodic_after_extension.value.size() >= periodic_before.value.size() &&
          std::equal(periodic_before.value.begin(), periodic_before.value.end(),
                     periodic_after_extension.value.begin()),
      "tail extension moved existing periodic placement distances");
  const RoadCorridor main_before_branch_delete =
      *FindRoadCorridor(state.graph(), main_before.id);
  const auto periodic_before_branch_delete =
      periodic_after_extension.value;
  const auto deleted_branch =
      state.DeleteSegment(DeleteSegmentRequest{branch.value});
  ROAD_CONTRACT_EXPECT(deleted_branch.ok, deleted_branch.error);
  const RoadCorridor* main_after_branch_delete =
      FindRoadCorridor(state.graph(), main_before.id);
  const auto periodic_after_branch_delete = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), main_before.id,
      RepeatingPlacementPolicy{10.0, 0.0});
  ROAD_CONTRACT_EXPECT(
      main_after_branch_delete != nullptr &&
          main_after_branch_delete->segments ==
              main_before_branch_delete.segments &&
          periodic_after_branch_delete.ok &&
          periodic_after_branch_delete.value ==
              periodic_before_branch_delete,
      "branch deletion changed the main corridor or its periodic placement");
  return true;
}

bool split_preserves_corridor_distance_and_world_position(
    std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeBezier({0.0, 0.0}, {12.0, 8.0}, {28.0, -8.0},
                           {40.0, 0.0})}),
      1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const RoadCorridor& before_corridor = state.graph().corridors.front();
  const RoadCorridorId corridor_id = before_corridor.id;
  const auto distances_before = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), before_corridor.id,
      RepeatingPlacementPolicy{5.0, 0.0});
  ROAD_CONTRACT_EXPECT(distances_before.ok, distances_before.error);
  std::vector<Vec2d> world_before{};
  for (const double distance : distances_before.value) {
    const auto resolved = ResolveCorridorDistance(
        state.graph(), state.derived(),
        CorridorDistanceRef{corridor_id, distance});
    ROAD_CONTRACT_EXPECT(resolved.ok, resolved.error);
    const Path* alignment =
        FindCanonicalAlignment(state.derived(), resolved.value.segment_id);
    ROAD_CONTRACT_EXPECT(alignment != nullptr,
                         "split baseline alignment is missing");
    const auto point =
        EvaluatePath(*alignment, resolved.value.segment_distance_m);
    ROAD_CONTRACT_EXPECT(point.ok, point.error);
    world_before.push_back(point.value);
  }
  const auto split =
      state.SplitSegmentAtDistance(SplitSegmentAtDistanceRequest{added.value,
                                                               17.0});
  ROAD_CONTRACT_EXPECT(split.ok, split.error);
  const RoadCorridor& after_corridor = state.graph().corridors.front();
  ROAD_CONTRACT_EXPECT(after_corridor.id == corridor_id &&
                           after_corridor.segments.size() == 2,
                       "split changed corridor identity or segment count");
  const auto distances_after = DeriveRepeatingPlacementDistances(
      state.graph(), state.derived(), after_corridor.id,
      RepeatingPlacementPolicy{5.0, 0.0});
  ROAD_CONTRACT_EXPECT(distances_after.ok &&
                           distances_after.value == distances_before.value,
                       "split changed corridor periodic distances");
  for (std::size_t index = 0; index < distances_after.value.size(); ++index) {
    const auto resolved = ResolveCorridorDistance(
        state.graph(), state.derived(),
        CorridorDistanceRef{after_corridor.id, distances_after.value[index]});
    ROAD_CONTRACT_EXPECT(resolved.ok, resolved.error);
    const Path* alignment =
        FindCanonicalAlignment(state.derived(), resolved.value.segment_id);
    ROAD_CONTRACT_EXPECT(alignment != nullptr,
                         "split result alignment is missing");
    const auto point =
        EvaluatePath(*alignment, resolved.value.segment_distance_m);
    const double drift =
        point.ok ? std::hypot(point.value.x - world_before[index].x,
                              point.value.y - world_before[index].y)
                 : std::numeric_limits<double>::infinity();
    ROAD_CONTRACT_EXPECT(
        point.ok && drift <= 1e-6,
        "split changed a corridor-distance world position by " +
            std::to_string(drift));
  }
  return true;
}

bool delete_range_splits_corridor_without_touching_outer_shapes(
    std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const Path* before_alignment =
      FindCanonicalAlignment(state.derived(), added.value);
  ROAD_CONTRACT_EXPECT(before_alignment != nullptr,
                       "range deletion baseline alignment is missing");
  const auto before_point = EvaluatePath(*before_alignment, 5.0);
  ROAD_CONTRACT_EXPECT(before_point.ok, before_point.error);
  const RoadCorridorId corridor_id = state.graph().corridors.front().id;
  const auto deleted = state.DeleteSegmentRange(
      DeleteSegmentRangeRequest{added.value, 10.0, 20.0});
  ROAD_CONTRACT_EXPECT(deleted.ok, deleted.error);
  ROAD_CONTRACT_EXPECT(state.graph().segments.size() == 2 &&
                           state.graph().corridors.size() == 2,
                       "range deletion did not retain two local segments and split the corridor");
  ROAD_CONTRACT_EXPECT(state.graph().corridors.front().id == corridor_id,
                       "range deletion did not preserve the start-side corridor ID");
  const Path* retained =
      FindCanonicalAlignment(state.derived(), added.value);
  ROAD_CONTRACT_EXPECT(retained != nullptr,
                       "range deletion retained alignment is missing");
  const auto after_point = EvaluatePath(*retained, 5.0);
  ROAD_CONTRACT_EXPECT(
      after_point.ok &&
          std::hypot(after_point.value.x - before_point.value.x,
                     after_point.value.y - before_point.value.y) <= 1e-6,
      "range deletion changed unaffected world geometry");
  return true;
}

bool delete_range_migrates_unaffected_segment_owned_state(
    std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const auto before_line = state.AddManualLine(ManualLineRequest{
      added.value, MakePath({MakeLine({2.0, 0.5}, {5.0, 0.5})}),
      builtin_marking_styles::kWhiteSolid});
  const auto after_line = state.AddManualLine(ManualLineRequest{
      added.value, MakePath({MakeLine({30.0, 0.5}, {35.0, 0.5})}),
      builtin_marking_styles::kWhiteSolid});
  const auto after_area = state.AddManualArea(ManualAreaRequest{
      added.value, {34.0, 0.0}, 0.0, 2.0, 2.0,
      builtin_marking_styles::kCrosswalk});
  ROAD_CONTRACT_EXPECT(before_line.ok && after_line.ok && after_area.ok,
                       "delete-range owner migration setup failed");
  const AutoMarkingKey suppression{
      MarkingOwner{MarkingOwner::Kind::kRoadSegment, added.value, 0, 0},
      MarkingRole::kCenterLine,
      MarkingTrackKey{added.value, 200, MarkingRole::kCenterLine},
      std::nullopt};
  const auto suppressed =
      state.SuppressAutoMarking(SuppressAutoMarkingRequest{suppression});
  ROAD_CONTRACT_EXPECT(suppressed.ok, suppressed.error);

  const auto deleted =
      state.DeleteSegmentRange(DeleteSegmentRangeRequest{added.value, 10.0, 20.0});
  ROAD_CONTRACT_EXPECT(deleted.ok, deleted.error);
  ROAD_CONTRACT_EXPECT(state.graph().segments.size() == 2,
                       "delete range did not retain two outer segments");
  const RoadSegmentId after_segment_id =
      std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                   [id = added.value](const RoadSegment& segment) {
                     return segment.id != id;
                   })
          ->id;
  const auto restored_before = std::find_if(
      state.graph().manual_lines.begin(), state.graph().manual_lines.end(),
      [id = before_line.value](const ManualLineMarking& marking) {
        return marking.id == id;
      });
  const auto restored_after = std::find_if(
      state.graph().manual_lines.begin(), state.graph().manual_lines.end(),
      [id = after_line.value](const ManualLineMarking& marking) {
        return marking.id == id;
      });
  const auto restored_area = std::find_if(
      state.graph().manual_areas.begin(), state.graph().manual_areas.end(),
      [id = after_area.value](const ManualAreaMarking& marking) {
        return marking.id == id;
      });
  ROAD_CONTRACT_EXPECT(
      restored_before != state.graph().manual_lines.end() &&
          restored_before->owner_segment_id == added.value &&
          restored_before->path.spans.front().p0.x == 2.0,
      "start-side manual owner moved during range deletion");
  ROAD_CONTRACT_EXPECT(
      restored_after != state.graph().manual_lines.end() &&
          restored_after->owner_segment_id == after_segment_id &&
          restored_after->path.spans.front().p0.x == 10.0,
      "end-side manual line was not mapped by deleted range length");
  ROAD_CONTRACT_EXPECT(
      restored_area != state.graph().manual_areas.end() &&
          restored_area->owner_segment_id == after_segment_id &&
          restored_area->frame_origin.x == 14.0,
      "end-side manual area was not mapped by deleted range length");
  ROAD_CONTRACT_EXPECT(
      state.graph().auto_marking_overrides.size() == 2 &&
          std::any_of(
              state.graph().auto_marking_overrides.begin(),
              state.graph().auto_marking_overrides.end(),
              [after_segment_id](const AutoMarkingOverride& value) {
                return value.key.owner.segment_id == after_segment_id &&
                       value.key.track.has_value() &&
                       value.key.track->segment_id == after_segment_id;
              }),
      "segment-wide semantic suppression was not copied to both retained pieces");
  return true;
}

bool standard_delete_removes_only_requested_segment(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(first.ok, first.error);
  const RoadCorridorId corridor_id = state.graph().corridors.front().id;
  const RoadNodeId first_end = state.graph().segments.front().node_b;
  const auto middle = state.ExtendCorridorFromEnd(ExtendCorridorFromEndRequest{
      corridor_id, first_end,
      MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(middle.ok, middle.error);
  const RoadNodeId middle_end =
      std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                   [id = middle.value](const RoadSegment& segment) {
                     return segment.id == id;
                   })
          ->node_b;
  const auto last = state.ExtendCorridorFromEnd(ExtendCorridorFromEndRequest{
      corridor_id, middle_end,
      MakePath({MakeLine({40.0, 0.0}, {60.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(last.ok, last.error);
  const Path* first_before =
      FindCanonicalAlignment(state.derived(), first.value);
  const Path* last_before =
      FindCanonicalAlignment(state.derived(), last.value);
  ROAD_CONTRACT_EXPECT(first_before != nullptr && last_before != nullptr,
                       "standard deletion baseline alignment is missing");
  const std::string first_shape = path_observation(*first_before);
  const std::string last_shape = path_observation(*last_before);

  const auto deleted =
      state.DeleteSegment(DeleteSegmentRequest{middle.value});
  ROAD_CONTRACT_EXPECT(deleted.ok, deleted.error);
  ROAD_CONTRACT_EXPECT(
      state.graph().segments.size() == 2 &&
          std::none_of(state.graph().segments.begin(),
                       state.graph().segments.end(),
                       [id = middle.value](const RoadSegment& segment) {
                         return segment.id == id;
                       }),
      "standard deletion removed more or less than the requested RoadSegment");
  ROAD_CONTRACT_EXPECT(
      state.graph().corridors.size() == 2 &&
          state.graph().corridors.front().id == corridor_id,
      "middle segment deletion did not deterministically split its corridor");
  const Path* first_after =
      FindCanonicalAlignment(state.derived(), first.value);
  const Path* last_after =
      FindCanonicalAlignment(state.derived(), last.value);
  ROAD_CONTRACT_EXPECT(
      first_after != nullptr && last_after != nullptr &&
          path_observation(*first_after) == first_shape &&
          path_observation(*last_after) == last_shape,
      "standard deletion changed an unrelated segment shape");
  return true;
}

bool standard_delete_preserves_unrelated_approach_override(
    std::string& failure) {
  RoadState state{};
  const auto main = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(main.ok, main.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), 1,
          main.value, 20.0});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);
  const auto branch_segment =
      std::find_if(state.graph().segments.begin(), state.graph().segments.end(),
                   [id = branch.value](const RoadSegment& segment) {
                     return segment.id == id;
                   });
  ROAD_CONTRACT_EXPECT(branch_segment != state.graph().segments.end(),
                       "branch segment is missing before standard deletion");
  const auto connection =
      std::find_if(state.derived().connections.begin(),
                   state.derived().connections.end(),
                   [node_id = branch_segment->node_a](
                       const ResolvedConnection& value) {
                     return value.node_id == node_id;
                   });
  ROAD_CONTRACT_EXPECT(connection != state.derived().connections.end(),
                       "branch connection is missing before standard deletion");
  const auto retained_approach =
      std::find_if(connection->approaches.begin(), connection->approaches.end(),
                   [branch_id = branch.value](const ResolvedApproach& value) {
                     return value.key.segment_id != branch_id;
                   });
  ROAD_CONTRACT_EXPECT(retained_approach != connection->approaches.end(),
                       "main-road approach is missing before branch deletion");
  const ApproachKey retained_key = retained_approach->key;
  const auto overridden = state.SetApproachLateralShiftOverride(
      SetApproachLateralShiftOverrideRequest{retained_key, 0.1});
  ROAD_CONTRACT_EXPECT(overridden.ok, overridden.error);

  const auto deleted = state.DeleteSegment(DeleteSegmentRequest{branch.value});
  ROAD_CONTRACT_EXPECT(deleted.ok, deleted.error);
  ROAD_CONTRACT_EXPECT(
      state.graph().approach_geometry_overrides.size() == 1 &&
          state.graph().approach_geometry_overrides.front().key == retained_key,
      "standard deletion removed an unrelated approach override");
  return true;
}
bool marking_invalid_interval_splits_polyline_runs(std::string& failure) {
  DerivedSegment segment{};
  segment.id = 77;
  segment.alignment =
      MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})});
  segment.length_m = 40.0;
  segment.surface_segment_distances_m = {0.0, 10.0, 20.0, 30.0, 40.0};
  for (const double distance : segment.surface_segment_distances_m) {
    SectionBoundarySample boundary{};
    boundary.boundary_id = 200;
    boundary.role = BoundaryRole::kLaneDivider;
    boundary.marking = AutoMarkingPolicy{
        true, MarkingRole::kCenterLine,
        builtin_marking_styles::kCenterLine};
    boundary.left_strip_id = 20;
    boundary.right_strip_id = 30;
    boundary.left_strip_width_m = distance == 20.0 ? 0.0 : 3.0;
    boundary.right_strip_width_m = 3.0;
    segment.sections.push_back(
        SectionEvaluation{77, distance, 1, {boundary}, {}});
  }
  const auto markings = city::road::generation::derive_markings(
      SavedRoadGraph{}, {segment}, {}, {});
  std::ostringstream diagnostic;
  diagnostic << "ok=" << markings.ok << " runs=" << markings.value.size();
  for (std::size_t index = 0; index < markings.value.size(); ++index) {
    const auto& points = markings.value[index].points;
    diagnostic << " run[" << index << "]=" << points.size();
    if (!points.empty()) {
      diagnostic << ':' << points.front().x << "->" << points.back().x;
    }
  }
  ROAD_CONTRACT_EXPECT(
      markings.ok && markings.value.size() == 2 &&
          markings.value[0].points.size() == 2 &&
          markings.value[1].points.size() == 2 &&
          std::abs(markings.value[0].points.back().x - 10.0) <= 1e-6 &&
          std::abs(markings.value[1].points.front().x - 30.0) <= 1e-6,
      "marking polyline bridged a degenerate interval: " + diagnostic.str());
  return true;
}

bool lane_endpoint_identity_ignores_template_order(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const RoadSegment& segment = state.graph().segments.front();
  SavedRoadGraph reordered = state.graph();
  auto section = std::find_if(
      reordered.section_templates.begin(), reordered.section_templates.end(),
      [&segment](const CrossSectionTemplate& candidate) {
        return candidate.id == segment.section_template;
      });
  ROAD_CONTRACT_EXPECT(section != reordered.section_templates.end(),
                       "lane endpoint template is missing");
  std::reverse(section->lane_bands.begin(), section->lane_bands.end());

  const LaneEndpointKey lane_key{
      segment.id, 1010, EndpointRole::kEnd};
  const auto lane = city::road::internal::find_lane_endpoint(reordered, lane_key);
  ROAD_CONTRACT_EXPECT(
      lane.segment != nullptr && lane.section != nullptr &&
          lane.lane != nullptr && lane.lane->id == 1010 &&
          lane.node_id == segment.node_b &&
          lane.lane->direction == LaneTravelDirection::kAlongSegment,
      "lane endpoint lookup depended on lane array order");

  const BoundaryEndpointKey boundary_key{
      segment.id, 200, EndpointRole::kEnd};
  const auto boundary =
      city::road::internal::find_boundary_endpoint(reordered, boundary_key);
  ROAD_CONTRACT_EXPECT(
      boundary.boundary != nullptr && boundary.boundary->boundary_id == 200 &&
          boundary.node_id == segment.node_b,
      "boundary endpoint lookup did not use stable boundary identity");
  ROAD_CONTRACT_EXPECT(
      city::road::internal::find_lane_endpoint(
          reordered, LaneEndpointKey{segment.id, 999999, EndpointRole::kEnd})
              .lane == nullptr,
      "lane endpoint lookup guessed a missing lane");
  return true;
}

bool seeded_operation_sequences_preserve_contracts(std::string& failure) {
  for (std::uint32_t seed = 1; seed <= 4; ++seed) {
    std::mt19937 random(seed);
    RoadState state{};
    for (int step = 0; step < 8; ++step) {
      const double x = static_cast<double>(step * 30);
      const double y = static_cast<double>(random() % 21) - 10.0;
      const auto added = state.AddSegment(AddSegmentRequest{
          MakePath({MakeLine({x, y}, {x + 20.0, y + static_cast<double>(random() % 5)})}), 1});
      ROAD_CONTRACT_EXPECT(added.ok, "seed " + std::to_string(seed) + " AddSegment: " + added.error);
      const RoadSegment& segment = state.graph().segments.back();
      if (step % 3 == 0) {
        const auto node = std::find_if(state.graph().nodes.begin(), state.graph().nodes.end(),
                                       [&segment](const auto& item) { return item.id == segment.node_b; });
        ROAD_CONTRACT_EXPECT(node != state.graph().nodes.end(), "seed node lookup failed");
        const auto moved = state.MoveNode(MoveNodeRequest{node->id, {node->position.x, node->position.y + 0.5}});
        ROAD_CONTRACT_EXPECT(moved.ok, "seed " + std::to_string(seed) + " MoveNode: " + moved.error);
      }
      if (step % 4 == 0) {
        const Path* path = FindCanonicalAlignment(state.derived(), added.value);
        ROAD_CONTRACT_EXPECT(path != nullptr, "seed canonical alignment lookup failed");
        const auto shape = SegmentShapeFromPath(*path);
        ROAD_CONTRACT_EXPECT(shape.ok, "seed shape conversion failed");
        SegmentShape edited_shape = shape.value;
        edited_shape.start_handle.y += 0.25;
        edited_shape.end_handle.y -= 0.25;
        const auto edited = state.EditSegmentShape(EditSegmentShapeRequest{added.value, edited_shape});
        ROAD_CONTRACT_EXPECT(edited.ok, "seed " + std::to_string(seed) + " EditSegmentShape: " + edited.error);
      }
      ROAD_CONTRACT_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok,
                           "seed " + std::to_string(seed) + " invariant failed at step " + std::to_string(step));
      const auto saved = state.Save();
      ROAD_CONTRACT_EXPECT(saved.ok, "seed save failed");
      const auto loaded = RoadState::Load(saved.value);
      ROAD_CONTRACT_EXPECT(loaded.ok && loaded.value.Save().value == saved.value,
                           "seed " + std::to_string(seed) + " round-trip failed at step " + std::to_string(step));
      ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                               state, [&state] { return state.MoveNode(MoveNodeRequest{999999, {0.0, 0.0}}); },
                               "seed invalid MoveNode", failure),
                           failure);
    }
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
      {"all_public_operation_validation_failures_are_atomic", all_public_operation_validation_failures_are_atomic},
      {"add_segment_build_failure_is_atomic", add_segment_build_failure_is_atomic},
      {"node_identity_does_not_come_from_position", node_identity_does_not_come_from_position},
      {"segment_shape_edit_does_not_move_endpoint_authority", segment_shape_edit_does_not_move_endpoint_authority},
      {"move_node_rederives_incident_alignment_endpoints", move_node_rederives_incident_alignment_endpoints},
      {"generate_road_is_pure", generate_road_is_pure},
      {"generate_is_deterministic", generate_is_deterministic},
      {"confirmation_boundaries_define_segment_units",
       confirmation_boundaries_define_segment_units},
      {"reverse_input_has_equivalent_geometry",
       reverse_input_has_equivalent_geometry},
      {"extension_semantic_boundaries_are_atomic",
       extension_semantic_boundaries_are_atomic},
      {"isolated_segment_uses_simple_path", isolated_segment_uses_simple_path},
      {"approach_override_resolves_and_persists_manual_fields",
       approach_override_resolves_and_persists_manual_fields},
      {"connection_section_and_gate_have_single_owners",
       connection_section_and_gate_have_single_owners},
      {"approach_identity_survives_geometry_changes",
       approach_identity_survives_geometry_changes},
      {"same_angle_approaches_use_id_tie_break",
       same_angle_approaches_use_id_tie_break},
      {"unsupported_junction_section_is_atomic",
       unsupported_junction_section_is_atomic},
      {"derived_segment_owns_all_semantic_distances",
       derived_segment_owns_all_semantic_distances},
      {"corridor_distance_and_periodic_placement_cross_segment_boundaries",
       corridor_distance_and_periodic_placement_cross_segment_boundaries},
      {"branch_and_tail_extension_preserve_existing_corridor",
       branch_and_tail_extension_preserve_existing_corridor},
      {"split_preserves_corridor_distance_and_world_position",
       split_preserves_corridor_distance_and_world_position},
      {"delete_range_splits_corridor_without_touching_outer_shapes",
       delete_range_splits_corridor_without_touching_outer_shapes},
      {"delete_range_migrates_unaffected_segment_owned_state",
       delete_range_migrates_unaffected_segment_owned_state},
      {"standard_delete_removes_only_requested_segment",
       standard_delete_removes_only_requested_segment},
      {"standard_delete_preserves_unrelated_approach_override",
       standard_delete_preserves_unrelated_approach_override},
      {"marking_invalid_interval_splits_polyline_runs",
       marking_invalid_interval_splits_polyline_runs},
      {"lane_endpoint_identity_ignores_template_order",
       lane_endpoint_identity_ignores_template_order},
      {"seeded_operation_sequences_preserve_contracts", seeded_operation_sequences_preserve_contracts},
  };
  int failed = 0;
  for (const auto& test : tests) {
    std::string failure;
    const bool ok = test.run(failure);
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << test.name << '\n';
    if (!ok) {
      std::cerr << "  reason: " << failure << '\n';
      ++failed;
    }
  }
  if (failed != 0) return 1;
  std::cout << "road architecture contract tests passed (" << std::size(tests) << " cases)\n";
  return 0;
}
