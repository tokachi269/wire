#include "city/road/road.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <locale>
#include <random>
#include <sstream>
#include <string>
#include <string_view>

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
      << value.height_m << ',' << static_cast<int>(value.marking_rule) << ';';
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

std::string derived_observation(const DerivedRoad& derived) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::hexfloat;
  for (const std::size_t runs : derived.build_stage_runs) out << runs << ',';
  out << ':' << derived.setback_calculation_count << ',' << derived.section_evaluation_count << ':';
  out << derived.canonical_alignments.size() << ':';
  for (const auto& alignment : derived.canonical_alignments) {
    out << alignment.segment_id << ',' << alignment.path.spans.size() << ':';
    for (const auto& span : alignment.path.spans) {
      out << span.p0.x << ',' << span.p0.y << ',' << span.p1.x << ',' << span.p1.y << ','
          << span.p2.x << ',' << span.p2.y << ',' << span.p3.x << ',' << span.p3.y << ';';
    }
  }
  out << derived.node_connection_decisions.size() << ':';
  for (const auto& decision : derived.node_connection_decisions) {
    out << decision.node_id << ',' << static_cast<int>(decision.kind) << ','
        << decision.corner_radius_m << ',' << decision.corner_control_m << ','
        << decision.junction_corner_control_m << ',' << decision.applied_policy_override_id << ':';
    for (const ApproachKey& key : decision.ordered_approaches) {
      append_approach(out, key);
      out << ';';
    }
    for (const ApproachConnectionDecision& approach : decision.approaches) {
      append_approach(out, approach.key);
      out << ',' << approach.endpoint_template_id << ',' << approach.setback_m << ','
          << approach.gate_station_m << ';';
    }
  }
  out << derived.sampling_plans.size() << ':';
  for (const auto& plan : derived.sampling_plans) {
    out << plan.segment_id << ':';
    for (double station : plan.semantic_stations_m) out << station << ',';
    out << ':';
    for (double station : plan.surface_stations_m) out << station << ',';
    out << ';';
  }
  out << derived.section_evaluations.size() << ':';
  for (const auto& section : derived.section_evaluations) {
    out << section.segment_id << ',' << section.station_m << ','
        << section.resolved_template_id << ':';
    for (const auto& boundary : section.boundaries) append_boundary(out, boundary);
    for (const auto& style : section.surface_styles) {
      out << static_cast<int>(style.domain) << ':' << style.value << ';';
    }
  }
  append_meshes(out, derived.segment_meshes);
  append_meshes(out, derived.marking_meshes);
  out << derived.terrain_masks.size() << ':';
  for (const auto& mask : derived.terrain_masks) {
    out << mask.segment_id << ',' << mask.points.size() << ':';
    for (const auto& point : mask.points) out << point.x << ',' << point.y << ';';
  }
  out << derived.connection_gates.size() << ':';
  for (const auto& gate : derived.connection_gates) append_gate(out, gate);
  out << derived.connection_areas.size() << ':';
  for (const auto& area : derived.connection_areas) {
    out << area.node_id << ',' << area.gates.size() << ':';
    for (const auto& gate : area.gates) append_gate(out, gate);
  }
  out << derived.connection_geometries.size() << ':';
  for (const ConnectionGeometry& geometry : derived.connection_geometries) {
    out << geometry.node_id << ',' << geometry.surface_strips.size() << ':';
    for (const ResolvedSurfaceStrip& strip : geometry.surface_strips) {
      out << static_cast<int>(strip.style.domain) << ':' << strip.style.value << ','
          << strip.left_boundary_id << ',' << strip.right_boundary_id << ':';
      for (const Vec3d& point : strip.left) {
        out << point.x << ',' << point.y << ',' << point.z << ';';
      }
      for (const Vec3d& point : strip.right) {
        out << point.x << ',' << point.y << ',' << point.z << ';';
      }
    }
  }
  append_meshes(out, derived.connection_meshes);
  out << derived.junction_areas.size() << ':';
  for (const auto& area : derived.junction_areas) {
    out << area.policy_override_id << ',' << area.node_id << ',' << area.gates.size() << ':';
    for (const auto& gate : area.gates) append_gate(out, gate);
  }
  out << derived.junction_geometries.size() << ':';
  for (const JunctionGeometry& geometry : derived.junction_geometries) {
    out << geometry.node_id << ',' << geometry.surface_regions.size() << ','
        << geometry.surface_strips.size() << ','
        << geometry.auto_markings.size() << ':';
    for (const ResolvedSurfaceRegion& region : geometry.surface_regions) {
      out << static_cast<int>(region.style.domain) << ':' << region.style.value << ':';
      for (const Vec3d& point : region.perimeter) {
        out << point.x << ',' << point.y << ',' << point.z << ';';
      }
    }
    for (const ResolvedAutoMarking& marking : geometry.auto_markings) {
      append_approach(out, marking.approach);
      out << ':' << marking.quads.size() << ':';
      for (const auto& quad : marking.quads) {
        for (const Vec3d& point : quad) {
          out << point.x << ',' << point.y << ',' << point.z << ';';
        }
      }
    }
  }
  append_meshes(out, derived.junction_meshes);
  append_meshes(out, derived.junction_marking_meshes);
  append_meshes(out, derived.manual_marking_meshes);
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
  for (const auto& gate : state.derived().connection_gates) out << 'g' << gate.segment_id << ',' << gate.node_id << ';';
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
                             return state.ExtendSegment(
                                 ExtendSegmentRequest{999999, 999998, line, 1});
                           },
                           "ExtendSegment", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{line, 1, 999999}); },
                           "AddSegmentConnectedTo", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{line, 1, 999999, 10.0}); },
                           "AddSegmentConnectedToSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.EditSegmentShape(city::road::EditSegmentShapeRequest{999999, {}}); }, "EditSegmentShape", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.MoveNode(city::road::MoveNodeRequest{999999, {0.0, 0.0}}); }, "MoveNode", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.DeleteSegment(city::road::DeleteSegmentRequest{999999}); }, "DeleteSegment", failure), failure);
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
  invalid_transition.start = {StationRefKind::kFromStart, 1.0};
  invalid_transition.end = {StationRefKind::kFromStart, 2.0};
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

bool extension_history_normalizes_to_one_segment(std::string& failure) {
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
  const auto extended = incremental.ExtendSegment(
      ExtendSegmentRequest{segment.id, segment.node_b, second, 1});
  ROAD_CONTRACT_EXPECT(extended.ok, extended.error);
  ROAD_CONTRACT_EXPECT(extended.value == first_added.value,
                       "extension changed the segment identity");
  ROAD_CONTRACT_EXPECT(incremental.graph().segments.size() == 1 &&
                           incremental.graph().nodes.size() == 2,
                       "extension created a gesture-owned node or segment");
  ROAD_CONTRACT_EXPECT(observe(incremental) == observe(one_operation),
                       "one-shot and incremental extension observations differ");

  const Path prepend =
      MakePath({MakeLine({0.0, 0.0}, {-12.0, 16.0})});
  const Path prepended_complete =
      MakePath({BezierSpan{prepend.spans.front().p3,
                           prepend.spans.front().p2,
                           prepend.spans.front().p1,
                           prepend.spans.front().p0},
                first.spans.front()});
  RoadState prepended_once{};
  const auto prepended_once_added =
      prepended_once.AddSegment(AddSegmentRequest{prepended_complete, 1});
  ROAD_CONTRACT_EXPECT(prepended_once_added.ok, prepended_once_added.error);
  RoadState prepended_incremental{};
  const auto prepend_base =
      prepended_incremental.AddSegment(AddSegmentRequest{first, 1});
  ROAD_CONTRACT_EXPECT(prepend_base.ok, prepend_base.error);
  const RoadSegment prepend_segment =
      prepended_incremental.graph().segments.front();
  const auto prepended = prepended_incremental.ExtendSegment(
      ExtendSegmentRequest{prepend_segment.id, prepend_segment.node_a,
                           prepend, 1});
  ROAD_CONTRACT_EXPECT(prepended.ok, prepended.error);
  ROAD_CONTRACT_EXPECT(observe(prepended_incremental) ==
                           observe(prepended_once),
                       "one-shot and prepend observations differ");
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
  const Path* forward_alignment =
      FindCanonicalAlignment(forward.derived(), forward_added.value);
  const Path* reverse_alignment =
      FindCanonicalAlignment(reverse.derived(), reverse_added.value);
  ROAD_CONTRACT_EXPECT(forward_alignment != nullptr &&
                           reverse_alignment != nullptr &&
                           forward_alignment->spans.size() ==
                               reverse_alignment->spans.size(),
                       "reverse canonical alignment is missing");
  const auto close = [](Vec2d a, Vec2d b) {
    return std::abs(a.x - b.x) <= 1e-9 &&
           std::abs(a.y - b.y) <= 1e-9;
  };
  for (std::size_t index = 0; index < forward_alignment->spans.size();
       ++index) {
    const BezierSpan& a = forward_alignment->spans[index];
    const BezierSpan& b =
        reverse_alignment
            ->spans[reverse_alignment->spans.size() - 1 - index];
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
  std::string marking_diagnostic;
  ROAD_CONTRACT_EXPECT(
      equivalent_vertices(forward.derived().marking_meshes,
                          reverse.derived().marking_meshes,
                          marking_diagnostic),
      "reverse marking geometry differs: " + marking_diagnostic);
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
            return connected.ExtendSegment(ExtendSegmentRequest{
                connected_segment.id, connected_segment.node_b,
                extension, 1});
          },
          "degree-two ExtendSegment", failure),
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
            return mixed_section.ExtendSegment(ExtendSegmentRequest{
                mixed_segment.id, mixed_segment.node_b, extension,
                section.value});
          },
          "mixed-section ExtendSegment", failure),
      failure);

  RoadState marked{};
  const auto marked_base =
      marked.AddSegment(AddSegmentRequest{base_path, 1});
  ROAD_CONTRACT_EXPECT(marked_base.ok, marked_base.error);
  const auto marking = marked.AddManualArea(
      ManualAreaRequest{marked_base.value, {10.0, 0.0}, 2.0, 2.0,
                        builtin_marking_styles::kCrosswalk});
  ROAD_CONTRACT_EXPECT(marking.ok, marking.error);
  const RoadSegment marked_segment = marked.graph().segments.front();
  const Path prepend =
      MakePath({MakeLine({0.0, 0.0}, {-12.0, 16.0})});
  ROAD_CONTRACT_EXPECT(
      expect_failed_unchanged(
          marked,
          [&] {
            return marked.ExtendSegment(ExtendSegmentRequest{
                marked_segment.id, marked_segment.node_a, prepend, 1});
          },
          "marked prepend ExtendSegment", failure),
      failure);
  return true;
}

bool isolated_segment_uses_simple_path(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(AddSegmentRequest{
      MakePath({MakeLine({0.0, 0.0}, {30.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  ROAD_CONTRACT_EXPECT(state.derived().node_connection_decisions.empty(),
                       "isolated segment created node connection decisions");
  ROAD_CONTRACT_EXPECT(state.derived().connection_gates.empty(),
                       "isolated segment created connection gates");
  ROAD_CONTRACT_EXPECT(state.derived().connection_geometries.empty() &&
                           state.derived().junction_geometries.empty(),
                       "isolated segment entered connection geometry");
  return true;
}

bool add_segment_build_failure_is_atomic(std::string& failure) {
  RoadState state{};
  CrossSectionTemplate unusable{};
  unusable.bands = {
      SurfaceBand{10, SurfaceRole::kSidewalk, 1.0e308, 0.0, builtin_surface_styles::kSidewalk},
      SurfaceBand{20, SurfaceRole::kCarriageway, 1.0e308, 0.0, builtin_surface_styles::kAsphalt},
      SurfaceBand{30, SurfaceRole::kSidewalk, 1.0e308, 0.0, builtin_surface_styles::kSidewalk},
  };
  unusable.boundaries = {
      BoundaryProfile{11, BoundaryRole::kCurb, 0.0, 0.15, MarkingRule::kNone},
      BoundaryProfile{21, BoundaryRole::kCurb, 0.0, -0.15, MarkingRule::kNone},
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
  const auto moved = state.MoveNode(city::road::MoveNodeRequest{shared, {20.0, 5.0}});
  ROAD_CONTRACT_EXPECT(moved.ok, moved.error);
  const Path* first_path = FindCanonicalAlignment(state.derived(), first.value);
  const Path* second_path = FindCanonicalAlignment(state.derived(), second.value);
  ROAD_CONTRACT_EXPECT(first_path != nullptr && second_path != nullptr, "incident canonical alignment is missing");
  ROAD_CONTRACT_EXPECT(first_path->spans.back().p3.x == 20.0 && first_path->spans.back().p3.y == 5.0 &&
                           second_path->spans.front().p0.x == 20.0 && second_path->spans.front().p0.y == 5.0,
                       "MoveNode did not rederive every incident canonical endpoint");
  ROAD_CONTRACT_EXPECT(state.graph().segments[0].node_b == shared && state.graph().segments[1].node_a == shared,
                       "MoveNode changed segment connectivity");
  return true;
}

bool build_runs_each_stage_exactly_once(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  ROAD_CONTRACT_EXPECT(std::all_of(state.derived().build_stage_runs.begin(), state.derived().build_stage_runs.end(),
                                   [](std::size_t runs) { return runs == 1; }),
                       "Build did not execute every stage exactly once");
  return true;
}

const NodeConnectionDecision* find_decision(const DerivedRoad& derived, RoadNodeId node_id) {
  const auto found = std::find_if(
      derived.node_connection_decisions.begin(), derived.node_connection_decisions.end(),
      [node_id](const NodeConnectionDecision& decision) { return decision.node_id == node_id; });
  return found == derived.node_connection_decisions.end() ? nullptr : &*found;
}

bool same_boundaries(const std::vector<SectionBoundarySample>& a,
                     const std::vector<SectionBoundarySample>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t index = 0; index < a.size(); ++index) {
    if (a[index].boundary_id != b[index].boundary_id ||
        a[index].role != b[index].role ||
        a[index].lateral_m != b[index].lateral_m ||
        a[index].height_m != b[index].height_m ||
        a[index].marking_rule != b[index].marking_rule) {
      return false;
    }
  }
  return true;
}

bool decision_sampling_section_and_gate_have_single_owners(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(
      AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(
      AddSegmentConnectedToSegmentRequest{
          MakePath({MakeLine({20.0, 0.0}, {32.0, 24.0})}), 1, base.value, 20.0});
  ROAD_CONTRACT_EXPECT(branch.ok, branch.error);

  const DerivedRoad& derived = state.derived();
  std::size_t approach_count = 0;
  for (const NodeConnectionDecision& decision : derived.node_connection_decisions) {
    approach_count += decision.approaches.size();
    ROAD_CONTRACT_EXPECT(decision.approaches.size() == decision.ordered_approaches.size(),
                         "decision approach table and deterministic order differ in size");
    for (const ApproachConnectionDecision& approach : decision.approaches) {
      ROAD_CONTRACT_EXPECT(
          std::count_if(decision.approaches.begin(), decision.approaches.end(),
                        [&approach](const ApproachConnectionDecision& candidate) {
                          return candidate.key == approach.key;
                        }) == 1,
          "ApproachKey has more than one setback decision");
    }
  }
  ROAD_CONTRACT_EXPECT(derived.setback_calculation_count == approach_count,
                       "setback calculation count does not equal ApproachKey count");
  ROAD_CONTRACT_EXPECT(derived.connection_gates.size() == approach_count,
                       "ConnectionGate count does not equal ApproachKey count");
  ROAD_CONTRACT_EXPECT(derived.section_evaluation_count == derived.section_evaluations.size(),
                       "SectionEvaluation counter differs from the produced table");

  for (const ConnectionGate& gate : derived.connection_gates) {
    const NodeConnectionDecision* decision = find_decision(derived, gate.approach.node_id);
    ROAD_CONTRACT_EXPECT(decision != nullptr, "gate decision is missing");
    const auto approach = std::find_if(
        decision->approaches.begin(), decision->approaches.end(),
        [&gate](const ApproachConnectionDecision& value) {
          return value.key == gate.approach;
        });
    ROAD_CONTRACT_EXPECT(approach != decision->approaches.end(),
                         "gate ApproachKey has no decision row");
    const auto plan = std::find_if(
        derived.sampling_plans.begin(), derived.sampling_plans.end(),
        [&gate](const SegmentSamplingPlan& value) {
          return value.segment_id == gate.approach.segment_id;
        });
    ROAD_CONTRACT_EXPECT(plan != derived.sampling_plans.end(),
                         "gate sampling plan is missing");
    ROAD_CONTRACT_EXPECT(
        std::count(plan->semantic_stations_m.begin(), plan->semantic_stations_m.end(),
                   approach->gate_station_m) == 1,
        "gate station is not a unique semantic station");
    const auto section_count = std::count_if(
        derived.section_evaluations.begin(), derived.section_evaluations.end(),
        [&gate, &approach](const SectionEvaluation& section) {
          return section.segment_id == gate.approach.segment_id &&
                 section.station_m == approach->gate_station_m;
        });
    ROAD_CONTRACT_EXPECT(section_count == 1,
                         "gate station has no unique SectionEvaluation");
    const auto section = std::find_if(
        derived.section_evaluations.begin(), derived.section_evaluations.end(),
        [&gate, &approach](const SectionEvaluation& value) {
          return value.segment_id == gate.approach.segment_id &&
                 value.station_m == approach->gate_station_m;
        });
    ROAD_CONTRACT_EXPECT(same_boundaries(gate.boundaries, section->boundaries),
                         "gate boundaries differ from their SectionEvaluation");
  }
  return true;
}

std::vector<ApproachKey> sorted_gate_keys(const DerivedRoad& derived) {
  std::vector<ApproachKey> keys{};
  for (const ConnectionGate& gate : derived.connection_gates) keys.push_back(gate.approach);
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
  const NodeConnectionDecision* decision = find_decision(state.derived(), shared);
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
  const NodeConnectionDecision* loaded_decision =
      find_decision(loaded.value.derived(), shared);
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
  const NodeConnectionDecision* reordered_decision =
      find_decision(reordered.value.derived(), shared);
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

bool sampling_plan_owns_all_semantic_stations(std::string& failure) {
  RoadState state{};
  const Path alignment = MakePath({
      MakeLine({0.0, 0.0}, {20.0, 0.0}),
      MakeLine({20.0, 0.0}, {50.0, 0.0}),
  });
  const auto segment = state.AddSegment(AddSegmentRequest{alignment, 1});
  ROAD_CONTRACT_EXPECT(segment.ok, segment.error);
  SectionTransitionRequest transition{};
  transition.to_template = 2;
  transition.start = StationRef{StationRefKind::kFromStart, 10.0};
  transition.end = StationRef{StationRefKind::kFromStart, 30.0};
  transition.rules = {
      SectionTransitionRule{35, TransitionAction::kTaperIn},
  };
  const auto attached = state.AddTransitionToSegment(
      AddTransitionToSegmentRequest{segment.value, transition});
  ROAD_CONTRACT_EXPECT(attached.ok, attached.error);
  const auto plan = std::find_if(
      state.derived().sampling_plans.begin(),
      state.derived().sampling_plans.end(),
      [&segment](const SegmentSamplingPlan& value) {
        return value.segment_id == segment.value;
      });
  ROAD_CONTRACT_EXPECT(plan != state.derived().sampling_plans.end(),
                       "semantic sampling plan is missing");
  for (const double station : std::array<double, 5>{0.0, 10.0, 20.0, 30.0, 50.0}) {
    ROAD_CONTRACT_EXPECT(
        std::count(plan->semantic_stations_m.begin(),
                   plan->semantic_stations_m.end(), station) == 1,
        "required endpoint/span/transition semantic station is missing or duplicated");
    ROAD_CONTRACT_EXPECT(
        std::count_if(
            state.derived().section_evaluations.begin(),
            state.derived().section_evaluations.end(),
            [&segment, station](const SectionEvaluation& section) {
              return section.segment_id == segment.value &&
                     section.station_m == station;
            }) == 1,
        "semantic station has no unique SectionEvaluation");
  }
  ROAD_CONTRACT_EXPECT(
      std::is_sorted(plan->semantic_stations_m.begin(),
                     plan->semantic_stations_m.end()),
      "semantic stations are not sorted");
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
      {"build_runs_each_stage_exactly_once", build_runs_each_stage_exactly_once},
      {"extension_history_normalizes_to_one_segment",
       extension_history_normalizes_to_one_segment},
      {"reverse_input_has_equivalent_geometry",
       reverse_input_has_equivalent_geometry},
      {"extension_semantic_boundaries_are_atomic",
       extension_semantic_boundaries_are_atomic},
      {"isolated_segment_uses_simple_path", isolated_segment_uses_simple_path},
      {"decision_sampling_section_and_gate_have_single_owners",
       decision_sampling_section_and_gate_have_single_owners},
      {"approach_identity_survives_geometry_changes",
       approach_identity_survives_geometry_changes},
      {"same_angle_approaches_use_id_tie_break",
       same_angle_approaches_use_id_tie_break},
      {"unsupported_junction_section_is_atomic",
       unsupported_junction_section_is_atomic},
      {"sampling_plan_owns_all_semantic_stations",
       sampling_plan_owns_all_semantic_stations},
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
