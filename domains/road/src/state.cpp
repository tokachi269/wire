#include "city/road/road.hpp"

#include "generation/generation.hpp"
#include "geometry/geometry.hpp"
#include "geometry/section.hpp"
#include "lookup.hpp"
#include "geometry/alignment.hpp"
#include "operations/operation_plan.hpp"
#include "persistence/archive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace city::road {
namespace {

using internal::add;
using internal::node_degree;
using internal::align_first_span_start;
using internal::align_last_span_end;
using internal::almost_same;
using internal::apply_inherited_arc;
using internal::corridor_terminal_handle;
using internal::cross;
using internal::distance;
using internal::is_finite;
using internal::kCurveSamples;
using internal::distance_epsilon;
using internal::magnitude;
using internal::make_linear_shape;
using internal::manual_area_distance_bounds;
using internal::manual_line_distance_bounds;
using internal::scale;
using internal::path_end;
using internal::path_start;
using internal::PathSplit;
using internal::shift_manual_line_distance;
using internal::span_end;
using internal::span_start;
using internal::split_path_at_distance;
using internal::subtract;

constexpr double kP1MinSegmentLengthM = 8.0;
constexpr double kSnapDistancePointToleranceM = 0.6;




[[nodiscard]] const RoadLayoutTemplate* find_template(const SavedRoadGraph& graph, RoadLayoutTemplateId id) {
  const auto it = std::find_if(graph.layout_templates.begin(), graph.layout_templates.end(),
                               [id](const RoadLayoutTemplate& item) { return item.id == id; });
  return it == graph.layout_templates.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadLayoutTransition* find_transition(const SavedRoadGraph& graph, RoadLayoutTransitionId id) {
  const auto it = std::find_if(graph.transitions.begin(), graph.transitions.end(),
                               [id](const RoadLayoutTransition& item) { return item.id == id; });
  return it == graph.transitions.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadSegment* find_segment(const SavedRoadGraph& graph, RoadSegmentId id) {
  const auto it = std::find_if(graph.segments.begin(), graph.segments.end(),
                               [id](const RoadSegment& item) { return item.id == id; });
  return it == graph.segments.end() ? nullptr : &*it;
}

[[nodiscard]] RoadSegment* find_segment(SavedRoadGraph& graph, RoadSegmentId id) {
  const auto it = std::find_if(graph.segments.begin(), graph.segments.end(),
                               [id](const RoadSegment& item) { return item.id == id; });
  return it == graph.segments.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadNode* find_node(const SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

[[nodiscard]] const NodeConnectionPolicyOverride* find_policy_override(const SavedRoadGraph& graph,
                                                                                   RoadNodeId node_id) {
  const auto it = std::find_if(graph.connection_policy_overrides.begin(), graph.connection_policy_overrides.end(),
                               [node_id](const auto& item) { return item.node_id == node_id; });
  return it == graph.connection_policy_overrides.end() ? nullptr : &*it;
}

[[nodiscard]] const ApproachGeometryOverride* find_approach_override(const SavedRoadGraph& graph,
                                                                               const ApproachKey& key) {
  const auto it = std::find_if(graph.approach_geometry_overrides.begin(), graph.approach_geometry_overrides.end(),
                               [&key](const ApproachGeometryOverride& item) { return item.key == key; });
  return it == graph.approach_geometry_overrides.end() ? nullptr : &*it;
}

[[nodiscard]] RoadNode* find_node(SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}




} // namespace

namespace {
} // namespace

// A new road starts with no section at all. Which cross sections a product
// offers, and what they measure, belongs to whoever presents them.
RoadState::RoadState() = default;

const SavedRoadGraph& RoadState::graph() const {
  return graph_;
}

const DerivedRoad& RoadState::derived() const {
  return derived_;
}

Result<bool> RoadState::Execute(const operations::OperationPlan& plan) {
  RoadState trial = *this;
  const Result<bool> applied = operations::Apply(plan, trial.graph_, trial.next_id_);
  if (!applied.ok) return applied;
  const Result<bool> authoritative_valid =
      persistence::ValidateAuthoritativeGraph(trial.graph_, trial.next_id_);
  if (!authoritative_valid.ok) return authoritative_valid;
  Result<DerivedRoad> generated = generation::generate_road(trial.graph_);
  if (!generated.ok) {
    return Result<bool>::Fail(generated.failure_category, generated.error);
  }
  trial.derived_ = std::move(generated.value);
  const Result<bool> derived_valid = ValidateGraphInvariants(trial.graph_, trial.derived_);
  if (!derived_valid.ok) return derived_valid;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}




Result<std::string> RoadState::Save() const {
  return persistence::SaveRoad(graph_, next_id_);
}

Result<RoadState> RoadState::Load(const std::string& text) {
  Result<persistence::LoadedRoad> loaded = persistence::LoadRoad(text);
  if (!loaded.ok) {
    return Result<RoadState>::Fail(loaded.failure_category, loaded.error);
  }
  RoadState state{};
  state.graph_ = std::move(loaded.value.graph);
  state.next_id_ = loaded.value.next_id;
  const Result<bool> authoritative_valid =
      persistence::ValidateAuthoritativeGraph(state.graph_, state.next_id_);
  if (!authoritative_valid.ok) {
    return Result<RoadState>::Fail(authoritative_valid.failure_category,
                                   authoritative_valid.error);
  }
  Result<DerivedRoad> generated = generation::generate_road(state.graph_);
  if (!generated.ok) {
    return Result<RoadState>::Fail(generated.failure_category, generated.error);
  }
  state.derived_ = std::move(generated.value);
  const Result<bool> derived_valid = ValidateGraphInvariants(state.graph_, state.derived_);
  if (!derived_valid.ok) {
    return Result<RoadState>::Fail(derived_valid.failure_category, derived_valid.error);
  }
  return Result<RoadState>::Ok(std::move(state));
}

Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived) {
  std::unordered_set<std::uint64_t> ids{};
  for (const RoadLayoutTemplate& section : graph.layout_templates) {
    if (!ids.insert(section.id).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road section template ID invariant failed");
    }
  }
  for (const RoadNode& node : graph.nodes) {
    if (!ids.insert(node.id).second || !is_finite(node.position)) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road node invariant failed");
    }
  }
  for (const RoadSegment& segment : graph.segments) {
    if (!ids.insert(segment.id).second || find_node(graph, segment.node_a) == nullptr ||
        find_node(graph, segment.node_b) == nullptr || find_template(graph, segment.layout_template) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road segment reference invariant failed");
    }
    if (FindDerivedSegment(derived, segment.id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road derived segment is missing");
    }
  }

  std::size_t expected_approaches = 0;
  for (const RoadSegment& segment : graph.segments) {
    for (const RoadNodeId node_id :
         std::array<RoadNodeId, 2>{segment.node_a, segment.node_b}) {
      if (node_degree(graph, node_id) >= 2 ||
          find_policy_override(graph, node_id) != nullptr) {
        ++expected_approaches;
      }
    }
  }

  std::size_t resolved_approaches = 0;
  for (const ResolvedConnection& connection : derived.connections) {
    if (find_node(graph, connection.node_id) == nullptr ||
        connection.approaches.size() != connection.ordered_approaches.size()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road resolved connection invariant failed");
    }
    resolved_approaches += connection.approaches.size();
    for (const ResolvedApproach& approach : connection.approaches) {
      const RoadSegment* segment = find_segment(graph, approach.key.segment_id);
      const bool endpoint_matches =
          segment != nullptr && approach.key.node_id == connection.node_id &&
          ((approach.key.endpoint_role == EndpointRole::kStart &&
            segment->node_a == approach.key.node_id) ||
           (approach.key.endpoint_role == EndpointRole::kEnd &&
            segment->node_b == approach.key.node_id));
      if (!endpoint_matches || approach.endpoint_template_id == 0 ||
          !is_finite(approach.auto_setback_m) || approach.auto_setback_m < 0.0 ||
          !is_finite(approach.resolved_setback_m) || approach.resolved_setback_m < 0.0 ||
          !is_finite(approach.resolved_lateral_shift_m) ||
          !is_finite(approach.gate_segment_distance_m)) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road resolved approach invariant failed");
      }
      const std::size_t same_approach_rows = static_cast<std::size_t>(
          std::count_if(connection.approaches.begin(), connection.approaches.end(),
                        [&approach](const ResolvedApproach& candidate) {
                          return candidate.key == approach.key;
                        }));
      const std::size_t same_order_rows = static_cast<std::size_t>(
          std::count(connection.ordered_approaches.begin(),
                     connection.ordered_approaches.end(), approach.key));
      if (same_approach_rows != 1 || same_order_rows != 1) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road ApproachKey identity invariant failed");
      }

      const ConnectionGate& gate = approach.gate;
      if (gate.segment_id != approach.key.segment_id ||
          gate.node_id != approach.key.node_id || gate.approach != approach.key ||
          !is_finite(gate.position.x) || !is_finite(gate.position.y) || !is_finite(gate.position.z) ||
          !is_finite(gate.tangent.x) || !is_finite(gate.tangent.y) || !is_finite(gate.tangent.z) ||
          !is_finite(gate.lateral.x) || !is_finite(gate.lateral.y) || !is_finite(gate.lateral.z) ||
          !is_finite(gate.normal.x) || !is_finite(gate.normal.y) || !is_finite(gate.normal.z)) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road connection gate frame invariant failed");
      }
      const DerivedSegment* owner = FindDerivedSegment(derived, approach.key.segment_id);
      const SectionEvaluation* section =
          owner == nullptr ? nullptr : FindSectionAt(*owner, approach.gate_segment_distance_m);
      if (section == nullptr || section->boundaries.size() != gate.boundaries.size()) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInternalError,
            "road connection gate section evaluation invariant failed");
      }
      for (std::size_t index = 0; index < gate.boundaries.size(); ++index) {
        const SectionBoundarySample& gate_boundary = gate.boundaries[index];
        const SectionBoundarySample& section_boundary = section->boundaries[index];
        if (gate_boundary.boundary_id != section_boundary.boundary_id ||
            gate_boundary.role != section_boundary.role ||
            gate_boundary.lateral_m != section_boundary.lateral_m ||
            gate_boundary.height_m != section_boundary.height_m ||
            gate_boundary.marking != section_boundary.marking) {
          return Result<bool>::Fail(
              CommitFailureCategory::kInternalError,
              "road connection gate boundary copy invariant failed");
        }
      }
    }
  }
  if (resolved_approaches != expected_approaches) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "road approach coverage invariant failed");
  }
  for (const DerivedSegment& segment : derived.segments) {
    if (find_segment(graph, segment.id) == nullptr || !is_finite(segment.length_m) ||
        segment.length_m <= 0.0 || segment.alignment.spans.empty()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road derived segment invariant failed");
    }
    for (const SectionEvaluation& section : segment.sections) {
      if (section.segment_id != segment.id) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road section owner invariant failed");
      }
      double previous = -std::numeric_limits<double>::infinity();
      for (const SectionBoundarySample& boundary : section.boundaries) {
        if (!is_finite(boundary.lateral_m) || !is_finite(boundary.height_m) || boundary.lateral_m < previous) {
          return Result<bool>::Fail(CommitFailureCategory::kInternalError, "section boundary order invariant failed");
        }
        previous = boundary.lateral_m;
      }
    }
  }

  for (const DerivedMarking& marking : derived.markings) {
    const bool owner_exists =
        marking.owner.kind == MarkingOwner::Kind::kRoadSegment
            ? find_segment(graph, marking.owner.segment_id) != nullptr
            : (marking.owner.kind == MarkingOwner::Kind::kJunction
                   ? find_node(graph, marking.owner.node_id) != nullptr
                   : marking.owner.manual_id != 0);
    if (!owner_exists || !IsKnownMarkingStyle(marking.style_id) ||
        !is_finite(marking.width_m) || marking.width_m <= 0.0 ||
        (marking.points.empty() && marking.polygon.empty())) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road derived marking invariant failed");
    }
  }

  for (const NodeConnectionPolicyOverride& policy : graph.connection_policy_overrides) {
    if (!ids.insert(policy.id).second || find_node(graph, policy.node_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road connection policy override invariant failed");
    }
  }
  for (const ApproachGeometryOverride& override : graph.approach_geometry_overrides) {
    const RoadSegment* segment = find_segment(graph, override.key.segment_id);
    const bool endpoint_matches =
        segment != nullptr &&
        ((override.key.endpoint_role == EndpointRole::kStart &&
          segment->node_a == override.key.node_id) ||
         (override.key.endpoint_role == EndpointRole::kEnd &&
          segment->node_b == override.key.node_id));
    if (!endpoint_matches ||
        (!override.setback_m.has_value && !override.lateral_shift_m.has_value) ||
        (override.setback_m.has_value &&
         (!is_finite(override.setback_m.value) || override.setback_m.value < 0.0)) ||
        (override.lateral_shift_m.has_value &&
         !is_finite(override.lateral_shift_m.value))) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road approach geometry override invariant failed");
    }
  }
  for (const RoadLayoutTransition& transition : graph.transitions) {
    if (!ids.insert(transition.id).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road transition ID invariant failed");
    }
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "manual line ID or owner invariant failed");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "manual area ID or owner invariant failed");
    }
  }
  const auto valid_mesh = [](const Mesh& mesh) {
    for (const Vec3d& vertex : mesh.vertices) {
      if (!is_finite(vertex.x) || !is_finite(vertex.y) || !is_finite(vertex.z)) {
        return false;
      }
    }
    for (std::uint32_t index : mesh.indices) {
      if (index >= mesh.vertices.size()) return false;
    }
    return true;
  };
  const std::array<const std::vector<Mesh>*, 4> mesh_families{
      &derived.segment_meshes, &derived.marking_meshes, &derived.connection_meshes,
      &derived.junction_meshes};
  for (const auto* family : mesh_families) {
    if (std::any_of(family->begin(), family->end(), [&valid_mesh](const Mesh& mesh) { return !valid_mesh(mesh); })) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road mesh contains invalid geometry");
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road
