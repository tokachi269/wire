#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "wire/core/core_view.hpp"
#include "backbone_pipeline.hpp"
#include "../build_backbone/build_backbone_types.hpp"
#include "../build_backbone/build_span_layout_rules.hpp"
#include "../pole_facing/pole_facing_rules.hpp"
#include "../detail_utils.hpp"
#include "../bundle_spans/bundle_span_context.hpp"
#include "../bundle_spans/build_endpoint_heights.hpp"
#include "../bundle_spans/build_endpoint_directions.hpp"
#include "../support_policy.hpp"
#include "../../pole_orientation_utils.hpp"
#include "../../support_orientation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {

struct SpanFeasibility {
  std::unordered_map<ObjectId, JunctionFeasibility> feasibility_by_node{};
};

struct BundleGapSegment {
  std::size_t segment_index = 0;
  int missing_count = 0;
};

struct BundleGap {
  BackboneBundlePlan bundle_plan{};
  int missing_total = 0;
  std::size_t first_missing_segment = std::numeric_limits<std::size_t>::max();
  std::vector<BundleGapSegment> missing_segments{};

  [[nodiscard]] bool requires_allocation() const {
    return missing_total > 0;
  }

  [[nodiscard]] bool has_generation_start() const {
    return first_missing_segment != std::numeric_limits<std::size_t>::max();
  }
};

struct BundleAllocation {
  BackboneBundlePlan bundle_plan{};
  ObjectId bundle_id = kInvalidObjectId;
  ChangeSet change_set{};
};

struct SpanRun {
  std::size_t segment_start_index = 0;
  std::size_t segment_end_index = 0;
  std::vector<ObjectId> ordered_support_node_ids{};
  std::uint64_t variation_flow_key = 0;
  BackboneLoweringPolicy lowering_policy{};
};

struct GroupedSpanBuildOutput {
  ChangeSet change_set{};
  std::vector<ObjectId> span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct SpanSeedRules {
  std::vector<SpanSupportLayoutDecisionSeed> support_layout_seeds{};
};

struct GeneratedSpanMetadata {
  ChangeSet change_set{};
  std::vector<ObjectId> generated_span_ids{};
};

struct BundleSpanRunOutput {
  ChangeSet change_set{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::vector<SpanSupportLayoutDecisionSeed> support_layout_seeds{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

Vec3d normalize_forward_xy_commit(const Vec3d& value) {
  Vec3d out{value.x, value.y, 0.0};
  if (!Normalize(&out)) {
    return {};
  }
  return out;
}

Vec3d choose_continuous_axis_commit(const Vec3d& axis, const Vec3d& previous_forward) {
  Vec3d out = axis;
  if (!Normalize(&out)) {
    return {};
  }
  Vec3d prev = normalize_forward_xy_commit(previous_forward);
  if (Dot(out, prev) < 0.0) {
    out = ScaleVec(out, -1.0);
  }
  return out;
}

std::uint64_t splitmix64_commit(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine_commit(std::uint64_t seed, std::uint64_t value) {
  return splitmix64_commit(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

std::uint64_t make_flow_variation_key_commit(std::uint64_t generation_session_id, BundleKind bundle_template_id,
                                             BackboneFlowKind flow_kind, ObjectId run_start_node_id,
                                             ObjectId run_end_node_id) {
  std::uint64_t key = hash_combine_commit(generation_session_id, static_cast<std::uint64_t>(bundle_template_id));
  key = hash_combine_commit(key, static_cast<std::uint64_t>(flow_kind));
  key = hash_combine_commit(key, static_cast<std::uint64_t>(run_start_node_id));
  key = hash_combine_commit(key, static_cast<std::uint64_t>(run_end_node_id));
  return key;
}

SupportLayoutDecisionSeedEndpoint build_seed_endpoint_record_commit(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    const SegmentLaneAssignment& assignment, ObjectId endpoint_node_id, const Port& port,
    const EndpointContinuityDecision& decision) {
  return build_seed_endpoint_from_decision(edit_state, junction_relations_by_node, assignment, endpoint_node_id, port,
                                           decision);
}

void append_seed_support_group_decision(const CoreState& state, const Port& port,
                                               const SupportLayoutDecisionSeedEndpoint& endpoint,
                                               SpanSupportLayoutDecisionSeed* layout) {
  (void)port;
  if (layout == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
    return;
  }
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
  if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return;
  }
  const Pole* pole = state.view().poles().find(key.owner_pole_id);
  if (pole == nullptr) {
    return;
  }
  auto [it, inserted] = layout->support_group_decisions.try_emplace(key);
  if (inserted) {
    SupportGroupDecision& group = it->second;
    static_cast<SupportLayoutSemanticDecision&>(group) = endpoint;
    group.owner_pole_id = endpoint.owner_pole_id;
    group.support_group_id = endpoint.support_group_id;
    group.support_authority = endpoint.support_authority;
    group.side = endpoint.side;
    group.origin = endpoint.origin;
    group.order_decision_policy = endpoint.order_decision_policy;
    group.order_decision_choice = endpoint.order_decision_choice;
    group.order_decision_choice_reason = endpoint.order_decision_choice_reason;
    group.chosen_side = endpoint.chosen_side;
    group.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
  }
}

SpanSupportLayoutDecisionSeed build_seed_layout_record(
    const CoreState& state, ObjectId span_id, BackboneFlowKind flow_kind, CurvePassMode pass_mode,
    std::uint64_t variation_flow_key, BackboneLoweringKind lowering_kind, const Port& port_a, const Port& port_b,
    SupportLayoutDecisionSeedEndpoint start, SupportLayoutDecisionSeedEndpoint end) {
  SpanSupportLayoutDecisionSeed layout{};
  layout.span_id = span_id;
  layout.flow_kind = flow_kind;
  layout.pass_mode = pass_mode;
  layout.variation_flow_key = variation_flow_key;
  layout.lowering_kind = lowering_kind;
  layout.start = std::move(start);
  layout.end = std::move(end);
  append_seed_support_group_decision(state, port_a, layout.start, &layout);
  append_seed_support_group_decision(state, port_b, layout.end, &layout);
  return layout;
}

std::vector<SpanSupportLayoutDecisionSeed> build_seed_generated_support_layouts(
    const CoreState& state, const EditState& edit_state, const std::vector<ObjectId>& span_ids,
    const std::vector<SegmentLaneAssignment>& lane_assignments,
    const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node, std::uint64_t variation_flow_key) {
  std::vector<SpanSupportLayoutDecisionSeed> layouts{};
  layouts.reserve(span_ids.size());
  std::size_t span_index = 0;
  for (const SegmentLaneAssignment& assignment : lane_assignments) {
    const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
    for (std::size_t lane = 0; lane < lane_count && span_index < span_ids.size(); ++lane, ++span_index) {
      const ObjectId span_id = span_ids[span_index];
      const Span* span = edit_state.spans.find(span_id);
      const Port* port_a = edit_state.ports.find(assignment.port_ids_a[lane]);
      const Port* port_b = edit_state.ports.find(assignment.port_ids_b[lane]);
      if (span == nullptr || port_a == nullptr || port_b == nullptr) {
        continue;
      }
      const CurvePassMode pass_mode =
          (span->placement_context == ConnectionContext::kBranchAdd)
              ? CurvePassMode::kBranch
              : ((span->placement_context == ConnectionContext::kCornerPass) ? CurvePassMode::kPassThrough
                                                                             : CurvePassMode::kPassThrough);
      layouts.push_back(build_seed_layout_record(
          state, span_id, assignment.flow_kind, pass_mode, variation_flow_key, assignment.lowering_kind, *port_a, *port_b,
          build_seed_endpoint_record_commit(edit_state, junction_relations_by_node, assignment, span->endpoint_node_a_id,
                                            *port_a, assignment.decision_a),
          build_seed_endpoint_record_commit(edit_state, junction_relations_by_node, assignment, span->endpoint_node_b_id,
                                            *port_b, assignment.decision_b)));
    }
  }
  return layouts;
}

JunctionRelation remap_junction_relation_commit(const JunctionRelation& relation,
                                                const std::function<ObjectId(ObjectId)>& remap_node_id) {
  JunctionRelation remapped = relation;
  remapped.node_id = remap_node_id(remapped.node_id);
  remapped.primary_neighbor_id = remap_node_id(remapped.primary_neighbor_id);
  remapped.through_pair.neighbor_a_id = remap_node_id(remapped.through_pair.neighbor_a_id);
  remapped.through_pair.neighbor_b_id = remap_node_id(remapped.through_pair.neighbor_b_id);
  for (JunctionIncidentRelation& incident : remapped.incidents) {
    incident.neighbor_node_id = remap_node_id(incident.neighbor_node_id);
  }
  return remapped;
}

JunctionRoles remap_junction_roles(const JunctionRoles& roles, const std::function<ObjectId(ObjectId)>& remap_node_id) {
  JunctionRoles remapped{};
  remapped.edge_flow_by_segment = roles.edge_flow_by_segment;
  remapped.ordered.reserve(roles.ordered.size());
  for (const JunctionRelation& relation : roles.ordered) {
    remapped.ordered.push_back(remap_junction_relation_commit(relation, remap_node_id));
  }
  for (const auto& [node_id, relation] : roles.by_node) {
    remapped.by_node.emplace(remap_node_id(node_id), remap_junction_relation_commit(relation, remap_node_id));
  }
  for (const auto& [node_id, pair] : roles.main_pair_by_node) {
    BackbonePair remapped_pair{};
    remapped_pair.low = remap_node_id(pair.low);
    remapped_pair.high = remap_node_id(pair.high);
    remapped.main_pair_by_node.emplace(remap_node_id(node_id), remapped_pair);
  }
  for (const auto& [node_id, pair] : roles.cross_pair_by_node) {
    BackbonePair remapped_pair{};
    remapped_pair.low = remap_node_id(pair.low);
    remapped_pair.high = remap_node_id(pair.high);
    remapped.cross_pair_by_node.emplace(remap_node_id(node_id), remapped_pair);
  }
  return remapped;
}

double category_same_level_margin_m_commit(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return 0.30;
  case ConnectionCategory::kCommunication:
    return 0.18;
  case ConnectionCategory::kOptical:
    return 0.14;
  case ConnectionCategory::kLowVoltage:
    return 0.12;
  case ConnectionCategory::kDrop:
  default:
    return 0.08;
  }
}

SpanFeasibility evaluate_span_feasibility(
    const BackboneBundlePlan& bundle_plan, const std::unordered_map<ObjectId, JunctionRelation>& base_relations,
    const std::function<Vec3d(ObjectId)>& current_support_position) {
  SpanFeasibility output{};

  const bool bundle_like = bundle_plan.continuity_class == ContinuityCategoryClass::kBundleLike;
  const double envelope_width_m = std::max(0, bundle_plan.count - 1) * std::max(0.0, bundle_plan.spacing_m);
  const double required_clearance_m = envelope_width_m + category_same_level_margin_m_commit(bundle_plan.category);
  const double probe_distance_m = std::clamp(0.18 + envelope_width_m * 0.35 + required_clearance_m * 0.15, 0.18, 0.42);
  auto probe_point_for = [&](ObjectId node_id, ObjectId neighbor_id) {
    const Vec3d center = current_support_position(node_id);
    const Vec3d dir = normalize_forward_xy_commit(current_support_position(neighbor_id) - center);
    if (!std::isfinite(dir.x) || !std::isfinite(dir.y)) {
      return center;
    }
    return center + ScaleVec(dir, probe_distance_m);
  };
  auto projected_spacing_for = [&](ObjectId node_id, ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
    const Vec3d probe_a = probe_point_for(node_id, neighbor_a_id);
    const Vec3d probe_b = probe_point_for(node_id, neighbor_b_id);
    return std::hypot(probe_a.x - probe_b.x, probe_a.y - probe_b.y);
  };

  for (const auto& [node_id, relation] : base_relations) {
    JunctionFeasibility feasibility{};
    feasibility.node_id = node_id;
    feasibility.incidents.reserve(relation.incidents.size());
    for (const JunctionIncidentRelation& incident : relation.incidents) {
      JunctionIncidentFeasibility incident_feasibility{};
      incident_feasibility.neighbor_node_id = incident.neighbor_node_id;
      incident_feasibility.continuity_class = bundle_plan.continuity_class;
      incident_feasibility.default_lower_required = false;
      incident_feasibility.same_level_feasible = true;
      incident_feasibility.projected_spacing_topview_m = -1.0;
      incident_feasibility.required_clearance_m = required_clearance_m;
      incident_feasibility.reason = SameLevelFeasibilityReason::kNone;
      feasibility.incidents.push_back(incident_feasibility);
    }
    output.feasibility_by_node.emplace(node_id, std::move(feasibility));
  }

  for (const auto& [node_id, relation] : base_relations) {
    auto it_feasibility = output.feasibility_by_node.find(node_id);
    if (it_feasibility == output.feasibility_by_node.end()) {
      continue;
    }
    JunctionFeasibility& node_feasibility = it_feasibility->second;
    if (!bundle_like) {
      continue;
    }
    if (relation.incidents.size() < 2) {
      continue;
    }

    for (const JunctionIncidentRelation& incident : relation.incidents) {
      const bool skip_through_pair_main = incident.kind == JunctionRelationKind::kThroughMain && incident.in_through_pair;
      if (skip_through_pair_main) {
        continue;
      }
      const bool is_non_through_relation =
          incident.kind != JunctionRelationKind::kThroughMain && incident.kind != JunctionRelationKind::kNone;
      const bool allow_non_route_bundle_lower =
          bundle_like &&
          (incident.kind == JunctionRelationKind::kCornerContinuation ||
           incident.kind == JunctionRelationKind::kCrossUnderpass);
      if (!incident.in_route && !allow_non_route_bundle_lower) {
        continue;
      }
      auto it_incident = std::find_if(node_feasibility.incidents.begin(), node_feasibility.incidents.end(),
                                      [&](const JunctionIncidentFeasibility& candidate) {
                                        return candidate.neighbor_node_id == incident.neighbor_node_id;
                                      });
      if (it_incident == node_feasibility.incidents.end()) {
        continue;
      }
      if (bundle_like && is_non_through_relation && (incident.in_route || allow_non_route_bundle_lower)) {
        it_incident->default_lower_required = true;
        it_incident->same_level_feasible = false;
        it_incident->reason = SameLevelFeasibilityReason::kBundleRule;
        continue;
      }

      std::vector<ObjectId> comparison_neighbors{};
      if (relation.through_pair.accepted) {
        if (relation.through_pair.neighbor_a_id != kInvalidObjectId &&
            relation.through_pair.neighbor_a_id != incident.neighbor_node_id) {
          comparison_neighbors.push_back(relation.through_pair.neighbor_a_id);
        }
        if (relation.through_pair.neighbor_b_id != kInvalidObjectId &&
            relation.through_pair.neighbor_b_id != incident.neighbor_node_id) {
          comparison_neighbors.push_back(relation.through_pair.neighbor_b_id);
        }
      }
      if (comparison_neighbors.empty()) {
        for (const JunctionIncidentRelation& other : relation.incidents) {
          if (other.neighbor_node_id == incident.neighbor_node_id || !other.in_route) {
            continue;
          }
          comparison_neighbors.push_back(other.neighbor_node_id);
        }
      }
      if (comparison_neighbors.empty()) {
        continue;
      }

      double min_projected_spacing_m = std::numeric_limits<double>::infinity();
      for (ObjectId other_neighbor_id : comparison_neighbors) {
        min_projected_spacing_m =
            std::min(min_projected_spacing_m, projected_spacing_for(node_id, incident.neighbor_node_id, other_neighbor_id));
      }
      it_incident->projected_spacing_topview_m = min_projected_spacing_m;
      it_incident->required_clearance_m = required_clearance_m;
      if (!(std::isfinite(min_projected_spacing_m) && min_projected_spacing_m + 1e-9 >= required_clearance_m)) {
        it_incident->same_level_feasible = false;
        it_incident->reason =
            (std::isfinite(min_projected_spacing_m) && min_projected_spacing_m + 1e-9 < envelope_width_m)
                ? SameLevelFeasibilityReason::kEnvelopeOverlap
                : SameLevelFeasibilityReason::kNearNodeClearance;
      }
    }
  }

  return output;
}

BundleGap find_bundle_gaps(
    const BackboneBundlePlan& bundle_plan, const std::vector<ObjectId>& ordered_support_node_ids,
    const std::function<int(ObjectId, ObjectId, const BackboneBundlePlan&)>& count_existing_segment_spans) {
  BundleGap analysis{};
  analysis.bundle_plan = bundle_plan;
  for (std::size_t i = 0; i + 1 < ordered_support_node_ids.size(); ++i) {
    const int existing_count = count_existing_segment_spans(ordered_support_node_ids[i], ordered_support_node_ids[i + 1],
                                                            bundle_plan);
    const int missing = std::max(0, bundle_plan.count - existing_count);
    if (missing <= 0) {
      continue;
    }
    analysis.missing_total += missing;
    analysis.missing_segments.push_back({i, missing});
    if (!analysis.has_generation_start()) {
      analysis.first_missing_segment = i;
    }
  }
  return analysis;
}

EditResult<BundleAllocation> allocate_bundle_for_gap(
    const BundleGap& gap_analysis,
    const std::function<EditResult<ObjectId>(const BackboneBundlePlan&)>& create_bundle) {
  EditResult<BundleAllocation> result{};
  BundleAllocation allocation{};
  allocation.bundle_plan = gap_analysis.bundle_plan;
  if (!gap_analysis.requires_allocation()) {
    result.value = std::move(allocation);
    result.ok = true;
    return result;
  }

  EditResult<ObjectId> bundle_result = create_bundle(gap_analysis.bundle_plan);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }
  allocation.bundle_id = bundle_result.value;
  allocation.change_set = std::move(bundle_result.change_set);
  result.value = std::move(allocation);
  result.ok = true;
  return result;
}

std::vector<SpanRun> plan_span_runs(
    const BundleGap& gap_analysis, const JunctionRoles& roles,
    const std::vector<ObjectId>& ordered_support_node_ids, const SpanFeasibility& feasibility_phase,
    std::uint64_t session_id) {
  std::vector<SpanRun> runs{};
  if (!gap_analysis.has_generation_start() || gap_analysis.first_missing_segment >= ordered_support_node_ids.size() - 1) {
    return runs;
  }

  for (std::size_t run_start = gap_analysis.first_missing_segment; run_start + 1 < ordered_support_node_ids.size();) {
    const EdgeFlowInfo flow_info = roles.edge_flow_by_segment[run_start];
    std::size_t run_end = run_start;
    while (run_end + 1 < ordered_support_node_ids.size() - 1 &&
           roles.edge_flow_by_segment[run_end + 1].kind == flow_info.kind) {
      ++run_end;
    }

    SpanRun run{};
    run.segment_start_index = run_start;
    run.segment_end_index = run_end;
    run.ordered_support_node_ids.insert(run.ordered_support_node_ids.end(),
                                        ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(run_start),
                                        ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(run_end + 2));
    run.variation_flow_key = make_flow_variation_key_commit(session_id, gap_analysis.bundle_plan.template_id, flow_info.kind,
                                                            run.ordered_support_node_ids.front(),
                                                            run.ordered_support_node_ids.back());

    bool relation_has_lowering_candidate = false;
    for (std::size_t node_offset = run_start; node_offset <= run_end + 1; ++node_offset) {
      if (node_offset >= ordered_support_node_ids.size()) {
        continue;
      }
      const ObjectId node_id = ordered_support_node_ids[node_offset];
      const auto it_relation = feasibility_phase.feasibility_by_node.find(node_id);
      if (it_relation == feasibility_phase.feasibility_by_node.end()) {
        continue;
      }
      const auto decision_it = roles.by_node.find(node_id);
      if (decision_it == roles.by_node.end()) {
        continue;
      }
      for (const JunctionIncidentFeasibility& incident : it_relation->second.incidents) {
        if (incident.same_level_feasible) {
          continue;
        }
        auto incident_decision = std::find_if(decision_it->second.incidents.begin(), decision_it->second.incidents.end(),
                                              [&](const JunctionIncidentRelation& candidate) {
                                                return candidate.neighbor_node_id == incident.neighbor_node_id;
                                              });
        if (incident_decision == decision_it->second.incidents.end() || !incident_decision->in_route) {
          continue;
        }
        if (incident_decision->kind != JunctionRelationKind::kThroughMain &&
            incident_decision->kind != JunctionRelationKind::kNone) {
          relation_has_lowering_candidate = true;
        }
      }
    }

    const bool enable_uniform_lowering = gap_analysis.bundle_plan.enable_branch_down_offset && relation_has_lowering_candidate;
    run.lowering_policy.enable_cross_underpass = enable_uniform_lowering;
    run.lowering_policy.enable_branch_support = enable_uniform_lowering;
    run.lowering_policy.enable_acute_corner = enable_uniform_lowering;
    run.lowering_policy.offset_m =
        enable_uniform_lowering ? BranchDownOffsetForCategory(gap_analysis.bundle_plan.category) : 0.0;
    runs.push_back(std::move(run));
    run_start = run_end + 1;
  }

  return runs;
}

} // namespace

EditResult<bool> CoreState::build_real_node_topology_state(
    const BackboneTopologyPlan& topology_plan, std::uint64_t session_id,
    const std::unordered_map<ObjectId, SupportNode>& support_node_by_id,
    const std::unordered_map<ObjectId, ObjectId>& real_node_id_by_input_node_id,
    BackboneRuntimeTopology* out_state) const {
  EditResult<bool> result{};
  if (out_state == nullptr) {
    result.error = "backbone runtime topology output is null";
    return result;
  }

  auto remap_node_id = [&](ObjectId node_id) {
    const auto it = real_node_id_by_input_node_id.find(node_id);
    return (it == real_node_id_by_input_node_id.end()) ? node_id : it->second;
  };

  out_state->generation_backbone = topology_plan.generation_backbone;
  out_state->route_neighbors_by_node.clear();
  out_state->route_neighbors_by_node.reserve(topology_plan.route_neighbors_by_node.size());
  for (const auto& [node_id, neighbors] : topology_plan.route_neighbors_by_node) {
    std::vector<ObjectId>& remapped_neighbors = out_state->route_neighbors_by_node[remap_node_id(node_id)];
    remapped_neighbors.reserve(neighbors.size());
    for (ObjectId neighbor_id : neighbors) {
      remapped_neighbors.push_back(remap_node_id(neighbor_id));
    }
    std::sort(remapped_neighbors.begin(), remapped_neighbors.end());
    remapped_neighbors.erase(std::unique(remapped_neighbors.begin(), remapped_neighbors.end()), remapped_neighbors.end());
  }

  out_state->node_position_by_id = topology_plan.existing_node_position_by_id;
  for (const auto& [node_id, node] : support_node_by_id) {
    out_state->node_position_by_id[node_id] = node.position;
  }
  out_state->existing_prioritized_session_by_node = topology_plan.existing_prioritized_session_by_node;
  out_state->existing_incident_session_by_node = topology_plan.existing_incident_session_by_node;

  for (BackboneEdge& edge : out_state->generation_backbone.edges) {
    edge.node_a = remap_node_id(edge.node_a);
    edge.node_b = remap_node_id(edge.node_b);
    if (edge.node_b < edge.node_a) {
      std::swap(edge.node_a, edge.node_b);
    }
  }
  std::sort(out_state->generation_backbone.edges.begin(), out_state->generation_backbone.edges.end(),
            [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
              if (lhs.node_a != rhs.node_a) {
                return lhs.node_a < rhs.node_a;
              }
              return lhs.node_b < rhs.node_b;
            });
  out_state->generation_backbone.edges.erase(
      std::unique(out_state->generation_backbone.edges.begin(), out_state->generation_backbone.edges.end(),
                  [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
                    return lhs.node_a == rhs.node_a && lhs.node_b == rhs.node_b;
                  }),
      out_state->generation_backbone.edges.end());

  for (SupportNode& node : out_state->generation_backbone.nodes) {
    const ObjectId remapped_node_id = remap_node_id(node.node_id);
    node.node_id = remapped_node_id;
    if (node.pole_id != kInvalidObjectId) {
      node.pole_id = remap_node_id(node.pole_id);
    }
    if (const auto it = support_node_by_id.find(remapped_node_id); it != support_node_by_id.end()) {
      node.position = it->second.position;
      node.pole_id = it->second.pole_id;
      node.support_kind = it->second.support_kind;
      node.path_point_index = it->second.path_point_index;
      node.has_tangent_hint = it->second.has_tangent_hint;
      node.tangent_hint = it->second.tangent_hint;
    }
  }

  for (JunctionInfo& junction : out_state->generation_backbone.junctions) {
    junction.node_id = remap_node_id(junction.node_id);
    if (junction.prioritized_session_id == 0) {
      junction.prioritized_session_id = session_id;
    }
    for (JunctionIncident& incident : junction.incidents) {
      incident.neighbor_node_id = remap_node_id(incident.neighbor_node_id);
      if (incident.source_session_id == 0) {
        incident.source_session_id = session_id;
      }
    }
  }

  result.value = true;
  result.ok = true;
  return result;
}

EditResult<BackboneRuntimeState> CoreState::remap_backbone_build_to_real_nodes(
    const BackboneTopologyPlan& topology_plan, const JunctionRoles& roles, const PoleFacing& pole_facing, std::uint64_t session_id,
    std::vector<ObjectId> ordered_support_node_ids, std::unordered_map<ObjectId, SupportNode> support_node_by_id,
    std::unordered_map<ObjectId, ObjectId> real_node_id_by_input_node_id) const {
  EditResult<BackboneRuntimeState> result{};
  BackboneRuntimeState runtime{};
  runtime.session_id = session_id;
  runtime.ordered_support_node_ids = std::move(ordered_support_node_ids);
  runtime.support_node_by_id = std::move(support_node_by_id);
  runtime.real_node_id_by_input_node_id = std::move(real_node_id_by_input_node_id);
  runtime.roles = roles;
  runtime.pole_facing = pole_facing;
  if (runtime.ordered_support_node_ids.size() < 2) {
    result.error = "failed to build valid support-node chain";
    return result;
  }

  EditResult<bool> topology_state_result = build_real_node_topology_state(
      topology_plan, session_id, runtime.support_node_by_id, runtime.real_node_id_by_input_node_id, &runtime.topology);
  if (!topology_state_result.ok) {
    result.error = topology_state_result.error;
    return result;
  }

  auto remap_node_id = [&](ObjectId node_id) {
    const auto it = runtime.real_node_id_by_input_node_id.find(node_id);
    return (it == runtime.real_node_id_by_input_node_id.end()) ? node_id : it->second;
  };
  runtime.roles = remap_junction_roles(roles, remap_node_id);
  runtime.pole_facing.by_node.clear();
  runtime.pole_facing.by_node.reserve(pole_facing.by_node.size());
  for (const auto& [node_id, orientation] : pole_facing.by_node) {
    const ObjectId remapped_node_id = remap_node_id(node_id);
    BackbonePlannedPoleOrientation remapped = orientation;
    remapped.debug.pole_id = remapped_node_id;
    remapped.debug.primary_neighbor_id = remap_node_id(remapped.debug.primary_neighbor_id);
    remapped.debug.secondary_neighbor_id = remap_node_id(remapped.debug.secondary_neighbor_id);
    runtime.pole_facing.by_node.emplace(remapped_node_id, std::move(remapped));
  }
  result.value = std::move(runtime);
  result.ok = true;
  return result;
}

EditResult<GeneratedBackboneSpans> CoreState::build_bundle_spans_for_backbone(
    const BackboneGenerationRequestPlan& request_plan, const BackboneRuntimeState& runtime) {
  EditResult<GeneratedBackboneSpans> phase_result{};
  const std::vector<BackboneBundlePlan>& active_bundle_plans = request_plan.active_bundle_plans;
  const std::uint64_t session_id = runtime.session_id;
  const auto& ordered_support_node_ids = runtime.ordered_support_node_ids;
  const auto& support_node_by_id = runtime.support_node_by_id;
  const auto& topology_state = runtime.topology;
  const auto& existing_node_position_by_id = topology_state.node_position_by_id;

  auto current_support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state_access().poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    if (const auto it = existing_node_position_by_id.find(node_id); it != existing_node_position_by_id.end()) {
      return it->second;
    }
    return {};
  };
  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };
  auto count_existing_segment_spans = [&](ObjectId node_a, ObjectId node_b, const BackboneBundlePlan& bundle_plan) -> int {
    int count = 0;
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.layer != bundle_plan.layer || span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != bundle_plan.template_id) {
        continue;
      }
      const Port* port_a = edit_state_access().ports.find(span.port_a_id);
      const Port* port_b = edit_state_access().ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      const ObjectId span_node_a = resolve_span_endpoint_node(span, port_a, true);
      const ObjectId span_node_b = resolve_span_endpoint_node(span, port_b, false);
      const bool direct = (span_node_a == node_a && span_node_b == node_b);
      const bool reverse = (span_node_a == node_b && span_node_b == node_a);
      if (direct || reverse) {
        ++count;
      }
    }
    return count;
  };
  auto generate_grouped_spans_for_run =
      [&](const BundleAllocation& allocation, const SpanRun& run,
          const std::unordered_map<ObjectId, JunctionRelation>& current_relations,
          const std::unordered_map<ObjectId, JunctionFeasibility>& current_feasibility)
      -> EditResult<GroupedSpanBuildOutput> {
    EditResult<GroupedSpanBuildOutput> result{};
    GroupedSpanBuildOutput output{};
    output.junction_relations_by_node = current_relations;

    const EdgeFlowInfo flow_info = runtime.roles.edge_flow_by_segment[run.segment_start_index];
    EditResult<std::vector<ObjectId>> spans_result = generate_grouped_spans_between_support_nodes(
        run.ordered_support_node_ids, support_node_by_id, allocation.bundle_id, allocation.bundle_plan.category,
        allocation.bundle_plan.count, allocation.bundle_plan.spacing_m, true, allocation.bundle_plan.allow_mirror,
        allocation.bundle_plan.order_decision_policy, flow_info.kind, run.lowering_policy, &output.junction_relations_by_node,
        &current_feasibility,
        &output.lane_assignments, &output.edge_orientations, allocation.bundle_plan.template_id);
    if (!spans_result.ok) {
      result.error = spans_result.error;
      return result;
    }
    output.span_ids = std::move(spans_result.value);
    append_change_set(output.change_set, spans_result.change_set);
    for (std::size_t i = 0; i < output.lane_assignments.size(); ++i) {
      output.lane_assignments[i].segment_index += run.segment_start_index;
      output.lane_assignments[i].variation_flow_key = run.variation_flow_key;
      output.lane_assignments[i].flow_decision_rule = runtime.roles.edge_flow_by_segment[run.segment_start_index + i].rule;
    }
    for (std::size_t i = 0; i < output.edge_orientations.size(); ++i) {
      output.edge_orientations[i].variation_flow_key = run.variation_flow_key;
      output.edge_orientations[i].flow_decision_rule = runtime.roles.edge_flow_by_segment[run.segment_start_index + i].rule;
    }

    result.value = std::move(output);
    result.ok = true;
    return result;
  };
  auto build_support_layout_seed_authority_phase =
      [&](const GroupedSpanBuildOutput& grouped_span_phase, const SpanRun& run)
      -> SpanSeedRules {
    SpanSeedRules output{};
    output.support_layout_seeds = build_seed_generated_support_layouts(
        *this, edit_state_access(), grouped_span_phase.span_ids, grouped_span_phase.lane_assignments,
        grouped_span_phase.junction_relations_by_node, run.variation_flow_key);
    return output;
  };
  auto apply_generated_span_metadata_phase =
      [&](const BundleAllocation& allocation, const GroupedSpanBuildOutput& grouped_span_phase,
          const SpanRun& run, std::size_t generation_order_offset)
      -> GeneratedSpanMetadata {
    GeneratedSpanMetadata output{};
    for (std::size_t i = 0; i < grouped_span_phase.span_ids.size(); ++i) {
      const ObjectId span_id = grouped_span_phase.span_ids[i];
      Span* span = edit_state_access().spans.find(span_id);
      if (span != nullptr) {
        span->layer = allocation.bundle_plan.layer;
        span->generation.generated = true;
        span->generation.source = GenerationSource::kRoadAuto;
        span->generation.generation_session_id = session_id;
        span->generation.generation_order =
            static_cast<std::uint32_t>(generation_order_offset + output.generated_span_ids.size());
        span->generated_by_rule = true;
        add_unique_id(output.change_set.updated_ids, span->id);
      }
      auto runtime_it = span_runtime_states_access().find(span_id);
      if (runtime_it != span_runtime_states_access().end()) {
        runtime_it->second.variation_flow_key = run.variation_flow_key;
      }
      output.generated_span_ids.push_back(span_id);
    }
    return output;
  };
  auto build_bundle_span_run =
      [&](const BundleAllocation& allocation, const BundleGap& gap_analysis,
          const SpanFeasibility& feasibility_phase, std::size_t generation_order_offset)
      -> EditResult<BundleSpanRunOutput> {
    EditResult<BundleSpanRunOutput> result{};
    BundleSpanRunOutput output{};
    output.junction_relations_by_node = runtime.roles.by_node;

    const std::vector<SpanRun> runs = plan_span_runs(
        gap_analysis, runtime.roles, ordered_support_node_ids, feasibility_phase, session_id);
    for (const SpanRun& run : runs) {
      EditResult<GroupedSpanBuildOutput> grouped_span_result =
          generate_grouped_spans_for_run(allocation, run, output.junction_relations_by_node,
                                         feasibility_phase.feasibility_by_node);
      if (!grouped_span_result.ok) {
        result.error = grouped_span_result.error;
        return result;
      }
      GroupedSpanBuildOutput grouped_span_phase = std::move(grouped_span_result.value);
      append_change_set(output.change_set, grouped_span_phase.change_set);
      output.lane_assignments.insert(output.lane_assignments.end(), grouped_span_phase.lane_assignments.begin(),
                                     grouped_span_phase.lane_assignments.end());
      output.edge_orientations.insert(output.edge_orientations.end(), grouped_span_phase.edge_orientations.begin(),
                                      grouped_span_phase.edge_orientations.end());

      SpanSeedRules seed_authority_phase =
          build_support_layout_seed_authority_phase(grouped_span_phase, run);
      output.support_layout_seeds.insert(output.support_layout_seeds.end(),
                                         std::make_move_iterator(seed_authority_phase.support_layout_seeds.begin()),
                                         std::make_move_iterator(seed_authority_phase.support_layout_seeds.end()));

      GeneratedSpanMetadata metadata_phase =
          apply_generated_span_metadata_phase(allocation, grouped_span_phase, run,
                                              generation_order_offset + output.generated_span_ids.size());
      append_change_set(output.change_set, metadata_phase.change_set);
      output.generated_span_ids.insert(output.generated_span_ids.end(), metadata_phase.generated_span_ids.begin(),
                                       metadata_phase.generated_span_ids.end());
    }

    result.value = std::move(output);
    result.ok = true;
    return result;
  };

  phase_result.value.junctions = runtime.roles.by_node;
  for (const BackboneBundlePlan& bundle_plan : active_bundle_plans) {
    const BundleGap gap_analysis = find_bundle_gaps(
        bundle_plan, ordered_support_node_ids, count_existing_segment_spans);
    if (!gap_analysis.requires_allocation()) {
      continue;
    }

    EditResult<BundleAllocation> allocation_result = allocate_bundle_for_gap(
        gap_analysis, [&](const BackboneBundlePlan& plan_to_allocate) {
          return AddBundle(plan_to_allocate.count, plan_to_allocate.spacing_m, plan_to_allocate.template_id);
        });
    if (!allocation_result.ok) {
      phase_result.error = allocation_result.error;
      return phase_result;
    }
    BundleAllocation allocation = std::move(allocation_result.value);
    append_change_set(phase_result.value.change_set, allocation.change_set);
    phase_result.value.bundle_ids.push_back(allocation.bundle_id);
    if (phase_result.value.primary_bundle_id == kInvalidObjectId) {
      phase_result.value.primary_bundle_id = allocation.bundle_id;
    }

    if (!gap_analysis.has_generation_start() || gap_analysis.first_missing_segment >= ordered_support_node_ids.size() - 1) {
      continue;
    }
    const SpanFeasibility feasibility_phase = evaluate_span_feasibility(
        bundle_plan, phase_result.value.junctions, current_support_position);
    EditResult<BundleSpanRunOutput> span_phase_result = build_bundle_span_run(
        allocation, gap_analysis, feasibility_phase, phase_result.value.generated_span_ids.size());
    if (!span_phase_result.ok) {
      phase_result.error = span_phase_result.error;
      return phase_result;
    }

    BundleSpanRunOutput span_phase = std::move(span_phase_result.value);
    append_change_set(phase_result.value.change_set, span_phase.change_set);
    phase_result.value.lane_assignments.insert(phase_result.value.lane_assignments.end(), span_phase.lane_assignments.begin(),
                                               span_phase.lane_assignments.end());
    phase_result.value.edge_orientations.insert(phase_result.value.edge_orientations.end(),
                                                span_phase.edge_orientations.begin(), span_phase.edge_orientations.end());
    phase_result.value.generated_span_ids.insert(phase_result.value.generated_span_ids.end(),
                                                 span_phase.generated_span_ids.begin(), span_phase.generated_span_ids.end());
    for (SpanSupportLayoutDecisionSeed& layout_seed : span_phase.support_layout_seeds) {
      cache_span_support_layout_seed(std::move(layout_seed));
    }
  }

  phase_result.ok = true;
  return phase_result;
}

EditResult<RealizedBackboneSupport> CoreState::build_real_backbone_support(
    const BackboneSupportChainPlan& support_chain_plan) {
  EditResult<RealizedBackboneSupport> result{};
  RealizedBackboneSupport realized{};
  realized.session_id = next_generation_session_id_access()++;
  realized.generated_pole_ids.reserve(support_chain_plan.generated_pole_ids.size());
  realized.ordered_support_node_ids.reserve(support_chain_plan.ordered_support_node_ids.size());
  realized.support_node_by_id.reserve(support_chain_plan.support_node_by_id.size());
  realized.real_node_id_by_input_node_id.reserve(support_chain_plan.resolutions.size());

  std::unordered_map<ObjectId, const PlannedPoleCreation*> planned_pole_creation_by_id{};
  planned_pole_creation_by_id.reserve(support_chain_plan.pole_creations.size());
  for (const PlannedPoleCreation& creation : support_chain_plan.pole_creations) {
    planned_pole_creation_by_id[creation.planned_pole_id] = &creation;
  }

  auto authored_support_node_for = [&](ObjectId planned_node_id) -> const SupportNode* {
    const auto it = support_chain_plan.support_node_by_id.find(planned_node_id);
    return (it == support_chain_plan.support_node_by_id.end()) ? nullptr : &it->second;
  };
  auto record_real_support_node = [&](ObjectId planned_node_id, ObjectId real_node_id, ObjectId real_pole_id) {
    const SupportNode* authored_node = authored_support_node_for(planned_node_id);
    if (authored_node == nullptr) {
      result.error = "support-chain plan is missing authored support node";
      return false;
    }
    SupportNode node = *authored_node;
    node.node_id = real_node_id;
    node.pole_id = real_pole_id;
    realized.support_node_by_id[real_node_id] = std::move(node);
    realized.ordered_support_node_ids.push_back(real_node_id);
    realized.real_node_id_by_input_node_id[planned_node_id] = real_node_id;
    return true;
  };

  std::size_t real_pole_order = 0;
  for (const BackboneSupportResolution& resolution : support_chain_plan.resolutions) {
    if (resolution.support_kind != SupportKind::kPole) {
      if (resolution.kind == BackboneSupportResolutionKind::kReuseSupportNode) {
        const auto it_existing = support_chain_plan.support_node_by_id.find(resolution.planned_node_id);
        if (it_existing == support_chain_plan.support_node_by_id.end()) {
          result.error = "support-chain plan is missing reused support node";
          return result;
        }
        realized.support_node_by_id[resolution.planned_node_id] = it_existing->second;
      }
      if (!record_real_support_node(resolution.planned_node_id, resolution.planned_node_id, kInvalidObjectId)) {
        return result;
      }
      continue;
    }

    if (resolution.kind == BackboneSupportResolutionKind::kReusePole) {
      Pole* pole = edit_state_access().poles.find(resolution.existing_node_id);
      if (pole == nullptr) {
        result.error = "support-chain plan references missing reused pole";
        return result;
      }
      const Pole old_pole = *pole;
      pole->world_transform = resolution.authored_transform;
      pole->context = resolution.authored_context;
      apply_pole_placement_mode(*pole, resolution.placement_mode);
      finalize_pole_transform_update(pole->id, old_pole, &realized.change_set);
      if (!record_real_support_node(resolution.planned_node_id, pole->id, pole->id)) {
        return result;
      }
      ++real_pole_order;
      continue;
    }

    if (resolution.kind != BackboneSupportResolutionKind::kCreatePole) {
      result.error = "unsupported support-chain resolution kind";
      return result;
    }

    const auto it_creation = planned_pole_creation_by_id.find(resolution.planned_node_id);
    if (it_creation == planned_pole_creation_by_id.end() || it_creation->second == nullptr) {
      result.error = "support-chain plan is missing planned pole creation";
      return result;
    }
    const PlannedPoleCreation& creation = *it_creation->second;
    EditResult<ObjectId> add_pole = AddPole(creation.transform, creation.height_m, creation.name, creation.pole_kind,
                                            creation.placement_mode);
    if (!add_pole.ok) {
      result.error = add_pole.error;
      return result;
    }
    Pole* pole = edit_state_access().poles.find(add_pole.value);
    if (pole != nullptr) {
      pole->world_transform = creation.transform;
      pole->context = creation.context;
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = realized.session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(real_pole_order);
      add_unique_id(add_pole.change_set.updated_ids, pole->id);
    }
    EditResult<ObjectId> apply_type = ApplyPoleType(add_pole.value, creation.pole_type_id);
    if (!apply_type.ok) {
      result.error = apply_type.error;
      return result;
    }
    append_change_set(realized.change_set, add_pole.change_set);
    append_change_set(realized.change_set, apply_type.change_set);
    realized.generated_pole_ids.push_back(add_pole.value);
    if (!record_real_support_node(resolution.planned_node_id, add_pole.value, add_pole.value)) {
      return result;
    }
    ++real_pole_order;
  }

  std::vector<ObjectId> compact_support_ids{};
  compact_support_ids.reserve(realized.ordered_support_node_ids.size());
  for (ObjectId id : realized.ordered_support_node_ids) {
    if (compact_support_ids.empty() || compact_support_ids.back() != id) {
      compact_support_ids.push_back(id);
    }
  }
  realized.ordered_support_node_ids.swap(compact_support_ids);
  if (realized.ordered_support_node_ids.size() < 2) {
    result.error = "failed to commit valid support-node chain";
    return result;
  }

  result.value = std::move(realized);
  result.ok = true;
  return result;
}

} // namespace wire::core
