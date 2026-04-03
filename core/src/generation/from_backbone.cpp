#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "wire/core/core_view.hpp"
#include "../pole_orientation_utils.hpp"
#include "backbone_prepare.hpp"
#include "detail_utils.hpp"
#include "support_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {

Vec3d normalize_forward_xy(const Vec3d& value) {
  Vec3d out{value.x, value.y, 0.0};
  if (!Normalize(&out)) {
    return {};
  }
  return out;
}

Vec3d choose_continuous_axis(const Vec3d& axis, const Vec3d& previous_forward) {
  Vec3d out = axis;
  if (!Normalize(&out)) {
    return {};
  }
  Vec3d prev = normalize_forward_xy(previous_forward);
  if (Dot(out, prev) < 0.0) {
    out = ScaleVec(out, -1.0);
  }
  return out;
}

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

std::uint64_t make_flow_variation_key(std::uint64_t generation_session_id, BundleKind bundle_template_id,
                                      BackboneFlowKind flow_kind, ObjectId run_start_node_id,
                                      ObjectId run_end_node_id) {
  std::uint64_t key = hash_combine(generation_session_id, static_cast<std::uint64_t>(bundle_template_id));
  key = hash_combine(key, static_cast<std::uint64_t>(flow_kind));
  key = hash_combine(key, static_cast<std::uint64_t>(run_start_node_id));
  key = hash_combine(key, static_cast<std::uint64_t>(run_end_node_id));
  return key;
}

SupportLayoutOriginKind support_layout_origin_from_port_source(PortPlacementSourceKind source) {
  switch (source) {
  case PortPlacementSourceKind::kBranchSupport:
    return SupportLayoutOriginKind::kBranchSupport;
  case PortPlacementSourceKind::kAerialBranch:
    return SupportLayoutOriginKind::kAerialBranch;
  case PortPlacementSourceKind::kPlacementBandConstrained:
    return SupportLayoutOriginKind::kPlacementConstraint;
  case PortPlacementSourceKind::kPlacementBand:
  case PortPlacementSourceKind::kGenerated:
  case PortPlacementSourceKind::kManualEdit:
    return SupportLayoutOriginKind::kMainSupport;
  case PortPlacementSourceKind::kUnknown:
  default:
    return SupportLayoutOriginKind::kFallback;
  }
}

SupportLayoutDecisionSeedEndpoint make_support_layout_seed_endpoint(const Span& span, const Port& port,
                                                                    const SegmentLaneAssignment& assignment,
                                                                    const EndpointContinuityDecision& decision,
                                                                    bool is_start_endpoint) {
  SupportLayoutDecisionSeedEndpoint endpoint{};
  endpoint.endpoint_node_id = is_start_endpoint ? span.endpoint_node_a_id : span.endpoint_node_b_id;
  endpoint.owner_pole_id = port.owner_pole_id;
  endpoint.port_id = port.id;
  endpoint.decision = decision;
  endpoint.decision.owner_pole_id = endpoint.owner_pole_id;
  endpoint.flow_kind = assignment.flow_kind;
  endpoint.origin = support_layout_origin_from_port_source(port.placement_source);
  endpoint.endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  endpoint.port_source = port.placement_source;
  endpoint.side = port.template_side;
  endpoint.endpoint_mode = CurveEndpointMode::kDirectThrough;
  const bool uses_lowering = decision.lower_required && !decision.lowering_blocked_by_policy;
  endpoint.automatic_branch_down_offset_m = uses_lowering ? assignment.branch_down_offset_m : 0.0;
  endpoint.branch_down_offset_m = uses_lowering ? assignment.branch_down_offset_m : 0.0;
  return endpoint;
}

double template_layer_base_z_for_port_category_seed(const CoreState& state, const Pole& pole, ConnectionCategory category) {
  return state.view().port_category_base_z_for_pole(pole, category);
}

void append_seed_support_group_decision(const CoreState& state, const Port& port,
                                        const SupportLayoutDecisionSeedEndpoint& endpoint,
                                        SpanSupportLayoutDecisionSeed* layout) {
  if (layout == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint.decision)) {
    return;
  }
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint.decision);
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
    group.decision = endpoint.decision;
    group.decision.owner_pole_id = endpoint.owner_pole_id;
    group.decision.support_group_id = endpoint.decision.support_group_id;
    group.category = port.category;
    group.side = endpoint.side;
    group.origin = endpoint.origin;
    group.down_offset_m = endpoint.branch_down_offset_m;
    group.down_offset_variation = endpoint.down_offset_variation;
    Vec3d support_world = pole->world_transform.position;
    const double template_support_z = template_layer_base_z_for_port_category_seed(state, *pole, port.category);
    SetHeightAlongWorldUp(&support_world, template_support_z - endpoint.branch_down_offset_m);
    group.support_world = support_world;
  }
  it->second.grouped_port_count += 1;
  it->second.attachment_worlds.push_back(port.world_position);
}

struct BackboneBundlePlan {
  BundleKind template_id = BundleKind::kLowVoltage;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  SpanLayer layer = SpanLayer::kUnknown;
  int count = 1;
  double spacing_m = 0.2;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  bool allow_mirror = true;
  bool preserve_conductor_identity = false;
  bool allow_midair_node = true;
  bool allow_midair_branch = true;
  bool enable_branch_down_offset = false;
};

struct EdgeFlowInfo {
  BackboneFlowKind kind = BackboneFlowKind::kMain;
  BackboneFlowDecisionRule rule = BackboneFlowDecisionRule::kDefaultMain;
  bool underpass_at_cross = false;
};

struct BackboneDecisionPhaseOutput {
  std::vector<EdgeFlowInfo> edge_flow_by_segment{};
  std::vector<JunctionRelation> junction_relations_in_path_order{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct BackboneMaterializationPhaseOutput {
  ChangeSet change_set{};
  ObjectId primary_bundle_id = kInvalidObjectId;
  std::vector<ObjectId> bundle_ids{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct BackboneDecisionPhaseContext {
  const std::vector<ObjectId>& ordered_support_node_ids;
  const std::unordered_map<ObjectId, const JunctionInfo*>& existing_junction_by_node;
  const std::unordered_map<ObjectId, std::uint64_t>& existing_prioritized_session_by_node;
  const std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>>& existing_incident_session_by_node;
  BackboneResult& generation_backbone;
  std::unordered_map<ObjectId, const JunctionInfo*>& active_junction_by_node;
  std::uint64_t session_id = 0;
  std::function<EdgeFlowInfo(ObjectId, ObjectId)> classify_edge_flow;
  std::function<JunctionRelation(ObjectId)> classify_junction;
};

BackboneDecisionPhaseOutput run_backbone_decision_phase(const BackboneDecisionPhaseContext& context) {
  BackboneDecisionPhaseOutput phase{};
  phase.edge_flow_by_segment.reserve((context.ordered_support_node_ids.size() > 1)
                                         ? (context.ordered_support_node_ids.size() - 1)
                                         : 0);
  for (std::size_t i = 0; i + 1 < context.ordered_support_node_ids.size(); ++i) {
    phase.edge_flow_by_segment.push_back(
        context.classify_edge_flow(context.ordered_support_node_ids[i], context.ordered_support_node_ids[i + 1]));
  }
  phase.junction_relations_in_path_order.reserve(context.ordered_support_node_ids.size());
  phase.junction_relations_by_node.reserve(context.ordered_support_node_ids.size());
  for (ObjectId node_id : context.ordered_support_node_ids) {
    JunctionRelation relation = context.classify_junction(node_id);
    phase.junction_relations_by_node[node_id] = relation;
    phase.junction_relations_in_path_order.push_back(std::move(relation));
  }

  context.generation_backbone.junctions.clear();
  context.generation_backbone.junctions.reserve(phase.junction_relations_by_node.size());
  std::unordered_set<ObjectId> rebuilt_junction_nodes{};
  for (ObjectId node_id : context.ordered_support_node_ids) {
    if (!rebuilt_junction_nodes.insert(node_id).second) {
      continue;
    }
    const auto it_relation = phase.junction_relations_by_node.find(node_id);
    if (it_relation == phase.junction_relations_by_node.end()) {
      continue;
    }
    const JunctionRelation& relation = it_relation->second;
    if (relation.incidents.size() < 3) {
      continue;
    }
    JunctionInfo junction{};
    junction.node_id = node_id;
    if (const auto it_existing_prioritized = context.existing_prioritized_session_by_node.find(node_id);
        it_existing_prioritized != context.existing_prioritized_session_by_node.end()) {
      junction.prioritized_session_id = it_existing_prioritized->second;
    } else {
      junction.prioritized_session_id = context.session_id;
    }
    junction.used_neighbor_continuity = relation.through_pair.accepted &&
                                        std::any_of(relation.incidents.begin(), relation.incidents.end(),
                                                    [](const JunctionIncidentRelation& incident) {
                                                      return incident.in_through_pair && !incident.in_route;
                                                    });
    junction.incidents.reserve(relation.incidents.size());
    for (std::size_t index = 0; index < relation.incidents.size(); ++index) {
      const JunctionIncidentRelation& source = relation.incidents[index];
      JunctionIncident incident{};
      incident.neighbor_node_id = source.neighbor_node_id;
      incident.order = static_cast<int>(index);
      incident.primary = (index == 0);
      incident.source_session_id = context.session_id;
      if (const auto it_existing_node = context.existing_incident_session_by_node.find(node_id);
          it_existing_node != context.existing_incident_session_by_node.end()) {
        if (const auto it_existing_source = it_existing_node->second.find(source.neighbor_node_id);
            it_existing_source != it_existing_node->second.end()) {
          incident.source_session_id = it_existing_source->second;
        }
      }
      junction.incidents.push_back(incident);
    }
    context.generation_backbone.junctions.push_back(std::move(junction));
  }
  std::sort(context.generation_backbone.junctions.begin(), context.generation_backbone.junctions.end(),
            [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });

  context.active_junction_by_node.clear();
  context.active_junction_by_node.reserve(context.existing_junction_by_node.size() +
                                          context.generation_backbone.junctions.size());
  for (const auto& [node_id, junction] : context.existing_junction_by_node) {
    context.active_junction_by_node[node_id] = junction;
  }
  for (const JunctionInfo& junction : context.generation_backbone.junctions) {
    context.active_junction_by_node[junction.node_id] = &junction;
  }

  return phase;
}

std::vector<SpanSupportLayoutDecisionSeed>
build_seed_generated_support_layouts(const CoreState& state, const EditState& edit_state, const std::vector<ObjectId>& span_ids,
                                     const std::vector<SegmentLaneAssignment>& lane_assignments,
                                     std::uint64_t variation_flow_key) {
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
      SpanSupportLayoutDecisionSeed layout{};
      layout.span_id = span_id;
      layout.flow_kind = assignment.flow_kind;
      layout.pass_mode = (span->placement_context == ConnectionContext::kBranchAdd)
                             ? CurvePassMode::kBranch
                             : ((span->placement_context == ConnectionContext::kCornerPass)
                                    ? CurvePassMode::kPassThrough
                                    : CurvePassMode::kPassThrough);
      layout.variation_flow_key = variation_flow_key;
      layout.lowering_kind = assignment.lowering_kind;
      layout.start = make_support_layout_seed_endpoint(*span, *port_a, assignment, assignment.decision_a, true);
      layout.end = make_support_layout_seed_endpoint(*span, *port_b, assignment, assignment.decision_b, false);
      append_seed_support_group_decision(state, *port_a, layout.start, &layout);
      append_seed_support_group_decision(state, *port_b, layout.end, &layout);
      layouts.push_back(std::move(layout));
    }
  }
  return layouts;
}

} // namespace

EditResult<GenerateBundleFromPathResult>
CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  const BackboneSpec& request = spec;
  EditResult<GenerateBundleFromPathResult> result;
  if (request.path.polyline.size() < 2) {
    result.error = "backbone input path must contain at least 2 points";
    return result;
  }
  if (request.interval_m <= 0.0) {
    result.error = "interval_m must be > 0";
    return result;
  }
  if (find_pole_type(request.pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  generation::detail::NodeSpecByIndex node_spec_by_index{};
  generation::detail::NodeBundleModeByPoint node_bundle_mode_by_point{};
  if (!generation::detail::build_backbone_node_maps(request, &node_spec_by_index,
                                                              &node_bundle_mode_by_point, &result.error)) {
    return result;
  }

  const std::vector<BackboneBundleSpec>& bundle_requests = request.bundles;
  if (bundle_requests.empty()) {
    result.error = "bundles[] must contain at least one bundle request";
    return result;
  }

  std::vector<BackboneBundlePlan> bundle_plans{};
  bundle_plans.reserve(bundle_requests.size());
  for (const BackboneBundleSpec& bundle_request : bundle_requests) {
    const BundleTemplate* bundle_template = find_bundle_template(bundle_request.bundle_template_id);
    if (bundle_template == nullptr) {
      result.error = "bundle template not found";
      return result;
    }
    BackboneBundlePlan plan{};
    plan.template_id = bundle_template->id;
    plan.category = bundle_template->category;
    plan.layer = (bundle_request.layer == SpanLayer::kUnknown) ? bundle_template->default_layer : bundle_request.layer;
    plan.spacing_m = bundle_template->default_spacing_m;
    plan.continuity_class =
        (plan.count > 1) ? ContinuityCategoryClass::kBundleLike : ContinuityCategoryClass::kPointLike;
    plan.allow_mirror = bundle_template->allow_mirror;
    plan.preserve_conductor_identity = bundle_template->preserve_conductor_identity;
    plan.allow_midair_node = bundle_template->allow_midair_node;
    plan.allow_midair_branch = bundle_template->allow_midair_branch;
    plan.enable_branch_down_offset = bundle_template->enable_branch_down_offset;
    if (bundle_template->count_rule == BundleCountRuleKind::kFixed) {
      if (bundle_request.count > 0) {
        result.error = "count override is not allowed for fixed bundle template";
        return result;
      }
      plan.count = bundle_template->fixed_count;
    } else {
      plan.count = (bundle_request.count > 0) ? bundle_request.count : bundle_template->default_count;
      if (plan.count < bundle_template->min_count || plan.count > bundle_template->max_count) {
        result.error = "bundle count is out of template range";
        return result;
      }
    }
    if (plan.count <= 0) {
      result.error = "resolved bundle count must be > 0";
      return result;
    }
    plan.continuity_class =
        (plan.count > 1) ? ContinuityCategoryClass::kBundleLike : ContinuityCategoryClass::kPointLike;
    plan.order_decision_policy = bundle_template->order_decision_policy;
    if (plan.layer == SpanLayer::kUnknown) {
      result.error = "bundle layer could not be resolved";
      return result;
    }
    bundle_plans.push_back(plan);
  }

  auto support_kind_for_point = [&](std::size_t point_index) -> SupportKind {
    const auto it = node_spec_by_index.find(point_index);
    if (it == node_spec_by_index.end()) {
      return SupportKind::kPole;
    }
    return it->second.support_kind;
  };
  auto node_spec_for_point = [&](std::size_t point_index) -> const BackboneInputSpec::NodeSpec* {
    const auto it = node_spec_by_index.find(point_index);
    return (it == node_spec_by_index.end()) ? nullptr : &it->second;
  };

  auto request_point_reuses_source_edge_support = [&](std::size_t point_index) -> bool {
    const BackboneInputSpec::NodeSpec* node_spec = node_spec_for_point(point_index);
    if (node_spec == nullptr || node_spec->support_kind == SupportKind::kPole ||
        node_spec->node_id == kInvalidObjectId) {
      return false;
    }
    for (const SupportNode& node : debug_.last_generation_support_nodes) {
      if (node.node_id == node_spec->node_id) {
        return node.has_source_edge;
      }
    }
    return false;
  };

  bool request_has_non_pole_points = false;
  bool request_has_source_edge_branch_points = false;
  for (std::size_t point_index = 0; point_index < request.path.polyline.size(); ++point_index) {
    if (support_kind_for_point(point_index) == SupportKind::kPole) {
      continue;
    }
    request_has_non_pole_points = true;
    if (request_point_reuses_source_edge_support(point_index)) {
      request_has_source_edge_branch_points = true;
    }
  }

  std::vector<BackboneBundlePlan> active_bundle_plans{};
  active_bundle_plans.reserve(bundle_plans.size());
  for (const BackboneBundlePlan& plan : bundle_plans) {
    if (request_has_non_pole_points && !plan.allow_midair_node) {
      continue;
    }
    if (request_has_source_edge_branch_points && !plan.allow_midair_branch) {
      continue;
    }
    active_bundle_plans.push_back(plan);
  }

  for (std::size_t point_index = 0; point_index < request.path.polyline.size(); ++point_index) {
    const SupportKind support_kind = support_kind_for_point(point_index);
    if (support_kind == SupportKind::kPole) {
      continue;
    }
    const auto it_modes = node_bundle_mode_by_point.find(point_index);
    if (it_modes == node_bundle_mode_by_point.end()) {
      continue;
    }
    for (const auto& [bundle_template_id, mode] : it_modes->second) {
      const auto it_plan =
          std::find_if(bundle_plans.begin(), bundle_plans.end(),
                       [&](const BackboneBundlePlan& p) { return p.template_id == bundle_template_id; });
      if (it_plan == bundle_plans.end()) {
        result.error = "node_bundle_modes references bundle template that is not selected";
        return result;
      }
      if (mode != BundleNodeMode::kNotPresent && mode != BundleNodeMode::kPassThrough) {
        result.error = "unsupported bundle node mode";
        return result;
      }
    }
  }

  std::vector<Vec3d> guide_points{};
  PathDirectionEvaluationDebug direction_debug{};
  generation::detail::build_backbone_guide_points(request, &guide_points, &direction_debug);
  debug_.last_path_direction_debug = direction_debug;
  path_direction_debug_records_access().push_back(direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }

  if (active_bundle_plans.empty()) {
    result.ok = true;
    return result;
  }

  const BackboneResult existing_network_backbone = BuildBackboneResult();

  std::vector<generation::detail::SupportNodeCandidate> candidates{};
  if (!generation::detail::build_backbone_candidates(request, guide_points, node_spec_by_index,
                                                               &candidates, &result.error)) {
    return result;
  }

  const CoreState snapshot = *this;
  const std::uint64_t session_id = next_generation_session_id_access()++;
  Vec3d preferred_side_dir{0.0, 0.0, 0.0};
  bool has_preferred_side_dir = false;

  auto find_near_pole = [&](const Vec3d& world, PlacementMode preferred_mode) -> ObjectId {
    constexpr double kReuseRadius = 0.25;
    const double reuse_r2 = kReuseRadius * kReuseRadius;
    ObjectId best_id = kInvalidObjectId;
    double best_d2 = reuse_r2 + 1.0;
    bool best_mode_match = false;
    for (const Pole& pole : edit_state_access().poles.items()) {
      if (request.pole_placement.restrict_reuse_to_session) {
        if (request.pole_placement.reuse_session_id == 0 ||
            pole.generation.generation_session_id != request.pole_placement.reuse_session_id) {
          continue;
        }
      }
      const Vec3d d = pole.world_transform.position - world;
      const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
      if (d2 > reuse_r2) {
        continue;
      }
      const bool mode_match = (pole.placement_mode == preferred_mode);
      if (best_id == kInvalidObjectId || (mode_match && !best_mode_match) ||
          (mode_match == best_mode_match && d2 < best_d2)) {
        best_id = pole.id;
        best_d2 = d2;
        best_mode_match = mode_match;
      }
    }
    return best_id;
  };

  std::vector<ObjectId> ordered_pole_ids{};
  ordered_pole_ids.reserve(candidates.size());
  std::vector<ObjectId> ordered_support_node_ids{};
  ordered_support_node_ids.reserve(candidates.size());
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  ObjectId next_virtual_support_id = 0x8000000000000000ull;

  auto ensure_support_node = [&](ObjectId node_id, const generation::detail::SupportNodeCandidate& candidate,
                                 ObjectId pole_id) {
    SupportNode& node = support_node_by_id[node_id];
    node.node_id = node_id;
    node.support_kind = candidate.support_kind;
    node.position = candidate.world;
    node.pole_id = pole_id;
    node.path_point_index = candidate.vertex_index;
    node.has_tangent_hint = candidate.has_tangent_hint;
    node.tangent_hint = candidate.tangent_hint;
  };

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const generation::detail::SupportNodeCandidate& candidate = candidates[i];
    const BackboneInputSpec::NodeSpec* explicit_node_spec =
        (candidate.vertex_index >= 0) ? node_spec_for_point(static_cast<std::size_t>(candidate.vertex_index)) : nullptr;
    const ObjectId explicit_node_id =
        (explicit_node_spec != nullptr) ? explicit_node_spec->node_id : kInvalidObjectId;
    if (candidate.support_kind != SupportKind::kPole) {
      ObjectId support_node_id = explicit_node_id;
      if (support_node_id == kInvalidObjectId) {
        support_node_id = next_virtual_support_id++;
      } else {
        const SupportNode* existing_node = nullptr;
        for (const SupportNode& node : debug_.last_generation_support_nodes) {
          if (node.node_id == support_node_id) {
            existing_node = &node;
            break;
          }
        }
        if (existing_node == nullptr) {
          result.error = "node_specs node_id for support node was not found";
          return result;
        }
        if (existing_node->support_kind != candidate.support_kind) {
          result.error = "node_specs node_id support kind does not match path node kind";
          return result;
        }
        support_node_by_id[support_node_id] = *existing_node;
      }
      ensure_support_node(support_node_id, candidate, kInvalidObjectId);
      ordered_support_node_ids.push_back(support_node_id);
      continue;
    }
    ObjectId pole_id = explicit_node_id;
    if (pole_id != kInvalidObjectId && edit_state_access().poles.find(pole_id) == nullptr) {
      result.error = "node_specs node_id for pole was not found";
      return result;
    }
    if (pole_id == kInvalidObjectId) {
      pole_id = find_near_pole(candidate.world, candidate.mode);
    }
    if (pole_id != kInvalidObjectId) {
      Pole* pole = edit_state_access().poles.find(pole_id);
      if (pole != nullptr) {
        std::optional<AutoPoleTransformResult> auto_tf{};
        if (candidate.vertex_index >= 0) {
          Vec3d base_rotation_euler_deg = pole->world_transform.rotation_euler_deg;
          base_rotation_euler_deg.z = 0.0;
          auto_tf = compute_auto_pole_transform(guide_points, static_cast<std::size_t>(candidate.vertex_index),
                                                has_preferred_side_dir ? &preferred_side_dir : nullptr,
                                                &base_rotation_euler_deg);
          const PoleFrame preferred_frame = BuildPoleFrame(auto_tf->transform, auto_tf->transform.rotation_euler_deg.z);
          preferred_side_dir = preferred_frame.lateral;
          has_preferred_side_dir = Normalize(&preferred_side_dir);
        }
        const Pole old_pole = *pole;
        bool updated = false;
        if (candidate.mode == PlacementMode::kManual) {
          apply_pole_placement_mode(*pole, PlacementMode::kManual);
          updated = true;
        }

        if (candidate.vertex_index >= 0) {
          pole->context = classify_pole_context_from_path(guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
          apply_sharp_debug_to_context(&pole->context, auto_tf.has_value() ? auto_tf->sharp : SharpCornerOrientationDebug{});
          updated = true;
          // Reused poles must follow current corner-orientation rule unless explicitly overridden.
          if (!has_pole_orientation_override(pole->id)) {
            pole->world_transform.rotation_euler_deg.z =
                auto_tf.has_value() ? auto_tf->transform.rotation_euler_deg.z : pole->world_transform.rotation_euler_deg.z;
            updated = true;
          }
        } else {
          pole->context.kind = PoleContextKind::kStraight;
          apply_sharp_debug_to_context(&pole->context, SharpCornerOrientationDebug{});
          if (!has_pole_orientation_override(pole->id)) {
            const Vec3d dir = guide_points[candidate.segment_index + 1] - guide_points[candidate.segment_index];
            if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
              pole->world_transform.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
              updated = true;
            }
          }
        }

        if (updated) {
          // Reused poles need endpoint reprojection after context/yaw updates.
          finalize_pole_transform_update(pole->id, old_pole, &result.change_set);
        }
      }
      ordered_pole_ids.push_back(pole_id);
      ensure_support_node(pole_id, candidate, pole_id);
      ordered_support_node_ids.push_back(pole_id);
      continue;
    }

    Transformd tf{};
    SharpCornerOrientationDebug created_sharp_debug{};
    bool has_created_sharp_debug = false;
    tf.position = candidate.world;
    if (candidate.vertex_index >= 0) {
      const AutoPoleTransformResult auto_tf =
          compute_auto_pole_transform(guide_points, static_cast<std::size_t>(candidate.vertex_index),
                                      has_preferred_side_dir ? &preferred_side_dir : nullptr);
      tf = auto_tf.transform;
      tf.position = candidate.world;
      created_sharp_debug = auto_tf.sharp;
      has_created_sharp_debug = true;
      const PoleFrame preferred_frame = BuildPoleFrame(tf, tf.rotation_euler_deg.z);
      preferred_side_dir = preferred_frame.lateral;
      has_preferred_side_dir = Normalize(&preferred_side_dir);
    } else {
      const Vec3d dir = guide_points[candidate.segment_index + 1] - guide_points[candidate.segment_index];
      if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
        tf.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
      }
    }

    EditResult<ObjectId> add_pole =
        AddPole(tf, 10.0, "PathPole", PoleKind::kConcrete, candidate.mode);
    if (!add_pole.ok) {
      *this = snapshot;
      result.error = add_pole.error;
      return result;
    }
    Pole* pole = edit_state_access().poles.find(add_pole.value);
    if (pole != nullptr) {
      if (candidate.vertex_index >= 0) {
        pole->context = classify_pole_context_from_path(guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
        apply_sharp_debug_to_context(&pole->context, has_created_sharp_debug ? created_sharp_debug
                                                                              : SharpCornerOrientationDebug{});
      } else {
        pole->context.kind = PoleContextKind::kStraight;
        apply_sharp_debug_to_context(&pole->context, SharpCornerOrientationDebug{});
      }
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(ordered_pole_ids.size());
      add_unique_id(add_pole.change_set.updated_ids, pole->id);
    }
    EditResult<ObjectId> apply_type = ApplyPoleType(add_pole.value, request.pole_type_id);
    if (!apply_type.ok) {
      *this = snapshot;
      result.error = apply_type.error;
      return result;
    }
    append_change_set(result.change_set, add_pole.change_set);
    append_change_set(result.change_set, apply_type.change_set);
    ordered_pole_ids.push_back(add_pole.value);
    ensure_support_node(add_pole.value, candidate, add_pole.value);
    ordered_support_node_ids.push_back(add_pole.value);
    result.value.generated_pole_ids.push_back(add_pole.value);
  }

  if (ordered_support_node_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to create or resolve guide support nodes";
    return result;
  }
  {
    std::vector<ObjectId> compact_ids{};
    compact_ids.reserve(ordered_pole_ids.size());
    for (ObjectId id : ordered_pole_ids) {
      if (compact_ids.empty() || compact_ids.back() != id) {
        compact_ids.push_back(id);
      }
    }
    ordered_pole_ids.swap(compact_ids);
  }
  {
    std::vector<ObjectId> compact_support_ids{};
    compact_support_ids.reserve(ordered_support_node_ids.size());
    for (ObjectId id : ordered_support_node_ids) {
      if (compact_support_ids.empty() || compact_support_ids.back() != id) {
        compact_support_ids.push_back(id);
      }
    }
    ordered_support_node_ids.swap(compact_support_ids);
  }
  if (ordered_support_node_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to build valid support-node chain";
    return result;
  }
  struct ChainEdgeKey {
    ObjectId a = kInvalidObjectId;
    ObjectId b = kInvalidObjectId;
    bool operator==(const ChainEdgeKey& other) const { return a == other.a && b == other.b; }
  };
  struct ChainEdgeKeyHash {
    std::size_t operator()(const ChainEdgeKey& key) const {
      const std::size_t h1 = std::hash<ObjectId>{}(key.a);
      const std::size_t h2 = std::hash<ObjectId>{}(key.b);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };

  BackboneResult generation_backbone{};
  std::unordered_set<ChainEdgeKey, ChainEdgeKeyHash> unique_chain_edges{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint32_t>> incident_first_order{};

  auto update_incident_order = [&](ObjectId node_id, ObjectId neighbor_id, std::uint32_t order_index) {
    auto& by_neighbor = incident_first_order[node_id];
    auto it = by_neighbor.find(neighbor_id);
    if (it == by_neighbor.end() || order_index < it->second) {
      by_neighbor[neighbor_id] = order_index;
    }
  };

  auto support_position = [&](ObjectId node_id) -> Vec3d {
    const auto it = support_node_by_id.find(node_id);
    if (it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state_access().poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    return {};
  };
  for (std::size_t i = 0; i + 1 < ordered_support_node_ids.size(); ++i) {
    const ObjectId a = ordered_support_node_ids[i];
    const ObjectId b = ordered_support_node_ids[i + 1];
    if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) {
      continue;
    }
    update_incident_order(a, b, static_cast<std::uint32_t>(i));
    update_incident_order(b, a, static_cast<std::uint32_t>(i));

    const ObjectId key_a = std::min(a, b);
    const ObjectId key_b = std::max(a, b);
    if (unique_chain_edges.insert({key_a, key_b}).second) {
      BackboneEdge edge{};
      edge.node_a = key_a;
      edge.node_b = key_b;
      generation_backbone.edges.push_back(edge);
    }
  }
  std::sort(generation_backbone.edges.begin(), generation_backbone.edges.end(),
            [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
              if (lhs.node_a != rhs.node_a) {
                return lhs.node_a < rhs.node_a;
              }
              return lhs.node_b < rhs.node_b;
            });

  auto normalize_dir = [](const Vec3d& v) -> Vec3d {
    const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= 1e-9) {
      return {0.0, 0.0, 0.0};
    }
    return {v.x / len, v.y / len, v.z / len};
  };
  auto dot = [](const Vec3d& a, const Vec3d& b) -> double { return a.x * b.x + a.y * b.y + a.z * b.z; };

  const BackboneResult& existing_backbone = existing_network_backbone;
  std::unordered_map<ObjectId, ObjectId> existing_primary_neighbor_by_node{};
  std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
  for (const JunctionInfo& junction : existing_backbone.junctions) {
    existing_prioritized_session_by_node[junction.node_id] = junction.prioritized_session_id;
    for (const JunctionIncident& incident : junction.incidents) {
      existing_incident_session_by_node[junction.node_id][incident.neighbor_node_id] = incident.source_session_id;
      if (incident.primary) {
        existing_primary_neighbor_by_node[junction.node_id] = incident.neighbor_node_id;
      }
    }
  }

  std::unordered_map<ObjectId, std::vector<ObjectId>> route_neighbors_by_node{};
  for (std::size_t i = 0; i < ordered_support_node_ids.size(); ++i) {
    const ObjectId node_id = ordered_support_node_ids[i];
    std::vector<ObjectId>& neighbors = route_neighbors_by_node[node_id];
    if (i > 0) {
      const ObjectId prev = ordered_support_node_ids[i - 1];
      if (prev != node_id && std::find(neighbors.begin(), neighbors.end(), prev) == neighbors.end()) {
        neighbors.push_back(prev);
      }
    }
    if (i + 1 < ordered_support_node_ids.size()) {
      const ObjectId next = ordered_support_node_ids[i + 1];
      if (next != node_id && std::find(neighbors.begin(), neighbors.end(), next) == neighbors.end()) {
        neighbors.push_back(next);
      }
    }
  }

  std::unordered_map<ObjectId, Vec3d> existing_node_position_by_id{};
  existing_node_position_by_id.reserve(existing_network_backbone.nodes.size() + edit_state_access().poles.size());
  for (const SupportNode& node : existing_network_backbone.nodes) {
    existing_node_position_by_id[node.node_id] = node.position;
  }
  for (const Pole& pole : edit_state_access().poles.items()) {
    existing_node_position_by_id.try_emplace(pole.id, pole.world_transform.position);
  }

  std::unordered_map<ObjectId, std::vector<ObjectId>> existing_adjacency{};
  for (const BackboneEdge& edge : existing_network_backbone.edges) {
    if (edge.node_a == kInvalidObjectId || edge.node_b == kInvalidObjectId || edge.node_a == edge.node_b) {
      continue;
    }
    existing_adjacency[edge.node_a].push_back(edge.node_b);
    existing_adjacency[edge.node_b].push_back(edge.node_a);
  }
  for (auto& [_, neighbors] : existing_adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }

  std::unordered_map<ObjectId, const JunctionInfo*> existing_junction_by_node{};
  existing_junction_by_node.reserve(existing_network_backbone.junctions.size());
  for (const JunctionInfo& junction : existing_network_backbone.junctions) {
    existing_junction_by_node[junction.node_id] = &junction;
  }

  std::unordered_map<ObjectId, ObjectId> backbone_primary_neighbors{};
  for (const auto& [node_id, neighbors] : incident_first_order) {
    if (neighbors.size() < 3) {
      continue;
    }
    const Vec3d center_pos = support_position(node_id);

    struct Candidate {
      ObjectId neighbor_id = kInvalidObjectId;
      Vec3d dir{};
    };
    std::vector<Candidate> candidates{};
    candidates.reserve(neighbors.size());
    for (const auto& [neighbor_id, first_order] : neighbors) {
      (void)first_order;
      const Vec3d neighbor_pos = support_position(neighbor_id);
      Candidate c{};
      c.neighbor_id = neighbor_id;
      c.dir = normalize_dir(neighbor_pos - center_pos);
      candidates.push_back(c);
    }
    if (candidates.size() < 3) {
      continue;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.neighbor_id < b.neighbor_id; });

    double best_pair_straight = -2.0;
    int best_pair_i = -1;
    int best_pair_j = -1;
    int best_pair_anchor = -1;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        const double straight_score =
            dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y, -candidates[j].dir.z});
        int pair_anchor = static_cast<int>(i);
        if (candidates[j].neighbor_id < candidates[i].neighbor_id) {
          pair_anchor = static_cast<int>(j);
        }
        if (straight_score > best_pair_straight + 1e-9 ||
            (std::abs(straight_score - best_pair_straight) <= 1e-9 &&
             (best_pair_anchor < 0 || candidates[static_cast<std::size_t>(pair_anchor)].neighbor_id <
                                           candidates[static_cast<std::size_t>(best_pair_anchor)].neighbor_id))) {
          best_pair_straight = straight_score;
          best_pair_i = static_cast<int>(i);
          best_pair_j = static_cast<int>(j);
          best_pair_anchor = pair_anchor;
        }
      }
    }
    auto belongs_to_best_pair = [&](ObjectId neighbor_id) {
      if (best_pair_i < 0 || best_pair_j < 0) {
        return false;
      }
      return candidates[static_cast<std::size_t>(best_pair_i)].neighbor_id == neighbor_id ||
             candidates[static_cast<std::size_t>(best_pair_j)].neighbor_id == neighbor_id;
    };

    int anchor_index = -1;
    bool used_neighbor_continuity = false;
    const auto it_existing_primary = existing_primary_neighbor_by_node.find(node_id);
    if (it_existing_primary != existing_primary_neighbor_by_node.end() &&
        belongs_to_best_pair(it_existing_primary->second)) {
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].neighbor_id == it_existing_primary->second) {
          anchor_index = static_cast<int>(i);
          used_neighbor_continuity = true;
          break;
        }
      }
    }
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (anchor_index >= 0) {
        break;
      }
      const auto it_prev = backbone_primary_neighbors.find(candidates[i].neighbor_id);
      if (it_prev == backbone_primary_neighbors.end() || it_prev->second != node_id) {
        continue;
      }
      if (!belongs_to_best_pair(candidates[i].neighbor_id)) {
        continue;
      }
      const int idx = static_cast<int>(i);
      if (anchor_index < 0 || candidates[static_cast<std::size_t>(idx)].neighbor_id <
                                  candidates[static_cast<std::size_t>(anchor_index)].neighbor_id) {
        anchor_index = idx;
        used_neighbor_continuity = true;
      }
    }

    if (anchor_index < 0) {
      anchor_index = (best_pair_anchor >= 0) ? best_pair_anchor : 0;
    }

    int opposite_index = -1;
    if (best_pair_i >= 0 && best_pair_j >= 0) {
      opposite_index = (anchor_index == best_pair_i) ? best_pair_j : best_pair_i;
    }
    if (opposite_index < 0 || opposite_index == anchor_index) {
      double best_straight = -2.0;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx == anchor_index) {
          continue;
        }
        const double straight_score =
            dot(candidates[static_cast<std::size_t>(anchor_index)].dir,
                Vec3d{-candidates[i].dir.x, -candidates[i].dir.y, -candidates[i].dir.z});
        if (straight_score > best_straight + 1e-9 ||
            (std::abs(straight_score - best_straight) <= 1e-9 &&
             candidates[i].neighbor_id <
                 candidates[static_cast<std::size_t>(opposite_index < 0 ? idx : opposite_index)].neighbor_id)) {
          best_straight = straight_score;
          opposite_index = idx;
        }
      }
    }

    std::vector<int> order_indices{};
    order_indices.push_back(anchor_index);
    if (opposite_index >= 0 && opposite_index != anchor_index) {
      order_indices.push_back(opposite_index);
    }
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const int idx = static_cast<int>(i);
      if (idx == anchor_index || idx == opposite_index) {
        continue;
      }
      order_indices.push_back(idx);
    }

    JunctionInfo junction{};
    junction.node_id = node_id;
    junction.prioritized_session_id = session_id;
    const auto it_existing_prioritized = existing_prioritized_session_by_node.find(node_id);
    if (it_existing_prioritized != existing_prioritized_session_by_node.end()) {
      junction.prioritized_session_id = it_existing_prioritized->second;
    }
    junction.used_neighbor_continuity = used_neighbor_continuity;
    for (std::size_t rank = 0; rank < order_indices.size(); ++rank) {
      const Candidate& candidate = candidates[static_cast<std::size_t>(order_indices[rank])];
      JunctionIncident incident{};
      incident.neighbor_node_id = candidate.neighbor_id;
      incident.order = static_cast<int>(rank);
      incident.primary = (rank == 0);
      incident.source_session_id = session_id;
      const auto it_existing_node = existing_incident_session_by_node.find(node_id);
      if (it_existing_node != existing_incident_session_by_node.end()) {
        const auto it_existing_source = it_existing_node->second.find(candidate.neighbor_id);
        if (it_existing_source != it_existing_node->second.end()) {
          incident.source_session_id = it_existing_source->second;
        }
      }
      junction.incidents.push_back(incident);
    }
    if (!junction.incidents.empty()) {
      backbone_primary_neighbors[node_id] = junction.incidents.front().neighbor_node_id;
    }
    generation_backbone.junctions.push_back(std::move(junction));
  }
  std::sort(generation_backbone.junctions.begin(), generation_backbone.junctions.end(),
            [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });

  generation_backbone.nodes.reserve(support_node_by_id.size());
  for (const auto& [node_id, base_node] : support_node_by_id) {
    SupportNode node = base_node;
    std::unordered_map<BundleKind, BundleNodeMode> mode_by_bundle{};
    for (const BackboneBundlePlan& plan : bundle_plans) {
      mode_by_bundle[plan.template_id] = BundleNodeMode::kNotPresent;
    }
    if (node.path_point_index >= 0) {
      const auto it_mode_spec = node_bundle_mode_by_point.find(static_cast<std::size_t>(node.path_point_index));
      if (it_mode_spec != node_bundle_mode_by_point.end()) {
        for (const auto& [bundle_template_id, mode] : it_mode_spec->second) {
          mode_by_bundle[bundle_template_id] = mode;
        }
      }
    }

    node.bundle_modes.clear();
    node.bundle_modes.reserve(mode_by_bundle.size());
    for (const auto& [bundle_template_id, mode] : mode_by_bundle) {
      SupportNodeBundleMode bundle_mode{};
      bundle_mode.bundle_template_id = bundle_template_id;
      bundle_mode.mode = mode;
      node.bundle_modes.push_back(bundle_mode);
    }
    std::sort(node.bundle_modes.begin(), node.bundle_modes.end(),
              [](const SupportNodeBundleMode& a, const SupportNodeBundleMode& b) {
                return static_cast<int>(a.bundle_template_id) < static_cast<int>(b.bundle_template_id);
              });
    generation_backbone.nodes.push_back(std::move(node));
  }
  std::sort(generation_backbone.nodes.begin(), generation_backbone.nodes.end(),
            [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });

  std::unordered_map<ObjectId, const JunctionInfo*> active_junction_by_node = existing_junction_by_node;
  active_junction_by_node.reserve(existing_junction_by_node.size() + generation_backbone.junctions.size());
  for (const JunctionInfo& junction : generation_backbone.junctions) {
    active_junction_by_node[junction.node_id] = &junction;
  }

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
  auto combined_neighbors_for_node = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    for (const Span& span : edit_state_access().spans.items()) {
      const Port* port_a = edit_state_access().ports.find(span.port_a_id);
      const Port* port_b = edit_state_access().ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId && port_b->owner_pole_id != node_id &&
          std::find(neighbors.begin(), neighbors.end(), port_b->owner_pole_id) == neighbors.end()) {
        neighbors.push_back(port_b->owner_pole_id);
      }
      if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId && port_a->owner_pole_id != node_id &&
          std::find(neighbors.begin(), neighbors.end(), port_a->owner_pole_id) == neighbors.end()) {
        neighbors.push_back(port_a->owner_pole_id);
      }
    }
    if (const auto it_route = route_neighbors_by_node.find(node_id); it_route != route_neighbors_by_node.end()) {
      for (ObjectId neighbor_id : it_route->second) {
        if (std::find(neighbors.begin(), neighbors.end(), neighbor_id) == neighbors.end()) {
          neighbors.push_back(neighbor_id);
        }
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
  };
  auto connected_neighbors_for_support_axis = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    for (const Span& span : edit_state_access().spans.items()) {
      const Port* port_a = edit_state_access().ports.find(span.port_a_id);
      const Port* port_b = edit_state_access().ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId && port_b->owner_pole_id != node_id) {
        neighbors.push_back(port_b->owner_pole_id);
      }
      if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId && port_a->owner_pole_id != node_id) {
        neighbors.push_back(port_a->owner_pole_id);
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
  };

  struct BundleCategoryTieBreakKey {
    int category_rank = std::numeric_limits<int>::max();
    int bundle_rank = std::numeric_limits<int>::max();
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    BundleKind bundle_template_id = BundleKind::kLowVoltage;
  };
  auto bundle_category_key_tuple = [](const BundleCategoryTieBreakKey& key) {
    return std::tuple<int, int>{key.category_rank, key.bundle_rank};
  };
  auto better_bundle_category_key = [&](const BundleCategoryTieBreakKey& lhs, const BundleCategoryTieBreakKey& rhs) {
    return bundle_category_key_tuple(lhs) < bundle_category_key_tuple(rhs);
  };
  auto edge_key_for_neighbors = [&](ObjectId node_a, ObjectId node_b) {
    const std::uint64_t lo = static_cast<std::uint64_t>(std::min(node_a, node_b));
    const std::uint64_t hi = static_cast<std::uint64_t>(std::max(node_a, node_b));
    return hash_combine(lo, hi);
  };
  BundleCategoryTieBreakKey route_bundle_category_key{};
  for (const BackboneBundlePlan& plan : active_bundle_plans) {
    const BundleCategoryTieBreakKey candidate_key{
        static_cast<int>(plan.category), static_cast<int>(plan.template_id), plan.category, plan.template_id};
    if (better_bundle_category_key(candidate_key, route_bundle_category_key)) {
      route_bundle_category_key = candidate_key;
    }
  }
  std::unordered_map<std::uint64_t, BundleCategoryTieBreakKey> bundle_category_key_by_edge{};
  for (const Span& span : edit_state_access().spans.items()) {
    const Port* port_a = edit_state_access().ports.find(span.port_a_id);
    const Port* port_b = edit_state_access().ports.find(span.port_b_id);
    const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
    if (port_a == nullptr || port_b == nullptr || bundle == nullptr) {
      continue;
    }
    if (port_a->owner_pole_id == kInvalidObjectId || port_b->owner_pole_id == kInvalidObjectId ||
        port_a->owner_pole_id == port_b->owner_pole_id) {
      continue;
    }
    const BundleTemplate* bundle_template = find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr) {
      continue;
    }
    const BundleCategoryTieBreakKey candidate_key{
        static_cast<int>(bundle_template->category), static_cast<int>(bundle->bundle_template_id), bundle_template->category,
        bundle->bundle_template_id};
    BundleCategoryTieBreakKey& current_key =
        bundle_category_key_by_edge[edge_key_for_neighbors(port_a->owner_pole_id, port_b->owner_pole_id)];
    if (better_bundle_category_key(candidate_key, current_key)) {
      current_key = candidate_key;
    }
  }
  auto bundle_category_key_for_neighbor = [&](ObjectId node_id, ObjectId neighbor_id, bool is_route_neighbor) {
    BundleCategoryTieBreakKey key{};
    const auto it = bundle_category_key_by_edge.find(edge_key_for_neighbors(node_id, neighbor_id));
    if (it != bundle_category_key_by_edge.end()) {
      key = it->second;
    }
    if (is_route_neighbor && better_bundle_category_key(route_bundle_category_key, key)) {
      key = route_bundle_category_key;
    }
    return key;
  };
  auto row_layout_axis_mode_for_key = [&](const BundleCategoryTieBreakKey& key) {
    const BundleTemplate* bundle_template = find_bundle_template(key.bundle_template_id);
    return (bundle_template != nullptr) ? bundle_template->row_layout_axis_mode : RowLayoutAxisMode::kPoleYaw;
  };
  auto row_layout_axis_key_for_node = [&](ObjectId node_id) {
    BundleCategoryTieBreakKey selected{};
    const std::vector<ObjectId> route_neighbors =
        (route_neighbors_by_node.contains(node_id) ? route_neighbors_by_node.at(node_id) : std::vector<ObjectId>{});
    const std::vector<ObjectId> connected_neighbors = connected_neighbors_for_support_axis(node_id);
    for (ObjectId neighbor_id : connected_neighbors) {
      const bool is_route_neighbor =
          std::find(route_neighbors.begin(), route_neighbors.end(), neighbor_id) != route_neighbors.end();
      const BundleCategoryTieBreakKey candidate = bundle_category_key_for_neighbor(node_id, neighbor_id, is_route_neighbor);
      if (row_layout_axis_mode_for_key(candidate) != RowLayoutAxisMode::kSupportAxis) {
        continue;
      }
      if (better_bundle_category_key(candidate, selected)) {
        selected = candidate;
      }
    }
    if (row_layout_axis_mode_for_key(route_bundle_category_key) == RowLayoutAxisMode::kSupportAxis &&
        better_bundle_category_key(route_bundle_category_key, selected)) {
      selected = route_bundle_category_key;
    }
    return selected;
  };

  auto existing_continuation_neighbors_for_orientation = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr && junction->incidents.size() >= 2) {
        const JunctionIncident* primary_incident = nullptr;
        const JunctionIncident* continuation_incident = nullptr;
        for (const JunctionIncident& incident : junction->incidents) {
          if ((incident.primary || incident.order == 0) &&
              (primary_incident == nullptr ||
               (incident.primary && !primary_incident->primary) ||
               (incident.order >= 0 && (primary_incident->order < 0 || incident.order < primary_incident->order)))) {
            primary_incident = &incident;
          }
          if (incident.order == 1 &&
              (continuation_incident == nullptr || incident.neighbor_node_id < continuation_incident->neighbor_node_id)) {
            continuation_incident = &incident;
          }
        }
        const bool has_continuation_pair =
            primary_incident != nullptr && continuation_incident != nullptr &&
            primary_incident->neighbor_node_id != continuation_incident->neighbor_node_id &&
            (junction->used_neighbor_continuity || junction->incidents.size() == 2);
        if (has_continuation_pair) {
          neighbors.push_back(primary_incident->neighbor_node_id);
          neighbors.push_back(continuation_incident->neighbor_node_id);
          return neighbors;
        }
      }
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree == 1) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
        return it->second;
      }
    }
    if (route_degree == 0) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
        return it->second;
      }
    }
    return neighbors;
  };
  auto existing_continuation_neighbors_for_order = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    if (const auto it = existing_junction_by_node.find(node_id); it != existing_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr && junction->incidents.size() >= 2) {
        const JunctionIncident* primary_incident = nullptr;
        const JunctionIncident* continuation_incident = nullptr;
        for (const JunctionIncident& incident : junction->incidents) {
          if ((incident.primary || incident.order == 0) &&
              (primary_incident == nullptr ||
               (incident.primary && !primary_incident->primary) ||
               (incident.order >= 0 && (primary_incident->order < 0 || incident.order < primary_incident->order)))) {
            primary_incident = &incident;
          }
          if (incident.order == 1 &&
              (continuation_incident == nullptr || incident.neighbor_node_id < continuation_incident->neighbor_node_id)) {
            continuation_incident = &incident;
          }
        }
        const bool has_continuation_pair =
            primary_incident != nullptr && continuation_incident != nullptr &&
            primary_incident->neighbor_node_id != continuation_incident->neighbor_node_id &&
            (junction->used_neighbor_continuity || junction->incidents.size() == 2);
        if (has_continuation_pair) {
          neighbors.push_back(primary_incident->neighbor_node_id);
          neighbors.push_back(continuation_incident->neighbor_node_id);
          return neighbors;
        }
      }
    }
    if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
      return it->second;
    }
    return neighbors;
  };
  auto existing_primary_neighbor_for_orientation = [&](ObjectId node_id) -> ObjectId {
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.primary || incident.order == 0) {
            return incident.neighbor_node_id;
          }
        }
      }
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree == 1) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 1) {
        return it->second.front();
      }
    }
    if (route_degree == 0) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 1) {
        return it->second.front();
      }
    }
    return kInvalidObjectId;
  };
  auto existing_primary_neighbor_for_order = [&](ObjectId node_id) -> ObjectId {
    if (const auto it = existing_junction_by_node.find(node_id); it != existing_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.primary || incident.order == 0) {
            return incident.neighbor_node_id;
          }
        }
      }
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree <= 1) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 1) {
        return it->second.front();
      }
    }
    const auto it_existing_primary = existing_primary_neighbor_by_node.find(node_id);
    return (it_existing_primary == existing_primary_neighbor_by_node.end()) ? kInvalidObjectId
                                                                            : it_existing_primary->second;
  };
  auto preferred_straight_main_pair_for_orientation = [&](ObjectId node_id) -> std::pair<ObjectId, ObjectId> {
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    const std::vector<ObjectId> route_neighbors =
        (it_route == route_neighbors_by_node.end()) ? std::vector<ObjectId>{} : it_route->second;
    constexpr double kPreferredPairStraightnessThreshold = 0.3;
    if (route_degree == 0) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    std::vector<ObjectId> combined_neighbors = combined_neighbors_for_node(node_id);
    if (combined_neighbors.size() < 2) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    struct Candidate {
      ObjectId neighbor_id = kInvalidObjectId;
      Vec3d dir{};
    };
    std::vector<Candidate> candidates{};
    candidates.reserve(combined_neighbors.size());
    const Vec3d center = current_support_position(node_id);
    for (ObjectId neighbor_id : combined_neighbors) {
      const Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
      if (!std::isfinite(dir.x) || !std::isfinite(dir.y)) {
        continue;
      }
      candidates.push_back({neighbor_id, dir});
    }
    if (candidates.size() < 2) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    double best_score = -2.0;
    int best_i = -1;
    int best_j = -1;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        const double straight_score =
            dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y, -candidates[j].dir.z});
        if (straight_score > best_score + 1e-9) {
          best_score = straight_score;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
        }
      }
    }
    if (best_i < 0 || best_j < 0) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    std::vector<ObjectId> stable_existing_pair{};
    if (const auto it_existing_junction = existing_junction_by_node.find(node_id);
        it_existing_junction != existing_junction_by_node.end()) {
      const JunctionInfo* junction = it_existing_junction->second;
      if (junction != nullptr && junction->incidents.size() >= 2) {
        const JunctionIncident* primary_incident = nullptr;
        const JunctionIncident* continuation_incident = nullptr;
        for (const JunctionIncident& incident : junction->incidents) {
          if ((incident.primary || incident.order == 0) &&
              (primary_incident == nullptr ||
               (incident.primary && !primary_incident->primary) ||
               (incident.order >= 0 && (primary_incident->order < 0 || incident.order < primary_incident->order)))) {
            primary_incident = &incident;
          }
          if (incident.order == 1 &&
              (continuation_incident == nullptr || incident.neighbor_node_id < continuation_incident->neighbor_node_id)) {
            continuation_incident = &incident;
          }
        }
        if (primary_incident != nullptr && continuation_incident != nullptr &&
            primary_incident->neighbor_node_id != continuation_incident->neighbor_node_id &&
            (junction->used_neighbor_continuity || junction->incidents.size() == 2)) {
          stable_existing_pair.push_back(primary_incident->neighbor_node_id);
          stable_existing_pair.push_back(continuation_incident->neighbor_node_id);
        }
      }
    }
    if (stable_existing_pair.size() < 2) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
        stable_existing_pair = it->second;
      }
    }
    if (stable_existing_pair.size() >= 2) {
      auto pair_straightness_score = [&](ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
        const Vec3d axis_a = normalize_forward_xy(current_support_position(neighbor_a_id) - center);
        const Vec3d axis_b = normalize_forward_xy(current_support_position(neighbor_b_id) - center);
        if (!std::isfinite(axis_a.x) || !std::isfinite(axis_a.y) || !std::isfinite(axis_b.x) ||
            !std::isfinite(axis_b.y)) {
          return -2.0;
        }
        return dot(axis_a, Vec3d{-axis_b.x, -axis_b.y, -axis_b.z});
      };
      auto ordered_existing_pair = [&]() {
        ObjectId primary_neighbor_id = stable_existing_pair[0];
        ObjectId secondary_neighbor_id = stable_existing_pair[1];
        const auto it_existing_primary = existing_primary_neighbor_by_node.find(node_id);
        const ObjectId existing_primary_neighbor_id =
            (it_existing_primary == existing_primary_neighbor_by_node.end()) ? kInvalidObjectId : it_existing_primary->second;
        if (existing_primary_neighbor_id == secondary_neighbor_id) {
          std::swap(primary_neighbor_id, secondary_neighbor_id);
        } else if (existing_primary_neighbor_id != primary_neighbor_id && secondary_neighbor_id < primary_neighbor_id) {
          std::swap(primary_neighbor_id, secondary_neighbor_id);
        }
        return std::pair<ObjectId, ObjectId>{primary_neighbor_id, secondary_neighbor_id};
      };

      const double existing_score = pair_straightness_score(stable_existing_pair[0], stable_existing_pair[1]);
      const bool existing_pair_is_straight = existing_score + 1e-9 >= kPreferredPairStraightnessThreshold;
      if (route_degree == 1 && existing_pair_is_straight) {
        return ordered_existing_pair();
      }
      if (const bool existing_pair_is_route_disjoint =
              route_degree == 2 &&
              std::ranges::find(route_neighbors, stable_existing_pair[0]) == route_neighbors.end() &&
              std::ranges::find(route_neighbors, stable_existing_pair[1]) == route_neighbors.end();
          combined_neighbors.size() >= 4 && existing_pair_is_straight && existing_pair_is_route_disjoint) {
        const double route_score = pair_straightness_score(route_neighbors[0], route_neighbors[1]);
        const bool route_pair_is_straight = route_score + 1e-9 >= kPreferredPairStraightnessThreshold;
        if (route_pair_is_straight) {
          return ordered_existing_pair();
        }
      }

      if (!(best_score > existing_score + 1e-6)) {
        if (combined_neighbors.size() < 4) {
          return {kInvalidObjectId, kInvalidObjectId};
        }
        return ordered_existing_pair();
      }
    }

    ObjectId primary_neighbor_id = candidates[static_cast<std::size_t>(best_i)].neighbor_id;
    ObjectId secondary_neighbor_id = candidates[static_cast<std::size_t>(best_j)].neighbor_id;
    const ObjectId existing_primary_neighbor_id = existing_primary_neighbor_for_orientation(node_id);
    if (existing_primary_neighbor_id == primary_neighbor_id || existing_primary_neighbor_id == secondary_neighbor_id) {
      if (existing_primary_neighbor_id == secondary_neighbor_id) {
        std::swap(primary_neighbor_id, secondary_neighbor_id);
      }
    } else if (secondary_neighbor_id < primary_neighbor_id) {
      std::swap(primary_neighbor_id, secondary_neighbor_id);
    }
    return {primary_neighbor_id, secondary_neighbor_id};
  };
  auto continuation_neighbors_for_orientation = [&](ObjectId node_id) {
    const auto preferred_pair = preferred_straight_main_pair_for_orientation(node_id);
    if (preferred_pair.first != kInvalidObjectId && preferred_pair.second != kInvalidObjectId &&
        preferred_pair.first != preferred_pair.second) {
      return std::vector<ObjectId>{preferred_pair.first, preferred_pair.second};
    }
    return existing_continuation_neighbors_for_orientation(node_id);
  };
  auto primary_neighbor_for_orientation = [&](ObjectId node_id) -> ObjectId {
    const auto preferred_pair = preferred_straight_main_pair_for_orientation(node_id);
    if (preferred_pair.first != kInvalidObjectId) {
      return preferred_pair.first;
    }
    return existing_primary_neighbor_for_orientation(node_id);
  };
  auto has_existing_main_flow_context = [&](ObjectId node_id) -> bool {
    if (active_junction_by_node.contains(node_id)) {
      return true;
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree > 1) {
      return false;
    }
    if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end()) {
      return !it->second.empty();
    }
    return false;
  };
  auto choose_support_axis_for_layout = [&](ObjectId node_id, const Vec3d& center, const Vec3d& previous_support_axis,
                                            PoleOrientationDebugRecord* debug) {
    Vec3d chosen_axis = normalize_forward_xy(previous_support_axis);
    bool has_axis = Normalize(&chosen_axis);
    Vec3d normalized_previous_support_axis = normalize_forward_xy(previous_support_axis);
    const bool has_previous_support_axis = std::isfinite(normalized_previous_support_axis.x) &&
                                           std::isfinite(normalized_previous_support_axis.y) &&
                                           Normalize(&normalized_previous_support_axis);
    auto candidate_support_axis_from_neighbor = [&](ObjectId neighbor_id, Vec3d* out_axis) {
      if (out_axis == nullptr || neighbor_id == kInvalidObjectId) {
        return false;
      }
      Vec3d normalized_axis = normalize_forward_xy(current_support_position(neighbor_id) - center);
      if (!Normalize(&normalized_axis)) {
        return false;
      }
      Vec3d row_axis = ComputeLateralAxis(normalized_axis);
      if (!Normalize(&row_axis)) {
        return false;
      }
      *out_axis = choose_continuous_axis(row_axis, previous_support_axis);
      return Normalize(out_axis);
    };
    auto adopt_axis = [&](const Vec3d& axis, PoleSupportAxisRule rule, ObjectId primary_neighbor_id,
                          ObjectId secondary_neighbor_id) {
      Vec3d normalized_axis = normalize_forward_xy(axis);
      if (!Normalize(&normalized_axis)) {
        return false;
      }
      Vec3d row_axis = ComputeLateralAxis(normalized_axis);
      if (!Normalize(&row_axis)) {
        return false;
      }
      chosen_axis = choose_continuous_axis(row_axis, previous_support_axis);
      has_axis = Normalize(&chosen_axis);
      if (has_axis && debug != nullptr) {
        debug->support_axis_rule = rule;
        debug->primary_neighbor_id = primary_neighbor_id;
        debug->secondary_neighbor_id = secondary_neighbor_id;
      }
      return has_axis;
    };
    auto adopt_connected_direction_fit = [&](const std::vector<ObjectId>& neighbor_ids) {
      struct DirectionCandidate {
        ObjectId neighbor_id = kInvalidObjectId;
        Vec3d dir{};
      };
      std::vector<DirectionCandidate> directions{};
      directions.reserve(neighbor_ids.size());
      for (ObjectId neighbor_id : neighbor_ids) {
        Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
        if (Normalize(&dir)) {
          directions.push_back({neighbor_id, dir});
        }
      }
      if (directions.empty()) {
        return false;
      }

      struct BestCandidate {
        Vec3d forward{};
        ObjectId primary_neighbor_id = kInvalidObjectId;
        ObjectId secondary_neighbor_id = kInvalidObjectId;
        double score = -1.0;
        double continuity = -1.0;
      };
      BestCandidate best{};

      auto consider_forward = [&](Vec3d forward, ObjectId primary_neighbor_id, ObjectId secondary_neighbor_id) {
        if (!Normalize(&forward)) {
          return;
        }
        double score = 0.0;
        for (const DirectionCandidate& candidate : directions) {
          score += std::abs(Dot(forward, candidate.dir));
        }
        Vec3d row_axis = ComputeLateralAxis(forward);
        if (!Normalize(&row_axis)) {
          return;
        }
        const double continuity = std::abs(Dot(row_axis, normalize_forward_xy(previous_support_axis)));
        const bool better_score = score > best.score + 1e-9;
        const bool equal_score = std::abs(score - best.score) <= 1e-9;
        const bool better_continuity = continuity > best.continuity + 1e-9;
        const bool equal_continuity = std::abs(continuity - best.continuity) <= 1e-9;
        const bool better_ids =
            std::tie(primary_neighbor_id, secondary_neighbor_id) <
            std::tie(best.primary_neighbor_id, best.secondary_neighbor_id);
        if (better_score || (equal_score && (better_continuity || (equal_continuity && better_ids)))) {
          best.forward = forward;
          best.primary_neighbor_id = primary_neighbor_id;
          best.secondary_neighbor_id = secondary_neighbor_id;
          best.score = score;
          best.continuity = continuity;
        }
      };

      for (const DirectionCandidate& direction : directions) {
        consider_forward(direction.dir, direction.neighbor_id, kInvalidObjectId);
      }
      for (std::size_t i = 0; i < directions.size(); ++i) {
        for (std::size_t j = i + 1; j < directions.size(); ++j) {
          consider_forward(directions[i].dir + directions[j].dir, directions[i].neighbor_id, directions[j].neighbor_id);
        }
      }

      if (best.score < 0.0) {
        return false;
      }
      return adopt_axis(best.forward, PoleSupportAxisRule::kConnectedDirectionFit, best.primary_neighbor_id,
                        best.secondary_neighbor_id);
    };
    auto maybe_preserve_existing_pair_axis = [&](const Vec3d& candidate_axis) {
      const auto it_route = route_neighbors_by_node.find(node_id);
      const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
      if (route_degree > 1 || !has_previous_support_axis) {
        return false;
      }
      const std::vector<ObjectId> preserved_neighbors = existing_continuation_neighbors_for_orientation(node_id);
      if (preserved_neighbors.size() < 2) {
        return false;
      }
      double best_continuity = -1.0;
      ObjectId best_primary_neighbor_id = kInvalidObjectId;
      ObjectId best_secondary_neighbor_id = kInvalidObjectId;
      Vec3d best_axis{};
      for (std::size_t i = 0; i < preserved_neighbors.size(); ++i) {
        const ObjectId primary_neighbor_id = preserved_neighbors[i];
        const ObjectId secondary_neighbor_id = preserved_neighbors[1 - i];
        Vec3d preserved_axis{};
        if (!candidate_support_axis_from_neighbor(primary_neighbor_id, &preserved_axis)) {
          continue;
        }
        const double continuity = std::abs(Dot(preserved_axis, normalized_previous_support_axis));
        if (continuity > best_continuity + 1e-9) {
          best_continuity = continuity;
          best_primary_neighbor_id = primary_neighbor_id;
          best_secondary_neighbor_id = secondary_neighbor_id;
          best_axis = preserved_axis;
        }
      }
      if (best_primary_neighbor_id == kInvalidObjectId) {
        return false;
      }
      const double candidate_continuity = std::abs(Dot(candidate_axis, normalized_previous_support_axis));
      if (debug != nullptr && debug->support_axis_rule == PoleSupportAxisRule::kConnectedDirectionFit &&
          debug->primary_neighbor_id != kInvalidObjectId && debug->secondary_neighbor_id != kInvalidObjectId &&
          candidate_continuity >= 0.75 && candidate_continuity < 0.85) {
        chosen_axis = normalized_previous_support_axis;
        has_axis = true;
        debug->support_axis_rule = PoleSupportAxisRule::kMainChainPair;
        return true;
      }
      if (best_continuity < 0.85 || candidate_continuity >= 0.7) {
        return false;
      }
      chosen_axis = best_axis;
      has_axis = true;
      if (debug != nullptr) {
        debug->support_axis_rule = PoleSupportAxisRule::kMainChainPair;
        debug->primary_neighbor_id = best_primary_neighbor_id;
        debug->secondary_neighbor_id = best_secondary_neighbor_id;
      }
      return true;
    };

    const std::vector<ObjectId> combined_neighbors = combined_neighbors_for_node(node_id);
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    const auto preferred_pair = preferred_straight_main_pair_for_orientation(node_id);
    auto pair_straightness = [&](ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
      const Vec3d axis_a = normalize_forward_xy(current_support_position(neighbor_a_id) - center);
      const Vec3d axis_b = normalize_forward_xy(current_support_position(neighbor_b_id) - center);
      if (!std::isfinite(axis_a.x) || !std::isfinite(axis_a.y) || !std::isfinite(axis_b.x) || !std::isfinite(axis_b.y)) {
        return -2.0;
      }
      return dot(axis_a, Vec3d{-axis_b.x, -axis_b.y, -axis_b.z});
    };
    if (route_degree <= 2 &&
        preferred_pair.first != kInvalidObjectId && preferred_pair.second != kInvalidObjectId &&
        preferred_pair.first != preferred_pair.second &&
        pair_straightness(preferred_pair.first, preferred_pair.second) >= 0.95) {
      const Vec3d axis = current_support_position(preferred_pair.first) - center;
      if (adopt_axis(axis, PoleSupportAxisRule::kMainChainPair, preferred_pair.first, preferred_pair.second)) {
        return chosen_axis;
      }
    }
    const std::vector<ObjectId> connected_neighbors = connected_neighbors_for_support_axis(node_id);
    ObjectId route_peer_id = kInvalidObjectId;
    if (connected_neighbors.size() == 1) {
      route_peer_id = connected_neighbors.front();
    } else if (route_degree == 1) {
      route_peer_id = it_route->second.front();
    }
    if (route_peer_id != kInvalidObjectId) {
      const std::vector<ObjectId> peer_pair = continuation_neighbors_for_orientation(route_peer_id);
      if (peer_pair.size() >= 2 && peer_pair[0] != kInvalidObjectId && peer_pair[1] != kInvalidObjectId &&
          peer_pair[0] != peer_pair[1] && peer_pair[0] != node_id && peer_pair[1] != node_id) {
        const Vec3d peer_center = current_support_position(route_peer_id);
        const Vec3d axis = current_support_position(peer_pair[0]) - peer_center;
        if (adopt_axis(axis, PoleSupportAxisRule::kMainChainPair, peer_pair[0], peer_pair[1])) {
          return chosen_axis;
        }
      }
    }
    if (adopt_connected_direction_fit(combined_neighbors)) {
      if (maybe_preserve_existing_pair_axis(chosen_axis)) {
        return chosen_axis;
      }
      return chosen_axis;
    }

    if (adopt_connected_direction_fit(connected_neighbors)) {
      if (maybe_preserve_existing_pair_axis(chosen_axis)) {
        return chosen_axis;
      }
      return chosen_axis;
    }
    if (connected_neighbors.size() == 2) {
      const ObjectId neighbor_a = connected_neighbors[0];
      const ObjectId neighbor_b = connected_neighbors[1];
      const Vec3d axis = current_support_position(neighbor_a) - center;
      if (adopt_axis(axis, PoleSupportAxisRule::kMainChainPair, neighbor_a, neighbor_b)) {
        return chosen_axis;
      }
    }

    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const std::vector<ObjectId> continuation_neighbors = continuation_neighbors_for_orientation(node_id);
      if (continuation_neighbors.size() >= 2) {
        const Vec3d axis = current_support_position(continuation_neighbors[0]) - center;
        if (adopt_axis(axis, PoleSupportAxisRule::kMainChainPair, continuation_neighbors[0], continuation_neighbors[1])) {
          return chosen_axis;
        }
      }
      const ObjectId primary_neighbor_id = primary_neighbor_for_orientation(node_id);
      if (primary_neighbor_id != kInvalidObjectId) {
        const Vec3d axis = current_support_position(primary_neighbor_id) - center;
        if (adopt_axis(axis, PoleSupportAxisRule::kPrimaryIncident, primary_neighbor_id, kInvalidObjectId)) {
          return chosen_axis;
        }
      }
    }

    if (const auto it_route = route_neighbors_by_node.find(node_id); it_route != route_neighbors_by_node.end() &&
                                                         !it_route->second.empty()) {
      ObjectId primary_neighbor_id = it_route->second.front();
      ObjectId secondary_neighbor_id = kInvalidObjectId;
      PoleSupportAxisRule rule = PoleSupportAxisRule::kMainChainSingle;
      if (it_route->second.size() >= 2) {
        const ObjectId candidate_a = it_route->second[0];
        const ObjectId candidate_b = it_route->second[1];
        const Vec3d delta_a = current_support_position(candidate_a) - center;
        const Vec3d delta_b = current_support_position(candidate_b) - center;
        const double len2_a = delta_a.x * delta_a.x + delta_a.y * delta_a.y;
        const double len2_b = delta_b.x * delta_b.x + delta_b.y * delta_b.y;
        primary_neighbor_id = (len2_b > len2_a + 1e-9) ? candidate_b : candidate_a;
        secondary_neighbor_id = (primary_neighbor_id == candidate_a) ? candidate_b : candidate_a;
        rule = PoleSupportAxisRule::kMainChainPair;
      }
      const Vec3d axis = current_support_position(primary_neighbor_id) - center;
      if (adopt_axis(axis, rule, primary_neighbor_id, secondary_neighbor_id)) {
        return chosen_axis;
      }
    }

    if (debug != nullptr) {
      debug->support_axis_rule = PoleSupportAxisRule::kFallback;
    }
    return has_axis ? chosen_axis : Vec3d{};
  };

  const auto previous_pole_orientation_debug_records = debug_.pole_orientation_debug_records;
  debug_.pole_orientation_debug_records.clear();
  constexpr double kMainBisectorSupportAxisMaxForwardAlignment = 0.85;
  auto row_layout_yaw_override_from_debug = [&](const PoleOrientationDebugRecord& debug_record)
      -> std::optional<PortLayoutYawOverride> {
    if (debug_record.row_layout_axis_mode != RowLayoutAxisMode::kSupportAxis) {
      return std::nullopt;
    }
    Vec3d support_axis = debug_record.adopted_support_axis;
    if (!Normalize(&support_axis)) {
      return std::nullopt;
    }
    PortLayoutYawOverride override{};
    override.category = debug_record.row_layout_axis_category;
    override.yaw_deg =
        normalize_yaw_deg(std::atan2(support_axis.y, support_axis.x) * (180.0 / kPi) - 90.0);
    return override;
  };
  std::unordered_set<ObjectId> oriented_poles{};
  for (std::size_t ordered_index = 0; ordered_index < ordered_support_node_ids.size(); ++ordered_index) {
    const ObjectId node_id = ordered_support_node_ids[ordered_index];
    if (!oriented_poles.insert(node_id).second) {
      continue;
    }
    Pole* pole = edit_state_access().poles.find(node_id);
    if (pole == nullptr) {
      continue;
    }

    PoleOrientationDebugRecord debug{};
    debug.pole_id = pole->id;
    const Vec3d center = current_support_position(node_id);
    const Vec3d previous_forward = RotateAroundWorldUpDeg(WorldForward(), effective_pole_yaw_deg(*pole));
    const double previous_layout_yaw = effective_pole_layout_yaw_deg(*pole);
    std::optional<PortLayoutYawOverride> previous_row_layout_yaw_override{};
    Vec3d previous_support_axis{};
    if (const auto it_prev_debug = previous_pole_orientation_debug_records.find(pole->id);
        it_prev_debug != previous_pole_orientation_debug_records.end()) {
      previous_support_axis = it_prev_debug->second.adopted_support_axis;
      previous_row_layout_yaw_override = row_layout_yaw_override_from_debug(it_prev_debug->second);
    } else {
      previous_support_axis = side_axis_from_yaw_deg(previous_layout_yaw);
    }
    Vec3d chosen_forward = normalize_forward_xy(previous_forward);
    bool has_chosen_forward = Normalize(&chosen_forward);
    Vec3d chosen_support_axis = choose_support_axis_for_layout(node_id, center, previous_support_axis, &debug);
    if (!Normalize(&chosen_support_axis)) {
      chosen_support_axis = normalize_forward_xy(previous_support_axis);
    }
    if (ordered_index > 0) {
      const ObjectId prev_node_id = ordered_support_node_ids[ordered_index - 1];
      if (prev_node_id != node_id) {
        const Pole* prev_pole = edit_state_access().poles.find(prev_node_id);
        if (prev_pole != nullptr) {
          const auto it_prev_debug = debug_.pole_orientation_debug_records.find(prev_pole->id);
          if (it_prev_debug != debug_.pole_orientation_debug_records.end()) {
            Vec3d previous_route_axis = normalize_forward_xy(it_prev_debug->second.adopted_support_axis);
            if (Normalize(&previous_route_axis)) {
              chosen_support_axis = choose_continuous_axis(chosen_support_axis, previous_route_axis);
            }
          }
        }
      }
    }
    const bool apply_main_flow_orientation = has_existing_main_flow_context(node_id);

    if (apply_main_flow_orientation) {
      const std::vector<ObjectId> continuation_neighbors = continuation_neighbors_for_orientation(node_id);
      if (continuation_neighbors.size() >= 2) {
        const Vec3d dir_a = normalize_forward_xy(current_support_position(continuation_neighbors[0]) - center);
        const Vec3d dir_b = normalize_forward_xy(current_support_position(continuation_neighbors[1]) - center);
        Vec3d axis = normalize_forward_xy(dir_a + dir_b);
        if (Normalize(&axis)) {
          chosen_forward = choose_continuous_axis(axis, previous_forward);
          has_chosen_forward = Normalize(&chosen_forward);
          debug.rule = PoleForwardRule::kMainChainBisector;
          debug.primary_neighbor_id = continuation_neighbors[0];
          debug.secondary_neighbor_id = continuation_neighbors[1];
        } else {
          axis = normalize_forward_xy(dir_a - dir_b);
          if (Normalize(&axis)) {
            chosen_forward = choose_continuous_axis(axis, previous_forward);
            has_chosen_forward = Normalize(&chosen_forward);
            debug.rule = PoleForwardRule::kMainChainBisector;
            debug.primary_neighbor_id = continuation_neighbors[0];
            debug.secondary_neighbor_id = continuation_neighbors[1];
          }
        }
      } else {
        const ObjectId primary_neighbor_id = primary_neighbor_for_orientation(node_id);
        if (primary_neighbor_id != kInvalidObjectId) {
          Vec3d axis = normalize_forward_xy(current_support_position(primary_neighbor_id) - center);
          if (Normalize(&axis)) {
            chosen_forward = choose_continuous_axis(axis, previous_forward);
            has_chosen_forward = true;
            debug.rule = active_junction_by_node.contains(node_id) ? PoleForwardRule::kPrimaryIncident
                                                                   : PoleForwardRule::kMainChainSingle;
            debug.primary_neighbor_id = primary_neighbor_id;
          }
        } else {
          debug.rule = PoleForwardRule::kFallback;
        }
      }
    }

    if (!has_chosen_forward) {
      chosen_forward = normalize_forward_xy(previous_forward);
      debug.rule = PoleForwardRule::kFallback;
    }
    if (debug.rule == PoleForwardRule::kMainChainBisector) {
      bool preserve_pair_axis = false;
      if (debug.support_axis_rule == PoleSupportAxisRule::kMainChainPair && debug.primary_neighbor_id != kInvalidObjectId) {
        Vec3d preserved_pair_axis = normalize_forward_xy(current_support_position(debug.primary_neighbor_id) - center);
        if (Normalize(&preserved_pair_axis)) {
          preserved_pair_axis = ComputeLateralAxis(preserved_pair_axis);
          if (Normalize(&preserved_pair_axis)) {
            preserved_pair_axis = choose_continuous_axis(preserved_pair_axis, previous_support_axis);
            if (Normalize(&preserved_pair_axis)) {
              preserve_pair_axis = std::abs(Dot(preserved_pair_axis, chosen_support_axis)) >= 0.95;
            }
          }
        }
      }
      if (!preserve_pair_axis) {
      Vec3d normalized_support_axis = normalize_forward_xy(chosen_support_axis);
      if (Normalize(&normalized_support_axis)) {
        const double forward_alignment = std::abs(Dot(normalized_support_axis, chosen_forward));
        if (forward_alignment > kMainBisectorSupportAxisMaxForwardAlignment) {
          Vec3d lateral_axis = ComputeLateralAxis(chosen_forward);
          if (Normalize(&lateral_axis)) {
            chosen_support_axis = choose_continuous_axis(lateral_axis, normalized_support_axis);
            if (Normalize(&chosen_support_axis) && debug.secondary_neighbor_id != kInvalidObjectId) {
              debug.support_axis_rule = PoleSupportAxisRule::kMainChainPair;
            }
          }
        }
      }
      }
    }
    debug.adopted_forward = chosen_forward;
    debug.adopted_support_axis = chosen_support_axis;
    const BundleCategoryTieBreakKey row_layout_key = row_layout_axis_key_for_node(node_id);
    debug.row_layout_axis_mode = row_layout_axis_mode_for_key(row_layout_key);
    debug.row_layout_axis_category = row_layout_key.category;
    debug_.pole_orientation_debug_records[pole->id] = debug;

    const double next_layout_yaw = effective_pole_layout_yaw_deg(*pole);
    const double layout_yaw_delta = normalize_yaw_deg(next_layout_yaw - previous_layout_yaw);
    const std::optional<PortLayoutYawOverride> next_row_layout_yaw_override = row_layout_yaw_override_from_debug(debug);
    const bool row_layout_override_changed =
        (!previous_row_layout_yaw_override.has_value() && next_row_layout_yaw_override.has_value()) ||
        (previous_row_layout_yaw_override.has_value() && !next_row_layout_yaw_override.has_value()) ||
        (previous_row_layout_yaw_override.has_value() && next_row_layout_yaw_override.has_value() &&
         (previous_row_layout_yaw_override->category != next_row_layout_yaw_override->category ||
          std::abs(normalize_yaw_deg(next_row_layout_yaw_override->yaw_deg - previous_row_layout_yaw_override->yaw_deg)) >
              1e-6));

    if (!has_chosen_forward || has_pole_orientation_override(pole->id)) {
      if (std::abs(layout_yaw_delta) > 1e-6 || row_layout_override_changed) {
        const Pole old_pole = *pole;
        refresh_owned_endpoints_from_pole(
            pole->id, &result.change_set, &old_pole,
            previous_row_layout_yaw_override.has_value() ? &*previous_row_layout_yaw_override : nullptr);
      }
      continue;
    }

    const double desired_yaw = normalize_yaw_deg(std::atan2(chosen_forward.y, chosen_forward.x) * (180.0 / kPi));
    double yaw_delta = desired_yaw - pole->world_transform.rotation_euler_deg.z;
    yaw_delta = std::fmod(yaw_delta + 540.0, 360.0) - 180.0;
    if (std::abs(yaw_delta) <= 1e-6) {
      if (std::abs(layout_yaw_delta) > 1e-6 || row_layout_override_changed) {
        const Pole old_pole = *pole;
        refresh_owned_endpoints_from_pole(
            pole->id, &result.change_set, &old_pole,
            previous_row_layout_yaw_override.has_value() ? &*previous_row_layout_yaw_override : nullptr);
      }
      continue;
    }

    const Pole old_pole = *pole;
    pole->world_transform.rotation_euler_deg.z = desired_yaw;
    finalize_pole_transform_update(pole->id, old_pole, &result.change_set);
  }

  constexpr double kThroughPairStraightnessThreshold = 0.3;
  constexpr double kExplicitMiddleRouteNearStraightThreshold = 0.95;
  auto category_same_level_margin_m = [](ConnectionCategory category) {
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
  };
  auto classify_junction = [&](ObjectId node_id) {
    JunctionRelation relation{};
    relation.node_id = node_id;

    const JunctionInfo* junction = nullptr;
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      junction = it->second;
    }

    const std::vector<ObjectId> route_neighbors =
        (route_neighbors_by_node.contains(node_id) ? route_neighbors_by_node.at(node_id) : std::vector<ObjectId>{});
    const std::vector<ObjectId> combined_neighbors = combined_neighbors_for_node(node_id);
    relation.is_cross_like = combined_neighbors.size() >= 4;

    auto explicit_middle_route_corner_score = [&]() -> std::optional<double> {
      if (request.path.node_specs.size() != 1) {
        return std::nullopt;
      }
      if (route_neighbors.size() != 2) {
        return std::nullopt;
      }

      const BackboneInputSpec::NodeSpec* explicit_node_spec = nullptr;
      const Vec3d node_position = current_support_position(node_id);
      for (const BackboneInputSpec::NodeSpec& node_spec : request.path.node_specs) {
        if (node_spec.support_kind != SupportKind::kPole) {
          continue;
        }
        if (node_spec.point_index <= 0 ||
            node_spec.point_index >= static_cast<int>(request.path.polyline.size()) - 1) {
          continue;
        }
        const Vec3d point = request.path.polyline[static_cast<std::size_t>(node_spec.point_index)];
        const Vec3d delta = point - node_position;
        const bool same_point = std::abs(delta.x) <= 1e-6 && std::abs(delta.y) <= 1e-6;
        if (!same_point) {
          continue;
        }
        if (explicit_node_spec != nullptr) {
          return std::nullopt;
        }
        explicit_node_spec = &node_spec;
      }
      if (explicit_node_spec == nullptr) {
        return std::nullopt;
      }

      const int path_point_index = explicit_node_spec->point_index;
      if (path_point_index <= 0 ||
          path_point_index >= static_cast<int>(request.path.polyline.size()) - 1) {
        return std::nullopt;
      }

      const Vec3d center = request.path.polyline[static_cast<std::size_t>(path_point_index)];
      const Vec3d first_dir =
          normalize_forward_xy(request.path.polyline[static_cast<std::size_t>(path_point_index - 1)] - center);
      const Vec3d second_dir =
          normalize_forward_xy(request.path.polyline[static_cast<std::size_t>(path_point_index + 1)] - center);
      if (!std::isfinite(first_dir.x) || !std::isfinite(first_dir.y) || !std::isfinite(second_dir.x) ||
          !std::isfinite(second_dir.y)) {
        return std::nullopt;
      }
      return dot(first_dir, Vec3d{-second_dir.x, -second_dir.y, -second_dir.z});
    };

    if (combined_neighbors.empty()) {
      return relation;
    }

    struct Candidate {
      ObjectId neighbor_id = kInvalidObjectId;
      Vec3d dir{};
      bool is_route_neighbor = false;
      bool preserves_existing_continuity = false;
      bool continuity_primary = false;
      int order = std::numeric_limits<int>::max();
      std::uint64_t source_session_id = std::numeric_limits<std::uint64_t>::max();
      BundleCategoryTieBreakKey bundle_category_key{};
    };

    std::vector<Candidate> candidates{};
    candidates.reserve(combined_neighbors.size());
    const Vec3d center = current_support_position(node_id);
    const std::vector<ObjectId> existing_continuity_neighbors =
        existing_continuation_neighbors_for_order(node_id);
    const bool has_existing_continuity_pair =
        existing_continuity_neighbors.size() >= 2 &&
        existing_continuity_neighbors[0] != existing_continuity_neighbors[1];
    auto pair_straightness_score = [&](ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
      const Vec3d axis_a = normalize_forward_xy(current_support_position(neighbor_a_id) - center);
      const Vec3d axis_b = normalize_forward_xy(current_support_position(neighbor_b_id) - center);
      if (!std::isfinite(axis_a.x) || !std::isfinite(axis_a.y) || !std::isfinite(axis_b.x) ||
          !std::isfinite(axis_b.y)) {
        return -2.0;
      }
      return dot(axis_a, Vec3d{-axis_b.x, -axis_b.y, -axis_b.z});
    };
    const double existing_continuity_score =
        has_existing_continuity_pair
            ? pair_straightness_score(existing_continuity_neighbors[0], existing_continuity_neighbors[1])
            : -2.0;
    const bool preserve_existing_continuity =
        has_existing_continuity_pair &&
        existing_continuity_score + 1e-9 >= kThroughPairStraightnessThreshold;
    const ObjectId continuity_primary_neighbor = existing_primary_neighbor_for_order(node_id);
    for (ObjectId neighbor_id : combined_neighbors) {
      const Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
      if (!std::isfinite(dir.x) || !std::isfinite(dir.y)) {
        continue;
      }
      Candidate candidate{};
      candidate.neighbor_id = neighbor_id;
      candidate.dir = dir;
      candidate.is_route_neighbor =
          std::find(route_neighbors.begin(), route_neighbors.end(), neighbor_id) != route_neighbors.end();
      candidate.continuity_primary = (neighbor_id == continuity_primary_neighbor);
      candidate.preserves_existing_continuity =
          preserve_existing_continuity &&
          std::find(existing_continuity_neighbors.begin(), existing_continuity_neighbors.end(), neighbor_id) !=
              existing_continuity_neighbors.end();
      candidate.bundle_category_key =
          bundle_category_key_for_neighbor(node_id, neighbor_id, candidate.is_route_neighbor);
      if (junction != nullptr) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.neighbor_node_id == neighbor_id) {
            candidate.order = (incident.order >= 0) ? incident.order : candidate.order;
            candidate.source_session_id = incident.source_session_id;
            candidate.continuity_primary =
                candidate.continuity_primary || incident.primary || incident.order == 0;
            break;
          }
        }
      }
      candidates.push_back(candidate);
    }
    relation.route_incident_count = static_cast<int>(route_neighbors.size());
    relation.incidents.reserve(candidates.size());
    for (const Candidate& candidate : candidates) {
      JunctionIncidentRelation incident{};
      incident.neighbor_node_id = candidate.neighbor_id;
      incident.kind = JunctionRelationKind::kNone;
      incident.in_route = candidate.is_route_neighbor;
      relation.incidents.push_back(incident);
    }
    if (const std::optional<double> route_corner_score = explicit_middle_route_corner_score();
        route_corner_score.has_value() && *route_corner_score + 1e-9 < kExplicitMiddleRouteNearStraightThreshold) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route) {
          incident.kind = JunctionRelationKind::kCornerContinuation;
        }
      }
      return relation;
    }
    if (route_neighbors.size() == 1 && relation.is_cross_like) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route) {
          incident.kind = JunctionRelationKind::kThroughMain;
        }
      }
      return relation;
    }
    if (candidates.size() == 1) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route) {
          incident.kind = JunctionRelationKind::kThroughMain;
        }
      }
      return relation;
    }
    if (candidates.size() < 2) {
      if (route_neighbors.size() == 1) {
        for (JunctionIncidentRelation& incident : relation.incidents) {
          if (incident.in_route) {
            incident.kind = JunctionRelationKind::kThroughMain;
          }
        }
      }
      return relation;
    }

    double best_score = -2.0;
    int best_i = -1;
    int best_j = -1;
    bool best_used_priority_tiebreak = false;
    auto candidate_stable_key = [](const Candidate& candidate) {
      return std::tuple<int, std::uint64_t, ObjectId>{
          candidate.order, candidate.source_session_id, candidate.neighbor_id};
    };
    auto pair_matches_existing_continuity = [&](const Candidate& a, const Candidate& b) {
      if (!preserve_existing_continuity) {
        return false;
      }
      return a.preserves_existing_continuity && b.preserves_existing_continuity;
    };
    auto pair_bundle_category_key = [&](const Candidate& a, const Candidate& b) {
      const auto key_a = bundle_category_key_tuple(a.bundle_category_key);
      const auto key_b = bundle_category_key_tuple(b.bundle_category_key);
      return (key_b < key_a) ? std::tuple{key_b, key_a} : std::tuple{key_a, key_b};
    };
    auto pair_stable_key = [&](const Candidate& a, const Candidate& b) {
      const auto key_a = candidate_stable_key(a);
      const auto key_b = candidate_stable_key(b);
      return (key_b < key_a) ? std::tuple{key_b, key_a} : std::tuple{key_a, key_b};
    };
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        const double straight_score =
            dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y, -candidates[j].dir.z});
        const bool current_preserves_existing =
            pair_matches_existing_continuity(candidates[i], candidates[j]);
        const bool best_preserves_existing =
            (best_i >= 0 && best_j >= 0) &&
            pair_matches_existing_continuity(candidates[static_cast<std::size_t>(best_i)],
                                             candidates[static_cast<std::size_t>(best_j)]);
        bool choose_current = false;
        bool used_priority_tiebreak = false;
        if (best_i < 0 || best_j < 0) {
          choose_current = true;
        } else if (current_preserves_existing != best_preserves_existing) {
          choose_current = current_preserves_existing;
          used_priority_tiebreak = true;
        } else if (straight_score > best_score + 1e-9) {
          choose_current = true;
        } else if (std::abs(straight_score - best_score) <= 1e-9) {
          const auto current_bundle_key =
              pair_bundle_category_key(candidates[i], candidates[j]);
          const auto best_bundle_key =
              pair_bundle_category_key(candidates[static_cast<std::size_t>(best_i)],
                                       candidates[static_cast<std::size_t>(best_j)]);
          if (current_bundle_key != best_bundle_key) {
            choose_current = current_bundle_key < best_bundle_key;
            used_priority_tiebreak = true;
          } else {
            const auto current_stable_key =
                pair_stable_key(candidates[i], candidates[j]);
            const auto best_stable_key =
                pair_stable_key(candidates[static_cast<std::size_t>(best_i)],
                                candidates[static_cast<std::size_t>(best_j)]);
            if (current_stable_key < best_stable_key) {
              choose_current = true;
              used_priority_tiebreak = true;
            }
          }
        }
        if (!choose_current) {
          continue;
        }
        best_score = straight_score;
        best_i = static_cast<int>(i);
        best_j = static_cast<int>(j);
        best_used_priority_tiebreak = used_priority_tiebreak;
      }
    }

    relation.through_pair.straightness_score = best_score;
    for (JunctionIncidentRelation& incident : relation.incidents) {
      incident.straightness_score = best_score;
    }
    if (best_i < 0 || best_j < 0 || !(best_score + 1e-9 >= kThroughPairStraightnessThreshold)) {
      const bool has_existing_straightish_continuity =
          existing_continuation_neighbors_for_orientation(node_id).size() >= 2;
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route && route_neighbors.size() >= 2) {
          incident.kind = has_existing_straightish_continuity
                              ? JunctionRelationKind::kSideBranch
                              : JunctionRelationKind::kCornerContinuation;
        } else if (incident.in_route && combined_neighbors.size() == 1) {
          incident.kind = JunctionRelationKind::kThroughMain;
        } else if (incident.in_route && route_neighbors.size() == 1) {
          incident.kind = JunctionRelationKind::kSideBranch;
        }
      }
      return relation;
    }

    relation.through_pair.neighbor_a_id = candidates[static_cast<std::size_t>(best_i)].neighbor_id;
    relation.through_pair.neighbor_b_id = candidates[static_cast<std::size_t>(best_j)].neighbor_id;
    relation.through_pair.accepted = true;
    relation.through_pair.used_semantic_tiebreak = best_used_priority_tiebreak;

    auto in_through_pair = [&](ObjectId neighbor_id) {
      return neighbor_id == relation.through_pair.neighbor_a_id || neighbor_id == relation.through_pair.neighbor_b_id;
    };
    for (JunctionIncidentRelation& incident : relation.incidents) {
      incident.in_through_pair = in_through_pair(incident.neighbor_node_id);
      incident.used_semantic_tiebreak =
          relation.through_pair.used_semantic_tiebreak && incident.in_through_pair;
      if (in_through_pair(incident.neighbor_node_id)) {
        incident.kind = JunctionRelationKind::kThroughMain;
      } else if (relation.is_cross_like) {
        incident.kind = JunctionRelationKind::kCrossUnderpass;
      } else if (incident.in_route) {
        incident.kind = JunctionRelationKind::kSideBranch;
      }
    }
    auto candidate_for_neighbor = [&](ObjectId neighbor_id) -> const Candidate* {
      for (const Candidate& candidate : candidates) {
        if (candidate.neighbor_id == neighbor_id) {
          return &candidate;
        }
      }
      return nullptr;
    };
    auto incident_order_key = [&](const JunctionIncidentRelation& incident) {
      const Candidate* candidate = candidate_for_neighbor(incident.neighbor_node_id);
      const bool preserves_existing = (candidate != nullptr) ? candidate->preserves_existing_continuity : false;
      const bool continuity_primary = (candidate != nullptr) ? candidate->continuity_primary : false;
      const auto bundle_key =
          (candidate != nullptr) ? bundle_category_key_tuple(candidate->bundle_category_key)
                                 : std::tuple<int, int>{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
      const auto stable_key = (candidate != nullptr)
                                  ? candidate_stable_key(*candidate)
                                  : std::tuple<int, std::uint64_t, ObjectId>{
                                        std::numeric_limits<int>::max(),
                                        std::numeric_limits<std::uint64_t>::max(),
                                        incident.neighbor_node_id};
      return std::tuple<int, int, int, decltype(bundle_key), decltype(stable_key)>{
          preserves_existing ? 0 : 1,
          incident.in_through_pair ? 0 : 1,
          continuity_primary ? 0 : 1,
          bundle_key,
          stable_key};
    };
    std::sort(relation.incidents.begin(), relation.incidents.end(),
              [&](const JunctionIncidentRelation& lhs, const JunctionIncidentRelation& rhs) {
                return incident_order_key(lhs) < incident_order_key(rhs);
              });
    if (relation.through_pair.accepted && relation.incidents.size() >= 2) {
      relation.through_pair.neighbor_a_id = relation.incidents[0].neighbor_node_id;
      relation.through_pair.neighbor_b_id = relation.incidents[1].neighbor_node_id;
    }
    return relation;
  };
  auto find_incident_relation_ptr = [&](JunctionRelation& relation, ObjectId peer_id) -> JunctionIncidentRelation* {
    for (JunctionIncidentRelation& incident : relation.incidents) {
      if (incident.neighbor_node_id == peer_id) {
        return &incident;
      }
    }
    return nullptr;
  };
  auto find_incident_relation = [&](const JunctionRelation& relation, ObjectId peer_id) -> JunctionRelationKind {
    for (const JunctionIncidentRelation& incident : relation.incidents) {
      if (incident.neighbor_node_id == peer_id) {
        return incident.kind;
      }
    }
    return JunctionRelationKind::kNone;
  };
  auto annotate_same_level_feasibility = [&](const BackboneBundlePlan& plan,
                                             std::unordered_map<ObjectId, JunctionRelation>* relations_by_node) {
    if (relations_by_node == nullptr) {
      return;
    }
    const bool bundle_like = plan.continuity_class == ContinuityCategoryClass::kBundleLike;
    const double envelope_width_m = std::max(0, plan.count - 1) * std::max(0.0, plan.spacing_m);
    const double required_clearance_m = envelope_width_m + category_same_level_margin_m(plan.category);
    const double probe_distance_m = std::clamp(0.18 + envelope_width_m * 0.35 + required_clearance_m * 0.15, 0.18, 0.42);
    auto probe_point_for = [&](ObjectId node_id, ObjectId neighbor_id) {
      const Vec3d center = current_support_position(node_id);
      const Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
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

    for (auto& [node_id, relation] : *relations_by_node) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        incident.continuity_class = plan.continuity_class;
        incident.default_lower_required = false;
        incident.same_level_feasible = true;
        incident.projected_spacing_topview_m = -1.0;
        incident.required_clearance_m = required_clearance_m;
        incident.infeasible_reason = SameLevelFeasibilityReason::kNone;
      }
      if (relation.incidents.size() < 2) {
        continue;
      }

      for (const JunctionIncidentRelation& incident : relation.incidents) {
        const bool skip_through_pair_main =
            incident.kind == JunctionRelationKind::kThroughMain && incident.in_through_pair;
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
        JunctionIncidentRelation* mutable_incident = find_incident_relation_ptr(relation, incident.neighbor_node_id);
        if (mutable_incident == nullptr) {
          continue;
        }
        if (bundle_like && is_non_through_relation &&
            (incident.in_route || allow_non_route_bundle_lower)) {
          mutable_incident->default_lower_required = true;
          mutable_incident->same_level_feasible = false;
          mutable_incident->infeasible_reason = SameLevelFeasibilityReason::kBundleRule;
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
        mutable_incident->projected_spacing_topview_m = min_projected_spacing_m;
        mutable_incident->required_clearance_m = required_clearance_m;
        if (!(std::isfinite(min_projected_spacing_m) && min_projected_spacing_m + 1e-9 >= required_clearance_m)) {
          mutable_incident->same_level_feasible = false;
          mutable_incident->infeasible_reason =
              (std::isfinite(min_projected_spacing_m) && min_projected_spacing_m + 1e-9 < envelope_width_m)
                  ? SameLevelFeasibilityReason::kEnvelopeOverlap
                  : SameLevelFeasibilityReason::kNearNodeClearance;
        }
      }
    }
  };
  auto classify_edge_flow_at_node = [&](ObjectId node_id, ObjectId peer_id) {
    EdgeFlowInfo info{};
    const JunctionRelation relation = classify_junction(node_id);
    const JunctionRelationKind relation_kind = find_incident_relation(relation, peer_id);
    const bool relation_uses_existing_continuity =
        relation.through_pair.accepted &&
        std::any_of(relation.incidents.begin(), relation.incidents.end(),
                    [](const JunctionIncidentRelation& incident) {
                      return incident.in_through_pair && !incident.in_route;
                    });
    switch (relation_kind) {
    case JunctionRelationKind::kCrossUnderpass:
      info.kind = BackboneFlowKind::kBranch;
      info.rule = BackboneFlowDecisionRule::kJunctionOrderBranch;
      info.underpass_at_cross = true;
      return info;
    case JunctionRelationKind::kSideBranch:
      info.kind = BackboneFlowKind::kBranch;
      info.rule = relation_uses_existing_continuity
                      ? BackboneFlowDecisionRule::kExistingChainBranch
                      : BackboneFlowDecisionRule::kJunctionOrderBranch;
      return info;
    case JunctionRelationKind::kCornerContinuation:
      info.kind = BackboneFlowKind::kMain;
      info.rule = BackboneFlowDecisionRule::kDefaultMain;
      return info;
    case JunctionRelationKind::kThroughMain:
      info.kind = BackboneFlowKind::kMain;
      info.rule = relation_uses_existing_continuity
                      ? BackboneFlowDecisionRule::kExistingChainMain
                      : (relation.through_pair.accepted ? BackboneFlowDecisionRule::kJunctionOrderMain
                                                        : BackboneFlowDecisionRule::kDefaultMain);
      return info;
    case JunctionRelationKind::kNone:
    default:
      return info;
    }
  };
  auto classify_edge_flow = [&](ObjectId node_a, ObjectId node_b) {
    const EdgeFlowInfo flow_a = classify_edge_flow_at_node(node_a, node_b);
    const EdgeFlowInfo flow_b = classify_edge_flow_at_node(node_b, node_a);
    if (flow_a.kind == BackboneFlowKind::kBranch) {
      return flow_a;
    }
    if (flow_b.kind == BackboneFlowKind::kBranch) {
      return flow_b;
    }
    if (flow_a.rule != BackboneFlowDecisionRule::kDefaultMain) {
      return flow_a;
    }
    if (flow_b.rule != BackboneFlowDecisionRule::kDefaultMain) {
      return flow_b;
    }
    return flow_a;
  };

  const BackboneDecisionPhaseOutput decision_phase = run_backbone_decision_phase({
      ordered_support_node_ids,
      existing_junction_by_node,
      existing_prioritized_session_by_node,
      existing_incident_session_by_node,
      generation_backbone,
      active_junction_by_node,
      session_id,
      classify_edge_flow,
      classify_junction,
  });
  auto path_relation_kind_at = [&](std::size_t node_index) {
    if (node_index >= ordered_support_node_ids.size()) {
      return JunctionRelationKind::kNone;
    }
    const ObjectId node_id = ordered_support_node_ids[node_index];
    const JunctionRelation& relation = decision_phase.junction_relations_in_path_order[node_index];
    const ObjectId prev_node_id = (node_index > 0) ? ordered_support_node_ids[node_index - 1] : kInvalidObjectId;
    const ObjectId next_node_id =
        (node_index + 1 < ordered_support_node_ids.size()) ? ordered_support_node_ids[node_index + 1] : kInvalidObjectId;
    JunctionRelationKind kind = JunctionRelationKind::kNone;
    if (prev_node_id != kInvalidObjectId) {
      kind = find_incident_relation(relation, prev_node_id);
    }
    if (next_node_id != kInvalidObjectId) {
      const JunctionRelationKind next_kind = find_incident_relation(relation, next_node_id);
      auto rank = [](JunctionRelationKind relation_kind) {
        switch (relation_kind) {
        case JunctionRelationKind::kCrossUnderpass:
          return 4;
        case JunctionRelationKind::kSideBranch:
          return 3;
        case JunctionRelationKind::kCornerContinuation:
          return 2;
        case JunctionRelationKind::kThroughMain:
          return 1;
        case JunctionRelationKind::kNone:
        default:
          return 0;
        }
      };
      if (rank(next_kind) > rank(kind)) {
        kind = next_kind;
      }
    }
    if (kind == JunctionRelationKind::kNone && node_id != kInvalidObjectId) {
      const auto it_route = route_neighbors_by_node.find(node_id);
      if (it_route != route_neighbors_by_node.end() && !it_route->second.empty()) {
        kind = (it_route->second.size() >= 2) ? JunctionRelationKind::kCornerContinuation
                                              : JunctionRelationKind::kThroughMain;
      }
    }
    return kind;
  };

  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };
  auto count_existing_segment_spans = [&](ObjectId node_a, ObjectId node_b, const BackboneBundlePlan& plan) -> int {
    int count = 0;
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.layer != plan.layer || span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != plan.template_id) {
        continue;
      }
      const Port* pa = edit_state_access().ports.find(span.port_a_id);
      const Port* pb = edit_state_access().ports.find(span.port_b_id);
      if (pa == nullptr || pb == nullptr) {
        continue;
      }
      const ObjectId span_node_a = resolve_span_endpoint_node(span, pa, true);
      const ObjectId span_node_b = resolve_span_endpoint_node(span, pb, false);
      const bool direct = (span_node_a == node_a && span_node_b == node_b);
      const bool reverse = (span_node_a == node_b && span_node_b == node_a);
      if (direct || reverse) {
        ++count;
      }
    }
    return count;
  };

  auto run_materialization_phase = [&](const BackboneDecisionPhaseOutput& phase)
      -> EditResult<BackboneMaterializationPhaseOutput> {
    EditResult<BackboneMaterializationPhaseOutput> phase_result{};
    phase_result.value.junction_relations_by_node = phase.junction_relations_by_node;

    for (const BackboneBundlePlan& plan : active_bundle_plans) {
      int missing_total = 0;
      std::size_t first_missing_segment = ordered_support_node_ids.size();
      for (std::size_t i = 0; i + 1 < ordered_support_node_ids.size(); ++i) {
        const int existing_count =
            count_existing_segment_spans(ordered_support_node_ids[i], ordered_support_node_ids[i + 1], plan);
        const int missing = std::max(0, plan.count - existing_count);
        missing_total += missing;
        if (missing > 0 && first_missing_segment == ordered_support_node_ids.size()) {
          first_missing_segment = i;
        }
      }
      if (missing_total <= 0) {
        continue;
      }

      EditResult<ObjectId> bundle_result = AddBundle(plan.count, plan.spacing_m, plan.template_id);
      if (!bundle_result.ok) {
        *this = snapshot;
        phase_result.error = bundle_result.error;
        return phase_result;
      }
      const ObjectId bundle_id = bundle_result.value;
      append_change_set(phase_result.value.change_set, bundle_result.change_set);
      phase_result.value.bundle_ids.push_back(bundle_id);
      if (phase_result.value.primary_bundle_id == kInvalidObjectId) {
        phase_result.value.primary_bundle_id = bundle_id;
      }

      if (first_missing_segment >= ordered_support_node_ids.size() - 1) {
        continue;
      }
      std::unordered_map<ObjectId, JunctionRelation> plan_junction_relations_by_node =
          phase_result.value.junction_relations_by_node;
      annotate_same_level_feasibility(plan, &plan_junction_relations_by_node);
      for (std::size_t run_start = first_missing_segment; run_start + 1 < ordered_support_node_ids.size();) {
        const EdgeFlowInfo flow_info = phase.edge_flow_by_segment[run_start];
        std::size_t run_end = run_start;
        while (run_end + 1 < ordered_support_node_ids.size() - 1 &&
               phase.edge_flow_by_segment[run_end + 1].kind == flow_info.kind) {
          ++run_end;
        }

        std::vector<ObjectId> local_support_nodes{};
        local_support_nodes.insert(local_support_nodes.end(),
                                   ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(run_start),
                                   ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(run_end + 2));
        const std::uint64_t variation_flow_key =
            make_flow_variation_key(session_id, plan.template_id, flow_info.kind, local_support_nodes.front(),
                                    local_support_nodes.back());

        std::vector<SegmentLaneAssignment> lane_assignments{};
        std::vector<BackboneEdgeOrientation> edge_orientations{};
        BackboneLoweringPolicy lowering_policy{};
        bool relation_has_lowering_candidate = false;
        for (std::size_t node_offset = run_start; node_offset <= run_end + 1; ++node_offset) {
          if (node_offset >= ordered_support_node_ids.size()) {
            continue;
          }
          const ObjectId node_id = ordered_support_node_ids[node_offset];
          const auto it_relation = plan_junction_relations_by_node.find(node_id);
          if (it_relation == plan_junction_relations_by_node.end()) {
            continue;
          }
          for (const JunctionIncidentRelation& incident : it_relation->second.incidents) {
            if (!incident.in_route || incident.same_level_feasible) {
              continue;
            }
            if (incident.kind != JunctionRelationKind::kThroughMain && incident.kind != JunctionRelationKind::kNone) {
              relation_has_lowering_candidate = true;
            }
          }
        }
        const bool enable_uniform_lowering = plan.enable_branch_down_offset && relation_has_lowering_candidate;
        lowering_policy.enable_cross_underpass = enable_uniform_lowering;
        lowering_policy.enable_branch_support = enable_uniform_lowering;
        lowering_policy.enable_acute_corner = enable_uniform_lowering;
        lowering_policy.offset_m =
            enable_uniform_lowering ? generation::detail::BranchDownOffsetForCategory(plan.category) : 0.0;
        EditResult<std::vector<ObjectId>> spans_result = generate_grouped_spans_between_support_nodes(
            local_support_nodes, support_node_by_id, bundle_id, plan.category, plan.count, plan.spacing_m, true,
            plan.allow_mirror, plan.order_decision_policy, flow_info.kind, lowering_policy,
            &plan_junction_relations_by_node, &lane_assignments, &edge_orientations, plan.template_id);
        if (!spans_result.ok) {
          *this = snapshot;
          phase_result.error = spans_result.error;
          return phase_result;
        }
        append_change_set(phase_result.value.change_set, spans_result.change_set);
        for (std::size_t i = 0; i < lane_assignments.size(); ++i) {
          lane_assignments[i].segment_index += run_start;
          lane_assignments[i].variation_flow_key = variation_flow_key;
          lane_assignments[i].flow_decision_rule = phase.edge_flow_by_segment[run_start + i].rule;
        }
        for (std::size_t i = 0; i < edge_orientations.size(); ++i) {
          edge_orientations[i].variation_flow_key = variation_flow_key;
          edge_orientations[i].flow_decision_rule = phase.edge_flow_by_segment[run_start + i].rule;
        }
        phase_result.value.lane_assignments.insert(phase_result.value.lane_assignments.end(), lane_assignments.begin(),
                                                   lane_assignments.end());
        phase_result.value.edge_orientations.insert(phase_result.value.edge_orientations.end(),
                                                    edge_orientations.begin(), edge_orientations.end());
        std::vector<SpanSupportLayoutDecisionSeed> seeded_support_layouts =
            build_seed_generated_support_layouts(*this, edit_state_access(), spans_result.value, lane_assignments,
                                                 variation_flow_key);
        for (SpanSupportLayoutDecisionSeed& layout : seeded_support_layouts) {
          cache_span_support_layout_seed(std::move(layout));
        }

        for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
          const ObjectId span_id = spans_result.value[i];
          Span* span = edit_state_access().spans.find(span_id);
          if (span != nullptr) {
            span->layer = plan.layer;
            span->generation.generated = true;
            span->generation.source = GenerationSource::kRoadAuto;
            span->generation.generation_session_id = session_id;
            span->generation.generation_order = static_cast<std::uint32_t>(phase_result.value.generated_span_ids.size());
            span->generated_by_rule = true;
            add_unique_id(phase_result.value.change_set.updated_ids, span->id);
          }
          auto runtime_it = span_runtime_states_access().find(span_id);
          if (runtime_it != span_runtime_states_access().end()) {
            runtime_it->second.variation_flow_key = variation_flow_key;
          }
          phase_result.value.generated_span_ids.push_back(span_id);
        }

        run_start = run_end + 1;
      }
      for (const auto& [node_id, relation] : plan_junction_relations_by_node) {
        JunctionRelation& merged = phase_result.value.junction_relations_by_node[node_id];
        for (const JunctionIncidentRelation& incident : relation.incidents) {
          JunctionIncidentRelation* target = find_incident_relation_ptr(merged, incident.neighbor_node_id);
          if (target == nullptr) {
            continue;
          }
          target->continuity_class = incident.continuity_class;
          target->default_lower_required = target->default_lower_required || incident.default_lower_required;
          if (!incident.same_level_feasible &&
              (target->same_level_feasible ||
               (incident.projected_spacing_topview_m >= 0.0 &&
                (target->projected_spacing_topview_m < 0.0 ||
                 incident.projected_spacing_topview_m < target->projected_spacing_topview_m)))) {
            target->same_level_feasible = false;
            target->projected_spacing_topview_m = incident.projected_spacing_topview_m;
            target->required_clearance_m = incident.required_clearance_m;
            target->infeasible_reason = incident.infeasible_reason;
          }
        }
      }
    }

    phase_result.ok = true;
    return phase_result;
  };

  EditResult<BackboneMaterializationPhaseOutput> materialization_phase_result = run_materialization_phase(decision_phase);
  if (!materialization_phase_result.ok) {
    result.error = materialization_phase_result.error;
    return result;
  }
  BackboneMaterializationPhaseOutput materialization_phase = std::move(materialization_phase_result.value);
  append_change_set(result.change_set, materialization_phase.change_set);
  result.value.bundle_ids.insert(result.value.bundle_ids.end(), materialization_phase.bundle_ids.begin(),
                                 materialization_phase.bundle_ids.end());
  if (result.value.bundle_id == kInvalidObjectId) {
    result.value.bundle_id = materialization_phase.primary_bundle_id;
  }
  result.value.generated_span_ids.insert(result.value.generated_span_ids.end(),
                                         materialization_phase.generated_span_ids.begin(),
                                         materialization_phase.generated_span_ids.end());

  debug_.last_generation_support_nodes.clear();
  debug_.last_generation_support_nodes.reserve(generation_backbone.nodes.size());
  for (const SupportNode& node : generation_backbone.nodes) {
    if (node.support_kind != SupportKind::kPole) {
      debug_.last_generation_support_nodes.push_back(node);
    }
  }
  debug_.last_generation_lane_assignments = std::move(materialization_phase.lane_assignments);
  debug_.last_generation_edge_orientations = std::move(materialization_phase.edge_orientations);
  debug_.last_generation_junction_relations = std::move(materialization_phase.junction_relations_by_node);
  result.ok = true;
  return result;
}


} // namespace wire::core
