#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "detail_utils.hpp"
#include "support_policy.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {} // namespace

EditResult<std::vector<ObjectId>>
CoreState::generate_grouped_spans_between_support_nodes(
    const std::vector<ObjectId>& node_ids, const std::unordered_map<ObjectId, SupportNode>& support_node_by_id,
    ObjectId bundle_id, ConnectionCategory category, int conductor_count, double spacing_m, bool maintain_lane_order,
    bool allow_lane_mirror, BundleOrderPolicyKind bundle_order_policy, BackboneFlowKind flow_kind,
    const BackboneLoweringPolicy& lowering_policy,
    const std::unordered_map<ObjectId, JunctionRelation>* junction_relations_by_node,
    std::vector<SegmentLaneAssignment>* out_lane_assignments,
    std::vector<BackboneEdgeOrientation>* out_edge_orientations, BundleKind bundle_template_id) {
  EditResult<std::vector<ObjectId>> result;
  if (node_ids.size() < 2) {
    result.error = "at least 2 support nodes are required";
    return result;
  }
  const int lane_count = std::max(1, conductor_count);
  const PortLayer target_port_layer = category_to_port_layer(category);
  const bool use_lane_row_geometry = maintain_lane_order && lane_count > 1;
  struct SegmentRelationFeasibility {
    JunctionRelationKind kind = JunctionRelationKind::kNone;
    ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
    bool in_through_pair = false;
    bool default_lower_required = false;
    bool same_level_feasible = true;
    SameLevelFeasibilityReason reason = SameLevelFeasibilityReason::kNone;
    double projected_spacing_topview_m = -1.0;
    double required_clearance_m = 0.0;
  };
  auto incident_relation_for = [&](ObjectId node_id, ObjectId peer_id) -> const JunctionIncidentRelation* {
    if (junction_relations_by_node == nullptr || node_id == kInvalidObjectId || peer_id == kInvalidObjectId) {
      return nullptr;
    }
    const auto it = junction_relations_by_node->find(node_id);
    if (it == junction_relations_by_node->end()) {
      return nullptr;
    }
    for (const JunctionIncidentRelation& incident : it->second.incidents) {
      if (incident.neighbor_node_id == peer_id) {
        return &incident;
      }
    }
    return nullptr;
  };
  auto incident_relation_kind_for = [&](ObjectId node_id, ObjectId peer_id) -> JunctionRelationKind {
    const JunctionIncidentRelation* incident = incident_relation_for(node_id, peer_id);
    return (incident == nullptr) ? JunctionRelationKind::kNone : incident->kind;
  };
  auto incident_same_level_feasible_for = [&](ObjectId node_id, ObjectId peer_id) {
    const JunctionIncidentRelation* incident = incident_relation_for(node_id, peer_id);
    return (incident == nullptr) ? true : incident->same_level_feasible;
  };
  auto segment_relation_feasibility_for = [&](ObjectId node_id, ObjectId peer_id) {
    SegmentRelationFeasibility info{};
    const JunctionIncidentRelation* incident = incident_relation_for(node_id, peer_id);
    if (incident == nullptr) {
      return info;
    }
    info.kind = incident->kind;
    info.continuity_class = incident->continuity_class;
    info.in_through_pair = incident->in_through_pair;
    info.default_lower_required = incident->default_lower_required;
    info.same_level_feasible = incident->same_level_feasible;
    info.reason = incident->infeasible_reason;
    info.projected_spacing_topview_m = incident->projected_spacing_topview_m;
    info.required_clearance_m = incident->required_clearance_m;
    return info;
  };
  auto node_relation_kind_at = [&](std::size_t node_index) -> JunctionRelationKind {
    if (junction_relations_by_node != nullptr && node_index < node_ids.size()) {
      const auto it = junction_relations_by_node->find(node_ids[node_index]);
      if (it != junction_relations_by_node->end()) {
        const JunctionRelation& relation = it->second;
        auto rank = [](JunctionRelationKind kind) {
          switch (kind) {
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
        JunctionRelationKind kind = JunctionRelationKind::kNone;
        for (const JunctionIncidentRelation& incident : relation.incidents) {
          if (!incident.in_route) {
            continue;
          }
          if (rank(incident.kind) > rank(kind)) {
            kind = incident.kind;
          }
        }
        if (kind == JunctionRelationKind::kNone && relation.route_incident_count == 1) {
          kind = JunctionRelationKind::kThroughMain;
        }
        return kind;
      }
    }
    return JunctionRelationKind::kNone;
  };
  const bool has_relation_policy = junction_relations_by_node != nullptr;
  bool relation_has_branch_support = false;
  bool relation_has_acute_corner = false;
  if (has_relation_policy) {
    for (std::size_t node_index = 0; node_index < node_ids.size(); ++node_index) {
      const JunctionRelationKind kind = node_relation_kind_at(node_index);
      const ObjectId node_id = node_ids[node_index];
      const ObjectId prev_node_id = (node_index > 0) ? node_ids[node_index - 1] : kInvalidObjectId;
      const ObjectId next_node_id = (node_index + 1 < node_ids.size()) ? node_ids[node_index + 1] : kInvalidObjectId;
      const bool prev_infeasible = (prev_node_id != kInvalidObjectId) && !incident_same_level_feasible_for(node_id, prev_node_id);
      const bool next_infeasible = (next_node_id != kInvalidObjectId) && !incident_same_level_feasible_for(node_id, next_node_id);
      relation_has_branch_support =
          relation_has_branch_support ||
          ((kind == JunctionRelationKind::kSideBranch || kind == JunctionRelationKind::kCrossUnderpass) &&
           (prev_infeasible || next_infeasible));
      relation_has_acute_corner =
          relation_has_acute_corner || (kind == JunctionRelationKind::kCornerContinuation && (prev_infeasible || next_infeasible));
    }
  }
  const bool uses_branch_support =
      lowering_policy.offset_m > 1e-6 &&
      (has_relation_policy ? relation_has_branch_support
                           : (lowering_policy.enable_branch_support || lowering_policy.enable_cross_underpass));
  const bool uses_acute_corner_lowering =
      lowering_policy.offset_m > 1e-6 &&
      (has_relation_policy ? relation_has_acute_corner : lowering_policy.enable_acute_corner);
  const double effective_branch_down_offset_m = std::max(0.0, lowering_policy.offset_m);
  std::unordered_map<ObjectId, std::vector<ObjectId>> node_lane_ports_cache{};
  std::unordered_map<ObjectId, Vec3d> node_side_axis_hints{};
  auto support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state_access().poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    return {};
  };
  auto support_pole = [&](ObjectId node_id) -> const Pole* {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      if (it->second.pole_id != kInvalidObjectId) {
        if (const Pole* pole = edit_state_access().poles.find(it->second.pole_id); pole != nullptr) {
          return pole;
        }
      }
      if (it->second.support_kind != SupportKind::kPole) {
        return nullptr;
      }
    }
    return edit_state_access().poles.find(node_id);
  };
  auto support_kind = [&](ObjectId node_id) -> SupportKind {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.support_kind;
    }
    return (edit_state_access().poles.find(node_id) != nullptr) ? SupportKind::kPole : SupportKind::kMidair;
  };
  auto support_axis_for_pole = [&](const Pole& pole) -> Vec3d {
    if (const auto it = pole_orientation_debug_records_.find(pole.id); it != pole_orientation_debug_records_.end()) {
      Vec3d axis = it->second.adopted_support_axis;
      if (normalize_xy(&axis) && std::isfinite(axis.x) && std::isfinite(axis.y)) {
        return axis;
      }
    }
    Vec3d axis = side_axis_from_yaw_deg(effective_pole_layout_yaw_deg(pole));
    if (normalize_xy(&axis) && std::isfinite(axis.x) && std::isfinite(axis.y)) {
      return axis;
    }
    return Vec3d{0.0, 1.0, 0.0};
  };
  auto layout_yaw_for_pole = [&](const Pole& pole) { return effective_pole_layout_yaw_deg(pole); };
  auto highest_owned_port_z_for_pole = [&](const Pole& pole) -> std::optional<double> {
    const auto it_ports = relation_index_access().ports_by_pole.find(pole.id);
    if (it_ports == relation_index_access().ports_by_pole.end()) {
      return std::nullopt;
    }
    const PoleFrame frame = BuildPoleFrame(pole.world_transform, layout_yaw_for_pole(pole));
    const int target_layer = generation::detail::TemplateLayerForCategory(category);
    double best_z = -std::numeric_limits<double>::infinity();
    for (ObjectId port_id : it_ports->second) {
      const Port* port = edit_state_access().ports.find(port_id);
      if (port == nullptr || port->owner_pole_id != pole.id) {
        continue;
      }
      const bool layer_match = (port->layer == target_port_layer);
      const bool template_layer_match = (port->template_layer == target_layer);
      if (!layer_match && !template_layer_match) {
        continue;
      }
      const double local_z = WorldPointToLocal(frame, port->world_position).z;
      if (std::isfinite(local_z)) {
        best_z = std::max(best_z, local_z);
      }
    }
    return std::isfinite(best_z) ? std::optional<double>{best_z} : std::nullopt;
  };
  auto lane_row_base_z_for_pole = [&](const Pole& pole) {
    if (const auto port_z = highest_owned_port_z_for_pole(pole); port_z.has_value()) {
      return *port_z;
    }
    double best_z = -std::numeric_limits<double>::infinity();
    const int target_layer = generation::detail::TemplateLayerForCategory(category);
    if (const PoleTypeDefinition* pole_type = find_pole_type(pole.pole_type_id); pole_type != nullptr) {
      for (const PortPlacementBand& band : pole_type->port_bands) {
        if (!band.enabled) {
          continue;
        }
        if (band.layer == target_layer) {
          best_z = std::max(best_z, band.height_max_m);
        }
      }
      if (!std::isfinite(best_z)) {
        for (const PortPlacementBand& band : pole_type->port_bands) {
          if (band.enabled && band.category == category) {
            best_z = std::max(best_z, band.height_max_m);
          }
        }
      }
    }
    if (std::isfinite(best_z)) {
      return best_z;
    }
    return std::max(0.5, pole.height_m * 0.8);
  };
  auto span_context_for_segment = [&](ObjectId node_a, ObjectId node_b) -> ConnectionContext {
    if (category == ConnectionCategory::kDrop) {
      return ConnectionContext::kDropAdd;
    }
    if (uses_branch_support) {
      return ConnectionContext::kBranchAdd;
    }
    const Pole* pole_a = support_pole(node_a);
    const Pole* pole_b = support_pole(node_b);
    const bool corner_pass = (pole_a != nullptr && pole_a->context.kind == PoleContextKind::kCorner) ||
                             (pole_b != nullptr && pole_b->context.kind == PoleContextKind::kCorner);
    return corner_pass ? ConnectionContext::kCornerPass : ConnectionContext::kTrunkContinue;
  };
  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };
  auto node_requires_branch_support = [&](ObjectId node_id, ObjectId peer_id) -> bool {
    if (!uses_branch_support) {
      return false;
    }
    const JunctionRelationKind relation_kind = incident_relation_kind_for(node_id, peer_id);
    const bool same_level_feasible = incident_same_level_feasible_for(node_id, peer_id);
    if (!same_level_feasible &&
        relation_kind == JunctionRelationKind::kCrossUnderpass) {
      return lowering_policy.enable_cross_underpass && lowering_policy.offset_m > 1e-6;
    }
    if (!same_level_feasible &&
        relation_kind == JunctionRelationKind::kSideBranch) {
      return lowering_policy.enable_branch_support && lowering_policy.offset_m > 1e-6;
    }
    if (relation_kind == JunctionRelationKind::kCrossUnderpass) {
      return false;
    }
    if (relation_kind == JunctionRelationKind::kSideBranch) {
      return false;
    }
    if (junction_relations_by_node != nullptr) {
      return false;
    }
    const auto it_ports = relation_index_access().ports_by_pole.find(node_id);
    if (it_ports == relation_index_access().ports_by_pole.end()) {
      return false;
    }
    for (ObjectId port_id : it_ports->second) {
      const Port* port = edit_state_access().ports.find(port_id);
      if (port == nullptr || port->owner_pole_id != node_id) {
        continue;
      }
      const auto it_spans = connection_index_access().spans_by_port.find(port_id);
      if (it_spans == connection_index_access().spans_by_port.end()) {
        continue;
      }
      for (ObjectId span_id : it_spans->second) {
        const Span* span = edit_state_access().spans.find(span_id);
        if (span == nullptr || span->bundle_id == kInvalidObjectId) {
          continue;
        }
        const Bundle* bundle = edit_state_access().bundles.find(span->bundle_id);
        if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
          continue;
        }
        const Port* port_a = edit_state_access().ports.find(span->port_a_id);
        const Port* port_b = edit_state_access().ports.find(span->port_b_id);
        if (port_a == nullptr || port_b == nullptr) {
          continue;
        }
        const ObjectId other_node_id =
            (span->port_a_id == port_id) ? resolve_span_endpoint_node(*span, port_b, false)
                                         : resolve_span_endpoint_node(*span, port_a, true);
        if (other_node_id != kInvalidObjectId && other_node_id != node_id && other_node_id != peer_id) {
          return true;
        }
      }
    }
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
        continue;
      }
      const ObjectId endpoint_a = span.endpoint_node_a_id;
      const ObjectId endpoint_b = span.endpoint_node_b_id;
      if (endpoint_a == node_id && endpoint_b != kInvalidObjectId && endpoint_b != node_id && endpoint_b != peer_id) {
        return true;
      }
      if (endpoint_b == node_id && endpoint_a != kInvalidObjectId && endpoint_a != node_id && endpoint_a != peer_id) {
        return true;
      }
    }
    return false;
  };
  std::vector<Vec3d> side_axis_by_index(node_ids.size(), Vec3d{0.0, 1.0, 0.0});
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    if (const Pole* pole = support_pole(node_ids[i]); pole != nullptr) {
      side_axis_by_index[i] = support_axis_for_pole(*pole);
      continue;
    }
    const Vec3d center = support_position(node_ids[i]);
    Vec3d tangent{1.0, 0.0, 0.0};
    if (i == 0 && i + 1 < node_ids.size()) {
      tangent = support_position(node_ids[i + 1]) - center;
    } else if (i + 1 == node_ids.size() && i > 0) {
      tangent = center - support_position(node_ids[i - 1]);
    } else if (i > 0 && i + 1 < node_ids.size()) {
      const Vec3d in_dir = center - support_position(node_ids[i - 1]);
      const Vec3d out_dir = support_position(node_ids[i + 1]) - center;
      tangent = in_dir + out_dir;
      if (!normalize_xy(&tangent)) {
        tangent = out_dir;
      }
    }
    if (!normalize_xy(&tangent)) {
      tangent = {1.0, 0.0, 0.0};
    }
    side_axis_by_index[i] = Vec3d{-tangent.y, tangent.x, 0.0};
  }
  for (std::size_t i = 1; i < side_axis_by_index.size(); ++i) {
    if (dot_xy(side_axis_by_index[i - 1], side_axis_by_index[i]) < 0.0) {
      side_axis_by_index[i].x = -side_axis_by_index[i].x;
      side_axis_by_index[i].y = -side_axis_by_index[i].y;
    }
  }
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    node_side_axis_hints[node_ids[i]] = side_axis_by_index[i];
  }
  std::unordered_map<ObjectId, std::size_t> node_index_by_id{};
  node_index_by_id.reserve(node_ids.size());
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    node_index_by_id[node_ids[i]] = i;
  }
  std::vector<bool> node_uses_down_offset(node_ids.size(), false);
  if (uses_acute_corner_lowering && has_relation_policy) {
    for (std::size_t i = 0; i < node_ids.size(); ++i) {
      const ObjectId prev_node_id = (i > 0) ? node_ids[i - 1] : kInvalidObjectId;
      const ObjectId next_node_id = (i + 1 < node_ids.size()) ? node_ids[i + 1] : kInvalidObjectId;
      const bool prev_infeasible =
          (prev_node_id != kInvalidObjectId) && !incident_same_level_feasible_for(node_ids[i], prev_node_id);
      const bool next_infeasible =
          (next_node_id != kInvalidObjectId) && !incident_same_level_feasible_for(node_ids[i], next_node_id);
      node_uses_down_offset[i] =
          (node_relation_kind_at(i) == JunctionRelationKind::kCornerContinuation) && (prev_infeasible || next_infeasible);
    }
  } else if (uses_acute_corner_lowering && node_ids.size() >= 3) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kAngleEps = 1e-6;
    for (std::size_t i = 1; i + 1 < node_ids.size(); ++i) {
      Vec3d in_dir = support_position(node_ids[i - 1]) - support_position(node_ids[i]);
      Vec3d out_dir = support_position(node_ids[i + 1]) - support_position(node_ids[i]);
      if (!normalize_xy(&in_dir) || !normalize_xy(&out_dir)) {
        continue;
      }
      const double cos_theta = std::clamp(dot_xy(in_dir, out_dir), -1.0, 1.0);
      const double turn_angle_deg = std::acos(cos_theta) * (180.0 / kPi);
      if (turn_angle_deg + kAngleEps < layout_settings_.corner_threshold_deg) {
        node_uses_down_offset[i] = true;
      }
    }
  }
  auto lane_row_target_z_for_pole = [&](const Pole& pole, std::size_t node_index) {
    double target_z_m = lane_row_base_z_for_pole(pole);
    if (uses_acute_corner_lowering && node_index < node_uses_down_offset.size() && node_uses_down_offset[node_index]) {
      target_z_m = std::max(0.5, target_z_m - effective_branch_down_offset_m);
    }
    return target_z_m;
  };
  auto lowering_kind_rank = [](BackboneLoweringKind kind) {
    switch (kind) {
    case BackboneLoweringKind::kCrossUnderpass:
      return 3;
    case BackboneLoweringKind::kBranchSupport:
      return 2;
    case BackboneLoweringKind::kAcuteCorner:
      return 1;
    case BackboneLoweringKind::kNone:
    default:
      return 0;
    }
  };
  auto lowering_kind_for_relation = [](JunctionRelationKind kind) {
    switch (kind) {
    case JunctionRelationKind::kCrossUnderpass:
      return BackboneLoweringKind::kCrossUnderpass;
    case JunctionRelationKind::kSideBranch:
      return BackboneLoweringKind::kBranchSupport;
    case JunctionRelationKind::kCornerContinuation:
      return BackboneLoweringKind::kAcuteCorner;
    case JunctionRelationKind::kThroughMain:
    case JunctionRelationKind::kNone:
    default:
      return BackboneLoweringKind::kNone;
    }
  };
  auto canonical_side_axis_for_order = [&](ObjectId node_id, ObjectId peer_id) -> Vec3d {
    if (const auto it = node_side_axis_hints.find(node_id);
        it != node_side_axis_hints.end() && std::isfinite(it->second.x) && std::isfinite(it->second.y)) {
      return it->second;
    }
    const Pole* pole = support_pole(node_id);
    if (pole != nullptr) {
      Vec3d dir_xy = support_position(peer_id) - pole->world_transform.position;
      if (normalize_xy(&dir_xy) && std::isfinite(dir_xy.x) && std::isfinite(dir_xy.y)) {
        return ComputeLateralAxis(dir_xy);
      }
      Vec3d yaw_side_axis = support_axis_for_pole(*pole);
      if (std::isfinite(yaw_side_axis.x) && std::isfinite(yaw_side_axis.y)) {
        return yaw_side_axis;
      }
    }
    return Vec3d{0.0, 1.0, 0.0};
  };
  struct EndpointSideDecision {
    Vec3d side_axis{0.0, 1.0, 0.0};
    bool has_side_axis = false;
    double chosen_side_sign = 0.0;
    SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
    SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
    bool used_junction_pair_side_assignment = false;
  };
  auto through_pair_side_axis_for_node = [&](ObjectId node_id) -> std::optional<Vec3d> {
    if (junction_relations_by_node == nullptr || node_id == kInvalidObjectId) {
      return std::nullopt;
    }
    const auto it = junction_relations_by_node->find(node_id);
    if (it == junction_relations_by_node->end() || !it->second.through_pair.accepted) {
      return std::nullopt;
    }
    const Vec3d a = support_position(it->second.through_pair.neighbor_a_id);
    const Vec3d b = support_position(it->second.through_pair.neighbor_b_id);
    Vec3d through_dir = b - a;
    through_dir.z = 0.0;
    if (!normalize_xy(&through_dir)) {
      return std::nullopt;
    }
    Vec3d side_axis = ComputeLateralAxis(through_dir);
    if (!normalize_xy(&side_axis)) {
      return std::nullopt;
    }
    return side_axis;
  };
  auto bisector_axis_for_endpoint = [&](ObjectId node_id, ObjectId peer_id) -> std::optional<Vec3d> {
    if (junction_relations_by_node == nullptr || node_id == kInvalidObjectId || peer_id == kInvalidObjectId) {
      return std::nullopt;
    }
    const auto it = junction_relations_by_node->find(node_id);
    if (it == junction_relations_by_node->end()) {
      return std::nullopt;
    }
    Vec3d peer_dir = support_position(peer_id) - support_position(node_id);
    peer_dir.z = 0.0;
    if (!normalize_xy(&peer_dir)) {
      return std::nullopt;
    }
    auto score_neighbor = [&](ObjectId neighbor_id) {
      Vec3d dir = support_position(neighbor_id) - support_position(node_id);
      dir.z = 0.0;
      if (!normalize_xy(&dir)) {
        return -std::numeric_limits<double>::infinity();
      }
      return dot_xy(peer_dir, dir);
    };
    ObjectId companion_id = kInvalidObjectId;
    double companion_score = -std::numeric_limits<double>::infinity();
    const JunctionRelation& relation = it->second;
    if (relation.through_pair.accepted) {
      const bool peer_in_pair =
          (peer_id == relation.through_pair.neighbor_a_id || peer_id == relation.through_pair.neighbor_b_id);
      if (peer_in_pair) {
        companion_id = (peer_id == relation.through_pair.neighbor_a_id) ? relation.through_pair.neighbor_b_id
                                                                         : relation.through_pair.neighbor_a_id;
      } else {
        for (ObjectId candidate_id : {relation.through_pair.neighbor_a_id, relation.through_pair.neighbor_b_id}) {
          const double score = score_neighbor(candidate_id);
          if (score > companion_score) {
            companion_score = score;
            companion_id = candidate_id;
          }
        }
      }
    }
    if (companion_id == kInvalidObjectId) {
      for (const JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.neighbor_node_id == peer_id || !incident.in_route) {
          continue;
        }
        const double score = score_neighbor(incident.neighbor_node_id);
        if (score > companion_score) {
          companion_score = score;
          companion_id = incident.neighbor_node_id;
        }
      }
    }
    if (companion_id == kInvalidObjectId) {
      for (const JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.neighbor_node_id == peer_id) {
          continue;
        }
        const double score = score_neighbor(incident.neighbor_node_id);
        if (score > companion_score) {
          companion_score = score;
          companion_id = incident.neighbor_node_id;
        }
      }
    }
    if (companion_id == kInvalidObjectId) {
      return std::nullopt;
    }
    Vec3d companion_dir = support_position(companion_id) - support_position(node_id);
    companion_dir.z = 0.0;
    if (!normalize_xy(&companion_dir)) {
      return std::nullopt;
    }
    Vec3d bisector = peer_dir + companion_dir;
    if (!normalize_xy(&bisector)) {
      return std::nullopt;
    }
    Vec3d canonical = canonical_side_axis_for_order(node_id, peer_id);
    if (dot_xy(bisector, canonical) < 0.0) {
      bisector = ScaleVec(bisector, -1.0);
    }
    return bisector;
  };
  auto chord_side_axis_for_endpoint = [&](ObjectId node_id, ObjectId peer_id) {
    Vec3d axis = support_position(peer_id) - support_position(node_id);
    axis.z = 0.0;
    if (!normalize_xy(&axis)) {
      return canonical_side_axis_for_order(node_id, peer_id);
    }
    axis = ComputeLateralAxis(axis);
    if (!normalize_xy(&axis)) {
      return canonical_side_axis_for_order(node_id, peer_id);
    }
    return axis;
  };
  auto connected_lowered_support_decision_for_node =
      [&](ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility,
          BackboneLoweringKind lowering_kind) -> std::optional<EndpointSideDecision> {
    if (junction_relations_by_node == nullptr || node_id == kInvalidObjectId ||
        feasibility.continuity_class != ContinuityCategoryClass::kBundleLike ||
        lowering_kind == BackboneLoweringKind::kNone) {
      return std::nullopt;
    }
    const auto it = junction_relations_by_node->find(node_id);
    if (it == junction_relations_by_node->end()) {
      return std::nullopt;
    }
    std::vector<Vec3d> lowered_dirs{};
    lowered_dirs.reserve(it->second.incidents.size());
    for (const JunctionIncidentRelation& incident : it->second.incidents) {
      if (!incident.in_route || incident.continuity_class != ContinuityCategoryClass::kBundleLike ||
          incident.kind == JunctionRelationKind::kThroughMain || incident.kind == JunctionRelationKind::kNone ||
          (!incident.default_lower_required && incident.same_level_feasible)) {
        continue;
      }
      Vec3d dir = support_position(incident.neighbor_node_id) - support_position(node_id);
      dir.z = 0.0;
      if (normalize_xy(&dir) && std::isfinite(dir.x) && std::isfinite(dir.y)) {
        lowered_dirs.push_back(dir);
      }
    }
    if (lowered_dirs.empty()) {
      return std::nullopt;
    }

      EndpointSideDecision decision{};
      decision.has_side_axis = true;
      decision.chosen_side_sign = 1.0;

      Vec3d axis{};
      if (lowered_dirs.size() >= 2) {
        if (feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
          if (const auto through_pair_axis = through_pair_side_axis_for_node(node_id); through_pair_axis.has_value()) {
            axis = *through_pair_axis;
            decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
            decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
            decision.used_junction_pair_side_assignment = true;
          }
        }
        if (!normalize_xy(&axis)) {
          Vec3d sum{0.0, 0.0, 0.0};
          for (const Vec3d& dir : lowered_dirs) {
            sum = sum + dir;
          }
          if (normalize_xy(&sum)) {
            axis = sum;
            decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
            decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
          } else {
            axis = lowered_dirs.front();
            if (!normalize_xy(&axis)) {
              axis = canonical_side_axis_for_order(node_id, peer_id);
            }
            if (feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
              decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
              decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
            } else {
              decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
              decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
            }
          }
          decision.used_junction_pair_side_assignment = true;
          const Vec3d canonical = canonical_side_axis_for_order(node_id, peer_id);
          if (dot_xy(axis, canonical) < 0.0 &&
              feasibility.kind != JunctionRelationKind::kCrossUnderpass) {
            axis = ScaleVec(axis, -1.0);
          }
        }
      } else {
      if (feasibility.kind == JunctionRelationKind::kSideBranch ||
          feasibility.kind == JunctionRelationKind::kCornerContinuation) {
        if (const auto bisector_axis = bisector_axis_for_endpoint(node_id, peer_id); bisector_axis.has_value()) {
          axis = *bisector_axis;
          decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
          decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
          decision.used_junction_pair_side_assignment = true;
        } else {
          axis = lowered_dirs.front();
          decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
          decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
        }
      } else if (feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
        if (const auto through_pair_axis = through_pair_side_axis_for_node(node_id); through_pair_axis.has_value()) {
          axis = *through_pair_axis;
          decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
          decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
          decision.used_junction_pair_side_assignment = true;
        } else {
          axis = lowered_dirs.front();
          decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
          decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
        }
      } else {
        axis = lowered_dirs.front();
        decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
        decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
      }
    }
    decision.side_axis = axis;
    return decision;
  };
  auto preferred_side_axis_for_endpoint =
      [&](ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility,
          BackboneLoweringKind lowering_kind) -> EndpointSideDecision {
    EndpointSideDecision decision{};
    decision.side_axis = canonical_side_axis_for_order(node_id, peer_id);
    decision.has_side_axis = std::isfinite(decision.side_axis.x) && std::isfinite(decision.side_axis.y);
    const bool prefers_line_oriented_lowering =
        feasibility.continuity_class == ContinuityCategoryClass::kBundleLike && lowering_kind != BackboneLoweringKind::kNone;
    if (!prefers_line_oriented_lowering) {
      return decision;
    }
    if (const auto connected_lowered =
            connected_lowered_support_decision_for_node(node_id, peer_id, feasibility, lowering_kind);
        connected_lowered.has_value()) {
      return *connected_lowered;
    }
    if (feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
      if (const auto through_pair_axis = through_pair_side_axis_for_node(node_id); through_pair_axis.has_value()) {
        decision.side_axis = *through_pair_axis;
        decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
        decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
        decision.used_junction_pair_side_assignment = true;
        decision.has_side_axis = true;
        return decision;
      }
    }
    if (feasibility.kind == JunctionRelationKind::kSideBranch ||
        feasibility.kind == JunctionRelationKind::kCornerContinuation) {
      if (const auto bisector_axis = bisector_axis_for_endpoint(node_id, peer_id); bisector_axis.has_value()) {
        decision.side_axis = *bisector_axis;
        decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
        decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
        decision.used_junction_pair_side_assignment = true;
        decision.has_side_axis = true;
        return decision;
      }
    }
    decision.side_axis = chord_side_axis_for_endpoint(node_id, peer_id);
    decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
    decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
    decision.has_side_axis = true;
    return decision;
  };
  auto support_orientation_axis_for_endpoint = [&](ObjectId node_id, ObjectId peer_id,
                                                   const EndpointSideDecision& decision) {
    if (decision.has_side_axis && decision.support_orientation_rule != SupportOrientationRuleKind::kRadial) {
      return decision.side_axis;
    }
    if (decision.support_orientation_rule == SupportOrientationRuleKind::kChord) {
      return chord_side_axis_for_endpoint(node_id, peer_id);
    }
    if (decision.has_side_axis) {
      return decision.side_axis;
    }
    return canonical_side_axis_for_order(node_id, peer_id);
  };
  auto finalize_side_sign_for_ports = [&](EndpointSideDecision* decision, ObjectId node_id, ObjectId peer_id,
                                          const std::vector<ObjectId>& port_ids) {
    if (decision == nullptr || !decision->has_side_axis) {
      return;
    }
    if (decision->side_assignment_rule != SideAssignmentRuleKind::kPoleLocal &&
        std::abs(decision->chosen_side_sign) > 1e-9) {
      return;
    }
    const Vec3d base_position = support_position(node_id);
    Vec3d peer_dir = support_position(peer_id) - support_position(node_id);
    peer_dir.z = 0.0;
    if ((decision->side_assignment_rule == SideAssignmentRuleKind::kThroughPairNormal ||
         decision->side_assignment_rule == SideAssignmentRuleKind::kBisector) &&
        normalize_xy(&peer_dir)) {
      const double along = dot_xy(peer_dir, decision->side_axis);
      if (std::abs(along) > 1e-9) {
        decision->chosen_side_sign = (along >= 0.0) ? 1.0 : -1.0;
        return;
      }
    }
    double sum = 0.0;
    int count = 0;
    for (ObjectId port_id : port_ids) {
      const Port* port = edit_state_access().ports.find(port_id);
      if (port == nullptr) {
        continue;
      }
      sum += dot_xy(port->world_position - base_position, decision->side_axis);
      ++count;
    }
    if (count > 0) {
      const double mean = sum / static_cast<double>(count);
      if (std::abs(mean) > 1e-9) {
        decision->chosen_side_sign = (mean >= 0.0) ? 1.0 : -1.0;
        return;
      }
    }
    const Vec3d chord_side_axis = chord_side_axis_for_endpoint(node_id, peer_id);
    const double chord_dot = dot_xy(chord_side_axis, decision->side_axis);
    decision->chosen_side_sign = (std::abs(chord_dot) <= 1e-9) ? 1.0 : ((chord_dot >= 0.0) ? 1.0 : -1.0);
  };
  auto support_group_id_for_endpoint = [&](ObjectId node_id, const SegmentRelationFeasibility& feasibility,
                                           const EndpointSideDecision& side_decision) {
    if ((feasibility.default_lower_required || !feasibility.same_level_feasible) &&
        std::abs(side_decision.chosen_side_sign) > 1e-9) {
      std::uint64_t seed = static_cast<std::uint64_t>(node_id);
      seed ^= static_cast<std::uint64_t>(feasibility.kind) << 8;
      seed ^= static_cast<std::uint64_t>(feasibility.continuity_class) << 16;
      seed ^= static_cast<std::uint64_t>(side_decision.side_assignment_rule) << 24;
      seed ^= static_cast<std::uint64_t>(side_decision.support_orientation_rule) << 32;
      seed ^= (side_decision.chosen_side_sign < 0.0 ? 1ull : 2ull) << 40;
      seed ^= static_cast<std::uint64_t>(side_decision.used_junction_pair_side_assignment ? 1ull : 0ull) << 48;
      return static_cast<int>(seed % 1000000000ull);
    }
    return -1;
  };
  auto build_endpoint_decision = [&](const SegmentRelationFeasibility& feasibility,
                                     const EndpointSideDecision& side_decision,
                                     ObjectId node_id,
                                     BundleOrderChoiceKind order_choice,
                                     BundleOrderChoiceReason order_reason, bool solver_used_same_level_constraint,
                                     bool used_special_case_ports, bool lowering_blocked_by_policy,
                                     bool unresolved_same_level_conflict) {
    EndpointContinuityDecision decision{};
    const bool endpoint_has_conflict =
        feasibility.default_lower_required || !feasibility.same_level_feasible;
    decision.relation_kind = feasibility.kind;
    decision.continuity_class = feasibility.continuity_class;
    decision.in_through_pair = feasibility.in_through_pair;
    decision.support_group_id =
        lowering_blocked_by_policy ? -1 : support_group_id_for_endpoint(node_id, feasibility, side_decision);
    decision.lower_required = feasibility.default_lower_required || !feasibility.same_level_feasible;
    decision.default_lower_required = feasibility.default_lower_required;
    decision.same_level_feasible = feasibility.same_level_feasible;
    decision.same_level_reason =
        (lowering_blocked_by_policy && endpoint_has_conflict)
            ? SameLevelFeasibilityReason::kCategoryPolicyDisabled
            : feasibility.reason;
    decision.projected_spacing_topview_m = feasibility.projected_spacing_topview_m;
    decision.required_clearance_m = feasibility.required_clearance_m;
    decision.lowering_blocked_by_policy = lowering_blocked_by_policy && endpoint_has_conflict;
    decision.unresolved_same_level_conflict =
        unresolved_same_level_conflict && endpoint_has_conflict;
    decision.solver_used_same_level_constraint = solver_used_same_level_constraint;
    decision.used_special_case_ports = used_special_case_ports;
    decision.bundle_order_policy = bundle_order_policy;
    decision.bundle_order_choice = order_choice;
    decision.bundle_order_choice_reason = order_reason;
    decision.side_assignment_rule = side_decision.side_assignment_rule;
    decision.support_orientation_rule = side_decision.support_orientation_rule;
    decision.support_orientation_basis =
        SupportOrientationBasisFromDecision(side_decision.support_orientation_rule, side_decision.chosen_side_sign);
    decision.chosen_side = LateralSideChoiceFromSign(side_decision.chosen_side_sign);
    decision.used_junction_pair_side_assignment = side_decision.used_junction_pair_side_assignment;
    decision.has_side_axis = side_decision.has_side_axis;
    decision.side_axis = side_decision.side_axis;
    decision.chosen_side_sign = side_decision.chosen_side_sign;
    return decision;
  };
  auto sync_assignment_from_decisions = [&](SegmentLaneAssignment* assignment) {
    if (assignment == nullptr) {
      return;
    }
    assignment->relation_a = assignment->decision_a.relation_kind;
    assignment->relation_b = assignment->decision_b.relation_kind;
    assignment->continuity_class =
        (assignment->decision_a.continuity_class == ContinuityCategoryClass::kBundleLike ||
         assignment->decision_b.continuity_class == ContinuityCategoryClass::kBundleLike)
            ? ContinuityCategoryClass::kBundleLike
            : ContinuityCategoryClass::kPointLike;
    assignment->default_lower_required =
        assignment->decision_a.default_lower_required || assignment->decision_b.default_lower_required;
    assignment->same_level_feasible =
        assignment->decision_a.same_level_feasible && assignment->decision_b.same_level_feasible;
    assignment->same_level_reason =
        assignment->same_level_feasible ? SameLevelFeasibilityReason::kNone
                                        : assignment->decision_a.same_level_feasible
                                              ? assignment->decision_b.same_level_reason
                                              : assignment->decision_a.same_level_reason;
    assignment->projected_spacing_topview_m =
        (!assignment->decision_a.same_level_feasible && !assignment->decision_b.same_level_feasible)
            ? std::min(assignment->decision_a.projected_spacing_topview_m,
                       assignment->decision_b.projected_spacing_topview_m)
            : (!assignment->decision_a.same_level_feasible ? assignment->decision_a.projected_spacing_topview_m
                                                           : assignment->decision_b.projected_spacing_topview_m);
    assignment->required_clearance_m =
        std::max(assignment->decision_a.required_clearance_m, assignment->decision_b.required_clearance_m);
    assignment->lowering_blocked_by_policy =
        assignment->decision_a.lowering_blocked_by_policy || assignment->decision_b.lowering_blocked_by_policy;
    assignment->unresolved_same_level_conflict =
        assignment->decision_a.unresolved_same_level_conflict || assignment->decision_b.unresolved_same_level_conflict;
    assignment->side_assignment_rule_a = assignment->decision_a.side_assignment_rule;
    assignment->side_assignment_rule_b = assignment->decision_b.side_assignment_rule;
    assignment->support_orientation_rule_a = assignment->decision_a.support_orientation_rule;
    assignment->support_orientation_rule_b = assignment->decision_b.support_orientation_rule;
    assignment->used_junction_pair_side_assignment_a = assignment->decision_a.used_junction_pair_side_assignment;
    assignment->used_junction_pair_side_assignment_b = assignment->decision_b.used_junction_pair_side_assignment;
    assignment->has_side_axis_a = assignment->decision_a.has_side_axis;
    assignment->has_side_axis_b = assignment->decision_b.has_side_axis;
    assignment->side_axis_a = assignment->decision_a.side_axis;
    assignment->side_axis_b = assignment->decision_b.side_axis;
    assignment->chosen_side_sign_a = assignment->decision_a.chosen_side_sign;
    assignment->chosen_side_sign_b = assignment->decision_b.chosen_side_sign;
  };
  auto ensure_ports = [&](ObjectId node_id, ObjectId peer_id, int segment_index, bool prefer_existing_neighbor_order,
                          bool* out_seeded_from_previous = nullptr) -> EditResult<std::vector<ObjectId>> {
    EditResult<std::vector<ObjectId>> ports_result;
    if (out_seeded_from_previous != nullptr) {
      *out_seeded_from_previous = false;
    }
    if (const auto it_cached = node_lane_ports_cache.find(node_id); it_cached != node_lane_ports_cache.end()) {
      if (static_cast<int>(it_cached->second.size()) == lane_count) {
        ports_result.value = it_cached->second;
        ports_result.ok = true;
        return ports_result;
      }
    }
    const SupportKind kind = support_kind(node_id);
    const Pole* pole = support_pole(node_id);
    if (kind != SupportKind::kPole || pole == nullptr) {
      const Vec3d base_position = support_position(node_id);
      const Vec3d side_axis = canonical_side_axis_for_order(node_id, peer_id);
      auto nonpole_order_key = [&](const Port* port) {
        if (port == nullptr) {
          return std::tuple<double, double, ObjectId>(0.0, 0.0, kInvalidObjectId);
        }
        return std::tuple<double, double, ObjectId>(dot_xy(port->world_position - base_position, side_axis),
                                                    port->world_position.z, port->id);
      };
      auto sort_nonpole_ports = [&](std::vector<ObjectId>* port_ids) {
        if (port_ids == nullptr) {
          return;
        }
        std::sort(port_ids->begin(), port_ids->end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return nonpole_order_key(pa) < nonpole_order_key(pb);
        });
      };
      std::unordered_set<ObjectId> unique_nonpole_ports{};
      for (const Span& span : edit_state_access().spans.items()) {
        if (span.bundle_id == kInvalidObjectId) {
          continue;
        }
        const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
        if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
          continue;
        }
        const Port* pa = edit_state_access().ports.find(span.port_a_id);
        const Port* pb = edit_state_access().ports.find(span.port_b_id);
        if (pa == nullptr || pb == nullptr) {
          continue;
        }
        if (resolve_span_endpoint_node(span, pa, true) == node_id && pa->layer == target_port_layer &&
            unique_nonpole_ports.insert(pa->id).second) {
          ports_result.value.push_back(pa->id);
        }
        if (resolve_span_endpoint_node(span, pb, false) == node_id && pb->layer == target_port_layer &&
            unique_nonpole_ports.insert(pb->id).second) {
          ports_result.value.push_back(pb->id);
        }
      }
      sort_nonpole_ports(&ports_result.value);
      if (static_cast<int>(ports_result.value.size()) > lane_count) {
        ports_result.value.resize(static_cast<std::size_t>(lane_count));
      }

      const auto it_support = support_node_by_id.find(node_id);
      if (static_cast<int>(ports_result.value.size()) < lane_count && it_support != support_node_by_id.end() &&
          it_support->second.has_source_edge) {
        struct SourcePortSample {
          Vec3d world{};
          double side = 0.0;
        };
        std::vector<SourcePortSample> samples{};
        const SupportNode& support = it_support->second;
        for (const Span& span : edit_state_access().spans.items()) {
          if (span.bundle_id == kInvalidObjectId) {
            continue;
          }
          const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
          if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
            continue;
          }
          const Port* pa = edit_state_access().ports.find(span.port_a_id);
          const Port* pb = edit_state_access().ports.find(span.port_b_id);
          if (pa == nullptr || pb == nullptr || pa->layer != target_port_layer || pb->layer != target_port_layer) {
            continue;
          }
          const ObjectId endpoint_a = resolve_span_endpoint_node(span, pa, true);
          const ObjectId endpoint_b = resolve_span_endpoint_node(span, pb, false);
          double t = support.source_edge_t;
          if (endpoint_a == support.source_edge_node_a_id && endpoint_b == support.source_edge_node_b_id) {
            // keep t
          } else if (endpoint_a == support.source_edge_node_b_id && endpoint_b == support.source_edge_node_a_id) {
            t = 1.0 - t;
          } else {
            continue;
          }
          const Vec3d world{
              pa->world_position.x + (pb->world_position.x - pa->world_position.x) * t,
              pa->world_position.y + (pb->world_position.y - pa->world_position.y) * t,
              pa->world_position.z + (pb->world_position.z - pa->world_position.z) * t,
          };
          samples.push_back({world, dot_xy(world - base_position, side_axis)});
        }
        std::sort(samples.begin(), samples.end(), [](const SourcePortSample& a, const SourcePortSample& b) {
          if (std::abs(a.side - b.side) > 1e-9) {
            return a.side < b.side;
          }
          if (std::abs(a.world.z - b.world.z) > 1e-9) {
            return a.world.z < b.world.z;
          }
          if (std::abs(a.world.y - b.world.y) > 1e-9) {
            return a.world.y < b.world.y;
          }
          return a.world.x < b.world.x;
        });
        for (const SourcePortSample& sample : samples) {
          if (static_cast<int>(ports_result.value.size()) >= lane_count) {
            break;
          }
          EditResult<ObjectId> add_port =
              AddPort(kInvalidObjectId, sample.world, category_to_port_kind(category), target_port_layer);
          if (!add_port.ok) {
            ports_result.error = add_port.error;
            return ports_result;
          }
          append_change_set(result.change_set, add_port.change_set);
          Port* created_port = edit_state_access().ports.find(add_port.value);
          if (created_port != nullptr) {
            created_port->generated_by_rule = true;
            created_port->placement_context = ConnectionContext::kBranchAdd;
            apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kAerialBranch);
            add_unique_id(result.change_set.updated_ids, created_port->id);
          }
          ports_result.value.push_back(add_port.value);
        }
      }

      ports_result.value.reserve(static_cast<std::size_t>(lane_count));
      while (static_cast<int>(ports_result.value.size()) < lane_count) {
        EditResult<ObjectId> add_port =
            AddPort(kInvalidObjectId, base_position, category_to_port_kind(category), target_port_layer);
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_change_set(result.change_set, add_port.change_set);
        Port* created_port = edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kBranchAdd;
          apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kAerialBranch);
          add_unique_id(result.change_set.updated_ids, created_port->id);
        }
        ports_result.value.push_back(add_port.value);
      }
      sort_nonpole_ports(&ports_result.value);
      node_lane_ports_cache[node_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
    const bool use_branch_support_here = node_requires_branch_support(node_id, peer_id);
    auto try_solver_ports_for_pole = [&](ConnectionContext solver_context, SlotRole preferred_role,
                                         const SegmentRelationFeasibility& feasibility,
                                         std::vector<ObjectId>* out_ports) -> bool {
      if (pole == nullptr || out_ports == nullptr || feasibility.same_level_feasible) {
        return false;
      }
      const Pole* peer_pole = support_pole(peer_id);
      const double spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      std::unordered_set<ObjectId> used_ports{};
      std::vector<ObjectId> solved_ports{};
      solved_ports.reserve(static_cast<std::size_t>(lane_count));
      for (int lane = 0; lane < lane_count; ++lane) {
        const double target_y = (static_cast<double>(lane) - center) * spacing;
        PortResolutionRequest request{};
        request.pole_id = node_id;
        request.peer_pole_id = (peer_pole == nullptr) ? kInvalidObjectId : peer_pole->id;
        request.category = category;
        request.connection_context = solver_context;
        request.pole_context = pole->context.kind;
        request.corner_angle_deg = pole->context.corner_angle_deg;
        request.corner_turn_sign = pole->context.corner_turn_sign;
        request.allow_generate_port = true;
        request.prefer_template_match = true;
        request.preferred_template_layer = generation::detail::TemplateLayerForCategory(category);
        request.preferred_template_side =
            (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
        request.preferred_template_role = preferred_role;
        request.branch_index = static_cast<std::uint32_t>(lane);
        request.endpoint_decision.relation_kind = feasibility.kind;
        request.endpoint_decision.continuity_class = feasibility.continuity_class;
        request.endpoint_decision.in_through_pair = feasibility.in_through_pair;
        request.endpoint_decision.lower_required =
            feasibility.default_lower_required || !feasibility.same_level_feasible;
        request.endpoint_decision.default_lower_required = feasibility.default_lower_required;
        request.endpoint_decision.same_level_feasible = false;
        request.endpoint_decision.same_level_reason = feasibility.reason;
        request.endpoint_decision.projected_spacing_topview_m = feasibility.projected_spacing_topview_m;
        request.endpoint_decision.required_clearance_m = feasibility.required_clearance_m;
        request.endpoint_decision.bundle_order_policy = bundle_order_policy;
        request.excluded_port_ids.assign(solved_ports.begin(), solved_ports.end());
        EditResult<ObjectId> port_result = ensure_pole_connection_port(request);
        if (!port_result.ok) {
          return false;
        }
        Port* selected_port = edit_state_access().ports.find(port_result.value);
        if (selected_port == nullptr || selected_port->owner_pole_id != node_id ||
            (selected_port->placement_source != PortPlacementSourceKind::kPlacementBand &&
             selected_port->placement_source != PortPlacementSourceKind::kPlacementBandConstrained) ||
            !used_ports.insert(selected_port->id).second) {
          return false;
        }
        solved_ports.push_back(selected_port->id);
      }
      std::sort(solved_ports.begin(), solved_ports.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = edit_state_access().ports.find(a);
        const Port* pb = edit_state_access().ports.find(b);
        const double ya = (pa == nullptr)
                              ? 0.0
                              : WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_for_pole(*pole)),
                                                  pa->world_position)
                                    .y;
        const double yb = (pb == nullptr)
                              ? 0.0
                              : WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_for_pole(*pole)),
                                                  pb->world_position)
                                    .y;
        if (std::abs(ya - yb) > 1e-9) {
          return ya < yb;
        }
        return a < b;
      });
      if (static_cast<int>(solved_ports.size()) == lane_count && !solved_ports.empty()) {
        const double layout_yaw = layout_yaw_for_pole(*pole);
        const PoleFrame frame = BuildPoleFrame(pole->world_transform, layout_yaw);
        double uniform_z = std::numeric_limits<double>::infinity();
        double existing_bundle_port_min_z_m = std::numeric_limits<double>::infinity();
        if (const auto it_ports = relation_index_access().ports_by_pole.find(node_id);
            it_ports != relation_index_access().ports_by_pole.end()) {
          for (ObjectId existing_port_id : it_ports->second) {
            const Port* existing_port = edit_state_access().ports.find(existing_port_id);
            if (existing_port == nullptr || existing_port->owner_pole_id != node_id || existing_port->category != category) {
              continue;
            }
            if (used_ports.find(existing_port_id) != used_ports.end()) {
              continue;
            }
            const auto it_spans = connection_index_access().spans_by_port.find(existing_port_id);
            if (it_spans == connection_index_access().spans_by_port.end() || it_spans->second.empty()) {
              continue;
            }
            existing_bundle_port_min_z_m = std::min(existing_bundle_port_min_z_m, existing_port->world_position.z);
          }
        }
        for (ObjectId port_id : solved_ports) {
          Port* selected_port = edit_state_access().ports.find(port_id);
          if (selected_port == nullptr) {
            continue;
          }
          const Vec3d local = WorldPointToLocal(frame, selected_port->world_position);
          uniform_z = std::min(uniform_z, local.z);
        }
        if (std::isfinite(uniform_z)) {
          double target_uniform_z = uniform_z;
          if (std::isfinite(existing_bundle_port_min_z_m)) {
            target_uniform_z = std::min(target_uniform_z, existing_bundle_port_min_z_m);
          }
          if (feasibility.kind == JunctionRelationKind::kSideBranch ||
              feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
            target_uniform_z = std::max(0.5, uniform_z - effective_branch_down_offset_m);
            if (std::isfinite(existing_bundle_port_min_z_m)) {
              target_uniform_z =
                  std::max(0.5, std::min(target_uniform_z, existing_bundle_port_min_z_m - effective_branch_down_offset_m));
            }
          }
          for (ObjectId port_id : solved_ports) {
            Port* selected_port = edit_state_access().ports.find(port_id);
            if (selected_port == nullptr) {
              continue;
            }
            Vec3d local = WorldPointToLocal(frame, selected_port->world_position);
            if (std::abs(local.z - target_uniform_z) <= 1e-9) {
              continue;
            }
            local.z = target_uniform_z;
            selected_port->world_position = local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
            add_unique_id(result.change_set.updated_ids, selected_port->id);
          }
        }
      }
      *out_ports = std::move(solved_ports);
      return static_cast<int>(out_ports->size()) == lane_count;
    };
    if (use_branch_support_here) {
      const SegmentRelationFeasibility branch_feasibility = segment_relation_feasibility_for(node_id, peer_id);
      if (branch_feasibility.kind == JunctionRelationKind::kCrossUnderpass &&
          try_solver_ports_for_pole(ConnectionContext::kBranchAdd, SlotRole::kBranchPreferred, branch_feasibility,
                                    &ports_result.value)) {
        node_lane_ports_cache[node_id] = ports_result.value;
        ports_result.ok = true;
        return ports_result;
      }
      const Vec3d peer_delta = support_position(peer_id) - pole->world_transform.position;
      Vec3d branch_forward_axis = peer_delta;
      if (!normalize_xy(&branch_forward_axis)) {
        branch_forward_axis = {1.0, 0.0, 0.0};
      }
      const Vec3d branch_row_axis = ComputeLateralAxis(branch_forward_axis);
      const EndpointSideDecision preferred_side_decision = preferred_side_axis_for_endpoint(
          node_id, peer_id, branch_feasibility,
          (branch_feasibility.kind == JunctionRelationKind::kCrossUnderpass)
              ? BackboneLoweringKind::kCrossUnderpass
              : BackboneLoweringKind::kBranchSupport);
      const Vec3d side_axis = preferred_side_decision.has_side_axis ? preferred_side_decision.side_axis
                                                                    : canonical_side_axis_for_order(node_id, peer_id);
      Vec3d peer_dir = peer_delta;
      peer_dir.z = 0.0;
      double side_sign = preferred_side_decision.chosen_side_sign;
      if (std::abs(side_sign) <= 1e-9 &&
          preferred_side_decision.side_assignment_rule == SideAssignmentRuleKind::kThroughPairNormal &&
          normalize_xy(&peer_dir)) {
        const double along = dot_xy(peer_dir, side_axis);
        side_sign = (std::abs(along) <= 1e-9) ? 1.0 : ((along >= 0.0) ? 1.0 : -1.0);
      } else if (std::abs(side_sign) <= 1e-9) {
        side_sign = (dot_xy(side_axis, branch_row_axis) >= 0.0) ? 1.0 : -1.0;
        if (std::abs(dot_xy(side_axis, branch_row_axis)) <= 1e-9) {
          side_sign = 1.0;
        }
      }
      const SlotSide branch_side = (side_sign >= 0.0) ? SlotSide::kRight : SlotSide::kLeft;
      const double branch_support_yaw_deg =
          std::atan2(branch_forward_axis.y, branch_forward_axis.x) * (180.0 / kPi);

      const bool use_scaffold_layout = use_lane_row_geometry;
      double main_support_base_z_m = lane_row_base_z_for_pole(*pole);
      double branch_base_z_m = main_support_base_z_m;
      double existing_bundle_port_min_z_m = std::numeric_limits<double>::infinity();
      if (const auto it_ports = relation_index_access().ports_by_pole.find(node_id);
          it_ports != relation_index_access().ports_by_pole.end()) {
        for (ObjectId existing_port_id : it_ports->second) {
          const Port* existing_port = edit_state_access().ports.find(existing_port_id);
          if (existing_port == nullptr || existing_port->owner_pole_id != node_id || existing_port->category != category) {
            continue;
          }
          const auto it_spans = connection_index_access().spans_by_port.find(existing_port_id);
          if (it_spans == connection_index_access().spans_by_port.end()) {
            continue;
          }
          existing_bundle_port_min_z_m = std::min(existing_bundle_port_min_z_m, existing_port->world_position.z);
        }
      }
      if (std::isfinite(existing_bundle_port_min_z_m)) {
        main_support_base_z_m = std::min(main_support_base_z_m, existing_bundle_port_min_z_m);
      }
      if (use_scaffold_layout) {
        main_support_base_z_m = std::min(main_support_base_z_m, pole->height_m * 0.8);
        branch_base_z_m = main_support_base_z_m;
      }
      branch_base_z_m = std::max(0.5, main_support_base_z_m - effective_branch_down_offset_m);

      const double lane_spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      const double half_span = center * lane_spacing;
        const double min_outboard =
            pole_radius_at_height_m(*pole, branch_base_z_m) + cache_state_access().geometry_settings.pole_clearance_m +
            half_span + 0.25;
        const double branch_support_offset_m = std::max(0.65, min_outboard);
        ports_result.value.reserve(static_cast<std::size_t>(lane_count));
        for (int lane = 0; lane < lane_count; ++lane) {
          const double lane_offset = (static_cast<double>(lane) - center) * lane_spacing;
          const double lane_z_m = branch_base_z_m;
          Vec3d local{0.0, side_sign * branch_support_offset_m + lane_offset, lane_z_m};
        local = apply_pole_clearance_to_local(*pole, local, branch_side);
        const Vec3d world =
            local_to_world_on_pole_local(pole->world_transform, branch_support_yaw_deg, local);
        EditResult<ObjectId> add_port =
            AddPort(node_id, world, category_to_port_kind(category), category_to_port_layer(category));
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_change_set(result.change_set, add_port.change_set);
        Port* created_port = edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->category = category;
          created_port->template_layer = generation::detail::TemplateLayerForCategory(category);
          created_port->template_side = branch_side;
          created_port->template_role = SlotRole::kBranchPreferred;
          created_port->generated_from_template = false;
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kBranchAdd;
          apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kBranchSupport);
          add_unique_id(result.change_set.updated_ids, created_port->id);
        }
        ports_result.value.push_back(add_port.value);
      }
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = edit_state_access().ports.find(a);
        const Port* pb = edit_state_access().ports.find(b);
        const double ya = (pa == nullptr)
                              ? 0.0
                              : WorldPointToLocal(BuildPoleFrame(pole->world_transform, branch_support_yaw_deg),
                                                  pa->world_position)
                                    .y;
        const double yb = (pb == nullptr)
                              ? 0.0
                              : WorldPointToLocal(BuildPoleFrame(pole->world_transform, branch_support_yaw_deg),
                                                  pb->world_position)
                                    .y;
        if (std::abs(ya - yb) > 1e-9) {
          return ya < yb;
        }
        return a < b;
      });
      node_lane_ports_cache[node_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
    auto port_connection_count = [&](ObjectId port_id) -> std::size_t {
      const auto it = connection_index_access().spans_by_port.find(port_id);
      return (it == connection_index_access().spans_by_port.end()) ? 0 : it->second.size();
    };

    if (use_lane_row_geometry && pole != nullptr) {
      const SegmentRelationFeasibility relation_feasibility = segment_relation_feasibility_for(node_id, peer_id);
      if (!relation_feasibility.same_level_feasible) {
        const ConnectionContext solver_context =
            (relation_feasibility.kind == JunctionRelationKind::kCornerContinuation)
                ? ConnectionContext::kCornerPass
                : ((relation_feasibility.kind == JunctionRelationKind::kSideBranch ||
                    relation_feasibility.kind == JunctionRelationKind::kCrossUnderpass)
                       ? ConnectionContext::kBranchAdd
                       : ConnectionContext::kTrunkContinue);
        const SlotRole preferred_role =
            (relation_feasibility.kind == JunctionRelationKind::kSideBranch ||
             relation_feasibility.kind == JunctionRelationKind::kCrossUnderpass)
                ? SlotRole::kBranchPreferred
                : SlotRole::kNeutral;
        if (try_solver_ports_for_pole(solver_context, preferred_role, relation_feasibility, &ports_result.value)) {
          node_lane_ports_cache[node_id] = ports_result.value;
          ports_result.ok = true;
          return ports_result;
        }
      }
      std::unordered_set<ObjectId> unique{};
      std::vector<ObjectId> reusable_generated_ports{};

      if (const auto it_ports = relation_index_access().ports_by_pole.find(node_id);
          it_ports != relation_index_access().ports_by_pole.end()) {
        for (ObjectId port_id : it_ports->second) {
          const Port* port = edit_state_access().ports.find(port_id);
          if (port == nullptr || port->layer != target_port_layer || port->category != category) {
            continue;
          }
          if (port->generated_by_rule && port_connection_count(port_id) == 0) {
            reusable_generated_ports.push_back(port_id);
          }
        }
      }

      const auto it_index = node_index_by_id.find(node_id);
      const std::size_t node_index = (it_index == node_index_by_id.end()) ? node_ids.size() : it_index->second;
      const double target_z_m = lane_row_target_z_for_pole(*pole, node_index);
      const double spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      for (int lane = 0; lane < lane_count; ++lane) {
        const double target_y = (static_cast<double>(lane) - center) * spacing;
        const Vec3d local{0.0, target_y, target_z_m};
        const Vec3d world = local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);

        ObjectId port_id = kInvalidObjectId;
        while (!reusable_generated_ports.empty() && port_id == kInvalidObjectId) {
          const ObjectId candidate_id = reusable_generated_ports.back();
          reusable_generated_ports.pop_back();
          if (unique.find(candidate_id) == unique.end()) {
            port_id = candidate_id;
          }
        }

        if (port_id != kInvalidObjectId) {
          Port* reused_port = edit_state_access().ports.find(port_id);
          if (reused_port == nullptr) {
            continue;
          }
          reused_port->world_position = world;
          reused_port->category = category;
          reused_port->generated_by_rule = true;
          reused_port->placement_context = ConnectionContext::kTrunkContinue;
          reused_port->template_layer = generation::detail::TemplateLayerForCategory(category);
          reused_port->template_side = (target_y < -1e-9) ? SlotSide::kLeft
                                                           : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
          apply_port_position_mode(*reused_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
          add_unique_id(result.change_set.updated_ids, reused_port->id);
          unique.insert(port_id);
          ports_result.value.push_back(port_id);
          continue;
        }

        EditResult<ObjectId> add_port =
            AddPort(node_id, world, category_to_port_kind(category), category_to_port_layer(category));
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_change_set(result.change_set, add_port.change_set);
        Port* created_port = edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->category = category;
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kTrunkContinue;
          created_port->template_layer = generation::detail::TemplateLayerForCategory(category);
          created_port->template_side = (target_y < -1e-9) ? SlotSide::kLeft
                                                           : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
          apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
          add_unique_id(result.change_set.updated_ids, created_port->id);
        }
        unique.insert(add_port.value);
        ports_result.value.push_back(add_port.value);
      }
    } else {
      for (const Port& port : edit_state_access().ports.items()) {
        if (port.owner_pole_id == node_id && port.layer == target_port_layer) {
          ports_result.value.push_back(port.id);
        }
      }
    }

    auto port_links_to_neighbor = [&](ObjectId port_id, ObjectId neighbor_node_id) -> int {
      int count = 0;
      const auto it = connection_index_access().spans_by_port.find(port_id);
      if (it == connection_index_access().spans_by_port.end()) {
        return 0;
      }
      for (ObjectId span_id : it->second) {
        const Span* span = edit_state_access().spans.find(span_id);
        if (span == nullptr) {
          continue;
        }
        const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id : span->port_a_id;
        const Port* other_port = edit_state_access().ports.find(other_port_id);
        if (other_port == nullptr || other_port->layer != target_port_layer) {
          continue;
        }
        if (other_port->owner_pole_id == neighbor_node_id) {
          ++count;
        }
      }
      return count;
    };

    auto side_rank = [](SlotSide side) -> int {
      switch (side) {
      case SlotSide::kLeft:
        return 0;
      case SlotSide::kCenter:
        return 1;
      case SlotSide::kRight:
        return 2;
      default:
        return 3;
      }
    };
    auto local_y_of = [&](const Port* p) -> double {
      if (p == nullptr || pole == nullptr) {
        return 0.0;
      }
      return WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_for_pole(*pole)), p->world_position).y;
    };
    auto order_key = [&](const Port* p) -> std::tuple<double, int, int, int, int, ObjectId> {
      if (p == nullptr) {
        return {0.0, 1, 999, 999999, 999999, kInvalidObjectId};
      }
      return {local_y_of(p), 0, p->template_layer, side_rank(p->template_side),
              static_cast<int>(std::llround(HeightAlongWorldUp(p->world_position) * 1000.0)), p->id};
    };

    ObjectId continuity_neighbor_id = kInvalidObjectId;
    if (prefer_existing_neighbor_order) {
      std::unordered_map<ObjectId, int> neighbor_counts{};
      for (ObjectId port_id : ports_result.value) {
        const auto it = connection_index_access().spans_by_port.find(port_id);
        if (it == connection_index_access().spans_by_port.end()) {
          continue;
        }
        for (ObjectId span_id : it->second) {
          const Span* span = edit_state_access().spans.find(span_id);
          if (span == nullptr) {
            continue;
          }
          const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id : span->port_a_id;
          const Port* other_port = edit_state_access().ports.find(other_port_id);
          if (other_port == nullptr || other_port->layer != target_port_layer) {
            continue;
          }
          const ObjectId other_node_id = other_port->owner_pole_id;
          if (other_node_id == kInvalidObjectId || other_node_id == node_id || other_node_id == peer_id) {
            continue;
          }
          neighbor_counts[other_node_id] += 1;
        }
      }
      int best_count = 0;
      for (const auto& [neighbor_id, count] : neighbor_counts) {
        if (count > best_count || (count == best_count && count > 0 && neighbor_id < continuity_neighbor_id)) {
          best_count = count;
          continuity_neighbor_id = neighbor_id;
        }
      }
    }
    if (static_cast<int>(ports_result.value.size()) < lane_count) {
      ports_result.error = "insufficient ports for grouped generation";
      return ports_result;
    }
    if (prefer_existing_neighbor_order && continuity_neighbor_id != kInvalidObjectId) {
      // Seed extension order from the current live boundary edge, not the previous derived cache.
      std::vector<ObjectId> seeded_order{};
      std::unordered_set<ObjectId> unique_seeded_ports{};
      seeded_order.reserve(static_cast<std::size_t>(lane_count));
      for (ObjectId port_id : ports_result.value) {
        const Port* port = edit_state_access().ports.find(port_id);
        if (port == nullptr || port->owner_pole_id != node_id || port->layer != target_port_layer ||
            port_links_to_neighbor(port_id, continuity_neighbor_id) <= 0) {
          continue;
        }
        if (!unique_seeded_ports.insert(port_id).second) {
          continue;
        }
        seeded_order.push_back(port_id);
      }
      if (static_cast<int>(seeded_order.size()) >= lane_count) {
        std::sort(seeded_order.begin(), seeded_order.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return order_key(pa) < order_key(pb);
        });
        seeded_order.resize(static_cast<std::size_t>(lane_count));
        ports_result.value = std::move(seeded_order);
        node_lane_ports_cache[node_id] = ports_result.value;
        if (out_seeded_from_previous != nullptr) {
          *out_seeded_from_previous = true;
        }
        ports_result.ok = true;
        return ports_result;
      }
    }

    const bool use_scaffold_layout = use_lane_row_geometry;
    if (use_scaffold_layout && pole != nullptr) {
      const double spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      std::vector<double> target_local_y(static_cast<std::size_t>(lane_count), 0.0);
      for (int lane = 0; lane < lane_count; ++lane) {
        target_local_y[static_cast<std::size_t>(lane)] = (static_cast<double>(lane) - center) * spacing;
      }

      const SegmentRelationFeasibility scaffold_feasibility = segment_relation_feasibility_for(node_id, peer_id);
      const BackboneLoweringKind scaffold_lowering_kind =
          (!scaffold_feasibility.same_level_feasible &&
           scaffold_feasibility.kind == JunctionRelationKind::kCrossUnderpass)
              ? BackboneLoweringKind::kCrossUnderpass
              : ((!scaffold_feasibility.same_level_feasible &&
                  scaffold_feasibility.kind == JunctionRelationKind::kCornerContinuation)
                     ? BackboneLoweringKind::kAcuteCorner
                     : ((!scaffold_feasibility.same_level_feasible &&
                         scaffold_feasibility.kind == JunctionRelationKind::kSideBranch)
                            ? BackboneLoweringKind::kBranchSupport
                            : BackboneLoweringKind::kNone));
      const EndpointSideDecision scaffold_side_decision =
          preferred_side_axis_for_endpoint(node_id, peer_id, scaffold_feasibility, scaffold_lowering_kind);
      const Vec3d stable_side_axis =
          scaffold_side_decision.has_side_axis ? scaffold_side_decision.side_axis
                                               : canonical_side_axis_for_order(node_id, peer_id);
      const auto it_index = node_index_by_id.find(node_id);
      const std::size_t node_index = (it_index == node_index_by_id.end()) ? node_ids.size() : it_index->second;
      const bool is_terminal_node = (node_index == 0 || node_index + 1 >= node_ids.size());
      if (is_terminal_node) {
        auto port_connection_count = [&](ObjectId port_id) -> std::size_t {
          const auto it = connection_index_access().spans_by_port.find(port_id);
          return (it == connection_index_access().spans_by_port.end()) ? 0 : it->second.size();
        };
        auto terminal_order_key = [&](const Port* p) -> std::tuple<int, int, double, ObjectId> {
          if (p == nullptr) {
            return {999, 999, 0.0, kInvalidObjectId};
          }
          return {p->template_layer, side_rank(p->template_side), local_y_of(p), p->id};
        };
        std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return terminal_order_key(pa) < terminal_order_key(pb);
        });

        std::vector<ObjectId> normalized_ports{};
        normalized_ports.reserve(static_cast<std::size_t>(lane_count));
        std::vector<ObjectId> reusable_fallback_ports{};
        reusable_fallback_ports.reserve(ports_result.value.size());
        for (ObjectId port_id : ports_result.value) {
          const Port* port = edit_state_access().ports.find(port_id);
          if (port == nullptr) {
            continue;
          }
          if (port_connection_count(port_id) == 0) {
            reusable_fallback_ports.push_back(port_id);
          }
        }

        const double target_z_m = lane_row_target_z_for_pole(*pole, node_index);

        auto realize_terminal_world = [&](double target_y) {
          const Vec3d local{0.0, target_y, target_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        };

        for (int lane = static_cast<int>(normalized_ports.size()); lane < lane_count; ++lane) {
          const double target_y = target_local_y[static_cast<std::size_t>(lane)];
          const Vec3d world = realize_terminal_world(target_y);
          if (!reusable_fallback_ports.empty()) {
            const ObjectId port_id = reusable_fallback_ports.front();
            reusable_fallback_ports.erase(reusable_fallback_ports.begin());
            Port* port = edit_state_access().ports.find(port_id);
            if (port != nullptr) {
              port->world_position = world;
              port->category = category;
              port->generated_by_rule = true;
              port->template_layer = generation::detail::TemplateLayerForCategory(category);
              port->template_side =
                  (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
              apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
              add_unique_id(result.change_set.updated_ids, port->id);
              normalized_ports.push_back(port_id);
              continue;
            }
          }

          EditResult<ObjectId> add_port =
              AddPort(node_id, world, category_to_port_kind(category), category_to_port_layer(category));
          if (!add_port.ok) {
            ports_result.error = add_port.error;
            return ports_result;
          }
          append_change_set(result.change_set, add_port.change_set);
          Port* created_port = edit_state_access().ports.find(add_port.value);
          if (created_port != nullptr) {
            created_port->category = category;
            created_port->generated_by_rule = true;
            created_port->placement_context = ConnectionContext::kTrunkContinue;
            created_port->template_layer = generation::detail::TemplateLayerForCategory(category);
            created_port->template_side =
                (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
            apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
            add_unique_id(result.change_set.updated_ids, created_port->id);
          }
          normalized_ports.push_back(add_port.value);
        }

        std::sort(normalized_ports.begin(), normalized_ports.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return local_y_of(pa) < local_y_of(pb);
        });
        if (static_cast<int>(normalized_ports.size()) > lane_count) {
          normalized_ports.resize(static_cast<std::size_t>(lane_count));
        }

        ports_result.value = std::move(normalized_ports);
        node_lane_ports_cache[node_id] = ports_result.value;
        ports_result.ok = true;
        return ports_result;
      }

      const bool use_scaffold_geometry = use_lane_row_geometry;
      auto desired_world_for_target = [&](double target_y) {
        const Vec3d base = pole->world_transform.position;
        const double base_z_m = lane_row_target_z_for_pole(*pole, node_index);
        if (!use_scaffold_geometry || node_index == node_ids.size()) {
          const Vec3d local{0.0, target_y, base_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        }

        const Vec3d prev = support_position(node_ids[node_index - 1]);
        const Vec3d next = support_position(node_ids[node_index + 1]);
        if ((prev - base).x * (prev - base).x + (prev - base).y * (prev - base).y <= 1e-12 ||
            (next - base).x * (next - base).x + (next - base).y * (next - base).y <= 1e-12) {
          const Vec3d local{0.0, target_y, base_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        }

        Vec3d dir_in = base - prev;
        Vec3d dir_out = next - base;
        if (!normalize_xy(&dir_in) || !normalize_xy(&dir_out)) {
          const Vec3d local{0.0, target_y, base_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        }
        Vec3d normal_in = ComputeLateralAxis(dir_in);
        Vec3d normal_out = ComputeLateralAxis(dir_out);
        if (!normalize_xy(&normal_in)) {
          normal_in = stable_side_axis;
        }
        if (!normalize_xy(&normal_out)) {
          normal_out = stable_side_axis;
        }
        if (dot_xy(normal_in, stable_side_axis) < 0.0) {
          normal_in.x = -normal_in.x;
          normal_in.y = -normal_in.y;
        }
        if (dot_xy(normal_out, stable_side_axis) < 0.0) {
          normal_out.x = -normal_out.x;
          normal_out.y = -normal_out.y;
        }

        Vec3d joined_xy{};
        const Vec3d offset_in{base.x + normal_in.x * target_y, base.y + normal_in.y * target_y,
                              HeightAlongWorldUp(base)};
        const Vec3d offset_out{base.x + normal_out.x * target_y, base.y + normal_out.y * target_y,
                               HeightAlongWorldUp(base)};
        if (line_intersection_xy_local(offset_in, dir_in, offset_out, dir_out, &joined_xy)) {
          SetHeightAlongWorldUp(&joined_xy, HeightAlongWorldUp(base) + base_z_m);
          const double dx = joined_xy.x - base.x;
          const double dy = joined_xy.y - base.y;
          const double dist = std::sqrt(dx * dx + dy * dy);
          const double limit = std::max(spacing * 8.0, std::abs(target_y) * 8.0 + 0.2);
          if (dist <= limit || dist <= 1e-9) {
            return joined_xy;
          }
          const double scale = limit / dist;
          return Vec3d{base.x + dx * scale, base.y + dy * scale, HeightAlongWorldUp(base) + base_z_m};
        }

        const Vec3d local{0.0, target_y, base_z_m};
        return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
      };

      std::vector<ObjectId> ordered_ports(static_cast<std::size_t>(lane_count), kInvalidObjectId);
      std::unordered_set<ObjectId> used{};
      for (int lane = 0; lane < lane_count; ++lane) {
        const double target_y = target_local_y[static_cast<std::size_t>(lane)];
        const Vec3d desired_world = desired_world_for_target(target_y);
        ObjectId best_id = kInvalidObjectId;
        double best_dist = std::numeric_limits<double>::max();
        for (ObjectId candidate_id : ports_result.value) {
          if (used.find(candidate_id) != used.end()) {
            continue;
          }
          const Port* candidate = edit_state_access().ports.find(candidate_id);
          if (candidate == nullptr || candidate->owner_pole_id != node_id || candidate->layer != target_port_layer) {
            continue;
          }
          const double dist = use_scaffold_geometry
                                  ? std::sqrt(std::pow(candidate->world_position.x - desired_world.x, 2.0) +
                                              std::pow(candidate->world_position.y - desired_world.y, 2.0))
                                  : std::abs(dot_xy(candidate->world_position - pole->world_transform.position,
                                                     stable_side_axis) -
                                             target_y);
          if (dist < best_dist) {
            best_dist = dist;
            best_id = candidate_id;
          }
        }

        const double kTargetMatchTolerance =
            use_scaffold_geometry ? std::max(0.05, spacing * 0.35) : std::max(0.02, spacing * 0.1);
        if (best_id == kInvalidObjectId || best_dist > kTargetMatchTolerance) {
          EditResult<ObjectId> extra =
              AddPort(node_id, desired_world, category_to_port_kind(category), category_to_port_layer(category));
          if (!extra.ok) {
            ports_result.error = extra.error;
            return ports_result;
          }
          append_change_set(result.change_set, extra.change_set);
          best_id = extra.value;
        }
        if (use_scaffold_geometry) {
          Port* selected = edit_state_access().ports.find(best_id);
          if (selected != nullptr && selected->owner_pole_id == node_id) {
            selected->world_position = desired_world;
            add_unique_id(result.change_set.updated_ids, selected->id);
          }
        }

        used.insert(best_id);
        ordered_ports[static_cast<std::size_t>(lane)] = best_id;
      }

      ports_result.value = std::move(ordered_ports);
      node_lane_ports_cache[node_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
    std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
      const Port* pa = edit_state_access().ports.find(a);
      const Port* pb = edit_state_access().ports.find(b);
      return order_key(pa) < order_key(pb);
    });

    if (static_cast<int>(ports_result.value.size()) > lane_count) {
      if (continuity_neighbor_id != kInvalidObjectId) {
        std::stable_sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
          const int score_a = port_links_to_neighbor(a, continuity_neighbor_id);
          const int score_b = port_links_to_neighbor(b, continuity_neighbor_id);
          if (score_a != score_b) {
            return score_a > score_b;
          }
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return order_key(pa) < order_key(pb);
        });
        ports_result.value.resize(static_cast<std::size_t>(lane_count));
        std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return order_key(pa) < order_key(pb);
        });
      } else {
        const std::size_t target = static_cast<std::size_t>(lane_count);
        const std::size_t total = ports_result.value.size();
        std::size_t best_start = 0;
        double best_abs_mean = std::numeric_limits<double>::max();
        double best_span = std::numeric_limits<double>::max();
        for (std::size_t start = 0; start + target <= total; ++start) {
          double sum = 0.0;
          double y_min = std::numeric_limits<double>::max();
          double y_max = -std::numeric_limits<double>::max();
          for (std::size_t i = start; i < start + target; ++i) {
            const Port* p = edit_state_access().ports.find(ports_result.value[i]);
            const double y = local_y_of(p);
            sum += y;
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
          }
          const double abs_mean = std::abs(sum / static_cast<double>(target));
          const double span = y_max - y_min;
          if (abs_mean + 1e-9 < best_abs_mean || (std::abs(abs_mean - best_abs_mean) <= 1e-9 && span < best_span)) {
            best_abs_mean = abs_mean;
            best_span = span;
            best_start = start;
          }
        }
        std::vector<ObjectId> centered{};
        centered.reserve(target);
        for (std::size_t i = best_start; i < best_start + target; ++i) {
          centered.push_back(ports_result.value[i]);
        }
        ports_result.value.swap(centered);
      }
    }
    if (static_cast<int>(ports_result.value.size()) > lane_count) {
      ports_result.value.resize(static_cast<std::size_t>(lane_count));
    }
    node_lane_ports_cache[node_id] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  };

  bool first_seeded_from_previous = false;
  EditResult<std::vector<ObjectId>> first_ports =
      ensure_ports(node_ids.front(), node_ids[1], 0, true, &first_seeded_from_previous);
  if (!first_ports.ok) {
    result.error = first_ports.error;
    return result;
  }
  if (static_cast<int>(first_ports.value.size()) != lane_count) {
    result.error = "failed to seed first segment lanes";
    return result;
  }
  std::vector<std::vector<ObjectId>> base_ports_by_node(node_ids.size());
  base_ports_by_node.front() = first_ports.value;

  struct BundleOrderScore {
    int cross_y = 0;
    int cross_z = 0;
    int layer_jump = 0;
    double span_z_delta = 0.0;
  };
  auto secondary_score_less = [&](const BundleOrderScore& a, const BundleOrderScore& b) {
    const auto key_a =
        std::make_tuple(a.cross_z, a.layer_jump, static_cast<long long>(std::llround(a.span_z_delta * 1000.0)));
    const auto key_b =
        std::make_tuple(b.cross_z, b.layer_jump, static_cast<long long>(std::llround(b.span_z_delta * 1000.0)));
    return key_a < key_b;
  };
  auto evaluate_increment = [&](ObjectId node_a, ObjectId node_b, const std::vector<ObjectId>& lanes_a,
                                const std::vector<ObjectId>& lanes_b) -> BundleOrderScore {
    BundleOrderScore score{};
    const Pole* pa = support_pole(node_a);
    const Pole* pb = support_pole(node_b);
    const Vec3d pos_a = support_position(node_a);
    const Vec3d pos_b = support_position(node_b);

    Vec3d segment_dir{1.0, 0.0, 0.0};
    if (((pos_b.x - pos_a.x) * (pos_b.x - pos_a.x) + (pos_b.y - pos_a.y) * (pos_b.y - pos_a.y) +
         (pos_b.z - pos_a.z) * (pos_b.z - pos_a.z)) > 1e-12) {
      segment_dir = pos_b - pos_a;
      if (!normalize_xy(&segment_dir) || !std::isfinite(segment_dir.x) || !std::isfinite(segment_dir.y)) {
        segment_dir = {1.0, 0.0, 0.0};
      }
    }
    const Vec3d lateral_axis{-segment_dir.y, segment_dir.x, 0.0};
    auto axis_for_node = [&](ObjectId node_id) -> Vec3d {
      if (const Pole* pole = support_pole(node_id); pole != nullptr) {
        Vec3d support_axis = support_axis_for_pole(*pole);
        if (std::isfinite(support_axis.x) && std::isfinite(support_axis.y)) {
          return support_axis;
        }
      }
      const auto it = node_side_axis_hints.find(node_id);
      if (it != node_side_axis_hints.end() && std::isfinite(it->second.x) && std::isfinite(it->second.y)) {
        return it->second;
      }
      return lateral_axis;
    };
    const Vec3d axis_a = axis_for_node(node_a);
    const Vec3d axis_b = axis_for_node(node_b);
    double y_sign_b = 1.0;
    if (dot_xy(axis_a, axis_b) < 0.0) {
      y_sign_b = -1.0;
    }
    const bool use_local_y_metric = use_lane_row_geometry;

    std::vector<double> y_a(static_cast<std::size_t>(lane_count), 0.0);
    std::vector<double> y_b(static_cast<std::size_t>(lane_count), 0.0);
    std::vector<double> z_a(static_cast<std::size_t>(lane_count), 0.0);
    std::vector<double> z_b(static_cast<std::size_t>(lane_count), 0.0);

    for (int lane = 0; lane < lane_count; ++lane) {
      const std::size_t idx = static_cast<std::size_t>(lane);
      const Port* port_a = edit_state_access().ports.find(lanes_a[idx]);
      const Port* port_b = edit_state_access().ports.find(lanes_b[idx]);
      if (port_a == nullptr || port_b == nullptr) {
        score.layer_jump += 4;
        score.span_z_delta += 5.0;
        continue;
      }
      if (use_local_y_metric) {
        const Vec3d local_a =
            (pa == nullptr)
                ? port_a->world_position
                : WorldPointToLocal(BuildPoleFrame(pa->world_transform, layout_yaw_for_pole(*pa)),
                                    port_a->world_position);
        const Vec3d local_b =
            (pb == nullptr)
                ? port_b->world_position
                : WorldPointToLocal(BuildPoleFrame(pb->world_transform, layout_yaw_for_pole(*pb)),
                                    port_b->world_position);
        y_a[idx] = local_a.y;
        y_b[idx] = local_b.y * y_sign_b;
      } else {
        const Vec3d da = port_a->world_position - pos_a;
        const Vec3d db = port_b->world_position - pos_b;
        y_a[idx] = dot_xy(da, axis_a);
        y_b[idx] = dot_xy(db, axis_b) * y_sign_b;
      }
      z_a[idx] = port_a->world_position.z;
      z_b[idx] = port_b->world_position.z;
      score.layer_jump += std::abs(port_a->template_layer - port_b->template_layer);
      score.span_z_delta += std::abs(port_a->world_position.z - port_b->world_position.z);
    }

    for (int i = 0; i < lane_count; ++i) {
      for (int j = i + 1; j < lane_count; ++j) {
        const std::size_t ii = static_cast<std::size_t>(i);
        const std::size_t jj = static_cast<std::size_t>(j);
        const double dy_a = y_a[ii] - y_a[jj];
        const double dy_b = y_b[ii] - y_b[jj];
        constexpr double kOrderEps = 1e-4;
        if ((dy_a > kOrderEps && dy_b < -kOrderEps) || (dy_a < -kOrderEps && dy_b > kOrderEps)) {
          ++score.cross_y;
        }
        const double dz_a = z_a[ii] - z_a[jj];
        const double dz_b = z_b[ii] - z_b[jj];
        if ((dz_a > kOrderEps && dz_b < -kOrderEps) || (dz_a < -kOrderEps && dz_b > kOrderEps)) {
          ++score.cross_z;
        }
      }
    }
    return score;
  };
  auto count_adjacent_segment_xy_intersections = [&](const std::vector<ObjectId>& prev_a,
                                                     const std::vector<ObjectId>& prev_b,
                                                     const std::vector<ObjectId>& curr_a,
                                                     const std::vector<ObjectId>& curr_b) {
    int intersections = 0;
    for (int i = 0; i < lane_count; ++i) {
      const std::size_t ii = static_cast<std::size_t>(i);
      const Port* c0 = edit_state_access().ports.find(curr_a[ii]);
      const Port* c1 = edit_state_access().ports.find(curr_b[ii]);
      if (c0 == nullptr || c1 == nullptr) {
        continue;
      }
      for (int j = 0; j < lane_count; ++j) {
        if (i == j) {
          continue;
        }
        const std::size_t jj = static_cast<std::size_t>(j);
        const Port* p0 = edit_state_access().ports.find(prev_a[jj]);
        const Port* p1 = edit_state_access().ports.find(prev_b[jj]);
        if (p0 == nullptr || p1 == nullptr) {
          continue;
        }
        if (segments_intersect_xy_strict_local(c0->world_position, c1->world_position, p0->world_position,
                                               p1->world_position)) {
          ++intersections;
        }
      }
    }
    return intersections;
  };

  const bool bundle_order_permutable =
      bundle_order_policy == BundleOrderPolicyKind::kPermutableHomogeneous && lane_count > 1;
  const bool allow_bundle_order_reverse = bundle_order_permutable;
  constexpr double kAngleEps = 1e-6;
  constexpr double kReverseStraightAngleDeg = 179.999;

  const std::size_t segment_count = node_ids.size() - 1;
  std::vector<std::vector<ObjectId>> prepared_ports_b(segment_count);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId node_a = node_ids[seg];
    const ObjectId node_b = node_ids[seg + 1];
    const ObjectId right_order_peer = node_a;
    EditResult<std::vector<ObjectId>> right_ports =
        ensure_ports(node_b, right_order_peer, static_cast<int>(seg), false, nullptr);
    if (!right_ports.ok) {
      result.error = right_ports.error;
      return result;
    }
    if (static_cast<int>(right_ports.value.size()) != lane_count) {
      result.error = "failed to prepare right-side lane candidates";
      return result;
    }
    prepared_ports_b[seg] = std::move(right_ports.value);
    base_ports_by_node[seg + 1] = prepared_ports_b[seg];
  }

  auto order_for_choice = [&](const std::vector<ObjectId>& base_ports, BundleOrderChoiceKind choice) {
    std::vector<ObjectId> ordered = base_ports;
    if (choice == BundleOrderChoiceKind::kReversed) {
      std::reverse(ordered.begin(), ordered.end());
    }
    return ordered;
  };
  auto choice_for_parity = [](int parity) {
    return (parity != 0) ? BundleOrderChoiceKind::kReversed : BundleOrderChoiceKind::kNormal;
  };
  auto compute_turn_angle_deg = [&](std::size_t segment_index) -> double {
    if (segment_index == 0 || segment_index + 1 >= node_ids.size()) {
      return 180.0;
    }
    const Vec3d prev = support_position(node_ids[segment_index - 1]);
    const Vec3d curr = support_position(node_ids[segment_index]);
    const Vec3d next = support_position(node_ids[segment_index + 1]);
    const Vec3d in_check = prev - curr;
    const Vec3d out_check = next - curr;
    if ((in_check.x * in_check.x + in_check.y * in_check.y + in_check.z * in_check.z) <= 1e-12 ||
        (out_check.x * out_check.x + out_check.y * out_check.y + out_check.z * out_check.z) <= 1e-12) {
      return 180.0;
    }
    // Use vertex interior angle: straight-through == 180deg, acute corner < threshold.
    Vec3d in_dir = prev - curr;
    Vec3d out_dir = next - curr;
    if (!normalize_xy(&in_dir) || !normalize_xy(&out_dir)) {
      return 180.0;
    }
    const double d = std::clamp(dot_xy(in_dir, out_dir), -1.0, 1.0);
    const double angle = std::acos(d) * (180.0 / kPi);
    if (!std::isfinite(angle)) {
      return 180.0;
    }
    return angle;
  };

  struct OrientationPlanScore {
    BundleOrderScore order{};
    int adjacent_xy_intersections = 0;
    int orientation_flips = 0;
    int acute_orientation_flips = 0;
  };
  auto add_order_score = [](BundleOrderScore* dst, const BundleOrderScore& src) {
    if (dst == nullptr) {
      return;
    }
    dst->cross_y += src.cross_y;
    dst->cross_z += src.cross_z;
    dst->layer_jump += src.layer_jump;
    dst->span_z_delta += src.span_z_delta;
  };
  auto orientation_plan_less = [&](const OrientationPlanScore& a, const OrientationPlanScore& b) {
    if (use_lane_row_geometry && a.adjacent_xy_intersections != b.adjacent_xy_intersections) {
      return a.adjacent_xy_intersections < b.adjacent_xy_intersections;
    }
    if (a.order.cross_y != b.order.cross_y) {
      return a.order.cross_y < b.order.cross_y;
    }
    if (a.acute_orientation_flips != b.acute_orientation_flips) {
      return a.acute_orientation_flips < b.acute_orientation_flips;
    }
    if (a.orientation_flips != b.orientation_flips) {
      return a.orientation_flips < b.orientation_flips;
    }
    return secondary_score_less(a.order, b.order);
  };

  std::vector<double> turn_angle_by_segment(segment_count, 180.0);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    turn_angle_by_segment[seg] = compute_turn_angle_deg(seg);
  }

  std::vector<int> node_parity(node_ids.size(), 0);
  if (segment_count == 1) {
    const std::vector<int> first_candidates =
        (first_seeded_from_previous || !allow_bundle_order_reverse) ? std::vector<int>{0} : std::vector<int>{0, 1};
    OrientationPlanScore best_score{};
    bool has_best = false;
    for (int parity_a : first_candidates) {
      const std::vector<ObjectId> ports_a = order_for_choice(base_ports_by_node[0], choice_for_parity(parity_a));
      for (int parity_b : {0, 1}) {
        if (!allow_bundle_order_reverse && parity_b != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_b = order_for_choice(base_ports_by_node[1], choice_for_parity(parity_b));
        OrientationPlanScore candidate{};
        candidate.order = evaluate_increment(node_ids[0], node_ids[1], ports_a, ports_b);
        if (!has_best || orientation_plan_less(candidate, best_score)) {
          has_best = true;
          best_score = candidate;
          node_parity[0] = parity_a;
          node_parity[1] = parity_b;
        }
      }
    }
  } else {
    struct DpCell {
      bool reachable = false;
      OrientationPlanScore score{};
      int prev_prev_parity = -1;
    };
    std::vector<std::array<std::array<DpCell, 2>, 2>> dp(segment_count + 1);
    const std::vector<int> first_candidates =
        (first_seeded_from_previous || !allow_bundle_order_reverse) ? std::vector<int>{0} : std::vector<int>{0, 1};

    for (int parity_0 : first_candidates) {
      const std::vector<ObjectId> ports_0 = order_for_choice(base_ports_by_node[0], choice_for_parity(parity_0));
      for (int parity_1 : {0, 1}) {
        if (!allow_bundle_order_reverse && parity_1 != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_1 = order_for_choice(base_ports_by_node[1], choice_for_parity(parity_1));
        DpCell& cell = dp[1][parity_0][parity_1];
        cell.reachable = true;
        cell.prev_prev_parity = -1;
        cell.score.order = evaluate_increment(node_ids[0], node_ids[1], ports_0, ports_1);
      }
    }

    for (std::size_t step = 1; step < segment_count; ++step) {
      for (int parity_prev_prev : {0, 1}) {
        for (int parity_prev : {0, 1}) {
          const DpCell& cell = dp[step][parity_prev_prev][parity_prev];
          if (!cell.reachable) {
            continue;
          }
          const int prev_orientation = parity_prev_prev ^ parity_prev;
          for (int parity_curr : {0, 1}) {
            if (!allow_bundle_order_reverse && parity_curr != 0) {
              continue;
            }
            const std::vector<ObjectId> ports_prev =
                order_for_choice(base_ports_by_node[step], choice_for_parity(parity_prev));
            const std::vector<ObjectId> ports_curr =
                order_for_choice(base_ports_by_node[step + 1], choice_for_parity(parity_curr));
            OrientationPlanScore candidate = cell.score;
            add_order_score(&candidate.order,
                            evaluate_increment(node_ids[step], node_ids[step + 1], ports_prev, ports_curr));
            if (use_lane_row_geometry) {
              const std::vector<ObjectId> ports_prev_prev =
                  order_for_choice(base_ports_by_node[step - 1], choice_for_parity(parity_prev_prev));
              candidate.adjacent_xy_intersections +=
                  count_adjacent_segment_xy_intersections(ports_prev_prev, ports_prev, ports_prev, ports_curr);
            }
            const int curr_orientation = parity_prev ^ parity_curr;
            if (curr_orientation != prev_orientation) {
              const bool counts_as_flip = (turn_angle_by_segment[step] + kAngleEps < kReverseStraightAngleDeg);
              if (counts_as_flip) {
                ++candidate.orientation_flips;
                const bool is_acute_turn =
                    (turn_angle_by_segment[step] + kAngleEps < layout_settings_.corner_threshold_deg);
                if (is_acute_turn) {
                  ++candidate.acute_orientation_flips;
                }
              }
            }

            DpCell& next = dp[step + 1][parity_prev][parity_curr];
            if (!next.reachable || orientation_plan_less(candidate, next.score)) {
              next.reachable = true;
              next.score = candidate;
              next.prev_prev_parity = parity_prev_prev;
            }
          }
        }
      }
    }

    bool has_best = false;
    OrientationPlanScore best_score{};
    int best_prev = 0;
    int best_curr = 0;
    for (int parity_prev : {0, 1}) {
      for (int parity_curr : {0, 1}) {
        const DpCell& cell = dp[segment_count][parity_prev][parity_curr];
        if (!cell.reachable) {
          continue;
        }
        if (!has_best || orientation_plan_less(cell.score, best_score)) {
          has_best = true;
          best_score = cell.score;
          best_prev = parity_prev;
          best_curr = parity_curr;
        }
      }
    }
    if (!has_best) {
      result.error = "failed to resolve deterministic node orientation plan";
      return result;
    }

    node_parity[node_ids.size() - 2] = best_prev;
    node_parity[node_ids.size() - 1] = best_curr;
    for (std::size_t step = segment_count; step > 1; --step) {
      const DpCell& cell = dp[step][node_parity[step - 1]][node_parity[step]];
      node_parity[step - 2] = cell.prev_prev_parity;
    }
  }

  auto build_local_order_score = [&](std::size_t seg, int parity_a, int parity_b, int prev_parity,
                                     bool has_prev_segment) {
    OrientationPlanScore score{};
    const std::vector<ObjectId> ports_a = order_for_choice(base_ports_by_node[seg], choice_for_parity(parity_a));
    const std::vector<ObjectId> ports_b =
        order_for_choice(base_ports_by_node[seg + 1], choice_for_parity(parity_b));
    score.order = evaluate_increment(node_ids[seg], node_ids[seg + 1], ports_a, ports_b);
    if (use_lane_row_geometry && has_prev_segment) {
      const std::vector<ObjectId> prev_ports =
          order_for_choice(base_ports_by_node[seg - 1], choice_for_parity(prev_parity));
      score.adjacent_xy_intersections =
          count_adjacent_segment_xy_intersections(prev_ports, ports_a, ports_a, ports_b);
    }
    if (has_prev_segment) {
      const int prev_orientation = prev_parity ^ parity_a;
      const int curr_orientation = parity_a ^ parity_b;
      if (curr_orientation != prev_orientation &&
          (turn_angle_by_segment[seg] + kAngleEps < kReverseStraightAngleDeg)) {
        ++score.orientation_flips;
        if (turn_angle_by_segment[seg] + kAngleEps < layout_settings_.corner_threshold_deg) {
          ++score.acute_orientation_flips;
        }
      }
    }
    return score;
  };
  auto order_choice_reason_from_scores = [&](const OrientationPlanScore& chosen, const OrientationPlanScore& alternate,
                                             BundleOrderPolicyKind policy) {
    if (policy != BundleOrderPolicyKind::kPermutableHomogeneous) {
      return BundleOrderChoiceReason::kFixedOrder;
    }
    if (use_lane_row_geometry && chosen.adjacent_xy_intersections != alternate.adjacent_xy_intersections) {
      return BundleOrderChoiceReason::kCrossingFewer;
    }
    if (chosen.order.cross_y != alternate.order.cross_y) {
      return BundleOrderChoiceReason::kCrossingFewer;
    }
    if (chosen.order.cross_z != alternate.order.cross_z ||
        std::abs(chosen.order.span_z_delta - alternate.order.span_z_delta) > 1e-6) {
      return BundleOrderChoiceReason::kSpacingBetter;
    }
    if (chosen.acute_orientation_flips != alternate.acute_orientation_flips ||
        chosen.orientation_flips != alternate.orientation_flips) {
      return BundleOrderChoiceReason::kTwistSmaller;
    }
    return BundleOrderChoiceReason::kKeptDefault;
  };

  std::vector<BundleOrderChoiceKind> node_order_choices(node_ids.size(), BundleOrderChoiceKind::kNormal);
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    node_order_choices[i] = choice_for_parity(node_parity[i]);
  }

  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId node_a = node_ids[seg];
    const ObjectId node_b = node_ids[seg + 1];
    const ConnectionContext span_context = span_context_for_segment(node_a, node_b);

    SegmentLaneAssignment assignment{};
    assignment.segment_index = seg;
    assignment.pole_a_id = node_a;
    assignment.pole_b_id = node_b;
    assignment.bundle_id = bundle_id;
    assignment.flow_kind = flow_kind;
    assignment.bundle_order_policy = bundle_order_policy;
    assignment.bundle_order_choice_a = node_order_choices[seg];
    assignment.bundle_order_choice_b = node_order_choices[seg + 1];
    assignment.port_ids_a = order_for_choice(base_ports_by_node[seg], assignment.bundle_order_choice_a);
    assignment.port_ids_b = order_for_choice(base_ports_by_node[seg + 1], assignment.bundle_order_choice_b);
    const bool chosen_orientation_flip =
        (assignment.bundle_order_choice_a != assignment.bundle_order_choice_b);
    const double turn_angle_deg = compute_turn_angle_deg(seg);
    const bool is_acute_turn = (seg > 0) && (turn_angle_deg + kAngleEps < layout_settings_.corner_threshold_deg);
    LaneFlipReason flip_reason = LaneFlipReason::kNone;
    bool flipped_from_previous = false;
    const bool previous_orientation_flip =
        (seg > 0) ? (node_order_choices[seg - 1] != node_order_choices[seg]) : false;
    if (seg > 0 && chosen_orientation_flip != previous_orientation_flip &&
        (turn_angle_deg + kAngleEps < kReverseStraightAngleDeg)) {
      flipped_from_previous = true;
      if (is_acute_turn) {
        flip_reason = LaneFlipReason::kAcuteTurn;
      }
    }
    if (bundle_order_policy == BundleOrderPolicyKind::kPermutableHomogeneous) {
      const int prev_parity = (seg > 0) ? node_parity[seg - 1] : 0;
      const OrientationPlanScore chosen_score =
          build_local_order_score(seg, node_parity[seg], node_parity[seg + 1], prev_parity, seg > 0);
      const OrientationPlanScore alternate_a =
          build_local_order_score(seg, node_parity[seg] ^ 1, node_parity[seg + 1], prev_parity, seg > 0);
      const OrientationPlanScore alternate_b =
          build_local_order_score(seg, node_parity[seg], node_parity[seg + 1] ^ 1, prev_parity, seg > 0);
      assignment.bundle_order_choice_reason_a =
          order_choice_reason_from_scores(chosen_score, alternate_a, bundle_order_policy);
      assignment.bundle_order_choice_reason_b =
          order_choice_reason_from_scores(chosen_score, alternate_b, bundle_order_policy);
    } else {
      assignment.bundle_order_choice_reason_a = BundleOrderChoiceReason::kFixedOrder;
      assignment.bundle_order_choice_reason_b = BundleOrderChoiceReason::kFixedOrder;
    }
    assignment.flipped_from_previous = flipped_from_previous;
    assignment.flip_reason = flip_reason;
    assignment.turn_angle_deg = turn_angle_deg;
    const SegmentRelationFeasibility relation_a = segment_relation_feasibility_for(node_a, node_b);
    const SegmentRelationFeasibility relation_b = segment_relation_feasibility_for(node_b, node_a);
    SegmentRelationFeasibility effective_relation_a = relation_a;
    SegmentRelationFeasibility effective_relation_b = relation_b;
    assignment.relation_a = node_relation_kind_at(seg);
    assignment.relation_b = node_relation_kind_at(seg + 1);
    if (static_cast<int>(assignment.port_ids_a.size()) != lane_count ||
        static_cast<int>(assignment.port_ids_b.size()) != lane_count) {
      result.error = "failed to materialize lane assignment plan";
      return result;
    }

    auto ports_use_branch_support = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr && port->placement_source == PortPlacementSourceKind::kBranchSupport;
      });
    };
    auto ports_use_constrained_solver = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr && port->placement_source == PortPlacementSourceKind::kPlacementBandConstrained;
      });
    };
    auto ports_use_special_case_source = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr &&
               (port->placement_source == PortPlacementSourceKind::kGenerated ||
                port->placement_source == PortPlacementSourceKind::kBranchSupport ||
                port->placement_source == PortPlacementSourceKind::kAerialBranch);
      });
    };
    assignment.uses_branch_support =
        ports_use_branch_support(assignment.port_ids_a) || ports_use_branch_support(assignment.port_ids_b);
    assignment.solver_used_same_level_constraint =
        ports_use_constrained_solver(assignment.port_ids_a) || ports_use_constrained_solver(assignment.port_ids_b);
    auto relation_rank = [](JunctionRelationKind kind) {
      switch (kind) {
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
    const JunctionRelationKind segment_relation_kind =
        (relation_rank(assignment.relation_a) >= relation_rank(assignment.relation_b)) ? assignment.relation_a
                                                                                        : assignment.relation_b;
    const SegmentRelationFeasibility segment_feasibility =
        (relation_rank(relation_a.kind) >= relation_rank(relation_b.kind)) ? relation_a : relation_b;
    const bool segment_same_level_feasible =
        effective_relation_a.same_level_feasible && effective_relation_b.same_level_feasible;
    assignment.used_special_case_ports =
        !assignment.solver_used_same_level_constraint && !segment_same_level_feasible &&
        (ports_use_special_case_source(assignment.port_ids_a) || ports_use_special_case_source(assignment.port_ids_b));
    assignment.lowering_blocked_by_policy = false;
    const bool touches_lowered_corner =
        (segment_relation_kind == JunctionRelationKind::kCornerContinuation && !segment_same_level_feasible) ||
        ((seg < node_uses_down_offset.size() && node_uses_down_offset[seg]) ||
         (seg + 1 < node_uses_down_offset.size() && node_uses_down_offset[seg + 1]));
    const bool should_cross_lower =
        (segment_relation_kind == JunctionRelationKind::kCrossUnderpass && !segment_same_level_feasible);
    const bool should_branch_lower =
        (assignment.uses_branch_support || segment_relation_kind == JunctionRelationKind::kSideBranch) &&
        !segment_same_level_feasible;
    const bool should_corner_lower =
        (touches_lowered_corner && !segment_same_level_feasible);
    if (should_cross_lower && lowering_policy.enable_cross_underpass) {
      assignment.lowering_kind = BackboneLoweringKind::kCrossUnderpass;
    } else if (should_branch_lower && lowering_policy.enable_branch_support) {
      assignment.lowering_kind = BackboneLoweringKind::kBranchSupport;
    } else if (should_corner_lower && lowering_policy.enable_acute_corner) {
      assignment.lowering_kind = BackboneLoweringKind::kAcuteCorner;
    } else {
      assignment.lowering_kind = BackboneLoweringKind::kNone;
    }
    assignment.lowering_blocked_by_policy =
        assignment.lowering_kind == BackboneLoweringKind::kNone &&
        ((should_cross_lower && !lowering_policy.enable_cross_underpass) ||
         (should_branch_lower && !lowering_policy.enable_branch_support) ||
         (should_corner_lower && !lowering_policy.enable_acute_corner));
    if (assignment.lowering_blocked_by_policy) {
      assignment.same_level_reason = SameLevelFeasibilityReason::kCategoryPolicyDisabled;
    }
    assignment.unresolved_same_level_conflict =
        !segment_same_level_feasible && assignment.lowering_blocked_by_policy;
    assignment.branch_down_offset_m =
        (assignment.lowering_kind != BackboneLoweringKind::kNone) ? effective_branch_down_offset_m : 0.0;
    EndpointSideDecision side_decision_a =
        preferred_side_axis_for_endpoint(node_a, node_b, effective_relation_a, assignment.lowering_kind);
    EndpointSideDecision side_decision_b =
        preferred_side_axis_for_endpoint(node_b, node_a, effective_relation_b, assignment.lowering_kind);
    finalize_side_sign_for_ports(&side_decision_a, node_a, node_b, assignment.port_ids_a);
    finalize_side_sign_for_ports(&side_decision_b, node_b, node_a, assignment.port_ids_b);
    assignment.side_assignment_rule_a = side_decision_a.side_assignment_rule;
    assignment.side_assignment_rule_b = side_decision_b.side_assignment_rule;
    assignment.support_orientation_rule_a = side_decision_a.support_orientation_rule;
    assignment.support_orientation_rule_b = side_decision_b.support_orientation_rule;
    assignment.used_junction_pair_side_assignment_a = side_decision_a.used_junction_pair_side_assignment;
    assignment.used_junction_pair_side_assignment_b = side_decision_b.used_junction_pair_side_assignment;
    assignment.has_side_axis_a = side_decision_a.has_side_axis;
    assignment.has_side_axis_b = side_decision_b.has_side_axis;
    assignment.side_axis_a = support_orientation_axis_for_endpoint(node_a, node_b, side_decision_a);
    assignment.side_axis_b = support_orientation_axis_for_endpoint(node_b, node_a, side_decision_b);
    assignment.chosen_side_sign_a = side_decision_a.chosen_side_sign;
    assignment.chosen_side_sign_b = side_decision_b.chosen_side_sign;
    side_decision_a.side_axis = assignment.side_axis_a;
    side_decision_b.side_axis = assignment.side_axis_b;
    side_decision_a.has_side_axis = std::isfinite(assignment.side_axis_a.x) && std::isfinite(assignment.side_axis_a.y);
    side_decision_b.has_side_axis = std::isfinite(assignment.side_axis_b.x) && std::isfinite(assignment.side_axis_b.y);
    assignment.decision_a =
        build_endpoint_decision(effective_relation_a, side_decision_a, node_a, assignment.bundle_order_choice_a,
                                assignment.bundle_order_choice_reason_a, assignment.solver_used_same_level_constraint,
                                assignment.used_special_case_ports, assignment.lowering_blocked_by_policy,
                                assignment.unresolved_same_level_conflict);
    assignment.decision_b =
        build_endpoint_decision(effective_relation_b, side_decision_b, node_b, assignment.bundle_order_choice_b,
                                assignment.bundle_order_choice_reason_b, assignment.solver_used_same_level_constraint,
                                assignment.used_special_case_ports, assignment.lowering_blocked_by_policy,
                                assignment.unresolved_same_level_conflict);
    sync_assignment_from_decisions(&assignment);

    for (int lane = 0; lane < lane_count; ++lane) {
      const ObjectId port_a_id = assignment.port_ids_a[static_cast<std::size_t>(lane)];
      const ObjectId port_b_id = assignment.port_ids_b[static_cast<std::size_t>(lane)];
      const auto add = AddSpan(port_a_id, port_b_id, SpanKind::kDistribution, category_to_span_layer(category),
                               bundle_id);
      if (!add.ok) {
        result.error = add.error;
        return result;
      }
      append_change_set(result.change_set, add.change_set);
      result.value.push_back(add.value);

      const Port* pa = edit_state_access().ports.find(port_a_id);
      const Port* pb = edit_state_access().ports.find(port_b_id);
      (void)pa;
      (void)pb;

      Span* span = edit_state_access().spans.find(add.value);
      if (span != nullptr) {
        span->endpoint_node_a_id = node_a;
        span->endpoint_node_b_id = node_b;
        span->placement_context = span_context;
        span->generated_by_rule = true;
        span->generation.generated = true;
        add_unique_id(result.change_set.updated_ids, span->id);
      }
    }

    if (out_lane_assignments != nullptr) {
      out_lane_assignments->push_back(assignment);
    }
    if (out_edge_orientations != nullptr) {
      BackboneEdgeOrientation edge_orientation{};
      edge_orientation.node_a_id = assignment.pole_a_id;
      edge_orientation.node_b_id = assignment.pole_b_id;
      edge_orientation.bundle_template_id = bundle_template_id;
      edge_orientation.flow_decision_rule = assignment.flow_decision_rule;
      edge_orientation.flow_kind = flow_kind;
      edge_orientation.relation_a = assignment.relation_a;
      edge_orientation.relation_b = assignment.relation_b;
      edge_orientation.continuity_class = assignment.continuity_class;
      edge_orientation.default_lower_required = assignment.default_lower_required;
      edge_orientation.same_level_feasible = assignment.same_level_feasible;
      edge_orientation.same_level_reason = assignment.same_level_reason;
      edge_orientation.projected_spacing_topview_m = assignment.projected_spacing_topview_m;
      edge_orientation.required_clearance_m = assignment.required_clearance_m;
      edge_orientation.lowering_blocked_by_policy = assignment.lowering_blocked_by_policy;
      edge_orientation.unresolved_same_level_conflict = assignment.unresolved_same_level_conflict;
      edge_orientation.solver_used_same_level_constraint = assignment.solver_used_same_level_constraint;
      edge_orientation.used_special_case_ports = assignment.used_special_case_ports;
      edge_orientation.bundle_order_policy = assignment.bundle_order_policy;
      edge_orientation.bundle_order_choice_a = assignment.bundle_order_choice_a;
      edge_orientation.bundle_order_choice_b = assignment.bundle_order_choice_b;
      edge_orientation.bundle_order_choice_reason_a = assignment.bundle_order_choice_reason_a;
      edge_orientation.bundle_order_choice_reason_b = assignment.bundle_order_choice_reason_b;
    edge_orientation.orientation =
        (assignment.bundle_order_choice_a != assignment.bundle_order_choice_b) ? LaneOrientation::kReversed
                                                                               : LaneOrientation::kNormal;
      edge_orientation.uses_branch_support = assignment.uses_branch_support;
      edge_orientation.lowering_kind = assignment.lowering_kind;
      edge_orientation.branch_down_offset_m = assignment.branch_down_offset_m;
      edge_orientation.flipped_from_previous = assignment.flipped_from_previous;
      edge_orientation.flip_reason = assignment.flip_reason;
      edge_orientation.turn_angle_deg = assignment.turn_angle_deg;
      out_edge_orientations->push_back(edge_orientation);
    }
  }

  result.ok = !result.value.empty();
  if (!result.ok && result.error.empty()) {
    result.error = "failed to generate grouped spans";
  }
  return result;
}

} // namespace wire::core
