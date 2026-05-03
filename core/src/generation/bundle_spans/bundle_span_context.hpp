#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

#include "wire/core/core_authoritative_types.hpp"
#include "wire/core/core_runtime_types.hpp"
#include "wire/core/core_state_api_types.hpp"
#include "wire/core/debug_types.hpp"
#include "wire/core/support_layout_types.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core::generation::detail {

struct SegmentRelationFeasibility {
  JunctionRelationKind kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  bool through_pair_accepted = false;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool peer_relation_found = false;
  JunctionRelationKind peer_relation_kind = JunctionRelationKind::kNone;
  bool peer_in_route = false;
  bool peer_in_through_pair = false;
  bool peer_through_pair_accepted = false;
  ContinuityCategoryClass peer_continuity_class = ContinuityCategoryClass::kPointLike;
  bool peer_default_lower_required = false;
  bool peer_same_level_feasible = true;
  SameLevelFeasibilityReason peer_reason = SameLevelFeasibilityReason::kNone;
};

struct JunctionIncidentFeasibility {
  ObjectId neighbor_node_id = kInvalidObjectId;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
};

struct JunctionFeasibility {
  ObjectId node_id = kInvalidObjectId;
  std::vector<JunctionIncidentFeasibility> incidents{};
};

struct EndpointSideDecision {
  Vec3d side_axis{0.0, 0.0, 0.0};
  bool has_side_axis = false;
  double chosen_side_sign = 0.0;
  ObjectId pair_peer_low = kInvalidObjectId;
  ObjectId pair_peer_high = kInvalidObjectId;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  bool used_junction_pair_side_assignment = false;
};

struct LoweredSupportPairInfo {
  ObjectId companion_peer_id = kInvalidObjectId;
  ObjectId pair_peer_low = kInvalidObjectId;
  ObjectId pair_peer_high = kInvalidObjectId;
  bool has_pair = false;
};

struct GroupedSpanSharedContext {
  const std::vector<ObjectId>& node_ids;
  const std::unordered_map<ObjectId, SupportNode>& support_node_by_id;
  const EditState& edit_state;
  const RelationIndex& relation_index;
  const ConnectionIndex& connection_index;
  const std::unordered_map<ObjectId, JunctionRelation>* junction_relations_by_node = nullptr;
  const std::unordered_map<ObjectId, JunctionFeasibility>* junction_feasibility_by_node = nullptr;
  const std::unordered_map<ObjectId, PoleOrientationDebugRecord>* pole_orientation_debug_records = nullptr;
  const std::unordered_map<ObjectId, Vec3d>* node_side_axis_by_node = nullptr;
  const std::unordered_map<ObjectId, std::unordered_map<ObjectId, double>>* node_side_sign_by_peer = nullptr;
  const std::unordered_map<ObjectId, std::array<ObjectId, 2>>* through_pair_by_node = nullptr;
  const std::unordered_map<ObjectId, std::array<ObjectId, 2>>* cross_pair_by_node = nullptr;

  [[nodiscard]] ObjectId resolve_span_endpoint_node(const Span& span, const Port* port, bool is_a) const {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  }

  [[nodiscard]] ObjectId resolve_span_endpoint_node(const Span& span, ObjectId owner_pole_id, bool is_a) const {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return owner_pole_id;
  }

  [[nodiscard]] Vec3d support_position(ObjectId node_id) const {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state.poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    return {};
  }

  [[nodiscard]] const Pole* support_pole(ObjectId node_id) const {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      if (it->second.pole_id != kInvalidObjectId) {
        if (const Pole* pole = edit_state.poles.find(it->second.pole_id); pole != nullptr) {
          return pole;
        }
      }
      if (it->second.support_kind != SupportKind::kPole) {
        return nullptr;
      }
    }
    return edit_state.poles.find(node_id);
  }

  [[nodiscard]] SupportKind support_kind(ObjectId node_id) const {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.support_kind;
    }
    return (edit_state.poles.find(node_id) != nullptr) ? SupportKind::kPole : SupportKind::kMidair;
  }

  [[nodiscard]] const JunctionIncidentRelation* incident_relation_for(ObjectId node_id, ObjectId peer_id) const {
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
  }

  [[nodiscard]] const JunctionIncidentFeasibility* incident_feasibility_for(ObjectId node_id, ObjectId peer_id) const {
    if (junction_feasibility_by_node == nullptr || node_id == kInvalidObjectId || peer_id == kInvalidObjectId) {
      return nullptr;
    }
    const auto it = junction_feasibility_by_node->find(node_id);
    if (it == junction_feasibility_by_node->end()) {
      return nullptr;
    }
    for (const JunctionIncidentFeasibility& incident : it->second.incidents) {
      if (incident.neighbor_node_id == peer_id) {
        return &incident;
      }
    }
    return nullptr;
  }
};

} // namespace wire::core::generation::detail
