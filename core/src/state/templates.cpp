#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

namespace {

constexpr double kZeroLengthEps = 1e-9;
constexpr PoleTypeId kDistributionPoleType = 1;
constexpr PoleTypeId kCommunicationPoleType = 2;
constexpr CableTemplateId kHighVoltageCableTemplate = 1;
constexpr CableTemplateId kLowVoltageCableTemplate = 2;
constexpr CableTemplateId kCommunicationCableTemplate = 3;
constexpr CableTemplateId kOpticalCableTemplate = 4;
constexpr AttachmentTemplateId kGenericAttachmentTemplate = 1;
constexpr AttachmentTemplateId kHiddenAttachmentTemplate = 2;
constexpr AttachmentTemplateId kInternalPathAttachmentTemplate = 3;
ConnectionCategory port_layer_to_category(PortLayer layer) {
  switch (layer) {
  case PortLayer::kHighVoltage:
    return ConnectionCategory::kHighVoltage;
  case PortLayer::kLowVoltage:
    return ConnectionCategory::kLowVoltage;
  case PortLayer::kCommunication:
    return ConnectionCategory::kCommunication;
  case PortLayer::kOptical:
    return ConnectionCategory::kOptical;
  default:
    return ConnectionCategory::kLowVoltage;
  }
}

PortLayer span_layer_to_port_layer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return PortLayer::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return PortLayer::kLowVoltage;
  case SpanLayer::kCommunication:
    return PortLayer::kCommunication;
  case SpanLayer::kOptical:
    return PortLayer::kOptical;
  default:
    return PortLayer::kUnknown;
  }
}

ConnectionCategory span_layer_to_category(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return ConnectionCategory::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return ConnectionCategory::kLowVoltage;
  case SpanLayer::kCommunication:
    return ConnectionCategory::kCommunication;
  case SpanLayer::kOptical:
    return ConnectionCategory::kOptical;
  default:
    return ConnectionCategory::kLowVoltage;
  }
}

int target_template_layer_for_category(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return 2;
  case ConnectionCategory::kLowVoltage:
    return 1;
  case ConnectionCategory::kCommunication:
    return 1;
  case ConnectionCategory::kOptical:
    return 1;
  case ConnectionCategory::kDrop:
    return 0;
  default:
    return 1;
  }
}

int role_score_for_context(SlotRole role, ConnectionContext context) {
  switch (context) {
  case ConnectionContext::kTrunkContinue:
  case ConnectionContext::kCornerPass:
    if (role == SlotRole::kTrunkPreferred)
      return 120;
    if (role == SlotRole::kNeutral)
      return 40;
    return 0;
  case ConnectionContext::kBranchAdd:
    if (role == SlotRole::kBranchPreferred)
      return 120;
    if (role == SlotRole::kNeutral)
      return 20;
    if (role == SlotRole::kTrunkPreferred)
      return 10;
    return 0;
  case ConnectionContext::kDropAdd:
    if (role == SlotRole::kDropPreferred)
      return 120;
    if (role == SlotRole::kNeutral)
      return 10;
    return 0;
  default:
    return 0;
  }
}

SlotSide inner_side_for_turn(double turn_sign) {
  if (turn_sign > 1e-9) {
    return SlotSide::kLeft;
  }
  if (turn_sign < -1e-9) {
    return SlotSide::kRight;
  }
  return SlotSide::kCenter;
}

double apply_corner_side_scale(double local_y, SlotSide slot_side, double turn_sign, double side_scale) {
  if (slot_side == SlotSide::kCenter) {
    return local_y;
  }
  // Always widen non-center lanes for clearance; keep outer side wider than inner side.
  const double inner_scale = 1.0 + (side_scale - 1.0) * 0.35;
  const SlotSide inner_side = inner_side_for_turn(turn_sign);
  if (inner_side == SlotSide::kCenter) {
    return local_y * side_scale;
  }
  if (slot_side == inner_side) {
    return local_y * inner_scale;
  }
  return local_y * side_scale;
}

template <typename TValue> void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
  for (const TValue& value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

void append_change_set(ChangeSet& dst, const ChangeSet& src) {
  append_unique(dst.created_ids, src.created_ids);
  append_unique(dst.updated_ids, src.updated_ids);
  append_unique(dst.deleted_ids, src.deleted_ids);
  append_unique(dst.dirty_span_ids, src.dirty_span_ids);
}

Vec3d local_to_world_on_pole(const Transformd& tf, double yaw_deg, const Vec3d& local) {
  return LocalPointToWorld(BuildPoleFrame(tf, yaw_deg), local);
}

double normalize_yaw_deg(double yaw_deg) {
  return NormalizeYawDeg(yaw_deg);
}

} // namespace

PortLayer CoreState::category_to_port_layer(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return PortLayer::kHighVoltage;
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
    return PortLayer::kLowVoltage;
  case ConnectionCategory::kCommunication:
    return PortLayer::kCommunication;
  case ConnectionCategory::kOptical:
    return PortLayer::kOptical;
  default:
    return PortLayer::kUnknown;
  }
}

SpanLayer CoreState::category_to_span_layer(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return SpanLayer::kHighVoltage;
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
    return SpanLayer::kLowVoltage;
  case ConnectionCategory::kCommunication:
    return SpanLayer::kCommunication;
  case ConnectionCategory::kOptical:
    return SpanLayer::kOptical;
  default:
    return SpanLayer::kUnknown;
  }
}

BundleKind CoreState::category_to_bundle_kind(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return BundleKind::kHighVoltage;
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
    return BundleKind::kLowVoltage;
  case ConnectionCategory::kCommunication:
    return BundleKind::kCommunication;
  case ConnectionCategory::kOptical:
    return BundleKind::kOptical;
  default:
    return BundleKind::kLowVoltage;
  }
}

PortKind CoreState::category_to_port_kind(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
    return PortKind::kCommunication;
  default:
    return PortKind::kPower;
  }
}

void CoreState::register_default_pole_types() {
  PoleTypeDefinition dist{};
  dist.id = kDistributionPoleType;
  dist.name = "DistributionPole";
  dist.description = "Default distribution pole";
  dist.port_slots = {
      {100,
       ConnectionCategory::kHighVoltage,
       {0.0, -0.6, 9.2},
       {},
       2,
       SlotSide::kLeft,
       SlotRole::kTrunkPreferred,
       30,
       false,
       true},
      {101,
       ConnectionCategory::kHighVoltage,
       {0.0, 0.0, 9.3},
       {},
       2,
       SlotSide::kCenter,
       SlotRole::kTrunkPreferred,
       29,
       false,
       true},
      {102,
       ConnectionCategory::kHighVoltage,
       {0.0, 0.6, 9.2},
       {},
       2,
       SlotSide::kRight,
       SlotRole::kTrunkPreferred,
       28,
       false,
       true},
      {200,
       ConnectionCategory::kLowVoltage,
       {0.0, -0.4, 6.8},
       {},
       1,
       SlotSide::kLeft,
       SlotRole::kTrunkPreferred,
       20,
       false,
       true},
      {201,
       ConnectionCategory::kLowVoltage,
       {0.0, 0.4, 6.8},
       {},
       1,
       SlotSide::kRight,
       SlotRole::kTrunkPreferred,
       19,
       false,
       true},
      {202,
       ConnectionCategory::kLowVoltage,
       {0.0, 0.0, 6.6},
       {},
       1,
       SlotSide::kCenter,
       SlotRole::kBranchPreferred,
       18,
       false,
       true},
      {300,
       ConnectionCategory::kCommunication,
       {0.0, -0.8, 7.8},
       {},
       1,
       SlotSide::kLeft,
       SlotRole::kTrunkPreferred,
       15,
       false,
       true},
      {301,
       ConnectionCategory::kOptical,
       {0.0, 0.8, 7.8},
       {},
       1,
       SlotSide::kRight,
       SlotRole::kTrunkPreferred,
       14,
       false,
       true},
      {400,
       ConnectionCategory::kDrop,
       {0.0, 0.0, 4.2},
       {},
       0,
       SlotSide::kCenter,
       SlotRole::kDropPreferred,
       10,
       true,
       true},
  };
  dist.anchor_slots = {
      {500, AnchorSupportKind::kGround, {0.0, 0.0, 0.5}, 10, true},
  };
  pole_types_[dist.id] = dist;

  PoleTypeDefinition comm{};
  comm.id = kCommunicationPoleType;
  comm.name = "CommunicationPole";
  comm.description = "Communication-first pole";
  comm.port_slots = {
      {600,
       ConnectionCategory::kCommunication,
       {0.0, -0.7, 8.6},
       {},
       2,
       SlotSide::kLeft,
       SlotRole::kTrunkPreferred,
       30,
       false,
       true},
      {601,
       ConnectionCategory::kCommunication,
       {0.0, 0.0, 8.7},
       {},
       2,
       SlotSide::kCenter,
       SlotRole::kTrunkPreferred,
       29,
       false,
       true},
      {602,
       ConnectionCategory::kCommunication,
       {0.0, 0.7, 8.6},
       {},
       2,
       SlotSide::kRight,
       SlotRole::kTrunkPreferred,
       28,
       false,
       true},
      {700,
       ConnectionCategory::kOptical,
       {0.0, -0.4, 7.6},
       {},
       1,
       SlotSide::kLeft,
       SlotRole::kTrunkPreferred,
       25,
       false,
       true},
      {701,
       ConnectionCategory::kOptical,
       {0.0, 0.4, 7.6},
       {},
       1,
       SlotSide::kRight,
       SlotRole::kTrunkPreferred,
       24,
       false,
       true},
      {800,
       ConnectionCategory::kLowVoltage,
       {0.0, 0.0, 5.8},
       {},
       1,
       SlotSide::kCenter,
       SlotRole::kBranchPreferred,
       10,
       false,
       true},
      {801,
       ConnectionCategory::kDrop,
       {0.0, 0.0, 4.0},
       {},
       0,
       SlotSide::kCenter,
       SlotRole::kDropPreferred,
       9,
       true,
       true},
  };
  comm.anchor_slots = {
      {900, AnchorSupportKind::kGround, {0.0, 0.0, 0.5}, 10, true},
  };
  pole_types_[comm.id] = comm;
}

const PoleTypeDefinition* CoreState::find_pole_type(PoleTypeId pole_type_id) const {
  auto it = pole_types_.find(pole_type_id);
  if (it == pole_types_.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::register_default_bundle_templates() {
  BundleTemplate hv{};
  hv.id = BundleKind::kHighVoltage;
  hv.name = "HV_3PH";
  hv.category = ConnectionCategory::kHighVoltage;
  hv.cable_template_id = kHighVoltageCableTemplate;
  hv.default_layer = SpanLayer::kHighVoltage;
  hv.preserve_conductor_identity = true;
  hv.count_rule = BundleCountRuleKind::kFixed;
  hv.fixed_count = 3;
  hv.min_count = 3;
  hv.max_count = 3;
  hv.default_count = 3;
  hv.default_spacing_m = 0.45;
  hv.allow_mirror = true;
  hv.allow_midair_node = true;
  hv.allow_midair_branch = false;
  bundle_templates_[hv.id] = hv;

  BundleTemplate lv{};
  lv.id = BundleKind::kLowVoltage;
  lv.name = "DEFAULT_SINGLE";
  lv.category = ConnectionCategory::kLowVoltage;
  lv.cable_template_id = kLowVoltageCableTemplate;
  lv.default_layer = SpanLayer::kLowVoltage;
  lv.preserve_conductor_identity = false;
  lv.count_rule = BundleCountRuleKind::kFixed;
  lv.fixed_count = 1;
  lv.min_count = 1;
  lv.max_count = 1;
  lv.default_count = 1;
  lv.default_spacing_m = 0.20;
  lv.allow_mirror = true;
  lv.allow_midair_node = true;
  lv.allow_midair_branch = true;
  bundle_templates_[lv.id] = lv;

  BundleTemplate comm{};
  comm.id = BundleKind::kCommunication;
  comm.name = "COMM_BUNDLE";
  comm.category = ConnectionCategory::kCommunication;
  comm.cable_template_id = kCommunicationCableTemplate;
  comm.default_layer = SpanLayer::kCommunication;
  comm.preserve_conductor_identity = false;
  comm.count_rule = BundleCountRuleKind::kRange;
  comm.fixed_count = 0;
  comm.min_count = 1;
  comm.max_count = 8;
  comm.default_count = 1;
  comm.default_spacing_m = 0.20;
  comm.allow_mirror = true;
  comm.allow_midair_node = true;
  comm.allow_midair_branch = true;
  bundle_templates_[comm.id] = comm;

  BundleTemplate optical{};
  optical.id = BundleKind::kOptical;
  optical.name = "OPTICAL_FIXED";
  optical.category = ConnectionCategory::kOptical;
  optical.cable_template_id = kOpticalCableTemplate;
  optical.default_layer = SpanLayer::kOptical;
  optical.preserve_conductor_identity = false;
  optical.count_rule = BundleCountRuleKind::kFixed;
  optical.fixed_count = 1;
  optical.min_count = 1;
  optical.max_count = 1;
  optical.default_count = 1;
  optical.default_spacing_m = 0.20;
  optical.allow_mirror = true;
  optical.allow_midair_node = true;
  optical.allow_midair_branch = true;
  bundle_templates_[optical.id] = optical;
}

const BundleTemplate* CoreState::find_bundle_template(BundleKind bundle_template_id) const {
  auto it = bundle_templates_.find(bundle_template_id);
  if (it == bundle_templates_.end()) {
    return nullptr;
  }
  return &it->second;
}

std::vector<PortSlotTemplate> CoreState::sorted_port_slots(const PoleTypeDefinition& pole_type,
                                                           ConnectionCategory category) const {
  std::vector<PortSlotTemplate> out;
  for (const PortSlotTemplate& slot : pole_type.port_slots) {
    if (slot.enabled && slot.category == category) {
      out.push_back(slot);
    }
  }
  std::sort(out.begin(), out.end(), [](const PortSlotTemplate& a, const PortSlotTemplate& b) {
    if (a.priority != b.priority) {
      return a.priority > b.priority;
    }
    return a.slot_id < b.slot_id;
  });
  return out;
}

bool CoreState::is_port_slot_used(ObjectId pole_id, int slot_id) const {
  for (const Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id == pole_id && port.source_slot_id == slot_id) {
      return true;
    }
  }
  return false;
}

EditResult<ObjectId> CoreState::ensure_pole_slot_port(const SlotSelectionRequest& request, int* out_slot_id) {
  EditResult<ObjectId> result;
  const Pole* pole = edit_state_.poles.find(request.pole_id);

  SlotSelectionDebugRecord debug{};
  debug.pole_id = request.pole_id;
  debug.peer_pole_id = request.peer_pole_id;
  debug.reference_span_id = request.reference_span_id;
  debug.category = request.category;
  debug.connection_context = request.connection_context;
  debug.pole_context = request.pole_context;
  debug.corner_angle_deg = request.corner_angle_deg;
  debug.corner_turn_sign = request.corner_turn_sign;
  debug.side_scale = compute_side_scale(request.pole_context, request.corner_angle_deg);

  auto push_debug = [&]() {
    slot_selection_debug_records_.push_back(debug);
    constexpr std::size_t kMaxDebugRecords = 256;
    if (slot_selection_debug_records_.size() > kMaxDebugRecords) {
      slot_selection_debug_records_.erase(slot_selection_debug_records_.begin());
    }
  };

  if (pole == nullptr) {
    result.error = "pole not found";
    debug.result = result.error;
    push_debug();
    return result;
  }

  auto connection_count = [&](ObjectId port_id) -> std::size_t {
    auto it = connection_index_.spans_by_port.find(port_id);
    if (it == connection_index_.spans_by_port.end()) {
      return 0;
    }
    return it->second.size();
  };

  auto same_side_layer_usage = [&](SlotSide side, int layer) -> std::size_t {
    std::size_t count = 0;
    for (const Port& port : edit_state_.ports.items()) {
      if (port.owner_pole_id != request.pole_id) {
        continue;
      }
      if (port.template_side != side || port.template_layer != layer) {
        continue;
      }
      if (connection_count(port.id) > 0) {
        ++count;
      }
    }
    return count;
  };

  const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id);
  if (pole_type != nullptr) {
    auto slots = sorted_port_slots(*pole_type, request.category);
    if (request.preferred_slot_id >= 0) {
      auto it = std::find_if(slots.begin(), slots.end(),
                             [&](const PortSlotTemplate& slot) { return slot.slot_id == request.preferred_slot_id; });
      if (it != slots.end() && it != slots.begin()) {
        PortSlotTemplate preferred = *it;
        slots.erase(it);
        slots.insert(slots.begin(), preferred);
      }
    }

    const int target_layer = target_template_layer_for_category(request.category);
    const Pole* peer_pole = edit_state_.poles.find(request.peer_pole_id);
    const SlotSide preferred_side = preferred_side_from_geometry(*pole, peer_pole, 0.10);
    const bool prefer_non_center = (preferred_side != SlotSide::kCenter);

    int best_total = std::numeric_limits<int>::min();
    int best_tie = -1;
    const PortSlotTemplate* best_slot = nullptr;
    Port* best_port = nullptr;

    for (const PortSlotTemplate& slot : slots) {
      SlotCandidateDebug candidate{};
      candidate.slot_id = slot.slot_id;
      candidate.category_score = (slot.category == request.category) ? 500 : -100000;
      if (candidate.category_score < 0) {
        candidate.eligible = false;
        candidate.reason = "category mismatch";
        debug.candidates.push_back(candidate);
        continue;
      }

      candidate.role_score = role_score_for_context(slot.role, request.connection_context);
      candidate.context_score = 0;
      if (request.connection_context == ConnectionContext::kCornerPass) {
        candidate.context_score += (slot.side == SlotSide::kCenter) ? 10 : 30;
      } else if (request.connection_context == ConnectionContext::kBranchAdd) {
        candidate.context_score += (slot.side == SlotSide::kCenter) ? 0 : 20;
      } else if (request.connection_context == ConnectionContext::kDropAdd) {
        candidate.context_score += (slot.side == SlotSide::kCenter) ? 25 : 0;
      }

      candidate.layer_score = 60 - (20 * std::abs(slot.layer - target_layer));
      candidate.side_score = 0;
      if (prefer_non_center) {
        if (slot.side == preferred_side) {
          candidate.side_score += 220;
        } else if (slot.side == SlotSide::kCenter) {
          candidate.side_score -= 15;
        }
      } else {
        // Geometry cannot decide left/right near center; only apply tiny deterministic fallback.
        if (((request.branch_index & 1u) == 0u && slot.side == SlotSide::kLeft) ||
            ((request.branch_index & 1u) == 1u && slot.side == SlotSide::kRight)) {
          candidate.side_score += 2;
        }
      }
      candidate.priority_score = slot.priority;

      Port* slot_port = nullptr;
      for (Port& port : edit_state_.ports.items_mutable()) {
        if (port.owner_pole_id == request.pole_id && port.source_slot_id == slot.slot_id) {
          slot_port = &port;
          break;
        }
      }

      candidate.usage_count = (slot_port == nullptr) ? 0 : connection_count(slot_port->id);
      if (slot_port == nullptr) {
        candidate.usage_score = request.allow_generate_port ? 75 : -100000;
      } else if (candidate.usage_count == 0) {
        candidate.usage_score = 80;
      } else if (slot.allow_multiple) {
        candidate.usage_score = -25 * static_cast<int>(candidate.usage_count);
      } else {
        candidate.usage_score = -100000;
      }

      candidate.congestion_count = same_side_layer_usage(slot.side, slot.layer);
      candidate.congestion_score = -15 * static_cast<int>(candidate.congestion_count);
      candidate.tie_breaker = static_cast<int>(
          deterministic_tiebreak_0_255(request.pole_id, slot.slot_id, request.category, request.connection_context,
                                       request.peer_pole_id, request.reference_span_id, request.branch_index));
      candidate.total_score = candidate.category_score + candidate.context_score + candidate.layer_score +
                              candidate.side_score + candidate.role_score + candidate.priority_score +
                              candidate.usage_score + candidate.congestion_score + (candidate.tie_breaker / 16);

      candidate.eligible = candidate.usage_score > -100000;
      candidate.reason = candidate.eligible ? "ok" : "slot unavailable";
      debug.candidates.push_back(candidate);

      if (!candidate.eligible) {
        continue;
      }
      if (candidate.total_score > best_total ||
          (candidate.total_score == best_total && candidate.tie_breaker > best_tie)) {
        best_total = candidate.total_score;
        best_tie = candidate.tie_breaker;
        best_slot = &slot;
        best_port = slot_port;
      }
    }

    if (best_slot != nullptr) {
      if (best_port != nullptr) {
        result.ok = true;
        result.value = best_port->id;
        if (out_slot_id != nullptr) {
          *out_slot_id = best_slot->slot_id;
        }
        debug.selected_slot_id = best_slot->slot_id;
        debug.result = "selected existing slot port";
        push_debug();
        return result;
      }

      Vec3d adjusted_local = best_slot->local_position;
      const bool apply_angle_correction = layout_settings_.angle_correction_enabled &&
                                          request.pole_context == PoleContextKind::kCorner &&
                                          best_slot->side != SlotSide::kCenter;
      double applied_scale = 1.0;
      if (apply_angle_correction) {
        adjusted_local.y =
            apply_corner_side_scale(adjusted_local.y, best_slot->side, request.corner_turn_sign, debug.side_scale);
        if (std::abs(best_slot->local_position.y) > 1e-9) {
          applied_scale = std::abs(adjusted_local.y / best_slot->local_position.y);
        }
      }
      adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, best_slot->side);
      const Vec3d world_position =
          local_to_world_on_pole(pole->world_transform, effective_pole_yaw_deg(*pole), adjusted_local);
      EditResult<ObjectId> add_port_result =
          AddPort(request.pole_id, world_position, category_to_port_kind(request.category),
                  category_to_port_layer(request.category), best_slot->local_direction);
      if (!add_port_result.ok) {
        debug.result = "failed to create slot port: " + add_port_result.error;
        push_debug();
        return add_port_result;
      }
      Port* created = edit_state_.ports.find(add_port_result.value);
      if (created != nullptr) {
        created->category = request.category;
        created->source_slot_id = best_slot->slot_id;
        created->template_layer = best_slot->layer;
        created->template_side = best_slot->side;
        created->template_role = best_slot->role;
        created->generated_from_template = true;
        created->generated_by_rule = true;
        created->placement_context = request.connection_context;
        created->angle_correction_applied = apply_angle_correction;
        created->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
        apply_port_position_mode(*created, PortPositionMode::kAuto, PortPlacementSourceKind::kTemplateSlot);
        add_unique_id(add_port_result.change_set.updated_ids, created->id);
      }
      if (out_slot_id != nullptr) {
        *out_slot_id = best_slot->slot_id;
      }
      debug.selected_slot_id = best_slot->slot_id;
      debug.result = "created slot port";
      push_debug();
      return add_port_result;
    }
  }

  ObjectId fallback = kInvalidObjectId;
  std::size_t fallback_usage = std::numeric_limits<std::size_t>::max();
  for (const Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id != request.pole_id || port.category != request.category) {
      continue;
    }
    const std::size_t usage = connection_count(port.id);
    if (usage < fallback_usage) {
      fallback = port.id;
      fallback_usage = usage;
    }
  }
  if (fallback != kInvalidObjectId) {
    result.ok = true;
    result.value = fallback;
    if (out_slot_id != nullptr) {
      const Port* port = edit_state_.ports.find(fallback);
      *out_slot_id = (port == nullptr) ? -1 : port->source_slot_id;
    }
    debug.selected_slot_id = (out_slot_id == nullptr) ? -1 : *out_slot_id;
    debug.result = "fallback existing category port";
    push_debug();
    return result;
  }

  if (request.allow_generate_port) {
    const Vec3d local = apply_pole_clearance_to_local(
        *pole, Vec3d{0.0, 0.0, std::max(1.0, pole->height_m * 0.7)}, SlotSide::kCenter);
    const Vec3d world_position =
        local_to_world_on_pole(pole->world_transform, effective_pole_yaw_deg(*pole), local);
    EditResult<ObjectId> add_port_result =
        AddPort(request.pole_id, world_position, category_to_port_kind(request.category),
                category_to_port_layer(request.category));
    if (!add_port_result.ok) {
      debug.result = "fallback generated port failed: " + add_port_result.error;
      push_debug();
      return add_port_result;
    }
    Port* created = edit_state_.ports.find(add_port_result.value);
    if (created != nullptr) {
      created->category = request.category;
      created->generated_from_template = true;
      created->generated_by_rule = true;
      created->placement_context = request.connection_context;
      created->source_slot_id = -1;
      apply_port_position_mode(*created, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
      add_unique_id(add_port_result.change_set.updated_ids, created->id);
    }
    if (out_slot_id != nullptr) {
      *out_slot_id = -1;
    }
    debug.selected_slot_id = -1;
    debug.result = "fallback generated category port";
    push_debug();
    return add_port_result;
  }

  std::ostringstream oss;
  oss << "no port available for pole " << request.pole_id << " category " << static_cast<int>(request.category);
  result.error = oss.str();
  debug.result = result.error;
  push_debug();
  return result;
}

EditResult<ObjectId> CoreState::ensure_bundle_for_template(const AddConnectionByPoleOptions& options) {
  EditResult<ObjectId> result;
  if (options.bundle_id != kInvalidObjectId) {
    const Bundle* existing_bundle = edit_state_.bundles.find(options.bundle_id);
    if (existing_bundle == nullptr) {
      result.error = "bundle does not exist";
      return result;
    }
    if (options.use_bundle_template && existing_bundle->bundle_template_id != options.bundle_template_id) {
      result.error = "bundle_id kind and bundle_template_id mismatch";
      return result;
    }
    result.ok = true;
    result.value = options.bundle_id;
    return result;
  }

  if (!options.auto_create_bundle) {
    if (options.use_bundle_template) {
      result.error = "bundle_id is required when use_bundle_template=true and auto_create_bundle=false";
      return result;
    }
    result.ok = true;
    result.value = kInvalidObjectId;
    return result;
  }

  if (!options.use_bundle_template) {
    result.error = "bundle_template_id is required when auto_create_bundle is enabled";
    return result;
  }
  const BundleTemplate* bundle_template = find_bundle_template(options.bundle_template_id);
  if (bundle_template == nullptr) {
    result.error = "bundle template not found";
    return result;
  }
  const int conductor_count = (bundle_template->count_rule == BundleCountRuleKind::kFixed)
                                  ? bundle_template->fixed_count
                                  : bundle_template->default_count;
  if (conductor_count <= 0) {
    result.error = "bundle template resolved invalid conductor count";
    return result;
  }
  return AddBundle(conductor_count, std::max(0.01, bundle_template->default_spacing_m), bundle_template->id);
}

std::uint8_t CoreState::deterministic_tiebreak_0_255(ObjectId pole_id, int slot_id, ConnectionCategory category,
                                                     ConnectionContext context, ObjectId peer_pole_id,
                                                     ObjectId reference_span_id, std::uint32_t branch_index) {
  auto mix = [](std::uint32_t h, std::uint64_t v) -> std::uint32_t {
    h ^= static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
    h *= 16777619u;
    h ^= static_cast<std::uint32_t>((v >> 32) & 0xFFFFFFFFu);
    h *= 16777619u;
    return h;
  };

  std::uint32_t h = 2166136261u;
  h = mix(h, pole_id);
  h = mix(h, static_cast<std::uint64_t>(slot_id));
  h = mix(h, static_cast<std::uint64_t>(static_cast<std::uint8_t>(category)));
  h = mix(h, static_cast<std::uint64_t>(static_cast<std::uint8_t>(context)));
  h = mix(h, peer_pole_id);
  h = mix(h, reference_span_id);
  h = mix(h, branch_index);
  return static_cast<std::uint8_t>(h & 0xFFu);
}

bool CoreState::is_valid_slot_side(SlotSide side) {
  return side == SlotSide::kLeft || side == SlotSide::kCenter || side == SlotSide::kRight;
}

bool CoreState::is_valid_slot_role(SlotRole role) {
  return role == SlotRole::kNeutral || role == SlotRole::kTrunkPreferred || role == SlotRole::kBranchPreferred ||
         role == SlotRole::kDropPreferred;
}

void CoreState::register_default_cable_templates() {
  CableTemplate hv{};
  hv.id = kHighVoltageCableTemplate;
  hv.name = "HV_BARE";
  hv.outer_diameter_m = 0.030;
  hv.bend_stiffness = 1.4;
  hv.min_bend_radius_m = 0.8;
  hv.material_style = CableMaterialStyleKind::kBareConductor;
  hv.color_rgba = 0xBFC7CFFFu;
  hv.requires_insulator = true;
  hv.sag_factor = 0.03;
  hv.slack_factor = 0.0;
  hv.continuity_policy = CableContinuityPolicyHint::kPreferG1;
  hv.attachment_style = CableAttachmentStyleHint::kViaAttachment;
  cable_templates_[hv.id] = hv;

  CableTemplate lv{};
  lv.id = kLowVoltageCableTemplate;
  lv.name = "LV_INSULATED";
  lv.outer_diameter_m = 0.020;
  lv.bend_stiffness = 0.9;
  lv.min_bend_radius_m = 0.25;
  lv.material_style = CableMaterialStyleKind::kInsulated;
  lv.color_rgba = 0x2E2E2EFFu;
  lv.requires_insulator = true;
  lv.sag_factor = 0.03;
  lv.slack_factor = 0.0;
  lv.continuity_policy = CableContinuityPolicyHint::kAuto;
  lv.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  cable_templates_[lv.id] = lv;

  CableTemplate comm{};
  comm.id = kCommunicationCableTemplate;
  comm.name = "COMM_MULTI";
  comm.outer_diameter_m = 0.014;
  comm.bend_stiffness = 0.6;
  comm.min_bend_radius_m = 0.18;
  comm.material_style = CableMaterialStyleKind::kInsulated;
  comm.color_rgba = 0x5D5D5DFFu;
  comm.requires_insulator = false;
  comm.sag_factor = 0.025;
  comm.slack_factor = 0.02;
  comm.continuity_policy = CableContinuityPolicyHint::kPreferG2;
  comm.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  cable_templates_[comm.id] = comm;

  CableTemplate optical{};
  optical.id = kOpticalCableTemplate;
  optical.name = "OPTICAL_FIBER";
  optical.outer_diameter_m = 0.012;
  optical.bend_stiffness = 0.5;
  optical.min_bend_radius_m = 0.20;
  optical.material_style = CableMaterialStyleKind::kOptical;
  optical.color_rgba = 0x6EC9D8FFu;
  optical.requires_insulator = false;
  optical.sag_factor = 0.02;
  optical.slack_factor = 0.03;
  optical.continuity_policy = CableContinuityPolicyHint::kPreferG2;
  optical.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  cable_templates_[optical.id] = optical;
}

void CoreState::register_default_attachment_templates() {
  AttachmentTemplate generic{};
  generic.id = kGenericAttachmentTemplate;
  generic.name = "GENERIC_PASS_THROUGH";
  generic.kind = AttachmentKind::kGeneric;
  generic.line_interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  generic.sockets = {
      AttachmentSocketTemplate{0, {-0.12, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.12, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  attachment_templates_[generic.id] = generic;

  AttachmentTemplate hidden{};
  hidden.id = kHiddenAttachmentTemplate;
  hidden.name = "INLINE_HIDE_SEGMENT";
  hidden.kind = AttachmentKind::kMarker;
  hidden.line_interaction_mode = AttachmentLineInteractionMode::kHideSegment;
  hidden.sockets = {
      AttachmentSocketTemplate{0, {-0.10, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.10, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  attachment_templates_[hidden.id] = hidden;

  AttachmentTemplate internal{};
  internal.id = kInternalPathAttachmentTemplate;
  internal.name = "INLINE_INTERNAL_PATH";
  internal.kind = AttachmentKind::kSpacer;
  internal.line_interaction_mode = AttachmentLineInteractionMode::kReplaceWithInternalPath;
  internal.sockets = {
      AttachmentSocketTemplate{0, {-0.14, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.14, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  AttachmentInternalPathTemplate path{};
  path.start_socket_id = 0;
  path.end_socket_id = 1;
  path.local_points = {{-0.05, 0.0, 0.10}, {0.05, 0.0, 0.10}};
  internal.internal_paths.push_back(path);
  attachment_templates_[internal.id] = internal;
}

const CableTemplate* CoreState::find_cable_template(CableTemplateId cable_template_id) const {
  auto it = cable_templates_.find(cable_template_id);
  if (it == cable_templates_.end()) {
    return nullptr;
  }
  return &it->second;
}

const AttachmentTemplate* CoreState::find_attachment_template(AttachmentTemplateId attachment_template_id) const {
  auto it = attachment_templates_.find(attachment_template_id);
  if (it == attachment_templates_.end()) {
    return nullptr;
  }
  return &it->second;
}

} // namespace wire::core
