#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "../generation/support_policy.hpp"

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

double legacy_effective_pole_yaw_for_layout(const Pole& pole) {
  double yaw = pole.world_transform.rotation_euler_deg.z;
  if (pole.orientation_control.manual_yaw_override) {
    yaw = pole.orientation_control.manual_yaw_deg;
  }
  if (pole.orientation_control.flip_180) {
    yaw += 180.0;
  }
  return yaw;
}

bool attachment_socket_equals(const AttachmentSocketTemplate& a, const AttachmentSocketTemplate& b) {
  const auto same_vec = [](const Vec3d& lhs, const Vec3d& rhs) {
    return std::abs(lhs.x - rhs.x) <= 1e-12 && std::abs(lhs.y - rhs.y) <= 1e-12 && std::abs(lhs.z - rhs.z) <= 1e-12;
  };
  return a.id == b.id && same_vec(a.local_position, b.local_position) && same_vec(a.tangent_dir, b.tangent_dir) &&
         a.has_normal == b.has_normal && same_vec(a.normal_dir, b.normal_dir) &&
         a.has_binormal == b.has_binormal && same_vec(a.binormal_dir, b.binormal_dir) && a.kind == b.kind;
}

bool attachment_internal_path_equals(const AttachmentInternalPathTemplate& a, const AttachmentInternalPathTemplate& b) {
  if (a.start_socket_id != b.start_socket_id || a.end_socket_id != b.end_socket_id ||
      a.local_points.size() != b.local_points.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.local_points.size(); ++i) {
    const Vec3d& lhs = a.local_points[i];
    const Vec3d& rhs = b.local_points[i];
    if (std::abs(lhs.x - rhs.x) > 1e-12 || std::abs(lhs.y - rhs.y) > 1e-12 || std::abs(lhs.z - rhs.z) > 1e-12) {
      return false;
    }
  }
  return true;
}

bool attachment_template_equals(const AttachmentTemplate& a, const AttachmentTemplate& b) {
  if (a.id != b.id || a.name != b.name || a.kind != b.kind || a.line_interaction_mode != b.line_interaction_mode ||
      a.sockets.size() != b.sockets.size() || a.internal_paths.size() != b.internal_paths.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.sockets.size(); ++i) {
    if (!attachment_socket_equals(a.sockets[i], b.sockets[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.internal_paths.size(); ++i) {
    if (!attachment_internal_path_equals(a.internal_paths[i], b.internal_paths[i])) {
      return false;
    }
  }
  return true;
}

} // namespace

CoreState::CoreState() {
  register_default_pole_types();
  register_default_cable_templates();
  register_default_bundle_templates();
  register_default_attachment_templates();
}

EditResult<ObjectId> CoreState::AddPole(const Transformd& world_transform, double height_m, std::string_view name,
                                        PoleKind kind, PlacementMode placement_mode) {
  EditResult<ObjectId> result;
  if (height_m <= 0.0) {
    result.error = "pole height must be > 0";
    return result;
  }

  Pole pole{};
  pole.id = id_generator_.next();
  pole.display_id = next_display_id("P");
  pole.name = std::string(name);
  pole.world_transform = world_transform;
  pole.height_m = height_m;
  pole.kind = kind;
  pole.pole_type_id = kInvalidPoleTypeId;
  apply_pole_placement_mode(pole, placement_mode);
  edit_state_.poles.insert(pole);

  result.ok = true;
  result.value = pole.id;
  result.change_set.created_ids.push_back(pole.id);
  return result;
}

EditResult<ObjectId> CoreState::AddPort(ObjectId owner_pole_id, const Vec3d& world_position, PortKind kind,
                                        PortLayer layer, const Frame3d& direction) {
  EditResult<ObjectId> result;
  if (owner_pole_id != kInvalidObjectId && edit_state_.poles.find(owner_pole_id) == nullptr) {
    result.error = "owner pole does not exist";
    return result;
  }

  Port port{};
  port.id = id_generator_.next();
  port.display_id = next_display_id("PT");
  port.owner_pole_id = owner_pole_id;
  port.world_position = world_position;
  port.kind = kind;
  port.layer = layer;
  port.direction = direction;
  port.category = port_layer_to_category(layer);
  port.source_slot_id = -1;
  port.template_layer = generation::detail::TemplateLayerForCategory(port.category);
  port.template_side = SlotSide::kCenter;
  port.template_role = SlotRole::kNeutral;
  port.generated_from_template = false;
  port.generated_by_rule = false;
  port.placement_context = ConnectionContext::kTrunkContinue;
  port.angle_correction_applied = false;
  port.side_scale_applied = 1.0;
  apply_port_position_mode(port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  edit_state_.ports.insert(port);
  if (owner_pole_id != kInvalidObjectId) {
    index_add(relation_index_.ports_by_pole, owner_pole_id, port.id);
  }

  result.ok = true;
  result.value = port.id;
  result.change_set.created_ids.push_back(port.id);
  return result;
}

EditResult<ObjectId> CoreState::AddAnchor(ObjectId owner_pole_id, const Vec3d& world_position,
                                          AnchorSupportKind support_kind, double support_strength) {
  EditResult<ObjectId> result;
  if (owner_pole_id != kInvalidObjectId && edit_state_.poles.find(owner_pole_id) == nullptr) {
    result.error = "owner pole does not exist";
    return result;
  }
  if (support_strength <= 0.0) {
    result.error = "support strength must be > 0";
    return result;
  }

  Anchor anchor{};
  anchor.id = id_generator_.next();
  anchor.display_id = next_display_id("A");
  anchor.owner_pole_id = owner_pole_id;
  anchor.world_position = world_position;
  anchor.support_kind = support_kind;
  anchor.support_strength = support_strength;
  edit_state_.anchors.insert(anchor);
  if (owner_pole_id != kInvalidObjectId) {
    index_add(relation_index_.anchors_by_pole, owner_pole_id, anchor.id);
  }

  result.ok = true;
  result.value = anchor.id;
  result.change_set.created_ids.push_back(anchor.id);
  return result;
}

EditResult<ObjectId> CoreState::AddBundle(int conductor_count, double phase_spacing_m, BundleKind kind) {
  EditResult<ObjectId> result;
  if (conductor_count <= 0) {
    result.error = "conductor count must be > 0";
    return result;
  }
  if (phase_spacing_m <= 0.0) {
    result.error = "phase spacing must be > 0";
    return result;
  }

  Bundle bundle{};
  bundle.id = id_generator_.next();
  bundle.display_id = next_display_id("B");
  bundle.conductor_count = conductor_count;
  bundle.phase_spacing_m = phase_spacing_m;
  bundle.bundle_template_id = kind;
  edit_state_.bundles.insert(bundle);

  result.ok = true;
  result.value = bundle.id;
  result.change_set.created_ids.push_back(bundle.id);
  return result;
}


EditResult<ObjectId> CoreState::AddSpan(ObjectId port_a_id, ObjectId port_b_id, SpanKind kind, SpanLayer layer,
                                        ObjectId bundle_id, ObjectId anchor_a_id, ObjectId anchor_b_id) {
  EditResult<ObjectId> result;
  const Port* port_a = edit_state_.ports.find(port_a_id);
  const Port* port_b = edit_state_.ports.find(port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    result.error = "span ports do not exist";
    return result;
  }
  if (port_a_id == port_b_id) {
    result.error = "span endpoints must be different";
    return result;
  }
  if (has_zero_length(*port_a, *port_b)) {
    result.error = "zero-length span is not allowed";
    return result;
  }
  if (bundle_id != kInvalidObjectId && edit_state_.bundles.find(bundle_id) == nullptr) {
    result.error = "bundle does not exist";
    return result;
  }
  if (anchor_a_id != kInvalidObjectId && edit_state_.anchors.find(anchor_a_id) == nullptr) {
    result.error = "anchor_a does not exist";
    return result;
  }
  if (anchor_b_id != kInvalidObjectId && edit_state_.anchors.find(anchor_b_id) == nullptr) {
    result.error = "anchor_b does not exist";
    return result;
  }

  Span span{};
  span.id = id_generator_.next();
  span.display_id = next_display_id("SP");
  span.port_a_id = port_a_id;
  span.port_b_id = port_b_id;
  span.kind = kind;
  span.layer = layer;
  span.bundle_id = bundle_id;
  span.anchor_a_id = anchor_a_id;
  span.anchor_b_id = anchor_b_id;
  const double dx = port_b->world_position.x - port_a->world_position.x;
  const double dy = port_b->world_position.y - port_a->world_position.y;
  const double dz = port_b->world_position.z - port_a->world_position.z;
  span.reference_length_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  edit_state_.spans.insert(span);

  add_span_to_index(span);
  initialize_span_runtime_state(span.id);
  mark_span_dirty(span.id, DirtyBits::kTopology | DirtyBits::kGeometry, true);

  result.ok = true;
  result.value = span.id;
  result.change_set.created_ids.push_back(span.id);
  result.change_set.dirty_span_ids.push_back(span.id);
  return result;
}

EditResult<ObjectId> CoreState::AddAttachment(ObjectId span_id, double t, AttachmentKind kind, double display_offset_m,
                                              AttachmentTemplateId template_id) {
  EditResult<ObjectId> result;
  if (edit_state_.spans.find(span_id) == nullptr) {
    result.error = "span does not exist";
    return result;
  }
  if (t < 0.0 || t > 1.0) {
    result.error = "attachment t must be in [0, 1]";
    return result;
  }
  if (template_id == kInvalidAttachmentTemplateId) {
    for (const auto& [candidate_id, attachment_template] : attachment_templates_) {
      if (attachment_template.kind == kind) {
        template_id = candidate_id;
        break;
      }
    }
  }
  if (find_attachment_template(template_id) == nullptr) {
    result.error = "attachment template not found";
    return result;
  }

  Attachment attachment{};
  attachment.id = id_generator_.next();
  attachment.display_id = next_display_id("AT");
  attachment.span_id = span_id;
  attachment.template_id = template_id;
  attachment.t = t;
  attachment.kind = kind;
  attachment.display_offset_m = display_offset_m;
  edit_state_.attachments.insert(attachment);
  index_add(relation_index_.attachments_by_span, span_id, attachment.id);
  mark_span_dirty(span_id, DirtyBits::kGeometry | DirtyBits::kRender, true);

  result.ok = true;
  result.value = attachment.id;
  result.change_set.created_ids.push_back(attachment.id);
  result.change_set.updated_ids.push_back(span_id);
  result.change_set.dirty_span_ids.push_back(span_id);
  return result;
}


EditResult<ObjectId> CoreState::MovePole(ObjectId pole_id, const Transformd& new_world_transform) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }

  const Pole old_pole = *pole;
  pole->world_transform = new_world_transform;
  apply_pole_placement_mode(*pole, PlacementMode::kManual);
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<bool> CoreState::ApplyPoleTilt(const std::vector<ObjectId>& pole_ids, double tilt_x_deg, double tilt_y_deg) {
  EditResult<bool> result;
  std::vector<ObjectId> targets = pole_ids;
  if (targets.empty()) {
    targets.reserve(edit_state_.poles.size());
    for (const Pole& pole : edit_state_.poles.items()) {
      targets.push_back(pole.id);
    }
  }
  for (ObjectId pole_id : targets) {
    Pole* pole = edit_state_.poles.find(pole_id);
    if (pole == nullptr) {
      result.error = "pole not found";
      result.ok = false;
      return result;
    }
    const Pole old_pole = *pole;
    if (std::abs(pole->world_transform.rotation_euler_deg.x - tilt_x_deg) <= 1e-9 &&
        std::abs(pole->world_transform.rotation_euler_deg.y - tilt_y_deg) <= 1e-9) {
      continue;
    }
    pole->world_transform.rotation_euler_deg.x = tilt_x_deg;
    pole->world_transform.rotation_euler_deg.y = tilt_y_deg;
    finalize_pole_transform_update(pole->id, old_pole, &result.change_set);
  }
  result.ok = true;
  result.value = true;
  return result;
}

EditResult<ObjectId> CoreState::SetPoleTilt(ObjectId pole_id, double tilt_x_deg, double tilt_y_deg) {
  EditResult<ObjectId> result;
  const auto apply = ApplyPoleTilt({pole_id}, tilt_x_deg, tilt_y_deg);
  result.ok = apply.ok;
  result.error = apply.error;
  result.change_set = apply.change_set;
  result.value = pole_id;
  return result;
}

EditResult<bool> CoreState::SetAllPoleTilt(double tilt_x_deg, double tilt_y_deg) {
  return ApplyPoleTilt({}, tilt_x_deg, tilt_y_deg);
}

EditResult<ObjectId> CoreState::MovePort(ObjectId port_id, const Vec3d& new_world_position) {
  return SetPortWorldPositionManual(port_id, new_world_position);
}

EditResult<ObjectId> CoreState::SetPortWorldPositionManual(ObjectId port_id, const Vec3d& new_world_position) {
  EditResult<ObjectId> result;
  Port* port = edit_state_.ports.find(port_id);
  if (port == nullptr) {
    result.error = "port not found";
    return result;
  }
  port->world_position = new_world_position;
  apply_port_position_mode(*port, PortPositionMode::kManual, PortPlacementSourceKind::kManualEdit);
  result.change_set.updated_ids.push_back(port_id);
  mark_connected_spans_dirty_from_port(port_id, DirtyBits::kGeometry, &result.change_set);
  result.ok = true;
  result.value = port_id;
  return result;
}

EditResult<ObjectId> CoreState::ResetPortPositionToAuto(ObjectId port_id) {
  EditResult<ObjectId> result;
  Port* port = edit_state_.ports.find(port_id);
  if (port == nullptr) {
    result.error = "port not found";
    return result;
  }

  apply_port_position_mode(*port, PortPositionMode::kAuto, port->placement_source);

  bool recomputed = false;
  if (port->owner_pole_id != kInvalidObjectId && port->source_slot_id >= 0) {
    const Pole* pole = edit_state_.poles.find(port->owner_pole_id);
    if (pole != nullptr) {
      const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id);
      if (pole_type != nullptr) {
        const PortSlotTemplate* slot_ptr = nullptr;
        for (const PortSlotTemplate& slot : pole_type->port_slots) {
          if (slot.slot_id == port->source_slot_id) {
            slot_ptr = &slot;
            break;
          }
        }
        if (slot_ptr != nullptr) {
          Vec3d adjusted_local = slot_ptr->local_position;
          const bool apply_angle_correction = layout_settings_.angle_correction_enabled &&
                                              pole->context.kind == PoleContextKind::kCorner &&
                                              slot_ptr->side != SlotSide::kCenter;
          double applied_scale = 1.0;
          if (apply_angle_correction) {
            adjusted_local.y = apply_corner_side_scale(adjusted_local.y, slot_ptr->side, pole->context.corner_turn_sign,
                                                       pole->context.side_scale);
            if (std::abs(slot_ptr->local_position.y) > 1e-9) {
              applied_scale = std::abs(adjusted_local.y / slot_ptr->local_position.y);
            }
          }
          adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, slot_ptr->side);
          port->world_position =
              local_to_world_on_pole(pole->world_transform, effective_pole_yaw_deg(*pole), adjusted_local);
          port->angle_correction_applied = apply_angle_correction;
          port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
          apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kTemplateSlot);
          recomputed = true;
        }
      }
    }
  }
  if (!recomputed && port->placement_source == PortPlacementSourceKind::kManualEdit) {
    apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  }

  result.change_set.updated_ids.push_back(port_id);
  mark_connected_spans_dirty_from_port(port_id, DirtyBits::kGeometry, &result.change_set);
  result.ok = true;
  result.value = port_id;
  return result;
}

EditResult<ObjectId> CoreState::MoveAnchor(ObjectId anchor_id, const Vec3d& new_world_position) {
  EditResult<ObjectId> result;
  Anchor* anchor = edit_state_.anchors.find(anchor_id);
  if (anchor == nullptr) {
    result.error = "anchor not found";
    return result;
  }
  anchor->world_position = new_world_position;
  result.change_set.updated_ids.push_back(anchor_id);
  mark_connected_spans_dirty_from_anchor(anchor_id, DirtyBits::kGeometry, &result.change_set);
  result.ok = true;
  result.value = anchor_id;
  return result;
}

EditResult<ObjectId> CoreState::SetPoleFlip180(ObjectId pole_id, bool flip_180) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  PoleOrientationOverride next = override_state_.pole_orientation_by_pole[pole_id];
  if (next.flip_180.has_value() && *next.flip_180 == flip_180) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  next.flip_180 = flip_180;
  override_state_.pole_orientation_by_pole[pole_id] = next;
  pole->orientation_control.flip_180 = flip_180;
  pole->orientation_override_flag = true;
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::SetPoleManualYawOverride(ObjectId pole_id, double manual_yaw_deg) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  if (!std::isfinite(manual_yaw_deg)) {
    result.error = "manual yaw must be finite";
    return result;
  }

  PoleOrientationOverride next = override_state_.pole_orientation_by_pole[pole_id];
  if (next.manual_yaw_deg.has_value() && std::abs(*next.manual_yaw_deg - manual_yaw_deg) <= 1e-9) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  next.manual_yaw_deg = normalize_yaw_deg(manual_yaw_deg);
  override_state_.pole_orientation_by_pole[pole_id] = next;
  pole->orientation_control.manual_yaw_override = true;
  pole->orientation_control.manual_yaw_deg = *next.manual_yaw_deg;
  pole->orientation_override_flag = true;
  pole->world_transform.rotation_euler_deg.z = *next.manual_yaw_deg;
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::ClearPoleOrientationOverride(ObjectId pole_id) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }

  const bool had_override = override_state_.pole_orientation_by_pole.erase(pole_id) > 0 ||
                            pole->orientation_control.manual_yaw_override || pole->orientation_control.flip_180 ||
                            pole->orientation_override_flag;
  if (!had_override) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  pole->orientation_control.manual_yaw_override = false;
  pole->orientation_control.manual_yaw_deg = 0.0;
  pole->orientation_control.flip_180 = false;
  pole->orientation_override_flag = false;
  if (const auto it = pole_orientation_debug_records_.find(pole_id); it != pole_orientation_debug_records_.end()) {
    const Vec3d forward = it->second.adopted_forward;
    if ((forward.x * forward.x + forward.y * forward.y + forward.z * forward.z) > 1e-12) {
      pole->world_transform.rotation_euler_deg.z =
          normalize_yaw_deg(std::atan2(forward.y, forward.x) * (180.0 / 3.14159265358979323846));
    }
  }
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::SetSpanEndpointSocketOverride(ObjectId span_id, bool is_start_endpoint, int socket_id) {
  EditResult<ObjectId> result;
  Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  SpanEndpointOverride next = override_state_.span_endpoint_by_span[span_id];
  std::optional<int>& slot = is_start_endpoint ? next.socket_a_id : next.socket_b_id;
  if (slot.has_value() && *slot == socket_id) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  slot = socket_id;
  override_state_.span_endpoint_by_span[span_id] = next;
  if (is_start_endpoint) {
    span->endpoint_socket_a_id = socket_id;
  } else {
    span->endpoint_socket_b_id = socket_id;
  }
  span->orientation_override_flag = true;
  mark_span_dirty(span_id, DirtyBits::kGeometry, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::ClearSpanEndpointSocketOverride(ObjectId span_id, bool is_start_endpoint) {
  EditResult<ObjectId> result;
  Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  auto it = override_state_.span_endpoint_by_span.find(span_id);
  bool changed = false;
  if (it != override_state_.span_endpoint_by_span.end()) {
    std::optional<int>& slot = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    changed = slot.has_value();
    slot.reset();
    if (!it->second.socket_a_id.has_value() && !it->second.socket_b_id.has_value()) {
      override_state_.span_endpoint_by_span.erase(it);
    }
  }
  if (is_start_endpoint) {
    changed = changed || span->endpoint_socket_a_id >= 0;
    span->endpoint_socket_a_id = -1;
  } else {
    changed = changed || span->endpoint_socket_b_id >= 0;
    span->endpoint_socket_b_id = -1;
  }
  if (!changed) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  mark_span_dirty(span_id, DirtyBits::kGeometry, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::SetSpanBranchDownOffsetOverride(ObjectId span_id, double branch_down_offset_m) {
  EditResult<ObjectId> result;
  Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  if (!std::isfinite(branch_down_offset_m) || branch_down_offset_m < 0.0) {
    result.error = "branch down offset override must be finite and >= 0";
    return result;
  }
  SpanSupportOverride next = override_state_.span_support_by_span[span_id];
  if (next.branch_down_offset_m.has_value() && std::abs(*next.branch_down_offset_m - branch_down_offset_m) <= 1e-9) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  next.branch_down_offset_m = branch_down_offset_m;
  override_state_.span_support_by_span[span_id] = next;
  span->orientation_override_flag = true;
  mark_span_dirty(span_id, DirtyBits::kGeometry | DirtyBits::kRender, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::ClearSpanBranchDownOffsetOverride(ObjectId span_id) {
  EditResult<ObjectId> result;
  Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  if (override_state_.span_support_by_span.erase(span_id) == 0) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  mark_span_dirty(span_id, DirtyBits::kGeometry | DirtyBits::kRender, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::SetPolePlacementMode(ObjectId pole_id, PlacementMode mode) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  if (pole->placement_mode == mode) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }
  apply_pole_placement_mode(*pole, mode);
  add_unique_id(result.change_set.updated_ids, pole_id);
  result.ok = true;
  result.value = pole_id;
  return result;
}

void CoreState::finalize_pole_transform_update(ObjectId pole_id, const Pole& old_pole, ChangeSet* change_set) {
  if (change_set != nullptr) {
    add_unique_id(change_set->updated_ids, pole_id);
  }
  refresh_owned_endpoints_from_pole(pole_id, change_set, &old_pole);
}

void CoreState::refresh_owned_endpoints_from_pole(ObjectId pole_id, ChangeSet* change_set, const Pole* previous_pole) {
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    return;
  }

  const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id);
  auto find_port_slot = [&](int slot_id) -> const PortSlotTemplate* {
    if (pole_type == nullptr) {
      return nullptr;
    }
    for (const PortSlotTemplate& slot : pole_type->port_slots) {
      if (slot.slot_id == slot_id) {
        return &slot;
      }
    }
    return nullptr;
  };
  auto find_anchor_slot = [&](int slot_id) -> const AnchorSlotTemplate* {
    if (pole_type == nullptr) {
      return nullptr;
    }
    for (const AnchorSlotTemplate& slot : pole_type->anchor_slots) {
      if (slot.slot_id == slot_id) {
        return &slot;
      }
    }
    return nullptr;
  };

  const double effective_yaw = effective_pole_yaw_deg(*pole);

  for (Port& port : edit_state_.ports.items_mutable()) {
    if (port.owner_pole_id != pole_id || port.position_mode == PortPositionMode::kManual) {
      continue;
    }
    const PortSlotTemplate* slot = find_port_slot(port.source_slot_id);
    Vec3d new_world = port.world_position;
    bool apply_angle_correction = false;
    double applied_scale = 1.0;
    if (slot != nullptr) {
      Vec3d adjusted_local = slot->local_position;
      apply_angle_correction = layout_settings_.angle_correction_enabled &&
                               pole->context.kind == PoleContextKind::kCorner &&
                               slot->side != SlotSide::kCenter;
      if (apply_angle_correction) {
        adjusted_local.y = apply_corner_side_scale(adjusted_local.y, slot->side, pole->context.corner_turn_sign,
                                                   pole->context.side_scale);
        if (std::abs(slot->local_position.y) > 1e-9) {
          applied_scale = std::abs(adjusted_local.y / slot->local_position.y);
        }
      }
      adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, slot->side);
      new_world = local_to_world_on_pole(pole->world_transform, effective_yaw, adjusted_local);
    } else if (previous_pole != nullptr) {
      const Vec3d old_local = to_local_on_pole(*previous_pole, port.world_position);
      new_world = local_to_world_on_pole(pole->world_transform, effective_yaw, old_local);
    }
    const bool moved = std::abs(new_world.x - port.world_position.x) > 1e-9 ||
                       std::abs(new_world.y - port.world_position.y) > 1e-9 ||
                       std::abs(new_world.z - port.world_position.z) > 1e-9;
    const bool changed_scale = std::abs(port.side_scale_applied - (apply_angle_correction ? applied_scale : 1.0)) > 1e-9;
    const bool changed_angle_flag = port.angle_correction_applied != apply_angle_correction;
    if (!moved && !changed_scale && !changed_angle_flag) {
      continue;
    }

    port.world_position = new_world;
    port.angle_correction_applied = apply_angle_correction;
    port.side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
    apply_port_position_mode(port, PortPositionMode::kAuto, PortPlacementSourceKind::kTemplateSlot);
    if (change_set != nullptr) {
      add_unique_id(change_set->updated_ids, port.id);
      mark_connected_spans_dirty_from_port(port.id, DirtyBits::kGeometry, change_set);
    }
  }

  for (Anchor& anchor : edit_state_.anchors.items_mutable()) {
    if (anchor.owner_pole_id != pole_id) {
      continue;
    }
    const AnchorSlotTemplate* slot = find_anchor_slot(anchor.source_slot_id);
    Vec3d new_world = anchor.world_position;
    if (slot != nullptr) {
      new_world = local_to_world_on_pole(pole->world_transform, effective_yaw, slot->local_position);
    } else if (previous_pole != nullptr) {
      const Vec3d old_local = to_local_on_pole(*previous_pole, anchor.world_position);
      new_world = local_to_world_on_pole(pole->world_transform, effective_yaw, old_local);
    }
    const bool moved = std::abs(new_world.x - anchor.world_position.x) > 1e-9 ||
                       std::abs(new_world.y - anchor.world_position.y) > 1e-9 ||
                       std::abs(new_world.z - anchor.world_position.z) > 1e-9;
    if (!moved) {
      continue;
    }

    anchor.world_position = new_world;
    if (change_set != nullptr) {
      add_unique_id(change_set->updated_ids, anchor.id);
      mark_connected_spans_dirty_from_anchor(anchor.id, DirtyBits::kGeometry, change_set);
    }
  }
}

EditResult<ObjectId> CoreState::DeleteSpan(ObjectId span_id) {
  EditResult<ObjectId> result;
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }

  const Span copy = *span;
  remove_span_from_indexes(copy);
  edit_state_.spans.remove(span_id);
  span_runtime_states_.erase(span_id);
  remove_span_from_caches(span_id);

  std::vector<ObjectId> remove_attachments{};
  if (const auto it = relation_index_.attachments_by_span.find(span_id); it != relation_index_.attachments_by_span.end()) {
    remove_attachments = it->second;
  }
  for (ObjectId attachment_id : remove_attachments) {
    edit_state_.attachments.remove(attachment_id);
    add_unique_id(result.change_set.deleted_ids, attachment_id);
  }
  relation_index_.attachments_by_span.erase(span_id);

  result.ok = true;
  result.value = span_id;
  result.change_set.deleted_ids.push_back(span_id);
  return result;
}

EditResult<CoreState::SplitSpanResult> CoreState::SplitSpan(ObjectId span_id, double t) {
  EditResult<SplitSpanResult> result;
  if (!(t > 0.0 && t < 1.0)) {
    result.error = "split t must be in (0, 1)";
    return result;
  }

  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  const Span old_span = *span;
  const Port* port_a = edit_state_.ports.find(old_span.port_a_id);
  const Port* port_b = edit_state_.ports.find(old_span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    result.error = "span endpoints are missing";
    return result;
  }

  const Vec3d split_pos{
      port_a->world_position.x + (port_b->world_position.x - port_a->world_position.x) * t,
      port_a->world_position.y + (port_b->world_position.y - port_a->world_position.y) * t,
      port_a->world_position.z + (port_b->world_position.z - port_a->world_position.z) * t,
  };
  const ConnectionCategory category = span_layer_to_category(old_span.layer);

  EditResult<ObjectId> add_port_result =
      AddPort(kInvalidObjectId, split_pos, category_to_port_kind(category), span_layer_to_port_layer(old_span.layer));
  if (!add_port_result.ok) {
    result.error = add_port_result.error;
    return result;
  }
  if (Port* split_port = edit_state_.ports.find(add_port_result.value); split_port != nullptr) {
    apply_port_position_mode(*split_port, PortPositionMode::kAuto, PortPlacementSourceKind::kAerialBranch);
    add_unique_id(add_port_result.change_set.updated_ids, split_port->id);
  }

  EditResult<ObjectId> add_span_a_result =
      AddSpan(old_span.port_a_id, add_port_result.value, old_span.kind, old_span.layer, old_span.bundle_id,
              old_span.anchor_a_id, kInvalidObjectId);
  if (!add_span_a_result.ok) {
    result.error = add_span_a_result.error;
    return result;
  }

  EditResult<ObjectId> add_span_b_result =
      AddSpan(add_port_result.value, old_span.port_b_id, old_span.kind, old_span.layer, old_span.bundle_id,
              kInvalidObjectId, old_span.anchor_b_id);
  if (!add_span_b_result.ok) {
    result.error = add_span_b_result.error;
    return result;
  }

  EditResult<ObjectId> delete_result = DeleteSpan(old_span.id);
  if (!delete_result.ok) {
    result.error = delete_result.error;
    return result;
  }

  result.ok = true;
  result.value.old_span_id = old_span.id;
  result.value.new_port_id = add_port_result.value;
  result.value.new_span_a_id = add_span_a_result.value;
  result.value.new_span_b_id = add_span_b_result.value;
  append_change_set(result.change_set, add_port_result.change_set);
  append_change_set(result.change_set, add_span_a_result.change_set);
  append_change_set(result.change_set, add_span_b_result.change_set);
  append_change_set(result.change_set, delete_result.change_set);
  return result;
}

EditResult<ObjectId> CoreState::ApplyPoleType(ObjectId pole_id, PoleTypeId pole_type_id) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  const PoleTypeDefinition* pole_type = find_pole_type(pole_type_id);
  if (pole_type == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  pole->pole_type_id = pole_type_id;
  result.change_set.updated_ids.push_back(pole_id);

  for (const PortSlotTemplate& slot : pole_type->port_slots) {
    if (!slot.enabled || is_port_slot_used(pole_id, slot.slot_id)) {
      continue;
    }
    Vec3d adjusted_local = slot.local_position;
    const bool apply_angle_correction = layout_settings_.angle_correction_enabled &&
                                        pole->context.kind == PoleContextKind::kCorner &&
                                        slot.side != SlotSide::kCenter;
    double applied_scale = 1.0;
    if (apply_angle_correction) {
      adjusted_local.y = apply_corner_side_scale(adjusted_local.y, slot.side, pole->context.corner_turn_sign,
                                                 pole->context.side_scale);
      if (std::abs(slot.local_position.y) > 1e-9) {
        applied_scale = std::abs(adjusted_local.y / slot.local_position.y);
      }
    }
    adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, slot.side);
    const Vec3d world_position =
        local_to_world_on_pole(pole->world_transform, effective_pole_yaw_deg(*pole), adjusted_local);
    EditResult<ObjectId> add_port_result = AddPort(pole_id, world_position, category_to_port_kind(slot.category),
                                                   category_to_port_layer(slot.category), slot.local_direction);
    if (!add_port_result.ok) {
      result.error = add_port_result.error;
      return result;
    }
    Port* created = edit_state_.ports.find(add_port_result.value);
    if (created != nullptr) {
      created->category = slot.category;
      created->source_slot_id = slot.slot_id;
      created->template_layer = slot.layer;
      created->template_side = slot.side;
      created->template_role = slot.role;
      created->generated_from_template = true;
      created->generated_by_rule = true;
      created->angle_correction_applied = apply_angle_correction;
      created->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
      apply_port_position_mode(*created, PortPositionMode::kAuto, PortPlacementSourceKind::kTemplateSlot);
      add_unique_id(result.change_set.updated_ids, created->id);
    }
    append_change_set(result.change_set, add_port_result.change_set);
  }

  for (const AnchorSlotTemplate& slot : pole_type->anchor_slots) {
    if (!slot.enabled) {
      continue;
    }
    bool exists = false;
    for (const Anchor& anchor : edit_state_.anchors.items()) {
      if (anchor.owner_pole_id == pole_id && anchor.source_slot_id == slot.slot_id) {
        exists = true;
        break;
      }
    }
    if (exists) {
      continue;
    }

    const Vec3d world_position =
        local_to_world_on_pole(pole->world_transform, effective_pole_yaw_deg(*pole), slot.local_position);
    EditResult<ObjectId> add_anchor_result = AddAnchor(pole_id, world_position, slot.usage, 1.0);
    if (!add_anchor_result.ok) {
      result.error = add_anchor_result.error;
      return result;
    }
    Anchor* created = edit_state_.anchors.find(add_anchor_result.value);
    if (created != nullptr) {
      created->source_slot_id = slot.slot_id;
      created->generated_from_template = true;
      add_unique_id(result.change_set.updated_ids, created->id);
    }
    append_change_set(result.change_set, add_anchor_result.change_set);
  }

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<CoreState::AddConnectionByPoleResult>
CoreState::AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id, ConnectionCategory category,
                               const AddConnectionByPoleOptions& options) {
  EditResult<AddConnectionByPoleResult> result;
  if (edit_state_.poles.find(pole_a_id) == nullptr || edit_state_.poles.find(pole_b_id) == nullptr) {
    result.error = "pole does not exist";
    return result;
  }
  if (pole_a_id == pole_b_id) {
    result.error = "cannot connect same pole";
    return result;
  }

  ConnectionCategory resolved_category = category;
  SpanLayer resolved_span_layer = category_to_span_layer(category);
  const BundleTemplate* resolved_bundle_template = nullptr;
  if (options.use_bundle_template) {
    resolved_bundle_template = find_bundle_template(options.bundle_template_id);
  } else if (options.bundle_id != kInvalidObjectId) {
    const Bundle* existing_bundle = edit_state_.bundles.find(options.bundle_id);
    if (existing_bundle != nullptr) {
      resolved_bundle_template = find_bundle_template(existing_bundle->bundle_template_id);
    }
  }
  if (resolved_bundle_template != nullptr) {
    resolved_category = resolved_bundle_template->category;
    resolved_span_layer = resolved_bundle_template->default_layer;
  }
  if (options.span_layer != SpanLayer::kUnknown) {
    if (resolved_bundle_template != nullptr && options.span_layer != resolved_bundle_template->default_layer) {
      result.error = "span_layer override conflicts with bundle template default layer";
      return result;
    }
    resolved_span_layer = options.span_layer;
  }

  int slot_a_id = -1;
  int slot_b_id = -1;

  const Pole* pole_a = edit_state_.poles.find(pole_a_id);
  const Pole* pole_b = edit_state_.poles.find(pole_b_id);
  const PoleContextKind pole_context_a = (options.pole_context_a != PoleContextKind::kStraight || (pole_a == nullptr))
                                             ? options.pole_context_a
                                             : pole_a->context.kind;
  const PoleContextKind pole_context_b = (options.pole_context_b != PoleContextKind::kStraight || (pole_b == nullptr))
                                             ? options.pole_context_b
                                             : pole_b->context.kind;
  const double corner_angle_a = (std::abs(options.corner_angle_deg_a) > 1e-9 || pole_a == nullptr)
                                    ? options.corner_angle_deg_a
                                    : pole_a->context.corner_angle_deg;
  const double corner_angle_b = (std::abs(options.corner_angle_deg_b) > 1e-9 || pole_b == nullptr)
                                    ? options.corner_angle_deg_b
                                    : pole_b->context.corner_angle_deg;
  const double corner_turn_sign_a = (std::abs(options.corner_turn_sign_a) > 1e-9 || pole_a == nullptr)
                                        ? options.corner_turn_sign_a
                                        : pole_a->context.corner_turn_sign;
  const double corner_turn_sign_b = (std::abs(options.corner_turn_sign_b) > 1e-9 || pole_b == nullptr)
                                        ? options.corner_turn_sign_b
                                        : pole_b->context.corner_turn_sign;

  auto resolve_port = [&](ObjectId pole_id, ObjectId preferred_port_id, int* out_slot_id,
                          int preferred_slot_id) -> EditResult<ObjectId> {
    if (preferred_port_id != kInvalidObjectId) {
      const Port* preferred_port = edit_state_.ports.find(preferred_port_id);
      if (preferred_port != nullptr && preferred_port->owner_pole_id == pole_id &&
          (preferred_port->category == resolved_category ||
           preferred_port->layer == category_to_port_layer(resolved_category))) {
        EditResult<ObjectId> preferred_result;
        preferred_result.ok = true;
        preferred_result.value = preferred_port_id;
        if (out_slot_id != nullptr) {
          *out_slot_id = preferred_port->source_slot_id;
        }
        return preferred_result;
      }
    }
    SlotSelectionRequest request{};
    request.pole_id = pole_id;
    request.peer_pole_id = (pole_id == pole_a_id) ? pole_b_id : pole_a_id;
    request.reference_span_id = options.reference_span_id;
    request.category = resolved_category;
    request.connection_context = options.connection_context;
    request.pole_context = (pole_id == pole_a_id) ? pole_context_a : pole_context_b;
    request.corner_angle_deg = (pole_id == pole_a_id) ? corner_angle_a : corner_angle_b;
    request.corner_turn_sign = (pole_id == pole_a_id) ? corner_turn_sign_a : corner_turn_sign_b;
    request.allow_generate_port = options.allow_generate_port;
    request.preferred_slot_id = preferred_slot_id;
    request.branch_index = options.branch_index;
    return ensure_pole_slot_port(request, out_slot_id);
  };

  EditResult<ObjectId> port_a_result = resolve_port(pole_a_id, options.preferred_port_a_id, &slot_a_id, -1);
  if (!port_a_result.ok) {
    result.error = port_a_result.error;
    return result;
  }
  EditResult<ObjectId> port_b_result = resolve_port(pole_b_id, options.preferred_port_b_id, &slot_b_id, slot_a_id);
  if (!port_b_result.ok) {
    result.error = port_b_result.error;
    return result;
  }

  EditResult<ObjectId> bundle_result = ensure_bundle_for_template(options);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }

  const SpanKind span_kind = (resolved_category == ConnectionCategory::kDrop) ? SpanKind::kService : options.span_kind;
  EditResult<ObjectId> span_result = AddSpan(port_a_result.value, port_b_result.value, span_kind,
                                             resolved_span_layer, bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
  }
  Span* created_span = edit_state_.spans.find(span_result.value);
  if (created_span != nullptr) {
    created_span->placement_context = options.connection_context;
    created_span->generated_by_rule = (created_span->generation.source == GenerationSource::kRoadAuto) ||
                                      options.connection_context != ConnectionContext::kTrunkContinue;
    add_unique_id(span_result.change_set.updated_ids, created_span->id);
  }

  result.ok = true;
  result.value.span_id = span_result.value;
  result.value.port_a_id = port_a_result.value;
  result.value.port_b_id = port_b_result.value;
  result.value.slot_a_id = slot_a_id;
  result.value.slot_b_id = slot_b_id;
  append_change_set(result.change_set, port_a_result.change_set);
  append_change_set(result.change_set, port_b_result.change_set);
  append_change_set(result.change_set, bundle_result.change_set);
  append_change_set(result.change_set, span_result.change_set);
  return result;
}

EditResult<CoreState::AddDropResult>
CoreState::AddDropFromPole(ObjectId source_pole_id, const Vec3d& target_world_position, ConnectionCategory category) {
  EditResult<AddDropResult> result;
  if (edit_state_.poles.find(source_pole_id) == nullptr) {
    result.error = "source pole does not exist";
    return result;
  }
  const BundleTemplate* bundle_template = find_bundle_template(category_to_bundle_kind(category));
  if (bundle_template == nullptr) {
    result.error = "bundle template not found";
    return result;
  }
  const ConnectionCategory resolved_category = bundle_template->category;
  const SpanLayer resolved_span_layer = bundle_template->default_layer;
  const PortLayer resolved_port_layer = span_layer_to_port_layer(resolved_span_layer);
  const int conductor_count = (bundle_template->count_rule == BundleCountRuleKind::kFixed)
                                  ? bundle_template->fixed_count
                                  : bundle_template->default_count;
  if (conductor_count <= 0) {
    result.error = "bundle template resolved invalid conductor count";
    return result;
  }

  int slot_id = -1;
  SlotSelectionRequest request{};
  request.pole_id = source_pole_id;
  request.category = resolved_category;
  request.connection_context = ConnectionContext::kDropAdd;
  const Pole* source_pole = edit_state_.poles.find(source_pole_id);
  request.pole_context = (source_pole == nullptr) ? PoleContextKind::kTerminal : source_pole->context.kind;
  request.corner_angle_deg = (source_pole == nullptr) ? 0.0 : source_pole->context.corner_angle_deg;
  request.corner_turn_sign = (source_pole == nullptr) ? 0.0 : source_pole->context.corner_turn_sign;
  request.allow_generate_port = true;
  request.preferred_slot_id = -1;
  EditResult<ObjectId> source_port_result = ensure_pole_slot_port(request, &slot_id);
  if (!source_port_result.ok) {
    result.error = source_port_result.error;
    return result;
  }
  EditResult<ObjectId> target_port_result =
      AddPort(kInvalidObjectId, target_world_position, category_to_port_kind(resolved_category), resolved_port_layer);
  if (!target_port_result.ok) {
    result.error = target_port_result.error;
    return result;
  }
  EditResult<ObjectId> bundle_result =
      AddBundle(conductor_count, std::max(0.01, bundle_template->default_spacing_m), bundle_template->id);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }
  EditResult<ObjectId> span_result = AddSpan(source_port_result.value, target_port_result.value, SpanKind::kService,
                                             resolved_span_layer, bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
  }
  Span* created_span = edit_state_.spans.find(span_result.value);
  if (created_span != nullptr) {
    created_span->placement_context = ConnectionContext::kDropAdd;
    created_span->generated_by_rule = true;
    add_unique_id(span_result.change_set.updated_ids, created_span->id);
  }

  result.ok = true;
  result.value.span_id = span_result.value;
  result.value.source_port_id = source_port_result.value;
  result.value.target_port_id = target_port_result.value;
  result.value.split_port_id = kInvalidObjectId;
  append_change_set(result.change_set, source_port_result.change_set);
  append_change_set(result.change_set, target_port_result.change_set);
  append_change_set(result.change_set, bundle_result.change_set);
  append_change_set(result.change_set, span_result.change_set);
  return result;
}

EditResult<CoreState::AddDropResult> CoreState::AddDropFromSpan(ObjectId source_span_id, double t,
                                                                const Vec3d& target_world_position,
                                                                ConnectionCategory category) {
  EditResult<AddDropResult> result;
  const BundleTemplate* bundle_template = find_bundle_template(category_to_bundle_kind(category));
  if (bundle_template == nullptr) {
    result.error = "bundle template not found";
    return result;
  }
  const ConnectionCategory resolved_category = bundle_template->category;
  const SpanLayer resolved_span_layer = bundle_template->default_layer;
  const PortLayer resolved_port_layer = span_layer_to_port_layer(resolved_span_layer);
  const int conductor_count = (bundle_template->count_rule == BundleCountRuleKind::kFixed)
                                  ? bundle_template->fixed_count
                                  : bundle_template->default_count;
  if (conductor_count <= 0) {
    result.error = "bundle template resolved invalid conductor count";
    return result;
  }
  EditResult<SplitSpanResult> split_result = SplitSpan(source_span_id, t);
  if (!split_result.ok) {
    result.error = split_result.error;
    return result;
  }
  EditResult<ObjectId> target_port_result =
      AddPort(kInvalidObjectId, target_world_position, category_to_port_kind(resolved_category), resolved_port_layer);
  if (!target_port_result.ok) {
    result.error = target_port_result.error;
    return result;
  }
  EditResult<ObjectId> bundle_result =
      AddBundle(conductor_count, std::max(0.01, bundle_template->default_spacing_m), bundle_template->id);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }
  EditResult<ObjectId> span_result = AddSpan(split_result.value.new_port_id, target_port_result.value,
                                             SpanKind::kService, resolved_span_layer, bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
  }
  Span* created_span = edit_state_.spans.find(span_result.value);
  if (created_span != nullptr) {
    created_span->placement_context = ConnectionContext::kDropAdd;
    created_span->generated_by_rule = true;
    add_unique_id(span_result.change_set.updated_ids, created_span->id);
  }

  result.ok = true;
  result.value.span_id = span_result.value;
  result.value.source_port_id = split_result.value.new_port_id;
  result.value.target_port_id = target_port_result.value;
  result.value.split_port_id = split_result.value.new_port_id;
  append_change_set(result.change_set, split_result.change_set);
  append_change_set(result.change_set, target_port_result.change_set);
  append_change_set(result.change_set, bundle_result.change_set);
  append_change_set(result.change_set, span_result.change_set);
  return result;
}

EditResult<bool> CoreState::UpdateGeometrySettings(const GeometrySettings& settings, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  GeometrySettings normalized = settings;
  normalized.curve_samples = std::max(2, normalized.curve_samples);
  if (normalized.sag_factor < 0.0) {
    normalized.sag_factor = 0.0;
  }
  normalized.pole_clearance_m = std::max(0.0, normalized.pole_clearance_m);

  const bool changed = normalized.curve_samples != cache_state_.geometry_settings.curve_samples ||
                       normalized.sag_enabled != cache_state_.geometry_settings.sag_enabled ||
                       std::abs(normalized.sag_factor - cache_state_.geometry_settings.sag_factor) > 1e-12 ||
                       std::abs(normalized.pole_clearance_m - cache_state_.geometry_settings.pole_clearance_m) > 1e-12;

  cache_state_.geometry_settings = normalized;
  result.ok = true;
  result.value = changed;

  if (changed && mark_all_spans_dirty) {
    for (const Span& span : edit_state_.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kGeometry, true);
      add_unique_id(result.change_set.dirty_span_ids, span.id);
      add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  return result;
}

EditResult<bool> CoreState::UpdateLayoutSettings(const LayoutSettings& settings) {
  EditResult<bool> result;
  LayoutSettings normalized = settings;
  normalized.corner_threshold_deg = std::clamp(normalized.corner_threshold_deg, 0.0, 179.0);
  normalized.min_side_scale = std::clamp(normalized.min_side_scale, 0.5, 4.0);
  normalized.max_side_scale = std::clamp(normalized.max_side_scale, normalized.min_side_scale, 6.0);

  const bool changed = normalized.angle_correction_enabled != layout_settings_.angle_correction_enabled ||
                       std::abs(normalized.corner_threshold_deg - layout_settings_.corner_threshold_deg) > 1e-9 ||
                       std::abs(normalized.min_side_scale - layout_settings_.min_side_scale) > 1e-9 ||
                       std::abs(normalized.max_side_scale - layout_settings_.max_side_scale) > 1e-9;

  layout_settings_ = normalized;
  result.ok = true;
  result.value = changed;
  return result;
}

EditResult<bool> CoreState::UpdateVisualSettings(const VisualSettings& settings, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  VisualSettings normalized = settings;
  normalized.support_center_threshold_m = std::max(0.0, normalized.support_center_threshold_m);
  normalized.support_arm_extra_m = std::max(0.0, normalized.support_arm_extra_m);
  normalized.insulator_radius_m = std::max(0.0, normalized.insulator_radius_m);
  normalized.insulator_length_m = std::max(0.0, normalized.insulator_length_m);

  const bool changed = normalized.enable_support_structures != cache_state_.visual_settings.enable_support_structures ||
                       normalized.enable_insulators != cache_state_.visual_settings.enable_insulators ||
                       std::abs(normalized.support_center_threshold_m -
                                cache_state_.visual_settings.support_center_threshold_m) > 1e-12 ||
                       std::abs(normalized.support_arm_extra_m - cache_state_.visual_settings.support_arm_extra_m) > 1e-12 ||
                       std::abs(normalized.insulator_radius_m - cache_state_.visual_settings.insulator_radius_m) > 1e-12 ||
                       std::abs(normalized.insulator_length_m - cache_state_.visual_settings.insulator_length_m) > 1e-12;

  cache_state_.visual_settings = normalized;
  result.ok = true;
  result.value = changed;
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : edit_state_.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kRender, true);
      add_unique_id(result.change_set.dirty_span_ids, span.id);
      add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  return result;
}

EditResult<bool> CoreState::UpdateVariationSettings(const VariationSettings& settings, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  VariationSettings normalized = settings;
  normalized.global_seed = (normalized.global_seed == 0) ? 1 : normalized.global_seed;
  normalized.world_cell_size_m = std::max(1.0, normalized.world_cell_size_m);
  normalized.world_bias_scale = std::max(0.0, normalized.world_bias_scale);
  normalized.flow_bias_scale = std::max(0.0, normalized.flow_bias_scale);
  normalized.pole_delta_scale = std::max(0.0, normalized.pole_delta_scale);
  normalized.local_jitter_scale = std::max(0.0, normalized.local_jitter_scale);
  normalized.sag_variation_scale = std::max(0.0, normalized.sag_variation_scale);
  normalized.branch_down_offset_variation_scale = std::max(0.0, normalized.branch_down_offset_variation_scale);

  const VariationSettings& current = cache_state_.variation_settings;
  const bool changed =
      normalized.enabled != current.enabled || normalized.global_seed != current.global_seed ||
      std::abs(normalized.world_cell_size_m - current.world_cell_size_m) > 1e-12 ||
      std::abs(normalized.world_bias_scale - current.world_bias_scale) > 1e-12 ||
      std::abs(normalized.flow_bias_scale - current.flow_bias_scale) > 1e-12 ||
      std::abs(normalized.pole_delta_scale - current.pole_delta_scale) > 1e-12 ||
      std::abs(normalized.local_jitter_scale - current.local_jitter_scale) > 1e-12 ||
      std::abs(normalized.sag_variation_scale - current.sag_variation_scale) > 1e-12 ||
      std::abs(normalized.branch_down_offset_variation_scale - current.branch_down_offset_variation_scale) > 1e-12;

  cache_state_.variation_settings = normalized;
  result.ok = true;
  result.value = changed;
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : edit_state_.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kGeometry | DirtyBits::kRender, true);
      add_unique_id(result.change_set.dirty_span_ids, span.id);
      add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  return result;
}

EditResult<bool> CoreState::UpdateCableTemplate(const CableTemplate& cable_template,
                                                const std::vector<ObjectId>& preferred_visible_span_ids) {
  EditResult<bool> result;
  auto it = cable_templates_.find(cable_template.id);
  if (it == cable_templates_.end()) {
    result.error = "cable template not found";
    return result;
  }

  CableTemplate normalized = cable_template;
  normalized.outer_diameter_m = std::max(0.0, normalized.outer_diameter_m);
  normalized.bend_stiffness = std::max(0.0, normalized.bend_stiffness);
  normalized.min_bend_radius_m = std::max(0.0, normalized.min_bend_radius_m);
  normalized.sag_factor = std::max(0.0, normalized.sag_factor);
  normalized.slack_factor = std::max(0.0, normalized.slack_factor);
  normalized.version = it->second.version;

  const bool changed =
      normalized.name != it->second.name || std::abs(normalized.outer_diameter_m - it->second.outer_diameter_m) > 1e-12 ||
      std::abs(normalized.bend_stiffness - it->second.bend_stiffness) > 1e-12 ||
      std::abs(normalized.min_bend_radius_m - it->second.min_bend_radius_m) > 1e-12 ||
      normalized.material_style != it->second.material_style || normalized.color_rgba != it->second.color_rgba ||
      normalized.requires_insulator != it->second.requires_insulator ||
      std::abs(normalized.sag_factor - it->second.sag_factor) > 1e-12 ||
      std::abs(normalized.slack_factor - it->second.slack_factor) > 1e-12 ||
      normalized.continuity_policy != it->second.continuity_policy ||
      normalized.attachment_style != it->second.attachment_style;
  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  std::vector<ObjectId> ordered_target_span_ids{};
  std::unordered_set<ObjectId> target_span_ids{};
  ordered_target_span_ids.reserve(preferred_visible_span_ids.size() + relation_index_.spans_by_bundle.size());
  for (ObjectId span_id : preferred_visible_span_ids) {
    if (target_span_ids.insert(span_id).second) {
      ordered_target_span_ids.push_back(span_id);
    }
  }
  for (const auto& [bundle_id, span_ids] : relation_index_.spans_by_bundle) {
    const Bundle* bundle = edit_state_.bundles.find(bundle_id);
    if (bundle == nullptr) {
      continue;
    }
    const BundleTemplate* bundle_template = find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr || bundle_template->cable_template_id != normalized.id) {
      continue;
    }
    for (ObjectId span_id : span_ids) {
      if (target_span_ids.insert(span_id).second) {
        ordered_target_span_ids.push_back(span_id);
      }
    }
  }
  for (ObjectId span_id : ordered_target_span_ids) {
    mark_span_dirty(span_id, DirtyBits::kGeometry | DirtyBits::kRender, true);
    add_unique_id(result.change_set.dirty_span_ids, span_id);
    add_unique_id(result.change_set.updated_ids, span_id);
  }
  return result;
}

EditResult<bool> CoreState::UpdateBundleTemplate(const BundleTemplate& bundle_template) {
  EditResult<bool> result;
  auto it = bundle_templates_.find(bundle_template.id);
  if (it == bundle_templates_.end()) {
    result.error = "bundle template not found";
    return result;
  }
  if (find_cable_template(bundle_template.cable_template_id) == nullptr) {
    result.error = "bundle template references unknown cable template";
    return result;
  }

  BundleTemplate normalized = bundle_template;
  normalized.version = it->second.version;
  bool changed = false;
  const bool visual_only_change =
      normalized.cable_template_id != it->second.cable_template_id && normalized.category == it->second.category &&
      normalized.default_layer == it->second.default_layer &&
      normalized.preserve_conductor_identity == it->second.preserve_conductor_identity &&
      normalized.count_rule == it->second.count_rule && normalized.fixed_count == it->second.fixed_count &&
      normalized.min_count == it->second.min_count && normalized.max_count == it->second.max_count &&
      normalized.default_count == it->second.default_count &&
      std::abs(normalized.default_spacing_m - it->second.default_spacing_m) <= 1e-12 &&
      normalized.allow_mirror == it->second.allow_mirror &&
      normalized.allow_midair_node == it->second.allow_midair_node &&
      normalized.allow_midair_branch == it->second.allow_midair_branch &&
      normalized.support_style == it->second.support_style && normalized.branch_policy == it->second.branch_policy &&
      normalized.continuity_policy == it->second.continuity_policy && normalized.name == it->second.name;

  const bool topology_change =
      normalized.category != it->second.category || normalized.default_layer != it->second.default_layer ||
      normalized.preserve_conductor_identity != it->second.preserve_conductor_identity ||
      normalized.count_rule != it->second.count_rule || normalized.fixed_count != it->second.fixed_count ||
      normalized.min_count != it->second.min_count || normalized.max_count != it->second.max_count ||
      normalized.default_count != it->second.default_count ||
      std::abs(normalized.default_spacing_m - it->second.default_spacing_m) > 1e-12 ||
      normalized.allow_mirror != it->second.allow_mirror ||
      normalized.allow_midair_node != it->second.allow_midair_node ||
      normalized.allow_midair_branch != it->second.allow_midair_branch ||
      normalized.support_style != it->second.support_style || normalized.branch_policy != it->second.branch_policy ||
      normalized.continuity_policy != it->second.continuity_policy;

  changed = visual_only_change || topology_change || normalized.name != it->second.name;
  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  template_dependency_state_.bundles_requiring_regeneration.clear();
  template_dependency_state_.sessions_requiring_regeneration.clear();

  for (Bundle& bundle : edit_state_.bundles.items_mutable()) {
    if (bundle.bundle_template_id != normalized.id) {
      continue;
    }
    add_unique_id(result.change_set.updated_ids, bundle.id);
    if (visual_only_change) {
      auto spans_it = relation_index_.spans_by_bundle.find(bundle.id);
      if (spans_it == relation_index_.spans_by_bundle.end()) {
        continue;
      }
      for (ObjectId span_id : spans_it->second) {
        mark_span_dirty(span_id, DirtyBits::kGeometry | DirtyBits::kRender, true);
        add_unique_id(result.change_set.dirty_span_ids, span_id);
        add_unique_id(result.change_set.updated_ids, span_id);
      }
      continue;
    }
    if (topology_change) {
      bundle.regeneration_required = true;
      add_unique_id(template_dependency_state_.bundles_requiring_regeneration, bundle.id);
      auto spans_it = relation_index_.spans_by_bundle.find(bundle.id);
      if (spans_it == relation_index_.spans_by_bundle.end()) {
        continue;
      }
      for (ObjectId span_id : spans_it->second) {
        const Span* span = edit_state_.spans.find(span_id);
        if (span == nullptr || span->generation.generation_session_id == 0) {
          continue;
        }
        add_unique_id(template_dependency_state_.sessions_requiring_regeneration,
                      span->generation.generation_session_id);
      }
    }
  }
  if (topology_change) {
    for (ObjectId bundle_id : template_dependency_state_.bundles_requiring_regeneration) {
      add_unique_id(result.change_set.updated_ids, bundle_id);
    }
  }
  return result;
}

EditResult<bool> CoreState::UpdateAttachmentTemplate(const AttachmentTemplate& attachment_template,
                                                     bool mark_dependent_spans_dirty) {
  EditResult<bool> result;
  auto it = attachment_templates_.find(attachment_template.id);
  if (it == attachment_templates_.end()) {
    result.error = "attachment template not found";
    return result;
  }

  AttachmentTemplate normalized = attachment_template;
  normalized.version = it->second.version;
  const bool changed = !attachment_template_equals(normalized, it->second);
  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  if (!mark_dependent_spans_dirty) {
    return result;
  }
  for (const Attachment& attachment : edit_state_.attachments.items()) {
    if (attachment.template_id != normalized.id) {
      continue;
    }
    mark_span_dirty(attachment.span_id, DirtyBits::kGeometry | DirtyBits::kRender, true);
    add_unique_id(result.change_set.dirty_span_ids, attachment.span_id);
    add_unique_id(result.change_set.updated_ids, attachment.span_id);
  }
  return result;
}

EditResult<bool> CoreState::ResetAllSpanReferenceLengths(bool mark_all_spans_dirty) {
  EditResult<bool> result;
  bool changed = false;
  for (Span& span : edit_state_.spans.items_mutable()) {
    const Port* a = edit_state_.ports.find(span.port_a_id);
    const Port* b = edit_state_.ports.find(span.port_b_id);
    if (a == nullptr || b == nullptr) {
      continue;
    }
    const double dx = b->world_position.x - a->world_position.x;
    const double dy = b->world_position.y - a->world_position.y;
    const double dz = b->world_position.z - a->world_position.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (std::abs(span.reference_length_m - length) <= 1e-9) {
      continue;
    }
    span.reference_length_m = length;
    changed = true;
    add_unique_id(result.change_set.updated_ids, span.id);
  }
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : edit_state_.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kGeometry | DirtyBits::kRender, true);
      add_unique_id(result.change_set.dirty_span_ids, span.id);
      add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  result.ok = true;
  result.value = changed;
  return result;
}
bool CoreState::has_pole_orientation_override(ObjectId pole_id) const {
  if (override_state_.pole_orientation_by_pole.contains(pole_id)) {
    return true;
  }
  const Pole* pole = edit_state_.poles.find(pole_id);
  return pole != nullptr &&
         (pole->orientation_control.manual_yaw_override || pole->orientation_control.flip_180 ||
          pole->orientation_override_flag);
}

bool CoreState::has_span_endpoint_socket_override(ObjectId span_id, bool is_start_endpoint) const {
  if (const auto it = override_state_.span_endpoint_by_span.find(span_id); it != override_state_.span_endpoint_by_span.end()) {
    const std::optional<int>& slot = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    if (slot.has_value()) {
      return true;
    }
  }
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    return false;
  }
  return is_start_endpoint ? (span->endpoint_socket_a_id >= 0) : (span->endpoint_socket_b_id >= 0);
}

bool CoreState::has_span_branch_down_offset_override(ObjectId span_id) const {
  const auto it = override_state_.span_support_by_span.find(span_id);
  return it != override_state_.span_support_by_span.end() && it->second.branch_down_offset_m.has_value();
}

std::optional<double> CoreState::resolve_pole_manual_yaw_override(const Pole& pole) const {
  if (const auto it = override_state_.pole_orientation_by_pole.find(pole.id); it != override_state_.pole_orientation_by_pole.end() &&
      it->second.manual_yaw_deg.has_value()) {
    return it->second.manual_yaw_deg;
  }
  if (pole.orientation_control.manual_yaw_override) {
    return pole.orientation_control.manual_yaw_deg;
  }
  return std::nullopt;
}

std::optional<bool> CoreState::resolve_pole_flip_180_override(const Pole& pole) const {
  if (const auto it = override_state_.pole_orientation_by_pole.find(pole.id); it != override_state_.pole_orientation_by_pole.end() &&
      it->second.flip_180.has_value()) {
    return it->second.flip_180;
  }
  if (pole.orientation_control.flip_180) {
    return true;
  }
  return std::nullopt;
}

int CoreState::resolve_span_endpoint_socket_id(const Span& span, bool is_start_endpoint) const {
  if (const auto it = override_state_.span_endpoint_by_span.find(span.id); it != override_state_.span_endpoint_by_span.end()) {
    const std::optional<int>& slot = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    if (slot.has_value()) {
      return *slot;
    }
  }
  return is_start_endpoint ? span.endpoint_socket_a_id : span.endpoint_socket_b_id;
}

double CoreState::resolve_span_branch_down_offset_m(const Span& span, double automatic_value) const {
  if (const auto it = override_state_.span_support_by_span.find(span.id); it != override_state_.span_support_by_span.end() &&
      it->second.branch_down_offset_m.has_value()) {
    return std::max(0.0, *it->second.branch_down_offset_m);
  }
  return automatic_value;
}

double CoreState::effective_pole_yaw_deg(const Pole& pole) const {
  double yaw = pole.world_transform.rotation_euler_deg.z;
  if (const std::optional<double> manual_yaw = resolve_pole_manual_yaw_override(pole); manual_yaw.has_value()) {
    yaw = *manual_yaw;
  }
  if (const std::optional<bool> flip_180 = resolve_pole_flip_180_override(pole); flip_180.value_or(false)) {
    yaw += 180.0;
  }
  return yaw;
}

Vec3d CoreState::to_local_on_pole(const Pole& pole, const Vec3d& world) const {
  return WorldPointToLocal(BuildPoleFrame(pole.world_transform, effective_pole_yaw_deg(pole)), world);
}

SlotSide CoreState::preferred_side_from_geometry(const Pole& pole, const Pole* peer, double eps) const {
  if (peer == nullptr) {
    return SlotSide::kCenter;
  }
  const Vec3d local = to_local_on_pole(pole, peer->world_transform.position);
  if (local.y > eps) {
    return SlotSide::kRight;
  }
  if (local.y < -eps) {
    return SlotSide::kLeft;
  }
  return SlotSide::kCenter;
}

double CoreState::compute_side_scale(PoleContextKind context, double corner_angle_deg) const {
  if (!layout_settings_.angle_correction_enabled || context != PoleContextKind::kCorner) {
    return 1.0;
  }
  const double threshold = std::max(0.0, layout_settings_.corner_threshold_deg);
  if (corner_angle_deg <= threshold + 1e-9) {
    return 1.0;
  }
  const double denom = std::max(1e-6, 180.0 - threshold);
  const double normalized = std::clamp((corner_angle_deg - threshold) / denom, 0.0, 1.0);
  const double scale = layout_settings_.min_side_scale +
                       (layout_settings_.max_side_scale - layout_settings_.min_side_scale) * normalized;
  return std::clamp(scale, layout_settings_.min_side_scale, layout_settings_.max_side_scale);
}

double CoreState::compute_corner_angle_deg(const Vec3d& prev, const Vec3d& curr, const Vec3d& next) {
  const Vec3d a{prev.x - curr.x, prev.y - curr.y, prev.z - curr.z};
  const Vec3d b{next.x - curr.x, next.y - curr.y, next.z - curr.z};
  const double la = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  const double lb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  if (la <= 1e-9 || lb <= 1e-9) {
    return 0.0;
  }
  double dot = (a.x * b.x + a.y * b.y + a.z * b.z) / (la * lb);
  dot = std::clamp(dot, -1.0, 1.0);
  const double interior_rad = std::acos(dot);
  const double turn_deg = 180.0 - (interior_rad * (180.0 / 3.14159265358979323846));
  return std::max(0.0, turn_deg);
}

double CoreState::compute_corner_turn_sign_xy(const Vec3d& prev, const Vec3d& curr, const Vec3d& next) {
  const Vec3d in{curr.x - prev.x, curr.y - prev.y, 0.0};
  const Vec3d out{next.x - curr.x, next.y - curr.y, 0.0};
  return in.x * out.y - in.y * out.x;
}

PoleContextInfo CoreState::classify_pole_context_from_path(const std::vector<Vec3d>& points, std::size_t index,
                                                           std::size_t pending_degree) const {
  PoleContextInfo info{};
  if (points.empty() || index >= points.size()) {
    return info;
  }

  if (pending_degree > 2) {
    info.kind = PoleContextKind::kBranch;
    info.corner_angle_deg = 0.0;
    info.corner_turn_sign = 0.0;
    info.side_scale = 1.0;
    info.angle_correction_applied = false;
    return info;
  }

  if (index == 0 || index + 1 >= points.size()) {
    info.kind = PoleContextKind::kTerminal;
    info.corner_angle_deg = 0.0;
    info.corner_turn_sign = 0.0;
    info.side_scale = 1.0;
    info.angle_correction_applied = false;
    return info;
  }

  info.corner_angle_deg = compute_corner_angle_deg(points[index - 1], points[index], points[index + 1]);
  const double turn_cross = compute_corner_turn_sign_xy(points[index - 1], points[index], points[index + 1]);
  if (turn_cross > 1e-9) {
    info.corner_turn_sign = 1.0;
  } else if (turn_cross < -1e-9) {
    info.corner_turn_sign = -1.0;
  } else {
    info.corner_turn_sign = 0.0;
  }
  if (info.corner_angle_deg >= layout_settings_.corner_threshold_deg) {
    info.kind = PoleContextKind::kCorner;
    info.side_scale = compute_side_scale(info.kind, info.corner_angle_deg);
    info.angle_correction_applied = layout_settings_.angle_correction_enabled;
  } else {
    info.kind = PoleContextKind::kStraight;
    info.corner_angle_deg = 0.0;
    info.corner_turn_sign = 0.0;
    info.side_scale = 1.0;
    info.angle_correction_applied = false;
  }
  return info;
}

std::string CoreState::next_display_id(std::string_view prefix) {
  std::uint64_t& serial = display_id_counters_[std::string(prefix)];
  ++serial;
  return make_display_id(prefix, serial);
}

void CoreState::add_unique_id(std::vector<ObjectId>& ids, ObjectId id) {
  if (id == kInvalidObjectId) {
    return;
  }
  if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
    ids.push_back(id);
  }
}

void CoreState::index_add(std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key, ObjectId value) {
  if (key == kInvalidObjectId || value == kInvalidObjectId) {
    return;
  }
  std::vector<ObjectId>& ids = map[key];
  if (std::find(ids.begin(), ids.end(), value) == ids.end()) {
    ids.push_back(value);
  }
}

void CoreState::index_remove(std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key, ObjectId value) {
  if (key == kInvalidObjectId || value == kInvalidObjectId) {
    return;
  }
  auto it = map.find(key);
  if (it == map.end()) {
    return;
  }
  std::vector<ObjectId>& ids = it->second;
  ids.erase(std::remove(ids.begin(), ids.end(), value), ids.end());
  if (ids.empty()) {
    map.erase(it);
  }
}

std::string CoreState::dirty_bits_to_string(DirtyBits bits) {
  std::string text;
  if (any(bits, DirtyBits::kTopology))
    text += "Topology|";
  if (any(bits, DirtyBits::kGeometry))
    text += "Geometry|";
  if (any(bits, DirtyBits::kBounds))
    text += "Bounds|";
  if (any(bits, DirtyBits::kRender))
    text += "Render|";
  if (any(bits, DirtyBits::kRaycast))
    text += "Raycast|";
  if (!text.empty()) {
    text.pop_back();
  }
  return text;
}

void CoreState::apply_pole_placement_mode(Pole& pole, PlacementMode mode) {
  pole.placement_mode = mode;
  pole.user_edited = (mode == PlacementMode::kManual);
}

void CoreState::apply_port_position_mode(Port& port, PortPositionMode mode, PortPlacementSourceKind source_hint) {
  port.position_mode = mode;
  if (mode == PortPositionMode::kManual) {
    port.user_edited_position = true;
    port.placement_source = PortPlacementSourceKind::kManualEdit;
    return;
  }

  // Auto mode keeps source semantics explicit and clears manual marker.
  port.user_edited_position = false;
  if (source_hint == PortPlacementSourceKind::kManualEdit) {
    port.placement_source = PortPlacementSourceKind::kGenerated;
  } else {
    port.placement_source = source_hint;
  }
}

double CoreState::pole_radius_at_height_m(const Pole& pole, double local_z_m) const {
  double base_radius = 0.16;
  switch (pole.kind) {
  case PoleKind::kWood:
    base_radius = 0.18;
    break;
  case PoleKind::kConcrete:
    base_radius = 0.22;
    break;
  case PoleKind::kSteel:
    base_radius = 0.14;
    break;
  default:
    base_radius = 0.16;
    break;
  }
  const double top_radius = std::max(0.06, base_radius * 0.55);
  const double h = std::max(0.1, pole.height_m);
  const double t = std::clamp(local_z_m / h, 0.0, 1.0);
  return base_radius + (top_radius - base_radius) * t;
}

Vec3d CoreState::apply_pole_clearance_to_local(const Pole& pole, const Vec3d& local, SlotSide side) const {
  Vec3d adjusted = local;
  const double min_offset = pole_radius_at_height_m(pole, std::max(0.0, adjusted.z)) + cache_state_.geometry_settings.pole_clearance_m;
  double sign = (adjusted.y >= 0.0) ? 1.0 : -1.0;
  if (side == SlotSide::kLeft) {
    sign = -1.0;
  } else if (side == SlotSide::kRight) {
    sign = 1.0;
  }
  if (std::abs(adjusted.y) < min_offset) {
    adjusted.y = sign * min_offset;
  }
  return adjusted;
}

bool CoreState::has_zero_length(const Port& a, const Port& b) {
  const double dx = a.world_position.x - b.world_position.x;
  const double dy = a.world_position.y - b.world_position.y;
  const double dz = a.world_position.z - b.world_position.z;
  return (dx * dx + dy * dy + dz * dz) <= (kZeroLengthEps * kZeroLengthEps);
}

std::unordered_map<ObjectId, std::vector<ObjectId>> CoreState::make_expected_port_index(const EditState& edit_state) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> out;
  for (const Span& span : edit_state.spans.items()) {
    out[span.port_a_id].push_back(span.id);
    out[span.port_b_id].push_back(span.id);
  }
  return out;
}

std::unordered_map<ObjectId, std::vector<ObjectId>> CoreState::make_expected_anchor_index(const EditState& edit_state) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> out;
  for (const Span& span : edit_state.spans.items()) {
    if (span.anchor_a_id != kInvalidObjectId) {
      out[span.anchor_a_id].push_back(span.id);
    }
    if (span.anchor_b_id != kInvalidObjectId) {
      out[span.anchor_b_id].push_back(span.id);
    }
  }
  return out;
}

std::unordered_map<ObjectId, std::vector<ObjectId>> CoreState::make_expected_pole_port_index(const EditState& edit_state) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> out;
  for (const Port& port : edit_state.ports.items()) {
    if (port.owner_pole_id != kInvalidObjectId) {
      out[port.owner_pole_id].push_back(port.id);
    }
  }
  return out;
}

std::unordered_map<ObjectId, std::vector<ObjectId>>
CoreState::make_expected_pole_anchor_index(const EditState& edit_state) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> out;
  for (const Anchor& anchor : edit_state.anchors.items()) {
    if (anchor.owner_pole_id != kInvalidObjectId) {
      out[anchor.owner_pole_id].push_back(anchor.id);
    }
  }
  return out;
}

std::unordered_map<ObjectId, std::vector<ObjectId>> CoreState::make_expected_bundle_span_index(const EditState& edit_state) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> out;
  for (const Span& span : edit_state.spans.items()) {
    if (span.bundle_id != kInvalidObjectId) {
      out[span.bundle_id].push_back(span.id);
    }
  }
  return out;
}

std::unordered_map<ObjectId, std::vector<ObjectId>> CoreState::make_expected_span_attachment_index(
    const EditState& edit_state) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> out;
  for (const Attachment& attachment : edit_state.attachments.items()) {
    if (attachment.span_id != kInvalidObjectId) {
      out[attachment.span_id].push_back(attachment.id);
    }
  }
  return out;
}

double CoreState::polyline_length(const std::vector<Vec3d>& polyline) {
  if (polyline.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const Vec3d d = polyline[i + 1] - polyline[i];
    total += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  }
  return total;
}

std::vector<Vec3d> CoreState::sample_polyline_points(const std::vector<Vec3d>& polyline, double interval) {
  if (polyline.size() < 2 || interval <= 0.0) {
    return {};
  }

  const std::size_t segment_count = polyline.size() - 1;
  std::vector<double> segment_lengths(segment_count, 0.0);
  double total = 0.0;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const Vec3d d = polyline[i + 1] - polyline[i];
    segment_lengths[i] = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    total += segment_lengths[i];
  }

  if (total <= kZeroLengthEps) {
    return {};
  }

  std::vector<double> targets;
  targets.push_back(0.0);
  for (double dist = interval; dist < total; dist += interval) {
    targets.push_back(dist);
  }
  if (targets.empty() || std::abs(targets.back() - total) > 1e-9) {
    targets.push_back(total);
  }

  std::vector<Vec3d> out;
  out.reserve(targets.size());
  std::size_t seg_idx = 0;
  double accum = 0.0;
  for (double target : targets) {
    while (seg_idx + 1 < segment_count && accum + segment_lengths[seg_idx] < target - 1e-9) {
      accum += segment_lengths[seg_idx];
      ++seg_idx;
    }

    const double seg_len = segment_lengths[seg_idx];
    const double local = (seg_len <= kZeroLengthEps) ? 0.0 : ((target - accum) / seg_len);
    const double t = std::clamp(local, 0.0, 1.0);

    const Vec3d& a = polyline[seg_idx];
    const Vec3d& b = polyline[seg_idx + 1];
    Vec3d p{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    };

    if (!out.empty()) {
      const Vec3d d = p - out.back();
      const double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
      if (dist2 <= (kZeroLengthEps * kZeroLengthEps)) {
        continue;
      }
    }
    out.push_back(p);
  }

  if (out.size() < 2 || std::abs(out.back().x - polyline.back().x) > 1e-9 ||
      std::abs(out.back().y - polyline.back().y) > 1e-9 || std::abs(out.back().z - polyline.back().z) > 1e-9) {
    out.push_back(polyline.back());
  }
  return out;
}

CoreState make_demo_state() {
  CoreState state;

  Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  Transformd b{};
  b.position = {12.0, 0.0, 0.0};
  Transformd c{};
  c.position = {24.0, 0.0, 0.0};

  const ObjectId pole_a = state.AddPole(a, 10.0, "Pole-1", PoleKind::kWood).value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "Pole-2", PoleKind::kConcrete).value;
  const ObjectId pole_c = state.AddPole(c, 10.0, "Pole-3", PoleKind::kConcrete).value;

  (void)state.ApplyPoleType(pole_a, kDistributionPoleType);
  (void)state.ApplyPoleType(pole_b, kDistributionPoleType);
  (void)state.ApplyPoleType(pole_c, kCommunicationPoleType);

  auto connect_with_template = [&](ObjectId a_id, ObjectId b_id, ConnectionCategory category) {
    CoreState::AddConnectionByPoleOptions options{};
    options.use_bundle_template = true;
    switch (category) {
    case ConnectionCategory::kHighVoltage:
      options.bundle_template_id = BundleKind::kHighVoltage;
      break;
    case ConnectionCategory::kCommunication:
      options.bundle_template_id = BundleKind::kCommunication;
      break;
    case ConnectionCategory::kOptical:
      options.bundle_template_id = BundleKind::kOptical;
      break;
    case ConnectionCategory::kLowVoltage:
    case ConnectionCategory::kDrop:
    default:
      options.bundle_template_id = BundleKind::kLowVoltage;
      break;
    }
    (void)state.AddConnectionByPole(a_id, b_id, category, options);
  };
  connect_with_template(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  connect_with_template(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  connect_with_template(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  connect_with_template(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  connect_with_template(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  connect_with_template(pole_a, pole_b, ConnectionCategory::kCommunication);
  connect_with_template(pole_a, pole_b, ConnectionCategory::kOptical);

  connect_with_template(pole_b, pole_c, ConnectionCategory::kLowVoltage);
  connect_with_template(pole_b, pole_c, ConnectionCategory::kCommunication);
  connect_with_template(pole_b, pole_c, ConnectionCategory::kCommunication);
  connect_with_template(pole_b, pole_c, ConnectionCategory::kOptical);

  (void)state.AddDropFromPole(pole_b, {13.0, 4.0, 3.0}, ConnectionCategory::kDrop);
  (void)state.AddDropFromPole(pole_b, {14.0, -4.0, 3.0}, ConnectionCategory::kDrop);
  (void)state.Commit();

  return state;
}

} // namespace wire::core

