#include "internal_services.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::state_internal {

namespace {

Vec3d local_to_world_on_pole(const Transformd& tf, double yaw_deg, const Vec3d& local) {
  return LocalPointToWorld(BuildPoleFrame(tf, yaw_deg), local);
}

double apply_corner_side_scale(double local_y, SlotSide slot_side, double turn_sign, double side_scale) {
  if (slot_side == SlotSide::kCenter) {
    return local_y;
  }
  const double inner_scale = 1.0 + (side_scale - 1.0) * 0.35;
  SlotSide inner_side = SlotSide::kCenter;
  if (turn_sign > 1e-9) {
    inner_side = SlotSide::kLeft;
  } else if (turn_sign < -1e-9) {
    inner_side = SlotSide::kRight;
  }
  if (inner_side == SlotSide::kCenter) {
    return local_y * side_scale;
  }
  if (slot_side == inner_side) {
    return local_y * inner_scale;
  }
  return local_y * side_scale;
}

const PortSlotTemplate* find_port_slot(const PoleTypeDefinition* pole_type, int slot_id) {
  if (pole_type == nullptr) {
    return nullptr;
  }
  for (const PortSlotTemplate& slot : pole_type->port_slots) {
    if (slot.slot_id == slot_id) {
      return &slot;
    }
  }
  return nullptr;
}

const AnchorSlotTemplate* find_anchor_slot(const PoleTypeDefinition* pole_type, int slot_id) {
  if (pole_type == nullptr) {
    return nullptr;
  }
  for (const AnchorSlotTemplate& slot : pole_type->anchor_slots) {
    if (slot.slot_id == slot_id) {
      return &slot;
    }
  }
  return nullptr;
}

} // namespace

OwnedEndpointIds EndpointRefreshService::CollectOwnedEndpointIds(const CoreState& state, ObjectId pole_id) {
  OwnedEndpointIds ids{};
  if (const auto it = state.relation_index_.ports_by_pole.find(pole_id); it != state.relation_index_.ports_by_pole.end()) {
    ids.port_ids = it->second;
    ids.port_ids.erase(
        std::remove_if(ids.port_ids.begin(), ids.port_ids.end(),
                       [&](ObjectId id) { return state.edit_state_.ports.find(id) == nullptr; }),
        ids.port_ids.end());
    std::sort(ids.port_ids.begin(), ids.port_ids.end());
    ids.port_ids.erase(std::unique(ids.port_ids.begin(), ids.port_ids.end()), ids.port_ids.end());
  }
  if (const auto it = state.relation_index_.anchors_by_pole.find(pole_id);
      it != state.relation_index_.anchors_by_pole.end()) {
    ids.anchor_ids = it->second;
    ids.anchor_ids.erase(
        std::remove_if(ids.anchor_ids.begin(), ids.anchor_ids.end(),
                       [&](ObjectId id) { return state.edit_state_.anchors.find(id) == nullptr; }),
        ids.anchor_ids.end());
    std::sort(ids.anchor_ids.begin(), ids.anchor_ids.end());
    ids.anchor_ids.erase(std::unique(ids.anchor_ids.begin(), ids.anchor_ids.end()), ids.anchor_ids.end());
  }
  return ids;
}

void EndpointRefreshService::RefreshOwnedEndpointsFromPole(CoreState& state, ObjectId pole_id, ChangeSet* change_set,
                                                           const Pole* previous_pole,
                                                           const double* previous_layout_yaw_override) {
  Pole* pole = state.edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    return;
  }

  const PoleTypeDefinition* pole_type = state.find_pole_type(pole->pole_type_id);
  const double effective_yaw = state.effective_pole_yaw_deg(*pole);
  const double layout_yaw = state.effective_pole_layout_yaw_deg(*pole);
  const double previous_layout_yaw =
      (previous_layout_yaw_override != nullptr)
          ? *previous_layout_yaw_override
          : ((previous_pole == nullptr) ? effective_yaw : state.effective_pole_layout_yaw_deg(*previous_pole));

  const OwnedEndpointIds owned = CollectOwnedEndpointIds(state, pole_id);

  for (ObjectId port_id : owned.port_ids) {
    Port* port = state.edit_state_.ports.find(port_id);
    if (port == nullptr || port->position_mode == PortPositionMode::kManual) {
      continue;
    }
    const PortSlotTemplate* slot = find_port_slot(pole_type, port->source_slot_id);
    Vec3d new_world = port->world_position;
    bool apply_angle_correction = false;
    double applied_scale = 1.0;
    if (slot != nullptr) {
      Vec3d adjusted_local = slot->local_position;
      apply_angle_correction = state.layout_settings_.angle_correction_enabled &&
                               pole->context.kind == PoleContextKind::kCorner && slot->side != SlotSide::kCenter;
      if (apply_angle_correction) {
        adjusted_local.y =
            apply_corner_side_scale(adjusted_local.y, slot->side, pole->context.corner_turn_sign, pole->context.side_scale);
        if (std::abs(slot->local_position.y) > 1e-9) {
          applied_scale = std::abs(adjusted_local.y / slot->local_position.y);
        }
      }
      adjusted_local = state.apply_pole_clearance_to_local(*pole, adjusted_local, slot->side);
      new_world = local_to_world_on_pole(pole->world_transform, layout_yaw, adjusted_local);
    } else if (previous_pole != nullptr) {
      const Vec3d old_local =
          WorldPointToLocal(BuildPoleFrame(previous_pole->world_transform, previous_layout_yaw), port->world_position);
      new_world = local_to_world_on_pole(pole->world_transform, layout_yaw, old_local);
    }

    const bool moved = std::abs(new_world.x - port->world_position.x) > 1e-9 ||
                       std::abs(new_world.y - port->world_position.y) > 1e-9 ||
                       std::abs(new_world.z - port->world_position.z) > 1e-9;
    const bool changed_scale =
        std::abs(port->side_scale_applied - (apply_angle_correction ? applied_scale : 1.0)) > 1e-9;
    const bool changed_angle_flag = port->angle_correction_applied != apply_angle_correction;
    if (!moved && !changed_scale && !changed_angle_flag) {
      continue;
    }

    port->world_position = new_world;
    port->angle_correction_applied = apply_angle_correction;
    port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
    state.apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kTemplateSlot);
    if (change_set != nullptr) {
      state.add_unique_id(change_set->updated_ids, port->id);
      state.mark_connected_spans_dirty_from_port(port->id, DirtyBits::kGeometry, change_set);
    }
  }

  for (ObjectId anchor_id : owned.anchor_ids) {
    Anchor* anchor = state.edit_state_.anchors.find(anchor_id);
    if (anchor == nullptr) {
      continue;
    }
    const AnchorSlotTemplate* slot = find_anchor_slot(pole_type, anchor->source_slot_id);
    Vec3d new_world = anchor->world_position;
    if (slot != nullptr) {
      new_world = local_to_world_on_pole(pole->world_transform, effective_yaw, slot->local_position);
    } else if (previous_pole != nullptr) {
      const Vec3d old_local = state.to_local_on_pole(*previous_pole, anchor->world_position);
      new_world = local_to_world_on_pole(pole->world_transform, effective_yaw, old_local);
    }
    const bool moved = std::abs(new_world.x - anchor->world_position.x) > 1e-9 ||
                       std::abs(new_world.y - anchor->world_position.y) > 1e-9 ||
                       std::abs(new_world.z - anchor->world_position.z) > 1e-9;
    if (!moved) {
      continue;
    }

    anchor->world_position = new_world;
    if (change_set != nullptr) {
      state.add_unique_id(change_set->updated_ids, anchor->id);
      state.mark_connected_spans_dirty_from_anchor(anchor->id, DirtyBits::kGeometry, change_set);
    }
  }
}

} // namespace wire::core::state_internal
