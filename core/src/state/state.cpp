#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "internal_services.hpp"
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

std::uint64_t mix_u64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

double unit_random_from_u64(std::uint64_t x) {
  constexpr double kInv = 1.0 / static_cast<double>(1ull << 53);
  return static_cast<double>((x >> 11) & ((1ull << 53) - 1)) * kInv;
}

double deterministic_pole_tilt_factor(ObjectId pole_id) {
  return unit_random_from_u64(mix_u64(static_cast<std::uint64_t>(pole_id) ^ 0x54D3C92F7A6B1E29ull));
}

double deterministic_pole_tilt_azimuth_deg(ObjectId pole_id) {
  return unit_random_from_u64(mix_u64(static_cast<std::uint64_t>(pole_id) ^ 0xA1937465C4FB2D81ull)) * 360.0;
}

Vec3d unit_xy_from_azimuth_deg(double azimuth_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double azimuth_rad = azimuth_deg * (kPi / 180.0);
  return {std::cos(azimuth_rad), std::sin(azimuth_rad), 0.0};
}

double azimuth_deg_from_xy(const Vec3d& dir) {
  constexpr double kPi = 3.14159265358979323846;
  return std::atan2(dir.y, dir.x) * (180.0 / kPi);
}

Vec3d tilt_euler_xy_from_local_polar_deg(double tilt_deg, double local_azimuth_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double tilt_rad = std::clamp(tilt_deg, 0.0, 89.0) * (kPi / 180.0);
  const double azimuth_rad = local_azimuth_deg * (kPi / 180.0);
  const double hx = std::sin(tilt_rad) * std::cos(azimuth_rad);
  const double hy = std::sin(tilt_rad) * std::sin(azimuth_rad);
  const double tilt_x_rad = -std::asin(std::clamp(hy, -1.0, 1.0));
  const double cos_x = std::max(1e-9, std::cos(tilt_x_rad));
  const double tilt_y_rad = std::asin(std::clamp(hx / cos_x, -1.0, 1.0));
  return {tilt_x_rad * (180.0 / kPi), tilt_y_rad * (180.0 / kPi), 0.0};
}

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
  case PortLayer::kDrop:
    return ConnectionCategory::kDrop;
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
  case SpanLayer::kDrop:
    return PortLayer::kDrop;
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
  case SpanLayer::kDrop:
    return ConnectionCategory::kDrop;
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

bool port_band_equals(const PortPlacementBand& a, const PortPlacementBand& b) {
  const auto same_double = [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; };
  return a.band_id == b.band_id && a.category == b.category && a.layer == b.layer && a.side == b.side &&
         a.role == b.role && same_double(a.lateral_center_m, b.lateral_center_m) &&
         same_double(a.lateral_min_m, b.lateral_min_m) && same_double(a.lateral_max_m, b.lateral_max_m) &&
         same_double(a.height_center_m, b.height_center_m) && same_double(a.height_min_m, b.height_min_m) &&
         same_double(a.height_max_m, b.height_max_m) && a.priority == b.priority &&
         same_double(a.min_spacing_m, b.min_spacing_m) && a.allow_multiple == b.allow_multiple &&
         a.overflow_policy == b.overflow_policy && a.enabled == b.enabled;
}

bool anchor_slot_equals(const AnchorSlotTemplate& a, const AnchorSlotTemplate& b) {
  const auto same_double = [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; };
  return a.slot_id == b.slot_id && a.usage == b.usage && same_double(a.local_position.x, b.local_position.x) &&
         same_double(a.local_position.y, b.local_position.y) && same_double(a.local_position.z, b.local_position.z) &&
         a.priority == b.priority && a.enabled == b.enabled;
}

bool pole_type_definition_equals(const PoleTypeDefinition& a, const PoleTypeDefinition& b) {
  if (a.id != b.id || a.name != b.name || a.description != b.description ||
      std::abs(a.default_height_m - b.default_height_m) > 1e-12 || a.port_bands.size() != b.port_bands.size() ||
      a.anchor_slots.size() != b.anchor_slots.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.port_bands.size(); ++i) {
    if (!port_band_equals(a.port_bands[i], b.port_bands[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.anchor_slots.size(); ++i) {
    if (!anchor_slot_equals(a.anchor_slots[i], b.anchor_slots[i])) {
      return false;
    }
  }
  return true;
}

double normalize_yaw_deg(double yaw_deg) {
  return NormalizeYawDeg(yaw_deg);
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
  pole.id = identity_.id_generator.next();
  pole.display_id = next_display_id("P");
  pole.name = std::string(name);
  pole.world_transform = world_transform;
  pole.height_m = height_m;
  pole.kind = kind;
  pole.pole_type_id = kInvalidPoleTypeId;
  apply_pole_placement_mode(pole, placement_mode);
  authoritative_.edit_state.poles.insert(pole);

  result.ok = true;
  result.value = pole.id;
  result.change_set.created_ids.push_back(pole.id);
  return result;
}

EditResult<ObjectId> CoreState::AddPort(ObjectId owner_pole_id, const Vec3d& world_position, PortKind kind,
                                        PortLayer layer) {
  return AddPort(owner_pole_id, world_position, kind, layer, Frame3d{});
}

EditResult<ObjectId> CoreState::AddPort(ObjectId owner_pole_id, const Vec3d& world_position, PortKind kind,
                                        PortLayer layer, const Frame3d& direction) {
  EditResult<ObjectId> result;
  if (owner_pole_id != kInvalidObjectId && authoritative_.edit_state.poles.find(owner_pole_id) == nullptr) {
    result.error = "owner pole does not exist";
    return result;
  }

  Port port{};
  port.id = identity_.id_generator.next();
  port.display_id = next_display_id("PT");
  port.owner_pole_id = owner_pole_id;
  port.world_position = world_position;
  port.kind = kind;
  port.layer = layer;
  port.direction = direction;
  port.category = port_layer_to_category(layer);
  port.template_layer = generation::detail::TemplateLayerForCategory(port.category);
  port.template_side = SlotSide::kCenter;
  port.template_role = SlotRole::kNeutral;
  port.generated_from_template = false;
  port.generated_by_rule = false;
  port.placement_context = ConnectionContext::kTrunkContinue;
  port.angle_correction_applied = false;
  port.side_scale_applied = 1.0;
  apply_port_position_mode(port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  authoritative_.edit_state.ports.insert(port);
  if (owner_pole_id != kInvalidObjectId) {
    index_add(runtime_.relation_index.ports_by_pole, owner_pole_id, port.id);
  }

  result.ok = true;
  result.value = port.id;
  result.change_set.created_ids.push_back(port.id);
  return result;
}

EditResult<ObjectId> CoreState::AddAnchor(ObjectId owner_pole_id, const Vec3d& world_position,
                                          AnchorSupportKind support_kind, double support_strength) {
  EditResult<ObjectId> result;
  if (owner_pole_id != kInvalidObjectId && authoritative_.edit_state.poles.find(owner_pole_id) == nullptr) {
    result.error = "owner pole does not exist";
    return result;
  }
  if (support_strength <= 0.0) {
    result.error = "support strength must be > 0";
    return result;
  }

  Anchor anchor{};
  anchor.id = identity_.id_generator.next();
  anchor.display_id = next_display_id("A");
  anchor.owner_pole_id = owner_pole_id;
  anchor.world_position = world_position;
  anchor.support_kind = support_kind;
  anchor.support_strength = support_strength;
  authoritative_.edit_state.anchors.insert(anchor);
  if (owner_pole_id != kInvalidObjectId) {
    index_add(runtime_.relation_index.anchors_by_pole, owner_pole_id, anchor.id);
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
  bundle.id = identity_.id_generator.next();
  bundle.display_id = next_display_id("B");
  bundle.conductor_count = conductor_count;
  bundle.phase_spacing_m = phase_spacing_m;
  bundle.bundle_template_id = kind;
  authoritative_.edit_state.bundles.insert(bundle);

  result.ok = true;
  result.value = bundle.id;
  result.change_set.created_ids.push_back(bundle.id);
  return result;
}


EditResult<ObjectId> CoreState::AddSpan(ObjectId port_a_id, ObjectId port_b_id, SpanKind kind, SpanLayer layer,
                                        ObjectId bundle_id, ObjectId anchor_a_id, ObjectId anchor_b_id) {
  EditResult<ObjectId> result;
  const Port* port_a = authoritative_.edit_state.ports.find(port_a_id);
  const Port* port_b = authoritative_.edit_state.ports.find(port_b_id);
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
  if (bundle_id != kInvalidObjectId && authoritative_.edit_state.bundles.find(bundle_id) == nullptr) {
    result.error = "bundle does not exist";
    return result;
  }
  if (anchor_a_id != kInvalidObjectId && authoritative_.edit_state.anchors.find(anchor_a_id) == nullptr) {
    result.error = "anchor_a does not exist";
    return result;
  }
  if (anchor_b_id != kInvalidObjectId && authoritative_.edit_state.anchors.find(anchor_b_id) == nullptr) {
    result.error = "anchor_b does not exist";
    return result;
  }

  Span span{};
  span.id = identity_.id_generator.next();
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
  authoritative_.edit_state.spans.insert(span);

  mark_topology_related_spans_for_ports_dirty({port_a_id, port_b_id}, span.id,
                                              DirtyBits::kTopology | DirtyBits::kDecision, &result.change_set);
  add_span_to_index(span);
  initialize_span_runtime_state(span.id);
  mark_span_dirty(span.id, DirtyBits::kTopology | DirtyBits::kDecision, true);

  result.ok = true;
  result.value = span.id;
  result.change_set.created_ids.push_back(span.id);
  result.change_set.dirty_span_ids.push_back(span.id);
  return result;
}

EditResult<bool> CoreState::set_span_endpoint_nodes(ObjectId span_id, ObjectId node_a_id, ObjectId node_b_id) {
  EditResult<bool> result{};
  Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span does not exist";
    return result;
  }
  span->endpoint_node_a_id = node_a_id;
  span->endpoint_node_b_id = node_b_id;
  result.ok = true;
  result.value = true;
  return result;
}

EditResult<ObjectId> CoreState::AddAttachment(ObjectId span_id, double t, AttachmentKind kind, double display_offset_m,
                                              AttachmentTemplateId template_id) {
  EditResult<ObjectId> result;
  if (authoritative_.edit_state.spans.find(span_id) == nullptr) {
    result.error = "span does not exist";
    return result;
  }
  if (t < 0.0 || t > 1.0) {
    result.error = "attachment t must be in [0, 1]";
    return result;
  }
  if (template_id == kInvalidAttachmentTemplateId) {
    for (const auto& [candidate_id, attachment_template] : authoritative_.attachment_templates) {
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
  attachment.id = identity_.id_generator.next();
  attachment.display_id = next_display_id("AT");
  attachment.span_id = span_id;
  attachment.template_id = template_id;
  attachment.t = t;
  attachment.kind = kind;
  attachment.display_offset_m = display_offset_m;
  authoritative_.edit_state.attachments.insert(attachment);
  index_add(runtime_.relation_index.attachments_by_span, span_id, attachment.id);
  mark_span_dirty(span_id, DirtyBits::kDecision, true);

  result.ok = true;
  result.value = attachment.id;
  result.change_set.created_ids.push_back(attachment.id);
  result.change_set.updated_ids.push_back(span_id);
  result.change_set.dirty_span_ids.push_back(span_id);
  return result;
}


EditResult<ObjectId> CoreState::MovePole(ObjectId pole_id, const Transformd& new_world_transform) {
  EditResult<ObjectId> result;
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
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

EditResult<bool> CoreState::ApplyPoleTilt(const std::vector<ObjectId>& pole_ids, double max_tilt_deg) {
  EditResult<bool> result;
  std::vector<ObjectId> targets = pole_ids;
  if (targets.empty()) {
    targets.reserve(authoritative_.edit_state.poles.size());
    for (const Pole& pole : authoritative_.edit_state.poles.items()) {
      targets.push_back(pole.id);
    }
  }
  const double clamped_max_tilt_deg = std::clamp(max_tilt_deg, 0.0, 45.0);
  bool changed = false;
  for (ObjectId pole_id : targets) {
    Pole* pole = authoritative_.edit_state.poles.find(pole_id);
    if (pole == nullptr) {
      result.error = "pole not found";
      result.ok = false;
      return result;
    }
    const Pole old_pole = *pole;
    const double random_tilt_factor = deterministic_pole_tilt_factor(pole_id);
    const Vec3d random_world_dir = unit_xy_from_azimuth_deg(deterministic_pole_tilt_azimuth_deg(pole_id));
    Vec3d pull_world_dir{};
    std::unordered_set<ObjectId> seen_spans{};
    std::size_t incident_span_count = 0;
    if (const auto ports_it = runtime_.relation_index.ports_by_pole.find(pole_id); ports_it != runtime_.relation_index.ports_by_pole.end()) {
      for (ObjectId port_id : ports_it->second) {
        const auto spans_it = runtime_.connection_index.spans_by_port.find(port_id);
        if (spans_it == runtime_.connection_index.spans_by_port.end()) {
          continue;
        }
        for (ObjectId span_id : spans_it->second) {
          if (!seen_spans.insert(span_id).second) {
            continue;
          }
          const Span* span = authoritative_.edit_state.spans.find(span_id);
          if (span == nullptr) {
            continue;
          }
          const ObjectId other_port_id =
              (span->port_a_id == port_id) ? span->port_b_id : (span->port_b_id == port_id ? span->port_a_id : kInvalidObjectId);
          if (other_port_id == kInvalidObjectId) {
            continue;
          }
          const Port* other_port = authoritative_.edit_state.ports.find(other_port_id);
          if (other_port == nullptr) {
            continue;
          }
          Vec3d span_dir = other_port->world_position - pole->world_transform.position;
          span_dir.z = 0.0;
          if (!NormalizeXY(&span_dir)) {
            continue;
          }
          pull_world_dir = pull_world_dir + span_dir;
          ++incident_span_count;
        }
      }
    }
    Vec3d tilt_world_dir = random_world_dir;
    double pull_strength = 0.0;
    if (incident_span_count > 0) {
      pull_strength = std::clamp(std::sqrt(LengthSquared(pull_world_dir)) / static_cast<double>(incident_span_count), 0.0, 1.0);
    }
    if (NormalizeXY(&pull_world_dir)) {
      const double pull_bias = 0.60 + 0.25 * pull_strength;
      tilt_world_dir =
          ScaleVec(pull_world_dir, pull_bias) + ScaleVec(random_world_dir, 1.0 - pull_bias);
      if (!NormalizeXY(&tilt_world_dir)) {
        tilt_world_dir = pull_world_dir;
      }
    }
    const double applied_tilt_scale =
        (incident_span_count > 0) ? (0.20 + 0.80 * pull_strength) : 1.0;
    const double applied_tilt_deg = clamped_max_tilt_deg * random_tilt_factor * applied_tilt_scale;
    const double layout_yaw_deg = effective_pole_layout_yaw_deg(*pole);
    const double local_azimuth_deg = NormalizeYawDeg(azimuth_deg_from_xy(tilt_world_dir) - layout_yaw_deg);
    const Vec3d tilt_euler_deg = tilt_euler_xy_from_local_polar_deg(applied_tilt_deg, local_azimuth_deg);
    if (std::abs(pole->tilt_magnitude_deg - applied_tilt_deg) <= 1e-9 &&
        std::abs(pole->world_transform.rotation_euler_deg.x - tilt_euler_deg.x) <= 1e-9 &&
        std::abs(pole->world_transform.rotation_euler_deg.y - tilt_euler_deg.y) <= 1e-9) {
      continue;
    }
    pole->tilt_magnitude_deg = applied_tilt_deg;
    pole->world_transform.rotation_euler_deg.x = tilt_euler_deg.x;
    pole->world_transform.rotation_euler_deg.y = tilt_euler_deg.y;
    finalize_pole_transform_update(pole->id, old_pole, &result.change_set);
    changed = true;
  }
  result.ok = true;
  result.value = changed;
  return result;
}

EditResult<ObjectId> CoreState::SetPoleTilt(ObjectId pole_id, double max_tilt_deg) {
  EditResult<ObjectId> result;
  const auto apply = ApplyPoleTilt({pole_id}, max_tilt_deg);
  result.ok = apply.ok;
  result.error = apply.error;
  result.change_set = apply.change_set;
  result.value = pole_id;
  return result;
}

EditResult<bool> CoreState::SetAllPoleTilt(double max_tilt_deg) {
  return ApplyPoleTilt({}, max_tilt_deg);
}

EditResult<ObjectId> CoreState::MovePort(ObjectId port_id, const Vec3d& new_world_position) {
  return SetPortWorldPositionManual(port_id, new_world_position);
}

EditResult<ObjectId> CoreState::SetPortWorldPositionManual(ObjectId port_id, const Vec3d& new_world_position) {
  EditResult<ObjectId> result;
  Port* port = authoritative_.edit_state.ports.find(port_id);
  if (port == nullptr) {
    result.error = "port not found";
    return result;
  }
  port->world_position = new_world_position;
  apply_port_position_mode(*port, PortPositionMode::kManual, PortPlacementSourceKind::kManualEdit);
  result.change_set.updated_ids.push_back(port_id);
  mark_connected_spans_dirty_from_port(port_id, DirtyBits::kGeometryRefresh, &result.change_set);
  result.ok = true;
  result.value = port_id;
  return result;
}

EditResult<ObjectId> CoreState::ResetPortPositionToAuto(ObjectId port_id) {
  EditResult<ObjectId> result;
  Port* port = authoritative_.edit_state.ports.find(port_id);
  if (port == nullptr) {
    result.error = "port not found";
    return result;
  }

  apply_port_position_mode(*port, PortPositionMode::kAuto, port->placement_source);

  bool recomputed = false;
  if (port->owner_pole_id != kInvalidObjectId && port->generated_from_template) {
    const Pole* pole = authoritative_.edit_state.poles.find(port->owner_pole_id);
    if (pole != nullptr) {
      const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id);
      if (pole_type != nullptr) {
        const PortPlacementBand* band_ptr = nullptr;
        for (const PortPlacementBand& band : pole_type->port_bands) {
          if (!band.enabled || band.category != port->category || band.layer != port->template_layer ||
              band.side != port->template_side || band.role != port->template_role) {
            continue;
          }
          if (band_ptr == nullptr || band.priority > band_ptr->priority ||
              (band.priority == band_ptr->priority && band.band_id < band_ptr->band_id)) {
            band_ptr = &band;
          }
        }
        if (band_ptr != nullptr) {
          const PoleFrame frame =
              BuildPoleFrame(pole->world_transform, effective_port_layout_yaw_deg(*pole, port->category));
          const Vec3d current_local = WorldPointToLocal(frame, port->world_position);
          Vec3d adjusted_local{
              0.0,
              std::clamp(current_local.y, band_ptr->lateral_min_m, band_ptr->lateral_max_m),
              std::clamp(current_local.z, band_ptr->height_min_m, band_ptr->height_max_m),
          };
          const bool apply_angle_correction = authoritative_.layout_settings.angle_correction_enabled &&
                                              pole->context.kind == PoleContextKind::kCorner &&
                                              band_ptr->side != SlotSide::kCenter;
          double applied_scale = 1.0;
          if (apply_angle_correction) {
            adjusted_local.y = apply_corner_side_scale(adjusted_local.y, band_ptr->side, pole->context.corner_turn_sign,
                                                       pole->context.side_scale);
            if (std::abs(current_local.y) > 1e-9) {
              applied_scale = std::abs(adjusted_local.y / current_local.y);
            }
          }
          adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, band_ptr->side);
          port->world_position =
              local_to_world_on_pole(pole->world_transform, effective_port_layout_yaw_deg(*pole, port->category),
                                     adjusted_local);
          port->angle_correction_applied = apply_angle_correction;
          port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
          apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBand);
          recomputed = true;
        }
      }
    }
  }
  if (!recomputed && port->placement_source == PortPlacementSourceKind::kManualEdit) {
    apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  }

  result.change_set.updated_ids.push_back(port_id);
  mark_connected_spans_dirty_from_port(port_id, DirtyBits::kGeometryRefresh, &result.change_set);
  result.ok = true;
  result.value = port_id;
  return result;
}

EditResult<ObjectId> CoreState::MoveAnchor(ObjectId anchor_id, const Vec3d& new_world_position) {
  EditResult<ObjectId> result;
  Anchor* anchor = authoritative_.edit_state.anchors.find(anchor_id);
  if (anchor == nullptr) {
    result.error = "anchor not found";
    return result;
  }
  anchor->world_position = new_world_position;
  result.change_set.updated_ids.push_back(anchor_id);
  mark_connected_spans_dirty_from_anchor(anchor_id, DirtyBits::kGeometryRefresh, &result.change_set);
  result.ok = true;
  result.value = anchor_id;
  return result;
}

EditResult<ObjectId> CoreState::SetPoleFlip180(ObjectId pole_id, bool flip_180) {
  EditResult<ObjectId> result;
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  PoleOrientationOverride next = authoritative_.override_state.pole_orientation_by_pole[pole_id];
  if (next.flip_180.has_value() && *next.flip_180 == flip_180) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  next.flip_180 = flip_180;
  authoritative_.override_state.pole_orientation_by_pole[pole_id] = next;
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::SetPoleManualYawOverride(ObjectId pole_id, double manual_yaw_deg) {
  EditResult<ObjectId> result;
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  if (!std::isfinite(manual_yaw_deg)) {
    result.error = "manual yaw must be finite";
    return result;
  }

  PoleOrientationOverride next = authoritative_.override_state.pole_orientation_by_pole[pole_id];
  if (next.manual_yaw_deg.has_value() && std::abs(*next.manual_yaw_deg - manual_yaw_deg) <= 1e-9) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  next.manual_yaw_deg = normalize_yaw_deg(manual_yaw_deg);
  authoritative_.override_state.pole_orientation_by_pole[pole_id] = next;
  pole->world_transform.rotation_euler_deg.z = *next.manual_yaw_deg;
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);
  runtime_.last_recalc_stats = ProcessDirtyQueues();

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::ClearPoleOrientationOverride(ObjectId pole_id) {
  EditResult<ObjectId> result;
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }

  const bool had_override = authoritative_.override_state.pole_orientation_by_pole.erase(pole_id) > 0;
  if (!had_override) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  if (const auto it = debug_.pole_orientation_debug_records.find(pole_id); it != debug_.pole_orientation_debug_records.end()) {
    const Vec3d forward = it->second.adopted_forward;
    if ((forward.x * forward.x + forward.y * forward.y + forward.z * forward.z) > 1e-12) {
      pole->world_transform.rotation_euler_deg.z =
          normalize_yaw_deg(std::atan2(forward.y, forward.x) * (180.0 / 3.14159265358979323846));
    }
  }
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);
  runtime_.last_recalc_stats = ProcessDirtyQueues();

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::SetSpanEndpointSocketOverride(ObjectId span_id, bool is_start_endpoint, int socket_id) {
  EditResult<ObjectId> result;
  Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  SpanEndpointOverride next = authoritative_.override_state.span_endpoint_by_span[span_id];
  std::optional<int>& slot = is_start_endpoint ? next.socket_a_id : next.socket_b_id;
  if (slot.has_value() && *slot == socket_id) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  slot = socket_id;
  authoritative_.override_state.span_endpoint_by_span[span_id] = next;
  mark_span_dirty(span_id, DirtyBits::kDecision, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::ClearSpanEndpointSocketOverride(ObjectId span_id, bool is_start_endpoint) {
  EditResult<ObjectId> result;
  Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  auto it = authoritative_.override_state.span_endpoint_by_span.find(span_id);
  bool changed = false;
  if (it != authoritative_.override_state.span_endpoint_by_span.end()) {
    std::optional<int>& slot = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    changed = slot.has_value();
    slot.reset();
    if (!it->second.socket_a_id.has_value() && !it->second.socket_b_id.has_value()) {
      authoritative_.override_state.span_endpoint_by_span.erase(it);
    }
  }
  if (!changed) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  mark_span_dirty(span_id, DirtyBits::kDecision, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::SetSpanBranchDownOffsetOverride(ObjectId span_id, double branch_down_offset_m) {
  EditResult<ObjectId> result;
  Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  if (!std::isfinite(branch_down_offset_m) || branch_down_offset_m < 0.0) {
    result.error = "branch down offset override must be finite and >= 0";
    return result;
  }
  SpanSupportOverride next = authoritative_.override_state.span_support_by_span[span_id];
  if (next.branch_down_offset_m.has_value() && std::abs(*next.branch_down_offset_m - branch_down_offset_m) <= 1e-9) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  next.branch_down_offset_m = branch_down_offset_m;
  authoritative_.override_state.span_support_by_span[span_id] = next;
  mark_span_dirty(span_id, DirtyBits::kDecision, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::ClearSpanBranchDownOffsetOverride(ObjectId span_id) {
  EditResult<ObjectId> result;
  Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  if (authoritative_.override_state.span_support_by_span.erase(span_id) == 0) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  mark_span_dirty(span_id, DirtyBits::kDecision, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  add_unique_id(result.change_set.dirty_span_ids, span_id);
  result.ok = true;
  result.value = span_id;
  return result;
}

EditResult<ObjectId> CoreState::SetPolePlacementMode(ObjectId pole_id, PlacementMode mode) {
  EditResult<ObjectId> result;
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
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

void CoreState::refresh_owned_endpoints_from_pole(ObjectId pole_id, ChangeSet* change_set, const Pole* previous_pole,
                                                  const PortLayoutYawOverride* previous_row_layout_yaw_override) {
  state_internal::EndpointRefreshService::RefreshOwnedEndpointsFromPole(*this, pole_id, change_set, previous_pole,
                                                                        previous_row_layout_yaw_override);
}

EditResult<ObjectId> CoreState::DeletePole(ObjectId pole_id) {
  EditResult<ObjectId> result;
  const Pole* pole = authoritative_.edit_state.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }

  std::vector<ObjectId> owned_ports{};
  if (const auto it = runtime_.relation_index.ports_by_pole.find(pole_id); it != runtime_.relation_index.ports_by_pole.end()) {
    owned_ports = it->second;
  }
  std::vector<ObjectId> owned_anchors{};
  if (const auto it = runtime_.relation_index.anchors_by_pole.find(pole_id); it != runtime_.relation_index.anchors_by_pole.end()) {
    owned_anchors = it->second;
  }

  for (ObjectId port_id : owned_ports) {
    const auto spans_it = runtime_.connection_index.spans_by_port.find(port_id);
    if (spans_it != runtime_.connection_index.spans_by_port.end() && !spans_it->second.empty()) {
      result.error = "pole still has connected spans";
      return result;
    }
  }
  for (ObjectId anchor_id : owned_anchors) {
    const auto spans_it = runtime_.connection_index.spans_by_anchor.find(anchor_id);
    if (spans_it != runtime_.connection_index.spans_by_anchor.end() && !spans_it->second.empty()) {
      result.error = "pole still has connected anchors";
      return result;
    }
  }

  for (ObjectId port_id : owned_ports) {
    runtime_.connection_index.spans_by_port.erase(port_id);
    if (authoritative_.edit_state.ports.remove(port_id)) {
      add_unique_id(result.change_set.deleted_ids, port_id);
    }
  }
  runtime_.relation_index.ports_by_pole.erase(pole_id);

  for (ObjectId anchor_id : owned_anchors) {
    runtime_.connection_index.spans_by_anchor.erase(anchor_id);
    if (authoritative_.edit_state.anchors.remove(anchor_id)) {
      add_unique_id(result.change_set.deleted_ids, anchor_id);
    }
  }
  runtime_.relation_index.anchors_by_pole.erase(pole_id);

  authoritative_.override_state.pole_orientation_by_pole.erase(pole_id);
  if (authoritative_.edit_state.poles.remove(pole_id)) {
    add_unique_id(result.change_set.deleted_ids, pole_id);
  }

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::DeleteSpan(ObjectId span_id) {
  EditResult<ObjectId> result;
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }

  const Span copy = *span;
  mark_topology_related_spans_for_ports_dirty({copy.port_a_id, copy.port_b_id}, copy.id,
                                              DirtyBits::kTopology | DirtyBits::kDecision, &result.change_set);
  remove_span_from_indexes(copy);
  authoritative_.edit_state.spans.remove(span_id);
  runtime_.span_runtime_states.erase(span_id);
  remove_span_from_caches(span_id);

  std::vector<ObjectId> remove_attachments{};
  if (const auto it = runtime_.relation_index.attachments_by_span.find(span_id); it != runtime_.relation_index.attachments_by_span.end()) {
    remove_attachments = it->second;
  }
  for (ObjectId attachment_id : remove_attachments) {
    authoritative_.edit_state.attachments.remove(attachment_id);
    add_unique_id(result.change_set.deleted_ids, attachment_id);
  }
  runtime_.relation_index.attachments_by_span.erase(span_id);

  result.ok = true;
  result.value = span_id;
  result.change_set.deleted_ids.push_back(span_id);
  return result;
}

EditResult<SplitSpanResult> CoreState::SplitSpan(ObjectId span_id, double t) {
  EditResult<SplitSpanResult> result;
  if (!(t > 0.0 && t < 1.0)) {
    result.error = "split t must be in (0, 1)";
    return result;
  }

  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  const Span old_span = *span;
  const Port* port_a = authoritative_.edit_state.ports.find(old_span.port_a_id);
  const Port* port_b = authoritative_.edit_state.ports.find(old_span.port_b_id);
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
  if (Port* split_port = authoritative_.edit_state.ports.find(add_port_result.value); split_port != nullptr) {
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
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
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
  if (std::abs(pole->height_m - pole_type->default_height_m) > 1e-12) {
    pole->height_m = pole_type->default_height_m;
    add_unique_id(result.change_set.updated_ids, pole_id);
  }

  auto recompute_band_local = [&](const PortPlacementBand& band, double* out_scale, bool* out_angle_correction) {
    Vec3d adjusted_local{0.0, band.lateral_center_m, band.height_center_m};
    const bool apply_angle_correction = authoritative_.layout_settings.angle_correction_enabled &&
                                        pole->context.kind == PoleContextKind::kCorner &&
                                        band.side != SlotSide::kCenter;
    double applied_scale = 1.0;
    if (apply_angle_correction) {
      adjusted_local.y =
          apply_corner_side_scale(adjusted_local.y, band.side, pole->context.corner_turn_sign, pole->context.side_scale);
      if (std::abs(band.lateral_center_m) > 1e-9) {
        applied_scale = std::abs(adjusted_local.y / band.lateral_center_m);
      }
    }
    adjusted_local.y = std::clamp(adjusted_local.y, band.lateral_min_m, band.lateral_max_m);
    adjusted_local.z = std::clamp(adjusted_local.z, band.height_min_m, band.height_max_m);
    adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, band.side);
    if (out_scale != nullptr) {
      *out_scale = applied_scale;
    }
    if (out_angle_correction != nullptr) {
      *out_angle_correction = apply_angle_correction;
    }
    return adjusted_local;
  };

  const auto owned_port_ids_it = runtime_.relation_index.ports_by_pole.find(pole_id);
  if (owned_port_ids_it != runtime_.relation_index.ports_by_pole.end()) {
    for (ObjectId port_id : owned_port_ids_it->second) {
      Port* existing_port = authoritative_.edit_state.ports.find(port_id);
      if (existing_port == nullptr || !existing_port->generated_from_template ||
          existing_port->position_mode != PortPositionMode::kAuto) {
        continue;
      }

      const PortPlacementBand* band_ptr = nullptr;
      for (const PortPlacementBand& band : pole_type->port_bands) {
        if (!band.enabled || band.category != existing_port->category || band.layer != existing_port->template_layer ||
            band.side != existing_port->template_side || band.role != existing_port->template_role) {
          continue;
        }
        if (band_ptr == nullptr || band.priority > band_ptr->priority ||
            (band.priority == band_ptr->priority && band.band_id < band_ptr->band_id)) {
          band_ptr = &band;
        }
      }
      if (band_ptr == nullptr) {
        continue;
      }

      double applied_scale = 1.0;
      bool apply_angle_correction = false;
      const Vec3d adjusted_local = recompute_band_local(*band_ptr, &applied_scale, &apply_angle_correction);
      const Vec3d world_position =
          local_to_world_on_pole(pole->world_transform,
                                 effective_port_layout_yaw_deg(*pole, existing_port->category), adjusted_local);
      if (LengthSquared(existing_port->world_position - world_position) > 1e-12 ||
          existing_port->angle_correction_applied != apply_angle_correction ||
          std::abs(existing_port->side_scale_applied - applied_scale) > 1e-12) {
        existing_port->world_position = world_position;
        existing_port->angle_correction_applied = apply_angle_correction;
        existing_port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
        apply_port_position_mode(*existing_port, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBand);
        add_unique_id(result.change_set.updated_ids, existing_port->id);
        mark_connected_spans_dirty_from_port(existing_port->id, DirtyBits::kGeometryRefresh, &result.change_set);
      }
    }
  }

  const double anchor_yaw = effective_pole_yaw_deg(*pole);
  std::vector<const Anchor*> anchors_on_pole;
  const auto anchor_ids_it = runtime_.relation_index.anchors_by_pole.find(pole_id);
  if (anchor_ids_it != runtime_.relation_index.anchors_by_pole.end()) {
    for (ObjectId anchor_id : anchor_ids_it->second) {
      const Anchor* anchor = authoritative_.edit_state.anchors.find(anchor_id);
      if (anchor != nullptr) {
        anchors_on_pole.push_back(anchor);
      }
    }
  }

  for (const PortPlacementBand& band : pole_type->port_bands) {
    if (!band.enabled || is_port_band_used(pole_id, band)) {
      continue;
    }
    double applied_scale = 1.0;
    bool apply_angle_correction = false;
    const Vec3d adjusted_local = recompute_band_local(band, &applied_scale, &apply_angle_correction);
    const Vec3d world_position =
        local_to_world_on_pole(pole->world_transform, effective_port_layout_yaw_deg(*pole, band.category), adjusted_local);
    EditResult<ObjectId> add_port_result = AddPort(pole_id, world_position, category_to_port_kind(band.category),
                                                   category_to_port_layer(band.category), band.local_direction);
    if (!add_port_result.ok) {
      result.error = add_port_result.error;
      return result;
    }
    Port* created = authoritative_.edit_state.ports.find(add_port_result.value);
    if (created != nullptr) {
      created->category = band.category;
      created->template_layer = band.layer;
      created->template_side = band.side;
      created->template_role = band.role;
      created->generated_from_template = true;
      created->generated_by_rule = true;
      created->angle_correction_applied = apply_angle_correction;
      created->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
      apply_port_position_mode(*created, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBand);
      add_unique_id(result.change_set.updated_ids, created->id);
    }
    append_change_set(result.change_set, add_port_result.change_set);
  }

  auto has_matching_anchor_hint = [&](const AnchorSlotTemplate& hint) -> bool {
    const PoleFrame frame = BuildPoleFrame(pole->world_transform, anchor_yaw);
    for (const Anchor* anchor : anchors_on_pole) {
      if (anchor == nullptr || anchor->support_kind != hint.usage) {
        continue;
      }
      const Vec3d local = WorldPointToLocal(frame, anchor->world_position);
      const Vec3d diff = local - hint.local_position;
      const double dist2 = Dot(diff, diff);
      if (dist2 <= 1e-6) {
        return true;
      }
    }
    return false;
  };

  for (const AnchorSlotTemplate& slot : pole_type->anchor_slots) {
    if (!slot.enabled) {
      continue;
    }
    if (has_matching_anchor_hint(slot)) {
      continue;
    }

    const Vec3d world_position =
        local_to_world_on_pole(pole->world_transform, effective_pole_yaw_deg(*pole), slot.local_position);
    EditResult<ObjectId> add_anchor_result = AddAnchor(pole_id, world_position, slot.usage, 1.0);
    if (!add_anchor_result.ok) {
      result.error = add_anchor_result.error;
      return result;
    }
    Anchor* created = authoritative_.edit_state.anchors.find(add_anchor_result.value);
    if (created != nullptr) {
      created->generated_from_template = true;
      add_unique_id(result.change_set.updated_ids, created->id);
      anchors_on_pole.push_back(created);
    }
    append_change_set(result.change_set, add_anchor_result.change_set);
  }

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<AddConnectionByPoleResult>
CoreState::AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id, ConnectionCategory category) {
  return AddConnectionByPole(pole_a_id, pole_b_id, category, AddConnectionByPoleOptions{});
}

EditResult<AddConnectionByPoleResult>
CoreState::AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id, ConnectionCategory category,
                               const AddConnectionByPoleOptions& options) {
  EditResult<AddConnectionByPoleResult> result;
  if (authoritative_.edit_state.poles.find(pole_a_id) == nullptr || authoritative_.edit_state.poles.find(pole_b_id) == nullptr) {
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
    const Bundle* existing_bundle = authoritative_.edit_state.bundles.find(options.bundle_id);
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

  const Pole* pole_a = authoritative_.edit_state.poles.find(pole_a_id);
  const Pole* pole_b = authoritative_.edit_state.poles.find(pole_b_id);
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

  auto resolve_port = [&](ObjectId pole_id, ObjectId preferred_port_id,
                          const Port* preferred_template_port) -> EditResult<ObjectId> {
    if (preferred_port_id != kInvalidObjectId) {
      const Port* preferred_port = authoritative_.edit_state.ports.find(preferred_port_id);
      if (preferred_port != nullptr && preferred_port->owner_pole_id == pole_id &&
          (preferred_port->category == resolved_category ||
           preferred_port->layer == category_to_port_layer(resolved_category))) {
        EditResult<ObjectId> preferred_result;
        preferred_result.ok = true;
        preferred_result.value = preferred_port_id;
        return preferred_result;
      }
    }
    PortResolutionRequest request{};
    request.pole_id = pole_id;
    request.peer_pole_id = (pole_id == pole_a_id) ? pole_b_id : pole_a_id;
    request.reference_span_id = options.reference_span_id;
    request.category = resolved_category;
    request.connection_context = options.connection_context;
    request.pole_context = (pole_id == pole_a_id) ? pole_context_a : pole_context_b;
    request.corner_angle_deg = (pole_id == pole_a_id) ? corner_angle_a : corner_angle_b;
    request.corner_turn_sign = (pole_id == pole_a_id) ? corner_turn_sign_a : corner_turn_sign_b;
    request.allow_generate_port = options.allow_generate_port;
    if (preferred_template_port != nullptr) {
      request.prefer_template_match = true;
      request.preferred_template_layer = preferred_template_port->template_layer;
      request.preferred_template_side = preferred_template_port->template_side;
      request.preferred_template_role = preferred_template_port->template_role;
    }
    request.branch_index = options.branch_index;
    return ensure_pole_connection_port(request);
  };

  EditResult<ObjectId> port_a_result = resolve_port(pole_a_id, options.preferred_port_a_id, nullptr);
  if (!port_a_result.ok) {
    result.error = port_a_result.error;
    return result;
  }
  const Port* preferred_template_port = authoritative_.edit_state.ports.find(port_a_result.value);
  EditResult<ObjectId> port_b_result = resolve_port(pole_b_id, options.preferred_port_b_id, preferred_template_port);
  if (!port_b_result.ok) {
    result.error = port_b_result.error;
    return result;
  }

  EditResult<ObjectId> bundle_result = ensure_bundle_for_template(options);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }

  const SpanKind span_kind = (options.span_kind == SpanKind::kGeneric)
                                 ? generation::detail::DefaultSpanKindForCategory(resolved_category)
                                 : options.span_kind;
  EditResult<ObjectId> span_result = AddSpan(port_a_result.value, port_b_result.value, span_kind,
                                             resolved_span_layer, bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
  }
  Span* created_span = authoritative_.edit_state.spans.find(span_result.value);
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
  append_change_set(result.change_set, port_a_result.change_set);
  append_change_set(result.change_set, port_b_result.change_set);
  append_change_set(result.change_set, bundle_result.change_set);
  append_change_set(result.change_set, span_result.change_set);
  const auto ensure_attachments = ensure_default_endpoint_attachments_for_span(result.value.span_id);
  if (!ensure_attachments.ok) {
    result.ok = false;
    result.error = ensure_attachments.error;
    return result;
  }
  append_change_set(result.change_set, ensure_attachments.change_set);
  return result;
}

EditResult<AddDropResult>
CoreState::AddDropFromPole(ObjectId source_pole_id, const Vec3d& target_world_position, ConnectionCategory category) {
  EditResult<AddDropResult> result;
  if (authoritative_.edit_state.poles.find(source_pole_id) == nullptr) {
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

  PortResolutionRequest request{};
  request.pole_id = source_pole_id;
  request.category = resolved_category;
  request.connection_context = ConnectionContext::kDropAdd;
  const Pole* source_pole = authoritative_.edit_state.poles.find(source_pole_id);
  request.pole_context = (source_pole == nullptr) ? PoleContextKind::kTerminal : source_pole->context.kind;
  request.corner_angle_deg = (source_pole == nullptr) ? 0.0 : source_pole->context.corner_angle_deg;
  request.corner_turn_sign = (source_pole == nullptr) ? 0.0 : source_pole->context.corner_turn_sign;
  request.allow_generate_port = true;
  EditResult<ObjectId> source_port_result = ensure_pole_connection_port(request);
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
  Span* created_span = authoritative_.edit_state.spans.find(span_result.value);
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
  const auto ensure_attachments = ensure_default_endpoint_attachments_for_span(result.value.span_id);
  if (!ensure_attachments.ok) {
    result.ok = false;
    result.error = ensure_attachments.error;
    return result;
  }
  append_change_set(result.change_set, ensure_attachments.change_set);
  return result;
}

EditResult<AddDropResult> CoreState::AddDropFromSpan(ObjectId source_span_id, double t,
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
  Span* created_span = authoritative_.edit_state.spans.find(span_result.value);
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
  const auto ensure_attachments = ensure_default_endpoint_attachments_for_span(result.value.span_id);
  if (!ensure_attachments.ok) {
    result.ok = false;
    result.error = ensure_attachments.error;
    return result;
  }
  append_change_set(result.change_set, ensure_attachments.change_set);
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

  const bool changed = normalized.curve_samples != runtime_.cache_state.geometry_settings.curve_samples ||
                       normalized.sag_enabled != runtime_.cache_state.geometry_settings.sag_enabled ||
                       std::abs(normalized.sag_factor - runtime_.cache_state.geometry_settings.sag_factor) > 1e-12 ||
                       std::abs(normalized.pole_clearance_m - runtime_.cache_state.geometry_settings.pole_clearance_m) > 1e-12;

  runtime_.cache_state.geometry_settings = normalized;
  result.ok = true;
  result.value = changed;

  if (changed && mark_all_spans_dirty) {
    for (const Span& span : authoritative_.edit_state.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kGeometryRefresh, true);
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

  const bool changed = normalized.angle_correction_enabled != authoritative_.layout_settings.angle_correction_enabled ||
                       std::abs(normalized.corner_threshold_deg - authoritative_.layout_settings.corner_threshold_deg) > 1e-9 ||
                       std::abs(normalized.min_side_scale - authoritative_.layout_settings.min_side_scale) > 1e-9 ||
                       std::abs(normalized.max_side_scale - authoritative_.layout_settings.max_side_scale) > 1e-9;

  authoritative_.layout_settings = normalized;
  result.ok = true;
  result.value = changed;
  return result;
}

EditResult<bool> CoreState::UpdateVisualSettings(const VisualSettings& settings, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  VisualSettings normalized = settings;
  normalized.support_center_threshold_m = std::max(0.0, normalized.support_center_threshold_m);
  normalized.support_arm_extra_m = std::max(0.0, normalized.support_arm_extra_m);
  normalized.support_arm_radius_m = std::max(0.0, normalized.support_arm_radius_m);
  normalized.insulator_radius_m = std::max(0.0, normalized.insulator_radius_m);
  normalized.insulator_length_m = std::max(0.0, normalized.insulator_length_m);

  const bool changed = normalized.enable_support_structures != runtime_.cache_state.visual_settings.enable_support_structures ||
                       normalized.enable_insulators != runtime_.cache_state.visual_settings.enable_insulators ||
                       std::abs(normalized.support_center_threshold_m -
                                runtime_.cache_state.visual_settings.support_center_threshold_m) > 1e-12 ||
                       std::abs(normalized.support_arm_extra_m - runtime_.cache_state.visual_settings.support_arm_extra_m) > 1e-12 ||
                       std::abs(normalized.support_arm_radius_m - runtime_.cache_state.visual_settings.support_arm_radius_m) > 1e-12 ||
                       std::abs(normalized.insulator_radius_m - runtime_.cache_state.visual_settings.insulator_radius_m) > 1e-12 ||
                       std::abs(normalized.insulator_length_m - runtime_.cache_state.visual_settings.insulator_length_m) > 1e-12;

  runtime_.cache_state.visual_settings = normalized;
  result.ok = true;
  result.value = changed;
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : authoritative_.edit_state.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kRenderRefresh, true);
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

  const VariationSettings& current = runtime_.cache_state.variation_settings;
  const bool changed =
      normalized.enabled != current.enabled || normalized.global_seed != current.global_seed ||
      std::abs(normalized.world_cell_size_m - current.world_cell_size_m) > 1e-12 ||
      std::abs(normalized.world_bias_scale - current.world_bias_scale) > 1e-12 ||
      std::abs(normalized.flow_bias_scale - current.flow_bias_scale) > 1e-12 ||
      std::abs(normalized.pole_delta_scale - current.pole_delta_scale) > 1e-12 ||
      std::abs(normalized.local_jitter_scale - current.local_jitter_scale) > 1e-12 ||
      std::abs(normalized.sag_variation_scale - current.sag_variation_scale) > 1e-12 ||
      std::abs(normalized.branch_down_offset_variation_scale - current.branch_down_offset_variation_scale) > 1e-12;

  runtime_.cache_state.variation_settings = normalized;
  result.ok = true;
  result.value = changed;
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : authoritative_.edit_state.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kDecision, true);
      add_unique_id(result.change_set.dirty_span_ids, span.id);
      add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  return result;
}

EditResult<bool> CoreState::UpdateContextProfile(const ContextProfile& profile, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  ContextProfile normalized = profile;
  normalized.age = std::clamp(normalized.age, 0.0, 1.0);
  normalized.clutter = std::clamp(normalized.clutter, 0.0, 1.0);
  normalized.regularity = std::clamp(normalized.regularity, 0.0, 1.0);
  normalized.service_mix = std::clamp(normalized.service_mix, 0.0, 1.0);
  normalized.style_seed = (normalized.style_seed == 0) ? 1 : normalized.style_seed;

  const ContextProfile& current = authoritative_.context_profile;
  const bool changed = std::abs(normalized.age - current.age) > 1e-12 ||
                       std::abs(normalized.clutter - current.clutter) > 1e-12 ||
                       std::abs(normalized.regularity - current.regularity) > 1e-12 ||
                       std::abs(normalized.service_mix - current.service_mix) > 1e-12 ||
                       normalized.style_seed != current.style_seed;

  authoritative_.context_profile = normalized;
  result.ok = true;
  result.value = changed;
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : authoritative_.edit_state.spans.items()) {
      mark_span_dirty(span.id, DirtyBits::kDecision, true);
      add_unique_id(result.change_set.dirty_span_ids, span.id);
      add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  return result;
}

EditResult<bool> CoreState::UpdateCableTemplate(const CableTemplate& cable_template) {
  static const std::vector<ObjectId> kNoPreferredVisibleSpans{};
  return UpdateCableTemplate(cable_template, kNoPreferredVisibleSpans);
}

EditResult<bool> CoreState::UpdateCableTemplate(const CableTemplate& cable_template,
                                                const std::vector<ObjectId>& preferred_visible_span_ids) {
  return state_internal::TemplateMutationService::UpdateCableTemplate(*this, cable_template, preferred_visible_span_ids);
}

EditResult<bool> CoreState::UpdatePoleTypeDefinition(const PoleTypeDefinition& pole_type) {
  return update_pole_type_and_refresh_instances(pole_type);
}

EditResult<bool> CoreState::UpdateBundleTemplate(const BundleTemplate& bundle_template) {
  return state_internal::TemplateMutationService::UpdateBundleTemplate(*this, bundle_template);
}

EditResult<bool> CoreState::ApplyBundleRelatedPoleTypeToExistingPoles(BundleKind bundle_template_id) {
  EditResult<bool> result;
  const BundleTemplate* bundle_template = find_bundle_template(bundle_template_id);
  if (bundle_template == nullptr) {
    result.error = "bundle template not found";
    return result;
  }
  if (bundle_template->related_pole_type_id == kInvalidPoleTypeId) {
    result.ok = true;
    result.value = false;
    return result;
  }
  if (find_pole_type(bundle_template->related_pole_type_id) == nullptr) {
    result.error = "related pole type not found";
    return result;
  }

  std::unordered_set<ObjectId> target_pole_ids;
  for (const Bundle& bundle : authoritative_.edit_state.bundles.items()) {
    if (bundle.bundle_template_id != bundle_template_id) {
      continue;
    }
    const auto spans_it = runtime_.relation_index.spans_by_bundle.find(bundle.id);
    if (spans_it == runtime_.relation_index.spans_by_bundle.end()) {
      continue;
    }
    for (ObjectId span_id : spans_it->second) {
      const Span* span = authoritative_.edit_state.spans.find(span_id);
      if (span == nullptr) {
        continue;
      }
      const Port* port_a = authoritative_.edit_state.ports.find(span->port_a_id);
      const Port* port_b = authoritative_.edit_state.ports.find(span->port_b_id);
      if (port_a != nullptr && port_a->owner_pole_id != kInvalidObjectId) {
        target_pole_ids.insert(port_a->owner_pole_id);
      }
      if (port_b != nullptr && port_b->owner_pole_id != kInvalidObjectId) {
        target_pole_ids.insert(port_b->owner_pole_id);
      }
    }
  }

  result.ok = true;
  result.value = false;
  for (ObjectId pole_id : target_pole_ids) {
    const auto apply = ApplyPoleType(pole_id, bundle_template->related_pole_type_id);
    if (!apply.ok) {
      result.ok = false;
      result.error = apply.error;
      return result;
    }
    append_change_set(result.change_set, apply.change_set);
    result.value = true;
  }
  return result;
}

EditResult<bool> CoreState::UpdateAttachmentTemplate(const AttachmentTemplate& attachment_template,
                                                     bool mark_dependent_spans_dirty) {
  return state_internal::TemplateMutationService::UpdateAttachmentTemplate(*this, attachment_template,
                                                                          mark_dependent_spans_dirty);
}

EditResult<bool> CoreState::ResetAllSpanReferenceLengths(bool mark_all_spans_dirty) {
  return state_internal::TemplateMutationService::ResetAllSpanReferenceLengths(*this, mark_all_spans_dirty);
}

EditResult<bool> CoreState::ensure_default_endpoint_attachments_for_span(ObjectId span_id) {
  EditResult<bool> result;
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    result.error = "span not found";
    return result;
  }
  const Bundle* bundle = authoritative_.edit_state.bundles.find(span->bundle_id);
  if (bundle == nullptr) {
    result.ok = true;
    result.value = false;
    return result;
  }
  const BundleTemplate* bundle_template = find_bundle_template(bundle->bundle_template_id);
  if (bundle_template == nullptr) {
    result.error = "bundle template not found";
    return result;
  }
  const CableTemplate* cable_template = find_cable_template(bundle_template->cable_template_id);
  if (cable_template == nullptr || cable_template->default_endpoint_attachment_template_id == kInvalidAttachmentTemplateId) {
    result.ok = true;
    result.value = false;
    return result;
  }
  const AttachmentTemplate* attachment_template =
      find_attachment_template(cable_template->default_endpoint_attachment_template_id);
  if (attachment_template == nullptr) {
    result.error = "default endpoint attachment template not found";
    return result;
  }

  Span* span_edit = authoritative_.edit_state.spans.find(span_id);
  if (span_edit == nullptr) {
    result.error = "span not found";
    return result;
  }

  auto ensure_endpoint_attachment = [&](bool is_start_endpoint, double t) -> bool {
    ObjectId& attachment_slot = is_start_endpoint ? span_edit->endpoint_attachment_a_id : span_edit->endpoint_attachment_b_id;
    if (attachment_slot != kInvalidObjectId) {
      return true;
    }
    const auto add_attachment =
        AddAttachment(span_id, t, attachment_template->kind, 0.0, cable_template->default_endpoint_attachment_template_id);
    if (!add_attachment.ok) {
      result.error = add_attachment.error;
      return false;
    }
    attachment_slot = add_attachment.value;
    append_change_set(result.change_set, add_attachment.change_set);
    add_unique_id(result.change_set.updated_ids, span_id);
    result.value = true;
    return true;
  };

  result.ok = ensure_endpoint_attachment(true, 0.0) && ensure_endpoint_attachment(false, 1.0);
  return result;
}

EditResult<bool> CoreState::update_pole_type_and_refresh_instances(const PoleTypeDefinition& pole_type) {
  EditResult<bool> result;
  auto it = authoritative_.pole_types.find(pole_type.id);
  if (it == authoritative_.pole_types.end()) {
    result.error = "pole type not found";
    return result;
  }
  if (pole_type_definition_equals(it->second, pole_type)) {
    result.ok = true;
    result.value = false;
    return result;
  }
  it->second = pole_type;
  result.ok = true;
  result.value = true;

  std::vector<ObjectId> pole_ids{};
  pole_ids.reserve(authoritative_.edit_state.poles.size());
  for (const Pole& pole : authoritative_.edit_state.poles.items()) {
    if (pole.pole_type_id == pole_type.id) {
      pole_ids.push_back(pole.id);
    }
  }
  for (ObjectId pole_id : pole_ids) {
    const auto apply = ApplyPoleType(pole_id, pole_type.id);
    if (!apply.ok) {
      result.ok = false;
      result.error = apply.error;
      return result;
    }
    append_change_set(result.change_set, apply.change_set);
    add_unique_id(result.change_set.updated_ids, pole_id);
  }
  return result;
}

bool CoreState::has_pole_orientation_override(ObjectId pole_id) const {
  return state_internal::OverrideResolutionService::HasPoleOrientationOverride(*this, pole_id);
}

bool CoreState::has_span_endpoint_socket_override(ObjectId span_id, bool is_start_endpoint) const {
  return state_internal::OverrideResolutionService::HasSpanEndpointSocketOverride(*this, span_id, is_start_endpoint);
}

bool CoreState::has_span_branch_down_offset_override(ObjectId span_id) const {
  return state_internal::OverrideResolutionService::HasSpanBranchDownOffsetOverride(*this, span_id);
}

std::optional<double> CoreState::resolve_pole_manual_yaw_override(const Pole& pole) const {
  return state_internal::OverrideResolutionService::ResolvePoleManualYawOverride(*this, pole);
}

std::optional<bool> CoreState::resolve_pole_flip_180_override(const Pole& pole) const {
  return state_internal::OverrideResolutionService::ResolvePoleFlip180Override(*this, pole);
}

int CoreState::resolve_span_endpoint_socket_id(const Span& span, bool is_start_endpoint) const {
  return state_internal::OverrideResolutionService::ResolveSpanEndpointSocketId(*this, span, is_start_endpoint);
}

double CoreState::resolve_span_branch_down_offset_m(const Span& span, double automatic_value) const {
  return state_internal::OverrideResolutionService::ResolveSpanBranchDownOffsetM(*this, span, automatic_value);
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

double CoreState::effective_pole_layout_yaw_deg(const Pole& pole) const {
  return effective_pole_yaw_deg(pole);
}

double CoreState::effective_port_layout_yaw_deg(const Pole& pole, ConnectionCategory category,
                                                const PortLayoutYawOverride* row_layout_yaw_override) const {
  if (has_pole_orientation_override(pole.id)) {
    return effective_pole_yaw_deg(pole);
  }
  if (row_layout_yaw_override != nullptr && row_layout_yaw_override->category == category) {
    return normalize_yaw_deg(row_layout_yaw_override->yaw_deg);
  }
  if (const auto it = debug_.pole_orientation_debug_records.find(pole.id); it != debug_.pole_orientation_debug_records.end()) {
    const bool uses_support_axis_layout =
        it->second.row_layout_axis_mode == RowLayoutAxisMode::kSupportAxis &&
        it->second.row_layout_axis_category == category;
    if (!uses_support_axis_layout) {
      return effective_pole_yaw_deg(pole);
    }
    Vec3d support_axis = it->second.adopted_support_axis;
    if (Normalize(&support_axis)) {
      return normalize_yaw_deg(std::atan2(support_axis.y, support_axis.x) * (180.0 / 3.14159265358979323846) - 90.0);
    }
  }
  return effective_pole_yaw_deg(pole);
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
  if (!authoritative_.layout_settings.angle_correction_enabled || context != PoleContextKind::kCorner) {
    return 1.0;
  }
  const double threshold = std::max(0.0, authoritative_.layout_settings.corner_threshold_deg);
  if (corner_angle_deg <= threshold + 1e-9) {
    return 1.0;
  }
  const double denom = std::max(1e-6, 180.0 - threshold);
  const double normalized = std::clamp((corner_angle_deg - threshold) / denom, 0.0, 1.0);
  const double scale = authoritative_.layout_settings.min_side_scale +
                       (authoritative_.layout_settings.max_side_scale - authoritative_.layout_settings.min_side_scale) * normalized;
  return std::clamp(scale, authoritative_.layout_settings.min_side_scale, authoritative_.layout_settings.max_side_scale);
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
  if (info.corner_angle_deg >= authoritative_.layout_settings.corner_threshold_deg) {
    info.kind = PoleContextKind::kCorner;
    info.side_scale = compute_side_scale(info.kind, info.corner_angle_deg);
    info.angle_correction_applied = authoritative_.layout_settings.angle_correction_enabled;
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
  std::uint64_t& serial = identity_.display_id_counters[std::string(prefix)];
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
  if (any(bits, DirtyBits::kDecision))
    text += "Decision|";
  if (any(bits, DirtyBits::kGeometryRefresh))
    text += "GeometryRefresh|";
  if (any(bits, DirtyBits::kBounds))
    text += "Bounds|";
  if (any(bits, DirtyBits::kRenderRefresh))
    text += "RenderRefresh|";
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
  const double min_offset = pole_radius_at_height_m(pole, std::max(0.0, adjusted.z)) + runtime_.cache_state.geometry_settings.pole_clearance_m;
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
    AddConnectionByPoleOptions options{};
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
    case ConnectionCategory::kDrop:
      options.bundle_template_id = BundleKind::kDrop;
      break;
    case ConnectionCategory::kLowVoltage:
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
