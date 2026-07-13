#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "../collection_utils.hpp"
#include "internal_services.hpp"
#include "port_placement.hpp"
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

struct PoleTiltResolution {
  double magnitude_deg = 0.0;
  Vec3d rotation_euler_xy_deg{};
};

PoleTiltResolution resolve_pole_tilt_from_pull(ObjectId pole_id, double max_tilt_deg, double layout_yaw_deg,
                                               const Vec3d& pull_world_dir, std::size_t incident_span_count) {
  const double clamped_max_tilt_deg = std::clamp(max_tilt_deg, 0.0, 45.0);
  const double random_tilt_factor = deterministic_pole_tilt_factor(pole_id);
  const Vec3d random_world_dir = unit_xy_from_azimuth_deg(deterministic_pole_tilt_azimuth_deg(pole_id));
  Vec3d resolved_pull = pull_world_dir;
  resolved_pull.z = 0.0;
  Vec3d tilt_world_dir = random_world_dir;
  double pull_strength = 0.0;
  if (incident_span_count > 0) {
    pull_strength = std::clamp(Length(resolved_pull) / static_cast<double>(incident_span_count), 0.0, 1.0);
  }
  if (NormalizeXY(&resolved_pull)) {
    const double pull_bias = 0.60 + 0.25 * pull_strength;
    tilt_world_dir = ScaleVec(resolved_pull, pull_bias) + ScaleVec(random_world_dir, 1.0 - pull_bias);
    if (!NormalizeXY(&tilt_world_dir)) {
      tilt_world_dir = resolved_pull;
    }
  }
  const double applied_tilt_scale = (incident_span_count > 0) ? (0.20 + 0.80 * pull_strength) : 1.0;
  const double applied_tilt_deg = clamped_max_tilt_deg * random_tilt_factor * applied_tilt_scale;
  const double local_azimuth_deg = NormalizeYawDeg(YawDegFromXY(tilt_world_dir) - layout_yaw_deg);
  return {applied_tilt_deg, tilt_euler_xy_from_local_polar_deg(applied_tilt_deg, local_azimuth_deg)};
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

using detail::append_unique;

void append_change_set(ChangeSet& dst, const ChangeSet& src) {
  append_unique(dst.created_ids, src.created_ids);
  append_unique(dst.updated_ids, src.updated_ids);
  append_unique(dst.deleted_ids, src.deleted_ids);
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

bool vec3_equals(const Vec3d& a, const Vec3d& b) {
  return std::abs(a.x - b.x) <= 1e-12 && std::abs(a.y - b.y) <= 1e-12 &&
         std::abs(a.z - b.z) <= 1e-12;
}

bool frame_equals(const Frame3d& a, const Frame3d& b) {
  return vec3_equals(a.origin, b.origin) && vec3_equals(a.forward, b.forward) &&
         vec3_equals(a.right, b.right) && vec3_equals(a.up, b.up);
}

bool port_band_placement_only_change(const PortPlacementBand& a, const PortPlacementBand& b) {
  return a.band_id == b.band_id && a.category == b.category && frame_equals(a.local_direction, b.local_direction) &&
         a.layer == b.layer && a.side == b.side && a.role == b.role && a.priority == b.priority &&
         std::abs(a.min_spacing_m - b.min_spacing_m) <= 1e-12 && a.allow_multiple == b.allow_multiple &&
         a.overflow_policy == b.overflow_policy && a.enabled == b.enabled;
}

bool pole_type_placement_only_change(const PoleTypeDefinition& before, const PoleTypeDefinition& after) {
  if (before.id != after.id || before.port_bands.size() != after.port_bands.size() ||
      before.anchor_slots.size() != after.anchor_slots.size()) {
    return false;
  }
  for (std::size_t i = 0; i < before.anchor_slots.size(); ++i) {
    if (!anchor_slot_equals(before.anchor_slots[i], after.anchor_slots[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < before.port_bands.size(); ++i) {
    if (!port_band_placement_only_change(before.port_bands[i], after.port_bands[i])) {
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

EditResult<ObjectId> CoreState::AddBundle(int conductor_count, double phase_spacing_m,
                                          BundleTemplateId bundle_template_id) {
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
  bundle.bundle_template_id = bundle_template_id;
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
  span.reference_length_m = Length(port_b->world_position - port_a->world_position);
  authoritative_.edit_state.spans.insert(span);

  add_span_to_index(span);
  initialize_span_runtime_state(span.id);
  touch_span(span.id, true);

  result.ok = true;
  result.value = span.id;
  result.change_set.created_ids.push_back(span.id);
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
  touch_span(span_id, true);

  result.ok = true;
  result.value = attachment.id;
  result.change_set.created_ids.push_back(attachment.id);
  result.change_set.updated_ids.push_back(span_id);
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
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPole, pole_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<bool> CoreState::apply_pole_tilt_from_pull(ObjectId pole_id, double max_tilt_deg,
                                                      const Vec3d& pull_world_dir,
                                                      std::size_t incident_span_count, ChangeSet* change_set) {
  EditResult<bool> result{};
  Pole* pole = authoritative_.edit_state.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }
  const PoleTiltResolution resolved =
      resolve_pole_tilt_from_pull(pole_id, max_tilt_deg, effective_pole_layout_yaw_deg(*pole), pull_world_dir,
                                  incident_span_count);
  if (std::abs(pole->tilt_magnitude_deg - resolved.magnitude_deg) <= 1e-9 &&
      std::abs(pole->world_transform.rotation_euler_deg.x - resolved.rotation_euler_xy_deg.x) <= 1e-9 &&
      std::abs(pole->world_transform.rotation_euler_deg.y - resolved.rotation_euler_xy_deg.y) <= 1e-9) {
    result.ok = true;
    result.value = false;
    return result;
  }
  pole->tilt_magnitude_deg = resolved.magnitude_deg;
  pole->world_transform.rotation_euler_deg.x = resolved.rotation_euler_xy_deg.x;
  pole->world_transform.rotation_euler_deg.y = resolved.rotation_euler_xy_deg.y;
  if (change_set != nullptr) {
    add_unique_id(change_set->updated_ids, pole_id);
  }
  result.ok = true;
  result.value = true;
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
  bool changed = false;
  std::vector<ObjectId> changed_poles{};
  for (ObjectId pole_id : targets) {
    Pole* pole = authoritative_.edit_state.poles.find(pole_id);
    if (pole == nullptr) {
      result.error = "pole not found";
      result.ok = false;
      return result;
    }
    const Pole old_pole = *pole;
    Vec3d pull_world_dir{};
    std::unordered_set<ObjectId> seen_spans{};
    std::size_t incident_span_count = 0;
    if (const auto ports_it = runtime_.relation_index.ports_by_pole.find(pole_id);
        ports_it != runtime_.relation_index.ports_by_pole.end()) {
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
          const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id
                                      : (span->port_b_id == port_id ? span->port_a_id : kInvalidObjectId);
          if (other_port_id == kInvalidObjectId) {
            continue;
          }
          const Port* other_port = authoritative_.edit_state.ports.find(other_port_id);
          if (other_port == nullptr) {
            continue;
          }
          Vec3d other_world = other_port->world_position;
          if (other_port->owner_pole_id != kInvalidObjectId) {
            if (const Pole* other_pole = authoritative_.edit_state.poles.find(other_port->owner_pole_id);
                other_pole != nullptr) {
              other_world = other_pole->world_transform.position;
            }
          }
          Vec3d span_dir = other_world - pole->world_transform.position;
          span_dir.z = 0.0;
          if (!NormalizeXY(&span_dir)) {
            continue;
          }
          pull_world_dir = pull_world_dir + span_dir;
          ++incident_span_count;
        }
      }
    }
    const auto applied = apply_pole_tilt_from_pull(pole_id, max_tilt_deg, pull_world_dir, incident_span_count,
                                                   &result.change_set);
    if (!applied.ok) {
      result.error = applied.error;
      result.ok = false;
      return result;
    }
    if (!applied.value) {
      continue;
    }
    finalize_pole_transform_update(pole_id, old_pole, &result.change_set);
    add_unique_id(changed_poles, pole_id);
    changed = true;
  }
  UpdatePlan combined_plan{};
  combined_plan.kind = UpdateKind::kReposition;
  auto append_unique = [](std::vector<ObjectId>* target, const std::vector<ObjectId>& source) {
    for (ObjectId id : source) {
      if (std::find(target->begin(), target->end(), id) == target->end()) {
        target->push_back(id);
      }
    }
  };
  for (ObjectId pole_id : changed_poles) {
    const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPole, pole_id});
    if (!plan.ok) {
      result.error = plan.error;
      return result;
    }
    append_unique(&combined_plan.affected.poles, plan.value.affected.poles);
    append_unique(&combined_plan.affected.ports, plan.value.affected.ports);
    append_unique(&combined_plan.affected.spans, plan.value.affected.spans);
    append_unique(&combined_plan.affected.edges, plan.value.affected.edges);
    combined_plan.plan_ms += plan.value.plan_ms;
  }
  if (!changed_poles.empty()) {
    const auto updated = execute_update_plan(combined_plan);
    if (!updated.ok) {
      result.error = updated.error;
      return result;
    }
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
  touch_connected_spans_from_port(port_id, &result.change_set);
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPort, port_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }
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

  const SavedBackbonePortBinding* backbone_binding = view().backbone_port_binding_for_port(port_id);
  const Pole* owner = port->owner_pole_id == kInvalidObjectId
                          ? nullptr
                          : authoritative_.edit_state.poles.find(port->owner_pole_id);
  const PoleTypeDefinition* owner_type =
      owner == nullptr ? nullptr : find_pole_type(owner->pole_type_id);
  const PortPlacementBand* resolved_band =
      owner_type == nullptr ? nullptr : state_internal::FindPortPlacementBandForPort(*this, *owner_type, *port);
  if (backbone_binding != nullptr && resolved_band == nullptr) {
    result.error = "backbone unsupported: saved placement band is missing from pole type";
    return result;
  }

  bool recomputed = false;
  if (owner != nullptr && resolved_band != nullptr &&
      (port->generated_from_template || backbone_binding != nullptr)) {
    const PortPlacementBand* band_ptr = resolved_band;
          const PoleFrame frame =
              BuildPoleFrame(owner->world_transform,
                             effective_port_layout_yaw_deg(*owner, port->id, port->category));
          const Vec3d current_local = WorldPointToLocal(frame, port->world_position);
          Vec3d adjusted_local{
              0.0,
              std::clamp(current_local.y, band_ptr->lateral_min_m, band_ptr->lateral_max_m),
              std::clamp(current_local.z, band_ptr->height_min_m, band_ptr->height_max_m),
          };
          const bool apply_angle_correction = authoritative_.layout_settings.angle_correction_enabled &&
                                              owner->context.kind == PoleContextKind::kCorner &&
                                              band_ptr->side != SlotSide::kCenter;
          double applied_scale = 1.0;
          if (apply_angle_correction) {
            adjusted_local.y = state_internal::apply_corner_side_scale(
                adjusted_local.y, band_ptr->side, owner->context.corner_turn_sign, owner->context.side_scale);
            if (std::abs(current_local.y) > 1e-9) {
              applied_scale = std::abs(adjusted_local.y / current_local.y);
            }
          }
          adjusted_local = apply_pole_clearance_to_local(*owner, adjusted_local, band_ptr->side);
          port->world_position =
              local_to_world_on_pole(owner->world_transform,
                                     effective_port_layout_yaw_deg(*owner, port->id, port->category),
                                     adjusted_local);
          port->angle_correction_applied = apply_angle_correction;
          port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
          apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBand);
          recomputed = true;
  }
  apply_port_position_mode(*port, PortPositionMode::kAuto, port->placement_source);
  if (!recomputed && port->placement_source == PortPlacementSourceKind::kManualEdit) {
    apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  }

  result.change_set.updated_ids.push_back(port_id);
  touch_connected_spans_from_port(port_id, &result.change_set);
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPort, port_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }
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
  touch_connected_spans_from_anchor(anchor_id, &result.change_set);
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
  if (!next.base_yaw_deg.has_value()) {
    next.base_yaw_deg = pole->world_transform.rotation_euler_deg.z;
  }
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
  if (!next.base_yaw_deg.has_value()) {
    next.base_yaw_deg = pole->world_transform.rotation_euler_deg.z;
  }
  if (next.manual_yaw_deg.has_value() && std::abs(*next.manual_yaw_deg - manual_yaw_deg) <= 1e-9) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const Pole old_pole = *pole;
  next.manual_yaw_deg = NormalizeYawDeg(manual_yaw_deg);
  authoritative_.override_state.pole_orientation_by_pole[pole_id] = next;
  pole->world_transform.rotation_euler_deg.z = *next.manual_yaw_deg;
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPole, pole_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }

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

  const auto override_it = authoritative_.override_state.pole_orientation_by_pole.find(pole_id);
  if (override_it == authoritative_.override_state.pole_orientation_by_pole.end()) {
    result.ok = true;
    result.value = pole_id;
    return result;
  }

  const PoleOrientationOverride previous_override = override_it->second;
  authoritative_.override_state.pole_orientation_by_pole.erase(override_it);
  const Pole old_pole = *pole;
  if (previous_override.base_yaw_deg.has_value()) {
    pole->world_transform.rotation_euler_deg.z = *previous_override.base_yaw_deg;
  }
  finalize_pole_transform_update(pole_id, old_pole, &result.change_set);
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPole, pole_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }

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
  SpanEndpointOverride next{};
  if (const auto existing = authoritative_.override_state.span_endpoint_by_span.find(span_id);
      existing != authoritative_.override_state.span_endpoint_by_span.end()) {
    next = existing->second;
  }
  std::optional<int>& slot = is_start_endpoint ? next.socket_a_id : next.socket_b_id;
  if (slot.has_value() && *slot == socket_id) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  if (runtime_.backbone_index.span_edge_bundle.contains(span_id)) {
    slot = socket_id;
    CoreState trial = *this;
    trial.authoritative_.override_state.span_endpoint_by_span[span_id] = next;
    ChangeSet regenerated_changes{};
    const auto regenerated = trial.regenerate_backbone_span_override(span_id, &regenerated_changes);
    if (!regenerated.ok) {
      result.error = regenerated.error;
      return result;
    }
    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    result.change_set = std::move(regenerated_changes);
    add_unique_id(result.change_set.updated_ids, span_id);
    result.ok = true;
    result.value = span_id;
    return result;
  }
  slot = socket_id;
  authoritative_.override_state.span_endpoint_by_span[span_id] = next;
  touch_span(span_id, true);
  add_unique_id(result.change_set.updated_ids, span_id);
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
  const bool changed = it != authoritative_.override_state.span_endpoint_by_span.end() &&
                       (is_start_endpoint ? it->second.socket_a_id.has_value()
                                          : it->second.socket_b_id.has_value());
  if (!changed) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  if (runtime_.backbone_index.span_edge_bundle.contains(span_id)) {
    CoreState trial = *this;
    auto trial_it = trial.authoritative_.override_state.span_endpoint_by_span.find(span_id);
    if (trial_it != trial.authoritative_.override_state.span_endpoint_by_span.end()) {
      std::optional<int>& trial_slot = is_start_endpoint ? trial_it->second.socket_a_id : trial_it->second.socket_b_id;
      trial_slot.reset();
      if (!trial_it->second.socket_a_id.has_value() && !trial_it->second.socket_b_id.has_value()) {
        trial.authoritative_.override_state.span_endpoint_by_span.erase(trial_it);
      }
    }
    ChangeSet regenerated_changes{};
    const auto regenerated = trial.regenerate_backbone_span_override(span_id, &regenerated_changes);
    if (!regenerated.ok) {
      result.error = regenerated.error;
      return result;
    }
    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    result.change_set = std::move(regenerated_changes);
    add_unique_id(result.change_set.updated_ids, span_id);
    result.ok = true;
    result.value = span_id;
    return result;
  }
  std::optional<int>& slot = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
  slot.reset();
  if (!it->second.socket_a_id.has_value() && !it->second.socket_b_id.has_value()) {
    authoritative_.override_state.span_endpoint_by_span.erase(it);
  }
  touch_span(span_id, true);
  add_unique_id(result.change_set.updated_ids, span_id);
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
  SpanSupportOverride next{};
  if (const auto existing = authoritative_.override_state.span_support_by_span.find(span_id);
      existing != authoritative_.override_state.span_support_by_span.end()) {
    next = existing->second;
  }
  if (next.branch_down_offset_m.has_value() && std::abs(*next.branch_down_offset_m - branch_down_offset_m) <= 1e-9) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  if (runtime_.backbone_index.span_edge_bundle.contains(span_id)) {
    next.branch_down_offset_m = branch_down_offset_m;
    CoreState trial = *this;
    trial.authoritative_.override_state.span_support_by_span[span_id] = next;
    ChangeSet regenerated_changes{};
    const auto regenerated = trial.regenerate_backbone_span_override(span_id, &regenerated_changes);
    if (!regenerated.ok) {
      result.error = regenerated.error;
      return result;
    }
    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    result.change_set = std::move(regenerated_changes);
    add_unique_id(result.change_set.updated_ids, span_id);
    result.ok = true;
    result.value = span_id;
    return result;
  }
  next.branch_down_offset_m = branch_down_offset_m;
  authoritative_.override_state.span_support_by_span[span_id] = next;
  touch_span(span_id, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kSpan, span_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }
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
  if (authoritative_.override_state.span_support_by_span.find(span_id) ==
      authoritative_.override_state.span_support_by_span.end()) {
    result.ok = true;
    result.value = span_id;
    return result;
  }
  if (runtime_.backbone_index.span_edge_bundle.contains(span_id)) {
    CoreState trial = *this;
    trial.authoritative_.override_state.span_support_by_span.erase(span_id);
    ChangeSet regenerated_changes{};
    const auto regenerated = trial.regenerate_backbone_span_override(span_id, &regenerated_changes);
    if (!regenerated.ok) {
      result.error = regenerated.error;
      return result;
    }
    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    result.change_set = std::move(regenerated_changes);
    add_unique_id(result.change_set.updated_ids, span_id);
    result.ok = true;
    result.value = span_id;
    return result;
  }
  authoritative_.override_state.span_support_by_span.erase(span_id);
  touch_span(span_id, true);
  add_unique_id(result.change_set.updated_ids, span_id);
  const auto plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kSpan, span_id});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
  const auto updated = execute_update_plan(plan.value);
  if (!updated.ok) {
    result.error = updated.error;
    return result;
  }
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
  touch_topology_related_spans_for_ports({copy.port_a_id, copy.port_b_id}, copy.id, &result.change_set);
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
  if (const auto ports_it = runtime_.relation_index.ports_by_pole.find(pole_id);
      ports_it != runtime_.relation_index.ports_by_pole.end()) {
    for (ObjectId port_id : ports_it->second) {
      const Port* port = authoritative_.edit_state.ports.find(port_id);
      if (port == nullptr || state_internal::FindPortPlacementBandForPort(*this, *pole_type, *port) != nullptr) {
        continue;
      }
      if (view().backbone_port_binding_for_port(port_id) != nullptr) {
        result.error = "backbone unsupported: saved placement band is missing from target pole type";
        return result;
      }
    }
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
          state_internal::apply_corner_side_scale(
              adjusted_local.y, band.side, pole->context.corner_turn_sign, pole->context.side_scale);
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

      const PortPlacementBand* band_ptr =
          state_internal::FindPortPlacementBandForPort(*this, *pole_type, *existing_port);
      if (band_ptr == nullptr) {
        continue;
      }

      double applied_scale = 1.0;
      bool apply_angle_correction = false;
      const Vec3d adjusted_local = recompute_band_local(*band_ptr, &applied_scale, &apply_angle_correction);
      const Vec3d world_position =
          local_to_world_on_pole(pole->world_transform,
                                 effective_port_layout_yaw_deg(*pole, existing_port->id, existing_port->category),
                                 adjusted_local);
      if (LengthSquared(existing_port->world_position - world_position) > 1e-12 ||
          existing_port->angle_correction_applied != apply_angle_correction ||
          std::abs(existing_port->side_scale_applied - applied_scale) > 1e-12) {
        existing_port->world_position = world_position;
        existing_port->angle_correction_applied = apply_angle_correction;
        existing_port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
        apply_port_position_mode(*existing_port, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBand);
        add_unique_id(result.change_set.updated_ids, existing_port->id);
        touch_connected_spans_from_port(existing_port->id, &result.change_set);
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
        local_to_world_on_pole(pole->world_transform,
                               effective_port_layout_yaw_deg(*pole, kInvalidObjectId, band.category),
                               adjusted_local);
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

EditResult<bool> CoreState::UpdateGeometrySettings(const GeometrySettings& settings, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  GeometrySettings normalized = settings;
  normalized.curve_samples = std::max(2, normalized.curve_samples);
  if (normalized.sag_factor < 0.0) {
    normalized.sag_factor = 0.0;
  }
  normalized.pole_clearance_m = std::max(0.0, normalized.pole_clearance_m);

  const bool changed = normalized.curve_samples != authoritative_.geometry_settings.curve_samples ||
                       normalized.sag_enabled != authoritative_.geometry_settings.sag_enabled ||
                       std::abs(normalized.sag_factor - authoritative_.geometry_settings.sag_factor) > 1e-12 ||
                       std::abs(normalized.pole_clearance_m - authoritative_.geometry_settings.pole_clearance_m) > 1e-12;

  EditResult<UpdatePlan> plan{};
  if (changed) {
    bool has_source_projection_endpoint = false;
    for (const SavedBackboneSpanBinding& binding : authoritative_.backbone.span_bindings) {
      const SpanLayoutRulesView rules = span_layout_rules(binding.span_id);
      if (!rules.has_rule()) {
        continue;
      }
      if (rules.rule->start.source_projection.valid() || rules.rule->end.source_projection.valid()) {
        has_source_projection_endpoint = true;
        break;
      }
    }
    plan = make_update_plan({has_source_projection_endpoint ? UpdateKind::kReposition : UpdateKind::kReshape,
                             UpdateTargetKind::kAllSpans, kInvalidObjectId});
    if (!plan.ok) {
      result.error = plan.error;
      return result;
    }
  }
  authoritative_.geometry_settings = normalized;
  result.ok = true;
  result.value = changed;

  if (changed) {
    const auto updated = execute_update_plan(plan.value);
    if (!updated.ok) {
      result.error = updated.error;
      result.ok = false;
      return result;
    }
    result.change_set.updated_ids = plan.value.affected.spans;
  }
  return result;
}

EditResult<bool> CoreState::UpdateLayoutSettings(const LayoutSettings& settings) {
  EditResult<bool> result;
  LayoutSettings normalized = settings;
  normalized.corner_threshold_deg = std::clamp(normalized.corner_threshold_deg, 0.0, 179.0);
  normalized.min_side_scale = std::clamp(normalized.min_side_scale, 0.5, kMaxCornerSideScale);
  normalized.max_side_scale =
      std::clamp(normalized.max_side_scale, normalized.min_side_scale, kMaxCornerSideScale);

  const bool changed = normalized.angle_correction_enabled != authoritative_.layout_settings.angle_correction_enabled ||
                       std::abs(normalized.corner_threshold_deg - authoritative_.layout_settings.corner_threshold_deg) > 1e-9 ||
                       std::abs(normalized.min_side_scale - authoritative_.layout_settings.min_side_scale) > 1e-9 ||
                       std::abs(normalized.max_side_scale - authoritative_.layout_settings.max_side_scale) > 1e-9;

  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  if (!authoritative_.backbone.span_bindings.empty()) {
    struct LayoutRegenerateScope {
      BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
      ObjectId bundle_id = kInvalidObjectId;
      std::vector<ObjectId> edge_bundle_ids{};
    };

    const SavedBackboneGraph& graph = view().backbone();
    auto edge_by_id = [&](ObjectId edge_id) -> const SavedBackboneEdge* {
      for (const SavedBackboneEdge& edge : graph.edges) {
        if (edge.edge_id == edge_id) {
          return &edge;
        }
      }
      return nullptr;
    };
    auto edge_id_in = [](const std::vector<const SavedBackboneEdge*>& edges, ObjectId edge_id) {
      return std::any_of(edges.begin(), edges.end(), [&](const SavedBackboneEdge* edge) {
        return edge != nullptr && edge->edge_id == edge_id;
      });
    };
    auto vector_has_id = [](const std::vector<ObjectId>& ids, ObjectId id) {
      return std::find(ids.begin(), ids.end(), id) != ids.end();
    };
    auto component_edges = [&](const SavedBackboneEdge& seed) {
      std::vector<const SavedBackboneEdge*> edges{&seed};
      auto find_adjacent = [&](const SavedBackboneEdge& anchor, bool forward,
                               bool* ambiguous) -> const SavedBackboneEdge* {
        const SavedBackboneEdge* match = nullptr;
        if (ambiguous != nullptr) {
          *ambiguous = false;
        }
        for (const SavedBackboneEdge& candidate : graph.edges) {
          if (candidate.edge_id == anchor.edge_id || candidate.route != seed.route) {
            continue;
          }
          const bool adjacent = forward ? (candidate.order == anchor.order + 1 && candidate.node_a == anchor.node_b)
                                        : (anchor.order == candidate.order + 1 && candidate.node_b == anchor.node_a);
          if (!adjacent) {
            continue;
          }
          if (match != nullptr) {
            if (ambiguous != nullptr) {
              *ambiguous = true;
            }
            return nullptr;
          }
          match = &candidate;
        }
        return match;
      };
      for (;;) {
        bool ambiguous = false;
        const SavedBackboneEdge* previous = find_adjacent(*edges.front(), false, &ambiguous);
        if (ambiguous) {
          result.error = "backbone unsupported: layout settings route adjacency is ambiguous";
          return std::vector<const SavedBackboneEdge*>{};
        }
        if (previous == nullptr) {
          break;
        }
        edges.insert(edges.begin(), previous);
      }
      for (;;) {
        bool ambiguous = false;
        const SavedBackboneEdge* next = find_adjacent(*edges.back(), true, &ambiguous);
        if (ambiguous) {
          result.error = "backbone unsupported: layout settings route adjacency is ambiguous";
          return std::vector<const SavedBackboneEdge*>{};
        }
        if (next == nullptr) {
          break;
        }
        edges.push_back(next);
      }
      return edges;
    };

    std::vector<LayoutRegenerateScope> scopes{};
    std::vector<ObjectId> covered_edge_bundle_ids{};
    for (const SavedBackboneEdgeBundle& seed_edge_bundle : graph.edge_bundles) {
      if (vector_has_id(covered_edge_bundle_ids, seed_edge_bundle.edge_bundle_id)) {
        continue;
      }
      const SavedBackboneEdge* seed_edge = edge_by_id(seed_edge_bundle.edge_id);
      const Bundle* seed_bundle = view().bundles().find(seed_edge_bundle.bundle_id);
      if (seed_edge == nullptr || seed_bundle == nullptr) {
        result.error = "backbone regenerate: layout settings scope is incomplete";
        return result;
      }
      std::vector<const SavedBackboneEdge*> edges = component_edges(*seed_edge);
      if (!result.error.empty()) {
        return result;
      }
      LayoutRegenerateScope scope{};
      scope.bundle_template_id = seed_bundle->bundle_template_id;
      scope.bundle_id = seed_edge_bundle.bundle_id;
      for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
        const Bundle* bundle = view().bundles().find(edge_bundle.bundle_id);
        if (bundle != nullptr && edge_bundle.bundle_id == scope.bundle_id &&
            bundle->bundle_template_id == scope.bundle_template_id &&
            edge_id_in(edges, edge_bundle.edge_id)) {
          if (!vector_has_id(scope.edge_bundle_ids, edge_bundle.edge_bundle_id)) {
            scope.edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
          }
          if (!vector_has_id(covered_edge_bundle_ids, edge_bundle.edge_bundle_id)) {
            covered_edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
          }
        }
      }
      if (scope.edge_bundle_ids.empty()) {
        result.error = "backbone regenerate: layout settings scope has no edge bundles";
        return result;
      }
      scopes.push_back(std::move(scope));
    }

    CoreState trial = *this;
    trial.authoritative_.layout_settings = normalized;
    ChangeSet regenerated_changes{};
    for (const LayoutRegenerateScope& scope : scopes) {
      const auto bundle_template_it = trial.authoritative_.bundle_templates.find(scope.bundle_template_id);
      if (bundle_template_it == trial.authoritative_.bundle_templates.end()) {
        result.error = "bundle template not found";
        return result;
      }
      const BundleTemplate previous = bundle_template_it->second;
      const auto regenerated = trial.regenerate_backbone_edge_bundles(scope.bundle_template_id, previous, previous,
                                                                      &regenerated_changes, nullptr,
                                                                      &scope.edge_bundle_ids, nullptr,
                                                                      BackboneRegenerateCause::kLayoutSettings);
      if (!regenerated.ok) {
        result.error = regenerated.error;
        return result;
      }
    }
    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    result.change_set = std::move(regenerated_changes);
    result.ok = true;
    result.value = true;
    return result;
  }

  authoritative_.layout_settings = normalized;
  result.ok = true;
  result.value = true;
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

  const bool changed = normalized.enable_support_structures != authoritative_.visual_settings.enable_support_structures ||
                       normalized.enable_insulators != authoritative_.visual_settings.enable_insulators ||
                       std::abs(normalized.support_center_threshold_m -
                                authoritative_.visual_settings.support_center_threshold_m) > 1e-12 ||
                       std::abs(normalized.support_arm_extra_m - authoritative_.visual_settings.support_arm_extra_m) > 1e-12 ||
                       std::abs(normalized.support_arm_radius_m - authoritative_.visual_settings.support_arm_radius_m) > 1e-12 ||
                       std::abs(normalized.insulator_radius_m - authoritative_.visual_settings.insulator_radius_m) > 1e-12 ||
                       std::abs(normalized.insulator_length_m - authoritative_.visual_settings.insulator_length_m) > 1e-12;

  EditResult<UpdatePlan> plan{};
  if (changed) {
    plan = make_update_plan({UpdateKind::kRedraw, UpdateTargetKind::kAllSpans, kInvalidObjectId});
    if (!plan.ok) {
      result.error = plan.error;
      return result;
    }
  }
  authoritative_.visual_settings = normalized;
  result.ok = true;
  result.value = changed;
  if (changed) {
    const auto updated = execute_update_plan(plan.value);
    if (!updated.ok) {
      result.error = updated.error;
      result.ok = false;
      return result;
    }
    result.change_set.updated_ids = plan.value.affected.spans;
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

  const VariationSettings& current = authoritative_.variation_settings;
  const bool changed =
      normalized.enabled != current.enabled || normalized.global_seed != current.global_seed ||
      std::abs(normalized.world_cell_size_m - current.world_cell_size_m) > 1e-12 ||
      std::abs(normalized.world_bias_scale - current.world_bias_scale) > 1e-12 ||
      std::abs(normalized.flow_bias_scale - current.flow_bias_scale) > 1e-12 ||
      std::abs(normalized.pole_delta_scale - current.pole_delta_scale) > 1e-12 ||
      std::abs(normalized.local_jitter_scale - current.local_jitter_scale) > 1e-12 ||
      std::abs(normalized.sag_variation_scale - current.sag_variation_scale) > 1e-12 ||
      std::abs(normalized.branch_down_offset_variation_scale - current.branch_down_offset_variation_scale) > 1e-12;

  if (changed && !authoritative_.backbone.span_bindings.empty()) {
    result.error = "backbone unsupported: variation settings are not consumed by generated outputs";
    return result;
  }
  authoritative_.variation_settings = normalized;
  result.ok = true;
  result.value = changed;
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

  if (changed && !authoritative_.backbone.span_bindings.empty()) {
    result.error = "backbone unsupported: context profile is not consumed by generated outputs";
    return result;
  }
  authoritative_.context_profile = normalized;
  result.ok = true;
  result.value = changed;
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

EditResult<bool> CoreState::ApplyBundleRelatedPoleTypeToExistingPoles(BundleTemplateId bundle_template_id) {
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
  std::vector<ObjectId> ordered_target_pole_ids(target_pole_ids.begin(), target_pole_ids.end());
  std::sort(ordered_target_pole_ids.begin(), ordered_target_pole_ids.end());

  std::vector<ObjectId> active_backbone_pole_ids{};
  for (ObjectId pole_id : ordered_target_pole_ids) {
    if (runtime_.backbone_index.pole_node.contains(pole_id)) {
      active_backbone_pole_ids.push_back(pole_id);
    }
  }

  if (!active_backbone_pole_ids.empty()) {
    struct RelatedPoleRegenerateScope {
      BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
      std::size_t route = 0;
      ObjectId bundle_id = kInvalidObjectId;
      std::vector<ObjectId> edge_bundle_ids{};
    };

    std::vector<RelatedPoleRegenerateScope> scopes{};
    auto add_scope_seed = [&](BundleTemplateId scoped_template_id, std::size_t route, ObjectId bundle_id) {
      const auto existing = std::find_if(scopes.begin(), scopes.end(), [&](const RelatedPoleRegenerateScope& scope) {
        return scope.bundle_template_id == scoped_template_id && scope.route == route && scope.bundle_id == bundle_id;
      });
      if (existing == scopes.end()) {
        RelatedPoleRegenerateScope scope{};
        scope.bundle_template_id = scoped_template_id;
        scope.route = route;
        scope.bundle_id = bundle_id;
        scopes.push_back(std::move(scope));
      }
    };

    for (ObjectId pole_id : active_backbone_pole_ids) {
      const BackboneFrontier frontier = view().pole_frontier(pole_id);
      for (ObjectId edge_bundle_id : frontier.edge_bundle_ids) {
        const SavedBackboneEdgeBundle* edge_bundle = view().backbone_edge_bundle(edge_bundle_id);
        const SavedBackboneEdge* edge = edge_bundle == nullptr ? nullptr : view().backbone_edge(edge_bundle->edge_id);
        const Bundle* bundle = edge_bundle == nullptr ? nullptr : view().bundles().find(edge_bundle->bundle_id);
        if (edge_bundle == nullptr || edge == nullptr || bundle == nullptr) {
          result.error = "backbone regenerate: related pole type scope is incomplete";
          return result;
        }
        add_scope_seed(bundle->bundle_template_id, edge->route, edge_bundle->bundle_id);
      }
    }

    for (RelatedPoleRegenerateScope& scope : scopes) {
      for (const SavedBackboneEdgeBundle& edge_bundle : view().backbone().edge_bundles) {
        const SavedBackboneEdge* edge = view().backbone_edge(edge_bundle.edge_id);
        const Bundle* bundle = view().bundles().find(edge_bundle.bundle_id);
        if (edge != nullptr && bundle != nullptr && edge->route == scope.route &&
            edge_bundle.bundle_id == scope.bundle_id && bundle->bundle_template_id == scope.bundle_template_id) {
          add_unique_id(scope.edge_bundle_ids, edge_bundle.edge_bundle_id);
        }
      }
      if (scope.edge_bundle_ids.empty()) {
        result.error = "backbone regenerate: related pole type scope has no edge bundles";
        return result;
      }
    }

    const PoleTypeDefinition* related_type = find_pole_type(bundle_template->related_pole_type_id);
    if (related_type == nullptr) {
      result.error = "related pole type not found";
      return result;
    }

    CoreState trial = *this;
    ChangeSet trial_changes{};
    for (ObjectId pole_id : ordered_target_pole_ids) {
      if (std::find(active_backbone_pole_ids.begin(), active_backbone_pole_ids.end(), pole_id) !=
          active_backbone_pole_ids.end()) {
        Pole* pole = trial.authoritative_.edit_state.poles.find(pole_id);
        if (pole == nullptr) {
          result.error = "pole not found";
          return result;
        }
        pole->pole_type_id = bundle_template->related_pole_type_id;
        if (std::abs(pole->height_m - related_type->default_height_m) > 1e-12) {
          pole->height_m = related_type->default_height_m;
        }
        add_unique_id(trial_changes.updated_ids, pole_id);
        continue;
      }
      const auto apply = trial.ApplyPoleType(pole_id, bundle_template->related_pole_type_id);
      if (!apply.ok) {
        result.error = apply.error;
        return result;
      }
      append_change_set(trial_changes, apply.change_set);
    }

    ChangeSet regenerated_changes{};
    for (const RelatedPoleRegenerateScope& scope : scopes) {
      const auto bundle_template_it = trial.authoritative_.bundle_templates.find(scope.bundle_template_id);
      if (bundle_template_it == trial.authoritative_.bundle_templates.end()) {
        result.error = "bundle template not found";
        return result;
      }
      const BundleTemplate previous = bundle_template_it->second;
      auto regenerated = trial.regenerate_backbone_edge_bundles(scope.bundle_template_id, previous, previous,
                                                                &regenerated_changes, nullptr,
                                                                &scope.edge_bundle_ids, related_type);
      if (!regenerated.ok) {
        result.error = regenerated.error;
        return result;
      }
    }

    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    append_change_set(result.change_set, trial_changes);
    append_change_set(result.change_set, regenerated_changes);
    result.ok = true;
    result.value = true;
    return result;
  }

  result.ok = true;
  result.value = false;
  for (ObjectId pole_id : ordered_target_pole_ids) {
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
  if (cable_template == nullptr) {
    result.error = "cable template not found";
    return result;
  }
  const AttachmentTemplateId desired_template_id = cable_template->default_endpoint_attachment_template_id;
  const AttachmentTemplate* attachment_template = desired_template_id == kInvalidAttachmentTemplateId
                                                     ? nullptr
                                                     : find_attachment_template(desired_template_id);
  if (desired_template_id != kInvalidAttachmentTemplateId && attachment_template == nullptr) {
    result.error = "default endpoint attachment template not found";
    return result;
  }

  auto ensure_endpoint_attachment = [&](bool is_start_endpoint, double t) -> bool {
    Span* span_edit = authoritative_.edit_state.spans.find(span_id);
    if (span_edit == nullptr) {
      result.error = "span not found";
      return false;
    }
    ObjectId& attachment_slot = is_start_endpoint ? span_edit->endpoint_attachment_a_id : span_edit->endpoint_attachment_b_id;
    if (attachment_slot != kInvalidObjectId) {
      const Attachment* existing = authoritative_.edit_state.attachments.find(attachment_slot);
      if (existing == nullptr) {
        result.error = "endpoint attachment not found";
        return false;
      }
      if (existing->origin == AttachmentOrigin::kUser || existing->template_id == desired_template_id) {
        return true;
      }
      const ObjectId old_attachment_id = existing->id;
      authoritative_.edit_state.attachments.remove(old_attachment_id);
      index_remove(runtime_.relation_index.attachments_by_span, span_id, old_attachment_id);
      attachment_slot = kInvalidObjectId;
      touch_span(span_id, true);
      add_unique_id(result.change_set.deleted_ids, old_attachment_id);
      add_unique_id(result.change_set.updated_ids, span_id);
      result.value = true;
    }
    if (desired_template_id == kInvalidAttachmentTemplateId) {
      return true;
    }
    const auto add_attachment =
        AddAttachment(span_id, t, attachment_template->kind, 0.0, desired_template_id);
    if (!add_attachment.ok) {
      result.error = add_attachment.error;
      return false;
    }
    Attachment* added = authoritative_.edit_state.attachments.find(add_attachment.value);
    Span* updated_span = authoritative_.edit_state.spans.find(span_id);
    if (added == nullptr || updated_span == nullptr) {
      result.error = "default endpoint attachment creation failed";
      return false;
    }
    added->origin = AttachmentOrigin::kDefaultEndpoint;
    ObjectId& updated_slot = is_start_endpoint ? updated_span->endpoint_attachment_a_id : updated_span->endpoint_attachment_b_id;
    updated_slot = add_attachment.value;
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

  const PoleTypeDefinition before = it->second;
  std::vector<ObjectId> pole_ids{};
  std::vector<ObjectId> active_backbone_pole_ids{};
  pole_ids.reserve(authoritative_.edit_state.poles.size());
  for (const Pole& pole : authoritative_.edit_state.poles.items()) {
    if (pole.pole_type_id != pole_type.id) {
      continue;
    }
    pole_ids.push_back(pole.id);
    if (runtime_.backbone_index.pole_node.contains(pole.id)) {
      active_backbone_pole_ids.push_back(pole.id);
    }
  }

  if (!active_backbone_pole_ids.empty() && !pole_type_placement_only_change(before, pole_type)) {
    struct PoleTypeRegenerateScope {
      BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
      std::size_t route = 0;
      ObjectId bundle_id = kInvalidObjectId;
      std::vector<ObjectId> edge_bundle_ids{};
    };

    std::vector<PoleTypeRegenerateScope> scopes{};
    auto add_scope_seed = [&](BundleTemplateId bundle_template_id, std::size_t route, ObjectId bundle_id) {
      const auto existing = std::find_if(scopes.begin(), scopes.end(), [&](const PoleTypeRegenerateScope& scope) {
        return scope.bundle_template_id == bundle_template_id && scope.route == route && scope.bundle_id == bundle_id;
      });
      if (existing == scopes.end()) {
        PoleTypeRegenerateScope scope{};
        scope.bundle_template_id = bundle_template_id;
        scope.route = route;
        scope.bundle_id = bundle_id;
        scopes.push_back(std::move(scope));
      }
    };

    for (ObjectId pole_id : active_backbone_pole_ids) {
      const BackboneFrontier frontier = view().pole_frontier(pole_id);
      for (ObjectId edge_bundle_id : frontier.edge_bundle_ids) {
        const SavedBackboneEdgeBundle* edge_bundle = view().backbone_edge_bundle(edge_bundle_id);
        const SavedBackboneEdge* edge = edge_bundle == nullptr ? nullptr : view().backbone_edge(edge_bundle->edge_id);
        const Bundle* bundle = edge_bundle == nullptr ? nullptr : view().bundles().find(edge_bundle->bundle_id);
        if (edge_bundle == nullptr || edge == nullptr || bundle == nullptr) {
          result.error = "backbone regenerate: pole type scope is incomplete";
          return result;
        }
        add_scope_seed(bundle->bundle_template_id, edge->route, edge_bundle->bundle_id);
      }
    }

    for (PoleTypeRegenerateScope& scope : scopes) {
      for (const SavedBackboneEdgeBundle& edge_bundle : view().backbone().edge_bundles) {
        const SavedBackboneEdge* edge = view().backbone_edge(edge_bundle.edge_id);
        const Bundle* bundle = view().bundles().find(edge_bundle.bundle_id);
        if (edge != nullptr && bundle != nullptr && edge->route == scope.route &&
            edge_bundle.bundle_id == scope.bundle_id && bundle->bundle_template_id == scope.bundle_template_id) {
          add_unique_id(scope.edge_bundle_ids, edge_bundle.edge_bundle_id);
        }
      }
      if (scope.edge_bundle_ids.empty()) {
        result.error = "backbone regenerate: pole type scope has no edge bundles";
        return result;
      }
    }

    CoreState trial = *this;
    trial.authoritative_.pole_types[pole_type.id] = pole_type;
    for (ObjectId pole_id : pole_ids) {
      Pole* pole = trial.authoritative_.edit_state.poles.find(pole_id);
      if (pole == nullptr) {
        result.error = "pole not found";
        return result;
      }
      if (std::abs(pole->height_m - pole_type.default_height_m) > 1e-12) {
        pole->height_m = pole_type.default_height_m;
        add_unique_id(result.change_set.updated_ids, pole_id);
      }
      if (std::find(active_backbone_pole_ids.begin(), active_backbone_pole_ids.end(), pole_id) ==
          active_backbone_pole_ids.end()) {
        const auto apply = trial.ApplyPoleType(pole_id, pole_type.id);
        if (!apply.ok) {
          result.error = apply.error;
          return result;
        }
        append_change_set(result.change_set, apply.change_set);
      }
    }

    ChangeSet regenerated_changes{};
    for (const PoleTypeRegenerateScope& scope : scopes) {
      const auto bundle_template_it = trial.authoritative_.bundle_templates.find(scope.bundle_template_id);
      if (bundle_template_it == trial.authoritative_.bundle_templates.end()) {
        result.error = "bundle template not found";
        return result;
      }
      const BundleTemplate previous = bundle_template_it->second;
      auto regenerated = trial.regenerate_backbone_edge_bundles(scope.bundle_template_id, previous, previous,
                                                                &regenerated_changes, nullptr,
                                                                &scope.edge_bundle_ids, &pole_type);
      if (!regenerated.ok) {
        result.error = regenerated.error;
        return result;
      }
    }

    identity_ = trial.identity_;
    authoritative_ = trial.authoritative_;
    runtime_ = trial.runtime_;
    debug_ = trial.debug_;
    append_change_set(result.change_set, regenerated_changes);
    add_unique_id(result.change_set.updated_ids, pole_type.id);
    result.ok = true;
    result.value = true;
    return result;
  }

  UpdatePlan active_plan{};
  active_plan.kind = UpdateKind::kReposition;
  if (!active_backbone_pole_ids.empty()) {
    for (ObjectId pole_id : active_backbone_pole_ids) {
      EditResult<UpdatePlan> plan = make_update_plan({UpdateKind::kReposition, UpdateTargetKind::kPole, pole_id});
      if (!plan.ok) {
        result.error = plan.error;
        return result;
      }
      append_unique(active_plan.affected.poles, plan.value.affected.poles);
      append_unique(active_plan.affected.ports, plan.value.affected.ports);
      append_unique(active_plan.affected.spans, plan.value.affected.spans);
      append_unique(active_plan.affected.edges, plan.value.affected.edges);
    }
    for (ObjectId pole_id : active_backbone_pole_ids) {
      const auto ports_it = runtime_.relation_index.ports_by_pole.find(pole_id);
      if (ports_it == runtime_.relation_index.ports_by_pole.end()) {
        continue;
      }
      for (ObjectId port_id : ports_it->second) {
        const Port* port = authoritative_.edit_state.ports.find(port_id);
        if (port == nullptr || view().backbone_port_binding_for_port(port_id) == nullptr) {
          continue;
        }
        if (state_internal::FindPortPlacementBandForPort(*this, pole_type, *port) == nullptr) {
          result.error = "backbone unsupported: active pole saved placement band no longer exists";
          return result;
        }
      }
    }
  }

  it->second = pole_type;
  result.ok = true;
  result.value = true;

  auto apply_active_backbone_placement = [&](ObjectId pole_id) -> EditResult<bool> {
    EditResult<bool> applied{};
    Pole* pole = authoritative_.edit_state.poles.find(pole_id);
    if (pole == nullptr) {
      applied.error = "pole not found";
      return applied;
    }
    if (std::abs(pole->height_m - pole_type.default_height_m) > 1e-12) {
      pole->height_m = pole_type.default_height_m;
      add_unique_id(result.change_set.updated_ids, pole_id);
      applied.value = true;
    }
    const auto owned_port_ids_it = runtime_.relation_index.ports_by_pole.find(pole_id);
    if (owned_port_ids_it != runtime_.relation_index.ports_by_pole.end()) {
      for (ObjectId port_id : owned_port_ids_it->second) {
        Port* existing_port = authoritative_.edit_state.ports.find(port_id);
        const bool backbone_bound_port = runtime_.backbone_index.port_bindings_by_port.contains(port_id);
        if (existing_port == nullptr || (!existing_port->generated_from_template && !backbone_bound_port) ||
            existing_port->position_mode == PortPositionMode::kManual) {
          continue;
        }
        const PortPlacementBand* band_ptr =
            state_internal::FindPortPlacementBandForPort(*this, pole_type, *existing_port);
        if (band_ptr == nullptr) {
          applied.error = "backbone unsupported: active pole port band no longer resolves";
          return applied;
        }
        const PortPlacementBand* previous_band_ptr =
            state_internal::FindPortPlacementBandById(before, band_ptr->band_id);
        if (previous_band_ptr == nullptr) {
          applied.error = "backbone unsupported: active pole previous port band no longer resolves";
          return applied;
        }
        if (port_band_equals(*previous_band_ptr, *band_ptr)) {
          continue;
        }

        const double layout_yaw =
            effective_port_layout_yaw_deg(*pole, existing_port->id, existing_port->category);
        const Vec3d current_local =
            WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw),
                              existing_port->world_position);
        Vec3d adjusted_local{
            0.0,
            current_local.y +
                (band_ptr->lateral_center_m - previous_band_ptr->lateral_center_m),
            current_local.z +
                (band_ptr->height_center_m - previous_band_ptr->height_center_m),
        };
        const bool apply_angle_correction = authoritative_.layout_settings.angle_correction_enabled &&
                                            pole->context.kind == PoleContextKind::kCorner &&
                                            band_ptr->side != SlotSide::kCenter;
        double applied_scale = 1.0;
        if (apply_angle_correction) {
          adjusted_local.y = state_internal::apply_corner_side_scale(
              adjusted_local.y, band_ptr->side, pole->context.corner_turn_sign, pole->context.side_scale);
          if (std::abs(band_ptr->lateral_center_m) > 1e-9) {
            applied_scale = std::abs(adjusted_local.y / band_ptr->lateral_center_m);
          }
        }
        adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, band_ptr->side);
        const Vec3d world_position =
            local_to_world_on_pole(pole->world_transform,
                                   layout_yaw,
                                   adjusted_local);
        if (LengthSquared(existing_port->world_position - world_position) > 1e-12 ||
            existing_port->angle_correction_applied != apply_angle_correction ||
            std::abs(existing_port->side_scale_applied - applied_scale) > 1e-12) {
          existing_port->world_position = world_position;
          existing_port->angle_correction_applied = apply_angle_correction;
          existing_port->side_scale_applied = apply_angle_correction ? applied_scale : 1.0;
          apply_port_position_mode(*existing_port, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBand);
          add_unique_id(result.change_set.updated_ids, existing_port->id);
          touch_connected_spans_from_port(existing_port->id, &result.change_set);
          applied.value = true;
        }
      }
    }
    applied.ok = true;
    return applied;
  };

  for (ObjectId pole_id : pole_ids) {
    if (std::find(active_backbone_pole_ids.begin(), active_backbone_pole_ids.end(), pole_id) !=
        active_backbone_pole_ids.end()) {
      const auto apply = apply_active_backbone_placement(pole_id);
      if (!apply.ok) {
        result.ok = false;
        result.error = apply.error;
        return result;
      }
      result.value = result.value || apply.value;
      continue;
    }
    const auto apply = ApplyPoleType(pole_id, pole_type.id);
    if (!apply.ok) {
      result.ok = false;
      result.error = apply.error;
      return result;
    }
    append_change_set(result.change_set, apply.change_set);
    add_unique_id(result.change_set.updated_ids, pole_id);
  }
  if (!active_backbone_pole_ids.empty() && !active_plan.affected.spans.empty()) {
    const auto derived = execute_update_plan(active_plan);
    if (!derived.ok) {
      result.ok = false;
      result.error = derived.error;
      return result;
    }
    append_unique(result.change_set.updated_ids, active_plan.affected.spans);
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

double CoreState::effective_port_layout_yaw_deg(const Pole& pole, ObjectId port_id,
                                                ConnectionCategory category,
                                                const PortLayoutYawOverride* row_layout_yaw_override) const {
  if (has_pole_orientation_override(pole.id)) {
    return effective_pole_yaw_deg(pole);
  }
  if (row_layout_yaw_override != nullptr && row_layout_yaw_override->category == category) {
    return NormalizeYawDeg(row_layout_yaw_override->yaw_deg);
  }
  if (const SavedBackbonePortBinding* binding = view().backbone_port_binding_for_port(port_id);
      binding != nullptr) {
    return NormalizeYawDeg(binding->layout_yaw_deg);
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
  const double min_offset = pole_radius_at_height_m(pole, std::max(0.0, adjusted.z)) + authoritative_.geometry_settings.pole_clearance_m;
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
  return PolylineLength(polyline);
}

} // namespace wire::core
