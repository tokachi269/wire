#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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

template <typename TValue>
void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
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

template <typename TKey>
std::unordered_map<TKey, std::vector<ObjectId>> canonical_index_map(
    const std::unordered_map<TKey, std::vector<ObjectId>>& map) {
  auto out = map;
  for (auto& [_, ids] : out) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  }
  return out;
}

}  // namespace

bool ValidationResult::has_errors() const {
  for (const ValidationIssue& issue : issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return true;
    }
  }
  return false;
}

CoreState::CoreState() {
  register_default_pole_types();
}

EditResult<ObjectId> CoreState::AddPole(
    const Transformd& world_transform,
    double height_m,
    std::string_view name,
    PoleKind kind) {
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
  edit_state_.poles.insert(pole);

  result.ok = true;
  result.value = pole.id;
  result.change_set.created_ids.push_back(pole.id);
  return result;
}

EditResult<ObjectId> CoreState::AddPort(
    ObjectId owner_pole_id,
    const Vec3d& world_position,
    PortKind kind,
    PortLayer layer,
    const Frame3d& direction) {
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
  port.generated_from_template = false;
  edit_state_.ports.insert(port);

  result.ok = true;
  result.value = port.id;
  result.change_set.created_ids.push_back(port.id);
  return result;
}

EditResult<ObjectId> CoreState::AddAnchor(
    ObjectId owner_pole_id,
    const Vec3d& world_position,
    AnchorSupportKind support_kind,
    double support_strength) {
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

  result.ok = true;
  result.value = anchor.id;
  result.change_set.created_ids.push_back(anchor.id);
  return result;
}

EditResult<ObjectId> CoreState::AddBundle(
    int conductor_count,
    double phase_spacing_m,
    BundleKind kind) {
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
  bundle.kind = kind;
  edit_state_.bundles.insert(bundle);

  result.ok = true;
  result.value = bundle.id;
  result.change_set.created_ids.push_back(bundle.id);
  return result;
}

EditResult<ObjectId> CoreState::AddSpan(
    ObjectId port_a_id,
    ObjectId port_b_id,
    SpanKind kind,
    SpanLayer layer,
    ObjectId bundle_id,
    ObjectId anchor_a_id,
    ObjectId anchor_b_id) {
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

EditResult<ObjectId> CoreState::AddAttachment(
    ObjectId span_id,
    double t,
    AttachmentKind kind,
    double offset_m) {
  EditResult<ObjectId> result;
  if (edit_state_.spans.find(span_id) == nullptr) {
    result.error = "span does not exist";
    return result;
  }
  if (t < 0.0 || t > 1.0) {
    result.error = "attachment t must be in [0, 1]";
    return result;
  }

  Attachment attachment{};
  attachment.id = id_generator_.next();
  attachment.display_id = next_display_id("AT");
  attachment.span_id = span_id;
  attachment.t = t;
  attachment.kind = kind;
  attachment.offset_m = offset_m;
  edit_state_.attachments.insert(attachment);

  result.ok = true;
  result.value = attachment.id;
  result.change_set.created_ids.push_back(attachment.id);
  return result;
}

EditResult<ObjectId> CoreState::MovePole(ObjectId pole_id, const Transformd& new_world_transform) {
  EditResult<ObjectId> result;
  Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }

  const Vec3d delta = new_world_transform.position - pole->world_transform.position;
  pole->world_transform = new_world_transform;
  result.change_set.updated_ids.push_back(pole_id);

  std::vector<ObjectId> moved_port_ids;
  for (Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id == pole_id) {
      port.world_position = port.world_position + delta;
      moved_port_ids.push_back(port.id);
      add_unique_id(result.change_set.updated_ids, port.id);
    }
  }

  std::vector<ObjectId> moved_anchor_ids;
  for (Anchor& anchor : edit_state_.anchors.items()) {
    if (anchor.owner_pole_id == pole_id) {
      anchor.world_position = anchor.world_position + delta;
      moved_anchor_ids.push_back(anchor.id);
      add_unique_id(result.change_set.updated_ids, anchor.id);
    }
  }

  const DirtyBits dirty = DirtyBits::kGeometry;
  for (ObjectId port_id : moved_port_ids) {
    mark_connected_spans_dirty_from_port(port_id, dirty, &result.change_set);
  }
  for (ObjectId anchor_id : moved_anchor_ids) {
    mark_connected_spans_dirty_from_anchor(anchor_id, dirty, &result.change_set);
  }

  result.ok = true;
  result.value = pole_id;
  return result;
}

EditResult<ObjectId> CoreState::MovePort(ObjectId port_id, const Vec3d& new_world_position) {
  EditResult<ObjectId> result;
  Port* port = edit_state_.ports.find(port_id);
  if (port == nullptr) {
    result.error = "port not found";
    return result;
  }
  port->world_position = new_world_position;
  result.change_set.updated_ids.push_back(port_id);
  mark_connected_spans_dirty_from_port(
      port_id,
      DirtyBits::kGeometry,
      &result.change_set);
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
  mark_connected_spans_dirty_from_anchor(
      anchor_id,
      DirtyBits::kGeometry,
      &result.change_set);
  result.ok = true;
  result.value = anchor_id;
  return result;
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

  std::vector<ObjectId> remove_attachments;
  for (const Attachment& attachment : edit_state_.attachments.items()) {
    if (attachment.span_id == span_id) {
      remove_attachments.push_back(attachment.id);
    }
  }
  for (ObjectId attachment_id : remove_attachments) {
    edit_state_.attachments.remove(attachment_id);
    add_unique_id(result.change_set.deleted_ids, attachment_id);
  }

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

  EditResult<ObjectId> add_port_result = AddPort(
      kInvalidObjectId,
      split_pos,
      category_to_port_kind(category),
      span_layer_to_port_layer(old_span.layer));
  if (!add_port_result.ok) {
    result.error = add_port_result.error;
    return result;
  }

  EditResult<ObjectId> add_span_a_result = AddSpan(
      old_span.port_a_id,
      add_port_result.value,
      old_span.kind,
      old_span.layer,
      old_span.bundle_id,
      old_span.anchor_a_id,
      kInvalidObjectId);
  if (!add_span_a_result.ok) {
    result.error = add_span_a_result.error;
    return result;
  }

  EditResult<ObjectId> add_span_b_result = AddSpan(
      add_port_result.value,
      old_span.port_b_id,
      old_span.kind,
      old_span.layer,
      old_span.bundle_id,
      kInvalidObjectId,
      old_span.anchor_b_id);
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
    const Vec3d world_position = pole->world_transform.position + slot.local_position;
    EditResult<ObjectId> add_port_result = AddPort(
        pole_id,
        world_position,
        category_to_port_kind(slot.category),
        category_to_port_layer(slot.category),
        slot.local_direction);
    if (!add_port_result.ok) {
      result.error = add_port_result.error;
      return result;
    }
    Port* created = edit_state_.ports.find(add_port_result.value);
    if (created != nullptr) {
      created->category = slot.category;
      created->source_slot_id = slot.slot_id;
      created->generated_from_template = true;
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

    const Vec3d world_position = pole->world_transform.position + slot.local_position;
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

EditResult<CoreState::AddConnectionByPoleResult> CoreState::AddConnectionByPole(
    ObjectId pole_a_id,
    ObjectId pole_b_id,
    ConnectionCategory category,
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

  int slot_a_id = -1;
  int slot_b_id = -1;

  auto resolve_port = [&](ObjectId pole_id, ObjectId preferred_port_id, int* out_slot_id, int preferred_slot_id) -> EditResult<ObjectId> {
    if (preferred_port_id != kInvalidObjectId) {
      const Port* preferred_port = edit_state_.ports.find(preferred_port_id);
      if (preferred_port != nullptr &&
          preferred_port->owner_pole_id == pole_id &&
          (preferred_port->category == category || preferred_port->layer == category_to_port_layer(category))) {
        EditResult<ObjectId> preferred_result;
        preferred_result.ok = true;
        preferred_result.value = preferred_port_id;
        if (out_slot_id != nullptr) {
          *out_slot_id = preferred_port->source_slot_id;
        }
        return preferred_result;
      }
    }
    return ensure_pole_slot_port(
        pole_id,
        category,
        options.allow_generate_port,
        out_slot_id,
        preferred_slot_id);
  };

  EditResult<ObjectId> port_a_result =
      resolve_port(pole_a_id, options.preferred_port_a_id, &slot_a_id, -1);
  if (!port_a_result.ok) {
    result.error = port_a_result.error;
    return result;
  }
  EditResult<ObjectId> port_b_result =
      resolve_port(pole_b_id, options.preferred_port_b_id, &slot_b_id, slot_a_id);
  if (!port_b_result.ok) {
    result.error = port_b_result.error;
    return result;
  }

  EditResult<ObjectId> bundle_result = ensure_bundle_for_category(category, options);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }

  const SpanKind span_kind = (category == ConnectionCategory::kDrop) ? SpanKind::kService : options.span_kind;
  EditResult<ObjectId> span_result = AddSpan(
      port_a_result.value,
      port_b_result.value,
      span_kind,
      category_to_span_layer(category),
      bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
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

EditResult<CoreState::AddDropResult> CoreState::AddDropFromPole(
    ObjectId source_pole_id,
    const Vec3d& target_world_position,
    ConnectionCategory category) {
  EditResult<AddDropResult> result;
  if (edit_state_.poles.find(source_pole_id) == nullptr) {
    result.error = "source pole does not exist";
    return result;
  }

  int slot_id = -1;
  EditResult<ObjectId> source_port_result = ensure_pole_slot_port(source_pole_id, category, true, &slot_id);
  if (!source_port_result.ok) {
    result.error = source_port_result.error;
    return result;
  }
  EditResult<ObjectId> target_port_result = AddPort(
      kInvalidObjectId,
      target_world_position,
      category_to_port_kind(category),
      category_to_port_layer(category));
  if (!target_port_result.ok) {
    result.error = target_port_result.error;
    return result;
  }
  EditResult<ObjectId> bundle_result = AddBundle(1, 0.15, category_to_bundle_kind(category));
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }
  EditResult<ObjectId> span_result = AddSpan(
      source_port_result.value,
      target_port_result.value,
      SpanKind::kService,
      category_to_span_layer(category),
      bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
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

EditResult<CoreState::AddDropResult> CoreState::AddDropFromSpan(
    ObjectId source_span_id,
    double t,
    const Vec3d& target_world_position,
    ConnectionCategory category) {
  EditResult<AddDropResult> result;
  EditResult<SplitSpanResult> split_result = SplitSpan(source_span_id, t);
  if (!split_result.ok) {
    result.error = split_result.error;
    return result;
  }
  EditResult<ObjectId> target_port_result = AddPort(
      kInvalidObjectId,
      target_world_position,
      category_to_port_kind(category),
      category_to_port_layer(category));
  if (!target_port_result.ok) {
    result.error = target_port_result.error;
    return result;
  }
  EditResult<ObjectId> bundle_result = AddBundle(1, 0.15, category_to_bundle_kind(category));
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }
  EditResult<ObjectId> span_result = AddSpan(
      split_result.value.new_port_id,
      target_port_result.value,
      SpanKind::kService,
      category_to_span_layer(category),
      bundle_result.value);
  if (!span_result.ok) {
    result.error = span_result.error;
    return result;
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

EditResult<std::vector<ObjectId>> CoreState::GeneratePolesAlongRoad(
    const RoadSegment& road,
    double interval,
    PoleTypeId pole_type_id) {
  EditResult<std::vector<ObjectId>> result;
  if (road.polyline.size() < 2) {
    result.error = "road polyline must contain at least 2 points";
    return result;
  }
  if (interval <= 0.0) {
    result.error = "interval must be > 0";
    return result;
  }
  if (find_pole_type(pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  const std::vector<Vec3d> points = sample_polyline_points(road.polyline, interval);
  if (points.size() < 2) {
    result.error = "failed to sample road points";
    return result;
  }

  const std::uint64_t session_id = next_generation_session_id_++;
  for (std::size_t i = 0; i < points.size(); ++i) {
    Transformd tf{};
    tf.position = points[i];
    EditResult<ObjectId> add_pole_result = AddPole(tf, 10.0, "AutoPole", PoleKind::kConcrete);
    if (!add_pole_result.ok) {
      result.error = add_pole_result.error;
      return result;
    }
    Pole* pole = edit_state_.poles.find(add_pole_result.value);
    if (pole != nullptr) {
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(i);
      add_unique_id(add_pole_result.change_set.updated_ids, pole->id);
    }

    EditResult<ObjectId> apply_result = ApplyPoleType(add_pole_result.value, pole_type_id);
    if (!apply_result.ok) {
      result.error = apply_result.error;
      return result;
    }

    result.value.push_back(add_pole_result.value);
    append_change_set(result.change_set, add_pole_result.change_set);
    append_change_set(result.change_set, apply_result.change_set);
  }

  result.ok = true;
  return result;
}

EditResult<std::vector<ObjectId>> CoreState::GenerateSpansBetweenPoles(
    const std::vector<ObjectId>& poles,
    ConnectionCategory category) {
  EditResult<std::vector<ObjectId>> result;
  if (poles.size() < 2) {
    result.error = "at least 2 poles are required";
    return result;
  }

  const std::uint64_t session_id = next_generation_session_id_++;
  std::ostringstream errors;
  bool has_failure = false;
  ObjectId carry_port_on_next_left = kInvalidObjectId;

  for (std::size_t i = 0; i + 1 < poles.size(); ++i) {
    AddConnectionByPoleOptions options{};
    options.preferred_port_a_id = carry_port_on_next_left;

    EditResult<AddConnectionByPoleResult> add_result =
        AddConnectionByPole(poles[i], poles[i + 1], category, options);
    if (!add_result.ok) {
      has_failure = true;
      errors << "[segment " << i << "] " << add_result.error << "; ";
      carry_port_on_next_left = kInvalidObjectId;
      continue;
    }

    carry_port_on_next_left = add_result.value.port_b_id;

    Span* span = edit_state_.spans.find(add_result.value.span_id);
    if (span != nullptr) {
      span->generation.generated = true;
      span->generation.source = GenerationSource::kRoadAuto;
      span->generation.generation_session_id = session_id;
      span->generation.generation_order = static_cast<std::uint32_t>(i);
      add_unique_id(add_result.change_set.updated_ids, span->id);
    }

    result.value.push_back(add_result.value.span_id);
    append_change_set(result.change_set, add_result.change_set);
  }

  if (result.value.empty()) {
    result.error = has_failure ? errors.str() : "failed to generate spans";
    return result;
  }
  if (has_failure) {
    result.error = errors.str();
    return result;
  }

  result.ok = true;
  return result;
}

EditResult<CoreState::GenerateSimpleLineResult> CoreState::GenerateSimpleLine(
    const RoadSegment& road,
    double interval,
    PoleTypeId pole_type_id,
    ConnectionCategory category) {
  EditResult<GenerateSimpleLineResult> result;
  const std::uint64_t session_id = next_generation_session_id_++;

  EditResult<std::vector<ObjectId>> poles_result = GeneratePolesAlongRoad(road, interval, pole_type_id);
  if (!poles_result.ok) {
    result.error = poles_result.error;
    return result;
  }

  for (std::size_t i = 0; i < poles_result.value.size(); ++i) {
    Pole* pole = edit_state_.poles.find(poles_result.value[i]);
    if (pole != nullptr) {
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(i);
    }
  }

  EditResult<std::vector<ObjectId>> spans_result = GenerateSpansBetweenPoles(poles_result.value, category);
  if (!spans_result.ok) {
    result.error = spans_result.error;
    return result;
  }
  for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
    Span* span = edit_state_.spans.find(spans_result.value[i]);
    if (span != nullptr) {
      span->generation.generated = true;
      span->generation.source = GenerationSource::kRoadAuto;
      span->generation.generation_session_id = session_id;
      span->generation.generation_order = static_cast<std::uint32_t>(i);
    }
  }

  result.ok = true;
  result.value.pole_ids = poles_result.value;
  result.value.span_ids = spans_result.value;
  result.value.generation_session_id = session_id;
  append_change_set(result.change_set, poles_result.change_set);
  append_change_set(result.change_set, spans_result.change_set);
  return result;
}

CoreState::PoleDetailInfo CoreState::GetPoleDetail(ObjectId pole_id) const {
  PoleDetailInfo detail{};
  detail.pole = edit_state_.poles.find(pole_id);
  if (detail.pole == nullptr) {
    return detail;
  }
  detail.pole_type = find_pole_type(detail.pole->pole_type_id);
  for (const Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id == pole_id) {
      detail.owned_ports.push_back(&port);
    }
  }
  for (const Anchor& anchor : edit_state_.anchors.items()) {
    if (anchor.owner_pole_id == pole_id) {
      detail.owned_anchors.push_back(&anchor);
    }
  }
  std::sort(detail.owned_ports.begin(), detail.owned_ports.end(), [](const Port* a, const Port* b) { return a->id < b->id; });
  std::sort(detail.owned_anchors.begin(), detail.owned_anchors.end(), [](const Anchor* a, const Anchor* b) { return a->id < b->id; });
  return detail;
}

EditResult<bool> CoreState::UpdateGeometrySettings(const GeometrySettings& settings, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  GeometrySettings normalized = settings;
  normalized.curve_samples = std::max(2, normalized.curve_samples);
  if (normalized.sag_factor < 0.0) {
    normalized.sag_factor = 0.0;
  }

  const bool changed =
      normalized.curve_samples != cache_state_.geometry_settings.curve_samples ||
      normalized.sag_enabled != cache_state_.geometry_settings.sag_enabled ||
      std::abs(normalized.sag_factor - cache_state_.geometry_settings.sag_factor) > 1e-12;

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

const CurveCacheEntry* CoreState::find_curve_cache(ObjectId span_id) const {
  auto it = cache_state_.curve_cache.by_span.find(span_id);
  if (it == cache_state_.curve_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const BoundsCacheEntry* CoreState::find_bounds_cache(ObjectId span_id) const {
  auto it = cache_state_.bounds_cache.by_span.find(span_id);
  if (it == cache_state_.bounds_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

RecalcStats CoreState::ProcessDirtyQueues() {
  RecalcStats stats{};

  std::unordered_set<ObjectId> processed_topology;
  std::unordered_set<ObjectId> processed_geometry;
  std::unordered_set<ObjectId> processed_bounds;
  std::unordered_set<ObjectId> processed_render;
  std::unordered_set<ObjectId> processed_raycast;

  for (ObjectId span_id : dirty_queue_.topology_dirty_span_ids) {
    if (!processed_topology.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kTopology)) {
      continue;
    }
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kTopology;
    ++stats.topology_processed;
  }

  for (ObjectId span_id : dirty_queue_.geometry_dirty_span_ids) {
    if (!processed_geometry.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kGeometry)) {
      continue;
    }

    std::string error_message;
    if (!rebuild_span_curve(span_id, &error_message)) {
      continue;
    }
    it->second.geometry_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kGeometry;
    ++stats.geometry_processed;
    mark_span_dirty(span_id, DirtyBits::kBounds | DirtyBits::kRender, false);
  }

  std::vector<ObjectId> bounds_queue = dirty_queue_.bounds_dirty_span_ids;
  for (ObjectId span_id : bounds_queue) {
    if (!processed_bounds.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kBounds)) {
      continue;
    }

    std::string error_message;
    if (!rebuild_span_bounds(span_id, &error_message)) {
      continue;
    }
    it->second.bounds_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kBounds;
    ++stats.bounds_processed;
  }

  std::vector<ObjectId> render_queue = dirty_queue_.render_dirty_span_ids;
  for (ObjectId span_id : render_queue) {
    if (!processed_render.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kRender)) {
      continue;
    }
    it->second.render_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRender;
    ++stats.render_processed;
  }

  for (ObjectId span_id : dirty_queue_.raycast_dirty_span_ids) {
    if (!processed_raycast.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kRaycast)) {
      continue;
    }
    it->second.raycast_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRaycast;
    ++stats.raycast_processed;
  }

  dirty_queue_ = DirtyQueue{};
  last_recalc_stats_ = stats;
  return stats;
}

ValidationResult CoreState::Validate() const {
  ValidationResult result;

  for (const Pole& pole : edit_state_.poles.items()) {
    if (pole.pole_type_id != kInvalidPoleTypeId && !pole_types_.contains(pole.pole_type_id)) {
      result.issues.push_back({ValidationSeverity::kError, "PoleTypeMissing", "Pole references unknown PoleType", pole.id});
    }
  }

  for (const Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id != kInvalidObjectId && edit_state_.poles.find(port.owner_pole_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "PortOwnerMissing", "Port owner pole is missing", port.id});
    }
    if (port.source_slot_id >= 0 && port.owner_pole_id != kInvalidObjectId) {
      const Pole* owner = edit_state_.poles.find(port.owner_pole_id);
      const PoleTypeDefinition* pole_type = (owner == nullptr) ? nullptr : find_pole_type(owner->pole_type_id);
      bool slot_found = false;
      if (pole_type != nullptr) {
        for (const PortSlotTemplate& slot : pole_type->port_slots) {
          if (slot.slot_id != port.source_slot_id) {
            continue;
          }
          slot_found = true;
          if (slot.category != port.category) {
            result.issues.push_back({
                ValidationSeverity::kError,
                "PortSlotCategoryMismatch",
                "Port category differs from slot category",
                port.id,
            });
          }
          break;
        }
      }
      if (!slot_found) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortSlotMissing",
            "Port source slot id is not defined on owner PoleType",
            port.id,
        });
      }
    }
  }

  for (const Anchor& anchor : edit_state_.anchors.items()) {
    if (anchor.owner_pole_id != kInvalidObjectId && edit_state_.poles.find(anchor.owner_pole_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "AnchorOwnerMissing", "Anchor owner pole is missing", anchor.id});
    }
  }

  for (const Span& span : edit_state_.spans.items()) {
    const Port* port_a = edit_state_.ports.find(span.port_a_id);
    const Port* port_b = edit_state_.ports.find(span.port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanPortMissing", "Span references missing port", span.id});
      continue;
    }
    if (span.port_a_id == span.port_b_id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSelfReference", "Span has same endpoint", span.id});
    }
    if (has_zero_length(*port_a, *port_b)) {
      result.issues.push_back({ValidationSeverity::kWarning, "SpanZeroLength", "Span endpoints overlap", span.id});
    }
    if (span.bundle_id != kInvalidObjectId && edit_state_.bundles.find(span.bundle_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanBundleMissing", "Span bundle is missing", span.id});
    }
    if (span.anchor_a_id != kInvalidObjectId && edit_state_.anchors.find(span.anchor_a_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAnchorMissing", "Span anchorA is missing", span.id});
    }
    if (span.anchor_b_id != kInvalidObjectId && edit_state_.anchors.find(span.anchor_b_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAnchorMissing", "Span anchorB is missing", span.id});
    }
  }

  const auto expected_port_index = canonical_index_map(make_expected_port_index(edit_state_));
  const auto expected_anchor_index = canonical_index_map(make_expected_anchor_index(edit_state_));
  const auto actual_port_index = canonical_index_map(connection_index_.spans_by_port);
  const auto actual_anchor_index = canonical_index_map(connection_index_.spans_by_anchor);

  if (expected_port_index != actual_port_index) {
    result.issues.push_back({ValidationSeverity::kError, "PortIndexMismatch", "Port->Span index mismatch", kInvalidObjectId});
  }
  if (expected_anchor_index != actual_anchor_index) {
    result.issues.push_back({ValidationSeverity::kError, "AnchorIndexMismatch", "Anchor->Span index mismatch", kInvalidObjectId});
  }

  for (const Span& span : edit_state_.spans.items()) {
    auto it = span_runtime_states_.find(span.id);
    if (it == span_runtime_states_.end()) {
      result.issues.push_back({ValidationSeverity::kError, "SpanRuntimeMissing", "Span runtime state missing", span.id});
      continue;
    }
    if (it->second.span_id != span.id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanRuntimeCorrupt", "Span runtime id mismatch", span.id});
    }
  }
  for (const auto& [span_id, runtime] : span_runtime_states_) {
    if (edit_state_.spans.find(span_id) == nullptr || runtime.span_id != span_id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanRuntimeDangling", "Runtime state points to removed span", span_id});
    }
  }

  for (const Span& span : edit_state_.spans.items()) {
    const auto runtime_it = span_runtime_states_.find(span.id);
    if (runtime_it == span_runtime_states_.end()) {
      continue;
    }
    const SpanRuntimeState& runtime = runtime_it->second;

    auto curve_it = cache_state_.curve_cache.by_span.find(span.id);
    if (curve_it != cache_state_.curve_cache.by_span.end()) {
      const CurveCacheEntry& curve = curve_it->second;
      if (curve.points.size() < 2) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "CurveSampleCountInvalid",
            "Curve cache has less than 2 points",
            span.id,
        });
      }
      if (curve.source_version != runtime.geometry_version) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "GeometryVersionMismatch",
            "Curve cache sourceVersion does not match geometryVersion",
            span.id,
        });
      }
    }

    auto bounds_it = cache_state_.bounds_cache.by_span.find(span.id);
    if (bounds_it != cache_state_.bounds_cache.by_span.end()) {
      const BoundsCacheEntry& bounds = bounds_it->second;
      if (bounds.whole.min.x > bounds.whole.max.x ||
          bounds.whole.min.y > bounds.whole.max.y ||
          bounds.whole.min.z > bounds.whole.max.z) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "BoundsInvalid",
            "Whole bounds has min > max",
            span.id,
        });
      }
      for (const AABBd& segment : bounds.segments) {
        if (segment.min.x > segment.max.x ||
            segment.min.y > segment.max.y ||
            segment.min.z > segment.max.z) {
          result.issues.push_back({
              ValidationSeverity::kError,
              "SegmentBoundsInvalid",
              "Segment bounds has min > max",
              span.id,
          });
          break;
        }
      }
      if (bounds.source_version != runtime.bounds_version) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "BoundsVersionMismatch",
            "Bounds cache sourceVersion does not match boundsVersion",
            span.id,
        });
      }
    }
  }

  return result;
}

const SpanRuntimeState* CoreState::find_span_runtime_state(ObjectId span_id) const {
  auto it = span_runtime_states_.find(span_id);
  if (it == span_runtime_states_.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::remove_span_from_indexes(const Span& span) {
  auto remove_from_map = [&](std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key) {
    auto it = map.find(key);
    if (it == map.end()) {
      return;
    }
    std::vector<ObjectId>& ids = it->second;
    ids.erase(std::remove(ids.begin(), ids.end(), span.id), ids.end());
    if (ids.empty()) {
      map.erase(it);
    }
  };

  remove_from_map(connection_index_.spans_by_port, span.port_a_id);
  remove_from_map(connection_index_.spans_by_port, span.port_b_id);
  if (span.anchor_a_id != kInvalidObjectId) {
    remove_from_map(connection_index_.spans_by_anchor, span.anchor_a_id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    remove_from_map(connection_index_.spans_by_anchor, span.anchor_b_id);
  }
}

void CoreState::add_span_to_index(const Span& span) {
  connection_index_.spans_by_port[span.port_a_id].push_back(span.id);
  connection_index_.spans_by_port[span.port_b_id].push_back(span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    connection_index_.spans_by_anchor[span.anchor_a_id].push_back(span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    connection_index_.spans_by_anchor[span.anchor_b_id].push_back(span.id);
  }
}

void CoreState::initialize_span_runtime_state(ObjectId span_id) {
  SpanRuntimeState runtime{};
  runtime.span_id = span_id;
  runtime.data_version = 0;
  runtime.geometry_version = 0;
  runtime.bounds_version = 0;
  runtime.render_version = 0;
  runtime.raycast_version = 0;
  runtime.dirty_bits = DirtyBits::kNone;
  span_runtime_states_[span_id] = runtime;
}

void CoreState::mark_span_dirty(ObjectId span_id, DirtyBits dirty_bits, bool bump_data_version) {
  if (edit_state_.spans.find(span_id) == nullptr) {
    return;
  }
  SpanRuntimeState& runtime = span_runtime_states_[span_id];
  if (runtime.span_id == kInvalidObjectId) {
    runtime.span_id = span_id;
  }
  if (bump_data_version || runtime.data_version == 0) {
    runtime.data_version = next_data_version_++;
  }
  runtime.dirty_bits |= dirty_bits;
  add_dirty_queue(span_id, dirty_bits);
}

void CoreState::add_dirty_queue(ObjectId span_id, DirtyBits dirty_bits) {
  if (any(dirty_bits, DirtyBits::kTopology)) dirty_queue_.topology_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kGeometry)) dirty_queue_.geometry_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kBounds)) dirty_queue_.bounds_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRender)) dirty_queue_.render_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRaycast)) dirty_queue_.raycast_dirty_span_ids.push_back(span_id);
}

void CoreState::mark_connected_spans_dirty_from_port(ObjectId port_id, DirtyBits dirty_bits, ChangeSet* change_set) {
  auto it = connection_index_.spans_by_port.find(port_id);
  if (it == connection_index_.spans_by_port.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    mark_span_dirty(span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, span_id);
      add_unique_id(change_set->updated_ids, span_id);
    }
  }
}

void CoreState::mark_connected_spans_dirty_from_anchor(ObjectId anchor_id, DirtyBits dirty_bits, ChangeSet* change_set) {
  auto it = connection_index_.spans_by_anchor.find(anchor_id);
  if (it == connection_index_.spans_by_anchor.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    mark_span_dirty(span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, span_id);
      add_unique_id(change_set->updated_ids, span_id);
    }
  }
}

bool CoreState::rebuild_span_curve(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }

  std::vector<Vec3d> points = generate_span_points(*span, error_message);
  if (points.size() < 2) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "generated points are invalid";
    }
    return false;
  }

  CurveCacheEntry entry{};
  entry.points = std::move(points);
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  cache_state_.curve_cache.by_span[span_id] = std::move(entry);
  return true;
}

bool CoreState::rebuild_span_bounds(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  auto curve_it = cache_state_.curve_cache.by_span.find(span_id);
  if (curve_it == cache_state_.curve_cache.by_span.end()) {
    if (error_message != nullptr) {
      *error_message = "curve cache missing";
    }
    return false;
  }
  const std::vector<Vec3d>& points = curve_it->second.points;
  if (points.size() < 2) {
    if (error_message != nullptr) {
      *error_message = "curve cache has too few points";
    }
    return false;
  }

  BoundsCacheEntry bounds{};
  bounds.whole = build_aabb_from_points(points);
  bounds.segments.reserve(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    bounds.segments.push_back(build_aabb_from_two_points(points[i], points[i + 1]));
  }
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  bounds.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  cache_state_.bounds_cache.by_span[span_id] = std::move(bounds);
  return true;
}

std::vector<Vec3d> CoreState::generate_span_points(const Span& span, std::string* error_message) const {
  const Port* port_a = edit_state_.ports.find(span.port_a_id);
  const Port* port_b = edit_state_.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const int samples = std::max(2, cache_state_.geometry_settings.curve_samples);
  std::vector<Vec3d> points;
  points.reserve(static_cast<std::size_t>(samples));

  const Vec3d a = port_a->world_position;
  const Vec3d b = port_b->world_position;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const bool use_sag = cache_state_.geometry_settings.sag_enabled && distance > kZeroLengthEps;
  const double sag_amount = cache_state_.geometry_settings.sag_factor * distance;

  for (int i = 0; i < samples; ++i) {
    const double t = (samples <= 1) ? 0.0 : (static_cast<double>(i) / static_cast<double>(samples - 1));
    Vec3d p{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    };
    if (use_sag) {
      const double sag_shape = 4.0 * t * (1.0 - t);
      p.z -= sag_amount * sag_shape;
    }
    points.push_back(p);
  }
  return points;
}

AABBd CoreState::build_aabb_from_points(const std::vector<Vec3d>& points) {
  AABBd box{};
  if (points.empty()) {
    return box;
  }
  box.min = points.front();
  box.max = points.front();
  for (const Vec3d& p : points) {
    box.min.x = std::min(box.min.x, p.x);
    box.min.y = std::min(box.min.y, p.y);
    box.min.z = std::min(box.min.z, p.z);
    box.max.x = std::max(box.max.x, p.x);
    box.max.y = std::max(box.max.y, p.y);
    box.max.z = std::max(box.max.z, p.z);
  }
  return box;
}

AABBd CoreState::build_aabb_from_two_points(const Vec3d& a, const Vec3d& b) {
  AABBd box{};
  box.min = {
      std::min(a.x, b.x),
      std::min(a.y, b.y),
      std::min(a.z, b.z),
  };
  box.max = {
      std::max(a.x, b.x),
      std::max(a.y, b.y),
      std::max(a.z, b.z),
  };
  return box;
}

void CoreState::remove_span_from_caches(ObjectId span_id) {
  cache_state_.curve_cache.by_span.erase(span_id);
  cache_state_.bounds_cache.by_span.erase(span_id);
}

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
      {100, ConnectionCategory::kHighVoltage, {0.0, -0.6, 9.2}, {}, 30, false, true},
      {101, ConnectionCategory::kHighVoltage, {0.0, 0.0, 9.3}, {}, 29, false, true},
      {102, ConnectionCategory::kHighVoltage, {0.0, 0.6, 9.2}, {}, 28, false, true},
      {200, ConnectionCategory::kLowVoltage, {0.0, -0.4, 6.8}, {}, 20, false, true},
      {201, ConnectionCategory::kLowVoltage, {0.0, 0.4, 6.8}, {}, 19, false, true},
      {300, ConnectionCategory::kCommunication, {0.0, -0.8, 7.8}, {}, 15, false, true},
      {301, ConnectionCategory::kOptical, {0.0, 0.8, 7.8}, {}, 14, false, true},
      {400, ConnectionCategory::kDrop, {0.0, 0.0, 4.2}, {}, 10, true, true},
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
      {600, ConnectionCategory::kCommunication, {0.0, -0.7, 8.6}, {}, 30, false, true},
      {601, ConnectionCategory::kCommunication, {0.0, 0.0, 8.7}, {}, 29, false, true},
      {602, ConnectionCategory::kCommunication, {0.0, 0.7, 8.6}, {}, 28, false, true},
      {700, ConnectionCategory::kOptical, {0.0, -0.4, 7.6}, {}, 25, false, true},
      {701, ConnectionCategory::kOptical, {0.0, 0.4, 7.6}, {}, 24, false, true},
      {800, ConnectionCategory::kLowVoltage, {0.0, 0.0, 5.8}, {}, 10, false, true},
      {801, ConnectionCategory::kDrop, {0.0, 0.0, 4.0}, {}, 9, true, true},
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

std::vector<PortSlotTemplate> CoreState::sorted_port_slots(
    const PoleTypeDefinition& pole_type,
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

EditResult<ObjectId> CoreState::ensure_pole_slot_port(
    ObjectId pole_id,
    ConnectionCategory category,
    bool allow_generate_port,
    int* out_slot_id,
    int preferred_slot_id) {
  EditResult<ObjectId> result;
  const Pole* pole = edit_state_.poles.find(pole_id);
  if (pole == nullptr) {
    result.error = "pole not found";
    return result;
  }

  auto connection_count = [&](ObjectId port_id) -> std::size_t {
    auto it = connection_index_.spans_by_port.find(port_id);
    if (it == connection_index_.spans_by_port.end()) {
      return 0;
    }
    return it->second.size();
  };

  const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id);
  if (pole_type != nullptr) {
    auto slots = sorted_port_slots(*pole_type, category);
    if (preferred_slot_id >= 0) {
      auto it = std::find_if(slots.begin(), slots.end(), [&](const PortSlotTemplate& slot) {
        return slot.slot_id == preferred_slot_id;
      });
      if (it != slots.end() && it != slots.begin()) {
        PortSlotTemplate preferred = *it;
        slots.erase(it);
        slots.insert(slots.begin(), preferred);
      }
    }
    for (const PortSlotTemplate& slot : slots) {
      Port* slot_port = nullptr;
      for (Port& port : edit_state_.ports.items()) {
        if (port.owner_pole_id == pole_id && port.source_slot_id == slot.slot_id) {
          slot_port = &port;
          break;
        }
      }
      if (slot_port != nullptr) {
        const std::size_t usage = connection_count(slot_port->id);
        if (usage == 0 || slot.allow_multiple) {
          result.ok = true;
          result.value = slot_port->id;
          if (out_slot_id != nullptr) {
            *out_slot_id = slot.slot_id;
          }
          return result;
        }
      } else if (allow_generate_port) {
        const Vec3d world_position = pole->world_transform.position + slot.local_position;
        EditResult<ObjectId> add_port_result = AddPort(
            pole_id,
            world_position,
            category_to_port_kind(category),
            category_to_port_layer(category),
            slot.local_direction);
        if (!add_port_result.ok) {
          return add_port_result;
        }
        Port* created = edit_state_.ports.find(add_port_result.value);
        if (created != nullptr) {
          created->category = category;
          created->source_slot_id = slot.slot_id;
          created->generated_from_template = true;
          add_unique_id(add_port_result.change_set.updated_ids, created->id);
        }
        if (out_slot_id != nullptr) {
          *out_slot_id = slot.slot_id;
        }
        return add_port_result;
      }
    }
  }

  ObjectId fallback = kInvalidObjectId;
  std::size_t fallback_usage = std::numeric_limits<std::size_t>::max();
  for (const Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id != pole_id || port.category != category) {
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
    return result;
  }

  if (allow_generate_port) {
    // Fallback: no template slot and no category port found.
    // Create one generated port near the pole so Pole->Pole flow remains usable.
    const Vec3d world_position{
        pole->world_transform.position.x,
        pole->world_transform.position.y,
        pole->world_transform.position.z + std::max(1.0, pole->height_m * 0.7),
    };
    EditResult<ObjectId> add_port_result = AddPort(
        pole_id,
        world_position,
        category_to_port_kind(category),
        category_to_port_layer(category));
    if (!add_port_result.ok) {
      return add_port_result;
    }
    Port* created = edit_state_.ports.find(add_port_result.value);
    if (created != nullptr) {
      created->category = category;
      created->generated_from_template = true;
      created->source_slot_id = -1;
      add_unique_id(add_port_result.change_set.updated_ids, created->id);
    }
    if (out_slot_id != nullptr) {
      *out_slot_id = -1;
    }
    return add_port_result;
  }

  std::ostringstream oss;
  oss << "no port available for pole " << pole_id << " category " << static_cast<int>(category);
  result.error = oss.str();
  return result;
}

EditResult<ObjectId> CoreState::ensure_bundle_for_category(
    ConnectionCategory category,
    const AddConnectionByPoleOptions& options) {
  EditResult<ObjectId> result;
  if (options.bundle_id != kInvalidObjectId) {
    if (edit_state_.bundles.find(options.bundle_id) == nullptr) {
      result.error = "bundle does not exist";
      return result;
    }
    result.ok = true;
    result.value = options.bundle_id;
    return result;
  }

  if (!options.auto_create_bundle) {
    result.ok = true;
    result.value = kInvalidObjectId;
    return result;
  }

  const int conductor_count = (category == ConnectionCategory::kHighVoltage) ? 3 : 1;
  const double spacing = (category == ConnectionCategory::kHighVoltage) ? 0.45 : 0.15;
  return AddBundle(conductor_count, spacing, category_to_bundle_kind(category));
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

std::string CoreState::dirty_bits_to_string(DirtyBits bits) {
  std::string text;
  if (any(bits, DirtyBits::kTopology)) text += "Topology|";
  if (any(bits, DirtyBits::kGeometry)) text += "Geometry|";
  if (any(bits, DirtyBits::kBounds)) text += "Bounds|";
  if (any(bits, DirtyBits::kRender)) text += "Render|";
  if (any(bits, DirtyBits::kRaycast)) text += "Raycast|";
  if (!text.empty()) {
    text.pop_back();
  }
  return text;
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
      std::abs(out.back().y - polyline.back().y) > 1e-9 ||
      std::abs(out.back().z - polyline.back().z) > 1e-9) {
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

  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kCommunication);
  (void)state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kOptical);

  (void)state.AddConnectionByPole(pole_b, pole_c, ConnectionCategory::kLowVoltage);
  (void)state.AddConnectionByPole(pole_b, pole_c, ConnectionCategory::kCommunication);
  (void)state.AddConnectionByPole(pole_b, pole_c, ConnectionCategory::kCommunication);
  (void)state.AddConnectionByPole(pole_b, pole_c, ConnectionCategory::kOptical);

  (void)state.AddDropFromPole(pole_b, {13.0, 4.0, 3.0}, ConnectionCategory::kDrop);
  (void)state.AddDropFromPole(pole_b, {14.0, -4.0, 3.0}, ConnectionCategory::kDrop);
  (void)state.ProcessDirtyQueues();

  return state;
}

}  // namespace wire::core
