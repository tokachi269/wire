#include "wire/core/core_state.hpp"

#include <algorithm>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace wire::core {
namespace {

class StateWriter {
public:
  StateWriter() { text_ = "wire_state_v1\n"; }

  void value(const std::string& key, bool input) { line(key, input ? "1" : "0"); }

  template <typename T>
    requires(std::is_integral_v<T> && !std::is_same_v<T, bool>)
  void value(const std::string& key, T input) {
    if constexpr (std::is_signed_v<T>) {
      line(key, std::to_string(static_cast<long long>(input)));
    } else {
      line(key, std::to_string(static_cast<unsigned long long>(input)));
    }
  }

  template <typename T>
    requires std::is_enum_v<T>
  void value(const std::string& key, T input) {
    value(key, static_cast<std::underlying_type_t<T>>(input));
  }

  void value(const std::string& key, double input) {
    char buffer[64]{};
    const int size = std::snprintf(buffer, sizeof(buffer), "%a", input);
    line(key, size > 0 ? std::string(buffer, static_cast<std::size_t>(size)) : std::string{});
  }

  void string_value(const std::string& key, const std::string& input) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string encoded{};
    encoded.reserve(input.size() * 2u);
    for (unsigned char byte : input) {
      encoded.push_back(kHex[byte >> 4u]);
      encoded.push_back(kHex[byte & 0x0fu]);
    }
    line(key, encoded);
  }

  [[nodiscard]] std::string finish() && { return std::move(text_); }

private:
  void line(const std::string& key, const std::string& encoded) {
    text_.append(key);
    text_.push_back('=');
    text_.append(encoded);
    text_.push_back('\n');
  }

  std::string text_{};
};

class StateReader {
public:
  bool parse(const std::string& text) {
    static constexpr std::string_view kHeader = "wire_state_v1\n";
    if (!text.starts_with(kHeader)) {
      error_ = "authoritative deserialization: unsupported or missing version";
      return false;
    }
    if (text.empty() || text.back() != '\n') {
      error_ = "authoritative deserialization: truncated final line";
      return false;
    }
    std::size_t line_begin = kHeader.size();
    while (line_begin < text.size()) {
      const std::size_t line_end = text.find('\n', line_begin);
      const std::string_view line(text.data() + line_begin, line_end - line_begin);
      const std::size_t separator = line.find('=');
      if (separator == std::string_view::npos || separator == 0) {
        error_ = "authoritative deserialization: malformed key=value line";
        return false;
      }
      std::string key(line.substr(0, separator));
      std::string encoded(line.substr(separator + 1));
      if (!values_.emplace(key, std::move(encoded)).second) {
        error_ = "authoritative deserialization: duplicate key " + key;
        return false;
      }
      line_begin = line_end + 1;
    }
    return true;
  }

  bool value(const std::string& key, bool* out) {
    const std::string* encoded = take(key);
    if (encoded == nullptr) return false;
    if (*encoded == "0") {
      *out = false;
      return true;
    }
    if (*encoded == "1") {
      *out = true;
      return true;
    }
    return invalid(key);
  }

  template <typename T>
    requires(std::is_integral_v<T> && !std::is_same_v<T, bool>)
  bool value(const std::string& key, T* out) {
    const std::string* encoded = take(key);
    if (encoded == nullptr) return false;
    T parsed{};
    const auto result = std::from_chars(encoded->data(), encoded->data() + encoded->size(), parsed);
    if (result.ec != std::errc{} || result.ptr != encoded->data() + encoded->size()) return invalid(key);
    *out = parsed;
    return true;
  }

  template <typename T>
    requires std::is_enum_v<T>
  bool value(const std::string& key, T* out) {
    std::underlying_type_t<T> raw{};
    if (!value(key, &raw)) return false;
    *out = static_cast<T>(raw);
    return true;
  }

  bool value(const std::string& key, double* out) {
    const std::string* encoded = take(key);
    if (encoded == nullptr) return false;
    char* end = nullptr;
    const double parsed = std::strtod(encoded->c_str(), &end);
    if (encoded->empty() || end == encoded->c_str() || end != encoded->c_str() + encoded->size()) return invalid(key);
    *out = parsed;
    return true;
  }

  bool string_value(const std::string& key, std::string* out) {
    const std::string* encoded = take(key);
    if (encoded == nullptr) return false;
    if ((encoded->size() % 2u) != 0) return invalid(key);
    out->clear();
    out->reserve(encoded->size() / 2u);
    for (std::size_t i = 0; i < encoded->size(); i += 2) {
      const int high = hex_digit((*encoded)[i]);
      const int low = hex_digit((*encoded)[i + 1]);
      if (high < 0 || low < 0) return invalid(key);
      out->push_back(static_cast<char>((high << 4) | low));
    }
    return true;
  }

  bool count(const std::string& key, std::size_t* out) {
    if (!value(key, out)) return false;
    if (*out > 1000000u) {
      error_ = "authoritative deserialization: count exceeds limit at " + key;
      return false;
    }
    return true;
  }

  bool record_ids(const std::string& prefix, std::size_t expected_count, std::vector<std::uint64_t>* out) {
    const std::string marker = prefix + ".";
    out->clear();
    for (const auto& [key, encoded] : values_) {
      static_cast<void>(encoded);
      if (!key.starts_with(marker)) continue;
      const std::size_t id_end = key.find('.', marker.size());
      if (id_end == std::string::npos) continue;
      const std::string_view id_text(key.data() + marker.size(), id_end - marker.size());
      std::uint64_t id = 0;
      const auto parsed = std::from_chars(id_text.data(), id_text.data() + id_text.size(), id);
      if (parsed.ec != std::errc{} || parsed.ptr != id_text.data() + id_text.size()) continue;
      out->push_back(id);
    }
    std::sort(out->begin(), out->end());
    out->erase(std::unique(out->begin(), out->end()), out->end());
    if (out->size() != expected_count) {
      error_ = "authoritative deserialization: record count mismatch at " + prefix;
      return false;
    }
    return true;
  }

  bool finish() {
    if (!error_.empty()) return false;
    if (!values_.empty()) {
      error_ = "authoritative deserialization: unknown key " + values_.begin()->first;
      return false;
    }
    return true;
  }

  [[nodiscard]] const std::string& error() const { return error_; }

private:
  const std::string* take(const std::string& key) {
    const auto it = values_.find(key);
    if (it == values_.end()) {
      error_ = "authoritative deserialization: missing field " + key;
      return nullptr;
    }
    taken_ = std::move(it->second);
    values_.erase(it);
    return &taken_;
  }

  bool invalid(const std::string& key) {
    error_ = "authoritative deserialization: invalid value at " + key;
    return false;
  }

  static int hex_digit(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
  }

  std::unordered_map<std::string, std::string> values_{};
  std::string taken_{};
  std::string error_{};
};

std::string child(const std::string& prefix, std::string_view field) {
  return prefix + "." + std::string(field);
}

std::string indexed(const std::string& prefix, std::size_t index) {
  return prefix + "." + std::to_string(index);
}

class WriteFieldArchive {
public:
  static constexpr bool loading = false;
  explicit WriteFieldArchive(StateWriter& writer) : writer_(writer) {}

  template <typename T> bool value(const std::string& key, const T& input) {
    writer_.value(key, input);
    return true;
  }

  bool string_value(const std::string& key, const std::string& input) {
    writer_.string_value(key, input);
    return true;
  }

  bool count(const std::string& key, std::size_t& input) {
    writer_.value(key, input);
    return true;
  }

  template <typename T> bool optional(const std::string& prefix, const std::optional<T>& input) {
    writer_.value(child(prefix, "has"), input.has_value());
    if (input.has_value()) writer_.value(child(prefix, "value"), *input);
    return true;
  }

private:
  StateWriter& writer_;
};

class ReadFieldArchive {
public:
  static constexpr bool loading = true;
  explicit ReadFieldArchive(StateReader& reader) : reader_(reader) {}

  template <typename T> bool value(const std::string& key, T& output) {
    return reader_.value(key, &output);
  }

  bool string_value(const std::string& key, std::string& output) {
    return reader_.string_value(key, &output);
  }

  bool count(const std::string& key, std::size_t& output) { return reader_.count(key, &output); }

  template <typename T> bool optional(const std::string& prefix, std::optional<T>& output) {
    bool has = false;
    if (!reader_.value(child(prefix, "has"), &has)) return false;
    if (!has) {
      output.reset();
      return true;
    }
    T parsed{};
    if (!reader_.value(child(prefix, "value"), &parsed)) return false;
    output = parsed;
    return true;
  }

private:
  StateReader& reader_;
};

#define ARCHIVE_VALUE(field) archive.value(child(prefix, #field), value.field)
#define ARCHIVE_STRING(field) archive.string_value(child(prefix, #field), value.field)

template <typename Archive, typename Value>
bool archive_vec3(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(x) && ARCHIVE_VALUE(y) && ARCHIVE_VALUE(z);
}

template <typename Archive, typename Value>
bool archive_transform(Archive& archive, const std::string& prefix, Value& value) {
  return archive_vec3(archive, child(prefix, "position"), value.position) &&
         archive_vec3(archive, child(prefix, "rotation_euler_deg"), value.rotation_euler_deg) &&
         archive_vec3(archive, child(prefix, "scale"), value.scale);
}

template <typename Archive, typename Value>
bool archive_frame(Archive& archive, const std::string& prefix, Value& value) {
  return archive_vec3(archive, child(prefix, "origin"), value.origin) &&
         archive_vec3(archive, child(prefix, "forward"), value.forward) &&
         archive_vec3(archive, child(prefix, "right"), value.right) &&
         archive_vec3(archive, child(prefix, "up"), value.up);
}

template <typename Archive, typename Value>
bool archive_generation(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(generated) && ARCHIVE_VALUE(source) && ARCHIVE_VALUE(generation_session_id) &&
         ARCHIVE_VALUE(generation_order);
}

template <typename Archive, typename Value>
bool archive_pole_context(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(kind) && ARCHIVE_VALUE(corner_angle_deg) && ARCHIVE_VALUE(corner_turn_sign) &&
         ARCHIVE_VALUE(side_scale) && ARCHIVE_VALUE(angle_correction_applied);
}

template <typename Archive, typename Value>
bool archive_pole(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(id) && ARCHIVE_STRING(display_id) && ARCHIVE_STRING(name) &&
         archive_transform(archive, child(prefix, "world_transform"), value.world_transform) &&
         ARCHIVE_VALUE(tilt_magnitude_deg) && ARCHIVE_VALUE(height_m) && ARCHIVE_VALUE(kind) &&
         ARCHIVE_VALUE(pole_type_id) && archive_pole_context(archive, child(prefix, "context"), value.context) &&
         ARCHIVE_VALUE(placement_mode) && ARCHIVE_VALUE(user_edited) && ARCHIVE_VALUE(placement_override_flag) &&
         archive_generation(archive, child(prefix, "generation"), value.generation);
}

template <typename Archive, typename Value>
bool archive_port(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(id) && ARCHIVE_STRING(display_id) && ARCHIVE_VALUE(owner_pole_id) &&
         archive_vec3(archive, child(prefix, "world_position"), value.world_position) && ARCHIVE_VALUE(kind) &&
         ARCHIVE_VALUE(layer) && archive_frame(archive, child(prefix, "direction"), value.direction) &&
         ARCHIVE_VALUE(category) && ARCHIVE_VALUE(template_layer) && ARCHIVE_VALUE(template_side) &&
         ARCHIVE_VALUE(template_role) && ARCHIVE_VALUE(generated_from_template) && ARCHIVE_VALUE(generated_by_rule) &&
         ARCHIVE_VALUE(placement_context) && ARCHIVE_VALUE(angle_correction_applied) &&
         ARCHIVE_VALUE(side_scale_applied) && ARCHIVE_VALUE(position_mode) && ARCHIVE_VALUE(placement_source) &&
         ARCHIVE_VALUE(user_edited_position) && ARCHIVE_VALUE(placement_override_flag) &&
         ARCHIVE_VALUE(orientation_override_flag);
}

template <typename Archive, typename Value>
bool archive_anchor(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(id) && ARCHIVE_STRING(display_id) && ARCHIVE_VALUE(owner_pole_id) &&
         archive_vec3(archive, child(prefix, "world_position"), value.world_position) &&
         ARCHIVE_VALUE(support_kind) && ARCHIVE_VALUE(support_strength) && ARCHIVE_VALUE(generated_from_template);
}

template <typename Archive, typename Value>
bool archive_bundle(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(id) && ARCHIVE_STRING(display_id) && ARCHIVE_VALUE(conductor_count) &&
         ARCHIVE_VALUE(phase_spacing_m) && ARCHIVE_VALUE(bundle_template_id);
}

template <typename Archive, typename Value>
bool archive_span(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(id) && ARCHIVE_STRING(display_id) && ARCHIVE_VALUE(port_a_id) &&
         ARCHIVE_VALUE(port_b_id) && ARCHIVE_VALUE(endpoint_node_a_id) && ARCHIVE_VALUE(endpoint_node_b_id) &&
         ARCHIVE_VALUE(kind) && ARCHIVE_VALUE(layer) && ARCHIVE_VALUE(bundle_id) && ARCHIVE_VALUE(anchor_a_id) &&
         ARCHIVE_VALUE(anchor_b_id) && ARCHIVE_VALUE(endpoint_attachment_a_id) &&
         ARCHIVE_VALUE(endpoint_attachment_b_id) && ARCHIVE_VALUE(placement_context) &&
         ARCHIVE_VALUE(generated_by_rule) && ARCHIVE_VALUE(placement_override_flag) &&
         ARCHIVE_VALUE(reference_length_m) && archive_generation(archive, child(prefix, "generation"), value.generation);
}

template <typename Archive, typename Value>
bool archive_attachment(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(id) && ARCHIVE_STRING(display_id) && ARCHIVE_VALUE(span_id) && ARCHIVE_VALUE(template_id) &&
         ARCHIVE_VALUE(t) && ARCHIVE_VALUE(kind) && ARCHIVE_VALUE(display_offset_m) && ARCHIVE_VALUE(origin);
}

#undef ARCHIVE_VALUE
#undef ARCHIVE_STRING

#define WRITE_ENTITY_WRAPPER(name, type)                                                                               \
  void write_##name(StateWriter& writer, const std::string& prefix, const type& value) {                              \
    WriteFieldArchive archive(writer);                                                                                 \
    static_cast<void>(archive_##name(archive, prefix, value));                                                         \
  }

WRITE_ENTITY_WRAPPER(vec3, Vec3d)
WRITE_ENTITY_WRAPPER(transform, Transformd)
WRITE_ENTITY_WRAPPER(frame, Frame3d)
WRITE_ENTITY_WRAPPER(generation, GenerationMeta)
WRITE_ENTITY_WRAPPER(pole_context, PoleContextInfo)
WRITE_ENTITY_WRAPPER(pole, Pole)
WRITE_ENTITY_WRAPPER(port, Port)
WRITE_ENTITY_WRAPPER(anchor, Anchor)
WRITE_ENTITY_WRAPPER(bundle, Bundle)
WRITE_ENTITY_WRAPPER(span, Span)
WRITE_ENTITY_WRAPPER(attachment, Attachment)

#undef WRITE_ENTITY_WRAPPER

template <typename T, typename Write>
void write_object_store(StateWriter& writer, const std::string& prefix, const ObjectStore<T>& store, Write write) {
  std::vector<const T*> ordered{};
  ordered.reserve(store.size());
  for (const T& value : store.items()) {
    ordered.push_back(&value);
  }
  std::sort(ordered.begin(), ordered.end(), [](const T* a, const T* b) { return a->id < b->id; });
  writer.value(child(prefix, "count"), ordered.size());
  for (const T* value : ordered) {
    write(writer, indexed(prefix, value->id), *value);
  }
}

void write_edit_state(StateWriter& writer, const EditState& state) {
  write_object_store(writer, "authoritative.edit_state.poles", state.poles, write_pole);
  write_object_store(writer, "authoritative.edit_state.ports", state.ports, write_port);
  write_object_store(writer, "authoritative.edit_state.anchors", state.anchors, write_anchor);
  write_object_store(writer, "authoritative.edit_state.bundles", state.bundles, write_bundle);
  write_object_store(writer, "authoritative.edit_state.spans", state.spans, write_span);
  write_object_store(writer, "authoritative.edit_state.attachments", state.attachments, write_attachment);
}

#define ARCHIVE_VALUE(field) archive.value(child(prefix, #field), value.field)

template <typename Archive, typename Value>
bool archive_bundle_mode(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(bundle_template_id) && ARCHIVE_VALUE(mode);
}

template <typename Archive, typename Value>
bool archive_saved_node(Archive& archive, const std::string& prefix, Value& value) {
  if (!ARCHIVE_VALUE(node_id) || !ARCHIVE_VALUE(pole_id) || !ARCHIVE_VALUE(support_kind) ||
      !archive_vec3(archive, child(prefix, "position"), value.position) || !ARCHIVE_VALUE(has_source_edge) ||
      !ARCHIVE_VALUE(source_edge_node_a) || !ARCHIVE_VALUE(source_edge_node_b) ||
      !ARCHIVE_VALUE(source_edge_t) || !ARCHIVE_VALUE(path_point_index)) return false;
  std::size_t count = value.bundle_modes.size();
  if (!archive.count(child(prefix, "bundle_modes.count"), count)) return false;
  if constexpr (Archive::loading) value.bundle_modes.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive_bundle_mode(archive, indexed(child(prefix, "bundle_modes"), i), value.bundle_modes[i])) return false;
  }
  return true;
}

template <typename Archive, typename Value>
bool archive_saved_edge(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(edge_id) && ARCHIVE_VALUE(node_a) && ARCHIVE_VALUE(node_b) && ARCHIVE_VALUE(route) &&
         ARCHIVE_VALUE(order) && archive_vec3(archive, child(prefix, "dir"), value.dir) &&
         ARCHIVE_VALUE(lateral_offset_m);
}

template <typename Archive, typename Value>
bool archive_saved_edge_bundle(Archive& archive, const std::string& prefix, Value& value) {
  if (!ARCHIVE_VALUE(edge_bundle_id) || !ARCHIVE_VALUE(edge_id) || !ARCHIVE_VALUE(bundle_id) ||
      !ARCHIVE_VALUE(edge_forward) || !ARCHIVE_VALUE(route) || !ARCHIVE_VALUE(order) ||
      !archive_vec3(archive, child(prefix, "dir"), value.dir)) return false;
  std::size_t count = value.span_ids.size();
  if (!archive.count(child(prefix, "span_ids.count"), count)) return false;
  if constexpr (Archive::loading) value.span_ids.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive.value(indexed(child(prefix, "span_ids"), i), value.span_ids[i])) return false;
  }
  return true;
}

template <typename Archive, typename Value>
bool archive_row_key(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(node_id) && ARCHIVE_VALUE(source_is_open) && ARCHIVE_VALUE(source_edge_a) &&
         ARCHIVE_VALUE(source_edge_b);
}

template <typename Archive, typename Value>
bool archive_saved_port_binding(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(edge_bundle_id) && archive_row_key(archive, child(prefix, "row_key"), value.row_key) &&
         ARCHIVE_VALUE(lane_index) && ARCHIVE_VALUE(bundle_template_id) && ARCHIVE_VALUE(port_kind) &&
         ARCHIVE_VALUE(port_layer) && ARCHIVE_VALUE(placement_band_id) && ARCHIVE_VALUE(layout_yaw_deg) &&
         ARCHIVE_VALUE(port_id);
}

template <typename Archive, typename Value>
bool archive_saved_span_binding(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_VALUE(edge_bundle_id) && ARCHIVE_VALUE(lane_index) && ARCHIVE_VALUE(span_id);
}

#undef ARCHIVE_VALUE

#define WRITE_BACKBONE_WRAPPER(name, type)                                                                             \
  void write_##name(StateWriter& writer, const std::string& prefix, const type& value) {                              \
    WriteFieldArchive archive(writer);                                                                                 \
    static_cast<void>(archive_##name(archive, prefix, value));                                                         \
  }

WRITE_BACKBONE_WRAPPER(bundle_mode, SupportNodeBundleMode)
WRITE_BACKBONE_WRAPPER(saved_node, SavedBackboneNode)
WRITE_BACKBONE_WRAPPER(saved_edge, SavedBackboneEdge)
WRITE_BACKBONE_WRAPPER(saved_edge_bundle, SavedBackboneEdgeBundle)
WRITE_BACKBONE_WRAPPER(row_key, SavedBackboneRowKey)
WRITE_BACKBONE_WRAPPER(saved_port_binding, SavedBackbonePortBinding)
WRITE_BACKBONE_WRAPPER(saved_span_binding, SavedBackboneSpanBinding)

#undef WRITE_BACKBONE_WRAPPER

template <typename T, typename Id, typename Write>
void write_id_vector(StateWriter& writer, const std::string& prefix, const std::vector<T>& values, Id id, Write write) {
  std::vector<const T*> ordered{};
  ordered.reserve(values.size());
  for (const T& value : values) {
    ordered.push_back(&value);
  }
  std::sort(ordered.begin(), ordered.end(), [&](const T* a, const T* b) { return id(*a) < id(*b); });
  writer.value(child(prefix, "count"), ordered.size());
  for (const T* value : ordered) {
    write(writer, indexed(prefix, id(*value)), *value);
  }
}

template <typename T, typename Less, typename Write>
void write_ordered_vector(StateWriter& writer, const std::string& prefix, const std::vector<T>& values,
                          Less less, Write write) {
  std::vector<const T*> ordered{};
  ordered.reserve(values.size());
  for (const T& value : values) ordered.push_back(&value);
  std::sort(ordered.begin(), ordered.end(), [&](const T* a, const T* b) { return less(*a, *b); });
  writer.value(child(prefix, "count"), ordered.size());
  for (std::size_t i = 0; i < ordered.size(); ++i) write(writer, indexed(prefix, i), *ordered[i]);
}

void write_backbone(StateWriter& writer, const SavedBackboneGraph& graph) {
  write_id_vector(writer, "authoritative.backbone.nodes", graph.nodes,
                  [](const SavedBackboneNode& value) { return value.node_id; }, write_saved_node);
  write_id_vector(writer, "authoritative.backbone.edges", graph.edges,
                  [](const SavedBackboneEdge& value) { return value.edge_id; }, write_saved_edge);
  write_id_vector(writer, "authoritative.backbone.edge_bundles", graph.edge_bundles,
                  [](const SavedBackboneEdgeBundle& value) { return value.edge_bundle_id; }, write_saved_edge_bundle);
  write_ordered_vector(writer, "authoritative.backbone.port_bindings", graph.port_bindings,
                       [](const SavedBackbonePortBinding& a, const SavedBackbonePortBinding& b) {
                         return std::tie(a.edge_bundle_id, a.row_key.node_id, a.row_key.source_is_open,
                                         a.row_key.source_edge_a, a.row_key.source_edge_b, a.lane_index, a.port_id) <
                                std::tie(b.edge_bundle_id, b.row_key.node_id, b.row_key.source_is_open,
                                         b.row_key.source_edge_a, b.row_key.source_edge_b, b.lane_index, b.port_id);
                       }, write_saved_port_binding);
  write_ordered_vector(writer, "authoritative.backbone.span_bindings", graph.span_bindings,
                       [](const SavedBackboneSpanBinding& a, const SavedBackboneSpanBinding& b) {
                         return std::tie(a.edge_bundle_id, a.lane_index, a.span_id) <
                                std::tie(b.edge_bundle_id, b.lane_index, b.span_id);
                       }, write_saved_span_binding);
}

void write_port_band(StateWriter& writer, const std::string& prefix, const PortPlacementBand& value) {
  writer.value(child(prefix, "band_id"), value.band_id);
  writer.value(child(prefix, "category"), value.category);
  write_frame(writer, child(prefix, "local_direction"), value.local_direction);
  writer.value(child(prefix, "layer"), value.layer);
  writer.value(child(prefix, "side"), value.side);
  writer.value(child(prefix, "role"), value.role);
  writer.value(child(prefix, "lateral_center_m"), value.lateral_center_m);
  writer.value(child(prefix, "lateral_min_m"), value.lateral_min_m);
  writer.value(child(prefix, "lateral_max_m"), value.lateral_max_m);
  writer.value(child(prefix, "height_center_m"), value.height_center_m);
  writer.value(child(prefix, "height_min_m"), value.height_min_m);
  writer.value(child(prefix, "height_max_m"), value.height_max_m);
  writer.value(child(prefix, "priority"), value.priority);
  writer.value(child(prefix, "min_spacing_m"), value.min_spacing_m);
  writer.value(child(prefix, "allow_multiple"), value.allow_multiple);
  writer.value(child(prefix, "overflow_policy"), value.overflow_policy);
  writer.value(child(prefix, "enabled"), value.enabled);
}

void write_anchor_slot(StateWriter& writer, const std::string& prefix, const AnchorSlotTemplate& value) {
  writer.value(child(prefix, "slot_id"), value.slot_id);
  writer.value(child(prefix, "usage"), value.usage);
  write_vec3(writer, child(prefix, "local_position"), value.local_position);
  writer.value(child(prefix, "priority"), value.priority);
  writer.value(child(prefix, "enabled"), value.enabled);
}

void write_pole_type(StateWriter& writer, const std::string& prefix, const PoleTypeDefinition& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "name"), value.name);
  writer.string_value(child(prefix, "description"), value.description);
  writer.value(child(prefix, "default_height_m"), value.default_height_m);
  writer.value(child(prefix, "port_bands.count"), value.port_bands.size());
  for (std::size_t i = 0; i < value.port_bands.size(); ++i) {
    write_port_band(writer, indexed(child(prefix, "port_bands"), i), value.port_bands[i]);
  }
  writer.value(child(prefix, "anchor_slots.count"), value.anchor_slots.size());
  for (std::size_t i = 0; i < value.anchor_slots.size(); ++i) {
    write_anchor_slot(writer, indexed(child(prefix, "anchor_slots"), i), value.anchor_slots[i]);
  }
}

void write_supplemental_path(StateWriter& writer, const std::string& prefix,
                             const CableSupplementalPathTemplate& value) {
  writer.value(child(prefix, "anchor_mode"), value.anchor_mode);
  writer.value(child(prefix, "profile_kind"), value.profile_kind);
  writer.value(child(prefix, "interaction_mode"), value.interaction_mode);
  writer.value(child(prefix, "pole_band_id"), value.pole_band_id);
  writer.value(child(prefix, "endpoint_trim_m"), value.endpoint_trim_m);
  writer.value(child(prefix, "lateral_offset_m"), value.lateral_offset_m);
  writer.value(child(prefix, "vertical_offset_m"), value.vertical_offset_m);
  writer.value(child(prefix, "wobble_amplitude_m"), value.wobble_amplitude_m);
  writer.value(child(prefix, "wobble_wavelength_m"), value.wobble_wavelength_m);
  writer.value(child(prefix, "wobble_phase_bias"), value.wobble_phase_bias);
  writer.value(child(prefix, "endpoint_envelope_ratio"), value.endpoint_envelope_ratio);
  writer.value(child(prefix, "coil_radius_m"), value.coil_radius_m);
  writer.value(child(prefix, "coil_turns_per_meter"), value.coil_turns_per_meter);
  writer.value(child(prefix, "coil_samples_per_turn"), value.coil_samples_per_turn);
}

void write_cable_template(StateWriter& writer, const std::string& prefix, const CableTemplate& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "name"), value.name);
  writer.value(child(prefix, "outer_diameter_m"), value.outer_diameter_m);
  writer.value(child(prefix, "default_grouped_support_fanout_spacing_m"),
               value.default_grouped_support_fanout_spacing_m);
  writer.value(child(prefix, "bend_stiffness"), value.bend_stiffness);
  writer.value(child(prefix, "min_bend_radius_m"), value.min_bend_radius_m);
  writer.value(child(prefix, "material_style"), value.material_style);
  writer.value(child(prefix, "color_rgba"), value.color_rgba);
  writer.value(child(prefix, "requires_insulator"), value.requires_insulator);
  writer.value(child(prefix, "insulator_attachment_height_m"), value.insulator_attachment_height_m);
  writer.value(child(prefix, "sag_factor"), value.sag_factor);
  writer.value(child(prefix, "slack_factor"), value.slack_factor);
  writer.value(child(prefix, "continuity_policy"), value.continuity_policy);
  writer.value(child(prefix, "attachment_style"), value.attachment_style);
  writer.value(child(prefix, "default_endpoint_attachment_template_id"),
               value.default_endpoint_attachment_template_id);
  writer.value(child(prefix, "supplemental_paths.count"), value.supplemental_paths.size());
  for (std::size_t i = 0; i < value.supplemental_paths.size(); ++i) {
    write_supplemental_path(writer, indexed(child(prefix, "supplemental_paths"), i), value.supplemental_paths[i]);
  }
  writer.value(child(prefix, "version"), value.version);
}

void write_reserve(StateWriter& writer, const std::string& prefix, const PlacementReserve& value) {
  writer.value(child(prefix, "reserve_id"), value.reserve_id);
  writer.value(child(prefix, "pole_type_id"), value.pole_type_id);
  writer.value(child(prefix, "band_id"), value.band_id);
  writer.value(child(prefix, "lateral_min_m"), value.lateral_min_m);
  writer.value(child(prefix, "lateral_max_m"), value.lateral_max_m);
  writer.value(child(prefix, "height_min_m"), value.height_min_m);
  writer.value(child(prefix, "height_max_m"), value.height_max_m);
}

void write_population_rule(StateWriter& writer, const std::string& prefix, const CablePopulationRule& value) {
  writer.value(child(prefix, "rule_id"), value.rule_id);
  writer.value(child(prefix, "explicit_seed"), value.explicit_seed);
  writer.value(child(prefix, "priority"), value.priority);
  writer.value(child(prefix, "min_extra_count"), value.min_extra_count);
  writer.value(child(prefix, "max_extra_count"), value.max_extra_count);
  writer.value(child(prefix, "min_spacing_m"), value.min_spacing_m);
  writer.value(child(prefix, "lateral_min_m"), value.lateral_min_m);
  writer.value(child(prefix, "lateral_max_m"), value.lateral_max_m);
  writer.value(child(prefix, "height_min_m"), value.height_min_m);
  writer.value(child(prefix, "height_max_m"), value.height_max_m);
  writer.value(child(prefix, "randomness"), value.randomness);
  writer.value(child(prefix, "profile"), value.profile);
  writer.value(child(prefix, "wrap_radius_m"), value.wrap_radius_m);
  writer.value(child(prefix, "wrap_turns_per_meter"), value.wrap_turns_per_meter);
  writer.value(child(prefix, "wrap_phase"), value.wrap_phase);
  writer.value(child(prefix, "wrap_direction"), value.wrap_direction);
  writer.value(child(prefix, "end_trim_m"), value.end_trim_m);
  writer.value(child(prefix, "reserves.count"), value.reserves.size());
  for (std::size_t i = 0; i < value.reserves.size(); ++i) {
    write_reserve(writer, indexed(child(prefix, "reserves"), i), value.reserves[i]);
  }
}

void write_bundle_template(StateWriter& writer, const std::string& prefix, const BundleTemplate& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.value(child(prefix, "kind"), value.kind);
  writer.string_value(child(prefix, "name"), value.name);
  writer.value(child(prefix, "category"), value.category);
  writer.value(child(prefix, "cable_template_id"), value.cable_template_id);
  writer.value(child(prefix, "default_layer"), value.default_layer);
  writer.value(child(prefix, "related_pole_type_id"), value.related_pole_type_id);
  writer.value(child(prefix, "preserve_conductor_identity"), value.preserve_conductor_identity);
  writer.value(child(prefix, "count_rule"), value.count_rule);
  writer.value(child(prefix, "fixed_count"), value.fixed_count);
  writer.value(child(prefix, "min_count"), value.min_count);
  writer.value(child(prefix, "max_count"), value.max_count);
  writer.value(child(prefix, "default_count"), value.default_count);
  writer.value(child(prefix, "default_spacing_m"), value.default_spacing_m);
  writer.value(child(prefix, "grouped_support_fanout_spacing_m"), value.grouped_support_fanout_spacing_m);
  writer.value(child(prefix, "allow_mirror"), value.allow_mirror);
  writer.value(child(prefix, "allow_midair_node"), value.allow_midair_node);
  writer.value(child(prefix, "allow_midair_branch"), value.allow_midair_branch);
  writer.value(child(prefix, "enable_branch_down_offset"), value.enable_branch_down_offset);
  writer.value(child(prefix, "branch_endpoint_offset_m"), value.branch_endpoint_offset_m);
  writer.value(child(prefix, "order_decision_policy"), value.order_decision_policy);
  writer.value(child(prefix, "row_layout_axis_mode"), value.row_layout_axis_mode);
  writer.value(child(prefix, "support_style"), value.support_style);
  writer.value(child(prefix, "branch_policy"), value.branch_policy);
  writer.value(child(prefix, "continuity_policy"), value.continuity_policy);
  writer.value(child(prefix, "support_wire_pole_band_id"), value.support_wire_pole_band_id);
  writer.value(child(prefix, "population_rules.count"), value.population_rules.size());
  for (std::size_t i = 0; i < value.population_rules.size(); ++i) {
    write_population_rule(writer, indexed(child(prefix, "population_rules"), i), value.population_rules[i]);
  }
  writer.value(child(prefix, "version"), value.version);
}

void write_attachment_socket(StateWriter& writer, const std::string& prefix,
                             const AttachmentSocketTemplate& value) {
  writer.value(child(prefix, "id"), value.id);
  write_vec3(writer, child(prefix, "local_position"), value.local_position);
  write_vec3(writer, child(prefix, "tangent_dir"), value.tangent_dir);
  writer.value(child(prefix, "has_normal"), value.has_normal);
  write_vec3(writer, child(prefix, "normal_dir"), value.normal_dir);
  writer.value(child(prefix, "has_binormal"), value.has_binormal);
  write_vec3(writer, child(prefix, "binormal_dir"), value.binormal_dir);
  writer.value(child(prefix, "kind"), value.kind);
}

void write_internal_path(StateWriter& writer, const std::string& prefix,
                         const AttachmentInternalPathTemplate& value) {
  writer.value(child(prefix, "start_socket_id"), value.start_socket_id);
  writer.value(child(prefix, "end_socket_id"), value.end_socket_id);
  writer.value(child(prefix, "profile_kind"), value.profile_kind);
  writer.value(child(prefix, "local_points.count"), value.local_points.size());
  for (std::size_t i = 0; i < value.local_points.size(); ++i) {
    write_vec3(writer, indexed(child(prefix, "local_points"), i), value.local_points[i]);
  }
  writer.value(child(prefix, "coil_radius_m"), value.coil_radius_m);
  writer.value(child(prefix, "coil_turn_count"), value.coil_turn_count);
  writer.value(child(prefix, "coil_samples_per_turn"), value.coil_samples_per_turn);
}

void write_attachment_template(StateWriter& writer, const std::string& prefix, const AttachmentTemplate& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "name"), value.name);
  writer.value(child(prefix, "kind"), value.kind);
  writer.value(child(prefix, "line_interaction_mode"), value.line_interaction_mode);
  writer.value(child(prefix, "sockets.count"), value.sockets.size());
  for (std::size_t i = 0; i < value.sockets.size(); ++i) {
    write_attachment_socket(writer, indexed(child(prefix, "sockets"), i), value.sockets[i]);
  }
  writer.value(child(prefix, "internal_paths.count"), value.internal_paths.size());
  for (std::size_t i = 0; i < value.internal_paths.size(); ++i) {
    write_internal_path(writer, indexed(child(prefix, "internal_paths"), i), value.internal_paths[i]);
  }
  writer.value(child(prefix, "version"), value.version);
}

template <typename K, typename V, typename Write>
void write_map(StateWriter& writer, const std::string& prefix, const std::unordered_map<K, V>& values, Write write) {
  std::vector<K> keys{};
  keys.reserve(values.size());
  for (const auto& [key, value] : values) {
    static_cast<void>(value);
    keys.push_back(key);
  }
  std::sort(keys.begin(), keys.end());
  writer.value(child(prefix, "count"), keys.size());
  for (K key : keys) {
    write(writer, indexed(prefix, key), values.at(key));
  }
}

template <typename Archive, typename Value>
bool archive_pole_orientation_override(Archive& archive, const std::string& prefix, Value& value) {
  return archive.optional(child(prefix, "base_yaw_deg"), value.base_yaw_deg) &&
         archive.optional(child(prefix, "manual_yaw_deg"), value.manual_yaw_deg) &&
         archive.optional(child(prefix, "flip_180"), value.flip_180) &&
         archive.value(child(prefix, "version"), value.version);
}

template <typename Archive, typename Value>
bool archive_span_endpoint_override(Archive& archive, const std::string& prefix, Value& value) {
  return archive.optional(child(prefix, "socket_a_id"), value.socket_a_id) &&
         archive.optional(child(prefix, "socket_b_id"), value.socket_b_id) &&
         archive.value(child(prefix, "version"), value.version);
}

template <typename Archive, typename Value>
bool archive_span_support_override(Archive& archive, const std::string& prefix, Value& value) {
  return archive.optional(child(prefix, "branch_down_offset_m"), value.branch_down_offset_m) &&
         archive.value(child(prefix, "version"), value.version);
}

#define ARCHIVE_FIELD(field) archive.value(child(prefix, #field), value.field)

template <typename Archive, typename Value>
bool archive_context_profile(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_FIELD(age) && ARCHIVE_FIELD(clutter) && ARCHIVE_FIELD(regularity) &&
         ARCHIVE_FIELD(service_mix) && ARCHIVE_FIELD(style_seed);
}

template <typename Archive, typename Value>
bool archive_layout_settings(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_FIELD(angle_correction_enabled) && ARCHIVE_FIELD(corner_threshold_deg) &&
         ARCHIVE_FIELD(min_side_scale) && ARCHIVE_FIELD(max_side_scale);
}

template <typename Archive, typename Value>
bool archive_geometry_settings(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_FIELD(curve_samples) && ARCHIVE_FIELD(sag_enabled) && ARCHIVE_FIELD(sag_factor) &&
         ARCHIVE_FIELD(pole_clearance_m);
}

template <typename Archive, typename Value>
bool archive_visual_settings(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_FIELD(enable_support_structures) && ARCHIVE_FIELD(enable_insulators) &&
         ARCHIVE_FIELD(support_center_threshold_m) && ARCHIVE_FIELD(support_arm_extra_m) &&
         ARCHIVE_FIELD(support_arm_radius_m) && ARCHIVE_FIELD(insulator_radius_m) &&
         ARCHIVE_FIELD(insulator_length_m);
}

template <typename Archive, typename Value>
bool archive_variation_settings(Archive& archive, const std::string& prefix, Value& value) {
  return ARCHIVE_FIELD(enabled) && ARCHIVE_FIELD(global_seed) && ARCHIVE_FIELD(world_cell_size_m) &&
         ARCHIVE_FIELD(world_bias_scale) && ARCHIVE_FIELD(flow_bias_scale) &&
         ARCHIVE_FIELD(pole_delta_scale) && ARCHIVE_FIELD(local_jitter_scale) &&
         ARCHIVE_FIELD(sag_variation_scale) && ARCHIVE_FIELD(branch_down_offset_variation_scale);
}

#undef ARCHIVE_FIELD

#define WRITE_ARCHIVE_WRAPPER(name, type)                                                                             \
  void write_##name(StateWriter& writer, const std::string& prefix, const type& value) {                              \
    WriteFieldArchive archive(writer);                                                                                 \
    static_cast<void>(archive_##name(archive, prefix, value));                                                         \
  }

WRITE_ARCHIVE_WRAPPER(pole_orientation_override, PoleOrientationOverride)
WRITE_ARCHIVE_WRAPPER(span_endpoint_override, SpanEndpointOverride)
WRITE_ARCHIVE_WRAPPER(span_support_override, SpanSupportOverride)

#undef WRITE_ARCHIVE_WRAPPER

void write_context_profile(StateWriter& writer, const ContextProfile& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_context_profile(archive, "authoritative.context_profile", value));
}

void write_layout_settings(StateWriter& writer, const LayoutSettings& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_layout_settings(archive, "authoritative.layout_settings", value));
}

void write_geometry_settings(StateWriter& writer, const GeometrySettings& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_geometry_settings(archive, "authoritative.geometry_settings", value));
}

void write_visual_settings(StateWriter& writer, const VisualSettings& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_visual_settings(archive, "authoritative.visual_settings", value));
}

void write_variation_settings(StateWriter& writer, const VariationSettings& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_variation_settings(archive, "authoritative.variation_settings", value));
}

#define READ_VALUE(field)                                                                                              \
  if (!reader.value(child(prefix, #field), &value->field)) return false
#define READ_STRING(field)                                                                                             \
  if (!reader.string_value(child(prefix, #field), &value->field)) return false

#define READ_ENTITY_WRAPPER(name, type)                                                                                \
  bool read_##name(StateReader& reader, const std::string& prefix, type* value) {                                     \
    ReadFieldArchive archive(reader);                                                                                  \
    return archive_##name(archive, prefix, *value);                                                                    \
  }

READ_ENTITY_WRAPPER(vec3, Vec3d)
READ_ENTITY_WRAPPER(transform, Transformd)
READ_ENTITY_WRAPPER(frame, Frame3d)
READ_ENTITY_WRAPPER(generation, GenerationMeta)
READ_ENTITY_WRAPPER(pole_context, PoleContextInfo)
READ_ENTITY_WRAPPER(pole, Pole)
READ_ENTITY_WRAPPER(port, Port)
READ_ENTITY_WRAPPER(anchor, Anchor)
READ_ENTITY_WRAPPER(bundle, Bundle)
READ_ENTITY_WRAPPER(span, Span)
READ_ENTITY_WRAPPER(attachment, Attachment)

#undef READ_ENTITY_WRAPPER

template <typename T, typename Read>
bool read_object_store(StateReader& reader, const std::string& prefix, ObjectStore<T>* store, Read read) {
  std::size_t count = 0;
  if (!reader.count(child(prefix, "count"), &count)) return false;
  std::vector<std::uint64_t> ids{};
  if (!reader.record_ids(prefix, count, &ids)) return false;
  for (std::uint64_t id : ids) {
    T value{};
    if (!read(reader, indexed(prefix, id), &value) || value.id != id || id == kInvalidObjectId) return false;
    store->insert(std::move(value));
  }
  return true;
}

bool read_edit_state(StateReader& reader, EditState* state) {
  return read_object_store(reader, "authoritative.edit_state.poles", &state->poles, read_pole) &&
         read_object_store(reader, "authoritative.edit_state.ports", &state->ports, read_port) &&
         read_object_store(reader, "authoritative.edit_state.anchors", &state->anchors, read_anchor) &&
         read_object_store(reader, "authoritative.edit_state.bundles", &state->bundles, read_bundle) &&
         read_object_store(reader, "authoritative.edit_state.spans", &state->spans, read_span) &&
         read_object_store(reader, "authoritative.edit_state.attachments", &state->attachments, read_attachment);
}

#define READ_BACKBONE_WRAPPER(name, type)                                                                              \
  bool read_##name(StateReader& reader, const std::string& prefix, type* value) {                                     \
    ReadFieldArchive archive(reader);                                                                                  \
    return archive_##name(archive, prefix, *value);                                                                    \
  }

READ_BACKBONE_WRAPPER(bundle_mode, SupportNodeBundleMode)
READ_BACKBONE_WRAPPER(saved_node, SavedBackboneNode)
READ_BACKBONE_WRAPPER(saved_edge, SavedBackboneEdge)
READ_BACKBONE_WRAPPER(saved_edge_bundle, SavedBackboneEdgeBundle)
READ_BACKBONE_WRAPPER(row_key, SavedBackboneRowKey)
READ_BACKBONE_WRAPPER(saved_port_binding, SavedBackbonePortBinding)
READ_BACKBONE_WRAPPER(saved_span_binding, SavedBackboneSpanBinding)

#undef READ_BACKBONE_WRAPPER

template <typename T, typename Id, typename Read>
bool read_id_vector(StateReader& reader, const std::string& prefix, std::vector<T>* values, Id id, Read read) {
  std::size_t count = 0;
  if (!reader.count(child(prefix, "count"), &count)) return false;
  std::vector<std::uint64_t> ids{};
  if (!reader.record_ids(prefix, count, &ids)) return false;
  values->reserve(count);
  for (std::uint64_t record_id : ids) {
    T value{};
    if (!read(reader, indexed(prefix, record_id), &value) || id(value) != record_id) return false;
    values->push_back(std::move(value));
  }
  return true;
}

bool read_backbone(StateReader& reader, SavedBackboneGraph* graph) {
  if (!read_id_vector(reader, "authoritative.backbone.nodes", &graph->nodes,
                      [](const SavedBackboneNode& value) { return value.node_id; }, read_saved_node) ||
      !read_id_vector(reader, "authoritative.backbone.edges", &graph->edges,
                      [](const SavedBackboneEdge& value) { return value.edge_id; }, read_saved_edge) ||
      !read_id_vector(reader, "authoritative.backbone.edge_bundles", &graph->edge_bundles,
                      [](const SavedBackboneEdgeBundle& value) { return value.edge_bundle_id; },
                      read_saved_edge_bundle)) return false;
  std::size_t port_count = 0;
  if (!reader.count("authoritative.backbone.port_bindings.count", &port_count)) return false;
  graph->port_bindings.resize(port_count);
  for (std::size_t i = 0; i < port_count; ++i) {
    if (!read_saved_port_binding(reader, indexed("authoritative.backbone.port_bindings", i),
                                 &graph->port_bindings[i])) return false;
  }
  std::size_t span_count = 0;
  if (!reader.count("authoritative.backbone.span_bindings.count", &span_count)) return false;
  graph->span_bindings.resize(span_count);
  for (std::size_t i = 0; i < span_count; ++i) {
    if (!read_saved_span_binding(reader, indexed("authoritative.backbone.span_bindings", i),
                                 &graph->span_bindings[i])) return false;
  }
  return true;
}

bool read_port_band(StateReader& reader, const std::string& prefix, PortPlacementBand* value) {
  READ_VALUE(band_id);
  READ_VALUE(category);
  if (!read_frame(reader, child(prefix, "local_direction"), &value->local_direction)) return false;
  READ_VALUE(layer);
  READ_VALUE(side);
  READ_VALUE(role);
  READ_VALUE(lateral_center_m);
  READ_VALUE(lateral_min_m);
  READ_VALUE(lateral_max_m);
  READ_VALUE(height_center_m);
  READ_VALUE(height_min_m);
  READ_VALUE(height_max_m);
  READ_VALUE(priority);
  READ_VALUE(min_spacing_m);
  READ_VALUE(allow_multiple);
  READ_VALUE(overflow_policy);
  READ_VALUE(enabled);
  return true;
}

bool read_anchor_slot(StateReader& reader, const std::string& prefix, AnchorSlotTemplate* value) {
  READ_VALUE(slot_id);
  READ_VALUE(usage);
  if (!read_vec3(reader, child(prefix, "local_position"), &value->local_position)) return false;
  READ_VALUE(priority);
  READ_VALUE(enabled);
  return true;
}

bool read_pole_type(StateReader& reader, const std::string& prefix, PoleTypeDefinition* value) {
  READ_VALUE(id);
  READ_STRING(name);
  READ_STRING(description);
  READ_VALUE(default_height_m);
  std::size_t port_count = 0;
  if (!reader.count(child(prefix, "port_bands.count"), &port_count)) return false;
  value->port_bands.resize(port_count);
  for (std::size_t i = 0; i < port_count; ++i) {
    if (!read_port_band(reader, indexed(child(prefix, "port_bands"), i), &value->port_bands[i])) return false;
  }
  std::size_t anchor_count = 0;
  if (!reader.count(child(prefix, "anchor_slots.count"), &anchor_count)) return false;
  value->anchor_slots.resize(anchor_count);
  for (std::size_t i = 0; i < anchor_count; ++i) {
    if (!read_anchor_slot(reader, indexed(child(prefix, "anchor_slots"), i), &value->anchor_slots[i])) return false;
  }
  return true;
}

bool read_supplemental_path(StateReader& reader, const std::string& prefix,
                            CableSupplementalPathTemplate* value) {
  READ_VALUE(anchor_mode);
  READ_VALUE(profile_kind);
  READ_VALUE(interaction_mode);
  READ_VALUE(pole_band_id);
  READ_VALUE(endpoint_trim_m);
  READ_VALUE(lateral_offset_m);
  READ_VALUE(vertical_offset_m);
  READ_VALUE(wobble_amplitude_m);
  READ_VALUE(wobble_wavelength_m);
  READ_VALUE(wobble_phase_bias);
  READ_VALUE(endpoint_envelope_ratio);
  READ_VALUE(coil_radius_m);
  READ_VALUE(coil_turns_per_meter);
  READ_VALUE(coil_samples_per_turn);
  return true;
}

bool read_cable_template(StateReader& reader, const std::string& prefix, CableTemplate* value) {
  READ_VALUE(id);
  READ_STRING(name);
  READ_VALUE(outer_diameter_m);
  READ_VALUE(default_grouped_support_fanout_spacing_m);
  READ_VALUE(bend_stiffness);
  READ_VALUE(min_bend_radius_m);
  READ_VALUE(material_style);
  READ_VALUE(color_rgba);
  READ_VALUE(requires_insulator);
  READ_VALUE(insulator_attachment_height_m);
  READ_VALUE(sag_factor);
  READ_VALUE(slack_factor);
  READ_VALUE(continuity_policy);
  READ_VALUE(attachment_style);
  READ_VALUE(default_endpoint_attachment_template_id);
  std::size_t count = 0;
  if (!reader.count(child(prefix, "supplemental_paths.count"), &count)) return false;
  value->supplemental_paths.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!read_supplemental_path(reader, indexed(child(prefix, "supplemental_paths"), i),
                                &value->supplemental_paths[i])) return false;
  }
  READ_VALUE(version);
  return true;
}

bool read_reserve(StateReader& reader, const std::string& prefix, PlacementReserve* value) {
  READ_VALUE(reserve_id);
  READ_VALUE(pole_type_id);
  READ_VALUE(band_id);
  READ_VALUE(lateral_min_m);
  READ_VALUE(lateral_max_m);
  READ_VALUE(height_min_m);
  READ_VALUE(height_max_m);
  return true;
}

bool read_population_rule(StateReader& reader, const std::string& prefix, CablePopulationRule* value) {
  READ_VALUE(rule_id);
  READ_VALUE(explicit_seed);
  READ_VALUE(priority);
  READ_VALUE(min_extra_count);
  READ_VALUE(max_extra_count);
  READ_VALUE(min_spacing_m);
  READ_VALUE(lateral_min_m);
  READ_VALUE(lateral_max_m);
  READ_VALUE(height_min_m);
  READ_VALUE(height_max_m);
  READ_VALUE(randomness);
  READ_VALUE(profile);
  READ_VALUE(wrap_radius_m);
  READ_VALUE(wrap_turns_per_meter);
  READ_VALUE(wrap_phase);
  READ_VALUE(wrap_direction);
  READ_VALUE(end_trim_m);
  std::size_t count = 0;
  if (!reader.count(child(prefix, "reserves.count"), &count)) return false;
  value->reserves.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!read_reserve(reader, indexed(child(prefix, "reserves"), i), &value->reserves[i])) return false;
  }
  return true;
}

bool read_bundle_template(StateReader& reader, const std::string& prefix, BundleTemplate* value) {
  READ_VALUE(id);
  READ_VALUE(kind);
  READ_STRING(name);
  READ_VALUE(category);
  READ_VALUE(cable_template_id);
  READ_VALUE(default_layer);
  READ_VALUE(related_pole_type_id);
  READ_VALUE(preserve_conductor_identity);
  READ_VALUE(count_rule);
  READ_VALUE(fixed_count);
  READ_VALUE(min_count);
  READ_VALUE(max_count);
  READ_VALUE(default_count);
  READ_VALUE(default_spacing_m);
  READ_VALUE(grouped_support_fanout_spacing_m);
  READ_VALUE(allow_mirror);
  READ_VALUE(allow_midair_node);
  READ_VALUE(allow_midair_branch);
  READ_VALUE(enable_branch_down_offset);
  READ_VALUE(branch_endpoint_offset_m);
  READ_VALUE(order_decision_policy);
  READ_VALUE(row_layout_axis_mode);
  READ_VALUE(support_style);
  READ_VALUE(branch_policy);
  READ_VALUE(continuity_policy);
  READ_VALUE(support_wire_pole_band_id);
  std::size_t count = 0;
  if (!reader.count(child(prefix, "population_rules.count"), &count)) return false;
  value->population_rules.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!read_population_rule(reader, indexed(child(prefix, "population_rules"), i),
                              &value->population_rules[i])) return false;
  }
  READ_VALUE(version);
  return true;
}

bool read_attachment_socket(StateReader& reader, const std::string& prefix, AttachmentSocketTemplate* value) {
  READ_VALUE(id);
  if (!read_vec3(reader, child(prefix, "local_position"), &value->local_position) ||
      !read_vec3(reader, child(prefix, "tangent_dir"), &value->tangent_dir)) return false;
  READ_VALUE(has_normal);
  if (!read_vec3(reader, child(prefix, "normal_dir"), &value->normal_dir)) return false;
  READ_VALUE(has_binormal);
  if (!read_vec3(reader, child(prefix, "binormal_dir"), &value->binormal_dir)) return false;
  READ_VALUE(kind);
  return true;
}

bool read_internal_path(StateReader& reader, const std::string& prefix, AttachmentInternalPathTemplate* value) {
  READ_VALUE(start_socket_id);
  READ_VALUE(end_socket_id);
  READ_VALUE(profile_kind);
  std::size_t count = 0;
  if (!reader.count(child(prefix, "local_points.count"), &count)) return false;
  value->local_points.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!read_vec3(reader, indexed(child(prefix, "local_points"), i), &value->local_points[i])) return false;
  }
  READ_VALUE(coil_radius_m);
  READ_VALUE(coil_turn_count);
  READ_VALUE(coil_samples_per_turn);
  return true;
}

bool read_attachment_template(StateReader& reader, const std::string& prefix, AttachmentTemplate* value) {
  READ_VALUE(id);
  READ_STRING(name);
  READ_VALUE(kind);
  READ_VALUE(line_interaction_mode);
  std::size_t socket_count = 0;
  if (!reader.count(child(prefix, "sockets.count"), &socket_count)) return false;
  value->sockets.resize(socket_count);
  for (std::size_t i = 0; i < socket_count; ++i) {
    if (!read_attachment_socket(reader, indexed(child(prefix, "sockets"), i), &value->sockets[i])) return false;
  }
  std::size_t path_count = 0;
  if (!reader.count(child(prefix, "internal_paths.count"), &path_count)) return false;
  value->internal_paths.resize(path_count);
  for (std::size_t i = 0; i < path_count; ++i) {
    if (!read_internal_path(reader, indexed(child(prefix, "internal_paths"), i), &value->internal_paths[i])) return false;
  }
  READ_VALUE(version);
  return true;
}

template <typename K, typename V, typename Read>
bool read_map(StateReader& reader, const std::string& prefix, std::unordered_map<K, V>* values, Read read) {
  std::size_t count = 0;
  if (!reader.count(child(prefix, "count"), &count)) return false;
  std::vector<std::uint64_t> keys{};
  if (!reader.record_ids(prefix, count, &keys)) return false;
  for (std::uint64_t raw_key : keys) {
    if (raw_key > std::numeric_limits<K>::max()) return false;
    const K key = static_cast<K>(raw_key);
    V value{};
    if (!read(reader, indexed(prefix, key), &value) || value.id != key) return false;
    values->emplace(key, std::move(value));
  }
  return true;
}

bool read_pole_orientation_override(StateReader& reader, const std::string& prefix,
                                    PoleOrientationOverride* value) {
  ReadFieldArchive archive(reader);
  return archive_pole_orientation_override(archive, prefix, *value);
}

bool read_span_endpoint_override(StateReader& reader, const std::string& prefix, SpanEndpointOverride* value) {
  ReadFieldArchive archive(reader);
  return archive_span_endpoint_override(archive, prefix, *value);
}

bool read_span_support_override(StateReader& reader, const std::string& prefix, SpanSupportOverride* value) {
  ReadFieldArchive archive(reader);
  return archive_span_support_override(archive, prefix, *value);
}

template <typename V, typename Read>
bool read_object_id_map(StateReader& reader, const std::string& prefix,
                        std::unordered_map<ObjectId, V>* values, Read read) {
  std::size_t count = 0;
  if (!reader.count(child(prefix, "count"), &count)) return false;
  std::vector<std::uint64_t> keys{};
  if (!reader.record_ids(prefix, count, &keys)) return false;
  for (std::uint64_t key : keys) {
    V value{};
    if (!read(reader, indexed(prefix, key), &value)) return false;
    values->emplace(key, std::move(value));
  }
  return true;
}

bool read_context_profile(StateReader& reader, ContextProfile* value) {
  ReadFieldArchive archive(reader);
  return archive_context_profile(archive, "authoritative.context_profile", *value);
}

bool read_layout_settings(StateReader& reader, LayoutSettings* value) {
  ReadFieldArchive archive(reader);
  return archive_layout_settings(archive, "authoritative.layout_settings", *value);
}

bool read_geometry_settings(StateReader& reader, GeometrySettings* value) {
  ReadFieldArchive archive(reader);
  return archive_geometry_settings(archive, "authoritative.geometry_settings", *value);
}

bool read_visual_settings(StateReader& reader, VisualSettings* value) {
  ReadFieldArchive archive(reader);
  return archive_visual_settings(archive, "authoritative.visual_settings", *value);
}

bool read_variation_settings(StateReader& reader, VariationSettings* value) {
  ReadFieldArchive archive(reader);
  return archive_variation_settings(archive, "authoritative.variation_settings", *value);
}

bool read_identity(StateReader& reader, CoreStateIdentityStorage* identity) {
  ObjectId next_id = 0;
  if (!reader.value("identity.id_generator.next", &next_id) || next_id == kInvalidObjectId ||
      !reader.value("identity.next_data_version", &identity->next_data_version) ||
      !reader.value("identity.next_generation_session_id", &identity->next_generation_session_id)) return false;
  identity->id_generator.reset(next_id);
  std::size_t count = 0;
  if (!reader.count("identity.display_id_counters.count", &count)) return false;
  for (std::size_t i = 0; i < count; ++i) {
    const std::string prefix = "identity.display_id_counters." + std::to_string(i);
    std::string key{};
    std::uint64_t value = 0;
    if (!reader.string_value(child(prefix, "key"), &key) || !reader.value(child(prefix, "value"), &value) ||
        !identity->display_id_counters.emplace(std::move(key), value).second) return false;
  }
  return true;
}

bool read_authoritative(StateReader& reader, CoreStateAuthoritativeStorage* authoritative) {
  return read_edit_state(reader, &authoritative->edit_state) &&
         read_backbone(reader, &authoritative->backbone) &&
         read_map(reader, "authoritative.pole_types", &authoritative->pole_types, read_pole_type) &&
         read_map(reader, "authoritative.cable_templates", &authoritative->cable_templates, read_cable_template) &&
         read_map(reader, "authoritative.bundle_templates", &authoritative->bundle_templates, read_bundle_template) &&
         read_map(reader, "authoritative.attachment_templates", &authoritative->attachment_templates,
                  read_attachment_template) &&
         read_context_profile(reader, &authoritative->context_profile) &&
         read_object_id_map(reader, "authoritative.override_state.pole_orientation_by_pole",
                            &authoritative->override_state.pole_orientation_by_pole,
                            read_pole_orientation_override) &&
         read_object_id_map(reader, "authoritative.override_state.span_endpoint_by_span",
                            &authoritative->override_state.span_endpoint_by_span, read_span_endpoint_override) &&
         read_object_id_map(reader, "authoritative.override_state.span_support_by_span",
                            &authoritative->override_state.span_support_by_span, read_span_support_override) &&
         read_layout_settings(reader, &authoritative->layout_settings) &&
         read_geometry_settings(reader, &authoritative->geometry_settings) &&
         read_visual_settings(reader, &authoritative->visual_settings) &&
         read_variation_settings(reader, &authoritative->variation_settings);
}

#undef READ_VALUE
#undef READ_STRING

} // namespace

EditResult<bool> CoreState::SerializeAuthoritative(std::string* out) const {
  EditResult<bool> result{};
  if (out == nullptr) {
    result.error = "authoritative serialization: output is null";
    return result;
  }

  StateWriter writer{};
  writer.value("identity.id_generator.next", identity_.id_generator.peek());
  writer.value("identity.next_data_version", identity_.next_data_version);
  writer.value("identity.next_generation_session_id", identity_.next_generation_session_id);
  std::vector<std::string> display_prefixes{};
  display_prefixes.reserve(identity_.display_id_counters.size());
  for (const auto& [prefix, value] : identity_.display_id_counters) {
    static_cast<void>(value);
    display_prefixes.push_back(prefix);
  }
  std::sort(display_prefixes.begin(), display_prefixes.end());
  writer.value("identity.display_id_counters.count", display_prefixes.size());
  for (std::size_t i = 0; i < display_prefixes.size(); ++i) {
    const std::string prefix = "identity.display_id_counters." + std::to_string(i);
    writer.string_value(child(prefix, "key"), display_prefixes[i]);
    writer.value(child(prefix, "value"), identity_.display_id_counters.at(display_prefixes[i]));
  }

  write_edit_state(writer, authoritative_.edit_state);
  write_backbone(writer, authoritative_.backbone);
  write_map(writer, "authoritative.pole_types", authoritative_.pole_types, write_pole_type);
  write_map(writer, "authoritative.cable_templates", authoritative_.cable_templates, write_cable_template);
  write_map(writer, "authoritative.bundle_templates", authoritative_.bundle_templates, write_bundle_template);
  write_map(writer, "authoritative.attachment_templates", authoritative_.attachment_templates,
            write_attachment_template);
  write_context_profile(writer, authoritative_.context_profile);
  write_map(writer, "authoritative.override_state.pole_orientation_by_pole",
            authoritative_.override_state.pole_orientation_by_pole, write_pole_orientation_override);
  write_map(writer, "authoritative.override_state.span_endpoint_by_span",
            authoritative_.override_state.span_endpoint_by_span, write_span_endpoint_override);
  write_map(writer, "authoritative.override_state.span_support_by_span",
            authoritative_.override_state.span_support_by_span, write_span_support_override);
  write_layout_settings(writer, authoritative_.layout_settings);
  write_geometry_settings(writer, authoritative_.geometry_settings);
  write_visual_settings(writer, authoritative_.visual_settings);
  write_variation_settings(writer, authoritative_.variation_settings);

  *out = std::move(writer).finish();
  result.ok = true;
  result.value = true;
  return result;
}

EditResult<bool> CoreState::DeserializeAuthoritative(const std::string& text) {
  EditResult<bool> result{};
  StateReader reader{};
  if (!reader.parse(text)) {
    result.error = reader.error();
    return result;
  }

  CoreStateIdentityStorage loaded_identity{};
  CoreStateAuthoritativeStorage loaded_authoritative{};
  if (!read_identity(reader, &loaded_identity) || !read_authoritative(reader, &loaded_authoritative) ||
      !reader.finish()) {
    result.error = reader.error().empty() ? "authoritative deserialization: invalid field" : reader.error();
    return result;
  }

  CoreState trial{};
  trial.identity_ = std::move(loaded_identity);
  trial.authoritative_ = std::move(loaded_authoritative);
  trial.runtime_ = {};
  trial.debug_ = {};

  for (const Port& port : trial.authoritative_.edit_state.ports.items()) {
    index_add(trial.runtime_.relation_index.ports_by_pole, port.owner_pole_id, port.id);
  }
  for (const Anchor& anchor : trial.authoritative_.edit_state.anchors.items()) {
    index_add(trial.runtime_.relation_index.anchors_by_pole, anchor.owner_pole_id, anchor.id);
  }
  for (const Span& span : trial.authoritative_.edit_state.spans.items()) {
    trial.add_span_to_index(span);
    trial.initialize_span_runtime_state(span.id);
  }
  for (const Attachment& attachment : trial.authoritative_.edit_state.attachments.items()) {
    index_add(trial.runtime_.relation_index.attachments_by_span, attachment.span_id, attachment.id);
  }

  const SavedBackboneGraph& graph = trial.authoritative_.backbone;
  for (const SavedBackboneNode& node : graph.nodes) {
    if (node.pole_id != kInvalidObjectId) trial.runtime_.backbone_index.pole_node[node.pole_id] = node.node_id;
  }
  for (const SavedBackboneEdge& edge : graph.edges) {
    index_add(trial.runtime_.backbone_index.node_edges, edge.node_a, edge.edge_id);
    index_add(trial.runtime_.backbone_index.node_edges, edge.node_b, edge.edge_id);
    const BackboneEdgeKey key{std::min(edge.node_a, edge.node_b), std::max(edge.node_a, edge.node_b)};
    if (!trial.runtime_.backbone_index.edge_by_nodes.emplace(key, edge.edge_id).second) {
      result.error = "authoritative deserialization: duplicate saved edge endpoints";
      return result;
    }
  }
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    index_add(trial.runtime_.backbone_index.edge_bundles, edge_bundle.edge_id, edge_bundle.edge_bundle_id);
    index_add(trial.runtime_.backbone_index.bundle_edge, edge_bundle.bundle_id, edge_bundle.edge_id);
    for (ObjectId span_id : edge_bundle.span_ids) {
      index_add(trial.runtime_.backbone_index.edge_bundle_spans, edge_bundle.edge_bundle_id, span_id);
      if (!trial.runtime_.backbone_index.span_edge_bundle.emplace(span_id, edge_bundle.edge_bundle_id).second) {
        result.error = "authoritative deserialization: span belongs to multiple edge bundles";
        return result;
      }
    }
  }
  for (std::size_t i = 0; i < graph.span_bindings.size(); ++i) {
    const SavedBackboneSpanBinding& binding = graph.span_bindings[i];
    trial.runtime_.backbone_index.edge_bundle_span_bindings[binding.edge_bundle_id].push_back(i);
    trial.runtime_.backbone_index.span_bindings_by_span[binding.span_id].push_back(i);
  }
  for (std::size_t i = 0; i < graph.port_bindings.size(); ++i) {
    const SavedBackbonePortBinding& binding = graph.port_bindings[i];
    trial.runtime_.backbone_index.edge_bundle_ports[binding.edge_bundle_id].push_back(i);
    trial.runtime_.backbone_index.port_bindings_by_port[binding.port_id].push_back(i);
  }

  const CoreStateIdentityStorage persisted_identity = trial.identity_;
  const auto rebuilt = trial.rebuild_loaded_outputs();
  if (!rebuilt.ok) {
    result.error = rebuilt.error;
    return result;
  }
  trial.identity_ = persisted_identity;
  const ValidationResult validation = trial.Validate();
  if (!validation.ok()) {
    result.error = "authoritative deserialization: loaded state failed validation";
    return result;
  }

  identity_ = std::move(trial.identity_);
  authoritative_ = std::move(trial.authoritative_);
  runtime_ = std::move(trial.runtime_);
  debug_ = std::move(trial.debug_);
  result.ok = true;
  result.value = true;
  return result;
}

} // namespace wire::core
