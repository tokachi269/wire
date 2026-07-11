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
  template <typename T> bool field(const std::string& prefix, std::string_view name, const T& input) {
    return value(child(prefix, name), input);
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
  template <typename T> bool field(const std::string& prefix, std::string_view name, T& output) {
    return value(child(prefix, name), output);
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


template <typename Archive, typename Value>
bool archive_vec3(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "x", value.x) && archive.field(prefix, "y", value.y) && archive.field(prefix, "z", value.z);
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
  return archive.field(prefix, "generated", value.generated) && archive.field(prefix, "source", value.source) && archive.field(prefix, "generation_session_id", value.generation_session_id) &&
         archive.field(prefix, "generation_order", value.generation_order);
}

template <typename Archive, typename Value>
bool archive_pole_context(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "kind", value.kind) && archive.field(prefix, "corner_angle_deg", value.corner_angle_deg) && archive.field(prefix, "corner_turn_sign", value.corner_turn_sign) &&
         archive.field(prefix, "side_scale", value.side_scale) && archive.field(prefix, "angle_correction_applied", value.angle_correction_applied);
}

template <typename Archive, typename Value>
bool archive_pole(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.string_value(child(prefix, "name"), value.name) &&
         archive_transform(archive, child(prefix, "world_transform"), value.world_transform) &&
         archive.field(prefix, "tilt_magnitude_deg", value.tilt_magnitude_deg) && archive.field(prefix, "height_m", value.height_m) && archive.field(prefix, "kind", value.kind) &&
         archive.field(prefix, "pole_type_id", value.pole_type_id) && archive_pole_context(archive, child(prefix, "context"), value.context) &&
         archive.field(prefix, "placement_mode", value.placement_mode) && archive.field(prefix, "user_edited", value.user_edited) && archive.field(prefix, "placement_override_flag", value.placement_override_flag) &&
         archive_generation(archive, child(prefix, "generation"), value.generation);
}

template <typename Archive, typename Value>
bool archive_port(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "owner_pole_id", value.owner_pole_id) &&
         archive_vec3(archive, child(prefix, "world_position"), value.world_position) && archive.field(prefix, "kind", value.kind) &&
         archive.field(prefix, "layer", value.layer) && archive_frame(archive, child(prefix, "direction"), value.direction) &&
         archive.field(prefix, "category", value.category) && archive.field(prefix, "template_layer", value.template_layer) && archive.field(prefix, "template_side", value.template_side) &&
         archive.field(prefix, "template_role", value.template_role) && archive.field(prefix, "generated_from_template", value.generated_from_template) && archive.field(prefix, "generated_by_rule", value.generated_by_rule) &&
         archive.field(prefix, "placement_context", value.placement_context) && archive.field(prefix, "angle_correction_applied", value.angle_correction_applied) &&
         archive.field(prefix, "side_scale_applied", value.side_scale_applied) && archive.field(prefix, "position_mode", value.position_mode) && archive.field(prefix, "placement_source", value.placement_source) &&
         archive.field(prefix, "user_edited_position", value.user_edited_position) && archive.field(prefix, "placement_override_flag", value.placement_override_flag) &&
         archive.field(prefix, "orientation_override_flag", value.orientation_override_flag);
}

template <typename Archive, typename Value>
bool archive_anchor(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "owner_pole_id", value.owner_pole_id) &&
         archive_vec3(archive, child(prefix, "world_position"), value.world_position) &&
         archive.field(prefix, "support_kind", value.support_kind) && archive.field(prefix, "support_strength", value.support_strength) && archive.field(prefix, "generated_from_template", value.generated_from_template);
}

template <typename Archive, typename Value>
bool archive_bundle(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "conductor_count", value.conductor_count) &&
         archive.field(prefix, "phase_spacing_m", value.phase_spacing_m) && archive.field(prefix, "bundle_template_id", value.bundle_template_id);
}

template <typename Archive, typename Value>
bool archive_span(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "port_a_id", value.port_a_id) &&
         archive.field(prefix, "port_b_id", value.port_b_id) && archive.field(prefix, "endpoint_node_a_id", value.endpoint_node_a_id) && archive.field(prefix, "endpoint_node_b_id", value.endpoint_node_b_id) &&
         archive.field(prefix, "kind", value.kind) && archive.field(prefix, "layer", value.layer) && archive.field(prefix, "bundle_id", value.bundle_id) && archive.field(prefix, "anchor_a_id", value.anchor_a_id) &&
         archive.field(prefix, "anchor_b_id", value.anchor_b_id) && archive.field(prefix, "endpoint_attachment_a_id", value.endpoint_attachment_a_id) &&
         archive.field(prefix, "endpoint_attachment_b_id", value.endpoint_attachment_b_id) && archive.field(prefix, "placement_context", value.placement_context) &&
         archive.field(prefix, "generated_by_rule", value.generated_by_rule) && archive.field(prefix, "placement_override_flag", value.placement_override_flag) &&
         archive.field(prefix, "reference_length_m", value.reference_length_m) && archive_generation(archive, child(prefix, "generation"), value.generation);
}

template <typename Archive, typename Value>
bool archive_attachment(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "span_id", value.span_id) && archive.field(prefix, "template_id", value.template_id) &&
         archive.field(prefix, "t", value.t) && archive.field(prefix, "kind", value.kind) && archive.field(prefix, "display_offset_m", value.display_offset_m) && archive.field(prefix, "origin", value.origin);
}

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


template <typename Archive, typename Value>
bool archive_bundle_mode(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "bundle_template_id", value.bundle_template_id) && archive.field(prefix, "mode", value.mode);
}

template <typename Archive, typename Value>
bool archive_saved_node(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "node_id", value.node_id) || !archive.field(prefix, "pole_id", value.pole_id) || !archive.field(prefix, "support_kind", value.support_kind) ||
      !archive_vec3(archive, child(prefix, "position"), value.position) || !archive.field(prefix, "has_source_edge", value.has_source_edge) ||
      !archive.field(prefix, "source_edge_node_a", value.source_edge_node_a) || !archive.field(prefix, "source_edge_node_b", value.source_edge_node_b) ||
      !archive.field(prefix, "source_edge_t", value.source_edge_t) || !archive.field(prefix, "path_point_index", value.path_point_index)) return false;
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
  return archive.field(prefix, "edge_id", value.edge_id) && archive.field(prefix, "node_a", value.node_a) && archive.field(prefix, "node_b", value.node_b) && archive.field(prefix, "route", value.route) &&
         archive.field(prefix, "order", value.order) && archive_vec3(archive, child(prefix, "dir"), value.dir) &&
         archive.field(prefix, "lateral_offset_m", value.lateral_offset_m);
}

template <typename Archive, typename Value>
bool archive_saved_edge_bundle(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) || !archive.field(prefix, "edge_id", value.edge_id) || !archive.field(prefix, "bundle_id", value.bundle_id) ||
      !archive.field(prefix, "edge_forward", value.edge_forward) || !archive.field(prefix, "route", value.route) || !archive.field(prefix, "order", value.order) ||
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
  return archive.field(prefix, "node_id", value.node_id) && archive.field(prefix, "source_is_open", value.source_is_open) && archive.field(prefix, "source_edge_a", value.source_edge_a) &&
         archive.field(prefix, "source_edge_b", value.source_edge_b);
}

template <typename Archive, typename Value>
bool archive_saved_port_binding(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) && archive_row_key(archive, child(prefix, "row_key"), value.row_key) &&
         archive.field(prefix, "lane_index", value.lane_index) && archive.field(prefix, "bundle_template_id", value.bundle_template_id) && archive.field(prefix, "port_kind", value.port_kind) &&
         archive.field(prefix, "port_layer", value.port_layer) && archive.field(prefix, "placement_band_id", value.placement_band_id) && archive.field(prefix, "layout_yaw_deg", value.layout_yaw_deg) &&
         archive.field(prefix, "port_id", value.port_id);
}

template <typename Archive, typename Value>
bool archive_saved_span_binding(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) && archive.field(prefix, "lane_index", value.lane_index) && archive.field(prefix, "span_id", value.span_id);
}

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


template <typename Archive, typename Value>
bool archive_port_band(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "band_id", value.band_id) && archive.field(prefix, "category", value.category) &&
         archive_frame(archive, child(prefix, "local_direction"), value.local_direction) &&
         archive.field(prefix, "layer", value.layer) && archive.field(prefix, "side", value.side) && archive.field(prefix, "role", value.role) &&
         archive.field(prefix, "lateral_center_m", value.lateral_center_m) && archive.field(prefix, "lateral_min_m", value.lateral_min_m) && archive.field(prefix, "lateral_max_m", value.lateral_max_m) &&
         archive.field(prefix, "height_center_m", value.height_center_m) && archive.field(prefix, "height_min_m", value.height_min_m) && archive.field(prefix, "height_max_m", value.height_max_m) &&
         archive.field(prefix, "priority", value.priority) && archive.field(prefix, "min_spacing_m", value.min_spacing_m) && archive.field(prefix, "allow_multiple", value.allow_multiple) &&
         archive.field(prefix, "overflow_policy", value.overflow_policy) && archive.field(prefix, "enabled", value.enabled);
}

template <typename Archive, typename Value>
bool archive_anchor_slot(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "slot_id", value.slot_id) && archive.field(prefix, "usage", value.usage) &&
         archive_vec3(archive, child(prefix, "local_position"), value.local_position) &&
         archive.field(prefix, "priority", value.priority) && archive.field(prefix, "enabled", value.enabled);
}

template <typename Archive, typename Value>
bool archive_pole_type(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "id", value.id) || !archive.string_value(child(prefix, "name"), value.name) || !archive.string_value(child(prefix, "description"), value.description) ||
      !archive.field(prefix, "default_height_m", value.default_height_m)) return false;
  std::size_t port_band_count = value.port_bands.size();
  if (!archive.count(child(prefix, "port_bands.count"), port_band_count)) return false;
  if constexpr (Archive::loading) value.port_bands.resize(port_band_count);
  for (std::size_t i = 0; i < port_band_count; ++i) {
    if (!archive_port_band(archive, indexed(child(prefix, "port_bands"), i), value.port_bands[i])) return false;
  }
  std::size_t anchor_slot_count = value.anchor_slots.size();
  if (!archive.count(child(prefix, "anchor_slots.count"), anchor_slot_count)) return false;
  if constexpr (Archive::loading) value.anchor_slots.resize(anchor_slot_count);
  for (std::size_t i = 0; i < anchor_slot_count; ++i) {
    if (!archive_anchor_slot(archive, indexed(child(prefix, "anchor_slots"), i), value.anchor_slots[i])) return false;
  }
  return true;
}

#define WRITE_POLE_TEMPLATE_WRAPPER(name, type)                                                                        \
  void write_##name(StateWriter& writer, const std::string& prefix, const type& value) {                              \
    WriteFieldArchive archive(writer);                                                                                 \
    static_cast<void>(archive_##name(archive, prefix, value));                                                         \
  }

WRITE_POLE_TEMPLATE_WRAPPER(port_band, PortPlacementBand)
WRITE_POLE_TEMPLATE_WRAPPER(anchor_slot, AnchorSlotTemplate)
WRITE_POLE_TEMPLATE_WRAPPER(pole_type, PoleTypeDefinition)

#undef WRITE_POLE_TEMPLATE_WRAPPER

template <typename Archive, typename Value>
bool archive_supplemental_path(Archive& ar, const std::string& p, Value& v) {
  return ar.field(p, "anchor_mode", v.anchor_mode) && ar.field(p, "profile_kind", v.profile_kind) &&
         ar.field(p, "interaction_mode", v.interaction_mode) && ar.field(p, "pole_band_id", v.pole_band_id) &&
         ar.field(p, "endpoint_trim_m", v.endpoint_trim_m) && ar.field(p, "lateral_offset_m", v.lateral_offset_m) &&
         ar.field(p, "vertical_offset_m", v.vertical_offset_m) && ar.field(p, "wobble_amplitude_m", v.wobble_amplitude_m) &&
         ar.field(p, "wobble_wavelength_m", v.wobble_wavelength_m) && ar.field(p, "wobble_phase_bias", v.wobble_phase_bias) &&
         ar.field(p, "endpoint_envelope_ratio", v.endpoint_envelope_ratio) && ar.field(p, "coil_radius_m", v.coil_radius_m) &&
         ar.field(p, "coil_turns_per_meter", v.coil_turns_per_meter) && ar.field(p, "coil_samples_per_turn", v.coil_samples_per_turn);
}

template <typename Archive, typename Value>
bool archive_cable_template(Archive& ar, const std::string& p, Value& v) {
  if (!ar.field(p, "id", v.id) || !ar.string_value(child(p, "name"), v.name) ||
      !ar.field(p, "outer_diameter_m", v.outer_diameter_m) ||
      !ar.field(p, "default_grouped_support_fanout_spacing_m", v.default_grouped_support_fanout_spacing_m) ||
      !ar.field(p, "bend_stiffness", v.bend_stiffness) || !ar.field(p, "min_bend_radius_m", v.min_bend_radius_m) ||
      !ar.field(p, "material_style", v.material_style) || !ar.field(p, "color_rgba", v.color_rgba) ||
      !ar.field(p, "requires_insulator", v.requires_insulator) ||
      !ar.field(p, "insulator_attachment_height_m", v.insulator_attachment_height_m) ||
      !ar.field(p, "sag_factor", v.sag_factor) || !ar.field(p, "slack_factor", v.slack_factor) ||
      !ar.field(p, "continuity_policy", v.continuity_policy) || !ar.field(p, "attachment_style", v.attachment_style) ||
      !ar.field(p, "default_endpoint_attachment_template_id", v.default_endpoint_attachment_template_id)) return false;
  std::size_t count = v.supplemental_paths.size();
  if (!ar.count(child(p, "supplemental_paths.count"), count)) return false;
  if constexpr (Archive::loading) v.supplemental_paths.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive_supplemental_path(ar, indexed(child(p, "supplemental_paths"), i), v.supplemental_paths[i])) return false;
  }
  return ar.field(p, "version", v.version);
}

template <typename Archive, typename Value>
bool archive_reserve(Archive& ar, const std::string& p, Value& v) {
  return ar.field(p, "reserve_id", v.reserve_id) && ar.field(p, "pole_type_id", v.pole_type_id) &&
         ar.field(p, "band_id", v.band_id) && ar.field(p, "lateral_min_m", v.lateral_min_m) &&
         ar.field(p, "lateral_max_m", v.lateral_max_m) && ar.field(p, "height_min_m", v.height_min_m) &&
         ar.field(p, "height_max_m", v.height_max_m);
}

template <typename Archive, typename Value>
bool archive_population_rule(Archive& ar, const std::string& p, Value& v) {
  if (!ar.field(p, "rule_id", v.rule_id) || !ar.field(p, "explicit_seed", v.explicit_seed) ||
      !ar.field(p, "priority", v.priority) || !ar.field(p, "min_extra_count", v.min_extra_count) ||
      !ar.field(p, "max_extra_count", v.max_extra_count) || !ar.field(p, "min_spacing_m", v.min_spacing_m) ||
      !ar.field(p, "lateral_min_m", v.lateral_min_m) || !ar.field(p, "lateral_max_m", v.lateral_max_m) ||
      !ar.field(p, "height_min_m", v.height_min_m) || !ar.field(p, "height_max_m", v.height_max_m) ||
      !ar.field(p, "randomness", v.randomness) || !ar.field(p, "profile", v.profile) ||
      !ar.field(p, "wrap_radius_m", v.wrap_radius_m) || !ar.field(p, "wrap_turns_per_meter", v.wrap_turns_per_meter) ||
      !ar.field(p, "wrap_phase", v.wrap_phase) || !ar.field(p, "wrap_direction", v.wrap_direction) ||
      !ar.field(p, "end_trim_m", v.end_trim_m)) return false;
  std::size_t count = v.reserves.size();
  if (!ar.count(child(p, "reserves.count"), count)) return false;
  if constexpr (Archive::loading) v.reserves.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive_reserve(ar, indexed(child(p, "reserves"), i), v.reserves[i])) return false;
  }
  return true;
}

template <typename Archive, typename Value>
bool archive_bundle_template(Archive& ar, const std::string& p, Value& v) {
  if (!ar.field(p, "id", v.id) || !ar.field(p, "kind", v.kind) || !ar.string_value(child(p, "name"), v.name) ||
      !ar.field(p, "category", v.category) || !ar.field(p, "cable_template_id", v.cable_template_id) ||
      !ar.field(p, "default_layer", v.default_layer) || !ar.field(p, "related_pole_type_id", v.related_pole_type_id) ||
      !ar.field(p, "preserve_conductor_identity", v.preserve_conductor_identity) || !ar.field(p, "count_rule", v.count_rule) ||
      !ar.field(p, "fixed_count", v.fixed_count) || !ar.field(p, "min_count", v.min_count) ||
      !ar.field(p, "max_count", v.max_count) || !ar.field(p, "default_count", v.default_count) ||
      !ar.field(p, "default_spacing_m", v.default_spacing_m) ||
      !ar.field(p, "grouped_support_fanout_spacing_m", v.grouped_support_fanout_spacing_m) ||
      !ar.field(p, "allow_mirror", v.allow_mirror) || !ar.field(p, "allow_midair_node", v.allow_midair_node) ||
      !ar.field(p, "allow_midair_branch", v.allow_midair_branch) ||
      !ar.field(p, "enable_branch_down_offset", v.enable_branch_down_offset) ||
      !ar.field(p, "branch_endpoint_offset_m", v.branch_endpoint_offset_m) ||
      !ar.field(p, "order_decision_policy", v.order_decision_policy) ||
      !ar.field(p, "row_layout_axis_mode", v.row_layout_axis_mode) || !ar.field(p, "support_style", v.support_style) ||
      !ar.field(p, "branch_policy", v.branch_policy) || !ar.field(p, "continuity_policy", v.continuity_policy) ||
      !ar.field(p, "support_wire_pole_band_id", v.support_wire_pole_band_id)) return false;
  std::size_t count = v.population_rules.size();
  if (!ar.count(child(p, "population_rules.count"), count)) return false;
  if constexpr (Archive::loading) v.population_rules.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive_population_rule(ar, indexed(child(p, "population_rules"), i), v.population_rules[i])) return false;
  }
  return ar.field(p, "version", v.version);
}

void write_supplemental_path(StateWriter& writer, const std::string& prefix,
                             const CableSupplementalPathTemplate& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_supplemental_path(archive, prefix, value));
}

void write_cable_template(StateWriter& writer, const std::string& prefix, const CableTemplate& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_cable_template(archive, prefix, value));
}

void write_reserve(StateWriter& writer, const std::string& prefix, const PlacementReserve& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_reserve(archive, prefix, value));
}

void write_population_rule(StateWriter& writer, const std::string& prefix, const CablePopulationRule& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_population_rule(archive, prefix, value));
}

void write_bundle_template(StateWriter& writer, const std::string& prefix, const BundleTemplate& value) {
  WriteFieldArchive archive(writer);
  static_cast<void>(archive_bundle_template(archive, prefix, value));
}


template <typename Archive, typename Value>
bool archive_attachment_socket(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive_vec3(archive, child(prefix, "local_position"), value.local_position) &&
         archive_vec3(archive, child(prefix, "tangent_dir"), value.tangent_dir) && archive.field(prefix, "has_normal", value.has_normal) &&
         archive_vec3(archive, child(prefix, "normal_dir"), value.normal_dir) && archive.field(prefix, "has_binormal", value.has_binormal) &&
         archive_vec3(archive, child(prefix, "binormal_dir"), value.binormal_dir) && archive.field(prefix, "kind", value.kind);
}

template <typename Archive, typename Value>
bool archive_internal_path(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "start_socket_id", value.start_socket_id) || !archive.field(prefix, "end_socket_id", value.end_socket_id) || !archive.field(prefix, "profile_kind", value.profile_kind)) return false;
  std::size_t point_count = value.local_points.size();
  if (!archive.count(child(prefix, "local_points.count"), point_count)) return false;
  if constexpr (Archive::loading) value.local_points.resize(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    if (!archive_vec3(archive, indexed(child(prefix, "local_points"), i), value.local_points[i])) return false;
  }
  return archive.field(prefix, "coil_radius_m", value.coil_radius_m) && archive.field(prefix, "coil_turn_count", value.coil_turn_count) && archive.field(prefix, "coil_samples_per_turn", value.coil_samples_per_turn);
}

template <typename Archive, typename Value>
bool archive_attachment_template(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "id", value.id) || !archive.string_value(child(prefix, "name"), value.name) || !archive.field(prefix, "kind", value.kind) ||
      !archive.field(prefix, "line_interaction_mode", value.line_interaction_mode)) return false;
  std::size_t socket_count = value.sockets.size();
  if (!archive.count(child(prefix, "sockets.count"), socket_count)) return false;
  if constexpr (Archive::loading) value.sockets.resize(socket_count);
  for (std::size_t i = 0; i < socket_count; ++i) {
    if (!archive_attachment_socket(archive, indexed(child(prefix, "sockets"), i), value.sockets[i])) return false;
  }
  std::size_t path_count = value.internal_paths.size();
  if (!archive.count(child(prefix, "internal_paths.count"), path_count)) return false;
  if constexpr (Archive::loading) value.internal_paths.resize(path_count);
  for (std::size_t i = 0; i < path_count; ++i) {
    if (!archive_internal_path(archive, indexed(child(prefix, "internal_paths"), i), value.internal_paths[i])) return false;
  }
  return archive.field(prefix, "version", value.version);
}

#define WRITE_ATTACHMENT_TEMPLATE_WRAPPER(name, type)                                                                  \
  void write_##name(StateWriter& writer, const std::string& prefix, const type& value) {                              \
    WriteFieldArchive archive(writer);                                                                                 \
    static_cast<void>(archive_##name(archive, prefix, value));                                                         \
  }

WRITE_ATTACHMENT_TEMPLATE_WRAPPER(attachment_socket, AttachmentSocketTemplate)
WRITE_ATTACHMENT_TEMPLATE_WRAPPER(internal_path, AttachmentInternalPathTemplate)
WRITE_ATTACHMENT_TEMPLATE_WRAPPER(attachment_template, AttachmentTemplate)

#undef WRITE_ATTACHMENT_TEMPLATE_WRAPPER

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


template <typename Archive, typename Value>
bool archive_context_profile(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "age", value.age) && archive.field(prefix, "clutter", value.clutter) && archive.field(prefix, "regularity", value.regularity) &&
         archive.field(prefix, "service_mix", value.service_mix) && archive.field(prefix, "style_seed", value.style_seed);
}

template <typename Archive, typename Value>
bool archive_layout_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "angle_correction_enabled", value.angle_correction_enabled) && archive.field(prefix, "corner_threshold_deg", value.corner_threshold_deg) &&
         archive.field(prefix, "min_side_scale", value.min_side_scale) && archive.field(prefix, "max_side_scale", value.max_side_scale);
}

template <typename Archive, typename Value>
bool archive_geometry_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "curve_samples", value.curve_samples) && archive.field(prefix, "sag_enabled", value.sag_enabled) && archive.field(prefix, "sag_factor", value.sag_factor) &&
         archive.field(prefix, "pole_clearance_m", value.pole_clearance_m);
}

template <typename Archive, typename Value>
bool archive_visual_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "enable_support_structures", value.enable_support_structures) && archive.field(prefix, "enable_insulators", value.enable_insulators) &&
         archive.field(prefix, "support_center_threshold_m", value.support_center_threshold_m) && archive.field(prefix, "support_arm_extra_m", value.support_arm_extra_m) &&
         archive.field(prefix, "support_arm_radius_m", value.support_arm_radius_m) && archive.field(prefix, "insulator_radius_m", value.insulator_radius_m) &&
         archive.field(prefix, "insulator_length_m", value.insulator_length_m);
}

template <typename Archive, typename Value>
bool archive_variation_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "enabled", value.enabled) && archive.field(prefix, "global_seed", value.global_seed) && archive.field(prefix, "world_cell_size_m", value.world_cell_size_m) &&
         archive.field(prefix, "world_bias_scale", value.world_bias_scale) && archive.field(prefix, "flow_bias_scale", value.flow_bias_scale) &&
         archive.field(prefix, "pole_delta_scale", value.pole_delta_scale) && archive.field(prefix, "local_jitter_scale", value.local_jitter_scale) &&
         archive.field(prefix, "sag_variation_scale", value.sag_variation_scale) && archive.field(prefix, "branch_down_offset_variation_scale", value.branch_down_offset_variation_scale);
}

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

#define READ_POLE_TEMPLATE_WRAPPER(name, type)                                                                         \
  bool read_##name(StateReader& reader, const std::string& prefix, type* value) {                                     \
    ReadFieldArchive archive(reader);                                                                                  \
    return archive_##name(archive, prefix, *value);                                                                    \
  }

READ_POLE_TEMPLATE_WRAPPER(port_band, PortPlacementBand)
READ_POLE_TEMPLATE_WRAPPER(anchor_slot, AnchorSlotTemplate)
READ_POLE_TEMPLATE_WRAPPER(pole_type, PoleTypeDefinition)

#undef READ_POLE_TEMPLATE_WRAPPER

#define READ_CABLE_TEMPLATE_WRAPPER(name, type)                                                                        \
  bool read_##name(StateReader& reader, const std::string& prefix, type* value) {                                     \
    ReadFieldArchive archive(reader);                                                                                  \
    return archive_##name(archive, prefix, *value);                                                                    \
  }

READ_CABLE_TEMPLATE_WRAPPER(supplemental_path, CableSupplementalPathTemplate)
READ_CABLE_TEMPLATE_WRAPPER(cable_template, CableTemplate)
READ_CABLE_TEMPLATE_WRAPPER(reserve, PlacementReserve)
READ_CABLE_TEMPLATE_WRAPPER(population_rule, CablePopulationRule)
READ_CABLE_TEMPLATE_WRAPPER(bundle_template, BundleTemplate)

#undef READ_CABLE_TEMPLATE_WRAPPER

#define READ_ATTACHMENT_TEMPLATE_WRAPPER(name, type)                                                                   \
  bool read_##name(StateReader& reader, const std::string& prefix, type* value) {                                     \
    ReadFieldArchive archive(reader);                                                                                  \
    return archive_##name(archive, prefix, *value);                                                                    \
  }

READ_ATTACHMENT_TEMPLATE_WRAPPER(attachment_socket, AttachmentSocketTemplate)
READ_ATTACHMENT_TEMPLATE_WRAPPER(internal_path, AttachmentInternalPathTemplate)
READ_ATTACHMENT_TEMPLATE_WRAPPER(attachment_template, AttachmentTemplate)

#undef READ_ATTACHMENT_TEMPLATE_WRAPPER

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
