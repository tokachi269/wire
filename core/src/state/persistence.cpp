#include "wire/core/core_state.hpp"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {
namespace {

class StateWriter {
public:
  StateWriter() { text_ = "wire_state_v2\n"; }

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
    static constexpr std::string_view kHeaderV1 = "wire_state_v1\n";
    static constexpr std::string_view kHeaderV2 = "wire_state_v2\n";
    std::size_t header_size = 0;
    if (text.starts_with(kHeaderV2)) {
      version_ = 2;
      header_size = kHeaderV2.size();
    } else if (text.starts_with(kHeaderV1)) {
      version_ = 1;
      header_size = kHeaderV1.size();
    } else {
      error_ = "authoritative deserialization: unsupported or missing version";
      return false;
    }
    if (text.empty() || text.back() != '\n') {
      error_ = "authoritative deserialization: truncated final line";
      return false;
    }
    std::size_t line_begin = header_size;
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

  [[nodiscard]] bool contains(const std::string& key) const {
    return values_.contains(key);
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

  [[nodiscard]] int version() const { return version_; }

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
  int version_ = 0;
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
  template <typename T>
  bool compatible_field(const std::string& prefix, std::string_view name, const T& input, const T&) {
    return field(prefix, name, input);
  }
  template <typename T>
  bool legacy_field(const std::string&, std::string_view, const T&, const T&) {
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
  template <typename T> bool field(const std::string& prefix, std::string_view name, T& output) {
    return value(child(prefix, name), output);
  }
  template <typename T>
  bool compatible_field(const std::string& prefix, std::string_view name, T& output, const T& legacy_default) {
    const std::string key = child(prefix, name);
    if (!reader_.contains(key)) {
      output = legacy_default;
      return true;
    }
    return value(key, output);
  }
  template <typename T>
  bool legacy_field(const std::string& prefix, std::string_view name, T& output, const T& legacy_default) {
    return compatible_field(prefix, name, output, legacy_default);
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

#ifdef _MSC_VER
static_assert(sizeof(Vec3d) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_transform(Archive& archive, const std::string& prefix, Value& value) {
  return archive_vec3(archive, child(prefix, "position"), value.position) &&
         archive_vec3(archive, child(prefix, "rotation_euler_deg"), value.rotation_euler_deg) &&
         archive_vec3(archive, child(prefix, "scale"), value.scale);
}

#ifdef _MSC_VER
static_assert(sizeof(Transformd) == 72, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_frame(Archive& archive, const std::string& prefix, Value& value) {
  return archive_vec3(archive, child(prefix, "origin"), value.origin) &&
         archive_vec3(archive, child(prefix, "forward"), value.forward) &&
         archive_vec3(archive, child(prefix, "right"), value.right) &&
         archive_vec3(archive, child(prefix, "up"), value.up);
}

#ifdef _MSC_VER
static_assert(sizeof(Frame3d) == 96, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_generation(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "generated", value.generated) && archive.field(prefix, "source", value.source) && archive.field(prefix, "generation_session_id", value.generation_session_id) &&
         archive.field(prefix, "generation_order", value.generation_order);
}

#ifdef _MSC_VER
static_assert(sizeof(GenerationMeta) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_pole_context(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "kind", value.kind) && archive.field(prefix, "corner_angle_deg", value.corner_angle_deg) && archive.field(prefix, "corner_turn_sign", value.corner_turn_sign) &&
         archive.field(prefix, "side_scale", value.side_scale) && archive.field(prefix, "angle_correction_applied", value.angle_correction_applied);
}

#ifdef _MSC_VER
static_assert(sizeof(PoleContextInfo) == 40, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_pole(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.string_value(child(prefix, "name"), value.name) &&
         archive_transform(archive, child(prefix, "world_transform"), value.world_transform) &&
         archive.field(prefix, "tilt_magnitude_deg", value.tilt_magnitude_deg) && archive.field(prefix, "height_m", value.height_m) && archive.field(prefix, "kind", value.kind) &&
         archive.field(prefix, "pole_type_id", value.pole_type_id) && archive_pole_context(archive, child(prefix, "context"), value.context) &&
         archive.field(prefix, "placement_mode", value.placement_mode) && archive.field(prefix, "user_edited", value.user_edited) && archive.field(prefix, "placement_override_flag", value.placement_override_flag) &&
         archive_generation(archive, child(prefix, "generation"), value.generation);
}

#ifdef _MSC_VER
static_assert(sizeof(Pole) == 256, "field added: update archive visitor and full-fat persistence fixture");
#endif

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

#ifdef _MSC_VER
static_assert(sizeof(Port) == 216, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_anchor(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "owner_pole_id", value.owner_pole_id) &&
         archive_vec3(archive, child(prefix, "world_position"), value.world_position) &&
         archive.field(prefix, "support_kind", value.support_kind) && archive.field(prefix, "support_strength", value.support_strength) && archive.field(prefix, "generated_from_template", value.generated_from_template);
}

#ifdef _MSC_VER
static_assert(sizeof(Anchor) == 104, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_bundle(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "conductor_count", value.conductor_count) &&
         archive.field(prefix, "phase_spacing_m", value.phase_spacing_m) &&
         archive.compatible_field(prefix, "spacing_override_m", value.spacing_override_m, 0.0) &&
         archive.compatible_field(prefix, "placement_explicit", value.placement_explicit, false) &&
         archive.compatible_field(prefix, "height_m", value.height_m, 0.0) &&
         archive.compatible_field(prefix, "lateral_m", value.lateral_m, 0.0) &&
         archive.field(prefix, "bundle_template_id", value.bundle_template_id) &&
         archive.compatible_field(prefix, "placement_key", value.placement_key, static_cast<std::uint64_t>(0));
}

#ifdef _MSC_VER
static_assert(sizeof(Bundle) == 112, "field added: update archive visitor and full-fat persistence fixture");
#endif

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

#ifdef _MSC_VER
static_assert(sizeof(Span) == 168, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_attachment(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive.string_value(child(prefix, "display_id"), value.display_id) && archive.field(prefix, "span_id", value.span_id) && archive.field(prefix, "template_id", value.template_id) &&
         archive.field(prefix, "t", value.t) && archive.field(prefix, "kind", value.kind) && archive.field(prefix, "display_offset_m", value.display_offset_m) && archive.field(prefix, "origin", value.origin);
}

#ifdef _MSC_VER
static_assert(sizeof(Attachment) == 96, "field added: update archive visitor and full-fat persistence fixture");
#endif

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

template <typename FieldArchive>
void write_edit_state_as(StateWriter& writer, const EditState& state) {
  write_object_store(writer, "authoritative.edit_state.poles", state.poles,
                     [](auto& out, const auto& prefix, const Pole& value) { FieldArchive a(out); (void)archive_pole(a, prefix, value); });
  write_object_store(writer, "authoritative.edit_state.ports", state.ports,
                     [](auto& out, const auto& prefix, const Port& value) { FieldArchive a(out); (void)archive_port(a, prefix, value); });
  write_object_store(writer, "authoritative.edit_state.anchors", state.anchors,
                     [](auto& out, const auto& prefix, const Anchor& value) { FieldArchive a(out); (void)archive_anchor(a, prefix, value); });
  write_object_store(writer, "authoritative.edit_state.bundles", state.bundles,
                     [](auto& out, const auto& prefix, const Bundle& value) { FieldArchive a(out); (void)archive_bundle(a, prefix, value); });
  write_object_store(writer, "authoritative.edit_state.spans", state.spans,
                     [](auto& out, const auto& prefix, const Span& value) { FieldArchive a(out); (void)archive_span(a, prefix, value); });
  write_object_store(writer, "authoritative.edit_state.attachments", state.attachments,
                     [](auto& out, const auto& prefix, const Attachment& value) { FieldArchive a(out); (void)archive_attachment(a, prefix, value); });
}


template <typename Archive, typename Value>
bool archive_bundle_mode(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "bundle_template_id", value.bundle_template_id) && archive.field(prefix, "mode", value.mode);
}

#ifdef _MSC_VER
static_assert(sizeof(SupportNodeBundleMode) == 8, "field added: update archive visitor and full-fat persistence fixture");
#endif

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

#ifdef _MSC_VER
static_assert(sizeof(SavedBackboneNode) == 120, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_saved_edge(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "edge_id", value.edge_id) && archive.field(prefix, "node_a", value.node_a) && archive.field(prefix, "node_b", value.node_b) &&
         archive.legacy_field(prefix, "route", value.route, std::size_t{0}) &&
         archive.legacy_field(prefix, "order", value.order, std::size_t{0}) &&
         archive_vec3(archive, child(prefix, "dir"), value.dir) &&
         archive.field(prefix, "lateral_offset_m", value.lateral_offset_m);
}

#ifdef _MSC_VER
static_assert(sizeof(SavedBackboneEdge) == 72, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_saved_edge_bundle(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) || !archive.field(prefix, "edge_id", value.edge_id) || !archive.field(prefix, "bundle_id", value.bundle_id) ||
      !archive.field(prefix, "edge_forward", value.edge_forward) ||
      !archive.legacy_field(prefix, "route", value.route, std::size_t{0}) ||
      !archive.legacy_field(prefix, "order", value.order, std::size_t{0}) ||
      !archive_vec3(archive, child(prefix, "dir"), value.dir)) return false;
  std::size_t count = value.span_ids.size();
  if (!archive.count(child(prefix, "span_ids.count"), count)) return false;
  if constexpr (Archive::loading) value.span_ids.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive.value(indexed(child(prefix, "span_ids"), i), value.span_ids[i])) return false;
  }
  return true;
}

#ifdef _MSC_VER
static_assert(sizeof(SavedBackboneEdgeBundle) == 104, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_row_key(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "node_id", value.node_id) &&
         archive.field(prefix, "edge_id", value.edge_id);
}

#ifdef _MSC_VER
static_assert(sizeof(SavedBackboneRowKey) == 16, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_saved_port_binding(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) && archive_row_key(archive, child(prefix, "row_key"), value.row_key) &&
         archive.field(prefix, "lane_index", value.lane_index) && archive.field(prefix, "bundle_template_id", value.bundle_template_id) && archive.field(prefix, "port_kind", value.port_kind) &&
         archive.field(prefix, "port_layer", value.port_layer) && archive.field(prefix, "placement_band_id", value.placement_band_id) &&
         archive.compatible_field(prefix, "support_level", value.support_level, -1) &&
         archive.compatible_field(prefix, "support_group_id", value.support_group_id, -2) &&
         archive.field(prefix, "layout_yaw_deg", value.layout_yaw_deg) &&
         archive.field(prefix, "port_id", value.port_id);
}

struct LegacySavedBackboneRowKey {
  ObjectId node_id = kInvalidObjectId;
  bool source_is_open = false;
  ObjectId source_edge_a = kInvalidObjectId;
  ObjectId source_edge_b = kInvalidObjectId;
};

bool read_legacy_saved_port_binding(StateReader& reader, const std::string& prefix,
                                    SavedBackbonePortBinding* value,
                                    LegacySavedBackboneRowKey* legacy_row_key) {
  if (value == nullptr || legacy_row_key == nullptr) {
    return false;
  }
  ReadFieldArchive archive(reader);
  const std::string row_prefix = child(prefix, "row_key");
  return archive.field(prefix, "edge_bundle_id", value->edge_bundle_id) &&
         archive.field(row_prefix, "node_id", legacy_row_key->node_id) &&
         archive.field(row_prefix, "source_is_open",
                       legacy_row_key->source_is_open) &&
         archive.field(row_prefix, "source_edge_a",
                       legacy_row_key->source_edge_a) &&
         archive.field(row_prefix, "source_edge_b",
                       legacy_row_key->source_edge_b) &&
         archive.field(prefix, "lane_index", value->lane_index) &&
         archive.field(prefix, "bundle_template_id",
                       value->bundle_template_id) &&
         archive.field(prefix, "port_kind", value->port_kind) &&
         archive.field(prefix, "port_layer", value->port_layer) &&
         archive.field(prefix, "placement_band_id",
                       value->placement_band_id) &&
         archive.compatible_field(prefix, "support_level",
                                  value->support_level, -1) &&
         archive.compatible_field(prefix, "support_group_id",
                                  value->support_group_id, -2) &&
         archive.field(prefix, "layout_yaw_deg", value->layout_yaw_deg) &&
         archive.field(prefix, "port_id", value->port_id);
}

#ifdef _MSC_VER
static_assert(sizeof(SavedBackbonePortBinding) == 72, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_saved_span_binding(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) && archive.field(prefix, "lane_index", value.lane_index) && archive.field(prefix, "span_id", value.span_id);
}

#ifdef _MSC_VER
static_assert(sizeof(SavedBackboneSpanBinding) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_saved_row_continuity_endpoint(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "edge_bundle_id", value.edge_bundle_id) &&
         archive.field(prefix, "lane_index", value.lane_index);
}

template <typename Archive, typename Value>
bool archive_saved_row_continuity(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "node_id", value.node_id) &&
         archive_saved_row_continuity_endpoint(archive, child(prefix, "a"), value.a) &&
         archive_saved_row_continuity_endpoint(archive, child(prefix, "b"), value.b);
}

#ifdef _MSC_VER
static_assert(sizeof(SavedBackboneRowContinuityEndpoint) == 16, "field added: update archive visitor and full-fat persistence fixture");
static_assert(sizeof(SavedBackboneRowContinuity) == 40, "field added: update archive visitor and full-fat persistence fixture");
#endif

ObjectId migrated_edge_bundle_bundle_id(const SavedBackboneGraph& graph, ObjectId edge_bundle_id) {
  const auto found = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                                  [&](const SavedBackboneEdgeBundle& edge_bundle) {
                                    return edge_bundle.edge_bundle_id == edge_bundle_id;
                                  });
  return found == graph.edge_bundles.end() ? kInvalidObjectId : found->bundle_id;
}

void append_migrated_row_continuity(SavedBackboneGraph* graph,
                                    ObjectId node_id,
                                    ObjectId edge_bundle_a,
                                    std::size_t lane_a,
                                    ObjectId edge_bundle_b,
                                    std::size_t lane_b) {
  if (graph == nullptr || node_id == kInvalidObjectId ||
      edge_bundle_a == kInvalidObjectId || edge_bundle_b == kInvalidObjectId ||
      edge_bundle_a == edge_bundle_b) {
    return;
  }
  for (const SavedBackboneRowContinuity& existing : graph->row_continuities) {
    const bool forward = existing.node_id == node_id &&
                         existing.a.edge_bundle_id == edge_bundle_a &&
                         existing.a.lane_index == lane_a &&
                         existing.b.edge_bundle_id == edge_bundle_b &&
                         existing.b.lane_index == lane_b;
    const bool reverse = existing.node_id == node_id &&
                         existing.a.edge_bundle_id == edge_bundle_b &&
                         existing.a.lane_index == lane_b &&
                         existing.b.edge_bundle_id == edge_bundle_a &&
                         existing.b.lane_index == lane_a;
    if (forward || reverse) {
      return;
    }
  }
  SavedBackboneRowContinuity continuity{};
  continuity.node_id = node_id;
  continuity.a.edge_bundle_id = edge_bundle_a;
  continuity.a.lane_index = lane_a;
  continuity.b.edge_bundle_id = edge_bundle_b;
  continuity.b.lane_index = lane_b;
  graph->row_continuities.push_back(continuity);
}

void migrate_v1_row_continuities(
    SavedBackboneGraph* graph,
    const std::vector<std::optional<LegacySavedBackboneRowKey>>&
        legacy_row_keys) {
  if (graph == nullptr) {
    return;
  }
  std::vector<std::size_t> pair_bindings{};
  pair_bindings.reserve(graph->port_bindings.size());
  for (std::size_t index = 0; index < graph->port_bindings.size() &&
                              index < legacy_row_keys.size();
       ++index) {
    const SavedBackbonePortBinding& binding = graph->port_bindings[index];
    const std::optional<LegacySavedBackboneRowKey>& legacy =
        legacy_row_keys[index];
    if (legacy.has_value() && !legacy->source_is_open &&
        legacy->node_id != kInvalidObjectId &&
        legacy->source_edge_a != kInvalidObjectId &&
        legacy->source_edge_b != kInvalidObjectId &&
        migrated_edge_bundle_bundle_id(*graph, binding.edge_bundle_id) != kInvalidObjectId) {
      pair_bindings.push_back(index);
    }
  }
  std::sort(pair_bindings.begin(), pair_bindings.end(),
            [&](std::size_t a_index, std::size_t b_index) {
              const SavedBackbonePortBinding& a = graph->port_bindings[a_index];
              const SavedBackbonePortBinding& b = graph->port_bindings[b_index];
              const LegacySavedBackboneRowKey& a_key = *legacy_row_keys[a_index];
              const LegacySavedBackboneRowKey& b_key = *legacy_row_keys[b_index];
              return std::make_tuple(a_key.node_id, a_key.source_edge_a,
                                     a_key.source_edge_b, a.lane_index,
                                     migrated_edge_bundle_bundle_id(*graph, a.edge_bundle_id),
                                     a.edge_bundle_id) <
                     std::make_tuple(b_key.node_id, b_key.source_edge_a,
                                     b_key.source_edge_b, b.lane_index,
                                     migrated_edge_bundle_bundle_id(*graph, b.edge_bundle_id),
                                     b.edge_bundle_id);
            });
  for (std::size_t first = 0; first < pair_bindings.size();) {
    std::size_t last = first + 1;
    const SavedBackbonePortBinding& first_binding =
        graph->port_bindings[pair_bindings[first]];
    const LegacySavedBackboneRowKey& first_key =
        *legacy_row_keys[pair_bindings[first]];
    while (last < pair_bindings.size() &&
           legacy_row_keys[pair_bindings[last]]->node_id == first_key.node_id &&
           legacy_row_keys[pair_bindings[last]]->source_edge_a ==
               first_key.source_edge_a &&
           legacy_row_keys[pair_bindings[last]]->source_edge_b ==
               first_key.source_edge_b &&
           graph->port_bindings[pair_bindings[last]].lane_index ==
               first_binding.lane_index &&
           migrated_edge_bundle_bundle_id(
               *graph,
               graph->port_bindings[pair_bindings[last]].edge_bundle_id) ==
               migrated_edge_bundle_bundle_id(*graph,
                                               first_binding.edge_bundle_id)) {
      ++last;
    }
    if (last - first == 2 &&
        graph->port_bindings[pair_bindings[first]].edge_bundle_id !=
            graph->port_bindings[pair_bindings[first + 1]].edge_bundle_id) {
      const SavedBackbonePortBinding& a =
          graph->port_bindings[pair_bindings[first]];
      const SavedBackbonePortBinding& b =
          graph->port_bindings[pair_bindings[first + 1]];
      append_migrated_row_continuity(graph, first_key.node_id,
                                     a.edge_bundle_id, a.lane_index,
                                     b.edge_bundle_id, b.lane_index);
    }
    first = last;
  }
}

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

template <typename FieldArchive>
void write_backbone_as(StateWriter& writer, const SavedBackboneGraph& graph) {
  write_id_vector(writer, "authoritative.backbone.nodes", graph.nodes,
                  [](const SavedBackboneNode& value) { return value.node_id; },
                  [](auto& out, const auto& prefix, const SavedBackboneNode& value) {
                    FieldArchive a(out); (void)archive_saved_node(a, prefix, value);
                  });
  write_id_vector(writer, "authoritative.backbone.edges", graph.edges,
                  [](const SavedBackboneEdge& value) { return value.edge_id; },
                  [](auto& out, const auto& prefix, const SavedBackboneEdge& value) {
                    FieldArchive a(out); (void)archive_saved_edge(a, prefix, value);
                  });
  write_id_vector(writer, "authoritative.backbone.edge_bundles", graph.edge_bundles,
                  [](const SavedBackboneEdgeBundle& value) { return value.edge_bundle_id; },
                  [](auto& out, const auto& prefix, const SavedBackboneEdgeBundle& value) {
                    FieldArchive a(out); (void)archive_saved_edge_bundle(a, prefix, value);
                  });
  write_ordered_vector(writer, "authoritative.backbone.port_bindings", graph.port_bindings,
                       [](const SavedBackbonePortBinding& a, const SavedBackbonePortBinding& b) {
                         return std::tie(a.edge_bundle_id, a.row_key.node_id,
                                         a.row_key.edge_id, a.lane_index,
                                         a.port_id) <
                                std::tie(b.edge_bundle_id, b.row_key.node_id,
                                         b.row_key.edge_id, b.lane_index,
                                         b.port_id);
                       }, [](auto& out, const auto& prefix, const SavedBackbonePortBinding& value) {
                         FieldArchive a(out); (void)archive_saved_port_binding(a, prefix, value);
                       });
  write_ordered_vector(writer, "authoritative.backbone.span_bindings", graph.span_bindings,
                       [](const SavedBackboneSpanBinding& a, const SavedBackboneSpanBinding& b) {
                         return std::tie(a.edge_bundle_id, a.lane_index, a.span_id) <
                                std::tie(b.edge_bundle_id, b.lane_index, b.span_id);
                       }, [](auto& out, const auto& prefix, const SavedBackboneSpanBinding& value) {
                         FieldArchive a(out); (void)archive_saved_span_binding(a, prefix, value);
                       });
  write_ordered_vector(writer, "authoritative.backbone.row_continuities", graph.row_continuities,
                       [](const SavedBackboneRowContinuity& a, const SavedBackboneRowContinuity& b) {
                         return std::tie(a.node_id, a.a.edge_bundle_id, a.a.lane_index,
                                         a.b.edge_bundle_id, a.b.lane_index) <
                                std::tie(b.node_id, b.a.edge_bundle_id, b.a.lane_index,
                                         b.b.edge_bundle_id, b.b.lane_index);
                       }, [](auto& out, const auto& prefix, const SavedBackboneRowContinuity& value) {
                         FieldArchive a(out); (void)archive_saved_row_continuity(a, prefix, value);
                       });
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

#ifdef _MSC_VER
static_assert(sizeof(PortPlacementBand) == 184, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_anchor_slot(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "slot_id", value.slot_id) && archive.field(prefix, "usage", value.usage) &&
         archive_vec3(archive, child(prefix, "local_position"), value.local_position) &&
         archive.field(prefix, "priority", value.priority) && archive.field(prefix, "enabled", value.enabled);
}

#ifdef _MSC_VER
static_assert(sizeof(AnchorSlotTemplate) == 40, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_pole_type(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "id", value.id) || !archive.string_value(child(prefix, "name"), value.name) || !archive.string_value(child(prefix, "description"), value.description) ||
      !archive.field(prefix, "default_height_m", value.default_height_m) ||
      !archive.compatible_field(prefix, "radius_base_m", value.radius_base_m, 0.0) ||
      !archive.compatible_field(prefix, "radius_top_m", value.radius_top_m, 0.0) ||
      !archive.field(prefix, "pole_visual_assembly_id", value.pole_visual_assembly_id)) return false;
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

#ifdef _MSC_VER
static_assert(sizeof(PoleTypeDefinition) == 184, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_supplemental_path(Archive& ar, const std::string& p, Value& v) {
  return ar.field(p, "anchor_mode", v.anchor_mode) && ar.field(p, "profile_kind", v.profile_kind) &&
         ar.field(p, "interaction_mode", v.interaction_mode) && ar.field(p, "pole_band_id", v.pole_band_id) &&
         ar.field(p, "endpoint_trim_m", v.endpoint_trim_m) && ar.field(p, "lateral_offset_m", v.lateral_offset_m) &&
         ar.field(p, "vertical_offset_m", v.vertical_offset_m) && ar.field(p, "wobble_amplitude_m", v.wobble_amplitude_m) &&
         ar.field(p, "wobble_wavelength_m", v.wobble_wavelength_m) && ar.field(p, "wobble_phase_bias", v.wobble_phase_bias) &&
         ar.field(p, "endpoint_envelope_ratio", v.endpoint_envelope_ratio);
}

#ifdef _MSC_VER
static_assert(sizeof(CableSupplementalPathTemplate) == 64, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_cable_template(Archive& ar, const std::string& p, Value& v) {
  if (!ar.field(p, "id", v.id) || !ar.string_value(child(p, "name"), v.name) ||
      !ar.field(p, "outer_diameter_m", v.outer_diameter_m) ||
      !ar.field(p, "default_grouped_support_fanout_spacing_m", v.default_grouped_support_fanout_spacing_m) ||
      !ar.field(p, "bend_stiffness", v.bend_stiffness) || !ar.field(p, "min_bend_radius_m", v.min_bend_radius_m) ||
      !ar.field(p, "material_style", v.material_style) || !ar.field(p, "color_rgba", v.color_rgba) ||
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

#ifdef _MSC_VER
static_assert(sizeof(CableTemplate) == 152, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_reserve(Archive& ar, const std::string& p, Value& v) {
  return ar.field(p, "reserve_id", v.reserve_id) && ar.field(p, "pole_type_id", v.pole_type_id) &&
         ar.field(p, "band_id", v.band_id) && ar.field(p, "lateral_min_m", v.lateral_min_m) &&
         ar.field(p, "lateral_max_m", v.lateral_max_m) && ar.field(p, "height_min_m", v.height_min_m) &&
         ar.field(p, "height_max_m", v.height_max_m);
}

#ifdef _MSC_VER
static_assert(sizeof(PlacementReserve) == 48, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_population_rule(Archive& ar, const std::string& p, Value& v) {
  if (!ar.field(p, "rule_id", v.rule_id) || !ar.field(p, "explicit_seed", v.explicit_seed) ||
      !ar.field(p, "priority", v.priority) || !ar.field(p, "min_extra_count", v.min_extra_count) ||
      !ar.field(p, "max_extra_count", v.max_extra_count) || !ar.field(p, "min_spacing_m", v.min_spacing_m) ||
      !ar.field(p, "lateral_min_m", v.lateral_min_m) || !ar.field(p, "lateral_max_m", v.lateral_max_m) ||
      !ar.field(p, "height_min_m", v.height_min_m) || !ar.field(p, "height_max_m", v.height_max_m) ||
      !ar.field(p, "randomness", v.randomness)) return false;
  std::size_t count = v.reserves.size();
  if (!ar.count(child(p, "reserves.count"), count)) return false;
  if constexpr (Archive::loading) v.reserves.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive_reserve(ar, indexed(child(p, "reserves"), i), v.reserves[i])) return false;
  }
  return true;
}

#ifdef _MSC_VER
static_assert(sizeof(CablePopulationRule) == 112, "field added: update archive visitor and full-fat persistence fixture");
#endif

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
      !ar.field(p, "support_wire_pole_band_id", v.support_wire_pole_band_id) ||
      !ar.field(p, "row_fixture_assembly_id", v.row_fixture_assembly_id) ||
      !ar.field(p, "endpoint_fixture_assembly_id", v.endpoint_fixture_assembly_id) ||
      !ar.field(p, "span_visual_assembly.helix_enabled", v.span_visual_assembly.helix_enabled) ||
      !ar.compatible_field(p, "span_visual_assembly.support_path_enabled",
                           v.span_visual_assembly.support_path_enabled,
                           v.span_visual_assembly.helix_enabled) ||
      !ar.field(p, "span_visual_assembly.helix_radius_m", v.span_visual_assembly.helix_radius_m) ||
      !ar.field(p, "span_visual_assembly.helix_clearance_m", v.span_visual_assembly.helix_clearance_m) ||
      !ar.field(p, "span_visual_assembly.helix_turns_per_meter", v.span_visual_assembly.helix_turns_per_meter) ||
      !ar.field(p, "span_visual_assembly.helix_samples_per_turn", v.span_visual_assembly.helix_samples_per_turn) ||
      !ar.field(p, "span_visual_assembly.endpoint_trim_m", v.span_visual_assembly.endpoint_trim_m) ||
      !ar.field(p, "span_visual_assembly.member_wander_ratio", v.span_visual_assembly.member_wander_ratio) ||
      !ar.field(p, "span_visual_assembly.member_wander_wavelength_m", v.span_visual_assembly.member_wander_wavelength_m) ||
      !ar.field(p, "span_visual_assembly.member_wander_phase_bias", v.span_visual_assembly.member_wander_phase_bias) ||
      !ar.field(p, "span_visual_assembly.member_twist_turns_per_meter", v.span_visual_assembly.member_twist_turns_per_meter) ||
      !ar.field(p, "span_visual_assembly.member_twist_phase", v.span_visual_assembly.member_twist_phase)) return false;
  std::size_t count = v.population_rules.size();
  if (!ar.count(child(p, "population_rules.count"), count)) return false;
  if constexpr (Archive::loading) v.population_rules.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!archive_population_rule(ar, indexed(child(p, "population_rules"), i), v.population_rules[i])) return false;
  }
  return ar.field(p, "version", v.version);
}


#ifdef _MSC_VER
static_assert(sizeof(BundleTemplate) == 272, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_attachment_socket(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "id", value.id) && archive_vec3(archive, child(prefix, "local_position"), value.local_position) &&
         archive_vec3(archive, child(prefix, "tangent_dir"), value.tangent_dir) && archive.field(prefix, "has_normal", value.has_normal) &&
         archive_vec3(archive, child(prefix, "normal_dir"), value.normal_dir) && archive.field(prefix, "has_binormal", value.has_binormal) &&
         archive_vec3(archive, child(prefix, "binormal_dir"), value.binormal_dir) && archive.field(prefix, "kind", value.kind);
}

#ifdef _MSC_VER
static_assert(sizeof(AttachmentSocketTemplate) == 128, "field added: update archive visitor and full-fat persistence fixture");
#endif

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

#ifdef _MSC_VER
static_assert(sizeof(AttachmentInternalPathTemplate) == 64, "field added: update archive visitor and full-fat persistence fixture");
#endif

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

#ifdef _MSC_VER
static_assert(sizeof(AttachmentTemplate) == 128, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_model_assembly_socket(Archive& archive, const std::string& prefix, Value& value) {
  return archive.string_value(child(prefix, "name"), value.name) &&
         archive_vec3(archive, child(prefix, "local_position"), value.local_position) &&
         archive_vec3(archive, child(prefix, "local_direction"), value.local_direction);
}

template <typename Archive, typename Value>
bool archive_model_assembly_part(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "part_id", value.part_id) ||
      !archive.string_value(child(prefix, "model_key"), value.model_key) ||
      !archive.field(prefix, "descriptor_version", value.descriptor_version) ||
      !archive_transform(archive, child(prefix, "local_transform"), value.local_transform) ||
      !archive.field(prefix, "fit_mode", value.fit_mode)) return false;
  std::size_t socket_count = value.sockets.size();
  if (!archive.count(child(prefix, "sockets.count"), socket_count)) return false;
  if constexpr (Archive::loading) value.sockets.resize(socket_count);
  for (std::size_t i = 0; i < socket_count; ++i) {
    if (!archive_model_assembly_socket(archive, indexed(child(prefix, "sockets"), i), value.sockets[i])) return false;
  }
  return true;
}

template <typename Archive, typename Value>
bool archive_model_assembly_template(Archive& archive, const std::string& prefix, Value& value) {
  if (!archive.field(prefix, "id", value.id)) return false;
  std::size_t part_count = value.parts.size();
  if (!archive.count(child(prefix, "parts.count"), part_count)) return false;
  if constexpr (Archive::loading) value.parts.resize(part_count);
  for (std::size_t i = 0; i < part_count; ++i) {
    if (!archive_model_assembly_part(archive, indexed(child(prefix, "parts"), i), value.parts[i])) return false;
  }
  bool has_wire_socket = value.wire_socket.has_value();
  if (!archive.field(prefix, "wire_socket.has", has_wire_socket)) return false;
  if constexpr (Archive::loading) {
    if (has_wire_socket) value.wire_socket.emplace();
    else value.wire_socket.reset();
  }
  if (has_wire_socket) {
    if (!archive.field(prefix, "wire_socket.part_id", value.wire_socket->part_id) ||
        !archive.string_value(child(prefix, "wire_socket.socket_name"), value.wire_socket->socket_name)) return false;
  }
  bool has_endpoint_mount_socket = value.endpoint_mount_socket.has_value();
  if (!archive.compatible_field(prefix, "endpoint_mount_socket.has",
                                has_endpoint_mount_socket, false)) return false;
  if constexpr (Archive::loading) {
    if (has_endpoint_mount_socket) value.endpoint_mount_socket.emplace();
    else value.endpoint_mount_socket.reset();
  }
  if (has_endpoint_mount_socket) {
    if (!archive.field(prefix, "endpoint_mount_socket.part_id",
                       value.endpoint_mount_socket->part_id) ||
        !archive.string_value(child(prefix, "endpoint_mount_socket.socket_name"),
                              value.endpoint_mount_socket->socket_name)) return false;
  }
  return archive.field(prefix, "version", value.version);
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

#ifdef _MSC_VER
static_assert(sizeof(PoleOrientationOverride) == 48, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_span_endpoint_override(Archive& archive, const std::string& prefix, Value& value) {
  return archive.optional(child(prefix, "socket_a_id"), value.socket_a_id) &&
         archive.optional(child(prefix, "socket_b_id"), value.socket_b_id) &&
         archive.value(child(prefix, "version"), value.version);
}

#ifdef _MSC_VER
static_assert(sizeof(SpanEndpointOverride) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_span_support_override(Archive& archive, const std::string& prefix, Value& value) {
  return archive.optional(child(prefix, "branch_down_offset_m"), value.branch_down_offset_m) &&
         archive.value(child(prefix, "version"), value.version);
}


#ifdef _MSC_VER
static_assert(sizeof(SpanSupportOverride) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_context_profile(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "age", value.age) && archive.field(prefix, "clutter", value.clutter) && archive.field(prefix, "regularity", value.regularity) &&
         archive.field(prefix, "service_mix", value.service_mix) && archive.field(prefix, "style_seed", value.style_seed);
}

#ifdef _MSC_VER
static_assert(sizeof(ContextProfile) == 40, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_layout_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "angle_correction_enabled", value.angle_correction_enabled) && archive.field(prefix, "corner_threshold_deg", value.corner_threshold_deg) &&
         archive.field(prefix, "min_side_scale", value.min_side_scale) && archive.field(prefix, "max_side_scale", value.max_side_scale);
}

#ifdef _MSC_VER
static_assert(sizeof(LayoutSettings) == 32, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_geometry_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "curve_samples", value.curve_samples) && archive.field(prefix, "sag_enabled", value.sag_enabled) && archive.field(prefix, "sag_factor", value.sag_factor) &&
         archive.field(prefix, "pole_clearance_m", value.pole_clearance_m);
}

#ifdef _MSC_VER
static_assert(sizeof(GeometrySettings) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_visual_settings(Archive& archive, const std::string& prefix, Value& value) {
  if constexpr (Archive::loading) {
    bool legacy_enable_support_structures = false;
    double legacy_support_center_threshold_m = 0.0;
    double legacy_support_arm_extra_m = 0.0;
    double legacy_support_arm_radius_m = 0.0;
    if (archive.compatible_field(prefix, "enable_support_structures",
                                 legacy_enable_support_structures, false) &&
        archive.compatible_field(prefix, "support_center_threshold_m",
                                 legacy_support_center_threshold_m, 0.0) &&
        archive.compatible_field(prefix, "support_arm_extra_m", legacy_support_arm_extra_m, 0.0) &&
        archive.compatible_field(prefix, "support_arm_radius_m", legacy_support_arm_radius_m, 0.0)) {
      static_cast<void>(legacy_enable_support_structures);
      static_cast<void>(legacy_support_center_threshold_m);
      static_cast<void>(legacy_support_arm_extra_m);
      static_cast<void>(legacy_support_arm_radius_m);
    } else {
      return false;
    }
  }
  return archive.field(prefix, "enable_insulators", value.enable_insulators) &&
         archive.field(prefix, "insulator_radius_m", value.insulator_radius_m) &&
         archive.field(prefix, "insulator_length_m", value.insulator_length_m);
}

#ifdef _MSC_VER
static_assert(sizeof(VisualSettings) == 24, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename Archive, typename Value>
bool archive_variation_settings(Archive& archive, const std::string& prefix, Value& value) {
  return archive.field(prefix, "enabled", value.enabled) && archive.field(prefix, "global_seed", value.global_seed) && archive.field(prefix, "world_cell_size_m", value.world_cell_size_m) &&
         archive.field(prefix, "world_bias_scale", value.world_bias_scale) && archive.field(prefix, "flow_bias_scale", value.flow_bias_scale) &&
         archive.field(prefix, "pole_delta_scale", value.pole_delta_scale) && archive.field(prefix, "local_jitter_scale", value.local_jitter_scale) &&
         archive.field(prefix, "sag_variation_scale", value.sag_variation_scale) && archive.field(prefix, "branch_down_offset_variation_scale", value.branch_down_offset_variation_scale);
}

// Independent of WriteFieldArchive/ReadFieldArchive so a field omitted by both
// persistence directions is still visible to authoritative_equals().
class Comparer {
public:
  static constexpr bool loading = false;
  explicit Comparer(StateWriter& fields) : fields_(fields) {}

  template <typename T> bool value(const std::string& key, const T& input) {
    fields_.value(key, input);
    return true;
  }
  template <typename T> bool field(const std::string& prefix, std::string_view name, const T& input) {
    return value(child(prefix, name), input);
  }
  template <typename T>
  bool compatible_field(const std::string& prefix, std::string_view name, const T& input, const T&) {
    return field(prefix, name, input);
  }
  template <typename T>
  bool legacy_field(const std::string&, std::string_view, const T&, const T&) {
    return true;
  }
  bool string_value(const std::string& key, const std::string& input) {
    fields_.string_value(key, input);
    return true;
  }
  bool count(const std::string& key, std::size_t& input) {
    fields_.value(key, input);
    return true;
  }
  template <typename T> bool optional(const std::string& prefix, const std::optional<T>& input) {
    fields_.value(child(prefix, "has"), input.has_value());
    if (input.has_value()) fields_.value(child(prefix, "value"), *input);
    return true;
  }

private:
  StateWriter& fields_;
};

#ifdef _MSC_VER
static_assert(sizeof(VariationSettings) == 72, "field added: update archive visitor and full-fat persistence fixture");
#endif

template <typename FieldArchive>
void write_authoritative_as(StateWriter& writer, const CoreStateAuthoritativeStorage& authoritative) {
  write_edit_state_as<FieldArchive>(writer, authoritative.edit_state);
  write_backbone_as<FieldArchive>(writer, authoritative.backbone);
  write_map(writer, "authoritative.pole_types", authoritative.pole_types,
            [](auto& out, const auto& prefix, const PoleTypeDefinition& value) {
              FieldArchive a(out); (void)archive_pole_type(a, prefix, value);
            });
  write_map(writer, "authoritative.cable_templates", authoritative.cable_templates,
            [](auto& out, const auto& prefix, const CableTemplate& value) {
              FieldArchive a(out); (void)archive_cable_template(a, prefix, value);
            });
  write_map(writer, "authoritative.bundle_templates", authoritative.bundle_templates,
            [](auto& out, const auto& prefix, const BundleTemplate& value) {
              FieldArchive a(out); (void)archive_bundle_template(a, prefix, value);
            });
  write_map(writer, "authoritative.attachment_templates", authoritative.attachment_templates,
            [](auto& out, const auto& prefix, const AttachmentTemplate& value) {
              FieldArchive a(out); (void)archive_attachment_template(a, prefix, value);
            });
  write_map(writer, "authoritative.model_assembly_templates", authoritative.model_assembly_templates,
            [](auto& out, const auto& prefix, const ModelAssemblyTemplate& value) {
              FieldArchive a(out); (void)archive_model_assembly_template(a, prefix, value);
            });
  {
    FieldArchive a(writer);
    (void)archive_context_profile(a, "authoritative.context_profile", authoritative.context_profile);
  }
  write_map(writer, "authoritative.override_state.pole_orientation_by_pole",
            authoritative.override_state.pole_orientation_by_pole,
            [](auto& out, const auto& prefix, const PoleOrientationOverride& value) {
              FieldArchive a(out); (void)archive_pole_orientation_override(a, prefix, value);
            });
  write_map(writer, "authoritative.override_state.span_endpoint_by_span",
            authoritative.override_state.span_endpoint_by_span,
            [](auto& out, const auto& prefix, const SpanEndpointOverride& value) {
              FieldArchive a(out); (void)archive_span_endpoint_override(a, prefix, value);
            });
  write_map(writer, "authoritative.override_state.span_support_by_span",
            authoritative.override_state.span_support_by_span,
            [](auto& out, const auto& prefix, const SpanSupportOverride& value) {
              FieldArchive a(out); (void)archive_span_support_override(a, prefix, value);
            });
  {
    FieldArchive a(writer);
    (void)archive_layout_settings(a, "authoritative.layout_settings", authoritative.layout_settings);
    (void)archive_geometry_settings(a, "authoritative.geometry_settings", authoritative.geometry_settings);
    (void)archive_visual_settings(a, "authoritative.visual_settings", authoritative.visual_settings);
    (void)archive_variation_settings(a, "authoritative.variation_settings", authoritative.variation_settings);
  }
}

std::string authoritative_signature(const CoreStateAuthoritativeStorage& authoritative) {
  StateWriter fields{};
  write_authoritative_as<Comparer>(fields, authoritative);
  return std::move(fields).finish();
}


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
  return read_object_store(reader, "authoritative.edit_state.poles", &state->poles,
                           [](auto& in, const auto& prefix, Pole* value) { ReadFieldArchive a(in); return archive_pole(a, prefix, *value); }) &&
         read_object_store(reader, "authoritative.edit_state.ports", &state->ports,
                           [](auto& in, const auto& prefix, Port* value) { ReadFieldArchive a(in); return archive_port(a, prefix, *value); }) &&
         read_object_store(reader, "authoritative.edit_state.anchors", &state->anchors,
                           [](auto& in, const auto& prefix, Anchor* value) { ReadFieldArchive a(in); return archive_anchor(a, prefix, *value); }) &&
         read_object_store(reader, "authoritative.edit_state.bundles", &state->bundles,
                           [](auto& in, const auto& prefix, Bundle* value) { ReadFieldArchive a(in); return archive_bundle(a, prefix, *value); }) &&
         read_object_store(reader, "authoritative.edit_state.spans", &state->spans,
                           [](auto& in, const auto& prefix, Span* value) { ReadFieldArchive a(in); return archive_span(a, prefix, *value); }) &&
         read_object_store(reader, "authoritative.edit_state.attachments", &state->attachments,
                           [](auto& in, const auto& prefix, Attachment* value) { ReadFieldArchive a(in); return archive_attachment(a, prefix, *value); });
}

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
                      [](const SavedBackboneNode& value) { return value.node_id; },
                      [](auto& in, const auto& prefix, SavedBackboneNode* value) {
                        ReadFieldArchive a(in); return archive_saved_node(a, prefix, *value);
                      }) ||
      !read_id_vector(reader, "authoritative.backbone.edges", &graph->edges,
                      [](const SavedBackboneEdge& value) { return value.edge_id; },
                      [](auto& in, const auto& prefix, SavedBackboneEdge* value) {
                        ReadFieldArchive a(in); return archive_saved_edge(a, prefix, *value);
                      }) ||
      !read_id_vector(reader, "authoritative.backbone.edge_bundles", &graph->edge_bundles,
                      [](const SavedBackboneEdgeBundle& value) { return value.edge_bundle_id; },
                      [](auto& in, const auto& prefix, SavedBackboneEdgeBundle* value) {
                        ReadFieldArchive a(in); return archive_saved_edge_bundle(a, prefix, *value);
                      })) return false;
  std::size_t port_count = 0;
  if (!reader.count("authoritative.backbone.port_bindings.count", &port_count)) return false;
  graph->port_bindings.resize(port_count);
  std::vector<std::optional<LegacySavedBackboneRowKey>> legacy_row_keys(
      port_count);
  for (std::size_t i = 0; i < port_count; ++i) {
    const std::string prefix =
        indexed("authoritative.backbone.port_bindings", i);
    if (reader.contains(child(child(prefix, "row_key"), "edge_id"))) {
      ReadFieldArchive archive(reader);
      if (!archive_saved_port_binding(archive, prefix,
                                      graph->port_bindings[i])) {
        return false;
      }
    } else {
      LegacySavedBackboneRowKey legacy{};
      if (!read_legacy_saved_port_binding(
              reader, prefix, &graph->port_bindings[i], &legacy)) {
        return false;
      }
      legacy_row_keys[i] = legacy;
    }
  }
  for (std::size_t i = 0; i < legacy_row_keys.size(); ++i) {
    if (!legacy_row_keys[i].has_value()) {
      continue;
    }
    SavedBackbonePortBinding& binding = graph->port_bindings[i];
    const auto edge_bundle = std::find_if(
        graph->edge_bundles.begin(), graph->edge_bundles.end(),
        [&](const SavedBackboneEdgeBundle& value) {
          return value.edge_bundle_id == binding.edge_bundle_id;
        });
    const LegacySavedBackboneRowKey& legacy = *legacy_row_keys[i];
    if (edge_bundle == graph->edge_bundles.end() ||
        legacy.node_id == kInvalidObjectId ||
        edge_bundle->edge_id == kInvalidObjectId) {
      return false;
    }
    const bool valid_open =
        legacy.source_is_open &&
        legacy.source_edge_a == edge_bundle->edge_id &&
        legacy.source_edge_b == kInvalidObjectId;
    const bool valid_pair =
        !legacy.source_is_open &&
        legacy.source_edge_a != kInvalidObjectId &&
        legacy.source_edge_b != kInvalidObjectId &&
        (legacy.source_edge_a == edge_bundle->edge_id ||
         legacy.source_edge_b == edge_bundle->edge_id);
    if (!valid_open && !valid_pair) {
      return false;
    }
    binding.row_key = {legacy.node_id, edge_bundle->edge_id};
  }
  std::size_t span_count = 0;
  if (!reader.count("authoritative.backbone.span_bindings.count", &span_count)) return false;
  graph->span_bindings.resize(span_count);
  for (std::size_t i = 0; i < span_count; ++i) {
    ReadFieldArchive archive(reader);
    if (!archive_saved_span_binding(archive, indexed("authoritative.backbone.span_bindings", i),
                                    graph->span_bindings[i])) return false;
  }
  std::size_t continuity_count = 0;
  if (reader.version() >= 2) {
    if (!reader.count("authoritative.backbone.row_continuities.count", &continuity_count)) return false;
    graph->row_continuities.resize(continuity_count);
    for (std::size_t i = 0; i < continuity_count; ++i) {
      ReadFieldArchive archive(reader);
      if (!archive_saved_row_continuity(archive, indexed("authoritative.backbone.row_continuities", i),
                                        graph->row_continuities[i])) return false;
    }
  } else {
    migrate_v1_row_continuities(graph, legacy_row_keys);
  }
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

bool normalize_saved_support_levels(CoreStateAuthoritativeStorage* authoritative) {
  if (authoritative == nullptr) {
    return false;
  }
  SavedBackboneGraph& graph = authoritative->backbone;
  struct record {
    std::size_t binding_index = 0;
    ObjectId node_id = kInvalidObjectId;
    BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
    std::uint64_t placement_key = 0;
    ObjectId row_edge_bundle_a = kInvalidObjectId;
    ObjectId row_edge_bundle_b = kInvalidObjectId;
  };
  std::unordered_map<ObjectId, ObjectId> bundle_by_edge_bundle{};
  bundle_by_edge_bundle.reserve(graph.edge_bundles.size());
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    bundle_by_edge_bundle.emplace(edge_bundle.edge_bundle_id, edge_bundle.bundle_id);
  }
  std::unordered_map<ObjectId, std::vector<const SavedBackbonePortBinding*>>
      bindings_by_port{};
  for (const SavedBackbonePortBinding& binding : graph.port_bindings) {
    bindings_by_port[binding.port_id].push_back(&binding);
  }
  std::vector<record> records{};
  records.reserve(graph.port_bindings.size());
  for (std::size_t index = 0; index < graph.port_bindings.size(); ++index) {
    const SavedBackbonePortBinding& binding = graph.port_bindings[index];
    const auto bundle_id_it = bundle_by_edge_bundle.find(binding.edge_bundle_id);
    if (bundle_id_it == bundle_by_edge_bundle.end()) {
      return false;
    }
    const Bundle* bundle = authoritative->edit_state.bundles.find(bundle_id_it->second);
    if (bundle == nullptr) {
      return false;
    }
    ObjectId row_a = binding.edge_bundle_id;
    ObjectId row_b = kInvalidObjectId;
    for (const SavedBackboneRowContinuity& continuity :
         graph.row_continuities) {
      if (continuity.node_id != binding.row_key.node_id) {
        continue;
      }
      const bool is_a =
          continuity.a.edge_bundle_id == binding.edge_bundle_id &&
          continuity.a.lane_index == binding.lane_index;
      const bool is_b =
          continuity.b.edge_bundle_id == binding.edge_bundle_id &&
          continuity.b.lane_index == binding.lane_index;
      if (!is_a && !is_b) {
        continue;
      }
      if (row_b != kInvalidObjectId) {
        return false;
      }
      const ObjectId peer =
          is_a ? continuity.b.edge_bundle_id : continuity.a.edge_bundle_id;
      row_a = std::min(binding.edge_bundle_id, peer);
      row_b = std::max(binding.edge_bundle_id, peer);
    }
    if (row_b == kInvalidObjectId) {
      const auto shared = bindings_by_port.find(binding.port_id);
      if (shared != bindings_by_port.end() && shared->second.size() == 2) {
        const SavedBackbonePortBinding* other =
            shared->second.front() == &binding ? shared->second.back()
                                               : shared->second.front();
        if (other != nullptr &&
            other->row_key.node_id == binding.row_key.node_id &&
            other->lane_index == binding.lane_index) {
          row_a = std::min(binding.edge_bundle_id, other->edge_bundle_id);
          row_b = std::max(binding.edge_bundle_id, other->edge_bundle_id);
        }
      }
    }
    records.push_back({index, binding.row_key.node_id,
                       binding.bundle_template_id, bundle->placement_key,
                       row_a, row_b});
  }
  const auto row_tuple = [](const record& value) {
    return std::make_tuple(
        value.node_id, value.bundle_template_id, value.placement_key,
        value.row_edge_bundle_a, value.row_edge_bundle_b);
  };
  std::sort(records.begin(), records.end(),
            [&](const record& lhs, const record& rhs) {
              return row_tuple(lhs) < row_tuple(rhs);
            });
  auto same_scope = [](const record& lhs, const record& rhs) {
    return lhs.node_id == rhs.node_id &&
           lhs.bundle_template_id == rhs.bundle_template_id &&
           lhs.placement_key == rhs.placement_key;
  };
  auto same_row = [&](const record& lhs, const record& rhs) {
    return same_scope(lhs, rhs) &&
           lhs.row_edge_bundle_a == rhs.row_edge_bundle_a &&
           lhs.row_edge_bundle_b == rhs.row_edge_bundle_b;
  };
  for (std::size_t scope_begin = 0; scope_begin < records.size();) {
    std::size_t scope_end = scope_begin + 1;
    while (scope_end < records.size() &&
           same_scope(records[scope_begin], records[scope_end])) {
      ++scope_end;
    }
    const auto template_it = authoritative->bundle_templates.find(
        records[scope_begin].bundle_template_id);
    if (template_it == authoritative->bundle_templates.end()) {
      return false;
    }
    if (!template_it->second.enable_branch_down_offset) {
      for (std::size_t i = scope_begin; i < scope_end; ++i) {
        SavedBackbonePortBinding& binding =
            graph.port_bindings[records[i].binding_index];
        if (binding.support_level < 0) {
          binding.support_level = 0;
        } else if (binding.support_level != 0) {
          return false;
        }
      }
      scope_begin = scope_end;
      continue;
    }
    bool has_missing = false;
    bool has_saved = false;
    for (std::size_t i = scope_begin; i < scope_end; ++i) {
      const int level =
          graph.port_bindings[records[i].binding_index].support_level;
      has_missing = has_missing || level < 0;
      has_saved = has_saved || level >= 0;
    }
    if (has_missing && has_saved) {
      return false;
    }
    std::vector<int> row_levels{};
    for (std::size_t row_begin = scope_begin; row_begin < scope_end;) {
      std::size_t row_end = row_begin + 1;
      while (row_end < scope_end &&
             same_row(records[row_begin], records[row_end])) {
        ++row_end;
      }
      const int level =
          has_missing
              ? static_cast<int>(row_levels.size())
              : graph.port_bindings[records[row_begin].binding_index].support_level;
      if (level < 0) {
        return false;
      }
      for (std::size_t i = row_begin; i < row_end; ++i) {
        SavedBackbonePortBinding& binding =
            graph.port_bindings[records[i].binding_index];
        if (!has_missing && binding.support_level != level) {
          return false;
        }
        binding.support_level = level;
      }
      if (std::find(row_levels.begin(), row_levels.end(), level) !=
          row_levels.end()) {
        return false;
      }
      row_levels.push_back(level);
      row_begin = row_end;
    }
    std::sort(row_levels.begin(), row_levels.end());
    for (std::size_t level = 0; level < row_levels.size(); ++level) {
      if (row_levels[level] != static_cast<int>(level)) {
        return false;
      }
    }
    scope_begin = scope_end;
  }

  struct missing_group_row {
    std::size_t begin = 0;
    std::size_t end = 0;
    ObjectId node_id = kInvalidObjectId;
    int support_level = 0;
  };
  std::vector<missing_group_row> missing_group_rows{};
  std::unordered_map<ObjectId, std::unordered_set<int>> used_group_ids_by_node{};
  std::unordered_map<ObjectId, int> next_group_id_by_node{};
  for (std::size_t row_begin = 0; row_begin < records.size();) {
    std::size_t row_end = row_begin + 1;
    while (row_end < records.size() &&
           same_row(records[row_begin], records[row_end])) {
      ++row_end;
    }
    const int support_level =
        graph.port_bindings[records[row_begin].binding_index].support_level;
    const int first_group_id =
        graph.port_bindings[records[row_begin].binding_index].support_group_id;
    const bool group_missing = first_group_id == -2;
    for (std::size_t i = row_begin; i < row_end; ++i) {
      const int group_id =
          graph.port_bindings[records[i].binding_index].support_group_id;
      if ((group_id == -2) != group_missing ||
          (!group_missing && group_id != first_group_id)) {
        return false;
      }
    }
    if (group_missing) {
      missing_group_rows.push_back(
          {row_begin, row_end, records[row_begin].node_id, support_level});
    } else if ((support_level == 0 && first_group_id != -1) ||
               (support_level > 0 && first_group_id < 0)) {
      return false;
    } else if (support_level > 0) {
      auto& used = used_group_ids_by_node[records[row_begin].node_id];
      if (!used.insert(first_group_id).second) {
        return false;
      }
      next_group_id_by_node[records[row_begin].node_id] =
          std::max(next_group_id_by_node[records[row_begin].node_id],
                   first_group_id + 1);
    }
    row_begin = row_end;
  }
  for (const missing_group_row& row : missing_group_rows) {
    const int group_id =
        row.support_level == 0 ? -1 : next_group_id_by_node[row.node_id]++;
    if (group_id >= 0) {
      used_group_ids_by_node[row.node_id].insert(group_id);
    }
    for (std::size_t i = row.begin; i < row.end; ++i) {
      graph.port_bindings[records[i].binding_index].support_group_id =
          group_id;
    }
  }
  return true;
}

bool read_authoritative(StateReader& reader, CoreStateAuthoritativeStorage* authoritative) {
  if (!(read_edit_state(reader, &authoritative->edit_state) &&
         read_backbone(reader, &authoritative->backbone) &&
         read_map(reader, "authoritative.pole_types", &authoritative->pole_types,
                  [](auto& in, const auto& prefix, PoleTypeDefinition* value) {
                    ReadFieldArchive a(in); return archive_pole_type(a, prefix, *value);
                  }) &&
         read_map(reader, "authoritative.cable_templates", &authoritative->cable_templates,
                  [](auto& in, const auto& prefix, CableTemplate* value) {
                    ReadFieldArchive a(in); return archive_cable_template(a, prefix, *value);
                  }) &&
         read_map(reader, "authoritative.bundle_templates", &authoritative->bundle_templates,
                  [](auto& in, const auto& prefix, BundleTemplate* value) {
                    ReadFieldArchive a(in); return archive_bundle_template(a, prefix, *value);
                  }) &&
         read_map(reader, "authoritative.attachment_templates", &authoritative->attachment_templates,
                  [](auto& in, const auto& prefix, AttachmentTemplate* value) {
                    ReadFieldArchive a(in); return archive_attachment_template(a, prefix, *value);
                  }) &&
         read_map(reader, "authoritative.model_assembly_templates", &authoritative->model_assembly_templates,
                  [](auto& in, const auto& prefix, ModelAssemblyTemplate* value) {
                    ReadFieldArchive a(in); return archive_model_assembly_template(a, prefix, *value);
                  }))) return false;
  ReadFieldArchive fields(reader);
  if (!archive_context_profile(fields, "authoritative.context_profile", authoritative->context_profile) ||
      !read_object_id_map(reader, "authoritative.override_state.pole_orientation_by_pole",
                            &authoritative->override_state.pole_orientation_by_pole,
                            [](auto& in, const auto& prefix, PoleOrientationOverride* value) {
                              ReadFieldArchive a(in); return archive_pole_orientation_override(a, prefix, *value);
                            }) ||
      !read_object_id_map(reader, "authoritative.override_state.span_endpoint_by_span",
                          &authoritative->override_state.span_endpoint_by_span,
                          [](auto& in, const auto& prefix, SpanEndpointOverride* value) {
                            ReadFieldArchive a(in); return archive_span_endpoint_override(a, prefix, *value);
                          }) ||
      !read_object_id_map(reader, "authoritative.override_state.span_support_by_span",
                          &authoritative->override_state.span_support_by_span,
                          [](auto& in, const auto& prefix, SpanSupportOverride* value) {
                            ReadFieldArchive a(in); return archive_span_support_override(a, prefix, *value);
                          }) ||
      !archive_layout_settings(fields, "authoritative.layout_settings", authoritative->layout_settings) ||
      !archive_geometry_settings(fields, "authoritative.geometry_settings", authoritative->geometry_settings) ||
      !archive_visual_settings(fields, "authoritative.visual_settings", authoritative->visual_settings) ||
      !archive_variation_settings(fields, "authoritative.variation_settings", authoritative->variation_settings)) return false;
  return normalize_saved_support_levels(authoritative);
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

  write_authoritative_as<WriteFieldArchive>(writer, authoritative_);

  *out = std::move(writer).finish();
  result.ok = true;
  result.value = true;
  return result;
}

bool CoreState::authoritative_equals(const CoreState& other) const {
  return authoritative_signature(authoritative_) == authoritative_signature(other.authoritative_);
}

EditResult<bool> CoreState::DeserializeAuthoritative(const std::string& text) {
  EditResult<bool> result{};
  StateReader reader{};
  if (!reader.parse(text)) {
    result.error = reader.error();
    result.classify_error();
    return result;
  }

  CoreStateIdentityStorage loaded_identity{};
  CoreStateAuthoritativeStorage loaded_authoritative{};
  if (!read_identity(reader, &loaded_identity) || !read_authoritative(reader, &loaded_authoritative) ||
      !reader.finish()) {
    result.error = reader.error().empty() ? "authoritative deserialization: invalid field" : reader.error();
    result.classify_error();
    return result;
  }

  CoreState trial{};
  trial.identity_ = std::move(loaded_identity);
  trial.authoritative_ = std::move(loaded_authoritative);
  trial.runtime_ = {};
  trial.session_ = {};
  trial.debug_ = {};

  auto migrate_shared_pair_ports = [&]() -> bool {
    SavedBackboneGraph& graph = trial.authoritative_.backbone;
    std::unordered_map<ObjectId, std::vector<std::size_t>> bindings_by_port{};
    for (std::size_t index = 0; index < graph.port_bindings.size(); ++index) {
      bindings_by_port[graph.port_bindings[index].port_id].push_back(index);
    }
    auto edge_bundle_for = [&](ObjectId edge_bundle_id)
        -> const SavedBackboneEdgeBundle* {
      const auto found = std::find_if(
          graph.edge_bundles.begin(), graph.edge_bundles.end(),
          [&](const SavedBackboneEdgeBundle& value) {
            return value.edge_bundle_id == edge_bundle_id;
          });
      return found == graph.edge_bundles.end() ? nullptr : &*found;
    };
    auto saved_node_for = [&](ObjectId node_id) -> const SavedBackboneNode* {
      const auto found = std::find_if(
          graph.nodes.begin(), graph.nodes.end(),
          [&](const SavedBackboneNode& value) { return value.node_id == node_id; });
      return found == graph.nodes.end() ? nullptr : &*found;
    };
    auto spans_for = [&](ObjectId edge_bundle_id, std::size_t lane_index) {
      std::vector<ObjectId> span_ids{};
      for (const SavedBackboneSpanBinding& binding : graph.span_bindings) {
        if (binding.edge_bundle_id == edge_bundle_id &&
            binding.lane_index == lane_index) {
          span_ids.push_back(binding.span_id);
        }
      }
      return span_ids;
    };
    auto has_continuity = [&](const SavedBackbonePortBinding& a,
                              const SavedBackbonePortBinding& b) {
      return std::any_of(
          graph.row_continuities.begin(), graph.row_continuities.end(),
          [&](const SavedBackboneRowContinuity& continuity) {
            if (continuity.node_id != a.row_key.node_id) {
              return false;
            }
            const auto matches = [](const SavedBackboneRowContinuityEndpoint& endpoint,
                                    const SavedBackbonePortBinding& binding) {
              return endpoint.edge_bundle_id == binding.edge_bundle_id &&
                     endpoint.lane_index == binding.lane_index;
            };
            return (matches(continuity.a, a) && matches(continuity.b, b)) ||
                   (matches(continuity.a, b) && matches(continuity.b, a));
          });
    };

    for (auto& [port_id, binding_indices] : bindings_by_port) {
      if (binding_indices.size() == 1) {
        continue;
      }
      if (binding_indices.size() != 2) {
        result.error =
            "authoritative migration unsupported: shared port has ambiguous endpoint bindings";
        return false;
      }
      std::sort(binding_indices.begin(), binding_indices.end(),
                [&](std::size_t a, std::size_t b) {
                  return graph.port_bindings[a].edge_bundle_id <
                         graph.port_bindings[b].edge_bundle_id;
                });
      SavedBackbonePortBinding& keep =
          graph.port_bindings[binding_indices[0]];
      SavedBackbonePortBinding& split =
          graph.port_bindings[binding_indices[1]];
      const bool same_scope =
          keep.row_key.node_id == split.row_key.node_id &&
          keep.lane_index == split.lane_index &&
          keep.bundle_template_id == split.bundle_template_id &&
          keep.port_kind == split.port_kind &&
          keep.port_layer == split.port_layer &&
          keep.placement_band_id == split.placement_band_id &&
          keep.support_level == split.support_level &&
          keep.support_group_id == split.support_group_id &&
          std::bit_cast<std::uint64_t>(keep.layout_yaw_deg) ==
              std::bit_cast<std::uint64_t>(split.layout_yaw_deg) &&
          keep.edge_bundle_id != split.edge_bundle_id;
      const SavedBackboneEdgeBundle* keep_edge_bundle =
          edge_bundle_for(keep.edge_bundle_id);
      const SavedBackboneEdgeBundle* split_edge_bundle =
          edge_bundle_for(split.edge_bundle_id);
      const SavedBackboneNode* node = saved_node_for(keep.row_key.node_id);
      const Port* source_port =
          trial.authoritative_.edit_state.ports.find(port_id);
      const std::vector<ObjectId> split_spans =
          spans_for(split.edge_bundle_id, split.lane_index);
      if (!same_scope || keep_edge_bundle == nullptr ||
          split_edge_bundle == nullptr || node == nullptr ||
          source_port == nullptr ||
          keep.row_key.edge_id != keep_edge_bundle->edge_id ||
          split.row_key.edge_id != split_edge_bundle->edge_id ||
          !has_continuity(keep, split) || split_spans.size() != 1) {
        result.error =
            "authoritative migration unsupported: shared pair port cannot be split exactly";
        return false;
      }
      Span* split_span =
          trial.authoritative_.edit_state.spans.find(split_spans.front());
      if (split_span == nullptr) {
        result.error =
            "authoritative migration unsupported: shared pair span is missing";
        return false;
      }
      const bool replace_a =
          split_span->port_a_id == port_id &&
          split_span->endpoint_node_a_id == node->pole_id;
      const bool replace_b =
          split_span->port_b_id == port_id &&
          split_span->endpoint_node_b_id == node->pole_id;
      if (replace_a == replace_b) {
        result.error =
            "authoritative migration unsupported: shared pair span endpoint is ambiguous";
        return false;
      }

      const Port source = *source_port;
      EditResult<ObjectId> added =
          trial.AddPort(source.owner_pole_id, source.world_position, source.kind,
                        source.layer, source.direction);
      if (!added.ok) {
        result.error = "authoritative migration unsupported: " + added.error;
        return false;
      }
      Port* created =
          trial.authoritative_.edit_state.ports.find(added.value);
      if (created == nullptr) {
        result.error =
            "authoritative migration unsupported: split port was not created";
        return false;
      }
      const ObjectId new_port_id = created->id;
      const std::string new_display_id = created->display_id;
      *created = source;
      created->id = new_port_id;
      created->display_id = new_display_id;
      split.port_id = new_port_id;
      if (replace_a) {
        split_span->port_a_id = new_port_id;
      } else {
        split_span->port_b_id = new_port_id;
      }
    }
    return true;
  };
  if (!migrate_shared_pair_ports()) {
    result.classify_error();
    return result;
  }

  for (const Port& port : trial.authoritative_.edit_state.ports.items()) {
    index_add(trial.runtime_.relation_index.ports_by_pole, port.owner_pole_id, port.id);
  }
  for (const Anchor& anchor : trial.authoritative_.edit_state.anchors.items()) {
    index_add(trial.runtime_.relation_index.anchors_by_pole, anchor.owner_pole_id, anchor.id);
  }
  std::uint64_t loaded_span_data_version = 1;
  for (const Span& span : trial.authoritative_.edit_state.spans.items()) {
    trial.add_span_to_index(span);
    trial.initialize_span_runtime_state(span.id);
    trial.runtime_.span_runtime_states.at(span.id).data_version = loaded_span_data_version++;
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
      result.classify_error();
      return result;
    }
  }
  for (std::size_t position = 0; position < graph.edge_bundles.size(); ++position) {
    const SavedBackboneEdgeBundle& edge_bundle = graph.edge_bundles[position];
    index_add(trial.runtime_.backbone_index.edge_bundles, edge_bundle.edge_id, edge_bundle.edge_bundle_id);
    trial.runtime_.backbone_index.edge_bundle_by_edge_and_bundle[
        {edge_bundle.edge_id, edge_bundle.bundle_id}] = edge_bundle.edge_bundle_id;
    trial.runtime_.backbone_index.edge_bundle_positions[edge_bundle.edge_bundle_id] = position;
    index_add(trial.runtime_.backbone_index.bundle_edge, edge_bundle.bundle_id, edge_bundle.edge_id);
    for (ObjectId span_id : edge_bundle.span_ids) {
      index_add(trial.runtime_.backbone_index.edge_bundle_spans, edge_bundle.edge_bundle_id, span_id);
      if (!trial.runtime_.backbone_index.span_edge_bundle.emplace(span_id, edge_bundle.edge_bundle_id).second) {
        result.error = "authoritative deserialization: span belongs to multiple edge bundles";
        result.classify_error();
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
  auto saved_node_exists = [&](ObjectId node_id) {
    return std::any_of(graph.nodes.begin(), graph.nodes.end(),
                       [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  };
  auto graph_edge_by_id = [&](ObjectId edge_id) -> const SavedBackboneEdge* {
    const auto found = std::find_if(graph.edges.begin(), graph.edges.end(),
                                    [&](const SavedBackboneEdge& edge) { return edge.edge_id == edge_id; });
    return found == graph.edges.end() ? nullptr : &*found;
  };
  auto saved_edge_bundle_for = [&](ObjectId edge_bundle_id) -> const SavedBackboneEdgeBundle* {
    const auto found = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                                    [&](const SavedBackboneEdgeBundle& edge_bundle) {
                                      return edge_bundle.edge_bundle_id == edge_bundle_id;
                                    });
    return found == graph.edge_bundles.end() ? nullptr : &*found;
  };
  auto edge_bundle_incident_to_node = [&](ObjectId edge_bundle_id, ObjectId node_id) {
    const SavedBackboneEdgeBundle* edge_bundle = saved_edge_bundle_for(edge_bundle_id);
    const SavedBackboneEdge* edge =
        edge_bundle == nullptr ? nullptr : graph_edge_by_id(edge_bundle->edge_id);
    return edge != nullptr && (edge->node_a == node_id || edge->node_b == node_id);
  };
  auto edge_bundle_has_lane = [&](ObjectId edge_bundle_id, std::size_t lane) {
    for (const SavedBackboneSpanBinding& binding : graph.span_bindings) {
      if (binding.edge_bundle_id == edge_bundle_id && binding.lane_index == lane) {
        return true;
      }
    }
    return false;
  };
  for (const SavedBackboneRowContinuity& continuity : graph.row_continuities) {
    if (!saved_node_exists(continuity.node_id) ||
        !edge_bundle_incident_to_node(continuity.a.edge_bundle_id, continuity.node_id) ||
        !edge_bundle_incident_to_node(continuity.b.edge_bundle_id, continuity.node_id) ||
        continuity.a.edge_bundle_id == continuity.b.edge_bundle_id ||
        !edge_bundle_has_lane(continuity.a.edge_bundle_id, continuity.a.lane_index) ||
        !edge_bundle_has_lane(continuity.b.edge_bundle_id, continuity.b.lane_index)) {
      result.error = "backbone internal: row continuity endpoint is missing";
      result.classify_error();
      return result;
    }
  }

  const CoreStateIdentityStorage persisted_identity = trial.identity_;
  const CoreStateAuthoritativeStorage persisted_authoritative = trial.authoritative_;
  const auto rebuilt = trial.rebuild_loaded_outputs();
  if (!rebuilt.ok) {
    result.error = rebuilt.error;
    result.error_kind = rebuilt.effective_error_kind();
    return result;
  }
  trial.identity_ = persisted_identity;
  trial.authoritative_ = persisted_authoritative;
  trial.runtime_.relation_index = {};
  trial.runtime_.backbone_index = {};
  for (const Port& port : trial.authoritative_.edit_state.ports.items()) {
    index_add(trial.runtime_.relation_index.ports_by_pole, port.owner_pole_id, port.id);
  }
  for (const Anchor& anchor : trial.authoritative_.edit_state.anchors.items()) {
    index_add(trial.runtime_.relation_index.anchors_by_pole, anchor.owner_pole_id, anchor.id);
  }
  for (const Span& span : trial.authoritative_.edit_state.spans.items()) {
    trial.add_span_to_index(span);
  }
  for (const Attachment& attachment : trial.authoritative_.edit_state.attachments.items()) {
    index_add(trial.runtime_.relation_index.attachments_by_span, attachment.span_id, attachment.id);
  }
  for (const SavedBackboneNode& node : trial.authoritative_.backbone.nodes) {
    if (node.pole_id != kInvalidObjectId) trial.runtime_.backbone_index.pole_node[node.pole_id] = node.node_id;
  }
  for (const SavedBackboneEdge& edge : trial.authoritative_.backbone.edges) {
    index_add(trial.runtime_.backbone_index.node_edges, edge.node_a, edge.edge_id);
    index_add(trial.runtime_.backbone_index.node_edges, edge.node_b, edge.edge_id);
    const BackboneEdgeKey key{std::min(edge.node_a, edge.node_b), std::max(edge.node_a, edge.node_b)};
    trial.runtime_.backbone_index.edge_by_nodes[key] = edge.edge_id;
  }
  for (std::size_t position = 0; position < trial.authoritative_.backbone.edge_bundles.size(); ++position) {
    const SavedBackboneEdgeBundle& edge_bundle = trial.authoritative_.backbone.edge_bundles[position];
    index_add(trial.runtime_.backbone_index.edge_bundles, edge_bundle.edge_id, edge_bundle.edge_bundle_id);
    trial.runtime_.backbone_index.edge_bundle_by_edge_and_bundle[
        {edge_bundle.edge_id, edge_bundle.bundle_id}] = edge_bundle.edge_bundle_id;
    trial.runtime_.backbone_index.edge_bundle_positions[edge_bundle.edge_bundle_id] = position;
    index_add(trial.runtime_.backbone_index.bundle_edge, edge_bundle.bundle_id, edge_bundle.edge_id);
    for (ObjectId span_id : edge_bundle.span_ids) {
      index_add(trial.runtime_.backbone_index.edge_bundle_spans, edge_bundle.edge_bundle_id, span_id);
      trial.runtime_.backbone_index.span_edge_bundle[span_id] = edge_bundle.edge_bundle_id;
    }
  }
  for (std::size_t i = 0; i < trial.authoritative_.backbone.span_bindings.size(); ++i) {
    const SavedBackboneSpanBinding& binding = trial.authoritative_.backbone.span_bindings[i];
    trial.runtime_.backbone_index.edge_bundle_span_bindings[binding.edge_bundle_id].push_back(i);
    trial.runtime_.backbone_index.span_bindings_by_span[binding.span_id].push_back(i);
  }
  for (std::size_t i = 0; i < trial.authoritative_.backbone.port_bindings.size(); ++i) {
    const SavedBackbonePortBinding& binding = trial.authoritative_.backbone.port_bindings[i];
    trial.runtime_.backbone_index.edge_bundle_ports[binding.edge_bundle_id].push_back(i);
    trial.runtime_.backbone_index.port_bindings_by_port[binding.port_id].push_back(i);
  }
  const ValidationResult validation = trial.Validate();
  if (!validation.ok()) {
    result.error = "authoritative deserialization: loaded state failed validation";
    const auto issue = std::find_if(validation.issues.begin(), validation.issues.end(),
                                    [](const ValidationIssue& candidate) {
                                      return candidate.severity == ValidationSeverity::kError;
                                    });
    if (issue != validation.issues.end()) {
      result.error += ": " + issue->code + ": " + issue->message;
      if (issue->object_id != kInvalidObjectId) {
        result.error += " (object " + std::to_string(issue->object_id) + ")";
      }
    }
    result.classify_error();
    return result;
  }

  identity_ = std::move(trial.identity_);
  authoritative_ = std::move(trial.authoritative_);
  runtime_ = std::move(trial.runtime_);
  session_ = std::move(trial.session_);
  debug_ = std::move(trial.debug_);
  result.ok = true;
  result.value = true;
  return result;
}

} // namespace wire::core
