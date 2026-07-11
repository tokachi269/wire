#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <type_traits>
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

std::string child(const std::string& prefix, std::string_view field) {
  return prefix + "." + std::string(field);
}

std::string indexed(const std::string& prefix, std::size_t index) {
  return prefix + "." + std::to_string(index);
}

void write_vec3(StateWriter& writer, const std::string& prefix, const Vec3d& value) {
  writer.value(child(prefix, "x"), value.x);
  writer.value(child(prefix, "y"), value.y);
  writer.value(child(prefix, "z"), value.z);
}

void write_transform(StateWriter& writer, const std::string& prefix, const Transformd& value) {
  write_vec3(writer, child(prefix, "position"), value.position);
  write_vec3(writer, child(prefix, "rotation_euler_deg"), value.rotation_euler_deg);
  write_vec3(writer, child(prefix, "scale"), value.scale);
}

void write_frame(StateWriter& writer, const std::string& prefix, const Frame3d& value) {
  write_vec3(writer, child(prefix, "origin"), value.origin);
  write_vec3(writer, child(prefix, "forward"), value.forward);
  write_vec3(writer, child(prefix, "right"), value.right);
  write_vec3(writer, child(prefix, "up"), value.up);
}

void write_generation(StateWriter& writer, const std::string& prefix, const GenerationMeta& value) {
  writer.value(child(prefix, "generated"), value.generated);
  writer.value(child(prefix, "source"), value.source);
  writer.value(child(prefix, "generation_session_id"), value.generation_session_id);
  writer.value(child(prefix, "generation_order"), value.generation_order);
}

void write_pole_context(StateWriter& writer, const std::string& prefix, const PoleContextInfo& value) {
  writer.value(child(prefix, "kind"), value.kind);
  writer.value(child(prefix, "corner_angle_deg"), value.corner_angle_deg);
  writer.value(child(prefix, "corner_turn_sign"), value.corner_turn_sign);
  writer.value(child(prefix, "side_scale"), value.side_scale);
  writer.value(child(prefix, "angle_correction_applied"), value.angle_correction_applied);
}

void write_pole(StateWriter& writer, const std::string& prefix, const Pole& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "display_id"), value.display_id);
  writer.string_value(child(prefix, "name"), value.name);
  write_transform(writer, child(prefix, "world_transform"), value.world_transform);
  writer.value(child(prefix, "tilt_magnitude_deg"), value.tilt_magnitude_deg);
  writer.value(child(prefix, "height_m"), value.height_m);
  writer.value(child(prefix, "kind"), value.kind);
  writer.value(child(prefix, "pole_type_id"), value.pole_type_id);
  write_pole_context(writer, child(prefix, "context"), value.context);
  writer.value(child(prefix, "placement_mode"), value.placement_mode);
  writer.value(child(prefix, "user_edited"), value.user_edited);
  writer.value(child(prefix, "placement_override_flag"), value.placement_override_flag);
  write_generation(writer, child(prefix, "generation"), value.generation);
}

void write_port(StateWriter& writer, const std::string& prefix, const Port& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "display_id"), value.display_id);
  writer.value(child(prefix, "owner_pole_id"), value.owner_pole_id);
  write_vec3(writer, child(prefix, "world_position"), value.world_position);
  writer.value(child(prefix, "kind"), value.kind);
  writer.value(child(prefix, "layer"), value.layer);
  write_frame(writer, child(prefix, "direction"), value.direction);
  writer.value(child(prefix, "category"), value.category);
  writer.value(child(prefix, "template_layer"), value.template_layer);
  writer.value(child(prefix, "template_side"), value.template_side);
  writer.value(child(prefix, "template_role"), value.template_role);
  writer.value(child(prefix, "generated_from_template"), value.generated_from_template);
  writer.value(child(prefix, "generated_by_rule"), value.generated_by_rule);
  writer.value(child(prefix, "placement_context"), value.placement_context);
  writer.value(child(prefix, "angle_correction_applied"), value.angle_correction_applied);
  writer.value(child(prefix, "side_scale_applied"), value.side_scale_applied);
  writer.value(child(prefix, "position_mode"), value.position_mode);
  writer.value(child(prefix, "placement_source"), value.placement_source);
  writer.value(child(prefix, "user_edited_position"), value.user_edited_position);
  writer.value(child(prefix, "placement_override_flag"), value.placement_override_flag);
  writer.value(child(prefix, "orientation_override_flag"), value.orientation_override_flag);
}

void write_anchor(StateWriter& writer, const std::string& prefix, const Anchor& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "display_id"), value.display_id);
  writer.value(child(prefix, "owner_pole_id"), value.owner_pole_id);
  write_vec3(writer, child(prefix, "world_position"), value.world_position);
  writer.value(child(prefix, "support_kind"), value.support_kind);
  writer.value(child(prefix, "support_strength"), value.support_strength);
  writer.value(child(prefix, "generated_from_template"), value.generated_from_template);
}

void write_bundle(StateWriter& writer, const std::string& prefix, const Bundle& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "display_id"), value.display_id);
  writer.value(child(prefix, "conductor_count"), value.conductor_count);
  writer.value(child(prefix, "phase_spacing_m"), value.phase_spacing_m);
  writer.value(child(prefix, "bundle_template_id"), value.bundle_template_id);
}

void write_span(StateWriter& writer, const std::string& prefix, const Span& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "display_id"), value.display_id);
  writer.value(child(prefix, "port_a_id"), value.port_a_id);
  writer.value(child(prefix, "port_b_id"), value.port_b_id);
  writer.value(child(prefix, "endpoint_node_a_id"), value.endpoint_node_a_id);
  writer.value(child(prefix, "endpoint_node_b_id"), value.endpoint_node_b_id);
  writer.value(child(prefix, "kind"), value.kind);
  writer.value(child(prefix, "layer"), value.layer);
  writer.value(child(prefix, "bundle_id"), value.bundle_id);
  writer.value(child(prefix, "anchor_a_id"), value.anchor_a_id);
  writer.value(child(prefix, "anchor_b_id"), value.anchor_b_id);
  writer.value(child(prefix, "endpoint_attachment_a_id"), value.endpoint_attachment_a_id);
  writer.value(child(prefix, "endpoint_attachment_b_id"), value.endpoint_attachment_b_id);
  writer.value(child(prefix, "placement_context"), value.placement_context);
  writer.value(child(prefix, "generated_by_rule"), value.generated_by_rule);
  writer.value(child(prefix, "placement_override_flag"), value.placement_override_flag);
  writer.value(child(prefix, "reference_length_m"), value.reference_length_m);
  write_generation(writer, child(prefix, "generation"), value.generation);
}

void write_attachment(StateWriter& writer, const std::string& prefix, const Attachment& value) {
  writer.value(child(prefix, "id"), value.id);
  writer.string_value(child(prefix, "display_id"), value.display_id);
  writer.value(child(prefix, "span_id"), value.span_id);
  writer.value(child(prefix, "template_id"), value.template_id);
  writer.value(child(prefix, "t"), value.t);
  writer.value(child(prefix, "kind"), value.kind);
  writer.value(child(prefix, "display_offset_m"), value.display_offset_m);
  writer.value(child(prefix, "origin"), value.origin);
}

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

void write_bundle_mode(StateWriter& writer, const std::string& prefix, const SupportNodeBundleMode& value) {
  writer.value(child(prefix, "bundle_template_id"), value.bundle_template_id);
  writer.value(child(prefix, "mode"), value.mode);
}

void write_saved_node(StateWriter& writer, const std::string& prefix, const SavedBackboneNode& value) {
  writer.value(child(prefix, "node_id"), value.node_id);
  writer.value(child(prefix, "pole_id"), value.pole_id);
  writer.value(child(prefix, "support_kind"), value.support_kind);
  write_vec3(writer, child(prefix, "position"), value.position);
  writer.value(child(prefix, "has_source_edge"), value.has_source_edge);
  writer.value(child(prefix, "source_edge_node_a"), value.source_edge_node_a);
  writer.value(child(prefix, "source_edge_node_b"), value.source_edge_node_b);
  writer.value(child(prefix, "source_edge_t"), value.source_edge_t);
  writer.value(child(prefix, "path_point_index"), value.path_point_index);
  writer.value(child(prefix, "bundle_modes.count"), value.bundle_modes.size());
  for (std::size_t i = 0; i < value.bundle_modes.size(); ++i) {
    write_bundle_mode(writer, indexed(child(prefix, "bundle_modes"), i), value.bundle_modes[i]);
  }
}

void write_saved_edge(StateWriter& writer, const std::string& prefix, const SavedBackboneEdge& value) {
  writer.value(child(prefix, "edge_id"), value.edge_id);
  writer.value(child(prefix, "node_a"), value.node_a);
  writer.value(child(prefix, "node_b"), value.node_b);
  writer.value(child(prefix, "route"), value.route);
  writer.value(child(prefix, "order"), value.order);
  write_vec3(writer, child(prefix, "dir"), value.dir);
  writer.value(child(prefix, "lateral_offset_m"), value.lateral_offset_m);
}

void write_saved_edge_bundle(StateWriter& writer, const std::string& prefix,
                             const SavedBackboneEdgeBundle& value) {
  writer.value(child(prefix, "edge_bundle_id"), value.edge_bundle_id);
  writer.value(child(prefix, "edge_id"), value.edge_id);
  writer.value(child(prefix, "bundle_id"), value.bundle_id);
  writer.value(child(prefix, "edge_forward"), value.edge_forward);
  writer.value(child(prefix, "route"), value.route);
  writer.value(child(prefix, "order"), value.order);
  write_vec3(writer, child(prefix, "dir"), value.dir);
  writer.value(child(prefix, "span_ids.count"), value.span_ids.size());
  for (std::size_t i = 0; i < value.span_ids.size(); ++i) {
    writer.value(indexed(child(prefix, "span_ids"), i), value.span_ids[i]);
  }
}

void write_row_key(StateWriter& writer, const std::string& prefix, const SavedBackboneRowKey& value) {
  writer.value(child(prefix, "node_id"), value.node_id);
  writer.value(child(prefix, "source_is_open"), value.source_is_open);
  writer.value(child(prefix, "source_edge_a"), value.source_edge_a);
  writer.value(child(prefix, "source_edge_b"), value.source_edge_b);
}

void write_saved_port_binding(StateWriter& writer, const std::string& prefix,
                              const SavedBackbonePortBinding& value) {
  writer.value(child(prefix, "edge_bundle_id"), value.edge_bundle_id);
  write_row_key(writer, child(prefix, "row_key"), value.row_key);
  writer.value(child(prefix, "lane_index"), value.lane_index);
  writer.value(child(prefix, "bundle_template_id"), value.bundle_template_id);
  writer.value(child(prefix, "port_kind"), value.port_kind);
  writer.value(child(prefix, "port_layer"), value.port_layer);
  writer.value(child(prefix, "placement_band_id"), value.placement_band_id);
  writer.value(child(prefix, "layout_yaw_deg"), value.layout_yaw_deg);
  writer.value(child(prefix, "port_id"), value.port_id);
}

void write_saved_span_binding(StateWriter& writer, const std::string& prefix,
                              const SavedBackboneSpanBinding& value) {
  writer.value(child(prefix, "edge_bundle_id"), value.edge_bundle_id);
  writer.value(child(prefix, "lane_index"), value.lane_index);
  writer.value(child(prefix, "span_id"), value.span_id);
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

void write_backbone(StateWriter& writer, const SavedBackboneGraph& graph) {
  write_id_vector(writer, "authoritative.backbone.nodes", graph.nodes,
                  [](const SavedBackboneNode& value) { return value.node_id; }, write_saved_node);
  write_id_vector(writer, "authoritative.backbone.edges", graph.edges,
                  [](const SavedBackboneEdge& value) { return value.edge_id; }, write_saved_edge);
  write_id_vector(writer, "authoritative.backbone.edge_bundles", graph.edge_bundles,
                  [](const SavedBackboneEdgeBundle& value) { return value.edge_bundle_id; }, write_saved_edge_bundle);
  write_id_vector(writer, "authoritative.backbone.port_bindings", graph.port_bindings,
                  [](const SavedBackbonePortBinding& value) { return value.port_id; }, write_saved_port_binding);
  write_id_vector(writer, "authoritative.backbone.span_bindings", graph.span_bindings,
                  [](const SavedBackboneSpanBinding& value) { return value.span_id; }, write_saved_span_binding);
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

void write_pole_orientation_override(StateWriter& writer, const std::string& prefix,
                                     const PoleOrientationOverride& value) {
  writer.value(child(prefix, "base_yaw_deg.has"), value.base_yaw_deg.has_value());
  if (value.base_yaw_deg.has_value()) writer.value(child(prefix, "base_yaw_deg.value"), *value.base_yaw_deg);
  writer.value(child(prefix, "manual_yaw_deg.has"), value.manual_yaw_deg.has_value());
  if (value.manual_yaw_deg.has_value()) writer.value(child(prefix, "manual_yaw_deg.value"), *value.manual_yaw_deg);
  writer.value(child(prefix, "flip_180.has"), value.flip_180.has_value());
  if (value.flip_180.has_value()) writer.value(child(prefix, "flip_180.value"), *value.flip_180);
  writer.value(child(prefix, "version"), value.version);
}

void write_span_endpoint_override(StateWriter& writer, const std::string& prefix, const SpanEndpointOverride& value) {
  writer.value(child(prefix, "socket_a_id.has"), value.socket_a_id.has_value());
  if (value.socket_a_id.has_value()) writer.value(child(prefix, "socket_a_id.value"), *value.socket_a_id);
  writer.value(child(prefix, "socket_b_id.has"), value.socket_b_id.has_value());
  if (value.socket_b_id.has_value()) writer.value(child(prefix, "socket_b_id.value"), *value.socket_b_id);
  writer.value(child(prefix, "version"), value.version);
}

void write_span_support_override(StateWriter& writer, const std::string& prefix, const SpanSupportOverride& value) {
  writer.value(child(prefix, "branch_down_offset_m.has"), value.branch_down_offset_m.has_value());
  if (value.branch_down_offset_m.has_value()) {
    writer.value(child(prefix, "branch_down_offset_m.value"), *value.branch_down_offset_m);
  }
  writer.value(child(prefix, "version"), value.version);
}

void write_context_profile(StateWriter& writer, const ContextProfile& value) {
  writer.value("authoritative.context_profile.age", value.age);
  writer.value("authoritative.context_profile.clutter", value.clutter);
  writer.value("authoritative.context_profile.regularity", value.regularity);
  writer.value("authoritative.context_profile.service_mix", value.service_mix);
  writer.value("authoritative.context_profile.style_seed", value.style_seed);
}

void write_layout_settings(StateWriter& writer, const LayoutSettings& value) {
  writer.value("authoritative.layout_settings.angle_correction_enabled", value.angle_correction_enabled);
  writer.value("authoritative.layout_settings.corner_threshold_deg", value.corner_threshold_deg);
  writer.value("authoritative.layout_settings.min_side_scale", value.min_side_scale);
  writer.value("authoritative.layout_settings.max_side_scale", value.max_side_scale);
}

void write_geometry_settings(StateWriter& writer, const GeometrySettings& value) {
  writer.value("authoritative.geometry_settings.curve_samples", value.curve_samples);
  writer.value("authoritative.geometry_settings.sag_enabled", value.sag_enabled);
  writer.value("authoritative.geometry_settings.sag_factor", value.sag_factor);
  writer.value("authoritative.geometry_settings.pole_clearance_m", value.pole_clearance_m);
}

void write_visual_settings(StateWriter& writer, const VisualSettings& value) {
  writer.value("authoritative.visual_settings.enable_support_structures", value.enable_support_structures);
  writer.value("authoritative.visual_settings.enable_insulators", value.enable_insulators);
  writer.value("authoritative.visual_settings.support_center_threshold_m", value.support_center_threshold_m);
  writer.value("authoritative.visual_settings.support_arm_extra_m", value.support_arm_extra_m);
  writer.value("authoritative.visual_settings.support_arm_radius_m", value.support_arm_radius_m);
  writer.value("authoritative.visual_settings.insulator_radius_m", value.insulator_radius_m);
  writer.value("authoritative.visual_settings.insulator_length_m", value.insulator_length_m);
}

void write_variation_settings(StateWriter& writer, const VariationSettings& value) {
  writer.value("authoritative.variation_settings.enabled", value.enabled);
  writer.value("authoritative.variation_settings.global_seed", value.global_seed);
  writer.value("authoritative.variation_settings.world_cell_size_m", value.world_cell_size_m);
  writer.value("authoritative.variation_settings.world_bias_scale", value.world_bias_scale);
  writer.value("authoritative.variation_settings.flow_bias_scale", value.flow_bias_scale);
  writer.value("authoritative.variation_settings.pole_delta_scale", value.pole_delta_scale);
  writer.value("authoritative.variation_settings.local_jitter_scale", value.local_jitter_scale);
  writer.value("authoritative.variation_settings.sag_variation_scale", value.sag_variation_scale);
  writer.value("authoritative.variation_settings.branch_down_offset_variation_scale",
               value.branch_down_offset_variation_scale);
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

EditResult<bool> CoreState::DeserializeAuthoritative(const std::string&) {
  EditResult<bool> result{};
  result.error = "authoritative deserialization is not implemented";
  return result;
}

} // namespace wire::core
