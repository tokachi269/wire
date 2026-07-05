#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"

namespace {

using emscripten::val;
using wire::core::BackboneBundleSpec;
using wire::core::BackboneSpec;
using wire::core::BundleKind;
using wire::core::CoreState;
using wire::core::CoreView;
using wire::core::PoleTypeId;
using wire::core::SpanLayer;
using wire::core::Vec3d;

[[nodiscard]] val result_value(bool ok, const std::string& error) {
  val result = val::object();
  result.set("ok", ok);
  result.set("error", error);
  return result;
}

template <typename T> [[nodiscard]] T property(const val& object, const char* name) {
  return object[name].as<T>();
}

[[nodiscard]] BundleKind bundle_kind(int raw) {
  if (raw < static_cast<int>(BundleKind::kLowVoltage) ||
      raw > static_cast<int>(BundleKind::kOpticalWithSupport)) {
    throw std::invalid_argument("bundle template id is out of range");
  }
  return static_cast<BundleKind>(raw);
}

[[nodiscard]] SpanLayer span_layer(BundleKind kind) {
  switch (kind) {
  case BundleKind::kHighVoltage:
    return SpanLayer::kHighVoltage;
  case BundleKind::kCommunication:
    return SpanLayer::kCommunication;
  case BundleKind::kOptical:
  case BundleKind::kOpticalWithSupport:
    return SpanLayer::kOptical;
  case BundleKind::kDrop:
    return SpanLayer::kDrop;
  case BundleKind::kLowVoltage:
  default:
    return SpanLayer::kLowVoltage;
  }
}

class WireState {
public:
  WireState() : state_(std::make_unique<CoreState>()) {}

  val generate(const val& flat_points, const val& bundle_template_ids, double interval_m, int pole_type_id,
               const val& counts, int direction_mode) {
    const std::size_t value_count = flat_points["length"].as<std::size_t>();
    if (value_count % 3 != 0) {
      return result_value(false, "point array length must be divisible by 3");
    }

    BackboneSpec spec{};
    spec.path.polyline.reserve(value_count / 3);
    for (std::size_t index = 0; index < value_count; index += 3) {
      spec.path.polyline.push_back(
          Vec3d{flat_points[index].as<double>(), flat_points[index + 1].as<double>(),
                flat_points[index + 2].as<double>()});
    }
    spec.interval_m = interval_m;
    spec.pole_type_id = static_cast<PoleTypeId>(pole_type_id);
    spec.direction_mode = static_cast<wire::core::PathDirectionMode>(direction_mode);

    const std::size_t bundle_count = bundle_template_ids["length"].as<std::size_t>();
    if (bundle_count == 0 || counts["length"].as<std::size_t>() != bundle_count) {
      return result_value(false, "bundle template ids and counts must be non-empty and aligned");
    }
    for (std::size_t index = 0; index < bundle_count; ++index) {
      try {
        const BundleKind kind = bundle_kind(bundle_template_ids[index].as<int>());
        spec.bundles.push_back(BackboneBundleSpec{kind, span_layer(kind), counts[index].as<int>()});
      } catch (const std::invalid_argument& error) {
        return result_value(false, error.what());
      }
    }

    const auto generated = state_->GenerateFromBackboneSpec(spec);
    val result = result_value(generated.ok, generated.error);
    result.set("generatedPoleCount", generated.value.generated_pole_ids.size());
    result.set("generatedSpanCount", generated.value.generated_span_ids.size());
    result.set("totalMs", generated.value.timing.total_ms);
    return result;
  }

  [[nodiscard]] std::size_t visual_part_count() const {
    return state_->visual_curve_parts().parts.size();
  }

  [[nodiscard]] val visual_part(std::size_t index) const {
    const auto& parts = state_->visual_curve_parts().parts;
    if (index >= parts.size()) {
      throw std::out_of_range("visual part index is out of range");
    }
    const auto& part = parts[index];
    val output = val::object();
    output.set("kind", static_cast<int>(part.kind));
    output.set("wireRadius", part.wire_radius_m);
    output.set("colorRgba", part.color_rgba);
    output.set("sourceNodeId", std::to_string(part.source_node_id));
    output.set("sourceEdgeId", std::to_string(part.source_edge_id));
    output.set("sourceSpanId", std::to_string(part.source_span_id));
    output.set("sourceBundleId", std::to_string(part.source_bundle_id));
    output.set("bundleTemplateId", static_cast<int>(part.bundle_template_id));
    output.set("laneIndex", part.lane_index);
    output.set("sampleCount", part.samples.size());
    return output;
  }

  val visual_part_samples(std::size_t index) {
    const auto& parts = state_->visual_curve_parts().parts;
    if (index >= parts.size()) {
      throw std::out_of_range("visual part index is out of range");
    }
    sample_buffer_.clear();
    sample_buffer_.reserve(parts[index].samples.size() * 3);
    for (const Vec3d& point : parts[index].samples) {
      sample_buffer_.push_back(point.x);
      sample_buffer_.push_back(point.y);
      sample_buffer_.push_back(point.z);
    }
    return val(emscripten::typed_memory_view(sample_buffer_.size(), sample_buffer_.data()));
  }

  [[nodiscard]] std::size_t pole_count() const {
    return CoreView(*state_).poles().size();
  }

  [[nodiscard]] val pole(std::size_t index) const {
    const auto& poles = CoreView(*state_).poles();
    const auto* pole = poles.at_index(index);
    if (pole == nullptr) {
      throw std::out_of_range("pole index is out of range");
    }
    val output = val::object();
    output.set("id", std::to_string(pole->id));
    output.set("height", pole->height_m);
    output.set("positionX", pole->world_transform.position.x);
    output.set("positionY", pole->world_transform.position.y);
    output.set("positionZ", pole->world_transform.position.z);
    output.set("rotationX", pole->world_transform.rotation_euler_deg.x);
    output.set("rotationY", pole->world_transform.rotation_euler_deg.y);
    output.set("rotationZ", pole->world_transform.rotation_euler_deg.z);
    output.set("scaleX", pole->world_transform.scale.x);
    output.set("scaleY", pole->world_transform.scale.y);
    output.set("scaleZ", pole->world_transform.scale.z);
    return output;
  }

  [[nodiscard]] std::size_t port_count() const {
    return CoreView(*state_).ports().size();
  }

  [[nodiscard]] val port(std::size_t index) const {
    const auto* port = CoreView(*state_).ports().at_index(index);
    if (port == nullptr) {
      throw std::out_of_range("port index is out of range");
    }
    val output = val::object();
    output.set("id", std::to_string(port->id));
    output.set("ownerPoleId", std::to_string(port->owner_pole_id));
    output.set("x", port->world_position.x);
    output.set("y", port->world_position.y);
    output.set("z", port->world_position.z);
    output.set("category", static_cast<int>(port->category));
    output.set("layer", port->template_layer);
    return output;
  }

  [[nodiscard]] std::size_t span_count() const {
    return CoreView(*state_).spans().size();
  }

  [[nodiscard]] val span(std::size_t index) const {
    const auto* span = CoreView(*state_).spans().at_index(index);
    if (span == nullptr) {
      throw std::out_of_range("span index is out of range");
    }
    val output = val::object();
    output.set("id", std::to_string(span->id));
    output.set("portAId", std::to_string(span->port_a_id));
    output.set("portBId", std::to_string(span->port_b_id));
    output.set("bundleId", std::to_string(span->bundle_id));
    return output;
  }

  [[nodiscard]] std::size_t support_node_count() {
    support_nodes_ = state_->SavedBackboneResult().nodes;
    return support_nodes_.size();
  }

  [[nodiscard]] val support_node(std::size_t index) const {
    if (index >= support_nodes_.size()) {
      throw std::out_of_range("support node index is out of range");
    }
    const auto& node = support_nodes_[index];
    val output = val::object();
    output.set("id", std::to_string(node.node_id));
    output.set("kind", static_cast<int>(node.support_kind));
    output.set("poleId", std::to_string(node.pole_id));
    output.set("x", node.position.x);
    output.set("y", node.position.y);
    output.set("z", node.position.z);
    return output;
  }

  val clear_pole_orientation_override(const std::string& pole_id) {
    const auto result =
        state_->ClearPoleOrientationOverride(static_cast<wire::core::ObjectId>(std::stoull(pole_id)));
    return result_value(result.ok, result.error);
  }

  val clear_span_socket_override(const std::string& span_id, bool is_start_endpoint) {
    const auto result = state_->ClearSpanEndpointSocketOverride(
        static_cast<wire::core::ObjectId>(std::stoull(span_id)), is_start_endpoint);
    return result_value(result.ok, result.error);
  }

  val clear_span_branch_down_override(const std::string& span_id) {
    const auto result = state_->ClearSpanBranchDownOffsetOverride(
        static_cast<wire::core::ObjectId>(std::stoull(span_id)));
    return result_value(result.ok, result.error);
  }

  [[nodiscard]] std::size_t bundle_template_count() const {
    return CoreView(*state_).bundle_templates().size();
  }

  [[nodiscard]] val bundle_template(std::size_t index) const {
    const auto& templates = CoreView(*state_).bundle_templates();
    std::vector<BundleKind> ids{};
    ids.reserve(templates.size());
    for (const auto& [id, bundle_template] : templates) {
      (void)bundle_template;
      ids.push_back(id);
    }
    std::ranges::sort(ids, {}, [](BundleKind id) { return static_cast<int>(id); });
    if (index >= ids.size()) {
      throw std::out_of_range("bundle template index is out of range");
    }
    const auto& bundle_template = templates.at(ids[index]);
    val output = val::object();
    output.set("id", static_cast<int>(bundle_template.id));
    output.set("name", bundle_template.name);
    output.set("defaultCount", bundle_template.default_count);
    output.set("fixedCount", bundle_template.count_rule == wire::core::BundleCountRuleKind::kFixed);
    output.set("fixedCountValue", bundle_template.fixed_count);
    output.set("minCount", bundle_template.min_count);
    output.set("maxCount", bundle_template.max_count);
    output.set("cableTemplateId", bundle_template.cable_template_id);
    output.set("relatedPoleTypeId", bundle_template.related_pole_type_id);
    output.set("defaultLayer", static_cast<int>(bundle_template.default_layer));
    output.set("allowMirror", bundle_template.allow_mirror);
    output.set("allowMidairNode", bundle_template.allow_midair_node);
    output.set("allowMidairBranch", bundle_template.allow_midair_branch);
    output.set("groupedSupportFanoutSpacing", bundle_template.grouped_support_fanout_spacing_m);
    output.set("supportStyle", static_cast<int>(bundle_template.support_style));
    output.set("branchPolicy", static_cast<int>(bundle_template.branch_policy));
    output.set("continuityPolicy", static_cast<int>(bundle_template.continuity_policy));
    return output;
  }

  val update_bundle_template(const val& input) {
    const BundleKind id = bundle_kind(property<int>(input, "id"));
    const auto& templates = CoreView(*state_).bundle_templates();
    const auto it = templates.find(id);
    if (it == templates.end()) {
      return result_value(false, "bundle template is missing");
    }
    auto bundle_template = it->second;
    bundle_template.cable_template_id = property<wire::core::CableTemplateId>(input, "cableTemplateId");
    bundle_template.related_pole_type_id = property<PoleTypeId>(input, "relatedPoleTypeId");
    bundle_template.default_layer = static_cast<SpanLayer>(property<int>(input, "defaultLayer"));
    bundle_template.allow_mirror = property<bool>(input, "allowMirror");
    bundle_template.allow_midair_node = property<bool>(input, "allowMidairNode");
    bundle_template.allow_midair_branch = property<bool>(input, "allowMidairBranch");
    bundle_template.grouped_support_fanout_spacing_m = property<double>(input, "groupedSupportFanoutSpacing");
    bundle_template.support_style =
        static_cast<wire::core::BundleSupportStyleHint>(property<int>(input, "supportStyle"));
    bundle_template.branch_policy =
        static_cast<wire::core::BundleBranchPolicyHint>(property<int>(input, "branchPolicy"));
    bundle_template.continuity_policy =
        static_cast<wire::core::CableContinuityPolicyHint>(property<int>(input, "continuityPolicy"));
    const auto updated = state_->UpdateBundleTemplate(bundle_template);
    return result_value(updated.ok, updated.error);
  }

  val apply_related_pole_type(int bundle_template_id) {
    try {
      const auto updated =
          state_->ApplyBundleRelatedPoleTypeToExistingPoles(bundle_kind(bundle_template_id));
      return result_value(updated.ok, updated.error);
    } catch (const std::invalid_argument& error) {
      return result_value(false, error.what());
    }
  }

  [[nodiscard]] std::size_t cable_template_count() const {
    return CoreView(*state_).cable_templates().size();
  }

  [[nodiscard]] val cable_template(std::size_t index) const {
    const auto& templates = CoreView(*state_).cable_templates();
    std::vector<wire::core::CableTemplateId> ids{};
    ids.reserve(templates.size());
    for (const auto& [id, cable_template] : templates) {
      (void)cable_template;
      ids.push_back(id);
    }
    std::ranges::sort(ids);
    if (index >= ids.size()) {
      throw std::out_of_range("cable template index is out of range");
    }
    const auto& cable_template = templates.at(ids[index]);
    val output = val::object();
    output.set("id", cable_template.id);
    output.set("name", cable_template.name);
    output.set("outerDiameter", cable_template.outer_diameter_m);
    output.set("bendStiffness", cable_template.bend_stiffness);
    output.set("minBendRadius", cable_template.min_bend_radius_m);
    output.set("materialStyle", static_cast<int>(cable_template.material_style));
    output.set("colorRgba", cable_template.color_rgba);
    output.set("requiresInsulator", cable_template.requires_insulator);
    output.set("insulatorAttachmentHeight", cable_template.insulator_attachment_height_m);
    output.set("sagFactor", cable_template.sag_factor);
    output.set("slackFactor", cable_template.slack_factor);
    output.set("groupedFanoutSpacing", cable_template.default_grouped_support_fanout_spacing_m);
    output.set("continuityPolicy", static_cast<int>(cable_template.continuity_policy));
    const auto supplemental = std::ranges::find_if(
        cable_template.supplemental_paths, [](const wire::core::CableSupplementalPathTemplate& path) {
          return path.anchor_mode == wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
                 path.profile_kind == wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
        });
    const bool has_supplemental = supplemental != cable_template.supplemental_paths.end();
    output.set("supplementalEnabled", has_supplemental);
    output.set("supplementalLateralOffset", has_supplemental ? supplemental->lateral_offset_m : 0.0);
    output.set("supplementalVerticalOffset", has_supplemental ? supplemental->vertical_offset_m : 0.0);
    output.set("supplementalWobbleAmplitude", has_supplemental ? supplemental->wobble_amplitude_m : 0.0);
    output.set("supplementalWobbleWavelength", has_supplemental ? supplemental->wobble_wavelength_m : 0.0);
    output.set("supplementalWobblePhase", has_supplemental ? supplemental->wobble_phase_bias : 0.0);
    output.set("supplementalEndpointEnvelope", has_supplemental ? supplemental->endpoint_envelope_ratio : 0.0);
    return output;
  }

  val update_cable_template(const val& input, const val& preferred_span_ids) {
    const wire::core::CableTemplateId id = property<wire::core::CableTemplateId>(input, "id");
    const auto& templates = CoreView(*state_).cable_templates();
    const auto it = templates.find(id);
    if (it == templates.end()) {
      return result_value(false, "cable template is missing");
    }
    auto cable_template = it->second;
    cable_template.outer_diameter_m = property<double>(input, "outerDiameter");
    cable_template.bend_stiffness = property<double>(input, "bendStiffness");
    cable_template.min_bend_radius_m = property<double>(input, "minBendRadius");
    cable_template.material_style =
        static_cast<wire::core::CableMaterialStyleKind>(property<int>(input, "materialStyle"));
    cable_template.color_rgba = property<std::uint32_t>(input, "colorRgba");
    cable_template.requires_insulator = property<bool>(input, "requiresInsulator");
    cable_template.insulator_attachment_height_m = property<double>(input, "insulatorAttachmentHeight");
    cable_template.sag_factor = property<double>(input, "sagFactor");
    cable_template.slack_factor = property<double>(input, "slackFactor");
    cable_template.default_grouped_support_fanout_spacing_m = property<double>(input, "groupedFanoutSpacing");
    cable_template.continuity_policy =
        static_cast<wire::core::CableContinuityPolicyHint>(property<int>(input, "continuityPolicy"));

    auto supplemental = std::ranges::find_if(
        cable_template.supplemental_paths, [](const wire::core::CableSupplementalPathTemplate& path) {
          return path.anchor_mode == wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
                 path.profile_kind == wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
        });
    if (property<bool>(input, "supplementalEnabled")) {
      if (supplemental == cable_template.supplemental_paths.end()) {
        wire::core::CableSupplementalPathTemplate path{};
        path.anchor_mode = wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset;
        path.profile_kind = wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
        path.interaction_mode = wire::core::AttachmentLineInteractionMode::kAddInternalPath;
        cable_template.supplemental_paths.push_back(path);
        supplemental = std::prev(cable_template.supplemental_paths.end());
      }
      supplemental->lateral_offset_m = property<double>(input, "supplementalLateralOffset");
      supplemental->vertical_offset_m = property<double>(input, "supplementalVerticalOffset");
      supplemental->wobble_amplitude_m = property<double>(input, "supplementalWobbleAmplitude");
      supplemental->wobble_wavelength_m = property<double>(input, "supplementalWobbleWavelength");
      supplemental->wobble_phase_bias = property<double>(input, "supplementalWobblePhase");
      supplemental->endpoint_envelope_ratio = property<double>(input, "supplementalEndpointEnvelope");
    } else if (supplemental != cable_template.supplemental_paths.end()) {
      cable_template.supplemental_paths.erase(supplemental);
    }

    std::vector<wire::core::ObjectId> preferred{};
    const std::size_t preferred_count = preferred_span_ids["length"].as<std::size_t>();
    preferred.reserve(preferred_count);
    for (std::size_t preferred_index = 0; preferred_index < preferred_count; ++preferred_index) {
      preferred.push_back(
          static_cast<wire::core::ObjectId>(std::stoull(preferred_span_ids[preferred_index].as<std::string>())));
    }
    const auto updated = state_->UpdateCableTemplate(cable_template, preferred);
    return result_value(updated.ok, updated.error);
  }

  [[nodiscard]] std::size_t pole_template_count() const {
    return CoreView(*state_).pole_types().size();
  }

  [[nodiscard]] val pole_template(std::size_t index) const {
    const auto& templates = CoreView(*state_).pole_types();
    std::vector<PoleTypeId> ids{};
    ids.reserve(templates.size());
    for (const auto& [id, pole_template] : templates) {
      (void)pole_template;
      ids.push_back(id);
    }
    std::ranges::sort(ids);
    if (index >= ids.size()) {
      throw std::out_of_range("pole template index is out of range");
    }
    const auto& pole_template = templates.at(ids[index]);
    val output = val::object();
    output.set("id", pole_template.id);
    output.set("name", pole_template.name);
    output.set("description", pole_template.description);
    output.set("defaultHeight", pole_template.default_height_m);
    val bands = val::array();
    for (std::size_t band_index = 0; band_index < pole_template.port_bands.size(); ++band_index) {
      const auto& band = pole_template.port_bands[band_index];
      val item = val::object();
      item.set("bandId", band.band_id);
      item.set("category", static_cast<int>(band.category));
      item.set("layer", band.layer);
      item.set("side", static_cast<int>(band.side));
      item.set("role", static_cast<int>(band.role));
      item.set("lateralCenter", band.lateral_center_m);
      item.set("lateralMin", band.lateral_min_m);
      item.set("lateralMax", band.lateral_max_m);
      item.set("heightCenter", band.height_center_m);
      item.set("heightMin", band.height_min_m);
      item.set("heightMax", band.height_max_m);
      item.set("priority", band.priority);
      item.set("minSpacing", band.min_spacing_m);
      item.set("allowMultiple", band.allow_multiple);
      item.set("overflowPolicy", static_cast<int>(band.overflow_policy));
      item.set("enabled", band.enabled);
      bands.set(band_index, item);
    }
    output.set("portBands", bands);
    val slots = val::array();
    for (std::size_t slot_index = 0; slot_index < pole_template.anchor_slots.size(); ++slot_index) {
      const auto& slot = pole_template.anchor_slots[slot_index];
      val item = val::object();
      item.set("slotId", slot.slot_id);
      item.set("usage", static_cast<int>(slot.usage));
      item.set("localX", slot.local_position.x);
      item.set("localY", slot.local_position.y);
      item.set("localZ", slot.local_position.z);
      item.set("priority", slot.priority);
      item.set("enabled", slot.enabled);
      slots.set(slot_index, item);
    }
    output.set("anchorSlots", slots);
    return output;
  }

  val update_pole_template(const val& input) {
    wire::core::PoleTypeDefinition pole_template{};
    pole_template.id = property<PoleTypeId>(input, "id");
    pole_template.name = property<std::string>(input, "name");
    pole_template.description = property<std::string>(input, "description");
    pole_template.default_height_m = property<double>(input, "defaultHeight");
    const val bands = input["portBands"];
    const std::size_t band_count = bands["length"].as<std::size_t>();
    pole_template.port_bands.reserve(band_count);
    for (std::size_t band_index = 0; band_index < band_count; ++band_index) {
      const val item = bands[band_index];
      wire::core::PortPlacementBand band{};
      band.band_id = property<int>(item, "bandId");
      band.category = static_cast<wire::core::ConnectionCategory>(property<int>(item, "category"));
      band.layer = property<int>(item, "layer");
      band.side = static_cast<wire::core::SlotSide>(property<int>(item, "side"));
      band.role = static_cast<wire::core::SlotRole>(property<int>(item, "role"));
      band.lateral_center_m = property<double>(item, "lateralCenter");
      band.lateral_min_m = property<double>(item, "lateralMin");
      band.lateral_max_m = property<double>(item, "lateralMax");
      band.height_center_m = property<double>(item, "heightCenter");
      band.height_min_m = property<double>(item, "heightMin");
      band.height_max_m = property<double>(item, "heightMax");
      band.priority = property<int>(item, "priority");
      band.min_spacing_m = property<double>(item, "minSpacing");
      band.allow_multiple = property<bool>(item, "allowMultiple");
      band.overflow_policy =
          static_cast<wire::core::BandOverflowPolicy>(property<int>(item, "overflowPolicy"));
      band.enabled = property<bool>(item, "enabled");
      pole_template.port_bands.push_back(band);
    }
    const val slots = input["anchorSlots"];
    const std::size_t slot_count = slots["length"].as<std::size_t>();
    pole_template.anchor_slots.reserve(slot_count);
    for (std::size_t slot_index = 0; slot_index < slot_count; ++slot_index) {
      const val item = slots[slot_index];
      wire::core::AnchorSlotTemplate slot{};
      slot.slot_id = property<int>(item, "slotId");
      slot.usage = static_cast<wire::core::AnchorSupportKind>(property<int>(item, "usage"));
      slot.local_position = {
          property<double>(item, "localX"),
          property<double>(item, "localY"),
          property<double>(item, "localZ"),
      };
      slot.priority = property<int>(item, "priority");
      slot.enabled = property<bool>(item, "enabled");
      pole_template.anchor_slots.push_back(slot);
    }
    const auto updated = state_->UpdatePoleTypeDefinition(pole_template);
    return result_value(updated.ok, updated.error);
  }

  [[nodiscard]] val geometry_settings() const {
    const auto& settings = CoreView(*state_).geometry_settings();
    val output = val::object();
    output.set("curveSamples", settings.curve_samples);
    output.set("sagEnabled", settings.sag_enabled);
    output.set("sagFactor", settings.sag_factor);
    output.set("poleClearance", settings.pole_clearance_m);
    return output;
  }

  val update_geometry_settings(const val& input) {
    wire::core::GeometrySettings settings{};
    settings.curve_samples = property<int>(input, "curveSamples");
    settings.sag_enabled = property<bool>(input, "sagEnabled");
    settings.sag_factor = property<double>(input, "sagFactor");
    settings.pole_clearance_m = property<double>(input, "poleClearance");
    const auto updated = state_->UpdateGeometrySettings(settings, true);
    return result_value(updated.ok, updated.error);
  }

  [[nodiscard]] val layout_settings() const {
    const auto& settings = CoreView(*state_).layout_settings();
    val output = val::object();
    output.set("angleCorrectionEnabled", settings.angle_correction_enabled);
    output.set("cornerThresholdDeg", settings.corner_threshold_deg);
    output.set("minSideScale", settings.min_side_scale);
    output.set("maxSideScale", settings.max_side_scale);
    return output;
  }

  val update_layout_settings(const val& input) {
    wire::core::LayoutSettings settings{};
    settings.angle_correction_enabled = property<bool>(input, "angleCorrectionEnabled");
    settings.corner_threshold_deg = property<double>(input, "cornerThresholdDeg");
    settings.min_side_scale = property<double>(input, "minSideScale");
    settings.max_side_scale = property<double>(input, "maxSideScale");
    const auto updated = state_->UpdateLayoutSettings(settings);
    return result_value(updated.ok, updated.error);
  }

  [[nodiscard]] val visual_settings() const {
    const auto& settings = CoreView(*state_).visual_settings();
    val output = val::object();
    output.set("enableSupportStructures", settings.enable_support_structures);
    output.set("enableInsulators", settings.enable_insulators);
    output.set("supportCenterThreshold", settings.support_center_threshold_m);
    output.set("supportArmExtra", settings.support_arm_extra_m);
    output.set("insulatorRadius", settings.insulator_radius_m);
    output.set("insulatorLength", settings.insulator_length_m);
    return output;
  }

  val update_visual_settings(const val& input) {
    wire::core::VisualSettings settings{};
    settings.enable_support_structures = property<bool>(input, "enableSupportStructures");
    settings.enable_insulators = property<bool>(input, "enableInsulators");
    settings.support_center_threshold_m = property<double>(input, "supportCenterThreshold");
    settings.support_arm_extra_m = property<double>(input, "supportArmExtra");
    settings.insulator_radius_m = property<double>(input, "insulatorRadius");
    settings.insulator_length_m = property<double>(input, "insulatorLength");
    const auto updated = state_->UpdateVisualSettings(settings, true);
    return result_value(updated.ok, updated.error);
  }

  val apply_pole_tilt(const val& pole_ids, double max_tilt_deg) {
    std::vector<wire::core::ObjectId> ids{};
    const std::size_t count = pole_ids["length"].as<std::size_t>();
    ids.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      ids.push_back(static_cast<wire::core::ObjectId>(std::stoull(pole_ids[index].as<std::string>())));
    }
    const auto updated = state_->ApplyPoleTilt(ids, max_tilt_deg);
    return result_value(updated.ok, updated.error);
  }

  val reset_span_reference_lengths() {
    const auto updated = state_->ResetAllSpanReferenceLengths(true);
    return result_value(updated.ok, updated.error);
  }

private:
  std::unique_ptr<CoreState> state_;
  std::vector<double> sample_buffer_{};
  std::vector<wire::core::SupportNode> support_nodes_{};
};

} // namespace

EMSCRIPTEN_BINDINGS(wire_web_core) {
  emscripten::class_<WireState>("WireState")
      .constructor<>()
      .function("generate", &WireState::generate)
      .function("visualPartCount", &WireState::visual_part_count)
      .function("visualPart", &WireState::visual_part)
      .function("visualPartSamples", &WireState::visual_part_samples)
      .function("poleCount", &WireState::pole_count)
      .function("pole", &WireState::pole)
      .function("portCount", &WireState::port_count)
      .function("port", &WireState::port)
      .function("spanCount", &WireState::span_count)
      .function("span", &WireState::span)
      .function("supportNodeCount", &WireState::support_node_count)
      .function("supportNode", &WireState::support_node)
      .function("clearPoleOrientationOverride", &WireState::clear_pole_orientation_override)
      .function("clearSpanSocketOverride", &WireState::clear_span_socket_override)
      .function("clearSpanBranchDownOverride", &WireState::clear_span_branch_down_override)
      .function("bundleTemplateCount", &WireState::bundle_template_count)
      .function("bundleTemplate", &WireState::bundle_template)
      .function("updateBundleTemplate", &WireState::update_bundle_template)
      .function("applyRelatedPoleType", &WireState::apply_related_pole_type)
      .function("cableTemplateCount", &WireState::cable_template_count)
      .function("cableTemplate", &WireState::cable_template)
      .function("updateCableTemplate", &WireState::update_cable_template)
      .function("poleTemplateCount", &WireState::pole_template_count)
      .function("poleTemplate", &WireState::pole_template)
      .function("updatePoleTemplate", &WireState::update_pole_template)
      .function("geometrySettings", &WireState::geometry_settings)
      .function("updateGeometrySettings", &WireState::update_geometry_settings)
      .function("layoutSettings", &WireState::layout_settings)
      .function("updateLayoutSettings", &WireState::update_layout_settings)
      .function("visualSettings", &WireState::visual_settings)
      .function("updateVisualSettings", &WireState::update_visual_settings)
      .function("applyPoleTilt", &WireState::apply_pole_tilt)
      .function("resetSpanReferenceLengths", &WireState::reset_span_reference_lengths);
}
