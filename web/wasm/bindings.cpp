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
#include "wire/core/model_descriptor.hpp"

namespace {

using emscripten::val;
using wire::core::BackboneBundleSpec;
using wire::core::BackboneInputSpec;
using wire::core::BackboneSpec;
using wire::core::BundleKind;
using wire::core::BundleTemplateId;
using wire::core::CoreState;
using wire::core::CoreView;
using wire::core::GenerationTiming;
using wire::core::ObjectId;
using wire::core::PickHitKind;
using wire::core::PickResult;
using wire::core::PoleTypeId;
using wire::core::ResolveBranchPickOptions;
using wire::core::SpanLayer;
using wire::core::SupportKind;
using wire::core::Vec3d;

[[nodiscard]] val generation_timing_value(const GenerationTiming& timing) {
  val result = val::object();
  result.set("prepareMs", timing.prepare_ms);
  result.set("checkMs", timing.check_ms);
  result.set("pairsMs", timing.pairs_ms);
  result.set("preflightMs", timing.preflight_ms);
  result.set("intentMs", timing.intent_ms);
  result.set("supportGroupsMs", timing.support_groups_ms);
  result.set("emitMs", timing.emit_ms);
  result.set("saveGraphMs", timing.save_graph_ms);
  result.set("rulesMs", timing.rules_ms);
  result.set("layoutMs", timing.layout_ms);
  result.set("geomMs", timing.geom_ms);
  result.set("drawMs", timing.draw_ms);
  result.set("totalMs", timing.total_ms);
  return result;
}

[[nodiscard]] val result_value(bool ok, const std::string& error) {
  val result = val::object();
  result.set("ok", ok);
  result.set("error", error);
  return result;
}

template <typename T> [[nodiscard]] T property(const val& object, const char* name) {
  return object[name].as<T>();
}

[[nodiscard]] BundleTemplateId bundle_template_id(int raw) {
  return raw <= 0 ? wire::core::kInvalidBundleTemplateId : static_cast<BundleTemplateId>(raw);
}

[[nodiscard]] wire::core::Transformd transform_value(const val& input) {
  wire::core::Transformd transform{};
  transform.position = {
      property<double>(input, "positionX"),
      property<double>(input, "positionY"),
      property<double>(input, "positionZ"),
  };
  transform.rotation_euler_deg = {
      property<double>(input, "rotationX"),
      property<double>(input, "rotationY"),
      property<double>(input, "rotationZ"),
  };
  transform.scale = {
      property<double>(input, "scaleX"),
      property<double>(input, "scaleY"),
      property<double>(input, "scaleZ"),
  };
  return transform;
}

[[nodiscard]] wire::core::EditResult<bool> apply_model_bootstrap(CoreState& state,
                                                                  const val& input) {
  wire::core::EditResult<bool> result{};
  CoreState trial = state;
  const val assemblies = input["assemblies"];
  const std::size_t assembly_count = assemblies["length"].as<std::size_t>();
  for (std::size_t assembly_index = 0; assembly_index < assembly_count; ++assembly_index) {
    const val assembly_input = assemblies[assembly_index];
    wire::core::ModelAssemblyTemplate assembly{};
    assembly.id = property<wire::core::ModelAssemblyTemplateId>(assembly_input, "id");
    assembly.version = property<std::uint64_t>(assembly_input, "version");
    const val parts = assembly_input["parts"];
    const std::size_t part_count = parts["length"].as<std::size_t>();
    assembly.parts.reserve(part_count);
    for (std::size_t part_index = 0; part_index < part_count; ++part_index) {
      const val part_input = parts[part_index];
      wire::core::ModelDescriptor descriptor{};
      descriptor.measurement.name = property<std::string>(part_input, "descriptorName");
      descriptor.measurement.version = property<std::uint64_t>(part_input, "descriptorVersion");
      const val sockets = part_input["sockets"];
      const std::size_t socket_count = sockets["length"].as<std::size_t>();
      descriptor.measurement.sockets.reserve(socket_count);
      for (std::size_t socket_index = 0; socket_index < socket_count; ++socket_index) {
        const val socket_input = sockets[socket_index];
        wire::core::ModelSocket socket{};
        socket.name = property<std::string>(socket_input, "name");
        socket.local_position = {
            property<double>(socket_input, "positionX"),
            property<double>(socket_input, "positionY"),
            property<double>(socket_input, "positionZ"),
        };
        socket.local_direction = {
            property<double>(socket_input, "directionX"),
            property<double>(socket_input, "directionY"),
            property<double>(socket_input, "directionZ"),
        };
        descriptor.measurement.sockets.push_back(std::move(socket));
      }
      const auto built = wire::core::build_model_assembly_part(
          descriptor, property<std::uint32_t>(part_input, "partId"),
          property<std::string>(part_input, "modelKey"),
          transform_value(part_input["localTransform"]),
          static_cast<wire::core::ModelFitMode>(property<int>(part_input, "fitMode")));
      if (!built.report.conflicts.empty()) {
        result.error = "model bootstrap: " + built.report.conflicts.front().message;
        return result;
      }
      assembly.parts.push_back(built.part);
    }
    const val wire_socket = assembly_input["wireSocket"];
    if (!wire_socket.isNull() && !wire_socket.isUndefined()) {
      assembly.wire_socket = wire::core::AssemblySocketRef{
          property<std::uint32_t>(wire_socket, "partId"),
          property<std::string>(wire_socket, "socketName"),
      };
    }
    const val endpoint_mount_socket = assembly_input["endpointMountSocket"];
    if (!endpoint_mount_socket.isNull() && !endpoint_mount_socket.isUndefined()) {
      assembly.endpoint_mount_socket = wire::core::AssemblySocketRef{
          property<std::uint32_t>(endpoint_mount_socket, "partId"),
          property<std::string>(endpoint_mount_socket, "socketName"),
      };
    }
    const auto existing = CoreView(trial).model_assembly_templates().find(assembly.id);
    if (existing != CoreView(trial).model_assembly_templates().end()) {
      if (!(existing->second == assembly)) {
        if (assembly.version <= existing->second.version) {
          result.error = "model bootstrap: existing assembly differs without a newer adapter version";
          return result;
        }
        const auto updated = trial.UpdateModelAssemblyTemplate(assembly);
        if (!updated.ok) {
          result.error = updated.error;
          return result;
        }
      }
    } else {
      const auto registered = trial.RegisterModelAssemblyTemplate(assembly);
      if (!registered.ok) {
        result.error = registered.error;
        return result;
      }
    }
  }

  const val pole_assignments = input["poleAssignments"];
  const std::size_t pole_assignment_count = pole_assignments["length"].as<std::size_t>();
  for (std::size_t index = 0; index < pole_assignment_count; ++index) {
    const val assignment = pole_assignments[index];
    const PoleTypeId pole_type_id = property<PoleTypeId>(assignment, "poleTypeId");
    const auto pole_type_it = CoreView(trial).pole_types().find(pole_type_id);
    if (pole_type_it == CoreView(trial).pole_types().end()) {
      result.error = "model bootstrap: pole type is missing";
      return result;
    }
    auto pole_type = pole_type_it->second;
    pole_type.pole_visual_assembly_id =
        property<wire::core::ModelAssemblyTemplateId>(assignment, "assemblyId");
    const auto updated = trial.UpdatePoleTypeDefinition(pole_type);
    if (!updated.ok) {
      result.error = updated.error;
      return result;
    }
  }

  const val bundle_assignments = input["bundleAssignments"];
  const std::size_t bundle_assignment_count = bundle_assignments["length"].as<std::size_t>();
  for (std::size_t index = 0; index < bundle_assignment_count; ++index) {
    const val assignment = bundle_assignments[index];
    const BundleTemplateId id = property<BundleTemplateId>(assignment, "bundleTemplateId");
    const auto bundle_it = CoreView(trial).bundle_templates().find(id);
    if (bundle_it == CoreView(trial).bundle_templates().end()) {
      result.error = "model bootstrap: bundle template is missing";
      return result;
    }
    auto bundle_template = bundle_it->second;
    bundle_template.row_fixture_assembly_id =
        property<wire::core::ModelAssemblyTemplateId>(assignment, "rowAssemblyId");
    bundle_template.endpoint_fixture_assembly_id =
        property<wire::core::ModelAssemblyTemplateId>(assignment, "endpointAssemblyId");
    const auto updated = trial.UpdateBundleTemplate(bundle_template);
    if (!updated.ok) {
      result.error = updated.error;
      return result;
    }
  }

  state = std::move(trial);
  result.ok = true;
  result.value = true;
  return result;
}

class WireState {
public:
  WireState() : state_(std::make_unique<CoreState>()) {}

  val generate(const val& flat_points, const val& bundle_template_ids, double interval_m, int pole_type_id,
               const val& counts, int direction_mode, double max_tilt_deg, const val& node_specs = val::undefined()) {
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
    if (!node_specs.isUndefined() && !node_specs.isNull()) {
      const std::size_t node_spec_count = node_specs["length"].as<std::size_t>();
      for (std::size_t index = 0; index < node_spec_count; ++index) {
        const val item = node_specs[index];
        BackboneInputSpec::NodeSpec node_spec{};
        node_spec.point_index = item["pointIndex"].as<std::size_t>();
        node_spec.support_kind = static_cast<SupportKind>(item["supportKind"].as<int>());
        const std::string node_id = item["nodeId"].as<std::string>();
        if (!node_id.empty() && node_id != "0") {
          node_spec.node_id = static_cast<ObjectId>(std::stoull(node_id));
        }
        spec.path.node_specs.push_back(node_spec);
      }
    }
    spec.interval_m = interval_m;
    spec.pole_type_id = static_cast<PoleTypeId>(pole_type_id);
    spec.direction_mode = static_cast<wire::core::PathDirectionMode>(direction_mode);
    spec.pole_placement.enable_tilt = max_tilt_deg > 0.0;
    spec.pole_placement.max_tilt_deg = max_tilt_deg;

    const std::size_t bundle_count = bundle_template_ids["length"].as<std::size_t>();
    if (bundle_count == 0 || counts["length"].as<std::size_t>() != bundle_count) {
      return result_value(false, "bundle template ids and counts must be non-empty and aligned");
    }
    for (std::size_t index = 0; index < bundle_count; ++index) {
      const BundleTemplateId id = bundle_template_id(bundle_template_ids[index].as<int>());
      if (id == wire::core::kInvalidBundleTemplateId) {
        return result_value(false, "bundle template id is invalid");
      }
      const auto template_it = CoreView(*state_).bundle_templates().find(id);
      if (template_it == CoreView(*state_).bundle_templates().end()) {
        return result_value(false, "bundle template is missing");
      }
      spec.bundles.push_back(BackboneBundleSpec{id, template_it->second.default_layer, counts[index].as<int>()});
    }

    const auto generated = state_->GenerateFromBackboneSpec(spec);
    val result = result_value(generated.ok, generated.error);
    result.set("generatedPoleCount", generated.value.generated_pole_ids.size());
    result.set("generatedSpanCount", generated.value.generated_span_ids.size());
    result.set("totalMs", generated.value.timing.total_ms);
    result.set("timing", generation_timing_value(generated.value.timing));
    return result;
  }

  val resolve_branch_pick(const val& input, const val& selected_bundle_template_ids) {
    PickResult pick{};
    pick.hit_kind = static_cast<PickHitKind>(property<int>(input, "hitKind"));
    const std::string hit_id = property<std::string>(input, "hitId");
    if (!hit_id.empty() && hit_id != "0") {
      pick.hit_id = static_cast<ObjectId>(std::stoull(hit_id));
    }
    pick.hit_pos_world = Vec3d{
        property<double>(input, "hitX"),
        property<double>(input, "hitY"),
        property<double>(input, "hitZ"),
    };
    pick.has_segment_endpoints = property<bool>(input, "hasSegmentEndpoints");
    const std::string segment_node_a_id = property<std::string>(input, "segmentNodeAId");
    const std::string segment_node_b_id = property<std::string>(input, "segmentNodeBId");
    if (!segment_node_a_id.empty() && segment_node_a_id != "0") {
      pick.segment_node_a_id = static_cast<ObjectId>(std::stoull(segment_node_a_id));
    }
    if (!segment_node_b_id.empty() && segment_node_b_id != "0") {
      pick.segment_node_b_id = static_cast<ObjectId>(std::stoull(segment_node_b_id));
    }
    pick.segment_endpoint_a_world = Vec3d{
        property<double>(input, "segmentEndpointAX"),
        property<double>(input, "segmentEndpointAY"),
        property<double>(input, "segmentEndpointAZ"),
    };
    pick.segment_endpoint_b_world = Vec3d{
        property<double>(input, "segmentEndpointBX"),
        property<double>(input, "segmentEndpointBY"),
        property<double>(input, "segmentEndpointBZ"),
    };

    ResolveBranchPickOptions options{};
    options.create_midair_node = true;
    options.create_midair_node_set = true;
    const std::size_t selected_count = selected_bundle_template_ids["length"].as<std::size_t>();
    options.selected_bundle_template_ids.reserve(selected_count);
    for (std::size_t index = 0; index < selected_count; ++index) {
      const BundleTemplateId id = bundle_template_id(selected_bundle_template_ids[index].as<int>());
      if (id != wire::core::kInvalidBundleTemplateId) {
        options.selected_bundle_template_ids.push_back(id);
      }
    }

    const auto resolved = state_->ResolveBranchPick(pick, options);
    val output = result_value(resolved.ok, resolved.error);
    output.set("positionX", resolved.value.position.x);
    output.set("positionY", resolved.value.position.y);
    output.set("positionZ", resolved.value.position.z);
    output.set("supportKind", static_cast<int>(resolved.value.support_kind));
    output.set("nodeId", std::to_string(resolved.value.resolved_node_id));
    return output;
  }

  [[nodiscard]] val last_generation_timing() const {
    return generation_timing_value(CoreView(*state_).last_generation_timing());
  }



  val visual_scene() {
    const auto& parts = state_->visual_curve_parts().parts;
    sample_buffer_.clear();
    std::size_t sample_value_count = 0;
    for (const auto& part : parts) {
      sample_value_count += part.samples.size() * 3;
    }
    sample_buffer_.reserve(sample_value_count);
    val descriptors = val::array();
    for (std::size_t index = 0; index < parts.size(); ++index) {
      const auto& part = parts[index];
      const std::size_t sample_offset = sample_buffer_.size();
      for (const Vec3d& point : part.samples) {
        sample_buffer_.push_back(point.x);
        sample_buffer_.push_back(point.y);
        sample_buffer_.push_back(point.z);
      }
      std::string part_key = std::to_string(static_cast<int>(part.kind)) + ":" +
                             std::to_string(static_cast<int>(part.supplemental_kind)) + ":" +
                             std::to_string(part.source_node_id) + ":" + std::to_string(part.source_edge_id) + ":" +
                             std::to_string(part.source_span_id) + ":" + std::to_string(part.source_bundle_id) + ":" +
                             std::to_string(part.bundle_template_id) + ":" + std::to_string(part.lane_index);
      if (part.has_section_key) {
        part_key += ":s:" + std::to_string(part.section_key.logical_span_id) + ":" +
                    std::to_string(part.section_key.edge_bundle_id) + ":" +
                    std::to_string(part.section_key.rule_owner_id) + ":" + std::to_string(part.section_key.rule_id) +
                    ":" + std::to_string(part.section_key.instance_index);
      }
      for (ObjectId edge_id : part.incident_edge_ids) {
        part_key += ":e:" + std::to_string(edge_id);
      }
      val output = val::object();
      output.set("partKey", part_key);
      output.set("sourceVersion", std::to_string(part.source_version));
      output.set("sampleOffset", sample_offset);
      output.set("sampleCount", part.samples.size());
      output.set("kind", static_cast<int>(part.kind));
      output.set("wireRadius", part.wire_radius_m);
      output.set("colorRgba", part.color_rgba);
      output.set("sourceNodeId", std::to_string(part.source_node_id));
      output.set("sourceEdgeId", std::to_string(part.source_edge_id));
      output.set("sourceSpanId", std::to_string(part.source_span_id));
      output.set("sourceBundleId", std::to_string(part.source_bundle_id));
      output.set("bundleTemplateId", static_cast<int>(part.bundle_template_id));
      output.set("laneIndex", part.lane_index);
      output.set("runId", static_cast<double>(part.cable_run_id));
      descriptors.set(index, output);
    }
    val result = val::object();
    result.set("parts", descriptors);
    val models = val::array();
    const auto& model_instances = state_->visual_model_instances().instances;
    for (std::size_t index = 0; index < model_instances.size(); ++index) {
      const auto& instance = model_instances[index];
      val output = val::object();
      output.set("stableKey", instance.stable_key);
      output.set("modelKey", instance.model_key);
      output.set("contentVersion", std::to_string(instance.content_version));
      output.set("positionX", instance.world_transform.position.x);
      output.set("positionY", instance.world_transform.position.y);
      output.set("positionZ", instance.world_transform.position.z);
      output.set("rotationX", instance.world_transform.rotation_euler_deg.x);
      output.set("rotationY", instance.world_transform.rotation_euler_deg.y);
      output.set("rotationZ", instance.world_transform.rotation_euler_deg.z);
      output.set("scaleX", instance.world_transform.scale.x);
      output.set("scaleY", instance.world_transform.scale.y);
      output.set("scaleZ", instance.world_transform.scale.z);
      models.set(index, output);
    }
    result.set("models", models);
    result.set("samples", val(emscripten::typed_memory_view(sample_buffer_.size(), sample_buffer_.data())));
    return result;
  }

  val configure_model_assemblies(const val& input) {
    const auto configured = apply_model_bootstrap(*state_, input);
    return result_value(configured.ok, configured.error);
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
    output.set("poleTypeId", static_cast<int>(pole->pole_type_id));
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

  [[nodiscard]] std::size_t backbone_edge_count() {
    backbone_edges_ = state_->SavedBackboneEdges();
    return backbone_edges_.size();
  }

  [[nodiscard]] val backbone_edge(std::size_t index) const {
    if (index >= backbone_edges_.size()) {
      throw std::out_of_range("backbone edge index is out of range");
    }
    const auto& edge = backbone_edges_[index];
    val output = val::object();
    output.set("nodeAId", std::to_string(edge.node_a));
    output.set("nodeBId", std::to_string(edge.node_b));
    val bundles = val::array();
    for (std::size_t bundle_index = 0; bundle_index < edge.bundles.size(); ++bundle_index) {
      bundles.set(bundle_index, std::to_string(edge.bundles[bundle_index]));
    }
    output.set("bundleIds", bundles);
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
    std::vector<BundleTemplateId> ids{};
    ids.reserve(templates.size());
    for (const auto& [id, bundle_template] : templates) {
      (void)bundle_template;
      ids.push_back(id);
    }
    std::ranges::sort(ids);
    if (index >= ids.size()) {
      throw std::out_of_range("bundle template index is out of range");
    }
    const auto& bundle_template = templates.at(ids[index]);
    val output = val::object();
    output.set("id", static_cast<int>(bundle_template.id));
    output.set("kind", static_cast<int>(bundle_template.kind));
    output.set("name", bundle_template.name);
    output.set("defaultCount", bundle_template.default_count);
    output.set("fixedCount", bundle_template.count_rule == wire::core::BundleCountRuleKind::kFixed);
    output.set("fixedCountValue", bundle_template.fixed_count);
    output.set("minCount", bundle_template.min_count);
    output.set("maxCount", bundle_template.max_count);
    output.set("cableTemplateId", bundle_template.cable_template_id);
    output.set("relatedPoleTypeId", bundle_template.related_pole_type_id);
    output.set("defaultLayer", static_cast<int>(bundle_template.default_layer));
    output.set("allowMidairNode", bundle_template.allow_midair_node);
    output.set("allowMidairBranch", bundle_template.allow_midair_branch);
    output.set("supportWirePoleBandId", bundle_template.support_wire_pole_band_id);
    output.set("rowFixtureAssemblyId", bundle_template.row_fixture_assembly_id);
    output.set("endpointFixtureAssemblyId", bundle_template.endpoint_fixture_assembly_id);
    const auto& assembly = bundle_template.span_visual_assembly;
    val assembly_output = val::object();
    assembly_output.set("supportPathEnabled", assembly.support_path_enabled);
    assembly_output.set("helixEnabled", assembly.helix_enabled);
    assembly_output.set("helixRadius", assembly.helix_radius_m);
    assembly_output.set("helixClearance", assembly.helix_clearance_m);
    assembly_output.set("helixTurnsPerMeter", assembly.helix_turns_per_meter);
    assembly_output.set("helixSamplesPerTurn", assembly.helix_samples_per_turn);
    assembly_output.set("endpointTrim", assembly.endpoint_trim_m);
    assembly_output.set("memberWanderRatio", assembly.member_wander_ratio);
    assembly_output.set("memberWanderWavelength", assembly.member_wander_wavelength_m);
    assembly_output.set("memberWanderPhaseBias", assembly.member_wander_phase_bias);
    assembly_output.set("memberTwistTurnsPerMeter", assembly.member_twist_turns_per_meter);
    assembly_output.set("memberTwistPhase", assembly.member_twist_phase);
    output.set("spanVisualAssembly", assembly_output);
    val population_rules = val::array();
    for (std::size_t index = 0; index < bundle_template.population_rules.size(); ++index) {
      const auto& rule = bundle_template.population_rules[index];
      val item = val::object();
      item.set("ruleId", static_cast<double>(rule.rule_id));
      item.set("explicitSeed", static_cast<double>(rule.explicit_seed));
      item.set("priority", rule.priority);
      item.set("minExtraCount", rule.min_extra_count);
      item.set("maxExtraCount", rule.max_extra_count);
      item.set("minSpacing", rule.min_spacing_m);
      item.set("lateralMin", rule.lateral_min_m);
      item.set("lateralMax", rule.lateral_max_m);
      item.set("heightMin", rule.height_min_m);
      item.set("heightMax", rule.height_max_m);
      population_rules.set(index, item);
    }
    output.set("populationRules", population_rules);
    return output;
  }

  val update_bundle_template(const val& input) {
    const BundleTemplateId id = bundle_template_id(property<int>(input, "id"));
    if (id == wire::core::kInvalidBundleTemplateId) {
      return result_value(false, "bundle template id is invalid");
    }
    const auto& templates = CoreView(*state_).bundle_templates();
    const auto it = templates.find(id);
    if (it == templates.end()) {
      return result_value(false, "bundle template is missing");
    }
    auto bundle_template = it->second;
    bundle_template.cable_template_id = property<wire::core::CableTemplateId>(input, "cableTemplateId");
    bundle_template.related_pole_type_id = property<PoleTypeId>(input, "relatedPoleTypeId");
    bundle_template.default_layer = static_cast<SpanLayer>(property<int>(input, "defaultLayer"));
    bundle_template.allow_midair_node = property<bool>(input, "allowMidairNode");
    bundle_template.allow_midair_branch = property<bool>(input, "allowMidairBranch");
    bundle_template.support_wire_pole_band_id = property<int>(input, "supportWirePoleBandId");
    bundle_template.row_fixture_assembly_id =
        property<wire::core::ModelAssemblyTemplateId>(input, "rowFixtureAssemblyId");
    bundle_template.endpoint_fixture_assembly_id =
        property<wire::core::ModelAssemblyTemplateId>(input, "endpointFixtureAssemblyId");
    const val assembly = input["spanVisualAssembly"];
    bundle_template.span_visual_assembly.support_path_enabled =
        property<bool>(assembly, "supportPathEnabled");
    bundle_template.span_visual_assembly.helix_enabled = property<bool>(assembly, "helixEnabled");
    bundle_template.span_visual_assembly.helix_radius_m = property<double>(assembly, "helixRadius");
    bundle_template.span_visual_assembly.helix_clearance_m = property<double>(assembly, "helixClearance");
    bundle_template.span_visual_assembly.helix_turns_per_meter = property<double>(assembly, "helixTurnsPerMeter");
    bundle_template.span_visual_assembly.helix_samples_per_turn = property<int>(assembly, "helixSamplesPerTurn");
    bundle_template.span_visual_assembly.endpoint_trim_m = property<double>(assembly, "endpointTrim");
    bundle_template.span_visual_assembly.member_wander_ratio = property<double>(assembly, "memberWanderRatio");
    bundle_template.span_visual_assembly.member_wander_wavelength_m = property<double>(assembly, "memberWanderWavelength");
    bundle_template.span_visual_assembly.member_wander_phase_bias = property<double>(assembly, "memberWanderPhaseBias");
    bundle_template.span_visual_assembly.member_twist_turns_per_meter = property<double>(assembly, "memberTwistTurnsPerMeter");
    bundle_template.span_visual_assembly.member_twist_phase = property<double>(assembly, "memberTwistPhase");
    const std::vector<wire::core::CablePopulationRule> existing_population_rules =
        bundle_template.population_rules;
    bundle_template.population_rules.clear();
    const val population_rules = input["populationRules"];
    const std::size_t population_rule_count = population_rules["length"].as<std::size_t>();
    bundle_template.population_rules.reserve(population_rule_count);
    for (std::size_t index = 0; index < population_rule_count; ++index) {
      const val item = population_rules[index];
      const wire::core::CableSectionRuleId rule_id =
          property<wire::core::CableSectionRuleId>(item, "ruleId");
      const auto existing_rule = std::ranges::find_if(
          existing_population_rules,
          [rule_id](const wire::core::CablePopulationRule& rule) { return rule.rule_id == rule_id; });
      wire::core::CablePopulationRule rule =
          existing_rule == existing_population_rules.end() ? wire::core::CablePopulationRule{} : *existing_rule;
      rule.rule_id = rule_id;
      rule.explicit_seed = property<std::uint64_t>(item, "explicitSeed");
      rule.priority = property<int>(item, "priority");
      rule.min_extra_count = property<int>(item, "minExtraCount");
      rule.max_extra_count = property<int>(item, "maxExtraCount");
      rule.min_spacing_m = property<double>(item, "minSpacing");
      rule.lateral_min_m = property<double>(item, "lateralMin");
      rule.lateral_max_m = property<double>(item, "lateralMax");
      rule.height_min_m = property<double>(item, "heightMin");
      rule.height_max_m = property<double>(item, "heightMax");
      bundle_template.population_rules.push_back(rule);
    }
    const auto updated = state_->UpdateBundleTemplate(bundle_template);
    return result_value(updated.ok, updated.error);
  }

  val apply_related_pole_type(int bundle_template_id) {
    const BundleTemplateId id = ::bundle_template_id(bundle_template_id);
    if (id == wire::core::kInvalidBundleTemplateId) {
      return result_value(false, "bundle template id is invalid");
    }
    const auto updated = state_->ApplyBundleRelatedPoleTypeToExistingPoles(id);
    return result_value(updated.ok, updated.error);
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
    output.set("sagFactor", cable_template.sag_factor);
    output.set("slackFactor", cable_template.slack_factor);
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
    cable_template.sag_factor = property<double>(input, "sagFactor");
    cable_template.slack_factor = property<double>(input, "slackFactor");
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
    output.set("poleVisualAssemblyId", pole_template.pole_visual_assembly_id);
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
    pole_template.pole_visual_assembly_id =
        property<wire::core::ModelAssemblyTemplateId>(input, "poleVisualAssemblyId");
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

  std::string save_state() const {
    std::string text{};
    const auto saved = state_->SerializeAuthoritative(&text);
    return saved.ok ? text : std::string{};
  }

  val load_state(const std::string& text) {
    const auto loaded = state_->DeserializeAuthoritative(text);
    return result_value(loaded.ok, loaded.error);
  }

  val load_state_with_models(const std::string& text, const val& input) {
    CoreState trial = *state_;
    const auto loaded = trial.DeserializeAuthoritative(text);
    if (!loaded.ok) {
      return result_value(false, loaded.error);
    }
    const auto configured = apply_model_bootstrap(trial, input);
    if (!configured.ok) {
      return result_value(false, configured.error);
    }
    *state_ = std::move(trial);
    return result_value(true, {});
  }

private:
  std::unique_ptr<CoreState> state_;
  std::vector<double> sample_buffer_{};
  std::vector<wire::core::SupportNode> support_nodes_{};
  std::vector<wire::core::BackboneEdge> backbone_edges_{};
};

} // namespace

EMSCRIPTEN_BINDINGS(wire_web_core) {
  emscripten::class_<WireState>("WireState")
      .constructor<>()
      .function("generate", &WireState::generate)
      .function("resolveBranchPick", &WireState::resolve_branch_pick)
      .function("lastGenerationTiming", &WireState::last_generation_timing)
      .function("visualScene", &WireState::visual_scene)
      .function("configureModelAssemblies", &WireState::configure_model_assemblies)
      .function("poleCount", &WireState::pole_count)
      .function("pole", &WireState::pole)
      .function("portCount", &WireState::port_count)
      .function("port", &WireState::port)
      .function("spanCount", &WireState::span_count)
      .function("span", &WireState::span)
      .function("supportNodeCount", &WireState::support_node_count)
      .function("supportNode", &WireState::support_node)
      .function("backboneEdgeCount", &WireState::backbone_edge_count)
      .function("backboneEdge", &WireState::backbone_edge)
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
      .function("resetSpanReferenceLengths", &WireState::reset_span_reference_lengths)
      .function("saveState", &WireState::save_state)
      .function("loadState", &WireState::load_state)
      .function("loadStateWithModels", &WireState::load_state_with_models);
}
