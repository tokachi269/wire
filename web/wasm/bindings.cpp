#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/model_descriptor.hpp"
#include "city/road/road.hpp"

namespace {

#ifndef WIRE_BUILD_COMMIT
#define WIRE_BUILD_COMMIT "unknown"
#endif

#ifndef WIRE_BUILD_VERSION
#define WIRE_BUILD_VERSION "unknown"
#endif

using emscripten::val;
using city::wire::BackboneBundleSpec;
using city::wire::BackboneInputSpec;
using city::wire::BackboneSpec;
using city::wire::BundleKind;
using city::wire::BundleTemplateId;
using city::wire::CoreState;
using city::wire::CoreView;
using city::wire::CommitFailureCategory;
using city::wire::GenerationTiming;
using city::wire::ObjectId;
using city::wire::PickHitKind;
using city::wire::PickResult;
using city::wire::PoleTypeId;
using city::wire::ResolveBranchPickOptions;
using city::wire::SpanLayer;
using city::wire::SupportKind;
using city::wire::Vec3d;

[[nodiscard]] std::string wire_build_commit() { return WIRE_BUILD_COMMIT; }
[[nodiscard]] std::string wire_build_version() { return WIRE_BUILD_VERSION; }

[[nodiscard]] val road_result_value(bool ok,
                                    const std::string& error,
                                    city::road::CommitFailureCategory failure_category = city::road::CommitFailureCategory::kNone,
                                    const std::string& reason_code = {}) {
  val result = val::object();
  result.set("ok", ok);
  result.set("error", error);
  result.set("failureCategory", static_cast<int>(failure_category));
  result.set("reasonCode", ok ? std::string{} :
      (reason_code.empty() ? city::road::DefaultReasonCode(failure_category) : reason_code));
  return result;
}

[[nodiscard]] std::string road_material_key(city::road::RenderStyleRef style) {
  if (style.domain == city::road::RenderStyleDomain::kSurface) {
    if (style.value == city::road::builtin_surface_styles::kAsphalt.value) return "asphalt";
    if (style.value == city::road::builtin_surface_styles::kSidewalk.value) return "sidewalk";
    if (style.value == city::road::builtin_surface_styles::kCurb.value) return "curb";
    if (style.value == city::road::builtin_surface_styles::kMedian.value) return "median";
  }
  if (style.domain == city::road::RenderStyleDomain::kMarking) {
    if (style.value == city::road::builtin_marking_styles::kWhiteSolid.value) return "road_marking";
    if (style.value == city::road::builtin_marking_styles::kCenterLine.value) return "road_marking_center";
    if (style.value == city::road::builtin_marking_styles::kStopLine.value) return "road_marking_stop";
    if (style.value == city::road::builtin_marking_styles::kCrosswalk.value) return "road_marking_crosswalk";
    if (style.value == city::road::builtin_marking_styles::kWhiteDashed.value) return "road_marking_dashed";
  }
  throw std::runtime_error("unknown road render style id");
}

[[nodiscard]] std::optional<city::road::MarkingStyleId> road_marking_style_id(const std::string& key) {
  if (key == "road_marking" || key == "white_solid" || key == "white") {
    return city::road::builtin_marking_styles::kWhiteSolid;
  }
  if (key == "road_marking_center" || key == "center_line") return city::road::builtin_marking_styles::kCenterLine;
  if (key == "road_marking_stop" || key == "stop_line") return city::road::builtin_marking_styles::kStopLine;
  if (key == "road_marking_crosswalk" || key == "crosswalk" || key == "zebra") {
    return city::road::builtin_marking_styles::kCrosswalk;
  }
  return std::nullopt;
}

[[nodiscard]] city::road::ApproachKey road_approach_key_value(const val& input) {
  const int role = input["endpointRole"].as<int>();
  return city::road::ApproachKey{
      input["nodeId"].as<city::road::RoadNodeId>(),
      input["segmentId"].as<city::road::RoadSegmentId>(),
      role == 1 ? city::road::EndpointRole::kEnd : city::road::EndpointRole::kStart,
  };
}

[[nodiscard]] city::road::LaneSide road_lane_side_value(const val& input) {
  return input["side"].as<std::string>() == "right" ? city::road::LaneSide::kRight
                                                    : city::road::LaneSide::kLeft;
}

[[nodiscard]] city::road::RoadSide road_side_value(const val& input) {
  return input["side"].as<std::string>() == "right"
             ? city::road::RoadSide::kRight
             : city::road::RoadSide::kLeft;
}

[[nodiscard]] city::road::EndpointRole road_endpoint_role(int value) {
  return value == 1 ? city::road::EndpointRole::kEnd
                    : city::road::EndpointRole::kStart;
}

[[nodiscard]] city::road::LaneTravelDirection road_lane_direction(int value) {
  return value == 1 ? city::road::LaneTravelDirection::kAgainstSegment
                    : city::road::LaneTravelDirection::kAlongSegment;
}

[[nodiscard]] city::road::LaneConnectionKind road_lane_connection_kind(int value) {
  if (value == 1) return city::road::LaneConnectionKind::kTransition;
  if (value == 2) return city::road::LaneConnectionKind::kMerge;
  if (value == 3) return city::road::LaneConnectionKind::kSplit;
  if (value == 4) return city::road::LaneConnectionKind::kJunctionMovement;
  return city::road::LaneConnectionKind::kContinuation;
}

[[nodiscard]] city::road::BoundaryContinuationKind
road_boundary_continuation_kind(int value) {
  if (value == 1) return city::road::BoundaryContinuationKind::kMerge;
  if (value == 2) return city::road::BoundaryContinuationKind::kSplit;
  return city::road::BoundaryContinuationKind::kContinuation;
}

[[nodiscard]] city::road::LaneEndpointKey road_lane_endpoint_value(const val& input) {
  return city::road::LaneEndpointKey{
      input["segmentId"].as<city::road::RoadSegmentId>(),
      input["laneId"].as<city::road::LaneId>(),
      road_endpoint_role(input["endpointRole"].as<int>())};
}

[[nodiscard]] city::road::BoundaryEndpointKey
road_boundary_endpoint_value(const val& input) {
  return city::road::BoundaryEndpointKey{
      input["segmentId"].as<city::road::RoadSegmentId>(),
      input["boundaryId"].as<city::road::BoundaryId>(),
      road_endpoint_role(input["endpointRole"].as<int>())};
}

[[nodiscard]] city::road::JunctionMarkingAction road_junction_marking_action(int value) {
  if (value == 1) return city::road::JunctionMarkingAction::kConnectToApproach;
  if (value == 2) return city::road::JunctionMarkingAction::kSuppress;
  return city::road::JunctionMarkingAction::kTerminateAtGate;
}

[[nodiscard]] city::road::JunctionMarkingEndpoint road_junction_marking_endpoint(const val& input) {
  city::road::JunctionMarkingEndpoint endpoint{};
  endpoint.approach = road_approach_key_value(input);
  endpoint.boundary_id = input["boundaryId"].as<std::uint64_t>();
  endpoint.role = static_cast<city::road::MarkingRole>(input["role"].as<int>());
  return endpoint;
}

[[nodiscard]] city::road::AutoMarkingKey road_junction_marking_key(const val& input) {
  const auto node_id = input["nodeId"].as<city::road::RoadNodeId>();
  const auto role = static_cast<city::road::MarkingRole>(input["role"].as<int>());
  return city::road::AutoMarkingKey{
      city::road::MarkingOwner{city::road::MarkingOwner::Kind::kJunction, 0, node_id, 0},
      role,
      std::nullopt,
      road_approach_key_value(input)};
}

[[nodiscard]] val road_mesh_value(const city::road::Mesh& mesh) {
  val vertices = val::array();
  for (const auto& vertex : mesh.vertices) {
    vertices.call<void>("push", vertex.x);
    vertices.call<void>("push", vertex.y);
    vertices.call<void>("push", vertex.z);
  }
  val indices = val::array();
  for (const auto index : mesh.indices) {
    indices.call<void>("push", index);
  }
  val result = val::object();
  result.set("ownerSegmentId",
             static_cast<double>(mesh.owner_segment_id));
  result.set("material", road_material_key(mesh.style));
  result.set("vertices", vertices);
  result.set("indices", indices);
  return result;
}

[[nodiscard]] val road_preview_meshes(const city::road::RoadState& state) {
  val meshes = val::array();
  const auto append = [&meshes](const auto& source) {
    for (const auto& mesh : source)
      meshes.call<void>("push", road_mesh_value(mesh));
  };
  append(state.derived().segment_meshes);
  append(state.derived().connection_meshes);
  append(state.derived().junction_meshes);
  append(state.derived().marking_meshes);
  return meshes;
}

[[nodiscard]] city::road::BezierSpan road_span_value(const val& input) {
  const city::road::Vec2d start{
      input["startX"].as<double>(),
      input["startY"].as<double>(),
  };
  const city::road::Vec2d end{
      input["endX"].as<double>(),
      input["endY"].as<double>(),
  };
  const std::string kind = input["kind"].as<std::string>();
  if (kind == "line") {
    return city::road::MakeLine(start, end);
  }
  if (kind == "bezier") {
    const city::road::Vec2d handle_a{
        input["handleAX"].as<double>(),
        input["handleAY"].as<double>(),
    };
    const city::road::Vec2d handle_b{
        input["handleBX"].as<double>(),
        input["handleBY"].as<double>(),
    };
    return city::road::MakeBezier(start, handle_a, handle_b, end);
  }
  return city::road::BezierSpan{};
}

[[nodiscard]] std::optional<city::road::SegmentShapeIntent> road_shape_intent(const val& input) {
  const val kind = input["kind"];
  if (kind.isUndefined()) return std::nullopt;
  const std::string value = kind.as<std::string>();
  if (value == "line") return city::road::SegmentShapeIntent::kStraight;
  if (value == "bezier") return city::road::SegmentShapeIntent::kCurve;
  return std::nullopt;
}

[[nodiscard]] city::road::Path road_path_value(const val& input) {
  const val spans = input["spans"];
  if (!spans.isUndefined()) {
    std::vector<city::road::BezierSpan> path_spans{};
    const unsigned length = spans["length"].as<unsigned>();
    path_spans.reserve(length);
    for (unsigned index = 0; index < length; ++index) {
      path_spans.push_back(road_span_value(spans[index]));
    }
    return city::road::MakePath(std::move(path_spans));
  }
  return city::road::MakePath({road_span_value(input)});
}

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

[[nodiscard]] val result_value(bool ok,
                               const std::string& error,
                               CommitFailureCategory failure_category = CommitFailureCategory::kNone,
                               const std::string& reason_code = {}) {
  val result = val::object();
  result.set("ok", ok);
  result.set("error", error);
  const CommitFailureCategory effective_kind =
      ok ? CommitFailureCategory::kNone
         : (failure_category == CommitFailureCategory::kNone ? city::wire::ClassifyCommitFailure(error) : failure_category);
  result.set("failureCategory", static_cast<int>(effective_kind));
  result.set("reasonCode", ok ? std::string{} :
      (reason_code.empty() ? city::wire::DefaultReasonCode(effective_kind) : reason_code));
  return result;
}

template <typename T> [[nodiscard]] T property(const val& object, const char* name) {
  return object[name].as<T>();
}

[[nodiscard]] BundleTemplateId bundle_template_id(int raw) {
  return raw <= 0 ? city::wire::kInvalidBundleTemplateId : static_cast<BundleTemplateId>(raw);
}

[[nodiscard]] city::wire::Transformd transform_value(const val& input) {
  city::wire::Transformd transform{};
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

[[nodiscard]] city::wire::EditResult<bool> apply_model_bootstrap(CoreState& state,
                                                                  const val& input) {
  city::wire::EditResult<bool> result{};
  CoreState trial = state;
  const val assemblies = input["assemblies"];
  const std::size_t assembly_count = assemblies["length"].as<std::size_t>();
  for (std::size_t assembly_index = 0; assembly_index < assembly_count; ++assembly_index) {
    const val assembly_input = assemblies[assembly_index];
    city::wire::ModelAssemblyTemplate assembly{};
    assembly.id = property<city::wire::ModelAssemblyTemplateId>(assembly_input, "id");
    assembly.version = property<std::uint64_t>(assembly_input, "version");
    const val parts = assembly_input["parts"];
    const std::size_t part_count = parts["length"].as<std::size_t>();
    assembly.parts.reserve(part_count);
    for (std::size_t part_index = 0; part_index < part_count; ++part_index) {
      const val part_input = parts[part_index];
      city::wire::ModelDescriptor descriptor{};
      descriptor.measurement.name = property<std::string>(part_input, "descriptorName");
      descriptor.measurement.version = property<std::uint64_t>(part_input, "descriptorVersion");
      const val sockets = part_input["sockets"];
      const std::size_t socket_count = sockets["length"].as<std::size_t>();
      descriptor.measurement.sockets.reserve(socket_count);
      for (std::size_t socket_index = 0; socket_index < socket_count; ++socket_index) {
        const val socket_input = sockets[socket_index];
        city::wire::ModelSocket socket{};
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
      const auto built = city::wire::build_model_assembly_part(
          descriptor, property<std::uint32_t>(part_input, "partId"),
          property<std::string>(part_input, "modelKey"),
          transform_value(part_input["localTransform"]),
          static_cast<city::wire::ModelFitMode>(property<int>(part_input, "fitMode")));
      if (!built.report.conflicts.empty()) {
        result.error = "model bootstrap: " + built.report.conflicts.front().message;
        return result;
      }
      assembly.parts.push_back(built.part);
    }
    const val wire_socket = assembly_input["wireSocket"];
    if (!wire_socket.isNull() && !wire_socket.isUndefined()) {
      assembly.wire_socket = city::wire::AssemblySocketRef{
          property<std::uint32_t>(wire_socket, "partId"),
          property<std::string>(wire_socket, "socketName"),
      };
    }
    const val endpoint_mount_socket = assembly_input["endpointMountSocket"];
    if (!endpoint_mount_socket.isNull() && !endpoint_mount_socket.isUndefined()) {
      assembly.endpoint_mount_socket = city::wire::AssemblySocketRef{
          property<std::uint32_t>(endpoint_mount_socket, "partId"),
          property<std::string>(endpoint_mount_socket, "socketName"),
      };
    }
    const auto existing = CoreView(trial).model_assembly_templates().find(assembly.id);
    if (existing != CoreView(trial).model_assembly_templates().end()) {
      if (!(existing->second == assembly)) {
        if (assembly.version <= existing->second.version) {
          result.error = "model bootstrap: existing assembly " + std::to_string(assembly.id) +
                         " differs without a newer adapter version";
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
        property<city::wire::ModelAssemblyTemplateId>(assignment, "assemblyId");
    pole_type.radius_base_m = property<double>(assignment, "radiusBaseM");
    pole_type.radius_top_m = property<double>(assignment, "radiusTopM");
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
        property<city::wire::ModelAssemblyTemplateId>(assignment, "rowAssemblyId");
    bundle_template.endpoint_fixture_assembly_id =
        property<city::wire::ModelAssemblyTemplateId>(assignment, "endpointAssemblyId");
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
               const val& counts, int direction_mode, double max_tilt_deg,
               const val& node_specs = val::undefined()) {
    const std::size_t count = bundle_template_ids["length"].as<std::size_t>();
    if (count == 0 || counts["length"].as<std::size_t>() != count) {
      return result_value(false, "bundle template ids and counts must be non-empty and aligned");
    }
    val placements = val::array();
    for (std::size_t index = 0; index < count; ++index) {
      const BundleTemplateId id = bundle_template_id(bundle_template_ids[index].as<int>());
      const auto template_it = CoreView(*state_).bundle_templates().find(id);
      if (template_it == CoreView(*state_).bundle_templates().end()) {
        return result_value(false, "bundle template is missing");
      }
      val placement = val::object();
      placement.set("bundleTemplateId", static_cast<int>(id));
      placement.set("count", counts[index]);
      placement.set("explicit", false);
      placement.set("height", 0.0);
      placement.set("offset", 0.0);
      placement.set("spacing", template_it->second.default_spacing_m);
      placements.set(index, placement);
    }
    return generate_placements(flat_points, placements, interval_m, pole_type_id,
                               direction_mode, max_tilt_deg, node_specs);
  }

  val generate_placements(const val& flat_points, const val& bundle_placements, double interval_m,
                          int pole_type_id, int direction_mode, double max_tilt_deg,
                          const val& node_specs = val::undefined()) {
    return run_placements(*state_, flat_points, bundle_placements, interval_m, pole_type_id,
                          direction_mode, max_tilt_deg, node_specs, false);
  }

  val preview_placements(const val& flat_points, const val& bundle_placements, double interval_m,
                         int pole_type_id, int direction_mode, double max_tilt_deg,
                         const val& node_specs = val::undefined()) {
    CoreState trial = *state_;
    return run_placements(trial, flat_points, bundle_placements, interval_m, pole_type_id,
                          direction_mode, max_tilt_deg, node_specs, true);
  }

  val resolve_branch_pick(const val& input, const val& selected_bundle_template_ids) {
    return resolve_branch_pick_on(*state_, input, selected_bundle_template_ids);
  }

  val preview_resolve_branch_pick(const val& input, const val& selected_bundle_template_ids) {
    CoreState trial = *state_;
    return resolve_branch_pick_on(trial, input, selected_bundle_template_ids);
  }

private:
  val run_placements(CoreState& target, const val& flat_points, const val& bundle_placements,
                     double interval_m, int pole_type_id, int direction_mode, double max_tilt_deg,
                     const val& node_specs, bool include_preview_scene) {
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
    spec.direction_mode = static_cast<city::wire::PathDirectionMode>(direction_mode);
    spec.pole_placement.enable_tilt = max_tilt_deg > 0.0;
    spec.pole_placement.max_tilt_deg = max_tilt_deg;

    const std::size_t bundle_count = bundle_placements["length"].as<std::size_t>();
    if (bundle_count == 0) {
      return result_value(false, "backbone unsupported: bundle placements must be non-empty");
    }
    for (std::size_t index = 0; index < bundle_count; ++index) {
      const val placement = bundle_placements[index];
      const BundleTemplateId id = bundle_template_id(placement["bundleTemplateId"].as<int>());
      if (id == city::wire::kInvalidBundleTemplateId) {
        return result_value(false, "bundle template id is invalid");
      }
      const auto template_it = CoreView(target).bundle_templates().find(id);
      if (template_it == CoreView(target).bundle_templates().end()) {
        return result_value(false, "bundle template is missing");
      }
      BackboneBundleSpec bundle{};
      bundle.bundle_template_id = id;
      const val placement_id = placement["id"];
      if (!placement_id.isUndefined() && !placement_id.isNull()) {
        bundle.placement_key = placement_id.as<std::uint64_t>();
      }
      bundle.layer = template_it->second.default_layer;
      bundle.count = placement["count"].as<int>();
      bundle.placement_explicit = placement["explicit"].as<bool>();
      bundle.height_m = placement["height"].as<double>();
      bundle.lateral_m = placement["offset"].as<double>();
      bundle.spacing_m = placement["spacing"].as<double>();
      const val generated_bundle_id = placement["generatedBundleId"];
      if (!generated_bundle_id.isUndefined() && !generated_bundle_id.isNull()) {
        const std::string value = generated_bundle_id.as<std::string>();
        if (!value.empty() && value != "0") {
          bundle.source_bundle_id = static_cast<ObjectId>(std::stoull(value));
        }
      }
      spec.bundles.push_back(bundle);
    }

    const auto generated = target.GenerateFromBackboneSpec(spec);
    val result = result_value(generated.ok, generated.error,
                              generated.effective_failure_category(), generated.reason_code);
    result.set("generatedPoleCount", generated.value.generated_pole_ids.size());
    result.set("generatedSpanCount", generated.value.generated_span_ids.size());
    val generated_bundle_ids = val::array();
    for (std::size_t index = 0; index < generated.value.bundle_ids.size(); ++index) {
      generated_bundle_ids.set(index, std::to_string(generated.value.bundle_ids[index]));
    }
    result.set("generatedBundleIds", generated_bundle_ids);
    val generated_pole_ids = val::array();
    for (std::size_t index = 0; index < generated.value.generated_pole_ids.size(); ++index) {
      generated_pole_ids.set(index, std::to_string(generated.value.generated_pole_ids[index]));
    }
    result.set("generatedPoleIds", generated_pole_ids);
    val generated_span_ids = val::array();
    for (std::size_t index = 0; index < generated.value.generated_span_ids.size(); ++index) {
      generated_span_ids.set(index, std::to_string(generated.value.generated_span_ids[index]));
    }
    result.set("generatedSpanIds", generated_span_ids);
    result.set("totalMs", generated.value.timing.total_ms);
    result.set("timing", generation_timing_value(generated.value.timing));
    if (include_preview_scene && generated.ok) {
      const val scene = visual_scene_for(target);
      result.set("parts", scene["parts"]);
      result.set("models", scene["models"]);
      result.set("samples", scene["samples"]);
      val poles = val::array();
      std::size_t pole_index = 0;
      for (ObjectId pole_id : generated.value.generated_pole_ids) {
        const auto* pole = CoreView(target).poles().find(pole_id);
        if (pole == nullptr) continue;
        poles.set(pole_index++, pole_value(*pole));
      }
      result.set("poles", poles);
    }
    return result;
  }

  val resolve_branch_pick_on(CoreState& target, const val& input, const val& selected_bundle_template_ids) {
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
      if (id != city::wire::kInvalidBundleTemplateId) {
        options.selected_bundle_template_ids.push_back(id);
      }
    }

    const auto resolved = target.ResolveBranchPick(pick, options);
    val output = result_value(resolved.ok, resolved.error,
                              resolved.effective_failure_category(), resolved.reason_code);
    output.set("positionX", resolved.value.position.x);
    output.set("positionY", resolved.value.position.y);
    output.set("positionZ", resolved.value.position.z);
    output.set("supportKind", static_cast<int>(resolved.value.support_kind));
    output.set("nodeId", std::to_string(resolved.value.resolved_node_id));
    return output;
  }

public:

  [[nodiscard]] val clear_pending_support_nodes() {
    const auto cleared = state_->ClearPendingSupportNodes();
    return result_value(cleared.ok, cleared.error,
                        cleared.effective_failure_category(), cleared.reason_code);
  }

  [[nodiscard]] val last_generation_timing() const {
    return generation_timing_value(CoreView(*state_).last_generation_timing());
  }



  val visual_scene() {
    return visual_scene_for(*state_);
  }

private:
  val visual_scene_for(const CoreState& source) {
    const auto& parts = source.visual_curve_parts().parts;
    const CoreView view(source);
    sample_buffer_.clear();
    std::size_t sample_value_count = 0;
    for (const auto& part : parts) {
      sample_value_count += part.samples.size() * 3;
    }
    for (const city::wire::Span& span : view.spans().items()) {
      const city::wire::SpanVisualCacheEntry* visual = view.find_span_visual_cache(span.id);
      if (visual == nullptr) continue;
      sample_value_count += visual->parts.size() * 6;
    }
    sample_buffer_.reserve(sample_value_count);
    val descriptors = val::array();
    std::size_t descriptor_index = 0;
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
      output.set("supplementalKind", static_cast<int>(part.supplemental_kind));
      output.set("wireRadius", part.wire_radius_m);
      output.set("colorRgba", part.color_rgba);
      output.set("sourceNodeId", std::to_string(part.source_node_id));
      output.set("sourceEdgeId", std::to_string(part.source_edge_id));
      output.set("sourceSpanId", std::to_string(part.source_span_id));
      output.set("sourceBundleId", std::to_string(part.source_bundle_id));
      output.set("bundleTemplateId", static_cast<int>(part.bundle_template_id));
      output.set("laneIndex", part.lane_index);
      output.set("runId", static_cast<double>(part.cable_run_id));
      descriptors.set(descriptor_index++, output);
    }
    val result = val::object();
    result.set("parts", descriptors);
    val models = val::array();
    const auto& model_instances = source.visual_model_instances().instances;
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

  static val pole_value(const city::wire::Pole& pole) {
    val output = val::object();
    output.set("id", std::to_string(pole.id));
    output.set("poleTypeId", static_cast<int>(pole.pole_type_id));
    output.set("height", pole.height_m);
    output.set("positionX", pole.world_transform.position.x);
    output.set("positionY", pole.world_transform.position.y);
    output.set("positionZ", pole.world_transform.position.z);
    output.set("rotationX", pole.world_transform.rotation_euler_deg.x);
    output.set("rotationY", pole.world_transform.rotation_euler_deg.y);
    output.set("rotationZ", pole.world_transform.rotation_euler_deg.z);
    output.set("scaleX", pole.world_transform.scale.x);
    output.set("scaleY", pole.world_transform.scale.y);
    output.set("scaleZ", pole.world_transform.scale.z);
    return output;
  }

public:

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
    return pole_value(*pole);
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

  [[nodiscard]] val span_layout(const std::string& span_id_text) const {
    const ObjectId span_id = span_id_text.empty() || span_id_text == "0"
                                 ? city::wire::kInvalidObjectId
                                 : static_cast<ObjectId>(std::stoull(span_id_text));
    const city::wire::SpanLayoutView layout = state_->span_layout(span_id);
    if (!layout.has_layout()) {
      return result_value(false, "span layout is missing");
    }
    auto endpoint_value = [](const city::wire::LayoutEndpoint& endpoint) {
      val output = val::object();
      output.set("portId", std::to_string(endpoint.port_id));
      output.set("supportZ", endpoint.support_world.z);
      output.set("endpointZ", endpoint.endpoint_world.z);
      output.set("defaultLowerRequired", endpoint.default_lower_required);
      output.set("lowerRequired", endpoint.lower_required);
      output.set("branchDownOffset", endpoint.branch_down_offset_m);
      return output;
    };
    val output = result_value(true, {});
    output.set("start", endpoint_value(layout.entry->start));
    output.set("end", endpoint_value(layout.entry->end));
    return output;
  }

  [[nodiscard]] std::size_t support_node_count() const {
    return state_->SavedBackboneResult().nodes.size();
  }

  [[nodiscard]] val support_node(std::size_t index) const {
    const std::vector<city::wire::SupportNode> nodes = state_->SavedBackboneResult().nodes;
    if (index >= nodes.size()) {
      throw std::out_of_range("support node index is out of range");
    }
    const auto& node = nodes[index];
    val output = val::object();
    output.set("id", std::to_string(node.node_id));
    output.set("kind", static_cast<int>(node.support_kind));
    output.set("poleId", std::to_string(node.pole_id));
    output.set("x", node.position.x);
    output.set("y", node.position.y);
    output.set("z", node.position.z);
    return output;
  }

  [[nodiscard]] std::size_t backbone_edge_count() const {
    return state_->SavedBackboneEdges().size();
  }

  [[nodiscard]] val backbone_edge(std::size_t index) const {
    const std::vector<city::wire::BackboneEdge> edges = state_->SavedBackboneEdges();
    if (index >= edges.size()) {
      throw std::out_of_range("backbone edge index is out of range");
    }
    const auto& edge = edges[index];
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
        state_->ClearPoleOrientationOverride(static_cast<city::wire::ObjectId>(std::stoull(pole_id)));
    return result_value(result.ok, result.error);
  }

  val clear_span_socket_override(const std::string& span_id, bool is_start_endpoint) {
    const auto result = state_->ClearSpanEndpointSocketOverride(
        static_cast<city::wire::ObjectId>(std::stoull(span_id)), is_start_endpoint);
    return result_value(result.ok, result.error);
  }

  val clear_span_branch_down_override(const std::string& span_id) {
    const auto result = state_->ClearSpanBranchDownOffsetOverride(
        static_cast<city::wire::ObjectId>(std::stoull(span_id)));
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
    output.set("category", static_cast<int>(bundle_template.category));
    output.set("name", bundle_template.name);
    output.set("defaultCount", bundle_template.default_count);
    output.set("defaultSpacing", bundle_template.default_spacing_m);
    output.set("fixedCount", bundle_template.count_rule == city::wire::BundleCountRuleKind::kFixed);
    output.set("fixedCountValue", bundle_template.fixed_count);
    output.set("minCount", bundle_template.min_count);
    output.set("maxCount", bundle_template.max_count);
    output.set("cableTemplateId", bundle_template.cable_template_id);
    output.set("relatedPoleTypeId", bundle_template.related_pole_type_id);
    output.set("defaultLayer", static_cast<int>(bundle_template.default_layer));
    output.set("allowMidairNode", bundle_template.allow_midair_node);
    output.set("allowMidairBranch", bundle_template.allow_midair_branch);
    output.set("enableBranchDownOffset", bundle_template.enable_branch_down_offset);
    output.set("branchEndpointOffset", bundle_template.branch_endpoint_offset_m);
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
    if (id == city::wire::kInvalidBundleTemplateId) {
      return result_value(false, "bundle template id is invalid");
    }
    const auto& templates = CoreView(*state_).bundle_templates();
    const auto it = templates.find(id);
    if (it == templates.end()) {
      return result_value(false, "bundle template is missing");
    }
    auto bundle_template = it->second;
    bundle_template.cable_template_id = property<city::wire::CableTemplateId>(input, "cableTemplateId");
    bundle_template.related_pole_type_id = property<PoleTypeId>(input, "relatedPoleTypeId");
    bundle_template.default_layer = static_cast<SpanLayer>(property<int>(input, "defaultLayer"));
    bundle_template.allow_midair_node = property<bool>(input, "allowMidairNode");
    bundle_template.allow_midair_branch = property<bool>(input, "allowMidairBranch");
    bundle_template.enable_branch_down_offset = property<bool>(input, "enableBranchDownOffset");
    bundle_template.branch_endpoint_offset_m = property<double>(input, "branchEndpointOffset");
    bundle_template.support_wire_pole_band_id = property<int>(input, "supportWirePoleBandId");
    bundle_template.row_fixture_assembly_id =
        property<city::wire::ModelAssemblyTemplateId>(input, "rowFixtureAssemblyId");
    bundle_template.endpoint_fixture_assembly_id =
        property<city::wire::ModelAssemblyTemplateId>(input, "endpointFixtureAssemblyId");
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
    const std::vector<city::wire::CablePopulationRule> existing_population_rules =
        bundle_template.population_rules;
    bundle_template.population_rules.clear();
    const val population_rules = input["populationRules"];
    const std::size_t population_rule_count = population_rules["length"].as<std::size_t>();
    bundle_template.population_rules.reserve(population_rule_count);
    for (std::size_t index = 0; index < population_rule_count; ++index) {
      const val item = population_rules[index];
      const city::wire::CableSectionRuleId rule_id =
          property<city::wire::CableSectionRuleId>(item, "ruleId");
      const auto existing_rule = std::ranges::find_if(
          existing_population_rules,
          [rule_id](const city::wire::CablePopulationRule& rule) { return rule.rule_id == rule_id; });
      city::wire::CablePopulationRule rule =
          existing_rule == existing_population_rules.end() ? city::wire::CablePopulationRule{} : *existing_rule;
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
    if (id == city::wire::kInvalidBundleTemplateId) {
      return result_value(false, "bundle template id is invalid");
    }
    const auto updated = state_->ApplyBundleRelatedPoleTypeToExistingPoles(id);
    return result_value(updated.ok, updated.error);
  }

  val resolve_default_bundle_placement(int bundle_template_id, int pole_type_id, int count) const {
    const auto resolved = state_->ResolveDefaultBundlePlacement(
        ::bundle_template_id(bundle_template_id), static_cast<PoleTypeId>(pole_type_id), count);
    val output = result_value(resolved.ok, resolved.error);
    output.set("height", resolved.ok ? resolved.value.height_m : 0.0);
    output.set("offset", resolved.ok ? resolved.value.lateral_m : 0.0);
    output.set("spacing", resolved.ok ? resolved.value.spacing_m : 0.0);
    return output;
  }

  val update_backbone_bundle_placement(const std::string& bundle_id, const val& placement) {
    const ObjectId id = static_cast<ObjectId>(std::stoull(bundle_id));
    const auto updated = state_->UpdateBackboneBundlePlacement(
        id,
        property<bool>(placement, "explicit"),
        property<double>(placement, "height"),
        property<double>(placement, "offset"),
        property<double>(placement, "spacing"));
    return result_value(updated.ok, updated.error);
  }

  [[nodiscard]] std::size_t cable_template_count() const {
    return CoreView(*state_).cable_templates().size();
  }

  [[nodiscard]] val cable_template(std::size_t index) const {
    const auto& templates = CoreView(*state_).cable_templates();
    std::vector<city::wire::CableTemplateId> ids{};
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
        cable_template.supplemental_paths, [](const city::wire::CableSupplementalPathTemplate& path) {
          return path.anchor_mode == city::wire::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
                 path.profile_kind == city::wire::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
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
    const city::wire::CableTemplateId id = property<city::wire::CableTemplateId>(input, "id");
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
        static_cast<city::wire::CableMaterialStyleKind>(property<int>(input, "materialStyle"));
    cable_template.color_rgba = property<std::uint32_t>(input, "colorRgba");
    cable_template.sag_factor = property<double>(input, "sagFactor");
    cable_template.slack_factor = property<double>(input, "slackFactor");
    cable_template.continuity_policy =
        static_cast<city::wire::CableContinuityPolicyHint>(property<int>(input, "continuityPolicy"));

    auto supplemental = std::ranges::find_if(
        cable_template.supplemental_paths, [](const city::wire::CableSupplementalPathTemplate& path) {
          return path.anchor_mode == city::wire::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
                 path.profile_kind == city::wire::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
        });
    if (property<bool>(input, "supplementalEnabled")) {
      if (supplemental == cable_template.supplemental_paths.end()) {
        city::wire::CableSupplementalPathTemplate path{};
        path.anchor_mode = city::wire::CableSupplementalPathTemplate::AnchorMode::kCurveOffset;
        path.profile_kind = city::wire::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
        path.interaction_mode = city::wire::AttachmentLineInteractionMode::kAddInternalPath;
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

    std::vector<city::wire::ObjectId> preferred{};
    const std::size_t preferred_count = preferred_span_ids["length"].as<std::size_t>();
    preferred.reserve(preferred_count);
    for (std::size_t preferred_index = 0; preferred_index < preferred_count; ++preferred_index) {
      preferred.push_back(
          static_cast<city::wire::ObjectId>(std::stoull(preferred_span_ids[preferred_index].as<std::string>())));
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
    const PoleTypeId pole_type_id = property<PoleTypeId>(input, "id");
    const auto existing = CoreView(*state_).pole_types().find(pole_type_id);
    if (existing == CoreView(*state_).pole_types().end()) {
      return result_value(false, "pole template is missing");
    }
    city::wire::PoleTypeDefinition pole_template = existing->second;
    pole_template.id = pole_type_id;
    pole_template.name = property<std::string>(input, "name");
    pole_template.description = property<std::string>(input, "description");
    pole_template.default_height_m = property<double>(input, "defaultHeight");
    pole_template.pole_visual_assembly_id =
        property<city::wire::ModelAssemblyTemplateId>(input, "poleVisualAssemblyId");
    const val bands = input["portBands"];
    const std::size_t band_count = bands["length"].as<std::size_t>();
    pole_template.port_bands.clear();
    pole_template.port_bands.reserve(band_count);
    for (std::size_t band_index = 0; band_index < band_count; ++band_index) {
      const val item = bands[band_index];
      city::wire::PortPlacementBand band{};
      band.band_id = property<int>(item, "bandId");
      band.category = static_cast<city::wire::ConnectionCategory>(property<int>(item, "category"));
      band.layer = property<int>(item, "layer");
      band.side = static_cast<city::wire::SlotSide>(property<int>(item, "side"));
      band.role = static_cast<city::wire::SlotRole>(property<int>(item, "role"));
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
          static_cast<city::wire::BandOverflowPolicy>(property<int>(item, "overflowPolicy"));
      band.enabled = property<bool>(item, "enabled");
      pole_template.port_bands.push_back(band);
    }
    const val slots = input["anchorSlots"];
    const std::size_t slot_count = slots["length"].as<std::size_t>();
    pole_template.anchor_slots.clear();
    pole_template.anchor_slots.reserve(slot_count);
    for (std::size_t slot_index = 0; slot_index < slot_count; ++slot_index) {
      const val item = slots[slot_index];
      city::wire::AnchorSlotTemplate slot{};
      slot.slot_id = property<int>(item, "slotId");
      slot.usage = static_cast<city::wire::AnchorSupportKind>(property<int>(item, "usage"));
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
    city::wire::GeometrySettings settings{};
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
    city::wire::LayoutSettings settings{};
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
    output.set("enableInsulators", settings.enable_insulators);
    output.set("insulatorRadius", settings.insulator_radius_m);
    output.set("insulatorLength", settings.insulator_length_m);
    return output;
  }

  val update_visual_settings(const val& input) {
    city::wire::VisualSettings settings{};
    settings.enable_insulators = property<bool>(input, "enableInsulators");
    settings.insulator_radius_m = property<double>(input, "insulatorRadius");
    settings.insulator_length_m = property<double>(input, "insulatorLength");
    const auto updated = state_->UpdateVisualSettings(settings, true);
    return result_value(updated.ok, updated.error);
  }

  val apply_pole_tilt(const val& pole_ids, double max_tilt_deg) {
    std::vector<city::wire::ObjectId> ids{};
    const std::size_t count = pole_ids["length"].as<std::size_t>();
    ids.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      ids.push_back(static_cast<city::wire::ObjectId>(std::stoull(pole_ids[index].as<std::string>())));
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
};

class RoadStateBinding {
public:
  RoadStateBinding() : state_(std::make_unique<city::road::RoadState>()) {}

  val add_lane(const val& input) {
    const auto result = state_->AddLane(
        city::road::AddLaneRequest{
            input["corridorId"].as<city::road::RoadCorridorId>(),
            road_lane_direction(input["direction"].as<int>()),
            road_side_value(input),
            city::road::SegmentPosition{
                input["startSegmentId"].as<city::road::RoadSegmentId>(),
                input["startT"].as<double>()},
            city::road::SegmentPosition{
                input["completeSegmentId"].as<city::road::RoadSegmentId>(),
                input["completeT"].as<double>()},
            input["continuationEndNodeId"].as<city::road::RoadNodeId>(),
            input["laneWidthM"].as<double>()});
    val output = road_result_value(result.ok, result.error, result.failure_category);
    if (result.ok) output.set("laneId", static_cast<double>(result.value));
    return output;
  }

  val add_connected_lane_segment(const val& input) {
    return connected_lane_segment(input, false);
  }

  val preview_connected_lane_segment(const val& input) const {
    return connected_lane_segment(input, true);
  }

  val add_segment(const val& input) {
    const auto path = road_path_value(input);
    if (path.spans.empty()) {
      return road_result_value(false, "unsupported road primitive", city::road::CommitFailureCategory::kNotImplemented);
    }
    const auto start_node_id = input["startNodeId"].as<std::uint64_t>();
    const auto start_segment_id = input["startSegmentId"].as<std::uint64_t>();
    const auto end_node_id = input["endNodeId"].isUndefined()
                                 ? city::road::RoadNodeId{0}
                                 : input["endNodeId"].as<city::road::RoadNodeId>();
    const auto end_segment_id = input["endSegmentId"].isUndefined()
                                    ? city::road::RoadSegmentId{0}
                                    : input["endSegmentId"].as<city::road::RoadSegmentId>();
    const auto extension_corridor_id =
        input["extensionCorridorId"].isUndefined()
            ? city::road::RoadCorridorId{0}
            : input["extensionCorridorId"].as<city::road::RoadCorridorId>();
    const double start_segment_distance_m = input["startSegmentDistanceM"].as<double>();
    const double end_segment_distance_m = input["endSegmentDistanceM"].isUndefined()
                                              ? 0.0
                                              : input["endSegmentDistanceM"].as<double>();
    const auto section_template_id = input["sectionTemplateId"].isUndefined()
                                         ? city::road::CrossSectionTemplateId{1}
                                         : input["sectionTemplateId"].as<city::road::CrossSectionTemplateId>();
    city::road::Result<city::road::RoadSegmentId> result{};
    if ((start_node_id != 0 || start_segment_id != 0 ||
         extension_corridor_id != 0) &&
        (end_node_id != 0 || end_segment_id != 0)) {
      result = city::road::Result<city::road::RoadSegmentId>::Fail(
          city::road::CommitFailureCategory::kNotImplemented,
          "connecting both road endpoints in one interval is unsupported");
    } else if (end_segment_id != 0) {
      result = state_->AddSegmentConnectedToSegment(
          city::road::AddSegmentConnectedToSegmentRequest{
              path, section_template_id, end_segment_id,
              end_segment_distance_m, city::road::EndpointRole::kEnd});
    } else if (end_node_id != 0) {
      result = state_->AddSegmentConnectedTo(
          city::road::AddSegmentConnectedToRequest{
              path, section_template_id, end_node_id,
              city::road::EndpointRole::kEnd});
    } else if (extension_corridor_id != 0) {
      result = state_->ExtendCorridorFromEnd(
          city::road::ExtendCorridorFromEndRequest{
              extension_corridor_id, start_node_id, path,
              section_template_id, road_shape_intent(input)});
    } else if (start_segment_id != 0) {
      result = state_->AddSegmentConnectedToSegment(
          city::road::AddSegmentConnectedToSegmentRequest{path, section_template_id, start_segment_id, start_segment_distance_m,
                                                          city::road::EndpointRole::kStart});
    } else if (start_node_id != 0) {
      result = state_->AddSegmentConnectedTo(
          city::road::AddSegmentConnectedToRequest{path, section_template_id, start_node_id,
                                                   city::road::EndpointRole::kStart});
    } else {
      result = state_->AddSegment(city::road::AddSegmentRequest{path, section_template_id, road_shape_intent(input)});
    }
    val output = road_result_value(result.ok, result.error, result.failure_category);
    if (result.ok) {
      output.set("segmentId", static_cast<double>(result.value));
      const auto segment = std::find_if(
          state_->graph().segments.begin(), state_->graph().segments.end(),
          [&result](const city::road::RoadSegment& item) {
            return item.id == result.value;
          });
      const city::road::RoadNodeId output_end_node_id =
          segment == state_->graph().segments.end() ? 0 : segment->node_b;
      output.set("endNodeId", static_cast<double>(output_end_node_id));
      const city::road::RoadCorridor* corridor =
          city::road::FindCorridorForSegment(state_->graph(), result.value);
      const bool connected_at_end = end_node_id != 0 || end_segment_id != 0;
      output.set("corridorId", static_cast<double>(
          connected_at_end || corridor == nullptr ? 0 : corridor->id));
    }
    return output;
  }

  // The guide the draw tool shows before committing. Core owns the curve rule so
  // the guide cannot disagree with the road the same points produce.
  val preview_interval(const val& input) const {
    const city::road::Vec2d start{input["startX"].as<double>(), input["startY"].as<double>()};
    const city::road::Vec2d end{input["endX"].as<double>(), input["endY"].as<double>()};
    const auto corridor_id = input["extensionCorridorId"].isUndefined()
                                 ? city::road::RoadCorridorId{0}
                                 : input["extensionCorridorId"].as<city::road::RoadCorridorId>();
    const auto endpoint_node_id = input["startNodeId"].isUndefined()
                                      ? city::road::RoadNodeId{0}
                                      : input["startNodeId"].as<city::road::RoadNodeId>();
    const std::optional<city::road::SegmentShapeIntent> intent = road_shape_intent(input);
    const city::road::Path path = city::road::PreviewDrawnInterval(
        state_->graph(), corridor_id, endpoint_node_id, start, end,
        intent.value_or(city::road::SegmentShapeIntent::kCurve));
    val output = val::object();
    const city::road::BezierSpan& span = path.spans.front();
    output.set("startX", span.p0.x);
    output.set("startY", span.p0.y);
    output.set("handleAX", span.p1.x);
    output.set("handleAY", span.p1.y);
    output.set("handleBX", span.p2.x);
    output.set("handleBY", span.p2.y);
    output.set("endX", span.p3.x);
    output.set("endY", span.p3.y);
    return output;
  }

  val preview_segment(const val& input) const {
    city::road::RoadState trial = *state_;
    const auto path = road_path_value(input);
    if (path.spans.empty()) {
      val result = road_result_value(false, "unsupported road primitive", city::road::CommitFailureCategory::kNotImplemented);
      result.set("meshes", val::array());
      return result;
    }
    const auto start_node_id = input["startNodeId"].as<std::uint64_t>();
    const auto start_segment_id = input["startSegmentId"].as<std::uint64_t>();
    const auto end_node_id = input["endNodeId"].isUndefined()
                                 ? city::road::RoadNodeId{0}
                                 : input["endNodeId"].as<city::road::RoadNodeId>();
    const auto end_segment_id = input["endSegmentId"].isUndefined()
                                    ? city::road::RoadSegmentId{0}
                                    : input["endSegmentId"].as<city::road::RoadSegmentId>();
    const auto extension_corridor_id =
        input["extensionCorridorId"].isUndefined()
            ? city::road::RoadCorridorId{0}
            : input["extensionCorridorId"].as<city::road::RoadCorridorId>();
    const double start_segment_distance_m = input["startSegmentDistanceM"].as<double>();
    const double end_segment_distance_m = input["endSegmentDistanceM"].isUndefined()
                                              ? 0.0
                                              : input["endSegmentDistanceM"].as<double>();
    const auto section_template_id = input["sectionTemplateId"].isUndefined()
                                         ? city::road::CrossSectionTemplateId{1}
                                         : input["sectionTemplateId"].as<city::road::CrossSectionTemplateId>();
    city::road::Result<city::road::RoadSegmentId> added{};
    if ((start_node_id != 0 || start_segment_id != 0 ||
         extension_corridor_id != 0) &&
        (end_node_id != 0 || end_segment_id != 0)) {
      added = city::road::Result<city::road::RoadSegmentId>::Fail(
          city::road::CommitFailureCategory::kNotImplemented,
          "connecting both road endpoints in one interval is unsupported");
    } else if (end_segment_id != 0) {
      added = trial.AddSegmentConnectedToSegment(
          city::road::AddSegmentConnectedToSegmentRequest{
              path, section_template_id, end_segment_id,
              end_segment_distance_m, city::road::EndpointRole::kEnd});
    } else if (end_node_id != 0) {
      added = trial.AddSegmentConnectedTo(
          city::road::AddSegmentConnectedToRequest{
              path, section_template_id, end_node_id,
              city::road::EndpointRole::kEnd});
    } else if (extension_corridor_id != 0) {
      added = trial.ExtendCorridorFromEnd(
          city::road::ExtendCorridorFromEndRequest{
              extension_corridor_id, start_node_id, path,
              section_template_id, road_shape_intent(input)});
    } else if (start_segment_id != 0) {
      added = trial.AddSegmentConnectedToSegment(
          city::road::AddSegmentConnectedToSegmentRequest{path, section_template_id, start_segment_id, start_segment_distance_m,
                                                          city::road::EndpointRole::kStart});
    } else if (start_node_id != 0) {
      added = trial.AddSegmentConnectedTo(
          city::road::AddSegmentConnectedToRequest{path, section_template_id, start_node_id,
                                                   city::road::EndpointRole::kStart});
    } else {
      added = trial.AddSegment(city::road::AddSegmentRequest{path, section_template_id, road_shape_intent(input)});
    }
    val result = road_result_value(added.ok, added.error, added.failure_category);
    val meshes = val::array();
    if (added.ok) {
      for (const auto& mesh : trial.derived().segment_meshes) {
        if (mesh.owner_segment_id == added.value) meshes.call<void>("push", road_mesh_value(mesh));
      }
    }
    result.set("meshes", meshes);
    return result;
  }

  val scene() const {
    const auto& graph = state_->graph();
    const auto& derived = state_->derived();
    val nodes = val::array();
    std::unordered_map<city::road::RoadNodeId,
                       std::pair<std::size_t, city::road::RoadCorridorId>>
        node_incidence{};
    for (const auto& segment : graph.segments) {
      for (const city::road::RoadNodeId node_id :
           std::array<city::road::RoadNodeId, 2>{segment.node_a,
                                                 segment.node_b}) {
        auto& incidence = node_incidence[node_id];
        ++incidence.first;
      }
    }
    for (const city::road::RoadCorridor& corridor : graph.corridors) {
      if (corridor.segments.empty()) continue;
      const city::road::DirectedSegmentRef& last =
          corridor.segments.back();
      const auto segment =
          std::find_if(graph.segments.begin(), graph.segments.end(),
                       [&last](const city::road::RoadSegment& candidate) {
                         return candidate.id == last.segment_id;
                       });
      if (segment == graph.segments.end()) continue;
      const city::road::RoadNodeId end =
          last.reversed ? segment->node_a : segment->node_b;
      node_incidence[end].second = corridor.id;
    }
    for (const auto& node : graph.nodes) {
      val item = val::object();
      item.set("id", static_cast<double>(node.id));
      item.set("x", node.position.x);
      item.set("y", node.position.y);
      const auto incidence = node_incidence.find(node.id);
      item.set("extensionCorridorId",
               static_cast<double>(incidence != node_incidence.end() &&
                                            incidence->second.first == 1
                                        ? incidence->second.second
                                        : city::road::RoadCorridorId{0}));
      nodes.call<void>("push", item);
    }
    val centerline_segments = val::array();
    for (const auto& segment : graph.segments) {
      const city::road::Path* alignment = city::road::FindCanonicalAlignment(derived, segment.id);
      if (alignment == nullptr) continue;
      const double total = city::road::PathLength(*alignment).value;
      const int piece_count = alignment->spans.size() == 1 &&
                                      city::road::IsLinearSpan(alignment->spans.front())
                                  ? 1
                                  : std::max(2, static_cast<int>(std::ceil(total / 2.0)));
      for (int piece = 0; piece < piece_count; ++piece) {
        const double start_distance = total * piece / piece_count;
        const double end_distance = total * (piece + 1) / piece_count;
        const city::road::Vec2d start = city::road::EvaluatePath(*alignment, start_distance).value;
        const city::road::Vec2d end = city::road::EvaluatePath(*alignment, end_distance).value;
        val item = val::object();
        item.set("id", static_cast<double>(segment.id));
        item.set("startX", start.x);
        item.set("startY", start.y);
        item.set("endX", end.x);
        item.set("endY", end.y);
        item.set("startSegmentDistanceM", start_distance);
        item.set("endSegmentDistanceM", end_distance);
        centerline_segments.call<void>("push", item);
      }
    }
    val lane_paths = val::array();
    for (const auto& lane : derived.segment_lane_paths) {
      val item = val::object();
      item.set("segmentId", static_cast<double>(lane.segment_id));
      item.set("laneId", static_cast<double>(lane.lane_id));
      item.set("direction", lane.direction == city::road::LaneTravelDirection::kAgainstSegment ? 1 : 0);
      item.set("startSectionTemplateId", static_cast<double>(lane.start_template_id));
      item.set("endSectionTemplateId", static_cast<double>(lane.end_template_id));
      item.set("startSegmentDistanceM", lane.start_segment_distance_m);
      item.set("endSegmentDistanceM", lane.end_segment_distance_m);
      const auto source_segment = std::find_if(
          graph.segments.begin(), graph.segments.end(),
          [&lane](const auto& segment) { return segment.id == lane.segment_id; });
      item.set("nodeAId", static_cast<double>(source_segment == graph.segments.end() ? 0 : source_segment->node_a));
      item.set("nodeBId", static_cast<double>(source_segment == graph.segments.end() ? 0 : source_segment->node_b));
      val points = val::array();
      for (const auto& point : lane.points) {
        points.call<void>("push", point.x);
        points.call<void>("push", point.y);
        points.call<void>("push", point.z);
      }
      item.set("points", points);
      lane_paths.call<void>("push", item);
    }
    val section_templates = val::array();
    for (const auto& section : graph.section_templates) {
      val item = val::object();
      item.set("id", static_cast<double>(section.id));
      item.set("name", section.id == 1 ? "JP 2 lane"
                           : section.id == 2 ? "JP 3 lane"
                           : section.id == 3 ? "JP 2 lane / no left sidewalk"
                           : section.id == 4 ? "JP 2 lane / median"
                           : section.id == 5 ? "JP 2 lane / shoulder"
                                             : "Custom section");
      double sidewalk_width = 0.0;
      double lane_width = 0.0;
      double median_width = 0.0;
      int sidewalk_count = 0;
      int lane_count = static_cast<int>(section.lane_bands.size());
      val strips = val::array();
      for (const auto& strip : section.strips) {
        val strip_item = val::object();
        strip_item.set("id", static_cast<double>(strip.id));
        strip_item.set(
            "function",
            strip.function == city::road::StripFunction::kSidewalk
                ? "sidewalk"
                : strip.function == city::road::StripFunction::kShoulder
                      ? "shoulder"
                      : strip.function == city::road::StripFunction::kMedian
                            ? "median"
                            : "carriageway");
        strip_item.set("widthM", strip.width_m);
        strips.call<void>("push", strip_item);
        if (strip.function == city::road::StripFunction::kSidewalk) {
          sidewalk_width += strip.width_m;
          ++sidewalk_count;
        }
        if (strip.function == city::road::StripFunction::kCarriageway) {
          lane_width += strip.width_m;
          if (section.lane_bands.empty()) ++lane_count;
        }
        if (strip.function == city::road::StripFunction::kMedian)
          median_width += strip.width_m;
      }
      item.set("strips", strips);
      item.set("sidewalkWidthM", sidewalk_count == 0 ? 0.0 : sidewalk_width / sidewalk_count);
      item.set("laneWidthM", lane_count == 0 ? 0.0 : lane_width / lane_count);
      item.set("medianWidthM", median_width);
      item.set("laneCount", lane_count);
      val lanes = val::array();
      for (const auto& lane : section.lane_bands) {
        val lane_item = val::object();
        lane_item.set("id", static_cast<double>(lane.id));
        lane_item.set("direction", lane.direction == city::road::LaneTravelDirection::kAgainstSegment ? 1 : 0);
        lanes.call<void>("push", lane_item);
      }
      val boundaries = val::array();
      for (const auto& boundary : section.boundaries) {
        val boundary_item = val::object();
        boundary_item.set("id", static_cast<double>(boundary.boundary_id));
        boundary_item.set("role", static_cast<int>(boundary.role));
        boundaries.call<void>("push", boundary_item);
      }
      item.set("lanes", lanes);
      item.set("boundaries", boundaries);
      item.set("hasCenterLine", std::any_of(section.boundaries.begin(), section.boundaries.end(), [](const auto& boundary) {
        return boundary.marking.enabled &&
               boundary.marking.role == city::road::MarkingRole::kCenterLine;
      }));
      item.set("hasOuterLines", std::any_of(section.boundaries.begin(), section.boundaries.end(), [](const auto& boundary) {
        return boundary.marking.enabled &&
               boundary.marking.role == city::road::MarkingRole::kCarriagewayEdge;
      }));
      section_templates.call<void>("push", item);
    }
    val editable_segments = val::array();
    for (const auto& segment : graph.segments) {
      const city::road::Path* alignment = city::road::FindCanonicalAlignment(derived, segment.id);
      if (alignment == nullptr || alignment->spans.size() != 1) continue;
      const auto& span = alignment->spans.front();
      const bool linear =
          segment.shape.intent == city::road::SegmentShapeIntent::kStraight;
      val item = val::object();
      item.set("id", static_cast<double>(segment.id));
      item.set("nodeAId", static_cast<double>(segment.node_a));
      item.set("nodeBId", static_cast<double>(segment.node_b));
      item.set("kind", linear ? "line" : "bezier");
      val points = val::array();
      const auto push_point = [&points](city::road::Vec2d point) {
        val value = val::object();
        value.set("x", point.x);
        value.set("y", point.y);
        points.call<void>("push", value);
      };
      push_point(span.p0);
      push_point(span.p1);
      push_point(span.p2);
      push_point(span.p3);
      item.set("points", points);
      editable_segments.call<void>("push", item);
    }
    val surface_meshes = val::array();
    for (const auto& mesh : derived.segment_meshes) {
      surface_meshes.call<void>("push", road_mesh_value(mesh));
    }
    for (const auto& mesh : derived.connection_meshes) {
      surface_meshes.call<void>("push", road_mesh_value(mesh));
    }
    for (const auto& mesh : derived.junction_meshes) {
      surface_meshes.call<void>("push", road_mesh_value(mesh));
    }
    val marking_meshes = val::array();
    for (const auto& mesh : derived.marking_meshes) {
      marking_meshes.call<void>("push", road_mesh_value(mesh));
    }
    val approaches = val::array();
    for (const auto& connection : derived.connections) {
      for (const auto& resolved : connection.approaches) {
        const auto override = std::find_if(
            graph.approach_geometry_overrides.begin(), graph.approach_geometry_overrides.end(),
            [&resolved](const city::road::ApproachGeometryOverride& item) {
              return item.key == resolved.key;
            });
        val item = val::object();
        item.set("nodeId", static_cast<double>(resolved.key.node_id));
        item.set("segmentId", static_cast<double>(resolved.key.segment_id));
        item.set("endpointRole", resolved.key.endpoint_role == city::road::EndpointRole::kEnd ? 1 : 0);
        item.set("kind", static_cast<int>(connection.kind));
        item.set("autoSetbackM", resolved.auto_setback_m);
        item.set("resolvedSetbackM", resolved.resolved_setback_m);
        item.set("manualSetback", override != graph.approach_geometry_overrides.end() &&
                                      override->setback_m.has_value);
        item.set("manualSetbackM", override != graph.approach_geometry_overrides.end() &&
                                       override->setback_m.has_value
                                     ? override->setback_m.value
                                     : 0.0);
        item.set("autoLateralShiftM", resolved.auto_lateral_shift_m);
        item.set("resolvedLateralShiftM", resolved.resolved_lateral_shift_m);
        item.set("manualLateralShift", override != graph.approach_geometry_overrides.end() &&
                                           override->lateral_shift_m.has_value);
        item.set("manualLateralShiftM", override != graph.approach_geometry_overrides.end() &&
                                            override->lateral_shift_m.has_value
                                          ? override->lateral_shift_m.value
                                          : 0.0);
        approaches.call<void>("push", item);
      }
    }
    // Junction marking candidates come from core so the viewer never infers a
    // target approach or boundary.
    val junctions = val::array();
    for (const auto& area : derived.connections) {
      if (area.kind != city::road::NodeConnectionKind::kJunction) continue;
      val junction = val::object();
      junction.set("nodeId", static_cast<double>(area.node_id));
      val gates = val::array();
      for (const auto& approach : area.approaches) {
        const city::road::ConnectionGate& gate = approach.gate;
        val gate_value = val::object();
        gate_value.set("nodeId", static_cast<double>(gate.approach.node_id));
        gate_value.set("segmentId", static_cast<double>(gate.approach.segment_id));
        gate_value.set("endpointRole",
                       gate.approach.endpoint_role == city::road::EndpointRole::kEnd ? 1 : 0);
        val boundaries = val::array();
        for (const auto& boundary : gate.boundaries) {
          if (!boundary.marking.enabled) continue;
          val item = val::object();
          item.set("boundaryId", static_cast<double>(boundary.boundary_id));
          item.set("role", static_cast<int>(boundary.marking.role));
          boundaries.call<void>("push", item);
        }
        gate_value.set("markingBoundaries", boundaries);
        gates.call<void>("push", gate_value);
      }
      junction.set("gates", gates);
      val overrides = val::array();
      for (const auto& override : graph.junction_marking_overrides) {
        if (override.node_id != area.node_id) continue;
        val item = val::object();
        item.set("overrideId", static_cast<double>(override.id));
        item.set("action", static_cast<int>(override.action));
        item.set("sourceSegmentId", static_cast<double>(override.source.approach.segment_id));
        item.set("sourceBoundaryId", static_cast<double>(override.source.boundary_id));
        item.set("sourceRole", static_cast<int>(override.source.role));
        item.set("hasTarget", override.target.has_value());
        if (override.target.has_value()) {
          item.set("targetSegmentId", static_cast<double>(override.target->approach.segment_id));
          item.set("targetBoundaryId", static_cast<double>(override.target->boundary_id));
          item.set("targetRole", static_cast<int>(override.target->role));
        }
        overrides.call<void>("push", item);
      }
      junction.set("markingOverrides", overrides);
      junctions.call<void>("push", junction);
    }

    val result = val::object();
    val corridors = val::array();
    for (const auto& corridor : graph.corridors) {
      val item = val::object();
      item.set("id", static_cast<double>(corridor.id));
      item.set("sectionTemplateId",
               static_cast<double>(corridor.section_template_id));
      double length_m = 0.0;
      val segment_refs = val::array();
      for (const auto& ref : corridor.segments) {
        val ref_item = val::object();
        ref_item.set("segmentId", static_cast<double>(ref.segment_id));
        ref_item.set("reversed", ref.reversed);
        const auto* segment =
            city::road::FindDerivedSegment(derived, ref.segment_id);
        ref_item.set("lengthM", segment == nullptr ? 0.0 : segment->length_m);
        segment_refs.call<void>("push", ref_item);
        if (segment != nullptr) length_m += segment->length_m;
      }
      item.set("lengthM", length_m);
      item.set("segments", segment_refs);
      corridors.call<void>("push", item);
    }
    result.set("junctions", junctions);
    result.set("segmentCount", graph.segments.size());
    result.set("sectionTemplateCount", graph.section_templates.size());
    result.set("transitionCount", graph.transitions.size());
    result.set("markingCount", graph.manual_lines.size() + graph.manual_areas.size());
    std::size_t gate_count = 0;
    std::size_t junction_count = 0;
    for (const auto& connection : derived.connections) {
      gate_count += connection.approaches.size();
      if (connection.kind == city::road::NodeConnectionKind::kJunction) ++junction_count;
    }
    result.set("connectionGateCount", gate_count);
    result.set("junctionCount", junction_count);
    result.set("corridorCount", graph.corridors.size());
    result.set("nodes", nodes);
    result.set("centerlineSegments", centerline_segments);
    result.set("lanePaths", lane_paths);
    result.set("corridors", corridors);
    result.set("sectionTemplates", section_templates);
    result.set("editableSegments", editable_segments);
    result.set("surfaceMeshes", surface_meshes);
    result.set("markingMeshes", marking_meshes);
    result.set("approaches", approaches);
    return result;
  }

private:
  val connected_lane_segment(const val& input, bool preview) const {
    city::road::AddConnectedLaneSegmentRequest request{};
    request.start_node = input["startNodeId"].as<city::road::RoadNodeId>();
    request.alignment = road_path_value(input);
    request.section_template =
        input["sectionTemplateId"].as<city::road::CrossSectionTemplateId>();
    const val lane_targets = input["laneConnections"];
    for (unsigned index = 0; index < lane_targets["length"].as<unsigned>(); ++index) {
      const val item = lane_targets[index];
      request.lane_connections.push_back(city::road::LaneTargetConnection{
          road_lane_endpoint_value(item["source"]),
          item["targetLaneId"].as<city::road::LaneId>(),
          road_lane_connection_kind(item["kind"].as<int>())});
    }
    const val boundary_targets = input["boundaryContinuations"];
    for (unsigned index = 0; index < boundary_targets["length"].as<unsigned>(); ++index) {
      const val item = boundary_targets[index];
      request.boundary_continuations.push_back(
          city::road::BoundaryTargetContinuation{
              road_boundary_endpoint_value(item["source"]),
              item["targetBoundaryId"].as<city::road::BoundaryId>(),
              road_boundary_continuation_kind(item["kind"].as<int>())});
    }
    const val lane_sources = input["sourceLaneConnections"];
    for (unsigned index = 0; index < lane_sources["length"].as<unsigned>(); ++index) {
      const val item = lane_sources[index];
      request.source_lane_connections.push_back(city::road::LaneSourceConnection{
          item["sourceLaneId"].as<city::road::LaneId>(),
          road_lane_endpoint_value(item["target"]),
          road_lane_connection_kind(item["kind"].as<int>())});
    }
    const val boundary_sources = input["sourceBoundaryContinuations"];
    for (unsigned index = 0; index < boundary_sources["length"].as<unsigned>(); ++index) {
      const val item = boundary_sources[index];
      request.source_boundary_continuations.push_back(
          city::road::BoundarySourceContinuation{
              item["sourceBoundaryId"].as<city::road::BoundaryId>(),
              road_boundary_endpoint_value(item["target"]),
              road_boundary_continuation_kind(item["kind"].as<int>())});
    }
    if (preview) {
      city::road::RoadState trial = *state_;
      const auto result = trial.AddConnectedLaneSegment(std::move(request));
      val output = road_result_value(result.ok, result.error, result.failure_category);
      output.set("meshes", result.ok ? road_preview_meshes(trial) : val::array());
      return output;
    }
    const auto result = state_->AddConnectedLaneSegment(std::move(request));
    val output = road_result_value(result.ok, result.error, result.failure_category);
    if (result.ok) output.set("segmentId", static_cast<double>(result.value));
    return output;
  }

public:

  val delete_segment(std::uint64_t segment_id) {
    const auto result = state_->DeleteSegment(
        city::road::DeleteSegmentRequest{segment_id});
    return road_result_value(result.ok, result.error, result.failure_category);
  }

  val split_segment_at_distance(const val& input) {
    const auto result = state_->SplitSegmentAtDistance(
        city::road::SplitSegmentAtDistanceRequest{
            input["segmentId"].as<city::road::RoadSegmentId>(),
            input["segmentDistanceM"].as<double>()});
    val output =
        road_result_value(result.ok, result.error, result.failure_category);
    if (result.ok) {
      output.set("segmentId", static_cast<double>(result.value));
    }
    return output;
  }

  val move_node(const val& input) {
    const auto result = state_->MoveNode(city::road::MoveNodeRequest{
        input["nodeId"].as<city::road::RoadNodeId>(),
        city::road::Vec2d{input["x"].as<double>(), input["y"].as<double>()}});
    return road_result_value(result.ok, result.error, result.failure_category);
  }

  val preview_move_node(const val& input) const {
    city::road::RoadState trial = *state_;
    const auto moved = trial.MoveNode(city::road::MoveNodeRequest{
        input["nodeId"].as<city::road::RoadNodeId>(),
        city::road::Vec2d{input["x"].as<double>(), input["y"].as<double>()}});
    val result = road_result_value(moved.ok, moved.error, moved.failure_category);
    val meshes = val::array();
    if (moved.ok) {
      for (const auto& mesh : trial.derived().segment_meshes)
        meshes.call<void>("push", road_mesh_value(mesh));
      for (const auto& mesh : trial.derived().connection_meshes)
        meshes.call<void>("push", road_mesh_value(mesh));
      for (const auto& mesh : trial.derived().junction_meshes)
        meshes.call<void>("push", road_mesh_value(mesh));
    }
    result.set("meshes", meshes);
    return result;
  }

  val edit_segment(std::uint64_t segment_id, const val& input) {
    const auto shape = city::road::SegmentShapeFromPath(road_path_value(input));
    if (!shape.ok) return road_result_value(false, shape.error, shape.failure_category);
    const auto result = state_->EditSegmentShape(city::road::EditSegmentShapeRequest{segment_id, shape.value});
    return road_result_value(result.ok, result.error, result.failure_category);
  }

  val preview_edit_segment(std::uint64_t segment_id, const val& input) const {
    city::road::RoadState trial = *state_;
    const auto shape = city::road::SegmentShapeFromPath(road_path_value(input));
    if (!shape.ok) return road_result_value(false, shape.error, shape.failure_category);
    const auto edited = trial.EditSegmentShape(city::road::EditSegmentShapeRequest{segment_id, shape.value});
    val result = road_result_value(edited.ok, edited.error, edited.failure_category);
    val meshes = val::array();
    if (edited.ok) {
      for (const auto& mesh : trial.derived().segment_meshes) {
        if (mesh.owner_segment_id == segment_id) meshes.call<void>("push", road_mesh_value(mesh));
      }
    }
    result.set("meshes", meshes);
    return result;
  }

  val update_section_template(const val& input) {
    const auto id = input["id"].as<city::road::CrossSectionTemplateId>();
    const auto it = std::find_if(state_->graph().section_templates.begin(), state_->graph().section_templates.end(),
                                 [id](const auto& section) { return section.id == id; });
    if (it == state_->graph().section_templates.end()) {
      return road_result_value(false, "section template does not exist", city::road::CommitFailureCategory::kInvalidInput);
    }
    city::road::CrossSectionTemplate section = *it;
    const double sidewalk_width = input["sidewalkWidthM"].as<double>();
    const double lane_width = input["laneWidthM"].as<double>();
    const double median_width = input["medianWidthM"].as<double>();
    for (auto& strip : section.strips) {
      const double previous_width = strip.width_m;
      if (strip.function == city::road::StripFunction::kSidewalk)
        strip.width_m = sidewalk_width;
      if (strip.function == city::road::StripFunction::kCarriageway)
        strip.width_m = lane_width;
      if (strip.function == city::road::StripFunction::kMedian)
        strip.width_m = median_width;
      if (previous_width > 0.0 && strip.width_m != previous_width) {
        for (auto& lane : section.lane_bands) {
          if (lane.surface_strip_id != strip.id) continue;
          const double scale = strip.width_m / previous_width;
          lane.lateral_start_m *= scale;
          lane.lateral_end_m *= scale;
        }
      }
    }
    const bool center_line = input["hasCenterLine"].as<bool>();
    const bool outer_lines = input["hasOuterLines"].as<bool>();
    for (auto& boundary : section.boundaries) {
      if (boundary.role == city::road::BoundaryRole::kLaneDivider) {
        boundary.marking = center_line
                               ? city::road::AutoMarkingPolicy{
                                     true, city::road::MarkingRole::kCenterLine,
                                     city::road::builtin_marking_styles::kCenterLine}
                               : city::road::AutoMarkingPolicy{};
      }
      if (boundary.role == city::road::BoundaryRole::kCurb) {
        boundary.marking = outer_lines
                               ? city::road::AutoMarkingPolicy{
                                     true,
                                     city::road::MarkingRole::kCarriagewayEdge,
                                     city::road::builtin_marking_styles::kWhiteSolid}
                               : city::road::AutoMarkingPolicy{};
      }
    }
    const auto result = state_->EditSectionTemplate(city::road::EditSectionTemplateRequest{std::move(section)});
    return road_result_value(result.ok, result.error, result.failure_category);
  }

  val undo_segment() {
    if (state_->graph().segments.empty()) {
      return road_result_value(true, {});
    }
    const auto result = state_->DeleteSegment(
        city::road::DeleteSegmentRequest{state_->graph().segments.back().id});
    return road_result_value(result.ok, result.error, result.failure_category);
  }

  val clear() {
    state_ = std::make_unique<city::road::RoadState>();
    return road_result_value(true, {});
  }

  std::string save_state() const {
    const auto result = state_->Save();
    if (!result.ok) throw std::runtime_error(result.error);
    return result.value;
  }

  val load_state(const std::string& text) {
    const auto loaded = city::road::RoadState::Load(text);
    if (!loaded.ok) {
      return road_result_value(false, loaded.error, loaded.failure_category);
    }
    *state_ = loaded.value;
    return road_result_value(true, {});
  }

private:
  std::unique_ptr<city::road::RoadState> state_;
};

} // namespace

EMSCRIPTEN_BINDINGS(wire_web_core) {
  emscripten::function("wireBuildCommit", &wire_build_commit);
  emscripten::function("wireBuildVersion", &wire_build_version);
  emscripten::class_<RoadStateBinding>("RoadState")
      .constructor<>()
      .function("addSegment", &RoadStateBinding::add_segment)
      .function("previewSegment", &RoadStateBinding::preview_segment)
      .function("previewInterval", &RoadStateBinding::preview_interval)
      .function("addLane", &RoadStateBinding::add_lane)
      .function("addConnectedLaneSegment", &RoadStateBinding::add_connected_lane_segment)
      .function("previewConnectedLaneSegment", &RoadStateBinding::preview_connected_lane_segment)
      .function("scene", &RoadStateBinding::scene)
      .function("deleteSegment", &RoadStateBinding::delete_segment)
      .function("splitSegmentAtDistance",
                &RoadStateBinding::split_segment_at_distance)
      .function("moveNode", &RoadStateBinding::move_node)
      .function("previewMoveNode", &RoadStateBinding::preview_move_node)
      .function("editSegment", &RoadStateBinding::edit_segment)
      .function("previewEditSegment", &RoadStateBinding::preview_edit_segment)
      .function("updateSectionTemplate", &RoadStateBinding::update_section_template)
      .function("undoSegment", &RoadStateBinding::undo_segment)
      .function("clear", &RoadStateBinding::clear)
      .function("saveState", &RoadStateBinding::save_state)
      .function("loadState", &RoadStateBinding::load_state);

  emscripten::class_<WireState>("WireState")
      .constructor<>()
      .function("generate", &WireState::generate)
      .function("generatePlacements", &WireState::generate_placements)
      .function("previewPlacements", &WireState::preview_placements)
      .function("resolveBranchPick", &WireState::resolve_branch_pick)
      .function("previewResolveBranchPick", &WireState::preview_resolve_branch_pick)
      .function("clearPendingSupportNodes", &WireState::clear_pending_support_nodes)
      .function("lastGenerationTiming", &WireState::last_generation_timing)
      .function("visualScene", &WireState::visual_scene)
      .function("configureModelAssemblies", &WireState::configure_model_assemblies)
      .function("poleCount", &WireState::pole_count)
      .function("pole", &WireState::pole)
      .function("portCount", &WireState::port_count)
      .function("port", &WireState::port)
      .function("spanCount", &WireState::span_count)
      .function("span", &WireState::span)
      .function("spanLayout", &WireState::span_layout)
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
      .function("updateBackboneBundlePlacement", &WireState::update_backbone_bundle_placement)
      .function("applyRelatedPoleType", &WireState::apply_related_pole_type)
      .function("resolveDefaultBundlePlacement", &WireState::resolve_default_bundle_placement)
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
