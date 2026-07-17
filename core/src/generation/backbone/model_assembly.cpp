#include "model_assembly.hpp"

#include "emit_shared.hpp"

#include "wire/core/coord_utils.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace wire::core::generation::backbone {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Matrix3 {
  double m[3][3]{};
};

struct PortFixtureContext {
  const Pole* pole = nullptr;
  const ModelAssemblyTemplate* assembly = nullptr;
  const ModelAssemblyTemplate* row_assembly = nullptr;
  double layout_yaw_deg = 0.0;
};

struct RowFixtureContext {
  const Pole* pole = nullptr;
  SavedBackboneRowKey row_key{};
  ObjectId bundle_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  ModelAssemblyTemplateId assembly_id = kInvalidModelAssemblyTemplateId;
  double layout_yaw_deg = 0.0;
  struct Member {
    ObjectId port_id = kInvalidObjectId;
    std::size_t lane_index = 0;
    int placement_band_id = 0;
    double edge_lateral_offset_m = 0.0;
    bool placement_explicit = false;
    double bundle_lateral_m = 0.0;
    double phase_spacing_m = 0.0;
  };
  std::vector<Member> members{};
};

struct RowFixtureContexts {
  std::vector<RowFixtureContext> rows{};
  std::vector<ObjectId> endpoint_ports{};
};

Matrix3 matrix_from_euler(const Vec3d& euler_deg) {
  const Vec3d x = RotateEulerXYZDeg({1.0, 0.0, 0.0}, euler_deg);
  const Vec3d y = RotateEulerXYZDeg({0.0, 1.0, 0.0}, euler_deg);
  const Vec3d z = RotateEulerXYZDeg({0.0, 0.0, 1.0}, euler_deg);
  return {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
}

Matrix3 matrix_from_frame(const PoleFrame& frame) {
  return {{{frame.forward.x, frame.lateral.x, frame.up.x},
           {frame.forward.y, frame.lateral.y, frame.up.y},
           {frame.forward.z, frame.lateral.z, frame.up.z}}};
}

Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
  Matrix3 out{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int k = 0; k < 3; ++k) {
        out.m[row][column] += a.m[row][k] * b.m[k][column];
      }
    }
  }
  return out;
}

Vec3d multiply(const Matrix3& matrix, const Vec3d& value) {
  return {
      matrix.m[0][0] * value.x + matrix.m[0][1] * value.y + matrix.m[0][2] * value.z,
      matrix.m[1][0] * value.x + matrix.m[1][1] * value.y + matrix.m[1][2] * value.z,
      matrix.m[2][0] * value.x + matrix.m[2][1] * value.y + matrix.m[2][2] * value.z,
  };
}

Vec3d euler_from_matrix(const Matrix3& matrix) {
  const double y = std::asin(std::clamp(-matrix.m[2][0], -1.0, 1.0));
  const double cos_y = std::cos(y);
  double x = 0.0;
  double z = 0.0;
  if (std::abs(cos_y) > 1e-9) {
    x = std::atan2(matrix.m[2][1], matrix.m[2][2]);
    z = std::atan2(matrix.m[1][0], matrix.m[0][0]);
  } else {
    x = std::atan2(-matrix.m[1][2], matrix.m[1][1]);
  }
  constexpr double kDegrees = 180.0 / kPi;
  return {x * kDegrees, y * kDegrees, z * kDegrees};
}

Transformd transform_from_frame(const PoleFrame& frame, const Vec3d& position) {
  Transformd out{};
  out.position = position;
  out.rotation_euler_deg = euler_from_matrix(matrix_from_frame(frame));
  return out;
}

Transformd compose(const Transformd& parent, const Transformd& local) {
  const Matrix3 parent_rotation = matrix_from_euler(parent.rotation_euler_deg);
  const Matrix3 local_rotation = matrix_from_euler(local.rotation_euler_deg);
  const Vec3d scaled_local_position{
      local.position.x * parent.scale.x,
      local.position.y * parent.scale.y,
      local.position.z * parent.scale.z,
  };
  Transformd out{};
  out.position = parent.position + multiply(parent_rotation, scaled_local_position);
  out.rotation_euler_deg = euler_from_matrix(multiply(parent_rotation, local_rotation));
  out.scale = {
      parent.scale.x * local.scale.x,
      parent.scale.y * local.scale.y,
      parent.scale.z * local.scale.z,
  };
  return out;
}

Vec3d transform_point(const Transformd& transform, const Vec3d& local) {
  const Vec3d scaled{
      local.x * transform.scale.x,
      local.y * transform.scale.y,
      local.z * transform.scale.z,
  };
  return transform.position + multiply(matrix_from_euler(transform.rotation_euler_deg), scaled);
}

void hash_bytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ull;
  }
}

void hash_u64(std::uint64_t* hash, std::uint64_t value) {
  hash_bytes(hash, &value, sizeof(value));
}

void hash_double(std::uint64_t* hash, double value) {
  hash_u64(hash, std::bit_cast<std::uint64_t>(value));
}

std::uint64_t content_version(const ModelAssemblyTemplate& assembly,
                              const ModelAssemblyPart& part,
                              const Transformd& world_transform) {
  std::uint64_t hash = 1469598103934665603ull;
  hash_u64(&hash, assembly.id);
  hash_u64(&hash, assembly.version);
  hash_u64(&hash, part.part_id);
  hash_u64(&hash, part.descriptor_version);
  hash_bytes(&hash, part.model_key.data(), part.model_key.size());
  hash_double(&hash, world_transform.position.x);
  hash_double(&hash, world_transform.position.y);
  hash_double(&hash, world_transform.position.z);
  hash_double(&hash, world_transform.rotation_euler_deg.x);
  hash_double(&hash, world_transform.rotation_euler_deg.y);
  hash_double(&hash, world_transform.rotation_euler_deg.z);
  hash_double(&hash, world_transform.scale.x);
  hash_double(&hash, world_transform.scale.y);
  hash_double(&hash, world_transform.scale.z);
  return hash == 0 ? 1 : hash;
}

EditResult<PortFixtureContext> port_fixture_context(const CoreState& state, const Port& port) {
  EditResult<PortFixtureContext> out{};
  const CoreView view = state.view();
  const Pole* pole = view.poles().find(port.owner_pole_id);
  if (pole == nullptr) {
    out.ok = true;
    return out;
  }
  const std::vector<const SavedBackbonePortBinding*> bindings =
      view.backbone_port_bindings_for_port(port.id);
  if (bindings.empty()) {
    out.ok = true;
    return out;
  }

  std::optional<ModelAssemblyTemplateId> assembly_id{};
  std::optional<ModelAssemblyTemplateId> row_assembly_id{};
  double layout_yaw_deg = bindings.front()->layout_yaw_deg;
  for (const SavedBackbonePortBinding* binding : bindings) {
    const auto bundle_it = view.bundle_templates().find(binding->bundle_template_id);
    if (bundle_it == view.bundle_templates().end()) {
      out.error = "model assembly unsupported: Port binding bundle template is missing";
      return out;
    }
    const ModelAssemblyTemplateId candidate = bundle_it->second.endpoint_fixture_assembly_id;
    const ModelAssemblyTemplateId row_candidate = bundle_it->second.row_fixture_assembly_id;
    if (assembly_id.has_value() && candidate != *assembly_id) {
      out.error = "model assembly unsupported: shared Port resolves different endpoint assemblies";
      return out;
    }
    if (row_assembly_id.has_value() && row_candidate != *row_assembly_id) {
      out.error = "model assembly unsupported: shared Port resolves different row assemblies";
      return out;
    }
    if (std::abs(binding->layout_yaw_deg - layout_yaw_deg) > 1e-9) {
      out.error = "model assembly unsupported: shared Port resolves different row frames";
      return out;
    }
    assembly_id = candidate;
    row_assembly_id = row_candidate;
  }
  if (row_assembly_id.has_value() && *row_assembly_id != kInvalidModelAssemblyTemplateId) {
    const auto row_it = view.model_assembly_templates().find(*row_assembly_id);
    if (row_it == view.model_assembly_templates().end()) {
      out.error = "model assembly unsupported: row assembly is missing";
      return out;
    }
    out.value.row_assembly = &row_it->second;
  }
  if (assembly_id.has_value() && *assembly_id != kInvalidModelAssemblyTemplateId) {
    const auto assembly_it = view.model_assembly_templates().find(*assembly_id);
    if (assembly_it == view.model_assembly_templates().end() || !assembly_it->second.wire_socket.has_value()) {
      out.error = "model assembly unsupported: endpoint assembly or wire socket is missing";
      return out;
    }
    out.value.assembly = &assembly_it->second;
  }
  out.value.pole = pole;
  out.value.layout_yaw_deg = layout_yaw_deg;
  out.ok = true;
  return out;
}

EditResult<Transformd> resolve_model_part_world_transform(const CoreState& state,
                                                          const Pole& pole,
                                                          const PoleFrame& frame,
                                                          double placement_height_m,
                                                          const Transformd& owner_root,
                                                          const Vec3d& owner_offset_world,
                                                          const ModelAssemblyPart& part) {
  EditResult<Transformd> out{};
  Transformd fitted = part.local_transform;
  Transformd part_root = owner_root;
  if (part.fit_mode == ModelFitMode::kPoleHeight) {
    const auto pole_type_it = state.view().pole_types().find(pole.pole_type_id);
    if (pole_type_it == state.view().pole_types().end() || pole_type_it->second.default_height_m <= 1e-9) {
      out.error = "model assembly unsupported: pole height fit requires a valid PoleType default height";
      return out;
    }
    fitted.scale.z *= pole.height_m / pole_type_it->second.default_height_m;
  } else if (part.fit_mode == ModelFitMode::kPoleRadial) {
    const double radius = state.view().pole_radius_at_height_m(pole, placement_height_m);
    part_root.position = LocalPointToWorld(frame, {0.0, 0.0, placement_height_m});
    fitted.scale.x *= radius;
    fitted.scale.y *= radius;
  } else if (part.fit_mode == ModelFitMode::kPoleSurface) {
    part_root.position = owner_root.position + owner_offset_world +
                         ScaleVec(frame.forward, state.view().pole_radius_at_height_m(pole, placement_height_m));
  } else {
    part_root.position = part_root.position + owner_offset_world;
  }
  out.value = compose(part_root, fitted);
  out.ok = true;
  return out;
}

const ModelAssemblySocket* socket_for(const ModelAssemblyTemplate& assembly,
                                      const AssemblySocketRef& ref,
                                      const ModelAssemblyPart** part_out) {
  for (const ModelAssemblyPart& part : assembly.parts) {
    if (part.part_id != ref.part_id) {
      continue;
    }
    for (const ModelAssemblySocket& socket : part.sockets) {
      if (socket.name == ref.socket_name) {
        if (part_out != nullptr) {
          *part_out = &part;
        }
        return &socket;
      }
    }
  }
  return nullptr;
}

void append_instances(const CoreState& state, const Pole& pole, double placement_height_m,
                      const ModelAssemblyTemplate& assembly, const Transformd& root,
                      const Vec3d& rigid_offset_world,
                      const std::string& key_prefix, VisualModelInstanceCache* cache,
                      std::string* error) {
  for (const ModelAssemblyPart& part : assembly.parts) {
    const PoleFrame frame = BuildPoleFrame(pole.world_transform, root.rotation_euler_deg.z);
    const EditResult<Transformd> world =
        resolve_model_part_world_transform(state, pole, frame, placement_height_m, root, rigid_offset_world, part);
    if (!world.ok) {
      *error = world.error;
      return;
    }
    VisualModelInstance instance{};
    instance.stable_key = key_prefix + ":" + std::to_string(assembly.id) + ":" +
                          std::to_string(part.part_id);
    instance.model_key = part.model_key;
    instance.world_transform = world.value;
    instance.content_version = content_version(assembly, part, instance.world_transform);
    cache->instances.push_back(std::move(instance));
  }
}

EditResult<double> row_lateral_position(const CoreState& state,
                                        const RowFixtureContext& row) {
  EditResult<double> out{};
  const CoreView view = state.view();
  const auto pole_type_it = view.pole_types().find(row.pole->pole_type_id);
  if (pole_type_it == view.pole_types().end() || row.members.empty()) {
    out.error = "model assembly unsupported: row placement definition is missing";
    return out;
  }
  const bool placement_explicit = row.members.front().placement_explicit;
  const bool uses_lane_bands = !placement_explicit && std::any_of(
      row.members.begin() + 1, row.members.end(), [&](const RowFixtureContext::Member& member) {
        return member.placement_band_id != row.members.front().placement_band_id;
      });
  double sum = 0.0;
  for (const RowFixtureContext::Member& member : row.members) {
    const auto band_it = std::find_if(
        pole_type_it->second.port_bands.begin(), pole_type_it->second.port_bands.end(),
        [&](const PortPlacementBand& band) { return band.band_id == member.placement_band_id; });
    if (band_it == pole_type_it->second.port_bands.end()) {
      out.error = "model assembly unsupported: row placement band is missing";
      return out;
    }
    const double lane_offset = uses_lane_bands
                                   ? 0.0
                                   : LaneOffset(member.lane_index,
                                                static_cast<int>(row.members.size()),
                                                member.phase_spacing_m);
    const double placement_lateral = placement_explicit
                                         ? member.bundle_lateral_m
                                         : band_it->lateral_center_m;
    sum += placement_lateral + lane_offset + member.edge_lateral_offset_m;
  }
  out.value = sum / static_cast<double>(row.members.size());
  out.ok = true;
  return out;
}

std::string row_key_text(const SavedBackboneRowKey& row_key) {
  std::ostringstream out;
  out << row_key.node_id << ":" << (row_key.source_is_open ? 1 : 0) << ":"
      << row_key.source_edge_a << ":" << row_key.source_edge_b;
  return out.str();
}

EditResult<RowFixtureContexts> row_fixture_contexts(const CoreState& state) {
  EditResult<RowFixtureContexts> out{};
  const CoreView view = state.view();
  for (const SavedBackbonePortBinding& binding : view.backbone().port_bindings) {
    const Port* port = view.ports().find(binding.port_id);
    const Pole* pole = port == nullptr ? nullptr : view.poles().find(port->owner_pole_id);
    const auto bundle_it = view.bundle_templates().find(binding.bundle_template_id);
    if (port == nullptr || pole == nullptr || bundle_it == view.bundle_templates().end()) {
      continue;
    }
    if (std::find(out.value.endpoint_ports.begin(), out.value.endpoint_ports.end(), port->id) ==
        out.value.endpoint_ports.end()) {
      out.value.endpoint_ports.push_back(port->id);
    }
    if (bundle_it->second.row_fixture_assembly_id == kInvalidModelAssemblyTemplateId) {
      continue;
    }
    const SavedBackboneEdgeBundle* edge_bundle =
        view.backbone_edge_bundle(binding.edge_bundle_id);
    const SavedBackboneEdge* edge =
        edge_bundle == nullptr ? nullptr : view.backbone_edge(edge_bundle->edge_id);
    const Bundle* bundle =
        edge_bundle == nullptr ? nullptr : view.bundles().find(edge_bundle->bundle_id);
    if (edge_bundle == nullptr || edge == nullptr || bundle == nullptr) {
      out.error = "model assembly unsupported: row source bundle is missing";
      return out;
    }
    auto row_it = std::find_if(out.value.rows.begin(), out.value.rows.end(), [&](const RowFixtureContext& row) {
      return row.pole->id == pole->id && row.row_key == binding.row_key &&
             row.bundle_id == edge_bundle->bundle_id;
    });
    if (row_it == out.value.rows.end()) {
      out.value.rows.push_back({pole, binding.row_key, edge_bundle->bundle_id, binding.bundle_template_id,
                                bundle_it->second.row_fixture_assembly_id, binding.layout_yaw_deg,
                                {{port->id, binding.lane_index, binding.placement_band_id,
                                  edge->lateral_offset_m, bundle->placement_explicit,
                                  bundle->lateral_m,
                                  bundle->phase_spacing_m}}});
    } else {
      if (row_it->assembly_id != bundle_it->second.row_fixture_assembly_id ||
          std::abs(row_it->layout_yaw_deg - binding.layout_yaw_deg) > 1e-9) {
        out.error = "model assembly unsupported: saved row resolves inconsistent fixture data";
        return out;
      }
      const auto member_it = std::find_if(
          row_it->members.begin(), row_it->members.end(),
          [&](const RowFixtureContext::Member& member) { return member.port_id == port->id; });
      if (member_it == row_it->members.end()) {
        row_it->members.push_back({port->id, binding.lane_index, binding.placement_band_id,
                                   edge->lateral_offset_m, bundle->placement_explicit,
                                   bundle->lateral_m,
                                   bundle->phase_spacing_m});
      }
    }
  }
  std::sort(out.value.endpoint_ports.begin(), out.value.endpoint_ports.end());
  out.ok = true;
  return out;
}

EditResult<RowFixturePlacementPlan> row_fixture_placement_plan(
    const CoreState& state, const RowFixtureContext& row,
    const FixturePlacementPlanByPort& fixture_plan) {
  EditResult<RowFixturePlacementPlan> out{};
  const CoreView view = state.view();
  if (row.members.empty()) {
    out.error = "model assembly unsupported: row fixture has no members";
    return out;
  }
  const PoleFrame frame = BuildPoleFrame(row.pole->world_transform, row.layout_yaw_deg);
  double height_m = 0.0;
  for (const RowFixtureContext::Member& member : row.members) {
    const Port* port = view.ports().find(member.port_id);
    if (port == nullptr) {
      out.error = "model assembly unsupported: row fixture port is missing";
      return out;
    }
    const auto plan_it = fixture_plan.find(member.port_id);
    const double member_down_offset_m =
        plan_it == fixture_plan.end() ? 0.0 : plan_it->second.down_offset_m;
    height_m += WorldPointToLocal(frame, port->world_position).z - member_down_offset_m;
  }
  height_m /= static_cast<double>(row.members.size());
  const EditResult<double> lateral = row_lateral_position(state, row);
  if (!lateral.ok) {
    out.error = lateral.error;
    return out;
  }
  out.value.available = true;
  out.value.placement_height_m = height_m;
  out.value.root = transform_from_frame(frame, LocalPointToWorld(frame, {0.0, 0.0, height_m}));
  out.value.rigid_offset_world = ScaleVec(frame.lateral, lateral.value);
  out.ok = true;
  return out;
}

} // namespace

namespace {

EditResult<ResolvedEndpointPlacement> resolve_endpoint_placement(
    const CoreState& state, const Port& port, double down_offset_m,
    const RowFixturePlacementPlan* row_fixture) {
  EditResult<ResolvedEndpointPlacement> out{};
  out.value.fixture_root.position = port.world_position;
  out.value.wire_endpoint = port.world_position;
  const EditResult<PortFixtureContext> context = port_fixture_context(state, port);
  if (!context.ok) {
    out.error = context.error;
    return out;
  }
  if (context.value.pole == nullptr) {
    out.ok = true;
    return out;
  }

  const PoleFrame frame = BuildPoleFrame(context.value.pole->world_transform,
                                         context.value.layout_yaw_deg);
  const double resolved_down_offset_m = std::max(0.0, down_offset_m);
  const Vec3d final_anchor = port.world_position - ScaleVec(frame.up, resolved_down_offset_m);
  const double placement_height_m = WorldPointToLocal(frame, final_anchor).z;
  Transformd fixture_root = transform_from_frame(frame, final_anchor);

  if (context.value.row_assembly != nullptr &&
      context.value.row_assembly->endpoint_mount_socket.has_value()) {
    const ModelAssemblyPart* mount_part = nullptr;
    const ModelAssemblySocket* mount = socket_for(
        *context.value.row_assembly, *context.value.row_assembly->endpoint_mount_socket,
        &mount_part);
    if (mount == nullptr || mount_part == nullptr) {
      out.error = "model assembly unsupported: row endpoint mount socket does not resolve";
      return out;
    }
    const Transformd& row_root =
        row_fixture != nullptr && row_fixture->available ? row_fixture->root : fixture_root;
    const double row_height =
        row_fixture != nullptr && row_fixture->available ? row_fixture->placement_height_m : placement_height_m;
    const Vec3d row_offset =
        row_fixture != nullptr && row_fixture->available ? row_fixture->rigid_offset_world : Vec3d{};
    const EditResult<Transformd> mount_world = resolve_model_part_world_transform(
        state, *context.value.pole, frame, row_height, row_root, row_offset, *mount_part);
    if (!mount_world.ok) {
      out.error = mount_world.error;
      return out;
    }
    fixture_root.position = fixture_root.position +
        (transform_point(mount_world.value, mount->local_position) - final_anchor);
  }
  out.value.fixture_root = fixture_root;
  out.value.wire_endpoint = fixture_root.position;

  if (context.value.assembly != nullptr) {
    const ModelAssemblyPart* socket_part = nullptr;
    const ModelAssemblySocket* socket = socket_for(
        *context.value.assembly, *context.value.assembly->wire_socket, &socket_part);
    if (socket == nullptr || socket_part == nullptr) {
      out.error = "model assembly unsupported: endpoint wire socket does not resolve";
      return out;
    }
    const EditResult<Transformd> world = resolve_model_part_world_transform(
        state, *context.value.pole, frame, placement_height_m, fixture_root, {}, *socket_part);
    if (!world.ok) {
      out.error = world.error;
      return out;
    }
    out.value.wire_endpoint = transform_point(world.value, socket->local_position);
  }
  out.ok = true;
  return out;
}

} // namespace

EditResult<ResolvedEndpointPlacement> resolve_endpoint_placement(const CoreState& state,
                                                                  const Port& port,
                                                                  double down_offset_m) {
  EditResult<FixturePlacementPlanByPort> plan = fixture_placement_plan_from_cache(state);
  if (plan.ok) {
    const auto plan_it = plan.value.find(port.id);
    if (plan_it != plan.value.end() &&
        std::abs(plan_it->second.down_offset_m - std::max(0.0, down_offset_m)) <= 1e-9) {
      EditResult<ResolvedEndpointPlacement> out{};
      out.value.fixture_root = plan_it->second.endpoint_fixture_root;
      out.value.wire_endpoint = plan_it->second.wire_endpoint;
      out.ok = true;
      return out;
    }
  }
  return resolve_endpoint_placement(state, port, down_offset_m, nullptr);
}

EditResult<Vec3d> resolve_model_assembly_wire_socket(const CoreState& state, const Port& port,
                                                     double down_offset_m) {
  EditResult<Vec3d> out{};
  const EditResult<ResolvedEndpointPlacement> placement = resolve_endpoint_placement(state, port, down_offset_m);
  out.ok = placement.ok;
  out.error = placement.error;
  out.value = placement.value.wire_endpoint;
  return out;
}

double endpoint_down_offset(const EndpointLayoutRule& endpoint) {
  if (!endpoint.default_lower_required && !endpoint.semantic.lower_required) {
    return 0.0;
  }
  if (endpoint.branch_down_offset_m > 0.0) {
    return endpoint.branch_down_offset_m;
  }
  if (endpoint.endpoint_offset_z_m < 0.0) {
    return -endpoint.endpoint_offset_z_m;
  }
  if (endpoint.automatic_branch_down_offset_m > 0.0) {
    return endpoint.automatic_branch_down_offset_m;
  }
  return std::max(0.0, -endpoint.automatic_endpoint_offset_z_m);
}

namespace {

void append_fixture_placement_plan(EditResult<FixturePlacementPlanByPort>* out,
                                   const CoreState& state,
                                   const EndpointLayoutRule& endpoint) {
  if (out == nullptr || !out->ok || endpoint.port_id == kInvalidObjectId) {
    return;
  }
  static_cast<void>(state);
  const double down_offset = endpoint_down_offset(endpoint);
  FixturePlacementPlan plan{};
  plan.down_offset_m = down_offset;
  const auto [it, inserted] = out->value.emplace(endpoint.port_id, plan);
  if (!inserted && std::abs(it->second.down_offset_m - plan.down_offset_m) > 1e-9) {
    out->ok = false;
    out->error = "model assembly unsupported: shared port resolves multiple lowered fixture heights";
  }
}

void attach_row_fixture_placement_plans(EditResult<FixturePlacementPlanByPort>* out,
                                        const CoreState& state) {
  if (out == nullptr || !out->ok) {
    return;
  }
  EditResult<RowFixtureContexts> contexts = row_fixture_contexts(state);
  if (!contexts.ok) {
    out->ok = false;
    out->error = contexts.error;
    return;
  }
  for (const RowFixtureContext& row : contexts.value.rows) {
    EditResult<RowFixturePlacementPlan> row_plan =
        row_fixture_placement_plan(state, row, out->value);
    if (!row_plan.ok) {
      out->ok = false;
      out->error = row_plan.error;
      return;
    }
    for (const RowFixtureContext::Member& member : row.members) {
      const auto plan_it = out->value.find(member.port_id);
      if (plan_it != out->value.end()) {
        plan_it->second.row_fixture = row_plan.value;
      }
    }
  }
}

void resolve_endpoint_fixture_placements(EditResult<FixturePlacementPlanByPort>* out,
                                         const CoreState& state) {
  if (out == nullptr || !out->ok) {
    return;
  }
  const CoreView view = state.view();
  for (auto& [port_id, plan] : out->value) {
    const Port* port = view.ports().find(port_id);
    if (port == nullptr) {
      out->ok = false;
      out->error = "model assembly unsupported: endpoint fixture plan port is missing";
      return;
    }
    const RowFixturePlacementPlan* row_fixture =
        plan.row_fixture.available ? &plan.row_fixture : nullptr;
    EditResult<ResolvedEndpointPlacement> placement =
        resolve_endpoint_placement(state, *port, plan.down_offset_m, row_fixture);
    if (!placement.ok) {
      out->ok = false;
      out->error = placement.error;
      return;
    }
    plan.endpoint_fixture_root = placement.value.fixture_root;
    plan.wire_endpoint = placement.value.wire_endpoint;
  }
}

} // namespace

EditResult<FixturePlacementPlanByPort> fixture_placement_plan_from_rules(
    const CoreState& state, const std::vector<SpanLayoutRule>& rules) {
  EditResult<FixturePlacementPlanByPort> out{};
  out.ok = true;
  for (const SpanLayoutRule& rule : rules) {
    append_fixture_placement_plan(&out, state, rule.start);
    append_fixture_placement_plan(&out, state, rule.end);
  }
  attach_row_fixture_placement_plans(&out, state);
  resolve_endpoint_fixture_placements(&out, state);
  return out;
}

EditResult<FixturePlacementPlanByPort> fixture_placement_plan_from_cache(const CoreState& state) {
  EditResult<FixturePlacementPlanByPort> out{};
  out.ok = true;
  state.view().cache_state().span_layout_cache.for_each_layout_record(
      [&](ObjectId, const SpanLayoutCacheRecord& record, const SpanLayoutEntry&) {
        const SpanLayoutRule* rule = record.span_layout_rule();
        if (rule == nullptr) {
          return;
        }
        append_fixture_placement_plan(&out, state, rule->start);
        append_fixture_placement_plan(&out, state, rule->end);
      });
  attach_row_fixture_placement_plans(&out, state);
  resolve_endpoint_fixture_placements(&out, state);
  return out;
}

EditResult<VisualModelInstanceCache> materialize_model_assemblies(const CoreState& state,
                                                                  const FixturePlacementPlanByPort* fixture_plan) {
  EditResult<VisualModelInstanceCache> out{};
  FixturePlacementPlanByPort cached_fixture_plan{};
  const FixturePlacementPlanByPort* effective_fixture_plan = fixture_plan;
  if (effective_fixture_plan == nullptr) {
    EditResult<FixturePlacementPlanByPort> from_cache = fixture_placement_plan_from_cache(state);
    if (!from_cache.ok) {
      out.error = from_cache.error;
      return out;
    }
    cached_fixture_plan = std::move(from_cache.value);
    effective_fixture_plan = &cached_fixture_plan;
  }
  const CoreView view = state.view();

  for (const Pole& pole : view.poles().items()) {
    const auto pole_type_it = view.pole_types().find(pole.pole_type_id);
    if (pole_type_it == view.pole_types().end() ||
        pole_type_it->second.pole_visual_assembly_id == kInvalidModelAssemblyTemplateId) {
      continue;
    }
    const auto assembly_it = view.model_assembly_templates().find(
        pole_type_it->second.pole_visual_assembly_id);
    if (assembly_it == view.model_assembly_templates().end()) {
      out.error = "model assembly unsupported: pole visual assembly is missing";
      return out;
    }
    std::string error{};
    append_instances(state, pole, 0.0, assembly_it->second, pole.world_transform, {},
                     "pole:" + std::to_string(pole.id), &out.value, &error);
    if (!error.empty()) {
      out.error = std::move(error);
      return out;
    }
  }

  EditResult<RowFixtureContexts> contexts = row_fixture_contexts(state);
  if (!contexts.ok) {
    out.error = contexts.error;
    return out;
  }

  for (const RowFixtureContext& row : contexts.value.rows) {
    const auto assembly_it = view.model_assembly_templates().find(row.assembly_id);
    if (assembly_it == view.model_assembly_templates().end() || row.members.empty()) {
      out.error = "model assembly unsupported: row fixture assembly is missing";
      return out;
    }
    const RowFixturePlacementPlan* row_plan = nullptr;
    if (effective_fixture_plan != nullptr) {
      for (const RowFixtureContext::Member& member : row.members) {
        const auto plan_it = effective_fixture_plan->find(member.port_id);
        if (plan_it != effective_fixture_plan->end() && plan_it->second.row_fixture.available) {
          row_plan = &plan_it->second.row_fixture;
          break;
        }
      }
    }
    RowFixturePlacementPlan fallback_plan{};
    if (row_plan == nullptr) {
      FixturePlacementPlanByPort empty_plan{};
      EditResult<RowFixturePlacementPlan> resolved = row_fixture_placement_plan(state, row, empty_plan);
      if (!resolved.ok) {
        out.error = resolved.error;
        return out;
      }
      fallback_plan = resolved.value;
      row_plan = &fallback_plan;
    }
    std::string error{};
    append_instances(
        state, *row.pole, row_plan->placement_height_m, assembly_it->second, row_plan->root,
        row_plan->rigid_offset_world,
        "row:" + std::to_string(row.pole->id) + ":" + row_key_text(row.row_key) + ":" +
            std::to_string(row.bundle_id),
        &out.value, &error);
    if (!error.empty()) {
      out.error = std::move(error);
      return out;
    }
  }

  for (ObjectId port_id : contexts.value.endpoint_ports) {
    const Port* port = view.ports().find(port_id);
    if (port == nullptr) {
      continue;
    }
    const EditResult<PortFixtureContext> context = port_fixture_context(state, *port);
    if (!context.ok) {
      out.error = context.error;
      return out;
    }
    if (context.value.assembly == nullptr || context.value.pole == nullptr) {
      continue;
    }
    const PoleFrame frame = BuildPoleFrame(context.value.pole->world_transform,
                                           context.value.layout_yaw_deg);
    const double height_m = WorldPointToLocal(frame, port->world_position).z;
    ResolvedEndpointPlacement placement{};
    bool has_plan = false;
    if (effective_fixture_plan != nullptr) {
      const auto plan_it = effective_fixture_plan->find(port->id);
      if (plan_it != effective_fixture_plan->end()) {
        placement.fixture_root = plan_it->second.endpoint_fixture_root;
        placement.wire_endpoint = plan_it->second.wire_endpoint;
        has_plan = true;
      }
    }
    if (!has_plan) {
      const EditResult<ResolvedEndpointPlacement> resolved = resolve_endpoint_placement(state, *port, 0.0);
      if (!resolved.ok) {
        out.error = resolved.error;
        return out;
      }
      placement = resolved.value;
    }
    std::string error{};
    append_instances(state, *context.value.pole, height_m, *context.value.assembly,
                     placement.fixture_root, {},
                     "port:" + std::to_string(port->id), &out.value, &error);
    if (!error.empty()) {
      out.error = std::move(error);
      return out;
    }
  }

  std::sort(out.value.instances.begin(), out.value.instances.end(),
            [](const VisualModelInstance& a, const VisualModelInstance& b) {
              return a.stable_key < b.stable_key;
            });
  out.ok = true;
  return out;
}

EditResult<VisualModelInstanceCache> materialize_model_assemblies(
    const CoreState& state, const std::vector<SpanLayoutRule>& rules) {
  EditResult<FixturePlacementPlanByPort> fixture_plan = fixture_placement_plan_from_rules(state, rules);
  if (!fixture_plan.ok) {
    EditResult<VisualModelInstanceCache> out{};
    out.error = fixture_plan.error;
    return out;
  }
  return materialize_model_assemblies(state, &fixture_plan.value);
}

} // namespace wire::core::generation::backbone
