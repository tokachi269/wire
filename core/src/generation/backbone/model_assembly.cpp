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

EditResult<Transformd> fitted_part_transform(const CoreState& state,
                                             const Pole& pole,
                                             double placement_height_m,
                                             const ModelAssemblyPart& part,
                                             const Transformd& root) {
  EditResult<Transformd> out{};
  Transformd fitted = part.local_transform;
  if (part.fit_mode == ModelFitMode::kPoleHeight) {
    const auto pole_type_it = state.view().pole_types().find(pole.pole_type_id);
    if (pole_type_it == state.view().pole_types().end() || pole_type_it->second.default_height_m <= 1e-9) {
      out.error = "model assembly unsupported: pole height fit requires a valid PoleType default height";
      return out;
    }
    fitted.scale.z *= pole.height_m / pole_type_it->second.default_height_m;
  } else if (part.fit_mode == ModelFitMode::kPoleRadial) {
    const double radius = state.view().pole_radius_at_height_m(pole, placement_height_m);
    fitted.scale.x *= radius;
    fitted.scale.y *= radius;
  }
  out.value = compose(root, fitted);
  out.ok = true;
  return out;
}

Vec3d pole_surface_position_at_root(const CoreState& state, const Pole& pole,
                                    double placement_height_m, const Transformd& root) {
  const Matrix3 rotation = matrix_from_euler(root.rotation_euler_deg);
  const Vec3d up = multiply(rotation, Vec3d{0.0, 0.0, 1.0});
  const Vec3d axis_position = pole.world_transform.position +
      multiply(rotation, Vec3d{0.0, 0.0, placement_height_m});
  Vec3d direction = root.position - axis_position;
  direction = direction - ScaleVec(up, Dot(direction, up));
  if (Length(direction) <= 1e-9) {
    direction = multiply(rotation, Vec3d{0.0, 1.0, 0.0});
  }
  Normalize(&direction);
  return axis_position + ScaleVec(
      direction, state.view().pole_radius_at_height_m(pole, placement_height_m));
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
    Transformd part_root = root;
    if (part.fit_mode == ModelFitMode::kPoleRadial) {
      part_root.position = pole.world_transform.position +
          multiply(matrix_from_euler(root.rotation_euler_deg),
                   Vec3d{0.0, 0.0, placement_height_m});
    } else if (part.fit_mode == ModelFitMode::kPoleSurface) {
      part_root.position = pole_surface_position_at_root(
          state, pole, placement_height_m, root);
    } else {
      part_root.position = part_root.position + rigid_offset_world;
    }
    const EditResult<Transformd> world =
        fitted_part_transform(state, pole, placement_height_m, part, part_root);
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

} // namespace

EditResult<ResolvedEndpointPlacement> resolve_endpoint_placement(const CoreState& state,
                                                                  const Port& port) {
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
  const double placement_height_m = WorldPointToLocal(frame, port.world_position).z;
  Transformd fixture_root = transform_from_frame(frame, port.world_position);

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
    const EditResult<Transformd> mount_world = fitted_part_transform(
        state, *context.value.pole, placement_height_m, *mount_part, fixture_root);
    if (!mount_world.ok) {
      out.error = mount_world.error;
      return out;
    }
    fixture_root.position = fixture_root.position +
        (transform_point(mount_world.value, mount->local_position) - port.world_position);
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
    Transformd socket_root = fixture_root;
    if (socket_part->fit_mode == ModelFitMode::kPoleSurface) {
      socket_root.position = pole_surface_position_at_root(
          state, *context.value.pole, placement_height_m, fixture_root);
    }
    const EditResult<Transformd> world = fitted_part_transform(
        state, *context.value.pole, placement_height_m, *socket_part, socket_root);
    if (!world.ok) {
      out.error = world.error;
      return out;
    }
    out.value.wire_endpoint = transform_point(world.value, socket->local_position);
  }
  out.ok = true;
  return out;
}

EditResult<Vec3d> resolve_model_assembly_wire_socket(const CoreState& state, const Port& port) {
  EditResult<Vec3d> out{};
  const EditResult<ResolvedEndpointPlacement> placement = resolve_endpoint_placement(state, port);
  out.ok = placement.ok;
  out.error = placement.error;
  out.value = placement.value.wire_endpoint;
  return out;
}

EditResult<VisualModelInstanceCache> materialize_model_assemblies(const CoreState& state) {
  EditResult<VisualModelInstanceCache> out{};
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

  std::vector<RowFixtureContext> rows{};
  std::vector<ObjectId> endpoint_ports{};
  for (const SavedBackbonePortBinding& binding : view.backbone().port_bindings) {
    const Port* port = view.ports().find(binding.port_id);
    const Pole* pole = port == nullptr ? nullptr : view.poles().find(port->owner_pole_id);
    const auto bundle_it = view.bundle_templates().find(binding.bundle_template_id);
    if (port == nullptr || pole == nullptr || bundle_it == view.bundle_templates().end()) {
      continue;
    }
    if (std::find(endpoint_ports.begin(), endpoint_ports.end(), port->id) == endpoint_ports.end()) {
      endpoint_ports.push_back(port->id);
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
    auto row_it = std::find_if(rows.begin(), rows.end(), [&](const RowFixtureContext& row) {
      return row.pole->id == pole->id && row.row_key == binding.row_key &&
             row.bundle_id == edge_bundle->bundle_id;
    });
    if (row_it == rows.end()) {
      rows.push_back({pole, binding.row_key, edge_bundle->bundle_id, binding.bundle_template_id,
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

  for (const RowFixtureContext& row : rows) {
    const auto assembly_it = view.model_assembly_templates().find(row.assembly_id);
    if (assembly_it == view.model_assembly_templates().end() || row.members.empty()) {
      out.error = "model assembly unsupported: row fixture assembly is missing";
      return out;
    }
    const PoleFrame frame = BuildPoleFrame(row.pole->world_transform, row.layout_yaw_deg);
    double height_m = 0.0;
    for (const RowFixtureContext::Member& member : row.members) {
      height_m += WorldPointToLocal(frame, view.ports().find(member.port_id)->world_position).z;
    }
    height_m /= static_cast<double>(row.members.size());
    const EditResult<double> lateral = row_lateral_position(state, row);
    if (!lateral.ok) {
      out.error = lateral.error;
      return out;
    }
    const Transformd root = transform_from_frame(frame, LocalPointToWorld(frame, {0.0, 0.0, height_m}));
    std::string error{};
    append_instances(
        state, *row.pole, height_m, assembly_it->second, root,
        ScaleVec(frame.lateral, lateral.value),
        "row:" + std::to_string(row.pole->id) + ":" + row_key_text(row.row_key) + ":" +
            std::to_string(row.bundle_id),
        &out.value, &error);
    if (!error.empty()) {
      out.error = std::move(error);
      return out;
    }
  }

  std::sort(endpoint_ports.begin(), endpoint_ports.end());
  for (ObjectId port_id : endpoint_ports) {
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
    const EditResult<ResolvedEndpointPlacement> placement =
        resolve_endpoint_placement(state, *port);
    if (!placement.ok) {
      out.error = placement.error;
      return out;
    }
    std::string error{};
    append_instances(state, *context.value.pole, height_m, *context.value.assembly,
                     placement.value.fixture_root, {},
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

} // namespace wire::core::generation::backbone
