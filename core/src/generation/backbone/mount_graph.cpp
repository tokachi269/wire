#include "mount_graph.hpp"

#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wire::core::generation::backbone {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Matrix3 {
  double m[3][3]{};
};

Matrix3 matrix_from_euler(const Vec3d& euler_deg) {
  const Vec3d x = RotateEulerXYZDeg({1.0, 0.0, 0.0}, euler_deg);
  const Vec3d y = RotateEulerXYZDeg({0.0, 1.0, 0.0}, euler_deg);
  const Vec3d z = RotateEulerXYZDeg({0.0, 0.0, 1.0}, euler_deg);
  return {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
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

const MountGraphNode* node_by_id(const std::vector<MountGraphNode>& nodes, std::size_t node_id) {
  const auto found = std::find_if(nodes.begin(), nodes.end(),
                                  [&](const MountGraphNode& node) { return node.node == node_id; });
  return found == nodes.end() ? nullptr : &*found;
}

const MountGraphSocket* socket_by_name(const MountGraphNode& node, const std::string& socket_name) {
  const auto found = std::find_if(node.sockets.begin(), node.sockets.end(),
                                  [&](const MountGraphSocket& socket) { return socket.name == socket_name; });
  return found == node.sockets.end() ? nullptr : &*found;
}

EditResult<Transformd> resolve_node_impl(const std::vector<MountGraphNode>& nodes,
                                         std::size_t node_id,
                                         std::vector<std::size_t>* stack);

EditResult<Transformd> resolve_socket_impl(const std::vector<MountGraphNode>& nodes,
                                           std::size_t node_id,
                                           const std::string& socket_name,
                                           std::vector<std::size_t>* stack) {
  EditResult<Transformd> out{};
  const MountGraphNode* node = node_by_id(nodes, node_id);
  const MountGraphSocket* socket = node == nullptr ? nullptr : socket_by_name(*node, socket_name);
  if (node == nullptr || socket == nullptr) {
    out.error = "model mount graph unsupported: socket reference is missing";
    return out;
  }
  EditResult<Transformd> parent = resolve_node_impl(nodes, node_id, stack);
  if (!parent.ok) {
    out.error = parent.error;
    return out;
  }
  out.value = compose_mount_transform(parent.value, socket->local_transform);
  out.ok = true;
  return out;
}

EditResult<Transformd> resolve_node_impl(const std::vector<MountGraphNode>& nodes,
                                         std::size_t node_id,
                                         std::vector<std::size_t>* stack) {
  EditResult<Transformd> out{};
  if (stack == nullptr) {
    out.error = "model mount graph internal: stack missing";
    return out;
  }
  if (std::find(stack->begin(), stack->end(), node_id) != stack->end()) {
    out.error = "model mount graph unsupported: cyclic mount reference";
    return out;
  }
  const MountGraphNode* node = node_by_id(nodes, node_id);
  if (node == nullptr) {
    out.error = "model mount graph unsupported: node reference is missing";
    return out;
  }
  stack->push_back(node_id);
  Transformd parent{};
  if (node->parent.kind == MountRefKind::kRoot) {
    parent = node->parent.root_transform;
  } else if (node->parent.kind == MountRefKind::kInstanceSocket) {
    EditResult<Transformd> socket =
        resolve_socket_impl(nodes, node->parent.parent_node, node->parent.socket_name, stack);
    if (!socket.ok) {
      out.error = socket.error;
      return out;
    }
    parent = socket.value;
  }
  stack->pop_back();
  out.value = compose_mount_transform(parent, node->local_transform);
  out.ok = true;
  return out;
}

} // namespace

Transformd compose_mount_transform(const Transformd& parent, const Transformd& local) {
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

Vec3d transform_mount_point(const Transformd& transform, const Vec3d& local) {
  const Vec3d scaled{
      local.x * transform.scale.x,
      local.y * transform.scale.y,
      local.z * transform.scale.z,
  };
  return transform.position + multiply(matrix_from_euler(transform.rotation_euler_deg), scaled);
}

EditResult<Transformd> resolve_mount_node(const std::vector<MountGraphNode>& nodes, std::size_t node_id) {
  std::vector<std::size_t> stack{};
  return resolve_node_impl(nodes, node_id, &stack);
}

EditResult<Transformd> resolve_mount_socket(const std::vector<MountGraphNode>& nodes,
                                            std::size_t node_id,
                                            const std::string& socket_name) {
  std::vector<std::size_t> stack{};
  return resolve_socket_impl(nodes, node_id, socket_name, &stack);
}

} // namespace wire::core::generation::backbone
