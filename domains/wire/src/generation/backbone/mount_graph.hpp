#pragma once

#include "city/wire/core_state.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace city::wire::generation::backbone {

enum class MountRefKind : std::uint8_t {
  kPoleFrame,
  kSpanAnchor,
  kInstanceSocket,
};

struct MountGraphSocket {
  std::string name{};
  Transformd local_transform{};
};

struct MountGraphRef {
  MountRefKind kind = MountRefKind::kPoleFrame;
  Transformd anchor_transform{};
  std::size_t parent_node = static_cast<std::size_t>(-1);
  std::string socket_name{};
};

struct MountGraphNode {
  std::size_t node = static_cast<std::size_t>(-1);
  MountGraphRef parent{};
  Transformd local_transform{};
  std::vector<MountGraphSocket> sockets{};
};

[[nodiscard]] Transformd compose_mount_transform(const Transformd& parent, const Transformd& local);
[[nodiscard]] Vec3d transform_mount_point(const Transformd& transform, const Vec3d& local);
[[nodiscard]] EditResult<Transformd> resolve_mount_node(const std::vector<MountGraphNode>& nodes,
                                                        std::size_t node_id);
[[nodiscard]] EditResult<Transformd> resolve_mount_socket(const std::vector<MountGraphNode>& nodes,
                                                          std::size_t node_id,
                                                          const std::string& socket_name);

} // namespace city::wire::generation::backbone
