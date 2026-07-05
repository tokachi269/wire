#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "wire/core/types.hpp"

namespace wire::core {

namespace model_descriptor_detail {

inline bool same_vec3d(const Vec3d& a, const Vec3d& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

} // namespace model_descriptor_detail

enum class ModelSocketRole : std::uint8_t {
  kLineIn = 0,
  kLineOut = 1,
  kDrop = 2,
  kMount = 3,
};

enum class ModelSectionShape : std::uint8_t {
  kCircle = 0,
  kRect = 1,
};

struct ModelSocket {
  std::string name{};
  ModelSocketRole role = ModelSocketRole::kMount;
  Vec3d local_position{};
  Vec3d local_direction{1.0, 0.0, 0.0};

  bool operator==(const ModelSocket& other) const {
    return name == other.name && role == other.role &&
           model_descriptor_detail::same_vec3d(local_position, other.local_position) &&
           model_descriptor_detail::same_vec3d(local_direction, other.local_direction);
  }
};

struct ModelSectionSample {
  double height_m = 0.0;
  ModelSectionShape shape = ModelSectionShape::kCircle;
  double radius_m = 0.0;
  double width_m = 0.0;
  double depth_m = 0.0;

  bool operator==(const ModelSectionSample&) const = default;
};

struct ModelKeepOutZone {
  double height_min_m = 0.0;
  double height_max_m = 0.0;
  double yaw_min_deg = 0.0;
  double yaw_max_deg = 0.0;

  bool operator==(const ModelKeepOutZone&) const = default;
};

struct ModelMeasurement {
  std::string name{};
  std::uint64_t version = 0;
  double total_height_m = 0.0;
  std::vector<ModelSocket> sockets{};
  std::vector<ModelSectionSample> sections{};
  std::vector<ModelKeepOutZone> keep_out_zones{};
  double replace_length_m = 0.0;

  bool operator==(const ModelMeasurement&) const = default;
};

struct ModelOverride {
  struct SocketPosition {
    std::string socket_name{};
    Vec3d local_position{};

    bool operator==(const SocketPosition& other) const {
      return socket_name == other.socket_name &&
             model_descriptor_detail::same_vec3d(local_position, other.local_position);
    }
  };

  struct SocketDirection {
    std::string socket_name{};
    Vec3d local_direction{};

    bool operator==(const SocketDirection& other) const {
      return socket_name == other.socket_name &&
             model_descriptor_detail::same_vec3d(local_direction, other.local_direction);
    }
  };

  struct SectionDimension {
    double height_m = 0.0;
    std::string item_name{};
    double value = 0.0;

    bool operator==(const SectionDimension&) const = default;
  };

  std::vector<SocketPosition> socket_position_overrides{};
  std::vector<SocketDirection> socket_direction_overrides{};
  std::vector<SectionDimension> section_dimension_overrides{};

  bool operator==(const ModelOverride&) const = default;
};

struct ModelDescriptor {
  ModelMeasurement measurement{};

  bool operator==(const ModelDescriptor&) const = default;
};

struct ModelMergeReport {
  struct Conflict {
    std::string marker_name{};
    std::string item_name{};
    std::string message{};

    bool operator==(const Conflict&) const = default;
  };

  std::vector<Conflict> conflicts{};

  bool operator==(const ModelMergeReport&) const = default;
};

struct ModelMergeResult {
  ModelDescriptor descriptor{};
  ModelMergeReport report{};

  bool operator==(const ModelMergeResult&) const = default;
};

namespace model_descriptor_detail {

inline ModelSocket* find_socket(std::vector<ModelSocket>& sockets, const std::string& name) {
  for (ModelSocket& socket : sockets) {
    if (socket.name == name) {
      return &socket;
    }
  }
  return nullptr;
}

inline ModelSectionSample* find_section(std::vector<ModelSectionSample>& sections, double height_m) {
  for (ModelSectionSample& section : sections) {
    if (section.height_m == height_m) {
      return &section;
    }
  }
  return nullptr;
}

inline void add_conflict(ModelMergeReport& report, std::string marker_name, std::string item_name,
                         std::string message) {
  report.conflicts.push_back(
      ModelMergeReport::Conflict{std::move(marker_name), std::move(item_name), std::move(message)});
}

} // namespace model_descriptor_detail

inline ModelMergeResult merge(const ModelMeasurement& measurement, const ModelOverride& override_values) {
  ModelMergeResult result{};
  result.descriptor.measurement = measurement;

  for (const ModelOverride::SocketPosition& override_position : override_values.socket_position_overrides) {
    ModelSocket* socket =
        model_descriptor_detail::find_socket(result.descriptor.measurement.sockets, override_position.socket_name);
    if (socket == nullptr) {
      model_descriptor_detail::add_conflict(result.report, override_position.socket_name, "socket.local_position",
                                            "ModelOverride references a missing socket");
      continue;
    }
    socket->local_position = override_position.local_position;
  }

  for (const ModelOverride::SocketDirection& override_direction : override_values.socket_direction_overrides) {
    ModelSocket* socket =
        model_descriptor_detail::find_socket(result.descriptor.measurement.sockets, override_direction.socket_name);
    if (socket == nullptr) {
      model_descriptor_detail::add_conflict(result.report, override_direction.socket_name, "socket.local_direction",
                                            "ModelOverride references a missing socket");
      continue;
    }
    socket->local_direction = override_direction.local_direction;
  }

  for (const ModelOverride::SectionDimension& override_dimension : override_values.section_dimension_overrides) {
    ModelSectionSample* section =
        model_descriptor_detail::find_section(result.descriptor.measurement.sections, override_dimension.height_m);
    if (section == nullptr) {
      model_descriptor_detail::add_conflict(result.report, std::to_string(override_dimension.height_m),
                                            override_dimension.item_name,
                                            "ModelOverride references a missing section sample");
      continue;
    }
    if (override_dimension.item_name == "radius_m") {
      section->radius_m = override_dimension.value;
    } else if (override_dimension.item_name == "width_m") {
      section->width_m = override_dimension.value;
    } else if (override_dimension.item_name == "depth_m") {
      section->depth_m = override_dimension.value;
    } else {
      model_descriptor_detail::add_conflict(result.report, std::to_string(override_dimension.height_m),
                                            override_dimension.item_name,
                                            "ModelOverride references an unknown section dimension");
    }
  }

  return result;
}

} // namespace wire::core
