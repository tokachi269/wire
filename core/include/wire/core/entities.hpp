#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "wire/core/id.hpp"
#include "wire/core/types.hpp"

namespace wire::core {

using PoleTypeId = std::uint32_t;
constexpr PoleTypeId kInvalidPoleTypeId = 0;

enum class ConnectionCategory : std::uint8_t {
  kHighVoltage = 0,
  kLowVoltage = 1,
  kCommunication = 2,
  kOptical = 3,
  kDrop = 4,
};

enum class PortKind : std::uint8_t {
  kGeneric = 0,
  kPower = 1,
  kCommunication = 2,
};

enum class PortLayer : std::uint8_t {
  kUnknown = 0,
  kHighVoltage = 1,
  kLowVoltage = 2,
  kCommunication = 3,
  kOptical = 4,
};

enum class SpanKind : std::uint8_t {
  kGeneric = 0,
  kDistribution = 1,
  kService = 2,
};

enum class SpanLayer : std::uint8_t {
  kUnknown = 0,
  kHighVoltage = 1,
  kLowVoltage = 2,
  kCommunication = 3,
  kOptical = 4,
};

enum class PoleKind : std::uint8_t {
  kGeneric = 0,
  kWood = 1,
  kConcrete = 2,
  kSteel = 3,
};

enum class AnchorSupportKind : std::uint8_t {
  kGeneric = 0,
  kGround = 1,
  kBuilding = 2,
  kMidair = 3,
};

enum class BundleKind : std::uint8_t {
  kLowVoltage = 0,
  kHighVoltage = 1,
  kCommunication = 2,
  kOptical = 3,
};

enum class AttachmentKind : std::uint8_t {
  kGeneric = 0,
  kDamper = 1,
  kSpacer = 2,
  kMarker = 3,
};

struct PortSlotTemplate {
  int slot_id = 0;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  Vec3d local_position{};
  Frame3d local_direction{};
  int priority = 0;
  bool allow_multiple = false;
  bool enabled = true;
};

struct AnchorSlotTemplate {
  int slot_id = 0;
  AnchorSupportKind usage = AnchorSupportKind::kGeneric;
  Vec3d local_position{};
  int priority = 0;
  bool enabled = true;
};

struct PoleTypeDefinition {
  PoleTypeId id = kInvalidPoleTypeId;
  std::string name{};
  std::string description{};
  std::vector<PortSlotTemplate> port_slots{};
  std::vector<AnchorSlotTemplate> anchor_slots{};
};

struct Pole {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  std::string name{};
  Transformd world_transform{};
  double height_m = 10.0;
  PoleKind kind = PoleKind::kGeneric;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
};

struct Port {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId owner_pole_id = kInvalidObjectId;
  Vec3d world_position{};
  PortKind kind = PortKind::kGeneric;
  PortLayer layer = PortLayer::kUnknown;
  Frame3d direction{};
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  int source_slot_id = -1;
  bool generated_from_template = false;
};

struct Anchor {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId owner_pole_id = kInvalidObjectId;
  Vec3d world_position{};
  AnchorSupportKind support_kind = AnchorSupportKind::kGeneric;
  double support_strength = 1.0;
  int source_slot_id = -1;
  bool generated_from_template = false;
};

struct Bundle {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  int conductor_count = 1;
  double phase_spacing_m = 0.3;
  BundleKind kind = BundleKind::kLowVoltage;
};

struct Span {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId port_a_id = kInvalidObjectId;
  ObjectId port_b_id = kInvalidObjectId;
  SpanKind kind = SpanKind::kGeneric;
  SpanLayer layer = SpanLayer::kUnknown;
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId anchor_a_id = kInvalidObjectId;
  ObjectId anchor_b_id = kInvalidObjectId;
};

struct Attachment {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId span_id = kInvalidObjectId;
  double t = 0.0;
  AttachmentKind kind = AttachmentKind::kGeneric;
  double offset_m = 0.0;
};

}  // namespace wire::core
