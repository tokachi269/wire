#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "wire/core/id.hpp"
#include "wire/core/types.hpp"

namespace wire::core {

using PoleTypeId = std::uint32_t;
constexpr PoleTypeId kInvalidPoleTypeId = 0;
using RoadId = std::uint64_t;

enum class ConnectionCategory : std::uint8_t {
  kHighVoltage = 0,
  kLowVoltage = 1,
  kCommunication = 2,
  kOptical = 3,
  kDrop = 4,
};

enum class ConnectionContext : std::uint8_t {
  kTrunkContinue = 0,
  kCornerPass = 1,
  kBranchAdd = 2,
  kDropAdd = 3,
};

enum class PathDirectionMode : std::uint8_t {
  kAuto = 0,
  kForward = 1,
  kReverse = 2,
};

enum class PathDirectionChosen : std::uint8_t {
  kForward = 0,
  kReverse = 1,
};

enum class ConductorGroupKind : std::uint8_t {
  kSingle = 0,
  kParallel = 1,
  kThreePhase = 2,
};

enum class PoleContextKind : std::uint8_t {
  kStraight = 0,
  kCorner = 1,
  kBranch = 2,
  kTerminal = 3,
};

enum class SlotSide : std::uint8_t {
  kLeft = 0,
  kCenter = 1,
  kRight = 2,
};

enum class SlotRole : std::uint8_t {
  kNeutral = 0,
  kTrunkPreferred = 1,
  kBranchPreferred = 2,
  kDropPreferred = 3,
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

enum class PortPositionMode : std::uint8_t {
  kAuto = 0,
  kManual = 1,
};

enum class PortPlacementSourceKind : std::uint8_t {
  kUnknown = 0,
  kTemplateSlot = 1,
  kGenerated = 2,
  kManualEdit = 3,
  kAerialBranch = 4,
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

enum class WireGroupKind : std::uint8_t {
  kUnknown = 0,
  kPowerHighVoltage = 1,
  kPowerLowVoltage = 2,
  kComm = 3,
  kOptical = 4,
};

enum class WireLaneRole : std::uint8_t {
  kUnknown = 0,
  kPhaseA = 1,
  kPhaseB = 2,
  kPhaseC = 3,
  kNeutral = 4,
  kCommLine = 5,
  kOpticalFiber = 6,
  kAux = 7,
};

enum class AttachmentKind : std::uint8_t {
  kGeneric = 0,
  kDamper = 1,
  kSpacer = 2,
  kMarker = 3,
};

enum class GenerationSource : std::uint8_t {
  kManual = 0,
  kRoadAuto = 1,
};

struct GenerationMeta {
  bool generated = false;
  GenerationSource source = GenerationSource::kManual;
  std::uint64_t generation_session_id = 0;
  std::uint32_t generation_order = 0;
};

// Workflow input polyline for generation commands (not an entity).
struct RoadSegment {
  RoadId id = 0;
  std::vector<Vec3d> polyline{};
};

// Definition-layer slot candidate. This is not a runtime connection endpoint.
struct PortSlotTemplate {
  int slot_id = 0;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  Vec3d local_position{};
  Frame3d local_direction{};
  int layer = 1;
  SlotSide side = SlotSide::kCenter;
  SlotRole role = SlotRole::kNeutral;
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

struct PoleContextInfo {
  PoleContextKind kind = PoleContextKind::kStraight;
  double corner_angle_deg = 0.0;
  double corner_turn_sign = 0.0;  // -1:right turn, +1:left turn, 0:undefined/straight
  double side_scale = 1.0;
  bool angle_correction_applied = false;
};

struct PoleOrientationControl {
  bool flip_180 = false;
  bool manual_yaw_override = false;
  double manual_yaw_deg = 0.0;
};

// Entity-layer support object.
struct Pole {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  std::string name{};
  Transformd world_transform{};
  double height_m = 10.0;
  PoleKind kind = PoleKind::kGeneric;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  PoleContextInfo context{};
  PoleOrientationControl orientation_control{};
  bool placement_override_flag = false;
  bool orientation_override_flag = false;
  GenerationMeta generation{};
};

// Entity-layer endpoint used by spans. Ports may originate from template slots.
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
  int template_layer = 1;
  SlotSide template_side = SlotSide::kCenter;
  SlotRole template_role = SlotRole::kNeutral;
  bool generated_from_template = false;
  bool generated_by_rule = false;
  ConnectionContext placement_context = ConnectionContext::kTrunkContinue;
  bool angle_correction_applied = false;
  double side_scale_applied = 1.0;
  PortPositionMode position_mode = PortPositionMode::kAuto;
  PortPlacementSourceKind placement_source = PortPlacementSourceKind::kUnknown;
  bool user_edited_position = false;
  bool placement_override_flag = false;
  bool orientation_override_flag = false;
};

// Entity-layer support point.
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

// Entity-layer bundle attribute object.
struct Bundle {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  int conductor_count = 1;
  double phase_spacing_m = 0.3;
  BundleKind kind = BundleKind::kLowVoltage;
};

// Entity-layer grouped wiring logical unit (separate responsibility from visual Bundle).
struct WireGroup {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  WireGroupKind kind = WireGroupKind::kUnknown;
  std::string network_tag{};
  std::string feeder_tag{};
  std::uint64_t generation_session_id = 0;
  bool user_edited = false;
  bool visible = true;
};

// Entity-layer lane identifier inside WireGroup.
struct WireLane {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId wire_group_id = kInvalidObjectId;
  int lane_index = 0;
  WireLaneRole role = WireLaneRole::kUnknown;
  bool enabled = true;
  bool user_locked_order = false;
};

// Workflow-layer grouped generation input.
struct ConductorGroupSpec {
  ConnectionCategory category = ConnectionCategory::kHighVoltage;
  ConductorGroupKind group_kind = ConductorGroupKind::kSingle;
  int conductor_count = 1;
  double lane_spacing_m = 0.3;
  bool maintain_lane_order = true;
  bool allow_lane_mirror = true;
};

struct ConductorLaneId {
  int lane_index = 0;
};

struct ConductorGroupState {
  ObjectId bundle_id = kInvalidObjectId;
  ConductorGroupSpec spec{};
  std::vector<int> canonical_lane_order{};
};

// Entity-layer connection edge.
struct Span {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId port_a_id = kInvalidObjectId;
  ObjectId port_b_id = kInvalidObjectId;
  SpanKind kind = SpanKind::kGeneric;
  SpanLayer layer = SpanLayer::kUnknown;
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId wire_group_id = kInvalidObjectId;
  ObjectId wire_lane_id = kInvalidObjectId;
  ObjectId anchor_a_id = kInvalidObjectId;
  ObjectId anchor_b_id = kInvalidObjectId;
  ConnectionContext placement_context = ConnectionContext::kTrunkContinue;
  bool generated_by_rule = false;
  bool placement_override_flag = false;
  bool orientation_override_flag = false;
  GenerationMeta generation{};
};

// Entity-layer span attachment.
struct Attachment {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId span_id = kInvalidObjectId;
  double t = 0.0;
  AttachmentKind kind = AttachmentKind::kGeneric;
  double offset_m = 0.0;
};

}  // namespace wire::core
