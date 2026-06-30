#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "wire/core/id.hpp"
#include "wire/core/types.hpp"

namespace wire::core {

using PoleTypeId = std::uint32_t;
constexpr PoleTypeId kInvalidPoleTypeId = 0;
using CableTemplateId = std::uint32_t;
constexpr CableTemplateId kInvalidCableTemplateId = 0;
using AttachmentTemplateId = std::uint32_t;
constexpr AttachmentTemplateId kInvalidAttachmentTemplateId = 0;
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
  kDrop = 5,
};

enum class PlacementMode : std::uint8_t {
  kAuto = 0,
  kManual = 1,
};

enum class PortPositionMode : std::uint8_t {
  kAuto = 0,
  kManual = 1,
};

enum class PortPlacementSourceKind : std::uint8_t {
  kUnknown = 0,
  kPlacementBand = 1,
  kGenerated = 2,
  kManualEdit = 3,
  kAerialBranch = 4,
  kBranchSupport = 5,
  kPlacementBandConstrained = 6,
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
  kDrop = 5,
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
  kExternal = 2,
  kMidair = 3,
};

enum class BundleKind : std::uint8_t {
  kLowVoltage = 0,
  kHighVoltage = 1,
  kCommunication = 2,
  kOptical = 3,
  kDrop = 4,
  kOpticalWithSupport = 5,
};

enum class CableMaterialStyleKind : std::uint8_t {
  kGeneric = 0,
  kBareConductor = 1,
  kInsulated = 2,
  kOptical = 3,
};

enum class CableContinuityPolicyHint : std::uint8_t {
  kAuto = 0,
  // Prefer a direct G1-safe cubic. Do not attempt to preserve G2 context.
  kPreferG1 = 1,
  // Prefer G2-like continuity when the span context is smooth, but allow explicit fallback to G1.
  kPreferG2 = 2,
};

enum class CableAttachmentStyleHint : std::uint8_t {
  kAuto = 0,
  kDirectThrough = 1,
  kViaAttachment = 2,
};

enum class BundleSupportStyleHint : std::uint8_t {
  kAuto = 0,
  kCenterPreferred = 1,
  kSideStructurePreferred = 2,
};

enum class BundleBranchPolicyHint : std::uint8_t {
  kAuto = 0,
  kPreferPassThrough = 1,
  kPreferExplicitBranch = 2,
};

enum class SupportKind : std::uint8_t {
  kPole = 0,
  kMidair = 1,
  kExternal = 2,
  kGround = 3,
};

enum class AttachmentKind : std::uint8_t {
  kGeneric = 0,
  kDamper = 1,
  kSpacer = 2,
  kMarker = 3,
};

enum class AttachmentSocketKind : std::uint8_t {
  kGeneric = 0,
  kInput = 1,
  kOutput = 2,
};

enum class AttachmentLineInteractionMode : std::uint8_t {
  kPassThrough = 0,
  kReplaceWithInternalPath = 1,
  kHideSegment = 2,
  kAddInternalPath = 3,
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

enum class BandOverflowPolicy : std::uint8_t {
  kTrySiblingBand = 0,
  kRaiseHeight = 1,
  kConstrainedFallback = 2,
};

// Definition-layer placement band. This is not a runtime connection endpoint.
struct PortPlacementBand {
  int band_id = 0;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  Frame3d local_direction{};
  int layer = 1;
  SlotSide side = SlotSide::kCenter;
  SlotRole role = SlotRole::kNeutral;
  // Allowed placement range in the pole-local lateral axis.
  double lateral_center_m = 0.0;
  double lateral_min_m = -0.1;
  double lateral_max_m = 0.1;
  // Allowed placement range in the pole-local height axis.
  double height_center_m = 6.0;
  double height_min_m = 5.8;
  double height_max_m = 6.2;
  int priority = 0;
  double min_spacing_m = 0.25;
  bool allow_multiple = false;
  BandOverflowPolicy overflow_policy = BandOverflowPolicy::kTrySiblingBand;
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
  double default_height_m = 10.0;
  std::vector<PortPlacementBand> port_bands{};
  std::vector<AnchorSlotTemplate> anchor_slots{};
};

struct PoleContextInfo {
  PoleContextKind kind = PoleContextKind::kStraight;
  double corner_angle_deg = 0.0;
  double corner_turn_sign = 0.0; // -1:right turn, +1:left turn, 0:undefined/straight
  double side_scale = 1.0;
  bool angle_correction_applied = false;
  // Sharp-corner (theta <= threshold) orientation diagnostics.
  bool sharp_orientation_applied = false;
  double sharp_theta_deg = 0.0;
  Vec3d sharp_bisector_dir{};
  Vec3d sharp_side_dir{};
};

// Entity-layer support object.
struct Pole {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  std::string name{};
  // Pole tilt lives on the instance transform and must not be overwritten by template edits.
  Transformd world_transform{};
  double tilt_magnitude_deg = 0.0;
  double height_m = 10.0;
  PoleKind kind = PoleKind::kGeneric;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  PoleContextInfo context{};
  PlacementMode placement_mode = PlacementMode::kAuto;
  bool user_edited = false;
  bool placement_override_flag = false;
  GenerationMeta generation{};
};

// Entity-layer endpoint used by spans.
struct Port {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId owner_pole_id = kInvalidObjectId;
  Vec3d world_position{};
  PortKind kind = PortKind::kGeneric;
  PortLayer layer = PortLayer::kUnknown;
  Frame3d direction{};
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
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
  bool generated_from_template = false;
};

// Entity-layer bundle attribute object.
struct Bundle {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  int conductor_count = 1;
  double phase_spacing_m = 0.3;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
};

// Entity-layer connection edge.
struct Span {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId port_a_id = kInvalidObjectId;
  ObjectId port_b_id = kInvalidObjectId;
  ObjectId endpoint_node_a_id = kInvalidObjectId;
  ObjectId endpoint_node_b_id = kInvalidObjectId;
  SpanKind kind = SpanKind::kGeneric;
  SpanLayer layer = SpanLayer::kUnknown;
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId anchor_a_id = kInvalidObjectId;
  ObjectId anchor_b_id = kInvalidObjectId;
  ObjectId endpoint_attachment_a_id = kInvalidObjectId;
  ObjectId endpoint_attachment_b_id = kInvalidObjectId;
  ConnectionContext placement_context = ConnectionContext::kTrunkContinue;
  bool generated_by_rule = false;
  bool placement_override_flag = false;
  // Reference length to keep visual tension stable when poles tilt/move.
  double reference_length_m = 0.0;
  GenerationMeta generation{};
};

// Entity-layer span attachment.
struct Attachment {
  ObjectId id = kInvalidObjectId;
  std::string display_id{};
  ObjectId span_id = kInvalidObjectId;
  AttachmentTemplateId template_id = kInvalidAttachmentTemplateId;
  double t = 0.0;
  AttachmentKind kind = AttachmentKind::kGeneric;
  double display_offset_m = 0.0;
};

// Backbone route edge (NodeId refers to SupportNode id).
struct BackboneEdge {
  ObjectId node_a = kInvalidObjectId;
  ObjectId node_b = kInvalidObjectId;
  std::vector<ObjectId> bundles{};
};

} // namespace wire::core
