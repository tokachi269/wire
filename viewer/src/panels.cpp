#include "panels.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "core_state_adapter.hpp"
#include "imgui.h"
#include "raylib.h"
#include "ui_common.hpp"

namespace {

std::string DirtyBitsToText(wire::core::DirtyBits bits) {
  std::string out;
  if ((bits & wire::core::DirtyBits::kTopology) != wire::core::DirtyBits::kNone) out += "T";
  if ((bits & wire::core::DirtyBits::kDecision) != wire::core::DirtyBits::kNone) out += "D";
  if ((bits & wire::core::DirtyBits::kGeometryRefresh) != wire::core::DirtyBits::kNone) out += "G";
  if ((bits & wire::core::DirtyBits::kBounds) != wire::core::DirtyBits::kNone) out += "B";
  if ((bits & wire::core::DirtyBits::kRenderRefresh) != wire::core::DirtyBits::kNone) out += "R";
  if ((bits & wire::core::DirtyBits::kRaycast) != wire::core::DirtyBits::kNone) out += "X";
  return out.empty() ? std::string("-") : out;
}

const char* SpanLayerLabel(wire::core::SpanLayer layer) {
  switch (layer) {
  case wire::core::SpanLayer::kHighVoltage:
    return "HighVoltage";
  case wire::core::SpanLayer::kLowVoltage:
    return "LowVoltage";
  case wire::core::SpanLayer::kCommunication:
    return "Communication";
  case wire::core::SpanLayer::kOptical:
    return "Optical";
  case wire::core::SpanLayer::kDrop:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* CableMaterialStyleLabel(wire::core::CableMaterialStyleKind kind) {
  switch (kind) {
  case wire::core::CableMaterialStyleKind::kGeneric:
    return "Generic";
  case wire::core::CableMaterialStyleKind::kBareConductor:
    return "BareConductor";
  case wire::core::CableMaterialStyleKind::kInsulated:
    return "Insulated";
  case wire::core::CableMaterialStyleKind::kOptical:
    return "Optical";
  default:
    return "Unknown";
  }
}

const char* CableAttachmentStyleLabel(wire::core::CableAttachmentStyleHint kind) {
  switch (kind) {
  case wire::core::CableAttachmentStyleHint::kAuto:
    return "Auto";
  case wire::core::CableAttachmentStyleHint::kDirectThrough:
    return "DirectThrough";
  case wire::core::CableAttachmentStyleHint::kViaAttachment:
    return "ViaAttachment";
  default:
    return "Unknown";
  }
}

const char* StyleObjectKindLabel(wire::core::StyleObjectKind kind) {
  switch (kind) {
  case wire::core::StyleObjectKind::kSpan:
    return "Span";
  case wire::core::StyleObjectKind::kEndpoint:
    return "Endpoint";
  case wire::core::StyleObjectKind::kAttachment:
    return "Attachment";
  case wire::core::StyleObjectKind::kSupport:
    return "Support";
  case wire::core::StyleObjectKind::kPoleAccessory:
    return "PoleAccessory";
  default:
    return "Unknown";
  }
}

const char* ContinuityPolicyLabel(wire::core::CableContinuityPolicyHint policy) {
  switch (policy) {
  case wire::core::CableContinuityPolicyHint::kAuto:
    return "Auto";
  case wire::core::CableContinuityPolicyHint::kPreferG1:
    return "PreferG1";
  case wire::core::CableContinuityPolicyHint::kPreferG2:
    return "PreferG2";
  default:
    return "Unknown";
  }
}

const char* SupportLayoutEndpointSourceLabel(wire::core::SupportLayoutEndpointSourceKind source) {
  switch (source) {
  case wire::core::SupportLayoutEndpointSourceKind::kPlainSupport:
    return "PlainSupport";
  case wire::core::SupportLayoutEndpointSourceKind::kAttachmentSocket:
    return "AttachmentSocket";
  case wire::core::SupportLayoutEndpointSourceKind::kAttachmentSocketOverride:
    return "AttachmentSocketOverride";
  case wire::core::SupportLayoutEndpointSourceKind::kFallback:
  default:
    return "Fallback";
  }
}

const char* EndpointAttachmentRequestKindLabel(wire::core::EndpointAttachmentRequestKind kind) {
  switch (kind) {
  case wire::core::EndpointAttachmentRequestKind::kNone:
    return "None";
  case wire::core::EndpointAttachmentRequestKind::kAttachmentAuto:
    return "AttachmentAuto";
  case wire::core::EndpointAttachmentRequestKind::kAttachmentSocket:
    return "AttachmentSocket";
  case wire::core::EndpointAttachmentRequestKind::kDanglingSocket:
    return "DanglingSocket";
  default:
    return "Unknown";
  }
}

const char* DetailCurveContinuityModeLabel(wire::core::DetailCurveContinuityMode mode) {
  switch (mode) {
  case wire::core::DetailCurveContinuityMode::kG1:
    return "G1";
  case wire::core::DetailCurveContinuityMode::kG2:
    return "G2";
  default:
    return "Unknown";
  }
}

const char* DetailCurveContinuityReasonLabel(wire::core::DetailCurveContinuityReason reason) {
  switch (reason) {
  case wire::core::DetailCurveContinuityReason::kAutoBalanced:
    return "AutoBalanced";
  case wire::core::DetailCurveContinuityReason::kSmoothPassThrough:
    return "SmoothPassThrough";
  case wire::core::DetailCurveContinuityReason::kPolicyPreferG1:
    return "PolicyPreferG1";
  case wire::core::DetailCurveContinuityReason::kShortSpan:
    return "ShortSpan";
  case wire::core::DetailCurveContinuityReason::kBranchPass:
    return "BranchPass";
  case wire::core::DetailCurveContinuityReason::kCornerPass:
    return "CornerPass";
  case wire::core::DetailCurveContinuityReason::kEndpointConstraintPriority:
    return "EndpointConstraintPriority";
  case wire::core::DetailCurveContinuityReason::kConflictingTangents:
    return "ConflictingTangents";
  case wire::core::DetailCurveContinuityReason::kContextInsufficient:
    return "ContextInsufficient";
  case wire::core::DetailCurveContinuityReason::kPoorQualityFallback:
    return "PoorQualityFallback";
  default:
    return "Unknown";
  }
}

const char* CurveShapePolicyLabel(wire::core::CurveShapePolicyKind kind) {
  switch (kind) {
  case wire::core::CurveShapePolicyKind::kNearStraight:
    return "NearStraight";
  case wire::core::CurveShapePolicyKind::kSmoothPass:
    return "SmoothPass";
  case wire::core::CurveShapePolicyKind::kSharpCorner:
    return "SharpCorner";
  case wire::core::CurveShapePolicyKind::kBranchPass:
    return "BranchPass";
  case wire::core::CurveShapePolicyKind::kTerminate:
    return "Terminate";
  case wire::core::CurveShapePolicyKind::kViaAttachment:
    return "ViaAttachment";
  default:
    return "Unknown";
  }
}

const char* DetailCurveEndpointTangentRuleLabel(wire::core::DetailCurveEndpointTangentRule rule) {
  switch (rule) {
  case wire::core::DetailCurveEndpointTangentRule::kFallbackChord:
    return "FallbackChord";
  case wire::core::DetailCurveEndpointTangentRule::kMainFlowBlend:
    return "MainFlowBlend";
  case wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority:
    return "BranchChordPriority";
  case wire::core::DetailCurveEndpointTangentRule::kTerminateEndpointPriority:
    return "TerminateEndpointPriority";
  case wire::core::DetailCurveEndpointTangentRule::kAttachmentEndpointPriority:
    return "AttachmentEndpointPriority";
  default:
    return "Unknown";
  }
}

const char* SupportLayoutOriginLabel(wire::core::SupportLayoutOriginKind origin) {
  switch (origin) {
  case wire::core::SupportLayoutOriginKind::kMainSupport:
    return "MainSupport";
  case wire::core::SupportLayoutOriginKind::kBranchSupport:
    return "BranchSupport";
  case wire::core::SupportLayoutOriginKind::kAerialBranch:
    return "AerialBranch";
  case wire::core::SupportLayoutOriginKind::kFallback:
  default:
    return "Fallback";
  }
}

const char* CurveEndpointModeLabel(wire::core::CurveEndpointMode mode) {
  switch (mode) {
  case wire::core::CurveEndpointMode::kDirectThrough:
    return "DirectThrough";
  case wire::core::CurveEndpointMode::kOffsetEndpoint:
    return "OffsetEndpoint";
  default:
    return "Unknown";
  }
}

const char* EntityKindLabel(wire::core::EntityKind kind) {
  switch (kind) {
  case wire::core::EntityKind::kPole:
    return "Pole";
  case wire::core::EntityKind::kJunction:
    return "Junction";
  case wire::core::EntityKind::kBackboneEdge:
    return "BackboneEdge";
  case wire::core::EntityKind::kSupportNode:
    return "SupportNode";
  case wire::core::EntityKind::kSupportLayout:
    return "SupportLayout";
  case wire::core::EntityKind::kSpan:
    return "Span";
  case wire::core::EntityKind::kBundle:
    return "Bundle";
  case wire::core::EntityKind::kDetailCurve:
    return "DetailCurve";
  case wire::core::EntityKind::kAttachmentEndpoint:
    return "AttachmentEndpoint";
  case wire::core::EntityKind::kTemplate:
    return "Template";
  case wire::core::EntityKind::kOverride:
    return "Override";
  default:
    return "Unknown";
  }
}

const char* EntityRoleKindLabel(wire::core::EntityRoleKind role) {
  switch (role) {
  case wire::core::EntityRoleKind::kAuthoritative:
    return "authoritative";
  case wire::core::EntityRoleKind::kOverride:
    return "override";
  case wire::core::EntityRoleKind::kDerived:
    return "derived";
  case wire::core::EntityRoleKind::kDetailDerived:
    return "detail derived";
  default:
    return "unknown";
  }
}

const char* DecisionTraceTopicLabel(wire::core::DecisionTraceTopic topic) {
  switch (topic) {
  case wire::core::DecisionTraceTopic::kPoleOrientation:
    return "pole orientation";
  case wire::core::DecisionTraceTopic::kFlowClassification:
    return "flow classification";
  case wire::core::DecisionTraceTopic::kSupportLayoutSelection:
    return "support layout";
  case wire::core::DecisionTraceTopic::kTangentGeneration:
    return "tangent generation";
  case wire::core::DecisionTraceTopic::kContinuitySelection:
    return "continuity";
  case wire::core::DecisionTraceTopic::kContinuityDegrade:
    return "G2/G1 degrade";
  case wire::core::DecisionTraceTopic::kSagProfile:
    return "sag profile";
  case wire::core::DecisionTraceTopic::kOverrideResolution:
    return "override resolution";
  default:
    return "unknown";
  }
}

const char* BundleSupportStyleLabel(wire::core::BundleSupportStyleHint style) {
  switch (style) {
  case wire::core::BundleSupportStyleHint::kAuto:
    return "Auto";
  case wire::core::BundleSupportStyleHint::kCenterPreferred:
    return "CenterPreferred";
  case wire::core::BundleSupportStyleHint::kSideStructurePreferred:
    return "SideStructurePreferred";
  default:
    return "Unknown";
  }
}

const char* BundleBranchPolicyLabel(wire::core::BundleBranchPolicyHint policy) {
  switch (policy) {
  case wire::core::BundleBranchPolicyHint::kAuto:
    return "Auto";
  case wire::core::BundleBranchPolicyHint::kPreferPassThrough:
    return "PreferPassThrough";
  case wire::core::BundleBranchPolicyHint::kPreferExplicitBranch:
    return "PreferExplicitBranch";
  default:
    return "Unknown";
  }
}

const char* AttachmentLineInteractionModeLabel(wire::core::AttachmentLineInteractionMode mode) {
  switch (mode) {
  case wire::core::AttachmentLineInteractionMode::kPassThrough:
    return "PassThrough";
  case wire::core::AttachmentLineInteractionMode::kReplaceWithInternalPath:
    return "ReplaceWithInternalPath";
  case wire::core::AttachmentLineInteractionMode::kHideSegment:
    return "HideSegment";
  case wire::core::AttachmentLineInteractionMode::kAddInternalPath:
    return "AddInternalPath";
  default:
    return "Unknown";
  }
}

const char* BackboneFlowKindLabel(wire::core::BackboneFlowKind kind) {
  switch (kind) {
  case wire::core::BackboneFlowKind::kMain:
    return "Main";
  case wire::core::BackboneFlowKind::kBranch:
    return "Branch";
  default:
    return "Unknown";
  }
}

const char* BackboneLoweringKindLabel(wire::core::BackboneLoweringKind kind) {
  switch (kind) {
  case wire::core::BackboneLoweringKind::kNone:
    return "None";
  case wire::core::BackboneLoweringKind::kBranchSupport:
    return "BranchSupport";
  case wire::core::BackboneLoweringKind::kCrossUnderpass:
    return "CrossUnderpass";
  case wire::core::BackboneLoweringKind::kAcuteCorner:
    return "AcuteCorner";
  default:
    return "Unknown";
  }
}

const char* OrderDecisionChoiceLabel(wire::core::OrderDecisionChoiceKind choice) {
  switch (choice) {
  case wire::core::OrderDecisionChoiceKind::kNormal:
    return "Normal";
  case wire::core::OrderDecisionChoiceKind::kReversed:
    return "Reversed";
  default:
    return "Unknown";
  }
}

const char* OrderDecisionChoiceReasonLabel(wire::core::OrderDecisionChoiceReason reason) {
  switch (reason) {
  case wire::core::OrderDecisionChoiceReason::kFixedOrder:
    return "FixedOrder";
  case wire::core::OrderDecisionChoiceReason::kCrossingFewer:
    return "CrossingFewer";
  case wire::core::OrderDecisionChoiceReason::kSpacingBetter:
    return "SpacingBetter";
  case wire::core::OrderDecisionChoiceReason::kTwistSmaller:
    return "TwistSmaller";
  case wire::core::OrderDecisionChoiceReason::kKeptDefault:
    return "KeptDefault";
  default:
    return "Unknown";
  }
}

const char* BackboneFlowDecisionRuleLabel(wire::core::BackboneFlowDecisionRule rule) {
  switch (rule) {
  case wire::core::BackboneFlowDecisionRule::kDefaultMain:
    return "DefaultMain";
  case wire::core::BackboneFlowDecisionRule::kJunctionOrderMain:
    return "JunctionOrderMain";
  case wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch:
    return "JunctionOrderBranch";
  default:
    return "Unknown";
  }
}

const char* JunctionRelationKindLabel(wire::core::JunctionRelationKind kind) {
  switch (kind) {
  case wire::core::JunctionRelationKind::kNone:
    return "None";
  case wire::core::JunctionRelationKind::kThroughMain:
    return "ThroughMain";
  case wire::core::JunctionRelationKind::kSideBranch:
    return "SideBranch";
  case wire::core::JunctionRelationKind::kCornerContinuation:
    return "CornerContinuation";
  case wire::core::JunctionRelationKind::kCrossUnderpass:
    return "CrossUnderpass";
  default:
    return "Unknown";
  }
}

const char* ContinuityCategoryClassLabel(wire::core::ContinuityCategoryClass continuity_class) {
  switch (continuity_class) {
  case wire::core::ContinuityCategoryClass::kPointLike:
    return "PointLike";
  case wire::core::ContinuityCategoryClass::kBundleLike:
    return "BundleLike";
  default:
    return "Unknown";
  }
}

const char* SameLevelFeasibilityReasonLabel(wire::core::SameLevelFeasibilityReason reason) {
  switch (reason) {
  case wire::core::SameLevelFeasibilityReason::kNone:
    return "None";
  case wire::core::SameLevelFeasibilityReason::kBundleRule:
    return "BundleRule";
  case wire::core::SameLevelFeasibilityReason::kEnvelopeOverlap:
    return "EnvelopeOverlap";
  case wire::core::SameLevelFeasibilityReason::kNearNodeClearance:
    return "NearNodeClearance";
  case wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled:
    return "CategoryPolicyDisabled";
  default:
    return "Unknown";
  }
}

const char* OrderDecisionPolicyLabel(wire::core::OrderDecisionPolicyKind policy) {
  switch (policy) {
  case wire::core::OrderDecisionPolicyKind::kFixedOrder:
    return "FixedOrder";
  case wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous:
    return "PermutableHomogeneous";
  default:
    return "Unknown";
  }
}

const char* LateralSideChoiceLabel(wire::core::LateralSideChoiceKind choice) {
  switch (choice) {
  case wire::core::LateralSideChoiceKind::kCenter:
    return "Center";
  case wire::core::LateralSideChoiceKind::kLeft:
    return "Left";
  case wire::core::LateralSideChoiceKind::kRight:
    return "Right";
  default:
    return "Unknown";
  }
}

const char* SideAssignmentRuleKindLabel(wire::core::SideAssignmentRuleKind rule) {
  switch (rule) {
  case wire::core::SideAssignmentRuleKind::kPoleLocal:
    return "PoleLocal";
  case wire::core::SideAssignmentRuleKind::kChord:
    return "Chord";
  case wire::core::SideAssignmentRuleKind::kThroughPairNormal:
    return "ThroughPairNormal";
  case wire::core::SideAssignmentRuleKind::kBisector:
    return "Bisector";
  default:
    return "Unknown";
  }
}

const char* SupportOrientationRuleKindLabel(wire::core::SupportOrientationRuleKind rule) {
  switch (rule) {
  case wire::core::SupportOrientationRuleKind::kRadial:
    return "Radial";
  case wire::core::SupportOrientationRuleKind::kChord:
    return "Chord";
  case wire::core::SupportOrientationRuleKind::kThroughPairNormal:
    return "ThroughPairNormal";
  case wire::core::SupportOrientationRuleKind::kBisector:
    return "Bisector";
  default:
    return "Unknown";
  }
}

const char* SupportOrientationBasisKindLabel(wire::core::SupportOrientationBasisKind basis) {
  switch (basis) {
  case wire::core::SupportOrientationBasisKind::kRadial:
    return "Radial";
  case wire::core::SupportOrientationBasisKind::kChordForward:
    return "ChordForward";
  case wire::core::SupportOrientationBasisKind::kChordReverse:
    return "ChordReverse";
  case wire::core::SupportOrientationBasisKind::kBisectorForward:
    return "BisectorForward";
  case wire::core::SupportOrientationBasisKind::kBisectorReverse:
    return "BisectorReverse";
  case wire::core::SupportOrientationBasisKind::kPairNormalPositive:
    return "PairNormalPositive";
  case wire::core::SupportOrientationBasisKind::kPairNormalNegative:
    return "PairNormalNegative";
  default:
    return "Unknown";
  }
}

const char* SupportGroupingRuleKindLabel(wire::core::SupportGroupingRuleKind rule) {
  switch (rule) {
  case wire::core::SupportGroupingRuleKind::kPerPort:
    return "PerPort";
  case wire::core::SupportGroupingRuleKind::kDecisionGroup:
    return "DecisionGroup";
  default:
    return "Unknown";
  }
}

wire::core::BundleKind BundleTemplateForCategory(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return wire::core::BundleKind::kHighVoltage;
  case wire::core::ConnectionCategory::kCommunication:
    return wire::core::BundleKind::kCommunication;
  case wire::core::ConnectionCategory::kOptical:
    return wire::core::BundleKind::kOptical;
  case wire::core::ConnectionCategory::kDrop:
    return wire::core::BundleKind::kDrop;
  case wire::core::ConnectionCategory::kLowVoltage:
  default:
    return wire::core::BundleKind::kLowVoltage;
  }
}

const char* PoleForwardRuleLabel(wire::core::PoleForwardRule rule) {
  switch (rule) {
  case wire::core::PoleForwardRule::kFallback:
    return "Fallback";
  case wire::core::PoleForwardRule::kPrimaryIncident:
    return "PrimaryIncident";
  case wire::core::PoleForwardRule::kMainChainSingle:
    return "MainChainSingle";
  case wire::core::PoleForwardRule::kMainChainBisector:
    return "MainChainBisector";
  default:
    return "Unknown";
  }
}

const char* PoleSupportAxisRuleLabel(wire::core::PoleSupportAxisRule rule) {
  switch (rule) {
  case wire::core::PoleSupportAxisRule::kFallback:
    return "Fallback";
  case wire::core::PoleSupportAxisRule::kPrimaryIncident:
    return "PrimaryIncident";
  case wire::core::PoleSupportAxisRule::kMainChainSingle:
    return "MainChainSingle";
  case wire::core::PoleSupportAxisRule::kMainChainPair:
    return "MainChainPair";
  default:
    return "Unknown";
  }
}

std::vector<wire::core::CableTemplateId> SortedCableTemplateIds(const wire::core::CoreView& view) {
  std::vector<wire::core::CableTemplateId> ids;
  ids.reserve(view.cable_templates().size());
  for (const auto& [id, _] : view.cable_templates()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

void LoadPoleTemplateState(const wire::core::CoreView& view, ViewerUiState& ui_state, wire::core::PoleTypeId id);
wire::core::PoleTypeId SuggestedPoleTypeForBundleCategory(const wire::core::CoreView& view,
                                                          wire::core::ConnectionCategory category);

const wire::core::CableSupplementalPathTemplate* FindCurveOffsetStraightSupplemental(
    const wire::core::CableTemplate& tpl) {
  for (const auto& supplemental : tpl.supplemental_paths) {
    if (supplemental.anchor_mode == wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
        supplemental.profile_kind == wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable) {
      return &supplemental;
    }
  }
  return nullptr;
}

void LoadCableTemplateState(const wire::core::CoreView& view, ViewerUiState& ui_state, wire::core::CableTemplateId id) {
  const auto it = view.cable_templates().find(id);
  if (it == view.cable_templates().end()) {
    return;
  }
  ui_state.selected_cable_template_id = id;
  ui_state.cable_template_name = it->second.name;
  ui_state.cable_outer_diameter = it->second.outer_diameter_m;
  ui_state.cable_bend_stiffness = it->second.bend_stiffness;
  ui_state.cable_min_bend_radius = it->second.min_bend_radius_m;
  ui_state.cable_material_style = static_cast<int>(it->second.material_style);
  ui_state.cable_requires_insulator = it->second.requires_insulator;
  ui_state.cable_insulator_attachment_height = it->second.insulator_attachment_height_m;
  ui_state.cable_sag_factor = it->second.sag_factor;
  ui_state.cable_slack_factor = it->second.slack_factor;
  ui_state.cable_default_grouped_support_fanout_spacing = it->second.default_grouped_support_fanout_spacing_m;
  ui_state.cable_continuity_policy = static_cast<int>(it->second.continuity_policy);
  if (const auto* supplemental = FindCurveOffsetStraightSupplemental(it->second); supplemental != nullptr) {
    ui_state.cable_curve_offset_straight_supplemental_enabled = true;
    ui_state.cable_curve_offset_straight_lateral_offset = supplemental->lateral_offset_m;
    ui_state.cable_curve_offset_straight_vertical_offset = supplemental->vertical_offset_m;
    ui_state.cable_curve_offset_straight_wobble_amplitude = supplemental->wobble_amplitude_m;
    ui_state.cable_curve_offset_straight_wobble_wavelength = supplemental->wobble_wavelength_m;
    ui_state.cable_curve_offset_straight_wobble_phase_bias = supplemental->wobble_phase_bias;
    ui_state.cable_curve_offset_straight_endpoint_envelope_ratio = supplemental->endpoint_envelope_ratio;
  } else {
    ui_state.cable_curve_offset_straight_supplemental_enabled = false;
    ui_state.cable_curve_offset_straight_lateral_offset = 0.0;
    ui_state.cable_curve_offset_straight_vertical_offset = 0.0;
    ui_state.cable_curve_offset_straight_wobble_amplitude = 0.0;
    ui_state.cable_curve_offset_straight_wobble_wavelength = 0.0;
    ui_state.cable_curve_offset_straight_wobble_phase_bias = 0.0;
    ui_state.cable_curve_offset_straight_endpoint_envelope_ratio = 0.2;
  }
}

void LoadBundleTemplateState(const wire::core::CoreView& view, ViewerUiState& ui_state, wire::core::BundleKind id) {
  const auto it = view.bundle_templates().find(id);
  if (it == view.bundle_templates().end()) {
    return;
  }
  ui_state.selected_bundle_template_id = id;
  ui_state.bundle_template_cable_template_id = it->second.cable_template_id;
  ui_state.bundle_template_default_layer = static_cast<int>(it->second.default_layer);
  ui_state.bundle_template_allow_mirror = it->second.allow_mirror;
  ui_state.bundle_template_allow_midair_node = it->second.allow_midair_node;
  ui_state.bundle_template_allow_midair_branch = it->second.allow_midair_branch;
  ui_state.bundle_template_support_style = static_cast<int>(it->second.support_style);
  ui_state.bundle_template_branch_policy = static_cast<int>(it->second.branch_policy);
  ui_state.bundle_template_continuity_policy = static_cast<int>(it->second.continuity_policy);
  ui_state.bundle_template_grouped_support_fanout_spacing = it->second.grouped_support_fanout_spacing_m;
  LoadCableTemplateState(view, ui_state, it->second.cable_template_id);
  if (it->second.related_pole_type_id != wire::core::kInvalidPoleTypeId) {
    LoadPoleTemplateState(view, ui_state, it->second.related_pole_type_id);
  } else if (const wire::core::PoleTypeId suggested_id = SuggestedPoleTypeForBundleCategory(view, it->second.category);
             suggested_id != wire::core::kInvalidPoleTypeId) {
    LoadPoleTemplateState(view, ui_state, suggested_id);
  }
}

const char* BandOverflowPolicyLabel(wire::core::BandOverflowPolicy policy) {
  switch (policy) {
  case wire::core::BandOverflowPolicy::kTrySiblingBand:
    return "TrySiblingBand";
  case wire::core::BandOverflowPolicy::kRaiseHeight:
    return "RaiseHeight";
  case wire::core::BandOverflowPolicy::kConstrainedFallback:
    return "ConstrainedFallback";
  default:
    return "Unknown";
  }
}

const char* AnchorSupportKindLabel(wire::core::AnchorSupportKind kind) {
  switch (kind) {
  case wire::core::AnchorSupportKind::kGeneric:
    return "Generic";
  case wire::core::AnchorSupportKind::kGround:
    return "Ground";
  case wire::core::AnchorSupportKind::kBuilding:
    return "Building";
  case wire::core::AnchorSupportKind::kMidair:
    return "Midair";
  default:
    return "Unknown";
  }
}

bool InputTextString(const char* label, std::string* value) {
  if (value == nullptr) {
    return false;
  }
  std::vector<char> buffer(value->begin(), value->end());
  buffer.resize(std::max<std::size_t>(256, buffer.size() + 64), '\0');
  const bool changed = ImGui::InputText(label, buffer.data(), buffer.size());
  if (changed) {
    *value = buffer.data();
  }
  return changed;
}

std::vector<wire::core::PoleTypeId> SortedPoleTypeIds(const wire::core::CoreView& view) {
  std::vector<wire::core::PoleTypeId> ids;
  ids.reserve(view.pole_types().size());
  for (const auto& [id, _] : view.pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::optional<wire::core::CableTemplateId> FindCableTemplateIdByName(const wire::core::CoreView& view,
                                                                     const std::string& name) {
  for (const auto& [id, tpl] : view.cable_templates()) {
    if (tpl.name == name) {
      return id;
    }
  }
  return std::nullopt;
}

std::optional<wire::core::PoleTypeId> FindPoleTypeIdByName(const wire::core::CoreView& view, const std::string& name) {
  for (const auto& [id, pole_type] : view.pole_types()) {
    if (pole_type.name == name) {
      return id;
    }
  }
  return std::nullopt;
}

void ApplyStartupCableEditorDefaults(ViewerUiState& ui_state) {
  ui_state.cable_template_name = "HV_BARE";
  ui_state.cable_outer_diameter = 0.048;
  ui_state.cable_bend_stiffness = 2.8;
  ui_state.cable_min_bend_radius = 0.7;
  ui_state.cable_material_style = static_cast<int>(wire::core::CableMaterialStyleKind::kBareConductor);
  ui_state.cable_requires_insulator = true;
  ui_state.cable_insulator_attachment_height = 0.145;
  ui_state.cable_sag_factor = 0.045;
  ui_state.cable_slack_factor = 0.025;
  ui_state.cable_default_grouped_support_fanout_spacing = 0.35;
  ui_state.cable_continuity_policy = static_cast<int>(wire::core::CableContinuityPolicyHint::kPreferG1);
  ui_state.cable_curve_offset_straight_supplemental_enabled = true;
  ui_state.cable_curve_offset_straight_lateral_offset = 0.0;
  ui_state.cable_curve_offset_straight_vertical_offset = 0.0;
  ui_state.cable_curve_offset_straight_wobble_amplitude = 0.0;
  ui_state.cable_curve_offset_straight_wobble_wavelength = 0.0;
  ui_state.cable_curve_offset_straight_wobble_phase_bias = 0.0;
  ui_state.cable_curve_offset_straight_endpoint_envelope_ratio = 0.2;
}

void SyncDrawPathPoleTypeSelection(const wire::core::CoreView& view, ViewerUiState& ui_state, wire::core::PoleTypeId id) {
  const auto type_ids = SortedPoleTypeIds(view);
  for (std::size_t i = 0; i < type_ids.size(); ++i) {
    if (type_ids[i] == id) {
      ui_state.road_pole_type_index = static_cast<int>(i);
      return;
    }
  }
}

void LoadPoleTemplateState(const wire::core::CoreView& view, ViewerUiState& ui_state, wire::core::PoleTypeId id) {
  const auto it = view.pole_types().find(id);
  if (it == view.pole_types().end()) {
    return;
  }
  ui_state.selected_pole_template_id = id;
  ui_state.pole_template_edit = it->second;
  ui_state.pole_template_loaded = true;
  SyncDrawPathPoleTypeSelection(view, ui_state, id);
}

wire::core::PoleTypeId SuggestedPoleTypeForBundleCategory(const wire::core::CoreView& view,
                                                          wire::core::ConnectionCategory category) {
  wire::core::PoleTypeId best_id = wire::core::kInvalidPoleTypeId;
  int best_score = -1;
  for (const auto& [id, pole_type] : view.pole_types()) {
    int score = 0;
    for (const auto& band : pole_type.port_bands) {
      if (band.enabled && band.category == category) {
        score += 1;
      }
    }
    if (score > best_score || (score == best_score && best_id == wire::core::kInvalidPoleTypeId)) {
      best_score = score;
      best_id = id;
    }
  }
  return best_id;
}

int EditablePolePlacementLayerForCategory(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return 2;
  case wire::core::ConnectionCategory::kLowVoltage:
  case wire::core::ConnectionCategory::kCommunication:
  case wire::core::ConnectionCategory::kOptical:
    return 1;
  case wire::core::ConnectionCategory::kDrop:
    return 0;
  default:
    return 1;
  }
}

bool CategoryHasEditableTargetLayerBands(const wire::core::PoleTypeDefinition& pole_type,
                                         wire::core::ConnectionCategory category) {
  const int target_layer = EditablePolePlacementLayerForCategory(category);
  return std::any_of(pole_type.port_bands.begin(), pole_type.port_bands.end(), [&](const auto& band) {
    return band.enabled && band.category == category && band.layer == target_layer;
  });
}

bool BandMatchesEditableCategoryScope(const wire::core::PoleTypeDefinition& pole_type,
                                      const wire::core::PortPlacementBand& band,
                                      wire::core::ConnectionCategory category) {
  return band.enabled && band.category == category &&
         (!CategoryHasEditableTargetLayerBands(pole_type, category) ||
          band.layer == EditablePolePlacementLayerForCategory(category));
}

double AverageBandHeightForCategory(const wire::core::PoleTypeDefinition& pole_type,
                                    wire::core::ConnectionCategory category,
                                    double fallback_m) {
  double total = 0.0;
  int count = 0;
  for (const auto& band : pole_type.port_bands) {
    if (!BandMatchesEditableCategoryScope(pole_type, band, category)) {
      continue;
    }
    total += band.height_center_m;
    ++count;
  }
  return count > 0 ? total / static_cast<double>(count) : fallback_m;
}

double AverageBandLateralForCategory(const wire::core::PoleTypeDefinition& pole_type,
                                     wire::core::ConnectionCategory category,
                                     double fallback_m) {
  double total = 0.0;
  int count = 0;
  for (const auto& band : pole_type.port_bands) {
    if (!BandMatchesEditableCategoryScope(pole_type, band, category)) {
      continue;
    }
    total += band.lateral_center_m;
    ++count;
  }
  return count > 0 ? total / static_cast<double>(count) : fallback_m;
}

double MaxBandLateralSpreadForCategory(const wire::core::PoleTypeDefinition& pole_type,
                                       wire::core::ConnectionCategory category,
                                       double center_m) {
  double spread = 0.0;
  for (const auto& band : pole_type.port_bands) {
    if (!BandMatchesEditableCategoryScope(pole_type, band, category)) {
      continue;
    }
    spread = std::max(spread, std::abs(band.lateral_center_m - center_m));
  }
  return spread;
}

void SetBandCategoryHeights(wire::core::PoleTypeDefinition* pole_type,
                            wire::core::ConnectionCategory category,
                            double target_center_m) {
  if (pole_type == nullptr) {
    return;
  }
  for (auto& band : pole_type->port_bands) {
    if (!BandMatchesEditableCategoryScope(*pole_type, band, category)) {
      continue;
    }
    const double half_range = std::max(0.0, 0.5 * (band.height_max_m - band.height_min_m));
    band.height_center_m = target_center_m;
    band.height_min_m = target_center_m - half_range;
    band.height_max_m = target_center_m + half_range;
  }
}

void SetBandCategoryLateralCenter(wire::core::PoleTypeDefinition* pole_type,
                                  wire::core::ConnectionCategory category,
                                  double target_center_m) {
  if (pole_type == nullptr) {
    return;
  }
  const double current_center_m = AverageBandLateralForCategory(*pole_type, category, 0.0);
  const double delta = target_center_m - current_center_m;
  for (auto& band : pole_type->port_bands) {
    if (!BandMatchesEditableCategoryScope(*pole_type, band, category)) {
      continue;
    }
    band.lateral_center_m += delta;
    band.lateral_min_m += delta;
    band.lateral_max_m += delta;
  }
}

void SetBandCategorySpread(wire::core::PoleTypeDefinition* pole_type,
                           wire::core::ConnectionCategory category,
                           double target_spread_m) {
  if (pole_type == nullptr) {
    return;
  }
  const double center_m = AverageBandLateralForCategory(*pole_type, category, 0.0);
  const double current_spread_m = MaxBandLateralSpreadForCategory(*pole_type, category, center_m);
  const double scale = (current_spread_m <= 1e-9) ? 1.0 : (target_spread_m / current_spread_m);
  for (auto& band : pole_type->port_bands) {
    if (!BandMatchesEditableCategoryScope(*pole_type, band, category)) {
      continue;
    }
    const double half_range = std::max(0.0, 0.5 * (band.lateral_max_m - band.lateral_min_m));
    const double deviation = band.lateral_center_m - center_m;
    band.lateral_center_m = center_m + deviation * scale;
    band.lateral_min_m = band.lateral_center_m - half_range;
    band.lateral_max_m = band.lateral_center_m + half_range;
  }
}

bool PoleTypeHasCategory(const wire::core::PoleTypeDefinition& pole_type, wire::core::ConnectionCategory category) {
  return std::any_of(pole_type.port_bands.begin(), pole_type.port_bands.end(), [&](const auto& band) {
    return band.enabled && band.category == category;
  });
}

void DrawObjectList(ViewerUiState& ui_state, const char* header, SelectedType type, const std::vector<ObjectId>& ids,
                    std::function<std::string(ObjectId)> make_label) {
  if (!ImGui::CollapsingHeader(header, ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }

  for (ObjectId id : ids) {
    const std::string label = make_label(id);
    const bool is_selected = SelectionContains(ui_state, type, id);
    if (ImGui::Selectable(label.c_str(), is_selected)) {
      SetPrimarySelection(ui_state, type, id);
    }
  }
}

enum class PoleHeightMarkerKind : std::uint8_t {
  kPort = 0,
  kSupport = 1,
  kBundle = 2,
};

struct PoleHeightMarker {
  PoleHeightMarkerKind kind = PoleHeightMarkerKind::kPort;
  std::string label{};
  double height_m = 0.0;
  double x_escape_m = 0.0;
  std::vector<ObjectId> port_ids{};
  bool editable = false;
  bool warning = false;
};

double MarkerBaseOffsetForKind(PoleHeightMarkerKind kind) {
  switch (kind) {
  case PoleHeightMarkerKind::kPort:
    return 0.08;
  case PoleHeightMarkerKind::kSupport:
    return 0.22;
  case PoleHeightMarkerKind::kBundle:
    return 0.40;
  default:
    return 0.08;
  }
}

ImU32 MarkerColorForKind(PoleHeightMarkerKind kind, bool warning) {
  if (warning) {
    return IM_COL32(214, 92, 72, 255);
  }
  switch (kind) {
  case PoleHeightMarkerKind::kPort:
    return IM_COL32(212, 148, 78, 255);
  case PoleHeightMarkerKind::kSupport:
    return IM_COL32(92, 148, 188, 255);
  case PoleHeightMarkerKind::kBundle:
    return IM_COL32(92, 176, 124, 255);
  default:
    return IM_COL32(180, 180, 180, 255);
  }
}

double AveragePortHeight(const std::vector<const wire::core::Port*>& ports) {
  if (ports.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const wire::core::Port* port : ports) {
    sum += port->world_position.z;
  }
  return sum / static_cast<double>(ports.size());
}

bool NearlySameWorld(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps) {
  return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps && std::abs(a.z - b.z) <= eps;
}

std::vector<PoleHeightMarker> BuildPoleHeightMarkers(const wire::core::CoreView& view,
                                                     const wire::core::Pole& pole, const ViewerUiState& ui_state) {
  std::vector<PoleHeightMarker> markers{};
  std::vector<const wire::core::Port*> owned_ports{};
  if (const auto ports_it = view.relation_index().ports_by_pole.find(pole.id);
      ports_it != view.relation_index().ports_by_pole.end()) {
    owned_ports.reserve(ports_it->second.size());
    for (ObjectId port_id : ports_it->second) {
      if (const wire::core::Port* port = view.ports().find(port_id); port != nullptr) {
        owned_ports.push_back(port);
      }
    }
  }
  std::sort(owned_ports.begin(), owned_ports.end(),
            [](const wire::core::Port* a, const wire::core::Port* b) { return a->world_position.z > b->world_position.z; });

  if (ui_state.pole_height_view_show_ports) {
    for (std::size_t i = 0; i < owned_ports.size(); ++i) {
      const wire::core::Port* port = owned_ports[i];
      if (port == nullptr) {
        continue;
      }
      PoleHeightMarker marker{};
      marker.kind = PoleHeightMarkerKind::kPort;
      marker.label = "Port " + port->display_id;
      marker.height_m = port->world_position.z;
      marker.x_escape_m =
          view.pole_radius_at_height_m(pole, std::max(0.0, marker.height_m - pole.world_transform.position.z)) +
          view.geometry_settings().pole_clearance_m + MarkerBaseOffsetForKind(marker.kind) + static_cast<double>(i) * 0.05;
      marker.port_ids.push_back(port->id);
      marker.editable = true;
      markers.push_back(std::move(marker));
    }
  }

  if (ui_state.pole_height_view_show_supports) {
    std::size_t support_index = 0;
    for (const wire::core::Port* port : owned_ports) {
      if (port == nullptr) {
        continue;
      }
      PoleHeightMarker marker{};
      marker.kind = PoleHeightMarkerKind::kSupport;
      marker.label = "Support " + port->display_id;
      marker.height_m = port->world_position.z;
      marker.x_escape_m =
          view.pole_radius_at_height_m(pole, std::max(0.0, marker.height_m - pole.world_transform.position.z)) +
          view.geometry_settings().pole_clearance_m + MarkerBaseOffsetForKind(marker.kind) +
          static_cast<double>(support_index++) * 0.05;
      marker.port_ids.push_back(port->id);
      marker.editable = true;
      markers.push_back(std::move(marker));
    }

  }

  if (ui_state.pole_height_view_show_bundles) {
    std::vector<ObjectId> bundle_ids{};
    std::vector<std::vector<const wire::core::Port*>> bundle_ports{};
    for (const wire::core::Port* port : owned_ports) {
      if (port == nullptr) {
        continue;
      }
      const auto spans_it = view.connection_index().spans_by_port.find(port->id);
      if (spans_it == view.connection_index().spans_by_port.end()) {
        continue;
      }
      for (ObjectId span_id : spans_it->second) {
        const wire::core::Span* span = view.spans().find(span_id);
        if (span == nullptr || span->bundle_id == wire::core::kInvalidObjectId) {
          continue;
        }
        auto existing = std::find(bundle_ids.begin(), bundle_ids.end(), span->bundle_id);
        if (existing == bundle_ids.end()) {
          bundle_ids.push_back(span->bundle_id);
          bundle_ports.push_back({});
          existing = std::prev(bundle_ids.end());
        }
        const std::size_t index = static_cast<std::size_t>(std::distance(bundle_ids.begin(), existing));
        if (std::find(bundle_ports[index].begin(), bundle_ports[index].end(), port) == bundle_ports[index].end()) {
          bundle_ports[index].push_back(port);
        }
      }
    }
    for (std::size_t i = 0; i < bundle_ids.size(); ++i) {
      const wire::core::Bundle* bundle = view.bundles().find(bundle_ids[i]);
      if (bundle == nullptr || bundle_ports[i].empty()) {
        continue;
      }
      PoleHeightMarker marker{};
      marker.kind = PoleHeightMarkerKind::kBundle;
      marker.label = "Bundle " + bundle->display_id;
      marker.height_m = AveragePortHeight(bundle_ports[i]);
      marker.x_escape_m =
          view.pole_radius_at_height_m(pole, std::max(0.0, marker.height_m - pole.world_transform.position.z)) +
          view.geometry_settings().pole_clearance_m + MarkerBaseOffsetForKind(marker.kind) + static_cast<double>(i) * 0.05;
      for (const wire::core::Port* port : bundle_ports[i]) {
        marker.port_ids.push_back(port->id);
      }
      marker.editable = !marker.port_ids.empty();
      markers.push_back(std::move(marker));
    }
  }

  constexpr double kWarningGapM = 0.25;
  for (std::size_t i = 0; i < markers.size(); ++i) {
    for (std::size_t j = i + 1; j < markers.size(); ++j) {
      if (std::abs(markers[i].height_m - markers[j].height_m) < kWarningGapM) {
        markers[i].warning = true;
        markers[j].warning = true;
      }
    }
  }
  return markers;
}

void ApplyPoleHeightMarkerDelta(CoreState& state, const PoleHeightMarker& marker, double new_height_m) {
  if (!marker.editable || marker.port_ids.empty()) {
    return;
  }
  const auto view = viewer_core_state::View(state);
  std::vector<const wire::core::Port*> ports{};
  for (ObjectId port_id : marker.port_ids) {
    if (const wire::core::Port* port = view.ports().find(port_id); port != nullptr) {
      ports.push_back(port);
    }
  }
  if (ports.empty()) {
    return;
  }
  const double current_height = (marker.kind == PoleHeightMarkerKind::kBundle) ? AveragePortHeight(ports) : ports.front()->world_position.z;
  const double delta = new_height_m - current_height;
  if (std::abs(delta) <= 1e-6) {
    return;
  }
  for (const wire::core::Port* port : ports) {
    wire::core::Vec3d world = port->world_position;
    world.z += delta;
    (void)viewer_core_state::SetPortWorldPositionManual(state, port->id, world);
  }
}

void DrawPoleHeightDebugView(CoreState& state, ViewerUiState& ui_state, const wire::core::Pole& pole) {
  if (!ImGui::CollapsingHeader("Pole Height Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }
  const auto view = viewer_core_state::View(state);

  ImGui::Checkbox("Show Ports", &ui_state.pole_height_view_show_ports);
  ImGui::SameLine();
  ImGui::Checkbox("Show Supports", &ui_state.pole_height_view_show_supports);
  ImGui::SameLine();
  ImGui::Checkbox("Show Bundles", &ui_state.pole_height_view_show_bundles);
  ImGui::TextUnformatted("Height only. X is auto-resolved from pole radius + clearance.");

  const std::vector<PoleHeightMarker> markers = BuildPoleHeightMarkers(view, pole, ui_state);
  const float canvas_width = std::max(280.0f, ImGui::GetContentRegionAvail().x);
  const float canvas_height = 260.0f;
  const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("##pole_height_canvas", ImVec2(canvas_width, canvas_height));
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 canvas_min = canvas_pos;
  const ImVec2 canvas_max = ImVec2(canvas_pos.x + canvas_width, canvas_pos.y + canvas_height);
  draw_list->AddRectFilled(canvas_min, canvas_max, IM_COL32(20, 22, 25, 255), 4.0f);
  draw_list->AddRect(canvas_min, canvas_max, IM_COL32(74, 78, 84, 255), 4.0f);

  double min_height = pole.world_transform.position.z;
  double max_height = pole.world_transform.position.z + pole.height_m;
  for (const PoleHeightMarker& marker : markers) {
    min_height = std::min(min_height, marker.height_m);
    max_height = std::max(max_height, marker.height_m);
  }
  min_height -= 0.4;
  max_height += 0.4;
  const double height_span = std::max(1.0, max_height - min_height);
  const float line_x = canvas_min.x + 56.0f;
  draw_list->AddLine(ImVec2(line_x, canvas_min.y + 14.0f), ImVec2(line_x, canvas_max.y - 14.0f), IM_COL32(180, 184, 190, 255),
                     1.5f);

  auto height_to_y = [&](double height) -> float {
    const double t = (height - min_height) / height_span;
    return canvas_max.y - 16.0f - static_cast<float>(t) * (canvas_height - 32.0f);
  };

  for (int i = 0; i <= 4; ++i) {
    const double tick_height = min_height + height_span * (static_cast<double>(i) / 4.0);
    const float y = height_to_y(tick_height);
    draw_list->AddLine(ImVec2(canvas_min.x + 8.0f, y), ImVec2(canvas_max.x - 8.0f, y), IM_COL32(48, 52, 58, 255), 1.0f);
    char text[32];
    std::snprintf(text, sizeof(text), "%.2f", tick_height);
    draw_list->AddText(ImVec2(canvas_min.x + 10.0f, y - 8.0f), IM_COL32(188, 192, 198, 255), text);
  }

  for (std::size_t i = 0; i < markers.size(); ++i) {
    const PoleHeightMarker& marker = markers[i];
    const float y = height_to_y(marker.height_m);
    const float x = line_x + static_cast<float>(marker.x_escape_m * 90.0);
    const ImU32 color = MarkerColorForKind(marker.kind, marker.warning);
    draw_list->AddLine(ImVec2(line_x, y), ImVec2(x, y), IM_COL32(110, 118, 128, 180), 1.0f);
    draw_list->AddCircleFilled(ImVec2(x, y), 5.0f, color);
    std::ostringstream label;
    label.setf(std::ios::fixed);
    label.precision(2);
    label << marker.label << " " << marker.height_m;
    if (marker.warning) {
      label << " !";
    }
    draw_list->AddText(ImVec2(x + 8.0f, y - 8.0f), color, label.str().c_str());

    ImGui::SetCursorScreenPos(ImVec2(x - 8.0f, y - 8.0f));
    ImGui::PushID(static_cast<int>(i));
    ImGui::InvisibleButton("##pole_height_marker", ImVec2(16.0f, 16.0f));
    if (marker.editable && ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      const float mouse_y = ImGui::GetIO().MousePos.y;
      const double t = std::clamp(static_cast<double>(canvas_max.y - 16.0f - mouse_y) / static_cast<double>(canvas_height - 32.0),
                                  0.0, 1.0);
      const double new_height = min_height + t * height_span;
      ApplyPoleHeightMarkerDelta(state, marker, new_height);
    }
    ImGui::PopID();
  }

  ImGui::Text("Markers: %d", static_cast<int>(markers.size()));
  ImGui::TextUnformatted("Red markers mean heights are closer than the debug clearance threshold.");
}

std::optional<wire::core::EntityRef> SelectedEntityRef(const ViewerUiState& ui_state) {
  using wire::core::EntityKind;
  switch (ui_state.selected_type) {
  case SelectedType::kPole:
    return wire::core::EntityRef{EntityKind::kPole, ui_state.selected_id};
  case SelectedType::kSpan:
    return wire::core::EntityRef{EntityKind::kSpan, ui_state.selected_id};
  case SelectedType::kSupportNode:
    return wire::core::EntityRef{EntityKind::kSupportNode, ui_state.selected_id};
  case SelectedType::kSupportLayout:
    return wire::core::EntityRef{EntityKind::kSupportLayout, ui_state.selected_id};
  case SelectedType::kDetailCurve:
    return wire::core::EntityRef{EntityKind::kDetailCurve, ui_state.selected_id};
  case SelectedType::kJunction:
    return wire::core::EntityRef{EntityKind::kJunction, ui_state.selected_id};
  default:
    return std::nullopt;
  }
}

void SelectFromEntityRef(ViewerUiState& ui_state, const wire::core::EntityRef& ref) {
  switch (ref.kind) {
  case wire::core::EntityKind::kPole:
    SetPrimarySelection(ui_state, SelectedType::kPole, static_cast<ObjectId>(ref.stable_id));
    break;
  case wire::core::EntityKind::kSpan:
    SetPrimarySelection(ui_state, SelectedType::kSpan, static_cast<ObjectId>(ref.stable_id));
    break;
  case wire::core::EntityKind::kSupportNode:
    SetPrimarySelection(ui_state, SelectedType::kSupportNode, static_cast<ObjectId>(ref.stable_id));
    break;
  case wire::core::EntityKind::kSupportLayout:
    SetPrimarySelection(ui_state, SelectedType::kSupportLayout, static_cast<ObjectId>(ref.stable_id));
    break;
  case wire::core::EntityKind::kDetailCurve:
    SetPrimarySelection(ui_state, SelectedType::kDetailCurve, static_cast<ObjectId>(ref.stable_id));
    break;
  case wire::core::EntityKind::kJunction:
    SetPrimarySelection(ui_state, SelectedType::kJunction, static_cast<ObjectId>(ref.stable_id));
    break;
  default:
    break;
  }
}

void DrawEntityMetaBlock(const wire::core::EntityMeta& meta) {
  ImGui::Separator();
  ImGui::TextUnformatted("Entity");
  ImGui::Text("kind: %s", EntityKindLabel(meta.ref.kind));
  ImGui::Text("stableId: %llu", static_cast<unsigned long long>(meta.ref.stable_id));
  ImGui::Text("display: %s", meta.display_name.c_str());
  ImGui::Text("role: %s", EntityRoleKindLabel(meta.role));
  ImGui::Text("editable: %s", meta.editable ? "true" : "false");
  ImGui::Text("overrideable: %s", meta.overrideable ? "true" : "false");
  ImGui::Text("provenance: %s", meta.provenance.c_str());
}

void DrawDecisionTraceBlock(const wire::core::CoreView& view, const wire::core::EntityRef& ref) {
  const auto trace = view.collect_decision_trace(ref);
  ImGui::Separator();
  ImGui::TextUnformatted("DecisionTrace");
  if (trace.empty()) {
    ImGui::TextUnformatted("(none)");
    return;
  }
  for (const auto& entry : trace) {
    ImGui::Text("[%s] %s", DecisionTraceTopicLabel(entry.topic), entry.rule.c_str());
    ImGui::TextWrapped("%s", entry.summary.c_str());
  }
}

void DrawStyleInspectionBlock(const wire::core::StyleInspectionView& style) {
  ImGui::Separator();
  ImGui::TextUnformatted("StyleContext");
  if (!style.has_context) {
    ImGui::TextUnformatted("(none)");
    return;
  }
  ImGui::Text("route: family=%llu bundle=%d category=%s flow=%s",
              static_cast<unsigned long long>(style.route_key.family_id),
              static_cast<int>(style.route_key.bundle_template_id), CategoryLabel(style.route_key.category),
              BackboneFlowKindLabel(style.route_key.flow_kind));
  ImGui::Text("object: segment=%u lane=%u kind=%s ordinal=%u start=%s", style.object_key.segment_index,
              style.object_key.lane_index, StyleObjectKindLabel(style.object_key.kind), style.object_key.ordinal,
              style.object_key.is_start_endpoint ? "true" : "false");
  ImGui::Text("scope: district=%llu route=%llu cluster=%llu object=%llu",
              static_cast<unsigned long long>(style.resolved.scope.district_seed),
              static_cast<unsigned long long>(style.resolved.scope.route_seed),
              static_cast<unsigned long long>(style.resolved.scope.cluster_seed),
              static_cast<unsigned long long>(style.resolved.scope.object_seed));
  ImGui::Text("profile: age=%.2f clutter=%.2f regularity=%.2f serviceMix=%.2f seed=%llu",
              style.resolved.profile.age, style.resolved.profile.clutter, style.resolved.profile.regularity,
              style.resolved.profile.service_mix, static_cast<unsigned long long>(style.resolved.profile.style_seed));
  ImGui::Text("district: cable=%s attachment=%s age=%.2f clutter=%.2f regularity=%.2f serviceMix=%.2f",
              CableMaterialStyleLabel(style.resolved.district.cable_family),
              CableAttachmentStyleLabel(style.resolved.district.attachment_family), style.resolved.district.age,
              style.resolved.district.clutter, style.resolved.district.regularity,
              style.resolved.district.service_mix);
  ImGui::Text("routeStyle: cable=%s attachment=%s age=%.3f clutter=%.3f regularity=%.3f serviceMix=%.3f sag=%.3f",
              CableMaterialStyleLabel(style.resolved.route.cable_family),
              CableAttachmentStyleLabel(style.resolved.route.attachment_family), style.resolved.route.age_bias,
              style.resolved.route.clutter_bias, style.resolved.route.regularity_bias,
              style.resolved.route.service_mix_bias, style.resolved.route.sag_bias);
  ImGui::Text("cluster: index=%u clutter=%.3f serviceMix=%.3f familyMix=%.3f",
              style.resolved.cluster.key.cluster_index, style.resolved.cluster.clutter_bias,
              style.resolved.cluster.service_mix_bias, style.resolved.cluster.family_mix);
  ImGui::Text("objectVariation: offset=%.3f %.3f %.3f sag=%.3f attachment=%.3f choice=%.3f",
              style.resolved.object.local_offset_m.x, style.resolved.object.local_offset_m.y,
              style.resolved.object.local_offset_m.z, style.resolved.object.sag_delta_m,
              style.resolved.object.attachment_offset_m, style.resolved.object.choice_bias);
}

void DrawRelatedLinks(ViewerUiState& ui_state, const std::vector<wire::core::RelatedEntityLink>& links) {
  ImGui::Separator();
  ImGui::TextUnformatted("Links");
  if (links.empty()) {
    ImGui::TextUnformatted("(none)");
    return;
  }
  for (const auto& link : links) {
    if (ImGui::SmallButton(link.label.c_str())) {
      SelectFromEntityRef(ui_state, link.ref);
    }
  }
}

void DrawTemplateViewBlock(const char* title, const std::optional<wire::core::TemplateInspectionView>& view_opt) {
  if (!view_opt.has_value()) {
    return;
  }
  ImGui::Separator();
  ImGui::TextUnformatted(title);
  ImGui::Text("name: %s", view_opt->meta.display_name.c_str());
  ImGui::Text("role: %s", EntityRoleKindLabel(view_opt->meta.role));
  for (const auto& property : view_opt->properties) {
    ImGui::Text("%s: %s", property.name.c_str(), property.value.c_str());
  }
}

void DrawOverrideViewBlock(const std::optional<wire::core::OverrideInspectionView>& view_opt) {
  if (!view_opt.has_value()) {
    return;
  }
  ImGui::Separator();
  ImGui::TextUnformatted("Override");
  for (const auto& entry : view_opt->entries) {
    ImGui::Text("%s: active=%s", entry.name.c_str(), entry.active ? "true" : "false");
    ImGui::Text("  auto=%s", entry.automatic_value.c_str());
    ImGui::Text("  override=%s", entry.override_value.c_str());
    ImGui::Text("  final=%s", entry.resolved_value.c_str());
  }
}

void DrawEndpointDecisionSummary(const char* label, const wire::core::SupportLayoutEndpointView& endpoint) {
  ImGui::Text("%s relation=%s class=%s lower=%s defaultLower=%s", label,
              JunctionRelationKindLabel(endpoint.relation_kind),
              ContinuityCategoryClassLabel(endpoint.continuity_class),
              endpoint.lower_required ? "true" : "false",
              endpoint.default_lower_required ? "true" : "false");
  ImGui::Text("  sameLevel=%s reason=%s blocked=%s unresolved=%s solver=%s specialPorts=%s",
              endpoint.same_level_feasible ? "true" : "false",
              SameLevelFeasibilityReasonLabel(endpoint.same_level_reason),
              endpoint.lowering_blocked_by_policy ? "true" : "false",
              endpoint.unresolved_same_level_conflict ? "true" : "false",
              endpoint.solver_used_same_level_constraint ? "true" : "false",
              endpoint.used_special_case_ports ? "true" : "false");
  ImGui::Text("  order=%s choice=%s(%s) side=%s sign=%.2f sideRule=%s pairSide=%s",
              OrderDecisionPolicyLabel(endpoint.order_decision_policy),
              OrderDecisionChoiceLabel(endpoint.order_decision_choice),
              OrderDecisionChoiceReasonLabel(endpoint.order_decision_choice_reason),
              LateralSideChoiceLabel(endpoint.chosen_side), endpoint.chosen_side_sign,
              SideAssignmentRuleKindLabel(endpoint.side_assignment_rule),
              endpoint.used_junction_pair_side_assignment ? "true" : "false");
  ImGui::Text("  orientation=%s basis=%s", 
              SupportOrientationRuleKindLabel(endpoint.support_orientation_rule),
              SupportOrientationBasisKindLabel(endpoint.support_orientation_basis));
  ImGui::Text("  spacing=%.2f clearance=%.2f throughPair=%s", endpoint.projected_spacing_topview_m,
              endpoint.required_clearance_m, endpoint.in_through_pair ? "true" : "false");

}
void DrawEndpointDecisionSummary(const char* label, const wire::core::LayoutEndpoint& endpoint) {
  ImGui::Text("%s relation=%s class=%s lower=%s defaultLower=%s", label,
              JunctionRelationKindLabel(endpoint.relation_kind),
              ContinuityCategoryClassLabel(endpoint.continuity_class),
              endpoint.lower_required ? "true" : "false",
              endpoint.default_lower_required ? "true" : "false");
  ImGui::Text("  sameLevel=%s reason=%s blocked=%s unresolved=%s solver=%s specialPorts=%s",
              endpoint.same_level_feasible ? "true" : "false",
              SameLevelFeasibilityReasonLabel(endpoint.same_level_reason),
              endpoint.lowering_blocked_by_policy ? "true" : "false",
              endpoint.unresolved_same_level_conflict ? "true" : "false",
              endpoint.solver_used_same_level_constraint ? "true" : "false",
              endpoint.used_special_case_ports ? "true" : "false");
  ImGui::Text("  order=%s choice=%s(%s) side=%s sign=%.2f sideRule=%s pairSide=%s",
              OrderDecisionPolicyLabel(endpoint.order_decision_policy),
              OrderDecisionChoiceLabel(endpoint.order_decision_choice),
              OrderDecisionChoiceReasonLabel(endpoint.order_decision_choice_reason),
              LateralSideChoiceLabel(endpoint.chosen_side), endpoint.chosen_side_sign,
              SideAssignmentRuleKindLabel(endpoint.side_assignment_rule),
              endpoint.used_junction_pair_side_assignment ? "true" : "false");
  ImGui::Text("  orientation=%s basis=%s",
              SupportOrientationRuleKindLabel(endpoint.support_orientation_rule),
              SupportOrientationBasisKindLabel(endpoint.support_orientation_basis));
  ImGui::Text("  spacing=%.2f clearance=%.2f throughPair=%s", endpoint.projected_spacing_topview_m,
              endpoint.required_clearance_m, endpoint.in_through_pair ? "true" : "false");
}

void DrawLoweredSupportGroupsBlock(const std::vector<wire::core::LoweredSupportGroupInspectionView>& groups) {
  ImGui::Separator();
  ImGui::Text("LoweredSupportGroups: %d", static_cast<int>(groups.size()));
  if (groups.empty()) {
    ImGui::TextUnformatted("(none)");
    return;
  }
  for (std::size_t index = 0; index < groups.size(); ++index) {
    const auto& group = groups[index];
    ImGui::Text("[%d] pole=%llu groupId=%d groupedPorts=%d rule=%s origin=%s", static_cast<int>(index),
                static_cast<unsigned long long>(group.owner_pole_id), group.support_group_id,
                group.grouped_port_count, SupportGroupingRuleKindLabel(group.grouping_rule), group.origin.c_str());
    ImGui::Text("  relation=%s class=%s lower=%s order=%s %s(%s)",
                JunctionRelationKindLabel(group.relation_kind),
                ContinuityCategoryClassLabel(group.continuity_class),
                group.lower_required ? "true" : "false",
                OrderDecisionPolicyLabel(group.order_decision_policy), OrderDecisionChoiceLabel(group.order_decision_choice),
                OrderDecisionChoiceReasonLabel(group.order_decision_choice_reason));
    ImGui::Text("  side=%s sign=%.2f sideRule=%s orient=%s basis=%s pairSide=%s down=%.2f",
                LateralSideChoiceLabel(group.chosen_side), group.chosen_side_sign,
                SideAssignmentRuleKindLabel(group.side_assignment_rule),
                SupportOrientationRuleKindLabel(group.support_orientation_rule),
                SupportOrientationBasisKindLabel(group.support_orientation_basis),
                group.used_junction_pair_side_assignment ? "true" : "false", group.down_offset_m);
  }
}

void DrawSelectedInfo(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Selected");
  ImGui::Separator();
  if (ui_state.selected_type == SelectedType::kNone || ui_state.selected_id == wire::core::kInvalidObjectId) {
    ImGui::TextUnformatted("None");
    return;
  }

  const auto view = viewer_core_state::View(state);
  const auto& edit = view.edit_state();
  if (const auto ref = SelectedEntityRef(ui_state); ref.has_value()) {
    if (const auto meta = view.describe_entity(*ref); meta.has_value()) {
      DrawEntityMetaBlock(*meta);
    }
    DrawDecisionTraceBlock(view, *ref);
  }
  switch (ui_state.selected_type) {
  case SelectedType::kPole: {
    const auto* pole = edit.poles.find(ui_state.selected_id);
    if (pole == nullptr) {
      ImGui::TextUnformatted("Selected pole is missing");
      return;
    }
    ImGui::Text("Type: Pole");
    ImGui::Text("ID: %s", pole->display_id.c_str());
    ImGui::Text("Name: %s", pole->name.c_str());
    ImGui::Text("PoleTypeId: %u", static_cast<unsigned int>(pole->pole_type_id));
    ImGui::Text("Generated: %s", pole->generation.generated ? "true" : "false");
    ImGui::Text("Gen Source: %d", static_cast<int>(pole->generation.source));
    ImGui::Text("Gen Session: %llu", static_cast<unsigned long long>(pole->generation.generation_session_id));
    ImGui::Text("Pos: %.2f %.2f %.2f", pole->world_transform.position.x, pole->world_transform.position.y,
                pole->world_transform.position.z);
    ImGui::Text("Height: %.2f", pole->height_m);
    ImGui::Text("PoleContext: %s", PoleContextLabel(pole->context.kind));
    ImGui::Text("cornerAngle: %.2f", pole->context.corner_angle_deg);
    ImGui::Text("cornerTurnSign: %.0f", pole->context.corner_turn_sign);
    ImGui::Text("sideScale: %.3f", pole->context.side_scale);
    ImGui::Text("angleCorrectionApplied: %s", pole->context.angle_correction_applied ? "true" : "false");
    ImGui::Text("sharpOrientationApplied: %s", pole->context.sharp_orientation_applied ? "true" : "false");
    ImGui::Text("sharpTheta: %.2f", pole->context.sharp_theta_deg);
    ImGui::Text("sharpBisector: %.3f %.3f %.3f", pole->context.sharp_bisector_dir.x, pole->context.sharp_bisector_dir.y,
                pole->context.sharp_bisector_dir.z);
    ImGui::Text("sharpSideDir: %.3f %.3f %.3f", pole->context.sharp_side_dir.x, pole->context.sharp_side_dir.y,
                pole->context.sharp_side_dir.z);
    ImGui::Text("placementOverride: %s", pole->placement_override_flag ? "true" : "false");
    if (const auto pole_view = view.inspect_pole(pole->id); pole_view.has_value()) {
      const auto override_view = view.inspect_overrides({wire::core::EntityKind::kPole, pole->id});
      ImGui::Text("forwardRule: %s", PoleForwardRuleLabel(pole_view->forward_rule));
      if (pole_view->has_forward) {
        ImGui::Text("forwardDir: %.3f %.3f %.3f", pole_view->forward_dir.x, pole_view->forward_dir.y,
                    pole_view->forward_dir.z);
      }
      ImGui::Text("supportAxisRule: %s", PoleSupportAxisRuleLabel(pole_view->support_axis_rule));
      if (pole_view->has_support_axis) {
        ImGui::Text("supportAxis: %.3f %.3f %.3f", pole_view->support_axis_dir.x, pole_view->support_axis_dir.y,
                    pole_view->support_axis_dir.z);
      }
      ImGui::Text("mainNeighbors: %llu / %llu",
                  static_cast<unsigned long long>(pole_view->primary_neighbor_id),
                  static_cast<unsigned long long>(pole_view->secondary_neighbor_id));
      ImGui::Text("orientationOverride: %s", pole_view->orientation_override ? "true" : "false");
      ImGui::Text("manualYawOverride: %s",
                  pole_view->manual_yaw_override_deg.has_value() ? std::to_string(*pole_view->manual_yaw_override_deg).c_str()
                                                                 : "auto");
      ImGui::Text("flip180Override: %s",
                  pole_view->flip_180_override.has_value()
                      ? (*pole_view->flip_180_override ? "true" : "false")
                      : "auto");
      ImGui::Text("autoYaw: %.2f", pole_view->automatic_yaw_deg);
      ImGui::Text("finalYaw: %.2f", pole_view->final_yaw_deg);
      ImGui::Text("layoutYaw: %.2f", pole_view->layout_yaw_deg);
      DrawRelatedLinks(ui_state, pole_view->links);
      DrawOverrideViewBlock(override_view);
      if (pole_view->orientation_override && ImGui::Button("Clear Pole Orientation Override")) {
        const auto clear = viewer_core_state::ClearPoleOrientationOverride(state, pole->id);
        PushLog(ui_state, clear.ok ? "ClearPoleOrientationOverride updated" : "ClearPoleOrientationOverride failed");
      }
      DrawTemplateViewBlock("Template", view.inspect_pole_template(pole->pole_type_id));
    }
    DrawPoleHeightDebugView(state, ui_state, *pole);
    return;
  }
  case SelectedType::kPort: {
    const auto* port = edit.ports.find(ui_state.selected_id);
    if (port == nullptr) {
      ImGui::TextUnformatted("Selected port is missing");
      return;
    }
    ImGui::Text("Type: Port");
    ImGui::Text("ID: %s", port->display_id.c_str());
    ImGui::Text("Owner Pole: %llu", static_cast<unsigned long long>(port->owner_pole_id));
    ImGui::Text("Pos: %.2f %.2f %.2f", port->world_position.x, port->world_position.y, port->world_position.z);
    ImGui::Text("Category: %s", CategoryLabel(port->category));
    ImGui::Text("Layer: %d Side: %s Role: %s", port->template_layer, SlotSideLabel(port->template_side),
                SlotRoleLabel(port->template_role));
    ImGui::Text("PositionMode: %s", PortPositionModeLabel(port->position_mode));
    ImGui::Text("PlacementSource: %s", PortPlacementSourceLabel(port->placement_source));
    ImGui::Text("UserEditedPos: %s", port->user_edited_position ? "true" : "false");
    ImGui::Text("GeneratedByRule: %s", port->generated_by_rule ? "true" : "false");
    ImGui::Text("PlacementContext: %s", ContextLabel(port->placement_context));
    ImGui::Text("AngleCorrected: %s sideScale=%.3f", port->angle_correction_applied ? "true" : "false",
                port->side_scale_applied);
    ImGui::Text("placementOverride: %s", port->placement_override_flag ? "true" : "false");
    ImGui::Text("orientationOverride: %s", port->orientation_override_flag ? "true" : "false");
    return;
  }
  case SelectedType::kSpan: {
    const auto* span = edit.spans.find(ui_state.selected_id);
    if (span == nullptr) {
      ImGui::TextUnformatted("Selected span is missing");
      return;
    }
    ImGui::Text("Type: Span");
    ImGui::Text("ID: %s", span->display_id.c_str());
    ImGui::Text("portA: %llu", static_cast<unsigned long long>(span->port_a_id));
    ImGui::Text("portB: %llu", static_cast<unsigned long long>(span->port_b_id));
    ImGui::Text("bundle: %llu", static_cast<unsigned long long>(span->bundle_id));
    ImGui::Text("Generated: %s", span->generation.generated ? "true" : "false");
    ImGui::Text("Gen Session: %llu", static_cast<unsigned long long>(span->generation.generation_session_id));
    ImGui::Text("GeneratedByRule: %s", span->generated_by_rule ? "true" : "false");
    ImGui::Text("PlacementContext: %s", ContextLabel(span->placement_context));
    ImGui::Text("placementOverride: %s", span->placement_override_flag ? "true" : "false");
    const auto* bounds = view.find_bounds_cache(span->id);
    if (bounds != nullptr) {
      const double sx = bounds->whole.max.x - bounds->whole.min.x;
      const double sy = bounds->whole.max.y - bounds->whole.min.y;
      const double sz = bounds->whole.max.z - bounds->whole.min.z;
      ImGui::Text("AABB size: %.2f %.2f %.2f", sx, sy, sz);
      ImGui::Text("segmentAABBs: %d", static_cast<int>(bounds->segments.size()));
    }
    const auto* runtime_state = view.find_span_runtime_state(span->id);
    if (runtime_state != nullptr) {
      ImGui::Separator();
      ImGui::Text("dataVersion: %llu", static_cast<unsigned long long>(runtime_state->data_version));
      ImGui::Text("geometryVersion: %llu", static_cast<unsigned long long>(runtime_state->geometry_version));
      ImGui::Text("boundsVersion: %llu", static_cast<unsigned long long>(runtime_state->bounds_version));
      ImGui::Text("renderVersion: %llu", static_cast<unsigned long long>(runtime_state->render_version));
      ImGui::Text("dirtyBits: %s", DirtyBitsToText(runtime_state->dirty_bits).c_str());
    }
    if (const auto span_view = view.inspect_span(span->id); span_view.has_value()) {
      const auto layout_view = view.span_layout(span->id);
      const auto curve_view = view.inspect_detail_curve(span->id);
      const auto override_view = view.inspect_overrides({wire::core::EntityKind::kSpan, span->id});
      if (curve_view.has_value()) {
        ImGui::Text("curveLength: %.2f", curve_view->curve_length_m);
        ImGui::Text("segments: %d", static_cast<int>(curve_view->segment_count));
        ImGui::Text("arcSamples: %d", static_cast<int>(curve_view->arc_length_sample_count));
        ImGui::Text("visible/hidden/replaced/supplemental: %d / %d / %d / %d",
                    static_cast<int>(curve_view->visible_interval_count),
                    static_cast<int>(curve_view->hidden_interval_count),
                    static_cast<int>(curve_view->replacement_path_count),
                    static_cast<int>(curve_view->supplemental_path_count));
        ImGui::Text("continuity: %s (%s)", DetailCurveContinuityModeLabel(curve_view->adopted_continuity),
                    DetailCurveContinuityReasonLabel(curve_view->continuity_reason));
        ImGui::Text("shapePolicy: %s", CurveShapePolicyLabel(curve_view->shape_policy));
        ImGui::Text("continuityPolicy: %s attemptedG2=%s degraded=%s",
                    ContinuityPolicyLabel(curve_view->requested_continuity),
                    curve_view->attempted_g2 ? "true" : "false",
                    curve_view->degraded_to_g1 ? "true" : "false");
        ImGui::Text("handles: %.2f / %.2f scale=%.2f", curve_view->handle_length_start_m,
                    curve_view->handle_length_end_m, curve_view->tangent_scale);
        ImGui::Text("handleScales: base=%.2f policy=%.2f angle=%.2f/%.2f", curve_view->base_handle_scale,
                    curve_view->policy_handle_scale, curve_view->start_angle_scale, curve_view->end_angle_scale);
        ImGui::Text("tangentRule: %s / %s",
                    DetailCurveEndpointTangentRuleLabel(curve_view->start_tangent_rule),
                    DetailCurveEndpointTangentRuleLabel(curve_view->end_tangent_rule));
        ImGui::Text("tangentMix start: support=%.2f chord=%.2f dep=%.2f lat=%.2f",
                    curve_view->start_support_weight, curve_view->start_chord_weight,
                    curve_view->start_departure_length_m, curve_view->start_lateral_ratio_limit);
        ImGui::Text("tangentMix end: support=%.2f chord=%.2f dep=%.2f lat=%.2f",
                    curve_view->end_support_weight, curve_view->end_chord_weight,
                    curve_view->end_departure_length_m, curve_view->end_lateral_ratio_limit);
        ImGui::Text("lateralSuppression: %.2f", curve_view->lateral_suppression);
        ImGui::Text("sag: amp=%.3f base=%.3f len=%.2f pass=%.2f rigid=%.2f", curve_view->sag_amplitude_m,
                    curve_view->sag_base_ratio, curve_view->sag_length_scale, curve_view->sag_pass_scale,
                    curve_view->sag_rigidity_scale);
        ImGui::Text("sagVariation: flow=%llu final=%.3f",
                    static_cast<unsigned long long>(curve_view->sag_variation.flow_key),
                    curve_view->sag_variation.final_value);
        ImGui::Text("variationLayers: world=%.3f flow=%.3f pole=%.3f local=%.3f",
                    curve_view->sag_variation.world_bias, curve_view->sag_variation.flow_bias,
                    curve_view->sag_variation.pole_delta, curve_view->sag_variation.local_jitter);
      } else {
        ImGui::TextUnformatted("curve: (none)");
      }
      ImGui::Separator();
      ImGui::Text("flowKind: %s", BackboneFlowKindLabel(span_view->flow_kind));
      ImGui::Text("flowRule: %s", BackboneFlowDecisionRuleLabel(span_view->flow_rule));
      ImGui::Text("lowering: %s branchSupport=%s downOffset=%.2f", BackboneLoweringKindLabel(span_view->lowering_kind),
                  span_view->uses_branch_support ? "true" : "false", span_view->branch_down_offset_m);
      ImGui::Text("continuity: class=%s defaultLower=%s sameLevel=%s(%s)",
          ContinuityCategoryClassLabel(span_view->continuity_class),
          span_view->default_lower_required ? "true" : "false",
          span_view->same_level_feasible ? "true" : "false",
          SameLevelFeasibilityReasonLabel(span_view->same_level_reason));
      ImGui::Text("clearance: topview=%.2f required=%.2f blocked=%s unresolved=%s solver=%s specialPorts=%s",
          span_view->projected_spacing_topview_m, span_view->required_clearance_m,
          span_view->lowering_blocked_by_policy ? "true" : "false",
          span_view->unresolved_same_level_conflict ? "true" : "false",
          span_view->solver_used_same_level_constraint ? "true" : "false",
          span_view->used_special_case_ports ? "true" : "false");
      ImGui::Text("orderDecision: A=%s(%s) B=%s(%s) flippedPrev=%s turn=%.2f",
                  OrderDecisionChoiceLabel(span_view->order_decision_choice_a),
                  OrderDecisionChoiceReasonLabel(span_view->order_decision_choice_reason_a),
                  OrderDecisionChoiceLabel(span_view->order_decision_choice_b),
                  OrderDecisionChoiceReasonLabel(span_view->order_decision_choice_reason_b),
                  span_view->flipped_from_previous ? "true" : "false", span_view->turn_angle_deg);
      DrawStyleInspectionBlock(span_view->style);
      if (layout_view.has_layout()) {
        const wire::core::SpanLayoutEntry& layout = *layout_view.entry;
        auto draw_layout_endpoint = [&](const char* label, const wire::core::LayoutEndpoint& endpoint) {
          ImGui::Text("%s: origin=%s src=%s flow=%s portSource=%s mode=%s", label,
                      SupportLayoutOriginLabel(endpoint.origin), SupportLayoutEndpointSourceLabel(endpoint.endpoint_source),
                      BackboneFlowKindLabel(endpoint.flow_kind), PortPlacementSourceLabel(endpoint.port_source),
                      CurveEndpointModeLabel(endpoint.endpoint_mode));
          ImGui::Text("  endpoint=%.2f %.2f %.2f departure=%.2f %.2f %.2f", endpoint.endpoint_world.x,
                      endpoint.endpoint_world.y, endpoint.endpoint_world.z, endpoint.departure_dir.x,
                      endpoint.departure_dir.y, endpoint.departure_dir.z);
          ImGui::Text("  support=%.2f %.2f %.2f dep=%.2f down=%.2f autoDown=%.2f", endpoint.support_world.x,
                      endpoint.support_world.y, endpoint.support_world.z, endpoint.local_departure_length_m,
                      endpoint.branch_down_offset_m, endpoint.automatic_branch_down_offset_m);
          ImGui::Text("  request=%s attach=%s requestedSocket=%s resolvedSocket=%s",
                      EndpointAttachmentRequestKindLabel(endpoint.attachment_request.kind),
                      endpoint.attachment_request.attachment_id.has_value()
                          ? std::to_string(static_cast<unsigned long long>(*endpoint.attachment_request.attachment_id)).c_str()
                          : "none",
                      endpoint.attachment_request.requested_socket_id.has_value()
                          ? std::to_string(*endpoint.attachment_request.requested_socket_id).c_str()
                          : "none",
                      endpoint.resolved_socket_id.has_value() ? std::to_string(*endpoint.resolved_socket_id).c_str()
                                                              : "none");
          ImGui::Text("  variation: flow=%llu final=%.3f world=%.3f flow=%.3f pole=%.3f local=%.3f",
                      static_cast<unsigned long long>(endpoint.down_offset_variation.flow_key),
                      endpoint.down_offset_variation.final_value, endpoint.down_offset_variation.world_bias,
                      endpoint.down_offset_variation.flow_bias, endpoint.down_offset_variation.pole_delta,
                      endpoint.down_offset_variation.local_jitter);
        };
        ImGui::Separator();
        ImGui::Text("spanLayout: flow=%s pass=%d flowKey=%llu", BackboneFlowKindLabel(layout.flow_kind),
                    static_cast<int>(layout.pass_mode), static_cast<unsigned long long>(layout.variation_flow_key));
        ImGui::Text("  lowering=%s supportGroupKeys=%d", BackboneLoweringKindLabel(layout.lowering_kind),
                    static_cast<int>(layout.lowered_support_group_keys.size()));
        draw_layout_endpoint("endpointA", layout.start);
        DrawEndpointDecisionSummary("endpointA decision", layout.start);
        draw_layout_endpoint("endpointB", layout.end);
        DrawEndpointDecisionSummary("endpointB decision", layout.end);
        if (!layout.lowered_support_group_keys.empty()) {
          ImGui::Separator();
          ImGui::Text("LoweredSupportGroupKeys: %d", static_cast<int>(layout.lowered_support_group_keys.size()));
          for (std::size_t index = 0; index < layout.lowered_support_group_keys.size(); ++index) {
            const auto& key = layout.lowered_support_group_keys[index];
            ImGui::Text("[%d] pole=%llu groupId=%d", static_cast<int>(index),
                        static_cast<unsigned long long>(key.owner_pole_id), key.support_group_id);
          }
        }
      } else {
        ImGui::TextUnformatted("spanLayout: (none)");
      }
      bool has_socket_override = false;
      bool has_branch_down_override = false;
      if (override_view.has_value()) {
        for (const auto& entry : override_view->entries) {
          has_socket_override =
              has_socket_override || ((entry.name == "endpointSocketA" || entry.name == "endpointSocketB") && entry.active);
          has_branch_down_override =
              has_branch_down_override || (entry.name == "branchDownOffset" && entry.active);
        }
      }
      ImGui::Text("orientationOverride: %s", (has_socket_override || has_branch_down_override) ? "true" : "false");
      DrawRelatedLinks(ui_state, span_view->links);
      DrawOverrideViewBlock(override_view);
      if (has_socket_override) {
        if (ImGui::Button("Clear Span Socket Override A")) {
          const auto clear = viewer_core_state::ClearSpanEndpointSocketOverride(state, span->id, true);
          PushLog(ui_state, clear.ok ? "ClearSpanEndpointSocketOverride A updated"
                                     : "ClearSpanEndpointSocketOverride A failed");
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Span Socket Override B")) {
          const auto clear = viewer_core_state::ClearSpanEndpointSocketOverride(state, span->id, false);
          PushLog(ui_state, clear.ok ? "ClearSpanEndpointSocketOverride B updated"
                                     : "ClearSpanEndpointSocketOverride B failed");
        }
      }
      if (has_branch_down_override && ImGui::Button("Clear Branch Down Offset Override")) {
        const auto clear = viewer_core_state::ClearSpanBranchDownOffsetOverride(state, span->id);
        PushLog(ui_state, clear.ok ? "ClearSpanBranchDownOffsetOverride updated"
                                   : "ClearSpanBranchDownOffsetOverride failed");
      }
      if (const auto* bundle = edit.bundles.find(span->bundle_id); bundle != nullptr) {
        DrawTemplateViewBlock("Bundle Template", view.inspect_bundle_template(bundle->bundle_template_id));
        if (const auto bundle_template = view.inspect_bundle_template(bundle->bundle_template_id); bundle_template.has_value()) {
          for (const auto& link : bundle_template->links) {
            if (link.ref.kind == wire::core::EntityKind::kTemplate) {
              DrawTemplateViewBlock("Cable Template", view.inspect_cable_template(static_cast<wire::core::CableTemplateId>(link.ref.stable_id)));
              break;
            }
          }
        }
      }
    }
    return;
  }
  case SelectedType::kAnchor: {
    const auto* anchor = edit.anchors.find(ui_state.selected_id);
    if (anchor == nullptr) {
      ImGui::TextUnformatted("Selected anchor is missing");
      return;
    }
    ImGui::Text("Type: Anchor");
    ImGui::Text("ID: %s", anchor->display_id.c_str());
    ImGui::Text("Owner Pole: %llu", static_cast<unsigned long long>(anchor->owner_pole_id));
    ImGui::Text("Pos: %.2f %.2f %.2f", anchor->world_position.x, anchor->world_position.y, anchor->world_position.z);
    return;
  }
  case SelectedType::kBundle: {
    const auto* bundle = edit.bundles.find(ui_state.selected_id);
    if (bundle == nullptr) {
      ImGui::TextUnformatted("Selected bundle is missing");
      return;
    }
    ImGui::Text("Type: Bundle");
    ImGui::Text("ID: %s", bundle->display_id.c_str());
    ImGui::Text("Conductor count: %d", bundle->conductor_count);
    ImGui::Text("Spacing: %.2f", bundle->phase_spacing_m);
    return;
  }
  case SelectedType::kAttachment: {
    const auto* attachment = edit.attachments.find(ui_state.selected_id);
    if (attachment == nullptr) {
      ImGui::TextUnformatted("Selected attachment is missing");
      return;
    }
    ImGui::Text("Type: Attachment");
    ImGui::Text("ID: %s", attachment->display_id.c_str());
    ImGui::Text("Span: %llu", static_cast<unsigned long long>(attachment->span_id));
    ImGui::Text("Template: %u", static_cast<unsigned int>(attachment->template_id));
    ImGui::Text("t: %.3f", attachment->t);
    ImGui::Text("displayOffset: %.3f", attachment->display_offset_m);
    if (const auto* attachment_template = view.find_attachment_template(attachment->template_id); attachment_template != nullptr) {
      ImGui::Text("interaction: %s", AttachmentLineInteractionModeLabel(attachment_template->line_interaction_mode));
      ImGui::Text("sockets: %d internalPaths: %d", static_cast<int>(attachment_template->sockets.size()),
                  static_cast<int>(attachment_template->internal_paths.size()));
    }
    DrawTemplateViewBlock("Attachment Template", view.inspect_attachment_template(attachment->template_id));
    return;
  }
  case SelectedType::kSupportNode: {
    const wire::core::BackboneResult backbone = viewer_core_state::BuildSavedBackboneResult(state);
    const auto it =
        std::find_if(backbone.nodes.begin(), backbone.nodes.end(),
                     [&](const wire::core::SupportNode& node) { return node.node_id == ui_state.selected_id; });
    if (it == backbone.nodes.end()) {
      ImGui::TextUnformatted("Selected support node is missing");
      return;
    }
    ImGui::Text("Type: SupportNode");
    ImGui::Text("ID: %llu", static_cast<unsigned long long>(it->node_id));
    ImGui::Text("SupportKind: %s", SupportKindLabel(it->support_kind));
    ImGui::Text("PoleId: %llu", static_cast<unsigned long long>(it->pole_id));
    ImGui::Text("Pos: %.2f %.2f %.2f", it->position.x, it->position.y, it->position.z);
    ImGui::Text("pathPointIndex: %d", it->path_point_index);
    ImGui::Text("sourceEdge: %s (%llu -> %llu @ %.3f)", it->has_source_edge ? "true" : "false",
                static_cast<unsigned long long>(it->source_edge_node_a_id),
                static_cast<unsigned long long>(it->source_edge_node_b_id), it->source_edge_t);
    ImGui::Text("hasTangentHint: %s", it->has_tangent_hint ? "true" : "false");
    if (it->has_tangent_hint) {
      ImGui::Text("tangentHint: %.3f %.3f %.3f", it->tangent_hint.x, it->tangent_hint.y, it->tangent_hint.z);
    }
    return;
  }
  case SelectedType::kSupportLayout: {
    const auto layout_view = view.inspect_support_layout(ui_state.selected_id);
    if (!layout_view.has_value()) {
      ImGui::TextUnformatted("Selected support layout is missing");
      return;
    }
    ImGui::TextUnformatted("Legacy/Recalc SupportLayout Inspection");
    ImGui::Text("Type: SupportLayout");
    ImGui::Text("flow: %s pass=%d flowKey=%llu", BackboneFlowKindLabel(layout_view->flow_kind),
                static_cast<int>(layout_view->pass_mode),
                static_cast<unsigned long long>(layout_view->variation_flow_key));
    ImGui::Text("relation: A=%s B=%s class=%s lower=%s", JunctionRelationKindLabel(layout_view->relation_a),
                JunctionRelationKindLabel(layout_view->relation_b),
                ContinuityCategoryClassLabel(layout_view->continuity_class),
                layout_view->default_lower_required ? "true" : "false");
    ImGui::Text("sameLevel=%s reason=%s blocked=%s unresolved=%s solver=%s specialPorts=%s",
                layout_view->same_level_feasible ? "true" : "false",
                SameLevelFeasibilityReasonLabel(layout_view->same_level_reason),
                layout_view->lowering_blocked_by_policy ? "true" : "false",
                layout_view->unresolved_same_level_conflict ? "true" : "false",
                layout_view->solver_used_same_level_constraint ? "true" : "false",
                layout_view->used_special_case_ports ? "true" : "false");
    ImGui::Text("orderPolicy=%s lowering=%s topview=%.2f required=%.2f",
                OrderDecisionPolicyLabel(layout_view->order_decision_policy),
                BackboneLoweringKindLabel(layout_view->lowering_kind),
                layout_view->projected_spacing_topview_m, layout_view->required_clearance_m);
    auto draw_endpoint = [&](const char* label, const wire::core::SupportLayoutEndpointView& endpoint) {
      ImGui::Text("%s origin=%s source=%s flow=%s port=%s", label, endpoint.origin.c_str(),
                  SupportLayoutEndpointSourceLabel(endpoint.endpoint_source),
                  BackboneFlowKindLabel(endpoint.flow_kind), endpoint.port_source.c_str());
      ImGui::Text("  endpoint=%.2f %.2f %.2f", endpoint.endpoint_world.x, endpoint.endpoint_world.y, endpoint.endpoint_world.z);
      ImGui::Text("  departure=%.2f %.2f %.2f localDep=%.2f", endpoint.departure_dir.x, endpoint.departure_dir.y,
                  endpoint.departure_dir.z, endpoint.local_departure_length_m);
      ImGui::Text("  downOffset=%.2f request=%s attach=%s requestedSocket=%s resolvedSocket=%s",
                  endpoint.branch_down_offset_m, EndpointAttachmentRequestKindLabel(endpoint.attachment_request.kind),
                  endpoint.attachment_request.attachment_id.has_value()
                      ? std::to_string(static_cast<unsigned long long>(*endpoint.attachment_request.attachment_id)).c_str()
                      : "none",
                  endpoint.attachment_request.requested_socket_id.has_value()
                      ? std::to_string(*endpoint.attachment_request.requested_socket_id).c_str()
                      : "none",
                  endpoint.resolved_socket_id.has_value() ? std::to_string(*endpoint.resolved_socket_id).c_str()
                                                          : "none");
      DrawEndpointDecisionSummary("  decision", endpoint);
    };
    draw_endpoint("start", layout_view->start_endpoint);
    draw_endpoint("end", layout_view->end_endpoint);
    DrawLoweredSupportGroupsBlock(layout_view->lowered_support_groups);
    DrawRelatedLinks(ui_state, layout_view->links);
    return;
  }
  case SelectedType::kDetailCurve: {
    const auto curve_view = view.inspect_detail_curve(ui_state.selected_id);
    if (!curve_view.has_value()) {
      ImGui::TextUnformatted("Selected detail curve is missing");
      return;
    }
    ImGui::Text("Type: DetailCurve");
    ImGui::Text("sourceSpan: %llu", static_cast<unsigned long long>(curve_view->source_span.stable_id));
    ImGui::Text("continuity: %s (%s)", DetailCurveContinuityModeLabel(curve_view->adopted_continuity),
                DetailCurveContinuityReasonLabel(curve_view->continuity_reason));
    ImGui::Text("requested: %s degraded=%s", ContinuityPolicyLabel(curve_view->requested_continuity),
                curve_view->degraded_to_g1 ? "true" : "false");
    ImGui::Text("sagAmplitude: %.3f", curve_view->sag_amplitude_m);
    ImGui::Text("curveLength: %.3f", curve_view->curve_length_m);
    ImGui::Text("segments: %d", static_cast<int>(curve_view->segment_count));
    ImGui::Text("controlP1: %.2f %.2f %.2f", curve_view->control_points[1].x, curve_view->control_points[1].y,
                curve_view->control_points[1].z);
    ImGui::Text("controlP2: %.2f %.2f %.2f", curve_view->control_points[2].x, curve_view->control_points[2].y,
                curve_view->control_points[2].z);
    ImGui::Text("arcSamples=%d visible=%d hidden=%d replaced=%d supplemental=%d",
                static_cast<int>(curve_view->arc_length_sample_count),
                static_cast<int>(curve_view->visible_interval_count), static_cast<int>(curve_view->hidden_interval_count),
                static_cast<int>(curve_view->replacement_path_count),
                static_cast<int>(curve_view->supplemental_path_count));
    ImGui::Text("tangentRule: %s / %s", DetailCurveEndpointTangentRuleLabel(curve_view->start_tangent_rule),
                DetailCurveEndpointTangentRuleLabel(curve_view->end_tangent_rule));
    ImGui::Text("lateralSuppression: %.2f", curve_view->lateral_suppression);
    ImGui::Text("endpointSource: %s / %s", SupportLayoutEndpointSourceLabel(curve_view->start_endpoint_source),
                SupportLayoutEndpointSourceLabel(curve_view->end_endpoint_source));
    ImGui::Text("endpointRequest: %s[%s -> %s] / %s[%s -> %s]",
                EndpointAttachmentRequestKindLabel(curve_view->start_attachment_request.kind),
                curve_view->start_attachment_request.requested_socket_id.has_value()
                    ? std::to_string(*curve_view->start_attachment_request.requested_socket_id).c_str()
                    : "none",
                curve_view->start_resolved_socket_id.has_value()
                    ? std::to_string(*curve_view->start_resolved_socket_id).c_str()
                    : "none",
                EndpointAttachmentRequestKindLabel(curve_view->end_attachment_request.kind),
                curve_view->end_attachment_request.requested_socket_id.has_value()
                    ? std::to_string(*curve_view->end_attachment_request.requested_socket_id).c_str()
                    : "none",
                curve_view->end_resolved_socket_id.has_value()
                    ? std::to_string(*curve_view->end_resolved_socket_id).c_str()
                    : "none");
    DrawStyleInspectionBlock(curve_view->style);
    DrawRelatedLinks(ui_state, curve_view->links);
    return;
  }
  case SelectedType::kJunction: {
    const auto junction_view = view.inspect_junction(ui_state.selected_id);
    if (!junction_view.has_value()) {
      ImGui::TextUnformatted("Selected junction is missing");
      return;
    }
    ImGui::Text("Type: Junction");
    ImGui::Text("nodeId: %llu", static_cast<unsigned long long>(junction_view->node_id));
    ImGui::Text("hasPrimary: %s incidents=%d", junction_view->has_primary ? "true" : "false",
                static_cast<int>(junction_view->incidents.size()));
    for (const auto& incident : junction_view->incidents) {
      ImGui::Text("neighbor=%llu order=%d primary=%s", static_cast<unsigned long long>(incident.neighbor_node_id),
                  incident.order, incident.primary ? "true" : "false");
    }
    DrawRelatedLinks(ui_state, junction_view->links);
    return;
  }
  default:
    break;
  }
}

void DrawEditSelectedPanel(CoreState& state, ViewerUiState& ui_state) {
  (void)state;
  ImGui::Separator();
  ImGui::TextUnformatted("Direct object editing is disabled.");
  ImGui::TextUnformatted("Use DrawPath as the only generation/edit entry.");
}

void DrawTopbarWindow(const wire::core::CoreView& view, ViewerUiState& ui_state) {
  const float w = static_cast<float>(GetScreenWidth());
  const float topbar_h = 74.0f;
  ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(std::max(320.0f, w - 16.0f), topbar_h), ImGuiCond_Always);
  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
  if (!ImGui::Begin("Topbar", nullptr, flags)) {
    ImGui::End();
    return;
  }

  ImGui::TextUnformatted("Tool: DrawPath");
  ImGui::SameLine();
  ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - 250.0f));
  ImGui::Checkbox("Unified UI", &ui_state.ui_unified_workspace);
  ImGui::SameLine();
  ImGui::Checkbox("Show Workspace", &ui_state.ui_show_workspace);
  if (ui_state.ui_unified_workspace) {
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    if (ui_state.ui_workspace_width <= 1.0f) {
      ui_state.ui_workspace_width = 420.0f;
    }
    ImGui::SliderFloat("##WorkspaceWidth", &ui_state.ui_workspace_width, 300.0f, 760.0f, "W %.0f");
  }
  ImGui::Separator();
  ImGui::Text("Poles:%d  Ports:%d  Spans:%d  Bundles:%d", static_cast<int>(view.poles().size()),
              static_cast<int>(view.ports().size()), static_cast<int>(view.spans().size()),
              static_cast<int>(view.bundles().size()));
  ImGui::SameLine();
  ImGui::Text("|  Mode: %s", ModeLabel(ui_state.mode));
  ImGui::SameLine();
  ImGui::Text("|  FOV %.1f", ui_state.camera_fov_deg);
  ImGui::SameLine();
  ImGui::Text("|  Walk %s", ui_state.camera_walk_mode ? "ON" : "OFF");
  if (ui_state.selected_type == SelectedType::kPole) {
    if (const auto* pole = view.poles().find(ui_state.selected_id); pole != nullptr) {
      ImGui::SameLine();
      ImGui::Text("|  Selected Pole Placement: %s", PolePlacementModeLabel(pole->placement_mode));
    }
  }
  ImGui::End();
}

void DrawToolboxContent(CoreState& state, ViewerUiState& ui_state) {
  ImGui::Text("Active Tool: %s", ModeLabel(ui_state.mode));
  ImGui::Separator();
  DrawPathModePanel(state, ui_state);
}

void DrawToolboxWindow(CoreState& state, ViewerUiState& ui_state) {
  ImGui::SetNextWindowPos(ImVec2(8.0f, 90.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 520.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Toolbox", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawToolboxContent(state, ui_state);
  ImGui::End();
}

void DrawInspectorContent(CoreState& state, ViewerUiState& ui_state) {
  DrawSelectedInfo(state, ui_state);
  DrawEditSelectedPanel(state, ui_state);

  ImGui::Separator();
  ImGui::Text("Selection Count: %d", static_cast<int>(ui_state.selection_items.size()));
  ImGui::Text("Poles=%d Midair=%d Spans=%d", SelectionCountByType(ui_state, SelectedType::kPole),
              SelectionCountByType(ui_state, SelectedType::kSupportNode),
              SelectionCountByType(ui_state, SelectedType::kSpan));
  if (ImGui::Button("Clear Selection")) {
    ClearSelection(ui_state);
  }
}

void DrawInspectorWindow(CoreState& state, ViewerUiState& ui_state) {
  const float w = static_cast<float>(GetScreenWidth());
  ImGui::SetNextWindowPos(ImVec2(std::max(440.0f, w - 430.0f), 90.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 620.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Inspector", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawInspectorContent(state, ui_state);
  ImGui::End();
}

void DrawOutlinerContent(const wire::core::CoreView& view, const wire::core::BackboneResult& backbone,
                         ViewerUiState& ui_state) {
  DrawObjectList(
      ui_state, "Poles", SelectedType::kPole,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(view.poles().size());
        for (const auto& pole : view.poles().items()) {
          ids.push_back(pole.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* pole = view.poles().find(id);
        if (pole == nullptr) {
          return std::to_string(id);
        }
        return pole->display_id + " " + pole->name;
      });

  DrawObjectList(
      ui_state, "Ports", SelectedType::kPort,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(view.ports().size());
        for (const auto& port : view.ports().items()) {
          ids.push_back(port.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* port = view.ports().find(id);
        if (port == nullptr) {
          return std::to_string(id);
        }
        return port->display_id;
      });

  DrawObjectList(
      ui_state, "Spans", SelectedType::kSpan,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(view.spans().size());
        for (const auto& span : view.spans().items()) {
          ids.push_back(span.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* span = view.spans().find(id);
        if (span == nullptr) {
          return std::to_string(id);
        }
        return span->display_id;
      });

  DrawObjectList(
      ui_state, "Midair SupportNodes", SelectedType::kSupportNode,
      [&]() {
        std::vector<ObjectId> ids;
        for (const wire::core::SupportNode& node : backbone.nodes) {
          if (node.support_kind == wire::core::SupportKind::kMidair) {
            ids.push_back(node.node_id);
          }
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto it = std::find_if(backbone.nodes.begin(), backbone.nodes.end(),
                                     [id](const wire::core::SupportNode& node) { return node.node_id == id; });
        if (it == backbone.nodes.end()) {
          return std::to_string(id);
        }
        return std::string("midair ") + std::to_string(static_cast<unsigned long long>(it->node_id));
      });
}

void DrawOutlinerWindow(CoreState& state, ViewerUiState& ui_state) {
  const auto view = viewer_core_state::View(state);
  const wire::core::BackboneResult backbone = viewer_core_state::BuildSavedBackboneResult(state);
  ImGui::SetNextWindowPos(ImVec2(8.0f, 620.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 260.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Outliner", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawOutlinerContent(view, backbone, ui_state);
  ImGui::End();
}

void DrawDiagnosticsContent(CoreState& state, ViewerUiState& ui_state) {
  const auto view = viewer_core_state::View(state);
  if (!ui_state.geometry_settings_loaded) {
    const auto& gs = view.geometry_settings();
    ui_state.geometry_samples = gs.curve_samples;
    ui_state.geometry_sag_enabled = gs.sag_enabled;
    ui_state.geometry_sag_factor = gs.sag_factor;
    ui_state.geometry_pole_clearance = gs.pole_clearance_m;
    ui_state.geometry_settings_loaded = true;
  }
  if (!ui_state.layout_settings_loaded) {
    const auto& ls = view.layout_settings();
    ui_state.layout_angle_correction_enabled = ls.angle_correction_enabled;
    ui_state.layout_corner_threshold_deg = ls.corner_threshold_deg;
    ui_state.layout_min_side_scale = ls.min_side_scale;
    ui_state.layout_max_side_scale = ls.max_side_scale;
    ui_state.layout_settings_loaded = true;
  }
  if (!ui_state.visual_settings_loaded) {
    const auto& vs = view.visual_settings();
    ui_state.visual_enable_support_structures = vs.enable_support_structures;
    ui_state.visual_enable_insulators = vs.enable_insulators;
    ui_state.visual_support_center_threshold = vs.support_center_threshold_m;
    ui_state.visual_support_arm_extra = vs.support_arm_extra_m;
    ui_state.visual_insulator_radius = vs.insulator_radius_m;
    ui_state.visual_insulator_length = vs.insulator_length_m;
    ui_state.visual_settings_loaded = true;
  }
  if (!ui_state.cable_template_loaded) {
    if (const auto id = FindCableTemplateIdByName(view, "HV_BARE"); id.has_value()) {
      ui_state.selected_cable_template_id = *id;
      LoadCableTemplateState(view, ui_state, *id);
    } else {
      for (const auto& [id, tpl] : view.cable_templates()) {
        (void)tpl;
        LoadCableTemplateState(view, ui_state, id);
        break;
      }
    }
    ApplyStartupCableEditorDefaults(ui_state);
    ui_state.cable_template_loaded = true;
  }
  if (!ui_state.bundle_template_loaded) {
    if (view.bundle_templates().contains(ui_state.selected_bundle_template_id)) {
      LoadBundleTemplateState(view, ui_state, ui_state.selected_bundle_template_id);
    } else {
      for (const auto& [id, _] : view.bundle_templates()) {
        LoadBundleTemplateState(view, ui_state, id);
        break;
      }
    }
    if (const auto id = FindCableTemplateIdByName(view, "HV_BARE"); id.has_value()) {
      ui_state.selected_cable_template_id = *id;
    }
    ApplyStartupCableEditorDefaults(ui_state);
    ui_state.bundle_template_loaded = true;
  }
  if (!ui_state.pole_template_loaded) {
    if (const auto id = FindPoleTypeIdByName(view, "CommunicationPole"); id.has_value()) {
      LoadPoleTemplateState(view, ui_state, *id);
    } else {
      for (const auto& [id, _] : view.pole_types()) {
        LoadPoleTemplateState(view, ui_state, id);
        break;
      }
    }
  }

  const auto& recalc = view.last_recalc_stats();
  ImGui::Text("Dirty T/D/G/B/R/X: %d / %d / %d / %d / %d / %d",
              static_cast<int>(view.dirty_queue().topology_dirty_span_ids.size()),
              static_cast<int>(view.dirty_queue().decision_dirty_span_ids.size()),
              static_cast<int>(view.dirty_queue().geometry_dirty_span_ids.size()),
              static_cast<int>(view.dirty_queue().bounds_dirty_span_ids.size()),
              static_cast<int>(view.dirty_queue().render_dirty_span_ids.size()),
              static_cast<int>(view.dirty_queue().raycast_dirty_span_ids.size()));
  ImGui::Text("Legacy Recalc total=%d decision=%d geom=%d bounds=%d render=%d", static_cast<int>(recalc.total_processed()),
              static_cast<int>(recalc.decision_processed), static_cast<int>(recalc.geometry_processed),
              static_cast<int>(recalc.bounds_processed),
              static_cast<int>(recalc.render_processed));
  if (ImGui::Button("Run Legacy Recalc")) {
    wire::core::CommitOptions options{};
    options.run_recalc = true;
    options.run_validate = false;
    const auto stats = viewer_core_state::Commit(state, options).recalc_stats;
    PushLog(ui_state, "Recalc processed=" + std::to_string(stats.total_processed()));
  }

  if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SliderFloat("FOV", &ui_state.camera_fov_deg, 20.0f, 110.0f, "%.1f deg");
    ImGui::SliderFloat("Walk Speed", &ui_state.camera_walk_speed, 0.5f, 80.0f, "%.1f");
    ImGui::SliderFloat("Mouse Sensitivity", &ui_state.camera_mouse_sensitivity, 0.001f, 0.02f, "%.4f");
    if (ImGui::Button(ui_state.camera_walk_mode ? "Stop Walk (Shift+F)" : "Start Walk (Shift+F)")) {
      ui_state.camera_walk_mode = !ui_state.camera_walk_mode;
      if (ui_state.camera_walk_mode) {
        DisableCursor();
        PushLog(ui_state, "Camera: Walk mode ON (WASD + mouse, Q/E up/down, Esc exit)");
      } else {
        EnableCursor();
        PushLog(ui_state, "Camera: Walk mode OFF");
      }
    }
    ImGui::TextUnformatted("Alt+LMB: set orbit pivot on draw plane");
    ImGui::TextUnformatted("[ / ]: FOV down/up");
  }

  if (ImGui::CollapsingHeader("Geometry/Layout", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SliderInt("Curve Samples", &ui_state.geometry_samples, 2, 64);
    ImGui::Checkbox("Sag Enabled", &ui_state.geometry_sag_enabled);
    ImGui::InputDouble("Sag Factor", &ui_state.geometry_sag_factor, 0.005, 0.01, "%.4f");
    ImGui::InputDouble("Pole Clearance", &ui_state.geometry_pole_clearance, 0.005, 0.01, "%.3f");
    if (ImGui::Button("Apply Geometry")) {
      wire::core::GeometrySettings settings{};
      settings.curve_samples = ui_state.geometry_samples;
      settings.sag_enabled = ui_state.geometry_sag_enabled;
      settings.sag_factor = ui_state.geometry_sag_factor;
      settings.pole_clearance_m = ui_state.geometry_pole_clearance;
      const auto result = viewer_core_state::UpdateGeometrySettings(state, settings, true);
      if (!result.ok) {
        ui_state.last_error = result.error;
        PushLog(ui_state, "UpdateGeometrySettings failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Geometry settings updated");
      }
    }
    ImGui::Separator();
    ImGui::Checkbox("Angle Correction Enabled", &ui_state.layout_angle_correction_enabled);
    ImGui::InputDouble("Corner Threshold Deg", &ui_state.layout_corner_threshold_deg, 1.0, 5.0, "%.2f");
    ImGui::InputDouble("Min Side Scale", &ui_state.layout_min_side_scale, 0.05, 0.1, "%.3f");
    ImGui::InputDouble("Max Side Scale", &ui_state.layout_max_side_scale, 0.05, 0.1, "%.3f");
    if (ImGui::Button("Apply Layout")) {
      wire::core::LayoutSettings settings{};
      settings.angle_correction_enabled = ui_state.layout_angle_correction_enabled;
      settings.corner_threshold_deg = ui_state.layout_corner_threshold_deg;
      settings.min_side_scale = ui_state.layout_min_side_scale;
      settings.max_side_scale = ui_state.layout_max_side_scale;
      const auto result = viewer_core_state::UpdateLayoutSettings(state, settings);
      if (!result.ok) {
        ui_state.last_error = result.error;
        PushLog(ui_state, "UpdateLayoutSettings failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Layout settings updated");
      }
    }
  }

  if (ImGui::CollapsingHeader("Pole Tilt / Templates", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputDouble("Max Tilt (deg)", &ui_state.tilt_all_max_deg, 0.5, 1.0, "%.2f");
    {
      std::vector<ObjectId> selected_pole_ids{};
      selected_pole_ids.reserve(ui_state.selection_items.size());
      for (const SelectionItem& item : ui_state.selection_items) {
        if (item.type == SelectedType::kPole) {
          selected_pole_ids.push_back(item.id);
        }
      }
      if (ImGui::Button("Apply Tilt To Selected Poles")) {
        if (selected_pole_ids.empty()) {
          PushLog(ui_state, "No poles selected");
        } else {
          const auto tilt = viewer_core_state::ApplyPoleTilt(state, selected_pole_ids, ui_state.tilt_all_max_deg);
          if (!tilt.ok) {
            ui_state.last_error = tilt.error;
            PushLog(ui_state, "ApplyPoleTilt(selected) failed");
          } else {
            ui_state.last_error.clear();
            PushLog(ui_state, "Applied tilt to selected poles count=" +
                                  std::to_string(static_cast<unsigned long long>(selected_pole_ids.size())));
          }
        }
      }
      ImGui::SameLine();
      ImGui::Text("selected=%d", static_cast<int>(selected_pole_ids.size()));
    }
    ImGui::Checkbox("Select Poles", &ui_state.selection_include_poles);
    ImGui::SameLine();
    ImGui::Checkbox("Select Midair", &ui_state.selection_include_midair_nodes);
    ImGui::SameLine();
    ImGui::Checkbox("Select Spans", &ui_state.selection_include_spans);
    ImGui::TextUnformatted("Viewport: LMB click select, Shift+LMB drag box select");
    if (ImGui::Button("Apply Tilt To All Poles")) {
      const auto tilt = viewer_core_state::ApplyPoleTilt(state, {}, ui_state.tilt_all_max_deg);
      if (!tilt.ok) {
        ui_state.last_error = tilt.error;
        PushLog(ui_state, "ApplyPoleTilt failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Applied tilt to all poles");
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset All Span Reference Lengths")) {
      const auto reset = viewer_core_state::ResetAllSpanReferenceLengths(state, true);
      if (!reset.ok) {
        ui_state.last_error = reset.error;
        PushLog(ui_state, "ResetAllSpanReferenceLengths failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Span reference lengths reset from current geometry");
      }
    }

    ImGui::Separator();
    const auto cable_ids = SortedCableTemplateIds(view);
    if (const auto it = view.cable_templates().find(ui_state.selected_cable_template_id);
        it != view.cable_templates().end()) {
      if (ImGui::BeginCombo("Cable Template", it->second.name.c_str())) {
        for (wire::core::CableTemplateId id : cable_ids) {
          const auto jt = view.cable_templates().find(id);
          if (jt == view.cable_templates().end()) {
            continue;
          }
          const bool selected = (id == ui_state.selected_cable_template_id);
          if (ImGui::Selectable(jt->second.name.c_str(), selected)) {
            LoadCableTemplateState(view, ui_state, id);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
    }
    ImGui::Text("Cable Name: %s", ui_state.cable_template_name.c_str());
    ImGui::InputDouble("Cable Outer Diameter", &ui_state.cable_outer_diameter, 0.001, 0.005, "%.4f");
    ImGui::InputDouble("Cable Bend Stiffness", &ui_state.cable_bend_stiffness, 0.1, 0.2, "%.3f");
    ImGui::InputDouble("Cable Min Bend Radius", &ui_state.cable_min_bend_radius, 0.01, 0.05, "%.3f");
    const auto selected_material = static_cast<wire::core::CableMaterialStyleKind>(ui_state.cable_material_style);
    if (ImGui::BeginCombo("Cable Material", CableMaterialStyleLabel(selected_material))) {
      for (int raw = static_cast<int>(wire::core::CableMaterialStyleKind::kGeneric);
           raw <= static_cast<int>(wire::core::CableMaterialStyleKind::kOptical); ++raw) {
        const auto material = static_cast<wire::core::CableMaterialStyleKind>(raw);
        const bool selected = (raw == ui_state.cable_material_style);
        if (ImGui::Selectable(CableMaterialStyleLabel(material), selected)) {
          ui_state.cable_material_style = raw;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Checkbox("Cable Requires Insulator", &ui_state.cable_requires_insulator);
    ImGui::InputDouble("Cable Insulator Attach Height", &ui_state.cable_insulator_attachment_height, 0.005, 0.01, "%.3f");
    ImGui::InputDouble("Cable Sag Factor", &ui_state.cable_sag_factor, 0.005, 0.01, "%.4f");
    ImGui::InputDouble("Cable Slack Factor", &ui_state.cable_slack_factor, 0.005, 0.01, "%.4f");
    ImGui::InputDouble("Cable Default Grouped Fanout Spacing", &ui_state.cable_default_grouped_support_fanout_spacing,
                       0.01, 0.05, "%.3f");
    const auto selected_cable_continuity =
        static_cast<wire::core::CableContinuityPolicyHint>(ui_state.cable_continuity_policy);
    if (ImGui::BeginCombo("Cable Continuity", ContinuityPolicyLabel(selected_cable_continuity))) {
      for (int raw = static_cast<int>(wire::core::CableContinuityPolicyHint::kAuto);
           raw <= static_cast<int>(wire::core::CableContinuityPolicyHint::kPreferG2); ++raw) {
        const auto policy = static_cast<wire::core::CableContinuityPolicyHint>(raw);
        const bool selected = (raw == ui_state.cable_continuity_policy);
        if (ImGui::Selectable(ContinuityPolicyLabel(policy), selected)) {
          ui_state.cable_continuity_policy = raw;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Separator();
    ImGui::Checkbox("CurveOffset Straight Supplemental", &ui_state.cable_curve_offset_straight_supplemental_enabled);
    if (ui_state.cable_curve_offset_straight_supplemental_enabled) {
      ImGui::InputDouble("Supplemental Lateral Offset", &ui_state.cable_curve_offset_straight_lateral_offset, 0.01, 0.05,
                         "%.3f");
      ImGui::InputDouble("Supplemental Vertical Offset", &ui_state.cable_curve_offset_straight_vertical_offset, 0.01, 0.05,
                         "%.3f");
      ImGui::InputDouble("Supplemental Wobble Amplitude", &ui_state.cable_curve_offset_straight_wobble_amplitude, 0.005,
                         0.01, "%.3f");
      ImGui::InputDouble("Supplemental Wobble Wavelength", &ui_state.cable_curve_offset_straight_wobble_wavelength, 0.1,
                         0.5, "%.3f");
      ImGui::InputDouble("Supplemental Wobble Phase", &ui_state.cable_curve_offset_straight_wobble_phase_bias, 0.1, 0.5,
                         "%.3f");
      ImGui::InputDouble("Supplemental Endpoint Envelope", &ui_state.cable_curve_offset_straight_endpoint_envelope_ratio,
                         0.05, 0.1, "%.3f");
    }
    if (ImGui::Button("Apply Cable Template")) {
      const auto it = view.cable_templates().find(ui_state.selected_cable_template_id);
      if (it == view.cable_templates().end()) {
        ui_state.last_error = "selected cable template missing";
      } else {
        wire::core::CableTemplate tpl = it->second;
        tpl.outer_diameter_m = ui_state.cable_outer_diameter;
        tpl.bend_stiffness = ui_state.cable_bend_stiffness;
        tpl.min_bend_radius_m = ui_state.cable_min_bend_radius;
        tpl.material_style = static_cast<wire::core::CableMaterialStyleKind>(ui_state.cable_material_style);
        tpl.requires_insulator = ui_state.cable_requires_insulator;
        tpl.insulator_attachment_height_m = ui_state.cable_insulator_attachment_height;
        tpl.sag_factor = ui_state.cable_sag_factor;
        tpl.slack_factor = ui_state.cable_slack_factor;
        tpl.default_grouped_support_fanout_spacing_m = ui_state.cable_default_grouped_support_fanout_spacing;
        tpl.continuity_policy =
            static_cast<wire::core::CableContinuityPolicyHint>(ui_state.cable_continuity_policy);
        auto supplemental_it =
            std::find_if(tpl.supplemental_paths.begin(), tpl.supplemental_paths.end(),
                         [](const wire::core::CableSupplementalPathTemplate& supplemental) {
                           return supplemental.anchor_mode ==
                                      wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
                                  supplemental.profile_kind ==
                                      wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
                         });
        if (ui_state.cable_curve_offset_straight_supplemental_enabled) {
          wire::core::CableSupplementalPathTemplate supplemental{};
          if (supplemental_it != tpl.supplemental_paths.end()) {
            supplemental = *supplemental_it;
          }
          supplemental.anchor_mode = wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset;
          supplemental.profile_kind = wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
          supplemental.lateral_offset_m = ui_state.cable_curve_offset_straight_lateral_offset;
          supplemental.vertical_offset_m = ui_state.cable_curve_offset_straight_vertical_offset;
          supplemental.wobble_amplitude_m = ui_state.cable_curve_offset_straight_wobble_amplitude;
          supplemental.wobble_wavelength_m = ui_state.cable_curve_offset_straight_wobble_wavelength;
          supplemental.wobble_phase_bias = ui_state.cable_curve_offset_straight_wobble_phase_bias;
          supplemental.endpoint_envelope_ratio =
              std::clamp(ui_state.cable_curve_offset_straight_endpoint_envelope_ratio, 0.0, 0.5);
          if (supplemental_it != tpl.supplemental_paths.end()) {
            *supplemental_it = supplemental;
          } else {
            tpl.supplemental_paths.push_back(supplemental);
          }
        } else if (supplemental_it != tpl.supplemental_paths.end()) {
          tpl.supplemental_paths.erase(supplemental_it);
        }
        const auto apply = viewer_core_state::UpdateCableTemplate(state, tpl, ui_state.preferred_visible_span_ids);
        if (!apply.ok) {
          ui_state.last_error = apply.error;
          PushLog(ui_state, "UpdateCableTemplate failed");
        } else {
          ui_state.last_error.clear();
          LoadCableTemplateState(viewer_core_state::View(state), ui_state, tpl.id);
          PushLog(ui_state, "Cable template updated; visible-first=" +
                                    std::to_string(ui_state.preferred_visible_span_count) + " dirty spans=" +
                                    std::to_string(apply.change_set.dirty_span_ids.size()));
        }
      }
    }
    ImGui::Text("Preferred Visible Spans: %d", ui_state.preferred_visible_span_count);

    ImGui::Separator();
    const auto bundle_ids = SortedBundleTemplateKinds(view);
    if (const auto it = view.bundle_templates().find(ui_state.selected_bundle_template_id);
        it != view.bundle_templates().end()) {
      if (ImGui::BeginCombo("Bundle Template", it->second.name.c_str())) {
        for (wire::core::BundleKind id : bundle_ids) {
          const auto jt = view.bundle_templates().find(id);
          if (jt == view.bundle_templates().end()) {
            continue;
          }
          const bool selected = (id == ui_state.selected_bundle_template_id);
          if (ImGui::Selectable(jt->second.name.c_str(), selected)) {
            LoadBundleTemplateState(view, ui_state, id);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      ImGui::Text("Category: %s", CategoryLabel(it->second.category));
      ImGui::Text("Count Rule: %s", it->second.count_rule == wire::core::BundleCountRuleKind::kFixed ? "Fixed" : "Range");
      if (it->second.count_rule == wire::core::BundleCountRuleKind::kFixed) {
        ImGui::Text("Fixed Count: %d", it->second.fixed_count);
      } else {
        ImGui::Text("Count Range: %d..%d default=%d", it->second.min_count, it->second.max_count,
                    it->second.default_count);
      }
      const auto selected_bundle_cable_it = view.cable_templates().find(ui_state.bundle_template_cable_template_id);
      const char* selected_bundle_cable_label =
          selected_bundle_cable_it != view.cable_templates().end() ? selected_bundle_cable_it->second.name.c_str()
                                                                   : "(missing)";
      if (ImGui::BeginCombo("Bundle Cable Template", selected_bundle_cable_label)) {
        for (wire::core::CableTemplateId id : cable_ids) {
          const auto cable_it = view.cable_templates().find(id);
          if (cable_it == view.cable_templates().end()) {
            continue;
          }
          const bool selected = (id == ui_state.bundle_template_cable_template_id);
          if (ImGui::Selectable(cable_it->second.name.c_str(), selected)) {
            ui_state.bundle_template_cable_template_id = id;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto pole_type_ids = SortedPoleTypeIds(view);
      const auto selected_related_pole_it = view.pole_types().find(ui_state.selected_pole_template_id);
      const char* selected_related_pole_label =
          selected_related_pole_it != view.pole_types().end() ? selected_related_pole_it->second.name.c_str() : "(none)";
      if (ImGui::BeginCombo("Related Pole Template##bundle", selected_related_pole_label)) {
        for (wire::core::PoleTypeId id : pole_type_ids) {
          const auto pole_it = view.pole_types().find(id);
          if (pole_it == view.pole_types().end()) {
            continue;
          }
          const bool selected = (id == ui_state.selected_pole_template_id);
          if (ImGui::Selectable(pole_it->second.name.c_str(), selected)) {
            LoadPoleTemplateState(view, ui_state, id);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto selected_layer = static_cast<wire::core::SpanLayer>(ui_state.bundle_template_default_layer);
      if (ImGui::BeginCombo("Bundle Default Layer", SpanLayerLabel(selected_layer))) {
        constexpr wire::core::SpanLayer kLayers[] = {
            wire::core::SpanLayer::kHighVoltage, wire::core::SpanLayer::kLowVoltage,
            wire::core::SpanLayer::kCommunication, wire::core::SpanLayer::kOptical,
            wire::core::SpanLayer::kDrop};
        for (wire::core::SpanLayer layer : kLayers) {
          const bool selected = (layer == selected_layer);
          if (ImGui::Selectable(SpanLayerLabel(layer), selected)) {
            ui_state.bundle_template_default_layer = static_cast<int>(layer);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      ImGui::Checkbox("Bundle Allow Mirror", &ui_state.bundle_template_allow_mirror);
      ImGui::Checkbox("Bundle Allow Midair Node", &ui_state.bundle_template_allow_midair_node);
      ImGui::Checkbox("Bundle Allow Midair Branch", &ui_state.bundle_template_allow_midair_branch);
      ImGui::InputDouble("Bundle Grouped Fanout Spacing", &ui_state.bundle_template_grouped_support_fanout_spacing,
                         0.01, 0.05, "%.3f");
      const auto selected_support_style =
          static_cast<wire::core::BundleSupportStyleHint>(ui_state.bundle_template_support_style);
      if (ImGui::BeginCombo("Bundle Support Style", BundleSupportStyleLabel(selected_support_style))) {
        for (int raw = static_cast<int>(wire::core::BundleSupportStyleHint::kAuto);
             raw <= static_cast<int>(wire::core::BundleSupportStyleHint::kSideStructurePreferred); ++raw) {
          const auto style = static_cast<wire::core::BundleSupportStyleHint>(raw);
          const bool selected = (raw == ui_state.bundle_template_support_style);
          if (ImGui::Selectable(BundleSupportStyleLabel(style), selected)) {
            ui_state.bundle_template_support_style = raw;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto selected_branch_policy =
          static_cast<wire::core::BundleBranchPolicyHint>(ui_state.bundle_template_branch_policy);
      if (ImGui::BeginCombo("Bundle Branch Policy", BundleBranchPolicyLabel(selected_branch_policy))) {
        for (int raw = static_cast<int>(wire::core::BundleBranchPolicyHint::kAuto);
             raw <= static_cast<int>(wire::core::BundleBranchPolicyHint::kPreferExplicitBranch); ++raw) {
          const auto policy = static_cast<wire::core::BundleBranchPolicyHint>(raw);
          const bool selected = (raw == ui_state.bundle_template_branch_policy);
          if (ImGui::Selectable(BundleBranchPolicyLabel(policy), selected)) {
            ui_state.bundle_template_branch_policy = raw;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto selected_bundle_continuity =
          static_cast<wire::core::CableContinuityPolicyHint>(ui_state.bundle_template_continuity_policy);
      if (ImGui::BeginCombo("Bundle Continuity", ContinuityPolicyLabel(selected_bundle_continuity))) {
        for (int raw = static_cast<int>(wire::core::CableContinuityPolicyHint::kAuto);
             raw <= static_cast<int>(wire::core::CableContinuityPolicyHint::kPreferG2); ++raw) {
          const auto policy = static_cast<wire::core::CableContinuityPolicyHint>(raw);
          const bool selected = (raw == ui_state.bundle_template_continuity_policy);
          if (ImGui::Selectable(ContinuityPolicyLabel(policy), selected)) {
            ui_state.bundle_template_continuity_policy = raw;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      if (ImGui::Button("Apply Bundle Template")) {
        wire::core::BundleTemplate tpl = it->second;
        tpl.cable_template_id = ui_state.bundle_template_cable_template_id;
        tpl.related_pole_type_id = ui_state.selected_pole_template_id;
        tpl.default_layer = static_cast<wire::core::SpanLayer>(ui_state.bundle_template_default_layer);
        tpl.allow_mirror = ui_state.bundle_template_allow_mirror;
        tpl.allow_midair_node = ui_state.bundle_template_allow_midair_node;
        tpl.allow_midair_branch = ui_state.bundle_template_allow_midair_branch;
        tpl.support_style = static_cast<wire::core::BundleSupportStyleHint>(ui_state.bundle_template_support_style);
        tpl.branch_policy = static_cast<wire::core::BundleBranchPolicyHint>(ui_state.bundle_template_branch_policy);
        tpl.grouped_support_fanout_spacing_m = ui_state.bundle_template_grouped_support_fanout_spacing;
        tpl.continuity_policy =
            static_cast<wire::core::CableContinuityPolicyHint>(ui_state.bundle_template_continuity_policy);
        const auto apply = viewer_core_state::UpdateBundleTemplate(state, tpl);
        if (!apply.ok) {
          ui_state.last_error = apply.error;
          PushLog(ui_state, "UpdateBundleTemplate failed");
        } else {
          const auto apply_related_poles =
              viewer_core_state::ApplyBundleRelatedPoleTypeToExistingPoles(state, tpl.id);
          if (!apply_related_poles.ok) {
            ui_state.last_error = apply_related_poles.error;
            PushLog(ui_state, "ApplyBundleRelatedPoleTypeToExistingPoles failed");
            return;
          }
          ui_state.last_error.clear();
          const auto& deps = viewer_core_state::View(state).template_dependency_state();
          if (!deps.bundles_requiring_regeneration.empty() || !deps.sessions_requiring_regeneration.empty()) {
            PushLog(ui_state, "Bundle template updated; regeneration required bundles=" +
                                      std::to_string(deps.bundles_requiring_regeneration.size()) + " sessions=" +
                                      std::to_string(deps.sessions_requiring_regeneration.size()));
          } else {
            PushLog(ui_state, "Bundle template updated; dirty spans=" +
                                      std::to_string(apply.change_set.dirty_span_ids.size()) + " related apply updates=" +
                                      std::to_string(apply_related_poles.change_set.updated_ids.size()));
          }
          LoadBundleTemplateState(viewer_core_state::View(state), ui_state, tpl.id);
        }
      }
    }
    ImGui::Separator();
    const auto pole_type_ids = SortedPoleTypeIds(view);
    if (!pole_type_ids.empty()) {
      const auto selected_pole_it = view.pole_types().find(ui_state.selected_pole_template_id);
      ImGui::TextUnformatted("Bundle-linked Pole Template");
      ImGui::Text("Editing: %s",
                  selected_pole_it != view.pole_types().end() ? selected_pole_it->second.name.c_str() : "(missing)");

      if (selected_pole_it != view.pole_types().end()) {
        if (ui_state.pole_template_edit.id != selected_pole_it->first ||
            ui_state.pole_template_edit.id == wire::core::kInvalidPoleTypeId) {
          LoadPoleTemplateState(view, ui_state, selected_pole_it->first);
        }
        wire::core::PoleTypeDefinition& pole_type = ui_state.pole_template_edit;
        InputTextString("Pole Template Name", &pole_type.name);
        InputTextString("Pole Template Description", &pole_type.description);
        ImGui::InputDouble("Pole Default Height", &pole_type.default_height_m, 0.05, 0.10, "%.3f");

        if (ImGui::CollapsingHeader("Pole Placement", ImGuiTreeNodeFlags_DefaultOpen)) {
          constexpr wire::core::ConnectionCategory kEditableCategories[] = {
              wire::core::ConnectionCategory::kHighVoltage, wire::core::ConnectionCategory::kLowVoltage,
              wire::core::ConnectionCategory::kCommunication, wire::core::ConnectionCategory::kOptical,
              wire::core::ConnectionCategory::kDrop};
          if (ImGui::BeginTable("##pole_placement_table", 4,
                                ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame |
                                    ImGuiTableFlags_ScrollY,
                                ImVec2(0.0f, 220.0f))) {
            ImGui::TableSetupColumn("Category");
            ImGui::TableSetupColumn("Height");
            ImGui::TableSetupColumn("Offset");
            ImGui::TableSetupColumn("Spread");
            ImGui::TableHeadersRow();

            for (auto category : kEditableCategories) {
              if (!PoleTypeHasCategory(pole_type, category)) {
                continue;
              }
              ImGui::PushID(static_cast<int>(category));
              ImGui::TableNextRow();

              ImGui::TableNextColumn();
              ImGui::TextUnformatted(CategoryLabel(category));

              ImGui::TableNextColumn();
              double height_m = AverageBandHeightForCategory(pole_type, category, pole_type.default_height_m);
              if (ImGui::InputDouble("##height", &height_m, 0.05, 0.10, "%.3f")) {
                SetBandCategoryHeights(&pole_type, category, height_m);
              }

              ImGui::TableNextColumn();
              double lateral_center_m = AverageBandLateralForCategory(pole_type, category, 0.0);
              if (ImGui::InputDouble("##offset", &lateral_center_m, 0.02, 0.05, "%.3f")) {
                SetBandCategoryLateralCenter(&pole_type, category, lateral_center_m);
              }

              ImGui::TableNextColumn();
              double lateral_spread_m =
                  MaxBandLateralSpreadForCategory(pole_type, category,
                                                  AverageBandLateralForCategory(pole_type, category, 0.0));
              if (ImGui::InputDouble("##spread", &lateral_spread_m, 0.02, 0.05, "%.3f")) {
                SetBandCategorySpread(&pole_type, category, std::max(0.0, lateral_spread_m));
              }
              ImGui::PopID();
            }
            ImGui::EndTable();
          }
        }

        if (ImGui::CollapsingHeader("Advanced Port Bands")) {
          int remove_band_index = -1;
          for (std::size_t i = 0; i < pole_type.port_bands.size(); ++i) {
            auto& band = pole_type.port_bands[i];
            ImGui::PushID(static_cast<int>(i));
            std::ostringstream label;
            label << "Band " << band.band_id;
            if (ImGui::TreeNode(label.str().c_str())) {
              ImGui::InputInt("Band Id", &band.band_id);
              if (ImGui::BeginCombo("Category", CategoryLabel(band.category))) {
                constexpr wire::core::ConnectionCategory kCategories[] = {
                    wire::core::ConnectionCategory::kHighVoltage, wire::core::ConnectionCategory::kLowVoltage,
                    wire::core::ConnectionCategory::kCommunication, wire::core::ConnectionCategory::kOptical,
                    wire::core::ConnectionCategory::kDrop};
                for (auto category : kCategories) {
                  const bool selected = (category == band.category);
                  if (ImGui::Selectable(CategoryLabel(category), selected)) {
                    band.category = category;
                  }
                  if (selected) {
                    ImGui::SetItemDefaultFocus();
                  }
                }
                ImGui::EndCombo();
              }
              ImGui::InputInt("Layer", &band.layer);
              if (ImGui::BeginCombo("Side", SlotSideLabel(band.side))) {
                constexpr wire::core::SlotSide kSides[] = {wire::core::SlotSide::kLeft,
                                                           wire::core::SlotSide::kCenter,
                                                           wire::core::SlotSide::kRight};
                for (auto side : kSides) {
                  const bool selected = (side == band.side);
                  if (ImGui::Selectable(SlotSideLabel(side), selected)) {
                    band.side = side;
                  }
                  if (selected) {
                    ImGui::SetItemDefaultFocus();
                  }
                }
                ImGui::EndCombo();
              }
              if (ImGui::BeginCombo("Role", SlotRoleLabel(band.role))) {
                constexpr wire::core::SlotRole kRoles[] = {
                    wire::core::SlotRole::kNeutral, wire::core::SlotRole::kTrunkPreferred,
                    wire::core::SlotRole::kBranchPreferred, wire::core::SlotRole::kDropPreferred};
                for (auto role : kRoles) {
                  const bool selected = (role == band.role);
                  if (ImGui::Selectable(SlotRoleLabel(role), selected)) {
                    band.role = role;
                  }
                  if (selected) {
                    ImGui::SetItemDefaultFocus();
                  }
                }
                ImGui::EndCombo();
              }
              ImGui::InputDouble("Lateral Center", &band.lateral_center_m, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Lateral Min", &band.lateral_min_m, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Lateral Max", &band.lateral_max_m, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Height Center", &band.height_center_m, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Height Min", &band.height_min_m, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Height Max", &band.height_max_m, 0.01, 0.05, "%.3f");
              ImGui::InputInt("Priority", &band.priority);
              ImGui::InputDouble("Min Spacing", &band.min_spacing_m, 0.01, 0.05, "%.3f");
              ImGui::Checkbox("Allow Multiple", &band.allow_multiple);
              if (ImGui::BeginCombo("Overflow Policy", BandOverflowPolicyLabel(band.overflow_policy))) {
                constexpr wire::core::BandOverflowPolicy kPolicies[] = {
                    wire::core::BandOverflowPolicy::kTrySiblingBand,
                    wire::core::BandOverflowPolicy::kRaiseHeight,
                    wire::core::BandOverflowPolicy::kConstrainedFallback};
                for (auto policy : kPolicies) {
                  const bool selected = (policy == band.overflow_policy);
                  if (ImGui::Selectable(BandOverflowPolicyLabel(policy), selected)) {
                    band.overflow_policy = policy;
                  }
                  if (selected) {
                    ImGui::SetItemDefaultFocus();
                  }
                }
                ImGui::EndCombo();
              }
              ImGui::Checkbox("Enabled", &band.enabled);
              if (ImGui::Button("Remove Band")) {
                remove_band_index = static_cast<int>(i);
              }
              ImGui::TreePop();
            }
            ImGui::PopID();
          }
          if (remove_band_index >= 0) {
            pole_type.port_bands.erase(pole_type.port_bands.begin() + remove_band_index);
          }
          if (ImGui::Button("Add Port Band")) {
            wire::core::PortPlacementBand band{};
            int next_band_id = 1;
            for (const auto& existing : pole_type.port_bands) {
              next_band_id = std::max(next_band_id, existing.band_id + 1);
            }
            band.band_id = next_band_id;
            pole_type.port_bands.push_back(band);
          }
        }

        if (ImGui::CollapsingHeader("Advanced Anchor Slots")) {
          int remove_anchor_index = -1;
          for (std::size_t i = 0; i < pole_type.anchor_slots.size(); ++i) {
            auto& slot = pole_type.anchor_slots[i];
            ImGui::PushID(static_cast<int>(i + 1000));
            std::ostringstream label;
            label << "Anchor Slot " << slot.slot_id;
            if (ImGui::TreeNode(label.str().c_str())) {
              ImGui::InputInt("Slot Id", &slot.slot_id);
              if (ImGui::BeginCombo("Usage", AnchorSupportKindLabel(slot.usage))) {
                constexpr wire::core::AnchorSupportKind kUsages[] = {
                    wire::core::AnchorSupportKind::kGeneric, wire::core::AnchorSupportKind::kGround,
                    wire::core::AnchorSupportKind::kBuilding, wire::core::AnchorSupportKind::kMidair};
                for (auto usage : kUsages) {
                  const bool selected = (usage == slot.usage);
                  if (ImGui::Selectable(AnchorSupportKindLabel(usage), selected)) {
                    slot.usage = usage;
                  }
                  if (selected) {
                    ImGui::SetItemDefaultFocus();
                  }
                }
                ImGui::EndCombo();
              }
              ImGui::InputDouble("Local X", &slot.local_position.x, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Local Y", &slot.local_position.y, 0.01, 0.05, "%.3f");
              ImGui::InputDouble("Local Z", &slot.local_position.z, 0.01, 0.05, "%.3f");
              ImGui::InputInt("Anchor Priority", &slot.priority);
              ImGui::Checkbox("Anchor Enabled", &slot.enabled);
              if (ImGui::Button("Remove Anchor Slot")) {
                remove_anchor_index = static_cast<int>(i);
              }
              ImGui::TreePop();
            }
            ImGui::PopID();
          }
          if (remove_anchor_index >= 0) {
            pole_type.anchor_slots.erase(pole_type.anchor_slots.begin() + remove_anchor_index);
          }
          if (ImGui::Button("Add Anchor Slot")) {
            wire::core::AnchorSlotTemplate slot{};
            int next_slot_id = 1;
            for (const auto& existing : pole_type.anchor_slots) {
              next_slot_id = std::max(next_slot_id, existing.slot_id + 1);
            }
            slot.slot_id = next_slot_id;
            pole_type.anchor_slots.push_back(slot);
          }
        }

        if (ImGui::Button("Apply Pole Template")) {
          const auto apply = viewer_core_state::UpdatePoleTypeDefinition(state, pole_type);
          if (!apply.ok) {
            ui_state.last_error = apply.error;
            PushLog(ui_state, "UpdatePoleTypeDefinition failed");
          } else {
            wire::core::EditResult<bool> apply_related_poles{};
            apply_related_poles.ok = true;
            apply_related_poles.value = false;
            if (view.bundle_templates().find(ui_state.selected_bundle_template_id) != view.bundle_templates().end()) {
              apply_related_poles = viewer_core_state::ApplyBundleRelatedPoleTypeToExistingPoles(
                  state, ui_state.selected_bundle_template_id);
              if (!apply_related_poles.ok) {
                ui_state.last_error = apply_related_poles.error;
                PushLog(ui_state, "ApplyBundleRelatedPoleTypeToExistingPoles failed");
                return;
              }
            }
            ui_state.last_error.clear();
            LoadPoleTemplateState(viewer_core_state::View(state), ui_state, pole_type.id);
            PushLog(ui_state, "Pole template updated; dirty spans=" +
                                  std::to_string(apply.change_set.dirty_span_ids.size()) + " related apply updates=" +
                                  std::to_string(apply_related_poles.change_set.updated_ids.size()));
          }
        }
      }
    } else {
      ImGui::TextUnformatted("No pole templates");
    }
    const auto& template_deps = view.template_dependency_state();
    ImGui::Text("Regen Required Bundles: %d", static_cast<int>(template_deps.bundles_requiring_regeneration.size()));
    ImGui::Text("Regen Required Sessions: %d", static_cast<int>(template_deps.sessions_requiring_regeneration.size()));

    ImGui::Separator();
    ImGui::Checkbox("Enable Support Structures", &ui_state.visual_enable_support_structures);
    ImGui::Checkbox("Enable Insulators", &ui_state.visual_enable_insulators);
    ImGui::Checkbox("Solid Support Render", &ui_state.viewer_enable_solid_support_render);
    ImGui::InputDouble("Support Center Threshold", &ui_state.visual_support_center_threshold, 0.005, 0.01, "%.3f");
    ImGui::InputDouble("Support Arm Extra", &ui_state.visual_support_arm_extra, 0.01, 0.05, "%.3f");
    ImGui::InputDouble("Insulator Radius", &ui_state.visual_insulator_radius, 0.005, 0.01, "%.3f");
    ImGui::InputDouble("Insulator Length", &ui_state.visual_insulator_length, 0.005, 0.01, "%.3f");
    if (ImGui::Button("Apply Visual Cache Settings")) {
      wire::core::VisualSettings settings{};
      settings.enable_support_structures = ui_state.visual_enable_support_structures;
      settings.enable_insulators = ui_state.visual_enable_insulators;
      settings.support_center_threshold_m = ui_state.visual_support_center_threshold;
      settings.support_arm_extra_m = ui_state.visual_support_arm_extra;
      settings.insulator_radius_m = ui_state.visual_insulator_radius;
      settings.insulator_length_m = ui_state.visual_insulator_length;
      const auto apply = viewer_core_state::UpdateVisualSettings(state, settings, true);
      if (!apply.ok) {
        ui_state.last_error = apply.error;
        PushLog(ui_state, "UpdateVisualSettings failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Visual cache settings updated");
      }
    }
  }

  if (ImGui::CollapsingHeader("Debug View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Show Span AABB", &ui_state.show_whole_aabb);
    ImGui::Checkbox("Show Segment AABB", &ui_state.show_segment_aabb);
    ImGui::Checkbox("Highlight Selected Bundle", &ui_state.show_selected_bundle_highlight);
    wire::core::CommitOptions options{};
    options.run_recalc = false;
    options.run_validate = true;
    const wire::core::ValidationResult validation = viewer_core_state::Commit(state, options).validation;
    ImGui::Text("Validation: %s", validation.ok() ? "OK" : "ERROR");
  }

  if (ImGui::CollapsingHeader("Backbone Junction Debug")) {
    const wire::core::BackboneResult backbone = viewer_core_state::BuildSavedBackboneResult(state);
    ImGui::Text("SupportNodes: %d", static_cast<int>(backbone.nodes.size()));
    ImGui::Text("Edges: %d", static_cast<int>(backbone.edges.size()));
    ImGui::Text("Junctions(deg>=3): %d", static_cast<int>(backbone.junctions.size()));
    for (const auto& node : backbone.nodes) {
      ImGui::Text("Node=%llu kind=%s pole=%llu pos=(%.2f,%.2f,%.2f)", static_cast<unsigned long long>(node.node_id),
                  SupportKindLabel(node.support_kind), static_cast<unsigned long long>(node.pole_id), node.position.x,
                  node.position.y, node.position.z);
      if (!node.bundle_modes.empty()) {
        for (const auto& mode : node.bundle_modes) {
          ImGui::Text("  bundle=%d mode=%s", static_cast<int>(mode.bundle_template_id), BundleNodeModeLabel(mode.mode));
        }
      }
    }
    for (const auto& junction : backbone.junctions) {
      ImGui::Separator();
      ImGui::Text("Node=%llu session=%llu", static_cast<unsigned long long>(junction.node_id),
                  static_cast<unsigned long long>(junction.prioritized_session_id));
      for (const auto& inc : junction.incidents) {
        ImGui::Text("  -> neighbor=%llu order=%d primary=%s srcSession=%llu",
                    static_cast<unsigned long long>(inc.neighbor_node_id), inc.order,
                    inc.primary ? "true" : "false", static_cast<unsigned long long>(inc.source_session_id));
      }
    }
    if (!backbone.edge_orientations.empty()) {
      ImGui::Separator();
      ImGui::TextUnformatted("Edge Orientations");
      for (const auto& orientation : backbone.edge_orientations) {
        ImGui::Text("  %llu -> %llu bundle=%d flow=%s lowering=%s mirror=%s branchSupport=%s down=%.2f flipPrev=%s",
                    static_cast<unsigned long long>(orientation.node_a_id),
                    static_cast<unsigned long long>(orientation.node_b_id),
                    static_cast<int>(orientation.bundle_template_id), BackboneFlowKindLabel(orientation.flow_kind),
                    BackboneLoweringKindLabel(orientation.lowering_kind),
                    orientation.orientation == wire::core::LaneOrientation::kReversed ? "true" : "false",
                    orientation.uses_branch_support ? "true" : "false", orientation.branch_down_offset_m,
                    orientation.flipped_from_previous ? "true" : "false");
        ImGui::Text("    flowRule=%s", BackboneFlowDecisionRuleLabel(orientation.flow_decision_rule));
      }
    }
  }

  if (!ui_state.last_error.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Error: %s", ui_state.last_error.c_str());
  }
  ImGui::Separator();
  ImGui::BeginChild("LogArea", ImVec2(0.0f, 90.0f), true, ImGuiWindowFlags_HorizontalScrollbar);
  for (const std::string& line : ui_state.logs) {
    ImGui::TextWrapped("%s", line.c_str());
  }
  ImGui::EndChild();
}

void DrawDiagnosticsWindow(CoreState& state, ViewerUiState& ui_state) {
  const float w = static_cast<float>(GetScreenWidth());
  const float h = static_cast<float>(GetScreenHeight());
  ImGui::SetNextWindowPos(ImVec2(std::max(440.0f, w - 430.0f), std::max(90.0f, h - 360.0f)), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 340.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Diagnostics", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawDiagnosticsContent(state, ui_state);
  ImGui::End();
}

void DrawUnifiedWorkspaceWindow(CoreState& state, ViewerUiState& ui_state) {
  if (!ui_state.ui_show_workspace) {
    return;
  }
  const auto view = viewer_core_state::View(state);
  const wire::core::BackboneResult backbone = viewer_core_state::BuildSavedBackboneResult(state);
  const float screen_w = static_cast<float>(GetScreenWidth());
  const float screen_h = static_cast<float>(GetScreenHeight());
  const float topbar_h = 74.0f;
  const float margin = 8.0f;
  const float min_w = 300.0f;
  const float max_w = std::max(min_w, screen_w - margin * 2.0f);
  if (ui_state.ui_workspace_width <= 1.0f) {
    ui_state.ui_workspace_width = std::clamp(screen_w * 0.36f, min_w, std::min(760.0f, max_w));
  }
  ui_state.ui_workspace_width = std::clamp(ui_state.ui_workspace_width, min_w, std::min(760.0f, max_w));
  const float workspace_w = ui_state.ui_workspace_width;
  const float x = std::max(margin, screen_w - workspace_w - margin);
  const float y = topbar_h + margin + 8.0f;
  const float h = std::max(240.0f, screen_h - y - margin);

  ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(workspace_w, h), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(min_w, 240.0f), ImVec2(std::min(760.0f, max_w), h));
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
  if (!ImGui::Begin("Workspace", nullptr, flags)) {
    ImGui::End();
    return;
  }
  ui_state.ui_workspace_width = ImGui::GetWindowSize().x;
  if (ImGui::BeginTabBar("WorkspaceTabs")) {
    if (ImGui::BeginTabItem("Toolbox")) {
      DrawToolboxContent(state, ui_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Inspector")) {
      DrawInspectorContent(state, ui_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Outliner")) {
      DrawOutlinerContent(view, backbone, ui_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Diagnostics")) {
      DrawDiagnosticsContent(state, ui_state);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::End();
}

} // namespace

void DrawStatsPanel(CoreState& state, ViewerUiState& ui_state) {
  DrawTopbarWindow(viewer_core_state::View(state), ui_state);
  if (ui_state.ui_unified_workspace) {
    DrawUnifiedWorkspaceWindow(state, ui_state);
  } else {
    DrawToolboxWindow(state, ui_state);
    DrawInspectorWindow(state, ui_state);
    DrawOutlinerWindow(state, ui_state);
    DrawDiagnosticsWindow(state, ui_state);
  }
}
