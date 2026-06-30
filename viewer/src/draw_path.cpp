#include "app_state.hpp"
#include "core_state_adapter.hpp"
#include "path_pick_policy.hpp"
#include "ui_common.hpp"

#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <format>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <type_traits>
#include <utility>

#include "imgui.h"
#include "scene_query.hpp"

namespace {

template <typename Enum>
constexpr auto EnumOrdinal(Enum value) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(value);
}

const char* CategoryLabelLocal(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kLowVoltage:
    return "LowVoltage";
  case wire::core::ConnectionCategory::kHighVoltage:
    return "HighVoltage";
  case wire::core::ConnectionCategory::kCommunication:
    return "Communication";
  case wire::core::ConnectionCategory::kOptical:
    return "Optical";
  case wire::core::ConnectionCategory::kDrop:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* ContextLabelLocal(wire::core::ConnectionContext context) {
  switch (context) {
  case wire::core::ConnectionContext::kTrunkContinue:
    return "TrunkContinue";
  case wire::core::ConnectionContext::kCornerPass:
    return "CornerPass";
  case wire::core::ConnectionContext::kBranchAdd:
    return "BranchAdd";
  case wire::core::ConnectionContext::kDropAdd:
    return "DropAdd";
  default:
    return "Unknown";
  }
}

const char* PathDirectionModeLabelLocal(wire::core::PathDirectionMode mode) {
  switch (mode) {
  case wire::core::PathDirectionMode::kAuto:
    return "Auto";
  case wire::core::PathDirectionMode::kForward:
    return "Forward";
  case wire::core::PathDirectionMode::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* PathDirectionChosenLabelLocal(wire::core::PathDirectionChosen chosen) {
  switch (chosen) {
  case wire::core::PathDirectionChosen::kForward:
    return "Forward";
  case wire::core::PathDirectionChosen::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* CableMaterialStyleLabelLocal(wire::core::CableMaterialStyleKind kind) {
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

const char* CableAttachmentStyleLabelLocal(wire::core::CableAttachmentStyleHint kind) {
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

const char* StyleObjectKindLabelLocal(wire::core::StyleObjectKind kind) {
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

const char* SupportKindLabelLocal(wire::core::SupportKind kind) {
  switch (kind) {
  case wire::core::SupportKind::kPole:
    return "Pole";
  case wire::core::SupportKind::kMidair:
    return "Midair";
  case wire::core::SupportKind::kExternal:
    return "External";
  default:
    return "Unknown";
  }
}

const char* PickHitKindLabelLocal(wire::core::PickHitKind kind) {
  switch (kind) {
  case wire::core::PickHitKind::kNode:
    return "Node";
  case wire::core::PickHitKind::kSegment:
    return "Segment";
  case wire::core::PickHitKind::kGround:
    return "Ground";
  case wire::core::PickHitKind::kExternal:
    return "External";
  default:
    return "Empty";
  }
}

const char* LoweringKindLabelLocal(wire::core::BackboneLoweringKind kind) {
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

const char* FlowKindLabelLocal(wire::core::BackboneFlowKind kind) {
  switch (kind) {
  case wire::core::BackboneFlowKind::kMain:
    return "Main";
  case wire::core::BackboneFlowKind::kBranch:
    return "Branch";
  default:
    return "Unknown";
  }
}

const char* FlowRuleLabelLocal(wire::core::BackboneFlowDecisionRule rule) {
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

const char* JunctionRelationLabelLocal(wire::core::JunctionRelationKind kind) {
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

const char* ContinuityClassLabelLocal(wire::core::ContinuityCategoryClass continuity_class) {
  switch (continuity_class) {
  case wire::core::ContinuityCategoryClass::kPointLike:
    return "PointLike";
  case wire::core::ContinuityCategoryClass::kBundleLike:
    return "BundleLike";
  default:
    return "Unknown";
  }
}

const char* SameLevelReasonLabelLocal(wire::core::SameLevelFeasibilityReason reason) {
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

const char* OrderDecisionPolicyLabelLocal(wire::core::OrderDecisionPolicyKind policy) {
  switch (policy) {
  case wire::core::OrderDecisionPolicyKind::kFixedOrder:
    return "FixedOrder";
  case wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous:
    return "PermutableHomogeneous";
  default:
    return "Unknown";
  }
}

const char* OrderDecisionChoiceLabelLocal(wire::core::OrderDecisionChoiceKind choice) {
  switch (choice) {
  case wire::core::OrderDecisionChoiceKind::kNormal:
    return "Normal";
  case wire::core::OrderDecisionChoiceKind::kReversed:
    return "Reversed";
  default:
    return "Unknown";
  }
}

const char* OrderDecisionChoiceReasonLabelLocal(wire::core::OrderDecisionChoiceReason reason) {
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

const char* LateralSideChoiceLabelLocal(wire::core::LateralSideChoiceKind choice) {
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

const char* SideAssignmentRuleLabelLocal(wire::core::SideAssignmentRuleKind rule) {
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

const char* SupportOrientationRuleLabelLocal(wire::core::SupportOrientationRuleKind rule) {
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

const char* SupportOrientationBasisLabelLocal(wire::core::SupportOrientationBasisKind basis) {
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

const char* SupportGroupingRuleLabelLocal(wire::core::SupportGroupingRuleKind rule) {
  switch (rule) {
  case wire::core::SupportGroupingRuleKind::kPerPort:
    return "PerPort";
  case wire::core::SupportGroupingRuleKind::kDecisionGroup:
    return "DecisionGroup";
  default:
    return "Unknown";
  }
}

const char* DecisionTraceTopicLabelLocal(wire::core::DecisionTraceTopic topic) {
  switch (topic) {
  case wire::core::DecisionTraceTopic::kPoleOrientation:
    return "PoleOrientation";
  case wire::core::DecisionTraceTopic::kFlowClassification:
    return "FlowClassification";
  case wire::core::DecisionTraceTopic::kSpanLayout:
    return "SpanLayout";
  case wire::core::DecisionTraceTopic::kTangentGeneration:
    return "TangentGeneration";
  case wire::core::DecisionTraceTopic::kContinuitySelection:
    return "ContinuitySelection";
  case wire::core::DecisionTraceTopic::kContinuityDegrade:
    return "ContinuityDegrade";
  case wire::core::DecisionTraceTopic::kSagProfile:
    return "SagProfile";
  case wire::core::DecisionTraceTopic::kOverrideResolution:
    return "OverrideResolution";
  default:
    return "Unknown";
  }
}

const char* ModeLabelLocal(EditMode mode) {
  switch (mode) {
  case EditMode::kDrawPath:
    return "DrawPath";
  default:
    return "Unknown";
  }
}

const char* SelectedTypeLabelLocal(SelectedType selected_type) {
  switch (selected_type) {
  case SelectedType::kNone:
    return "None";
  case SelectedType::kPole:
    return "Pole";
  case SelectedType::kPort:
    return "Port";
  case SelectedType::kSpan:
    return "Span";
  case SelectedType::kAnchor:
    return "Anchor";
  case SelectedType::kBundle:
    return "Bundle";
  case SelectedType::kAttachment:
    return "Attachment";
  case SelectedType::kSupportNode:
    return "SupportNode";
  default:
    return "Unknown";
  }
}

std::optional<wire::core::EntityRef> SelectedEntityRefLocal(const ViewerUiState& ui_state) {
  using wire::core::EntityKind;
  switch (ui_state.selected_type) {
  case SelectedType::kPole:
    return wire::core::EntityRef{EntityKind::kPole, ui_state.selected_id};
  case SelectedType::kSpan:
    return wire::core::EntityRef{EntityKind::kSpan, ui_state.selected_id};
  case SelectedType::kSupportNode:
    return wire::core::EntityRef{EntityKind::kSupportNode, ui_state.selected_id};
  case SelectedType::kSpanLayout:
    return wire::core::EntityRef{EntityKind::kSpanLayout, ui_state.selected_id};
  case SelectedType::kDetailCurve:
    return wire::core::EntityRef{EntityKind::kDetailCurve, ui_state.selected_id};
  case SelectedType::kJunction:
    return wire::core::EntityRef{EntityKind::kJunction, ui_state.selected_id};
  default:
    return std::nullopt;
  }
}

std::string BundleTemplatePreviewOrIdLocal(const wire::core::CoreView& view, wire::core::BundleKind kind) {
  const auto it = view.bundle_templates().find(kind);
  if (it == view.bundle_templates().end()) {
    return std::to_string(static_cast<int>(kind));
  }
  return it->second.name;
}

std::string GeneratedEndpointSourceSummaryLocal(const wire::core::CoreView& view, const std::vector<ObjectId>& span_ids) {
  int plain = 0;
  int socket = 0;
  int socket_override = 0;
  int fallback = 0;
  int attachment_inputs = 0;
  for (ObjectId span_id : span_ids) {
    const auto layout_view = view.span_layout(span_id);
    if (!layout_view.has_layout()) {
      continue;
    }
    const auto accumulate = [&](const wire::core::LayoutEndpoint& endpoint) {
      if (endpoint.attachment_request.kind != wire::core::EndpointAttachmentRequestKind::kNone) {
        ++attachment_inputs;
      }
      if (endpoint.endpoint_source == wire::core::LayoutEndpointSourceKind::kPlainSupport) {
        ++plain;
      } else if (endpoint.endpoint_source == wire::core::LayoutEndpointSourceKind::kAttachmentSocket) {
        ++socket;
      } else if (endpoint.endpoint_source == wire::core::LayoutEndpointSourceKind::kAttachmentSocketOverride) {
        ++socket_override;
      } else {
        ++fallback;
      }
    };
    accumulate(layout_view.entry->start);
    accumulate(layout_view.entry->end);
  }
  std::ostringstream oss;
  oss << "endpointSources plain=" << plain << " socket=" << socket << " override=" << socket_override
      << " fallback=" << fallback << " attachmentInput=" << attachment_inputs;
  return oss.str();
}

void WriteVec3CaptureLocal(std::ofstream& ofs, const std::string& key, const wire::core::Vec3d& value) {
  ofs << key << "=" << value.x << "," << value.y << "," << value.z << "\n";
}

void WriteLayoutEndpointCaptureLocal(std::ofstream& ofs, const std::string& prefix,
                                     const wire::core::LayoutEndpoint& endpoint) {
  WriteVec3CaptureLocal(ofs, prefix + ".support", endpoint.support_world);
  WriteVec3CaptureLocal(ofs, prefix + ".endpoint", endpoint.endpoint_world);
  WriteVec3CaptureLocal(ofs, prefix + ".departure", endpoint.departure_dir);
  ofs << prefix << ".origin=" << static_cast<int>(endpoint.origin) << "\n";
  ofs << prefix << ".endpoint_source=" << static_cast<int>(endpoint.endpoint_source) << "\n";
  ofs << prefix << ".port_source=" << static_cast<int>(endpoint.port_source) << "\n";
  ofs << prefix << ".local_departure=" << endpoint.local_departure_length_m << "\n";
  ofs << prefix << ".branch_down_offset_m=" << endpoint.branch_down_offset_m << "\n";
  ofs << prefix << ".automatic_branch_down_offset_m=" << endpoint.automatic_branch_down_offset_m << "\n";
  ofs << prefix << ".relation=" << JunctionRelationLabelLocal(endpoint.relation_kind) << "\n";
  ofs << prefix << ".continuity_class=" << ContinuityClassLabelLocal(endpoint.continuity_class) << "\n";
  ofs << prefix << ".lower_required=" << (endpoint.lower_required ? 1 : 0) << "\n";
  ofs << prefix << ".same_level_reason=" << SameLevelReasonLabelLocal(endpoint.same_level_reason) << "\n";
  ofs << prefix << ".order_decision_policy=" << OrderDecisionPolicyLabelLocal(endpoint.order_decision_policy) << "\n";
  ofs << prefix << ".order_decision_choice=" << OrderDecisionChoiceLabelLocal(endpoint.order_decision_choice) << "\n";
  ofs << prefix << ".order_decision_reason="
      << OrderDecisionChoiceReasonLabelLocal(endpoint.order_decision_choice_reason) << "\n";
  ofs << prefix << ".side=" << LateralSideChoiceLabelLocal(endpoint.chosen_side) << "\n";
  ofs << prefix << ".side_assignment_rule=" << SideAssignmentRuleLabelLocal(endpoint.side_assignment_rule) << "\n";
  ofs << prefix << ".support_orientation_rule="
      << SupportOrientationRuleLabelLocal(endpoint.support_orientation_rule) << "\n";
  ofs << prefix << ".support_orientation_basis="
      << SupportOrientationBasisLabelLocal(endpoint.support_orientation_basis) << "\n";
}

void WriteNeutralSpanLayoutCaptureLocal(const wire::core::CoreView& view, std::ofstream& ofs,
                                        const std::string& prefix, ObjectId span_id) {
  const auto rules_view = view.span_layout_rules(span_id);
  const auto layout_state = view.span_layout_state(span_id);
  const auto layout_view = view.span_layout(span_id);
  ofs << prefix << ".rules_present=" << (rules_view.has_rule() ? 1 : 0) << "\n";
  ofs << prefix << ".layout_present=" << (layout_view.has_layout() ? 1 : 0) << "\n";
  ofs << prefix << ".layout_state.has_rules=" << (layout_state.has_rules ? 1 : 0) << "\n";
  ofs << prefix << ".layout_state.has_layout=" << (layout_state.has_layout ? 1 : 0) << "\n";
  if (rules_view.has_rule()) {
    const wire::core::SpanLayoutRule& rule = *rules_view.rule;
    ofs << prefix << ".rule_flow_kind=" << FlowKindLabelLocal(rule.flow_kind) << "\n";
    ofs << prefix << ".rule_pass_mode=" << static_cast<int>(rule.pass_mode) << "\n";
    ofs << prefix << ".rule_lowering_kind=" << LoweringKindLabelLocal(rule.lowering_kind) << "\n";
    ofs << prefix << ".rule_start_relation=" << JunctionRelationLabelLocal(rule.start.semantic.relation_kind) << "\n";
    ofs << prefix << ".rule_end_relation=" << JunctionRelationLabelLocal(rule.end.semantic.relation_kind) << "\n";
    ofs << prefix << ".rule_start_lower_required=" << (rule.start.semantic.lower_required ? 1 : 0) << "\n";
    ofs << prefix << ".rule_end_lower_required=" << (rule.end.semantic.lower_required ? 1 : 0) << "\n";
  }
  if (layout_view.has_layout()) {
    const wire::core::SpanLayoutEntry& layout = *layout_view.entry;
    ofs << prefix << ".layout_flow_kind=" << FlowKindLabelLocal(layout.flow_kind) << "\n";
    ofs << prefix << ".layout_pass_mode=" << static_cast<int>(layout.pass_mode) << "\n";
    ofs << prefix << ".layout_lowering_kind=" << LoweringKindLabelLocal(layout.lowering_kind) << "\n";
    WriteLayoutEndpointCaptureLocal(ofs, prefix + ".layout_start", layout.start);
    WriteLayoutEndpointCaptureLocal(ofs, prefix + ".layout_end", layout.end);
    ofs << prefix << ".layout_lowered_support_key_count=" << layout.lowered_support_group_keys.size() << "\n";
    for (std::size_t key_index = 0; key_index < layout.lowered_support_group_keys.size(); ++key_index) {
      const wire::core::LoweredSupportGroupKey& key = layout.lowered_support_group_keys[key_index];
      ofs << prefix << ".layout_lowered_support_key[" << key_index
          << "].owner_pole_id=" << static_cast<unsigned long long>(key.owner_pole_id) << "\n";
      ofs << prefix << ".layout_lowered_support_key[" << key_index << "].support_group_id=" << key.support_group_id
          << "\n";
    }
  }
}

int ResolveBundleTemplateCountLocal(ViewerUiState& ui_state, const wire::core::BundleTemplate& bundle_template,
                                    wire::core::BundleKind kind) {
  return ResolveBundleTemplateCount(ui_state, bundle_template, kind);
}

std::string PoleTypePreviewLocal(const wire::core::CoreView& view, wire::core::PoleTypeId pole_type_id) {
  const auto it = view.pole_types().find(pole_type_id);
  if (it == view.pole_types().end()) {
    return std::to_string(pole_type_id);
  }
  return it->second.name;
}

std::size_t ClampedTypeIndexLocal(int current, std::size_t count) {
  if (count == 0) {
    return 0;
  }
  const int max_index = static_cast<int>(count - 1);
  return static_cast<std::size_t>(std::clamp(current, 0, max_index));
}

Vector3 ToRaylibLocal(const wire::core::Vec3d& ue_xyz) {
  return Vector3{static_cast<float>(ue_xyz.x), static_cast<float>(ue_xyz.z), static_cast<float>(ue_xyz.y)};
}

double PolylineLengthLocal(const std::vector<wire::core::Vec3d>& points) {
  return wire::core::PolylineLength(points);
}

void PushLogLocal(ViewerUiState& ui_state, const std::string& line) {
  ui_state.logs.push_back(line);
  if (ui_state.logs.size() > 256) {
    ui_state.logs.erase(ui_state.logs.begin());
  }
}

} // namespace

void EnsureDrawPathPointKinds(ViewerUiState& ui_state) {
  if (ui_state.draw_path_point_support_kinds.size() < ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_support_kinds.resize(ui_state.draw_path_points.size(), wire::core::SupportKind::kPole);
  } else if (ui_state.draw_path_point_support_kinds.size() > ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_support_kinds.resize(ui_state.draw_path_points.size());
  }
  if (ui_state.draw_path_point_node_ids.size() < ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_node_ids.resize(ui_state.draw_path_points.size(), wire::core::kInvalidObjectId);
  } else if (ui_state.draw_path_point_node_ids.size() > ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_node_ids.resize(ui_state.draw_path_points.size());
  }
}

void DrawPathPushPoint(ViewerUiState& ui_state, const wire::core::Vec3d& point, wire::core::SupportKind support_kind,
                       ObjectId node_id) {
  ui_state.draw_path_points.push_back(point);
  ui_state.draw_path_point_support_kinds.push_back(support_kind);
  ui_state.draw_path_point_node_ids.push_back(node_id);
}

void DrawPathPopPoint(ViewerUiState& ui_state) {
  if (ui_state.draw_path_points.empty()) {
    return;
  }
  ui_state.draw_path_points.pop_back();
  if (!ui_state.draw_path_point_support_kinds.empty()) {
    ui_state.draw_path_point_support_kinds.pop_back();
  }
  if (!ui_state.draw_path_point_node_ids.empty()) {
    ui_state.draw_path_point_node_ids.pop_back();
  }
}

void DrawPathClearPoints(ViewerUiState& ui_state) {
  ui_state.draw_path_points.clear();
  ui_state.draw_path_point_support_kinds.clear();
  ui_state.draw_path_point_node_ids.clear();
}

wire::core::ResolveBranchPickResult DirectResolvedDrawPathTarget(const wire::core::PickResult& pick) {
  wire::core::ResolveBranchPickResult resolved{};
  resolved.resolution = wire::core::PickBranchResolutionKind::kNode;
  resolved.resolved_node_id = pick.hit_id;
  resolved.position = pick.hit_pos_world;
  resolved.support_kind = (pick.hit_kind == wire::core::PickHitKind::kExternal) ? wire::core::SupportKind::kExternal
                                                                                 : wire::core::SupportKind::kPole;
  resolved.snapped_from_segment_endpoint = false;
  return resolved;
}

bool ExecuteBackboneRequest(CoreState& state, ViewerUiState& ui_state, const wire::core::BackboneSpec& request,
                            bool clear_draw_path_on_success, const char* success_log,
                            const char* failure_log) {
  const auto view = viewer_core_state::View(state);
  ui_state.last_draw_path_request = request;
  const auto result = viewer_core_state::GenerateFromBackboneSpec(state, request);
  if (!result.ok) {
    ui_state.last_error = result.error;
    PushLogLocal(ui_state, failure_log);
    return false;
  }

  ui_state.last_error.clear();
  ui_state.last_generated_poles = static_cast<int>(result.value.generated_pole_ids.size());
  ui_state.last_generated_spans = static_cast<int>(result.value.generated_span_ids.size());
  ui_state.last_generated_pole_ids = result.value.generated_pole_ids;
  ui_state.last_generated_span_ids = result.value.generated_span_ids;
  if (!result.value.generated_pole_ids.empty()) {
    SetPrimarySelection(ui_state, SelectedType::kPole, result.value.generated_pole_ids.back());
  } else if (!result.value.generated_span_ids.empty()) {
    SetPrimarySelection(ui_state, SelectedType::kSpan, result.value.generated_span_ids.back());
  }

  if (clear_draw_path_on_success) {
    DrawPathClearPoints(ui_state);
  }
  PushLogLocal(ui_state, std::string(success_log) + " poles=" + std::to_string(ui_state.last_generated_poles) +
                             " spans=" + std::to_string(ui_state.last_generated_spans));
  PushLogLocal(ui_state, "DrawPath attachment/socket request input: unsupported in BackboneSpec node/path request");
  PushLogLocal(ui_state, GeneratedEndpointSourceSummaryLocal(view, result.value.generated_span_ids));
  return true;
}

void ExecuteGenerateFromDrawPath(CoreState& state, ViewerUiState& ui_state, bool from_enter_key) {
  const auto view = viewer_core_state::View(state);
  EnsureDrawPathPointKinds(ui_state);
  if (ui_state.draw_path_points.size() < 2) {
    ui_state.last_error = "path needs at least 2 points";
    return;
  }

  const auto type_ids = SortedPoleTypeIds(view);
  if (type_ids.empty()) {
    ui_state.last_error = "no pole type available";
    return;
  }

  const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
  ui_state.road_pole_type_index = static_cast<int>(road_type_index);

  const auto selected_templates = SelectedBundleTemplates(view, ui_state);
  if (selected_templates.empty()) {
    ui_state.last_error = "select at least one bundle template";
    return;
  }

  const int mode_index = std::clamp(ui_state.draw_direction_mode, 0, 2);
  ui_state.draw_direction_mode = mode_index;

  wire::core::BackboneSpec request{};
  request.path.polyline = ui_state.draw_path_points;
  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    const wire::core::SupportKind support_kind = ui_state.draw_path_point_support_kinds[i];
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : wire::core::kInvalidObjectId;
    if (support_kind == wire::core::SupportKind::kPole && node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    wire::core::BackboneInputSpec::NodeSpec node_spec{};
    node_spec.point_index = i;
    node_spec.support_kind = support_kind;
    node_spec.node_id = node_id;
    request.path.node_specs.push_back(node_spec);
  }
  if (ui_state.draw_clicked_points_only) {
    request.interval_m = std::max(0.001, PolylineLengthLocal(request.path.polyline) + 1.0);
  } else {
    request.interval_m = ui_state.draw_interval_m;
  }
  request.pole_type_id = type_ids[ui_state.road_pole_type_index];
  request.direction_mode = static_cast<wire::core::PathDirectionMode>(mode_index);
  for (wire::core::BundleKind kind : selected_templates) {
    const auto it = view.bundle_templates().find(kind);
    if (it == view.bundle_templates().end()) {
      ui_state.last_error = "selected bundle template is not available";
      return;
    }
    const wire::core::BundleTemplate& bundle_template = it->second;
    wire::core::BackboneBundleSpec bundle_request{};
    bundle_request.bundle_template_id = kind;
    bundle_request.layer = wire::core::SpanLayer::kUnknown;
    bundle_request.count = 0;
    if (bundle_template.count_rule == wire::core::BundleCountRuleKind::kRange) {
      bundle_request.count = ResolveBundleTemplateCountLocal(ui_state, bundle_template, kind);
    }
    request.bundles.push_back(bundle_request);
  }

  const std::string success_log =
      std::string(from_enter_key ? "Generated path (Enter)" : "Generated path") + " templates=" +
      BundleTemplateMultiPreview(view, ui_state);
  const char* failure_log = from_enter_key ? "Generate path (Enter) failed" : "Generate path failed";
  (void)ExecuteBackboneRequest(state, ui_state, request, !ui_state.draw_keep_path_after_generate, success_log.c_str(),
                               failure_log);
}

bool SaveDrawPathReproCapture(const CoreState& state, const ViewerUiState& ui_state, std::string* out_path,
                              std::string* out_error) {
  namespace fs = std::filesystem;

  std::time_t now = std::time(nullptr);
  const auto wall_now = std::chrono::system_clock::now();
  std::tm tm_now{};
#if defined(_WIN32)
  localtime_s(&tm_now, &now);
#else
  localtime_r(&now, &tm_now);
#endif
  char stamp[32]{};
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(wall_now.time_since_epoch()).count() % 1000;

  std::error_code ec;
  const fs::path capture_dir = fs::absolute(fs::path("captures"), ec);
  if (ec) {
    if (out_error != nullptr) {
      *out_error = "failed to resolve captures directory";
    }
    return false;
  }
  fs::create_directories(capture_dir, ec);
  if (ec) {
    if (out_error != nullptr) {
      *out_error = "failed to create captures directory: " + capture_dir.string();
    }
    return false;
  }

  fs::path capture_path{};
  for (int attempt = 0; attempt < 1000; ++attempt) {
    const int suffix = (static_cast<int>(millis) + attempt) % 1000;
    std::ostringstream name{};
    name << "drawpath_repro_" << stamp << "_" << std::setw(3) << std::setfill('0') << suffix << ".txt";
    capture_path = capture_dir / fs::path(name.str());
    if (!fs::exists(capture_path, ec)) {
      break;
    }
    ec.clear();
  }

  std::ofstream ofs(capture_path.string(), std::ios::trunc);
  if (!ofs.is_open()) {
    if (out_error != nullptr) {
      *out_error = "failed to open capture file for writing: " + capture_path.string();
    }
    return false;
  }

  const auto view = viewer_core_state::View(state);
  const auto selected_templates = SelectedBundleTemplates(view, ui_state);
  const auto& dir_debug = view.last_path_direction_debug();
  const auto write_vec3 = [&](const std::string& key, const wire::core::Vec3d& value) {
    ofs << key << "=" << value.x << "," << value.y << "," << value.z << "\n";
  };
  const auto write_decision_trace = [&](const std::string& prefix, const wire::core::EntityRef& ref) {
    const auto trace = view.collect_decision_trace(ref);
    ofs << prefix << ".decision_trace_count=" << trace.size() << "\n";
    for (std::size_t index = 0; index < trace.size(); ++index) {
      ofs << prefix << ".decision_trace[" << index << "].topic="
          << DecisionTraceTopicLabelLocal(trace[index].topic) << "\n";
      ofs << prefix << ".decision_trace[" << index << "].rule=" << trace[index].rule << "\n";
      ofs << prefix << ".decision_trace[" << index << "].summary=" << trace[index].summary << "\n";
    }
  };
  const auto write_style_inspection = [&](const std::string& prefix, const wire::core::StyleInspectionView& style) {
    ofs << prefix << ".has_context=" << (style.has_context ? 1 : 0) << "\n";
    if (!style.has_context) {
      return;
    }
    ofs << prefix << ".route.family_id=" << static_cast<unsigned long long>(style.route_key.family_id) << "\n";
    ofs << prefix << ".route.bundle_template_id=" << static_cast<int>(style.route_key.bundle_template_id) << "\n";
    ofs << prefix << ".route.category=" << CategoryLabelLocal(style.route_key.category) << "\n";
    ofs << prefix << ".route.flow_kind=" << FlowKindLabelLocal(style.route_key.flow_kind) << "\n";
    ofs << prefix << ".object.segment_index=" << style.object_key.segment_index << "\n";
    ofs << prefix << ".object.lane_index=" << style.object_key.lane_index << "\n";
    ofs << prefix << ".object.kind=" << StyleObjectKindLabelLocal(style.object_key.kind) << "\n";
    ofs << prefix << ".object.ordinal=" << style.object_key.ordinal << "\n";
    ofs << prefix << ".object.is_start_endpoint=" << (style.object_key.is_start_endpoint ? 1 : 0) << "\n";
    ofs << prefix << ".scope.district_seed=" << static_cast<unsigned long long>(style.resolved.scope.district_seed) << "\n";
    ofs << prefix << ".scope.route_seed=" << static_cast<unsigned long long>(style.resolved.scope.route_seed) << "\n";
    ofs << prefix << ".scope.cluster_seed=" << static_cast<unsigned long long>(style.resolved.scope.cluster_seed) << "\n";
    ofs << prefix << ".scope.object_seed=" << static_cast<unsigned long long>(style.resolved.scope.object_seed) << "\n";
    ofs << prefix << ".profile.age=" << style.resolved.profile.age << "\n";
    ofs << prefix << ".profile.clutter=" << style.resolved.profile.clutter << "\n";
    ofs << prefix << ".profile.regularity=" << style.resolved.profile.regularity << "\n";
    ofs << prefix << ".profile.service_mix=" << style.resolved.profile.service_mix << "\n";
    ofs << prefix << ".profile.style_seed=" << static_cast<unsigned long long>(style.resolved.profile.style_seed) << "\n";
    ofs << prefix << ".district.cable_family=" << CableMaterialStyleLabelLocal(style.resolved.district.cable_family) << "\n";
    ofs << prefix << ".district.attachment_family="
        << CableAttachmentStyleLabelLocal(style.resolved.district.attachment_family) << "\n";
    ofs << prefix << ".district.age=" << style.resolved.district.age << "\n";
    ofs << prefix << ".district.clutter=" << style.resolved.district.clutter << "\n";
    ofs << prefix << ".district.regularity=" << style.resolved.district.regularity << "\n";
    ofs << prefix << ".district.service_mix=" << style.resolved.district.service_mix << "\n";
    ofs << prefix << ".route_style.cable_family=" << CableMaterialStyleLabelLocal(style.resolved.route.cable_family)
        << "\n";
    ofs << prefix << ".route_style.attachment_family="
        << CableAttachmentStyleLabelLocal(style.resolved.route.attachment_family) << "\n";
    ofs << prefix << ".route_style.age_bias=" << style.resolved.route.age_bias << "\n";
    ofs << prefix << ".route_style.clutter_bias=" << style.resolved.route.clutter_bias << "\n";
    ofs << prefix << ".route_style.regularity_bias=" << style.resolved.route.regularity_bias << "\n";
    ofs << prefix << ".route_style.service_mix_bias=" << style.resolved.route.service_mix_bias << "\n";
    ofs << prefix << ".route_style.sag_bias=" << style.resolved.route.sag_bias << "\n";
    ofs << prefix << ".cluster.index=" << style.resolved.cluster.key.cluster_index << "\n";
    ofs << prefix << ".cluster.clutter_bias=" << style.resolved.cluster.clutter_bias << "\n";
    ofs << prefix << ".cluster.service_mix_bias=" << style.resolved.cluster.service_mix_bias << "\n";
    ofs << prefix << ".cluster.family_mix=" << style.resolved.cluster.family_mix << "\n";
    ofs << prefix << ".object_variation.local_offset=" << style.resolved.object.local_offset_m.x << ","
        << style.resolved.object.local_offset_m.y << "," << style.resolved.object.local_offset_m.z << "\n";
    ofs << prefix << ".object_variation.sag_delta_m=" << style.resolved.object.sag_delta_m << "\n";
    ofs << prefix << ".object_variation.attachment_offset_m=" << style.resolved.object.attachment_offset_m << "\n";
    ofs << prefix << ".object_variation.choice_bias=" << style.resolved.object.choice_bias << "\n";
  };
      const auto write_endpoint_junction = [&](const std::string& prefix, wire::core::ObjectId node_id,
                           wire::core::ObjectId peer_node_id) {
      ofs << prefix << ".node_id=" << static_cast<unsigned long long>(node_id) << "\n";
      if (const auto junction_view = view.inspect_junction(node_id); junction_view.has_value()) {
        ofs << prefix << ".has_local_relation=" << (junction_view->has_local_relation ? 1 : 0) << "\n";
        ofs << prefix << ".through_pair_accepted=" << (junction_view->through_pair_accepted ? 1 : 0) << "\n";
        ofs << prefix << ".is_cross_like=" << (junction_view->is_cross_like ? 1 : 0) << "\n";
        ofs << prefix << ".route_incident_count=" << junction_view->route_incident_count << "\n";
        ofs << prefix << ".through_pair_neighbor_a_id="
          << static_cast<unsigned long long>(junction_view->through_pair_neighbor_a_id) << "\n";
        ofs << prefix << ".through_pair_neighbor_b_id="
          << static_cast<unsigned long long>(junction_view->through_pair_neighbor_b_id) << "\n";
        ofs << prefix << ".through_pair_straightness_score=" << junction_view->through_pair_straightness_score
          << "\n";

        const auto relation_it = std::find_if(
          junction_view->local_relations.begin(), junction_view->local_relations.end(),
          [peer_node_id](const wire::core::JunctionIncidentRelationView& relation) {
          return relation.neighbor_node_id == peer_node_id;
          });
        ofs << prefix << ".peer_relation_found=" << (relation_it != junction_view->local_relations.end() ? 1 : 0)
          << "\n";
        if (relation_it != junction_view->local_relations.end()) {
        ofs << prefix << ".peer_relation_kind=" << JunctionRelationLabelLocal(relation_it->kind) << "\n";
        ofs << prefix << ".peer_straightness_score=" << relation_it->straightness_score << "\n";
        ofs << prefix << ".peer_in_route=" << (relation_it->in_route ? 1 : 0) << "\n";
        ofs << prefix << ".peer_in_through_pair=" << (relation_it->in_through_pair ? 1 : 0) << "\n";
        ofs << prefix << ".peer_continuity_class=" << ContinuityClassLabelLocal(relation_it->continuity_class)
          << "\n";
        ofs << prefix << ".peer_default_lower_required=" << (relation_it->default_lower_required ? 1 : 0)
          << "\n";
        ofs << prefix << ".peer_same_level_feasible=" << (relation_it->same_level_feasible ? 1 : 0) << "\n";
        ofs << prefix << ".peer_infeasible_reason=" << SameLevelReasonLabelLocal(relation_it->infeasible_reason)
          << "\n";
        ofs << prefix << ".peer_projected_spacing_topview_m=" << relation_it->projected_spacing_topview_m
          << "\n";
        ofs << prefix << ".peer_required_clearance_m=" << relation_it->required_clearance_m << "\n";
        }
      }
      };
      ofs << "capture.version=7\n";
  ofs << "capture.scope=all_state_plus_focus\n";
  ofs << "capture.includes_all_current_spans=1\n";
  ofs << "draw.endpoint_attachment_input_supported=0\n";
  ofs << "draw.endpoint_socket_input_supported=0\n";
  ofs << "capture.timestamp_unix=" << static_cast<long long>(now) << "\n";
  ofs << "capture.mode=" << ModeLabelLocal(ui_state.mode) << "\n";
  ofs << "capture.selected_type=" << SelectedTypeLabelLocal(ui_state.selected_type) << "\n";
  ofs << "capture.selected_id=" << static_cast<unsigned long long>(ui_state.selected_id) << "\n";
  ofs << "capture.focus_type=" << SelectedTypeLabelLocal(ui_state.selected_type) << "\n";
  ofs << "capture.focus_id=" << static_cast<unsigned long long>(ui_state.selected_id) << "\n";
  ofs << "capture.selection_item_count=" << ui_state.selection_items.size() << "\n";
  for (std::size_t selection_index = 0; selection_index < ui_state.selection_items.size(); ++selection_index) {
    const SelectionItem& item = ui_state.selection_items[selection_index];
    ofs << "capture.selection_item[" << selection_index << "].type=" << SelectedTypeLabelLocal(item.type) << "\n";
    ofs << "capture.selection_item[" << selection_index
        << "].id=" << static_cast<unsigned long long>(item.id) << "\n";
  }
  ofs << "capture.last_error=" << ui_state.last_error << "\n";
  ofs << "capture.last_generated_poles=" << ui_state.last_generated_poles << "\n";
  ofs << "capture.last_generated_spans=" << ui_state.last_generated_spans << "\n";
  ofs << "capture.last_generated_pole_id_count=" << ui_state.last_generated_pole_ids.size() << "\n";
  for (std::size_t i = 0; i < ui_state.last_generated_pole_ids.size(); ++i) {
    ofs << "capture.last_generated_pole_id[" << i << "]="
        << static_cast<unsigned long long>(ui_state.last_generated_pole_ids[i]) << "\n";
  }
  ofs << "capture.last_generated_span_id_count=" << ui_state.last_generated_span_ids.size() << "\n";
  for (std::size_t i = 0; i < ui_state.last_generated_span_ids.size(); ++i) {
    ofs << "capture.last_generated_span_id[" << i << "]="
        << static_cast<unsigned long long>(ui_state.last_generated_span_ids[i]) << "\n";
  }
  ofs << "backbone.generated_span_count=" << ui_state.last_generated_span_ids.size() << "\n";
  for (std::size_t generated_index = 0; generated_index < ui_state.last_generated_span_ids.size(); ++generated_index) {
    const ObjectId span_id = ui_state.last_generated_span_ids[generated_index];
    const std::string prefix = "backbone.generated_span[" + std::to_string(generated_index) + "]";
    ofs << prefix << ".span_id=" << static_cast<unsigned long long>(span_id) << "\n";

    const auto* span = view.spans().find(span_id);
    ofs << prefix << ".span_exists=" << (span != nullptr ? 1 : 0) << "\n";
    if (span != nullptr) {
      ofs << prefix << ".bundle_id=" << static_cast<unsigned long long>(span->bundle_id) << "\n";
      ofs << prefix << ".node_a_id=" << static_cast<unsigned long long>(span->endpoint_node_a_id) << "\n";
      ofs << prefix << ".node_b_id=" << static_cast<unsigned long long>(span->endpoint_node_b_id) << "\n";
      ofs << prefix << ".port_a_id=" << static_cast<unsigned long long>(span->port_a_id) << "\n";
      ofs << prefix << ".port_b_id=" << static_cast<unsigned long long>(span->port_b_id) << "\n";
    }

    const auto rules_view = view.span_layout_rules(span_id);
    const auto layout_state = view.span_layout_state(span_id);
    const auto layout_view = view.span_layout(span_id);
    const auto* curve = view.find_curve_cache(span_id);
    const auto* bounds = view.find_bounds_cache(span_id);
    const auto* visual = view.find_span_visual_cache(span_id);
    const auto* render = view.find_span_render_cache(span_id);

    ofs << prefix << ".rules_present=" << (rules_view.has_rule() ? 1 : 0) << "\n";
    ofs << prefix << ".layout_present=" << (layout_view.has_layout() ? 1 : 0) << "\n";
    ofs << prefix << ".layout_state.has_rules=" << (layout_state.has_rules ? 1 : 0) << "\n";
    ofs << prefix << ".layout_state.has_layout=" << (layout_state.has_layout ? 1 : 0) << "\n";
    ofs << prefix << ".curve_present=" << (curve != nullptr ? 1 : 0) << "\n";
    ofs << prefix << ".bounds_present=" << (bounds != nullptr ? 1 : 0) << "\n";
    ofs << prefix << ".visual_present=" << (visual != nullptr ? 1 : 0) << "\n";
    ofs << prefix << ".render_present=" << (render != nullptr ? 1 : 0) << "\n";

    if (layout_view.has_layout()) {
      write_vec3(prefix + ".layout.start.support_world", layout_view.entry->start.support_world);
      write_vec3(prefix + ".layout.start.endpoint_world", layout_view.entry->start.endpoint_world);
      write_vec3(prefix + ".layout.end.support_world", layout_view.entry->end.support_world);
      write_vec3(prefix + ".layout.end.endpoint_world", layout_view.entry->end.endpoint_world);
      ofs << prefix << ".layout.lowering_kind=" << LoweringKindLabelLocal(layout_view.entry->lowering_kind) << "\n";
      ofs << prefix << ".layout.start.lower_required=" << (layout_view.entry->start.lower_required ? 1 : 0) << "\n";
      ofs << prefix << ".layout.end.lower_required=" << (layout_view.entry->end.lower_required ? 1 : 0) << "\n";
    }
    if (curve != nullptr) {
      ofs << prefix << ".curve.sample_count=" << curve->detail.sample_points.size() << "\n";
      ofs << prefix << ".curve.segment_count=" << curve->detail.segments.size() << "\n";
      ofs << prefix << ".curve.total_length_m=" << curve->detail.total_length_m << "\n";
    }
    if (bounds != nullptr) {
      write_vec3(prefix + ".bounds.min", bounds->whole.min);
      write_vec3(prefix + ".bounds.max", bounds->whole.max);
      ofs << prefix << ".bounds.segment_count=" << bounds->segments.size() << "\n";
    }
    if (visual != nullptr) {
      ofs << prefix << ".visual.part_count=" << visual->parts.size() << "\n";
    }
    if (render != nullptr) {
      ofs << prefix << ".render.segment_count=" << render->segment_length_m.size() << "\n";
      ofs << prefix << ".render.wire_radius_m=" << render->wire_radius_m << "\n";
    }

    const wire::core::BackboneFrontier frontier = view.span_frontier(span_id);
    ofs << prefix << ".frontier.node_id=" << static_cast<unsigned long long>(frontier.node_id) << "\n";
    ofs << prefix << ".frontier.edge_id=" << static_cast<unsigned long long>(frontier.edge_id) << "\n";
    ofs << prefix << ".frontier.edge_bundle_id=" << static_cast<unsigned long long>(frontier.edge_bundle_id) << "\n";
    ofs << prefix << ".frontier.node_count=" << frontier.node_ids.size() << "\n";
    ofs << prefix << ".frontier.edge_count=" << frontier.edge_ids.size() << "\n";
    ofs << prefix << ".frontier.edge_bundle_count=" << frontier.edge_bundle_ids.size() << "\n";
    ofs << prefix << ".frontier.bundle_count=" << frontier.bundle_ids.size() << "\n";
    ofs << prefix << ".frontier.span_count=" << frontier.span_ids.size() << "\n";
    ofs << prefix << ".frontier.pole_count=" << frontier.pole_ids.size() << "\n";

    const auto span_binding_it = view.backbone_index().span_bindings_by_span.find(span_id);
    const std::size_t span_binding_count =
        (span_binding_it == view.backbone_index().span_bindings_by_span.end()) ? 0 : span_binding_it->second.size();
    ofs << prefix << ".span_binding_count=" << span_binding_count << "\n";
    for (std::size_t binding_i = 0; binding_i < span_binding_count; ++binding_i) {
      const std::size_t binding_index = span_binding_it->second[binding_i];
      if (binding_index >= view.backbone().span_bindings.size()) {
        continue;
      }
      const auto& binding = view.backbone().span_bindings[binding_index];
      ofs << prefix << ".span_binding[" << binding_i
          << "].edge_bundle_id=" << static_cast<unsigned long long>(binding.edge_bundle_id) << "\n";
      ofs << prefix << ".span_binding[" << binding_i << "].lane_index=" << binding.lane_index << "\n";
    }

    if (span != nullptr) {
      const std::array<std::pair<const char*, ObjectId>, 2> endpoints = {
          std::pair<const char*, ObjectId>{"a", span->port_a_id},
          std::pair<const char*, ObjectId>{"b", span->port_b_id},
      };
      for (const auto& [label, port_id] : endpoints) {
        const auto bindings = view.backbone_port_bindings_for_port(port_id);
        ofs << prefix << ".port_" << label << "_binding_count=" << bindings.size() << "\n";
        for (std::size_t binding_i = 0; binding_i < bindings.size(); ++binding_i) {
          const auto* binding = bindings[binding_i];
          if (binding == nullptr) {
            continue;
          }
          ofs << prefix << ".port_" << label << "_binding[" << binding_i
              << "].edge_bundle_id=" << static_cast<unsigned long long>(binding->edge_bundle_id) << "\n";
          ofs << prefix << ".port_" << label << "_binding[" << binding_i << "].lane_index=" << binding->lane_index
              << "\n";
          ofs << prefix << ".port_" << label << "_binding[" << binding_i
              << "].row_node_id=" << static_cast<unsigned long long>(binding->row_key.node_id) << "\n";
          ofs << prefix << ".port_" << label << "_binding[" << binding_i
              << "].row_source_is_open=" << (binding->row_key.source_is_open ? 1 : 0) << "\n";
          ofs << prefix << ".port_" << label << "_binding[" << binding_i
              << "].bundle_template_id=" << static_cast<int>(binding->bundle_template_id) << "\n";
          ofs << prefix << ".port_" << label << "_binding[" << binding_i
              << "].port_kind=" << static_cast<int>(binding->port_kind) << "\n";
          ofs << prefix << ".port_" << label << "_binding[" << binding_i
              << "].port_layer=" << static_cast<int>(binding->port_layer) << "\n";
        }
      }
    }
  }
  if (const auto selected_ref = SelectedEntityRefLocal(ui_state); selected_ref.has_value()) {
    ofs << "capture.selected_entity.kind=" << static_cast<int>(selected_ref->kind) << "\n";
    ofs << "capture.selected_entity.stable_id=" << selected_ref->stable_id << "\n";
    ofs << "capture.focus_entity.kind=" << static_cast<int>(selected_ref->kind) << "\n";
    ofs << "capture.focus_entity.stable_id=" << selected_ref->stable_id << "\n";
    if (const auto meta = view.describe_entity(*selected_ref); meta.has_value()) {
      ofs << "capture.selected_entity.display_name=" << meta->display_name << "\n";
      ofs << "capture.selected_entity.provenance=" << meta->provenance << "\n";
      ofs << "capture.selected_entity.role=" << static_cast<int>(meta->role) << "\n";
      ofs << "capture.focus_entity.display_name=" << meta->display_name << "\n";
      ofs << "capture.focus_entity.provenance=" << meta->provenance << "\n";
      ofs << "capture.focus_entity.role=" << static_cast<int>(meta->role) << "\n";
    }
    write_decision_trace("capture.selected_entity", *selected_ref);
    write_decision_trace("capture.focus_entity", *selected_ref);
  } else {
    ofs << "capture.focus_entity.kind=none\n";
    ofs << "capture.focus_entity.stable_id=0\n";
  }
  ofs << "capture.all_span_trace_count=" << view.edit_state().spans.size() << "\n";
  std::size_t all_span_trace_index = 0;
  for (const auto& span : view.edit_state().spans.items()) {
    const std::string prefix = std::format("capture.all_span[{}]", all_span_trace_index);
    ofs << prefix << ".span_id=" << static_cast<unsigned long long>(span.id) << "\n";
    ofs << prefix << ".bundle_id=" << static_cast<unsigned long long>(span.bundle_id) << "\n";
    ofs << prefix << ".endpoint_node_a_id=" << static_cast<unsigned long long>(span.endpoint_node_a_id) << "\n";
    ofs << prefix << ".endpoint_node_b_id=" << static_cast<unsigned long long>(span.endpoint_node_b_id) << "\n";
    write_decision_trace(prefix, wire::core::EntityRef{wire::core::EntityKind::kSpan, span.id});
    if (const auto span_view = view.inspect_span(span.id); span_view.has_value()) {
      write_style_inspection(prefix + ".style", span_view->style);
    }
    ++all_span_trace_index;
  }

  ofs << "draw.path_count=" << ui_state.draw_path_points.size() << "\n";
  ofs << "draw.path_support_kind_count=" << ui_state.draw_path_point_support_kinds.size() << "\n";
  ofs << "draw.path_node_id_count=" << ui_state.draw_path_point_node_ids.size() << "\n";
  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    const auto& p = ui_state.draw_path_points[i];
    ofs << "draw.path[" << i << "]=" << p.x << "," << p.y << "," << p.z << "\n";
    const wire::core::SupportKind support_kind =
        (i < ui_state.draw_path_point_support_kinds.size()) ? ui_state.draw_path_point_support_kinds[i]
                                                            : wire::core::SupportKind::kPole;
    ofs << "draw.path[" << i << "].support_kind=" << SupportKindLabelLocal(support_kind) << "\n";
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : wire::core::kInvalidObjectId;
    ofs << "draw.path[" << i << "].node_id=" << static_cast<unsigned long long>(node_id) << "\n";
  }
  const wire::core::BackboneSpec* replay_request =
      ui_state.last_draw_path_request.has_value() ? &*ui_state.last_draw_path_request : nullptr;
  ofs << "request.available=" << ((replay_request != nullptr) ? 1 : 0) << "\n";
  if (replay_request != nullptr) {
    ofs << "request.path_count=" << replay_request->path.polyline.size() << "\n";
    for (std::size_t i = 0; i < replay_request->path.polyline.size(); ++i) {
      const auto& p = replay_request->path.polyline[i];
      ofs << "request.path[" << i << "]=" << p.x << "," << p.y << "," << p.z << "\n";
    }
    ofs << "request.node_spec_count=" << replay_request->path.node_specs.size() << "\n";
    for (std::size_t i = 0; i < replay_request->path.node_specs.size(); ++i) {
      const auto& node = replay_request->path.node_specs[i];
      ofs << "request.node_spec[" << i << "].point_index=" << node.point_index << "\n";
      ofs << "request.node_spec[" << i << "].support_kind=" << SupportKindLabelLocal(node.support_kind) << "\n";
      ofs << "request.node_spec[" << i << "].node_id=" << static_cast<unsigned long long>(node.node_id) << "\n";
      ofs << "request.node_spec[" << i << "].has_tangent_hint=" << (node.has_tangent_hint ? 1 : 0) << "\n";
      if (node.has_tangent_hint) {
        ofs << "request.node_spec[" << i << "].tangent_hint=" << node.tangent_hint.x << "," << node.tangent_hint.y
            << "," << node.tangent_hint.z << "\n";
      }
    }
    ofs << "request.interval_m=" << replay_request->interval_m << "\n";
    ofs << "request.pole_type_id=" << static_cast<unsigned long long>(replay_request->pole_type_id) << "\n";
    ofs << "request.direction_mode="
        << PathDirectionModeLabelLocal(replay_request->direction_mode) << "\n";
    ofs << "request.bundle_count=" << replay_request->bundles.size() << "\n";
    for (std::size_t i = 0; i < replay_request->bundles.size(); ++i) {
      const auto& bundle = replay_request->bundles[i];
      ofs << "request.bundle[" << i << "].kind=" << static_cast<int>(bundle.bundle_template_id) << "\n";
      ofs << "request.bundle[" << i << "].layer=" << static_cast<int>(bundle.layer) << "\n";
      ofs << "request.bundle[" << i << "].count=" << bundle.count << "\n";
    }
  }
  ofs << "draw.interval_m=" << ui_state.draw_interval_m << "\n";
  ofs << "draw.clicked_points_only=" << (ui_state.draw_clicked_points_only ? 1 : 0) << "\n";
  ofs << "draw.direction_mode="
      << PathDirectionModeLabelLocal(
             static_cast<wire::core::PathDirectionMode>(std::clamp(ui_state.draw_direction_mode, 0, 2)))
      << "\n";
  ofs << "draw.keep_path_after_generate=" << (ui_state.draw_keep_path_after_generate ? 1 : 0) << "\n";
  ofs << "draw.plane_z=" << ui_state.draw_plane_z << "\n";

  const auto type_ids = SortedPoleTypeIds(view);
  if (!type_ids.empty()) {
    const std::size_t idx = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
    ofs << "draw.pole_type_id=" << static_cast<unsigned long long>(type_ids[idx]) << "\n";
  } else {
    ofs << "draw.pole_type_id=none\n";
  }

  ofs << "draw.bundle_template_count=" << selected_templates.size() << "\n";
  for (std::size_t i = 0; i < selected_templates.size(); ++i) {
    const wire::core::BundleKind kind = selected_templates[i];
    const auto it = view.bundle_templates().find(kind);
    ofs << "draw.bundle[" << i << "].kind=" << static_cast<int>(kind) << "\n";
    if (it != view.bundle_templates().end()) {
      const auto& tpl = it->second;
      ofs << "draw.bundle[" << i << "].name=" << tpl.name << "\n";
      ofs << "draw.bundle[" << i << "].category=" << CategoryLabelLocal(tpl.category) << "\n";
      ofs << "draw.bundle[" << i << "].default_layer=" << static_cast<int>(tpl.default_layer) << "\n";
      ofs << "draw.bundle[" << i << "].enable_branch_down_offset=" << (tpl.enable_branch_down_offset ? 1 : 0) << "\n";
      ofs << "draw.bundle[" << i << "].branch_endpoint_offset_m=" << tpl.branch_endpoint_offset_m << "\n";
      ofs << "draw.bundle[" << i << "].count_rule=" << static_cast<int>(tpl.count_rule) << "\n";
      if (tpl.count_rule == wire::core::BundleCountRuleKind::kFixed) {
        ofs << "draw.bundle[" << i << "].count=" << tpl.fixed_count << "\n";
      } else {
        const int key = static_cast<int>(kind);
        auto it_count = ui_state.draw_bundle_count_by_template.find(key);
        int count = (it_count == ui_state.draw_bundle_count_by_template.end()) ? tpl.default_count : it_count->second;
        count = std::clamp(count, tpl.min_count, tpl.max_count);
        ofs << "draw.bundle[" << i << "].count=" << count << "\n";
      }
    }
  }

  ofs << "result.direction.requested_mode=" << PathDirectionModeLabelLocal(dir_debug.requested_mode) << "\n";
  ofs << "result.direction.chosen=" << PathDirectionChosenLabelLocal(dir_debug.chosen) << "\n";
  ofs << "result.direction.cost.forward.total=" << dir_debug.forward_cost.total << "\n";
  ofs << "result.direction.cost.reverse.total=" << dir_debug.reverse_cost.total << "\n";
  ofs << "result.direction.cost.forward.cross=" << dir_debug.forward_cost.estimated_cross_penalty << "\n";
  ofs << "result.direction.cost.forward.side_flip=" << dir_debug.forward_cost.side_flip_penalty << "\n";
  ofs << "result.direction.cost.forward.layer_jump=" << dir_debug.forward_cost.layer_jump_penalty << "\n";
  ofs << "result.direction.cost.forward.corner=" << dir_debug.forward_cost.corner_compression_penalty << "\n";
  ofs << "result.direction.cost.forward.branch=" << dir_debug.forward_cost.branch_conflict_penalty << "\n";

  const wire::core::BackboneResult rebuilt_backbone = view.saved_backbone_result();
  const auto& edge_orientations = view.last_generation_edge_orientations();
  ofs << "result.backbone.snapshot_node_count=" << rebuilt_backbone.nodes.size() << "\n";
  ofs << "result.backbone.snapshot_edge_count=" << rebuilt_backbone.edges.size() << "\n";
  ofs << "result.backbone.rebuilt_node_count=" << rebuilt_backbone.nodes.size() << "\n";
  ofs << "result.backbone.rebuilt_edge_count=" << rebuilt_backbone.edges.size() << "\n";
  ofs << "result.backbone.node_count=" << rebuilt_backbone.nodes.size() << "\n";
  for (std::size_t i = 0; i < rebuilt_backbone.nodes.size(); ++i) {
    const auto& node = rebuilt_backbone.nodes[i];
    ofs << "result.backbone.node[" << i << "].id=" << static_cast<unsigned long long>(node.node_id) << "\n";
    ofs << "result.backbone.node[" << i << "].support_kind=" << SupportKindLabelLocal(node.support_kind) << "\n";
    ofs << "result.backbone.node[" << i << "].position=" << node.position.x << "," << node.position.y << ","
        << node.position.z << "\n";
    ofs << "result.backbone.node[" << i << "].pole_id=" << static_cast<unsigned long long>(node.pole_id) << "\n";
    ofs << "result.backbone.node[" << i << "].path_point_index=" << node.path_point_index << "\n";
    ofs << "result.backbone.node[" << i << "].has_tangent_hint=" << (node.has_tangent_hint ? 1 : 0) << "\n";
    if (node.has_tangent_hint) {
      ofs << "result.backbone.node[" << i << "].tangent_hint=" << node.tangent_hint.x << "," << node.tangent_hint.y
          << "," << node.tangent_hint.z << "\n";
    }
    ofs << "result.backbone.node[" << i << "].bundle_mode_count=" << node.bundle_modes.size() << "\n";
    for (std::size_t j = 0; j < node.bundle_modes.size(); ++j) {
      const auto& bundle_mode = node.bundle_modes[j];
      ofs << "result.backbone.node[" << i << "].bundle_mode[" << j
          << "].template_id=" << static_cast<int>(bundle_mode.bundle_template_id) << "\n";
      ofs << "result.backbone.node[" << i << "].bundle_mode[" << j
          << "].mode=" << static_cast<int>(bundle_mode.mode) << "\n";
    }
    if (node.pole_id != wire::core::kInvalidObjectId) {
      if (const auto* pole = view.edit_state().poles.find(node.pole_id); pole != nullptr) {
        ofs << "result.backbone.node[" << i << "].pole_yaw_deg=" << pole->world_transform.rotation_euler_deg.z
            << "\n";
        if (const auto pole_view = view.inspect_pole(node.pole_id); pole_view.has_value()) {
          ofs << "result.backbone.node[" << i << "].pole_layout_yaw_deg=" << pole_view->layout_yaw_deg << "\n";
          ofs << "result.backbone.node[" << i
              << "].support_axis_rule=" << static_cast<int>(pole_view->support_axis_rule) << "\n";
          ofs << "result.backbone.node[" << i << "].support_axis=" << pole_view->support_axis_dir.x << ","
              << pole_view->support_axis_dir.y << "," << pole_view->support_axis_dir.z << "\n";
        }
        ofs << "result.backbone.node[" << i
            << "].sharp_orientation_applied=" << (pole->context.sharp_orientation_applied ? 1 : 0) << "\n";
        ofs << "result.backbone.node[" << i << "].sharp_theta_deg=" << pole->context.sharp_theta_deg << "\n";
        ofs << "result.backbone.node[" << i << "].sharp_bisector_dir=" << pole->context.sharp_bisector_dir.x << ","
            << pole->context.sharp_bisector_dir.y << "," << pole->context.sharp_bisector_dir.z << "\n";
        ofs << "result.backbone.node[" << i << "].sharp_side_dir=" << pole->context.sharp_side_dir.x << ","
            << pole->context.sharp_side_dir.y << "," << pole->context.sharp_side_dir.z << "\n";
      }
    }
  }
  ofs << "result.backbone.edge_count=" << rebuilt_backbone.edges.size() << "\n";
  for (std::size_t i = 0; i < rebuilt_backbone.edges.size(); ++i) {
    const auto& edge = rebuilt_backbone.edges[i];
    ofs << "result.backbone.edge[" << i << "].node_a_id=" << static_cast<unsigned long long>(edge.node_a) << "\n";
    ofs << "result.backbone.edge[" << i << "].node_b_id=" << static_cast<unsigned long long>(edge.node_b) << "\n";
  }
  ofs << "result.backbone.edge_orientation_count=" << edge_orientations.size() << "\n";
  for (std::size_t i = 0; i < edge_orientations.size(); ++i) {
    const auto& edge_orientation = edge_orientations[i];
    ofs << "result.backbone.edge_orientation[" << i
        << "].node_a_id=" << static_cast<unsigned long long>(edge_orientation.node_a_id) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].node_b_id=" << static_cast<unsigned long long>(edge_orientation.node_b_id) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].bundle_template_id=" << static_cast<int>(edge_orientation.bundle_template_id) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].orientation=" << static_cast<int>(edge_orientation.orientation) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].lowering_kind=" << static_cast<int>(edge_orientation.lowering_kind) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].flipped_from_previous=" << (edge_orientation.flipped_from_previous ? 1 : 0) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].flip_reason=" << static_cast<int>(edge_orientation.flip_reason) << "\n";
    ofs << "result.backbone.edge_orientation[" << i << "].turn_angle_deg=" << edge_orientation.turn_angle_deg
        << "\n";
  }

  ofs << "result.current_span_count=" << view.edit_state().spans.size() << "\n";
  std::size_t current_span_index = 0;
  for (const auto& span : view.edit_state().spans.items()) {
    ofs << "result.current_span[" << current_span_index << "].span_id=" << static_cast<unsigned long long>(span.id)
      << "\n";
    ofs << "result.current_span[" << current_span_index << "].bundle_id="
      << static_cast<unsigned long long>(span.bundle_id) << "\n";
    ofs << "result.current_span[" << current_span_index << "].endpoint_node_a_id="
      << static_cast<unsigned long long>(span.endpoint_node_a_id) << "\n";
    ofs << "result.current_span[" << current_span_index << "].endpoint_node_b_id="
      << static_cast<unsigned long long>(span.endpoint_node_b_id) << "\n";
    ofs << "result.current_span[" << current_span_index << "].port_a_id="
      << static_cast<unsigned long long>(span.port_a_id) << "\n";
    ofs << "result.current_span[" << current_span_index << "].port_b_id="
      << static_cast<unsigned long long>(span.port_b_id) << "\n";
    if (const auto span_view = view.inspect_span(span.id); span_view.has_value()) {
      ofs << "result.current_span[" << current_span_index << "].flow_kind="
        << FlowKindLabelLocal(span_view->flow_kind) << "\n";
      ofs << "result.current_span[" << current_span_index << "].flow_rule="
        << FlowRuleLabelLocal(span_view->flow_rule) << "\n";
      ofs << "result.current_span[" << current_span_index << "].lowering_kind="
        << LoweringKindLabelLocal(span_view->lowering_kind) << "\n";
      ofs << "result.current_span[" << current_span_index << "].continuity_class="
        << ContinuityClassLabelLocal(span_view->continuity_class) << "\n";
      ofs << "result.current_span[" << current_span_index << "].default_lower_required="
        << (span_view->default_lower_required ? 1 : 0) << "\n";
      ofs << "result.current_span[" << current_span_index << "].same_level_feasible="
        << (span_view->same_level_feasible ? 1 : 0) << "\n";
      ofs << "result.current_span[" << current_span_index << "].same_level_reason="
        << SameLevelReasonLabelLocal(span_view->same_level_reason) << "\n";
      ofs << "result.current_span[" << current_span_index << "].order_decision_policy="
        << OrderDecisionPolicyLabelLocal(span_view->order_decision_policy) << "\n";
      ofs << "result.current_span[" << current_span_index << "].order_decision_choice_a="
        << OrderDecisionChoiceLabelLocal(span_view->order_decision_choice_a) << "\n";
      ofs << "result.current_span[" << current_span_index << "].order_decision_choice_b="
        << OrderDecisionChoiceLabelLocal(span_view->order_decision_choice_b) << "\n";
    }
    WriteNeutralSpanLayoutCaptureLocal(view, ofs, "result.current_span[" + std::to_string(current_span_index) + "]",
                                       span.id);
    write_endpoint_junction("result.current_span[" + std::to_string(current_span_index) + "].endpoint_a_junction",
                            span.endpoint_node_a_id, span.endpoint_node_b_id);
    write_endpoint_junction("result.current_span[" + std::to_string(current_span_index) + "].endpoint_b_junction",
                            span.endpoint_node_b_id, span.endpoint_node_a_id);
    write_decision_trace("result.current_span[" + std::to_string(current_span_index) + "]",
               wire::core::EntityRef{wire::core::EntityKind::kSpan, span.id});
    ++current_span_index;
    }

  std::map<std::pair<wire::core::ObjectId, int>, wire::core::LoweredSupportGroupKey> layout_lowered_support_keys{};
  for (const auto& span : view.edit_state().spans.items()) {
    const auto layout_view = view.span_layout(span.id);
    if (!layout_view.has_layout()) {
      continue;
    }
    for (const auto& key : layout_view.entry->lowered_support_group_keys) {
      layout_lowered_support_keys.emplace(std::make_pair(key.owner_pole_id, key.support_group_id), key);
    }
  }
  ofs << "result.layout_lowered_support_key_count=" << layout_lowered_support_keys.size() << "\n";
  std::size_t lowered_key_index = 0;
  for (const auto& [_, key] : layout_lowered_support_keys) {
    ofs << "result.layout_lowered_support_key[" << lowered_key_index
        << "].owner_pole_id=" << static_cast<unsigned long long>(key.owner_pole_id) << "\n";
    ofs << "result.layout_lowered_support_key[" << lowered_key_index
        << "].support_group_id=" << key.support_group_id << "\n";
    ++lowered_key_index;
  }

  ofs << "result.current_pole_count=" << view.edit_state().poles.size() << "\n";
  std::size_t current_pole_index = 0;
  for (const auto& pole : view.edit_state().poles.items()) {
    const std::string prefix = std::format("result.current_pole[{}]", current_pole_index);
    ofs << prefix << ".pole_id=" << static_cast<unsigned long long>(pole.id) << "\n";
    ofs << prefix << ".display_id=" << pole.display_id << "\n";
    ofs << prefix << ".name=" << pole.name << "\n";
    ofs << prefix << ".pole_type_id=" << static_cast<unsigned long long>(pole.pole_type_id) << "\n";
    ofs << prefix << ".position=" << pole.world_transform.position.x << "," << pole.world_transform.position.y << ","
        << pole.world_transform.position.z << "\n";
    ofs << prefix << ".rotation_euler_deg=" << pole.world_transform.rotation_euler_deg.x << ","
        << pole.world_transform.rotation_euler_deg.y << "," << pole.world_transform.rotation_euler_deg.z << "\n";
    ofs << prefix << ".height_m=" << pole.height_m << "\n";
    ofs << prefix << ".tilt_magnitude_deg=" << pole.tilt_magnitude_deg << "\n";
    ofs << prefix << ".placement_mode=" << EnumOrdinal(pole.placement_mode) << "\n";
    ofs << prefix << ".context_kind=" << EnumOrdinal(pole.context.kind) << "\n";
    ofs << prefix << ".sharp_orientation_applied=" << (pole.context.sharp_orientation_applied ? 1 : 0) << "\n";
    ofs << prefix << ".sharp_theta_deg=" << pole.context.sharp_theta_deg << "\n";
    ofs << prefix << ".sharp_bisector_dir=" << pole.context.sharp_bisector_dir.x << ","
        << pole.context.sharp_bisector_dir.y << "," << pole.context.sharp_bisector_dir.z << "\n";
    ofs << prefix << ".sharp_side_dir=" << pole.context.sharp_side_dir.x << "," << pole.context.sharp_side_dir.y
        << "," << pole.context.sharp_side_dir.z << "\n";
    if (const auto pole_view = view.inspect_pole(pole.id); pole_view.has_value()) {
      ofs << prefix << ".layout_yaw_deg=" << pole_view->layout_yaw_deg << "\n";
      ofs << prefix << ".support_axis_rule=" << static_cast<int>(pole_view->support_axis_rule) << "\n";
      ofs << prefix << ".support_axis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
          << "," << pole_view->support_axis_dir.z << "\n";
    }
    write_decision_trace(prefix, wire::core::EntityRef{wire::core::EntityKind::kPole, pole.id});
    ++current_pole_index;
  }

  ofs << "result.current_port_count=" << view.edit_state().ports.size() << "\n";
  std::size_t current_port_index = 0;
  for (const auto& port : view.edit_state().ports.items()) {
    const std::string prefix = "result.current_port[" + std::to_string(current_port_index) + "]";
    ofs << prefix << ".port_id=" << static_cast<unsigned long long>(port.id) << "\n";
    ofs << prefix << ".display_id=" << port.display_id << "\n";
    ofs << prefix << ".owner_pole_id=" << static_cast<unsigned long long>(port.owner_pole_id) << "\n";
    ofs << prefix << ".world_position=" << port.world_position.x << "," << port.world_position.y << ","
        << port.world_position.z << "\n";
    ofs << prefix << ".kind=" << static_cast<int>(port.kind) << "\n";
    ofs << prefix << ".layer=" << static_cast<int>(port.layer) << "\n";
    ofs << prefix << ".category=" << static_cast<int>(port.category) << "\n";
    ofs << prefix << ".template_layer=" << port.template_layer << "\n";
    ofs << prefix << ".template_side=" << static_cast<int>(port.template_side) << "\n";
    ofs << prefix << ".placement_context=" << static_cast<int>(port.placement_context) << "\n";
    ofs << prefix << ".position_mode=" << static_cast<int>(port.position_mode) << "\n";
    ofs << prefix << ".placement_source=" << static_cast<int>(port.placement_source) << "\n";
    ++current_port_index;
  }

  ofs << "result.current_bundle_count=" << view.edit_state().bundles.size() << "\n";
  std::size_t current_bundle_index = 0;
  for (const auto& bundle : view.edit_state().bundles.items()) {
    const std::string prefix = "result.current_bundle[" + std::to_string(current_bundle_index) + "]";
    ofs << prefix << ".bundle_id=" << static_cast<unsigned long long>(bundle.id) << "\n";
    ofs << prefix << ".display_id=" << bundle.display_id << "\n";
    ofs << prefix << ".bundle_template_id=" << static_cast<int>(bundle.bundle_template_id) << "\n";
    ofs << prefix << ".conductor_count=" << bundle.conductor_count << "\n";
    ofs << prefix << ".phase_spacing_m=" << bundle.phase_spacing_m << "\n";
    ++current_bundle_index;
  }

  ofs << "state.poles=" << view.edit_state().poles.size() << "\n";
  ofs << "state.ports=" << view.edit_state().ports.size() << "\n";
  ofs << "state.bundles=" << view.edit_state().bundles.size() << "\n";
  ofs << "state.spans=" << view.edit_state().spans.size() << "\n";

  ofs.flush();
  if (!ofs.good()) {
    if (out_error != nullptr) {
      *out_error = "failed to write capture file";
    }
    return false;
  }

  if (out_path != nullptr) {
    *out_path = capture_path.string();
  }
  return true;
}
void UpdateDrawPathInput(CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  const auto view = viewer_core_state::View(state);
  ui_state.draw_hover_pick = {};
  ui_state.draw_hover_has_resolution = false;
  ui_state.draw_hover_status.clear();
  if (ui_state.mode != EditMode::kDrawPath) {
    ui_state.draw_hover_valid = false;
    return;
  }
  EnsureDrawPathPointKinds(ui_state);

  ImGuiIO& io = ImGui::GetIO();
  wire::core::Vec3d hover{};
  const bool has_ground_hit = TryPickGroundPoint(camera, ui_state.draw_plane_z, &hover);
  ui_state.draw_hover_valid = has_ground_hit;
  if (has_ground_hit) {
    ui_state.draw_hover_point = hover;
  }
  const bool accept_mouse_input =
      !io.WantCaptureMouse && ui_state.camera_drag_mode == CameraDragMode::kNone && !ui_state.camera_walk_mode;

  if (ui_state.draw_pick_enabled && accept_mouse_input) {
    ViewerSceneQuery scene_query{};
    const wire::core::PickResult raw_pick = scene_query.Raycast(view, camera, ui_state.draw_plane_z);
    const double hover_snap_radius_world = std::max(ui_state.draw_snap_radius_world, 1.25);
    wire::core::PickResult pick =
        CanonicalizeDrawPathPick(view, raw_pick, hover, has_ground_hit, hover_snap_radius_world);
    ui_state.draw_hover_pick = pick;
    bool blocked_pick_target = false;
    if (pick.hit_kind == wire::core::PickHitKind::kEmpty) {
      if (ui_state.draw_hover_status.empty()) {
        ui_state.draw_hover_status = "target: Empty";
      }
    } else {
      if (pick.hit_kind == wire::core::PickHitKind::kNode && raw_pick.hit_kind != wire::core::PickHitKind::kNode) {
        ui_state.draw_hover_status = "target: " + std::string(PickHitKindLabelLocal(raw_pick.hit_kind)) +
                                     " -> snapped: Node " +
                                     std::to_string(static_cast<unsigned long long>(pick.hit_id));
      }
      const std::vector<wire::core::BundleKind> pick_template_ids =
          ResolveTemplateKindsForPathPick(view, ui_state.draw_bundle_template_mask, pick);
      if (ui_state.draw_hover_status.empty()) {
        ui_state.draw_hover_status =
            std::string("target: ") + PickHitKindLabelLocal(pick.hit_kind) + " " + PickTargetLabel(pick);
      }
      if (pick.hit_kind == wire::core::PickHitKind::kNode || pick.hit_kind == wire::core::PickHitKind::kSegment ||
          pick.hit_kind == wire::core::PickHitKind::kExternal) {
        wire::core::EditResult<wire::core::ResolveBranchPickResult> resolved{};
        if (pick.hit_kind == wire::core::PickHitKind::kNode || pick.hit_kind == wire::core::PickHitKind::kExternal) {
          resolved.ok = true;
          resolved.value = DirectResolvedDrawPathTarget(pick);
        } else {
          wire::core::ResolveBranchPickOptions options{};
          options.selected_bundle_template_ids = pick_template_ids;
          options.snap_radius_world = ui_state.draw_snap_radius_world;
          options.create_midair_node = false;
          options.enforce_midair_template_policy = true;
          resolved = viewer_core_state::ResolveBranchPick(state, pick, options);
        }
        if (resolved.ok) {
          const std::string blocked_template = FindMidairBranchBlockedTemplateName(view, pick_template_ids);
          if (resolved.value.resolution == wire::core::PickBranchResolutionKind::kMidair &&
              !blocked_template.empty()) {
            ui_state.draw_hover_status += " -> warn: template " + blocked_template + " will not connect here";
          }
          ui_state.draw_hover_has_resolution = true;
          ui_state.draw_hover_resolution = resolved.value;
          ui_state.draw_hover_point = resolved.value.position;
          ui_state.draw_hover_valid = true;
          ui_state.draw_hover_status +=
              (resolved.value.resolution == wire::core::PickBranchResolutionKind::kNode)
                  ? " | resolved: Node"
                  : " | resolved: Midair";
          if (resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
            ui_state.draw_hover_status +=
                " " + std::to_string(static_cast<unsigned long long>(resolved.value.resolved_node_id));
          }
        } else {
          ui_state.draw_hover_status += " | blocked: " + resolved.error;
          blocked_pick_target = true;
        }
      }
    }
    if (blocked_pick_target) {
      ui_state.draw_hover_valid = false;
    }
  }

  if (accept_mouse_input) {
    if (ui_state.draw_hover_valid && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      if (ui_state.draw_hover_has_resolution) {
        wire::core::EditResult<wire::core::ResolveBranchPickResult> applied{};
        if (ui_state.draw_hover_pick.hit_kind == wire::core::PickHitKind::kNode ||
            ui_state.draw_hover_pick.hit_kind == wire::core::PickHitKind::kExternal ||
            ui_state.draw_hover_resolution.resolution == wire::core::PickBranchResolutionKind::kNode) {
          applied.ok = true;
          applied.value = ui_state.draw_hover_resolution;
        } else {
          const std::vector<wire::core::BundleKind> click_template_ids =
              ResolveTemplateKindsForPathPick(view, ui_state.draw_bundle_template_mask, ui_state.draw_hover_pick);
          wire::core::ResolveBranchPickOptions click_options{};
          click_options.selected_bundle_template_ids = click_template_ids;
          click_options.snap_radius_world = ui_state.draw_snap_radius_world;
          click_options.create_midair_node = true;
          click_options.enforce_midair_template_policy = true;
          applied = viewer_core_state::ResolveBranchPick(state, ui_state.draw_hover_pick, click_options);
        }
        if (!applied.ok) {
          ui_state.last_error = applied.error;
          PushLogLocal(ui_state, "DrawPath ResolveBranchPick(click) failed: " + applied.error);
        } else {
          ui_state.last_error.clear();
          DrawPathPushPoint(ui_state, applied.value.position, applied.value.support_kind, applied.value.resolved_node_id);
        }
      } else {
        DrawPathPushPoint(ui_state, ui_state.draw_hover_point, wire::core::SupportKind::kPole,
                          wire::core::kInvalidObjectId);
      }
    }
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
      DrawPathPopPoint(ui_state);
    }
  }

  if (!io.WantCaptureKeyboard && !ui_state.camera_walk_mode && !ui_state.camera_consumed_escape) {
    if (IsKeyPressed(KEY_BACKSPACE)) {
      DrawPathPopPoint(ui_state);
    }
    if (IsKeyPressed(KEY_ESCAPE)) {
      DrawPathClearPoints(ui_state);
    }
    if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
      ExecuteGenerateFromDrawPath(state, ui_state, true);
    }
  }
}

void DrawPathPreview(const ViewerUiState& ui_state) {
  if (!ui_state.draw_show_preview || ui_state.mode != EditMode::kDrawPath) {
    return;
  }
  if (ui_state.draw_path_points.empty() && !ui_state.draw_hover_valid) {
    return;
  }

  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    const wire::core::SupportKind support_kind =
        (i < ui_state.draw_path_point_support_kinds.size()) ? ui_state.draw_path_point_support_kinds[i]
                                                            : wire::core::SupportKind::kPole;
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : wire::core::kInvalidObjectId;
    Color point_color = Color{196, 156, 68, 255};
    if (support_kind == wire::core::SupportKind::kMidair) {
      point_color = Color{90, 154, 176, 255};
    } else if (support_kind == wire::core::SupportKind::kExternal) {
      point_color = Color{110, 154, 100, 255};
    }
    DrawSphere(ToRaylibLocal(ui_state.draw_path_points[i]), 0.12f, point_color);
    if (node_id != wire::core::kInvalidObjectId) {
      DrawSphereWires(ToRaylibLocal(ui_state.draw_path_points[i]), 0.18f, 10, 16, Color{128, 126, 120, 200});
    }
    if (i + 1 < ui_state.draw_path_points.size()) {
      DrawLine3D(ToRaylibLocal(ui_state.draw_path_points[i]), ToRaylibLocal(ui_state.draw_path_points[i + 1]),
                 Color{152, 126, 72, 255});
    }
  }
  if (ui_state.draw_hover_valid) {
    DrawSphere(ToRaylibLocal(ui_state.draw_hover_point), 0.08f, Color{96, 152, 132, 200});
    if (!ui_state.draw_path_points.empty()) {
      DrawLine3D(ToRaylibLocal(ui_state.draw_path_points.back()), ToRaylibLocal(ui_state.draw_hover_point),
                 Color{96, 152, 132, 180});
    }
  }
}
void DrawPathModePanel(CoreState& state, ViewerUiState& ui_state) {
  const auto view = viewer_core_state::View(state);
  EnsureDrawPathPointKinds(ui_state);
  ImGui::TextUnformatted("Draw Path");
  ImGui::TextUnformatted("LMB: add point on Backbone node/edge (with draw-plane fallback)");
  ImGui::TextUnformatted("RMB/Backspace: undo last");
  ImGui::TextUnformatted("Esc: clear path, Enter: generate");
  ImGui::Checkbox("Enable DrawPath Pick (LMB)", &ui_state.draw_pick_enabled);
  ImGui::Checkbox("Show Backbone Overlay", &ui_state.draw_show_backbone_overlay);
  ImGui::InputDouble("Draw Snap Radius (world)", &ui_state.draw_snap_radius_world, 0.05, 0.2, "%.2f");
  ui_state.draw_snap_radius_world = std::clamp(ui_state.draw_snap_radius_world, 0.05, 5.0);
  ImGui::InputDouble("Draw Plane Z", &ui_state.draw_plane_z, 0.1, 1.0, "%.2f");
  ImGui::InputDouble("Path Interval (m)", &ui_state.draw_interval_m, 0.5, 1.0, "%.2f");
  ui_state.draw_interval_m = std::max(0.0, ui_state.draw_interval_m);
  ImGui::Checkbox("Clicked Points Only (No Intermediate Pole)", &ui_state.draw_clicked_points_only);
  ImGui::Checkbox("Show Preview", &ui_state.draw_show_preview);
  ImGui::Checkbox("Keep Path After Generate", &ui_state.draw_keep_path_after_generate);
  ImGui::TextUnformatted("Template-driven bundle generation");
  ImGui::Text("Path points: %d", static_cast<int>(ui_state.draw_path_points.size()));
  const int midair_points = static_cast<int>(
      std::count(ui_state.draw_path_point_support_kinds.begin(), ui_state.draw_path_point_support_kinds.end(),
                 wire::core::SupportKind::kMidair));
  const int building_points = static_cast<int>(
      std::count(ui_state.draw_path_point_support_kinds.begin(), ui_state.draw_path_point_support_kinds.end(),
                 wire::core::SupportKind::kExternal));
  const int anchored_points = static_cast<int>(
      std::count_if(ui_state.draw_path_point_node_ids.begin(), ui_state.draw_path_point_node_ids.end(),
                    [](ObjectId node_id) { return node_id != wire::core::kInvalidObjectId; }));
  ImGui::Text("Support kind points: Midair=%d External=%d", midair_points, building_points);
  ImGui::Text("Anchored Backbone points: %d", anchored_points);
  if (ui_state.draw_hover_valid) {
    ImGui::Text("Hover: %.2f %.2f %.2f", ui_state.draw_hover_point.x, ui_state.draw_hover_point.y,
                ui_state.draw_hover_point.z);
  } else {
    ImGui::TextUnformatted("Hover: (no ground hit)");
  }
  if (!ui_state.draw_hover_status.empty()) {
    ImGui::TextWrapped("%s", ui_state.draw_hover_status.c_str());
  }
  if (ui_state.draw_hover_has_resolution) {
    ImGui::Text("Hover resolved: %s kind=%s",
                (ui_state.draw_hover_resolution.resolution == wire::core::PickBranchResolutionKind::kNode)
                    ? "Node"
                    : "Midair",
                SupportKindLabelLocal(ui_state.draw_hover_resolution.support_kind));
  }

  const auto type_ids = SortedPoleTypeIds(view);
  if (!type_ids.empty()) {
    const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
    ui_state.road_pole_type_index = static_cast<int>(road_type_index);
    const wire::core::PoleTypeId road_type_id = type_ids[road_type_index];
    const std::string road_type_name = PoleTypePreviewLocal(view, road_type_id);
    if (ImGui::BeginCombo("Path PoleType", road_type_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const std::string label = PoleTypePreviewLocal(view, type_ids[i]);
        const bool selected = (i == road_type_index);
        if (ImGui::Selectable(label.c_str(), selected)) {
          ui_state.road_pole_type_index = static_cast<int>(i);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
  }

  const auto template_ids = SortedBundleTemplateKinds(view);
  if (template_ids.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "No bundle template registered in core");
  } else {
    if (ImGui::BeginCombo("Bundle Templates", BundleTemplateMultiPreview(view, ui_state).c_str())) {
      for (const wire::core::BundleKind kind : template_ids) {
        bool selected = IsBundleTemplateSelected(ui_state, kind);
        if (ImGui::Selectable(BundleTemplatePreviewOrIdLocal(view, kind).c_str(), selected,
                              ImGuiSelectableFlags_DontClosePopups)) {
          selected = !selected;
          SetBundleTemplateSelected(ui_state, kind, selected);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    const auto selected_templates = SelectedBundleTemplates(view, ui_state);
    if (selected_templates.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Select at least one bundle template");
    }
    for (wire::core::BundleKind kind : selected_templates) {
      const auto it = view.bundle_templates().find(kind);
      if (it == view.bundle_templates().end()) {
        continue;
      }
      const wire::core::BundleTemplate& bundle_template = it->second;
      ImGui::Separator();
      ImGui::TextUnformatted(BundleTemplatePreviewOrIdLocal(view, kind).c_str());
      ImGui::Text("Category: %s", CategoryLabelLocal(bundle_template.category));
      if (bundle_template.count_rule == wire::core::BundleCountRuleKind::kRange) {
        const int key = static_cast<int>(kind);
        int count = ResolveBundleTemplateCountLocal(ui_state, bundle_template, kind);
        ImGui::PushID(key);
        ImGui::InputInt("Count", &count);
        ImGui::PopID();
        count = std::clamp(count, bundle_template.min_count, bundle_template.max_count);
        ui_state.draw_bundle_count_by_template[key] = count;
        ImGui::Text("Allowed Range: %d..%d (default=%d)", bundle_template.min_count, bundle_template.max_count,
                    bundle_template.default_count);
      } else {
        ImGui::Text("Fixed Count: %d (template rule)", bundle_template.fixed_count);
      }
      ImGui::Text("Template Layer: %d", static_cast<int>(bundle_template.default_layer));
      ImGui::Text("Allow Mirror: %s", bundle_template.allow_mirror ? "true" : "false");
    }
  }
  if (ImGui::BeginCombo("Direction Mode",
                        PathDirectionModeLabelLocal(
                            static_cast<wire::core::PathDirectionMode>(ui_state.draw_direction_mode)))) {
    for (int i = 0; i < 3; ++i) {
      const bool selected = (i == ui_state.draw_direction_mode);
      const auto mode = static_cast<wire::core::PathDirectionMode>(i);
      if (ImGui::Selectable(PathDirectionModeLabelLocal(mode), selected)) {
        ui_state.draw_direction_mode = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  if (!ui_state.draw_clicked_points_only && ui_state.draw_interval_m <= 0.0) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Path Interval must be > 0");
  }
  if (ImGui::Button("Flip Direction (Manual)")) {
    if (ui_state.draw_direction_mode == static_cast<int>(wire::core::PathDirectionMode::kReverse)) {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kForward);
    } else {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kReverse);
    }
  }
  const auto& dir_debug = view.last_path_direction_debug();
  ImGui::Text("Direction chosen: %s (mode=%s)", PathDirectionChosenLabelLocal(dir_debug.chosen),
              PathDirectionModeLabelLocal(dir_debug.requested_mode));
  ImGui::Text("Cost F/R: %d / %d", dir_debug.forward_cost.total, dir_debug.reverse_cost.total);
  ImGui::Text("Cost detail cross=%d side=%d layer=%d corner=%d branch=%d",
              dir_debug.forward_cost.estimated_cross_penalty, dir_debug.forward_cost.side_flip_penalty,
              dir_debug.forward_cost.layer_jump_penalty, dir_debug.forward_cost.corner_compression_penalty,
              dir_debug.forward_cost.branch_conflict_penalty);

  if (ImGui::Button("Generate From Path")) {
    ExecuteGenerateFromDrawPath(state, ui_state, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("Save Repro Capture")) {
    std::string path{};
    std::string error{};
    if (SaveDrawPathReproCapture(state, ui_state, &path, &error)) {
      ui_state.last_repro_capture_path = path;
      PushLogLocal(ui_state, "Saved repro capture: " + path);
    } else {
      ui_state.last_error = error;
      PushLogLocal(ui_state, "Save repro capture failed");
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Undo Last Point")) {
    DrawPathPopPoint(ui_state);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Path")) {
    DrawPathClearPoints(ui_state);
  }
  if (!ui_state.last_repro_capture_path.empty()) {
    ImGui::TextWrapped("Last capture: %s", ui_state.last_repro_capture_path.c_str());
  }
}
