#include "app_state.hpp"
#include "path_pick_policy.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <utility>

#include "imgui.h"
#include "scene_query.hpp"

namespace {

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

double DefaultBranchDownOffsetForCategoryLocal(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return 0.275;
  case wire::core::ConnectionCategory::kCommunication:
    return 0.30;
  case wire::core::ConnectionCategory::kOptical:
    return 0.24;
  case wire::core::ConnectionCategory::kLowVoltage:
  case wire::core::ConnectionCategory::kDrop:
  default:
    return 0.35;
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

const char* SupportKindLabelLocal(wire::core::SupportKind kind) {
  switch (kind) {
  case wire::core::SupportKind::kPole:
    return "Pole";
  case wire::core::SupportKind::kMidair:
    return "Midair";
  case wire::core::SupportKind::kBuilding:
    return "Building";
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
  case wire::core::PickHitKind::kBuilding:
    return "Building";
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
  case wire::core::BackboneFlowDecisionRule::kExistingChainMain:
    return "ExistingChainMain";
  case wire::core::BackboneFlowDecisionRule::kExistingChainBranch:
    return "ExistingChainBranch";
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

const char* BundleOrderPolicyLabelLocal(wire::core::BundleOrderPolicyKind policy) {
  switch (policy) {
  case wire::core::BundleOrderPolicyKind::kFixedOrder:
    return "FixedOrder";
  case wire::core::BundleOrderPolicyKind::kPermutableHomogeneous:
    return "PermutableHomogeneous";
  default:
    return "Unknown";
  }
}

const char* BundleOrderChoiceLabelLocal(wire::core::BundleOrderChoiceKind choice) {
  switch (choice) {
  case wire::core::BundleOrderChoiceKind::kNormal:
    return "Normal";
  case wire::core::BundleOrderChoiceKind::kReversed:
    return "Reversed";
  default:
    return "Unknown";
  }
}

const char* BundleOrderChoiceReasonLabelLocal(wire::core::BundleOrderChoiceReason reason) {
  switch (reason) {
  case wire::core::BundleOrderChoiceReason::kFixedOrder:
    return "FixedOrder";
  case wire::core::BundleOrderChoiceReason::kCrossingFewer:
    return "CrossingFewer";
  case wire::core::BundleOrderChoiceReason::kSpacingBetter:
    return "SpacingBetter";
  case wire::core::BundleOrderChoiceReason::kTwistSmaller:
    return "TwistSmaller";
  case wire::core::BundleOrderChoiceReason::kKeptDefault:
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
  case wire::core::DecisionTraceTopic::kSupportLayoutSelection:
    return "SupportLayoutSelection";
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

std::vector<wire::core::BundleKind> SortedBundleTemplateKindsLocal(const CoreState& state) {
  std::vector<wire::core::BundleKind> ids;
  for (const auto& [kind, _] : state.view().bundle_templates()) {
    ids.push_back(kind);
  }
  std::sort(ids.begin(), ids.end(), [](auto a, auto b) { return static_cast<int>(a) < static_cast<int>(b); });
  return ids;
}

bool IsBundleTemplateSelectedLocal(const ViewerUiState& ui_state, wire::core::BundleKind kind) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  return (ui_state.draw_bundle_template_mask & bit) != 0;
}

void SetBundleTemplateSelectedLocal(ViewerUiState& ui_state, wire::core::BundleKind kind, bool selected) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  if (selected) {
    ui_state.draw_bundle_template_mask |= bit;
  } else {
    ui_state.draw_bundle_template_mask &= ~bit;
  }
}

std::vector<wire::core::BundleKind> SelectedBundleTemplatesLocal(const CoreState& state, const ViewerUiState& ui_state) {
  std::vector<wire::core::BundleKind> out;
  for (wire::core::BundleKind kind : SortedBundleTemplateKindsLocal(state)) {
    if (IsBundleTemplateSelectedLocal(ui_state, kind)) {
      out.push_back(kind);
    }
  }
  return out;
}

std::string BundleTemplatePreviewLocal(const CoreState& state, wire::core::BundleKind kind) {
  const auto it = state.view().bundle_templates().find(kind);
  if (it == state.view().bundle_templates().end()) {
    return std::to_string(static_cast<int>(kind));
  }
  return it->second.name;
}

std::string BundleTemplateMultiPreviewLocal(const CoreState& state, const ViewerUiState& ui_state) {
  const auto selected = SelectedBundleTemplatesLocal(state, ui_state);
  if (selected.empty()) {
    return "None";
  }
  if (selected.size() == 1) {
    return BundleTemplatePreviewLocal(state, selected.front());
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << BundleTemplatePreviewLocal(state, selected[i]);
  }
  return oss.str();
}

std::string GeneratedEndpointSourceSummaryLocal(const CoreState& state, const std::vector<ObjectId>& span_ids) {
  int plain = 0;
  int socket = 0;
  int socket_override = 0;
  int fallback = 0;
  int attachment_inputs = 0;
  for (ObjectId span_id : span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto accumulate = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
      if (endpoint.attachment_request.kind != wire::core::EndpointAttachmentRequestKind::kNone) {
        ++attachment_inputs;
      }
      if (endpoint.endpoint_source == wire::core::SupportLayoutEndpointSourceKind::kPlainSupport) {
        ++plain;
      } else if (endpoint.endpoint_source == wire::core::SupportLayoutEndpointSourceKind::kAttachmentSocket) {
        ++socket;
      } else if (endpoint.endpoint_source == wire::core::SupportLayoutEndpointSourceKind::kAttachmentSocketOverride) {
        ++socket_override;
      } else {
        ++fallback;
      }
    };
    accumulate(layout_view->start_endpoint);
    accumulate(layout_view->end_endpoint);
  }
  std::ostringstream oss;
  oss << "endpointSources plain=" << plain << " socket=" << socket << " override=" << socket_override
      << " fallback=" << fallback << " attachmentInput=" << attachment_inputs;
  return oss.str();
}

int ResolveBundleTemplateCountLocal(ViewerUiState& ui_state, const wire::core::BundleTemplate& bundle_template,
                                    wire::core::BundleKind kind) {
  const int key = static_cast<int>(kind);
  auto it = ui_state.draw_bundle_count_by_template.find(key);
  int count = (it == ui_state.draw_bundle_count_by_template.end()) ? bundle_template.default_count : it->second;
  count = std::clamp(count, bundle_template.min_count, bundle_template.max_count);
  ui_state.draw_bundle_count_by_template[key] = count;
  return count;
}

std::vector<wire::core::PoleTypeId> SortedPoleTypeIdsLocal(const CoreState& state) {
  std::vector<wire::core::PoleTypeId> ids;
  for (const auto& [id, _] : state.view().pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
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
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const wire::core::Vec3d d = points[i + 1] - points[i];
    length += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  }
  return length;
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

wire::core::CoreState::ResolveBranchPickResult DirectResolvedDrawPathTarget(const wire::core::PickResult& pick) {
  wire::core::CoreState::ResolveBranchPickResult resolved{};
  resolved.resolution = wire::core::CoreState::PickBranchResolutionKind::kNode;
  resolved.resolved_node_id = pick.hit_id;
  resolved.position = pick.hit_pos_world;
  resolved.support_kind = (pick.hit_kind == wire::core::PickHitKind::kBuilding) ? wire::core::SupportKind::kBuilding
                                                                                 : wire::core::SupportKind::kPole;
  resolved.snapped_from_segment_endpoint = false;
  return resolved;
}

bool ExecuteBackboneRequest(CoreState& state, ViewerUiState& ui_state, const wire::core::BackboneSpec& request,
                            bool clear_draw_path_on_success, const char* success_log,
                            const char* failure_log) {
  ui_state.last_draw_path_request = request;
  const auto result = state.GenerateFromBackboneSpec(request);
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
  PushLogLocal(ui_state, GeneratedEndpointSourceSummaryLocal(state, result.value.generated_span_ids));
  return true;
}

void ExecuteGenerateFromDrawPath(CoreState& state, ViewerUiState& ui_state, bool from_enter_key) {
  EnsureDrawPathPointKinds(ui_state);
  if (ui_state.draw_path_points.size() < 2) {
    ui_state.last_error = "path needs at least 2 points";
    return;
  }

  const auto type_ids = SortedPoleTypeIdsLocal(state);
  if (type_ids.empty()) {
    ui_state.last_error = "no pole type available";
    return;
  }

  const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
  ui_state.road_pole_type_index = static_cast<int>(road_type_index);

  wire::core::RoadSegment road{};
  road.id = ui_state.road_id++;
  road.polyline = ui_state.draw_path_points;

  const auto selected_templates = SelectedBundleTemplatesLocal(state, ui_state);
  if (selected_templates.empty()) {
    ui_state.last_error = "select at least one bundle template";
    return;
  }

  const int mode_index = std::clamp(ui_state.draw_direction_mode, 0, 2);
  ui_state.draw_direction_mode = mode_index;

  wire::core::BackboneSpec request{};
  request.path.polyline = road.polyline;
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
    request.interval_m = std::max(0.001, PolylineLengthLocal(road.polyline) + 1.0);
  } else {
    request.interval_m = ui_state.draw_interval_m;
  }
  request.pole_type_id = type_ids[ui_state.road_pole_type_index];
  request.direction_mode = static_cast<wire::core::PathDirectionMode>(mode_index);
  for (wire::core::BundleKind kind : selected_templates) {
    const auto it = state.view().bundle_templates().find(kind);
    if (it == state.view().bundle_templates().end()) {
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
      BundleTemplateMultiPreviewLocal(state, ui_state);
  const char* failure_log = from_enter_key ? "Generate path (Enter) failed" : "Generate path failed";
  (void)ExecuteBackboneRequest(state, ui_state, request, !ui_state.draw_keep_path_after_generate, success_log.c_str(),
                               failure_log);
}

bool SaveDrawPathReproCapture(const CoreState& state, const ViewerUiState& ui_state, std::string* out_path,
                              std::string* out_error) {
  namespace fs = std::filesystem;

  std::time_t now = std::time(nullptr);
  std::tm tm_now{};
#if defined(_WIN32)
  localtime_s(&tm_now, &now);
#else
  localtime_r(&now, &tm_now);
#endif
  char stamp[32]{};
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);

  std::error_code ec;
  const fs::path capture_dir = fs::path("captures");
  fs::create_directories(capture_dir, ec);
  if (ec) {
    if (out_error != nullptr) {
      *out_error = "failed to create captures directory";
    }
    return false;
  }
  const fs::path capture_path = capture_dir / fs::path(std::string("drawpath_repro_") + stamp + ".txt");
  std::ofstream ofs(capture_path.string(), std::ios::trunc);
  if (!ofs.is_open()) {
    if (out_error != nullptr) {
      *out_error = "failed to open capture file for writing";
    }
    return false;
  }

  const auto selected_templates = SelectedBundleTemplatesLocal(state, ui_state);
  const auto& view = state.view();
  const auto& dir_debug = view.last_path_direction_debug();
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
      const auto write_endpoint_junction = [&](const std::string& prefix, wire::core::ObjectId node_id,
                           wire::core::ObjectId peer_node_id) {
      ofs << prefix << ".node_id=" << static_cast<unsigned long long>(node_id) << "\n";
      if (const auto junction_view = view.inspect_junction(node_id); junction_view.has_value()) {
        ofs << prefix << ".has_local_relation=" << (junction_view->has_local_relation ? 1 : 0) << "\n";
        ofs << prefix << ".through_pair_accepted=" << (junction_view->through_pair_accepted ? 1 : 0) << "\n";
        ofs << prefix << ".through_pair_used_semantic_tiebreak="
          << (junction_view->through_pair_used_semantic_tiebreak ? 1 : 0) << "\n";
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
        ofs << prefix << ".peer_used_semantic_tiebreak=" << (relation_it->used_semantic_tiebreak ? 1 : 0)
          << "\n";
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
      ofs << "capture.version=6\n";
  ofs << "capture.scope=all_state_plus_focus\n";
  ofs << "capture.includes_all_current_spans=1\n";
  ofs << "capture.includes_all_lane_assignments=1\n";
  ofs << "draw.endpoint_attachment_input_supported=0\n";
  ofs << "draw.endpoint_socket_input_supported=0\n";
  ofs << "capture.timestamp_unix=" << static_cast<long long>(now) << "\n";
  ofs << "capture.mode=" << ModeLabelLocal(ui_state.mode) << "\n";
  ofs << "capture.selected_type=" << SelectedTypeLabelLocal(ui_state.selected_type) << "\n";
  ofs << "capture.selected_id=" << static_cast<unsigned long long>(ui_state.selected_id) << "\n";
  ofs << "capture.focus_type=" << SelectedTypeLabelLocal(ui_state.selected_type) << "\n";
  ofs << "capture.focus_id=" << static_cast<unsigned long long>(ui_state.selected_id) << "\n";
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

  const auto type_ids = SortedPoleTypeIdsLocal(state);
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
      ofs << "draw.bundle[" << i << "].auto_branch_down_offset_default="
          << (tpl.enable_branch_down_offset ? DefaultBranchDownOffsetForCategoryLocal(tpl.category) : 0.0)
          << "\n";
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

  const wire::core::BackboneResult rebuilt_backbone = state.BuildBackboneResult();
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

  const auto& assignments = view.last_lane_assignments();
  ofs << "result.lane_assignment_snapshot_count=" << assignments.size() << "\n";
  ofs << "result.lane_assignment_count=" << assignments.size() << "\n";
  auto find_span_by_assignment_lane = [&](const wire::core::SegmentLaneAssignment& assignment,
                                          std::size_t lane_index) -> const wire::core::Span* {
    if (lane_index >= assignment.port_ids_a.size() || lane_index >= assignment.port_ids_b.size()) {
      return nullptr;
    }
    const wire::core::ObjectId port_a_id = assignment.port_ids_a[lane_index];
    const wire::core::ObjectId port_b_id = assignment.port_ids_b[lane_index];
    for (const auto& span : view.edit_state().spans.items()) {
      if (span.bundle_id != assignment.bundle_id) {
        continue;
      }
      const bool same_forward = span.port_a_id == port_a_id && span.port_b_id == port_b_id;
      const bool same_reverse = span.port_a_id == port_b_id && span.port_b_id == port_a_id;
      if (same_forward || same_reverse) {
        return &span;
      }
    }
    return nullptr;
  };
  for (std::size_t i = 0; i < assignments.size(); ++i) {
    const auto& assignment = assignments[i];
    ofs << "result.lane_assignment[" << i << "].segment_index=" << assignment.segment_index << "\n";
    ofs << "result.lane_assignment[" << i << "].pole_a_id=" << static_cast<unsigned long long>(assignment.pole_a_id)
        << "\n";
    ofs << "result.lane_assignment[" << i << "].pole_b_id=" << static_cast<unsigned long long>(assignment.pole_b_id)
        << "\n";
    ofs << "result.lane_assignment[" << i << "].bundle_id=" << static_cast<unsigned long long>(assignment.bundle_id)
        << "\n";
    ofs << "result.lane_assignment[" << i << "].flow_kind=" << static_cast<int>(assignment.flow_kind) << "\n";
    ofs << "result.lane_assignment[" << i << "].flow_rule=" << static_cast<int>(assignment.flow_decision_rule) << "\n";
    ofs << "result.lane_assignment[" << i
        << "].uses_branch_support=" << (assignment.uses_branch_support ? 1 : 0) << "\n";
    ofs << "result.lane_assignment[" << i << "].lowering_kind=" << static_cast<int>(assignment.lowering_kind) << "\n";
    ofs << "result.lane_assignment[" << i << "].branch_down_offset_m=" << assignment.branch_down_offset_m << "\n";
    ofs << "result.lane_assignment[" << i << "].lane_count=" << assignment.port_ids_a.size() << "\n";
    for (std::size_t lane = 0; lane < assignment.port_ids_a.size() && lane < assignment.port_ids_b.size(); ++lane) {
      const wire::core::Port* port_a = view.edit_state().ports.find(assignment.port_ids_a[lane]);
      const wire::core::Port* port_b = view.edit_state().ports.find(assignment.port_ids_b[lane]);
      ofs << "result.lane_assignment[" << i << "].lane[" << lane
          << "].port_a_id=" << static_cast<unsigned long long>(assignment.port_ids_a[lane]) << "\n";
      ofs << "result.lane_assignment[" << i << "].lane[" << lane
          << "].port_b_id=" << static_cast<unsigned long long>(assignment.port_ids_b[lane]) << "\n";
      if (port_a != nullptr) {
        ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].port_a_world="
            << port_a->world_position.x << "," << port_a->world_position.y << "," << port_a->world_position.z << "\n";
        ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].port_a_z="
            << port_a->world_position.z << "\n";
        ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].port_a_template_layer=" << port_a->template_layer << "\n";
      }
      if (port_b != nullptr) {
        ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].port_b_world="
            << port_b->world_position.x << "," << port_b->world_position.y << "," << port_b->world_position.z << "\n";
        ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].port_b_z="
            << port_b->world_position.z << "\n";
        ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].port_b_template_layer=" << port_b->template_layer << "\n";
      }
      if (const wire::core::Span* span = find_span_by_assignment_lane(assignment, lane); span != nullptr) {
        ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_id=" << static_cast<unsigned long long>(span->id) << "\n";
        if (const auto span_view = view.inspect_span(span->id); span_view.has_value()) {
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].span_flow_kind=" << static_cast<int>(span_view->flow_kind) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_flow_kind_label=" << FlowKindLabelLocal(span_view->flow_kind) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_flow_rule=" << static_cast<int>(span_view->flow_rule) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_flow_rule_label=" << FlowRuleLabelLocal(span_view->flow_rule) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].span_lowering_kind=" << static_cast<int>(span_view->lowering_kind) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_lowering_kind_label=" << LoweringKindLabelLocal(span_view->lowering_kind) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].span_branch_down_offset_m=" << span_view->branch_down_offset_m << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_continuity_class=" << ContinuityClassLabelLocal(span_view->continuity_class) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_default_lower_required=" << (span_view->default_lower_required ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_same_level_feasible=" << (span_view->same_level_feasible ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_same_level_reason=" << SameLevelReasonLabelLocal(span_view->same_level_reason) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_projected_spacing_topview_m=" << span_view->projected_spacing_topview_m << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_required_clearance_m=" << span_view->required_clearance_m << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_lowering_blocked_by_policy=" << (span_view->lowering_blocked_by_policy ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_unresolved_same_level_conflict="
            << (span_view->unresolved_same_level_conflict ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_solver_used_same_level_constraint="
            << (span_view->solver_used_same_level_constraint ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_used_special_case_ports=" << (span_view->used_special_case_ports ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_bundle_order_policy=" << BundleOrderPolicyLabelLocal(span_view->bundle_order_policy) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_bundle_order_choice_a=" << BundleOrderChoiceLabelLocal(span_view->bundle_order_choice_a)
            << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_bundle_order_reason_a="
            << BundleOrderChoiceReasonLabelLocal(span_view->bundle_order_choice_reason_a) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_bundle_order_choice_b=" << BundleOrderChoiceLabelLocal(span_view->bundle_order_choice_b)
            << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].span_bundle_order_reason_b="
            << BundleOrderChoiceReasonLabelLocal(span_view->bundle_order_choice_reason_b) << "\n";
        }
        if (const auto layout_view = view.inspect_support_layout(span->id); layout_view.has_value()) {
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_flow_kind=" << FlowKindLabelLocal(layout_view->flow_kind) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_bundle_order_policy=" << BundleOrderPolicyLabelLocal(layout_view->bundle_order_policy)
            << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_relation_a=" << JunctionRelationLabelLocal(layout_view->relation_a) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_relation_b=" << JunctionRelationLabelLocal(layout_view->relation_b) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_continuity_class=" << ContinuityClassLabelLocal(layout_view->continuity_class) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_default_lower_required=" << (layout_view->default_lower_required ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_same_level_feasible=" << (layout_view->same_level_feasible ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_same_level_reason=" << SameLevelReasonLabelLocal(layout_view->same_level_reason) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_projected_spacing_topview_m=" << layout_view->projected_spacing_topview_m << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_required_clearance_m=" << layout_view->required_clearance_m << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_lowering_blocked_by_policy=" << (layout_view->lowering_blocked_by_policy ? 1 : 0)
            << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_unresolved_same_level_conflict="
            << (layout_view->unresolved_same_level_conflict ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_solver_used_same_level_constraint="
            << (layout_view->solver_used_same_level_constraint ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
            << "].layout_used_special_case_ports=" << (layout_view->used_special_case_ports ? 1 : 0) << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_support=" << layout_view->start_endpoint.support_world.x << ","
              << layout_view->start_endpoint.support_world.y << "," << layout_view->start_endpoint.support_world.z
              << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_support=" << layout_view->end_endpoint.support_world.x << ","
              << layout_view->end_endpoint.support_world.y << "," << layout_view->end_endpoint.support_world.z
              << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_origin=" << layout_view->start_endpoint.origin << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_origin=" << layout_view->end_endpoint.origin << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_local_departure=" << layout_view->start_endpoint.local_departure_length_m << "\n";
          ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_local_departure=" << layout_view->end_endpoint.local_departure_length_m << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_relation=" << JunctionRelationLabelLocal(layout_view->start_endpoint.relation_kind)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_relation=" << JunctionRelationLabelLocal(layout_view->end_endpoint.relation_kind)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_continuity_class="
              << ContinuityClassLabelLocal(layout_view->start_endpoint.continuity_class) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_continuity_class="
              << ContinuityClassLabelLocal(layout_view->end_endpoint.continuity_class) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_lower_required=" << (layout_view->start_endpoint.decision.lower_required ? 1 : 0)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_lower_required=" << (layout_view->end_endpoint.decision.lower_required ? 1 : 0)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_same_level_reason="
              << SameLevelReasonLabelLocal(layout_view->start_endpoint.same_level_reason) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_same_level_reason="
              << SameLevelReasonLabelLocal(layout_view->end_endpoint.same_level_reason) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_bundle_order_policy="
              << BundleOrderPolicyLabelLocal(layout_view->start_endpoint.bundle_order_policy) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_bundle_order_policy="
              << BundleOrderPolicyLabelLocal(layout_view->end_endpoint.bundle_order_policy) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_bundle_order_choice="
              << BundleOrderChoiceLabelLocal(layout_view->start_endpoint.bundle_order_choice) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_bundle_order_choice="
              << BundleOrderChoiceLabelLocal(layout_view->end_endpoint.bundle_order_choice) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_bundle_order_reason="
              << BundleOrderChoiceReasonLabelLocal(layout_view->start_endpoint.bundle_order_choice_reason)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_bundle_order_reason="
              << BundleOrderChoiceReasonLabelLocal(layout_view->end_endpoint.bundle_order_choice_reason)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_side=" << LateralSideChoiceLabelLocal(layout_view->start_endpoint.decision.chosen_side)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_side=" << LateralSideChoiceLabelLocal(layout_view->end_endpoint.decision.chosen_side)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_side_assignment_rule="
              << SideAssignmentRuleLabelLocal(layout_view->start_endpoint.side_assignment_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_side_assignment_rule="
              << SideAssignmentRuleLabelLocal(layout_view->end_endpoint.side_assignment_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_support_orientation_rule="
              << SupportOrientationRuleLabelLocal(layout_view->start_endpoint.support_orientation_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_support_orientation_rule="
              << SupportOrientationRuleLabelLocal(layout_view->end_endpoint.support_orientation_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_start_support_orientation_basis="
              << SupportOrientationBasisLabelLocal(layout_view->start_endpoint.decision.support_orientation_basis)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_end_support_orientation_basis="
              << SupportOrientationBasisLabelLocal(layout_view->end_endpoint.decision.support_orientation_basis)
              << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane
              << "].layout_lowered_support_group_count=" << layout_view->lowered_support_groups.size() << "\n";
            for (std::size_t group_index = 0; group_index < layout_view->lowered_support_groups.size(); ++group_index) {
            const auto& group = layout_view->lowered_support_groups[group_index];
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].owner_pole_id=" << static_cast<unsigned long long>(group.owner_pole_id) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].group_rule=" << SupportGroupingRuleLabelLocal(group.grouping_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].support_group_id=" << group.support_group_id << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].grouped_port_count=" << group.grouped_port_count << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].relation=" << JunctionRelationLabelLocal(group.decision.relation_kind) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].continuity_class=" << ContinuityClassLabelLocal(group.decision.continuity_class) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].lower_required=" << (group.decision.lower_required ? 1 : 0) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].same_level_reason=" << SameLevelReasonLabelLocal(group.decision.same_level_reason) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].bundle_order_policy=" << BundleOrderPolicyLabelLocal(group.bundle_order_policy) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].bundle_order_choice=" << BundleOrderChoiceLabelLocal(group.bundle_order_choice) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].bundle_order_reason="
              << BundleOrderChoiceReasonLabelLocal(group.bundle_order_choice_reason) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].side=" << LateralSideChoiceLabelLocal(group.decision.chosen_side) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].side_assignment_rule=" << SideAssignmentRuleLabelLocal(group.side_assignment_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].support_orientation_rule="
              << SupportOrientationRuleLabelLocal(group.support_orientation_rule) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].support_orientation_basis="
              << SupportOrientationBasisLabelLocal(group.decision.support_orientation_basis) << "\n";
            ofs << "result.lane_assignment[" << i << "].lane[" << lane << "].layout_group[" << group_index
              << "].down_offset_m=" << group.down_offset_m << "\n";
            }
            write_decision_trace("result.lane_assignment[" + std::to_string(i) + "].lane[" + std::to_string(lane) + "]",
                       wire::core::EntityRef{wire::core::EntityKind::kSpan, span->id});
        }
      }
    }
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
      ofs << "result.current_span[" << current_span_index << "].bundle_order_policy="
        << BundleOrderPolicyLabelLocal(span_view->bundle_order_policy) << "\n";
      ofs << "result.current_span[" << current_span_index << "].bundle_order_choice_a="
        << BundleOrderChoiceLabelLocal(span_view->bundle_order_choice_a) << "\n";
      ofs << "result.current_span[" << current_span_index << "].bundle_order_choice_b="
        << BundleOrderChoiceLabelLocal(span_view->bundle_order_choice_b) << "\n";
    }
    if (const auto layout_view = view.inspect_support_layout(span.id); layout_view.has_value()) {
      ofs << "result.current_span[" << current_span_index << "].layout_relation_a="
        << JunctionRelationLabelLocal(layout_view->relation_a) << "\n";
      ofs << "result.current_span[" << current_span_index << "].layout_relation_b="
        << JunctionRelationLabelLocal(layout_view->relation_b) << "\n";
      ofs << "result.current_span[" << current_span_index << "].layout_lowered_support_group_count="
        << layout_view->lowered_support_groups.size() << "\n";
      ofs << "result.current_span[" << current_span_index << "].layout_start_side="
        << LateralSideChoiceLabelLocal(layout_view->start_endpoint.decision.chosen_side) << "\n";
      ofs << "result.current_span[" << current_span_index << "].layout_end_side="
        << LateralSideChoiceLabelLocal(layout_view->end_endpoint.decision.chosen_side) << "\n";
      ofs << "result.current_span[" << current_span_index << "].layout_start_orientation_basis="
        << SupportOrientationBasisLabelLocal(layout_view->start_endpoint.decision.support_orientation_basis)
        << "\n";
      ofs << "result.current_span[" << current_span_index << "].layout_end_orientation_basis="
        << SupportOrientationBasisLabelLocal(layout_view->end_endpoint.decision.support_orientation_basis)
        << "\n";
    }
    write_endpoint_junction("result.current_span[" + std::to_string(current_span_index) + "].endpoint_a_junction",
                            span.endpoint_node_a_id, span.endpoint_node_b_id);
    write_endpoint_junction("result.current_span[" + std::to_string(current_span_index) + "].endpoint_b_junction",
                            span.endpoint_node_b_id, span.endpoint_node_a_id);
    write_decision_trace("result.current_span[" + std::to_string(current_span_index) + "]",
               wire::core::EntityRef{wire::core::EntityKind::kSpan, span.id});
    ++current_span_index;
    }

  std::map<std::pair<wire::core::ObjectId, int>, wire::core::LoweredSupportGroupInspectionView> grouped_support_index{};
  for (const auto& span : view.edit_state().spans.items()) {
    if (const auto layout_view = view.inspect_support_layout(span.id); layout_view.has_value()) {
      for (const auto& group : layout_view->lowered_support_groups) {
        grouped_support_index.emplace(std::make_pair(group.owner_pole_id, group.support_group_id), group);
      }
    }
  }
  ofs << "result.grouped_lowered_support_count=" << grouped_support_index.size() << "\n";
  std::size_t grouped_support_index_i = 0;
  for (const auto& [key, group] : grouped_support_index) {
    (void)key;
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].owner_pole_id=" << static_cast<unsigned long long>(group.owner_pole_id) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].support_group_id=" << group.support_group_id << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].relation_kind=" << JunctionRelationLabelLocal(group.decision.relation_kind) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].continuity_class=" << ContinuityClassLabelLocal(group.decision.continuity_class) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].lower_required=" << (group.decision.lower_required ? 1 : 0) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].side=" << LateralSideChoiceLabelLocal(group.decision.chosen_side) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].side_assignment_rule=" << SideAssignmentRuleLabelLocal(group.side_assignment_rule) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].support_orientation_rule=" << SupportOrientationRuleLabelLocal(group.support_orientation_rule) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].support_orientation_basis="
        << SupportOrientationBasisLabelLocal(group.decision.support_orientation_basis) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].has_side_axis=" << (group.has_side_axis ? 1 : 0) << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].side_axis=" << group.side_axis.x << "," << group.side_axis.y << "," << group.side_axis.z << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].chosen_side_sign=" << group.chosen_side_sign << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].mount_world=" << group.mount_world.x << "," << group.mount_world.y << "," << group.mount_world.z
        << "\n";
    ofs << "result.grouped_lowered_support[" << grouped_support_index_i
        << "].tip_world=" << group.tip_world.x << "," << group.tip_world.y << "," << group.tip_world.z << "\n";
    ++grouped_support_index_i;
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
    const wire::core::PickResult raw_pick = scene_query.Raycast(state, camera, ui_state.draw_plane_z);
    const double hover_snap_radius_world = std::max(ui_state.draw_snap_radius_world, 1.25);
    wire::core::PickResult pick =
        CanonicalizeDrawPathPick(state, raw_pick, hover, has_ground_hit, hover_snap_radius_world);
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
          ResolveTemplateKindsForPathPick(state, ui_state.draw_bundle_template_mask, pick);
      if (ui_state.draw_hover_status.empty()) {
        ui_state.draw_hover_status =
            std::string("target: ") + PickHitKindLabelLocal(pick.hit_kind) + " " + PickTargetLabel(pick);
      }
      if (pick.hit_kind == wire::core::PickHitKind::kNode || pick.hit_kind == wire::core::PickHitKind::kSegment ||
          pick.hit_kind == wire::core::PickHitKind::kBuilding) {
        wire::core::EditResult<wire::core::CoreState::ResolveBranchPickResult> resolved{};
        if (pick.hit_kind == wire::core::PickHitKind::kNode || pick.hit_kind == wire::core::PickHitKind::kBuilding) {
          resolved.ok = true;
          resolved.value = DirectResolvedDrawPathTarget(pick);
        } else {
          wire::core::ResolveBranchPickOptions options{};
          options.selected_bundle_template_ids = pick_template_ids;
          options.snap_radius_world = ui_state.draw_snap_radius_world;
          options.create_midair_node = false;
          options.enforce_midair_template_policy = false;
          resolved = state.ResolveBranchPick(pick, options);
        }
        if (resolved.ok) {
          const std::string blocked_template = FindMidairBranchBlockedTemplateName(state, pick_template_ids);
          if (resolved.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kMidair &&
              !blocked_template.empty()) {
            ui_state.draw_hover_status += " -> warn: template " + blocked_template + " will not connect here";
          }
          ui_state.draw_hover_has_resolution = true;
          ui_state.draw_hover_resolution = resolved.value;
          ui_state.draw_hover_point = resolved.value.position;
          ui_state.draw_hover_valid = true;
          ui_state.draw_hover_status +=
              (resolved.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode)
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
        wire::core::EditResult<wire::core::CoreState::ResolveBranchPickResult> applied{};
        if (ui_state.draw_hover_pick.hit_kind == wire::core::PickHitKind::kNode ||
            ui_state.draw_hover_pick.hit_kind == wire::core::PickHitKind::kBuilding ||
            ui_state.draw_hover_resolution.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode) {
          applied.ok = true;
          applied.value = ui_state.draw_hover_resolution;
        } else {
          const std::vector<wire::core::BundleKind> click_template_ids =
              ResolveTemplateKindsForPathPick(state, ui_state.draw_bundle_template_mask, ui_state.draw_hover_pick);
          wire::core::ResolveBranchPickOptions click_options{};
          click_options.selected_bundle_template_ids = click_template_ids;
          click_options.snap_radius_world = ui_state.draw_snap_radius_world;
          click_options.create_midair_node = true;
          click_options.enforce_midair_template_policy = false;
          applied = state.ResolveBranchPick(ui_state.draw_hover_pick, click_options);
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
    } else if (support_kind == wire::core::SupportKind::kBuilding) {
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
                 wire::core::SupportKind::kBuilding));
  const int anchored_points = static_cast<int>(
      std::count_if(ui_state.draw_path_point_node_ids.begin(), ui_state.draw_path_point_node_ids.end(),
                    [](ObjectId node_id) { return node_id != wire::core::kInvalidObjectId; }));
  ImGui::Text("Support kind points: Midair=%d Building=%d", midair_points, building_points);
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
                (ui_state.draw_hover_resolution.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode)
                    ? "Node"
                    : "Midair",
                SupportKindLabelLocal(ui_state.draw_hover_resolution.support_kind));
  }

  const auto type_ids = SortedPoleTypeIdsLocal(state);
  if (!type_ids.empty()) {
    const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
    ui_state.road_pole_type_index = static_cast<int>(road_type_index);
    const wire::core::PoleTypeId road_type_id = type_ids[road_type_index];
    const auto road_type_it = state.view().pole_types().find(road_type_id);
    const std::string road_type_name =
        (road_type_it != state.view().pole_types().end()) ? road_type_it->second.name : std::to_string(road_type_id);
    if (ImGui::BeginCombo("Path PoleType", road_type_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const auto it = state.view().pole_types().find(type_ids[i]);
        const std::string label = (it != state.view().pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
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

  const auto template_ids = SortedBundleTemplateKindsLocal(state);
  if (template_ids.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "No bundle template registered in core");
  } else {
    if (ImGui::BeginCombo("Bundle Templates", BundleTemplateMultiPreviewLocal(state, ui_state).c_str())) {
      for (const wire::core::BundleKind kind : template_ids) {
        bool selected = IsBundleTemplateSelectedLocal(ui_state, kind);
        if (ImGui::Selectable(BundleTemplatePreviewLocal(state, kind).c_str(), selected,
                              ImGuiSelectableFlags_DontClosePopups)) {
          selected = !selected;
          SetBundleTemplateSelectedLocal(ui_state, kind, selected);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    const auto selected_templates = SelectedBundleTemplatesLocal(state, ui_state);
    if (selected_templates.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Select at least one bundle template");
    }
    for (wire::core::BundleKind kind : selected_templates) {
      const auto it = state.view().bundle_templates().find(kind);
      if (it == state.view().bundle_templates().end()) {
        continue;
      }
      const wire::core::BundleTemplate& bundle_template = it->second;
      ImGui::Separator();
      ImGui::TextUnformatted(BundleTemplatePreviewLocal(state, kind).c_str());
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
  const auto& dir_debug = state.view().last_path_direction_debug();
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
