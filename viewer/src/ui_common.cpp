#include "ui_common.hpp"

#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

#include "host_coords.hpp"

constexpr std::array<wire::core::ConnectionCategory, 5> kAllCategories = {
    wire::core::ConnectionCategory::kHighVoltage,   wire::core::ConnectionCategory::kLowVoltage,
    wire::core::ConnectionCategory::kCommunication, wire::core::ConnectionCategory::kOptical,
    wire::core::ConnectionCategory::kDrop,
};

constexpr std::array<wire::core::ConnectionContext, 4> kAllConnectionContexts = {
    wire::core::ConnectionContext::kTrunkContinue,
    wire::core::ConnectionContext::kCornerPass,
    wire::core::ConnectionContext::kBranchAdd,
    wire::core::ConnectionContext::kDropAdd,
};

const std::array<wire::core::ConnectionCategory, 5>& AllCategories() {
  return kAllCategories;
}

const std::array<wire::core::ConnectionContext, 4>& AllConnectionContexts() {
  return kAllConnectionContexts;
}

const char* CategoryLabel(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return "HighVoltage";
  case wire::core::ConnectionCategory::kLowVoltage:
    return "LowVoltage";
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

const char* ContextLabel(wire::core::ConnectionContext context) {
  switch (context) {
  case wire::core::ConnectionContext::kTrunkContinue:
    return "Trunk";
  case wire::core::ConnectionContext::kCornerPass:
    return "Corner";
  case wire::core::ConnectionContext::kBranchAdd:
    return "Branch";
  case wire::core::ConnectionContext::kDropAdd:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* PoleContextLabel(wire::core::PoleContextKind context) {
  switch (context) {
  case wire::core::PoleContextKind::kStraight:
    return "Straight";
  case wire::core::PoleContextKind::kCorner:
    return "Corner";
  case wire::core::PoleContextKind::kBranch:
    return "Branch";
  case wire::core::PoleContextKind::kTerminal:
    return "Terminal";
  default:
    return "Unknown";
  }
}

const char* PolePlacementModeLabel(wire::core::PlacementMode mode) {
  switch (mode) {
  case wire::core::PlacementMode::kAuto:
    return "Auto";
  case wire::core::PlacementMode::kManual:
    return "Manual";
  default:
    return "Unknown";
  }
}

const char* SlotSideLabel(wire::core::SlotSide side) {
  switch (side) {
  case wire::core::SlotSide::kLeft:
    return "L";
  case wire::core::SlotSide::kCenter:
    return "C";
  case wire::core::SlotSide::kRight:
    return "R";
  default:
    return "?";
  }
}

const char* SlotRoleLabel(wire::core::SlotRole role) {
  switch (role) {
  case wire::core::SlotRole::kNeutral:
    return "Neutral";
  case wire::core::SlotRole::kTrunkPreferred:
    return "Trunk";
  case wire::core::SlotRole::kBranchPreferred:
    return "Branch";
  case wire::core::SlotRole::kDropPreferred:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* PortPositionModeLabel(wire::core::PortPositionMode mode) {
  switch (mode) {
  case wire::core::PortPositionMode::kAuto:
    return "Auto";
  case wire::core::PortPositionMode::kManual:
    return "Manual";
  default:
    return "Unknown";
  }
}

const char* PortPlacementSourceLabel(wire::core::PortPlacementSourceKind source) {
  switch (source) {
  case wire::core::PortPlacementSourceKind::kPlacementBand:
    return "Band";
  case wire::core::PortPlacementSourceKind::kGenerated:
    return "Generated";
  case wire::core::PortPlacementSourceKind::kManualEdit:
    return "ManualEdit";
  case wire::core::PortPlacementSourceKind::kAerialBranch:
    return "AerialBranch";
  case wire::core::PortPlacementSourceKind::kBranchSupport:
    return "BranchSupport";
  case wire::core::PortPlacementSourceKind::kUnknown:
  default:
    return "Unknown";
  }
}

const char* PathDirectionModeLabel(wire::core::PathDirectionMode mode) {
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

const char* PathDirectionChosenLabel(wire::core::PathDirectionChosen chosen) {
  switch (chosen) {
  case wire::core::PathDirectionChosen::kForward:
    return "Forward";
  case wire::core::PathDirectionChosen::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* SupportKindLabel(wire::core::SupportKind kind) {
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

const char* BundleNodeModeLabel(wire::core::BundleNodeMode mode) {
  switch (mode) {
  case wire::core::BundleNodeMode::kNotPresent:
    return "NotPresent";
  case wire::core::BundleNodeMode::kPassThrough:
    return "PassThrough";
  default:
    return "Unknown";
  }
}

const char* PickHitKindLabel(wire::core::PickHitKind kind) {
  switch (kind) {
  case wire::core::PickHitKind::kNode:
    return "Node";
  case wire::core::PickHitKind::kSegment:
    return "Segment";
  case wire::core::PickHitKind::kGround:
    return "Ground";
  case wire::core::PickHitKind::kExternal:
    return "External";
  case wire::core::PickHitKind::kEmpty:
  default:
    return "Empty";
  }
}

const char* ModeLabel(EditMode mode) {
  switch (mode) {
  case EditMode::kDrawPath:
    return "DrawPath";
  default:
    return "Unknown";
  }
}

const char* SelectedTypeLabel(SelectedType selected_type) {
  switch (selected_type) {
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
  case SelectedType::kSpanLayout:
    return "SpanLayout";
  case SelectedType::kDetailCurve:
    return "DetailCurve";
  case SelectedType::kJunction:
    return "Junction";
  case SelectedType::kNone:
  default:
    return "None";
  }
}

int FallbackParallelSpanCount(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return 3;
  case wire::core::ConnectionCategory::kLowVoltage:
    return 2;
  case wire::core::ConnectionCategory::kCommunication:
    return 1;
  case wire::core::ConnectionCategory::kOptical:
    return 1;
  case wire::core::ConnectionCategory::kDrop:
    return 1;
  default:
    return 1;
  }
}

int AutoParallelSpanCountFromPoleType(const wire::core::CoreView& view, wire::core::PoleTypeId pole_type_id,
                                      wire::core::ConnectionCategory category) {
  const int count = view.count_port_bands(pole_type_id, category);
  if (count <= 0) {
    return FallbackParallelSpanCount(category);
  }
  return count;
}

bool IsDrawCategorySelected(const ViewerUiState& ui_state, int category_index) {
  if (category_index < 0 || category_index >= static_cast<int>(kAllCategories.size())) {
    return false;
  }
  return (ui_state.draw_category_mask & (1u << category_index)) != 0u;
}

void SetDrawCategorySelected(ViewerUiState& ui_state, int category_index, bool selected) {
  if (category_index < 0 || category_index >= static_cast<int>(kAllCategories.size())) {
    return;
  }
  const std::uint32_t bit = (1u << category_index);
  if (selected) {
    ui_state.draw_category_mask |= bit;
  } else {
    ui_state.draw_category_mask &= ~bit;
  }
}

std::vector<wire::core::ConnectionCategory> SelectedDrawCategories(const ViewerUiState& ui_state) {
  std::vector<wire::core::ConnectionCategory> categories;
  for (int i = 0; i < static_cast<int>(kAllCategories.size()); ++i) {
    if (IsDrawCategorySelected(ui_state, i)) {
      categories.push_back(kAllCategories[i]);
    }
  }
  return categories;
}

std::string DrawCategoryPreview(const ViewerUiState& ui_state) {
  const auto categories = SelectedDrawCategories(ui_state);
  if (categories.empty()) {
    return "(none)";
  }
  if (categories.size() == 1) {
    return CategoryLabel(categories.front());
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < categories.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << CategoryLabel(categories[i]);
  }
  return oss.str();
}

std::vector<wire::core::BundleKind> SortedBundleTemplateKinds(const wire::core::CoreView& view) {
  std::vector<wire::core::BundleKind> ids;
  ids.reserve(view.bundle_templates().size());
  for (const auto& [id, _] : view.bundle_templates()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end(), [](wire::core::BundleKind a, wire::core::BundleKind b) {
    return static_cast<int>(a) < static_cast<int>(b);
  });
  return ids;
}

bool IsBundleTemplateSelected(const ViewerUiState& ui_state, wire::core::BundleKind kind) {
  const int bit_index = static_cast<int>(kind);
  if (bit_index < 0 || bit_index >= 32) {
    return false;
  }
  return (ui_state.draw_bundle_template_mask & (1u << bit_index)) != 0u;
}

void SetBundleTemplateSelected(ViewerUiState& ui_state, wire::core::BundleKind kind, bool selected) {
  const int bit_index = static_cast<int>(kind);
  if (bit_index < 0 || bit_index >= 32) {
    return;
  }
  const std::uint32_t bit = (1u << bit_index);
  if (selected) {
    ui_state.draw_bundle_template_mask |= bit;
  } else {
    ui_state.draw_bundle_template_mask &= ~bit;
  }
}

std::vector<wire::core::BundleKind> SelectedBundleTemplates(const wire::core::CoreView& view,
                                                            const ViewerUiState& ui_state) {
  std::vector<wire::core::BundleKind> selected{};
  const auto all = SortedBundleTemplateKinds(view);
  for (wire::core::BundleKind kind : all) {
    if (IsBundleTemplateSelected(ui_state, kind)) {
      selected.push_back(kind);
    }
  }
  return selected;
}

std::string BundleTemplatePreview(const wire::core::CoreView& view, wire::core::BundleKind kind) {
  const auto it = view.bundle_templates().find(kind);
  if (it == view.bundle_templates().end()) {
    return "UnknownTemplate";
  }
  return it->second.name.empty() ? std::to_string(static_cast<int>(kind)) : it->second.name;
}

std::string BundleTemplateMultiPreview(const wire::core::CoreView& view, const ViewerUiState& ui_state) {
  const auto selected = SelectedBundleTemplates(view, ui_state);
  if (selected.empty()) {
    return "(none)";
  }
  if (selected.size() == 1) {
    return BundleTemplatePreview(view, selected.front());
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << BundleTemplatePreview(view, selected[i]);
  }
  return oss.str();
}

int ResolveBundleTemplateCount(ViewerUiState& ui_state, const wire::core::BundleTemplate& bundle_template,
                               wire::core::BundleKind kind) {
  if (bundle_template.count_rule == wire::core::BundleCountRuleKind::kFixed) {
    return bundle_template.fixed_count;
  }
  const int key = static_cast<int>(kind);
  auto it = ui_state.draw_bundle_count_by_template.find(key);
  int value = (it == ui_state.draw_bundle_count_by_template.end()) ? bundle_template.default_count : it->second;
  value = std::clamp(value, bundle_template.min_count, bundle_template.max_count);
  ui_state.draw_bundle_count_by_template[key] = value;
  return value;
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

std::size_t ClampedTypeIndex(int current, std::size_t count) {
  if (count == 0) {
    return 0;
  }
  const int max_index = static_cast<int>(count - 1);
  return static_cast<std::size_t>(std::clamp(current, 0, max_index));
}

Vector3 ToRaylib(const wire::core::Vec3d& ue_xyz) {
  return InternalToHostWorld(ue_xyz);
}

BoundingBox ToRaylibBounds(const wire::core::AABBd& box_ue) {
  const Vector3 a = ToRaylib(box_ue.min);
  const Vector3 b = ToRaylib(box_ue.max);
  BoundingBox out{};
  out.min = {
      std::min(a.x, b.x),
      std::min(a.y, b.y),
      std::min(a.z, b.z),
  };
  out.max = {
      std::max(a.x, b.x),
      std::max(a.y, b.y),
      std::max(a.z, b.z),
  };
  return out;
}

double PolylineLength(const std::vector<wire::core::Vec3d>& points) {
  return wire::core::PolylineLength(points);
}

void PushLog(ViewerUiState& ui_state, const std::string& line) {
  ui_state.logs.push_back(line);
  if (ui_state.logs.size() > 12) {
    ui_state.logs.erase(ui_state.logs.begin());
  }
}
