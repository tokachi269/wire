#include "ui_common.hpp"

#include "city/wire/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

#include "host_coords.hpp"

constexpr std::array<city::wire::ConnectionCategory, 5> kAllCategories = {
    city::wire::ConnectionCategory::kHighVoltage,   city::wire::ConnectionCategory::kLowVoltage,
    city::wire::ConnectionCategory::kCommunication, city::wire::ConnectionCategory::kOptical,
    city::wire::ConnectionCategory::kDrop,
};

constexpr std::array<city::wire::ConnectionContext, 4> kAllConnectionContexts = {
    city::wire::ConnectionContext::kTrunkContinue,
    city::wire::ConnectionContext::kCornerPass,
    city::wire::ConnectionContext::kBranchAdd,
    city::wire::ConnectionContext::kDropAdd,
};

const std::array<city::wire::ConnectionCategory, 5>& AllCategories() {
  return kAllCategories;
}

const std::array<city::wire::ConnectionContext, 4>& AllConnectionContexts() {
  return kAllConnectionContexts;
}

const char* CategoryLabel(city::wire::ConnectionCategory category) {
  switch (category) {
  case city::wire::ConnectionCategory::kHighVoltage:
    return "HighVoltage";
  case city::wire::ConnectionCategory::kLowVoltage:
    return "LowVoltage";
  case city::wire::ConnectionCategory::kCommunication:
    return "Communication";
  case city::wire::ConnectionCategory::kOptical:
    return "Optical";
  case city::wire::ConnectionCategory::kDrop:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* ContextLabel(city::wire::ConnectionContext context) {
  switch (context) {
  case city::wire::ConnectionContext::kTrunkContinue:
    return "Trunk";
  case city::wire::ConnectionContext::kCornerPass:
    return "Corner";
  case city::wire::ConnectionContext::kBranchAdd:
    return "Branch";
  case city::wire::ConnectionContext::kDropAdd:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* PoleContextLabel(city::wire::PoleContextKind context) {
  switch (context) {
  case city::wire::PoleContextKind::kStraight:
    return "Straight";
  case city::wire::PoleContextKind::kCorner:
    return "Corner";
  case city::wire::PoleContextKind::kBranch:
    return "Branch";
  case city::wire::PoleContextKind::kTerminal:
    return "Terminal";
  default:
    return "Unknown";
  }
}

const char* PolePlacementModeLabel(city::wire::PlacementMode mode) {
  switch (mode) {
  case city::wire::PlacementMode::kAuto:
    return "Auto";
  case city::wire::PlacementMode::kManual:
    return "Manual";
  default:
    return "Unknown";
  }
}

const char* SlotSideLabel(city::wire::SlotSide side) {
  switch (side) {
  case city::wire::SlotSide::kLeft:
    return "L";
  case city::wire::SlotSide::kCenter:
    return "C";
  case city::wire::SlotSide::kRight:
    return "R";
  default:
    return "?";
  }
}

const char* SlotRoleLabel(city::wire::SlotRole role) {
  switch (role) {
  case city::wire::SlotRole::kNeutral:
    return "Neutral";
  case city::wire::SlotRole::kTrunkPreferred:
    return "Trunk";
  case city::wire::SlotRole::kBranchPreferred:
    return "Branch";
  case city::wire::SlotRole::kDropPreferred:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* PortPositionModeLabel(city::wire::PortPositionMode mode) {
  switch (mode) {
  case city::wire::PortPositionMode::kAuto:
    return "Auto";
  case city::wire::PortPositionMode::kManual:
    return "Manual";
  default:
    return "Unknown";
  }
}

const char* PortPlacementSourceLabel(city::wire::PortPlacementSourceKind source) {
  switch (source) {
  case city::wire::PortPlacementSourceKind::kPlacementBand:
    return "Band";
  case city::wire::PortPlacementSourceKind::kGenerated:
    return "Generated";
  case city::wire::PortPlacementSourceKind::kManualEdit:
    return "ManualEdit";
  case city::wire::PortPlacementSourceKind::kAerialBranch:
    return "AerialBranch";
  case city::wire::PortPlacementSourceKind::kBranchSupport:
    return "BranchSupport";
  case city::wire::PortPlacementSourceKind::kUnknown:
  default:
    return "Unknown";
  }
}

const char* PathDirectionModeLabel(city::wire::PathDirectionMode mode) {
  switch (mode) {
  case city::wire::PathDirectionMode::kAuto:
    return "Auto";
  case city::wire::PathDirectionMode::kForward:
    return "Forward";
  case city::wire::PathDirectionMode::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* PathDirectionChosenLabel(city::wire::PathDirectionChosen chosen) {
  switch (chosen) {
  case city::wire::PathDirectionChosen::kForward:
    return "Forward";
  case city::wire::PathDirectionChosen::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* SupportKindLabel(city::wire::SupportKind kind) {
  switch (kind) {
  case city::wire::SupportKind::kPole:
    return "Pole";
  case city::wire::SupportKind::kMidair:
    return "Midair";
  case city::wire::SupportKind::kExternal:
    return "External";
  default:
    return "Unknown";
  }
}

const char* BundleNodeModeLabel(city::wire::BundleNodeMode mode) {
  switch (mode) {
  case city::wire::BundleNodeMode::kNotPresent:
    return "NotPresent";
  case city::wire::BundleNodeMode::kPassThrough:
    return "PassThrough";
  default:
    return "Unknown";
  }
}

const char* PickHitKindLabel(city::wire::PickHitKind kind) {
  switch (kind) {
  case city::wire::PickHitKind::kNode:
    return "Node";
  case city::wire::PickHitKind::kSegment:
    return "Segment";
  case city::wire::PickHitKind::kGround:
    return "Ground";
  case city::wire::PickHitKind::kExternal:
    return "External";
  case city::wire::PickHitKind::kEmpty:
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

int FallbackParallelSpanCount(city::wire::ConnectionCategory category) {
  switch (category) {
  case city::wire::ConnectionCategory::kHighVoltage:
    return 3;
  case city::wire::ConnectionCategory::kLowVoltage:
    return 2;
  case city::wire::ConnectionCategory::kCommunication:
    return 1;
  case city::wire::ConnectionCategory::kOptical:
    return 1;
  case city::wire::ConnectionCategory::kDrop:
    return 1;
  default:
    return 1;
  }
}

int AutoParallelSpanCountFromPoleType(const city::wire::CoreView& view, city::wire::PoleTypeId pole_type_id,
                                      city::wire::ConnectionCategory category) {
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

std::vector<city::wire::ConnectionCategory> SelectedDrawCategories(const ViewerUiState& ui_state) {
  std::vector<city::wire::ConnectionCategory> categories;
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

std::vector<city::wire::BundleKind> SortedBundleTemplateKinds(const city::wire::CoreView& view) {
  std::vector<city::wire::BundleKind> ids;
  ids.reserve(view.bundle_templates().size());
  for (const auto& [id, _] : view.bundle_templates()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end(), [](city::wire::BundleKind a, city::wire::BundleKind b) {
    return static_cast<int>(a) < static_cast<int>(b);
  });
  return ids;
}

bool IsBundleTemplateSelected(const ViewerUiState& ui_state, city::wire::BundleKind kind) {
  const int bit_index = static_cast<int>(kind);
  if (bit_index < 0 || bit_index >= 32) {
    return false;
  }
  return (ui_state.draw_bundle_template_mask & (1u << bit_index)) != 0u;
}

void SetBundleTemplateSelected(ViewerUiState& ui_state, city::wire::BundleKind kind, bool selected) {
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

std::vector<city::wire::BundleKind> SelectedBundleTemplates(const city::wire::CoreView& view,
                                                            const ViewerUiState& ui_state) {
  std::vector<city::wire::BundleKind> selected{};
  const auto all = SortedBundleTemplateKinds(view);
  for (city::wire::BundleKind kind : all) {
    if (IsBundleTemplateSelected(ui_state, kind)) {
      selected.push_back(kind);
    }
  }
  return selected;
}

std::string BundleTemplatePreview(const city::wire::CoreView& view, city::wire::BundleKind kind) {
  const auto it = view.bundle_templates().find(kind);
  if (it == view.bundle_templates().end()) {
    return "UnknownTemplate";
  }
  return it->second.name.empty() ? std::to_string(static_cast<int>(kind)) : it->second.name;
}

std::string BundleTemplateMultiPreview(const city::wire::CoreView& view, const ViewerUiState& ui_state) {
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

int ResolveBundleTemplateCount(ViewerUiState& ui_state, const city::wire::BundleTemplate& bundle_template,
                               city::wire::BundleKind kind) {
  if (bundle_template.count_rule == city::wire::BundleCountRuleKind::kFixed) {
    return bundle_template.fixed_count;
  }
  const int key = static_cast<int>(kind);
  auto it = ui_state.draw_bundle_count_by_template.find(key);
  int value = (it == ui_state.draw_bundle_count_by_template.end()) ? bundle_template.default_count : it->second;
  value = std::clamp(value, bundle_template.min_count, bundle_template.max_count);
  ui_state.draw_bundle_count_by_template[key] = value;
  return value;
}

std::vector<city::wire::PoleTypeId> SortedPoleTypeIds(const city::wire::CoreView& view) {
  std::vector<city::wire::PoleTypeId> ids;
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

Vector3 ToRaylib(const city::wire::Vec3d& ue_xyz) {
  return InternalToHostWorld(ue_xyz);
}

BoundingBox ToRaylibBounds(const city::wire::AABBd& box_ue) {
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

double PolylineLength(const std::vector<city::wire::Vec3d>& points) {
  return city::wire::PolylineLength(points);
}

void PushLog(ViewerUiState& ui_state, const std::string& line) {
  ui_state.logs.push_back(line);
  if (ui_state.logs.size() > 12) {
    ui_state.logs.erase(ui_state.logs.begin());
  }
}
