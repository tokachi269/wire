#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "raylib.h"
#include "app_state.hpp"

const std::array<wire::core::ConnectionCategory, 5>& AllCategories();
const std::array<wire::core::ConnectionContext, 4>& AllConnectionContexts();

void PushLog(ViewerUiState& ui_state, const std::string& line);

const char* CategoryLabel(wire::core::ConnectionCategory category);
const char* ContextLabel(wire::core::ConnectionContext context);
const char* PoleContextLabel(wire::core::PoleContextKind context);
const char* PolePlacementModeLabel(wire::core::PlacementMode mode);
const char* SlotSideLabel(wire::core::SlotSide side);
const char* SlotRoleLabel(wire::core::SlotRole role);
const char* PortPositionModeLabel(wire::core::PortPositionMode mode);
const char* PortPlacementSourceLabel(wire::core::PortPlacementSourceKind source);
const char* PathDirectionModeLabel(wire::core::PathDirectionMode mode);
const char* PathDirectionChosenLabel(wire::core::PathDirectionChosen chosen);
const char* SupportKindLabel(wire::core::SupportKind kind);
const char* BundleNodeModeLabel(wire::core::BundleNodeMode mode);
const char* PickHitKindLabel(wire::core::PickHitKind kind);
const char* ModeLabel(EditMode mode);
const char* SelectedTypeLabel(SelectedType selected_type);

int FallbackParallelSpanCount(wire::core::ConnectionCategory category);
int AutoParallelSpanCountFromPoleType(const wire::core::CoreView& view, wire::core::PoleTypeId pole_type_id,
                                      wire::core::ConnectionCategory category);
bool IsDrawCategorySelected(const ViewerUiState& ui_state, int category_index);
void SetDrawCategorySelected(ViewerUiState& ui_state, int category_index, bool selected);
std::string DrawCategoryPreview(const ViewerUiState& ui_state);

std::vector<wire::core::BundleKind> SortedBundleTemplateKinds(const wire::core::CoreView& view);
bool IsBundleTemplateSelected(const ViewerUiState& ui_state, wire::core::BundleKind kind);
void SetBundleTemplateSelected(ViewerUiState& ui_state, wire::core::BundleKind kind, bool selected);
std::vector<wire::core::BundleKind> SelectedBundleTemplates(const wire::core::CoreView& view,
                                                            const ViewerUiState& ui_state);
std::string BundleTemplatePreview(const wire::core::CoreView& view, wire::core::BundleKind kind);
std::string BundleTemplateMultiPreview(const wire::core::CoreView& view, const ViewerUiState& ui_state);
int ResolveBundleTemplateCount(ViewerUiState& ui_state, const wire::core::BundleTemplate& bundle_template,
                               wire::core::BundleKind kind);

std::vector<wire::core::PoleTypeId> SortedPoleTypeIds(const wire::core::CoreView& view);
std::size_t ClampedTypeIndex(int current, std::size_t count);
Vector3 ToRaylib(const wire::core::Vec3d& ue_xyz);
BoundingBox ToRaylibBounds(const wire::core::AABBd& box_ue);
double PolylineLength(const std::vector<wire::core::Vec3d>& points);


