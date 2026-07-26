#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "raylib.h"
#include "app_state.hpp"

const std::array<city::wire::ConnectionCategory, 5>& AllCategories();
const std::array<city::wire::ConnectionContext, 4>& AllConnectionContexts();

void PushLog(ViewerUiState& ui_state, const std::string& line);

const char* CategoryLabel(city::wire::ConnectionCategory category);
const char* ContextLabel(city::wire::ConnectionContext context);
const char* PoleContextLabel(city::wire::PoleContextKind context);
const char* PolePlacementModeLabel(city::wire::PlacementMode mode);
const char* SlotSideLabel(city::wire::SlotSide side);
const char* SlotRoleLabel(city::wire::SlotRole role);
const char* PortPositionModeLabel(city::wire::PortPositionMode mode);
const char* PortPlacementSourceLabel(city::wire::PortPlacementSourceKind source);
const char* PathDirectionModeLabel(city::wire::PathDirectionMode mode);
const char* PathDirectionChosenLabel(city::wire::PathDirectionChosen chosen);
const char* SupportKindLabel(city::wire::SupportKind kind);
const char* BundleNodeModeLabel(city::wire::BundleNodeMode mode);
const char* PickHitKindLabel(city::wire::PickHitKind kind);
const char* ModeLabel(EditMode mode);
const char* SelectedTypeLabel(SelectedType selected_type);

int FallbackParallelSpanCount(city::wire::ConnectionCategory category);
int AutoParallelSpanCountFromPoleType(const city::wire::CoreView& view, city::wire::PoleTypeId pole_type_id,
                                      city::wire::ConnectionCategory category);
bool IsDrawCategorySelected(const ViewerUiState& ui_state, int category_index);
void SetDrawCategorySelected(ViewerUiState& ui_state, int category_index, bool selected);
std::string DrawCategoryPreview(const ViewerUiState& ui_state);

std::vector<city::wire::BundleKind> SortedBundleTemplateKinds(const city::wire::CoreView& view);
bool IsBundleTemplateSelected(const ViewerUiState& ui_state, city::wire::BundleKind kind);
void SetBundleTemplateSelected(ViewerUiState& ui_state, city::wire::BundleKind kind, bool selected);
std::vector<city::wire::BundleKind> SelectedBundleTemplates(const city::wire::CoreView& view,
                                                            const ViewerUiState& ui_state);
std::string BundleTemplatePreview(const city::wire::CoreView& view, city::wire::BundleKind kind);
std::string BundleTemplateMultiPreview(const city::wire::CoreView& view, const ViewerUiState& ui_state);
int ResolveBundleTemplateCount(ViewerUiState& ui_state, const city::wire::BundleTemplate& bundle_template,
                               city::wire::BundleKind kind);

std::vector<city::wire::PoleTypeId> SortedPoleTypeIds(const city::wire::CoreView& view);
std::size_t ClampedTypeIndex(int current, std::size_t count);
Vector3 ToRaylib(const city::wire::Vec3d& ue_xyz);
BoundingBox ToRaylibBounds(const city::wire::AABBd& box_ue);
double PolylineLength(const std::vector<city::wire::Vec3d>& points);


