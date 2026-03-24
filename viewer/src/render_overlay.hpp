#pragma once

#include "app_state.hpp"

void DrawAxes();
void UpdatePreferredVisibleSpans(const wire::core::CoreView& view, const Camera3D& camera, ViewerUiState& ui_state);
void DrawPickHighlight(const wire::core::PickResult& pick, bool has_resolution,
                       const wire::core::ResolveBranchPickResult& resolution);
void DrawBackboneOverlay(const wire::core::BackboneResult& backbone, const ViewerUiState& ui_state);
void DrawCore(const wire::core::CoreView& view, const ViewerUiState& ui_state);


