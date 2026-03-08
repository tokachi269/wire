#pragma once

#include "app_state.hpp"

void DrawAxes();
void UpdatePreferredVisibleSpans(const CoreState& state, const Camera3D& camera, ViewerUiState& ui_state);
void DrawPickHighlight(const CoreState& state, const wire::core::PickResult& pick, bool has_resolution,
                       const wire::core::CoreState::ResolveBranchPickResult& resolution);
void DrawBackboneOverlay(const wire::core::BackboneResult& backbone, const ViewerUiState& ui_state);
void DrawCore(const CoreState& state, const ViewerUiState& ui_state);


