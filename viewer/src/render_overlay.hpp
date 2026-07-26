#pragma once

#include "app_state.hpp"

void DrawAxes();
void UpdatePreferredVisibleSpans(const city::wire::CoreView& view, const Camera3D& camera, ViewerUiState& ui_state);
void DrawPickHighlight(const city::wire::PickResult& pick, bool has_resolution,
                       const city::wire::ResolveBranchPickResult& resolution);
void DrawBackboneOverlay(const city::wire::BackboneResult& backbone, const ViewerUiState& ui_state);
void DrawCore(const city::wire::CoreView& view, const ViewerUiState& ui_state);


