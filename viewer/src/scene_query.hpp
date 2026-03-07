#pragma once

#include <string>

#include "raylib.h"
#include "wire/core/core_state.hpp"

bool TryPickGroundPoint(const Camera3D& camera, double ue_plane_z, wire::core::Vec3d* out_ue_point);

double DistancePointToSegmentSquared(const wire::core::Vec3d& p, const wire::core::Vec3d& a, const wire::core::Vec3d& b,
                                     double* out_t, wire::core::Vec3d* out_closest);

class ISceneQuery {
public:
  virtual ~ISceneQuery() = default;
  virtual wire::core::PickResult Raycast(const wire::core::CoreState& state, const Camera3D& camera,
                                         double draw_plane_z) const = 0;
};

class ViewerSceneQuery final : public ISceneQuery {
public:
  wire::core::PickResult Raycast(const wire::core::CoreState& state, const Camera3D& camera,
                                 double draw_plane_z) const override;
};

std::string PickTargetLabel(const wire::core::PickResult& pick);
