#pragma once

#include <string>

#include "raylib.h"
#include "city/wire/types.hpp"
#include "city/wire/workflow_types.hpp"

namespace city::wire {
class CoreState;
class CoreView;
}

bool TryPickGroundPoint(const Camera3D& camera, double ue_plane_z, city::wire::Vec3d* out_ue_point);

double DistancePointToSegmentSquared(const city::wire::Vec3d& p, const city::wire::Vec3d& a, const city::wire::Vec3d& b,
                                     double* out_t, city::wire::Vec3d* out_closest);

class ISceneQuery {
public:
  virtual ~ISceneQuery() = default;
  virtual city::wire::PickResult Raycast(const city::wire::CoreView& view, const Camera3D& camera,
                                         double draw_plane_z) const = 0;
};

class ViewerSceneQuery final : public ISceneQuery {
public:
  city::wire::PickResult Raycast(const city::wire::CoreView& view, const Camera3D& camera,
                                 double draw_plane_z) const override;
};

std::string PickTargetLabel(const city::wire::PickResult& pick);
