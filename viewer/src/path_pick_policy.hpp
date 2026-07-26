#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "city/wire/types.hpp"
#include "city/wire/workflow_types.hpp"

namespace city::wire {
class CoreState;
class CoreView;
}

std::vector<city::wire::BundleKind> SelectedBundleTemplateKinds(const city::wire::CoreView& view,
                                                                std::uint32_t selected_template_mask);

std::vector<city::wire::BundleKind> ResolveTemplateKindsForPathPick(const city::wire::CoreView& view,
                                                                    std::uint32_t selected_template_mask,
                                                                    const city::wire::PickResult& pick);

std::string FindMidairBranchBlockedTemplateName(const city::wire::CoreView& view,
                                                const std::vector<city::wire::BundleKind>& template_ids);

city::wire::PickResult NormalizeDrawPathPick(const city::wire::CoreView& view, const city::wire::PickResult& pick,
                                             double endpoint_snap_radius_world);

city::wire::PickResult CanonicalizeDrawPathPick(const city::wire::CoreView& view, const city::wire::PickResult& pick,
                                                const city::wire::Vec3d& ground_hover_world, bool has_ground_hit,
                                                double snap_radius_world);

city::wire::PickResult PromoteGroundHoverToNearbyPolePick(const city::wire::CoreView& view,
                                                          const city::wire::Vec3d& ground_hover_world,
                                                          double snap_radius_world);
