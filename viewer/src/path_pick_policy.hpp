#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "wire/core/core_state.hpp"

std::vector<wire::core::BundleKind> SelectedBundleTemplateKinds(const wire::core::CoreState& state,
                                                                std::uint32_t selected_template_mask);

std::vector<wire::core::BundleKind> ResolveTemplateKindsForPathPick(const wire::core::CoreState& state,
                                                                    std::uint32_t selected_template_mask,
                                                                    const wire::core::PickResult& pick);

std::string FindMidairBranchBlockedTemplateName(const wire::core::CoreState& state,
                                                const std::vector<wire::core::BundleKind>& template_ids);

wire::core::PickResult NormalizeDrawPathPick(const wire::core::CoreState& state, const wire::core::PickResult& pick,
                                             double endpoint_snap_radius_world);

wire::core::PickResult CanonicalizeDrawPathPick(const wire::core::CoreState& state, const wire::core::PickResult& pick,
                                                const wire::core::Vec3d& ground_hover_world, bool has_ground_hit,
                                                double snap_radius_world);

wire::core::PickResult PromoteGroundHoverToNearbyPolePick(const wire::core::CoreState& state,
                                                          const wire::core::Vec3d& ground_hover_world,
                                                          double snap_radius_world);
