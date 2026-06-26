#pragma once

#include "support_layout_materialization.hpp"

namespace wire::core {

class CoreState;
struct SupportGroupDecision;
struct CacheState;
struct EditState;
struct LoweredSupportGroupPlacement;

void apply_materialized_visual_arm_geometry(const CoreState& state, const Port& port, const Pole* pole,
                                            LayoutEndpoint* endpoint);

void finalize_support_layout_materialization(const Vec3d& fallback_chord_dir, SpanLayoutEntry* layout);

LoweredSupportGroupPlacement build_grouped_support_placement_from_decision(const CoreState& state,
                                                                           const SupportGroupDecision& group_decision,
                                                                           const EditState& edit_state,
                                                                           const CacheState& cache_state,
                                                                           const ConnectionCategory* observed_category = nullptr,
                                                                           const std::vector<Vec3d>* observed_attachment_worlds = nullptr);

} // namespace wire::core
