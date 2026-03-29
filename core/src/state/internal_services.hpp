#pragma once

#include "wire/core/core_state.hpp"

#include <optional>
#include <vector>

namespace wire::core::state_internal {

struct OwnedEndpointIds {
  std::vector<ObjectId> port_ids{};
  std::vector<ObjectId> anchor_ids{};
};

struct OverrideResolutionService {
  static bool HasPoleOrientationOverride(const CoreState& state, ObjectId pole_id);
  static bool HasSpanEndpointSocketOverride(const CoreState& state, ObjectId span_id, bool is_start_endpoint);
  static bool HasSpanBranchDownOffsetOverride(const CoreState& state, ObjectId span_id);
  static std::optional<double> ResolvePoleManualYawOverride(const CoreState& state, const Pole& pole);
  static std::optional<bool> ResolvePoleFlip180Override(const CoreState& state, const Pole& pole);
  static int ResolveSpanEndpointSocketId(const CoreState& state, const Span& span, bool is_start_endpoint);
  static double ResolveSpanBranchDownOffsetM(const CoreState& state, const Span& span, double automatic_value);
};

struct EndpointRefreshService {
  static OwnedEndpointIds CollectOwnedEndpointIds(const CoreState& state, ObjectId pole_id);
  static void RefreshOwnedEndpointsFromPole(CoreState& state, ObjectId pole_id, ChangeSet* change_set,
                                            const Pole* previous_pole = nullptr,
                                            const PortLayoutYawOverride* previous_row_layout_yaw_override = nullptr);
};

struct TemplateMutationService {
  static EditResult<bool> UpdateCableTemplate(CoreState& state, const CableTemplate& cable_template,
                                              const std::vector<ObjectId>& preferred_visible_span_ids);
  static EditResult<bool> UpdateBundleTemplate(CoreState& state, const BundleTemplate& bundle_template);
  static EditResult<bool> UpdateAttachmentTemplate(CoreState& state, const AttachmentTemplate& attachment_template,
                                                   bool mark_dependent_spans_dirty);
  static EditResult<bool> ResetAllSpanReferenceLengths(CoreState& state, bool mark_all_spans_dirty);
};

} // namespace wire::core::state_internal
