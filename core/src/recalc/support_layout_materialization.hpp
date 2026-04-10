#pragma once

#include "wire/core/support_layout_types.hpp"

namespace wire::core {

class CoreState;
struct Span;
struct Bundle;
struct BundleTemplate;
struct CableTemplate;
struct Port;
struct Pole;
struct EditState;
struct CacheState;

[[nodiscard]] bool build_attachment_frame(const Vec3d& tangent, Vec3d* forward, Vec3d* lateral, Vec3d* up);

[[nodiscard]] Vec3d attachment_local_to_world(const Vec3d& origin, const Vec3d& forward, const Vec3d& lateral,
                                             const Vec3d& up, const Vec3d& local);

[[nodiscard]] bool resolve_attachment_socket_pair(const AttachmentTemplate& attachment_template,
                                                  const AttachmentSocketTemplate** out_a,
                                                  const AttachmentSocketTemplate** out_b,
                                                  const AttachmentInternalPathTemplate** out_internal_path);

[[nodiscard]] bool endpoint_uses_grouped_lowered_support(const SupportLayoutEndpoint* endpoint);

[[nodiscard]] double template_layer_base_z_for_port_category(const CoreState& state, const Pole& pole,
                                                             ConnectionCategory category);

[[nodiscard]] SupportLayoutOriginKind support_layout_origin_from_port(const Port& port);

[[nodiscard]] BackboneFlowKind support_layout_flow_kind_for_span(const Span& span, const Port& port_a, const Port& port_b);

[[nodiscard]] CurveEndpointMode curve_endpoint_mode_for_attachment_style(CableAttachmentStyleHint attachment_style,
                                                                         const Bundle* bundle,
                                                                         const BundleTemplate* bundle_template);

[[nodiscard]] CurveEndpointMode curve_endpoint_mode_for_template(const CableTemplate* cable_template,
                                                                 const Bundle* bundle,
                                                                 const BundleTemplate* bundle_template);

[[nodiscard]] SupportLayoutEndpoint build_support_layout_endpoint(
    const CoreState& state, const Span& span, const Port& port, const Pole* owner_pole, const Vec3d& chord_dir,
    double basis_length, double endpoint_offset_m, double effective_sag_ratio, double bend_stiffness_hint,
    double min_bend_radius_hint_m, CableContinuityPolicyHint continuity_preference, CurvePassMode pass_mode,
    CurveEndpointMode endpoint_mode, BackboneFlowKind flow_kind, int resolved_socket_id, bool socket_override_active,
    double automatic_branch_down_offset_m, const HierarchicalVariationSample& down_offset_variation,
    double resolved_branch_down_offset_m, bool is_start_endpoint);

void apply_endpoint_decision_to_layout_endpoint(const SupportLayoutSemanticDecision& decision,
                                                SupportLayoutEndpoint* endpoint);

void apply_support_layout_decision_seed_endpoint(const SupportLayoutDecisionSeedEndpoint& seed,
                                                 SupportLayoutEndpoint* endpoint);

void apply_support_layout_decision_seed(const SpanSupportLayoutDecisionSeed& seed, SpanSupportLayoutEntry* layout);

[[nodiscard]] std::vector<LoweredSupportGroupKey>
collect_support_group_keys_for_seed(const SpanSupportLayoutDecisionSeed& seed);

void rebuild_lowered_support_groups_for_keys(const CoreState& state, const EditState& edit_state, CacheState* cache_state,
                                             const std::vector<LoweredSupportGroupKey>& keys);

} // namespace wire::core
