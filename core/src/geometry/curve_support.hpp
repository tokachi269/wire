#pragma once

#include "wire/core/span_layout_types.hpp"

namespace wire::core {

struct Span;
struct Bundle;
struct BundleTemplate;
struct Port;

[[nodiscard]] bool build_attachment_frame(const Vec3d& tangent, Vec3d* forward, Vec3d* lateral, Vec3d* up);

[[nodiscard]] Vec3d attachment_local_to_world(const Vec3d& origin, const Vec3d& forward, const Vec3d& lateral,
                                             const Vec3d& up, const Vec3d& local);

[[nodiscard]] bool resolve_attachment_socket_pair(const AttachmentTemplate& attachment_template,
                                                  const AttachmentSocketTemplate** out_a,
                                                  const AttachmentSocketTemplate** out_b,
                                                  const AttachmentInternalPathTemplate** out_internal_path);

[[nodiscard]] BackboneFlowKind span_layout_flow_kind_for_span(const Span& span, const Port& port_a,
                                                              const Port& port_b);

[[nodiscard]] CurveEndpointMode curve_endpoint_mode_for_attachment_style(CableAttachmentStyleHint attachment_style,
                                                                         const Bundle* bundle,
                                                                         const BundleTemplate* bundle_template);

} // namespace wire::core
