#include "detail_curve_postprocess.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "support_layout_materialization.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core {

namespace {
constexpr double kZeroLengthEps = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double pole_band_chord_lateral_m(const CoreState& state, const Span& span, bool is_start_endpoint, const Pole& pole,
                                 double layout_yaw_deg, const Port& port, double fallback_lateral_m) {
  const PoleFrame frame = BuildPoleFrame(pole.world_transform, layout_yaw_deg);
  if (const SpanSupportLayoutEntry* layout = state.find_span_support_layout(span.id); layout != nullptr) {
    const SupportLayoutEndpoint& endpoint = is_start_endpoint ? layout->start : layout->end;
    if (endpoint.owner_pole_id == pole.id) {
      const Vec3d local = WorldPointToLocal(frame, endpoint.endpoint_world);
      if (std::isfinite(local.y)) {
        return local.y;
      }
    }
  }
  const Vec3d port_local = WorldPointToLocal(frame, port.world_position);
  if (std::isfinite(port_local.y)) {
    return port_local.y;
  }
  return fallback_lateral_m;
}

std::vector<CurveLengthInterval> merged_intervals(std::vector<CurveLengthInterval> intervals, double total_length_m) {
  std::vector<CurveLengthInterval> merged{};
  if (intervals.empty() || total_length_m <= kZeroLengthEps) {
    return merged;
  }
  for (CurveLengthInterval& interval : intervals) {
    interval.start_m = std::clamp(interval.start_m, 0.0, total_length_m);
    interval.end_m = std::clamp(interval.end_m, 0.0, total_length_m);
    if (interval.end_m < interval.start_m) {
      std::swap(interval.start_m, interval.end_m);
    }
  }
  std::sort(intervals.begin(), intervals.end(),
            [](const CurveLengthInterval& a, const CurveLengthInterval& b) { return a.start_m < b.start_m; });
  for (const CurveLengthInterval& interval : intervals) {
    if (interval.end_m - interval.start_m <= kZeroLengthEps) {
      continue;
    }
    if (merged.empty() || interval.start_m > merged.back().end_m + 1e-6) {
      merged.push_back(interval);
    } else {
      merged.back().end_m = std::max(merged.back().end_m, interval.end_m);
    }
  }
  return merged;
}

std::vector<CurveLengthInterval> visible_intervals_from_hidden(const std::vector<CurveLengthInterval>& hidden_intervals,
                                                               double total_length_m) {
  std::vector<CurveLengthInterval> visible{};
  double cursor = 0.0;
  for (const CurveLengthInterval& interval : hidden_intervals) {
    if (interval.start_m > cursor + kZeroLengthEps) {
      visible.push_back({cursor, interval.start_m});
    }
    cursor = std::max(cursor, interval.end_m);
  }
  if (cursor < total_length_m - kZeroLengthEps) {
    visible.push_back({cursor, total_length_m});
  }
  if (visible.empty() && total_length_m > kZeroLengthEps && hidden_intervals.empty()) {
    visible.push_back({0.0, total_length_m});
  }
  return visible;
}

std::vector<Vec3d> build_attachment_replacement_points(const AttachmentInternalPathTemplate& path,
                                                       const AttachmentSocketTemplate& socket_a,
                                                       const AttachmentSocketTemplate& socket_b,
                                                       const Vec3d& origin, const Vec3d& forward,
                                                       const Vec3d& lateral, const Vec3d& up) {
  std::vector<Vec3d> points{};
  const auto push_local = [&](const Vec3d& local) {
    points.push_back(attachment_local_to_world(origin, forward, lateral, up, local));
  };
  switch (path.profile_kind) {
  case AttachmentInternalPathTemplate::ProfileKind::kExplicitPolyline:
    push_local(socket_a.local_position);
    for (const Vec3d& local_point : path.local_points) {
      push_local(local_point);
    }
    push_local(socket_b.local_position);
    break;
  case AttachmentInternalPathTemplate::ProfileKind::kStraightCable:
    push_local(socket_a.local_position);
    push_local(socket_b.local_position);
    break;
  case AttachmentInternalPathTemplate::ProfileKind::kCoiledCable: {
    const int sample_count = std::max(8, path.coil_turn_count * std::max(4, path.coil_samples_per_turn));
    points.reserve(static_cast<std::size_t>(sample_count + 1));
    for (int i = 0; i <= sample_count; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(sample_count);
      Vec3d local{
          socket_a.local_position.x + (socket_b.local_position.x - socket_a.local_position.x) * t,
          socket_a.local_position.y + (socket_b.local_position.y - socket_a.local_position.y) * t,
          socket_a.local_position.z + (socket_b.local_position.z - socket_a.local_position.z) * t,
      };
      const double envelope = std::sin(kPi * t);
      const double phase = kTwoPi * static_cast<double>(path.coil_turn_count) * t;
      local.y += std::cos(phase) * path.coil_radius_m * envelope;
      local.z += std::sin(phase) * path.coil_radius_m * envelope;
      push_local(local);
    }
    break;
  }
  default:
    break;
  }
  return points;
}

std::vector<Vec3d> build_cable_supplemental_points(const CableSupplementalPathTemplate& path,
                                                   const CoreState& state, const Span& span, const DetailCurve& curve) {
  std::vector<Vec3d> points{};
  if (curve.Length() <= kZeroLengthEps || path.profile_kind == CableSupplementalPathTemplate::ProfileKind::kNone) {
    return points;
  }

  if (path.anchor_mode == CableSupplementalPathTemplate::AnchorMode::kPoleBandChord) {
    if (path.pole_band_id == 0) {
      return points;
    }
    const Port* port_a = state.view().edit_state().ports.find(span.port_a_id);
    const Port* port_b = state.view().edit_state().ports.find(span.port_b_id);
    if (port_a == nullptr || port_b == nullptr || port_a->owner_pole_id == kInvalidObjectId ||
        port_b->owner_pole_id == kInvalidObjectId) {
      return points;
    }
    const Pole* pole_a = state.view().edit_state().poles.find(port_a->owner_pole_id);
    const Pole* pole_b = state.view().edit_state().poles.find(port_b->owner_pole_id);
    if (pole_a == nullptr || pole_b == nullptr) {
      return points;
    }
    const auto pole_type_it_a = state.view().pole_types().find(pole_a->pole_type_id);
    const auto pole_type_it_b = state.view().pole_types().find(pole_b->pole_type_id);
    if (pole_type_it_a == state.view().pole_types().end() || pole_type_it_b == state.view().pole_types().end()) {
      return points;
    }
    const PoleTypeDefinition& pole_type_a = pole_type_it_a->second;
    const PoleTypeDefinition& pole_type_b = pole_type_it_b->second;
    const PortPlacementBand* band_a = nullptr;
    const PortPlacementBand* band_b = nullptr;
    for (const PortPlacementBand& band : pole_type_a.port_bands) {
      if (band.band_id == path.pole_band_id) {
        band_a = &band;
        break;
      }
    }
    for (const PortPlacementBand& band : pole_type_b.port_bands) {
      if (band.band_id == path.pole_band_id) {
        band_b = &band;
        break;
      }
    }
    if (band_a == nullptr || band_b == nullptr) {
      return points;
    }

    double layout_yaw_a = pole_a->world_transform.rotation_euler_deg.z;
    if (const auto pole_view = state.view().inspect_pole(pole_a->id); pole_view.has_value() && pole_view->has_layout_yaw) {
      layout_yaw_a = pole_view->layout_yaw_deg;
    }
    double layout_yaw_b = pole_b->world_transform.rotation_euler_deg.z;
    if (const auto pole_view = state.view().inspect_pole(pole_b->id); pole_view.has_value() && pole_view->has_layout_yaw) {
      layout_yaw_b = pole_view->layout_yaw_deg;
    }
    const double lateral_a =
        pole_band_chord_lateral_m(state, span, true, *pole_a, layout_yaw_a, *port_a, band_a->lateral_center_m);
    const double lateral_b =
        pole_band_chord_lateral_m(state, span, false, *pole_b, layout_yaw_b, *port_b, band_b->lateral_center_m);
    const Vec3d local_a{0.0, lateral_a, band_a->height_center_m};
    const Vec3d local_b{0.0, lateral_b, band_b->height_center_m};
    const Vec3d world_a = LocalPointToWorld(BuildPoleFrame(pole_a->world_transform, layout_yaw_a), local_a);
    const Vec3d world_b = LocalPointToWorld(BuildPoleFrame(pole_b->world_transform, layout_yaw_b), local_b);
    points.push_back(world_a);
    points.push_back(world_b);
    return points;
  }

  const double trim_m = std::max(0.0, path.endpoint_trim_m);
  const double start_s = std::clamp(trim_m, 0.0, curve.Length());
  const double end_s = std::clamp(curve.Length() - trim_m, 0.0, curve.Length());
  if (end_s - start_s <= kZeroLengthEps) {
    return points;
  }
  const double visible_length_m = end_s - start_s;
  int sample_count = 2;
  if (path.profile_kind == CableSupplementalPathTemplate::ProfileKind::kCoiledCable) {
    sample_count =
        std::max(16, static_cast<int>(std::ceil(visible_length_m * path.coil_turns_per_meter *
                                                std::max(4, path.coil_samples_per_turn))));
  } else {
    sample_count = std::max(2, static_cast<int>(std::ceil(visible_length_m / 1.0)));
  }

  points.reserve(static_cast<std::size_t>(sample_count + 1));
  for (int i = 0; i <= sample_count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(sample_count);
    const double s = start_s + visible_length_m * t;
    const Vec3d base = curve.PositionAtLength(s);
    const Vec3d tangent = curve.EvaluateTangent(curve.LengthToU(s));
    Vec3d forward{};
    Vec3d lateral{};
    Vec3d up{};
    if (!build_attachment_frame(tangent, &forward, &lateral, &up)) {
      continue;
    }

    Vec3d point = base + ScaleVec(lateral, path.lateral_offset_m) + ScaleVec(up, path.vertical_offset_m);
    if (path.profile_kind == CableSupplementalPathTemplate::ProfileKind::kCoiledCable) {
      const double phase = kTwoPi * path.coil_turns_per_meter * (s - start_s);
      point = point + ScaleVec(lateral, std::cos(phase) * path.coil_radius_m) +
              ScaleVec(up, std::sin(phase) * path.coil_radius_m);
    }
    points.push_back(point);
  }

  return points;
}

} // namespace

void apply_attachment_line_effects_to_curve(const CoreState& state, ObjectId span_id, DetailCurve* curve) {
  if (curve == nullptr || curve->Length() <= kZeroLengthEps) {
    return;
  }

  std::vector<CurveLengthInterval> hidden{};
  std::vector<CurveLengthInterval> replaced{};
  std::vector<DetailReplacementPath> replacement_paths{};
  std::vector<DetailSupplementalPath> supplemental_paths{};

  const auto attachments_it = state.view().relation_index().attachments_by_span.find(span_id);
  if (attachments_it != state.view().relation_index().attachments_by_span.end()) {
    for (ObjectId attachment_id : attachments_it->second) {
      const Attachment* attachment = state.view().attachments().find(attachment_id);
      if (attachment == nullptr) {
        continue;
      }
      const AttachmentTemplate* attachment_template = state.find_attachment_template(attachment->template_id);
      if (attachment_template == nullptr ||
          attachment_template->line_interaction_mode == AttachmentLineInteractionMode::kPassThrough) {
        continue;
      }

      const double center_s = std::clamp(curve->Length() * attachment->t, 0.0, curve->Length());
      Vec3d origin = curve->PositionAtLength(center_s);
      OffsetAlongWorldUp(&origin, attachment->display_offset_m);
      const Vec3d tangent = curve->EvaluateTangent(curve->LengthToU(center_s));
      Vec3d forward{};
      Vec3d lateral{};
      Vec3d up{};
      if (!build_attachment_frame(tangent, &forward, &lateral, &up)) {
        continue;
      }

      const AttachmentSocketTemplate* socket_a = nullptr;
      const AttachmentSocketTemplate* socket_b = nullptr;
      const AttachmentInternalPathTemplate* internal_path = nullptr;
      if (!resolve_attachment_socket_pair(*attachment_template, &socket_a, &socket_b, &internal_path) ||
          socket_a == nullptr || socket_b == nullptr) {
        continue;
      }

      const double start_s =
          std::clamp(center_s + std::min(socket_a->local_position.x, socket_b->local_position.x), 0.0, curve->Length());
      const double end_s =
          std::clamp(center_s + std::max(socket_a->local_position.x, socket_b->local_position.x), 0.0, curve->Length());
      if (end_s - start_s <= kZeroLengthEps) {
        continue;
      }

      if (attachment_template->line_interaction_mode == AttachmentLineInteractionMode::kHideSegment) {
        hidden.push_back({start_s, end_s});
        continue;
      }

      if ((attachment_template->line_interaction_mode != AttachmentLineInteractionMode::kReplaceWithInternalPath &&
           attachment_template->line_interaction_mode != AttachmentLineInteractionMode::kAddInternalPath) ||
          internal_path == nullptr) {
        continue;
      }

      const std::vector<Vec3d> path_points =
          build_attachment_replacement_points(*internal_path, *socket_a, *socket_b, origin, forward, lateral, up);
      if (path_points.size() < 2) {
        continue;
      }

      if (attachment_template->line_interaction_mode == AttachmentLineInteractionMode::kReplaceWithInternalPath) {
        hidden.push_back({start_s, end_s});
        DetailReplacementPath replacement{};
        replacement.attachment_id = attachment->id;
        replacement.attachment_template_id = attachment_template->id;
        replacement.interaction_mode = attachment_template->line_interaction_mode;
        replacement.replaced_interval = {start_s, end_s};
        replacement.points = path_points;
        replaced.push_back(replacement.replaced_interval);
        replacement_paths.push_back(std::move(replacement));
      } else {
        DetailSupplementalPath supplemental{};
        supplemental.attachment_id = attachment->id;
        supplemental.attachment_template_id = attachment_template->id;
        supplemental.interaction_mode = attachment_template->line_interaction_mode;
        supplemental.points = path_points;
        supplemental_paths.push_back(std::move(supplemental));
      }
    }
  }

  curve->hidden_intervals = merged_intervals(std::move(hidden), curve->Length());
  curve->replacement_intervals = merged_intervals(std::move(replaced), curve->Length());
  curve->visible_intervals = visible_intervals_from_hidden(curve->hidden_intervals, curve->Length());
  if (curve->visible_intervals.empty() && curve->Length() > kZeroLengthEps) {
    curve->visible_intervals.push_back({0.0, curve->Length()});
  }

  const Span* span = state.view().edit_state().spans.find(span_id);
  if (span != nullptr) {
    const Bundle* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    const auto it_bundle_template =
        (bundle == nullptr) ? state.view().bundle_templates().end() : state.view().bundle_templates().find(bundle->bundle_template_id);
    const BundleTemplate* bundle_template =
        (it_bundle_template == state.view().bundle_templates().end()) ? nullptr : &it_bundle_template->second;
    const auto it_cable_template =
        (bundle_template == nullptr) ? state.view().cable_templates().end()
                                     : state.view().cable_templates().find(bundle_template->cable_template_id);
    const CableTemplate* cable_template =
        (it_cable_template == state.view().cable_templates().end()) ? nullptr : &it_cable_template->second;
    if (cable_template != nullptr) {
      for (const CableSupplementalPathTemplate& path : cable_template->supplemental_paths) {
        if (path.profile_kind == CableSupplementalPathTemplate::ProfileKind::kNone) {
          continue;
        }
        const std::vector<Vec3d> path_points = build_cable_supplemental_points(path, state, *span, *curve);
        if (path_points.size() >= 2) {
          DetailSupplementalPath supplemental{};
          supplemental.attachment_template_id = kInvalidAttachmentTemplateId;
          supplemental.interaction_mode = AttachmentLineInteractionMode::kAddInternalPath;
          supplemental.points = path_points;
          supplemental_paths.push_back(std::move(supplemental));
        }
      }
    }
  }

  curve->replacement_paths = std::move(replacement_paths);
  curve->supplemental_paths = std::move(supplemental_paths);
}

} // namespace wire::core
