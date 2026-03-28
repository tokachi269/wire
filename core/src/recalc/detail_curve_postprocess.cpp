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

} // namespace

void apply_attachment_line_effects_to_curve(const CoreState& state, ObjectId span_id, DetailCurve* curve) {
  if (curve == nullptr || curve->Length() <= kZeroLengthEps) {
    return;
  }
  const auto attachments_it = state.view().relation_index().attachments_by_span.find(span_id);
  if (attachments_it == state.view().relation_index().attachments_by_span.end()) {
    return;
  }

  std::vector<CurveLengthInterval> hidden{};
  std::vector<CurveLengthInterval> replaced{};
  std::vector<DetailReplacementPath> replacement_paths{};

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

    const double start_s = std::clamp(center_s + std::min(socket_a->local_position.x, socket_b->local_position.x), 0.0,
                                      curve->Length());
    const double end_s = std::clamp(center_s + std::max(socket_a->local_position.x, socket_b->local_position.x), 0.0,
                                    curve->Length());
    if (end_s - start_s <= kZeroLengthEps) {
      continue;
    }

    hidden.push_back({start_s, end_s});

    if (attachment_template->line_interaction_mode != AttachmentLineInteractionMode::kReplaceWithInternalPath ||
        internal_path == nullptr) {
      continue;
    }

    DetailReplacementPath replacement{};
    replacement.attachment_id = attachment->id;
    replacement.attachment_template_id = attachment_template->id;
    replacement.interaction_mode = attachment_template->line_interaction_mode;
    replacement.replaced_interval = {start_s, end_s};
    replacement.points =
        build_attachment_replacement_points(*internal_path, *socket_a, *socket_b, origin, forward, lateral, up);
    if (replacement.points.size() >= 2) {
      replaced.push_back(replacement.replaced_interval);
      replacement_paths.push_back(std::move(replacement));
    }
  }

  curve->hidden_intervals = merged_intervals(std::move(hidden), curve->Length());
  curve->replacement_intervals = merged_intervals(std::move(replaced), curve->Length());
  curve->visible_intervals = visible_intervals_from_hidden(curve->hidden_intervals, curve->Length());
  if (curve->visible_intervals.empty() && curve->Length() > kZeroLengthEps) {
    curve->visible_intervals.push_back({0.0, curve->Length()});
  }
  curve->replacement_paths = std::move(replacement_paths);
}

} // namespace wire::core
