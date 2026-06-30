#include "out.hpp"

#include "wire/core/core_view.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::bb2 {
namespace {

Vec3d mul(Vec3d v, double k) {
  return Vec3d{v.x * k, v.y * k, v.z * k};
}

double length(Vec3d v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

AABBd box(const Vec3d& a, const Vec3d& b) {
  AABBd out{};
  out.min = {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
  out.max = {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
  return out;
}

AABBd box(const std::vector<Vec3d>& pts) {
  AABBd out{};
  if (pts.empty()) {
    return out;
  }
  out.min = pts.front();
  out.max = pts.front();
  for (const Vec3d& p : pts) {
    out.min.x = std::min(out.min.x, p.x);
    out.min.y = std::min(out.min.y, p.y);
    out.min.z = std::min(out.min.z, p.z);
    out.max.x = std::max(out.max.x, p.x);
    out.max.y = std::max(out.max.y, p.y);
    out.max.z = std::max(out.max.z, p.z);
  }
  return out;
}

} // namespace

DetailCurve line(const Vec3d& a, const Vec3d& b) {
  const Vec3d d = b - a;
  const double l = length(d);
  DetailCurve out{};
  out.start_constraint.point = a;
  out.end_constraint.point = b;
  out.control_points = {a, a + mul(d, 1.0 / 3.0), a + mul(d, 2.0 / 3.0), b};
  DetailCurveSegment segment{};
  segment.control_points = out.control_points;
  segment.u_start = 0.0;
  segment.u_end = 1.0;
  out.segments.push_back(segment);
  out.sample_points = {a, b};
  out.arc_length_table = {{0.0, 0.0}, {1.0, l}};
  out.visible_intervals = {{0.0, l}};
  out.total_length_m = l;
  return out;
}

DetailCurve make_curve(const CoreState& state, ObjectId span_id, const SpanLayoutEntry& layout) {
  const GeometrySettings& settings = state.view().geometry_settings();
  if (!settings.sag_enabled) {
    return line(layout.start.endpoint_world, layout.end.endpoint_world);
  }

  double sag_ratio = settings.sag_factor;
  const Span* span = state.view().spans().find(span_id);
  if (span != nullptr) {
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle != nullptr) {
      const auto bundle_template = state.view().bundle_templates().find(bundle->bundle_template_id);
      if (bundle_template != state.view().bundle_templates().end()) {
        const auto cable = state.view().cable_templates().find(bundle_template->second.cable_template_id);
        if (cable != state.view().cable_templates().end()) {
          sag_ratio = cable->second.sag_factor + cable->second.slack_factor;
        }
      }
    }
  }
  sag_ratio = std::max(0.0, sag_ratio);
  if (sag_ratio <= 0.0) {
    return line(layout.start.endpoint_world, layout.end.endpoint_world);
  }

  const Vec3d chord = layout.end.endpoint_world - layout.start.endpoint_world;
  const double chord_length = length(chord);
  if (chord_length <= 1e-9) {
    return line(layout.start.endpoint_world, layout.end.endpoint_world);
  }
  const Vec3d tangent = mul(chord, 1.0 / chord_length);
  CurveConstraint start{};
  start.point = layout.start.endpoint_world;
  start.tangent_dir = tangent;
  start.sag_hint = sag_ratio * 0.5;
  start.continuity_preference = layout.continuity_preference;
  start.bend_stiffness_hint = layout.bend_stiffness_hint;
  start.min_bend_radius_hint_m = layout.min_bend_radius_hint_m;
  start.pass_mode = layout.pass_mode;
  start.endpoint_mode = layout.start.endpoint_mode;
  start.profile_hint = layout.detail_curve_profile_hint;

  CurveConstraint end = start;
  end.point = layout.end.endpoint_world;
  end.endpoint_mode = layout.end.endpoint_mode;
  return BuildDetailCurve(start, end, std::max(2, settings.curve_samples));
}

BoundsCacheEntry bounds(const DetailCurve& curve, std::uint64_t source_version) {
  BoundsCacheEntry out{};
  out.whole = box(curve.sample_points);
  if (curve.sample_points.size() >= 2) {
    out.segments.reserve(curve.sample_points.size() - 1);
    for (std::size_t i = 0; i + 1 < curve.sample_points.size(); ++i) {
      out.segments.push_back(box(curve.sample_points[i], curve.sample_points[i + 1]));
    }
  }
  out.source_version = source_version;
  return out;
}

SpanRenderCacheEntry render(const CoreState& state, ObjectId span_id, const DetailCurve& detail) {
  SpanRenderCacheEntry out{};
  if (const Span* span = state.view().spans().find(span_id); span != nullptr) {
    if (const Bundle* bundle = state.view().bundles().find(span->bundle_id); bundle != nullptr) {
      const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
      if (bundle_template_it != state.view().bundle_templates().end()) {
        const auto cable_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
        if (cable_it != state.view().cable_templates().end()) {
          out.wire_radius_m = std::max(0.0005, cable_it->second.outer_diameter_m * 0.5);
          out.color_rgba = cable_it->second.color_rgba;
          out.material_style = cable_it->second.material_style;
        }
      }
    }
  }
  out.arc_length_m_by_point.reserve(detail.sample_points.size());
  out.arc_length_normalized_by_point.reserve(detail.sample_points.size());
  out.segment_length_m.reserve(detail.sample_points.size() > 0 ? detail.sample_points.size() - 1 : 0);
  double total = 0.0;
  for (std::size_t i = 0; i < detail.sample_points.size(); ++i) {
    if (i > 0) {
      const double segment = length(detail.sample_points[i] - detail.sample_points[i - 1]);
      total += segment;
      out.segment_length_m.push_back(static_cast<float>(segment));
    }
    out.arc_length_m_by_point.push_back(static_cast<float>(total));
  }
  const double denom = total > 1e-9 ? total : 1.0;
  for (float value : out.arc_length_m_by_point) {
    out.arc_length_normalized_by_point.push_back(static_cast<float>(static_cast<double>(value) / denom));
  }
  return out;
}

SpanVisualCacheEntry visual(const VisualSettings& settings, const SpanLayoutEntry& layout) {
  SpanVisualCacheEntry out{};
  auto add_part = [&](const LayoutEndpoint& endpoint) {
    if (!settings.enable_support_structures) {
      return;
    }
    if (!endpoint.default_lower_required && !endpoint.lower_required) {
      return;
    }
    const Vec3d support = endpoint.support_world;
    if (length(endpoint.endpoint_world - support) <= 1e-9) {
      return;
    }
    VisualPart part{};
    part.kind = VisualPartKind::kSupportArm;
    part.a = support;
    part.b = endpoint.endpoint_world;
    part.radius_m = settings.support_arm_radius_m;
    out.parts.push_back(part);
  };
  add_part(layout.start);
  add_part(layout.end);
  return out;
}

} // namespace wire::core::generation::bb2
