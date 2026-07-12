#include "span_visual_assembly.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace wire::core::generation::backbone {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

Vec3d unit_or(const Vec3d& value, const Vec3d& fallback) {
  const double length = Length(value);
  return length > 1e-9 ? ScaleVec(value, 1.0 / length) : fallback;
}

AABBd bounds_for(const std::vector<Vec3d>& points) {
  AABBd bounds{};
  if (points.empty()) return bounds;
  bounds.min = points.front();
  bounds.max = points.front();
  for (const Vec3d& point : points) {
    bounds.min.x = std::min(bounds.min.x, point.x); bounds.min.y = std::min(bounds.min.y, point.y); bounds.min.z = std::min(bounds.min.z, point.z);
    bounds.max.x = std::max(bounds.max.x, point.x); bounds.max.y = std::max(bounds.max.y, point.y); bounds.max.z = std::max(bounds.max.z, point.z);
  }
  return bounds;
}

void apply_member_twist(const SpanVisualAssemblyTemplate& settings, std::vector<VisualCurvePart*> members) {
  if (settings.member_twist_turns_per_meter == 0.0 || members.size() < 2) {
    return;
  }
  std::size_t sample_count = members.front()->samples.size();
  for (const VisualCurvePart* member : members) {
    sample_count = std::min(sample_count, member->samples.size());
  }
  if (sample_count < 2) {
    return;
  }
  double distance = 0.0;
  for (std::size_t index = 0; index < sample_count; ++index) {
    if (index > 0) {
      distance += Length(members.front()->samples[index] - members.front()->samples[index - 1]);
    }
    Vec3d centroid{};
    for (const VisualCurvePart* member : members) {
      centroid = centroid + member->samples[index];
    }
    centroid = ScaleVec(centroid, 1.0 / static_cast<double>(members.size()));
    const Vec3d tangent = index + 1 < sample_count
        ? unit_or(members.front()->samples[index + 1] - members.front()->samples[index], {1.0, 0.0, 0.0})
        : unit_or(members.front()->samples[index] - members.front()->samples[index - 1], {1.0, 0.0, 0.0});
    Vec3d lateral = Cross({0.0, 0.0, 1.0}, tangent);
    lateral = unit_or(lateral, {0.0, 1.0, 0.0});
    const Vec3d up = unit_or(Cross(tangent, lateral), {0.0, 0.0, 1.0});
    const double angle = settings.member_twist_phase + kTwoPi * settings.member_twist_turns_per_meter * distance;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (VisualCurvePart* member : members) {
      const Vec3d offset = member->samples[index] - centroid;
      const double x = Dot(offset, lateral);
      const double y = Dot(offset, up);
      const Vec3d axial = ScaleVec(tangent, Dot(offset, tangent));
      member->samples[index] = centroid + axial + ScaleVec(lateral, x * cosine - y * sine) +
                               ScaleVec(up, x * sine + y * cosine);
    }
  }
  for (VisualCurvePart* member : members) {
    member->bounds = bounds_for(member->samples);
  }
}

} // namespace

void apply_span_visual_assemblies(const CoreState& state, VisualCurvePartCache* cache) {
  if (cache == nullptr) {
    return;
  }
  std::unordered_map<ObjectId, std::vector<VisualCurvePart*>> members_by_span{};
  for (VisualCurvePart& part : cache->parts) {
    if (part.kind == VisualCurvePartKind::kEdgeBody && part.has_section_key) {
      members_by_span[part.section_key.logical_span_id].push_back(&part);
    }
  }
  for (auto& [logical_span_id, members] : members_by_span) {
    const Span* span = state.view().spans().find(logical_span_id);
    const Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    const auto template_it = bundle == nullptr ? state.view().bundle_templates().end() :
        state.view().bundle_templates().find(bundle->bundle_template_id);
    if (template_it == state.view().bundle_templates().end()) {
      continue;
    }
    apply_member_twist(template_it->second.span_visual_assembly, std::move(members));
  }
}

} // namespace wire::core::generation::backbone
