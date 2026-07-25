#include "model_placement_rules.hpp"

#include "wire/core/coord_utils.hpp"
#include "wire/core/support/numeric_tolerances.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::backbone {
namespace {

constexpr double kPi = 3.14159265358979323846;

double yaw_deg_from_xy(const Vec3d& direction) {
  return std::atan2(direction.y, direction.x) * (180.0 / kPi);
}

} // namespace

std::vector<ModelPlacementRule> placement_rules_from_bundle_template(const BundleTemplate& bundle_template) {
  std::vector<ModelPlacementRule> out{};
  if (bundle_template.row_fixture_assembly_id != kInvalidModelAssemblyTemplateId) {
    ModelPlacementRule rule{};
    rule.kind = ModelPlacementRuleKind::kAtRow;
    rule.assembly_id = bundle_template.row_fixture_assembly_id;
    out.push_back(rule);
  }
  if (bundle_template.endpoint_fixture_assembly_id != kInvalidModelAssemblyTemplateId) {
    ModelPlacementRule rule{};
    rule.kind = ModelPlacementRuleKind::kAtEndpoint;
    rule.assembly_id = bundle_template.endpoint_fixture_assembly_id;
    out.push_back(rule);
  }
  return out;
}

std::vector<SpanAnchorFrame> interval_anchor_frames(const Vec3d& start,
                                                    const Vec3d& end,
                                                    double spacing_m,
                                                    double phase_m,
                                                    ModelPlacementOrientationPolicy orientation) {
  std::vector<SpanAnchorFrame> out{};
  const Vec3d delta = end - start;
  const double length = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  if (!std::isfinite(length) || length <= kLengthToleranceM || !std::isfinite(spacing_m) || spacing_m <= kLengthToleranceM ||
      !std::isfinite(phase_m)) {
    return out;
  }
  const double first = phase_m < 0.0 ? std::fmod(phase_m, spacing_m) + spacing_m : std::fmod(phase_m, spacing_m);
  for (double distance = first; distance <= length + kLengthToleranceM; distance += spacing_m) {
    SpanAnchorFrame frame{};
    frame.t = std::clamp(distance / length, 0.0, 1.0);
    frame.transform.position = start + ScaleVec(delta, frame.t);
    if (orientation == ModelPlacementOrientationPolicy::kAlignTangent) {
      frame.transform.rotation_euler_deg.z = yaw_deg_from_xy(delta);
    }
    out.push_back(frame);
  }
  return out;
}

} // namespace wire::core::generation::backbone
