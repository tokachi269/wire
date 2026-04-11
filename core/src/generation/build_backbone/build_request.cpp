#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "build_backbone_types.hpp"
#include "../detail_utils.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::detail {

namespace {

constexpr double kNearDuplicateSupportNodeDistanceM = 1.5;

bool is_supported_bundle_node_mode(BundleNodeMode mode) {
  return mode == BundleNodeMode::kNotPresent || mode == BundleNodeMode::kPassThrough;
}

void add_unique_candidate(std::vector<SupportNodeCandidate>* candidates, const SupportNodeCandidate& candidate) {
  if (candidates == nullptr) {
    return;
  }
  if (candidates->empty()) {
    candidates->push_back(candidate);
    return;
  }
  const Vec3d d = candidate.world - candidates->back().world;
  const double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
  const bool can_collapse_near_duplicate =
      candidates->back().mode == PlacementMode::kAuto && candidate.mode == PlacementMode::kAuto &&
      candidates->back().support_kind == SupportKind::kPole && candidate.support_kind == SupportKind::kPole &&
      !candidates->back().has_tangent_hint && !candidate.has_tangent_hint &&
      dist2 < (kNearDuplicateSupportNodeDistanceM * kNearDuplicateSupportNodeDistanceM);
  if (can_collapse_near_duplicate) {
    if (candidates->back().vertex_index == 0 && candidate.vertex_index > 0) {
      return;
    }
    candidates->back() = candidate;
    return;
  }
  if (dist2 > 1e-10) {
    candidates->push_back(candidate);
  }
}

} // namespace

bool build_backbone_node_maps(const BackboneSpec& request, NodeSpecByIndex* out_node_spec_by_index,
                              NodeBundleModeByPoint* out_node_bundle_mode_by_point, std::string* error) {
  if (out_node_spec_by_index == nullptr || out_node_bundle_mode_by_point == nullptr) {
    if (error != nullptr) {
      *error = "backbone node map output is null";
    }
    return false;
  }
  out_node_spec_by_index->clear();
  out_node_bundle_mode_by_point->clear();

  for (const BackboneInputSpec::NodeSpec& node_spec : request.path.node_specs) {
    if (node_spec.point_index >= request.path.polyline.size()) {
      if (error != nullptr) {
        *error = "node_specs point_index is out of path range";
      }
      return false;
    }
    (*out_node_spec_by_index)[node_spec.point_index] = node_spec;
  }

  for (const BackboneSpec::NodeBundleModeSpec& mode_spec : request.node_bundle_modes) {
    if (mode_spec.point_index >= request.path.polyline.size()) {
      if (error != nullptr) {
        *error = "node_bundle_modes point_index is out of path range";
      }
      return false;
    }
    if (!is_supported_bundle_node_mode(mode_spec.mode)) {
      if (error != nullptr) {
        *error = "unsupported bundle node mode";
      }
      return false;
    }
    (*out_node_bundle_mode_by_point)[mode_spec.point_index][mode_spec.bundle_template_id] = mode_spec.mode;
  }

  return true;
}

void build_backbone_guide_points(const BackboneSpec& request, std::vector<Vec3d>* out_guide_points,
                                 PathDirectionEvaluationDebug* out_direction_debug) {
  if (out_guide_points == nullptr || out_direction_debug == nullptr) {
    return;
  }

  *out_guide_points = request.path.polyline;
  out_direction_debug->requested_mode = request.direction_mode;
  out_direction_debug->chosen = PathDirectionChosen::kForward;
  if (request.direction_mode == PathDirectionMode::kReverse) {
    std::reverse(out_guide_points->begin(), out_guide_points->end());
    out_direction_debug->chosen = PathDirectionChosen::kReverse;
  }
}

bool build_backbone_candidates(const BackboneSpec& request, const std::vector<Vec3d>& guide_points,
                               const NodeSpecByIndex& node_spec_by_index,
                               std::vector<SupportNodeCandidate>* out_candidates, std::string* error) {
  if (out_candidates == nullptr) {
    if (error != nullptr) {
      *error = "candidate output is null";
    }
    return false;
  }
  out_candidates->clear();
  out_candidates->reserve(guide_points.size() * 2);

  auto outside_avoid = [&](const Vec3d& point) -> bool {
    if (request.constraints.avoid_radius_m <= 0.0 || request.constraints.avoid_points.empty()) {
      return true;
    }
    const double r2 = request.constraints.avoid_radius_m * request.constraints.avoid_radius_m;
    for (const Vec3d& avoid : request.constraints.avoid_points) {
      const Vec3d d = point - avoid;
      if ((d.x * d.x + d.y * d.y + d.z * d.z) <= r2) {
        return false;
      }
    }
    return true;
  };

  const bool pin_endpoints = request.pole_placement.pin_endpoints;
  const bool pin_vertices = request.pole_placement.pin_vertices;
  for (std::size_t i = 0; i + 1 < guide_points.size(); ++i) {
    const Vec3d a = guide_points[i];
    const Vec3d b = guide_points[i + 1];
    const Vec3d seg = b - a;
    const double seg_len = std::sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z);
    if (seg_len <= generation::detail::kZeroLengthEps) {
      continue;
    }
    Vec3d dir = seg;
    dir.x /= seg_len;
    dir.y /= seg_len;
    dir.z /= seg_len;

    const Vec3d side_dir = ComputeLateralAxis(dir);
    const Vec3d lateral{
        side_dir.x * request.constraints.lateral_offset_m,
        side_dir.y * request.constraints.lateral_offset_m,
        side_dir.z * request.constraints.lateral_offset_m,
    };

    SupportNodeCandidate start{};
    start.world = a;
    start.segment_index = i;
    start.vertex_index = static_cast<int>(i);
    start.t = 0.0;
    const bool is_start_endpoint = (i == 0);
    start.mode = (pin_vertices || (pin_endpoints && is_start_endpoint)) ? PlacementMode::kManual : PlacementMode::kAuto;
    if (const auto it = node_spec_by_index.find(i); it != node_spec_by_index.end()) {
      start.support_kind = it->second.support_kind;
      start.has_tangent_hint = it->second.has_tangent_hint;
      start.tangent_hint = it->second.tangent_hint;
    }
    add_unique_candidate(out_candidates, start);

    if (request.interval_m > 0.0) {
      const double step_m =
          std::max(0.5, std::max(request.interval_m * 0.25, request.constraints.avoid_radius_m * 0.5));
      for (double dist = request.interval_m; dist < seg_len - 1e-9; dist += request.interval_m) {
        const double t0 = std::clamp(dist / seg_len, 0.0, 1.0);
        Vec3d point{a.x + seg.x * t0 + lateral.x, a.y + seg.y * t0 + lateral.y, a.z + seg.z * t0 + lateral.z};
        bool placed = outside_avoid(point);
        if (!placed && request.constraints.avoid_radius_m > 0.0) {
          constexpr int kMaxTries = 8;
          for (int k = 1; k <= kMaxTries && !placed; ++k) {
            for (double sign : {1.0, -1.0}) {
              const double shifted_t = std::clamp((dist + sign * step_m * static_cast<double>(k)) / seg_len, 0.0, 1.0);
              if (shifted_t <= 1e-9 || shifted_t >= 1.0 - 1e-9) {
                continue;
              }
              Vec3d shifted{a.x + seg.x * shifted_t + lateral.x, a.y + seg.y * shifted_t + lateral.y,
                            a.z + seg.z * shifted_t + lateral.z};
              if (outside_avoid(shifted)) {
                point = shifted;
                placed = true;
                break;
              }
            }
          }
        }
        if (!placed) {
          continue;
        }
        SupportNodeCandidate auto_candidate{};
        auto_candidate.world = point;
        auto_candidate.segment_index = i;
        auto_candidate.vertex_index = -1;
        auto_candidate.t = t0;
        auto_candidate.mode = PlacementMode::kAuto;
        add_unique_candidate(out_candidates, auto_candidate);
      }
    }
  }

  SupportNodeCandidate end{};
  end.world = guide_points.back();
  end.segment_index = guide_points.size() - 2;
  end.vertex_index = static_cast<int>(guide_points.size() - 1);
  end.t = 1.0;
  end.mode = pin_endpoints ? PlacementMode::kManual : PlacementMode::kAuto;
  if (const auto it = node_spec_by_index.find(guide_points.size() - 1); it != node_spec_by_index.end()) {
    end.support_kind = it->second.support_kind;
    end.has_tangent_hint = it->second.has_tangent_hint;
    end.tangent_hint = it->second.tangent_hint;
  }
  add_unique_candidate(out_candidates, end);

  if (out_candidates->size() < 2) {
    if (error != nullptr) {
      *error = "failed to build guide candidates";
    }
    return false;
  }
  return true;
}

EditResult<BackboneGenerationRequestPlan> build_backbone_generation_request_plan(
    const CoreState& state, const BackboneSpec& spec) {
  EditResult<BackboneGenerationRequestPlan> result{};
  BackboneGenerationRequestPlan plan{};
  plan.request = spec;
  if (spec.path.polyline.size() < 2) {
    result.error = "backbone input path must contain at least 2 points";
    return result;
  }
  if (spec.interval_m <= 0.0) {
    result.error = "interval_m must be > 0";
    return result;
  }
  const CoreView core_view = state.view();
  if (core_view.pole_types().find(spec.pole_type_id) == core_view.pole_types().end()) {
    result.error = "pole type not found";
    return result;
  }
  if (!generation::detail::build_backbone_node_maps(spec, &plan.node_spec_by_index, &plan.node_bundle_mode_by_point,
                                                    &result.error)) {
    return result;
  }
  if (spec.bundles.empty()) {
    result.error = "bundles[] must contain at least one bundle request";
    return result;
  }

  plan.bundle_plans.reserve(spec.bundles.size());
  for (const BackboneBundleSpec& bundle_request : spec.bundles) {
    const auto bundle_template_it = core_view.bundle_templates().find(bundle_request.bundle_template_id);
    if (bundle_template_it == core_view.bundle_templates().end()) {
      result.error = "bundle template not found";
      return result;
    }
    const BundleTemplate* bundle_template = &bundle_template_it->second;
    BackboneBundlePlan bundle_plan{};
    bundle_plan.template_id = bundle_template->id;
    bundle_plan.category = bundle_template->category;
    bundle_plan.layer =
        (bundle_request.layer == SpanLayer::kUnknown) ? bundle_template->default_layer : bundle_request.layer;
    bundle_plan.spacing_m = bundle_template->default_spacing_m;
    bundle_plan.allow_mirror = bundle_template->allow_mirror;
    bundle_plan.preserve_conductor_identity = bundle_template->preserve_conductor_identity;
    bundle_plan.allow_midair_node = bundle_template->allow_midair_node;
    bundle_plan.allow_midair_branch = bundle_template->allow_midair_branch;
    bundle_plan.enable_branch_down_offset = bundle_template->enable_branch_down_offset;
    if (bundle_template->count_rule == BundleCountRuleKind::kFixed) {
      if (bundle_request.count > 0) {
        result.error = "count override is not allowed for fixed bundle template";
        return result;
      }
      bundle_plan.count = bundle_template->fixed_count;
    } else {
      bundle_plan.count = (bundle_request.count > 0) ? bundle_request.count : bundle_template->default_count;
      if (bundle_plan.count < bundle_template->min_count || bundle_plan.count > bundle_template->max_count) {
        result.error = "bundle count is out of template range";
        return result;
      }
    }
    if (bundle_plan.count <= 0) {
      result.error = "resolved bundle count must be > 0";
      return result;
    }
    bundle_plan.continuity_class =
        (bundle_plan.count > 1) ? ContinuityCategoryClass::kBundleLike : ContinuityCategoryClass::kPointLike;
    bundle_plan.order_decision_policy = bundle_template->order_decision_policy;
    if (bundle_plan.layer == SpanLayer::kUnknown) {
      result.error = "bundle layer could not be resolved";
      return result;
    }
    plan.bundle_plans.push_back(bundle_plan);
  }

  auto support_kind_for_point = [&](std::size_t point_index) -> SupportKind {
    const auto it = plan.node_spec_by_index.find(point_index);
    return (it == plan.node_spec_by_index.end()) ? SupportKind::kPole : it->second.support_kind;
  };
  for (std::size_t point_index = 0; point_index < spec.path.polyline.size(); ++point_index) {
    if (support_kind_for_point(point_index) == SupportKind::kPole) {
      continue;
    }
    plan.request_has_non_pole_points = true;
  }

  plan.active_bundle_plans.reserve(plan.bundle_plans.size());
  for (const BackboneBundlePlan& bundle_plan : plan.bundle_plans) {
    if (plan.request_has_non_pole_points && !bundle_plan.allow_midair_node) {
      continue;
    }
    if (plan.request_has_source_edge_branch_points && !bundle_plan.allow_midair_branch) {
      continue;
    }
    plan.active_bundle_plans.push_back(bundle_plan);
  }
  for (std::size_t point_index = 0; point_index < spec.path.polyline.size(); ++point_index) {
    if (support_kind_for_point(point_index) == SupportKind::kPole) {
      continue;
    }
    const auto it_modes = plan.node_bundle_mode_by_point.find(point_index);
    if (it_modes == plan.node_bundle_mode_by_point.end()) {
      continue;
    }
    for (const auto& [bundle_template_id, mode] : it_modes->second) {
      const auto it_plan = std::find_if(plan.bundle_plans.begin(), plan.bundle_plans.end(),
                                        [&](const BackboneBundlePlan& p) { return p.template_id == bundle_template_id; });
      if (it_plan == plan.bundle_plans.end()) {
        result.error = "node_bundle_modes references bundle template that is not selected";
        return result;
      }
      if (mode != BundleNodeMode::kNotPresent && mode != BundleNodeMode::kPassThrough) {
        result.error = "unsupported bundle node mode";
        return result;
      }
    }
  }

  generation::detail::build_backbone_guide_points(spec, &plan.guide_points, &plan.direction_debug);
  if (!plan.active_bundle_plans.empty() &&
      !generation::detail::build_backbone_candidates(spec, plan.guide_points, plan.node_spec_by_index, &plan.candidates,
                                                     &result.error)) {
    return result;
  }
  result.value = std::move(plan);
  result.ok = true;
  return result;
}

} // namespace wire::core::generation::detail
