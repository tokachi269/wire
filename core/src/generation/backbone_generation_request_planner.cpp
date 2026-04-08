#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "backbone_generation_plan_internal.hpp"

#include <algorithm>

namespace wire::core::generation::detail {

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
