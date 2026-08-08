// Cross section templates. Core validates and stores what it is given; which
// sections a product offers lives in web/src/road_templates.ts.
#include "city/road/road.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/section.hpp"
#include "../lookup.hpp"
#include "operation_plan.hpp"
#include "../geometry/alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <unordered_set>

namespace city::road {
namespace {

using internal::add;
using internal::align_first_span_start;
using internal::align_last_span_end;
using internal::distance;
using internal::find_node;
using internal::find_segment;
using internal::find_template;
using internal::find_transition;
using internal::is_finite;
using internal::distance_epsilon;
using internal::magnitude;
using internal::make_linear_shape;
using internal::manual_area_distance_bounds;
using internal::manual_line_distance_bounds;
using internal::path_end;
using internal::path_start;
using internal::PathSplit;
using internal::scale;
using internal::shift_manual_line_distance;
using internal::split_path_at_distance;
using internal::subtract;

[[nodiscard]] const RoadLayoutStrip* find_strip(const RoadLayoutTemplate& section, RoadLayoutStripId id) {
  const auto it = std::find_if(section.strips.begin(), section.strips.end(),
                               [id](const RoadLayoutStrip& strip) { return strip.id == id; });
  return it == section.strips.end() ? nullptr : &*it;
}

[[nodiscard]] bool valid_marking_role(MarkingRole role) {
  return static_cast<int>(role) >= 0 && static_cast<int>(role) <= 5;
}

[[nodiscard]] Result<bool> validate_marking_policy(const AutoMarkingPolicy& policy) {
  if (!valid_marking_role(policy.role) ||
      (policy.enabled && !IsKnownMarkingStyle(policy.style_id))) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "marking policy is invalid");
  }
  if (policy.enabled && policy.placement == MarkingPlacement::kUnspecified) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "marking policy does not say where the line sits");
  }
  return Result<bool>::Ok(true);
}





[[nodiscard]] Result<bool> validate_layout_template(const RoadLayoutTemplate& section) {
  if (section.strips.empty() || section.boundaries.size() + 1 != section.strips.size()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template chain is incomplete");
  }
  std::unordered_set<RoadLayoutStripId> ids{};
  for (const RoadLayoutStrip& strip : section.strips) {
    if (strip.id == 0 || !ids.insert(strip.id).second || !is_finite(strip.width_m) ||
        !is_finite(strip.cross_slope) || strip.width_m <= 0.0 ||
        static_cast<int>(strip.function) < 0 || static_cast<int>(strip.function) > 3) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template strip is invalid");
    }
    if (!IsKnownSurfaceStyle(strip.style_id)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template surface style is unknown");
    }
    for (const AutoMarkingPolicy& side : {strip.side_marking.left, strip.side_marking.right}) {
      if (!validate_marking_policy(side).ok) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template lane side marking policy is invalid");
      }
      // A lane side line belongs to a lane, so only a carriageway strip can ask
      // for one.
      if (side.enabled && strip.function != StripFunction::kCarriageway) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "lane side marking requires a carriageway strip");
      }
    }
  }
  std::unordered_set<LaneId> lane_ids{};
  for (const LaneBand& lane : section.lane_bands) {
    const RoadLayoutStrip* strip = find_strip(section, lane.surface_strip_id);
    if (lane.id == 0 || !lane_ids.insert(lane.id).second || strip == nullptr ||
        strip->function != StripFunction::kCarriageway ||
        !is_finite(lane.lateral_start_m) || !is_finite(lane.lateral_end_m) ||
        lane.lateral_start_m < 0.0 ||
        lane.lateral_end_m <= lane.lateral_start_m ||
        lane.lateral_end_m > strip->width_m) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section template lane allocation is invalid");
    }
  }
  double total_width = 0.0;
  for (const RoadLayoutStrip& strip : section.strips) total_width += strip.width_m;
  ids.clear();
  for (std::size_t index = 0; index < section.boundaries.size(); ++index) {
    const BoundaryProfile& boundary = section.boundaries[index];
    if (boundary.boundary_id == 0 || !ids.insert(boundary.boundary_id).second ||
        boundary.contour.empty() ||
        boundary.segment_styles.size() + 1 != boundary.contour.size()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template boundary is invalid");
    }
    double previous_lateral = -std::numeric_limits<double>::infinity();
    for (const ProfilePoint& point : boundary.contour) {
      if (!is_finite(point.lateral_m) || !is_finite(point.height_m) ||
          point.lateral_m < previous_lateral) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template boundary profile is invalid");
      }
      previous_lateral = point.lateral_m;
    }
    for (const SurfaceStyleId& style : boundary.segment_styles) {
      if (!IsKnownSurfaceStyle(style)) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template boundary surface style is unknown");
      }
    }
    // A profile reaching past the strip beside it would fold the section back on
    // itself.
    const double reach_in = -boundary.contour.front().lateral_m;
    const double reach_out = boundary.contour.back().lateral_m;
    if (reach_in < -distance_epsilon || reach_out < -distance_epsilon ||
        reach_in > section.strips[index].width_m + distance_epsilon ||
        reach_out > section.strips[index + 1].width_m + distance_epsilon) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInvalidInput,
          "section template boundary profile does not fit its neighbouring strips");
    }
    if (!validate_marking_policy(boundary.marking).ok) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section template marking policy is invalid");
    }
  }
  // A layout that never set the offset arrives here non-finite, so no caller
  // gets a silently centred road.
  if (!is_finite(section.alignment_offset_from_left_m) ||
      section.alignment_offset_from_left_m < -distance_epsilon ||
      section.alignment_offset_from_left_m > total_width + distance_epsilon) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "section template alignment offset is outside the layout width");
  }
  return Result<bool>::Ok(true);
}
} // namespace

Result<RoadLayoutTemplateId> RoadState::AddRoadLayoutTemplate(AddRoadLayoutTemplateRequest request) {
  RoadLayoutTemplate layout_template = std::move(request.layout_template);
  const Result<bool> valid = validate_layout_template(layout_template);
  if (!valid.ok) {
    return Result<RoadLayoutTemplateId>::Fail(valid.failure_category, valid.error);
  }
  if (layout_template.id != 0 && find_template(graph_, layout_template.id) != nullptr) {
    return Result<RoadLayoutTemplateId>::Fail(CommitFailureCategory::kInvalidInput, "section template id already exists");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (layout_template.id == 0) layout_template.id = next_id++;
  const RoadLayoutTemplateId id = layout_template.id;
  plan.next_id_after = next_id;
  plan.add_layout_templates.push_back(std::move(layout_template));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadLayoutTemplateId>::Fail(executed.failure_category, executed.error);
  return Result<RoadLayoutTemplateId>::Ok(id);
}

Result<bool> RoadState::EditRoadLayoutTemplate(EditRoadLayoutTemplateRequest request) {
  RoadLayoutTemplate layout_template = std::move(request.layout_template);
  const Result<bool> valid = validate_layout_template(layout_template);
  if (!valid.ok) return valid;
  auto it = std::find_if(graph_.layout_templates.begin(), graph_.layout_templates.end(),
                         [&layout_template](const RoadLayoutTemplate& item) { return item.id == layout_template.id; });
  if (it == graph_.layout_templates.end()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_layout_templates.push_back(std::move(layout_template));
  return Execute(plan);
}

} // namespace city::road
