// Cross section templates. Core validates and stores what it is given; which
// sections a product offers lives in web/src/road_templates.ts.
#include "city/road/road.hpp"

#include "geometry/geometry.hpp"
#include "geometry/section.hpp"
#include "lookup.hpp"
#include "operations/operation_plan.hpp"
#include "road_path.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <unordered_set>

namespace city::road {
namespace {

using internal::add;
using internal::align_first_span_start;
using internal::align_last_span_end;
using internal::apply_inherited_arc;
using internal::corridor_terminal_handle;
using internal::distance;
using internal::find_node;
using internal::find_segment;
using internal::find_template;
using internal::find_transition;
using internal::is_finite;
using internal::kEpsilon;
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

[[nodiscard]] const SectionStrip* find_strip(const CrossSectionTemplate& section, SectionStripId id) {
  const auto it = std::find_if(section.strips.begin(), section.strips.end(),
                               [id](const SectionStrip& strip) { return strip.id == id; });
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
  return Result<bool>::Ok(true);
}





[[nodiscard]] Result<bool> validate_section_template(const CrossSectionTemplate& section) {
  if (section.strips.empty() || section.boundaries.size() + 1 != section.strips.size()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template chain is incomplete");
  }
  std::unordered_set<SectionStripId> ids{};
  for (const SectionStrip& strip : section.strips) {
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
    const SectionStrip* strip = find_strip(section, lane.surface_strip_id);
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
  ids.clear();
  for (const BoundaryProfile& boundary : section.boundaries) {
    if (boundary.boundary_id == 0 || !ids.insert(boundary.boundary_id).second || !is_finite(boundary.width_m) ||
        !is_finite(boundary.height_m) || boundary.width_m < 0.0) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template boundary is invalid");
    }
    if (static_cast<int>(boundary.marking.role) < 0 ||
        static_cast<int>(boundary.marking.role) > 5 ||
        (boundary.marking.enabled &&
         !IsKnownMarkingStyle(boundary.marking.style_id))) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section template marking policy is invalid");
    }
  }
  return Result<bool>::Ok(true);
}
} // namespace

Result<CrossSectionTemplateId> RoadState::AddSectionTemplate(AddSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) {
    return Result<CrossSectionTemplateId>::Fail(valid.failure_category, valid.error);
  }
  if (section_template.id != 0 && find_template(graph_, section_template.id) != nullptr) {
    return Result<CrossSectionTemplateId>::Fail(CommitFailureCategory::kInvalidInput, "section template id already exists");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (section_template.id == 0) section_template.id = next_id++;
  const CrossSectionTemplateId id = section_template.id;
  plan.next_id_after = next_id;
  plan.add_section_templates.push_back(std::move(section_template));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<CrossSectionTemplateId>::Fail(executed.failure_category, executed.error);
  return Result<CrossSectionTemplateId>::Ok(id);
}

Result<bool> RoadState::EditSectionTemplate(EditSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) return valid;
  auto it = std::find_if(graph_.section_templates.begin(), graph_.section_templates.end(),
                         [&section_template](const CrossSectionTemplate& item) { return item.id == section_template.id; });
  if (it == graph_.section_templates.end()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_section_templates.push_back(std::move(section_template));
  return Execute(plan);
}

} // namespace city::road
