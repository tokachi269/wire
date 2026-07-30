#include "section.hpp"

#include "../lookup.hpp"
#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace city::road::internal {
namespace {

const SectionStrip *find_strip(const CrossSectionTemplate &section,
                               SectionStripId id) {
  const auto found = std::find_if(
      section.strips.begin(), section.strips.end(),
      [id](const SectionStrip &strip) { return strip.id == id; });
  return found == section.strips.end() ? nullptr : &*found;
}

const BoundaryProfile *find_boundary(const CrossSectionTemplate &section,
                                     std::uint64_t id) {
  const auto found =
      std::find_if(section.boundaries.begin(), section.boundaries.end(),
                   [id](const BoundaryProfile &boundary) {
                     return boundary.boundary_id == id;
                   });
  return found == section.boundaries.end() ? nullptr : &*found;
}

double station_value(StationRef ref, double total_m) {
  if (ref.kind == StationRefKind::kFromEnd)
    return total_m - ref.value;
  if (ref.kind == StationRefKind::kRatio)
    return total_m * ref.value;
  return ref.value;
}

CrossSectionTemplate interpolate_section(const CrossSectionTemplate &from,
                                         const CrossSectionTemplate &to,
                                         double t) {
  const CrossSectionTemplate &structure =
      to.strips.size() >= from.strips.size() ? to : from;
  CrossSectionTemplate out{};
  out.id = t < 1.0 ? from.id : to.id;
  for (const SectionStrip &structure_strip : structure.strips) {
    const SectionStrip *a = find_strip(from, structure_strip.id);
    const SectionStrip *b = find_strip(to, structure_strip.id);
    SectionStrip strip = b != nullptr ? *b : *a;
    const double a_width = a != nullptr ? a->width_m : 0.0;
    const double b_width = b != nullptr ? b->width_m : 0.0;
    const double a_slope =
        a != nullptr ? a->cross_slope : (b != nullptr ? b->cross_slope : 0.0);
    const double b_slope = b != nullptr ? b->cross_slope : a_slope;
    strip.width_m = a_width + (b_width - a_width) * t;
    strip.cross_slope = a_slope + (b_slope - a_slope) * t;
    out.strips.push_back(std::move(strip));
  }
  out.lane_bands = t < 1.0 ? from.lane_bands : to.lane_bands;
  for (const BoundaryProfile &structure_boundary : structure.boundaries) {
    const BoundaryProfile *a =
        find_boundary(from, structure_boundary.boundary_id);
    const BoundaryProfile *b =
        find_boundary(to, structure_boundary.boundary_id);
    BoundaryProfile boundary = b != nullptr ? *b : *a;
    const double a_width = a != nullptr ? a->width_m : 0.0;
    const double b_width = b != nullptr ? b->width_m : 0.0;
    const double a_height = a != nullptr ? a->height_m : 0.0;
    const double b_height = b != nullptr ? b->height_m : 0.0;
    boundary.width_m = a_width + (b_width - a_width) * t;
    boundary.height_m = a_height + (b_height - a_height) * t;
    out.boundaries.push_back(std::move(boundary));
  }
  return out;
}

// Merge the marking requests that apply to a boundary into one policy. The
// only sources are the boundary itself and the side policies of the bands
// adjacent in element order; never lateral position or array index.
Result<std::vector<AutoMarkingPolicy>>
merge_boundary_policies(const CrossSectionTemplate &section) {
  std::vector<AutoMarkingPolicy> merged(section.boundaries.size(),
                                        AutoMarkingPolicy{});
  if (section.strips.empty()) {
    return Result<std::vector<AutoMarkingPolicy>>::Ok(std::move(merged));
  }
  // The outermost sides have no adjacent boundary element to resolve to.
  if (section.strips.front().side_marking.left.enabled ||
      section.strips.back().side_marking.right.enabled) {
    return Result<std::vector<AutoMarkingPolicy>>::Fail(
        ErrorKind::kUnsupported,
        "lane side marking has no adjacent boundary element");
  }
  for (std::size_t index = 0; index < section.boundaries.size(); ++index) {
    const BoundaryProfile &boundary = section.boundaries[index];
    AutoMarkingPolicy resolved{};
    bool has_request = false;
    const auto accept = [&](const AutoMarkingPolicy &request) {
      if (!request.enabled) {
        return true;
      }
      if (!has_request) {
        resolved = request;
        has_request = true;
        return true;
      }
      return resolved == request;
    };
    const bool compatible =
        accept(boundary.marking) &&
        accept(section.strips[index].side_marking.right) &&
        accept(index + 1 < section.strips.size()
                   ? section.strips[index + 1].side_marking.left
                   : AutoMarkingPolicy{});
    if (!compatible) {
      return Result<std::vector<AutoMarkingPolicy>>::Fail(
          ErrorKind::kUnsupported,
          "conflicting marking requests on section boundary " +
              std::to_string(boundary.boundary_id));
    }
    merged[index] = resolved;
  }
  return Result<std::vector<AutoMarkingPolicy>>::Ok(std::move(merged));
}

std::vector<SectionBoundarySample>
build_boundaries(const CrossSectionTemplate &section,
                 const std::vector<AutoMarkingPolicy> &policies) {
  std::vector<SectionBoundarySample> samples{};
  if (section.strips.empty())
    return samples;
  double total_width = 0.0;
  for (const SectionStrip &strip : section.strips)
    total_width += strip.width_m;
  for (const BoundaryProfile &boundary : section.boundaries) {
    total_width += boundary.width_m;
  }
  double lateral = -total_width * 0.5;
  double height = 0.0;
  double carriageway_floor = std::numeric_limits<double>::infinity();
  samples.push_back(
      SectionBoundarySample{1, BoundaryRole::kOuterEdge, lateral, height, {}});
  for (std::size_t index = 0; index < section.strips.size(); ++index) {
    const SectionStrip &strip = section.strips[index];
    const double strip_end_height = height + strip.cross_slope * strip.width_m;
    if (strip.function == StripFunction::kCarriageway) {
      carriageway_floor =
          std::min(carriageway_floor, std::min(height, strip_end_height));
    }
    lateral += strip.width_m;
    height = strip_end_height;
    if (index >= section.boundaries.size())
      continue;
    const BoundaryProfile &boundary = section.boundaries[index];
    const AutoMarkingPolicy policy =
        index < policies.size() ? policies[index] : boundary.marking;
    const SectionStrip &right_strip = section.strips[index + 1];
    const auto with_adjacency = [&](SectionBoundarySample sample) {
      sample.left_strip_id = strip.id;
      sample.right_strip_id = right_strip.id;
      sample.left_strip_width_m = strip.width_m;
      sample.right_strip_width_m = right_strip.width_m;
      return sample;
    };
    const bool structural_boundary = boundary.role == BoundaryRole::kCurb ||
                                     boundary.role == BoundaryRole::kMedianEdge;
    if (!structural_boundary && boundary.width_m <= station_epsilon &&
        std::abs(boundary.height_m) <= station_epsilon) {
      samples.push_back(with_adjacency(SectionBoundarySample{
          boundary.boundary_id, boundary.role, lateral, height, policy}));
      continue;
    }
    AutoMarkingPolicy before_rule{};
    AutoMarkingPolicy after_rule = policy;
    if (boundary.role == BoundaryRole::kCurb &&
        strip.function == StripFunction::kCarriageway &&
        right_strip.function == StripFunction::kSidewalk) {
      before_rule = policy;
      after_rule = {};
    }
    samples.push_back(with_adjacency(SectionBoundarySample{
        boundary.boundary_id, boundary.role, lateral, height, before_rule}));
    lateral += boundary.width_m;
    height += boundary.height_m;
    samples.push_back(with_adjacency(SectionBoundarySample{
        boundary.boundary_id, boundary.role, lateral, height, after_rule}));
  }
  samples.push_back(SectionBoundarySample{
      999, BoundaryRole::kOuterEdge, lateral, height, {}});
  if (is_finite(carriageway_floor)) {
    for (SectionBoundarySample &sample : samples) {
      sample.height_m -= carriageway_floor;
    }
  }
  return samples;
}

SurfaceStyleId SurfaceStyleForBoundaryRole(BoundaryRole role) {
  if (role == BoundaryRole::kCurb)
    return builtin_surface_styles::kCurb;
  if (role == BoundaryRole::kMedianEdge)
    return builtin_surface_styles::kMedian;
  return builtin_surface_styles::kAsphalt;
}

std::vector<RenderStyleRef>
build_surface_styles(const CrossSectionTemplate &section) {
  std::vector<RenderStyleRef> styles{};
  for (std::size_t index = 0; index < section.strips.size(); ++index) {
    const SectionStrip &strip = section.strips[index];
    styles.push_back(RenderStyleFromSurface(strip.style_id));
    if (index >= section.boundaries.size())
      continue;
    const BoundaryProfile &boundary = section.boundaries[index];
    if (boundary.role == BoundaryRole::kCurb ||
        boundary.role == BoundaryRole::kMedianEdge ||
        boundary.width_m > station_epsilon ||
        std::abs(boundary.height_m) > station_epsilon) {
      styles.push_back(
          RenderStyleFromSurface(SurfaceStyleForBoundaryRole(boundary.role)));
    }
  }
  return styles;
}

} // namespace

Result<CrossSectionTemplate> template_at(const SavedRoadGraph &graph,
                                         const RoadSegment &segment,
                                         double station_m, double total_m) {
  const CrossSectionTemplate *base =
      find_template(graph, segment.section_template);
  if (base == nullptr) {
    return Result<CrossSectionTemplate>::Fail(
        ErrorKind::kValidation, "road segment section template is missing");
  }
  if (!segment.transition.has_value()) {
    return Result<CrossSectionTemplate>::Ok(*base);
  }
  const SectionTransition *transition =
      find_transition(graph, *segment.transition);
  if (transition == nullptr) {
    return Result<CrossSectionTemplate>::Fail(
        ErrorKind::kValidation, "road segment transition is missing");
  }
  const CrossSectionTemplate *from =
      find_template(graph, transition->from_template);
  const CrossSectionTemplate *to =
      find_template(graph, transition->to_template);
  if (from == nullptr || to == nullptr) {
    return Result<CrossSectionTemplate>::Fail(
        ErrorKind::kValidation, "road transition template is missing");
  }
  const double start = station_value(transition->start, total_m);
  const double end = station_value(transition->end, total_m);
  if (start < 0.0 || end > total_m || end - start <= station_epsilon) {
    return Result<CrossSectionTemplate>::Fail(
        ErrorKind::kValidation, "road transition station range is invalid");
  }
  return Result<CrossSectionTemplate>::Ok(interpolate_section(
      *from, *to, std::clamp((station_m - start) / (end - start), 0.0, 1.0)));
}

Result<SectionEvaluation> section_at(const SavedRoadGraph &graph,
                                     const RoadSegment &segment,
                                     double station_m, double total_m) {
  Result<CrossSectionTemplate> section =
      template_at(graph, segment, station_m, total_m);
  if (!section.ok) {
    return Result<SectionEvaluation>::Fail(section.error_kind, section.error);
  }
  Result<std::vector<AutoMarkingPolicy>> policies =
      merge_boundary_policies(section.value);
  if (!policies.ok) {
    return Result<SectionEvaluation>::Fail(policies.error_kind, policies.error);
  }
  std::vector<SectionBoundarySample> boundaries =
      build_boundaries(section.value, policies.value);
  if (boundaries.empty()) {
    return Result<SectionEvaluation>::Fail(
        ErrorKind::kInternal, "road section evaluation produced no boundaries");
  }
  if (segment.transition.has_value()) {
    const SectionTransition *transition =
        find_transition(graph, *segment.transition);
    const CrossSectionTemplate *from =
        transition == nullptr ? nullptr
                              : find_template(graph, transition->from_template);
    if (transition == nullptr || from == nullptr) {
      return Result<SectionEvaluation>::Fail(
          ErrorKind::kValidation, "road transition anchor source is missing");
    }
    const std::vector<SectionBoundarySample> from_boundaries = build_boundaries(
        *from, std::vector<AutoMarkingPolicy>(from->boundaries.size(),
                                              AutoMarkingPolicy{}));
    const double shift =
        transition->anchor == TransitionAnchor::kLeftEdge
            ? from_boundaries.front().lateral_m - boundaries.front().lateral_m
            : (transition->anchor == TransitionAnchor::kRightEdge
                   ? from_boundaries.back().lateral_m -
                         boundaries.back().lateral_m
                   : 0.0);
    for (SectionBoundarySample &boundary : boundaries) {
      boundary.lateral_m += shift;
    }
  }
  return Result<SectionEvaluation>::Ok(SectionEvaluation{
      segment.id, station_m, section.value.id, std::move(boundaries),
      build_surface_styles(section.value)});
}

} // namespace city::road::internal
