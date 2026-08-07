#include "section.hpp"

#include "../lookup.hpp"
#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace city::road::internal {
namespace {

bool equivalent_strip(const RoadLayoutStrip &a, const RoadLayoutStrip &b) {
  return a.id == b.id && a.function == b.function && a.width_m == b.width_m &&
         a.cross_slope == b.cross_slope && a.style_id == b.style_id &&
         a.side_marking == b.side_marking;
}

bool equivalent_lane(const LaneBand &a, const LaneBand &b) {
  return a.id == b.id && a.surface_strip_id == b.surface_strip_id &&
         a.lateral_start_m == b.lateral_start_m &&
         a.lateral_end_m == b.lateral_end_m && a.direction == b.direction;
}

bool equivalent_boundary(const BoundaryProfile &a,
                         const BoundaryProfile &b) {
  return a.boundary_id == b.boundary_id && a.role == b.role &&
         a.marking == b.marking && a.segment_styles == b.segment_styles &&
         a.contour.size() == b.contour.size() &&
         std::equal(a.contour.begin(), a.contour.end(), b.contour.begin(),
                    [](const ProfilePoint &x, const ProfilePoint &y) {
                      return x.lateral_m == y.lateral_m &&
                             x.height_m == y.height_m;
                    });
}

const RoadLayoutStrip *find_strip(const RoadLayoutTemplate &section,
                               RoadLayoutStripId id) {
  const auto found = std::find_if(
      section.strips.begin(), section.strips.end(),
      [id](const RoadLayoutStrip &strip) { return strip.id == id; });
  return found == section.strips.end() ? nullptr : &*found;
}

const BoundaryProfile *find_boundary(const RoadLayoutTemplate &section,
                                     std::uint64_t id) {
  const auto found =
      std::find_if(section.boundaries.begin(), section.boundaries.end(),
                   [id](const BoundaryProfile &boundary) {
                     return boundary.boundary_id == id;
                   });
  return found == section.boundaries.end() ? nullptr : &*found;
}

double distance_value(DistanceRef ref, double total_m) {
  if (ref.kind == DistanceRefKind::kFromEnd)
    return total_m - ref.value;
  if (ref.kind == DistanceRefKind::kRatio)
    return total_m * ref.value;
  return ref.value;
}

RoadLayoutTemplate interpolate_section(const RoadLayoutTemplate &from,
                                         const RoadLayoutTemplate &to,
                                         double t) {
  const RoadLayoutTemplate &structure =
      to.strips.size() >= from.strips.size() ? to : from;
  RoadLayoutTemplate out{};
  out.id = t < 1.0 ? from.id : to.id;
  // The alignment origin travels with the widths it is measured against, so it
  // moves the same way they do across the transition.
  out.alignment_offset_from_left_m =
      from.alignment_offset_from_left_m +
      (to.alignment_offset_from_left_m - from.alignment_offset_from_left_m) * t;
  for (const RoadLayoutStrip &structure_strip : structure.strips) {
    const RoadLayoutStrip *a = find_strip(from, structure_strip.id);
    const RoadLayoutStrip *b = find_strip(to, structure_strip.id);
    RoadLayoutStrip strip = b != nullptr ? *b : *a;
    const double a_width = a != nullptr ? a->width_m : 0.0;
    const double b_width = b != nullptr ? b->width_m : 0.0;
    const double a_slope =
        a != nullptr ? a->cross_slope : (b != nullptr ? b->cross_slope : 0.0);
    const double b_slope = b != nullptr ? b->cross_slope : a_slope;
    strip.width_m = a_width + (b_width - a_width) * t;
    strip.cross_slope = a_slope + (b_slope - a_slope) * t;
    out.strips.push_back(std::move(strip));
  }
  out.lane_bands = from.lane_bands;
  for (const LaneBand &target_lane : to.lane_bands) {
    const auto existing = std::find_if(
        out.lane_bands.begin(), out.lane_bands.end(),
        [&target_lane](const LaneBand &lane) { return lane.id == target_lane.id; });
    if (existing != out.lane_bands.end())
      continue;
    const RoadLayoutStrip *strip = find_strip(out, target_lane.surface_strip_id);
    if (strip == nullptr || strip->width_m <= distance_epsilon)
      continue;
    LaneBand lane = target_lane;
    lane.lateral_start_m = 0.0;
    lane.lateral_end_m = strip->width_m;
    out.lane_bands.push_back(lane);
  }
  for (const BoundaryProfile &structure_boundary : structure.boundaries) {
    const BoundaryProfile *a =
        find_boundary(from, structure_boundary.boundary_id);
    const BoundaryProfile *b =
        find_boundary(to, structure_boundary.boundary_id);
    BoundaryProfile boundary = b != nullptr ? *b : *a;
    // A profile only blends into a profile of the same shape. Anything else is
    // a structure changing into a different structure, which the transition
    // rules do not describe, so the section keeps the one it is heading for.
    if (a != nullptr && b != nullptr &&
        a->contour.size() == b->contour.size()) {
      for (std::size_t point = 0; point < boundary.contour.size(); ++point) {
        const ProfilePoint &from_point = a->contour[point];
        const ProfilePoint &to_point = b->contour[point];
        boundary.contour[point].lateral_m =
            from_point.lateral_m +
            (to_point.lateral_m - from_point.lateral_m) * t;
        boundary.contour[point].height_m =
            from_point.height_m +
            (to_point.height_m - from_point.height_m) * t;
      }
    }
    out.boundaries.push_back(std::move(boundary));
  }
  return out;
}

// Merge the marking requests that apply to a boundary into one policy. The
// only sources are the boundary itself and the side policies of the bands
// adjacent in element order; never lateral position or array index.
Result<std::vector<AutoMarkingPolicy>>
merge_boundary_policies(const RoadLayoutTemplate &section) {
  std::vector<AutoMarkingPolicy> merged(section.boundaries.size(),
                                        AutoMarkingPolicy{});
  if (section.strips.empty()) {
    return Result<std::vector<AutoMarkingPolicy>>::Ok(std::move(merged));
  }
  // The outermost sides have no adjacent boundary element to resolve to.
  if (section.strips.front().side_marking.left.enabled ||
      section.strips.back().side_marking.right.enabled) {
    return Result<std::vector<AutoMarkingPolicy>>::Fail(
        CommitFailureCategory::kNotImplemented,
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
          CommitFailureCategory::kNotImplemented,
          "conflicting marking requests on section boundary " +
              std::to_string(boundary.boundary_id));
    }
    merged[index] = resolved;
  }
  return Result<std::vector<AutoMarkingPolicy>>::Ok(std::move(merged));
}

std::vector<SectionBoundarySample>
derive_boundaries(const RoadLayoutTemplate &section,
                 const std::vector<AutoMarkingPolicy> &policies) {
  std::vector<SectionBoundarySample> samples{};
  if (section.strips.empty())
    return samples;
  // Two positions run along the section. `datum` is where the layout says an
  // element sits: the alignment is `alignment_offset_from_left_m` from the left
  // outer end, and only strips move it. `lateral` is where the last surface
  // point was actually placed, which a boundary's profile pulls away from the
  // datum without the layout widths noticing.
  double datum = -section.alignment_offset_from_left_m;
  double lateral = datum;
  double height = 0.0;
  double carriageway_floor = std::numeric_limits<double>::infinity();
  samples.push_back(
      SectionBoundarySample{1, BoundaryRole::kOuterEdge, lateral, height, {}});
  for (std::size_t index = 0; index < section.strips.size(); ++index) {
    const RoadLayoutStrip &strip = section.strips[index];
    const double strip_start_height = height;
    datum += strip.width_m;
    if (index >= section.boundaries.size()) {
      // The last strip runs out to the layout's own edge.
      height += strip.cross_slope * (datum - lateral);
      lateral = datum;
      if (strip.function == StripFunction::kCarriageway) {
        carriageway_floor =
            std::min(carriageway_floor, std::min(strip_start_height, height));
      }
      continue;
    }
    const BoundaryProfile &boundary = section.boundaries[index];
    const AutoMarkingPolicy policy =
        index < policies.size() ? policies[index] : boundary.marking;
    const RoadLayoutStrip &right_strip = section.strips[index + 1];
    const auto with_adjacency = [&](SectionBoundarySample sample) {
      sample.left_strip_id = strip.id;
      sample.right_strip_id = right_strip.id;
      sample.left_strip_width_m = strip.width_m;
      sample.right_strip_width_m = right_strip.width_m;
      return sample;
    };
    // The strip's cross slope runs as far as its surface actually reaches,
    // which is where the profile takes over.
    const double first_lateral = datum + boundary.contour.front().lateral_m;
    height += strip.cross_slope * (first_lateral - lateral);
    lateral = first_lateral;
    if (strip.function == StripFunction::kCarriageway) {
      carriageway_floor =
          std::min(carriageway_floor, std::min(strip_start_height, height));
    }
    // A marking belongs to one side of the profile. A curb between carriageway
    // and walkway carries it on the road side; everything else carries it where
    // the profile ends.
    const bool road_side_marking =
        boundary.role == BoundaryRole::kCurb &&
        strip.function == StripFunction::kCarriageway &&
        right_strip.function == StripFunction::kSidewalk;
    const std::size_t marked = boundary.contour.size() == 1 || road_side_marking
                                   ? 0
                                   : boundary.contour.size() - 1;
    const double datum_height = height - boundary.contour.front().height_m;
    for (std::size_t point = 0; point < boundary.contour.size(); ++point) {
      lateral = datum + boundary.contour[point].lateral_m;
      height = datum_height + boundary.contour[point].height_m;
      samples.push_back(with_adjacency(SectionBoundarySample{
          boundary.boundary_id, boundary.role, lateral, height,
          point == marked ? policy : AutoMarkingPolicy{}}));
    }
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

std::vector<RenderStyleRef>
derive_surface_styles(const RoadLayoutTemplate &section) {
  std::vector<RenderStyleRef> styles{};
  for (std::size_t index = 0; index < section.strips.size(); ++index) {
    const RoadLayoutStrip &strip = section.strips[index];
    styles.push_back(RenderStyleFromSurface(strip.style_id));
    if (index >= section.boundaries.size())
      continue;
    for (const SurfaceStyleId &style : section.boundaries[index].segment_styles)
      styles.push_back(RenderStyleFromSurface(style));
  }
  return styles;
}

} // namespace

bool equivalent_section_definition(const RoadLayoutTemplate &a,
                                   const RoadLayoutTemplate &b) {
  return a.alignment_offset_from_left_m == b.alignment_offset_from_left_m &&
         a.strips.size() == b.strips.size() &&
         a.lane_bands.size() == b.lane_bands.size() &&
         a.boundaries.size() == b.boundaries.size() &&
         std::equal(a.strips.begin(), a.strips.end(), b.strips.begin(),
                    equivalent_strip) &&
         std::is_permutation(a.lane_bands.begin(), a.lane_bands.end(),
                             b.lane_bands.begin(), b.lane_bands.end(),
                             equivalent_lane) &&
         std::equal(a.boundaries.begin(), a.boundaries.end(),
                    b.boundaries.begin(), equivalent_boundary);
}

Result<double> lane_template_lateral(const RoadLayoutTemplate &section,
                                     const LaneBand &lane) {
  double lateral = -section.alignment_offset_from_left_m;
  for (const RoadLayoutStrip &strip : section.strips) {
    if (strip.id == lane.surface_strip_id) {
      return Result<double>::Ok(
          lateral + (lane.lateral_start_m + lane.lateral_end_m) * 0.5);
    }
    lateral += strip.width_m;
  }
  return Result<double>::Fail(CommitFailureCategory::kInternalError,
                              "lane allocation strip is missing");
}

Result<RoadLayoutTemplate> template_at(
    const SavedRoadGraph &graph, const RoadSegment &segment,
    double segment_distance_m, double total_m) {
  const RoadLayoutTemplate *base =
      find_template(graph, segment.layout_template);
  if (base == nullptr) {
    return Result<RoadLayoutTemplate>::Fail(
        CommitFailureCategory::kInvalidInput, "road segment section template is missing");
  }
  if (!segment.transition.has_value()) {
    return Result<RoadLayoutTemplate>::Ok(*base);
  }
  const RoadLayoutTransition *transition =
      find_transition(graph, *segment.transition);
  if (transition == nullptr) {
    return Result<RoadLayoutTemplate>::Fail(
        CommitFailureCategory::kInvalidInput, "road segment transition is missing");
  }
  const RoadLayoutTemplate *from =
      find_template(graph, transition->from_template);
  const RoadLayoutTemplate *to =
      find_template(graph, transition->to_template);
  if (from == nullptr || to == nullptr) {
    return Result<RoadLayoutTemplate>::Fail(
        CommitFailureCategory::kInvalidInput, "road transition template is missing");
  }
  const double start = distance_value(transition->start, total_m);
  const double end = distance_value(transition->end, total_m);
  if (start < 0.0 || end > total_m || end - start <= distance_epsilon) {
    return Result<RoadLayoutTemplate>::Fail(
        CommitFailureCategory::kInvalidInput, "road transition distance range is invalid");
  }
  return Result<RoadLayoutTemplate>::Ok(interpolate_section(
      *from, *to,
      std::clamp((segment_distance_m - start) / (end - start), 0.0, 1.0)));
}

Result<SectionEvaluation> section_at(const SavedRoadGraph &graph,
                                     const RoadSegment &segment,
                                     double segment_distance_m,
                                     double total_m) {
  Result<RoadLayoutTemplate> section =
      template_at(graph, segment, segment_distance_m, total_m);
  if (!section.ok) {
    return Result<SectionEvaluation>::Fail(section.failure_category, section.error);
  }
  Result<std::vector<AutoMarkingPolicy>> policies =
      merge_boundary_policies(section.value);
  if (!policies.ok) {
    return Result<SectionEvaluation>::Fail(policies.failure_category, policies.error);
  }
  std::vector<SectionBoundarySample> boundaries =
      derive_boundaries(section.value, policies.value);
  if (boundaries.empty()) {
    return Result<SectionEvaluation>::Fail(
        CommitFailureCategory::kInternalError, "road section evaluation produced no boundaries");
  }
  if (segment.transition.has_value()) {
    const RoadLayoutTransition *transition =
        find_transition(graph, *segment.transition);
    const RoadLayoutTemplate *from =
        transition == nullptr ? nullptr
                              : find_template(graph, transition->from_template);
    if (transition == nullptr || from == nullptr) {
      return Result<SectionEvaluation>::Fail(
          CommitFailureCategory::kInvalidInput, "road transition anchor source is missing");
    }
    const std::vector<SectionBoundarySample> from_boundaries = derive_boundaries(
        *from, std::vector<AutoMarkingPolicy>(from->boundaries.size(),
                                              AutoMarkingPolicy{}));
    double shift = 0.0;
    if (transition->anchor == TransitionAnchor::kLeftEdge) {
      shift = from_boundaries.front().lateral_m - boundaries.front().lateral_m;
    } else if (transition->anchor == TransitionAnchor::kRightEdge) {
      shift = from_boundaries.back().lateral_m - boundaries.back().lateral_m;
    } else if (transition->anchor == TransitionAnchor::kBoundary) {
      // Structural boundaries have two samples. The first sample is the
      // boundary's left edge in section order and is the stable anchor.
      const auto source = std::find_if(
          from_boundaries.begin(), from_boundaries.end(),
          [transition](const SectionBoundarySample &boundary) {
            return boundary.boundary_id == transition->anchor_boundary_id;
          });
      const auto current = std::find_if(
          boundaries.begin(), boundaries.end(),
          [transition](const SectionBoundarySample &boundary) {
            return boundary.boundary_id == transition->anchor_boundary_id;
          });
      if (source == from_boundaries.end() || current == boundaries.end()) {
        return Result<SectionEvaluation>::Fail(
            CommitFailureCategory::kInternalError,
            "road transition anchor boundary sample is missing");
      }
      shift = source->lateral_m - current->lateral_m;
    }
    for (SectionBoundarySample &boundary : boundaries) {
      boundary.lateral_m += shift;
    }
  }
  return Result<SectionEvaluation>::Ok(SectionEvaluation{
      segment.id, segment_distance_m, section.value.id, std::move(boundaries),
      derive_surface_styles(section.value)});
}

Result<LaneSectionPosition>
lane_position(const RoadLayoutTemplate &section, const LaneBand &lane,
              const SectionEvaluation &evaluation) {
  const auto strip = std::find_if(
      section.strips.begin(), section.strips.end(),
      [&lane](const RoadLayoutStrip &candidate) {
        return candidate.id == lane.surface_strip_id;
      });
  if (strip == section.strips.end()) {
    return Result<LaneSectionPosition>::Fail(
        CommitFailureCategory::kInternalError, "lane allocation strip is missing");
  }
  const SectionBoundarySample *left = nullptr;
  for (const SectionBoundarySample &boundary : evaluation.boundaries) {
    if (boundary.right_strip_id != lane.surface_strip_id)
      continue;
    if (left == nullptr || boundary.lateral_m > left->lateral_m)
      left = &boundary;
  }
  if (left == nullptr && strip == section.strips.begin() &&
      !evaluation.boundaries.empty()) {
    left = &evaluation.boundaries.front();
  }
  if (left == nullptr) {
    return Result<LaneSectionPosition>::Fail(
        CommitFailureCategory::kInternalError, "lane strip left boundary is missing");
  }
  const double offset =
      (lane.lateral_start_m + lane.lateral_end_m) * 0.5;
  return Result<LaneSectionPosition>::Ok(
      LaneSectionPosition{left->lateral_m + offset,
                          left->height_m + strip->cross_slope * offset});
}

} // namespace city::road::internal
