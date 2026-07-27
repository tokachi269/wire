#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace city::road::build {
namespace {

const SurfaceBand *find_band(const CrossSectionTemplate &section,
                             std::uint64_t id) {
  const auto found = std::find_if(
      section.bands.begin(), section.bands.end(),
      [id](const SurfaceBand &band) { return band.element_id == id; });
  return found == section.bands.end() ? nullptr : &*found;
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
      to.bands.size() >= from.bands.size() ? to : from;
  CrossSectionTemplate out{};
  out.id = t < 1.0 ? from.id : to.id;
  for (const SurfaceBand &structure_band : structure.bands) {
    const SurfaceBand *a = find_band(from, structure_band.element_id);
    const SurfaceBand *b = find_band(to, structure_band.element_id);
    SurfaceBand band = b != nullptr ? *b : *a;
    const double a_width = a != nullptr ? a->width_m : 0.0;
    const double b_width = b != nullptr ? b->width_m : 0.0;
    const double a_slope =
        a != nullptr ? a->cross_slope : (b != nullptr ? b->cross_slope : 0.0);
    const double b_slope = b != nullptr ? b->cross_slope : a_slope;
    band.width_m = a_width + (b_width - a_width) * t;
    band.cross_slope = a_slope + (b_slope - a_slope) * t;
    out.bands.push_back(std::move(band));
  }
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

std::vector<SectionBoundarySample>
build_boundaries(const CrossSectionTemplate &section) {
  std::vector<SectionBoundarySample> samples{};
  if (section.bands.empty())
    return samples;
  double total_width = 0.0;
  for (const SurfaceBand &band : section.bands)
    total_width += band.width_m;
  for (const BoundaryProfile &boundary : section.boundaries) {
    total_width += boundary.width_m;
  }
  double lateral = -total_width * 0.5;
  double height = 0.0;
  double carriageway_floor = std::numeric_limits<double>::infinity();
  samples.push_back(
      SectionBoundarySample{1, BoundaryRole::kOuterEdge, lateral, height, {}});
  for (std::size_t index = 0; index < section.bands.size(); ++index) {
    const SurfaceBand &band = section.bands[index];
    const double band_end_height = height + band.cross_slope * band.width_m;
    if (band.role == SurfaceRole::kCarriageway) {
      carriageway_floor =
          std::min(carriageway_floor, std::min(height, band_end_height));
    }
    lateral += band.width_m;
    height = band_end_height;
    if (index >= section.boundaries.size())
      continue;
    const BoundaryProfile &boundary = section.boundaries[index];
    const bool structural_boundary = boundary.role == BoundaryRole::kCurb ||
                                     boundary.role == BoundaryRole::kMedianEdge;
    if (!structural_boundary && boundary.width_m <= station_epsilon &&
        std::abs(boundary.height_m) <= station_epsilon) {
      samples.push_back(SectionBoundarySample{boundary.boundary_id,
                                              boundary.role, lateral, height,
                                              boundary.marking});
      continue;
    }
    AutoMarkingPolicy before_rule{};
    AutoMarkingPolicy after_rule = boundary.marking;
    if (boundary.role == BoundaryRole::kCurb &&
        index + 1 < section.bands.size() &&
        band.role == SurfaceRole::kCarriageway &&
        section.bands[index + 1].role == SurfaceRole::kSidewalk) {
      before_rule = boundary.marking;
      after_rule = {};
    }
    samples.push_back(SectionBoundarySample{boundary.boundary_id, boundary.role,
                                            lateral, height, before_rule});
    lateral += boundary.width_m;
    height += boundary.height_m;
    samples.push_back(SectionBoundarySample{boundary.boundary_id, boundary.role,
                                            lateral, height, after_rule});
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
  for (std::size_t index = 0; index < section.bands.size(); ++index) {
    const SurfaceBand &band = section.bands[index];
    styles.push_back(RenderStyleFromSurface(band.style_id));
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
  std::vector<SectionBoundarySample> boundaries =
      build_boundaries(section.value);
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
    const std::vector<SectionBoundarySample> from_boundaries =
        build_boundaries(*from);
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

Result<bool> make_sections(pipeline &pipe) {
  pipe.out.sections.clear();
  pipe.out.section_evaluation_count = 0;
  for (const RoadSegment &segment : pipe.source.segments) {
    const Path *alignment = find_alignment(pipe.out, segment.id);
    const SegmentSamplingPlan *plan = find_sampling(pipe.out, segment.id);
    if (alignment == nullptr || plan == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road section evaluation input is missing");
    }
    const Result<double> length = PathLength(*alignment);
    if (!length.ok)
      return Result<bool>::Fail(length.error_kind, length.error);
    std::vector<double> stations = plan->semantic_stations_m;
    stations.insert(stations.end(), plan->surface_stations_m.begin(),
                    plan->surface_stations_m.end());
    stations.insert(stations.end(), plan->marking_stations_m.begin(),
                    plan->marking_stations_m.end());
    stations.insert(stations.end(), plan->mask_stations_m.begin(),
                    plan->mask_stations_m.end());
    sort_unique_stations(stations);
    for (const double station : stations) {
      Result<SectionEvaluation> evaluated =
          section_at(pipe.source, segment, station, length.value);
      if (!evaluated.ok) {
        return Result<bool>::Fail(evaluated.error_kind, evaluated.error);
      }
      ++pipe.out.section_evaluation_count;
      pipe.out.sections.push_back(std::move(evaluated.value));
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
