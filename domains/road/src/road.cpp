#include "city/road/road.hpp"

#include "build/build_context.hpp"
#include "materialization/materialize.hpp"
#include "operations/operation_plan.hpp"
#include "persistence/schema.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iterator>
#include <iomanip>
#include <limits>
#include <locale>
#include <numbers>
#include <sstream>
#include <string_view>
#include <unordered_set>

namespace city::road {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kSampleStepM = 2.0;
constexpr int kCurveSamples = 24;
constexpr double kP1MinSegmentLengthM = 8.0;
constexpr double kP1MinConnectionAngleDeg = 45.0;
constexpr double kP1MaxConnectionAngleDeg = 135.0;
constexpr double kSnapStationPointToleranceM = 0.6;

[[nodiscard]] bool finite(double value) {
  return std::isfinite(value);
}

[[nodiscard]] bool finite(Vec2d value) {
  return finite(value.x) && finite(value.y);
}

[[nodiscard]] double dot(Vec2d a, Vec2d b) {
  return a.x * b.x + a.y * b.y;
}

[[nodiscard]] double cross(Vec2d a, Vec2d b) {
  return a.x * b.y - a.y * b.x;
}

[[nodiscard]] Vec2d add(Vec2d a, Vec2d b) {
  return {a.x + b.x, a.y + b.y};
}

[[nodiscard]] Vec2d sub(Vec2d a, Vec2d b) {
  return {a.x - b.x, a.y - b.y};
}

[[nodiscard]] Vec2d mul(Vec2d a, double scale) {
  return {a.x * scale, a.y * scale};
}

[[nodiscard]] double length(Vec2d value) {
  return std::sqrt(dot(value, value));
}

[[nodiscard]] Vec2d normalize(Vec2d value) {
  const double len = length(value);
  if (len <= kEpsilon) {
    return {1.0, 0.0};
  }
  return {value.x / len, value.y / len};
}

[[nodiscard]] double distance(Vec2d a, Vec2d b) {
  return length(sub(a, b));
}

[[nodiscard]] double angle_deg(Vec2d a, Vec2d b) {
  const Vec2d na = normalize(a);
  const Vec2d nb = normalize(b);
  const double c = std::clamp(dot(na, nb), -1.0, 1.0);
  return std::acos(c) * 180.0 / 3.14159265358979323846;
}

[[nodiscard]] bool almost_same(Vec2d a, Vec2d b) {
  return distance(a, b) <= 1e-6;
}

[[nodiscard]] Vec2d span_start(const BezierSpan& span) {
  return span.p0;
}

[[nodiscard]] Vec2d span_end(const BezierSpan& span) {
  return span.p3;
}

[[nodiscard]] Vec2d span_eval(const BezierSpan& span, double t) {
  t = std::clamp(t, 0.0, 1.0);
  const double u = 1.0 - t;
  return add(add(mul(span.p0, u * u * u), mul(span.p1, 3.0 * u * u * t)),
             add(mul(span.p2, 3.0 * u * t * t), mul(span.p3, t * t * t)));
}

[[nodiscard]] double span_length(const BezierSpan& span) {
  double out = 0.0;
  Vec2d prev = span.p0;
  for (int i = 1; i <= kCurveSamples; ++i) {
    const Vec2d current = span_eval(span, static_cast<double>(i) / kCurveSamples);
    out += distance(prev, current);
    prev = current;
  }
  return out;
}

[[nodiscard]] double span_parameter_at_length(const BezierSpan& span, double station_m) {
  if (station_m <= 0.0) return 0.0;
  double accumulated = 0.0;
  Vec2d previous = span.p0;
  for (int i = 1; i <= kCurveSamples; ++i) {
    const double t = static_cast<double>(i) / kCurveSamples;
    const Vec2d current = span_eval(span, t);
    const double piece_length = distance(previous, current);
    if (accumulated + piece_length >= station_m || i == kCurveSamples) {
      const double ratio = piece_length <= kEpsilon ? 0.0 : (station_m - accumulated) / piece_length;
      return (static_cast<double>(i - 1) + std::clamp(ratio, 0.0, 1.0)) / kCurveSamples;
    }
    accumulated += piece_length;
    previous = current;
  }
  return 1.0;
}

[[nodiscard]] std::pair<BezierSpan, BezierSpan> split_span(const BezierSpan& span, double t) {
  const Vec2d p01 = add(mul(span.p0, 1.0 - t), mul(span.p1, t));
  const Vec2d p12 = add(mul(span.p1, 1.0 - t), mul(span.p2, t));
  const Vec2d p23 = add(mul(span.p2, 1.0 - t), mul(span.p3, t));
  const Vec2d p012 = add(mul(p01, 1.0 - t), mul(p12, t));
  const Vec2d p123 = add(mul(p12, 1.0 - t), mul(p23, t));
  const Vec2d point = add(mul(p012, 1.0 - t), mul(p123, t));
  return {BezierSpan{span.p0, p01, p012, point}, BezierSpan{point, p123, p23, span.p3}};
}

struct PathSplit {
  Path before{};
  Path after{};
  Vec2d point{};
};

[[nodiscard]] Result<PathSplit> split_path_at_station(const Path& path, double station_m) {
  const Result<double> total = PathLength(path);
  if (!total.ok) return Result<PathSplit>::Fail(total.error_kind, total.error);
  if (!finite(station_m) || station_m <= kEpsilon || station_m >= total.value - kEpsilon) {
    return Result<PathSplit>::Fail(ErrorKind::kValidation, "road segment split station is outside its interior");
  }
  double remaining = station_m;
  for (std::size_t index = 0; index < path.spans.size(); ++index) {
    const BezierSpan& span = path.spans[index];
    const double length_m = span_length(span);
    if (index + 1 < path.spans.size() && std::abs(remaining - length_m) <= kEpsilon) {
      PathSplit result{};
      result.before.spans.insert(result.before.spans.end(), path.spans.begin(), path.spans.begin() + index + 1);
      result.after.spans.insert(result.after.spans.end(), path.spans.begin() + index + 1, path.spans.end());
      result.point = span.p3;
      return Result<PathSplit>::Ok(std::move(result));
    }
    if (remaining < length_m || index + 1 == path.spans.size()) {
      const double t = span_parameter_at_length(span, remaining);
      auto [left, right] = split_span(span, t);
      PathSplit result{};
      result.before.spans.insert(result.before.spans.end(), path.spans.begin(), path.spans.begin() + index);
      result.before.spans.push_back(left);
      result.after.spans.push_back(right);
      result.after.spans.insert(result.after.spans.end(), path.spans.begin() + index + 1, path.spans.end());
      result.point = left.p3;
      return Result<PathSplit>::Ok(std::move(result));
    }
    remaining -= length_m;
  }
  return Result<PathSplit>::Fail(ErrorKind::kInternal, "road segment split station resolution fell through");
}

[[nodiscard]] bool segments_intersect(Vec2d a, Vec2d b, Vec2d c, Vec2d d) {
  const auto orient = [](Vec2d p, Vec2d q, Vec2d r) {
    const double v = cross(sub(q, p), sub(r, p));
    if (std::abs(v) <= 1e-8) {
      return 0;
    }
    return v > 0.0 ? 1 : -1;
  };
  const auto on_segment = [](Vec2d p, Vec2d q, Vec2d r) {
    return std::min(p.x, r.x) - 1e-8 <= q.x && q.x <= std::max(p.x, r.x) + 1e-8 &&
           std::min(p.y, r.y) - 1e-8 <= q.y && q.y <= std::max(p.y, r.y) + 1e-8;
  };
  const int o1 = orient(a, b, c);
  const int o2 = orient(a, b, d);
  const int o3 = orient(c, d, a);
  const int o4 = orient(c, d, b);
  if (o1 != o2 && o3 != o4) {
    return true;
  }
  return (o1 == 0 && on_segment(a, c, b)) || (o2 == 0 && on_segment(a, d, b)) ||
         (o3 == 0 && on_segment(c, a, d)) || (o4 == 0 && on_segment(c, b, d));
}

[[nodiscard]] Result<bool> validate_no_self_intersection(const Path& path) {
  const std::vector<Vec2d> points = FlattenPath(path);
  if (points.size() < 4) {
    return Result<bool>::Ok(true);
  }
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    for (std::size_t j = i + 2; j + 1 < points.size(); ++j) {
      if (i == 0 && j + 1 == points.size() - 1 && almost_same(points.front(), points.back())) {
        continue;
      }
      if (segments_intersect(points[i], points[i + 1], points[j], points[j + 1])) {
        return Result<bool>::Fail(ErrorKind::kUnsupported, "road path self-intersection is unsupported");
      }
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] const CrossSectionTemplate* find_template(const SavedRoadGraph& graph, CrossSectionTemplateId id) {
  const auto it = std::find_if(graph.section_templates.begin(), graph.section_templates.end(),
                               [id](const CrossSectionTemplate& item) { return item.id == id; });
  return it == graph.section_templates.end() ? nullptr : &*it;
}

[[nodiscard]] const SectionTransition* find_transition(const SavedRoadGraph& graph, SectionTransitionId id) {
  const auto it = std::find_if(graph.transitions.begin(), graph.transitions.end(),
                               [id](const SectionTransition& item) { return item.id == id; });
  return it == graph.transitions.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadSegment* find_segment(const SavedRoadGraph& graph, RoadSegmentId id) {
  const auto it = std::find_if(graph.segments.begin(), graph.segments.end(),
                               [id](const RoadSegment& item) { return item.id == id; });
  return it == graph.segments.end() ? nullptr : &*it;
}

[[nodiscard]] RoadSegment* find_segment(SavedRoadGraph& graph, RoadSegmentId id) {
  const auto it = std::find_if(graph.segments.begin(), graph.segments.end(),
                               [id](const RoadSegment& item) { return item.id == id; });
  return it == graph.segments.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadNode* find_node(const SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

[[nodiscard]] const NodeConnectionPolicyOverride* find_connection_policy_override(const SavedRoadGraph& graph,
                                                                                   RoadNodeId node_id) {
  const auto it = std::find_if(graph.connection_policy_overrides.begin(), graph.connection_policy_overrides.end(),
                               [node_id](const auto& item) { return item.node_id == node_id; });
  return it == graph.connection_policy_overrides.end() ? nullptr : &*it;
}

[[nodiscard]] RoadNode* find_node(SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

[[nodiscard]] std::size_t node_degree(const SavedRoadGraph& graph, RoadNodeId id) {
  return static_cast<std::size_t>(std::count_if(graph.segments.begin(), graph.segments.end(), [id](const RoadSegment& segment) {
    return segment.node_a == id || segment.node_b == id;
  }));
}

[[nodiscard]] std::vector<SectionBoundarySample> evaluate_section(const CrossSectionTemplate& section) {
  std::vector<SectionBoundarySample> samples{};
  if (section.bands.empty()) {
    return samples;
  }
  double total_width = 0.0;
  for (const SurfaceBand& band : section.bands) {
    total_width += band.width_m;
  }
  for (const BoundaryProfile& boundary : section.boundaries) {
    total_width += boundary.width_m;
  }
  double lateral = -total_width * 0.5;
  double height = 0.0;
  double carriageway_floor = std::numeric_limits<double>::infinity();
  samples.push_back(SectionBoundarySample{1, BoundaryRole::kOuterEdge, lateral, height, MarkingRule::kNone});
  for (std::size_t i = 0; i < section.bands.size(); ++i) {
    const SurfaceBand& band = section.bands[i];
    const double band_end_height = height + band.cross_slope * band.width_m;
    if (band.role == SurfaceRole::kCarriageway) {
      carriageway_floor = std::min(carriageway_floor, std::min(height, band_end_height));
    }
    lateral += section.bands[i].width_m;
    height = band_end_height;
    if (i < section.boundaries.size()) {
      const BoundaryProfile& boundary = section.boundaries[i];
      const bool structural_boundary = boundary.role == BoundaryRole::kCurb ||
                                       boundary.role == BoundaryRole::kMedianEdge;
      if (!structural_boundary && boundary.width_m <= kEpsilon && std::abs(boundary.height_m) <= kEpsilon) {
        samples.push_back(
            SectionBoundarySample{boundary.boundary_id, boundary.role, lateral, height, boundary.marking_rule});
        continue;
      }
      samples.push_back(SectionBoundarySample{boundary.boundary_id, boundary.role, lateral, height, MarkingRule::kNone});
      lateral += boundary.width_m;
      height += boundary.height_m;
      samples.push_back(
          SectionBoundarySample{boundary.boundary_id, boundary.role, lateral, height, boundary.marking_rule});
    }
  }
  samples.push_back(SectionBoundarySample{999, BoundaryRole::kOuterEdge, lateral, height, MarkingRule::kNone});
  if (finite(carriageway_floor)) {
    for (SectionBoundarySample& sample : samples) sample.height_m -= carriageway_floor;
  }
  return samples;
}

[[nodiscard]] std::string boundary_material(BoundaryRole role) {
  if (role == BoundaryRole::kCurb) return "curb";
  if (role == BoundaryRole::kMedianEdge) return "median";
  return "asphalt";
}

[[nodiscard]] std::vector<std::string> surface_materials(const CrossSectionTemplate& section) {
  std::vector<std::string> materials{};
  for (std::size_t i = 0; i < section.bands.size(); ++i) {
    materials.push_back(section.bands[i].style.empty() ? "surface" : section.bands[i].style);
    if (i < section.boundaries.size()) {
      const BoundaryProfile& boundary = section.boundaries[i];
      if (boundary.role == BoundaryRole::kCurb || boundary.role == BoundaryRole::kMedianEdge ||
          boundary.width_m > kEpsilon || std::abs(boundary.height_m) > kEpsilon) {
        materials.push_back(boundary_material(boundary.role));
      }
    }
  }
  return materials;
}

[[nodiscard]] const SurfaceBand* find_band(const CrossSectionTemplate& section, std::uint64_t id) {
  const auto it = std::find_if(section.bands.begin(), section.bands.end(),
                               [id](const SurfaceBand& band) { return band.element_id == id; });
  return it == section.bands.end() ? nullptr : &*it;
}

[[nodiscard]] const BoundaryProfile* find_boundary(const CrossSectionTemplate& section, std::uint64_t id) {
  const auto it = std::find_if(section.boundaries.begin(), section.boundaries.end(),
                               [id](const BoundaryProfile& boundary) { return boundary.boundary_id == id; });
  return it == section.boundaries.end() ? nullptr : &*it;
}

[[nodiscard]] double station_value(StationRef ref, double total) {
  if (ref.kind == StationRefKind::kFromEnd) {
    return total - ref.value;
  }
  if (ref.kind == StationRefKind::kRatio) {
    return total * ref.value;
  }
  return ref.value;
}

[[nodiscard]] CrossSectionTemplate interpolate_section(const CrossSectionTemplate& from,
                                                        const CrossSectionTemplate& to, double t) {
  const CrossSectionTemplate& structure = to.bands.size() >= from.bands.size() ? to : from;
  CrossSectionTemplate out{};
  out.id = t < 1.0 ? from.id : to.id;
  for (const SurfaceBand& structure_band : structure.bands) {
    const SurfaceBand* a = find_band(from, structure_band.element_id);
    const SurfaceBand* b = find_band(to, structure_band.element_id);
    SurfaceBand band = b != nullptr ? *b : *a;
    const double a_width = a != nullptr ? a->width_m : 0.0;
    const double b_width = b != nullptr ? b->width_m : 0.0;
    const double a_slope = a != nullptr ? a->cross_slope : (b != nullptr ? b->cross_slope : 0.0);
    const double b_slope = b != nullptr ? b->cross_slope : a_slope;
    band.width_m = a_width + (b_width - a_width) * t;
    band.cross_slope = a_slope + (b_slope - a_slope) * t;
    out.bands.push_back(std::move(band));
  }
  for (const BoundaryProfile& structure_boundary : structure.boundaries) {
    const BoundaryProfile* a = find_boundary(from, structure_boundary.boundary_id);
    const BoundaryProfile* b = find_boundary(to, structure_boundary.boundary_id);
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

[[nodiscard]] Result<std::vector<SectionBoundarySample>> evaluate_segment_section(
    const SavedRoadGraph& graph, const RoadSegment& segment, double station, double total) {
  const CrossSectionTemplate* base = find_template(graph, segment.section_template);
  if (base == nullptr) {
    return Result<std::vector<SectionBoundarySample>>::Fail(ErrorKind::kValidation,
                                                            "road segment section template is missing");
  }
  if (!segment.transition.has_value()) {
    return Result<std::vector<SectionBoundarySample>>::Ok(evaluate_section(*base));
  }
  const SectionTransition* transition = find_transition(graph, *segment.transition);
  if (transition == nullptr) {
    return Result<std::vector<SectionBoundarySample>>::Fail(ErrorKind::kValidation,
                                                            "road segment transition is missing");
  }
  const CrossSectionTemplate* from = find_template(graph, transition->from_template);
  const CrossSectionTemplate* to = find_template(graph, transition->to_template);
  if (from == nullptr || to == nullptr) {
    return Result<std::vector<SectionBoundarySample>>::Fail(ErrorKind::kValidation,
                                                            "road transition template is missing");
  }
  const double start = station_value(transition->start, total);
  const double end = station_value(transition->end, total);
  if (start < 0.0 || end > total || end - start <= kEpsilon) {
    return Result<std::vector<SectionBoundarySample>>::Fail(ErrorKind::kValidation,
                                                            "road transition station range is invalid");
  }
  const double t = std::clamp((station - start) / (end - start), 0.0, 1.0);
  std::vector<SectionBoundarySample> samples = evaluate_section(interpolate_section(*from, *to, t));
  if (samples.empty()) {
    return Result<std::vector<SectionBoundarySample>>::Fail(ErrorKind::kInternal,
                                                            "road transition produced an empty section");
  }
  const double from_left = evaluate_section(interpolate_section(*from, *to, 0.0)).front().lateral_m;
  const double from_right = evaluate_section(interpolate_section(*from, *to, 0.0)).back().lateral_m;
  const double shift = transition->anchor == TransitionAnchor::kLeftEdge
                           ? from_left - samples.front().lateral_m
                           : (transition->anchor == TransitionAnchor::kRightEdge
                                  ? from_right - samples.back().lateral_m
                                  : 0.0);
  for (SectionBoundarySample& sample : samples) {
    sample.lateral_m += shift;
  }
  return Result<std::vector<SectionBoundarySample>>::Ok(std::move(samples));
}

[[nodiscard]] Result<CrossSectionTemplate> evaluate_segment_template(const SavedRoadGraph& graph,
                                                                     const RoadSegment& segment,
                                                                     double station, double total) {
  const CrossSectionTemplate* base = find_template(graph, segment.section_template);
  if (base == nullptr) {
    return Result<CrossSectionTemplate>::Fail(ErrorKind::kValidation, "road segment section template is missing");
  }
  if (!segment.transition.has_value()) return Result<CrossSectionTemplate>::Ok(*base);
  const SectionTransition* transition = find_transition(graph, *segment.transition);
  if (transition == nullptr) {
    return Result<CrossSectionTemplate>::Fail(ErrorKind::kValidation, "road segment transition is missing");
  }
  const CrossSectionTemplate* from = find_template(graph, transition->from_template);
  const CrossSectionTemplate* to = find_template(graph, transition->to_template);
  if (from == nullptr || to == nullptr) {
    return Result<CrossSectionTemplate>::Fail(ErrorKind::kValidation, "road transition template is missing");
  }
  const double start = station_value(transition->start, total);
  const double end = station_value(transition->end, total);
  if (start < 0.0 || end > total || end - start <= kEpsilon) {
    return Result<CrossSectionTemplate>::Fail(ErrorKind::kValidation, "road transition station range is invalid");
  }
  return Result<CrossSectionTemplate>::Ok(
      interpolate_section(*from, *to, std::clamp((station - start) / (end - start), 0.0, 1.0)));
}

[[nodiscard]] Vec3d to3(Vec2d p, double z = 0.0) {
  return {p.x, p.y, z};
}

[[nodiscard]] std::vector<double> stations_for_path(const Path& path) {
  const Result<double> length_result = PathLength(path);
  if (!length_result.ok) {
    return {};
  }
  const double total = length_result.value;
  const int count = std::max(1, static_cast<int>(std::ceil(total / kSampleStepM)));
  std::vector<double> stations{};
  stations.reserve(static_cast<std::size_t>(count) + 1);
  for (int i = 0; i <= count; ++i) {
    stations.push_back(total * static_cast<double>(i) / count);
  }
  return stations;
}

[[nodiscard]] std::optional<Vec2d> endpoint_outward_tangent(const RoadSegment& segment,
                                                            const Path& alignment,
                                                            RoadNodeId node_id) {
  const Result<double> length = PathLength(alignment);
  if (!length.ok) return std::nullopt;
  const double delta = std::min(0.1, length.value);
  if (node_id == segment.node_a) {
    const auto endpoint = EvaluatePath(alignment, 0.0);
    const auto inward = EvaluatePath(alignment, delta);
    if (!endpoint.ok || !inward.ok) return std::nullopt;
    return normalize(sub(inward.value, endpoint.value));
  }
  if (node_id == segment.node_b) {
    const auto endpoint = EvaluatePath(alignment, length.value);
    const auto inward = EvaluatePath(alignment, length.value - delta);
    if (!endpoint.ok || !inward.ok) return std::nullopt;
    return normalize(sub(inward.value, endpoint.value));
  }
  return std::nullopt;
}

[[nodiscard]] std::vector<const RoadSegment*> node_approaches(const SavedRoadGraph& graph, RoadNodeId node_id) {
  std::vector<const RoadSegment*> out{};
  for (const RoadSegment& segment : graph.segments) {
    if (segment.node_a == node_id || segment.node_b == node_id) out.push_back(&segment);
  }
  return out;
}

[[nodiscard]] double approach_outer_half_width(const SavedRoadGraph& graph,
                                               const RoadSegment& segment,
                                               const Path& alignment,
                                               RoadNodeId node_id) {
  const auto length = PathLength(alignment);
  if (!length.ok) return 0.0;
  const double station = node_id == segment.node_a ? 0.0 : length.value;
  const auto section = evaluate_segment_section(graph, segment, station, length.value);
  if (!section.ok || section.value.empty()) return 0.0;
  return std::max(std::abs(section.value.front().lateral_m), std::abs(section.value.back().lateral_m));
}

[[nodiscard]] bool is_straight_connection(const SavedRoadGraph& graph,
                                          const DerivedRoad& derived,
                                          RoadNodeId node_id) {
  const auto approaches = node_approaches(graph, node_id);
  if (approaches.size() != 2) return false;
  const Path* path_a = FindCanonicalAlignment(derived, approaches[0]->id);
  const Path* path_b = FindCanonicalAlignment(derived, approaches[1]->id);
  if (path_a == nullptr || path_b == nullptr) return false;
  const auto a = endpoint_outward_tangent(*approaches[0], *path_a, node_id);
  const auto b = endpoint_outward_tangent(*approaches[1], *path_b, node_id);
  return a.has_value() && b.has_value() && dot(*a, *b) <= -std::cos(5.0 * std::numbers::pi / 180.0);
}

[[nodiscard]] const build::NodeConnectionDecision* find_connection_decision(
    const std::vector<build::NodeConnectionDecision>& decisions, RoadNodeId node_id) {
  const auto found = std::find_if(decisions.begin(), decisions.end(), [node_id](const auto& decision) {
    return decision.node_id == node_id;
  });
  return found == decisions.end() ? nullptr : &*found;
}

[[nodiscard]] double node_gate_setback(const SavedRoadGraph& graph,
                                       const DerivedRoad& derived,
                                       build::NodeConnectionKind connection_kind,
                                       const RoadSegment& segment,
                                       const Path& alignment,
                                       RoadNodeId node_id) {
  const auto approaches = node_approaches(graph, node_id);
  if (connection_kind == build::NodeConnectionKind::kPassThrough) return 0.0;
  const auto tangent = endpoint_outward_tangent(segment, alignment, node_id);
  if (!tangent.has_value()) return 0.0;
  if (connection_kind == build::NodeConnectionKind::kCorner) {
    const RoadSegment* other = approaches[0] == &segment ? approaches[1] : approaches[0];
    const Path* other_path = FindCanonicalAlignment(derived, other->id);
    if (other_path == nullptr) return 0.0;
    const auto other_tangent = endpoint_outward_tangent(*other, *other_path, node_id);
    if (!other_tangent.has_value()) return 0.0;
    const double outward_angle = std::acos(std::clamp(dot(*tangent, *other_tangent), -1.0, 1.0));
    const double turn_angle = std::numbers::pi - outward_angle;
    return 4.0 * std::tan(turn_angle * 0.5);
  }
  double setback = 4.0;
  for (const RoadSegment* other : approaches) {
    if (other == &segment) continue;
    const Path* other_path = FindCanonicalAlignment(derived, other->id);
    if (other_path == nullptr) continue;
    const auto other_tangent = endpoint_outward_tangent(*other, *other_path, node_id);
    if (!other_tangent.has_value()) continue;
    const double sine = std::abs(cross(*tangent, *other_tangent));
    if (sine <= 1e-3) continue;
    setback = std::max(setback, approach_outer_half_width(graph, *other, *other_path, node_id) / sine);
  }
  return setback;
}

[[nodiscard]] std::vector<double> stations_for_segment_mesh(const SavedRoadGraph& graph,
                                                            const DerivedRoad& derived,
                                                            const std::vector<build::NodeConnectionDecision>& decisions,
                                                            const RoadSegment& segment,
                                                            const Path& alignment) {
  const Result<double> length_result = PathLength(alignment);
  if (!length_result.ok) return {};
  const double total = length_result.value;
  const auto* start_decision = find_connection_decision(decisions, segment.node_a);
  const auto* end_decision = find_connection_decision(decisions, segment.node_b);
  if (start_decision == nullptr || end_decision == nullptr) return {};
  const double start = node_gate_setback(graph, derived, start_decision->kind, segment, alignment, segment.node_a);
  const double end = total - node_gate_setback(graph, derived, end_decision->kind, segment, alignment, segment.node_b);
  if (end < start - kEpsilon) return {};
  if (end - start <= kEpsilon) return {start};
  const int count = std::max(1, static_cast<int>(std::ceil((end - start) / kSampleStepM)));
  std::vector<double> stations{};
  stations.reserve(static_cast<std::size_t>(count) + 1);
  for (int i = 0; i <= count; ++i) {
    stations.push_back(start + (end - start) * static_cast<double>(i) / count);
  }
  return stations;
}

[[nodiscard]] Vec2d path_start(const Path& path) {
  return span_start(path.spans.front());
}

[[nodiscard]] Vec2d path_end(const Path& path) {
  return span_end(path.spans.back());
}

[[nodiscard]] std::optional<Vec2d> path_tangent(const Path& path, double station, double total) {
  const double delta = std::min(0.1, total);
  const double before_station = std::max(0.0, station - delta);
  const double after_station = std::min(total, station + delta);
  if (after_station - before_station <= kEpsilon) {
    return std::nullopt;
  }
  const Result<Vec2d> before = EvaluatePath(path, before_station);
  const Result<Vec2d> after = EvaluatePath(path, after_station);
  if (!before.ok || !after.ok) {
    return std::nullopt;
  }
  return normalize(sub(after.value, before.value));
}

[[nodiscard]] double section_surface_height(const std::vector<SectionBoundarySample>& boundaries,
                                            double lateral_m) {
  if (boundaries.empty()) return 0.0;
  if (lateral_m <= boundaries.front().lateral_m) return boundaries.front().height_m;
  for (std::size_t i = 1; i < boundaries.size(); ++i) {
    const auto& a = boundaries[i - 1];
    const auto& b = boundaries[i];
    if (lateral_m > b.lateral_m) continue;
    const double width = b.lateral_m - a.lateral_m;
    const double t = width <= kEpsilon ? 0.0 : (lateral_m - a.lateral_m) / width;
    return a.height_m + (b.height_m - a.height_m) * t;
  }
  return boundaries.back().height_m;
}

[[nodiscard]] std::optional<Vec3d> owner_local_point(const SavedRoadGraph& graph,
                                                      const RoadSegment& owner,
                                                      const Path& alignment,
                                                      Vec2d local,
                                                      double z_offset) {
  const auto total_result = PathLength(alignment);
  if (!total_result.ok || local.x < 0.0 || local.x > total_result.value) return std::nullopt;
  const auto center = EvaluatePath(alignment, local.x);
  const auto tangent = path_tangent(alignment, local.x, total_result.value);
  const auto section = evaluate_segment_section(graph, owner, local.x, total_result.value);
  if (!center.ok || !tangent.has_value() || !section.ok) return std::nullopt;
  const Vec2d lateral{-tangent->y, tangent->x};
  const Vec2d world = add(center.value, mul(lateral, local.y));
  return Vec3d{world.x, world.y, section_surface_height(section.value, local.y) + z_offset};
}

[[nodiscard]] Result<bool> validate_transition(const SavedRoadGraph& graph, const SectionTransition& transition) {
  const CrossSectionTemplate* from = find_template(graph, transition.from_template);
  const CrossSectionTemplate* to = find_template(graph, transition.to_template);
  if (from == nullptr || to == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition references a missing template");
  }
  if (!finite(transition.start.value) || !finite(transition.end.value)) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition station is not finite");
  }
  const auto valid_station = [](StationRef station) {
    return station.value >= 0.0 &&
           (station.kind != StationRefKind::kRatio || station.value <= 1.0);
  };
  if (!valid_station(transition.start) || !valid_station(transition.end)) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition station reference is outside its range");
  }
  if (transition.rules.empty()) {
    return Result<bool>::Fail(ErrorKind::kUnsupported, "section transition must define element actions");
  }
  std::unordered_set<std::uint64_t> ruled_elements{};
  for (const SectionTransitionRule& rule : transition.rules) {
    if (rule.element_id == 0 || !ruled_elements.insert(rule.element_id).second ||
        (find_band(*from, rule.element_id) == nullptr && find_band(*to, rule.element_id) == nullptr)) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section transition rule element is invalid");
    }
    if (rule.action == TransitionAction::kUnsupported) {
      return Result<bool>::Fail(ErrorKind::kUnsupported, "section transition contains unsupported element action");
    }
  }
  const auto action_for = [&transition](std::uint64_t id) -> std::optional<TransitionAction> {
    const auto it = std::find_if(transition.rules.begin(), transition.rules.end(),
                                 [id](const SectionTransitionRule& rule) { return rule.element_id == id; });
    return it == transition.rules.end() ? std::nullopt : std::optional<TransitionAction>(it->action);
  };
  for (const SurfaceBand& band : to->bands) {
    if (find_band(*from, band.element_id) == nullptr && action_for(band.element_id) != TransitionAction::kTaperIn) {
      return Result<bool>::Fail(ErrorKind::kUnsupported, "appearing section element requires TaperIn");
    }
  }
  for (const SurfaceBand& band : from->bands) {
    if (find_band(*to, band.element_id) != nullptr) continue;
    const std::optional<TransitionAction> action = action_for(band.element_id);
    const TransitionAction required = band.role == SurfaceRole::kMedian ? TransitionAction::kEndCap
                                                                        : TransitionAction::kTaperOut;
    if (action != required) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                band.role == SurfaceRole::kMedian
                                    ? "disappearing median requires EndCap"
                                    : "disappearing section element requires TaperOut");
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] Result<bool> validate_section_template(const CrossSectionTemplate& section) {
  if (section.bands.empty() || section.boundaries.size() + 1 != section.bands.size()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section template chain is incomplete");
  }
  std::unordered_set<std::uint64_t> ids{};
  for (const SurfaceBand& band : section.bands) {
    if (band.element_id == 0 || !ids.insert(band.element_id).second || !finite(band.width_m) ||
        !finite(band.cross_slope) || band.width_m <= 0.0) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section template surface band is invalid");
    }
  }
  ids.clear();
  for (const BoundaryProfile& boundary : section.boundaries) {
    if (boundary.boundary_id == 0 || !ids.insert(boundary.boundary_id).second || !finite(boundary.width_m) ||
        !finite(boundary.height_m) || boundary.width_m < 0.0) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section template boundary is invalid");
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] std::optional<double> parse_double(std::string_view text) {
  if (text.empty()) {
    return std::nullopt;
  }
  double value = 0.0;
  std::istringstream stream{std::string(text)};
  stream.imbue(std::locale::classic());
  stream >> std::noskipws >> value;
  if (!stream || !stream.eof() || !finite(value)) {
    return std::nullopt;
  }
  return value;
}

[[nodiscard]] std::optional<std::uint64_t> parse_u64(std::string_view text) {
  std::uint64_t value = 0;
  const char* first = text.data();
  const char* last = text.data() + text.size();
  const std::from_chars_result result = std::from_chars(first, last, value);
  if (result.ec != std::errc{} || result.ptr != last) {
    return std::nullopt;
  }
  return value;
}

} // namespace

BezierSpan MakeLine(Vec2d a, Vec2d b) {
  const Vec2d delta = sub(b, a);
  return BezierSpan{a, add(a, mul(delta, 1.0 / 3.0)), add(a, mul(delta, 2.0 / 3.0)), b};
}

BezierSpan MakeBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3) {
  return BezierSpan{p0, p1, p2, p3};
}

Path MakePath(std::vector<BezierSpan> spans) {
  Path out{};
  out.spans = std::move(spans);
  return out;
}

Result<SegmentShape> SegmentShapeFromPath(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<SegmentShape>::Fail(valid.error_kind, valid.error);
  SegmentShape shape{};
  shape.start_handle = sub(path.spans.front().p1, path.spans.front().p0);
  shape.end_handle = sub(path.spans.back().p2, path.spans.back().p3);
  shape.internal_knots.reserve(path.spans.size() - 1);
  for (std::size_t i = 0; i + 1 < path.spans.size(); ++i) {
    const BezierSpan& before = path.spans[i];
    const BezierSpan& after = path.spans[i + 1];
    shape.internal_knots.push_back(
        SegmentKnot{before.p3, sub(before.p2, before.p3), sub(after.p1, after.p0)});
  }
  return Result<SegmentShape>::Ok(std::move(shape));
}

Result<Path> BuildCanonicalAlignment(Vec2d start, Vec2d end, const SegmentShape& shape) {
  if (!finite(start) || !finite(end) || !finite(shape.start_handle) || !finite(shape.end_handle)) {
    return Result<Path>::Fail(ErrorKind::kValidation, "road segment shape contains a non-finite point");
  }
  Path path{};
  Vec2d span_start = start;
  Vec2d handle_out = shape.start_handle;
  for (const SegmentKnot& knot : shape.internal_knots) {
    if (!finite(knot.position) || !finite(knot.handle_in) || !finite(knot.handle_out)) {
      return Result<Path>::Fail(ErrorKind::kValidation, "road segment shape contains a non-finite knot");
    }
    path.spans.push_back(
        MakeBezier(span_start, add(span_start, handle_out), add(knot.position, knot.handle_in), knot.position));
    span_start = knot.position;
    handle_out = knot.handle_out;
  }
  path.spans.push_back(MakeBezier(span_start, add(span_start, handle_out), add(end, shape.end_handle), end));
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<Path>::Fail(valid.error_kind, valid.error);
  return Result<Path>::Ok(std::move(path));
}

const Path* FindCanonicalAlignment(const DerivedRoad& derived, RoadSegmentId segment_id) {
  const auto found = std::find_if(derived.canonical_alignments.begin(), derived.canonical_alignments.end(),
                                  [segment_id](const CanonicalAlignment& item) {
                                    return item.segment_id == segment_id;
                                  });
  return found == derived.canonical_alignments.end() ? nullptr : &found->path;
}

bool IsLinearSpan(const BezierSpan& span) {
  return almost_same(span.p1, add(span.p0, mul(sub(span.p3, span.p0), 1.0 / 3.0))) &&
         almost_same(span.p2, add(span.p0, mul(sub(span.p3, span.p0), 2.0 / 3.0)));
}

RoadToolDraft PreviewRoadToolPath(Vec2d start, Vec2d end, std::optional<Vec2d> handle_a, std::optional<Vec2d> handle_b) {
  RoadToolDraft draft{};
  draft.has_live_preview = true;
  draft.supports_bezier_handles = true;
  if (handle_a.has_value() && handle_b.has_value()) {
    draft.preview_path = MakePath({MakeBezier(start, *handle_a, *handle_b, end)});
  } else {
    draft.preview_path = MakePath({MakeLine(start, end)});
  }
  return draft;
}

CrossSectionTemplate JapaneseUrbanTwoLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section{};
  section.id = id;
  section.bands = {
      {10, SurfaceRole::kSidewalk, 2.0, 0.01, "sidewalk"},
      {20, SurfaceRole::kCarriageway, 3.0, 0.02, "asphalt"},
      {30, SurfaceRole::kCarriageway, 3.0, -0.02, "asphalt"},
      {40, SurfaceRole::kSidewalk, 2.0, -0.01, "sidewalk"},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
  };
  return section;
}

CrossSectionTemplate ThreeLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands = {
      {10, SurfaceRole::kSidewalk, 2.0, 0.01, "sidewalk"},
      {20, SurfaceRole::kCarriageway, 3.0, 0.02, "asphalt"},
      {30, SurfaceRole::kCarriageway, 3.0, 0.0, "asphalt"},
      {35, SurfaceRole::kCarriageway, 3.0, -0.02, "asphalt"},
      {40, SurfaceRole::kSidewalk, 2.0, -0.01, "sidewalk"},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
  };
  return section;
}

CrossSectionTemplate NoLeftSidewalkTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands.erase(section.bands.begin());
  section.boundaries.erase(section.boundaries.begin());
  return section;
}

CrossSectionTemplate MedianTwoLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands.insert(section.bands.begin() + 2, {25, SurfaceRole::kMedian, 2.0, 0.0, "median"});
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
      {210, BoundaryRole::kMedianEdge, 0.2, 0.12, MarkingRule::kNone},
      {220, BoundaryRole::kMedianEdge, 0.2, -0.12, MarkingRule::kNone},
      {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
  };
  return section;
}

RoadState::RoadState() {
  graph_.section_templates.push_back(JapaneseUrbanTwoLaneTemplate(1));
  graph_.section_templates.push_back(ThreeLaneTemplate(2));
  graph_.section_templates.push_back(NoLeftSidewalkTemplate(3));
  graph_.section_templates.push_back(MedianTwoLaneTemplate(4));
  next_id_ = 5;
}

const SavedRoadGraph& RoadState::graph() const {
  return graph_;
}

const DerivedRoad& RoadState::derived() const {
  return derived_;
}

Result<bool> RoadState::Execute(const operations::OperationPlan& plan) {
  RoadState trial = *this;
  const Result<bool> applied = operations::Apply(plan, trial.graph_, trial.next_id_);
  if (!applied.ok) return applied;
  const Result<bool> built = trial.BuildDerived();
  if (!built.ok) return built;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<RoadSegmentId> RoadState::AddSegment(AddSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const Result<bool> valid = ValidatePath(alignment);
  if (!valid.ok) {
    return Result<RoadSegmentId>::Fail(valid.error_kind, valid.error);
  }
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment references a missing section template");
  }
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.error_kind, shape.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId node_a = next_id++;
  const RoadNodeId node_b = next_id++;
  const RoadSegmentId segment_id = next_id++;
  plan.next_id_after = next_id;
  plan.add_nodes = {RoadNode{node_a, path_start(alignment)}, RoadNode{node_b, path_end(alignment)}};
  plan.add_segments.push_back(
      RoadSegment{segment_id, node_a, node_b, shape.value, section_template, std::nullopt});
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedTo(AddSegmentConnectedToRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const RoadNodeId start_node = request.start_node;
  const RoadNode* node = find_node(graph_, start_node);
  if (node == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment start node does not exist");
  }
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation,
                                       "connected road references a missing section template");
  }
  const Result<double> length_result = PathLength(alignment);
  if (!length_result.ok) {
    return Result<RoadSegmentId>::Fail(length_result.error_kind, length_result.error);
  }
  if (length_result.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment is shorter than P1 minimum");
  }
  BezierSpan& first_span = alignment.spans.front();
  const Vec2d correction = sub(node->position, first_span.p0);
  first_span.p0 = node->position;
  first_span.p1 = add(first_span.p1, correction);
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.error_kind, shape.error);
  const Vec2d new_direction = normalize(sub(path_end(alignment), path_start(alignment)));
  const std::size_t existing_degree = static_cast<std::size_t>(std::count_if(
      graph_.segments.begin(), graph_.segments.end(), [start_node](const RoadSegment& item) {
        return item.node_a == start_node || item.node_b == start_node;
      }));
  if (existing_degree >= 4) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "road junction supports at most four approaches");
  }
  for (const RoadSegment& existing : graph_.segments) {
    if (existing.node_a != start_node && existing.node_b != start_node) {
      continue;
    }
    const Path* existing_path = FindCanonicalAlignment(derived_, existing.id);
    if (existing_path == nullptr) {
      return Result<RoadSegmentId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
    }
    const Result<double> existing_length = PathLength(*existing_path);
    if (!existing_length.ok) {
      return Result<RoadSegmentId>::Fail(existing_length.error_kind, existing_length.error);
    }
    const double endpoint_station = existing.node_a == start_node ? 0.0 : existing_length.value;
    const Result<CrossSectionTemplate> endpoint_section =
        evaluate_segment_template(graph_, existing, endpoint_station, existing_length.value);
    if (!endpoint_section.ok) {
      return Result<RoadSegmentId>::Fail(endpoint_section.error_kind, endpoint_section.error);
    }
    if (endpoint_section.value.id != section_template) {
      return Result<RoadSegmentId>::Fail(
          ErrorKind::kUnsupported,
          "connected road section must match the existing connection gate section");
    }
    const Vec2d other = existing.node_a == start_node ? path_end(*existing_path) : path_start(*existing_path);
    const double angle = angle_deg(sub(other, node->position), new_direction);
    const bool opposite_continuation = angle >= 180.0 - 1e-6;
    if (!opposite_continuation && (angle < kP1MinConnectionAngleDeg || angle > kP1MaxConnectionAngleDeg)) {
      return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment angle is outside P1 range");
    }
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  plan.add_nodes.push_back(RoadNode{end_node, path_end(alignment)});
  plan.add_segments.push_back(
      RoadSegment{segment_id, start_node, end_node, shape.value, section_template, std::nullopt});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedToSegment(AddSegmentConnectedToSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const RoadSegmentId start_segment = request.start_segment;
  const double station_m = request.station_m;
  const Result<bool> alignment_valid = ValidatePath(alignment);
  if (!alignment_valid.ok) {
    return Result<RoadSegmentId>::Fail(alignment_valid.error_kind, alignment_valid.error);
  }
  const RoadSegment* source = find_segment(graph_, start_segment);
  if (source == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment snap target does not exist");
  }
  if (source->transition.has_value()) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported,
                                       "splitting a transitioning road segment is unsupported");
  }
  if (source->section_template != section_template) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported,
                                       "road segment split requires a matching section template");
  }
  const Path* source_path = FindCanonicalAlignment(derived_, source->id);
  if (source_path == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  }
  const Result<PathSplit> path_split = split_path_at_station(*source_path, station_m);
  if (!path_split.ok) {
    return Result<RoadSegmentId>::Fail(path_split.error_kind, path_split.error);
  }
  if (distance(path_start(alignment), path_split.value.point) > kSnapStationPointToleranceM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation,
                                       "road segment input start does not match its explicit snap station");
  }
  BezierSpan& input_start = alignment.spans.front();
  if (IsLinearSpan(input_start)) {
    input_start = MakeLine(path_split.value.point, input_start.p3);
  } else {
    const Vec2d correction = sub(path_split.value.point, input_start.p0);
    input_start.p0 = path_split.value.point;
    input_start.p1 = add(input_start.p1, correction);
  }
  const Result<double> branch_length = PathLength(alignment);
  if (!branch_length.ok) return Result<RoadSegmentId>::Fail(branch_length.error_kind, branch_length.error);
  if (branch_length.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment is shorter than P1 minimum");
  }
  const Vec2d branch_direction = normalize(sub(path_end(alignment), path_split.value.point));
  for (const Vec2d other : std::array<Vec2d, 2>{path_start(*source_path), path_end(*source_path)}) {
    const double angle = angle_deg(sub(other, path_split.value.point), branch_direction);
    const bool opposite_continuation = angle >= 180.0 - 1e-6;
    if (!opposite_continuation && (angle < kP1MinConnectionAngleDeg || angle > kP1MaxConnectionAngleDeg)) {
      return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment angle is outside P1 range");
    }
  }

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId split_node = next_id++;
  const RoadSegmentId second_id = next_id++;
  const RoadNodeId branch_end_node = next_id++;
  const RoadSegmentId branch_id = next_id++;
  const Result<SegmentShape> first_shape = SegmentShapeFromPath(path_split.value.before);
  const Result<SegmentShape> second_shape = SegmentShapeFromPath(path_split.value.after);
  const Result<SegmentShape> branch_shape = SegmentShapeFromPath(alignment);
  if (!first_shape.ok || !second_shape.ok || !branch_shape.ok) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment split shape is invalid");
  }
  RoadSegment first = *source;
  first.node_b = split_node;
  first.shape = first_shape.value;
  plan.replace_segments.push_back(std::move(first));
  plan.add_nodes = {RoadNode{split_node, path_split.value.point}, RoadNode{branch_end_node, path_end(alignment)}};
  plan.add_segments = {
      RoadSegment{second_id, split_node, source->node_b, second_shape.value, source->section_template,
                  source->transition},
      RoadSegment{branch_id, split_node, branch_end_node, branch_shape.value, section_template, std::nullopt},
  };
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  return Result<RoadSegmentId>::Ok(branch_id);
}

Result<bool> RoadState::EditSegmentShape(EditSegmentShapeRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  SegmentShape shape = std::move(request.shape);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  const RoadNode* node_a = find_node(graph_, segment->node_a);
  const RoadNode* node_b = find_node(graph_, segment->node_b);
  const Result<Path> alignment = BuildCanonicalAlignment(node_a->position, node_b->position, shape);
  if (!alignment.ok) return Result<bool>::Fail(alignment.error_kind, alignment.error);
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadSegment replacement = *segment;
  replacement.shape = std::move(shape);
  plan.replace_segments.push_back(std::move(replacement));
  return Execute(plan);
}

Result<bool> RoadState::MoveNode(MoveNodeRequest request) {
  const RoadNodeId node_id = request.node_id;
  const Vec2d position = request.position;
  if (!finite(position)) return Result<bool>::Fail(ErrorKind::kValidation, "road node position is non-finite");
  const RoadNode* node = find_node(graph_, node_id);
  if (node == nullptr) return Result<bool>::Fail(ErrorKind::kValidation, "road node does not exist");
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadNode replacement = *node;
  replacement.position = position;
  plan.replace_nodes.push_back(replacement);
  return Execute(plan);
}

Result<bool> RoadState::DeleteSegment(DeleteSegmentRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const RoadSegment* target = find_segment(graph_, segment_id);
  if (target == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.remove_segments.push_back(segment_id);
  for (const RoadNodeId node_id : std::array<RoadNodeId, 2>{target->node_a, target->node_b}) {
    if (node_degree(graph_, node_id) == 1) plan.remove_nodes.push_back(node_id);
  }
  for (const NodeConnectionPolicyOverride& policy : graph_.connection_policy_overrides) {
    const std::size_t degree_after = node_degree(graph_, policy.node_id) -
                                     ((target->node_a == policy.node_id || target->node_b == policy.node_id) ? 1 : 0);
    if (degree_after == 0) plan.remove_connection_policy_overrides.push_back(policy.id);
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id == segment_id) plan.remove_manual_lines.push_back(marking.id);
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id == segment_id) plan.remove_manual_areas.push_back(marking.id);
  }
  const std::optional<SectionTransitionId> transition = target->transition;
  if (transition.has_value() &&
      std::none_of(graph_.segments.begin(), graph_.segments.end(), [segment_id, transition](const RoadSegment& segment) {
        return segment.id != segment_id && segment.transition == transition;
      })) {
    plan.remove_transitions.push_back(*transition);
  }
  return Execute(plan);
}

Result<CrossSectionTemplateId> RoadState::AddSectionTemplate(AddSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) {
    return Result<CrossSectionTemplateId>::Fail(valid.error_kind, valid.error);
  }
  if (section_template.id != 0 && find_template(graph_, section_template.id) != nullptr) {
    return Result<CrossSectionTemplateId>::Fail(ErrorKind::kValidation, "section template id already exists");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (section_template.id == 0) section_template.id = next_id++;
  const CrossSectionTemplateId id = section_template.id;
  plan.next_id_after = next_id;
  plan.add_section_templates.push_back(std::move(section_template));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<CrossSectionTemplateId>::Fail(executed.error_kind, executed.error);
  return Result<CrossSectionTemplateId>::Ok(id);
}

Result<bool> RoadState::EditSectionTemplate(EditSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) return valid;
  auto it = std::find_if(graph_.section_templates.begin(), graph_.section_templates.end(),
                         [&section_template](const CrossSectionTemplate& item) { return item.id == section_template.id; });
  if (it == graph_.section_templates.end()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section template does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_section_templates.push_back(std::move(section_template));
  return Execute(plan);
}

Result<SectionTransitionId> RoadState::AddTransition(SectionTransitionRequest request) {
  SectionTransition transition{0, request.from_template, request.to_template, request.start, request.end,
                               request.anchor, std::move(request.rules)};
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) {
    return Result<SectionTransitionId>::Fail(valid.error_kind, valid.error);
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (transition.id == 0) transition.id = next_id++;
  const SectionTransitionId id = transition.id;
  plan.next_id_after = next_id;
  plan.add_transitions.push_back(std::move(transition));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<SectionTransitionId>::Fail(executed.error_kind, executed.error);
  return Result<SectionTransitionId>::Ok(id);
}

Result<SectionTransitionId> RoadState::AddTransitionToSegment(AddTransitionToSegmentRequest operation) {
  const RoadSegmentId segment_id = operation.segment_id;
  SectionTransitionRequest request = std::move(operation.transition);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<SectionTransitionId>::Fail(ErrorKind::kValidation, "road transition segment does not exist");
  }
  SectionTransition transition{0, segment->section_template, request.to_template, request.start, request.end,
                               request.anchor, std::move(request.rules)};
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) return Result<SectionTransitionId>::Fail(valid.error_kind, valid.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (transition.id == 0) transition.id = next_id++;
  const SectionTransitionId id = transition.id;
  const std::optional<SectionTransitionId> old_transition = segment->transition;
  RoadSegment replacement = *segment;
  replacement.transition = id;
  plan.replace_segments.push_back(std::move(replacement));
  plan.add_transitions.push_back(std::move(transition));
  if (old_transition.has_value()) {
    plan.remove_transitions.push_back(*old_transition);
  }
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<SectionTransitionId>::Fail(executed.error_kind, executed.error);
  return Result<SectionTransitionId>::Ok(id);
}

Result<bool> RoadState::AttachSectionTransition(AttachSectionTransitionRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const SectionTransitionId transition_id = request.transition_id;
  const RoadSegment* segment = find_segment(graph_, segment_id);
  const SectionTransition* transition = find_transition(graph_, transition_id);
  if (segment == nullptr || transition == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition attachment reference is missing");
  }
  if (segment->section_template != transition->from_template) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition from-template does not match segment");
  }
  const Path* alignment = FindCanonicalAlignment(derived_, segment_id);
  if (alignment == nullptr) return Result<bool>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  const auto total = PathLength(*alignment);
  if (!total.ok) return Result<bool>::Fail(total.error_kind, total.error);
  const double start = station_value(transition->start, total.value);
  const double end = station_value(transition->end, total.value);
  if (start < 0.0 || end > total.value || end - start <= kEpsilon) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition station range is invalid");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadSegment replacement = *segment;
  replacement.transition = transition_id;
  plan.replace_segments.push_back(std::move(replacement));
  return Execute(plan);
}

Result<ManualMarkingId> RoadState::AddManualLine(ManualLineRequest request) {
  ManualLineMarking marking{0, request.owner_segment_id, std::move(request.path), std::move(request.style)};
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line owner segment does not exist");
  }
  const Result<bool> valid = ValidatePath(marking.path);
  if (!valid.ok) {
    return Result<ManualMarkingId>::Fail(valid.error_kind, valid.error);
  }
  const Path* alignment = FindCanonicalAlignment(derived_, owner->id);
  if (alignment == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  }
  const auto length_result = PathLength(*alignment);
  for (const Vec2d point : FlattenPath(marking.path)) {
    if (!length_result.ok || point.x < 0.0 || point.x > length_result.value || !finite(point.y)) {
      return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line lies outside owner station range");
    }
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (marking.id == 0) marking.id = next_id++;
  const ManualMarkingId id = marking.id;
  plan.next_id_after = next_id;
  plan.add_manual_lines.push_back(std::move(marking));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<ManualMarkingId>::Fail(executed.error_kind, executed.error);
  return Result<ManualMarkingId>::Ok(id);
}

Result<ManualMarkingId> RoadState::AddManualArea(ManualAreaRequest request) {
  ManualAreaMarking marking{0, request.owner_segment_id, request.frame_origin, request.width_m, request.length_m,
                            std::move(request.style)};
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area owner segment does not exist");
  }
  if (!finite(marking.frame_origin) || !finite(marking.width_m) || !finite(marking.length_m) ||
      marking.width_m <= 0.0 || marking.length_m <= 0.0) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area shape is invalid");
  }
  const Path* alignment = FindCanonicalAlignment(derived_, owner->id);
  if (alignment == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  }
  const auto owner_length = PathLength(*alignment);
  if (!owner_length.ok || marking.frame_origin.x - marking.length_m * 0.5 < 0.0 ||
      marking.frame_origin.x + marking.length_m * 0.5 > owner_length.value) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area lies outside owner station range");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (marking.id == 0) marking.id = next_id++;
  const ManualMarkingId id = marking.id;
  plan.next_id_after = next_id;
  plan.add_manual_areas.push_back(std::move(marking));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<ManualMarkingId>::Fail(executed.error_kind, executed.error);
  return Result<ManualMarkingId>::Ok(id);
}

Result<bool> RoadState::BuildDerived() {
  build::BuildContext context(graph_);
  std::vector<materialization::ConnectionInput> connection_inputs{};
  std::vector<materialization::JunctionInput> junction_inputs{};
  std::vector<materialization::ManualLineInput> manual_line_inputs{};
  std::vector<materialization::ManualAreaInput> manual_area_inputs{};
  const auto begin_stage = [&context](build::Stage stage) { return context.Begin(stage); };

  if (const auto started = begin_stage(build::Stage::kTopologyIndex); !started.ok) return started;
  context.topology.reserve(graph_.nodes.size());
  for (const RoadNode& node : graph_.nodes) {
    build::NodeTopology topology{};
    topology.node_id = node.id;
    for (const RoadSegment& segment : graph_.segments) {
      if (segment.node_a == node.id || segment.node_b == node.id) topology.approach_segment_ids.push_back(segment.id);
    }
    context.topology.push_back(std::move(topology));
  }

  if (const auto started = begin_stage(build::Stage::kCanonicalAlignment); !started.ok) return started;
  context.derived.canonical_alignments.reserve(graph_.segments.size());
  for (const RoadSegment& segment : graph_.segments) {
    const RoadNode* node_a = find_node(graph_, segment.node_a);
    const RoadNode* node_b = find_node(graph_, segment.node_b);
    if (node_a == nullptr || node_b == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment endpoint node is missing");
    }
    Result<Path> alignment = BuildCanonicalAlignment(node_a->position, node_b->position, segment.shape);
    if (!alignment.ok) return Result<bool>::Fail(alignment.error_kind, alignment.error);
    context.derived.canonical_alignments.push_back(CanonicalAlignment{segment.id, std::move(alignment.value)});
  }

  if (const auto started = begin_stage(build::Stage::kNodeConnectionDecision); !started.ok) return started;
  context.connection_decisions.reserve(context.topology.size());
  for (const build::NodeTopology& topology : context.topology) {
    build::NodeConnectionDecision decision{};
    decision.node_id = topology.node_id;
    const NodeConnectionPolicyOverride* policy = find_connection_policy_override(graph_, topology.node_id);
    if (policy != nullptr) {
      if (policy->policy == NodeConnectionPolicy::kForcePassThrough) {
        decision.kind = build::NodeConnectionKind::kPassThrough;
      } else if (policy->policy == NodeConnectionPolicy::kForceCorner) {
        decision.kind = build::NodeConnectionKind::kCorner;
      } else {
        decision.kind = build::NodeConnectionKind::kJunction;
      }
      decision.reason = "explicit policy override";
    } else if (topology.approach_segment_ids.size() <= 1) {
      decision.kind = build::NodeConnectionKind::kPassThrough;
      decision.reason = "endpoint";
    } else if (topology.approach_segment_ids.size() == 2) {
      const bool straight = is_straight_connection(graph_, context.derived, topology.node_id);
      decision.kind = straight ? build::NodeConnectionKind::kPassThrough : build::NodeConnectionKind::kCorner;
      decision.reason = straight ? "two aligned approaches" : "two turning approaches";
    } else if (topology.approach_segment_ids.size() <= 4) {
      decision.kind = build::NodeConnectionKind::kJunction;
      decision.reason = "three or four approaches";
    } else {
      decision.kind = build::NodeConnectionKind::kUnsupported;
      decision.reason = "more than four approaches";
      return Result<bool>::Fail(ErrorKind::kUnsupported, "road node has more than four approaches");
    }
    context.connection_decisions.push_back(std::move(decision));
  }

  if (const auto started = begin_stage(build::Stage::kSamplingPlan); !started.ok) return started;
  context.sampling_plans.reserve(graph_.segments.size());
  for (const RoadSegment& segment : graph_.segments) {
    const Path* alignment = FindCanonicalAlignment(context.derived, segment.id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
    }
    build::SegmentSamplingPlan plan{
        segment.id, stations_for_segment_mesh(graph_, context.derived, context.connection_decisions, segment, *alignment)};
    if (plan.stations_m.empty()) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment has no section stations");
    }
    context.sampling_plans.push_back(std::move(plan));
  }

  if (const auto started = begin_stage(build::Stage::kSectionEvaluation); !started.ok) return started;
  for (const RoadSegment& segment : graph_.segments) {
    if (find_template(graph_, segment.section_template) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment section template is missing");
    }
    const auto plan = std::find_if(context.sampling_plans.begin(), context.sampling_plans.end(),
                                   [&segment](const auto& item) { return item.segment_id == segment.id; });
    if (plan == context.sampling_plans.end()) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road segment sampling plan is missing");
    }
    const double total = plan->stations_m.back();
    for (double station : plan->stations_m) {
      const auto evaluated = evaluate_segment_section(graph_, segment, station, total);
      if (!evaluated.ok) return Result<bool>::Fail(evaluated.error_kind, evaluated.error);
      const auto evaluated_template = evaluate_segment_template(graph_, segment, station, total);
      if (!evaluated_template.ok) {
        return Result<bool>::Fail(evaluated_template.error_kind, evaluated_template.error);
      }
      context.derived.section_evaluations.push_back(
          SectionEvaluation{segment.id, station, evaluated.value, surface_materials(evaluated_template.value)});
    }
  }

  if (const auto started = begin_stage(build::Stage::kConnectionGate); !started.ok) return started;
  for (const RoadSegment& segment : graph_.segments) {
    const Path* alignment = FindCanonicalAlignment(context.derived, segment.id);
    if (alignment == nullptr) return Result<bool>::Fail(ErrorKind::kInternal, "road alignment is missing");
    const auto total_result = PathLength(*alignment);
    if (!total_result.ok) return Result<bool>::Fail(total_result.error_kind, total_result.error);
    const double total = total_result.value;
    for (RoadNodeId node_id : std::array<RoadNodeId, 2>{segment.node_a, segment.node_b}) {
      const RoadNode* node = find_node(graph_, node_id);
      if (node == nullptr) {
        return Result<bool>::Fail(ErrorKind::kValidation, "road segment endpoint node is missing");
      }
      const auto* decision = find_connection_decision(context.connection_decisions, node_id);
      if (decision == nullptr) return Result<bool>::Fail(ErrorKind::kInternal, "road connection decision is missing");
      const double offset = node_gate_setback(graph_, context.derived, decision->kind, segment, *alignment, node_id);
      const double gate_station = node_id == segment.node_a ? offset : total - offset;
      const auto gate_center = EvaluatePath(*alignment, gate_station);
      const auto path_direction = path_tangent(*alignment, gate_station, total);
      if (!gate_center.ok || !path_direction.has_value()) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road connection gate frame could not be evaluated");
      }
      const Vec2d tangent2 = node_id == segment.node_a ? *path_direction : mul(*path_direction, -1.0);
      const auto gate_section = evaluate_segment_section(graph_, segment, gate_station, total);
      if (!gate_section.ok) return Result<bool>::Fail(gate_section.error_kind, gate_section.error);
      context.derived.connection_gates.push_back(
          ConnectionGate{segment.id, node_id, to3(gate_center.value), to3(tangent2), gate_section.value});
    }
  }

  if (const auto started = begin_stage(build::Stage::kJunctionGeometry); !started.ok) return started;
  for (const build::NodeConnectionDecision& decision : context.connection_decisions) {
    if (decision.kind == build::NodeConnectionKind::kPassThrough) continue;
    if (decision.kind == build::NodeConnectionKind::kCorner) {
      ConnectionArea area{};
      area.node_id = decision.node_id;
      for (const ConnectionGate& gate : context.derived.connection_gates) {
        if (gate.node_id == decision.node_id) area.gates.push_back(gate);
      }
      if (area.gates.size() != 2) return Result<bool>::Fail(ErrorKind::kInternal, "road corner gates are invalid");
      const RoadNode* node = find_node(graph_, decision.node_id);
      const auto section = std::find_if(context.derived.section_evaluations.begin(),
                                        context.derived.section_evaluations.end(), [&area](const auto& item) {
                                          return item.segment_id == area.gates.front().segment_id;
                                        });
      if (node == nullptr || section == context.derived.section_evaluations.end()) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road corner materialization read model is missing");
      }
      connection_inputs.push_back(
          materialization::ConnectionInput{area, node->position, section->surface_materials});
      context.derived.connection_areas.push_back(std::move(area));
      continue;
    }
    const NodeConnectionPolicyOverride* policy = find_connection_policy_override(graph_, decision.node_id);
    JunctionArea area{};
    area.policy_override_id = policy == nullptr ? 0 : policy->id;
    area.node_id = decision.node_id;
    for (const ConnectionGate& gate : context.derived.connection_gates) {
      if (gate.node_id == decision.node_id) area.gates.push_back(gate);
    }
    if (area.gates.size() < 3) return Result<bool>::Fail(ErrorKind::kInternal, "road junction gates are invalid");
    const RoadNode* node = find_node(graph_, decision.node_id);
    if (node == nullptr) return Result<bool>::Fail(ErrorKind::kInternal, "road junction node is missing");
    junction_inputs.push_back(materialization::JunctionInput{area, node->position});
    context.derived.junction_areas.push_back(std::move(area));
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
    const Path* alignment = owner == nullptr ? nullptr : FindCanonicalAlignment(context.derived, owner->id);
    if (owner == nullptr || alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "manual line owner read model is missing");
    }
    materialization::ManualLineInput input{};
    input.marking_id = marking.id;
    const std::vector<Vec2d> points = FlattenPath(marking.path);
    constexpr double half_width = 0.05;
    for (std::size_t i = 0; i < points.size(); ++i) {
      Vec2d direction = i + 1 < points.size() ? sub(points[i + 1], points[i]) : sub(points[i], points[i - 1]);
      direction = normalize(direction);
      const Vec2d normal{-direction.y, direction.x};
      const auto left = owner_local_point(graph_, *owner, *alignment, add(points[i], mul(normal, -half_width)), 0.02);
      const auto right = owner_local_point(graph_, *owner, *alignment, add(points[i], mul(normal, half_width)), 0.02);
      if (!left.has_value() || !right.has_value()) {
        return Result<bool>::Fail(ErrorKind::kValidation, "manual line read model is invalid");
      }
      input.left.push_back(*left);
      input.right.push_back(*right);
    }
    manual_line_inputs.push_back(std::move(input));
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
    const Path* alignment = owner == nullptr ? nullptr : FindCanonicalAlignment(context.derived, owner->id);
    if (owner == nullptr || alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "manual area owner read model is missing");
    }
    materialization::ManualAreaInput input{};
    input.marking_id = marking.id;
    const double hw = marking.width_m * 0.5;
    const double hl = marking.length_m * 0.5;
    const std::array<Vec2d, 4> locals{Vec2d{marking.frame_origin.x - hl, marking.frame_origin.y - hw},
                                      Vec2d{marking.frame_origin.x + hl, marking.frame_origin.y - hw},
                                      Vec2d{marking.frame_origin.x - hl, marking.frame_origin.y + hw},
                                      Vec2d{marking.frame_origin.x + hl, marking.frame_origin.y + hw}};
    for (std::size_t i = 0; i < locals.size(); ++i) {
      const auto point = owner_local_point(graph_, *owner, *alignment, locals[i], 0.025);
      if (!point.has_value()) return Result<bool>::Fail(ErrorKind::kValidation, "manual area read model is invalid");
      input.corners[i] = *point;
    }
    manual_area_inputs.push_back(std::move(input));
  }

  if (const auto started = begin_stage(build::Stage::kMaterialization); !started.ok) return started;
  for (const CanonicalAlignment& canonical : context.derived.canonical_alignments) {
    const Path& alignment = canonical.path;
    const auto total = PathLength(alignment);
    if (!total.ok) return Result<bool>::Fail(total.error_kind, total.error);
    materialization::SegmentInput input{};
    input.segment_id = canonical.segment_id;
    for (const SectionEvaluation& section : context.derived.section_evaluations) {
      if (section.segment_id != canonical.segment_id) continue;
      const auto center = EvaluatePath(alignment, section.station_m);
      const auto tangent = path_tangent(alignment, section.station_m, total.value);
      if (!center.ok || !tangent.has_value()) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road materialization sample frame is invalid");
      }
      input.samples.push_back(
          materialization::SegmentSample{center.value, *tangent, section.boundaries, section.surface_materials});
    }
    auto output = materialization::MaterializeSegment(input);
    if (!output.ok) return Result<bool>::Fail(output.error_kind, output.error);
    context.derived.segment_meshes.insert(context.derived.segment_meshes.end(),
                                           std::make_move_iterator(output.value.surface_meshes.begin()),
                                           std::make_move_iterator(output.value.surface_meshes.end()));
    context.derived.marking_meshes.push_back(std::move(output.value.marking_mesh));
    context.derived.terrain_masks.push_back(std::move(output.value.terrain_mask));
  }
  for (const materialization::ConnectionInput& input : connection_inputs) {
    auto materialized = materialization::MaterializeConnection(input);
    if (!materialized.ok) return Result<bool>::Fail(materialized.error_kind, materialized.error);
    std::vector<Mesh>& meshes = materialized.value;
    context.derived.connection_meshes.insert(context.derived.connection_meshes.end(),
                                              std::make_move_iterator(meshes.begin()),
                                              std::make_move_iterator(meshes.end()));
  }
  for (const materialization::JunctionInput& input : junction_inputs) {
    auto materialized = materialization::MaterializeJunction(input);
    if (!materialized.ok) return Result<bool>::Fail(materialized.error_kind, materialized.error);
    std::vector<Mesh>& meshes = materialized.value.surface_meshes;
    context.derived.junction_meshes.insert(context.derived.junction_meshes.end(),
                                            std::make_move_iterator(meshes.begin()),
                                            std::make_move_iterator(meshes.end()));
    std::vector<Mesh>& markings = materialized.value.marking_meshes;
    context.derived.junction_marking_meshes.insert(context.derived.junction_marking_meshes.end(),
                                                    std::make_move_iterator(markings.begin()),
                                                    std::make_move_iterator(markings.end()));
  }
  for (const materialization::ManualLineInput& input : manual_line_inputs) {
    auto mesh = materialization::MaterializeManualLine(input);
    if (!mesh.ok) return Result<bool>::Fail(mesh.error_kind, mesh.error);
    context.derived.manual_marking_meshes.push_back(std::move(mesh.value));
  }
  for (const materialization::ManualAreaInput& input : manual_area_inputs) {
    auto mesh = materialization::MaterializeManualArea(input);
    if (!mesh.ok) return Result<bool>::Fail(mesh.error_kind, mesh.error);
    context.derived.manual_marking_meshes.push_back(std::move(mesh.value));
  }

  if (const auto started = begin_stage(build::Stage::kDerivedInvariant); !started.ok) return started;
  context.derived.build_stage_runs = context.stage_runs;
  const Result<bool> invariant = ValidateGraphInvariants(graph_, context.derived);
  if (!invariant.ok) {
    return invariant;
  }
  derived_ = std::move(context.derived);
  return Result<bool>::Ok(true);
}

Result<std::string> RoadState::Save() const {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << persistence::kHeader;
  out << "next_id=" << next_id_ << "\n";
  for (const CrossSectionTemplate& section : graph_.section_templates) {
    out << "section_template=" << section.id << "\n";
    for (const SurfaceBand& band : section.bands) {
      out << "surface_band=" << section.id << "," << band.element_id << "," << static_cast<int>(band.role) << ","
          << band.width_m << "," << band.cross_slope << "," << band.style << "\n";
    }
    for (const BoundaryProfile& boundary : section.boundaries) {
      out << "boundary=" << section.id << "," << boundary.boundary_id << "," << static_cast<int>(boundary.role)
          << "," << boundary.width_m << "," << boundary.height_m << "," << static_cast<int>(boundary.marking_rule)
          << "\n";
    }
  }
  for (const SectionTransition& transition : graph_.transitions) {
    out << "transition=" << transition.id << "," << transition.from_template << "," << transition.to_template << ","
        << static_cast<int>(transition.start.kind) << "," << transition.start.value << ","
        << static_cast<int>(transition.end.kind) << "," << transition.end.value << ","
        << static_cast<int>(transition.anchor) << "\n";
    for (const SectionTransitionRule& rule : transition.rules) {
      out << "transition_rule=" << transition.id << "," << rule.element_id << "," << static_cast<int>(rule.action)
          << "\n";
    }
  }
  for (const RoadNode& node : graph_.nodes) {
    out << "node=" << node.id << "," << node.position.x << "," << node.position.y << "\n";
  }
  for (const NodeConnectionPolicyOverride& policy : graph_.connection_policy_overrides) {
    out << "connection_policy_override=" << policy.id << "," << policy.node_id << ","
        << static_cast<int>(policy.policy) << "\n";
  }
  for (const RoadSegment& segment : graph_.segments) {
    out << "segment=" << segment.id << "," << segment.node_a << "," << segment.node_b << ","
        << segment.section_template << "," << segment.transition.value_or(0) << ","
        << segment.shape.internal_knots.size() << "\n";
    out << "segment_shape=" << segment.shape.start_handle.x << "," << segment.shape.start_handle.y << ","
        << segment.shape.end_handle.x << "," << segment.shape.end_handle.y << "\n";
    for (const SegmentKnot& knot : segment.shape.internal_knots) {
      out << "segment_knot=" << knot.position.x << "," << knot.position.y << "," << knot.handle_in.x << ","
          << knot.handle_in.y << "," << knot.handle_out.x << "," << knot.handle_out.y << "\n";
    }
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    out << "manual_line=" << marking.id << "," << marking.owner_segment_id << "," << marking.style << ","
        << marking.path.spans.size() << "\n";
    for (const BezierSpan& span : marking.path.spans) {
      out << "manual_span=" << span.p0.x << "," << span.p0.y << "," << span.p1.x << "," << span.p1.y << ","
          << span.p2.x << "," << span.p2.y << "," << span.p3.x << "," << span.p3.y << "\n";
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    out << "manual_area=" << marking.id << "," << marking.owner_segment_id << "," << marking.frame_origin.x << ","
        << marking.frame_origin.y << "," << marking.width_m << "," << marking.length_m << "," << marking.style << "\n";
  }
  return Result<std::string>::Ok(out.str());
}

Result<RoadState> RoadState::Load(const std::string& text) {
  const bool version5 = persistence::HasCurrentHeader(text);
  if (!version5 && persistence::HasRoadHeader(text)) {
    return Result<RoadState>::Fail(ErrorKind::kValidation, "legacy road graph version is unsupported");
  }
  if (!version5) {
    return Result<RoadState>::Fail(ErrorKind::kValidation, "unknown road graph version");
  }
  RoadState state{};
  state.graph_.nodes.clear();
  state.graph_.segments.clear();
  state.graph_.transitions.clear();
  state.graph_.connection_policy_overrides.clear();
  state.graph_.manual_lines.clear();
  state.graph_.manual_areas.clear();
  state.graph_.section_templates.clear();
  std::istringstream in(text);
  std::string line;
  RoadSegment* current_segment = nullptr;
  ManualLineMarking* current_manual_line = nullptr;
  std::vector<std::pair<RoadSegmentId, std::size_t>> expected_segment_knots{};
  std::unordered_set<RoadSegmentId> segment_shapes{};
  std::vector<std::pair<ManualMarkingId, std::size_t>> expected_manual_spans{};
  const auto span_from_parts = [](const std::vector<std::string_view>& parts) -> std::optional<BezierSpan> {
    if (parts.size() != 8) return std::nullopt;
    std::array<double, 8> values{};
    for (std::size_t i = 0; i < parts.size(); ++i) {
      const auto parsed = parse_double(parts[i]);
      if (!parsed.has_value()) return std::nullopt;
      values[i] = *parsed;
    }
    return MakeBezier({values[0], values[1]}, {values[2], values[3]},
                      {values[4], values[5]}, {values[6], values[7]});
  };
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    const std::size_t eq = line.find('=');
    if (eq == std::string::npos) {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "road archive line lacks key");
    }
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    std::vector<std::string_view> parts{};
    std::string_view view(value);
    while (true) {
      const std::size_t comma = view.find(',');
      parts.push_back(view.substr(0, comma));
      if (comma == std::string_view::npos) {
        break;
      }
      view.remove_prefix(comma + 1);
    }
    if (key == "road_graph_version") {
      if (value != "5") {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "unknown road graph version");
      }
    } else if (key == "next_id") {
      const std::optional<std::uint64_t> parsed = parse_u64(value);
      if (!parsed.has_value()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid next_id");
      }
      state.next_id_ = *parsed;
    } else if (key == "section_template") {
      const auto id = parse_u64(value);
      if (!id.has_value()) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid section template row");
      state.graph_.section_templates.push_back(CrossSectionTemplate{*id});
    } else if (key == "surface_band") {
      if (parts.size() != 6) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid surface band row");
      const auto template_id = parse_u64(parts[0]);
      const auto element_id = parse_u64(parts[1]);
      const auto role = parse_u64(parts[2]);
      const auto width = parse_double(parts[3]);
      const auto slope = parse_double(parts[4]);
      if (!template_id || !element_id || !role || !width || !slope) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid surface band value");
      }
      auto* section = const_cast<CrossSectionTemplate*>(find_template(state.graph_, *template_id));
      if (section == nullptr) return Result<RoadState>::Fail(ErrorKind::kValidation, "surface band template is missing");
      section->bands.push_back(SurfaceBand{*element_id, static_cast<SurfaceRole>(*role), *width, *slope,
                                           std::string(parts[5])});
    } else if (key == "boundary") {
      if (parts.size() != 6) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid boundary row");
      const auto template_id = parse_u64(parts[0]);
      const auto boundary_id = parse_u64(parts[1]);
      const auto role = parse_u64(parts[2]);
      const auto width = parse_double(parts[3]);
      const auto height = parse_double(parts[4]);
      const auto marking = parse_u64(parts[5]);
      if (!template_id || !boundary_id || !role || !width || !height || !marking) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid boundary value");
      }
      auto* section = const_cast<CrossSectionTemplate*>(find_template(state.graph_, *template_id));
      if (section == nullptr) return Result<RoadState>::Fail(ErrorKind::kValidation, "boundary template is missing");
      section->boundaries.push_back(BoundaryProfile{*boundary_id, static_cast<BoundaryRole>(*role), *width, *height,
                                                     static_cast<MarkingRule>(*marking)});
    } else if (key == "transition") {
      if (parts.size() != 8) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid transition row");
      std::array<std::optional<std::uint64_t>, 6> ints{parse_u64(parts[0]), parse_u64(parts[1]), parse_u64(parts[2]),
                                                       parse_u64(parts[3]), parse_u64(parts[5]), parse_u64(parts[7])};
      const auto start_value = parse_double(parts[4]);
      const auto end_value = parse_double(parts[6]);
      if (std::any_of(ints.begin(), ints.end(), [](const auto& item) { return !item.has_value(); }) ||
          !start_value || !end_value) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid transition value");
      }
      state.graph_.transitions.push_back(SectionTransition{*ints[0], *ints[1], *ints[2],
                                                            StationRef{static_cast<StationRefKind>(*ints[3]), *start_value},
                                                            StationRef{static_cast<StationRefKind>(*ints[4]), *end_value},
                                                            static_cast<TransitionAnchor>(*ints[5]), {}});
    } else if (key == "transition_rule") {
      if (parts.size() != 3) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid transition rule row");
      const auto transition_id = parse_u64(parts[0]);
      const auto element_id = parse_u64(parts[1]);
      const auto action = parse_u64(parts[2]);
      if (!transition_id || !element_id || !action) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid transition rule value");
      }
      auto transition = std::find_if(state.graph_.transitions.begin(), state.graph_.transitions.end(),
                                     [transition_id](const SectionTransition& item) { return item.id == *transition_id; });
      if (transition == state.graph_.transitions.end()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "transition rule owner is missing");
      }
      transition->rules.push_back(SectionTransitionRule{*element_id, static_cast<TransitionAction>(*action)});
    } else if (key == "node") {
      if (parts.size() != 3) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid node row");
      }
      const auto id = parse_u64(parts[0]);
      const auto x = parse_double(parts[1]);
      const auto y = parse_double(parts[2]);
      if (!id || !x || !y) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid node value");
      state.graph_.nodes.push_back(RoadNode{*id, {*x, *y}});
    } else if (key == "connection_policy_override") {
      if (parts.size() != 3) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid connection policy row");
      const auto id = parse_u64(parts[0]);
      const auto node = parse_u64(parts[1]);
      const auto policy = parse_u64(parts[2]);
      if (!id || !node || !policy) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid connection policy value");
      }
      state.graph_.connection_policy_overrides.push_back(
          NodeConnectionPolicyOverride{*id, *node, static_cast<NodeConnectionPolicy>(*policy)});
    } else if (key == "junction") {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "legacy junction authority is unsupported");
    } else if (key == "segment") {
      if (parts.size() != 6) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment row");
      }
      const auto id = parse_u64(parts[0]);
      const auto node_a = parse_u64(parts[1]);
      const auto node_b = parse_u64(parts[2]);
      const auto section = parse_u64(parts[3]);
      const auto transition = parse_u64(parts[4]);
      const auto knot_count = parse_u64(parts[5]);
      if (!id || !node_a || !node_b || !section || !transition || !knot_count) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment value");
      }
      state.graph_.segments.push_back(RoadSegment{*id, *node_a, *node_b, {}, *section,
                                                   *transition == 0 ? std::nullopt
                                                                    : std::optional<SectionTransitionId>(*transition)});
      current_segment = &state.graph_.segments.back();
      expected_segment_knots.push_back({*id, static_cast<std::size_t>(*knot_count)});
    } else if (key == "segment_shape") {
      if (current_segment == nullptr || parts.size() != 4) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment shape row");
      }
      const auto sx = parse_double(parts[0]);
      const auto sy = parse_double(parts[1]);
      const auto ex = parse_double(parts[2]);
      const auto ey = parse_double(parts[3]);
      if (!sx || !sy || !ex || !ey) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment shape value");
      }
      current_segment->shape.start_handle = {*sx, *sy};
      current_segment->shape.end_handle = {*ex, *ey};
      if (!segment_shapes.insert(current_segment->id).second) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "duplicate segment shape row");
      }
    } else if (key == "segment_knot") {
      if (current_segment == nullptr || parts.size() != 6) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment knot row");
      }
      std::array<std::optional<double>, 6> values{};
      for (std::size_t i = 0; i < values.size(); ++i) values[i] = parse_double(parts[i]);
      if (std::any_of(values.begin(), values.end(), [](const auto& value) { return !value.has_value(); })) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment knot value");
      }
      current_segment->shape.internal_knots.push_back(
          SegmentKnot{{*values[0], *values[1]}, {*values[2], *values[3]}, {*values[4], *values[5]}});
    } else if (key == "primitive" || key == "span") {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "legacy road segment persistence is unsupported");
    } else if (key == "manual_line") {
      if (parts.size() != 4) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual line row");
      const auto id = parse_u64(parts[0]);
      const auto owner = parse_u64(parts[1]);
      const auto span_count = parse_u64(parts[3]);
      if (!id || !owner || !span_count) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual line value");
      }
      state.graph_.manual_lines.push_back(ManualLineMarking{*id, *owner, {}, std::string(parts[2])});
      current_manual_line = &state.graph_.manual_lines.back();
      expected_manual_spans.push_back({*id, static_cast<std::size_t>(*span_count)});
    } else if (key == "manual_primitive") {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "legacy manual marking persistence is unsupported");
    } else if (key == "manual_span") {
      const auto span = span_from_parts(parts);
      if (current_manual_line == nullptr || !span.has_value()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual span row");
      }
      current_manual_line->path.spans.push_back(*span);
    } else if (key == "manual_area") {
      if (parts.size() != 7) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual area row");
      const auto id = parse_u64(parts[0]);
      const auto owner = parse_u64(parts[1]);
      const auto x = parse_double(parts[2]);
      const auto y = parse_double(parts[3]);
      const auto width = parse_double(parts[4]);
      const auto length = parse_double(parts[5]);
      if (!id || !owner || !x || !y || !width || !length) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual area value");
      }
      state.graph_.manual_areas.push_back(
          ManualAreaMarking{*id, *owner, {*x, *y}, *width, *length, std::string(parts[6])});
    } else {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "unknown road archive key");
    }
  }
  for (const auto& [segment_id, expected] : expected_segment_knots) {
    const RoadSegment* segment = find_segment(state.graph_, segment_id);
    if (segment == nullptr || !segment_shapes.contains(segment_id) || segment->shape.internal_knots.size() != expected) {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "road segment shape rows are truncated or excessive");
    }
  }
  for (const auto& [marking_id, expected] : expected_manual_spans) {
    const auto marking = std::find_if(state.graph_.manual_lines.begin(), state.graph_.manual_lines.end(),
                                      [marking_id](const auto& item) { return item.id == marking_id; });
    if (marking == state.graph_.manual_lines.end() || marking->path.spans.size() != expected) {
      return Result<RoadState>::Fail(ErrorKind::kValidation, "manual line rows are truncated or excessive");
    }
  }
  const Result<bool> rebuilt = state.BuildDerived();
  if (!rebuilt.ok) {
    return Result<RoadState>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return Result<RoadState>::Ok(state);
}

Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived) {
  if (std::any_of(derived.build_stage_runs.begin(), derived.build_stage_runs.end(),
                  [](std::size_t runs) { return runs != 1; })) {
    return Result<bool>::Fail(ErrorKind::kInternal, "road build stage count invariant failed");
  }
  std::unordered_set<std::uint64_t> ids{};
  for (const CrossSectionTemplate& section : graph.section_templates) {
    if (!ids.insert(section.id).second) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road section template ID invariant failed");
    }
  }
  for (const RoadNode& node : graph.nodes) {
    if (!ids.insert(node.id).second || !finite(node.position)) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road node invariant failed");
    }
  }
  for (const RoadSegment& segment : graph.segments) {
    if (!ids.insert(segment.id).second || find_node(graph, segment.node_a) == nullptr ||
        find_node(graph, segment.node_b) == nullptr || find_template(graph, segment.section_template) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road segment reference invariant failed");
    }
  }
  for (const SectionEvaluation& section : derived.section_evaluations) {
    double previous = -std::numeric_limits<double>::infinity();
    for (const SectionBoundarySample& boundary : section.boundaries) {
      if (!finite(boundary.lateral_m) || !finite(boundary.height_m) || boundary.lateral_m < previous) {
        return Result<bool>::Fail(ErrorKind::kInternal, "section boundary order invariant failed");
      }
      previous = boundary.lateral_m;
    }
  }
  for (const NodeConnectionPolicyOverride& policy : graph.connection_policy_overrides) {
    if (!ids.insert(policy.id).second || find_node(graph, policy.node_id) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road connection policy override invariant failed");
    }
  }
  for (const SectionTransition& transition : graph.transitions) {
    if (!ids.insert(transition.id).second) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road transition ID invariant failed");
    }
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "manual line ID or owner invariant failed");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "manual area ID or owner invariant failed");
    }
  }
  const auto valid_mesh = [](const Mesh& mesh) {
    for (const Vec3d& vertex : mesh.vertices) {
      if (!finite(vertex.x) || !finite(vertex.y) || !finite(vertex.z)) {
        return false;
      }
    }
    for (std::uint32_t index : mesh.indices) {
      if (index >= mesh.vertices.size()) return false;
    }
    return true;
  };
  const std::array<const std::vector<Mesh>*, 6> mesh_families{
      &derived.segment_meshes, &derived.marking_meshes, &derived.connection_meshes,
      &derived.junction_meshes, &derived.junction_marking_meshes, &derived.manual_marking_meshes};
  for (const auto* family : mesh_families) {
    if (std::any_of(family->begin(), family->end(), [&valid_mesh](const Mesh& mesh) { return !valid_mesh(mesh); })) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road mesh contains invalid geometry");
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> ValidatePath(const Path& path) {
  if (path.spans.empty()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road path has no spans");
  }
  for (std::size_t i = 0; i < path.spans.size(); ++i) {
    const BezierSpan& span = path.spans[i];
    if (!finite(span.p0) || !finite(span.p1) || !finite(span.p2) || !finite(span.p3)) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road path contains a non-finite control point");
    }
    if (span_length(span) <= kEpsilon) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road span has zero length");
    }
    if (i > 0 && !almost_same(span_start(span), span_end(path.spans[i - 1]))) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road path spans are not continuous");
    }
  }
  return validate_no_self_intersection(path);
}

Result<double> PathLength(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) {
    return Result<double>::Fail(valid.error_kind, valid.error);
  }
  double out = 0.0;
  for (const BezierSpan& span : path.spans) {
    out += span_length(span);
  }
  return Result<double>::Ok(out);
}

Result<Vec2d> EvaluatePath(const Path& path, double station_m) {
  const Result<double> length_result = PathLength(path);
  if (!length_result.ok) {
    return Result<Vec2d>::Fail(length_result.error_kind, length_result.error);
  }
  double remaining = std::clamp(station_m, 0.0, length_result.value);
  for (const BezierSpan& span : path.spans) {
    const double len = span_length(span);
    if (remaining <= len || &span == &path.spans.back()) {
      return Result<Vec2d>::Ok(span_eval(span, span_parameter_at_length(span, remaining)));
    }
    remaining -= len;
  }
  return Result<Vec2d>::Fail(ErrorKind::kInternal, "road path evaluation fell through");
}

std::vector<Vec2d> FlattenPath(const Path& path) {
  std::vector<Vec2d> points{};
  for (const BezierSpan& span : path.spans) {
    const int samples = IsLinearSpan(span) ? 1 : kCurveSamples;
    if (points.empty()) {
      points.push_back(span_start(span));
    }
    for (int i = 1; i <= samples; ++i) {
      points.push_back(span_eval(span, static_cast<double>(i) / samples));
    }
  }
  return points;
}

} // namespace city::road
