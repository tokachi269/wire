#include "city/road/road.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
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

[[nodiscard]] bool point_on_line_segment(Vec2d point, Vec2d a, Vec2d b) {
  const Vec2d ab = sub(b, a);
  const double len2 = dot(ab, ab);
  if (len2 <= kEpsilon) {
    return false;
  }
  const double t = dot(sub(point, a), ab) / len2;
  if (t <= 1e-6 || t >= 1.0 - 1e-6) {
    return false;
  }
  const Vec2d closest = add(a, mul(ab, t));
  return distance(point, closest) <= 1e-5;
}

[[nodiscard]] Vec2d primitive_start(const Primitive& primitive) {
  if (primitive.kind == Primitive::Kind::kArc) {
    return {primitive.center.x + std::cos(primitive.start_angle_rad) * primitive.radius,
            primitive.center.y + std::sin(primitive.start_angle_rad) * primitive.radius};
  }
  return primitive.p0;
}

[[nodiscard]] Vec2d primitive_end(const Primitive& primitive) {
  if (primitive.kind == Primitive::Kind::kArc) {
    const double angle = primitive.start_angle_rad + primitive.sweep_angle_rad;
    return {primitive.center.x + std::cos(angle) * primitive.radius,
            primitive.center.y + std::sin(angle) * primitive.radius};
  }
  return primitive.kind == Primitive::Kind::kBezier ? primitive.p3 : primitive.p1;
}

[[nodiscard]] Vec2d primitive_eval(const Primitive& primitive, double t) {
  t = std::clamp(t, 0.0, 1.0);
  if (primitive.kind == Primitive::Kind::kLine) {
    return add(mul(primitive.p0, 1.0 - t), mul(primitive.p1, t));
  }
  if (primitive.kind == Primitive::Kind::kArc) {
    const double angle = primitive.start_angle_rad + primitive.sweep_angle_rad * t;
    return {primitive.center.x + std::cos(angle) * primitive.radius,
            primitive.center.y + std::sin(angle) * primitive.radius};
  }
  const double u = 1.0 - t;
  return add(add(mul(primitive.p0, u * u * u), mul(primitive.p1, 3.0 * u * u * t)),
             add(mul(primitive.p2, 3.0 * u * t * t), mul(primitive.p3, t * t * t)));
}

[[nodiscard]] double primitive_length(const Primitive& primitive) {
  if (primitive.kind == Primitive::Kind::kLine) {
    return distance(primitive.p0, primitive.p1);
  }
  if (primitive.kind == Primitive::Kind::kArc) {
    return std::abs(primitive.radius * primitive.sweep_angle_rad);
  }
  double out = 0.0;
  Vec2d prev = primitive.p0;
  for (int i = 1; i <= kCurveSamples; ++i) {
    const Vec2d current = primitive_eval(primitive, static_cast<double>(i) / kCurveSamples);
    out += distance(prev, current);
    prev = current;
  }
  return out;
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

[[nodiscard]] const JunctionDefinition* find_junction(const SavedRoadGraph& graph, RoadNodeId node_id) {
  const auto it = std::find_if(graph.junctions.begin(), graph.junctions.end(),
                               [node_id](const JunctionDefinition& item) { return item.node_id == node_id; });
  return it == graph.junctions.end() ? nullptr : &*it;
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

[[nodiscard]] std::vector<double> stations_for_segment_mesh(const SavedRoadGraph& graph,
                                                            const RoadSegment& segment) {
  const Result<double> length_result = PathLength(segment.alignment);
  if (!length_result.ok) return {};
  const double total = length_result.value;
  const JunctionDefinition* start_junction = find_junction(graph, segment.node_a);
  const JunctionDefinition* end_junction = find_junction(graph, segment.node_b);
  const double start = start_junction == nullptr ? 0.0 : start_junction->corner_radius_m;
  const double end = total - (end_junction == nullptr ? 0.0 : end_junction->corner_radius_m);
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
  return primitive_start(path.primitives.front());
}

[[nodiscard]] Vec2d path_end(const Path& path) {
  return primitive_end(path.primitives.back());
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

[[nodiscard]] std::vector<Mesh> build_surface_meshes(const SavedRoadGraph& graph, const RoadSegment& segment) {
  const std::vector<double> stations = stations_for_segment_mesh(graph, segment);
  if (stations.empty()) {
    return {};
  }
  const double total = PathLength(segment.alignment).value;
  std::uint32_t width = 0;
  std::vector<Vec3d> vertices{};
  std::vector<std::string> materials{};
  for (double station : stations) {
    const auto section = evaluate_segment_section(graph, segment, station, total);
    const auto section_template = evaluate_segment_template(graph, segment, station, total);
    const Result<Vec2d> center = EvaluatePath(segment.alignment, station);
    const std::optional<Vec2d> tangent = path_tangent(segment.alignment, station, total);
    if (!section.ok || !section_template.ok || section.value.size() < 2 || !center.ok || !tangent.has_value()) {
      return {};
    }
    const std::vector<std::string> row_materials = surface_materials(section_template.value);
    if (row_materials.size() + 1 != section.value.size()) return {};
    if (materials.empty()) materials = row_materials;
    if (materials != row_materials) return {};
    if (width == 0) {
      width = static_cast<std::uint32_t>(section.value.size());
    } else if (width != section.value.size()) {
      return {};
    }
    const Vec2d right{-tangent->y, tangent->x};
    for (const SectionBoundarySample& boundary : section.value) {
      const Vec2d p = add(center.value, mul(right, boundary.lateral_m));
      vertices.push_back({p.x, p.y, boundary.height_m});
    }
  }
  std::vector<Mesh> meshes{};
  for (const std::string& material : materials) {
    if (std::any_of(meshes.begin(), meshes.end(), [&material](const Mesh& mesh) { return mesh.material == material; })) {
      continue;
    }
    Mesh mesh{};
    mesh.owner_segment_id = segment.id;
    mesh.material = material;
    mesh.vertices = vertices;
    for (std::uint32_t row = 0; row + 1 < stations.size(); ++row) {
      for (std::uint32_t col = 0; col < materials.size(); ++col) {
        if (materials[col] != material) continue;
        const std::uint32_t a = row * width + col;
        const std::uint32_t b = a + 1;
        const std::uint32_t c = (row + 1) * width + col;
        const std::uint32_t d = c + 1;
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
      }
    }
    meshes.push_back(std::move(mesh));
  }
  return meshes;
}

[[nodiscard]] TerrainMaskPolygon build_terrain_mask(const SavedRoadGraph& graph, const RoadSegment& segment) {
  TerrainMaskPolygon mask{};
  mask.segment_id = segment.id;
  const std::vector<double> stations = stations_for_segment_mesh(graph, segment);
  if (stations.empty()) {
    return mask;
  }
  const double total = PathLength(segment.alignment).value;
  std::vector<Vec2d> right_side{};
  std::vector<Vec2d> left_side{};
  for (double station : stations) {
    const auto section = evaluate_segment_section(graph, segment, station, total);
    const Result<Vec2d> center = EvaluatePath(segment.alignment, station);
    const std::optional<Vec2d> tangent = path_tangent(segment.alignment, station, total);
    if (!section.ok || section.value.size() < 2 || !center.ok || !tangent.has_value()) {
      return {};
    }
    const Vec2d lateral{-tangent->y, tangent->x};
    left_side.push_back(add(center.value, mul(lateral, section.value.front().lateral_m)));
    right_side.push_back(add(center.value, mul(lateral, section.value.back().lateral_m)));
  }
  mask.points = left_side;
  for (auto it = right_side.rbegin(); it != right_side.rend(); ++it) {
    mask.points.push_back(*it);
  }
  return mask;
}

[[nodiscard]] Mesh build_marking_mesh(const SavedRoadGraph& graph, const RoadSegment& segment) {
  Mesh mesh{};
  const std::vector<double> stations = stations_for_segment_mesh(graph, segment);
  if (stations.empty()) {
    return mesh;
  }
  const double total = PathLength(segment.alignment).value;
  const auto first_section = evaluate_segment_section(graph, segment, stations.front(), total);
  if (!first_section.ok) {
    return mesh;
  }
  for (std::size_t boundary_index = 0; boundary_index < first_section.value.size(); ++boundary_index) {
    const MarkingRule rule = first_section.value[boundary_index].marking_rule;
    if (rule == MarkingRule::kNone) continue;
    const double half_width = rule == MarkingRule::kCenterLine ? 0.06 : 0.05;
    const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
    for (double station : stations) {
      const auto section = evaluate_segment_section(graph, segment, station, total);
      const Result<Vec2d> center = EvaluatePath(segment.alignment, station);
      const std::optional<Vec2d> tangent = path_tangent(segment.alignment, station, total);
      if (!section.ok || boundary_index >= section.value.size() || !center.ok || !tangent.has_value()) {
        return {};
      }
      const SectionBoundarySample& boundary = section.value[boundary_index];
      const Vec2d lateral{-tangent->y, tangent->x};
      const Vec2d a = add(center.value, mul(lateral, boundary.lateral_m - half_width));
      const Vec2d b = add(center.value, mul(lateral, boundary.lateral_m + half_width));
      mesh.vertices.push_back({a.x, a.y, boundary.height_m + 0.02});
      mesh.vertices.push_back({b.x, b.y, boundary.height_m + 0.02});
    }
    const std::uint32_t row_count = static_cast<std::uint32_t>(stations.size());
    for (std::uint32_t row = 0; row + 1 < row_count; ++row) {
      const std::uint32_t a = base + row * 2;
      const std::uint32_t b = a + 1;
      const std::uint32_t c = a + 2;
      const std::uint32_t d = a + 3;
      mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
    }
  }
  return mesh;
}

[[nodiscard]] std::optional<Vec3d> owner_local_point(const RoadSegment& owner, Vec2d local, double z) {
  const auto total_result = PathLength(owner.alignment);
  if (!total_result.ok || local.x < 0.0 || local.x > total_result.value) return std::nullopt;
  const auto center = EvaluatePath(owner.alignment, local.x);
  const auto tangent = path_tangent(owner.alignment, local.x, total_result.value);
  if (!center.ok || !tangent.has_value()) return std::nullopt;
  const Vec2d lateral{-tangent->y, tangent->x};
  const Vec2d world = add(center.value, mul(lateral, local.y));
  return Vec3d{world.x, world.y, z};
}

[[nodiscard]] Mesh build_manual_line_mesh(const RoadSegment& owner, const ManualLineMarking& marking) {
  Mesh mesh{};
  const std::vector<Vec2d> points = FlattenPath(marking.path);
  constexpr double half_width = 0.05;
  for (std::size_t i = 0; i < points.size(); ++i) {
    Vec2d direction = i + 1 < points.size() ? sub(points[i + 1], points[i]) : sub(points[i], points[i - 1]);
    direction = normalize(direction);
    const Vec2d normal{-direction.y, direction.x};
    const auto a = owner_local_point(owner, add(points[i], mul(normal, -half_width)), 0.02);
    const auto b = owner_local_point(owner, add(points[i], mul(normal, half_width)), 0.02);
    if (!a.has_value() || !b.has_value()) return {};
    mesh.vertices.push_back(*a);
    mesh.vertices.push_back(*b);
  }
  for (std::uint32_t row = 0; row + 1 < points.size(); ++row) {
    const std::uint32_t a = row * 2;
    mesh.indices.insert(mesh.indices.end(), {a, a + 2, a + 1, a + 1, a + 2, a + 3});
  }
  return mesh;
}

[[nodiscard]] Mesh build_manual_area_mesh(const RoadSegment& owner, const ManualAreaMarking& marking) {
  Mesh mesh{};
  const double hw = marking.width_m * 0.5;
  const double hl = marking.length_m * 0.5;
  for (Vec2d local : std::array<Vec2d, 4>{Vec2d{marking.frame_origin.x - hl, marking.frame_origin.y - hw},
                                           Vec2d{marking.frame_origin.x + hl, marking.frame_origin.y - hw},
                                           Vec2d{marking.frame_origin.x - hl, marking.frame_origin.y + hw},
                                           Vec2d{marking.frame_origin.x + hl, marking.frame_origin.y + hw}}) {
    const auto point = owner_local_point(owner, local, 0.025);
    if (!point.has_value()) return {};
    mesh.vertices.push_back(*point);
  }
  mesh.indices = {0, 1, 2, 1, 3, 2};
  return mesh;
}

[[nodiscard]] std::pair<double, double> carriageway_edges(const ConnectionGate& gate) {
  struct Run {
    std::size_t first = 0;
    std::size_t last = 0;
  };
  std::vector<Run> curb_runs{};
  for (std::size_t i = 0; i < gate.boundaries.size(); ++i) {
    if (gate.boundaries[i].role != BoundaryRole::kCurb) continue;
    if (curb_runs.empty() || gate.boundaries[curb_runs.back().last].boundary_id != gate.boundaries[i].boundary_id) {
      curb_runs.push_back(Run{i, i});
    } else {
      curb_runs.back().last = i;
    }
  }
  if (curb_runs.size() >= 2) {
    return {gate.boundaries[curb_runs.front().last].lateral_m,
            gate.boundaries[curb_runs.back().first].lateral_m};
  }
  return {gate.boundaries.front().lateral_m, gate.boundaries.back().lateral_m};
}

[[nodiscard]] std::vector<Vec2d> convex_hull(std::vector<Vec2d> points) {
  std::sort(points.begin(), points.end(), [](Vec2d a, Vec2d b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  points.erase(std::unique(points.begin(), points.end(), [](Vec2d a, Vec2d b) { return almost_same(a, b); }),
               points.end());
  if (points.size() < 3) return {};
  std::vector<Vec2d> hull{};
  for (Vec2d point : points) {
    while (hull.size() >= 2 && cross(sub(hull.back(), hull[hull.size() - 2]), sub(point, hull.back())) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size &&
           cross(sub(hull.back(), hull[hull.size() - 2]), sub(*it, hull.back())) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  hull.pop_back();
  return hull;
}

[[nodiscard]] Mesh build_junction_mesh(const JunctionArea& area) {
  std::vector<Vec2d> edge_points{};
  for (const ConnectionGate& gate : area.gates) {
    const auto [left, right] = carriageway_edges(gate);
    const Vec2d center{gate.position.x, gate.position.y};
    const Vec2d lateral{-gate.tangent.y, gate.tangent.x};
    edge_points.push_back(add(center, mul(lateral, left)));
    edge_points.push_back(add(center, mul(lateral, right)));
  }
  const std::vector<Vec2d> hull = convex_hull(std::move(edge_points));
  Mesh mesh{};
  mesh.material = "asphalt";
  if (hull.size() < 3) return mesh;
  Vec2d center{};
  for (Vec2d point : hull) center = add(center, point);
  center = mul(center, 1.0 / hull.size());
  mesh.vertices.push_back({center.x, center.y, 0.0});
  for (Vec2d point : hull) mesh.vertices.push_back({point.x, point.y, 0.0});
  for (std::uint32_t i = 0; i < hull.size(); ++i) {
    mesh.indices.insert(mesh.indices.end(), {0, i + 1, static_cast<std::uint32_t>((i + 1) % hull.size()) + 1});
  }
  return mesh;
}

void append_quad(Mesh& mesh, Vec2d center, Vec2d tangent, double half_length, double left, double right) {
  const Vec2d lateral{-tangent.y, tangent.x};
  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  for (Vec2d point : std::array<Vec2d, 4>{
           add(add(center, mul(tangent, -half_length)), mul(lateral, left)),
           add(add(center, mul(tangent, half_length)), mul(lateral, left)),
           add(add(center, mul(tangent, -half_length)), mul(lateral, right)),
           add(add(center, mul(tangent, half_length)), mul(lateral, right))}) {
    mesh.vertices.push_back({point.x, point.y, 0.025});
  }
  mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base + 1, base + 3, base + 2});
}

[[nodiscard]] std::vector<Mesh> build_junction_markings(const JunctionArea& area) {
  std::vector<Mesh> meshes{};
  for (const ConnectionGate& gate : area.gates) {
    const auto [left, right] = carriageway_edges(gate);
    const Vec2d tangent{gate.tangent.x, gate.tangent.y};
    const Vec2d gate_center{gate.position.x, gate.position.y};
    Mesh stop{};
    stop.material = "marking";
    append_quad(stop, add(gate_center, mul(tangent, 0.35)), tangent, 0.08, left, right);
    meshes.push_back(std::move(stop));

    Mesh zebra{};
    zebra.material = "marking";
    for (int stripe = 0; stripe < 5; ++stripe) {
      append_quad(zebra, add(gate_center, mul(tangent, 0.9 + stripe * 0.55)), tangent, 0.16, left, right);
    }
    meshes.push_back(std::move(zebra));
  }
  return meshes;
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

[[nodiscard]] std::string primitive_kind_name(Primitive::Kind kind) {
  if (kind == Primitive::Kind::kLine) {
    return "line";
  }
  if (kind == Primitive::Kind::kArc) {
    return "arc";
  }
  return "bezier";
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

Primitive MakeLine(Vec2d a, Vec2d b) {
  Primitive out{};
  out.kind = Primitive::Kind::kLine;
  out.p0 = a;
  out.p1 = b;
  return out;
}

Primitive MakeArc(Vec2d center, double radius, double start_angle_rad, double sweep_angle_rad) {
  Primitive out{};
  out.kind = Primitive::Kind::kArc;
  out.center = center;
  out.radius = radius;
  out.start_angle_rad = start_angle_rad;
  out.sweep_angle_rad = sweep_angle_rad;
  return out;
}

Result<Primitive> MakeArcThroughPoints(Vec2d start, Vec2d through, Vec2d end) {
  if (!finite(start) || !finite(through) || !finite(end)) {
    return Result<Primitive>::Fail(ErrorKind::kValidation, "road arc point is not finite");
  }
  const double determinant = 2.0 * (start.x * (through.y - end.y) + through.x * (end.y - start.y) +
                                    end.x * (start.y - through.y));
  if (std::abs(determinant) <= 1e-8) {
    return Result<Primitive>::Fail(ErrorKind::kUnsupported, "collinear road arc points are unsupported");
  }
  const double start_norm = dot(start, start);
  const double through_norm = dot(through, through);
  const double end_norm = dot(end, end);
  const Vec2d center{
      (start_norm * (through.y - end.y) + through_norm * (end.y - start.y) +
       end_norm * (start.y - through.y)) /
          determinant,
      (start_norm * (end.x - through.x) + through_norm * (start.x - end.x) +
       end_norm * (through.x - start.x)) /
          determinant};
  const double radius = distance(center, start);
  const double start_angle = std::atan2(start.y - center.y, start.x - center.x);
  const double through_angle = std::atan2(through.y - center.y, through.x - center.x);
  const double end_angle = std::atan2(end.y - center.y, end.x - center.x);
  constexpr double two_pi = 6.28318530717958647692;
  const auto ccw_delta = [two_pi](double from, double to) {
    double delta = std::fmod(to - from, two_pi);
    if (delta < 0.0) delta += two_pi;
    return delta;
  };
  double sweep = ccw_delta(start_angle, end_angle);
  if (ccw_delta(start_angle, through_angle) > sweep) sweep -= two_pi;
  return Result<Primitive>::Ok(MakeArc(center, radius, start_angle, sweep));
}

Primitive MakeBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3) {
  Primitive out{};
  out.kind = Primitive::Kind::kBezier;
  out.p0 = p0;
  out.p1 = p1;
  out.p2 = p2;
  out.p3 = p3;
  return out;
}

Path MakePath(std::vector<Primitive> primitives) {
  Path out{};
  out.primitives = std::move(primitives);
  return out;
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

RoadState::RoadState() {
  graph_.section_templates.push_back(JapaneseUrbanTwoLaneTemplate(1));
  next_id_ = 2;
}

const SavedRoadGraph& RoadState::graph() const {
  return graph_;
}

const DerivedRoad& RoadState::derived() const {
  return derived_;
}

Result<RoadSegmentId> RoadState::AddSegment(Path alignment, CrossSectionTemplateId section_template) {
  const Result<bool> valid = ValidatePath(alignment);
  if (!valid.ok) {
    return Result<RoadSegmentId>::Fail(valid.error_kind, valid.error);
  }
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment references a missing section template");
  }
  const RoadNodeId node_a = next_id_++;
  const RoadNodeId node_b = next_id_++;
  const RoadSegmentId segment_id = next_id_++;
  graph_.nodes.push_back(RoadNode{node_a, path_start(alignment)});
  graph_.nodes.push_back(RoadNode{node_b, path_end(alignment)});
  graph_.segments.push_back(RoadSegment{segment_id, node_a, node_b, alignment, section_template, std::nullopt});
  const Result<bool> rebuilt = RebuildDerived();
  if (!rebuilt.ok) {
    return Result<RoadSegmentId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedTo(Path alignment, CrossSectionTemplateId section_template,
                                                       RoadNodeId start_node) {
  const RoadNode* node = find_node(graph_, start_node);
  if (node == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment start node does not exist");
  }
  if (!almost_same(path_start(alignment), node->position)) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "connected segment path does not start at node");
  }
  const Result<double> length_result = PathLength(alignment);
  if (!length_result.ok) {
    return Result<RoadSegmentId>::Fail(length_result.error_kind, length_result.error);
  }
  if (length_result.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment is shorter than P1 minimum");
  }
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
    const Vec2d other = existing.node_a == start_node ? path_end(existing.alignment) : path_start(existing.alignment);
    const double angle = angle_deg(sub(other, node->position), new_direction);
    const bool opposite_continuation = angle >= 180.0 - 1e-6;
    if (!opposite_continuation && (angle < kP1MinConnectionAngleDeg || angle > kP1MaxConnectionAngleDeg)) {
      return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment angle is outside P1 range");
    }
  }
  const Result<RoadSegmentId> added = AddSegment(std::move(alignment), section_template);
  if (!added.ok) {
    return added;
  }
  RoadSegment* segment = find_segment(graph_, added.value);
  if (segment == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kInternal, "new road segment disappeared");
  }
  graph_.nodes.erase(std::remove_if(graph_.nodes.begin(), graph_.nodes.end(),
                                    [segment](const RoadNode& candidate) { return candidate.id == segment->node_a; }),
                     graph_.nodes.end());
  segment->node_a = start_node;
  const std::size_t degree = static_cast<std::size_t>(std::count_if(
      graph_.segments.begin(), graph_.segments.end(), [start_node](const RoadSegment& item) {
        return item.node_a == start_node || item.node_b == start_node;
      }));
  if (degree >= 2 &&
      std::none_of(graph_.junctions.begin(), graph_.junctions.end(),
                   [start_node](const JunctionDefinition& item) { return item.node_id == start_node; })) {
    graph_.junctions.push_back(JunctionDefinition{next_id_++, start_node, 4.0});
  }
  const Result<bool> rebuilt = RebuildDerived();
  if (!rebuilt.ok) {
    return Result<RoadSegmentId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return added;
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedToSegment(Path alignment,
                                                              CrossSectionTemplateId section_template,
                                                              RoadSegmentId start_segment) {
  const RoadSegment* source = find_segment(graph_, start_segment);
  if (source == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment snap target does not exist");
  }
  if (source->transition.has_value()) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported,
                                       "splitting a transitioning road segment is unsupported");
  }
  if (source->alignment.primitives.size() != 1 ||
      source->alignment.primitives.front().kind != Primitive::Kind::kLine) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "road segment split supports straight segments only");
  }
  const Vec2d split_point = path_start(alignment);
  const Vec2d source_start = path_start(source->alignment);
  const Vec2d source_end = path_end(source->alignment);
  if (!point_on_line_segment(split_point, source_start, source_end)) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment snap point is not on the target segment");
  }

  RoadState trial = *this;
  RoadSegment* split = find_segment(trial.graph_, start_segment);
  if (split == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kInternal, "road segment split target disappeared");
  }
  const RoadNodeId old_end = split->node_b;
  const RoadNodeId split_node = trial.next_id_++;
  const RoadSegmentId second_id = trial.next_id_++;
  trial.graph_.nodes.push_back(RoadNode{split_node, split_point});
  split->node_b = split_node;
  split->alignment = MakePath({MakeLine(source_start, split_point)});
  trial.graph_.segments.push_back(RoadSegment{second_id,
                                              split_node,
                                              old_end,
                                              MakePath({MakeLine(split_point, source_end)}),
                                              split->section_template,
                                              split->transition});
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) {
    return Result<RoadSegmentId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  const Result<RoadSegmentId> added = trial.AddSegmentConnectedTo(std::move(alignment), section_template, split_node);
  if (!added.ok) {
    return added;
  }
  *this = std::move(trial);
  return added;
}

Result<bool> RoadState::EditSegmentPath(RoadSegmentId segment_id, Path alignment) {
  const Result<bool> valid = ValidatePath(alignment);
  if (!valid.ok) {
    return valid;
  }
  RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  const RoadNode* node_a = find_node(graph_, segment->node_a);
  const RoadNode* node_b = find_node(graph_, segment->node_b);
  if ((node_degree(graph_, segment->node_a) > 1 && !almost_same(path_start(alignment), node_a->position)) ||
      (node_degree(graph_, segment->node_b) > 1 && !almost_same(path_end(alignment), node_b->position))) {
    return Result<bool>::Fail(ErrorKind::kUnsupported, "connected road edit cannot move a shared endpoint");
  }
  RoadState trial = *this;
  RoadSegment* trial_segment = find_segment(trial.graph_, segment_id);
  trial_segment->alignment = alignment;
  RoadNode* a = find_node(trial.graph_, trial_segment->node_a);
  RoadNode* b = find_node(trial.graph_, trial_segment->node_b);
  if (a != nullptr) {
    a->position = path_start(alignment);
  }
  if (b != nullptr) {
    b->position = path_end(alignment);
  }
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) return rebuilt;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<bool> RoadState::DeleteSegment(RoadSegmentId segment_id) {
  RoadState trial = *this;
  const RoadSegment* target = find_segment(trial.graph_, segment_id);
  if (target == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  const std::optional<SectionTransitionId> transition = target->transition;
  trial.graph_.segments.erase(std::remove_if(trial.graph_.segments.begin(), trial.graph_.segments.end(),
                                       [segment_id](const RoadSegment& item) { return item.id == segment_id; }),
                        trial.graph_.segments.end());
  trial.graph_.manual_lines.erase(std::remove_if(trial.graph_.manual_lines.begin(), trial.graph_.manual_lines.end(),
                                           [segment_id](const ManualLineMarking& item) {
                                             return item.owner_segment_id == segment_id;
                                           }),
                            trial.graph_.manual_lines.end());
  trial.graph_.manual_areas.erase(std::remove_if(trial.graph_.manual_areas.begin(), trial.graph_.manual_areas.end(),
                                           [segment_id](const ManualAreaMarking& item) {
                                             return item.owner_segment_id == segment_id;
                                           }),
                            trial.graph_.manual_areas.end());
  trial.graph_.nodes.erase(std::remove_if(trial.graph_.nodes.begin(), trial.graph_.nodes.end(),
                                           [&trial](const RoadNode& node) {
                                             return node_degree(trial.graph_, node.id) == 0;
                                           }),
                           trial.graph_.nodes.end());
  trial.graph_.junctions.erase(std::remove_if(trial.graph_.junctions.begin(), trial.graph_.junctions.end(),
                                               [&trial](const JunctionDefinition& junction) {
                                                 return node_degree(trial.graph_, junction.node_id) < 2;
                                               }),
                               trial.graph_.junctions.end());
  if (transition.has_value() &&
      std::none_of(trial.graph_.segments.begin(), trial.graph_.segments.end(), [transition](const RoadSegment& segment) {
        return segment.transition == transition;
      })) {
    trial.graph_.transitions.erase(
        std::remove_if(trial.graph_.transitions.begin(), trial.graph_.transitions.end(),
                       [transition](const SectionTransition& item) { return item.id == *transition; }),
        trial.graph_.transitions.end());
  }
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) return rebuilt;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<CrossSectionTemplateId> RoadState::AddSectionTemplate(CrossSectionTemplate section_template) {
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) {
    return Result<CrossSectionTemplateId>::Fail(valid.error_kind, valid.error);
  }
  if (section_template.id == 0) {
    section_template.id = next_id_++;
  }
  if (find_template(graph_, section_template.id) != nullptr) {
    return Result<CrossSectionTemplateId>::Fail(ErrorKind::kValidation, "section template id already exists");
  }
  graph_.section_templates.push_back(std::move(section_template));
  return Result<CrossSectionTemplateId>::Ok(graph_.section_templates.back().id);
}

Result<bool> RoadState::EditSectionTemplate(CrossSectionTemplate section_template) {
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) return valid;
  auto it = std::find_if(graph_.section_templates.begin(), graph_.section_templates.end(),
                         [&section_template](const CrossSectionTemplate& item) { return item.id == section_template.id; });
  if (it == graph_.section_templates.end()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section template does not exist");
  }
  RoadState trial = *this;
  auto trial_it = std::find_if(trial.graph_.section_templates.begin(), trial.graph_.section_templates.end(),
                               [&section_template](const CrossSectionTemplate& item) {
                                 return item.id == section_template.id;
                               });
  *trial_it = std::move(section_template);
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) return rebuilt;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<SectionTransitionId> RoadState::AddTransition(SectionTransition transition) {
  if (transition.id == 0) {
    transition.id = next_id_++;
  }
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) {
    return Result<SectionTransitionId>::Fail(valid.error_kind, valid.error);
  }
  graph_.transitions.push_back(std::move(transition));
  return Result<SectionTransitionId>::Ok(graph_.transitions.back().id);
}

Result<bool> RoadState::AttachSectionTransition(RoadSegmentId segment_id, SectionTransitionId transition_id) {
  const RoadSegment* segment = find_segment(graph_, segment_id);
  const SectionTransition* transition = find_transition(graph_, transition_id);
  if (segment == nullptr || transition == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition attachment reference is missing");
  }
  if (segment->section_template != transition->from_template) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition from-template does not match segment");
  }
  const auto total = PathLength(segment->alignment);
  if (!total.ok) return Result<bool>::Fail(total.error_kind, total.error);
  const double start = station_value(transition->start, total.value);
  const double end = station_value(transition->end, total.value);
  if (start < 0.0 || end > total.value || end - start <= kEpsilon) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition station range is invalid");
  }
  RoadState trial = *this;
  find_segment(trial.graph_, segment_id)->transition = transition_id;
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) return rebuilt;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<ManualMarkingId> RoadState::AddManualLine(ManualLineMarking marking) {
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line owner segment does not exist");
  }
  const Result<bool> valid = ValidatePath(marking.path);
  if (!valid.ok) {
    return Result<ManualMarkingId>::Fail(valid.error_kind, valid.error);
  }
  const auto length_result = PathLength(owner->alignment);
  for (const Vec2d point : FlattenPath(marking.path)) {
    if (!length_result.ok || point.x < 0.0 || point.x > length_result.value || !finite(point.y)) {
      return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line lies outside owner station range");
    }
  }
  RoadState trial = *this;
  if (marking.id == 0) {
    marking.id = trial.next_id_++;
  }
  const ManualMarkingId id = marking.id;
  trial.graph_.manual_lines.push_back(std::move(marking));
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) {
    return Result<ManualMarkingId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  *this = std::move(trial);
  return Result<ManualMarkingId>::Ok(id);
}

Result<ManualMarkingId> RoadState::AddManualArea(ManualAreaMarking marking) {
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area owner segment does not exist");
  }
  if (!finite(marking.frame_origin) || !finite(marking.width_m) || !finite(marking.length_m) ||
      marking.width_m <= 0.0 || marking.length_m <= 0.0) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area shape is invalid");
  }
  const auto owner_length = PathLength(owner->alignment);
  if (!owner_length.ok || marking.frame_origin.x - marking.length_m * 0.5 < 0.0 ||
      marking.frame_origin.x + marking.length_m * 0.5 > owner_length.value) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area lies outside owner station range");
  }
  RoadState trial = *this;
  if (marking.id == 0) {
    marking.id = trial.next_id_++;
  }
  const ManualMarkingId id = marking.id;
  trial.graph_.manual_areas.push_back(std::move(marking));
  const Result<bool> rebuilt = trial.RebuildDerived();
  if (!rebuilt.ok) {
    return Result<ManualMarkingId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  *this = std::move(trial);
  return Result<ManualMarkingId>::Ok(id);
}

Result<bool> RoadState::RebuildDerived() {
  DerivedRoad next{};
  for (const RoadSegment& segment : graph_.segments) {
    const CrossSectionTemplate* section = find_template(graph_, segment.section_template);
    if (section == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment section template is missing");
    }
    const auto stations = stations_for_path(segment.alignment);
    if (stations.empty()) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment has no section stations");
    }
    const double total = stations.back();
    for (double station : stations) {
      const auto evaluated = evaluate_segment_section(graph_, segment, station, total);
      if (!evaluated.ok) return Result<bool>::Fail(evaluated.error_kind, evaluated.error);
      next.section_evaluations.push_back(SectionEvaluation{segment.id, station, evaluated.value});
    }
    std::vector<Mesh> surface_meshes = build_surface_meshes(graph_, segment);
    if (surface_meshes.empty()) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road surface materialization produced no meshes");
    }
    next.segment_meshes.insert(next.segment_meshes.end(), std::make_move_iterator(surface_meshes.begin()),
                               std::make_move_iterator(surface_meshes.end()));
    next.marking_meshes.push_back(build_marking_mesh(graph_, segment));
    next.terrain_masks.push_back(build_terrain_mask(graph_, segment));
    for (RoadNodeId node_id : std::array<RoadNodeId, 2>{segment.node_a, segment.node_b}) {
      const RoadNode* node = find_node(graph_, node_id);
      if (node == nullptr) {
        return Result<bool>::Fail(ErrorKind::kValidation, "road segment endpoint node is missing");
      }
      const JunctionDefinition* junction = find_junction(graph_, node_id);
      const double offset = junction == nullptr ? 0.0 : junction->corner_radius_m;
      const double gate_station = node_id == segment.node_a ? offset : total - offset;
      const auto gate_center = EvaluatePath(segment.alignment, gate_station);
      const auto path_direction = path_tangent(segment.alignment, gate_station, total);
      if (!gate_center.ok || !path_direction.has_value()) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road connection gate frame could not be evaluated");
      }
      const Vec2d tangent2 = node_id == segment.node_a ? *path_direction : mul(*path_direction, -1.0);
      const auto gate_section = evaluate_segment_section(graph_, segment, gate_station, total);
      if (!gate_section.ok) return Result<bool>::Fail(gate_section.error_kind, gate_section.error);
      next.connection_gates.push_back(
          ConnectionGate{segment.id, node_id, to3(gate_center.value), to3(tangent2), gate_section.value});
    }
  }
  for (const JunctionDefinition& junction : graph_.junctions) {
    JunctionArea area{};
    area.definition_id = junction.id;
    area.node_id = junction.node_id;
    for (const ConnectionGate& gate : next.connection_gates) {
      if (gate.node_id == junction.node_id) {
        area.gates.push_back(gate);
      }
    }
    if (area.gates.size() < 2) {
      return Result<bool>::Fail(ErrorKind::kInternal, "junction has fewer than two connection gates");
    }
    Mesh junction_mesh = build_junction_mesh(area);
    if (junction_mesh.indices.empty()) {
      return Result<bool>::Fail(ErrorKind::kInternal, "junction surface materialization failed");
    }
    next.junction_meshes.push_back(std::move(junction_mesh));
    std::vector<Mesh> junction_markings = build_junction_markings(area);
    next.junction_marking_meshes.insert(next.junction_marking_meshes.end(),
                                        std::make_move_iterator(junction_markings.begin()),
                                        std::make_move_iterator(junction_markings.end()));
    next.junction_areas.push_back(std::move(area));
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
    if (owner == nullptr) return Result<bool>::Fail(ErrorKind::kValidation, "manual line owner is missing");
    next.manual_marking_meshes.push_back(build_manual_line_mesh(*owner, marking));
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
    if (owner == nullptr) return Result<bool>::Fail(ErrorKind::kValidation, "manual area owner is missing");
    next.manual_marking_meshes.push_back(build_manual_area_mesh(*owner, marking));
  }
  const Result<bool> invariant = ValidateGraphInvariants(graph_, next);
  if (!invariant.ok) {
    return invariant;
  }
  derived_ = std::move(next);
  return Result<bool>::Ok(true);
}

Result<std::string> RoadState::Save() const {
  std::ostringstream out;
  out << "road_graph_version=2\n";
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
  for (const JunctionDefinition& junction : graph_.junctions) {
    out << "junction=" << junction.id << "," << junction.node_id << "," << junction.corner_radius_m << "\n";
  }
  for (const RoadSegment& segment : graph_.segments) {
    out << "segment=" << segment.id << "," << segment.node_a << "," << segment.node_b << ","
        << segment.section_template << "," << segment.transition.value_or(0) << ","
        << segment.alignment.primitives.size() << "\n";
    for (const Primitive& primitive : segment.alignment.primitives) {
      out << "primitive=" << primitive_kind_name(primitive.kind) << "," << primitive.p0.x << "," << primitive.p0.y
          << "," << primitive.p1.x << "," << primitive.p1.y << "," << primitive.p2.x << "," << primitive.p2.y << ","
          << primitive.p3.x << "," << primitive.p3.y << "," << primitive.center.x << "," << primitive.center.y << ","
          << primitive.radius << "," << primitive.start_angle_rad << "," << primitive.sweep_angle_rad << "\n";
    }
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    out << "manual_line=" << marking.id << "," << marking.owner_segment_id << "," << marking.style << ","
        << marking.path.primitives.size() << "\n";
    for (const Primitive& primitive : marking.path.primitives) {
      out << "manual_primitive=" << primitive_kind_name(primitive.kind) << "," << primitive.p0.x << ","
          << primitive.p0.y << "," << primitive.p1.x << "," << primitive.p1.y << "," << primitive.p2.x << ","
          << primitive.p2.y << "," << primitive.p3.x << "," << primitive.p3.y << "," << primitive.center.x << ","
          << primitive.center.y << "," << primitive.radius << "," << primitive.start_angle_rad << ","
          << primitive.sweep_angle_rad << "\n";
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    out << "manual_area=" << marking.id << "," << marking.owner_segment_id << "," << marking.frame_origin.x << ","
        << marking.frame_origin.y << "," << marking.width_m << "," << marking.length_m << "," << marking.style << "\n";
  }
  return Result<std::string>::Ok(out.str());
}

Result<RoadState> RoadState::Load(const std::string& text) {
  const bool version1 = text.starts_with("road_graph_version=1\n");
  const bool version2 = text.starts_with("road_graph_version=2\n");
  if (!version1 && !version2) {
    return Result<RoadState>::Fail(ErrorKind::kValidation, "unknown road graph version");
  }
  RoadState state{};
  state.graph_.nodes.clear();
  state.graph_.segments.clear();
  state.graph_.transitions.clear();
  state.graph_.junctions.clear();
  state.graph_.manual_lines.clear();
  state.graph_.manual_areas.clear();
  if (version2) state.graph_.section_templates.clear();
  std::istringstream in(text);
  std::string line;
  RoadSegment* current_segment = nullptr;
  ManualLineMarking* current_manual_line = nullptr;
  const auto primitive_from_parts = [](const std::vector<std::string_view>& parts) -> std::optional<Primitive> {
    if (parts.size() != 14) return std::nullopt;
    std::array<double, 13> values{};
    for (std::size_t i = 1; i < parts.size(); ++i) {
      const auto parsed = parse_double(parts[i]);
      if (!parsed.has_value()) return std::nullopt;
      values[i - 1] = *parsed;
    }
    Primitive primitive{};
    primitive.kind = parts[0] == "arc" ? Primitive::Kind::kArc
                                        : (parts[0] == "bezier" ? Primitive::Kind::kBezier : Primitive::Kind::kLine);
    primitive.p0 = {values[0], values[1]};
    primitive.p1 = {values[2], values[3]};
    primitive.p2 = {values[4], values[5]};
    primitive.p3 = {values[6], values[7]};
    primitive.center = {values[8], values[9]};
    primitive.radius = values[10];
    primitive.start_angle_rad = values[11];
    primitive.sweep_angle_rad = values[12];
    return primitive;
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
      if (value != "1" && value != "2") {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "unknown road graph version");
      }
    } else if (key == "next_id") {
      const std::optional<std::uint64_t> parsed = parse_u64(value);
      if (!parsed.has_value()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid next_id");
      }
      state.next_id_ = *parsed;
    } else if (key == "section_template" && version2) {
      const auto id = parse_u64(value);
      if (!id.has_value()) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid section template row");
      state.graph_.section_templates.push_back(CrossSectionTemplate{*id});
    } else if (key == "surface_band" && version2) {
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
    } else if (key == "boundary" && version2) {
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
    } else if (key == "transition" && version2) {
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
    } else if (key == "transition_rule" && version2) {
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
      state.graph_.nodes.push_back(RoadNode{*parse_u64(parts[0]), {*parse_double(parts[1]), *parse_double(parts[2])}});
    } else if (key == "junction" && version2) {
      if (parts.size() != 3) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid junction row");
      const auto id = parse_u64(parts[0]);
      const auto node = parse_u64(parts[1]);
      const auto radius = parse_double(parts[2]);
      if (!id || !node || !radius) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid junction value");
      state.graph_.junctions.push_back(JunctionDefinition{*id, *node, *radius});
    } else if (key == "segment") {
      if ((version1 && parts.size() != 5) || (version2 && parts.size() != 6)) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment row");
      }
      const auto id = parse_u64(parts[0]);
      const auto node_a = parse_u64(parts[1]);
      const auto node_b = parse_u64(parts[2]);
      const auto section = parse_u64(parts[3]);
      const auto transition = version2 ? parse_u64(parts[4]) : std::optional<std::uint64_t>{0};
      if (!id || !node_a || !node_b || !section || !transition) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment value");
      }
      state.graph_.segments.push_back(RoadSegment{*id, *node_a, *node_b, {}, *section,
                                                   *transition == 0 ? std::nullopt
                                                                    : std::optional<SectionTransitionId>(*transition)});
      current_segment = &state.graph_.segments.back();
    } else if (key == "primitive") {
      const auto primitive = primitive_from_parts(parts);
      if (current_segment == nullptr || !primitive.has_value()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid primitive row");
      }
      current_segment->alignment.primitives.push_back(*primitive);
    } else if (key == "manual_line" && version2) {
      if (parts.size() != 4) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual line row");
      const auto id = parse_u64(parts[0]);
      const auto owner = parse_u64(parts[1]);
      if (!id || !owner) return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual line value");
      state.graph_.manual_lines.push_back(ManualLineMarking{*id, *owner, {}, std::string(parts[2])});
      current_manual_line = &state.graph_.manual_lines.back();
    } else if (key == "manual_primitive" && version2) {
      const auto primitive = primitive_from_parts(parts);
      if (current_manual_line == nullptr || !primitive.has_value()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid manual primitive row");
      }
      current_manual_line->path.primitives.push_back(*primitive);
    } else if (key == "manual_area" && version2) {
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
  const Result<bool> rebuilt = state.RebuildDerived();
  if (!rebuilt.ok) {
    return Result<RoadState>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return Result<RoadState>::Ok(state);
}

Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived) {
  std::unordered_set<std::uint64_t> ids{};
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
  for (const Mesh& mesh : derived.segment_meshes) {
    for (const Vec3d& vertex : mesh.vertices) {
      if (!finite(vertex.x) || !finite(vertex.y) || !finite(vertex.z)) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road mesh contains non-finite vertex");
      }
    }
    for (std::uint32_t index : mesh.indices) {
      if (index >= mesh.vertices.size()) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road mesh contains out-of-range index");
      }
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> ValidatePath(const Path& path) {
  if (path.primitives.empty()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road path has no primitives");
  }
  for (std::size_t i = 0; i < path.primitives.size(); ++i) {
    const Primitive& primitive = path.primitives[i];
    if (!finite(primitive_start(primitive)) || !finite(primitive_end(primitive))) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road path contains non-finite endpoint");
    }
    if (primitive.kind == Primitive::Kind::kArc && (!finite(primitive.radius) || primitive.radius <= 0.0 ||
                                                   !finite(primitive.start_angle_rad) ||
                                                   !finite(primitive.sweep_angle_rad))) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road arc primitive is invalid");
    }
    if (primitive_length(primitive) <= kEpsilon) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road primitive has zero length");
    }
    if (i > 0 && !almost_same(primitive_start(primitive), primitive_end(path.primitives[i - 1]))) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road path primitives are not continuous");
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
  for (const Primitive& primitive : path.primitives) {
    out += primitive_length(primitive);
  }
  return Result<double>::Ok(out);
}

Result<Vec2d> EvaluatePath(const Path& path, double station_m) {
  const Result<double> length_result = PathLength(path);
  if (!length_result.ok) {
    return Result<Vec2d>::Fail(length_result.error_kind, length_result.error);
  }
  double remaining = std::clamp(station_m, 0.0, length_result.value);
  for (const Primitive& primitive : path.primitives) {
    const double len = primitive_length(primitive);
    if (remaining <= len || &primitive == &path.primitives.back()) {
      return Result<Vec2d>::Ok(primitive_eval(primitive, len <= kEpsilon ? 0.0 : remaining / len));
    }
    remaining -= len;
  }
  return Result<Vec2d>::Fail(ErrorKind::kInternal, "road path evaluation fell through");
}

std::vector<Vec2d> FlattenPath(const Path& path) {
  std::vector<Vec2d> points{};
  for (const Primitive& primitive : path.primitives) {
    const int samples = primitive.kind == Primitive::Kind::kLine ? 1 : kCurveSamples;
    if (points.empty()) {
      points.push_back(primitive_start(primitive));
    }
    for (int i = 1; i <= samples; ++i) {
      points.push_back(primitive_eval(primitive, static_cast<double>(i) / samples));
    }
  }
  return points;
}

} // namespace city::road
