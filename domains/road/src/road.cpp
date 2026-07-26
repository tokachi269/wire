#include "city/road/road.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
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

[[nodiscard]] RoadNode* find_node(SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

[[nodiscard]] double surface_height(SurfaceRole role) {
  return role == SurfaceRole::kSidewalk ? 0.15 : 0.0;
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
  double height = surface_height(section.bands.front().role);
  samples.push_back(SectionBoundarySample{1, BoundaryRole::kOuterEdge, lateral, height, MarkingRule::kNone});
  for (std::size_t i = 0; i < section.bands.size(); ++i) {
    lateral += section.bands[i].width_m;
    if (i < section.boundaries.size()) {
      const BoundaryProfile& boundary = section.boundaries[i];
      const double next_height =
          i + 1 < section.bands.size() ? surface_height(section.bands[i + 1].role) : height + boundary.height_m;
      if (boundary.width_m <= kEpsilon && std::abs(next_height - height) <= kEpsilon) {
        samples.push_back(
            SectionBoundarySample{boundary.boundary_id, boundary.role, lateral, height, boundary.marking_rule});
        continue;
      }
      samples.push_back(SectionBoundarySample{boundary.boundary_id, boundary.role, lateral, height, MarkingRule::kNone});
      lateral += boundary.width_m;
      height = next_height;
      samples.push_back(
          SectionBoundarySample{boundary.boundary_id, boundary.role, lateral, height, boundary.marking_rule});
    }
  }
  samples.push_back(SectionBoundarySample{999, BoundaryRole::kOuterEdge, total_width * 0.5, height, MarkingRule::kNone});
  return samples;
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

[[nodiscard]] Mesh build_surface_mesh(const RoadSegment& segment, const CrossSectionTemplate& section) {
  Mesh mesh{};
  const std::vector<double> stations = stations_for_path(segment.alignment);
  const std::vector<SectionBoundarySample> boundaries = evaluate_section(section);
  if (stations.empty() || boundaries.size() < 2) {
    return mesh;
  }
  const double total = stations.back();
  for (double station : stations) {
    const Result<Vec2d> center = EvaluatePath(segment.alignment, station);
    const std::optional<Vec2d> tangent = path_tangent(segment.alignment, station, total);
    if (!center.ok || !tangent.has_value()) {
      return {};
    }
    const Vec2d right{-tangent->y, tangent->x};
    for (const SectionBoundarySample& boundary : boundaries) {
      const Vec2d p = add(center.value, mul(right, boundary.lateral_m));
      mesh.vertices.push_back({p.x, p.y, boundary.height_m});
    }
  }
  const std::uint32_t width = static_cast<std::uint32_t>(boundaries.size());
  for (std::uint32_t row = 0; row + 1 < stations.size(); ++row) {
    for (std::uint32_t col = 0; col + 1 < width; ++col) {
      const std::uint32_t a = row * width + col;
      const std::uint32_t b = a + 1;
      const std::uint32_t c = (row + 1) * width + col;
      const std::uint32_t d = c + 1;
      mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
    }
  }
  return mesh;
}

[[nodiscard]] TerrainMaskPolygon build_terrain_mask(const RoadSegment& segment, const CrossSectionTemplate& section) {
  TerrainMaskPolygon mask{};
  mask.segment_id = segment.id;
  const std::vector<double> stations = stations_for_path(segment.alignment);
  const std::vector<SectionBoundarySample> boundaries = evaluate_section(section);
  if (stations.empty() || boundaries.size() < 2) {
    return mask;
  }
  const double left = boundaries.front().lateral_m;
  const double right = boundaries.back().lateral_m;
  const double total = stations.back();
  std::vector<Vec2d> right_side{};
  std::vector<Vec2d> left_side{};
  for (double station : stations) {
    const Result<Vec2d> center = EvaluatePath(segment.alignment, station);
    const std::optional<Vec2d> tangent = path_tangent(segment.alignment, station, total);
    if (!center.ok || !tangent.has_value()) {
      return {};
    }
    const Vec2d lateral{-tangent->y, tangent->x};
    left_side.push_back(add(center.value, mul(lateral, left)));
    right_side.push_back(add(center.value, mul(lateral, right)));
  }
  mask.points = left_side;
  for (auto it = right_side.rbegin(); it != right_side.rend(); ++it) {
    mask.points.push_back(*it);
  }
  return mask;
}

[[nodiscard]] Mesh build_marking_mesh(const RoadSegment& segment, const CrossSectionTemplate& section) {
  Mesh mesh{};
  const std::vector<double> stations = stations_for_path(segment.alignment);
  const std::vector<SectionBoundarySample> boundaries = evaluate_section(section);
  if (stations.empty()) {
    return mesh;
  }
  const double total = stations.back();
  for (const SectionBoundarySample& boundary : boundaries) {
    if (boundary.marking_rule == MarkingRule::kNone) {
      continue;
    }
    const double half_width = boundary.marking_rule == MarkingRule::kCenterLine ? 0.06 : 0.05;
    const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
    for (double station : stations) {
      const Result<Vec2d> center = EvaluatePath(segment.alignment, station);
      const std::optional<Vec2d> tangent = path_tangent(segment.alignment, station, total);
      if (!center.ok || !tangent.has_value()) {
        return {};
      }
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

[[nodiscard]] Mesh build_manual_line_mesh(const ManualLineMarking& marking) {
  Mesh mesh{};
  for (Vec2d p : FlattenPath(marking.path)) {
    mesh.vertices.push_back({p.x, p.y, 0.02});
  }
  return mesh;
}

[[nodiscard]] Mesh build_manual_area_mesh(const ManualAreaMarking& marking) {
  Mesh mesh{};
  const double hw = marking.width_m * 0.5;
  const double hl = marking.length_m * 0.5;
  mesh.vertices = {
      {marking.frame_origin.x - hl, marking.frame_origin.y - hw, 0.025},
      {marking.frame_origin.x + hl, marking.frame_origin.y - hw, 0.025},
      {marking.frame_origin.x - hl, marking.frame_origin.y + hw, 0.025},
      {marking.frame_origin.x + hl, marking.frame_origin.y + hw, 0.025},
  };
  mesh.indices = {0, 1, 2, 1, 3, 2};
  return mesh;
}

[[nodiscard]] Result<bool> validate_transition(const SavedRoadGraph& graph, const SectionTransition& transition) {
  if (find_template(graph, transition.from_template) == nullptr || find_template(graph, transition.to_template) == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition references a missing template");
  }
  if (!finite(transition.start.value) || !finite(transition.end.value)) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition station is not finite");
  }
  if (transition.rules.empty()) {
    return Result<bool>::Fail(ErrorKind::kUnsupported, "section transition must define element actions");
  }
  for (const SectionTransitionRule& rule : transition.rules) {
    if (rule.action == TransitionAction::kUnsupported) {
      return Result<bool>::Fail(ErrorKind::kUnsupported, "section transition contains unsupported element action");
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
      {10, SurfaceRole::kSidewalk, 2.0, -0.01, "sidewalk"},
      {20, SurfaceRole::kCarriageway, 3.0, -0.02, "asphalt"},
      {30, SurfaceRole::kCarriageway, 3.0, 0.02, "asphalt"},
      {40, SurfaceRole::kSidewalk, 2.0, 0.01, "sidewalk"},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {300, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
  };
  return section;
}

CrossSectionTemplate ThreeLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands = {
      {10, SurfaceRole::kSidewalk, 2.0, -0.01, "sidewalk"},
      {20, SurfaceRole::kCarriageway, 3.0, -0.02, "asphalt"},
      {30, SurfaceRole::kCarriageway, 3.0, 0.0, "asphalt"},
      {35, SurfaceRole::kCarriageway, 3.0, 0.02, "asphalt"},
      {40, SurfaceRole::kSidewalk, 2.0, 0.01, "sidewalk"},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {300, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
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
  for (const RoadSegment& existing : graph_.segments) {
    if (existing.node_a != start_node && existing.node_b != start_node) {
      continue;
    }
    const Vec2d other = existing.node_a == start_node ? path_end(existing.alignment) : path_start(existing.alignment);
    const double angle = angle_deg(sub(other, node->position), new_direction);
    if (angle < kP1MinConnectionAngleDeg || angle > kP1MaxConnectionAngleDeg) {
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

Result<bool> RoadState::EditSegmentPath(RoadSegmentId segment_id, Path alignment) {
  const Result<bool> valid = ValidatePath(alignment);
  if (!valid.ok) {
    return valid;
  }
  RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  SavedRoadGraph trial = graph_;
  RoadSegment* trial_segment = find_segment(trial, segment_id);
  trial_segment->alignment = alignment;
  RoadNode* a = find_node(trial, trial_segment->node_a);
  RoadNode* b = find_node(trial, trial_segment->node_b);
  if (a != nullptr) {
    a->position = path_start(alignment);
  }
  if (b != nullptr) {
    b->position = path_end(alignment);
  }
  graph_ = std::move(trial);
  return RebuildDerived();
}

Result<bool> RoadState::DeleteSegment(RoadSegmentId segment_id) {
  const auto old_size = graph_.segments.size();
  graph_.segments.erase(std::remove_if(graph_.segments.begin(), graph_.segments.end(),
                                       [segment_id](const RoadSegment& item) { return item.id == segment_id; }),
                        graph_.segments.end());
  if (graph_.segments.size() == old_size) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  graph_.manual_lines.erase(std::remove_if(graph_.manual_lines.begin(), graph_.manual_lines.end(),
                                           [segment_id](const ManualLineMarking& item) {
                                             return item.owner_segment_id == segment_id;
                                           }),
                            graph_.manual_lines.end());
  graph_.manual_areas.erase(std::remove_if(graph_.manual_areas.begin(), graph_.manual_areas.end(),
                                           [segment_id](const ManualAreaMarking& item) {
                                             return item.owner_segment_id == segment_id;
                                           }),
                            graph_.manual_areas.end());
  return RebuildDerived();
}

Result<CrossSectionTemplateId> RoadState::AddSectionTemplate(CrossSectionTemplate section_template) {
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
  auto it = std::find_if(graph_.section_templates.begin(), graph_.section_templates.end(),
                         [&section_template](const CrossSectionTemplate& item) { return item.id == section_template.id; });
  if (it == graph_.section_templates.end()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section template does not exist");
  }
  *it = std::move(section_template);
  return RebuildDerived();
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

Result<ManualMarkingId> RoadState::AddManualLine(ManualLineMarking marking) {
  if (find_segment(graph_, marking.owner_segment_id) == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line owner segment does not exist");
  }
  const Result<bool> valid = ValidatePath(marking.path);
  if (!valid.ok) {
    return Result<ManualMarkingId>::Fail(valid.error_kind, valid.error);
  }
  if (marking.id == 0) {
    marking.id = next_id_++;
  }
  graph_.manual_lines.push_back(std::move(marking));
  const Result<bool> rebuilt = RebuildDerived();
  if (!rebuilt.ok) {
    return Result<ManualMarkingId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return Result<ManualMarkingId>::Ok(graph_.manual_lines.back().id);
}

Result<ManualMarkingId> RoadState::AddManualArea(ManualAreaMarking marking) {
  if (find_segment(graph_, marking.owner_segment_id) == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area owner segment does not exist");
  }
  if (!finite(marking.frame_origin) || !finite(marking.width_m) || !finite(marking.length_m) ||
      marking.width_m <= 0.0 || marking.length_m <= 0.0) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area shape is invalid");
  }
  if (marking.id == 0) {
    marking.id = next_id_++;
  }
  graph_.manual_areas.push_back(std::move(marking));
  const Result<bool> rebuilt = RebuildDerived();
  if (!rebuilt.ok) {
    return Result<ManualMarkingId>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return Result<ManualMarkingId>::Ok(graph_.manual_areas.back().id);
}

Result<bool> RoadState::RebuildDerived() {
  DerivedRoad next{};
  for (const RoadSegment& segment : graph_.segments) {
    const CrossSectionTemplate* section = find_template(graph_, segment.section_template);
    if (section == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment section template is missing");
    }
    for (double station : stations_for_path(segment.alignment)) {
      next.section_evaluations.push_back(SectionEvaluation{segment.id, station, evaluate_section(*section)});
    }
    next.segment_meshes.push_back(build_surface_mesh(segment, *section));
    next.marking_meshes.push_back(build_marking_mesh(segment, *section));
    next.terrain_masks.push_back(build_terrain_mask(segment, *section));
    for (RoadNodeId node_id : std::array<RoadNodeId, 2>{segment.node_a, segment.node_b}) {
      const RoadNode* node = find_node(graph_, node_id);
      if (node == nullptr) {
        return Result<bool>::Fail(ErrorKind::kValidation, "road segment endpoint node is missing");
      }
      Vec2d other = node_id == segment.node_a ? path_end(segment.alignment) : path_start(segment.alignment);
      Vec2d tangent2 = normalize(sub(other, node->position));
      next.connection_gates.push_back(
          ConnectionGate{segment.id, node_id, to3(node->position), to3(tangent2), evaluate_section(*section)});
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
    next.junction_areas.push_back(std::move(area));
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    next.manual_marking_meshes.push_back(build_manual_line_mesh(marking));
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    next.manual_marking_meshes.push_back(build_manual_area_mesh(marking));
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
  out << "road_graph_version=1\n";
  out << "next_id=" << next_id_ << "\n";
  for (const RoadNode& node : graph_.nodes) {
    out << "node=" << node.id << "," << node.position.x << "," << node.position.y << "\n";
  }
  for (const RoadSegment& segment : graph_.segments) {
    out << "segment=" << segment.id << "," << segment.node_a << "," << segment.node_b << ","
        << segment.section_template << "," << segment.alignment.primitives.size() << "\n";
    for (const Primitive& primitive : segment.alignment.primitives) {
      out << "primitive=" << primitive_kind_name(primitive.kind) << "," << primitive.p0.x << "," << primitive.p0.y
          << "," << primitive.p1.x << "," << primitive.p1.y << "," << primitive.p2.x << "," << primitive.p2.y << ","
          << primitive.p3.x << "," << primitive.p3.y << "," << primitive.center.x << "," << primitive.center.y << ","
          << primitive.radius << "," << primitive.start_angle_rad << "," << primitive.sweep_angle_rad << "\n";
    }
  }
  return Result<std::string>::Ok(out.str());
}

Result<RoadState> RoadState::Load(const std::string& text) {
  RoadState state{};
  state.graph_.nodes.clear();
  state.graph_.segments.clear();
  std::istringstream in(text);
  std::string line;
  RoadSegment* current_segment = nullptr;
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
      if (value != "1") {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "unknown road graph version");
      }
    } else if (key == "next_id") {
      const std::optional<std::uint64_t> parsed = parse_u64(value);
      if (!parsed.has_value()) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid next_id");
      }
      state.next_id_ = *parsed;
    } else if (key == "node") {
      if (parts.size() != 3) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid node row");
      }
      state.graph_.nodes.push_back(RoadNode{*parse_u64(parts[0]), {*parse_double(parts[1]), *parse_double(parts[2])}});
    } else if (key == "segment") {
      if (parts.size() != 5) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid segment row");
      }
      state.graph_.segments.push_back(RoadSegment{*parse_u64(parts[0]), *parse_u64(parts[1]), *parse_u64(parts[2]),
                                                  {}, *parse_u64(parts[3]), std::nullopt});
      current_segment = &state.graph_.segments.back();
    } else if (key == "primitive") {
      if (current_segment == nullptr || parts.size() != 14) {
        return Result<RoadState>::Fail(ErrorKind::kValidation, "invalid primitive row");
      }
      Primitive primitive{};
      primitive.kind = parts[0] == "arc" ? Primitive::Kind::kArc
                                          : (parts[0] == "bezier" ? Primitive::Kind::kBezier : Primitive::Kind::kLine);
      primitive.p0 = {*parse_double(parts[1]), *parse_double(parts[2])};
      primitive.p1 = {*parse_double(parts[3]), *parse_double(parts[4])};
      primitive.p2 = {*parse_double(parts[5]), *parse_double(parts[6])};
      primitive.p3 = {*parse_double(parts[7]), *parse_double(parts[8])};
      primitive.center = {*parse_double(parts[9]), *parse_double(parts[10])};
      primitive.radius = *parse_double(parts[11]);
      primitive.start_angle_rad = *parse_double(parts[12]);
      primitive.sweep_angle_rad = *parse_double(parts[13]);
      current_segment->alignment.primitives.push_back(primitive);
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
