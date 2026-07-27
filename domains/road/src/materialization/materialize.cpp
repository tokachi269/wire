#include "materialize.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace city::road::materialization {
namespace {

Vec2d add(Vec2d a, Vec2d b) { return {a.x + b.x, a.y + b.y}; }
Vec2d mul(Vec2d value, double scale) { return {value.x * scale, value.y * scale}; }

void append_strip(Mesh& mesh, const std::vector<Vec3d>& a, const std::vector<Vec3d>& b) {
  if (a.size() != b.size() || a.size() < 2) return;
  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    mesh.vertices.push_back(a[i]);
    mesh.vertices.push_back(b[i]);
  }
  for (std::uint32_t i = 0; i + 1 < a.size(); ++i) {
    const std::uint32_t p = base + i * 2;
    mesh.indices.insert(mesh.indices.end(), {p, p + 2, p + 1, p + 1, p + 2, p + 3});
  }
}

RenderStyleRef StyleForMarkingRule(MarkingRule rule) {
  if (rule == MarkingRule::kCenterLine) {
    return RenderStyleFromMarking(builtin_marking_styles::kCenterLine);
  }
  return RenderStyleFromMarking(builtin_marking_styles::kWhiteSolid);
}

} // namespace

Result<SegmentOutput> MaterializeSegment(const SegmentInput& input) {
  if (input.samples.empty()) {
    return Result<SegmentOutput>::Fail(ErrorKind::kInternal, "road materialization input has no samples");
  }
  const std::size_t width = input.samples.front().boundaries.size();
  const std::vector<RenderStyleRef>& styles = input.samples.front().surface_styles;
  if (width < 2 || styles.size() + 1 != width) {
    return Result<SegmentOutput>::Fail(ErrorKind::kInternal, "road materialization section width is invalid");
  }
  std::vector<Vec3d> vertices{};
  for (const SegmentSample& sample : input.samples) {
    if (sample.boundaries.size() != width || sample.surface_styles != styles) {
      return Result<SegmentOutput>::Fail(ErrorKind::kUnsupported,
                                         "road materialization section topology changes between samples");
    }
    const Vec2d lateral{-sample.tangent.y, sample.tangent.x};
    for (const SectionBoundarySample& boundary : sample.boundaries) {
      const Vec2d point = add(sample.center, mul(lateral, boundary.lateral_m));
      vertices.push_back({point.x, point.y, boundary.height_m});
    }
  }

  SegmentOutput output{};
  for (const RenderStyleRef& style : styles) {
    if (std::any_of(output.surface_meshes.begin(), output.surface_meshes.end(),
                    [&style](const Mesh& mesh) { return mesh.style == style; })) continue;
    Mesh mesh{};
    mesh.owner_segment_id = input.segment_id;
    mesh.style = style;
    mesh.vertices = vertices;
    for (std::uint32_t row = 0; row + 1 < input.samples.size(); ++row) {
      for (std::uint32_t col = 0; col < styles.size(); ++col) {
        if (styles[col] != style) continue;
        const std::uint32_t a = row * static_cast<std::uint32_t>(width) + col;
        const std::uint32_t b = a + 1;
        const std::uint32_t c = (row + 1) * static_cast<std::uint32_t>(width) + col;
        const std::uint32_t d = c + 1;
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
      }
    }
    output.surface_meshes.push_back(std::move(mesh));
  }

  for (std::size_t boundary_index = 0; boundary_index < width; ++boundary_index) {
    const MarkingRule rule = input.samples.front().boundaries[boundary_index].marking_rule;
    if (rule == MarkingRule::kNone) continue;
    const RenderStyleRef marking_style = StyleForMarkingRule(rule);
    auto found = std::find_if(
        output.marking_meshes.begin(), output.marking_meshes.end(),
        [marking_style](const Mesh& mesh) { return mesh.style == marking_style; });
    if (found == output.marking_meshes.end()) {
      output.marking_meshes.push_back(Mesh{});
      found = std::prev(output.marking_meshes.end());
      found->owner_segment_id = input.segment_id;
      found->style = marking_style;
    }
    const double half_width = rule == MarkingRule::kCenterLine ? 0.06 : 0.05;
    const std::uint32_t base = static_cast<std::uint32_t>(found->vertices.size());
    for (const SegmentSample& sample : input.samples) {
      const SectionBoundarySample& boundary = sample.boundaries[boundary_index];
      const Vec2d lateral{-sample.tangent.y, sample.tangent.x};
      const Vec2d a = add(sample.center, mul(lateral, boundary.lateral_m - half_width));
      const Vec2d b = add(sample.center, mul(lateral, boundary.lateral_m + half_width));
      found->vertices.push_back({a.x, a.y, boundary.height_m + 0.02});
      found->vertices.push_back({b.x, b.y, boundary.height_m + 0.02});
    }
    for (std::uint32_t row = 0; row + 1 < input.samples.size(); ++row) {
      const std::uint32_t a = base + row * 2;
      found->indices.insert(found->indices.end(),
                            {a, a + 2, a + 1, a + 1, a + 2, a + 3});
    }
  }

  output.terrain_mask.segment_id = input.segment_id;
  std::vector<Vec2d> right{};
  for (const SegmentSample& sample : input.samples) {
    const Vec2d lateral{-sample.tangent.y, sample.tangent.x};
    output.terrain_mask.points.push_back(add(sample.center, mul(lateral, sample.boundaries.front().lateral_m)));
    right.push_back(add(sample.center, mul(lateral, sample.boundaries.back().lateral_m)));
  }
  output.terrain_mask.points.insert(output.terrain_mask.points.end(), right.rbegin(), right.rend());
  return Result<SegmentOutput>::Ok(std::move(output));
}

Result<Mesh> MaterializeManualLine(const ManualLineInput& input) {
  if (input.left.size() < 2 || input.left.size() != input.right.size()) {
    return Result<Mesh>::Fail(ErrorKind::kInternal, "manual line read model is invalid");
  }
  Mesh mesh{};
  mesh.style = input.style;
  for (std::size_t i = 0; i < input.left.size(); ++i) {
    mesh.vertices.push_back(input.left[i]);
    mesh.vertices.push_back(input.right[i]);
  }
  for (std::uint32_t row = 0; row + 1 < input.left.size(); ++row) {
    const std::uint32_t a = row * 2;
    mesh.indices.insert(mesh.indices.end(), {a, a + 2, a + 1, a + 1, a + 2, a + 3});
  }
  return Result<Mesh>::Ok(std::move(mesh));
}

Result<Mesh> MaterializeManualArea(const ManualAreaInput& input) {
  Mesh mesh{};
  mesh.style = input.style;
  mesh.vertices.assign(input.corners.begin(), input.corners.end());
  mesh.indices = {0, 1, 2, 1, 3, 2};
  return Result<Mesh>::Ok(std::move(mesh));
}

Result<std::vector<Mesh>> MaterializeConnection(const ConnectionGeometry& input) {
  if (input.surface_strips.empty()) {
    return Result<std::vector<Mesh>>::Fail(
        ErrorKind::kInternal, "connection resolved geometry has no surface strips");
  }
  std::vector<Mesh> meshes{};
  for (const ResolvedSurfaceStrip& strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<std::vector<Mesh>>::Fail(
          ErrorKind::kInternal, "connection resolved strip is invalid");
    }
    auto found = std::find_if(meshes.begin(), meshes.end(),
                              [&strip](const Mesh& mesh) {
                                return mesh.style == strip.style;
                              });
    if (found == meshes.end()) {
      meshes.push_back(Mesh{});
      found = std::prev(meshes.end());
      found->style = strip.style;
    }
    append_strip(*found, strip.left, strip.right);
  }
  return Result<std::vector<Mesh>>::Ok(std::move(meshes));
}

Result<JunctionOutput> MaterializeJunction(const JunctionGeometry& input) {
  JunctionOutput output{};
  for (const ResolvedSurfaceRegion& region : input.surface_regions) {
    if (region.perimeter.size() < 3) {
      return Result<JunctionOutput>::Fail(
          ErrorKind::kInternal, "junction resolved surface region is invalid");
    }
    Mesh mesh{};
    mesh.style = region.style;
    Vec3d center{};
    for (const Vec3d& point : region.perimeter) {
      center.x += point.x;
      center.y += point.y;
      center.z += point.z;
    }
    const double count = static_cast<double>(region.perimeter.size());
    center.x /= count;
    center.y /= count;
    center.z /= count;
    mesh.vertices.push_back(center);
    mesh.vertices.insert(mesh.vertices.end(), region.perimeter.begin(),
                         region.perimeter.end());
    for (std::uint32_t index = 0; index < region.perimeter.size(); ++index) {
      mesh.indices.insert(
          mesh.indices.end(),
          {0, index + 1,
           static_cast<std::uint32_t>((index + 1) % region.perimeter.size()) +
               1});
    }
    output.surface_meshes.push_back(std::move(mesh));
  }
  for (const ResolvedSurfaceStrip& strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<JunctionOutput>::Fail(
          ErrorKind::kInternal, "junction resolved surface strip is invalid");
    }
    auto found = std::find_if(
        output.surface_meshes.begin(), output.surface_meshes.end(),
        [&strip](const Mesh& mesh) { return mesh.style == strip.style; });
    if (found == output.surface_meshes.end()) {
      output.surface_meshes.push_back(Mesh{});
      found = std::prev(output.surface_meshes.end());
      found->style = strip.style;
    }
    append_strip(*found, strip.left, strip.right);
  }
  for (const ResolvedAutoMarking& marking : input.auto_markings) {
    Mesh mesh{};
    mesh.style = marking.style;
    for (const std::array<Vec3d, 4>& quad : marking.quads) {
      const std::uint32_t base =
          static_cast<std::uint32_t>(mesh.vertices.size());
      mesh.vertices.insert(mesh.vertices.end(), quad.begin(), quad.end());
      mesh.indices.insert(mesh.indices.end(),
                          {base, base + 1, base + 2, base + 1, base + 3,
                           base + 2});
    }
    output.marking_meshes.push_back(std::move(mesh));
  }
  return Result<JunctionOutput>::Ok(std::move(output));
}

} // namespace city::road::materialization
