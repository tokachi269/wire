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

} // namespace

Result<SegmentOutput> MaterializeSegment(const SegmentInput& input) {
  if (input.samples.empty()) {
    return Result<SegmentOutput>::Fail(ErrorKind::kValidation, "road materialization input has no samples");
  }
  const std::size_t width = input.samples.front().boundaries.size();
  const std::vector<std::string>& materials = input.samples.front().surface_materials;
  if (width < 2 || materials.size() + 1 != width) {
    return Result<SegmentOutput>::Fail(ErrorKind::kValidation, "road materialization section width is invalid");
  }
  std::vector<Vec3d> vertices{};
  for (const SegmentSample& sample : input.samples) {
    if (sample.boundaries.size() != width || sample.surface_materials != materials) {
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
  for (const std::string& material : materials) {
    if (std::any_of(output.surface_meshes.begin(), output.surface_meshes.end(),
                    [&material](const Mesh& mesh) { return mesh.material == material; })) continue;
    Mesh mesh{};
    mesh.owner_segment_id = input.segment_id;
    mesh.material = material;
    mesh.vertices = vertices;
    for (std::uint32_t row = 0; row + 1 < input.samples.size(); ++row) {
      for (std::uint32_t col = 0; col < materials.size(); ++col) {
        if (materials[col] != material) continue;
        const std::uint32_t a = row * static_cast<std::uint32_t>(width) + col;
        const std::uint32_t b = a + 1;
        const std::uint32_t c = (row + 1) * static_cast<std::uint32_t>(width) + col;
        const std::uint32_t d = c + 1;
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
      }
    }
    output.surface_meshes.push_back(std::move(mesh));
  }

  output.marking_mesh.owner_segment_id = input.segment_id;
  output.marking_mesh.material = "road_marking";
  for (std::size_t boundary_index = 0; boundary_index < width; ++boundary_index) {
    const MarkingRule rule = input.samples.front().boundaries[boundary_index].marking_rule;
    if (rule == MarkingRule::kNone) continue;
    const double half_width = rule == MarkingRule::kCenterLine ? 0.06 : 0.05;
    const std::uint32_t base = static_cast<std::uint32_t>(output.marking_mesh.vertices.size());
    for (const SegmentSample& sample : input.samples) {
      const SectionBoundarySample& boundary = sample.boundaries[boundary_index];
      const Vec2d lateral{-sample.tangent.y, sample.tangent.x};
      const Vec2d a = add(sample.center, mul(lateral, boundary.lateral_m - half_width));
      const Vec2d b = add(sample.center, mul(lateral, boundary.lateral_m + half_width));
      output.marking_mesh.vertices.push_back({a.x, a.y, boundary.height_m + 0.02});
      output.marking_mesh.vertices.push_back({b.x, b.y, boundary.height_m + 0.02});
    }
    for (std::uint32_t row = 0; row + 1 < input.samples.size(); ++row) {
      const std::uint32_t a = base + row * 2;
      output.marking_mesh.indices.insert(output.marking_mesh.indices.end(),
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
    return Result<Mesh>::Fail(ErrorKind::kValidation, "manual line read model is invalid");
  }
  Mesh mesh{};
  mesh.material = "road_marking";
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
  mesh.material = "road_marking";
  mesh.vertices.assign(input.corners.begin(), input.corners.end());
  mesh.indices = {0, 1, 2, 1, 3, 2};
  return Result<Mesh>::Ok(std::move(mesh));
}

Result<std::vector<Mesh>> MaterializeConnection(const ConnectionGeometry& input) {
  if (input.surface_strips.empty()) {
    return Result<std::vector<Mesh>>::Fail(
        ErrorKind::kValidation, "connection resolved geometry has no surface strips");
  }
  std::vector<Mesh> meshes{};
  for (const ResolvedSurfaceStrip& strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<std::vector<Mesh>>::Fail(
          ErrorKind::kValidation, "connection resolved strip is invalid");
    }
    auto found = std::find_if(meshes.begin(), meshes.end(),
                              [&strip](const Mesh& mesh) {
                                return mesh.material == strip.material;
                              });
    if (found == meshes.end()) {
      meshes.push_back(Mesh{});
      found = std::prev(meshes.end());
      found->material = strip.material;
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
          ErrorKind::kValidation, "junction resolved surface region is invalid");
    }
    Mesh mesh{};
    mesh.material = region.material;
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
          ErrorKind::kValidation, "junction resolved surface strip is invalid");
    }
    auto found = std::find_if(
        output.surface_meshes.begin(), output.surface_meshes.end(),
        [&strip](const Mesh& mesh) { return mesh.material == strip.material; });
    if (found == output.surface_meshes.end()) {
      output.surface_meshes.push_back(Mesh{});
      found = std::prev(output.surface_meshes.end());
      found->material = strip.material;
    }
    append_strip(*found, strip.left, strip.right);
  }
  for (const ResolvedAutoMarking& marking : input.auto_markings) {
    Mesh mesh{};
    mesh.material = marking.material;
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
