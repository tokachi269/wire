#include "mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace city::road::draw {
namespace {

Vec2d add(Vec2d a, Vec2d b) { return {a.x + b.x, a.y + b.y}; }
Vec2d mul(Vec2d value, double scale) {
  return {value.x * scale, value.y * scale};
}
Vec2d normalize(Vec2d value) {
  const double length = std::hypot(value.x, value.y);
  if (length <= 1e-9)
    return {};
  return {value.x / length, value.y / length};
}

void append_strip(Mesh &mesh, const std::vector<Vec3d> &a,
                  const std::vector<Vec3d> &b) {
  if (a.size() != b.size() || a.size() < 2)
    return;
  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    mesh.vertices.push_back(a[i]);
    mesh.vertices.push_back(b[i]);
  }
  for (std::uint32_t i = 0; i + 1 < a.size(); ++i) {
    const std::uint32_t p = base + i * 2;
    mesh.indices.insert(mesh.indices.end(),
                        {p, p + 2, p + 1, p + 1, p + 2, p + 3});
  }
}

} // namespace

Result<segment_output> make_segment(const segment_input &input) {
  if (input.samples.empty()) {
    return Result<segment_output>::Fail(ErrorKind::kInternal,
                                        "road draw input has no samples");
  }
  const std::size_t width = input.samples.front().boundaries.size();
  const std::vector<RenderStyleRef> &styles =
      input.samples.front().surface_styles;
  if (width < 2 || styles.size() + 1 != width) {
    return Result<segment_output>::Fail(ErrorKind::kInternal,
                                        "road draw section width is invalid");
  }
  std::vector<Vec3d> vertices{};
  for (const segment_sample &sample : input.samples) {
    if (sample.boundaries.size() != width || sample.surface_styles != styles) {
      return Result<segment_output>::Fail(
          ErrorKind::kUnsupported,
          "road draw section topology changes between samples");
    }
    const Vec2d lateral{-sample.tangent.y, sample.tangent.x};
    for (const SectionBoundarySample &boundary : sample.boundaries) {
      const Vec2d point = add(sample.center, mul(lateral, boundary.lateral_m));
      vertices.push_back({point.x, point.y, boundary.height_m});
    }
  }

  segment_output output{};
  for (const RenderStyleRef &style : styles) {
    if (std::any_of(output.surface_meshes.begin(), output.surface_meshes.end(),
                    [&style](const Mesh &mesh) { return mesh.style == style; }))
      continue;
    Mesh mesh{};
    mesh.owner_segment_id = input.segment_id;
    mesh.style = style;
    mesh.vertices = vertices;
    for (std::uint32_t row = 0; row + 1 < input.samples.size(); ++row) {
      for (std::uint32_t col = 0; col < styles.size(); ++col) {
        if (styles[col] != style)
          continue;
        const std::uint32_t a = row * static_cast<std::uint32_t>(width) + col;
        const std::uint32_t b = a + 1;
        const std::uint32_t c =
            (row + 1) * static_cast<std::uint32_t>(width) + col;
        const std::uint32_t d = c + 1;
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
      }
    }
    output.surface_meshes.push_back(std::move(mesh));
  }

  output.terrain_mask.segment_id = input.segment_id;
  std::vector<Vec2d> right{};
  for (const segment_sample &sample : input.samples) {
    const Vec2d lateral{-sample.tangent.y, sample.tangent.x};
    output.terrain_mask.points.push_back(
        add(sample.center, mul(lateral, sample.boundaries.front().lateral_m)));
    right.push_back(
        add(sample.center, mul(lateral, sample.boundaries.back().lateral_m)));
  }
  output.terrain_mask.points.insert(output.terrain_mask.points.end(),
                                    right.rbegin(), right.rend());
  return Result<segment_output>::Ok(std::move(output));
}

Result<std::vector<Mesh>> make_connection(const ConnectionGeometry &input) {
  if (input.surface_strips.empty()) {
    return Result<std::vector<Mesh>>::Fail(
        ErrorKind::kInternal,
        "connection resolved geometry has no surface strips");
  }
  std::vector<Mesh> meshes{};
  for (const ResolvedSurfaceStrip &strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<std::vector<Mesh>>::Fail(
          ErrorKind::kInternal, "connection resolved strip is invalid");
    }
    auto found =
        std::find_if(meshes.begin(), meshes.end(), [&strip](const Mesh &mesh) {
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

Result<junction_output> make_junction(const JunctionGeometry &input) {
  junction_output output{};
  for (const ResolvedSurfaceRegion &region : input.surface_regions) {
    if (region.perimeter.size() < 3) {
      return Result<junction_output>::Fail(
          ErrorKind::kInternal, "junction resolved surface region is invalid");
    }
    Mesh mesh{};
    mesh.style = region.style;
    Vec3d center{};
    for (const Vec3d &point : region.perimeter) {
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
  for (const ResolvedSurfaceStrip &strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<junction_output>::Fail(
          ErrorKind::kInternal, "junction resolved surface strip is invalid");
    }
    auto found = std::find_if(
        output.surface_meshes.begin(), output.surface_meshes.end(),
        [&strip](const Mesh &mesh) { return mesh.style == strip.style; });
    if (found == output.surface_meshes.end()) {
      output.surface_meshes.push_back(Mesh{});
      found = std::prev(output.surface_meshes.end());
      found->style = strip.style;
    }
    append_strip(*found, strip.left, strip.right);
  }
  return Result<junction_output>::Ok(std::move(output));
}

Result<std::vector<Mesh>> make_markings(const ResolvedMarkingGraph &input) {
  std::vector<Mesh> output{};
  for (const ResolvedMarkingPath &path : input.paths) {
    if (path.centerline.size() < 2 || path.width_m <= 0.0) {
      return Result<std::vector<Mesh>>::Fail(
          ErrorKind::kInternal, "resolved marking path is invalid");
    }
    Mesh mesh{};
    mesh.owner_segment_id = path.owner.segment_id;
    mesh.style = RenderStyleFromMarking(path.style_id);
    std::vector<Vec3d> left{};
    std::vector<Vec3d> right{};
    const double half_width = path.width_m * 0.5;
    for (std::size_t index = 0; index < path.centerline.size(); ++index) {
      const Vec3d &current = path.centerline[index];
      const Vec3d &next = index + 1 < path.centerline.size()
                              ? path.centerline[index + 1]
                              : path.centerline[index];
      const Vec3d &prev =
          index == 0 ? path.centerline[index] : path.centerline[index - 1];
      const Vec2d direction =
          normalize(Vec2d{next.x - prev.x, next.y - prev.y});
      const Vec2d lateral{-direction.y, direction.x};
      left.push_back(Vec3d{current.x + lateral.x * half_width,
                           current.y + lateral.y * half_width, current.z});
      right.push_back(Vec3d{current.x - lateral.x * half_width,
                            current.y - lateral.y * half_width, current.z});
    }
    append_strip(mesh, left, right);
    output.push_back(std::move(mesh));
  }
  for (const ResolvedMarkingArea &area : input.areas) {
    for (const std::vector<Vec3d> &polygon : area.polygons) {
      if (polygon.size() < 3) {
        return Result<std::vector<Mesh>>::Fail(
            ErrorKind::kInternal, "resolved marking area is invalid");
      }
      Mesh mesh{};
      mesh.owner_segment_id = area.owner.segment_id;
      mesh.style = RenderStyleFromMarking(area.style_id);
      mesh.vertices = polygon;
      for (std::uint32_t index = 1; index + 1 < polygon.size(); ++index) {
        mesh.indices.insert(mesh.indices.end(), {0, index, index + 1});
      }
      output.push_back(std::move(mesh));
    }
  }
  return Result<std::vector<Mesh>>::Ok(std::move(output));
}

} // namespace city::road::draw
