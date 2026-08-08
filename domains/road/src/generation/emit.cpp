#include "emit.hpp"

#include "generation.hpp"

#include "../geometry/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>

namespace city::road::generation {
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

double triangle_normal_z(Vec3d a, Vec3d b, Vec3d c) {
  const double ux = b.x - a.x;
  const double uy = b.y - a.y;
  const double vx = c.x - a.x;
  const double vy = c.y - a.y;
  return ux * vy - uy * vx;
}

void append_triangle(Mesh &mesh, std::uint32_t a, std::uint32_t b,
                     std::uint32_t c) {
  mesh.indices.insert(mesh.indices.end(), {a, b, c});
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
    append_triangle(mesh, p, p + 2, p + 1);
    append_triangle(mesh, p + 1, p + 2, p + 3);
  }
}

double polygon_area_2d(const std::vector<Vec3d> &points) {
  double twice_area = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Vec3d &a = points[index];
    const Vec3d &b = points[(index + 1) % points.size()];
    twice_area += a.x * b.y - b.x * a.y;
  }
  return twice_area * 0.5;
}

bool point_in_triangle_2d(Vec3d point, Vec3d a, Vec3d b, Vec3d c,
                          double orientation) {
  constexpr double epsilon = 1e-9;
  return triangle_normal_z(a, b, point) * orientation >= -epsilon &&
         triangle_normal_z(b, c, point) * orientation >= -epsilon &&
         triangle_normal_z(c, a, point) * orientation >= -epsilon;
}

Result<bool> append_polygon(Mesh &mesh, const std::vector<Vec3d> &perimeter) {
  constexpr double epsilon = 1e-9;
  const double area = polygon_area_2d(perimeter);
  if (std::abs(area) <= epsilon) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "junction surface polygon has zero area");
  }
  const double orientation = area > 0.0 ? 1.0 : -1.0;
  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  mesh.vertices.insert(mesh.vertices.end(), perimeter.begin(), perimeter.end());
  std::vector<std::uint32_t> remaining(perimeter.size());
  for (std::uint32_t index = 0; index < remaining.size(); ++index)
    remaining[index] = index;

  while (remaining.size() > 3) {
    bool clipped = false;
    for (std::size_t index = 0; index < remaining.size(); ++index) {
      const std::uint32_t previous =
          remaining[(index + remaining.size() - 1) % remaining.size()];
      const std::uint32_t current = remaining[index];
      const std::uint32_t next = remaining[(index + 1) % remaining.size()];
      if (triangle_normal_z(perimeter[previous], perimeter[current],
                            perimeter[next]) *
              orientation <=
          epsilon) {
        continue;
      }
      bool contains_point = false;
      for (const std::uint32_t candidate : remaining) {
        if (candidate == previous || candidate == current || candidate == next)
          continue;
        if (point_in_triangle_2d(perimeter[candidate], perimeter[previous],
                                 perimeter[current], perimeter[next],
                                 orientation)) {
          contains_point = true;
          break;
        }
      }
      if (contains_point)
        continue;
      append_triangle(mesh, base + previous, base + current, base + next);
      remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(index));
      clipped = true;
      break;
    }
    if (!clipped) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "junction surface polygon cannot be triangulated");
    }
  }
  append_triangle(mesh, base + remaining[0], base + remaining[1],
                  base + remaining[2]);
  return Result<bool>::Ok(true);
}

} // namespace

Result<segment_output> emit_segment(const segment_input &input) {
  if (input.samples.empty()) {
    return Result<segment_output>::Fail(CommitFailureCategory::kInternalError,
                                        "road draw input has no samples");
  }
  const std::size_t width = input.samples.front().boundaries.size();
  const std::vector<RenderStyleRef> &styles =
      input.samples.front().surface_styles;
  if (width < 2 || styles.size() + 1 != width) {
    return Result<segment_output>::Fail(CommitFailureCategory::kInternalError,
                                        "road draw section width is invalid");
  }
  std::vector<std::uint32_t> left_of{};
  std::vector<std::uint32_t> right_of{};
  std::uint32_t row_width = 0;
  for (std::size_t index = 0; index < width; ++index) {
    left_of.push_back(row_width);
    if (input.samples.front().boundaries[index].hard_edge) ++row_width;
    right_of.push_back(row_width);
    ++row_width;
  }

  std::vector<Vec3d> vertices{};
  for (const segment_sample &sample : input.samples) {
    if (sample.boundaries.size() != width || sample.surface_styles != styles) {
      return Result<segment_output>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road draw section topology changes between samples");
    }
    for (std::size_t index = 0; index < width; ++index) {
      const SectionBoundarySample &boundary = sample.boundaries[index];
      const Vec2d point =
          add(sample.center, mul(sample.lateral, boundary.lateral_m));
      const Vec3d vertex{point.x, point.y, boundary.height_m};
      vertices.push_back(vertex);
      if (input.samples.front().boundaries[index].hard_edge)
        vertices.push_back(vertex);
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
        const std::uint32_t a = row * row_width + right_of[col];
        const std::uint32_t b = row * row_width + left_of[col + 1];
        const std::uint32_t c = (row + 1) * row_width + right_of[col];
        const std::uint32_t d = (row + 1) * row_width + left_of[col + 1];
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
      }
    }
    output.surface_meshes.push_back(std::move(mesh));
  }

  output.terrain_mask.segment_id = input.segment_id;
  std::vector<Vec2d> right{};
  for (const segment_sample &sample : input.samples) {
    output.terrain_mask.points.push_back(
        add(sample.center,
            mul(sample.lateral, sample.boundaries.front().lateral_m)));
    right.push_back(
        add(sample.center,
            mul(sample.lateral, sample.boundaries.back().lateral_m)));
  }
  output.terrain_mask.points.insert(output.terrain_mask.points.end(),
                                    right.rbegin(), right.rend());
  return Result<segment_output>::Ok(std::move(output));
}

Result<std::vector<Mesh>> emit_connection(const ConnectionGeometry &input) {
  if (input.surface_strips.empty()) {
    return Result<std::vector<Mesh>>::Fail(
        CommitFailureCategory::kInternalError,
        "connection resolved geometry has no surface strips");
  }
  std::vector<Mesh> meshes{};
  for (const ResolvedSurfaceStrip &strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<std::vector<Mesh>>::Fail(
          CommitFailureCategory::kInternalError, "connection resolved strip is invalid");
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
    if (strip.winding == SurfaceWinding::kLeftToRight)
      append_strip(*found, strip.left, strip.right);
    else
      append_strip(*found, strip.right, strip.left);
  }
  return Result<std::vector<Mesh>>::Ok(std::move(meshes));
}

Result<junction_output> emit_junction(const JunctionGeometry &input) {
  junction_output output{};
  for (const ResolvedSurfaceRegion &region : input.surface_regions) {
    if (region.perimeter.size() < 3) {
      return Result<junction_output>::Fail(
          CommitFailureCategory::kInternalError, "junction resolved surface region is invalid");
    }
    Mesh mesh{};
    mesh.style = region.style;
    Result<bool> emitted = append_polygon(mesh, region.perimeter);
    if (!emitted.ok)
      return Result<junction_output>::Fail(emitted.failure_category,
                                           emitted.error);
    output.surface_meshes.push_back(std::move(mesh));
  }
  for (const ResolvedSurfaceStrip &strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<junction_output>::Fail(
          CommitFailureCategory::kInternalError, "junction resolved surface strip is invalid");
    }
    auto found = std::find_if(
        output.surface_meshes.begin(), output.surface_meshes.end(),
        [&strip](const Mesh &mesh) { return mesh.style == strip.style; });
    if (found == output.surface_meshes.end()) {
      output.surface_meshes.push_back(Mesh{});
      found = std::prev(output.surface_meshes.end());
      found->style = strip.style;
    }
    if (strip.winding == SurfaceWinding::kLeftToRight)
      append_strip(*found, strip.left, strip.right);
    else
      append_strip(*found, strip.right, strip.left);
  }
  return Result<junction_output>::Ok(std::move(output));
}

Result<std::vector<Mesh>>
emit_markings(const std::vector<DerivedMarking> &markings) {
  std::vector<Mesh> output{};
  for (const DerivedMarking &marking : markings) {
    if (!marking.points.empty()) {
      if (marking.points.size() < 2 || marking.width_m <= 0.0) {
        return Result<std::vector<Mesh>>::Fail(CommitFailureCategory::kInternalError,
                                               "derived marking line is invalid");
      }
      Mesh mesh{};
      mesh.owner_segment_id = marking.owner.segment_id;
      mesh.style = RenderStyleFromMarking(marking.style_id);
      std::vector<Vec3d> left{};
      std::vector<Vec3d> right{};
      const double half_width = marking.width_m * 0.5;
      for (std::size_t index = 0; index < marking.points.size(); ++index) {
        const Vec3d &current = marking.points[index];
        const Vec3d &next = index + 1 < marking.points.size()
                                ? marking.points[index + 1]
                                : marking.points[index];
        const Vec3d &prev =
            index == 0 ? marking.points[index] : marking.points[index - 1];
        const Vec2d direction =
            normalize(Vec2d{next.x - prev.x, next.y - prev.y});
        const Vec2d lateral{-direction.y, direction.x};
        left.push_back(Vec3d{current.x + lateral.x * half_width,
                             current.y + lateral.y * half_width, current.z});
        right.push_back(Vec3d{current.x - lateral.x * half_width,
                              current.y - lateral.y * half_width, current.z});
      }
      append_strip(mesh, right, left);
      output.push_back(std::move(mesh));
    }
    if (!marking.polygon.empty()) {
      if (marking.polygon.size() < 3) {
        return Result<std::vector<Mesh>>::Fail(CommitFailureCategory::kInternalError,
                                               "derived marking area is invalid");
      }
      Mesh mesh{};
      mesh.owner_segment_id = marking.owner.segment_id;
      mesh.style = RenderStyleFromMarking(marking.style_id);
      mesh.vertices = marking.polygon;
      for (std::uint32_t index = 1; index + 1 < marking.polygon.size(); ++index) {
        append_triangle(mesh, 0, index, index + 1);
      }
      output.push_back(std::move(mesh));
    }
  }
  return Result<std::vector<Mesh>>::Ok(std::move(output));
}

// Turns resolved geometry into meshes. It never decides connection kinds,
// setbacks, marking extents, ownership, styles or widths.
Result<bool> emit_geometry(DerivedRoad &derived) {
  derived.segment_meshes.clear();
  derived.connection_meshes.clear();
  derived.junction_meshes.clear();
  derived.marking_meshes.clear();
  derived.terrain_masks.clear();

  for (const DerivedSegment &segment : derived.segments) {
    if (segment.surface_segment_distances_m.empty()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road segment has no surface distances");
    }
    segment_input input{};
    input.segment_id = segment.id;
    for (const double distance : segment.surface_segment_distances_m) {
      const Result<Vec2d> center = EvaluatePath(segment.alignment, distance);
      const Result<Vec2d> lateral =
          internal::lateral_at(segment.alignment, distance);
      const SectionEvaluation *section = FindSectionAt(segment, distance);
      if (!center.ok || !lateral.ok || section == nullptr) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road surface sample is missing");
      }
      input.samples.push_back(segment_sample{center.value, lateral.value,
                                             section->boundaries,
                                             section->surface_styles});
    }
    Result<segment_output> output = emit_segment(input);
    if (!output.ok)
      return Result<bool>::Fail(output.failure_category, output.error);
    derived.segment_meshes.insert(
        derived.segment_meshes.end(),
        std::make_move_iterator(output.value.surface_meshes.begin()),
        std::make_move_iterator(output.value.surface_meshes.end()));
    derived.terrain_masks.push_back(std::move(output.value.terrain_mask));
  }

  for (const ResolvedConnection &connection : derived.connections) {
    if (connection.kind == NodeConnectionKind::kCorner) {
      Result<std::vector<Mesh>> meshes =
          emit_connection(connection.connection_geometry);
      if (!meshes.ok)
        return Result<bool>::Fail(meshes.failure_category, meshes.error);
      derived.connection_meshes.insert(
          derived.connection_meshes.end(),
          std::make_move_iterator(meshes.value.begin()),
          std::make_move_iterator(meshes.value.end()));
    } else if (connection.kind == NodeConnectionKind::kJunction) {
      Result<junction_output> output =
          emit_junction(connection.junction_geometry);
      if (!output.ok)
        return Result<bool>::Fail(output.failure_category, output.error);
      derived.junction_meshes.insert(
          derived.junction_meshes.end(),
          std::make_move_iterator(output.value.surface_meshes.begin()),
          std::make_move_iterator(output.value.surface_meshes.end()));
    }
  }

  Result<std::vector<Mesh>> marking_meshes = emit_markings(derived.markings);
  if (!marking_meshes.ok) {
    return Result<bool>::Fail(marking_meshes.failure_category, marking_meshes.error);
  }
  derived.marking_meshes = std::move(marking_meshes.value);
  return Result<bool>::Ok(true);
}

} // namespace city::road::generation
