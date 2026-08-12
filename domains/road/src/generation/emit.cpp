#include "emit.hpp"

#include "generation.hpp"

#include "../geometry/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <optional>

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

double distance(Vec3d a, Vec3d b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

Vec3d subtract(Vec3d a, Vec3d b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3d cross(Vec3d a, Vec3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

Vec3d normalize(Vec3d value) {
  const double length = std::hypot(std::hypot(value.x, value.y), value.z);
  if (length <= 1e-9)
    return {0.0, 0.0, 1.0};
  return {value.x / length, value.y / length, value.z / length};
}

constexpr PatchUvSettings kDefaultPatchUvSettings{};

double quantized_patch_u_end(double length_m) {
  const double seam_length_m =
      kDefaultPatchUvSettings.reference_length_m /
      static_cast<double>(kDefaultPatchUvSettings.seam_divisions);
  const auto seam_count = static_cast<int>(
      std::max(1.0, std::round(std::max(0.0, length_m) / seam_length_m)));
  return static_cast<double>(seam_count) /
         static_cast<double>(kDefaultPatchUvSettings.seam_divisions);
}

Vec2d world_uv(Vec3d point) {
  return {point.x / kDefaultPatchUvSettings.reference_length_m,
          point.y / kDefaultPatchUvSettings.reference_length_m};
}

void append_vertex(Mesh& mesh, Vec3d point, Vec2d uv) {
  mesh.vertices.push_back(point);
  mesh.uv0.push_back(uv);
}

void rebuild_normals(Mesh& mesh) {
  mesh.normals.assign(mesh.vertices.size(), {});
  for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
    const std::uint32_t ia = mesh.indices[index];
    const std::uint32_t ib = mesh.indices[index + 1];
    const std::uint32_t ic = mesh.indices[index + 2];
    if (ia >= mesh.vertices.size() || ib >= mesh.vertices.size() ||
        ic >= mesh.vertices.size())
      continue;
    const Vec3d normal =
        cross(subtract(mesh.vertices[ib], mesh.vertices[ia]),
              subtract(mesh.vertices[ic], mesh.vertices[ia]));
    mesh.normals[ia].x += normal.x;
    mesh.normals[ia].y += normal.y;
    mesh.normals[ia].z += normal.z;
    mesh.normals[ib].x += normal.x;
    mesh.normals[ib].y += normal.y;
    mesh.normals[ib].z += normal.z;
    mesh.normals[ic].x += normal.x;
    mesh.normals[ic].y += normal.y;
    mesh.normals[ic].z += normal.z;
  }
  for (Vec3d& normal : mesh.normals)
    normal = normalize(normal);
}

void finalize_mesh(Mesh& mesh, MeshUvMapping mapping) {
  mesh.uv_mapping = mapping;
  if (mesh.uv0.size() != mesh.vertices.size()) {
    mesh.uv0.clear();
    mesh.uv0.reserve(mesh.vertices.size());
    for (const Vec3d& vertex : mesh.vertices)
      mesh.uv0.push_back(world_uv(vertex));
  }
  rebuild_normals(mesh);
  mesh.material_groups.clear();
  if (!mesh.indices.empty()) {
    mesh.material_groups.push_back(
        MeshMaterialGroup{mesh.style, 0,
                          static_cast<std::uint32_t>(mesh.indices.size())});
  }
}

void append_triangle(Mesh &mesh, std::uint32_t a, std::uint32_t b,
                     std::uint32_t c) {
  mesh.indices.insert(mesh.indices.end(), {a, b, c});
}

struct render_column {
  std::size_t source_index = 0;
  bool generated_crown = false;
};

struct segment_render_grid {
  std::vector<render_column> columns{};
  std::vector<RenderStyleRef> face_styles{};
  std::vector<std::uint32_t> left_of{};
  std::vector<std::uint32_t> right_of{};
  std::uint32_t row_width = 0;
};

RenderStyleRef asphalt_style() {
  return RenderStyleFromSurface(builtin_surface_styles::kAsphalt);
}

std::optional<std::size_t> carriageway_crown_face(
    const std::vector<SectionBoundarySample> &boundaries,
    const std::vector<RenderStyleRef> &styles) {
  std::optional<std::size_t> first{};
  std::optional<std::size_t> last{};
  for (std::size_t index = 0; index < styles.size(); ++index) {
    if (styles[index] != asphalt_style()) continue;
    if (!first.has_value()) first = index;
    last = index;
  }
  if (!first.has_value() || !last.has_value() || *first == *last)
    return std::nullopt;
  const double crown =
      (boundaries[*first].lateral_m + boundaries[*last + 1].lateral_m) * 0.5;
  for (std::size_t index = *first; index <= *last; ++index) {
    if (std::abs(boundaries[index].lateral_m - crown) <= 1e-9 ||
        std::abs(boundaries[index + 1].lateral_m - crown) <= 1e-9) {
      return std::nullopt;
    }
    if (boundaries[index].lateral_m < crown &&
        crown < boundaries[index + 1].lateral_m) {
      return index;
    }
  }
  return std::nullopt;
}

double crown_height(const segment_sample &sample, std::size_t face_index) {
  const auto &boundaries = sample.boundaries;
  double left = boundaries.front().lateral_m;
  double right = boundaries.back().lateral_m;
  for (std::size_t index = 0; index < sample.surface_styles.size(); ++index) {
    if (sample.surface_styles[index] != asphalt_style()) continue;
    left = boundaries[index].lateral_m;
    while (index + 1 < sample.surface_styles.size() &&
           sample.surface_styles[index + 1] == asphalt_style()) {
      ++index;
    }
    right = boundaries[index + 1].lateral_m;
    break;
  }
  double grade = 0.0;
  for (std::size_t index = 0; index < sample.surface_styles.size(); ++index) {
    if (sample.surface_styles[index] != asphalt_style()) continue;
    const double width =
        boundaries[index + 1].lateral_m - boundaries[index].lateral_m;
    if (std::abs(width) <= 1e-9) continue;
    grade = std::max(grade, std::abs((boundaries[index + 1].height_m -
                                      boundaries[index].height_m) /
                                     width));
  }
  const double crown = (left + right) * 0.5;
  const double edge_height =
      std::max(boundaries[face_index].height_m,
               boundaries[face_index + 1].height_m);
  return std::max(edge_height,
                  grade * std::min(crown - left, right - crown));
}

segment_render_grid make_segment_render_grid(
    const std::vector<SectionBoundarySample> &boundaries,
    const std::vector<RenderStyleRef> &styles) {
  segment_render_grid grid{};
  const std::optional<std::size_t> crown_face =
      carriageway_crown_face(boundaries, styles);
  for (std::size_t index = 0; index < boundaries.size(); ++index) {
    if (crown_face.has_value() && *crown_face + 1 == index) {
      grid.columns.push_back(render_column{*crown_face, true});
    }
    grid.columns.push_back(render_column{index, false});
  }
  for (std::size_t index = 0; index < styles.size(); ++index) {
    grid.face_styles.push_back(styles[index]);
    if (crown_face.has_value() && *crown_face == index) {
      grid.face_styles.push_back(styles[index]);
    }
  }
  for (const render_column &column : grid.columns) {
    grid.left_of.push_back(grid.row_width);
    if (!column.generated_crown && boundaries[column.source_index].hard_edge)
      ++grid.row_width;
    grid.right_of.push_back(grid.row_width);
    ++grid.row_width;
  }
  return grid;
}

void append_strip(Mesh &mesh, const std::vector<Vec3d> &a,
                  const std::vector<Vec3d> &b) {
  if (a.size() != b.size() || a.size() < 2)
    return;
  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  std::vector<double> distances(a.size(), 0.0);
  for (std::size_t i = 1; i < a.size(); ++i) {
    distances[i] =
        distances[i - 1] +
        (distance(a[i - 1], a[i]) + distance(b[i - 1], b[i])) * 0.5;
  }
  const double u_end = quantized_patch_u_end(distances.back());
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double u =
        distances.back() > 1e-9 ? (distances[i] / distances.back()) * u_end
                                : 0.0;
    append_vertex(mesh, a[i], Vec2d{u, 0.0});
    append_vertex(mesh, b[i], Vec2d{u, 1.0});
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

Result<bool> append_fan_polygon(Mesh& mesh,
                                const std::vector<Vec3d>& perimeter) {
  constexpr double epsilon = 1e-9;
  const double area = polygon_area_2d(perimeter);
  if (std::abs(area) <= epsilon) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "junction surface polygon has zero area");
  }
  std::vector<Vec3d> ordered = perimeter;
  if (area < 0.0)
    std::reverse(ordered.begin(), ordered.end());
  Vec3d center{};
  for (const Vec3d& point : ordered) {
    center.x += point.x;
    center.y += point.y;
    center.z += point.z;
  }
  center.x /= static_cast<double>(ordered.size());
  center.y /= static_cast<double>(ordered.size());
  center.z /= static_cast<double>(ordered.size());

  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  append_vertex(mesh, center, world_uv(center));
  for (const Vec3d& point : ordered)
    append_vertex(mesh, point, world_uv(point));
  for (std::uint32_t index = 0; index < ordered.size(); ++index) {
    const std::uint32_t current = base + 1 + index;
    const std::uint32_t next =
        base + 1 + ((index + 1) % static_cast<std::uint32_t>(ordered.size()));
    append_triangle(mesh, base, current, next);
  }
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
  const segment_render_grid grid =
      make_segment_render_grid(input.samples.front().boundaries, styles);

  std::vector<Vec3d> vertices{};
  std::vector<Vec2d> uv0{};
  const double patch_start_m = input.samples.front().segment_distance_m;
  const double patch_length_m =
      std::max(0.0, input.samples.back().segment_distance_m - patch_start_m);
  const double patch_u_end = quantized_patch_u_end(patch_length_m);
  for (const segment_sample &sample : input.samples) {
    if (sample.boundaries.size() != width || sample.surface_styles != styles) {
      return Result<segment_output>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road draw section topology changes between samples");
    }
    const double u = patch_length_m > 1e-9
                         ? ((sample.segment_distance_m - patch_start_m) /
                            patch_length_m) *
                               patch_u_end
                         : 0.0;
    for (const render_column &column : grid.columns) {
      const SectionBoundarySample &boundary =
          sample.boundaries[column.source_index];
      const double lateral =
          column.generated_crown
              ? (sample.boundaries[column.source_index].lateral_m +
                 sample.boundaries[column.source_index + 1].lateral_m) *
                    0.5
              : boundary.lateral_m;
      const double height =
          column.generated_crown ? crown_height(sample, column.source_index)
                                 : boundary.height_m;
      const Vec2d point = add(sample.center, mul(sample.lateral, lateral));
      const Vec3d vertex{point.x, point.y, height};
      vertices.push_back(vertex);
      uv0.push_back(
          Vec2d{u, lateral / kDefaultPatchUvSettings.reference_length_m});
      if (!column.generated_crown &&
          input.samples.front().boundaries[column.source_index].hard_edge) {
        vertices.push_back(vertex);
        uv0.push_back(
            Vec2d{u, lateral / kDefaultPatchUvSettings.reference_length_m});
      }
    }
  }

  segment_output output{};
  for (const RenderStyleRef &style : grid.face_styles) {
    if (std::any_of(output.surface_meshes.begin(), output.surface_meshes.end(),
                    [&style](const Mesh &mesh) { return mesh.style == style; }))
      continue;
    Mesh mesh{};
    mesh.owner_segment_id = input.segment_id;
    mesh.style = style;
    mesh.uv_mapping = MeshUvMapping::kPatchQuantized;
    mesh.vertices = vertices;
    mesh.uv0 = uv0;
    for (std::uint32_t row = 0; row + 1 < input.samples.size(); ++row) {
      for (std::uint32_t col = 0; col < grid.face_styles.size(); ++col) {
        if (grid.face_styles[col] != style)
          continue;
        const std::uint32_t a = row * grid.row_width + grid.right_of[col];
        const std::uint32_t b = row * grid.row_width + grid.left_of[col + 1];
        const std::uint32_t c =
            (row + 1) * grid.row_width + grid.right_of[col];
        const std::uint32_t d =
            (row + 1) * grid.row_width + grid.left_of[col + 1];
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
      }
    }
    finalize_mesh(mesh, MeshUvMapping::kPatchQuantized);
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
  for (Mesh& mesh : meshes)
    finalize_mesh(mesh, MeshUvMapping::kPatchQuantized);
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
    mesh.uv_mapping = MeshUvMapping::kWorld;
    Result<bool> emitted = append_fan_polygon(mesh, region.perimeter);
    if (!emitted.ok)
      return Result<junction_output>::Fail(emitted.failure_category,
                                           emitted.error);
    finalize_mesh(mesh, MeshUvMapping::kWorld);
    output.surface_meshes.push_back(std::move(mesh));
  }
  for (const ResolvedSurfaceStrip &strip : input.surface_strips) {
    if (strip.left.size() < 2 || strip.left.size() != strip.right.size()) {
      return Result<junction_output>::Fail(
          CommitFailureCategory::kInternalError, "junction resolved surface strip is invalid");
    }
    auto found = std::find_if(
        output.surface_meshes.begin(), output.surface_meshes.end(),
        [&strip](const Mesh &mesh) {
          return mesh.style == strip.style &&
                 mesh.uv_mapping == MeshUvMapping::kPatchQuantized;
        });
    if (found == output.surface_meshes.end()) {
      output.surface_meshes.push_back(Mesh{});
      found = std::prev(output.surface_meshes.end());
      found->style = strip.style;
      found->uv_mapping = MeshUvMapping::kPatchQuantized;
    }
    if (strip.winding == SurfaceWinding::kLeftToRight)
      append_strip(*found, strip.left, strip.right);
    else
      append_strip(*found, strip.right, strip.left);
  }
  for (Mesh& mesh : output.surface_meshes)
    finalize_mesh(mesh, mesh.uv_mapping);
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
      mesh.uv_mapping = MeshUvMapping::kPatchQuantized;
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
      finalize_mesh(mesh, MeshUvMapping::kPatchQuantized);
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
      mesh.uv_mapping = MeshUvMapping::kWorld;
      for (const Vec3d& point : marking.polygon)
        append_vertex(mesh, point, world_uv(point));
      for (std::uint32_t index = 1; index + 1 < marking.polygon.size(); ++index) {
        append_triangle(mesh, 0, index, index + 1);
      }
      finalize_mesh(mesh, MeshUvMapping::kWorld);
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
                                             distance,
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
