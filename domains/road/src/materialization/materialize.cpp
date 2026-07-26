#include "materialize.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace city::road::materialization {
namespace {

Vec2d add(Vec2d a, Vec2d b) { return {a.x + b.x, a.y + b.y}; }
Vec2d mul(Vec2d value, double scale) { return {value.x * scale, value.y * scale}; }

Vec3d gate_point(const ConnectionGate& gate, const SectionBoundarySample& boundary) {
  const Vec2d tangent{gate.tangent.x, gate.tangent.y};
  const Vec2d lateral{-tangent.y, tangent.x};
  return {gate.position.x + lateral.x * boundary.lateral_m,
          gate.position.y + lateral.y * boundary.lateral_m,
          gate.position.z + boundary.height_m};
}

Vec3d cubic_point(Vec3d p0, Vec3d p1, Vec3d p2, Vec3d p3, double t) {
  const double u = 1.0 - t;
  return {p0.x * u * u * u + p1.x * 3.0 * u * u * t + p2.x * 3.0 * u * t * t + p3.x * t * t * t,
          p0.y * u * u * u + p1.y * 3.0 * u * u * t + p2.y * 3.0 * u * t * t + p3.y * t * t * t,
          p0.z * u * u * u + p1.z * 3.0 * u * u * t + p2.z * 3.0 * u * t * t + p3.z * t * t * t};
}

std::vector<Vec3d> corner_curve(Vec3d a, Vec3d b, Vec2d tangent_a, Vec2d tangent_b,
                                double control, int samples) {
  const Vec3d c1{a.x - tangent_a.x * control, a.y - tangent_a.y * control, a.z};
  const Vec3d c2{b.x - tangent_b.x * control, b.y - tangent_b.y * control, b.z};
  std::vector<Vec3d> out{};
  for (int i = 0; i <= samples; ++i) {
    out.push_back(cubic_point(a, c1, c2, b, static_cast<double>(i) / samples));
  }
  return out;
}

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

double surface_height(const ConnectionGate& gate, double lateral_m) {
  if (gate.boundaries.empty()) return 0.0;
  if (lateral_m <= gate.boundaries.front().lateral_m) return gate.boundaries.front().height_m;
  for (std::size_t i = 1; i < gate.boundaries.size(); ++i) {
    const auto& a = gate.boundaries[i - 1];
    const auto& b = gate.boundaries[i];
    if (lateral_m > b.lateral_m) continue;
    const double width = b.lateral_m - a.lateral_m;
    const double t = width == 0.0 ? 0.0 : (lateral_m - a.lateral_m) / width;
    return a.height_m + (b.height_m - a.height_m) * t;
  }
  return gate.boundaries.back().height_m;
}

std::pair<double, double> carriageway_edges(const ConnectionGate& gate) {
  std::vector<std::pair<std::size_t, std::size_t>> curbs{};
  for (std::size_t i = 0; i < gate.boundaries.size(); ++i) {
    if (gate.boundaries[i].role != BoundaryRole::kCurb) continue;
    if (curbs.empty() || gate.boundaries[curbs.back().second].boundary_id != gate.boundaries[i].boundary_id) {
      curbs.push_back({i, i});
    } else {
      curbs.back().second = i;
    }
  }
  if (curbs.size() >= 2) {
    return {gate.boundaries[curbs.front().second].lateral_m, gate.boundaries[curbs.back().first].lateral_m};
  }
  return {gate.boundaries.front().lateral_m, gate.boundaries.back().lateral_m};
}

void append_gate_quad(Mesh& mesh, const ConnectionGate& gate, double longitudinal_center,
                      double longitudinal_half, double lateral_center, double lateral_half) {
  const Vec2d tangent{gate.tangent.x, gate.tangent.y};
  const Vec2d lateral{-tangent.y, tangent.x};
  const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
  for (const auto [longitudinal, lateral_offset] : std::array<std::pair<double, double>, 4>{
           std::pair{longitudinal_center - longitudinal_half, lateral_center - lateral_half},
           std::pair{longitudinal_center + longitudinal_half, lateral_center - lateral_half},
           std::pair{longitudinal_center - longitudinal_half, lateral_center + lateral_half},
           std::pair{longitudinal_center + longitudinal_half, lateral_center + lateral_half}}) {
    const Vec2d point{gate.position.x + tangent.x * longitudinal + lateral.x * lateral_offset,
                      gate.position.y + tangent.y * longitudinal + lateral.y * lateral_offset};
    mesh.vertices.push_back({point.x, point.y, surface_height(gate, lateral_offset) + 0.025});
  }
  mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base + 1, base + 3, base + 2});
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

Result<std::vector<Mesh>> MaterializeConnection(const ConnectionInput& input) {
  const ConnectionArea& area = input.area;
  if (area.gates.size() != 2 || area.gates[0].boundaries.size() != area.gates[1].boundaries.size() ||
      input.surface_materials.size() + 1 != area.gates[0].boundaries.size()) {
    return Result<std::vector<Mesh>>::Fail(ErrorKind::kValidation, "connection read model is invalid");
  }
  const double control = std::min(std::hypot(area.gates[0].position.x - input.node_position.x,
                                             area.gates[0].position.y - input.node_position.y),
                                  std::hypot(area.gates[1].position.x - input.node_position.x,
                                             area.gates[1].position.y - input.node_position.y)) * 0.55;
  constexpr int samples = 8;
  std::vector<std::vector<Vec3d>> curves{};
  const Vec2d tangent_a{area.gates[0].tangent.x, area.gates[0].tangent.y};
  const Vec2d tangent_b{area.gates[1].tangent.x, area.gates[1].tangent.y};
  for (std::size_t i = 0; i < area.gates[0].boundaries.size(); ++i) {
    curves.push_back(corner_curve(gate_point(area.gates[0], area.gates[0].boundaries[i]),
                                  gate_point(area.gates[1], area.gates[1].boundaries[i]),
                                  tangent_a, tangent_b, control, samples));
  }
  std::vector<Vec3d> vertices{};
  for (int row = 0; row <= samples; ++row) {
    for (const auto& curve : curves) vertices.push_back(curve[row]);
  }
  std::vector<Mesh> meshes{};
  const std::uint32_t width = static_cast<std::uint32_t>(curves.size());
  for (const std::string& material : input.surface_materials) {
    if (std::any_of(meshes.begin(), meshes.end(), [&material](const Mesh& mesh) { return mesh.material == material; })) {
      continue;
    }
    Mesh mesh{};
    mesh.material = material;
    mesh.vertices = vertices;
    for (std::uint32_t row = 0; row < samples; ++row) {
      for (std::uint32_t col = 0; col < input.surface_materials.size(); ++col) {
        if (input.surface_materials[col] != material) continue;
        const std::uint32_t a = row * width + col;
        const std::uint32_t c = (row + 1) * width + col;
        mesh.indices.insert(mesh.indices.end(), {a, c, a + 1, a + 1, c, c + 1});
      }
    }
    meshes.push_back(std::move(mesh));
  }
  return Result<std::vector<Mesh>>::Ok(std::move(meshes));
}

Result<JunctionOutput> MaterializeJunction(const JunctionInput& input) {
  struct Side {
    RoadSegmentId segment_id = 0;
    Vec2d tangent{};
    Vec3d outer{};
    Vec3d sidewalk{};
    Vec3d carriageway{};
  };
  std::vector<Side> sides{};
  for (const ConnectionGate& gate : input.area.gates) {
    std::vector<std::pair<std::size_t, std::size_t>> curbs{};
    for (std::size_t i = 0; i < gate.boundaries.size(); ++i) {
      if (gate.boundaries[i].role != BoundaryRole::kCurb) continue;
      if (curbs.empty() || gate.boundaries[curbs.back().second].boundary_id != gate.boundaries[i].boundary_id) {
        curbs.push_back({i, i});
      } else {
        curbs.back().second = i;
      }
    }
    if (curbs.size() < 2) {
      return Result<JunctionOutput>::Fail(ErrorKind::kValidation, "junction gate has no carriageway edges");
    }
    const Vec2d tangent{gate.tangent.x, gate.tangent.y};
    sides.push_back({gate.segment_id, tangent, gate_point(gate, gate.boundaries.front()),
                     gate_point(gate, gate.boundaries[curbs.front().first]),
                     gate_point(gate, gate.boundaries[curbs.front().second])});
    sides.push_back({gate.segment_id, tangent, gate_point(gate, gate.boundaries.back()),
                     gate_point(gate, gate.boundaries[curbs.back().second]),
                     gate_point(gate, gate.boundaries[curbs.back().first])});
  }
  std::sort(sides.begin(), sides.end(), [&input](const Side& a, const Side& b) {
    return std::atan2(a.carriageway.y - input.node_position.y, a.carriageway.x - input.node_position.x) <
           std::atan2(b.carriageway.y - input.node_position.y, b.carriageway.x - input.node_position.x);
  });
  Mesh asphalt{};
  asphalt.material = "asphalt";
  Mesh sidewalk{};
  sidewalk.material = "sidewalk";
  Mesh curb{};
  curb.material = "curb";
  std::vector<Vec3d> perimeter{};
  for (std::size_t i = 0; i < sides.size(); ++i) {
    const Side& a = sides[i];
    const Side& b = sides[(i + 1) % sides.size()];
    perimeter.push_back(a.carriageway);
    if (a.segment_id == b.segment_id) continue;
    const double control = std::min(std::hypot(a.carriageway.x - input.node_position.x,
                                               a.carriageway.y - input.node_position.y),
                                    std::hypot(b.carriageway.x - input.node_position.x,
                                               b.carriageway.y - input.node_position.y)) * 0.45;
    const auto carriage = corner_curve(a.carriageway, b.carriageway, a.tangent, b.tangent, control, 6);
    const auto walk = corner_curve(a.sidewalk, b.sidewalk, a.tangent, b.tangent, control, 6);
    const auto outer = corner_curve(a.outer, b.outer, a.tangent, b.tangent, control, 6);
    perimeter.insert(perimeter.end(), carriage.begin() + 1, carriage.end() - 1);
    append_strip(curb, carriage, walk);
    append_strip(sidewalk, walk, outer);
  }
  Vec3d center{input.node_position.x, input.node_position.y, 0.0};
  for (const Vec3d& point : perimeter) center.z += point.z;
  center.z /= perimeter.size();
  asphalt.vertices.push_back(center);
  asphalt.vertices.insert(asphalt.vertices.end(), perimeter.begin(), perimeter.end());
  for (std::uint32_t i = 0; i < perimeter.size(); ++i) {
    asphalt.indices.insert(asphalt.indices.end(),
                           {0, i + 1, static_cast<std::uint32_t>((i + 1) % perimeter.size()) + 1});
  }
  JunctionOutput output{};
  output.surface_meshes = {std::move(asphalt), std::move(sidewalk), std::move(curb)};
  for (const ConnectionGate& gate : input.area.gates) {
    const auto [left, right] = carriageway_edges(gate);
    Mesh stop{};
    stop.material = "marking";
    append_gate_quad(stop, gate, 0.35, 0.08, (left + right) * 0.5, (right - left) * 0.5);
    output.marking_meshes.push_back(std::move(stop));
    Mesh zebra{};
    zebra.material = "marking";
    for (double lateral_center = left + 0.35; lateral_center + 0.175 <= right; lateral_center += 0.7) {
      append_gate_quad(zebra, gate, 2.0, 1.4, lateral_center, 0.175);
    }
    output.marking_meshes.push_back(std::move(zebra));
  }
  return Result<JunctionOutput>::Ok(std::move(output));
}

} // namespace city::road::materialization
