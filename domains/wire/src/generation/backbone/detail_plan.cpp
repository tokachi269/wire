#include "detail_plan.hpp"

#include "city/wire/core_view.hpp"
#include "city/wire/coord_utils.hpp"

#include "../../support/hash_mix.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace city::wire::generation::backbone {
namespace {

using support::hash_combine;

constexpr const char* kTransformerModelKey = "detail_transformer_box";
constexpr const char* kTerminalModelKey = "detail_terminal_post";
constexpr const char* kInlineDeviceModelKey = "detail_inline_device";
constexpr std::uint32_t kHvDetailCableColor = 0x202124FFu;
constexpr std::uint32_t kDeviceCableColor = 0x2E2B26FFu;

struct EndpointFrame {
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId span_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  BundleTemplateId template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  Vec3d position{};
  Vec3d forward{};
  std::uint64_t source_version = 0;
};

struct DetailAnchorFrame {
  Vec3d position{};
  Vec3d forward{1.0, 0.0, 0.0};
  Vec3d up{0.0, 0.0, 1.0};
};

struct DetailTemplate {
  const char* model_key = "";
  Vec3d approximate_size{};
  std::vector<std::pair<const char*, DetailAnchorFrame>> sockets{};
  std::vector<std::pair<const char*, DetailAnchorFrame>> guides{};
};

struct NodeGroup {
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  BundleTemplateId template_id = kInvalidBundleTemplateId;
  std::vector<EndpointFrame> lanes{};
  std::uint64_t source_version = 0;
};

[[nodiscard]] const DetailTemplate& transformer_template() {
  static const DetailTemplate value{
      kTransformerModelKey,
      {0.28, 0.16, 0.36},
      {
          {"hv_in_left", {{-0.12, 0.0, 0.20}, {-0.2, 0.0, 1.0}, {0.0, 0.0, 1.0}}},
          {"hv_in_center", {{0.0, 0.0, 0.20}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}}},
          {"hv_in_right", {{0.12, 0.0, 0.20}, {0.2, 0.0, 1.0}, {0.0, 0.0, 1.0}}},
      },
      {
          {"top_loop", {{0.0, 0.0, 0.08}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
          {"clutter_left", {{-0.16, 0.0, -0.20}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
          {"clutter_right", {{0.14, 0.0, -0.19}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
      }};
  return value;
}

[[nodiscard]] const DetailTemplate& terminal_template() {
  static const DetailTemplate value{
      kTerminalModelKey,
      {0.08, 0.08, 0.20},
      {{"wire", {{0.0, 0.0, 0.10}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}}}},
      {}};
  return value;
}

[[nodiscard]] const DetailTemplate& inline_device_template() {
  static const DetailTemplate value{
      kInlineDeviceModelKey,
      {0.34, 0.12, 0.12},
      {
          {"wire_in", {{-0.17, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
          {"wire_out", {{0.17, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
      },
      {{"side_bypass", {{0.0, 0.18, 0.04}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}}}};
  return value;
}

[[nodiscard]] Vec3d template_point(const DetailTemplate& tmpl, const char* name,
                                   const Vec3d& origin, const Vec3d& lateral) {
  const auto append = [&](const DetailAnchorFrame& frame) {
    return origin + ScaleVec(lateral, frame.position.x) + ScaleVec(WorldUp(), frame.position.z);
  };
  for (const auto& socket : tmpl.sockets) {
    if (std::string(socket.first) == name) return append(socket.second);
  }
  for (const auto& guide : tmpl.guides) {
    if (std::string(guide.first) == name) return append(guide.second);
  }
  return origin;
}

[[nodiscard]] bool is_hv_template(const CoreState& state, BundleTemplateId template_id) {
  const auto found = state.view().bundle_templates().find(template_id);
  return found != state.view().bundle_templates().end() &&
         found->second.category == ConnectionCategory::kHighVoltage &&
         found->second.default_layer == SpanLayer::kHighVoltage;
}

[[nodiscard]] Vec3d unit_or(Vec3d value, Vec3d fallback) {
  return Normalize(&value) ? value : fallback;
}

[[nodiscard]] Vec3d horizontal_or(Vec3d value, Vec3d fallback = WorldForward()) {
  value.z = 0.0;
  fallback.z = 0.0;
  if (Normalize(&value)) return value;
  return Normalize(&fallback) ? fallback : WorldForward();
}

[[nodiscard]] AABBd bounds_for(const std::vector<Vec3d>& points) {
  AABBd bounds{};
  if (points.empty()) return bounds;
  bounds.min = points.front();
  bounds.max = points.front();
  for (const Vec3d& point : points) {
    bounds.min.x = std::min(bounds.min.x, point.x);
    bounds.min.y = std::min(bounds.min.y, point.y);
    bounds.min.z = std::min(bounds.min.z, point.z);
    bounds.max.x = std::max(bounds.max.x, point.x);
    bounds.max.y = std::max(bounds.max.y, point.y);
    bounds.max.z = std::max(bounds.max.z, point.z);
  }
  return bounds;
}

[[nodiscard]] double yaw_deg_from_forward(Vec3d forward) {
  forward = horizontal_or(forward);
  return std::atan2(forward.y, forward.x) * 180.0 / 3.14159265358979323846;
}

[[nodiscard]] std::uint64_t qhash(std::uint64_t seed, const Vec3d& value) {
  auto quant = [](double v) {
    return static_cast<std::uint64_t>(std::llround(v * 1'000'000.0));
  };
  seed = hash_combine(seed, quant(value.x));
  seed = hash_combine(seed, quant(value.y));
  seed = hash_combine(seed, quant(value.z));
  return seed == 0 ? 1 : seed;
}

[[nodiscard]] std::uint64_t detail_key_for(std::uint64_t salt, const EndpointFrame& endpoint) {
  std::uint64_t key = hash_combine(salt, endpoint.node_id);
  key = hash_combine(key, endpoint.edge_id);
  key = hash_combine(key, endpoint.span_id);
  key = hash_combine(key, endpoint.bundle_id);
  key = hash_combine(key, endpoint.template_id);
  key = hash_combine(key, endpoint.lane_index);
  key = hash_combine(key, endpoint.source_version);
  return qhash(key, endpoint.position);
}

[[nodiscard]] std::uint64_t detail_key_for(std::uint64_t salt, const VisualCurvePart& carrier) {
  std::uint64_t key = hash_combine(salt, carrier.source_span_id);
  key = hash_combine(key, carrier.source_bundle_id);
  key = hash_combine(key, carrier.bundle_template_id);
  key = hash_combine(key, carrier.lane_index);
  key = hash_combine(key, carrier.source_version);
  return qhash(key, carrier.samples.empty() ? Vec3d{} : carrier.samples.front());
}

[[nodiscard]] std::uint64_t version_for(std::uint64_t seed, const std::vector<Vec3d>& points) {
  std::uint64_t out = seed;
  for (const Vec3d& point : points) out = qhash(out, point);
  return out == 0 ? 1 : out;
}

[[nodiscard]] std::vector<Vec3d> local_cable_points(
    const Vec3d& start, Vec3d start_forward, const Vec3d& guide_a, const Vec3d& guide_b,
    Vec3d end_forward, const Vec3d& end) {
  start_forward = unit_or(start_forward, end - start);
  end_forward = unit_or(end_forward, start - end);
  return {
      start,
      start + ScaleVec(start_forward, 0.18),
      guide_a,
      guide_b,
      end - ScaleVec(end_forward, 0.14),
      end,
  };
}

void append_cable(VisualCurvePartCache* cache, VisualSupplementalKind kind, std::uint64_t detail_key,
                  const EndpointFrame& source, const std::vector<Vec3d>& points,
                  double radius_m, std::uint32_t color_rgba) {
  if (cache == nullptr || points.size() < 2) return;
  VisualCurvePart part{};
  part.kind = VisualCurvePartKind::kSupplemental;
  part.supplemental_kind = kind;
  part.source_node_id = source.node_id;
  part.source_edge_id = source.edge_id;
  part.source_span_id = source.span_id;
  part.source_bundle_id = source.bundle_id;
  part.bundle_template_id = source.template_id;
  part.lane_index = source.lane_index;
  part.detail_key = detail_key;
  part.wire_radius_m = radius_m;
  part.color_rgba = color_rgba;
  part.material_style = CableMaterialStyleKind::kGeneric;
  part.samples = points;
  part.bounds = bounds_for(part.samples);
  part.source_version = version_for(hash_combine(source.source_version, detail_key), part.samples);
  cache->parts.push_back(std::move(part));
}

void append_model(VisualModelInstanceCache* cache, std::string stable_key, std::string model_key,
                  std::uint64_t content_version, const Vec3d& position, Vec3d forward,
                  const Vec3d& scale) {
  if (cache == nullptr) return;
  VisualModelInstance instance{};
  instance.stable_key = std::move(stable_key);
  instance.model_key = std::move(model_key);
  instance.content_version = content_version == 0 ? 1 : content_version;
  instance.world_transform.position = position;
  instance.world_transform.rotation_euler_deg = {0.0, 0.0, yaw_deg_from_forward(forward)};
  instance.world_transform.scale = scale;
  cache->instances.push_back(std::move(instance));
}

[[nodiscard]] std::string key(const char* prefix, ObjectId id, std::uint64_t detail_key) {
  std::ostringstream out{};
  out << prefix << ':' << id << ':' << detail_key;
  return out.str();
}

void append_endpoint(NodeGroup* group, EndpointFrame endpoint) {
  if (group == nullptr) return;
  const auto existing = std::find_if(group->lanes.begin(), group->lanes.end(),
                                    [&](const EndpointFrame& item) {
                                      return item.lane_index == endpoint.lane_index;
                                    });
  if (existing == group->lanes.end() ||
      std::tie(endpoint.span_id, endpoint.edge_id) < std::tie(existing->span_id, existing->edge_id)) {
    if (existing == group->lanes.end()) {
      group->lanes.push_back(std::move(endpoint));
    } else {
      *existing = std::move(endpoint);
    }
  }
}

void collect_endpoint(std::unordered_map<std::uint64_t, NodeGroup>* groups,
                      const VisualCurvePart& carrier, ObjectId node_id,
                      const Vec3d& position, Vec3d forward) {
  if (groups == nullptr || node_id == kInvalidObjectId) return;
  const std::uint64_t key_value =
      hash_combine(hash_combine(node_id, carrier.source_bundle_id), carrier.bundle_template_id);
  NodeGroup& group = (*groups)[key_value];
  group.node_id = node_id;
  group.edge_id = carrier.source_edge_id;
  group.bundle_id = carrier.source_bundle_id;
  group.template_id = carrier.bundle_template_id;
  group.source_version = hash_combine(group.source_version, carrier.source_version);
  append_endpoint(&group, {node_id,
                           carrier.source_edge_id,
                           carrier.source_span_id,
                           carrier.source_bundle_id,
                           carrier.bundle_template_id,
                           carrier.lane_index,
                           position,
                           unit_or(forward, WorldForward()),
                           carrier.source_version});
}

[[nodiscard]] Vec3d centroid(const std::vector<EndpointFrame>& lanes) {
  Vec3d out{};
  for (const EndpointFrame& lane : lanes) out = out + lane.position;
  return lanes.empty() ? out : ScaleVec(out, 1.0 / static_cast<double>(lanes.size()));
}

void append_support_detail(const NodeGroup& group, DetailVisuals* out) {
  if (out == nullptr || group.lanes.empty()) return;
  std::vector<EndpointFrame> lanes = group.lanes;
  std::sort(lanes.begin(), lanes.end(), [](const EndpointFrame& a, const EndpointFrame& b) {
    return a.lane_index < b.lane_index;
  });
  Vec3d forward{};
  for (const EndpointFrame& lane : lanes) forward = forward + lane.forward;
  forward = horizontal_or(forward);
  const Vec3d lateral = unit_or(Cross(WorldUp(), forward), {0.0, 1.0, 0.0});
  const Vec3d center = centroid(lanes);
  const Vec3d equipment = center + ScaleVec(lateral, 0.38) + ScaleVec(WorldUp(), -0.44);
  const DetailTemplate& transformer = transformer_template();
  const DetailTemplate& terminal_tmpl = terminal_template();
  const std::uint64_t equipment_key = hash_combine(group.source_version, group.node_id);
  append_model(&out->models, key("detail:transformer", group.node_id, equipment_key),
               transformer.model_key, equipment_key, equipment, forward, transformer.approximate_size);

  const double spacing = lanes.size() > 1 ? 0.22 : 0.0;
  const double origin = -0.5 * spacing * static_cast<double>(lanes.size() - 1);
  for (std::size_t index = 0; index < lanes.size(); ++index) {
    const EndpointFrame& lane = lanes[index];
    const double offset = origin + spacing * static_cast<double>(index);
    const char* socket_name = index == 0 ? "hv_in_left" : (index == lanes.size() - 1 ? "hv_in_right" : "hv_in_center");
    Vec3d terminal = template_point(transformer, socket_name, equipment, lateral);
    terminal = terminal + ScaleVec(lateral, offset - (origin + spacing * std::min<std::size_t>(index, 2)));
    const std::uint64_t terminal_key = detail_key_for(0x7151'0001u + index, lane);
    append_model(&out->models, key("detail:terminal", lane.node_id, terminal_key),
                 terminal_tmpl.model_key, terminal_key, terminal, forward, terminal_tmpl.approximate_size);

    const Vec3d guide_a = lane.position + ScaleVec(lane.forward, 0.22) +
                          ScaleVec(WorldUp(), -0.10 - 0.03 * static_cast<double>(index));
    const Vec3d guide_b = template_point(transformer, "top_loop", equipment, lateral) +
                          ScaleVec(lateral, offset * 0.65) +
                          ScaleVec(WorldUp(), 0.04 * static_cast<double>(index));
    append_cable(&out->curves, VisualSupplementalKind::kLocalDetailCable, terminal_key, lane,
                 local_cable_points(lane.position, lane.forward, guide_a, guide_b,
                                    ScaleVec(WorldUp(), 1.0), terminal),
                 0.012, kHvDetailCableColor);
  }

  const EndpointFrame seed = lanes.front();
  const Vec3d loop_start = equipment + ScaleVec(lateral, -0.14) + ScaleVec(WorldUp(), -0.08);
  const Vec3d loop_end = equipment + ScaleVec(lateral, 0.14) + ScaleVec(WorldUp(), -0.09);
  const std::uint64_t loop_key = detail_key_for(0x7151'00F0u, seed);
  append_cable(&out->curves, VisualSupplementalKind::kLocalDetailCable, loop_key, seed,
               local_cable_points(loop_start, ScaleVec(lateral, -1.0),
                                  template_point(transformer, "clutter_left", equipment, lateral),
                                  template_point(transformer, "clutter_right", equipment, lateral),
                                  lateral, loop_end),
               0.010, kDeviceCableColor);
}

void append_inline_detail(const VisualCurvePart& carrier, DetailVisuals* out) {
  if (out == nullptr || carrier.samples.size() < 4) return;
  const std::size_t mid = carrier.samples.size() / 2;
  const Vec3d before = carrier.samples[mid - 1];
  const Vec3d after = carrier.samples[std::min(mid + 1, carrier.samples.size() - 1)];
  const Vec3d tangent = unit_or(after - before, WorldForward());
  const Vec3d lateral = unit_or(Cross(WorldUp(), tangent), {0.0, 1.0, 0.0});
  const Vec3d position = carrier.samples[mid] + ScaleVec(lateral, 0.045);
  const DetailTemplate& inline_tmpl = inline_device_template();
  const std::uint64_t inline_key = detail_key_for(0x1A11'1E01u, carrier);
  append_model(&out->models, key("detail:inline", carrier.source_span_id, inline_key),
               inline_tmpl.model_key, inline_key, position, tangent, inline_tmpl.approximate_size);

  EndpointFrame endpoint{carrier.source_node_id,
                         carrier.source_edge_id,
                         carrier.source_span_id,
                         carrier.source_bundle_id,
                         carrier.bundle_template_id,
                         carrier.lane_index,
                         position,
                         tangent,
                         carrier.source_version};
  const Vec3d start = position - ScaleVec(tangent, 0.34);
  const Vec3d end = position + ScaleVec(tangent, 0.34);
  const Vec3d bypass = template_point(inline_tmpl, "side_bypass", position, lateral);
  append_cable(&out->curves, VisualSupplementalKind::kInlineDetailCable, inline_key, endpoint,
               local_cable_points(start, tangent,
                                  bypass,
                                  bypass + ScaleVec(WorldUp(), -0.08),
                                  tangent, end),
               0.009, kDeviceCableColor);
}

void sort_detail(DetailVisuals* detail) {
  if (detail == nullptr) return;
  std::sort(detail->curves.parts.begin(), detail->curves.parts.end(),
            [](const VisualCurvePart& a, const VisualCurvePart& b) {
              return std::tie(a.source_node_id, a.source_span_id, a.source_bundle_id,
                              a.bundle_template_id, a.lane_index, a.detail_key) <
                     std::tie(b.source_node_id, b.source_span_id, b.source_bundle_id,
                              b.bundle_template_id, b.lane_index, b.detail_key);
            });
  std::sort(detail->models.instances.begin(), detail->models.instances.end(),
            [](const VisualModelInstance& a, const VisualModelInstance& b) {
              return a.stable_key < b.stable_key;
            });
}

} // namespace

DetailVisuals make_detail_visuals(const CoreState& state, const VisualCurvePartCache& carriers) {
  DetailVisuals out{};
  std::unordered_map<std::uint64_t, NodeGroup> node_groups{};
  const VisualCurvePart* inline_carrier = nullptr;
  for (const VisualCurvePart& carrier : carriers.parts) {
    if (carrier.kind != VisualCurvePartKind::kEdgeBody ||
        !is_hv_template(state, carrier.bundle_template_id) ||
        carrier.samples.size() < 2) {
      continue;
    }
    if (inline_carrier == nullptr ||
        std::tie(carrier.source_span_id, carrier.lane_index) <
            std::tie(inline_carrier->source_span_id, inline_carrier->lane_index)) {
      inline_carrier = &carrier;
    }
    const SavedBackboneEdge* edge = state.view().backbone_edge(carrier.source_edge_id);
    if (edge == nullptr) continue;
    collect_endpoint(&node_groups, carrier, edge->node_a, carrier.samples.front(), carrier.tangent_a);
    collect_endpoint(&node_groups, carrier, edge->node_b, carrier.samples.back(), carrier.tangent_b);
  }

  std::vector<NodeGroup> groups{};
  groups.reserve(node_groups.size());
  for (auto& item : node_groups) groups.push_back(std::move(item.second));
  std::sort(groups.begin(), groups.end(), [](const NodeGroup& a, const NodeGroup& b) {
    return std::tie(a.node_id, a.bundle_id, a.template_id) < std::tie(b.node_id, b.bundle_id, b.template_id);
  });
  for (const NodeGroup& group : groups) {
    append_support_detail(group, &out);
  }
  if (inline_carrier != nullptr) {
    append_inline_detail(*inline_carrier, &out);
  }
  sort_detail(&out);
  return out;
}

} // namespace city::wire::generation::backbone
