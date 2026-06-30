#include "helpers.hpp"

#include "wire/core/coord_utils.hpp"
#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace helpers {
namespace {

constexpr double kPi = 3.14159265358979323846;

double length_xy(const wire::core::Vec3d& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

double distance3d(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d d = a - b;
  return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

double distance_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d d = a - b;
  return std::sqrt(d.x * d.x + d.y * d.y);
}

double axis_angle_deg(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d na = normalize_xy_safe(a);
  const wire::core::Vec3d nb = normalize_xy_safe(b);
  const double la = length_xy(na);
  const double lb = length_xy(nb);
  if (la <= 1e-9 || lb <= 1e-9) {
    return 0.0;
  }
  const double c = std::clamp(std::abs(dot_xy(na, nb)), 0.0, 1.0);
  return std::acos(c) * (180.0 / kPi);
}

wire::core::Vec3d farthest_pair_axis_xy(const std::vector<wire::core::Vec3d>& points) {
  wire::core::Vec3d best{0.0, 0.0, 0.0};
  double best_len2 = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      const wire::core::Vec3d d = points[j] - points[i];
      const double len2 = d.x * d.x + d.y * d.y;
      if (len2 > best_len2) {
        best = d;
        best_len2 = len2;
      }
    }
  }
  return normalize_xy_safe(best);
}

double min_pairwise_distance3d(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      min_distance = std::min(min_distance, distance3d(points[i], points[j]));
    }
  }
  return std::isfinite(min_distance) ? min_distance : 0.0;
}

double min_pairwise_distance_xy(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      min_distance = std::min(min_distance, distance_xy(points[i], points[j]));
    }
  }
  return std::isfinite(min_distance) ? min_distance : 0.0;
}

double mean_pairwise_distance_xy(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double sum = 0.0;
  int count = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      sum += distance_xy(points[i], points[j]);
      ++count;
    }
  }
  return (count > 0) ? (sum / static_cast<double>(count)) : 0.0;
}

const wire::core::Span* find_span_by_ports(const CoreState& state, ObjectId port_a_id, ObjectId port_b_id) {
  for (const wire::core::Span& span : state.view().edit_state().spans.items()) {
    const bool same_forward = span.port_a_id == port_a_id && span.port_b_id == port_b_id;
    const bool same_reverse = span.port_a_id == port_b_id && span.port_b_id == port_a_id;
    if (same_forward || same_reverse) {
      return &span;
    }
  }
  return nullptr;
}

struct ParsedCaptureRequest {
  wire::core::BackboneSpec spec{};
  bool available = false;
  struct BackboneNode {
    ObjectId node_id = wire::core::kInvalidObjectId;
    wire::core::SupportKind support_kind = wire::core::SupportKind::kPole;
    wire::core::Vec3d position{};
    double pole_layout_yaw_deg = 0.0;
    bool has_pole_layout_yaw = false;
  };
  struct BackboneEdge {
    ObjectId node_a_id = wire::core::kInvalidObjectId;
    ObjectId node_b_id = wire::core::kInvalidObjectId;
  };
  struct CurrentSpan {
    ObjectId span_id = wire::core::kInvalidObjectId;
    ObjectId bundle_id = wire::core::kInvalidObjectId;
    ObjectId endpoint_node_a_id = wire::core::kInvalidObjectId;
    ObjectId endpoint_node_b_id = wire::core::kInvalidObjectId;
  };
  struct CurrentBundle {
    ObjectId bundle_id = wire::core::kInvalidObjectId;
    wire::core::BundleKind bundle_template_id = wire::core::BundleKind::kLowVoltage;
  };
  struct CurrentPole {
    ObjectId pole_id = wire::core::kInvalidObjectId;
    wire::core::PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  };
  std::vector<BackboneNode> backbone_nodes{};
  std::vector<BackboneEdge> backbone_edges{};
  std::vector<CurrentSpan> current_spans{};
  std::vector<CurrentBundle> current_bundles{};
  std::vector<CurrentPole> current_poles{};
  std::vector<ObjectId> last_generated_pole_ids{};
  std::vector<ObjectId> last_generated_span_ids{};
};

std::optional<std::pair<std::string, std::string>> split_key_value_test(const std::string& line) {
  const std::size_t pos = line.find('=');
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  return std::pair{line.substr(0, pos), line.substr(pos + 1)};
}

bool parse_indexed_key_test(const std::string& key, std::string_view prefix, std::size_t* index, std::string* suffix) {
  if (!starts_with(key, std::string(prefix))) {
    return false;
  }
  const std::size_t begin = prefix.size();
  const std::size_t end = key.find(']', begin);
  if (end == std::string::npos) {
    return false;
  }
  try {
    *index = static_cast<std::size_t>(std::stoull(key.substr(begin, end - begin)));
  } catch (...) {
    return false;
  }
  *suffix = key.substr(end + 1);
  return true;
}

bool parse_vec3_test(const std::string& text, wire::core::Vec3d* out) {
  if (out == nullptr) {
    return false;
  }
  char comma_a = 0;
  char comma_b = 0;
  std::istringstream iss(text);
  wire::core::Vec3d value{};
  if (!(iss >> value.x >> comma_a >> value.y >> comma_b >> value.z) || comma_a != ',' || comma_b != ',') {
    return false;
  }
  *out = value;
  return true;
}

wire::core::SupportKind parse_support_kind_test(const std::string& text) {
  if (text == "Midair") {
    return wire::core::SupportKind::kMidair;
  }
  if (text == "Building") {
    return wire::core::SupportKind::kExternal;
  }
  if (text == "Ground") {
    return wire::core::SupportKind::kGround;
  }
  return wire::core::SupportKind::kPole;
}

wire::core::PathDirectionMode parse_direction_mode_test(const std::string& text) {
  if (text == "Forward") {
    return wire::core::PathDirectionMode::kForward;
  }
  if (text == "Reverse") {
    return wire::core::PathDirectionMode::kReverse;
  }
  return wire::core::PathDirectionMode::kAuto;
}

std::optional<ParsedCaptureRequest> parse_capture_request_test(const std::filesystem::path& path, std::string* error) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    if (error != nullptr) {
      *error = "failed to open capture";
    }
    return std::nullopt;
  }

  ParsedCaptureRequest parsed{};
  std::map<std::size_t, wire::core::Vec3d> path_points{};
  std::map<std::size_t, wire::core::BackboneInputSpec::NodeSpec> node_specs{};
  std::map<std::size_t, wire::core::BackboneBundleSpec> bundles{};
  std::map<std::size_t, ParsedCaptureRequest::BackboneNode> backbone_nodes{};
  std::map<std::size_t, ParsedCaptureRequest::BackboneEdge> backbone_edges{};
  std::map<std::size_t, ParsedCaptureRequest::CurrentSpan> current_spans{};
  std::map<std::size_t, ParsedCaptureRequest::CurrentBundle> current_bundles{};
  std::map<std::size_t, ParsedCaptureRequest::CurrentPole> current_poles{};

  std::string line{};
  while (std::getline(ifs, line)) {
    const auto kv = split_key_value_test(line);
    if (!kv.has_value()) {
      continue;
    }
    const std::string& key = kv->first;
    const std::string& value = kv->second;
    std::size_t index = 0;
    std::string suffix{};

    if (key == "request.available") {
      parsed.available = (value == "1");
      continue;
    }
    if (parse_indexed_key_test(key, "capture.last_generated_pole_id[", &index, &suffix) && suffix.empty()) {
      parsed.last_generated_pole_ids.push_back(static_cast<ObjectId>(std::stoull(value)));
      continue;
    }
    if (parse_indexed_key_test(key, "capture.last_generated_span_id[", &index, &suffix) && suffix.empty()) {
      parsed.last_generated_span_ids.push_back(static_cast<ObjectId>(std::stoull(value)));
      continue;
    }
    if (key == "request.interval_m") {
      parsed.spec.interval_m = std::stod(value);
      continue;
    }
    if (key == "request.pole_type_id") {
      parsed.spec.pole_type_id = static_cast<wire::core::PoleTypeId>(std::stoul(value));
      continue;
    }
    if (key == "request.direction_mode") {
      parsed.spec.direction_mode = parse_direction_mode_test(value);
      continue;
    }
    if (parse_indexed_key_test(key, "request.path[", &index, &suffix) && suffix.empty()) {
      wire::core::Vec3d point{};
      if (parse_vec3_test(value, &point)) {
        path_points[index] = point;
      }
      continue;
    }
    if (parse_indexed_key_test(key, "request.node_spec[", &index, &suffix)) {
      auto& node = node_specs[index];
      if (suffix == ".point_index") {
        node.point_index = static_cast<std::size_t>(std::stoull(value));
      } else if (suffix == ".support_kind") {
        node.support_kind = parse_support_kind_test(value);
      } else if (suffix == ".node_id") {
        node.node_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".has_tangent_hint") {
        node.has_tangent_hint = (value == "1");
      } else if (suffix == ".tangent_hint") {
        parse_vec3_test(value, &node.tangent_hint);
      }
      continue;
    }
    if (parse_indexed_key_test(key, "request.bundle[", &index, &suffix)) {
      auto& bundle = bundles[index];
      if (suffix == ".kind") {
        bundle.bundle_template_id = static_cast<wire::core::BundleKind>(std::stoi(value));
      } else if (suffix == ".layer") {
        bundle.layer = static_cast<wire::core::SpanLayer>(std::stoi(value));
      } else if (suffix == ".count") {
        bundle.count = std::stoi(value);
      }
      continue;
    }
    if (parse_indexed_key_test(key, "result.backbone.node[", &index, &suffix)) {
      auto& node = backbone_nodes[index];
      if (suffix == ".id") {
        node.node_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".support_kind") {
        node.support_kind = parse_support_kind_test(value);
      } else if (suffix == ".position") {
        parse_vec3_test(value, &node.position);
      } else if (suffix == ".pole_layout_yaw_deg") {
        node.pole_layout_yaw_deg = std::stod(value);
        node.has_pole_layout_yaw = true;
      }
      continue;
    }
    if (parse_indexed_key_test(key, "result.backbone.edge[", &index, &suffix)) {
      auto& edge = backbone_edges[index];
      if (suffix == ".node_a_id") {
        edge.node_a_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".node_b_id") {
        edge.node_b_id = static_cast<ObjectId>(std::stoull(value));
      }
      continue;
    }
    if (parse_indexed_key_test(key, "result.current_span[", &index, &suffix)) {
      auto& span = current_spans[index];
      if (suffix == ".span_id") {
        span.span_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".bundle_id") {
        span.bundle_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".endpoint_node_a_id") {
        span.endpoint_node_a_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".endpoint_node_b_id") {
        span.endpoint_node_b_id = static_cast<ObjectId>(std::stoull(value));
      }
      continue;
    }
    if (parse_indexed_key_test(key, "result.current_bundle[", &index, &suffix)) {
      auto& bundle = current_bundles[index];
      if (suffix == ".bundle_id") {
        bundle.bundle_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".bundle_template_id") {
        bundle.bundle_template_id = static_cast<wire::core::BundleKind>(std::stoi(value));
      }
      continue;
    }
    if (parse_indexed_key_test(key, "result.current_pole[", &index, &suffix)) {
      auto& pole = current_poles[index];
      if (suffix == ".pole_id") {
        pole.pole_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".pole_type_id") {
        pole.pole_type_id = static_cast<wire::core::PoleTypeId>(std::stoul(value));
      }
      continue;
    }
  }

  if (!parsed.available) {
    if (error != nullptr) {
      *error = "capture does not contain request.available=1";
    }
    return std::nullopt;
  }
  for (const auto& [index, point] : path_points) {
    if (index >= parsed.spec.path.polyline.size()) {
      parsed.spec.path.polyline.resize(index + 1);
    }
    parsed.spec.path.polyline[index] = point;
  }
  for (const auto& [index, node] : node_specs) {
    if (index >= parsed.spec.path.node_specs.size()) {
      parsed.spec.path.node_specs.resize(index + 1);
    }
    parsed.spec.path.node_specs[index] = node;
  }
  for (const auto& [index, bundle] : bundles) {
    if (index >= parsed.spec.bundles.size()) {
      parsed.spec.bundles.resize(index + 1);
    }
    parsed.spec.bundles[index] = bundle;
  }
  for (const auto& [_, node] : backbone_nodes) {
    parsed.backbone_nodes.push_back(node);
  }
  for (const auto& [_, edge] : backbone_edges) {
    parsed.backbone_edges.push_back(edge);
  }
  for (const auto& [_, span] : current_spans) {
    parsed.current_spans.push_back(span);
  }
  for (const auto& [_, bundle] : current_bundles) {
    parsed.current_bundles.push_back(bundle);
  }
  for (const auto& [_, pole] : current_poles) {
    parsed.current_poles.push_back(pole);
  }

  if (parsed.spec.path.polyline.size() < 2) {
    if (error != nullptr) {
      *error = "capture request path has fewer than 2 points";
    }
    return std::nullopt;
  }
  return parsed;
}

std::uint64_t stable_edge_key_test(ObjectId a, ObjectId b) {
  const std::uint64_t lo = static_cast<std::uint64_t>(std::min(a, b));
  const std::uint64_t hi = static_cast<std::uint64_t>(std::max(a, b));
  return (lo << 32) ^ hi;
}

} // namespace

CoreCounts snapshot_counts(const CoreState& state) {
  return {
      state.view().edit_state().poles.size(),   state.view().edit_state().ports.size(), state.view().edit_state().anchors.size(),
      state.view().edit_state().bundles.size(), state.view().edit_state().spans.size(), state.view().edit_state().attachments.size(),
  };
}

bool same_counts(const CoreCounts& a, const CoreCounts& b) {
  return a.poles == b.poles && a.ports == b.ports && a.anchors == b.anchors && a.bundles == b.bundles &&
         a.spans == b.spans && a.attachments == b.attachments;
}

bool regex_contains(const std::string& text, const std::string& pattern) {
  return std::regex_search(text, std::regex(pattern));
}

bool contains_id(const std::vector<ObjectId>& ids, ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool almost_equal(double a, double b, double eps) { return std::abs(a - b) <= eps; }

wire::core::EditResult<BackboneFixture>
make_backbone_fixture(CoreState& state, const std::vector<wire::core::Vec3d>& points,
                 const std::vector<wire::core::BundleKind>& bundles) {
  wire::core::EditResult<BackboneFixture> out{};
  if (points.size() < 2 || bundles.empty()) {
    out.error = "backbone fixture requires points and bundles";
    return out;
  }
  wire::core::BackboneSpec request{};
  request.path.polyline = points;
  request.interval_m = 1000.0;
  const auto first_template = state.view().bundle_templates().find(bundles.front());
  if (first_template == state.view().bundle_templates().end()) {
    out.error = "backbone fixture bundle template not found";
    return out;
  }
  request.pole_type_id = first_template->second.related_pole_type_id;
  if (request.pole_type_id == wire::core::kInvalidPoleTypeId) {
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      out.error = "backbone fixture pole type not found";
      return out;
    }
    request.pole_type_id = type_ids.front();
  }
  for (wire::core::BundleKind kind : bundles) {
    wire::core::BackboneBundleSpec spec{};
    spec.bundle_template_id = kind;
    request.bundles.push_back(spec);
  }
  auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok) {
    out.error = generated.error;
    return out;
  }
  out.value.generation = generated.value;
  out.value.poles = generated.value.generated_pole_ids;
  for (ObjectId pole_id : out.value.poles) {
    const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
    if (node == nullptr) {
      out.error = "backbone fixture node binding not found";
      return out;
    }
    out.value.nodes.push_back(node->node_id);
  }
  out.value.spans = generated.value.generated_span_ids;
  out.value.bundles = generated.value.bundle_ids;
  out.ok = true;
  return out;
}

bool almost_equal(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps) {
  return almost_equal(a.x, b.x, eps) && almost_equal(a.y, b.y, eps) && almost_equal(a.z, b.z, eps);
}

wire::core::Vec3d normalize_xy_safe(const wire::core::Vec3d& v) {
  const double len = std::sqrt(v.x * v.x + v.y * v.y);
  if (len <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }
  return {v.x / len, v.y / len, 0.0};
}

double dot_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) { return a.x * b.x + a.y * b.y; }

wire::core::Vec3d local_side_axis_from_yaw(double yaw_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = yaw_deg * (kPi / 180.0);
  return normalize_xy_safe(wire::core::Vec3d{-std::sin(rad), std::cos(rad), 0.0});
}

double angle_diff_abs_deg(double a, double b) {
  double d = std::fmod(a - b, 360.0);
  if (d <= -180.0) {
    d += 360.0;
  } else if (d > 180.0) {
    d -= 360.0;
  }
  return std::abs(d);
}

wire::core::Vec3d rotate_xy_by_yaw_test(const wire::core::Vec3d& local, double yaw_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = yaw_deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {local.x * c - local.y * s, local.x * s + local.y * c, local.z};
}

double effective_pole_yaw_deg_test(const wire::core::Pole& pole) { return pole.world_transform.rotation_euler_deg.z; }

wire::core::Vec3d to_local_on_pole_test(const wire::core::Pole& pole, const wire::core::Vec3d& world) {
  const wire::core::Vec3d delta = world - pole.world_transform.position;
  return rotate_xy_by_yaw_test(delta, -effective_pole_yaw_deg_test(pole));
}

bool aabb_valid(const wire::core::AABBd& aabb) {
  return aabb.min.x <= aabb.max.x && aabb.min.y <= aabb.max.y &&
         wire::core::HeightAlongWorldUp(aabb.min) <= wire::core::HeightAlongWorldUp(aabb.max);
}

bool starts_with(const std::string& value, const std::string& prefix) { return value.rfind(prefix, 0) == 0; }

wire::core::ValidationResult validate_now(CoreState& state) {
  return state.ValidateFast();
}

std::vector<PoleTypeId> sorted_pole_type_ids(const CoreState& state) {
  std::vector<PoleTypeId> ids;
  ids.reserve(state.view().pole_types().size());
  for (const auto& [id, _] : state.view().pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

const wire::core::JunctionInfo* find_junction(const wire::core::BackboneResult& backbone, ObjectId node_id) {
  for (const auto& junction : backbone.junctions) {
    if (junction.node_id == node_id) {
      return &junction;
    }
  }
  return nullptr;
}

const wire::core::SupportNode* find_support_node_by_point_index(const wire::core::BackboneResult& backbone,
                                                                int point_index) {
  for (const auto& node : backbone.nodes) {
    if (node.path_point_index == point_index) {
      return &node;
    }
  }
  return nullptr;
}

ObjectId find_pole_id_by_position(const CoreState& state, const wire::core::Vec3d& pos, double eps) {
  for (const auto& pole : state.view().poles().items()) {
    if (std::abs(pole.world_transform.position.x - pos.x) <= eps &&
        std::abs(pole.world_transform.position.y - pos.y) <= eps &&
        std::abs(pole.world_transform.position.z - pos.z) <= eps) {
      return pole.id;
    }
  }
  return wire::core::kInvalidObjectId;
}

bool is_monotonic(const std::vector<double>& values) {
  if (values.size() < 2) {
    return true;
  }
  bool non_decreasing = true;
  bool non_increasing = true;
  for (std::size_t i = 1; i < values.size(); ++i) {
    if (values[i] + 1e-9 < values[i - 1]) {
      non_decreasing = false;
    }
    if (values[i] > values[i - 1] + 1e-9) {
      non_increasing = false;
    }
  }
  return non_decreasing || non_increasing;
}

void add_backbone_bundle(wire::core::BackboneSpec& req, wire::core::BundleKind template_id,
                         wire::core::SpanLayer layer, int count) {
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = template_id;
  bundle.layer = layer;
  bundle.count = count;
  req.bundles.push_back(bundle);
}

wire::core::BundleKind bundle_template_for_category_test(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return wire::core::BundleKind::kHighVoltage;
  case wire::core::ConnectionCategory::kCommunication:
    return wire::core::BundleKind::kCommunication;
  case wire::core::ConnectionCategory::kOptical:
    return wire::core::BundleKind::kOptical;
  case wire::core::ConnectionCategory::kDrop:
    return wire::core::BundleKind::kDrop;
  case wire::core::ConnectionCategory::kLowVoltage:
  default:
    return wire::core::BundleKind::kLowVoltage;
  }
}

wire::core::ConnectionCategory category_for_bundle_template_test(wire::core::BundleKind bundle_template_id) {
  switch (bundle_template_id) {
  case wire::core::BundleKind::kHighVoltage:
    return wire::core::ConnectionCategory::kHighVoltage;
  case wire::core::BundleKind::kCommunication:
    return wire::core::ConnectionCategory::kCommunication;
  case wire::core::BundleKind::kOptical:
    return wire::core::ConnectionCategory::kOptical;
  case wire::core::BundleKind::kDrop:
    return wire::core::ConnectionCategory::kDrop;
  case wire::core::BundleKind::kLowVoltage:
  default:
    return wire::core::ConnectionCategory::kLowVoltage;
  }
}

AxisRelationMetrics measure_pole_axis_relation_metrics(const CoreState& state, ObjectId pole_id, wire::core::PortLayer layer,
                                                       const wire::core::Vec3d& span_axis) {
  AxisRelationMetrics metrics{};
  metrics.span_chord_axis = normalize_xy_safe(span_axis);
  const auto pole_view = state.view().inspect_pole(pole_id);
  if (!pole_view.has_value()) {
    return metrics;
  }

  std::vector<wire::core::Vec3d> port_points{};
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id == pole_id && port.layer == layer &&
        port.placement_source != wire::core::PortPlacementSourceKind::kBranchSupport) {
      port_points.push_back(port.world_position);
    }
  }
  metrics.row_axis = farthest_pair_axis_xy(port_points);
  if (length_xy(metrics.row_axis) <= 1e-9 && pole_view->has_support_axis) {
    metrics.row_axis = normalize_xy_safe(pole_view->support_axis_dir);
  }

  if (pole_view->has_forward) {
    metrics.support_forward_axis = normalize_xy_safe(pole_view->forward_dir);
  }
  if (length_xy(metrics.support_forward_axis) <= 1e-9 && length_xy(metrics.row_axis) > 1e-9) {
    metrics.support_forward_axis = normalize_xy_safe({metrics.row_axis.y, -metrics.row_axis.x, 0.0});
  }

  metrics.angle_row_vs_span_deg = axis_angle_deg(metrics.row_axis, metrics.span_chord_axis);
  metrics.angle_forward_vs_span_deg = axis_angle_deg(metrics.support_forward_axis, metrics.span_chord_axis);
  metrics.valid = length_xy(metrics.row_axis) > 1e-9 && length_xy(metrics.span_chord_axis) > 1e-9;
  return metrics;
}

BranchRunoutMetrics measure_branch_runout_metrics(const CoreState& state, ObjectId span_id) {
  BranchRunoutMetrics metrics{};
  const wire::core::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  if (layout == nullptr || curve == nullptr) {
    return metrics;
  }

  const bool branch_at_start = layout->start.origin == wire::core::LayoutOriginKind::kBranchSupport ||
                               layout->start.flow_kind == wire::core::BackboneFlowKind::kBranch ||
                               layout->start.local_departure_length_m >= layout->end.local_departure_length_m;
  const wire::core::Vec3d root = branch_at_start ? curve->detail.EvaluatePosition(0.0) : curve->detail.EvaluatePosition(1.0);
  const wire::core::Vec3d other =
      branch_at_start ? curve->detail.EvaluatePosition(1.0) : curve->detail.EvaluatePosition(0.0);
  const wire::core::Vec3d chord_axis = normalize_xy_safe(other - root);
  if (length_xy(chord_axis) <= 1e-9) {
    return metrics;
  }
  const wire::core::Vec3d lateral_axis{-chord_axis.y, chord_axis.x, 0.0};
  const double total = curve->detail.Length();
  metrics.chord_length_m = std::sqrt((other.x - root.x) * (other.x - root.x) + (other.y - root.y) * (other.y - root.y));
  metrics.support_departure_length_m =
      branch_at_start ? layout->start.local_departure_length_m : layout->end.local_departure_length_m;

  for (const wire::core::Vec3d& point : curve->detail.sample_points) {
    const double lateral = std::abs(dot_xy(point - root, lateral_axis));
    metrics.max_lateral_runout_m = std::max(metrics.max_lateral_runout_m, lateral);
  }

  const double departure_s = std::min(std::max(0.05, metrics.support_departure_length_m), std::max(0.05, total));
  const double mid_s = std::clamp(total * 0.5, 0.0, std::max(0.0, total));
  const wire::core::Vec3d departure_point =
      branch_at_start ? curve->detail.PositionAtLength(departure_s) : curve->detail.PositionAtLength(std::max(0.0, total - departure_s));
  const wire::core::Vec3d mid_point = curve->detail.PositionAtLength(mid_s);
  metrics.departure_lateral_offset_m = std::abs(dot_xy(departure_point - root, lateral_axis));
  metrics.midspan_lateral_offset_m = std::abs(dot_xy(mid_point - root, lateral_axis));
  metrics.lateral_runout_ratio =
      (metrics.chord_length_m > 1e-9) ? (metrics.max_lateral_runout_m / metrics.chord_length_m) : 0.0;
  metrics.local_departure_dominates =
      metrics.midspan_lateral_offset_m <= metrics.departure_lateral_offset_m + 0.05 &&
      metrics.max_lateral_runout_m <= std::max(metrics.support_departure_length_m + 0.05,
                                               metrics.departure_lateral_offset_m + 0.08);
  return metrics;
}

std::string describe_axis_relation_metrics(const AxisRelationMetrics& metrics) {
  std::ostringstream oss;
  oss << "rowAxis=" << metrics.row_axis.x << "," << metrics.row_axis.y << " forwardAxis="
      << metrics.support_forward_axis.x << "," << metrics.support_forward_axis.y << " spanAxis="
      << metrics.span_chord_axis.x << "," << metrics.span_chord_axis.y << " angleRowVsSpan="
      << metrics.angle_row_vs_span_deg << " angleForwardVsSpan=" << metrics.angle_forward_vs_span_deg
      << " valid=" << (metrics.valid ? 1 : 0);
  return oss.str();
}

std::string describe_branch_runout_metrics(const BranchRunoutMetrics& metrics) {
  std::ostringstream oss;
  oss << "maxLat=" << metrics.max_lateral_runout_m << " ratio=" << metrics.lateral_runout_ratio
      << " depLat=" << metrics.departure_lateral_offset_m << " midLat=" << metrics.midspan_lateral_offset_m
      << " depLen=" << metrics.support_departure_length_m << " chord=" << metrics.chord_length_m
      << " local=" << (metrics.local_departure_dominates ? 1 : 0);
  return oss.str();
}

wire::core::EditResult<wire::core::AddConnectionByPoleResult>
add_connection_by_category(wire::core::CoreState& state, wire::core::ObjectId pole_a_id, wire::core::ObjectId pole_b_id,
                           wire::core::ConnectionCategory category,
                           wire::core::AddConnectionByPoleOptions options) {
  wire::core::EditResult<wire::core::AddConnectionByPoleResult> out{};
  const wire::core::Pole* a = state.view().poles().find(pole_a_id);
  const wire::core::Pole* b = state.view().poles().find(pole_b_id);
  if (a == nullptr || b == nullptr || pole_a_id == pole_b_id) {
    out.error = "fixture poles are invalid";
    return out;
  }
  const wire::core::BundleKind kind =
      options.use_bundle_template ? options.bundle_template_id : bundle_template_for_category_test(category);
  const auto tmpl = state.view().bundle_templates().find(kind);
  if (tmpl == state.view().bundle_templates().end()) {
    out.error = "fixture bundle template not found";
    return out;
  }
  const int count = tmpl->second.count_rule == wire::core::BundleCountRuleKind::kFixed
                        ? tmpl->second.fixed_count
                        : tmpl->second.default_count;
  const auto bundle = state.AddBundle(count, tmpl->second.default_spacing_m, kind);
  if (!bundle.ok) {
    out.error = bundle.error;
    return out;
  }
  const wire::core::PortLayer port_layer =
      tmpl->second.default_layer == wire::core::SpanLayer::kHighVoltage
          ? wire::core::PortLayer::kHighVoltage
          : tmpl->second.default_layer == wire::core::SpanLayer::kCommunication
                ? wire::core::PortLayer::kCommunication
                : wire::core::PortLayer::kLowVoltage;
  const wire::core::Vec3d pa = a->world_transform.position + wire::core::Vec3d{0.0, 0.0, 9.0};
  const wire::core::Vec3d pb = b->world_transform.position + wire::core::Vec3d{0.0, 0.0, 9.0};
  const auto port_a = state.AddPort(pole_a_id, pa, wire::core::PortKind::kPower, port_layer);
  const auto port_b = state.AddPort(pole_b_id, pb, wire::core::PortKind::kPower, port_layer);
  if (!port_a.ok || !port_b.ok) {
    out.error = "fixture port creation failed";
    return out;
  }
  const auto span = state.AddSpan(port_a.value, port_b.value, wire::core::SpanKind::kDistribution,
                                  tmpl->second.default_layer, bundle.value);
  if (!span.ok) {
    out.error = span.error;
    return out;
  }
  out.value.span_id = span.value;
  out.value.port_a_id = port_a.value;
  out.value.port_b_id = port_b.value;
  out.change_set = span.change_set;
  out.ok = true;
  return out;
}

bool has_selected_port_in_candidates(const wire::core::PortResolutionDebugRecord& record) {
  if (record.selected_port_id == wire::core::kInvalidObjectId) {
    return true;
  }
  if (record.created_new_port) {
    return true;
  }
  for (const auto& candidate : record.candidates) {
    if (candidate.resolved_port_id == record.selected_port_id) {
      return true;
    }
  }
  return false;
}

bool restore_capture_request_scene(const std::filesystem::path& capture_path, CoreState& state,
                                   wire::core::BackboneSpec* remapped_spec, std::string* error) {
  if (remapped_spec == nullptr) {
    if (error != nullptr) {
      *error = "null remapped_spec";
    }
    return false;
  }
  const auto parsed = parse_capture_request_test(capture_path, error);
  if (!parsed.has_value()) {
    return false;
  }

  *remapped_spec = parsed->spec;
  const auto& view = state.view();
  const auto& pole_types = view.pole_types();
  if (pole_types.empty()) {
    if (error != nullptr) {
      *error = "no pole types available";
    }
    return false;
  }

  std::unordered_map<ObjectId, wire::core::BundleKind> bundle_templates_by_id{};
  for (const auto& bundle : parsed->current_bundles) {
    if (bundle.bundle_id != wire::core::kInvalidObjectId) {
      bundle_templates_by_id[bundle.bundle_id] = bundle.bundle_template_id;
    }
  }
  std::unordered_map<std::uint64_t, std::vector<wire::core::BundleKind>> restore_templates_by_edge{};
  wire::core::PoleTypeId restore_pole_type_id = wire::core::kInvalidPoleTypeId;
  auto supports_bundle_template = [&](wire::core::PoleTypeId pole_type_id, wire::core::BundleKind template_id) {
    return view.count_port_bands(pole_type_id, category_for_bundle_template_test(template_id)) > 0;
  };
  auto supports_restore_templates = [&](wire::core::PoleTypeId pole_type_id) {
    if (bundle_templates_by_id.empty()) {
      return supports_bundle_template(pole_type_id, wire::core::BundleKind::kLowVoltage);
    }
    for (const auto& [_, template_id] : bundle_templates_by_id) {
      if (!supports_bundle_template(pole_type_id, template_id)) {
        return false;
      }
    }
    return true;
  };
  std::unordered_set<ObjectId> excluded_generated_poles(parsed->last_generated_pole_ids.begin(),
                                                        parsed->last_generated_pole_ids.end());
  std::unordered_map<wire::core::PoleTypeId, int> current_pole_type_counts{};
  for (const auto& pole : parsed->current_poles) {
    if (pole.pole_id == wire::core::kInvalidObjectId || excluded_generated_poles.contains(pole.pole_id) ||
        pole.pole_type_id == wire::core::kInvalidPoleTypeId) {
      continue;
    }
    ++current_pole_type_counts[pole.pole_type_id];
  }
  int best_restore_pole_count = -1;
  for (const auto& [pole_type_id, count] : current_pole_type_counts) {
    if (pole_types.find(pole_type_id) == pole_types.end() || !supports_restore_templates(pole_type_id)) {
      continue;
    }
    if (count > best_restore_pole_count) {
      best_restore_pole_count = count;
      restore_pole_type_id = pole_type_id;
    }
  }
  if (restore_pole_type_id == wire::core::kInvalidPoleTypeId &&
      pole_types.find(remapped_spec->pole_type_id) != pole_types.end() &&
      supports_restore_templates(remapped_spec->pole_type_id)) {
    restore_pole_type_id = remapped_spec->pole_type_id;
  }
  if (restore_pole_type_id == wire::core::kInvalidPoleTypeId) {
    for (const auto& [pole_type_id, _] : pole_types) {
      if (supports_restore_templates(pole_type_id)) {
        restore_pole_type_id = pole_type_id;
        break;
      }
    }
  }
  if (restore_pole_type_id == wire::core::kInvalidPoleTypeId) {
    restore_pole_type_id = pole_types.begin()->first;
  }
  std::unordered_map<ObjectId, ParsedCaptureRequest::CurrentSpan> current_span_by_id{};
  for (const auto& span : parsed->current_spans) {
    current_span_by_id[span.span_id] = span;
    if (span.bundle_id == wire::core::kInvalidObjectId || span.endpoint_node_a_id == wire::core::kInvalidObjectId ||
        span.endpoint_node_b_id == wire::core::kInvalidObjectId || span.endpoint_node_a_id == span.endpoint_node_b_id) {
      continue;
    }
    if (std::ranges::find(parsed->last_generated_span_ids, span.span_id) != parsed->last_generated_span_ids.end()) {
      continue;
    }
    const auto bundle_it = bundle_templates_by_id.find(span.bundle_id);
    if (bundle_it == bundle_templates_by_id.end()) {
      continue;
    }
    auto& edge_templates = restore_templates_by_edge[stable_edge_key_test(span.endpoint_node_a_id, span.endpoint_node_b_id)];
    if (std::ranges::find(edge_templates, bundle_it->second) == edge_templates.end()) {
      edge_templates.push_back(bundle_it->second);
    }
  }
  std::unordered_set<std::uint64_t> excluded_generated_edges{};
  for (ObjectId span_id : parsed->last_generated_span_ids) {
    const auto it = current_span_by_id.find(span_id);
    if (it == current_span_by_id.end()) {
      continue;
    }
    const auto& span = it->second;
    if (span.endpoint_node_a_id == wire::core::kInvalidObjectId ||
        span.endpoint_node_b_id == wire::core::kInvalidObjectId ||
        span.endpoint_node_a_id == span.endpoint_node_b_id) {
      continue;
    }
    excluded_generated_edges.insert(stable_edge_key_test(span.endpoint_node_a_id, span.endpoint_node_b_id));
  }

  std::unordered_map<ObjectId, ObjectId> remapped_node_ids{};
  std::unordered_map<ObjectId, wire::core::Vec3d> backbone_positions_by_node_id{};
  for (const auto& node : parsed->backbone_nodes) {
    if (node.node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    backbone_positions_by_node_id[node.node_id] = node.position;
  }
  for (const auto& node : parsed->backbone_nodes) {
    if (node.node_id == wire::core::kInvalidObjectId || excluded_generated_poles.contains(node.node_id)) {
      continue;
    }
    if (node.support_kind != wire::core::SupportKind::kPole) {
      continue;
    }
    wire::core::Transformd tf{};
    tf.position = node.position;
    if (node.has_pole_layout_yaw) {
      tf.rotation_euler_deg.z = node.pole_layout_yaw_deg;
    }
    const auto add_pole = state.AddPole(tf, 10.0, "ReplayPole", wire::core::PoleKind::kConcrete,
                                        wire::core::PlacementMode::kManual);
    if (!add_pole.ok) {
      if (error != nullptr) {
        *error = add_pole.error;
      }
      return false;
    }
    const auto apply_type = state.ApplyPoleType(add_pole.value, restore_pole_type_id);
    if (!apply_type.ok) {
      if (error != nullptr) {
        *error = apply_type.error;
      }
      return false;
    }
    remapped_node_ids[node.node_id] = add_pole.value;
  }

  for (const auto& edge : parsed->backbone_edges) {
    if (edge.node_a_id == wire::core::kInvalidObjectId || edge.node_b_id == wire::core::kInvalidObjectId ||
        edge.node_a_id == edge.node_b_id) {
      continue;
    }
    if (excluded_generated_poles.contains(edge.node_a_id) || excluded_generated_poles.contains(edge.node_b_id) ||
        excluded_generated_edges.contains(stable_edge_key_test(edge.node_a_id, edge.node_b_id))) {
      continue;
    }
    const auto it_a = remapped_node_ids.find(edge.node_a_id);
    const auto it_b = remapped_node_ids.find(edge.node_b_id);
    if (it_a == remapped_node_ids.end() || it_b == remapped_node_ids.end()) {
      continue;
    }
    auto templates_it = restore_templates_by_edge.find(stable_edge_key_test(edge.node_a_id, edge.node_b_id));
    std::vector<wire::core::BundleKind> restore_templates =
        (templates_it == restore_templates_by_edge.end() || templates_it->second.empty())
            ? std::vector<wire::core::BundleKind>{wire::core::BundleKind::kLowVoltage}
            : templates_it->second;
    const auto pos_a_it = backbone_positions_by_node_id.find(edge.node_a_id);
    const auto pos_b_it = backbone_positions_by_node_id.find(edge.node_b_id);
    if (pos_a_it == backbone_positions_by_node_id.end() || pos_b_it == backbone_positions_by_node_id.end()) {
      continue;
    }

    wire::core::BackboneSpec edge_request{};
    edge_request.path.polyline = {pos_a_it->second, pos_b_it->second};
    edge_request.interval_m = 1000.0;
    edge_request.pole_type_id = restore_pole_type_id;
    wire::core::BackboneInputSpec::NodeSpec start{};
    start.point_index = 0;
    start.support_kind = wire::core::SupportKind::kPole;
    start.node_id = it_a->second;
    wire::core::BackboneInputSpec::NodeSpec end{};
    end.point_index = 1;
    end.support_kind = wire::core::SupportKind::kPole;
    end.node_id = it_b->second;
    edge_request.path.node_specs.push_back(start);
    edge_request.path.node_specs.push_back(end);
    for (wire::core::BundleKind bundle_template_id : restore_templates) {
      add_backbone_bundle(edge_request, bundle_template_id);
    }
    const auto replay_edge = state.GenerateFromBackboneSpec(edge_request);
    if (!replay_edge.ok) {
      if (error != nullptr) {
        std::ostringstream oss;
        oss << "replay edge " << edge.node_a_id << "-" << edge.node_b_id << " templates=";
        for (std::size_t i = 0; i < restore_templates.size(); ++i) {
          if (i > 0) {
            oss << ",";
          }
          oss << static_cast<int>(restore_templates[i]);
        }
        oss << " error=" << replay_edge.error;
        *error = oss.str();
      }
      return false;
    }
  }

  for (auto& node_spec : remapped_spec->path.node_specs) {
    if (node_spec.node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    const auto it = remapped_node_ids.find(node_spec.node_id);
    if (it == remapped_node_ids.end()) {
      node_spec.node_id = wire::core::kInvalidObjectId;
      node_spec.support_kind = wire::core::SupportKind::kPole;
    } else {
      node_spec.node_id = it->second;
    }
  }
  return true;
}

} // namespace helpers



#include "wire/core/coord_utils.hpp"
