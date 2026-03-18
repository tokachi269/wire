#include "wire/core/core_state.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace {

using wire::core::BackboneSpec;
using wire::core::BundleKind;
using wire::core::ObjectId;
using wire::core::PathDirectionMode;
using wire::core::SpanLayer;
using wire::core::SupportKind;
using wire::core::Vec3d;

struct ParsedRequest {
  BackboneSpec spec{};
  bool available = false;
  struct BackboneNode {
    ObjectId node_id = wire::core::kInvalidObjectId;
    SupportKind support_kind = SupportKind::kPole;
    Vec3d position{};
    double pole_layout_yaw_deg = 0.0;
    bool has_pole_layout_yaw = false;
  };
  struct BackboneEdge {
    ObjectId node_a_id = wire::core::kInvalidObjectId;
    ObjectId node_b_id = wire::core::kInvalidObjectId;
  };
  struct CurrentSpan {
    ObjectId span_id = wire::core::kInvalidObjectId;
    ObjectId endpoint_node_a_id = wire::core::kInvalidObjectId;
    ObjectId endpoint_node_b_id = wire::core::kInvalidObjectId;
  };

  std::vector<BackboneNode> backbone_nodes{};
  std::vector<BackboneEdge> backbone_edges{};
  std::vector<CurrentSpan> current_spans{};
  std::vector<ObjectId> last_generated_pole_ids{};
  std::vector<ObjectId> last_generated_span_ids{};
};

bool starts_with(std::string_view value, std::string_view prefix) {
  return value.substr(0, prefix.size()) == prefix;
}

std::optional<std::pair<std::string, std::string>> split_key_value(const std::string& line) {
  const std::size_t pos = line.find('=');
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  return std::pair{line.substr(0, pos), line.substr(pos + 1)};
}

bool parse_vec3(const std::string& text, Vec3d* out) {
  if (out == nullptr) {
    return false;
  }
  char comma_a = 0;
  char comma_b = 0;
  std::istringstream iss(text);
  Vec3d value{};
  if (!(iss >> value.x >> comma_a >> value.y >> comma_b >> value.z) || comma_a != ',' || comma_b != ',') {
    return false;
  }
  *out = value;
  return true;
}

SupportKind parse_support_kind(const std::string& text) {
  if (text == "Midair") {
    return SupportKind::kMidair;
  }
  if (text == "Building") {
    return SupportKind::kBuilding;
  }
  return SupportKind::kPole;
}

PathDirectionMode parse_direction_mode(const std::string& text) {
  if (text == "Forward") {
    return PathDirectionMode::kForward;
  }
  if (text == "Reverse") {
    return PathDirectionMode::kReverse;
  }
  return PathDirectionMode::kAuto;
}

bool parse_indexed_key(const std::string& key, std::string_view prefix, std::size_t* index, std::string* suffix) {
  if (!starts_with(key, prefix)) {
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

std::optional<ParsedRequest> parse_capture_request(const std::filesystem::path& path, std::string* error) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    if (error != nullptr) {
      *error = "failed to open capture";
    }
    return std::nullopt;
  }

  ParsedRequest parsed{};
  std::map<std::size_t, Vec3d> path_points{};
  std::map<std::size_t, wire::core::BackboneInputSpec::NodeSpec> node_specs{};
  std::map<std::size_t, wire::core::BackboneBundleSpec> bundles{};
  std::map<std::size_t, ParsedRequest::BackboneNode> backbone_nodes{};
  std::map<std::size_t, ParsedRequest::BackboneEdge> backbone_edges{};
  std::map<std::size_t, ParsedRequest::CurrentSpan> current_spans{};

  std::string line{};
  while (std::getline(ifs, line)) {
    const auto kv = split_key_value(line);
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
    if (parse_indexed_key(key, "capture.last_generated_pole_id[", &index, &suffix) && suffix.empty()) {
      parsed.last_generated_pole_ids.push_back(static_cast<ObjectId>(std::stoull(value)));
      continue;
    }
    if (parse_indexed_key(key, "capture.last_generated_span_id[", &index, &suffix) && suffix.empty()) {
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
      parsed.spec.direction_mode = parse_direction_mode(value);
      continue;
    }
    if (parse_indexed_key(key, "request.path[", &index, &suffix) && suffix.empty()) {
      Vec3d point{};
      if (parse_vec3(value, &point)) {
        path_points[index] = point;
      }
      continue;
    }
    if (parse_indexed_key(key, "request.node_spec[", &index, &suffix)) {
      auto& node = node_specs[index];
      if (suffix == ".point_index") {
        node.point_index = static_cast<std::size_t>(std::stoull(value));
      } else if (suffix == ".support_kind") {
        node.support_kind = parse_support_kind(value);
      } else if (suffix == ".node_id") {
        node.node_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".has_tangent_hint") {
        node.has_tangent_hint = (value == "1");
      } else if (suffix == ".tangent_hint") {
        parse_vec3(value, &node.tangent_hint);
      }
      continue;
    }
    if (parse_indexed_key(key, "request.bundle[", &index, &suffix)) {
      auto& bundle = bundles[index];
      if (suffix == ".kind") {
        bundle.bundle_template_id = static_cast<BundleKind>(std::stoi(value));
      } else if (suffix == ".layer") {
        bundle.layer = static_cast<SpanLayer>(std::stoi(value));
      } else if (suffix == ".count") {
        bundle.count = std::stoi(value);
      }
      continue;
    }
    if (parse_indexed_key(key, "result.backbone.node[", &index, &suffix)) {
      auto& node = backbone_nodes[index];
      if (suffix == ".id") {
        node.node_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".support_kind") {
        node.support_kind = parse_support_kind(value);
      } else if (suffix == ".position") {
        parse_vec3(value, &node.position);
      } else if (suffix == ".pole_layout_yaw_deg") {
        node.pole_layout_yaw_deg = std::stod(value);
        node.has_pole_layout_yaw = true;
      }
      continue;
    }
    if (parse_indexed_key(key, "result.backbone.edge[", &index, &suffix)) {
      auto& edge = backbone_edges[index];
      if (suffix == ".node_a_id") {
        edge.node_a_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".node_b_id") {
        edge.node_b_id = static_cast<ObjectId>(std::stoull(value));
      }
      continue;
    }
    if (parse_indexed_key(key, "result.current_span[", &index, &suffix)) {
      auto& span = current_spans[index];
      if (suffix == ".span_id") {
        span.span_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".endpoint_node_a_id") {
        span.endpoint_node_a_id = static_cast<ObjectId>(std::stoull(value));
      } else if (suffix == ".endpoint_node_b_id") {
        span.endpoint_node_b_id = static_cast<ObjectId>(std::stoull(value));
      }
      continue;
    }
  }

  if (!parsed.available) {
    if (error != nullptr) {
      *error = "capture does not contain request.available=1; capture was taken with old viewer";
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
  parsed.backbone_nodes.reserve(backbone_nodes.size());
  for (const auto& [index, node] : backbone_nodes) {
    (void)index;
    parsed.backbone_nodes.push_back(node);
  }
  parsed.backbone_edges.reserve(backbone_edges.size());
  for (const auto& [index, edge] : backbone_edges) {
    (void)index;
    parsed.backbone_edges.push_back(edge);
  }
  parsed.current_spans.reserve(current_spans.size());
  for (const auto& [index, span] : current_spans) {
    (void)index;
    parsed.current_spans.push_back(span);
  }

  if (parsed.spec.path.polyline.size() < 2) {
    if (error != nullptr) {
      *error = "capture request path has fewer than 2 points";
    }
    return std::nullopt;
  }

  return parsed;
}

std::uint64_t stable_edge_key(ObjectId a, ObjectId b) {
  const std::uint64_t lo = static_cast<std::uint64_t>(std::min(a, b));
  const std::uint64_t hi = static_cast<std::uint64_t>(std::max(a, b));
  return (lo << 32) ^ hi;
}

bool restore_pre_request_backbone(const ParsedRequest& parsed, wire::core::CoreState* state, BackboneSpec* remapped_spec,
                                  std::string* error) {
  if (state == nullptr || remapped_spec == nullptr) {
    if (error != nullptr) {
      *error = "restore_pre_request_backbone received null output";
    }
    return false;
  }

  *remapped_spec = parsed.spec;
  const auto& view = state->view();
  const auto& pole_types = view.pole_types();
  if (pole_types.empty()) {
    if (error != nullptr) {
      *error = "no pole types available for replay restore";
    }
    return false;
  }

  const std::vector<BundleKind> restore_templates{BundleKind::kLowVoltage};
  const std::vector<wire::core::ConnectionCategory> restore_categories{wire::core::ConnectionCategory::kLowVoltage};

  wire::core::PoleTypeId restore_pole_type_id = wire::core::kInvalidPoleTypeId;
  auto supports_restore_categories = [&](wire::core::PoleTypeId pole_type_id) {
    for (wire::core::ConnectionCategory category : restore_categories) {
      if (view.count_port_bands(pole_type_id, category) <= 0) {
        return false;
      }
    }
    return true;
  };
  if (pole_types.find(remapped_spec->pole_type_id) != pole_types.end() &&
      supports_restore_categories(remapped_spec->pole_type_id)) {
    restore_pole_type_id = remapped_spec->pole_type_id;
  }
  if (restore_pole_type_id == wire::core::kInvalidPoleTypeId) {
    for (const auto& [pole_type_id, _] : pole_types) {
      if (supports_restore_categories(pole_type_id)) {
        restore_pole_type_id = pole_type_id;
        break;
      }
    }
  }
  if (restore_pole_type_id == wire::core::kInvalidPoleTypeId) {
    restore_pole_type_id = pole_types.begin()->first;
  }

  std::unordered_set<ObjectId> excluded_generated_poles(parsed.last_generated_pole_ids.begin(),
                                                        parsed.last_generated_pole_ids.end());
  std::unordered_map<ObjectId, ParsedRequest::CurrentSpan> current_span_by_id{};
  current_span_by_id.reserve(parsed.current_spans.size());
  for (const auto& span : parsed.current_spans) {
    current_span_by_id[span.span_id] = span;
  }
  std::unordered_set<std::uint64_t> excluded_generated_edges{};
  for (ObjectId span_id : parsed.last_generated_span_ids) {
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
    excluded_generated_edges.insert(stable_edge_key(span.endpoint_node_a_id, span.endpoint_node_b_id));
  }

  std::unordered_map<ObjectId, ObjectId> remapped_node_ids{};
  remapped_node_ids.reserve(parsed.backbone_nodes.size());
  for (const auto& node : parsed.backbone_nodes) {
    if (node.node_id == wire::core::kInvalidObjectId || excluded_generated_poles.contains(node.node_id)) {
      continue;
    }
    if (node.support_kind != SupportKind::kPole) {
      if (error != nullptr) {
        *error = "replay restore currently supports only pole backbone nodes";
      }
      return false;
    }
    wire::core::Transformd tf{};
    tf.position = node.position;
    if (node.has_pole_layout_yaw) {
      tf.rotation_euler_deg.z = node.pole_layout_yaw_deg;
    }
    const auto add_pole = state->AddPole(tf, 10.0, "ReplayPole", wire::core::PoleKind::kConcrete,
                                         wire::core::PlacementMode::kManual);
    if (!add_pole.ok) {
      if (error != nullptr) {
        *error = add_pole.error;
      }
      return false;
    }
    const auto apply_type = state->ApplyPoleType(add_pole.value, restore_pole_type_id);
    if (!apply_type.ok) {
      if (error != nullptr) {
        *error = apply_type.error;
      }
      return false;
    }
    remapped_node_ids[node.node_id] = add_pole.value;
  }

  for (const auto& edge : parsed.backbone_edges) {
    if (edge.node_a_id == wire::core::kInvalidObjectId || edge.node_b_id == wire::core::kInvalidObjectId ||
        edge.node_a_id == edge.node_b_id) {
      continue;
    }
    if (excluded_generated_poles.contains(edge.node_a_id) || excluded_generated_poles.contains(edge.node_b_id) ||
        excluded_generated_edges.contains(stable_edge_key(edge.node_a_id, edge.node_b_id))) {
      continue;
    }
    const auto it_a = remapped_node_ids.find(edge.node_a_id);
    const auto it_b = remapped_node_ids.find(edge.node_b_id);
    if (it_a == remapped_node_ids.end() || it_b == remapped_node_ids.end()) {
      continue;
    }
    for (BundleKind bundle_template_id : restore_templates) {
      wire::core::AddConnectionByPoleOptions options{};
      options.use_bundle_template = true;
      options.bundle_template_id = bundle_template_id;
      const auto add_connection = state->AddConnectionByPole(it_a->second, it_b->second,
                                                             wire::core::ConnectionCategory::kLowVoltage, options);
      if (!add_connection.ok) {
        if (error != nullptr) {
          *error = add_connection.error;
        }
        return false;
      }
    }
  }

  for (auto& node_spec : remapped_spec->path.node_specs) {
    if (node_spec.node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    const auto it = remapped_node_ids.find(node_spec.node_id);
    if (it == remapped_node_ids.end()) {
      if (error != nullptr) {
        *error = "request node_id could not be remapped from restored backbone";
      }
      return false;
    }
    node_spec.node_id = it->second;
  }
  return true;
}

const char* flow_kind_text(wire::core::BackboneFlowKind kind) {
  return (kind == wire::core::BackboneFlowKind::kBranch) ? "Branch" : "Main";
}

const char* lowering_kind_text(wire::core::BackboneLoweringKind kind) {
  switch (kind) {
  case wire::core::BackboneLoweringKind::kBranchSupport:
    return "BranchSupport";
  case wire::core::BackboneLoweringKind::kCrossUnderpass:
    return "CrossUnderpass";
  case wire::core::BackboneLoweringKind::kAcuteCorner:
    return "AcuteCorner";
  case wire::core::BackboneLoweringKind::kNone:
  default:
    return "None";
  }
}

const char* same_level_reason_text(wire::core::SameLevelFeasibilityReason reason) {
  switch (reason) {
  case wire::core::SameLevelFeasibilityReason::kBundleRule:
    return "BundleRule";
  case wire::core::SameLevelFeasibilityReason::kEnvelopeOverlap:
    return "EnvelopeOverlap";
  case wire::core::SameLevelFeasibilityReason::kNearNodeClearance:
    return "NearNodeClearance";
  case wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled:
    return "CategoryPolicyDisabled";
  case wire::core::SameLevelFeasibilityReason::kNone:
  default:
    return "None";
  }
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: wire_capture_replay <capture.txt>\n";
    return 1;
  }

  std::string error{};
  const auto parsed = parse_capture_request(argv[1], &error);
  if (!parsed.has_value()) {
    std::cerr << "replay error: " << error << "\n";
    return 1;
  }

  wire::core::CoreState state{};
  BackboneSpec remapped_spec{};
  if (!restore_pre_request_backbone(*parsed, &state, &remapped_spec, &error)) {
    std::cerr << "replay restore error: " << error << "\n";
    return 1;
  }
  const auto generated = state.GenerateFromBackboneSpec(remapped_spec);
  if (!generated.ok) {
    std::cerr << "generate error: " << generated.error << "\n";
    return 1;
  }

  const auto view = state.view();
  std::cout << "restored_poles=" << view.poles().size() << "\n";
  std::cout << "generated_poles=" << generated.value.generated_pole_ids.size() << "\n";
  std::cout << "generated_spans=" << generated.value.generated_span_ids.size() << "\n";
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto span_view = view.inspect_span(span_id);
    const auto layout_view = view.inspect_support_layout(span_id);
    if (!span_view.has_value()) {
      continue;
    }
    std::cout << "span_id=" << span_id
              << " bundle_id=" << (span_view->bundle_ref.valid() ? span_view->bundle_ref.stable_id : 0)
              << " flow=" << flow_kind_text(span_view->flow_kind)
              << " lowering=" << lowering_kind_text(span_view->lowering_kind)
              << " same_level=" << (span_view->same_level_feasible ? 1 : 0)
              << " reason=" << same_level_reason_text(span_view->same_level_reason)
              << " lowered_groups=" << (layout_view.has_value() ? layout_view->lowered_support_groups.size() : 0)
              << "\n";
  }

  return 0;
}