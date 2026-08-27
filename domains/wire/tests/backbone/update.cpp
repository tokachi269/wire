#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "city/wire/core_test_hook.hpp"
#include "city/wire/core_view.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

namespace {

bool same_vec3(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  return almost_equal(a.x, b.x, 1e-12) && almost_equal(a.y, b.y, 1e-12) &&
         almost_equal(a.z, b.z, 1e-12);
}

bool same_visual_curve_samples(const city::wire::VisualCurvePartCache& lhs,
                               const city::wire::VisualCurvePartCache& rhs) {
  if (lhs.parts.size() != rhs.parts.size()) return false;
  for (std::size_t i = 0; i < lhs.parts.size(); ++i) {
    const city::wire::VisualCurvePart& a = lhs.parts[i];
    const city::wire::VisualCurvePart& b = rhs.parts[i];
    if (a.kind != b.kind || a.supplemental_kind != b.supplemental_kind || a.lane_index != b.lane_index ||
        a.has_section_key != b.has_section_key || a.samples.size() != b.samples.size()) {
      return false;
    }
    if (a.has_section_key && (a.section_key.logical_span_id != b.section_key.logical_span_id ||
                              a.section_key.rule_owner_id != b.section_key.rule_owner_id ||
                              a.section_key.rule_id != b.section_key.rule_id ||
                              a.section_key.instance_index != b.section_key.instance_index)) {
      return false;
    }
    for (std::size_t sample = 0; sample < a.samples.size(); ++sample) {
      if (!same_vec3(a.samples[sample], b.samples[sample])) return false;
    }
  }
  return true;
}

std::vector<city::wire::ObjectId> edge_bundle_span_ids(const city::wire::CoreState& state,
                                                       city::wire::ObjectId edge_bundle_id) {
  std::vector<city::wire::ObjectId> out{};
  const auto it = state.view().backbone_index().edge_bundle_spans.find(edge_bundle_id);
  if (it != state.view().backbone_index().edge_bundle_spans.end()) {
    out = it->second;
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool visual_member_samples_differ(const city::wire::VisualCurvePartCache& lhs,
                                  const city::wire::VisualCurvePartCache& rhs) {
  if (lhs.parts.size() != rhs.parts.size()) return true;
  for (std::size_t i = 0; i < lhs.parts.size(); ++i) {
    const city::wire::VisualCurvePart& a = lhs.parts[i];
    const city::wire::VisualCurvePart& b = rhs.parts[i];
    if (a.kind != city::wire::VisualCurvePartKind::kEdgeBody ||
        b.kind != city::wire::VisualCurvePartKind::kEdgeBody) continue;
    if (a.samples.size() != b.samples.size()) return true;
    for (std::size_t sample = 0; sample < a.samples.size(); ++sample) {
      if (!same_vec3(a.samples[sample], b.samples[sample])) return true;
    }
  }
  return false;
}

bool same_frame(const city::wire::Frame3d& a, const city::wire::Frame3d& b) {
  return same_vec3(a.origin, b.origin) && same_vec3(a.forward, b.forward) &&
         same_vec3(a.right, b.right) && same_vec3(a.up, b.up);
}

bool same_band(const city::wire::PortPlacementBand& a, const city::wire::PortPlacementBand& b) {
  return a.band_id == b.band_id && a.category == b.category && same_frame(a.local_direction, b.local_direction) &&
         a.layer == b.layer && a.side == b.side && a.role == b.role &&
         almost_equal(a.lateral_center_m, b.lateral_center_m, 1e-12) &&
         almost_equal(a.lateral_min_m, b.lateral_min_m, 1e-12) &&
         almost_equal(a.lateral_max_m, b.lateral_max_m, 1e-12) &&
         almost_equal(a.height_center_m, b.height_center_m, 1e-12) &&
         almost_equal(a.height_min_m, b.height_min_m, 1e-12) &&
         almost_equal(a.height_max_m, b.height_max_m, 1e-12) && a.priority == b.priority &&
         almost_equal(a.min_spacing_m, b.min_spacing_m, 1e-12) && a.allow_multiple == b.allow_multiple &&
         a.overflow_policy == b.overflow_policy && a.enabled == b.enabled;
}

bool same_pole_type(const city::wire::PoleTypeDefinition& a, const city::wire::PoleTypeDefinition& b) {
  if (a.id != b.id || a.name != b.name || a.description != b.description ||
      !almost_equal(a.default_height_m, b.default_height_m, 1e-12) ||
      !almost_equal(a.radius_base_m, b.radius_base_m, 1e-12) ||
      !almost_equal(a.radius_top_m, b.radius_top_m, 1e-12) ||
      a.port_bands.size() != b.port_bands.size() || a.anchor_slots.size() != b.anchor_slots.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.port_bands.size(); ++i) {
    if (!same_band(a.port_bands[i], b.port_bands[i])) {
      return false;
    }
  }
  return std::equal(a.anchor_slots.begin(), a.anchor_slots.end(), b.anchor_slots.begin(),
                    [](const city::wire::AnchorSlotTemplate& lhs,
                       const city::wire::AnchorSlotTemplate& rhs) {
                      return lhs.slot_id == rhs.slot_id && lhs.usage == rhs.usage &&
                             same_vec3(lhs.local_position, rhs.local_position) &&
                             lhs.priority == rhs.priority && lhs.enabled == rhs.enabled;
                    });
}

std::vector<city::wire::Vec3d> bound_port_positions(const city::wire::CoreState& state) {
  std::vector<city::wire::Vec3d> out{};
  if (state.view().backbone().edge_bundles.empty()) {
    return out;
  }
  const city::wire::ObjectId edge_bundle_id = state.view().backbone().edge_bundles.front().edge_bundle_id;
  for (const city::wire::SavedBackbonePortBinding* binding :
       state.view().backbone_port_bindings_for_edge_bundle(edge_bundle_id)) {
    if (binding == nullptr) {
      continue;
    }
    const city::wire::Port* port = state.view().ports().find(binding->port_id);
    if (port != nullptr) {
      out.push_back(port->world_position);
    }
  }
  std::sort(out.begin(), out.end(), [](const city::wire::Vec3d& lhs, const city::wire::Vec3d& rhs) {
    if (!almost_equal(lhs.x, rhs.x, 1e-12)) {
      return lhs.x < rhs.x;
    }
    if (!almost_equal(lhs.y, rhs.y, 1e-12)) {
      return lhs.y < rhs.y;
    }
    return lhs.z < rhs.z;
  });
  return out;
}

struct SpanCurveSignature {
  std::size_t lane = 0;
  city::wire::Vec3d port_a{};
  city::wire::Vec3d port_b{};
  std::vector<city::wire::Vec3d> samples{};
};

std::vector<SpanCurveSignature> span_curve_signatures(const city::wire::CoreState& state) {
  std::vector<SpanCurveSignature> out{};
  if (state.view().backbone().edge_bundles.empty()) {
    return out;
  }
  const city::wire::ObjectId edge_bundle_id = state.view().backbone().edge_bundles.front().edge_bundle_id;
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.edge_bundle_id != edge_bundle_id) {
      continue;
    }
    const city::wire::Span* span = state.view().spans().find(binding.span_id);
    const auto* curve = state.find_curve_cache(binding.span_id);
    if (span == nullptr || curve == nullptr) {
      return {};
    }
    const city::wire::Port* port_a = state.view().ports().find(span->port_a_id);
    const city::wire::Port* port_b = state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      return {};
    }
    SpanCurveSignature item{};
    item.lane = binding.lane_index;
    item.port_a = port_a->world_position;
    item.port_b = port_b->world_position;
    item.samples = curve->detail.sample_points;
    out.push_back(std::move(item));
  }
  std::sort(out.begin(), out.end(), [](const SpanCurveSignature& lhs, const SpanCurveSignature& rhs) {
    return lhs.lane < rhs.lane;
  });
  return out;
}

bool same_span_curve_signatures(const std::vector<SpanCurveSignature>& lhs,
                                const std::vector<SpanCurveSignature>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].lane != rhs[i].lane || !same_vec3(lhs[i].port_a, rhs[i].port_a) ||
        !same_vec3(lhs[i].port_b, rhs[i].port_b) || lhs[i].samples.size() != rhs[i].samples.size()) {
      return false;
    }
    for (std::size_t j = 0; j < lhs[i].samples.size(); ++j) {
      if (!same_vec3(lhs[i].samples[j], rhs[i].samples[j])) {
        return false;
      }
    }
  }
  return true;
}

std::vector<SpanCurveSignature> span_curve_signatures_for_edge_bundle(const city::wire::CoreState& state,
                                                                      city::wire::ObjectId edge_bundle_id) {
  std::vector<SpanCurveSignature> out{};
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.edge_bundle_id != edge_bundle_id) {
      continue;
    }
    const city::wire::Span* span = state.view().spans().find(binding.span_id);
    const auto* curve = state.find_curve_cache(binding.span_id);
    if (span == nullptr || curve == nullptr) {
      return {};
    }
    const city::wire::Port* port_a = state.view().ports().find(span->port_a_id);
    const city::wire::Port* port_b = state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      return {};
    }
    SpanCurveSignature item{};
    item.lane = binding.lane_index;
    item.port_a = port_a->world_position;
    item.port_b = port_b->world_position;
    item.samples = curve->detail.sample_points;
    out.push_back(std::move(item));
  }
  std::sort(out.begin(), out.end(), [](const SpanCurveSignature& lhs, const SpanCurveSignature& rhs) {
    return lhs.lane < rhs.lane;
  });
  return out;
}

city::wire::ObjectId edge_bundle_id_for_template(const city::wire::CoreState& state,
                                                 city::wire::BundleKind bundle_template_id) {
  const city::wire::BundleTemplateId template_id = city::wire::DefaultBundleTemplateId(bundle_template_id);
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    const city::wire::Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      return edge_bundle.edge_bundle_id;
    }
  }
  return city::wire::kInvalidObjectId;
}

std::vector<city::wire::ObjectId> edge_bundle_ids_for_template(const city::wire::CoreState& state,
                                                               city::wire::BundleKind bundle_template_id) {
  const city::wire::BundleTemplateId template_id = city::wire::DefaultBundleTemplateId(bundle_template_id);
  std::vector<city::wire::ObjectId> out{};
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    const city::wire::Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      out.push_back(edge_bundle.edge_bundle_id);
    }
  }
  return out;
}

std::vector<std::vector<SpanCurveSignature>> route_bundle_signatures(const city::wire::CoreState& state,
                                                                     city::wire::BundleKind bundle_template_id) {
  std::vector<std::vector<SpanCurveSignature>> out{};
  for (city::wire::ObjectId edge_bundle_id : edge_bundle_ids_for_template(state, bundle_template_id)) {
    out.push_back(span_curve_signatures_for_edge_bundle(state, edge_bundle_id));
  }
  return out;
}

std::vector<std::vector<SpanCurveSignature>> route_bundle_signatures_for_ids(
    const city::wire::CoreState& state, const std::vector<city::wire::ObjectId>& edge_bundle_ids) {
  std::vector<std::vector<SpanCurveSignature>> out{};
  out.reserve(edge_bundle_ids.size());
  for (city::wire::ObjectId edge_bundle_id : edge_bundle_ids) {
    out.push_back(span_curve_signatures_for_edge_bundle(state, edge_bundle_id));
  }
  return out;
}

bool same_route_bundle_signatures(const std::vector<std::vector<SpanCurveSignature>>& lhs,
                                  const std::vector<std::vector<SpanCurveSignature>>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (!same_span_curve_signatures(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
}

std::size_t visual_part_count(const city::wire::CoreState& state, city::wire::VisualCurvePartKind kind) {
  return static_cast<std::size_t>(
      std::count_if(state.view().visual_curve_parts().parts.begin(),
                    state.view().visual_curve_parts().parts.end(),
                    [&](const city::wire::VisualCurvePart& part) { return part.kind == kind; }));
}

bool route_bundle_curves_request_policy(const city::wire::CoreState& state,
                                        city::wire::BundleKind bundle_template_id,
                                        city::wire::CableContinuityPolicyHint policy) {
  for (city::wire::ObjectId edge_bundle_id : edge_bundle_ids_for_template(state, bundle_template_id)) {
    for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) {
        continue;
      }
      const city::wire::CurveCacheEntry* curve = state.find_curve_cache(binding.span_id);
      if (curve == nullptr || curve->detail.quality.requested_policy != policy) {
        return false;
      }
    }
  }
  return true;
}

struct EdgeBundleIdentitySnapshot {
  std::vector<city::wire::ObjectId> span_ids{};
  std::vector<std::pair<std::size_t, city::wire::ObjectId>> port_ids{};
  std::vector<std::pair<std::size_t, city::wire::Vec3d>> port_positions{};
};

EdgeBundleIdentitySnapshot edge_bundle_identity_snapshot(const city::wire::CoreState& state,
                                                         city::wire::ObjectId edge_bundle_id) {
  EdgeBundleIdentitySnapshot out{};
  const city::wire::SavedBackboneEdgeBundle* edge_bundle = nullptr;
  for (const city::wire::SavedBackboneEdgeBundle& candidate : state.view().backbone().edge_bundles) {
    if (candidate.edge_bundle_id == edge_bundle_id) {
      edge_bundle = &candidate;
      break;
    }
  }
  if (edge_bundle != nullptr) out.span_ids = edge_bundle_span_ids(state, edge_bundle_id);
  for (const city::wire::SavedBackbonePortBinding* binding :
       state.view().backbone_port_bindings_for_edge_bundle(edge_bundle_id)) {
    if (binding == nullptr) {
      continue;
    }
    out.port_ids.push_back({binding->lane_index, binding->port_id});
    const city::wire::Port* port = state.view().ports().find(binding->port_id);
    if (port != nullptr) {
      out.port_positions.push_back({binding->lane_index, port->world_position});
    }
  }
  std::sort(out.span_ids.begin(), out.span_ids.end());
  std::sort(out.port_ids.begin(), out.port_ids.end());
  std::sort(out.port_positions.begin(), out.port_positions.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
              }
              if (!almost_equal(lhs.second.x, rhs.second.x, 1e-12)) {
                return lhs.second.x < rhs.second.x;
              }
              if (!almost_equal(lhs.second.y, rhs.second.y, 1e-12)) {
                return lhs.second.y < rhs.second.y;
              }
              return lhs.second.z < rhs.second.z;
            });
  return out;
}

bool same_edge_bundle_identity_snapshot(const EdgeBundleIdentitySnapshot& lhs,
                                        const EdgeBundleIdentitySnapshot& rhs) {
  if (lhs.span_ids != rhs.span_ids || lhs.port_ids != rhs.port_ids ||
      lhs.port_positions.size() != rhs.port_positions.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.port_positions.size(); ++i) {
    if (lhs.port_positions[i].first != rhs.port_positions[i].first ||
        !same_vec3(lhs.port_positions[i].second, rhs.port_positions[i].second)) {
      return false;
    }
  }
  return true;
}

bool same_saved_nodes(const std::vector<city::wire::SavedBackboneNode>& lhs,
                      const std::vector<city::wire::SavedBackboneNode>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const auto& a = lhs[i];
    const auto& b = rhs[i];
    const bool same_modes =
        a.bundle_modes.size() == b.bundle_modes.size() &&
        std::equal(a.bundle_modes.begin(), a.bundle_modes.end(), b.bundle_modes.begin(),
                   [](const city::wire::SupportNodeBundleMode& x,
                      const city::wire::SupportNodeBundleMode& y) {
                     return x.bundle_template_id == y.bundle_template_id && x.mode == y.mode;
                   });
    if (a.node_id != b.node_id || a.pole_id != b.pole_id || a.support_kind != b.support_kind ||
        !same_vec3(a.position, b.position) || a.has_source_edge != b.has_source_edge ||
        a.source_edge_node_a != b.source_edge_node_a || a.source_edge_node_b != b.source_edge_node_b ||
        !almost_equal(a.source_edge_t, b.source_edge_t, 1e-12) ||
        a.path_point_index != b.path_point_index || !same_modes) {
      return false;
    }
  }
  return true;
}

bool same_saved_edges(const std::vector<city::wire::SavedBackboneEdge>& lhs,
                      const std::vector<city::wire::SavedBackboneEdge>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const auto& a = lhs[i];
    const auto& b = rhs[i];
    if (a.edge_id != b.edge_id || a.node_a != b.node_a || a.node_b != b.node_b ||
        a.route != b.route || a.order != b.order || !same_vec3(a.dir, b.dir) ||
        !almost_equal(a.lateral_offset_m, b.lateral_offset_m, 1e-12)) {
      return false;
    }
  }
  return true;
}

struct CountSnapshot {
  std::size_t poles = 0;
  std::size_t ports = 0;
  std::size_t bundles = 0;
  std::size_t spans = 0;
  std::size_t nodes = 0;
  std::size_t edges = 0;
  std::size_t edge_bundles = 0;
  std::size_t port_bindings = 0;
  std::size_t span_bindings = 0;
  int fixed_count = 0;
  std::uint64_t template_version = 0;
};

CountSnapshot count_snapshot(const city::wire::CoreState& state) {
  CountSnapshot out{};
  out.poles = state.view().poles().size();
  out.ports = state.view().ports().size();
  out.bundles = state.view().bundles().size();
  out.spans = state.view().spans().size();
  out.nodes = state.view().backbone().nodes.size();
  out.edges = state.view().backbone().edges.size();
  out.edge_bundles = state.view().backbone().edge_bundles.size();
  out.port_bindings = state.view().backbone().port_bindings.size();
  out.span_bindings = state.view().backbone().span_bindings.size();
  const auto it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  out.fixed_count = it == state.view().bundle_templates().end() ? 0 : it->second.fixed_count;
  out.template_version = it == state.view().bundle_templates().end() ? 0 : it->second.version;
  return out;
}

bool same_counts(const CountSnapshot& a, const CountSnapshot& b) {
  return a.poles == b.poles && a.ports == b.ports && a.bundles == b.bundles && a.spans == b.spans &&
         a.nodes == b.nodes && a.edges == b.edges && a.edge_bundles == b.edge_bundles &&
         a.port_bindings == b.port_bindings && a.span_bindings == b.span_bindings &&
         a.fixed_count == b.fixed_count && a.template_version == b.template_version;
}

bool update_low_voltage_count_to_two(city::wire::CoreState& state, std::string* error = nullptr) {
  const auto template_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate edited = template_it->second;
  edited.fixed_count = 2;
  const auto updated = state.UpdateBundleTemplate(edited);
  if (error != nullptr) {
    *error = updated.error;
  }
  return updated.ok && updated.value;
}

bool update_low_voltage_count_to_one(city::wire::CoreState& state, std::string* error = nullptr) {
  const auto template_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate edited = template_it->second;
  edited.fixed_count = 1;
  const auto updated = state.UpdateBundleTemplate(edited);
  if (error != nullptr) {
    *error = updated.error;
  }
  return updated.ok && updated.value;
}

bool set_low_voltage_count_before_generation(city::wire::CoreState& state, int fixed_count) {
  const auto template_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate edited = template_it->second;
  edited.fixed_count = fixed_count;
  const auto updated = state.UpdateBundleTemplate(edited);
  return updated.ok && updated.value;
}

bool set_low_voltage_range_before_generation(city::wire::CoreState& state, int default_count) {
  const auto template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  const auto template_it = state.view().bundle_templates().find(template_id);
  if (template_it == state.view().bundle_templates().end()) return false;
  city::wire::BundleTemplate edited = template_it->second;
  edited.count_rule = city::wire::BundleCountRuleKind::kRange;
  edited.fixed_count = 0;
  edited.min_count = 1;
  edited.max_count = 6;
  edited.default_count = default_count;
  const auto updated = state.UpdateBundleTemplate(edited);
  return updated.ok && updated.value;
}

struct FixedCountBundleComponent {
  city::wire::ObjectId bundle_id = city::wire::kInvalidObjectId;
  std::vector<city::wire::ObjectId> edge_bundle_ids{};
};

FixedCountBundleComponent generate_fixed_count_component(city::wire::CoreState& state,
                                                         double y) {
  FixedCountBundleComponent out{};
  city::wire::BackboneSpec request = poly3_req(state);
  for (city::wire::Vec3d& point : request.path.polyline) {
    point.y += y;
  }
  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok) return out;
  const city::wire::BundleTemplateId template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  for (city::wire::ObjectId bundle_id : generated.value.bundle_ids) {
    const city::wire::Bundle* bundle = state.view().bundles().find(bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      out.bundle_id = bundle_id;
      break;
    }
  }
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle :
       state.view().backbone().edge_bundles) {
    if (edge_bundle.bundle_id == out.bundle_id) {
      out.edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
    }
  }
  return out;
}

bool fixed_count_component_relations_are_internal(
    const city::wire::CoreState& state,
    const FixedCountBundleComponent& component,
    std::size_t lane_count) {
  if (component.bundle_id == city::wire::kInvalidObjectId ||
      component.edge_bundle_ids.size() != 2) return false;
  const city::wire::Bundle* bundle = state.view().bundles().find(component.bundle_id);
  if (bundle == nullptr || bundle->conductor_count != static_cast<int>(lane_count)) return false;
  std::unordered_set<city::wire::ObjectId> component_edge_bundles(
      component.edge_bundle_ids.begin(), component.edge_bundle_ids.end());
  for (city::wire::ObjectId edge_bundle_id : component.edge_bundle_ids) {
    const auto saved_edge_bundle = std::find_if(
        state.view().backbone().edge_bundles.begin(),
        state.view().backbone().edge_bundles.end(),
        [&](const city::wire::SavedBackboneEdgeBundle& candidate) {
          return candidate.edge_bundle_id == edge_bundle_id;
        });
    if (saved_edge_bundle == state.view().backbone().edge_bundles.end() ||
        saved_edge_bundle->bundle_id != component.bundle_id) return false;
    std::unordered_set<city::wire::ObjectId> bound_ports{};
    std::vector<std::size_t> span_lanes{};
    std::vector<std::size_t> port_lanes{};
    for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
      if (binding.edge_bundle_id == edge_bundle_id) {
        bound_ports.insert(binding.port_id);
        port_lanes.push_back(binding.lane_index);
      }
    }
    for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      const city::wire::Span* span = state.view().spans().find(binding.span_id);
      if (span == nullptr || span->bundle_id != component.bundle_id ||
          !bound_ports.contains(span->port_a_id) || !bound_ports.contains(span->port_b_id)) {
        return false;
      }
      span_lanes.push_back(binding.lane_index);
    }
    std::sort(span_lanes.begin(), span_lanes.end());
    std::sort(port_lanes.begin(), port_lanes.end());
    std::vector<std::size_t> expected_span_lanes{};
    std::vector<std::size_t> expected_port_lanes{};
    for (std::size_t lane = 0; lane < lane_count; ++lane) {
      expected_span_lanes.push_back(lane);
      expected_port_lanes.push_back(lane);
      expected_port_lanes.push_back(lane);
    }
    if (span_lanes != expected_span_lanes || port_lanes != expected_port_lanes) return false;
  }
  std::vector<std::size_t> lanes_a{};
  std::vector<std::size_t> lanes_b{};
  for (const city::wire::SavedBackboneRowContinuity& continuity :
       state.view().backbone().row_continuities) {
    const bool a_inside = component_edge_bundles.contains(continuity.a.edge_bundle_id);
    const bool b_inside = component_edge_bundles.contains(continuity.b.edge_bundle_id);
    if (!a_inside && !b_inside) continue;
    if (!a_inside || !b_inside || continuity.a.edge_bundle_id == continuity.b.edge_bundle_id) return false;
    if (continuity.a.edge_bundle_id == component.edge_bundle_ids.front()) {
      lanes_a.push_back(continuity.a.lane_index);
      lanes_b.push_back(continuity.b.lane_index);
    } else {
      lanes_a.push_back(continuity.b.lane_index);
      lanes_b.push_back(continuity.a.lane_index);
    }
  }
  std::sort(lanes_a.begin(), lanes_a.end());
  std::sort(lanes_b.begin(), lanes_b.end());
  std::vector<std::size_t> expected_lanes{};
  for (std::size_t lane = 0; lane < lane_count; ++lane) expected_lanes.push_back(lane);
  return lanes_a == expected_lanes && lanes_b == expected_lanes;
}

std::vector<city::wire::ObjectId> component_lane_span_ids(
    const city::wire::CoreState& state, const FixedCountBundleComponent& component,
    std::size_t lane) {
  std::vector<city::wire::ObjectId> out{};
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.lane_index == lane &&
        std::find(component.edge_bundle_ids.begin(), component.edge_bundle_ids.end(),
                  binding.edge_bundle_id) != component.edge_bundle_ids.end()) {
      out.push_back(binding.span_id);
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<city::wire::ObjectId> component_lane_port_ids(
    const city::wire::CoreState& state, const FixedCountBundleComponent& component,
    std::size_t lane) {
  std::vector<city::wire::ObjectId> out{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.lane_index == lane &&
        std::find(component.edge_bundle_ids.begin(), component.edge_bundle_ids.end(),
                  binding.edge_bundle_id) != component.edge_bundle_ids.end()) {
      out.push_back(binding.port_id);
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::vector<city::wire::ObjectId> span_ids_for_lane(const city::wire::CoreState& state, std::size_t lane) {
  std::vector<city::wire::ObjectId> out{};
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.lane_index == lane) {
      out.push_back(binding.span_id);
    }
  }
  return out;
}

std::vector<city::wire::ObjectId> port_ids_for_lane(const city::wire::CoreState& state, std::size_t lane) {
  std::vector<city::wire::ObjectId> out{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.lane_index == lane) {
      out.push_back(binding.port_id);
    }
  }
  return out;
}

bool no_binding_references(const city::wire::CoreState& state,
                           const std::vector<city::wire::ObjectId>& retired_spans,
                           const std::vector<city::wire::ObjectId>& retired_ports) {
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (std::find(retired_spans.begin(), retired_spans.end(), binding.span_id) != retired_spans.end()) {
      return false;
    }
  }
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (std::find(retired_ports.begin(), retired_ports.end(), binding.port_id) != retired_ports.end()) {
      return false;
    }
  }
  for (const auto& [edge_bundle_id, span_ids] : state.view().backbone_index().edge_bundle_spans) {
    static_cast<void>(edge_bundle_id);
    for (city::wire::ObjectId span_id : span_ids) {
      if (std::find(retired_spans.begin(), retired_spans.end(), span_id) != retired_spans.end()) return false;
    }
  }
  return true;
}

} // namespace

bool C611_backbone_direct_derive_restores_saved_span_outputs() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  if (!state.span_layout_rules(span_id).has_rule()) {
    return false;
  }
  city::wire::CacheState& cache = city::wire::CoreStateTestHook::cache_state(state);
  cache.span_layout_cache.clear_layout(span_id);
  cache.curve_cache.by_span.erase(span_id);
  cache.bounds_cache.by_span.erase(span_id);
  cache.visual_cache.by_span.erase(span_id);
  cache.render_cache.by_span.erase(span_id);
  if (state.span_layout(span_id).has_layout() || state.find_curve_cache(span_id) != nullptr ||
      state.find_bounds_cache(span_id) != nullptr || state.find_span_visual_cache(span_id) != nullptr ||
      state.find_span_render_cache(span_id) != nullptr) {
    return false;
  }
  const auto derived = state.DeriveGeneratedSpanOutputs(span_id);
  return derived.ok && state.span_layout(span_id).has_layout() && state.find_curve_cache(span_id) != nullptr &&
         state.find_bounds_cache(span_id) != nullptr && state.find_span_visual_cache(span_id) != nullptr &&
         state.find_span_render_cache(span_id) != nullptr;
}

bool C613_backbone_port_edit_rederives_generated_span_without_recalc() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  const city::wire::Span* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const city::wire::Port* port = state.view().ports().find(span->port_a_id);
  if (port == nullptr) {
    return false;
  }
  const city::wire::Vec3d moved{port->world_position.x, port->world_position.y + 1.25, port->world_position.z};
  const auto edit = state.SetPortWorldPositionManual(port->id, moved);
  if (!edit.ok) {
    return false;
  }
  const city::wire::SpanLayoutView layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
  const city::wire::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
  const city::wire::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
  if (!layout.has_layout() || curve == nullptr || bounds == nullptr || visual == nullptr || render == nullptr ||
      curve->detail.sample_points.empty()) {
    return false;
  }
  const bool layout_moved = almost_equal(layout.entry->start.support_world.x, moved.x, 1e-9) &&
                            almost_equal(layout.entry->start.support_world.y, moved.y, 1e-9) &&
                            almost_equal(layout.entry->start.support_world.z, moved.z, 1e-9);
  const bool curve_moved = almost_equal(curve->detail.sample_points.front().x, layout.entry->start.endpoint_world.x, 1e-9) &&
                           almost_equal(curve->detail.sample_points.front().y, layout.entry->start.endpoint_world.y, 1e-9) &&
                           almost_equal(curve->detail.sample_points.front().z, layout.entry->start.endpoint_world.z, 1e-9);
  return layout_moved && curve_moved;
}

bool C614_backbone_update_plan_uses_coarse_kinds() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_pole_ids.empty() || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  const city::wire::Span* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const auto pole_plan = city::wire::CoreStateTestHook::make_update_plan(
      state, {city::wire::UpdateKind::kReposition, city::wire::UpdateTargetKind::kPole,
              out.value.generated_pole_ids.front()});
  const auto port_plan = city::wire::CoreStateTestHook::make_update_plan(
      state, {city::wire::UpdateKind::kReposition, city::wire::UpdateTargetKind::kPort, span->port_a_id});
  const auto reshape_plan = city::wire::CoreStateTestHook::make_update_plan(
      state, {city::wire::UpdateKind::kReshape, city::wire::UpdateTargetKind::kAllSpans,
              city::wire::kInvalidObjectId});
  const auto redraw_plan = city::wire::CoreStateTestHook::make_update_plan(
      state, {city::wire::UpdateKind::kRedraw, city::wire::UpdateTargetKind::kAllSpans,
              city::wire::kInvalidObjectId});
  const auto regen_plan = city::wire::CoreStateTestHook::make_update_plan(
      state, {city::wire::UpdateKind::kRegenerate, city::wire::UpdateTargetKind::kSpan, span_id});
  return pole_plan.ok && pole_plan.value.kind == city::wire::UpdateKind::kReposition &&
         contains_id(pole_plan.value.affected.spans, span_id) && port_plan.ok &&
         port_plan.value.kind == city::wire::UpdateKind::kReposition &&
         contains_id(port_plan.value.affected.spans, span_id) && reshape_plan.ok &&
         reshape_plan.value.kind == city::wire::UpdateKind::kReshape &&
         contains_id(reshape_plan.value.affected.spans, span_id) && redraw_plan.ok &&
         redraw_plan.value.kind == city::wire::UpdateKind::kRedraw &&
         contains_id(redraw_plan.value.affected.spans, span_id) && regen_plan.ok &&
         regen_plan.value.kind == city::wire::UpdateKind::kRegenerate;
}

bool C615_backbone_regenerate_plan_is_not_local_fallback() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto plan = city::wire::CoreStateTestHook::make_update_plan(
      state, {city::wire::UpdateKind::kRegenerate, city::wire::UpdateTargetKind::kSpan,
              out.value.generated_span_ids.front()});
  if (!plan.ok) {
    return false;
  }
  const auto executed = city::wire::CoreStateTestHook::execute_update_plan(state, plan.value);
  return !executed.ok;
}

bool C616_backbone_reposition_keeps_saved_graph_identity() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  std::vector<city::wire::ObjectId> before_nodes{};
  std::vector<city::wire::ObjectId> before_edges{};
  std::vector<city::wire::ObjectId> before_edge_bundles{};
  std::vector<city::wire::ObjectId> before_span_bindings{};
  for (const city::wire::SavedBackboneNode& node : state.view().backbone().nodes) {
    before_nodes.push_back(node.node_id);
  }
  for (const city::wire::SavedBackboneEdge& edge : state.view().backbone().edges) {
    before_edges.push_back(edge.edge_id);
  }
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    before_edge_bundles.push_back(edge_bundle.edge_bundle_id);
  }
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    before_span_bindings.push_back(binding.span_id);
  }
  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  const city::wire::Span* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const city::wire::Port* port = state.view().ports().find(span->port_a_id);
  if (port == nullptr) {
    return false;
  }
  const auto edited =
      state.SetPortWorldPositionManual(port->id, {port->world_position.x, port->world_position.y + 0.75,
                                                  port->world_position.z});
  std::vector<city::wire::ObjectId> after_nodes{};
  std::vector<city::wire::ObjectId> after_edges{};
  std::vector<city::wire::ObjectId> after_edge_bundles{};
  std::vector<city::wire::ObjectId> after_span_bindings{};
  for (const city::wire::SavedBackboneNode& node : state.view().backbone().nodes) {
    after_nodes.push_back(node.node_id);
  }
  for (const city::wire::SavedBackboneEdge& edge : state.view().backbone().edges) {
    after_edges.push_back(edge.edge_id);
  }
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    after_edge_bundles.push_back(edge_bundle.edge_bundle_id);
  }
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    after_span_bindings.push_back(binding.span_id);
  }
  return edited.ok && before_nodes == after_nodes && before_edges == after_edges &&
         before_edge_bundles == after_edge_bundles && before_span_bindings == after_span_bindings;
}

bool C619_backbone_reposition_updates_only_affected_spans() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.size() < 2) {
    return false;
  }
  const city::wire::ObjectId first_span_id = out.value.generated_span_ids.front();
  const city::wire::ObjectId second_span_id = out.value.generated_span_ids.back();
  const city::wire::Span* first_span = state.view().spans().find(first_span_id);
  if (first_span == nullptr) {
    return false;
  }
  const city::wire::Port* first_port = state.view().ports().find(first_span->port_a_id);
  const city::wire::SpanLayoutView second_before_view = state.span_layout(second_span_id);
  if (first_port == nullptr || !second_before_view.has_layout()) {
    return false;
  }
  const double old_first_y = first_port->world_position.y;
  const city::wire::SpanLayoutEntry second_before = *second_before_view.entry;
  const auto edited =
      state.SetPortWorldPositionManual(first_port->id, {first_port->world_position.x, first_port->world_position.y + 1.0,
                                                        first_port->world_position.z});
  const city::wire::SpanLayoutView first_after = state.span_layout(first_span_id);
  const city::wire::SpanLayoutView second_after = state.span_layout(second_span_id);
  return edited.ok && first_after.has_layout() && second_after.has_layout() &&
         almost_equal(first_after.entry->start.support_world.y, old_first_y + 1.0, 1e-9) &&
         second_before.source_version == second_after.entry->source_version;
}

bool C621_backbone_sag_reshape_updates_geom_only() {
  city::wire::CoreState state;
  city::wire::GeometrySettings initial_settings = state.view().geometry_settings();
  initial_settings.sag_enabled = false;
  if (!state.UpdateGeometrySettings(initial_settings).ok) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  const city::wire::SpanLayoutView before_layout_view = state.span_layout(span_id);
  if (!before_layout_view.has_layout()) {
    return false;
  }
  const city::wire::SpanLayoutEntry before_layout = *before_layout_view.entry;
  std::vector<city::wire::ObjectId> node_ids{};
  std::vector<city::wire::ObjectId> edge_ids{};
  std::vector<city::wire::ObjectId> binding_spans{};
  for (const city::wire::SavedBackboneNode& node : state.view().backbone().nodes) {
    node_ids.push_back(node.node_id);
  }
  for (const city::wire::SavedBackboneEdge& edge : state.view().backbone().edges) {
    edge_ids.push_back(edge.edge_id);
  }
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    binding_spans.push_back(binding.span_id);
  }

  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = std::max(0.03, settings.sag_factor);
  settings.curve_samples = std::max(9, settings.curve_samples);
  const auto updated = state.UpdateGeometrySettings(settings);
  const city::wire::SpanLayoutView after_layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
  if (!updated.ok || !after_layout.has_layout() || curve == nullptr || bounds == nullptr ||
      curve->detail.sag_amplitude_m <= 0.0 || curve->detail.sample_points.size() < 3) {
    return false;
  }
  const city::wire::Vec3d start = curve->detail.EvaluatePosition(0.0);
  const city::wire::Vec3d end = curve->detail.EvaluatePosition(1.0);
  const double endpoint_min_z =
      std::min(before_layout.start.endpoint_world.z, before_layout.end.endpoint_world.z);
  if (!almost_equal(start, before_layout.start.endpoint_world, 1e-9) ||
      !almost_equal(end, before_layout.end.endpoint_world, 1e-9) ||
      !(bounds->whole.min.z < endpoint_min_z)) {
    return false;
  }

  std::vector<city::wire::ObjectId> after_node_ids{};
  std::vector<city::wire::ObjectId> after_edge_ids{};
  std::vector<city::wire::ObjectId> after_binding_spans{};
  for (const city::wire::SavedBackboneNode& node : state.view().backbone().nodes) {
    after_node_ids.push_back(node.node_id);
  }
  for (const city::wire::SavedBackboneEdge& edge : state.view().backbone().edges) {
    after_edge_ids.push_back(edge.edge_id);
  }
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    after_binding_spans.push_back(binding.span_id);
  }
  return almost_equal(after_layout.entry->start.endpoint_world, before_layout.start.endpoint_world, 1e-9) &&
         almost_equal(after_layout.entry->end.endpoint_world, before_layout.end.endpoint_world, 1e-9) &&
         after_layout.entry->source_version == before_layout.source_version && node_ids == after_node_ids &&
         edge_ids == after_edge_ids && binding_spans == after_binding_spans;
}

bool C623_backbone_layout_settings_reject_before_mutation() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) return false;
  city::wire::LayoutSettings edited = state.view().layout_settings();
  edited.min_side_scale = 1.0;
  edited.max_side_scale = 1.0;
  const auto updated = state.UpdateLayoutSettings(edited);
  return updated.ok && updated.value &&
         almost_equal(state.view().layout_settings().max_side_scale, edited.max_side_scale, 1e-12) &&
         !updated.change_set.updated_ids.empty();
}

bool C624_backbone_variation_settings_reshape_existing_output() {
  using namespace city::wire;
  auto request = [](CoreState& state) {
    BackboneSpec spec = line_req(state);
    spec.bundles = {{kDefaultCommunicationBundleTemplateId, 62401,
                     SpanLayer::kCommunication, 1, true, 5.3, -0.3, 0.2}};
    return spec;
  };

  CoreState updated_state;
  VariationSettings initial = updated_state.view().variation_settings();
  initial.sag_variation_scale = 0.0;
  if (!updated_state.UpdateVariationSettings(initial).ok ||
      !updated_state.GenerateFromBackboneSpec(request(updated_state)).ok) return false;
  const VisualCurvePartCache before = updated_state.view().visual_curve_parts();
  const std::size_t before_span_count = updated_state.view().spans().size();
  const std::size_t before_port_count = updated_state.view().ports().size();

  VariationSettings edited = initial;
  edited.sag_variation_scale = 0.4;
  const auto updated = updated_state.UpdateVariationSettings(edited);
  if (!updated.ok || !updated.value || updated.change_set.updated_ids.empty()) return false;

  CoreState fresh;
  if (!fresh.UpdateVariationSettings(edited).ok ||
      !fresh.GenerateFromBackboneSpec(request(fresh)).ok) return false;
  return visual_member_samples_differ(before, updated_state.view().visual_curve_parts()) &&
         same_visual_curve_samples(updated_state.view().visual_curve_parts(),
                                   fresh.view().visual_curve_parts()) &&
         updated_state.view().spans().size() == before_span_count &&
         updated_state.view().ports().size() == before_port_count;
}

bool C625_backbone_context_profile_reject_before_mutation() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok) return false;
  const city::wire::ContextProfile before = state.view().context_profile();
  city::wire::ContextProfile edited = before;
  edited.age = std::clamp(before.age + 0.1, 0.0, 1.0);
  if (almost_equal(edited.age, before.age, 1e-12)) edited.age = std::max(0.0, before.age - 0.1);
  const auto updated = state.UpdateContextProfile(edited);
  return !updated.ok && contains_text(updated.error, "unsupported") &&
         almost_equal(state.view().context_profile().age, before.age, 1e-12);
}

bool C626_backbone_cable_template_updates_derive_outputs() {
  city::wire::CoreState state;
  city::wire::GeometrySettings geometry = state.view().geometry_settings();
  geometry.sag_enabled = true;
  geometry.curve_samples = std::max(9, geometry.curve_samples);
  if (!state.UpdateGeometrySettings(geometry).ok) return false;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) return false;
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::Span* span = state.view().spans().find(span_id);
  const city::wire::Bundle* bundle =
      span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
  if (bundle == nullptr) return false;
  const auto bundle_template = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_template == state.view().bundle_templates().end()) return false;
  const auto cable_it = state.view().cable_templates().find(bundle_template->second.cable_template_id);
  const city::wire::SpanLayoutView before_layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* before_curve = state.find_curve_cache(span_id);
  const city::wire::SpanRenderCacheEntry* before_render = state.find_span_render_cache(span_id);
  if (cable_it == state.view().cable_templates().end() || !before_layout.has_layout() ||
      before_curve == nullptr || before_render == nullptr) return false;
  const city::wire::SpanLayoutEntry layout_copy = *before_layout.entry;
  const double sag_before = before_curve->detail.sag_amplitude_m;

  city::wire::CableTemplate reshape = cable_it->second;
  reshape.sag_factor += 0.02;
  const auto reshaped = state.UpdateCableTemplate(reshape);
  const city::wire::CurveCacheEntry* reshaped_curve = state.find_curve_cache(span_id);
  if (!reshaped.ok || reshaped_curve == nullptr ||
      !(reshaped_curve->detail.sag_amplitude_m > sag_before) ||
      !almost_equal(state.span_layout(span_id).entry->start.endpoint_world, layout_copy.start.endpoint_world, 1e-9)) {
    return false;
  }

  city::wire::CableTemplate redraw = state.view().cable_templates().at(reshape.id);
  redraw.color_rgba ^= 0x000000FFu;
  const std::vector<city::wire::Vec3d> samples_after_reshape = reshaped_curve->detail.sample_points;
  const auto redrawn = state.UpdateCableTemplate(redraw);
  const city::wire::CurveCacheEntry* final_curve = state.find_curve_cache(span_id);
  const city::wire::SpanRenderCacheEntry* final_render = state.find_span_render_cache(span_id);
  const auto same_points = [](const std::vector<city::wire::Vec3d>& a,
                              const std::vector<city::wire::Vec3d>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
      if (!almost_equal(a[i], b[i], 1e-9)) return false;
    }
    return true;
  };
  return redrawn.ok && final_curve != nullptr && final_render != nullptr &&
         same_points(final_curve->detail.sample_points, samples_after_reshape) &&
         final_render->color_rgba == redraw.color_rgba;
}

bool C627_backbone_legacy_topology_apis_are_removed() {
  std::string state_header{};
  std::string api_types{};
  std::string cmake{};
  if (!file_text(repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_state.hpp", &state_header) ||
      !file_text(repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_state_api_types.hpp", &api_types) ||
      !file_text(repo_root() / "domains" / "wire" / "CMakeLists.txt", &cmake)) {
    return false;
  }
  const std::vector<std::string> retired = {
      "AddConnectionByPole", "AddDropFromPole", "AddDropFromSpan", "SplitSpan",
      "AddConnectionByPoleOptions", "AddConnectionByPoleResult", "AddDropResult", "SplitSpanResult",
  };
  for (const std::string& symbol : retired) {
    if (contains_text(state_header, symbol) || contains_text(api_types, symbol)) {
      return false;
    }
  }
  return !contains_text(cmake, "state/legacy/topology.cpp");
}

bool C628_backbone_active_pole_type_update_repositions() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty() ||
      generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId pole_id = generated.value.generated_pole_ids.front();
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::Pole* pole = state.view().poles().find(pole_id);
  const city::wire::Span* span = state.view().spans().find(span_id);
  if (pole == nullptr || span == nullptr) {
    return false;
  }
  const auto type_it = state.view().pole_types().find(pole->pole_type_id);
  const city::wire::Port* port_a = state.view().ports().find(span->port_a_id);
  const city::wire::Port* port_b = state.view().ports().find(span->port_b_id);
  const city::wire::SpanLayoutView layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const city::wire::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
  const city::wire::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
  if (type_it == state.view().pole_types().end() || port_a == nullptr || port_b == nullptr ||
      !layout.has_layout() || curve == nullptr || visual == nullptr || render == nullptr) {
    return false;
  }

  const city::wire::PoleTypeDefinition type_before = type_it->second;
  const double pole_height_before = pole->height_m;
  const city::wire::Vec3d port_a_before = port_a->world_position;
  const city::wire::Vec3d port_b_before = port_b->world_position;
  const city::wire::SpanLayoutEntry layout_before = *layout.entry;
  const std::vector<city::wire::Vec3d> curve_before = curve->detail.sample_points;
  const std::uint64_t visual_version_before = visual->source_version;
  const std::uint64_t render_version_before = render->source_version;
  const std::size_t node_count_before = state.view().backbone().nodes.size();
  const std::size_t edge_count_before = state.view().backbone().edges.size();
  const std::size_t binding_count_before = state.view().backbone().span_bindings.size();

  city::wire::PoleTypeDefinition edited = type_before;
  edited.default_height_m += 0.35;
  for (city::wire::PortPlacementBand& band : edited.port_bands) {
    if (!band.enabled) {
      continue;
    }
    band.height_center_m += 0.75;
    band.height_min_m += 0.75;
    band.height_max_m += 0.75;
  }
  const auto updated = state.UpdatePoleTypeDefinition(edited);
  const city::wire::Pole* pole_after = state.view().poles().find(pole_id);
  const city::wire::Port* port_a_after = state.view().ports().find(span->port_a_id);
  const city::wire::Port* port_b_after = state.view().ports().find(span->port_b_id);
  const city::wire::SpanLayoutView layout_after = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* curve_after = state.find_curve_cache(span_id);
  const city::wire::SpanVisualCacheEntry* visual_after = state.find_span_visual_cache(span_id);
  const city::wire::SpanRenderCacheEntry* render_after = state.find_span_render_cache(span_id);
  const auto type_after = state.view().pole_types().find(type_before.id);
  const bool curve_changed =
      curve_after != nullptr && curve_after->detail.sample_points.size() == curve_before.size() &&
      !std::equal(curve_before.begin(), curve_before.end(), curve_after->detail.sample_points.begin(),
                  [](const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
                    return almost_equal(a, b, 1e-12);
                  });
  if (!updated.ok || type_after == state.view().pole_types().end() ||
      !almost_equal(type_after->second.default_height_m, edited.default_height_m, 1e-12) ||
      pole_after == nullptr || !almost_equal((pole_after->height_m - pole_height_before), 0.35, 1e-12) ||
      port_a_after == nullptr || port_b_after == nullptr ||
      almost_equal(port_a_after->world_position, port_a_before, 1e-12) ||
      almost_equal(port_b_after->world_position, port_b_before, 1e-12) ||
      !layout_after.has_layout() ||
      almost_equal(layout_after.entry->start.endpoint_world, layout_before.start.endpoint_world, 1e-12) ||
      almost_equal(layout_after.entry->end.endpoint_world, layout_before.end.endpoint_world, 1e-12) ||
      !curve_changed ||
      visual_after == nullptr || visual_after->source_version == visual_version_before ||
      render_after == nullptr || render_after->source_version == render_version_before ||
      state.view().backbone().nodes.size() != node_count_before ||
      state.view().backbone().edges.size() != edge_count_before ||
      state.view().backbone().span_bindings.size() != binding_count_before) {
    return false;
  }

  return true;
}

bool C697_backbone_edge_saves_lateral_offset_echo() {
  city::wire::CoreState offset_state;
  city::wire::BackboneSpec offset_req = line_req(offset_state);
  offset_req.constraints.lateral_offset_m = 1.0;
  const auto offset_generated = offset_state.GenerateFromBackboneSpec(offset_req);
  if (!offset_generated.ok || offset_state.view().backbone().edges.size() != 1) {
    return false;
  }
  if (!almost_equal(offset_state.view().backbone().edges.front().lateral_offset_m, 1.0, 1e-12)) {
    return false;
  }

  city::wire::CoreState default_state;
  const auto default_generated = default_state.GenerateFromBackboneSpec(line_req(default_state));
  if (!default_generated.ok || default_state.view().backbone().edges.size() != 1) {
    return false;
  }
  if (!almost_equal(default_state.view().backbone().edges.front().lateral_offset_m, 0.0, 1e-12)) {
    return false;
  }

  city::wire::CoreState repeat_state;
  city::wire::BackboneSpec repeat_req = line_req(repeat_state);
  repeat_req.constraints.lateral_offset_m = 1.0;
  const auto repeat_generated = repeat_state.GenerateFromBackboneSpec(repeat_req);
  return repeat_generated.ok && repeat_state.view().backbone().edges.size() == 1 &&
         almost_equal(repeat_state.view().backbone().edges.front().lateral_offset_m,
                      offset_state.view().backbone().edges.front().lateral_offset_m, 1e-12);
}
bool C660_backbone_bundle_fixed_count_migration_updates_downstream_only() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 1) {
    return false;
  }
  const auto template_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == state.view().bundle_templates().end() ||
      template_it->second.count_rule != city::wire::BundleCountRuleKind::kFixed ||
      template_it->second.fixed_count != 1) {
    return false;
  }

  const std::vector<city::wire::SavedBackboneNode> nodes_before = state.view().backbone().nodes;
  const std::vector<city::wire::SavedBackboneEdge> edges_before = state.view().backbone().edges;
  const std::vector<city::wire::SavedBackboneEdgeBundle> edge_bundles_before =
      state.view().backbone().edge_bundles;
  const std::size_t span_count_before = state.view().spans().size();

  city::wire::BundleTemplate edited = template_it->second;
  edited.fixed_count = 2;
  const auto updated = state.UpdateBundleTemplate(edited);
  if (!updated.ok || !updated.value) {
    return false;
  }
  if (!same_saved_nodes(state.view().backbone().nodes, nodes_before) ||
      !same_saved_edges(state.view().backbone().edges, edges_before) ||
      state.view().backbone().edge_bundles.size() != edge_bundles_before.size() ||
      state.view().spans().size() != span_count_before + 1) {
    return false;
  }
  for (std::size_t i = 0; i < edge_bundles_before.size(); ++i) {
    const auto& after = state.view().backbone().edge_bundles[i];
    const auto& before = edge_bundles_before[i];
    if (after.edge_bundle_id != before.edge_bundle_id || after.edge_id != before.edge_id ||
        after.bundle_id != before.bundle_id || after.edge_forward != before.edge_forward ||
        after.route != before.route || after.order != before.order || !same_vec3(after.dir, before.dir)) {
      return false;
    }
  }
  const city::wire::ObjectId edge_bundle_id = state.view().backbone().edge_bundles.front().edge_bundle_id;
  const std::vector<city::wire::ObjectId> bundle_span_ids = edge_bundle_span_ids(state, edge_bundle_id);
  if (bundle_span_ids.size() != 2) {
    return false;
  }
  for (city::wire::ObjectId span_id : bundle_span_ids) {
    if (!state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr ||
        state.find_span_visual_cache(span_id) == nullptr ||
        state.find_span_render_cache(span_id) == nullptr) {
      return false;
    }
  }

  city::wire::CoreState fresh;
  const auto fresh_template_it = fresh.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (fresh_template_it == fresh.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate fresh_template = fresh_template_it->second;
  fresh_template.fixed_count = 2;
  const auto fresh_updated = fresh.UpdateBundleTemplate(fresh_template);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  if (!fresh_updated.ok || !fresh_generated.ok || fresh_generated.value.generated_span_ids.size() != 2 ||
      fresh.view().backbone().edge_bundles.empty() ||
      edge_bundle_span_ids(fresh, fresh.view().backbone().edge_bundles.front().edge_bundle_id).size() !=
          bundle_span_ids.size()) {
    return false;
  }
  return same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C668_backbone_bundle_count_migration_rejects_unreconstructable_lateral_offset() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = 0.35;
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const bool updated = update_low_voltage_count_to_two(state);
  city::wire::CoreState fresh;
  const auto fresh_template_it = fresh.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (fresh_template_it == fresh.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate fresh_template = fresh_template_it->second;
  fresh_template.fixed_count = 2;
  const auto fresh_updated = fresh.UpdateBundleTemplate(fresh_template);
  city::wire::BackboneSpec fresh_req = line_req(fresh);
  fresh_req.constraints.lateral_offset_m = 0.35;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(fresh_req);
  return updated && fresh_updated.ok && fresh_generated.ok &&
         same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C669_backbone_bundle_count_migration_rejects_multi_bundle_group_offset() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto generated = state.GenerateFromBackboneSpec(req);
  const city::wire::ObjectId lv_edge_bundle =
      edge_bundle_id_for_template(state, city::wire::BundleKind::kLowVoltage);
  const city::wire::ObjectId comm_edge_bundle =
      edge_bundle_id_for_template(state, city::wire::BundleKind::kCommunication);
  if (!generated.ok || lv_edge_bundle == city::wire::kInvalidObjectId ||
      comm_edge_bundle == city::wire::kInvalidObjectId) {
    return false;
  }
  const EdgeBundleIdentitySnapshot comm_before = edge_bundle_identity_snapshot(state, comm_edge_bundle);
  if (!update_low_voltage_count_to_one(state)) {
    return false;
  }
  if (!same_edge_bundle_identity_snapshot(comm_before, edge_bundle_identity_snapshot(state, comm_edge_bundle))) {
    return false;
  }

  city::wire::CoreState fresh;
  city::wire::BackboneSpec fresh_req = line_req(fresh);
  add_backbone_bundle(fresh_req, city::wire::BundleKind::kCommunication);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(fresh_req);
  const city::wire::ObjectId fresh_lv =
      edge_bundle_id_for_template(fresh, city::wire::BundleKind::kLowVoltage);
  const city::wire::ObjectId fresh_comm =
      edge_bundle_id_for_template(fresh, city::wire::BundleKind::kCommunication);
  return fresh_generated.ok &&
         same_span_curve_signatures(span_curve_signatures_for_edge_bundle(state, lv_edge_bundle),
                                    span_curve_signatures_for_edge_bundle(fresh, fresh_lv)) &&
         same_span_curve_signatures(span_curve_signatures_for_edge_bundle(state, comm_edge_bundle),
                                    span_curve_signatures_for_edge_bundle(fresh, fresh_comm));
}

bool C670_backbone_bundle_count_migration_rejects_pair_rows() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage).size() != 2) {
    return false;
  }
  if (!update_low_voltage_count_to_two(state)) {
    return false;
  }
  city::wire::CoreState fresh;
  if (!set_low_voltage_count_before_generation(fresh, 2)) {
    return false;
  }
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  return fresh_generated.ok &&
         same_route_bundle_signatures(route_bundle_signatures(state, city::wire::BundleKind::kLowVoltage),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage));
}


bool C671_backbone_bundle_count_migration_reuses_pipeline_stages() {
  std::string source{};
  if (!file_text(repo_root() / "domains/wire/src/generation/backbone/regenerate.cpp", &source)) {
    return false;
  }
  const std::size_t replay = source.find("build_input_from_saved_scope");
  const std::size_t mutation = source.find("trial.authoritative_", replay);
  if (replay == std::string::npos || mutation == std::string::npos) {
    return false;
  }
  return source.find("trial_pipeline.build(") != std::string::npos &&
         source.find("build_input_from_saved_scope") != std::string::npos &&
         source.find("return fail", mutation) == std::string::npos &&
         source.find("AddPort(") == std::string::npos && source.find("AddSpan(") == std::string::npos &&
         source.find("SpanLayoutRule") == std::string::npos && source.find("save_backbone_node") == std::string::npos &&
         source.find("save_backbone_edge") == std::string::npos;
}

bool C672_backbone_regenerate_preserves_manual_ports_on_surviving_lanes() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::Span* span = state.view().spans().find(generated.value.generated_span_ids.front());
  if (span == nullptr) {
    return false;
  }
  const city::wire::Port* port = state.view().ports().find(span->port_a_id);
  if (port == nullptr) {
    return false;
  }
  const auto moved = state.MovePort(port->id, {port->world_position.x + 0.1, port->world_position.y,
                                               port->world_position.z});
  if (!moved.ok) {
    return false;
  }
  const city::wire::Port* manual = state.view().ports().find(port->id);
  if (manual == nullptr) {
    return false;
  }
  const city::wire::Vec3d manual_position = manual->world_position;
  const CountSnapshot before = count_snapshot(state);
  const bool updated = update_low_voltage_count_to_two(state);
  const city::wire::Port* after = state.view().ports().find(port->id);
  return updated && after != nullptr && after->position_mode == city::wire::PortPositionMode::kManual &&
         after->user_edited_position && same_vec3(after->world_position, manual_position) &&
         state.view().spans().size() == before.spans + 1;
}

bool C714_backbone_regenerate_rejects_retired_manual_port() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  city::wire::ObjectId lane_one_port_id = city::wire::kInvalidObjectId;
  for (const city::wire::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.lane_index != 1) {
      continue;
    }
    const city::wire::Span* span = state.view().spans().find(binding.span_id);
    if (span == nullptr) {
      return false;
    }
    lane_one_port_id = span->port_a_id;
    break;
  }
  const city::wire::Port* port = state.view().ports().find(lane_one_port_id);
  if (port == nullptr) {
    return false;
  }
  const auto moved = state.MovePort(port->id, {port->world_position.x + 0.1, port->world_position.y,
                                               port->world_position.z});
  if (!moved.ok) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_one(state, &error);
  return !updated && contains_text(error, "manual ports") && same_counts(before, count_snapshot(state));
}

bool C715_backbone_span_branch_down_override_regenerates() {
  city::wire::CoreState state;
  const std::vector<city::wire::ObjectId> spans = lowering_branch_spans(state);
  if (spans.empty()) {
    return false;
  }
  city::wire::ObjectId span_id = city::wire::kInvalidObjectId;
  for (city::wire::ObjectId candidate : spans) {
    if (span_has_lowered_endpoint(state, candidate)) {
      span_id = candidate;
      break;
    }
  }
  const city::wire::SpanLayoutView before_layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* before_curve = state.find_curve_cache(span_id);
  if (!before_layout.has_layout() || before_curve == nullptr) {
    return false;
  }
  const std::vector<city::wire::Vec3d> before_points = before_curve->detail.sample_points;
  const double override_down = 1.25;
  const auto updated = state.SetSpanBranchDownOffsetOverride(span_id, override_down);
  const city::wire::SpanLayoutView after_layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* after_curve = state.find_curve_cache(span_id);
  if (!updated.ok || !after_layout.has_layout() || after_curve == nullptr) {
    return false;
  }
  const double start_down = after_layout.entry->start.branch_down_offset_m;
  const double end_down = after_layout.entry->end.branch_down_offset_m;
  const bool layout_consumed_override =
      almost_equal(start_down, override_down, 1e-9) || almost_equal(end_down, override_down, 1e-9);
  const bool curve_changed =
      after_curve->detail.sample_points.size() == before_points.size() &&
      !std::equal(before_points.begin(), before_points.end(), after_curve->detail.sample_points.begin(),
                  [](const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
                    return almost_equal(a, b, 1e-12);
                  });
  const auto cleared = state.ClearSpanBranchDownOffsetOverride(span_id);
  const city::wire::SpanLayoutView cleared_layout = state.span_layout(span_id);
  return layout_consumed_override && curve_changed && cleared.ok && cleared_layout.has_layout() &&
         !almost_equal(std::max(cleared_layout.entry->start.branch_down_offset_m,
                                cleared_layout.entry->end.branch_down_offset_m),
                       override_down, 1e-9);
}

bool C716_backbone_span_socket_override_regenerates() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::SpanLayoutView before_layout = state.span_layout(span_id);
  if (!before_layout.has_layout() || before_layout.entry->start.resolved_socket_id.has_value()) {
    return false;
  }
  const auto updated = state.SetSpanEndpointSocketOverride(span_id, true, 0);
  const city::wire::SpanLayoutView after_layout = state.span_layout(span_id);
  if (!updated.ok || !after_layout.has_layout() || !after_layout.entry->start.resolved_socket_id.has_value() ||
      *after_layout.entry->start.resolved_socket_id != 0 ||
      after_layout.entry->start.endpoint_source != city::wire::LayoutEndpointSourceKind::kAttachmentSocketOverride) {
    return false;
  }
  const auto cleared = state.ClearSpanEndpointSocketOverride(span_id, true);
  const city::wire::SpanLayoutView cleared_layout = state.span_layout(span_id);
  return cleared.ok && cleared_layout.has_layout() && !cleared_layout.entry->start.resolved_socket_id.has_value();
}

bool C739_span_override_keeps_unrelated_route_outputs_unchanged() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  city::wire::BackboneSpec second_req = line_req(state);
  for (city::wire::Vec3d& point : second_req.path.polyline) {
    point.y += 12.0;
  }
  const auto second = state.GenerateFromBackboneSpec(second_req);
  if (!first.ok || !second.ok || first.value.generated_span_ids.empty() || second.value.generated_span_ids.empty() ||
      state.view().backbone().edge_bundles.size() < 2) {
    return false;
  }
  const auto second_binding_it = std::find_if(
      state.view().backbone().span_bindings.begin(), state.view().backbone().span_bindings.end(),
      [&](const city::wire::SavedBackboneSpanBinding& binding) {
        return binding.span_id == second.value.generated_span_ids.front();
      });
  if (second_binding_it == state.view().backbone().span_bindings.end()) {
    return false;
  }
  const city::wire::ObjectId second_edge_bundle_id = second_binding_it->edge_bundle_id;
  const EdgeBundleIdentitySnapshot second_identity_before =
      edge_bundle_identity_snapshot(state, second_edge_bundle_id);
  const std::vector<SpanCurveSignature> second_curves_before =
      span_curve_signatures_for_edge_bundle(state, second_edge_bundle_id);
  const std::vector<SpanOutputSnapshot> second_outputs_before =
      snapshot_span_outputs(state, second.value.generated_span_ids);

  const auto updated = state.SetSpanBranchDownOffsetOverride(first.value.generated_span_ids.front(), 0.75);
  return updated.ok &&
         same_edge_bundle_identity_snapshot(second_identity_before,
                                            edge_bundle_identity_snapshot(state, second_edge_bundle_id)) &&
         same_span_curve_signatures(second_curves_before,
                                    span_curve_signatures_for_edge_bundle(state, second_edge_bundle_id)) &&
         same_span_output_snapshots(second_outputs_before, state);
}

bool C717_backbone_layout_settings_regenerate_matches_fresh() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  city::wire::LayoutSettings edited = state.view().layout_settings();
  edited.min_side_scale = 1.0;
  edited.max_side_scale = 1.0;
  const auto updated = state.UpdateLayoutSettings(edited);
  if (!updated.ok || !updated.value) {
    return false;
  }

  city::wire::CoreState fresh;
  city::wire::LayoutSettings fresh_settings = fresh.view().layout_settings();
  fresh_settings.min_side_scale = edited.min_side_scale;
  fresh_settings.max_side_scale = edited.max_side_scale;
  const auto fresh_updated = fresh.UpdateLayoutSettings(fresh_settings);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  return fresh_updated.ok && fresh_generated.ok &&
         same_route_bundle_signatures(route_bundle_signatures(state, city::wire::BundleKind::kLowVoltage),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage));
}

bool C673_backbone_bundle_count_migration_rejects_user_attachments() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const auto attachment = state.AddAttachment(span_id, 0.5);
  if (!attachment.ok) {
    return false;
  }
  if (!update_low_voltage_count_to_two(state)) {
    return false;
  }
  const city::wire::Attachment* found = state.view().attachments().find(attachment.value);
  const auto attachment_it = state.view().relation_index().attachments_by_span.find(span_id);
  return found != nullptr && found->span_id == span_id &&
         attachment_it != state.view().relation_index().attachments_by_span.end() &&
         contains_id(attachment_it->second, attachment.value);
}

bool C698_backbone_regenerate_fixed_count_decrease_retires_lanes() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 2 ||
      state.view().backbone().edge_bundles.empty()) {
    return false;
  }
  const std::vector<city::wire::SavedBackboneNode> nodes_before = state.view().backbone().nodes;
  const std::vector<city::wire::SavedBackboneEdge> edges_before = state.view().backbone().edges;
  const std::vector<city::wire::SavedBackboneEdgeBundle> edge_bundles_before =
      state.view().backbone().edge_bundles;
  const std::vector<city::wire::ObjectId> retired_spans = span_ids_for_lane(state, 1);
  const std::vector<city::wire::ObjectId> retired_ports = port_ids_for_lane(state, 1);
  const std::size_t span_count_before = state.view().spans().size();
  if (retired_spans.size() != 1 || retired_ports.size() != 2) {
    return false;
  }

  const bool updated = update_low_voltage_count_to_one(state);
  if (!updated || state.view().spans().size() != span_count_before - 1 ||
      !same_saved_nodes(state.view().backbone().nodes, nodes_before) ||
      !same_saved_edges(state.view().backbone().edges, edges_before) ||
      state.view().backbone().edge_bundles.size() != edge_bundles_before.size()) {
    return false;
  }
  for (std::size_t i = 0; i < edge_bundles_before.size(); ++i) {
    const auto& after = state.view().backbone().edge_bundles[i];
    const auto& before = edge_bundles_before[i];
    if (after.edge_bundle_id != before.edge_bundle_id || after.edge_id != before.edge_id ||
        after.bundle_id != before.bundle_id || after.edge_forward != before.edge_forward ||
        after.route != before.route || after.order != before.order || !same_vec3(after.dir, before.dir)) {
      return false;
    }
  }
  for (city::wire::ObjectId span_id : retired_spans) {
    if (state.view().spans().find(span_id) != nullptr || state.find_curve_cache(span_id) != nullptr ||
        state.find_span_visual_cache(span_id) != nullptr || state.find_span_render_cache(span_id) != nullptr) {
      return false;
    }
  }
  for (city::wire::ObjectId port_id : retired_ports) {
    if (state.view().ports().find(port_id) != nullptr) {
      return false;
    }
  }
  if (!no_binding_references(state, retired_spans, retired_ports)) {
    return false;
  }

  city::wire::CoreState fresh;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  return fresh_generated.ok && fresh_generated.value.generated_span_ids.size() == 1 &&
         !std::filesystem::exists(repo_root() / "domains/wire/src/generation/backbone/bundle_count_migration.cpp") &&
         same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C699_backbone_regenerate_fixed_count_decrease_preserves_lateral_offset() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  city::wire::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = 1.0;
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  if (!update_low_voltage_count_to_one(state)) {
    return false;
  }

  city::wire::CoreState fresh;
  city::wire::BackboneSpec fresh_req = line_req(fresh);
  fresh_req.constraints.lateral_offset_m = 1.0;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(fresh_req);
  city::wire::CoreState fresh_default;
  const auto fresh_default_generated = fresh_default.GenerateFromBackboneSpec(line_req(fresh_default));
  return fresh_generated.ok && fresh_generated.value.generated_span_ids.size() == 1 &&
         fresh_default_generated.ok && fresh_default_generated.value.generated_span_ids.size() == 1 &&
         state.view().backbone().edges.size() == 1 &&
         almost_equal(state.view().backbone().edges.front().lateral_offset_m, 1.0, 1e-12) &&
         !same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh_default)) &&
         same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C700_backbone_regenerate_fixed_count_decrease_rejects_retired_attachment() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  const std::vector<city::wire::ObjectId> retired_spans = span_ids_for_lane(state, 1);
  if (!generated.ok || retired_spans.size() != 1) {
    return false;
  }
  const auto attachment = state.AddAttachment(retired_spans.front(), 0.5);
  if (!attachment.ok) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_one(state, &error);
  return !updated && contains_text(error, "attachments") && same_counts(before, count_snapshot(state));
}

bool C701_backbone_regenerate_source_does_not_infer_topology_from_outputs() {
  std::string source{};
  if (!file_text(repo_root() / "domains/wire/src/generation/backbone/regenerate.cpp", &source)) {
    return false;
  }
  return source.find("build_input_from_saved_scope") != std::string::npos &&
         source.find("trial_pipeline.build(") != std::string::npos &&
         source.find("span_layout(") == std::string::npos &&
         source.find("find_curve_cache") == std::string::npos &&
         source.find("find_span_visual_cache") == std::string::npos &&
         source.find("find_span_render_cache") == std::string::npos &&
         source.find("world_position") == std::string::npos;
}

bool C702_backbone_regenerate_fixed_count_roundtrip_matches_fresh() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  const auto template_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate edited = template_it->second;
  edited.fixed_count = 3;
  const auto increased = state.UpdateBundleTemplate(edited);
  if (!increased.ok || !increased.value || state.view().spans().size() != 3) {
    return false;
  }
  if (!update_low_voltage_count_to_one(state) || state.view().spans().size() != 1) {
    return false;
  }

  city::wire::CoreState fresh;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  return fresh_generated.ok && fresh_generated.value.generated_span_ids.size() == 1 &&
         !std::filesystem::exists(repo_root() / "domains/wire/src/generation/backbone/bundle_count_migration.cpp") &&
         same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C703_backbone_regenerate_removes_migration_symbols() {
  const std::filesystem::path src = repo_root() / "domains/wire/src";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(src)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string source{};
    if (!file_text(entry.path(), &source)) {
      return false;
    }
    if (contains_text(source, "migrate_backbone_bundle") ||
        contains_text(source, "regenerate_backbone_bundle_count_change")) {
      return false;
    }
  }
  return !std::filesystem::exists(repo_root() / "domains/wire/src/generation/backbone/bundle_count_migration.cpp");
}

bool C704_backbone_regenerate_uses_per_api_entrypoint_not_plan_execution() {
  std::string pipeline_header{};
  std::string pipeline_source{};
  std::string regenerate_source{};
  std::string derive_source{};
  if (!file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.hpp", &pipeline_header) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.cpp", &pipeline_source) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/regenerate.cpp", &regenerate_source) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/derive.cpp", &derive_source)) {
    return false;
  }
  const std::size_t reject = derive_source.find("plan.kind == UpdateKind::kRegenerate");
  const std::size_t loop = derive_source.find("for (ObjectId span_id");
  const std::string old_entry = std::string("build_prepared_") + "migration";
  return pipeline_header.find(old_entry) == std::string::npos &&
         pipeline_source.find(old_entry) == std::string::npos &&
         regenerate_source.find(old_entry) == std::string::npos &&
         pipeline_header.find("build_prepared_regenerate") == std::string::npos &&
         pipeline_source.find("build_prepared_regenerate") == std::string::npos &&
         regenerate_source.find("build_prepared_regenerate") == std::string::npos &&
         regenerate_source.find("build_input_from_saved_scope") != std::string::npos &&
         regenerate_source.find("trial_pipeline.build(") != std::string::npos &&
         reject != std::string::npos && loop != std::string::npos && reject < loop;
}

bool C727_backbone_pipeline_execution_entry_is_build_input() {
  std::string source{};
  std::string header{};
  std::string regenerate_source{};
  if (!file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.cpp", &source) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.hpp", &header) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/regenerate.cpp", &regenerate_source)) {
    return false;
  }
  const std::vector<std::string> operation_names = {
      "UpdateBundleTemplate", "UpdateCableTemplate", "UpdatePoleTypeDefinition", "UpdateLayoutSettings",
      "ApplyBundleRelatedPoleTypeToExistingPoles", "SetSpanEndpointSocketOverride",
      "ClearSpanEndpointSocketOverride", "SetSpanBranchDownOffsetOverride", "ClearSpanBranchDownOffsetOverride"};
  for (const std::string& name : operation_names) {
    if (contains_text(source, name) || contains_text(header, name)) {
      return false;
    }
  }
  if (contains_text(header, "build_prepared_regenerate") || contains_text(source, "build_prepared_regenerate") ||
      contains_text(header, "run(") || contains_text(source, "pipeline::run(") ||
      contains_text(header, "run_input") || contains_text(source, "run_input")) {
    return false;
  }
  return contains_text(header, "struct build_input") &&
         contains_text(header, "graph made{}") &&
         contains_text(header, "std::vector<std::size_t> active_bundle_indices{}") &&
         contains_text(header, "std::vector<std::size_t> local_by_input{}") &&
         !contains_text(header, "template_overrides") && !contains_text(source, "template_overrides") &&
         !contains_text(header, "template_override(") && !contains_text(source, "template_override(") &&
         contains_text(header, "build_input() = default") &&
         contains_text(header, "friend class pipeline") &&
         !contains_text(header, "run_mode") && !contains_text(source, "run_mode") &&
         !contains_text(header, "mode_") && !contains_text(source, "mode_") &&
         !contains_text(header, "include_new_poles") && !contains_text(source, "include_new_poles") &&
         !contains_text(header, "run_preflight") && !contains_text(source, "run_preflight") &&
         !contains_text(header, "ready_") && !contains_text(source, "ready_") &&
         !contains_text(header, "bool ready") &&
         contains_text(header, "build_input_from_spec") &&
         contains_text(header, "build_input_from_saved_scope") &&
         contains_text(header, "EditResult<GenerateBundleFromPathResult> build(build_input input)") &&
         contains_text(source, "GenerationTiming* timing = input.timing == nullptr ? &out.value.timing : input.timing") &&
         contains_text(regenerate_source, "trial.authoritative_.bundle_templates[bundle_template_id] = next_template") &&
         contains_text(regenerate_source, "trial_pipeline.build(") &&
         contains_text(regenerate_source, "build_input_from_saved_scope");
}

bool C728_backbone_pipeline_has_no_run_mode_flags() {
  std::string source{};
  std::string header{};
  WIRE_TEST_EXPECT(
      file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.cpp",
                &source),
      "pipeline.cpp is missing");
  WIRE_TEST_EXPECT(
      file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.hpp",
                &header),
      "pipeline.hpp is missing");
  const std::vector<std::string> banned = {
      "run_mode", "mode_", "include_new_poles", "run_preflight", "ready_", "bool ready"};
  for (const std::string& token : banned) {
    WIRE_TEST_EXPECT(!contains_text(source, token) &&
                         !contains_text(header, token),
                     "pipeline run-mode token returned: " + token);
  }
  WIRE_TEST_EXPECT(
      contains_text(source,
                    "moved_more_than_epsilon(existing_port->world_position, p)"),
      "existing Port movement detection is missing");
  WIRE_TEST_EXPECT(contains_text(source, "path_index_by_local[i] >= 0"),
                   "path-local input detection is missing");
  std::string body{};
  WIRE_TEST_EXPECT(
      function_body(
          source,
          "EditResult<bool> pipeline::emit_ports(topo* made, const pairs& ps,",
          &body),
      "pipeline::emit_ports body is missing");
  WIRE_TEST_EXPECT(contains_text(body, "Port* existing_port"),
                   "existing Port resolution is missing");
  WIRE_TEST_EXPECT(
      contains_text(body,
                    "state_.update_backbone_port_binding_frame_exact("),
      "existing auto Port is not updated through the exact binding owner");
  WIRE_TEST_EXPECT(
      !contains_text(body, "existing_port->world_position = p"),
      "pipeline bypasses the exact binding owner with a direct Port assignment");
  WIRE_TEST_EXPECT(
      contains_text(body, "ApplyPortBandTemplateFields(existing_port, band)"),
      "existing Port band fields are not refreshed");
  WIRE_TEST_EXPECT(contains_text(body, "changes->updated_ids"),
                   "existing Port updates are not recorded");
  WIRE_TEST_EXPECT(!contains_text(body, "mode_") &&
                       !contains_text(body, "run_mode"),
                   "pipeline::emit_ports contains a run-mode branch");
  return true;
}

bool C729_backbone_regenerate_source_does_not_handbuild_outputs() {
  std::string source{};
  std::string pipeline_source{};
  if (!file_text(repo_root() / "domains/wire/src/generation/backbone/regenerate.cpp", &source) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/pipeline.cpp", &pipeline_source)) {
    return false;
  }
  const std::vector<std::string> banned = {
      "AddPort(", "AddSpan(", "SpanLayoutRule", "SpanLayoutEntry", "store_rules",
      "store_layout", "cache_span_curve", "cache_span_bounds", "execute_update_plan",
      "retire_from_trial", "remove_span_from_caches"};
  for (const std::string& token : banned) {
    if (contains_text(source, token)) {
      return false;
    }
  }
  return contains_text(source, "trial_pipeline.build(") &&
         contains_text(source, "build_input_from_saved_scope") &&
         contains_text(source, "CoreState trial = *this") &&
         contains_text(pipeline_source, "retire_untouched(&made.value)") &&
         contains_text(pipeline_source, "void pipeline::retire_untouched(route* route)") &&
         contains_text(pipeline_source, "state_.remove_span_from_caches(span_id)");
}
bool C705_backbone_edge_bundle_order_matches_bundle_spec_order() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || state.view().backbone().edge_bundles.size() != req.bundles.size()) {
    return false;
  }
  for (std::size_t i = 0; i < req.bundles.size(); ++i) {
    const city::wire::SavedBackboneEdgeBundle& edge_bundle = state.view().backbone().edge_bundles[i];
    const city::wire::Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != req.bundles[i].bundle_template_id) {
      return false;
    }
  }
  return true;
}

bool C706_backbone_regenerate_multi_bundle_count_change_matches_fresh() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto generated = state.GenerateFromBackboneSpec(req);
  const city::wire::ObjectId lv_edge_bundle =
      edge_bundle_id_for_template(state, city::wire::BundleKind::kLowVoltage);
  const city::wire::ObjectId comm_edge_bundle =
      edge_bundle_id_for_template(state, city::wire::BundleKind::kCommunication);
  if (!generated.ok || lv_edge_bundle == city::wire::kInvalidObjectId ||
      comm_edge_bundle == city::wire::kInvalidObjectId) {
    return false;
  }
  const EdgeBundleIdentitySnapshot comm_before = edge_bundle_identity_snapshot(state, comm_edge_bundle);
  if (!update_low_voltage_count_to_two(state)) {
    return false;
  }
  const city::wire::ObjectId lv_after =
      edge_bundle_id_for_template(state, city::wire::BundleKind::kLowVoltage);
  const city::wire::ObjectId comm_after =
      edge_bundle_id_for_template(state, city::wire::BundleKind::kCommunication);
  if (lv_after != lv_edge_bundle || comm_after != comm_edge_bundle ||
      !same_edge_bundle_identity_snapshot(comm_before, edge_bundle_identity_snapshot(state, comm_edge_bundle))) {
    return false;
  }

  city::wire::CoreState fresh;
  if (!set_low_voltage_count_before_generation(fresh, 2)) {
    return false;
  }
  city::wire::BackboneSpec fresh_req = line_req(fresh);
  add_backbone_bundle(fresh_req, city::wire::BundleKind::kCommunication);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(fresh_req);
  const city::wire::ObjectId fresh_lv =
      edge_bundle_id_for_template(fresh, city::wire::BundleKind::kLowVoltage);
  const city::wire::ObjectId fresh_comm =
      edge_bundle_id_for_template(fresh, city::wire::BundleKind::kCommunication);
  return fresh_generated.ok &&
         same_span_curve_signatures(span_curve_signatures_for_edge_bundle(state, lv_edge_bundle),
                                    span_curve_signatures_for_edge_bundle(fresh, fresh_lv)) &&
         same_span_curve_signatures(span_curve_signatures_for_edge_bundle(state, comm_edge_bundle),
                                    span_curve_signatures_for_edge_bundle(fresh, fresh_comm));
}

bool C707_backbone_saved_route_continuity_is_row_continuity() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || state.view().backbone().edges.size() != 2) {
    return false;
  }
  const std::vector<city::wire::ObjectId> edge_bundle_ids =
      edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage);
  if (edge_bundle_ids.size() != 2) {
    return false;
  }
  const city::wire::ObjectId a = edge_bundle_ids[0];
  const city::wire::ObjectId b = edge_bundle_ids[1];
  return std::any_of(state.view().backbone().row_continuities.begin(),
                     state.view().backbone().row_continuities.end(),
                     [&](const city::wire::SavedBackboneRowContinuity& continuity) {
                       const bool forward = continuity.a.edge_bundle_id == a &&
                                            continuity.b.edge_bundle_id == b &&
                                            continuity.a.lane_index == 0 &&
                                            continuity.b.lane_index == 0;
                       const bool reverse = continuity.a.edge_bundle_id == b &&
                                            continuity.b.edge_bundle_id == a &&
                                            continuity.a.lane_index == 0 &&
                                            continuity.b.lane_index == 0;
                       return forward || reverse;
                     });
}

bool C708_backbone_regenerate_polyline_decrease_matches_fresh() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage).size() != 2) {
    return false;
  }
  const auto target_edge_bundle_ids = edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage);
  city::wire::BackboneSpec outside_req = line_req(state);
  outside_req.bundles.clear();
  add_backbone_bundle(outside_req, city::wire::BundleKind::kCommunication);
  for (city::wire::Vec3d& point : outside_req.path.polyline) {
    point.y += 30.0;
  }
  const auto outside = state.GenerateFromBackboneSpec(outside_req);
  if (!outside.ok || outside.value.generated_span_ids.empty()) {
    return false;
  }
  const auto outside_before = snapshot_span_outputs(state, outside.value.generated_span_ids);
  if (!update_low_voltage_count_to_one(state)) {
    return false;
  }

  city::wire::CoreState fresh;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  return fresh_generated.ok &&
         same_route_bundle_signatures(route_bundle_signatures_for_ids(state, target_edge_bundle_ids),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage)) &&
         same_span_output_snapshots(outside_before, state) &&
         visual_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) ==
             visual_part_count(fresh, city::wire::VisualCurvePartKind::kNodePatch);
}

bool C709_backbone_regenerate_polyline_increase_matches_fresh() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage).size() != 2) {
    return false;
  }
  const auto target_edge_bundle_ids = edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage);
  city::wire::BackboneSpec outside_req = line_req(state);
  outside_req.bundles.clear();
  add_backbone_bundle(outside_req, city::wire::BundleKind::kCommunication);
  for (city::wire::Vec3d& point : outside_req.path.polyline) {
    point.y += 30.0;
  }
  const auto outside = state.GenerateFromBackboneSpec(outside_req);
  if (!outside.ok || outside.value.generated_span_ids.empty()) {
    return false;
  }
  const auto outside_before = snapshot_span_outputs(state, outside.value.generated_span_ids);
  if (!update_low_voltage_count_to_two(state)) {
    return false;
  }

  city::wire::CoreState fresh;
  if (!set_low_voltage_count_before_generation(fresh, 2)) {
    return false;
  }
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  return fresh_generated.ok &&
         same_route_bundle_signatures(route_bundle_signatures_for_ids(state, target_edge_bundle_ids),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage)) &&
         same_span_output_snapshots(outside_before, state) &&
         visual_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) ==
             visual_part_count(fresh, city::wire::VisualCurvePartKind::kNodePatch);
}

bool C710_backbone_regenerate_polyline_multi_bundle_matches_fresh() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = poly3_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage).size() != 2 ||
      edge_bundle_ids_for_template(state, city::wire::BundleKind::kCommunication).size() != 2) {
    return false;
  }
  const auto target_lv_edge_bundle_ids = edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage);
  const auto target_comm_edge_bundle_ids = edge_bundle_ids_for_template(state, city::wire::BundleKind::kCommunication);
  city::wire::BackboneSpec outside_req = line_req(state);
  outside_req.bundles.clear();
  add_backbone_bundle(outside_req, city::wire::BundleKind::kOptical);
  for (city::wire::Vec3d& point : outside_req.path.polyline) {
    point.y += 30.0;
  }
  const auto outside = state.GenerateFromBackboneSpec(outside_req);
  if (!outside.ok || outside.value.generated_span_ids.empty()) {
    return false;
  }
  const auto outside_before = snapshot_span_outputs(state, outside.value.generated_span_ids);
  if (!update_low_voltage_count_to_two(state)) {
    return false;
  }

  city::wire::CoreState fresh;
  if (!set_low_voltage_count_before_generation(fresh, 2)) {
    return false;
  }
  city::wire::BackboneSpec fresh_req = poly3_req(fresh);
  add_backbone_bundle(fresh_req, city::wire::BundleKind::kCommunication);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(fresh_req);
  return fresh_generated.ok &&
         same_route_bundle_signatures(route_bundle_signatures_for_ids(state, target_lv_edge_bundle_ids),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage)) &&
         same_route_bundle_signatures(route_bundle_signatures_for_ids(state, target_comm_edge_bundle_ids),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kCommunication)) &&
         same_span_output_snapshots(outside_before, state) &&
         visual_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) ==
             visual_part_count(fresh, city::wire::VisualCurvePartKind::kNodePatch);
}

bool C711_backbone_regenerate_decrease_preserves_surviving_attachment() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  const std::vector<city::wire::ObjectId> lane0_spans = span_ids_for_lane(state, 0);
  if (lane0_spans.size() != 1) {
    return false;
  }
  const auto attachment = state.AddAttachment(lane0_spans.front(), 0.5);
  if (!attachment.ok) {
    return false;
  }
  if (!update_low_voltage_count_to_one(state)) {
    return false;
  }
  const city::wire::Attachment* found = state.view().attachments().find(attachment.value);
  const auto attachment_it = state.view().relation_index().attachments_by_span.find(lane0_spans.front());
  return found != nullptr && found->span_id == lane0_spans.front() &&
         attachment_it != state.view().relation_index().attachments_by_span.end() &&
         contains_id(attachment_it->second, attachment.value);
}

bool C712_backbone_regenerate_cable_decision_matches_fresh() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage).size() != 2) {
    return false;
  }
  const auto bundle_template_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (bundle_template_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  city::wire::CableTemplate edited = cable_it->second;
  edited.continuity_policy = city::wire::CableContinuityPolicyHint::kPreferG1;
  const auto updated = state.UpdateCableTemplate(edited);
  if (!updated.ok || !updated.value ||
      state.view().cable_templates().at(edited.id).continuity_policy != edited.continuity_policy ||
      !route_bundle_curves_request_policy(state, city::wire::BundleKind::kLowVoltage, edited.continuity_policy)) {
    return false;
  }

  city::wire::CoreState fresh;
  const auto fresh_bundle_template_it = fresh.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (fresh_bundle_template_it == fresh.view().bundle_templates().end()) {
    return false;
  }
  auto fresh_cable_it = fresh.view().cable_templates().find(fresh_bundle_template_it->second.cable_template_id);
  if (fresh_cable_it == fresh.view().cable_templates().end()) {
    return false;
  }
  city::wire::CableTemplate fresh_edited = fresh_cable_it->second;
  fresh_edited.continuity_policy = edited.continuity_policy;
  const auto fresh_template_update = fresh.UpdateCableTemplate(fresh_edited);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  return fresh_template_update.ok && fresh_generated.ok &&
         same_route_bundle_signatures(route_bundle_signatures(state, city::wire::BundleKind::kLowVoltage),
                                      route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage)) &&
         visual_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) ==
             visual_part_count(fresh, city::wire::VisualCurvePartKind::kNodePatch);
}

bool C738_cable_default_endpoint_attachment_change_reconciles_auto_endpoints() {
  city::wire::CoreState state;
  const auto bundle_template_it =
      state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (bundle_template_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  city::wire::AttachmentTemplateId initial_attachment_template_id = city::wire::kInvalidAttachmentTemplateId;
  city::wire::AttachmentTemplateId replacement_attachment_template_id = city::wire::kInvalidAttachmentTemplateId;
  city::wire::AttachmentKind initial_attachment_kind = city::wire::AttachmentKind::kGeneric;
  for (const auto& [template_id, attachment_template] : state.view().attachment_templates()) {
    if (initial_attachment_template_id == city::wire::kInvalidAttachmentTemplateId) {
      initial_attachment_template_id = template_id;
      initial_attachment_kind = attachment_template.kind;
      continue;
    }
    replacement_attachment_template_id = template_id;
    break;
  }
  if (initial_attachment_template_id == city::wire::kInvalidAttachmentTemplateId ||
      replacement_attachment_template_id == city::wire::kInvalidAttachmentTemplateId) {
    return false;
  }

  city::wire::CableTemplate edited = cable_it->second;
  edited.default_endpoint_attachment_template_id = initial_attachment_template_id;
  if (!state.UpdateCableTemplate(edited).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::Span* generated_span = state.view().spans().find(span_id);
  if (generated_span == nullptr || generated_span->endpoint_attachment_a_id == city::wire::kInvalidObjectId ||
      generated_span->endpoint_attachment_b_id == city::wire::kInvalidObjectId) {
    return false;
  }
  const city::wire::ObjectId initial_endpoint_a_id = generated_span->endpoint_attachment_a_id;
  const city::wire::ObjectId initial_endpoint_b_id = generated_span->endpoint_attachment_b_id;
  const auto user_attachment = state.AddAttachment(span_id, 0.5, initial_attachment_kind, 0.0,
                                                   initial_attachment_template_id);
  if (!user_attachment.ok) {
    return false;
  }

  city::wire::BackboneSpec outside = line_req(state);
  outside.path.polyline = {{0.0, 100.0, 0.0}, {12.0, 100.0, 0.0}};
  outside.bundles.clear();
  add_backbone_bundle(outside, city::wire::BundleKind::kCommunication);
  const auto outside_generated = state.GenerateFromBackboneSpec(outside);
  if (!outside_generated.ok || outside_generated.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<SpanOutputSnapshot> outside_before =
      snapshot_span_outputs(state, outside_generated.value.generated_span_ids);

  edited.default_endpoint_attachment_template_id = replacement_attachment_template_id;
  const auto updated = state.UpdateCableTemplate(edited);
  const city::wire::Span* after_span = state.view().spans().find(span_id);
  const city::wire::Attachment* user_after = state.view().attachments().find(user_attachment.value);
  if (!updated.ok || !updated.value || after_span == nullptr || user_after == nullptr ||
      user_after->span_id != span_id || user_after->template_id != initial_attachment_template_id ||
      user_after->origin != city::wire::AttachmentOrigin::kUser || !almost_equal(user_after->t, 0.5, 1e-12) ||
      after_span->endpoint_attachment_a_id == city::wire::kInvalidObjectId ||
      after_span->endpoint_attachment_b_id == city::wire::kInvalidObjectId) {
    return false;
  }
  const city::wire::Attachment* endpoint_a = state.view().attachments().find(after_span->endpoint_attachment_a_id);
  const city::wire::Attachment* endpoint_b = state.view().attachments().find(after_span->endpoint_attachment_b_id);
  if (endpoint_a == nullptr || endpoint_b == nullptr || endpoint_a->origin != city::wire::AttachmentOrigin::kDefaultEndpoint ||
      endpoint_b->origin != city::wire::AttachmentOrigin::kDefaultEndpoint ||
      endpoint_a->template_id != replacement_attachment_template_id || endpoint_b->template_id != replacement_attachment_template_id ||
      state.view().attachments().find(initial_endpoint_a_id) != nullptr ||
      state.view().attachments().find(initial_endpoint_b_id) != nullptr ||
      !same_span_output_snapshots(outside_before, state)) {
    return false;
  }

  city::wire::CoreState fresh;
  const auto fresh_bundle_it =
      fresh.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (fresh_bundle_it == fresh.view().bundle_templates().end()) {
    return false;
  }
  const auto fresh_cable_it = fresh.view().cable_templates().find(fresh_bundle_it->second.cable_template_id);
  if (fresh_cable_it == fresh.view().cable_templates().end()) {
    return false;
  }
  city::wire::CableTemplate fresh_edited = fresh_cable_it->second;
  fresh_edited.default_endpoint_attachment_template_id = replacement_attachment_template_id;
  const auto fresh_updated = fresh.UpdateCableTemplate(fresh_edited);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  if (!fresh_updated.ok || !fresh_generated.ok || fresh_generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::Span* fresh_span = fresh.view().spans().find(fresh_generated.value.generated_span_ids.front());
  if (fresh_span == nullptr) {
    return false;
  }
  const city::wire::Attachment* fresh_a = fresh.view().attachments().find(fresh_span->endpoint_attachment_a_id);
  const city::wire::Attachment* fresh_b = fresh.view().attachments().find(fresh_span->endpoint_attachment_b_id);
  return fresh_a != nullptr && fresh_b != nullptr && fresh_a->origin == city::wire::AttachmentOrigin::kDefaultEndpoint &&
         fresh_b->origin == city::wire::AttachmentOrigin::kDefaultEndpoint && fresh_a->template_id == endpoint_a->template_id &&
         fresh_b->template_id == endpoint_b->template_id && almost_equal(fresh_a->t, endpoint_a->t, 1e-12) &&
         almost_equal(fresh_b->t, endpoint_b->t, 1e-12);
}

bool C713_backbone_regenerate_pole_type_structure_matches_fresh() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty()) {
    return false;
  }
  const city::wire::Pole* pole = state.view().poles().find(generated.value.generated_pole_ids.front());
  if (pole == nullptr) {
    return false;
  }
  const auto type_it = state.view().pole_types().find(pole->pole_type_id);
  if (type_it == state.view().pole_types().end()) {
    return false;
  }

  city::wire::PoleTypeDefinition edited = type_it->second;
  bool edited_band = false;
  for (city::wire::PortPlacementBand& band : edited.port_bands) {
    if (band.enabled && band.category == city::wire::ConnectionCategory::kLowVoltage &&
        band.role == city::wire::SlotRole::kTrunkPreferred && band.side == city::wire::SlotSide::kLeft) {
      band.side = city::wire::SlotSide::kCenter;
      band.lateral_center_m = 0.0;
      band.lateral_min_m = -0.18;
      band.lateral_max_m = 0.18;
      edited_band = true;
      break;
    }
  }
  if (!edited_band) {
    return false;
  }

  const auto updated = state.UpdatePoleTypeDefinition(edited);
  if (!updated.ok) {
    return false;
  }

  city::wire::CoreState fresh;
  const auto fresh_type_it = fresh.view().pole_types().find(edited.id);
  if (fresh_type_it == fresh.view().pole_types().end()) {
    return false;
  }
  city::wire::PoleTypeDefinition fresh_edited = fresh_type_it->second;
  bool fresh_edited_band = false;
  for (city::wire::PortPlacementBand& band : fresh_edited.port_bands) {
    if (band.enabled && band.category == city::wire::ConnectionCategory::kLowVoltage &&
        band.role == city::wire::SlotRole::kTrunkPreferred && band.side == city::wire::SlotSide::kLeft) {
      band.side = city::wire::SlotSide::kCenter;
      band.lateral_center_m = 0.0;
      band.lateral_min_m = -0.18;
      band.lateral_max_m = 0.18;
      fresh_edited_band = true;
      break;
    }
  }
  const auto fresh_template_update = fresh.UpdatePoleTypeDefinition(fresh_edited);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  WIRE_TEST_EXPECT_PRESENCE(fresh_edited_band,
                            "fresh pole type has no matching LV band");
  WIRE_TEST_EXPECT_PRESENCE(fresh_template_update.ok,
                            fresh_template_update.error);
  WIRE_TEST_EXPECT_PRESENCE(fresh_generated.ok, fresh_generated.error);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(fresh);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      same_route_bundle_signatures(
          route_bundle_signatures(state,
                                  city::wire::BundleKind::kLowVoltage),
          route_bundle_signatures(fresh,
                                  city::wire::BundleKind::kLowVoltage)) &&
          visual_part_count(state,
                            city::wire::VisualCurvePartKind::kNodePatch) ==
              visual_part_count(fresh,
                                city::wire::VisualCurvePartKind::kNodePatch),
      "regenerated pole structure does not match fresh generation");
  return true;
}

bool C676_backbone_noop_move_preserves_port_positions_exactly() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty()) {
    return false;
  }
  std::vector<std::pair<city::wire::ObjectId, city::wire::Vec3d>> before{};
  for (const city::wire::Port& port : state.view().ports().items()) {
    if (state.view().backbone_port_binding_for_port(port.id) != nullptr) {
      before.push_back({port.id, port.world_position});
    }
  }
  if (before.empty()) {
    return false;
  }
  for (city::wire::ObjectId pole_id : generated.value.generated_pole_ids) {
    const city::wire::Pole* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto moved = state.MovePole(pole_id, pole->world_transform);
    if (!moved.ok) {
      return false;
    }
  }
  for (const auto& [port_id, position] : before) {
    const city::wire::Port* port = state.view().ports().find(port_id);
    if (port == nullptr || port->world_position.x != position.x ||
        port->world_position.y != position.y || port->world_position.z != position.z) {
      return false;
    }
  }
  return true;
}

bool C622_backbone_stage_timing_is_diagnostic_only() {
  city::wire::CoreState state;
  city::wire::GeometrySettings initial_settings = state.view().geometry_settings();
  initial_settings.sag_enabled = false;
  if (!state.UpdateGeometrySettings(initial_settings).ok) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::GenerationTiming& timing = out.value.timing;
  const std::vector<double> stages = {
      timing.state_copy_ms,    timing.prepare_ms,     timing.check_ms, timing.pairs_ms, timing.preflight_ms,
      timing.intent_ms,        timing.support_groups_ms, timing.emit_ms, timing.save_graph_ms,
      timing.rules_ms,         timing.layout_ms,      timing.geom_ms,  timing.draw_ms,
  };
  if (timing.total_ms <= 0.0 ||
      std::find_if(stages.begin(), stages.end(), [](double value) { return value < 0.0; }) != stages.end()) {
    return false;
  }

  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  const auto updated = state.UpdateGeometrySettings(settings);
  const city::wire::UpdateTiming& update = state.view().last_update_timing();
  return updated.ok && update.kind == city::wire::UpdateKind::kReshape &&
         update.affected_span_count == out.value.generated_span_ids.size() && update.plan_ms >= 0.0 &&
         update.derive_ms >= 0.0 && update.total_ms >= update.derive_ms;
}

bool C742_backbone_bundle_count_decrease_allows_metadata_change() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  const auto template_it = state.view().bundle_templates().find(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate edited = template_it->second;
  edited.fixed_count = 1;
  edited.name += " renamed";
  const auto updated = state.UpdateBundleTemplate(edited);
  const auto after = state.view().bundle_templates().find(edited.id);
  return updated.ok && updated.value && state.view().spans().size() == 1 &&
         after != state.view().bundle_templates().end() && after->second.name == edited.name &&
         after->second.fixed_count == 1;
}

bool C743_backbone_bundle_template_change_classification_has_one_field_owner() {
  std::string source{};
  if (!file_text(repo_root() / "domains/wire/src/state/template/update.cpp", &source)) {
    return false;
  }
  const std::size_t begin = source.find("TemplateMutationService::UpdateBundleTemplate");
  const std::size_t end = source.find("TemplateMutationService::UpdateAttachmentTemplate", begin);
  if (begin == std::string::npos || end == std::string::npos) {
    return false;
  }
  const std::string update_body = source.substr(begin, end - begin);
  return update_body.find("classify_bundle_template_changes(previous, normalized)") != std::string::npos &&
         update_body.find("it->second.") == std::string::npos;
}

bool C744_backbone_span_layout_group_keys_have_one_definition() {
  const std::filesystem::path root = repo_root() / "domains/wire/src/generation/backbone";
  std::size_t definitions = 0;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".cpp") {
      continue;
    }
    std::string source{};
    if (!file_text(entry.path(), &source)) {
      return false;
    }
    std::size_t offset = 0;
    while ((offset = source.find("auto append_group_key =", offset)) != std::string::npos) {
      ++definitions;
      offset += std::string_view("auto append_group_key =").size();
    }
  }
  std::string pipeline{};
  std::string derive{};
  return definitions == 1 && file_text(root / "pipeline.cpp", &pipeline) &&
         file_text(root / "derive.cpp", &derive) &&
         contains_text(pipeline, "derive_span_layout(rule, endpoint_resolver, 0)") &&
         contains_text(derive, "derive_span_layout(");
}

bool C746_backbone_generation_trial_copy_stays_under_cost_gate() {
  city::wire::CoreState state;
  city::wire::BackboneSpec first = line_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {650.0, 0.0, 0.0}};
  first.interval_m = 10.0;
  const auto populated = state.GenerateFromBackboneSpec(first);
  if (!populated.ok || populated.value.generated_pole_ids.size() < 66) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {{0.0, 100.0, 0.0}, {650.0, 100.0, 0.0}};
  second.interval_m = 10.0;
  const auto measured = state.GenerateFromBackboneSpec(second);
  return measured.ok && measured.value.generated_pole_ids.size() >= 66 && measured.value.timing.total_ms > 0.0 &&
         measured.value.timing.state_copy_ms / measured.value.timing.total_ms <= 0.20;
}

bool C747_backbone_range_count_policy_validates_without_regenerate() {
  city::wire::CoreState accepted_state;
  const auto generated = accepted_state.GenerateFromBackboneSpec(line_req(accepted_state));
  if (!generated.ok || generated.value.generated_span_ids.empty() || accepted_state.view().backbone().edge_bundles.empty()) {
    return false;
  }
  const city::wire::ObjectId edge_bundle_id = accepted_state.view().backbone().edge_bundles.front().edge_bundle_id;
  const EdgeBundleIdentitySnapshot identity_before = edge_bundle_identity_snapshot(accepted_state, edge_bundle_id);
  const auto curves_before = span_curve_signatures(accepted_state);
  const auto template_it = accepted_state.view().bundle_templates().find(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (template_it == accepted_state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate accepted = template_it->second;
  accepted.count_rule = city::wire::BundleCountRuleKind::kRange;
  accepted.fixed_count = 0;
  accepted.min_count = 1;
  accepted.max_count = 3;
  accepted.default_count = 2;
  const auto accepted_out = accepted_state.UpdateBundleTemplate(accepted);
  const auto accepted_after = accepted_state.view().bundle_templates().find(accepted.id);
  if (!accepted_out.ok || !accepted_out.value || accepted_after == accepted_state.view().bundle_templates().end() ||
      accepted_after->second.count_rule != city::wire::BundleCountRuleKind::kRange ||
      accepted_after->second.min_count != 1 || accepted_after->second.max_count != 3 ||
      accepted_after->second.default_count != 2 ||
      !same_edge_bundle_identity_snapshot(identity_before, edge_bundle_identity_snapshot(accepted_state, edge_bundle_id)) ||
      !same_span_curve_signatures(curves_before, span_curve_signatures(accepted_state)) ||
      std::any_of(generated.value.generated_span_ids.begin(), generated.value.generated_span_ids.end(),
                  [&](city::wire::ObjectId span_id) { return contains_id(accepted_out.change_set.updated_ids, span_id); })) {
    return false;
  }

  city::wire::CoreState rejected_state;
  const auto rejected_generated = rejected_state.GenerateFromBackboneSpec(line_req(rejected_state));
  if (!rejected_generated.ok || rejected_state.view().backbone().edge_bundles.empty()) {
    return false;
  }
  const city::wire::ObjectId rejected_edge_bundle_id = rejected_state.view().backbone().edge_bundles.front().edge_bundle_id;
  const EdgeBundleIdentitySnapshot rejected_identity_before =
      edge_bundle_identity_snapshot(rejected_state, rejected_edge_bundle_id);
  const auto rejected_curves_before = span_curve_signatures(rejected_state);
  const auto rejected_template_it = rejected_state.view().bundle_templates().find(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (rejected_template_it == rejected_state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate rejected = rejected_template_it->second;
  rejected.count_rule = city::wire::BundleCountRuleKind::kRange;
  rejected.fixed_count = 0;
  rejected.min_count = 2;
  rejected.max_count = 3;
  rejected.default_count = 2;
  const auto rejected_out = rejected_state.UpdateBundleTemplate(rejected);
  return !rejected_out.ok && contains_text(rejected_out.error, std::to_string(rejected_state.view().bundles().items().front().id)) &&
         same_edge_bundle_identity_snapshot(rejected_identity_before,
                                            edge_bundle_identity_snapshot(rejected_state, rejected_edge_bundle_id)) &&
         same_span_curve_signatures(rejected_curves_before, span_curve_signatures(rejected_state)) &&
         rejected_state.view().bundle_templates().find(rejected.id)->second.count_rule ==
             city::wire::BundleCountRuleKind::kFixed;
}

bool C748_backbone_bundle_policy_regenerates_scope_or_rejects_before_mutation() {
  city::wire::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<city::wire::ObjectId> target_edge_bundle_ids =
      edge_bundle_ids_for_template(state, city::wire::BundleKind::kLowVoltage);
  if (target_edge_bundle_ids.empty()) {
    return false;
  }

  city::wire::BackboneSpec outside = line_req(state);
  outside.path.polyline = {{0.0, 100.0, 0.0}, {12.0, 100.0, 0.0}};
  outside.bundles.clear();
  add_backbone_bundle(outside, city::wire::BundleKind::kCommunication);
  const auto outside_generated = state.GenerateFromBackboneSpec(outside);
  if (!outside_generated.ok || outside_generated.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<SpanOutputSnapshot> outside_before =
      snapshot_span_outputs(state, outside_generated.value.generated_span_ids);

  const city::wire::BundleTemplateId template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  const auto template_it = state.view().bundle_templates().find(template_id);
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate spacing_edited = template_it->second;
  spacing_edited.default_spacing_m += 0.15;
  const auto spacing_updated = state.UpdateBundleTemplate(spacing_edited);
  if (!spacing_updated.ok || !spacing_updated.value || !same_span_output_snapshots(outside_before, state)) {
    return false;
  }

  city::wire::CoreState fresh;
  if (!set_low_voltage_count_before_generation(fresh, 2)) {
    return false;
  }
  const auto fresh_template_it = fresh.view().bundle_templates().find(template_id);
  if (fresh_template_it == fresh.view().bundle_templates().end()) {
    return false;
  }
  city::wire::BundleTemplate fresh_spacing_edited = fresh_template_it->second;
  fresh_spacing_edited.default_spacing_m = spacing_edited.default_spacing_m;
  if (!fresh.UpdateBundleTemplate(fresh_spacing_edited).ok) {
    return false;
  }
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(poly3_req(fresh));
  if (!fresh_generated.ok ||
      !same_route_bundle_signatures(route_bundle_signatures_for_ids(state, target_edge_bundle_ids),
                                    route_bundle_signatures(fresh, city::wire::BundleKind::kLowVoltage))) {
    return false;
  }

  const std::vector<SpanOutputSnapshot> target_before_reject =
      snapshot_span_outputs(state, generated.value.generated_span_ids);
  const std::uint64_t before_version = state.view().bundle_templates().at(template_id).version;
  city::wire::BundleTemplate invalid = state.view().bundle_templates().at(template_id);
  invalid.default_layer = city::wire::SpanLayer::kUnknown;
  const auto rejected = state.UpdateBundleTemplate(invalid);
  return !rejected.ok && same_span_output_snapshots(target_before_reject, state) &&
         same_span_output_snapshots(outside_before, state) &&
         state.view().bundle_templates().at(template_id).version == before_version;
}

bool C745_legacy_wrap_family_is_absent() {
  const std::vector<std::filesystem::path> sources = {
      repo_root() / "domains/wire/include/city/wire/entities.hpp",
      repo_root() / "domains/wire/include/city/wire/core_runtime_types.hpp",
      repo_root() / "domains/wire/src/generation/backbone/curve_parts.cpp",
      repo_root() / "domains/wire/src/geometry/detail_curve_postprocess.cpp",
      repo_root() / "domains/wire/src/geometry/detail_curve_postprocess.hpp",
      repo_root() / "domains/wire/src/state/template/update.cpp",
      repo_root() / "domains/wire/src/state/template/registry.cpp",
      repo_root() / "domains/wire/src/state/persistence.cpp",
      repo_root() / "domains/wire/src/state/inspection.cpp",
      repo_root() / "domains/wire/src/validation/validator.cpp",
      repo_root() / "web/wasm/bindings.cpp",
      repo_root() / "docs/wire/architecture.md",
      repo_root() / "docs/wire/cable_instance_section.md",
  };
  for (const std::filesystem::path& path : sources) {
    std::string source{};
    if (!file_text(path, &source) || contains_text(source, "CableSectionProfile") || contains_text(source, "kWrap") ||
        contains_text(source, "wrap_radius_m") || contains_text(source, "sample_wrap_helix_points")) {
      return false;
    }
  }
  const std::filesystem::path web_source = repo_root() / "web/src";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(web_source)) {
    if (!entry.is_regular_file()) continue;
    if (entry.path().extension() != ".ts" && entry.path().extension() != ".svelte") continue;
    std::string source{};
    if (!file_text(entry.path(), &source) || contains_text(source, "wrapRadius") ||
        contains_text(source, "wrapTurnsPerMeter") || contains_text(source, "wrapPhase") ||
        contains_text(source, "wrapDirection") || contains_text(source, "Wrap (carrier)")) {
      return false;
    }
  }
  std::string workflow{};
  if (!file_text(repo_root() / "domains/wire/include/city/wire/workflow_types.hpp", &workflow)) {
    return false;
  }
  const std::size_t supplemental_begin = workflow.find("struct CableSupplementalPathTemplate");
  const std::size_t supplemental_end = workflow.find("};", supplemental_begin);
  return supplemental_begin != std::string::npos && supplemental_end != std::string::npos &&
         workflow.substr(supplemental_begin, supplemental_end - supplemental_begin).find("kCoiledCable") ==
             std::string::npos;
}

bool C758_span_visual_assembly_emits_support_and_helix() {
  city::wire::CoreState state;
  const auto id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  city::wire::BundleTemplate tpl = state.view().bundle_templates().at(id);
  tpl.fixed_count = 2;
  tpl.default_count = 2;
  tpl.support_wire_pole_band_id = 100;
  tpl.span_visual_assembly.support_path_enabled = false;
  tpl.span_visual_assembly.helix_enabled = true;
  tpl.span_visual_assembly.helix_turns_per_meter = 1.0;
  tpl.span_visual_assembly.member_wander_ratio = 0.5;
  tpl.span_visual_assembly.member_wander_wavelength_m = 3.0;
  tpl.span_visual_assembly.member_twist_turns_per_meter = 0.2;
  if (state.UpdateBundleTemplate(tpl).ok) return false;
  tpl.span_visual_assembly.support_path_enabled = true;
  tpl.span_visual_assembly.helix_samples_per_turn = 12;
  tpl.span_visual_assembly.endpoint_trim_m = 0.25;
  tpl.span_visual_assembly.helix_radius_m = 1e-6;
  if (state.UpdateBundleTemplate(tpl).ok) return false;
  tpl.span_visual_assembly.helix_radius_m = 0.0;
  if (!state.UpdateBundleTemplate(tpl).ok) return false;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) return false;
  const std::size_t saved_edges_before = state.view().backbone().edges.size();
  city::wire::BundleTemplate edited = state.view().bundle_templates().at(id);
  edited.span_visual_assembly.helix_clearance_m = 0.08;
  const auto reshaped = state.UpdateBundleTemplate(edited);
  if (!reshaped.ok || state.view().last_update_timing().kind != city::wire::UpdateKind::kReshape ||
      state.view().backbone().edges.size() != saved_edges_before) return false;
  std::size_t support_count = 0;
  std::size_t helix_count = 0;
  std::size_t member_count = 0;
  bool support_is_curved = true;
  bool helix_is_trimmed = true;
  bool members_below_support = true;
  std::unordered_map<city::wire::ObjectId, const city::wire::VisualCurvePart*> supports{};
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.has_section_key) {
      ++member_count;
    } else if (part.kind == city::wire::VisualCurvePartKind::kSupplemental) {
      if (part.supplemental_kind == city::wire::VisualSupplementalKind::kSupportPath) {
        ++support_count;
        supports.emplace(part.source_span_id, &part);
        support_is_curved = support_is_curved && part.samples.size() > 2;
      } else if (part.supplemental_kind == city::wire::VisualSupplementalKind::kHelix) {
        ++helix_count;
        const auto support = supports.find(part.source_span_id);
        helix_is_trimmed = helix_is_trimmed && support != supports.end() && part.samples.size() >= 2 &&
            !same_vec3(part.boundary_a, support->second->boundary_a) &&
            !same_vec3(part.boundary_b, support->second->boundary_b);
      }
    }
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody || !part.has_section_key || part.samples.empty()) continue;
    const auto support = supports.find(part.section_key.logical_span_id);
    if (support == supports.end() || support->second->samples.empty()) return false;
    members_below_support = members_below_support &&
        part.samples[part.samples.size() / 2].z < support->second->samples[support->second->samples.size() / 2].z;
  }
  city::wire::CoreState fresh;
  city::wire::BundleTemplate fresh_template = fresh.view().bundle_templates().at(id);
  fresh_template.fixed_count = 2;
  fresh_template.default_count = 2;
  fresh_template.support_wire_pole_band_id = 100;
  fresh_template.span_visual_assembly.support_path_enabled = true;
  fresh_template.span_visual_assembly.helix_enabled = true;
  fresh_template.span_visual_assembly.helix_turns_per_meter = 1.0;
  fresh_template.span_visual_assembly.member_wander_ratio = 0.5;
  fresh_template.span_visual_assembly.member_wander_wavelength_m = 3.0;
  fresh_template.span_visual_assembly.member_twist_turns_per_meter = 0.2;
  fresh_template.span_visual_assembly.helix_samples_per_turn = 12;
  fresh_template.span_visual_assembly.endpoint_trim_m = 0.25;
  fresh_template.span_visual_assembly.helix_clearance_m = 0.08;
  const auto fresh_update = fresh.UpdateBundleTemplate(fresh_template);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  const bool matches_fresh = fresh_update.ok && fresh_generated.ok &&
      same_visual_curve_samples(state.view().visual_curve_parts(), fresh.view().visual_curve_parts());
  WIRE_TEST_EXPECT(support_count == generated.value.generated_span_ids.size(), "support count mismatch");
  WIRE_TEST_EXPECT(helix_count == generated.value.generated_span_ids.size(), "helix count mismatch");
  WIRE_TEST_EXPECT(support_is_curved && helix_is_trimmed && members_below_support,
                   "span visual assembly geometry contract failed");
  WIRE_TEST_EXPECT(member_count >= generated.value.generated_span_ids.size(), "member count mismatch");
  WIRE_TEST_EXPECT(state.view().visual_curve_parts().stats.curve_builds == member_count + support_count,
                   "curve build count mismatch");
  WIRE_TEST_EXPECT(matches_fresh, "span visual assembly fresh differential failed");
  return true;
}

bool C759_span_visual_assembly_has_one_geometry_owner() {
  std::string assembly{};
  std::string curve_parts{};
  if (!file_text(repo_root() / "domains/wire/src/generation/backbone/span_visual_assembly.cpp", &assembly) ||
      !file_text(repo_root() / "domains/wire/src/generation/backbone/curve_parts.cpp", &curve_parts)) {
    return false;
  }
  return contains_text(assembly, "make_helix_part") && contains_text(assembly, "contain_members") &&
         contains_text(assembly, "make_support_path") &&
         contains_text(assembly, "make_primary_curve_between") &&
         contains_text(assembly, "apply_member_twist") && contains_text(assembly, "apply_span_visual_assemblies") &&
         !contains_text(assembly, "find_nearest") && !contains_text(assembly, "SavedBackbone") &&
         !contains_text(assembly, "AddSpan") && !contains_text(assembly, "AddBundle") &&
         !contains_text(assembly, "AddPort") && contains_text(curve_parts, "apply_span_visual_assemblies(state, assembly_endpoints, &out)") &&
         !contains_text(curve_parts, "make_primary_curve_between") &&
         !contains_text(curve_parts, "member_wander_ratio") && !contains_text(curve_parts, "helix_turns_per_meter");
}

bool C850_backbone_fixed_count_increase_scopes_independent_bundle_components() {
  auto increase_to_four = [](city::wire::CoreState* state) {
    city::wire::BundleTemplate edited = state->view().bundle_templates().at(
        city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
    edited.fixed_count = 4;
    return state->UpdateBundleTemplate(edited);
  };

  city::wire::CoreState alone{};
  WIRE_TEST_EXPECT(set_low_voltage_count_before_generation(alone, 3),
                   "failed to set the single-component fixed count to three");
  const FixedCountBundleComponent alone_a = generate_fixed_count_component(alone, 0.0);
  WIRE_TEST_EXPECT(fixed_count_component_relations_are_internal(alone, alone_a, 3),
                   "single-component fixture is incomplete before increase");
  const auto alone_update = increase_to_four(&alone);
  WIRE_TEST_EXPECT(alone_update.ok && alone_update.value,
                   alone_update.error.empty() ? "single-component 3 to 4 update failed"
                                              : alone_update.error);
  const auto alone_signature = route_bundle_signatures_for_ids(alone, alone_a.edge_bundle_ids);

  city::wire::CoreState together{};
  WIRE_TEST_EXPECT(set_low_voltage_count_before_generation(together, 3),
                   "failed to set the two-component fixed count to three");
  const FixedCountBundleComponent together_a = generate_fixed_count_component(together, 0.0);
  const FixedCountBundleComponent together_b = generate_fixed_count_component(together, 30.0);
  WIRE_TEST_EXPECT(together_a.bundle_id != together_b.bundle_id,
                   "independent placements were merged before fixed-count update");
  const auto together_update = increase_to_four(&together);
  WIRE_TEST_EXPECT(together_update.ok && together_update.value,
                   together_update.error.empty() ? "two-component 3 to 4 update failed"
                                                 : together_update.error);
  WIRE_TEST_EXPECT(
      fixed_count_component_relations_are_internal(together, together_a, 4) &&
          fixed_count_component_relations_are_internal(together, together_b, 4),
      "3 to 4 update mixed PortBinding, SpanBinding, or continuity across Bundle components");
  WIRE_TEST_EXPECT(
      same_route_bundle_signatures(alone_signature,
                                   route_bundle_signatures_for_ids(together, together_a.edge_bundle_ids)),
      "Bundle A semantic regenerate result changed when unrelated Bundle B existed");

  city::wire::CoreState reversed{};
  WIRE_TEST_EXPECT(set_low_voltage_count_before_generation(reversed, 3),
                   "failed to set reversed-order fixed count to three");
  const FixedCountBundleComponent reversed_b = generate_fixed_count_component(reversed, 30.0);
  const FixedCountBundleComponent reversed_a = generate_fixed_count_component(reversed, 0.0);
  const auto reversed_update = increase_to_four(&reversed);
  WIRE_TEST_EXPECT(reversed_update.ok && reversed_update.value,
                   reversed_update.error.empty() ? "reversed-order 3 to 4 update failed"
                                                 : reversed_update.error);
  WIRE_TEST_EXPECT(
      fixed_count_component_relations_are_internal(reversed, reversed_a, 4) &&
          fixed_count_component_relations_are_internal(reversed, reversed_b, 4),
      "reversed generation order mixed component relations");
  WIRE_TEST_EXPECT(
      same_route_bundle_signatures(
          route_bundle_signatures_for_ids(together, together_a.edge_bundle_ids),
          route_bundle_signatures_for_ids(reversed, reversed_a.edge_bundle_ids)) &&
          same_route_bundle_signatures(
              route_bundle_signatures_for_ids(together, together_b.edge_bundle_ids),
              route_bundle_signatures_for_ids(reversed, reversed_b.edge_bundle_ids)),
      "fixed-count regenerate result depends on independent Bundle generation order");
  return true;
}

bool C851_backbone_fixed_count_decrease_retires_each_bundle_component_lanes() {
  city::wire::CoreState state{};
  WIRE_TEST_EXPECT(set_low_voltage_count_before_generation(state, 4),
                   "failed to set the two-component fixed count to four");
  const FixedCountBundleComponent component_a = generate_fixed_count_component(state, 0.0);
  const FixedCountBundleComponent component_b = generate_fixed_count_component(state, 30.0);
  WIRE_TEST_EXPECT(
      fixed_count_component_relations_are_internal(state, component_a, 4) &&
          fixed_count_component_relations_are_internal(state, component_b, 4),
      "two-component fixture is incomplete before decrease");
  const std::vector<city::wire::ObjectId> retired_a_spans = component_lane_span_ids(state, component_a, 3);
  const std::vector<city::wire::ObjectId> retired_b_spans = component_lane_span_ids(state, component_b, 3);
  const std::vector<city::wire::ObjectId> retired_a_ports = component_lane_port_ids(state, component_a, 3);
  const std::vector<city::wire::ObjectId> retired_b_ports = component_lane_port_ids(state, component_b, 3);
  std::vector<city::wire::ObjectId> surviving_a_spans{};
  std::vector<city::wire::ObjectId> surviving_b_spans{};
  std::vector<city::wire::ObjectId> surviving_a_ports{};
  std::vector<city::wire::ObjectId> surviving_b_ports{};
  for (std::size_t lane = 0; lane < 3; ++lane) {
    const auto a_spans = component_lane_span_ids(state, component_a, lane);
    const auto b_spans = component_lane_span_ids(state, component_b, lane);
    const auto a_ports = component_lane_port_ids(state, component_a, lane);
    const auto b_ports = component_lane_port_ids(state, component_b, lane);
    surviving_a_spans.insert(surviving_a_spans.end(), a_spans.begin(), a_spans.end());
    surviving_b_spans.insert(surviving_b_spans.end(), b_spans.begin(), b_spans.end());
    surviving_a_ports.insert(surviving_a_ports.end(), a_ports.begin(), a_ports.end());
    surviving_b_ports.insert(surviving_b_ports.end(), b_ports.begin(), b_ports.end());
  }
  WIRE_TEST_EXPECT(retired_a_spans.size() == 2 && retired_b_spans.size() == 2 &&
                       retired_a_ports.size() == 4 && retired_b_ports.size() == 4,
                   "lane three retirement identities are incomplete");
  city::wire::BundleTemplate edited = state.view().bundle_templates().at(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  edited.fixed_count = 3;
  const auto updated = state.UpdateBundleTemplate(edited);
  WIRE_TEST_EXPECT(updated.ok && updated.value,
                   updated.error.empty() ? "two-component 4 to 3 update failed" : updated.error);
  WIRE_TEST_EXPECT(
      fixed_count_component_relations_are_internal(state, component_a, 3) &&
          fixed_count_component_relations_are_internal(state, component_b, 3),
      "4 to 3 update damaged surviving component-local relations");
  for (city::wire::ObjectId span_id : surviving_a_spans) {
    WIRE_TEST_EXPECT(state.view().spans().find(span_id) != nullptr,
                     "Bundle A surviving Span was deleted by decrease");
  }
  for (city::wire::ObjectId span_id : surviving_b_spans) {
    WIRE_TEST_EXPECT(state.view().spans().find(span_id) != nullptr,
                     "Bundle B surviving Span was deleted by decrease");
  }
  for (city::wire::ObjectId port_id : surviving_a_ports) {
    WIRE_TEST_EXPECT(state.view().ports().find(port_id) != nullptr,
                     "Bundle A surviving Port was deleted by decrease");
  }
  for (city::wire::ObjectId port_id : surviving_b_ports) {
    WIRE_TEST_EXPECT(state.view().ports().find(port_id) != nullptr,
                     "Bundle B surviving Port was deleted by decrease");
  }
  for (city::wire::ObjectId span_id : retired_a_spans) {
    WIRE_TEST_EXPECT(state.view().spans().find(span_id) == nullptr,
                     "Bundle A retired Span survived decrease");
  }
  for (city::wire::ObjectId span_id : retired_b_spans) {
    WIRE_TEST_EXPECT(state.view().spans().find(span_id) == nullptr,
                     "Bundle B retired Span survived decrease");
  }
  for (city::wire::ObjectId port_id : retired_a_ports) {
    WIRE_TEST_EXPECT(state.view().ports().find(port_id) == nullptr,
                     "Bundle A retired Port survived decrease");
  }
  for (city::wire::ObjectId port_id : retired_b_ports) {
    WIRE_TEST_EXPECT(state.view().ports().find(port_id) == nullptr,
                     "Bundle B retired Port survived decrease");
  }
  return true;
}

bool C866_backbone_range_bundle_count_reconciles_exact_identity() {
  auto prepare = [](city::wire::CoreState* state, bool with_unrelated,
                    FixedCountBundleComponent* component_a,
                    FixedCountBundleComponent* component_b) {
    if (state == nullptr || component_a == nullptr || component_b == nullptr ||
        !set_low_voltage_range_before_generation(*state, 3)) return false;
    *component_a = generate_fixed_count_component(*state, 0.0);
    if (with_unrelated) *component_b = generate_fixed_count_component(*state, 30.0);
    return fixed_count_component_relations_are_internal(*state, *component_a, 3) &&
           (!with_unrelated || fixed_count_component_relations_are_internal(*state, *component_b, 3));
  };

  city::wire::CoreState alone{};
  FixedCountBundleComponent alone_a{};
  FixedCountBundleComponent unused{};
  WIRE_TEST_EXPECT_PRESENCE(prepare(&alone, false, &alone_a, &unused),
                            "single range Bundle fixture is incomplete");
  const auto alone_update = alone.UpdateBackboneBundleConductorCount(alone_a.bundle_id, 4);
  WIRE_TEST_EXPECT_PRESENCE(alone_update.ok && alone_update.value,
                            alone_update.error.empty() ? "single range Bundle 3 to 4 failed"
                                                       : alone_update.error);
  const auto alone_signature = route_bundle_signatures_for_ids(alone, alone_a.edge_bundle_ids);

  city::wire::CoreState together{};
  FixedCountBundleComponent together_a{};
  FixedCountBundleComponent together_b{};
  WIRE_TEST_EXPECT_PRESENCE(prepare(&together, true, &together_a, &together_b),
                            "two range Bundle fixture is incomplete");
  const auto a_bundle_id = together_a.bundle_id;
  const auto b_bundle_id = together_b.bundle_id;
  std::array<std::vector<city::wire::ObjectId>, 3> a_span_ids{};
  std::array<std::vector<city::wire::ObjectId>, 3> a_port_ids{};
  std::array<std::vector<city::wire::ObjectId>, 3> b_span_ids{};
  std::array<std::vector<city::wire::ObjectId>, 3> b_port_ids{};
  for (std::size_t lane = 0; lane < 3; ++lane) {
    a_span_ids[lane] = component_lane_span_ids(together, together_a, lane);
    a_port_ids[lane] = component_lane_port_ids(together, together_a, lane);
    b_span_ids[lane] = component_lane_span_ids(together, together_b, lane);
    b_port_ids[lane] = component_lane_port_ids(together, together_b, lane);
  }
  const auto b_signature_before = route_bundle_signatures_for_ids(together, together_b.edge_bundle_ids);

  const auto increased = together.UpdateBackboneBundleConductorCount(together_a.bundle_id, 4);
  WIRE_TEST_EXPECT_PRESENCE(increased.ok && increased.value,
                            increased.error.empty() ? "range Bundle A 3 to 4 failed" : increased.error);
  WIRE_TEST_EXPECT_ANCHOR(
      together_a.bundle_id == a_bundle_id && together_b.bundle_id == b_bundle_id &&
          together.view().bundles().find(a_bundle_id) != nullptr &&
          together.view().bundles().find(b_bundle_id) != nullptr,
      "range Bundle identity changed during exact count increase");
  WIRE_TEST_EXPECT_ANCHOR(
      fixed_count_component_relations_are_internal(together, together_a, 4) &&
          fixed_count_component_relations_are_internal(together, together_b, 3),
      "range count increase escaped the exact Bundle component");
  for (std::size_t lane = 0; lane < 3; ++lane) {
    WIRE_TEST_EXPECT_ANCHOR(
        component_lane_span_ids(together, together_a, lane) == a_span_ids[lane] &&
            component_lane_port_ids(together, together_a, lane) == a_port_ids[lane],
        "range count increase replaced a surviving Bundle A lane identity");
    WIRE_TEST_EXPECT_ANCHOR(
        component_lane_span_ids(together, together_b, lane) == b_span_ids[lane] &&
            component_lane_port_ids(together, together_b, lane) == b_port_ids[lane],
        "range count increase changed unrelated Bundle B identity");
  }
  WIRE_TEST_EXPECT_PRESENCE(
      component_lane_span_ids(together, together_a, 3).size() == 2 &&
          component_lane_port_ids(together, together_a, 3).size() == 4 &&
          component_lane_span_ids(together, together_b, 3).empty() &&
          component_lane_port_ids(together, together_b, 3).empty(),
      "range count increase did not add only Bundle A lane 3");
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      same_route_bundle_signatures(alone_signature,
                                   route_bundle_signatures_for_ids(together, together_a.edge_bundle_ids)) &&
          same_route_bundle_signatures(b_signature_before,
                                       route_bundle_signatures_for_ids(together, together_b.edge_bundle_ids)),
      "range Bundle A result depends on unrelated Bundle B");

  std::string saved{};
  WIRE_TEST_EXPECT_PRESENCE(together.SerializeAuthoritative(&saved).ok,
                            "range count increase state did not serialize");
  city::wire::CoreState loaded{};
  const auto loaded_result = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT_PRESENCE(loaded_result.ok, loaded_result.error);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      fixed_count_component_relations_are_internal(loaded, together_a, 4) &&
          fixed_count_component_relations_are_internal(loaded, together_b, 3),
      "range Bundle count or relations changed across save/load");

  const auto retired_spans = component_lane_span_ids(together, together_a, 3);
  const auto retired_ports = component_lane_port_ids(together, together_a, 3);
  const auto decreased = together.UpdateBackboneBundleConductorCount(together_a.bundle_id, 3);
  WIRE_TEST_EXPECT_PRESENCE(decreased.ok && decreased.value,
                            decreased.error.empty() ? "range Bundle A 4 to 3 failed" : decreased.error);
  WIRE_TEST_EXPECT_ANCHOR(
      fixed_count_component_relations_are_internal(together, together_a, 3) &&
          fixed_count_component_relations_are_internal(together, together_b, 3),
      "range count decrease damaged a surviving or unrelated component");
  for (std::size_t lane = 0; lane < 3; ++lane) {
    WIRE_TEST_EXPECT_ANCHOR(
        component_lane_span_ids(together, together_a, lane) == a_span_ids[lane] &&
            component_lane_port_ids(together, together_a, lane) == a_port_ids[lane] &&
            component_lane_span_ids(together, together_b, lane) == b_span_ids[lane] &&
            component_lane_port_ids(together, together_b, lane) == b_port_ids[lane],
        "range count decrease changed surviving lane identity");
  }
  WIRE_TEST_EXPECT_ANCHOR(no_binding_references(together, retired_spans, retired_ports),
                          "range count decrease left stale lane 3 bindings");
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(together);
  return true;
}

bool C867_backbone_range_bundle_count_retirement_conflicts_are_atomic() {
  auto make_four_lane = [](city::wire::CoreState* state,
                           FixedCountBundleComponent* component) {
    if (state == nullptr || component == nullptr ||
        !set_low_voltage_range_before_generation(*state, 4)) return false;
    *component = generate_fixed_count_component(*state, 0.0);
    return fixed_count_component_relations_are_internal(*state, *component, 4);
  };

  city::wire::CoreState with_attachment{};
  FixedCountBundleComponent attachment_component{};
  WIRE_TEST_EXPECT_PRESENCE(make_four_lane(&with_attachment, &attachment_component),
                            "user Attachment conflict fixture is incomplete");
  const auto retiring_spans = component_lane_span_ids(with_attachment, attachment_component, 3);
  WIRE_TEST_EXPECT_PRESENCE(!retiring_spans.empty() &&
                                with_attachment.AddAttachment(retiring_spans.front(), 0.5).ok,
                            "failed to add user Attachment to retiring lane");
  std::string attachment_before{};
  WIRE_TEST_EXPECT_PRESENCE(with_attachment.SerializeAuthoritative(&attachment_before).ok,
                            "failed to serialize user Attachment fixture");
  const auto attachment_update =
      with_attachment.UpdateBackboneBundleConductorCount(attachment_component.bundle_id, 3);
  std::string attachment_after{};
  WIRE_TEST_EXPECT_PRESENCE(with_attachment.SerializeAuthoritative(&attachment_after).ok,
                            "failed to serialize rejected user Attachment state");
  WIRE_TEST_EXPECT_DIFFERENTIAL(!attachment_update.ok && attachment_before == attachment_after,
                                "retiring lane user Attachment was not rejected atomically");

  city::wire::CoreState with_manual_port{};
  FixedCountBundleComponent port_component{};
  WIRE_TEST_EXPECT_PRESENCE(make_four_lane(&with_manual_port, &port_component),
                            "manual Port conflict fixture is incomplete");
  const auto retiring_ports = component_lane_port_ids(with_manual_port, port_component, 3);
  const city::wire::Port* port = retiring_ports.empty()
                                     ? nullptr
                                     : with_manual_port.view().ports().find(retiring_ports.front());
  WIRE_TEST_EXPECT_PRESENCE(
      port != nullptr &&
          with_manual_port.SetPortWorldPositionManual(
              port->id, {port->world_position.x, port->world_position.y,
                         port->world_position.z + 0.05}).ok,
      "failed to mark retiring lane Port manual");
  std::string port_before{};
  WIRE_TEST_EXPECT_PRESENCE(with_manual_port.SerializeAuthoritative(&port_before).ok,
                            "failed to serialize manual Port fixture");
  const auto port_update =
      with_manual_port.UpdateBackboneBundleConductorCount(port_component.bundle_id, 3);
  std::string port_after{};
  WIRE_TEST_EXPECT_PRESENCE(with_manual_port.SerializeAuthoritative(&port_after).ok,
                            "failed to serialize rejected manual Port state");
  WIRE_TEST_EXPECT_DIFFERENTIAL(!port_update.ok && port_before == port_after,
                                "retiring lane manual Port was not rejected atomically");
  return true;
}

bool C868_backbone_bundle_count_policy_rejects_invalid_individual_change() {
  city::wire::CoreState state{};
  const FixedCountBundleComponent component = generate_fixed_count_component(state, 0.0);
  WIRE_TEST_EXPECT_PRESENCE(
      fixed_count_component_relations_are_internal(state, component, 1),
      "fixed Bundle rejection fixture is incomplete");
  std::string before{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&before).ok,
                            "failed to serialize fixed Bundle fixture");
  const auto updated = state.UpdateBackboneBundleConductorCount(component.bundle_id, 2);
  std::string after{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&after).ok,
                            "failed to serialize rejected fixed Bundle state");
  WIRE_TEST_EXPECT_DIFFERENTIAL(!updated.ok && before == after,
                                "fixed template individual Bundle count change was not rejected atomically");

  city::wire::CoreState range_state{};
  WIRE_TEST_EXPECT_PRESENCE(set_low_voltage_range_before_generation(range_state, 3),
                            "range count policy rejection fixture setup failed");
  const FixedCountBundleComponent range_component = generate_fixed_count_component(range_state, 0.0);
  std::string range_before{};
  WIRE_TEST_EXPECT_PRESENCE(range_state.SerializeAuthoritative(&range_before).ok,
                            "failed to serialize range count policy fixture");
  const auto range_updated =
      range_state.UpdateBackboneBundleConductorCount(range_component.bundle_id, 7);
  std::string range_after{};
  WIRE_TEST_EXPECT_PRESENCE(range_state.SerializeAuthoritative(&range_after).ok,
                            "failed to serialize rejected range count state");
  WIRE_TEST_EXPECT_DIFFERENTIAL(!range_updated.ok && range_before == range_after,
                                "out-of-range individual Bundle count was not rejected atomically");
  return true;
}

namespace {

struct ExactBundleRetirementFixture {
  city::wire::ObjectId bundle_a = city::wire::kInvalidObjectId;
  city::wire::ObjectId bundle_b = city::wire::kInvalidObjectId;
  std::vector<city::wire::ObjectId> edge_bundles_a{};
  std::vector<city::wire::ObjectId> edge_bundles_b{};
};

std::vector<city::wire::ObjectId> exact_edge_bundles(
    const city::wire::CoreState& state, city::wire::ObjectId bundle_id) {
  std::vector<city::wire::ObjectId> out{};
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle :
       state.view().backbone().edge_bundles) {
    if (edge_bundle.bundle_id == bundle_id) out.push_back(edge_bundle.edge_bundle_id);
  }
  std::sort(out.begin(), out.end());
  return out;
}

city::wire::BackboneSpec exact_bundle_route_request(
    city::wire::CoreState& state, const std::vector<city::wire::Vec3d>& points) {
  city::wire::BackboneSpec request = line_req(state);
  request.path.polyline = points;
  city::wire::BackboneBundleSpec a = request.bundles.front();
  a.placement_key = 86901;
  a.placement_explicit = true;
  a.height_m = 7.15;
  a.lateral_m = -0.22;
  a.spacing_m = 0.20;
  city::wire::BackboneBundleSpec b = a;
  b.placement_key = 86902;
  b.height_m = 7.55;
  b.lateral_m = -0.42;
  request.bundles = {a, b};
  return request;
}

bool generate_exact_bundle_route(city::wire::CoreState* state,
                                 const std::vector<city::wire::Vec3d>& points,
                                 ExactBundleRetirementFixture* fixture,
                                 city::wire::GenerateBundleFromPathResult* generated = nullptr) {
  if (state == nullptr || fixture == nullptr) return false;
  const auto result = state->GenerateFromBackboneSpec(
      exact_bundle_route_request(*state, points));
  if (!result.ok || result.value.bundle_ids.size() != 2) return false;
  for (city::wire::ObjectId bundle_id : result.value.bundle_ids) {
    const city::wire::Bundle* bundle = state->view().bundles().find(bundle_id);
    if (bundle == nullptr) return false;
    if (bundle->placement_key == 86901) fixture->bundle_a = bundle_id;
    if (bundle->placement_key == 86902) fixture->bundle_b = bundle_id;
  }
  fixture->edge_bundles_a = exact_edge_bundles(*state, fixture->bundle_a);
  fixture->edge_bundles_b = exact_edge_bundles(*state, fixture->bundle_b);
  if (generated != nullptr) *generated = result.value;
  return fixture->bundle_a != city::wire::kInvalidObjectId &&
         fixture->bundle_b != city::wire::kInvalidObjectId &&
         !fixture->edge_bundles_a.empty() &&
         fixture->edge_bundles_a.size() == fixture->edge_bundles_b.size();
}

std::vector<EdgeBundleIdentitySnapshot> exact_bundle_identity_snapshot(
    const city::wire::CoreState& state,
    const std::vector<city::wire::ObjectId>& edge_bundle_ids) {
  std::vector<EdgeBundleIdentitySnapshot> out{};
  for (city::wire::ObjectId edge_bundle_id : edge_bundle_ids) {
    out.push_back(edge_bundle_identity_snapshot(state, edge_bundle_id));
  }
  return out;
}

bool same_exact_bundle_identity_snapshot(
    const std::vector<EdgeBundleIdentitySnapshot>& a,
    const std::vector<EdgeBundleIdentitySnapshot>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!same_edge_bundle_identity_snapshot(a[i], b[i])) return false;
  }
  return true;
}

bool exact_bundle_is_fully_retired(
    const city::wire::CoreState& state, city::wire::ObjectId bundle_id,
    const std::vector<city::wire::ObjectId>& edge_bundle_ids,
    const std::vector<city::wire::ObjectId>& span_ids,
    const std::vector<city::wire::ObjectId>& port_ids,
    const std::vector<city::wire::ObjectId>& attachment_ids) {
  if (state.view().bundles().find(bundle_id) != nullptr ||
      state.view().backbone_index().bundle_edge.contains(bundle_id) ||
      state.view().relation_index().spans_by_bundle.contains(bundle_id)) return false;
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle :
       state.view().backbone().edge_bundles) {
    if (edge_bundle.bundle_id == bundle_id ||
        std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  edge_bundle.edge_bundle_id) != edge_bundle_ids.end()) return false;
  }
  for (const city::wire::SavedBackboneSpanBinding& binding :
       state.view().backbone().span_bindings) {
    if (std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  binding.edge_bundle_id) != edge_bundle_ids.end() ||
        std::find(span_ids.begin(), span_ids.end(), binding.span_id) != span_ids.end()) return false;
  }
  for (const city::wire::SavedBackbonePortBinding& binding :
       state.view().backbone().port_bindings) {
    if (std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  binding.edge_bundle_id) != edge_bundle_ids.end()) return false;
  }
  for (const city::wire::SavedBackboneRowContinuity& continuity :
       state.view().backbone().row_continuities) {
    if (std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  continuity.a.edge_bundle_id) != edge_bundle_ids.end() ||
        std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  continuity.b.edge_bundle_id) != edge_bundle_ids.end()) return false;
  }
  for (city::wire::ObjectId span_id : span_ids) {
    if (state.view().spans().find(span_id) != nullptr ||
        state.find_curve_cache(span_id) != nullptr ||
        state.find_bounds_cache(span_id) != nullptr ||
        state.find_span_visual_cache(span_id) != nullptr ||
        state.find_span_render_cache(span_id) != nullptr) return false;
  }
  for (city::wire::ObjectId port_id : port_ids) {
    if (state.view().ports().find(port_id) != nullptr) return false;
  }
  for (city::wire::ObjectId attachment_id : attachment_ids) {
    if (state.view().attachments().find(attachment_id) != nullptr) return false;
  }
  return std::none_of(
      state.view().visual_curve_parts().parts.begin(),
      state.view().visual_curve_parts().parts.end(),
      [&](const city::wire::VisualCurvePart& part) {
        return part.source_bundle_id == bundle_id ||
               std::find(span_ids.begin(), span_ids.end(), part.source_span_id) != span_ids.end();
      });
}

void collect_exact_bundle_dependents(
    const city::wire::CoreState& state,
    const std::vector<city::wire::ObjectId>& edge_bundle_ids,
    std::vector<city::wire::ObjectId>* span_ids,
    std::vector<city::wire::ObjectId>* port_ids,
    std::vector<city::wire::ObjectId>* attachment_ids) {
  for (const city::wire::SavedBackboneSpanBinding& binding :
       state.view().backbone().span_bindings) {
    if (std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  binding.edge_bundle_id) != edge_bundle_ids.end()) {
      span_ids->push_back(binding.span_id);
    }
  }
  for (const city::wire::SavedBackbonePortBinding& binding :
       state.view().backbone().port_bindings) {
    if (std::find(edge_bundle_ids.begin(), edge_bundle_ids.end(),
                  binding.edge_bundle_id) != edge_bundle_ids.end()) {
      port_ids->push_back(binding.port_id);
    }
  }
  std::sort(span_ids->begin(), span_ids->end());
  span_ids->erase(std::unique(span_ids->begin(), span_ids->end()), span_ids->end());
  std::sort(port_ids->begin(), port_ids->end());
  port_ids->erase(std::unique(port_ids->begin(), port_ids->end()), port_ids->end());
  for (city::wire::ObjectId span_id : *span_ids) {
    const auto it = state.view().relation_index().attachments_by_span.find(span_id);
    if (it != state.view().relation_index().attachments_by_span.end()) {
      attachment_ids->insert(attachment_ids->end(), it->second.begin(), it->second.end());
    }
  }
}

struct SourceEdgeRetirementFixture {
  city::wire::ObjectId source_bundle = city::wire::kInvalidObjectId;
  city::wire::ObjectId branch_bundle = city::wire::kInvalidObjectId;
};

bool generate_source_edge_retirement_fixture(
    city::wire::CoreState* state, SourceEdgeRetirementFixture* fixture) {
  if (state == nullptr || fixture == nullptr) return false;
  const auto source = state->GenerateFromBackboneSpec(line_req(*state));
  if (!source.ok || source.value.bundle_ids.size() != 1 ||
      source.value.generated_span_ids.empty() ||
      state->view().backbone().edges.size() != 1) return false;
  fixture->source_bundle = source.value.bundle_ids.front();
  const city::wire::SavedBackboneEdge source_edge =
      state->view().backbone().edges.front();
  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = source_edge.node_a;
  pick.segment_node_b_id = source_edge.node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 0.0};
  city::wire::ResolveBranchPickOptions options{};
  options.selected_bundle_template_ids = {
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  const auto resolved = state->ResolveBranchPick(pick, options);
  if (!resolved.ok ||
      resolved.value.resolved_node_id == city::wire::kInvalidObjectId) return false;
  city::wire::BackboneSpec branch = line_req(*state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto generated = state->GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.bundle_ids.size() != 1) return false;
  fixture->branch_bundle = generated.value.bundle_ids.front();
  return fixture->branch_bundle != fixture->source_bundle;
}

} // namespace

bool C869_backbone_exact_bundle_retirement_removes_only_target_topology() {
  city::wire::CoreState state{};
  ExactBundleRetirementFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_exact_bundle_route(
          &state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}},
          &fixture),
      "two exact Bundle route fixture is incomplete");
  std::vector<city::wire::ObjectId> spans_a{};
  std::vector<city::wire::ObjectId> ports_a{};
  std::vector<city::wire::ObjectId> attachments_a{};
  collect_exact_bundle_dependents(state, fixture.edge_bundles_a,
                                  &spans_a, &ports_a, &attachments_a);
  const auto b_identity_before =
      exact_bundle_identity_snapshot(state, fixture.edge_bundles_b);
  const auto b_curves_before =
      route_bundle_signatures_for_ids(state, fixture.edge_bundles_b);

  const auto retired = state.RetireBackboneBundle(fixture.bundle_a);
  WIRE_TEST_EXPECT_PRESENCE(retired.ok && retired.value,
                            retired.error.empty() ? "exact Bundle retirement failed"
                                                  : retired.error);
  WIRE_TEST_EXPECT_ANCHOR(
      exact_bundle_is_fully_retired(state, fixture.bundle_a,
                                    fixture.edge_bundles_a, spans_a, ports_a,
                                    attachments_a),
      "exact Bundle retirement left authoritative or derived topology");
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      state.view().bundles().find(fixture.bundle_b) != nullptr &&
          same_exact_bundle_identity_snapshot(
              b_identity_before,
              exact_bundle_identity_snapshot(state, fixture.edge_bundles_b)) &&
          same_route_bundle_signatures(
              b_curves_before,
              route_bundle_signatures_for_ids(state, fixture.edge_bundles_b)),
      "exact Bundle retirement changed the peer Bundle on the same route");
  std::string saved{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&saved).ok,
                            "retired state serialization failed");
  city::wire::CoreState loaded{};
  const auto loaded_result = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT_PRESENCE(loaded_result.ok, loaded_result.error);
  std::string resaved{};
  WIRE_TEST_EXPECT_ANCHOR(
      loaded.SerializeAuthoritative(&resaved).ok && saved == resaved &&
          exact_bundle_is_fully_retired(
              loaded, fixture.bundle_a, fixture.edge_bundles_a,
              spans_a, ports_a, attachments_a),
      "exact Bundle retirement did not survive save-load");
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(loaded);

  city::wire::CoreState sole{};
  const auto sole_generated = sole.GenerateFromBackboneSpec(poly3_req(sole));
  WIRE_TEST_EXPECT_PRESENCE(
      sole_generated.ok && sole_generated.value.bundle_ids.size() == 1,
      "sole Bundle retirement fixture is incomplete");
  const city::wire::ObjectId sole_bundle = sole_generated.value.bundle_ids.front();
  const auto sole_edge_bundles = exact_edge_bundles(sole, sole_bundle);
  std::vector<city::wire::ObjectId> sole_spans{};
  std::vector<city::wire::ObjectId> sole_ports{};
  std::vector<city::wire::ObjectId> sole_attachments{};
  collect_exact_bundle_dependents(sole, sole_edge_bundles, &sole_spans,
                                  &sole_ports, &sole_attachments);
  const auto sole_retired = sole.RetireBackboneBundle(sole_bundle);
  WIRE_TEST_EXPECT_PRESENCE(sole_retired.ok && sole_retired.value,
                            sole_retired.error);
  WIRE_TEST_EXPECT_ANCHOR(
      sole.view().bundles().find(sole_bundle) == nullptr &&
          sole.view().backbone().edge_bundles.empty() &&
          sole.view().backbone().edges.empty() &&
          sole.view().spans().empty() &&
          exact_bundle_is_fully_retired(
              sole, sole_bundle, sole_edge_bundles, sole_spans, sole_ports,
              sole_attachments),
      "sole Bundle retirement left orphan saved edges or entities: bundles=" +
          std::to_string(sole.view().bundles().size()) + " edge_bundles=" +
          std::to_string(sole.view().backbone().edge_bundles.size()) + " edges=" +
          std::to_string(sole.view().backbone().edges.size()) + " spans=" +
          std::to_string(sole.view().spans().size()));
  std::string sole_saved{};
  city::wire::CoreState sole_loaded{};
  WIRE_TEST_EXPECT_ANCHOR(
      sole.SerializeAuthoritative(&sole_saved).ok &&
          sole_loaded.DeserializeAuthoritative(sole_saved).ok &&
          sole_loaded.view().backbone().edges.empty(),
      "sole Bundle retirement did not load without orphan saved edges");
  return true;
}

bool C870_backbone_exact_bundle_retirement_covers_cross_and_sharp_outputs() {
  city::wire::CoreState state{};
  ExactBundleRetirementFixture fixture{};
  city::wire::GenerateBundleFromPathResult base{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_exact_bundle_route(
          &state,
          {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0},
           {5.0, 8.660254037844386, 0.0}},
          &fixture, &base) && base.generated_pole_ids.size() == 3,
      "sharp exact Bundle fixture is incomplete");
  const city::wire::ObjectId junction_pole_id = base.generated_pole_ids[1];
  const city::wire::Pole* junction_pole =
      state.view().poles().find(junction_pole_id);
  WIRE_TEST_EXPECT_PRESENCE(junction_pole != nullptr,
                            "sharp fixture junction Pole is missing");
  city::wire::BackboneSpec cross = exact_bundle_route_request(
      state, {{10.0, -8.0, 0.0}, junction_pole->world_transform.position,
              {18.0, 2.0, 0.0}});
  cross.path.node_specs = {pole_spec(1, junction_pole_id)};
  cross.bundles[0].existing_bundle_id = fixture.bundle_a;
  cross.bundles[1].existing_bundle_id = fixture.bundle_b;
  const auto extended = state.GenerateFromBackboneSpec(cross);
  WIRE_TEST_EXPECT_PRESENCE(extended.ok, extended.error);
  fixture.edge_bundles_a = exact_edge_bundles(state, fixture.bundle_a);
  fixture.edge_bundles_b = exact_edge_bundles(state, fixture.bundle_b);
  const std::size_t target_connection_parts = static_cast<std::size_t>(
      std::count_if(state.view().visual_curve_parts().parts.begin(),
                    state.view().visual_curve_parts().parts.end(),
                    [&](const city::wire::VisualCurvePart& part) {
                      return part.source_bundle_id == fixture.bundle_a &&
                             (part.kind == city::wire::VisualCurvePartKind::kNodePatch ||
                              part.kind == city::wire::VisualCurvePartKind::kJumper);
                    }));
  WIRE_TEST_EXPECT_ORACLE(
      fixture.edge_bundles_a.size() >= 4 &&
          fixture.edge_bundles_a.size() == fixture.edge_bundles_b.size() &&
          target_connection_parts > 0,
      "cross/sharp fixture has no exact continuity components or derived curve: A=" +
          std::to_string(fixture.edge_bundles_a.size()) + " B=" +
          std::to_string(fixture.edge_bundles_b.size()) + " parts=" +
          std::to_string(target_connection_parts));
  const auto b_before =
      exact_bundle_identity_snapshot(state, fixture.edge_bundles_b);
  const auto retired = state.RetireBackboneBundle(fixture.bundle_a);
  WIRE_TEST_EXPECT_PRESENCE(retired.ok && retired.value,
                            retired.error.empty() ? "cross exact Bundle retirement failed"
                                                  : retired.error);
  WIRE_TEST_EXPECT_ANCHOR(
      exact_edge_bundles(state, fixture.bundle_a).empty() &&
          std::none_of(state.view().visual_curve_parts().parts.begin(),
                       state.view().visual_curve_parts().parts.end(),
                       [&](const city::wire::VisualCurvePart& part) {
                         return part.source_bundle_id == fixture.bundle_a;
                       }),
      "cross/sharp exact Bundle retirement left continuity or derived output");
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      same_exact_bundle_identity_snapshot(
          b_before,
          exact_bundle_identity_snapshot(state, fixture.edge_bundles_b)),
      "cross/sharp exact Bundle retirement changed peer identity");
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);

  city::wire::CoreState branch_retirement{};
  SourceEdgeRetirementFixture branch_fixture{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_source_edge_retirement_fixture(&branch_retirement,
                                              &branch_fixture),
      "source-edge branch retirement fixture is incomplete");
  const auto branch_retired =
      branch_retirement.RetireBackboneBundle(branch_fixture.branch_bundle);
  WIRE_TEST_EXPECT_ANCHOR(
      branch_retired.ok && branch_retired.value &&
          branch_retirement.view().bundles().find(
              branch_fixture.source_bundle) != nullptr &&
          branch_retirement.view().bundles().find(
              branch_fixture.branch_bundle) == nullptr,
      "source-edge branch Bundle was blanket-rejected or damaged its source");
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(branch_retirement);

  city::wire::CoreState source_retirement{};
  SourceEdgeRetirementFixture source_fixture{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_source_edge_retirement_fixture(&source_retirement,
                                              &source_fixture),
      "source-edge dependency rejection fixture is incomplete");
  std::string source_before{};
  WIRE_TEST_EXPECT_PRESENCE(
      source_retirement.SerializeAuthoritative(&source_before).ok,
      "failed to serialize source-edge dependency fixture");
  const auto source_branch_span = std::find_if(
      source_retirement.view().spans().items().begin(),
      source_retirement.view().spans().items().end(),
      [&](const city::wire::Span& span) {
        return span.bundle_id == source_fixture.branch_bundle;
      });
  WIRE_TEST_EXPECT_PRESENCE(
      source_branch_span != source_retirement.view().spans().items().end(),
      "source-edge dependency Span is missing");
  city::wire::CoreStateTestHook::cache_state(source_retirement)
      .span_layout_cache.clear_layout(source_branch_span->id);
  const auto source_retired =
      source_retirement.RetireBackboneBundle(source_fixture.source_bundle);
  std::string source_after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !source_retired.ok &&
          source_retirement.SerializeAuthoritative(&source_after).ok &&
          source_before == source_after,
      "surviving source-edge dependency was not rejected atomically");

  city::wire::CoreState unrelated_source{};
  SourceEdgeRetirementFixture unrelated_source_fixture{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_source_edge_retirement_fixture(&unrelated_source,
                                              &unrelated_source_fixture),
      "unrelated source-edge fixture is incomplete");
  ExactBundleRetirementFixture unrelated_target{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_exact_bundle_route(
          &unrelated_source,
          {{100.0, 0.0, 0.0}, {112.0, 0.0, 0.0}},
          &unrelated_target),
      "unrelated exact Bundle fixture is incomplete");
  const auto unrelated_branch_span = std::find_if(
      unrelated_source.view().spans().items().begin(),
      unrelated_source.view().spans().items().end(),
      [&](const city::wire::Span& span) {
        return span.bundle_id == unrelated_source_fixture.branch_bundle;
      });
  WIRE_TEST_EXPECT_PRESENCE(
      unrelated_branch_span != unrelated_source.view().spans().items().end(),
      "unrelated source-edge branch Span is missing");
  city::wire::CoreStateTestHook::cache_state(unrelated_source)
      .span_layout_cache.clear_layout(unrelated_branch_span->id);
  const auto unrelated_retired =
      unrelated_source.RetireBackboneBundle(unrelated_target.bundle_a);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      unrelated_retired.ok && unrelated_retired.value &&
          unrelated_source.view().bundles().find(
              unrelated_source_fixture.source_bundle) != nullptr &&
          unrelated_source.view().bundles().find(
              unrelated_source_fixture.branch_bundle) != nullptr,
      unrelated_retired.error.empty()
          ? "unrelated source-edge cache affected exact Bundle retirement"
          : unrelated_retired.error);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(unrelated_source);
  return true;
}

bool C871_backbone_exact_bundle_retirement_conflicts_are_atomic() {
  auto prepare = [](city::wire::CoreState* state,
                    ExactBundleRetirementFixture* fixture) {
    return generate_exact_bundle_route(
        state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}, fixture);
  };

  city::wire::CoreState with_attachment{};
  ExactBundleRetirementFixture attachment_fixture{};
  WIRE_TEST_EXPECT_PRESENCE(prepare(&with_attachment, &attachment_fixture),
                            "Attachment conflict fixture is incomplete");
  std::vector<city::wire::ObjectId> spans{};
  std::vector<city::wire::ObjectId> ports{};
  std::vector<city::wire::ObjectId> attachments{};
  collect_exact_bundle_dependents(with_attachment,
                                  attachment_fixture.edge_bundles_a,
                                  &spans, &ports, &attachments);
  WIRE_TEST_EXPECT_PRESENCE(
      !spans.empty() && with_attachment.AddAttachment(spans.front(), 0.5).ok,
      "failed to add user Attachment to exact Bundle");
  std::string attachment_before{};
  WIRE_TEST_EXPECT_PRESENCE(
      with_attachment.SerializeAuthoritative(&attachment_before).ok,
      "failed to serialize Attachment conflict fixture");
  const auto attachment_retired =
      with_attachment.RetireBackboneBundle(attachment_fixture.bundle_a);
  std::string attachment_after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !attachment_retired.ok &&
          with_attachment.SerializeAuthoritative(&attachment_after).ok &&
          attachment_before == attachment_after,
      "user Attachment exact Bundle retirement was not failure atomic");

  city::wire::CoreState with_manual_port{};
  ExactBundleRetirementFixture port_fixture{};
  WIRE_TEST_EXPECT_PRESENCE(prepare(&with_manual_port, &port_fixture),
                            "manual Port conflict fixture is incomplete");
  spans.clear();
  ports.clear();
  attachments.clear();
  collect_exact_bundle_dependents(with_manual_port,
                                  port_fixture.edge_bundles_a,
                                  &spans, &ports, &attachments);
  const city::wire::Port* port =
      ports.empty() ? nullptr : with_manual_port.view().ports().find(ports.front());
  WIRE_TEST_EXPECT_PRESENCE(
      port != nullptr &&
          with_manual_port.SetPortWorldPositionManual(
              port->id, {port->world_position.x, port->world_position.y,
                         port->world_position.z + 0.05}).ok,
      "failed to mark exact Bundle Port manual");
  std::string port_before{};
  WIRE_TEST_EXPECT_PRESENCE(
      with_manual_port.SerializeAuthoritative(&port_before).ok,
      "failed to serialize manual Port conflict fixture");
  const auto port_retired =
      with_manual_port.RetireBackboneBundle(port_fixture.bundle_a);
  std::string port_after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !port_retired.ok &&
          with_manual_port.SerializeAuthoritative(&port_after).ok &&
          port_before == port_after,
      "manual Port exact Bundle retirement was not failure atomic");

  city::wire::CoreState with_span_override{};
  ExactBundleRetirementFixture override_fixture{};
  WIRE_TEST_EXPECT_PRESENCE(prepare(&with_span_override, &override_fixture),
                            "Span override conflict fixture is incomplete");
  spans.clear();
  ports.clear();
  attachments.clear();
  collect_exact_bundle_dependents(with_span_override,
                                  override_fixture.edge_bundles_a,
                                  &spans, &ports, &attachments);
  WIRE_TEST_EXPECT_PRESENCE(
      !spans.empty() &&
          with_span_override.SetSpanBranchDownOffsetOverride(
              spans.front(), 0.25).ok,
      "failed to set exact Bundle Span override");
  std::string override_before{};
  WIRE_TEST_EXPECT_PRESENCE(
      with_span_override.SerializeAuthoritative(&override_before).ok,
      "failed to serialize Span override conflict fixture");
  const auto override_retired =
      with_span_override.RetireBackboneBundle(override_fixture.bundle_a);
  std::string override_after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !override_retired.ok &&
          with_span_override.SerializeAuthoritative(&override_after).ok &&
          override_before == override_after,
      "Span override exact Bundle retirement was not failure atomic");
  return true;
}

bool C872_backbone_exact_bundle_retirement_rejects_incomplete_binding_scope() {
  city::wire::CoreState state{};
  ExactBundleRetirementFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(
      generate_exact_bundle_route(
          &state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}, &fixture),
      "fault injection fixture is incomplete");
  WIRE_TEST_EXPECT_PRESENCE(
      city::wire::CoreStateTestHook::erase_backbone_span_binding(
          state, fixture.edge_bundles_a.front(), 0),
      "failed to inject an incomplete exact Bundle SpanBinding scope");
  std::string before{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&before).ok,
                            "failed to serialize injected binding fault");
  const auto retired = state.RetireBackboneBundle(fixture.bundle_a);
  std::string after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !retired.ok && state.SerializeAuthoritative(&after).ok && before == after,
      "incomplete exact Bundle binding scope did not fail closed atomically");
  return true;
}

} // namespace backbone_tests
