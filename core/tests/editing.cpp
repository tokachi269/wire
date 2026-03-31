#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace helpers;
using wire::core::PortKind;
using wire::core::PortLayer;
using wire::core::SpanKind;
using wire::core::SpanLayer;

namespace {
bool test_id_generator_monotonic_and_reset() {
  wire::core::IdGenerator gen(1);
  if (gen.next() != 1 || gen.next() != 2 || gen.next() != 3) {
    return false;
  }
  if (gen.peek() != 4) {
    return false;
  }
  gen.reset(42);
  return gen.peek() == 42 && gen.next() == 42 && gen.next() == 43;
}

bool test_object_store_integrity() {
  struct Dummy {
    ObjectId id = 0;
    int payload = 0;
  };

  wire::core::ObjectStore<Dummy> store;
  store.insert(Dummy{1, 10});
  store.insert(Dummy{2, 20});
  store.insert(Dummy{3, 30});
  if (store.size() != 3 || !store.contains(2)) {
    return false;
  }
  if (store.find(3) == nullptr || store.find(3)->payload != 30) {
    return false;
  }
  if (!store.remove(2) || store.contains(2) || store.size() != 2) {
    return false;
  }
  if (store.find(1) == nullptr || store.find(3) == nullptr) {
    return false;
  }

  // Upsert must replace value without changing count.
  store.insert(Dummy{3, 99});
  return store.size() == 2 && store.find(3) != nullptr && store.find(3)->payload == 99;
}

bool test_span_runtime_initialized_on_add() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {4.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto span_result = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage);
  if (!span_result.ok) {
    return false;
  }

  const auto* runtime = state.view().find_span_runtime_state(span_result.value);
  return runtime != nullptr && runtime->span_id == span_result.value && runtime->data_version > 0 &&
         has_dirty(runtime, DirtyBits::kTopology | DirtyBits::kGeometry);
}

bool test_move_pole_dirties_only_related_span() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;

  const ObjectId a1 = state.AddPort(pole_a, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId a2 = state.AddPort(pole_a, {3.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b1 = state.AddPort(pole_b, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b2 = state.AddPort(pole_b, {13.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;

  const ObjectId related = state.AddSpan(a1, a2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  const ObjectId unrelated = state.AddSpan(b1, b2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  wire::core::Transformd moved{};
  moved.position = {2.0, 0.0, 0.0};
  const auto move_result = state.MovePole(pole_a, moved);
  if (!move_result.ok) {
    return false;
  }

  return has_dirty(state.view().find_span_runtime_state(related), DirtyBits::kGeometry) &&
         !has_dirty(state.view().find_span_runtime_state(unrelated), DirtyBits::kGeometry);
}

bool test_split_span_creates_two_spans_and_port() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_id = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto split_result = state.SplitSpan(span_id, 0.5);
  if (!split_result.ok) {
    return false;
  }

  if (state.view().edit_state().spans.find(span_id) != nullptr) {
    return false;
  }
  if (state.view().edit_state().ports.find(split_result.value.new_port_id) == nullptr) {
    return false;
  }
  if (state.view().edit_state().spans.find(split_result.value.new_span_a_id) == nullptr ||
      state.view().edit_state().spans.find(split_result.value.new_span_b_id) == nullptr) {
    return false;
  }
  return validate_now(state).ok();
}

bool test_apply_pole_type_generates_template_ports() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.size() < 2) {
    return false;
  }

  const ObjectId pole_id = state.AddPole({}, 10.0, "P").value;
  const auto apply_result = state.ApplyPoleType(pole_id, pole_type_ids[0]);
  if (!apply_result.ok) {
    return false;
  }

  const auto detail = state.GetPoleDetail(pole_id);
  if (detail.pole == nullptr || detail.pole_type == nullptr || detail.owned_ports.empty()) {
    return false;
  }
  for (const auto* port : detail.owned_ports) {
    if (port->owner_pole_id != pole_id) {
      return false;
    }
  }
  return true;
}

bool test_different_pole_types_produce_different_port_hints() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.size() < 2) {
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok) {
    return false;
  }
  if (!state.ApplyPoleType(pole_b, pole_type_ids[1]).ok) {
    return false;
  }

  const auto detail_a = state.GetPoleDetail(pole_a);
  const auto detail_b = state.GetPoleDetail(pole_b);
  if (detail_a.owned_ports.empty() || detail_b.owned_ports.empty()) {
    return false;
  }
  if (detail_a.owned_ports.size() != detail_b.owned_ports.size()) {
    return true;
  }

  std::vector<long long> hints_a;
  std::vector<long long> hints_b;
  for (const auto* port : detail_a.owned_ports)
    hints_a.push_back((static_cast<long long>(port->template_layer) << 32) |
                      (static_cast<long long>(static_cast<int>(port->template_side)) << 16) |
                      static_cast<long long>(static_cast<int>(port->template_role)));
  for (const auto* port : detail_b.owned_ports)
    hints_b.push_back((static_cast<long long>(port->template_layer) << 32) |
                      (static_cast<long long>(static_cast<int>(port->template_side)) << 16) |
                      static_cast<long long>(static_cast<int>(port->template_role)));
  std::sort(hints_a.begin(), hints_a.end());
  std::sort(hints_b.begin(), hints_b.end());
  return hints_a != hints_b;
}

bool test_auto_port_allocation_prefers_unused_ports() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {20.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok || !state.ApplyPoleType(pole_b, pole_type_ids[0]).ok) {
    return false;
  }

  const auto first = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kHighVoltage);
  const auto second = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kHighVoltage);
  if (!first.ok || !second.ok) {
    return false;
  }
  return first.value.port_a_id != second.value.port_a_id;
}

bool test_add_connection_by_pole_updates_dirty_version_and_indices() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids[0]);
  (void)state.ApplyPoleType(pole_b, pole_type_ids[0]);

  const auto connection = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!connection.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(connection.value.span_id);
  const auto* port_a = state.view().edit_state().ports.find(connection.value.port_a_id);
  const auto* port_b = state.view().edit_state().ports.find(connection.value.port_b_id);
  const auto* runtime = state.view().find_span_runtime_state(connection.value.span_id);
  if (span == nullptr || port_a == nullptr || port_b == nullptr || runtime == nullptr) {
    return false;
  }
  if (port_a->owner_pole_id != pole_a || port_b->owner_pole_id != pole_b) {
    return false;
  }

  auto it_a = state.view().connection_index().spans_by_port.find(port_a->id);
  auto it_b = state.view().connection_index().spans_by_port.find(port_b->id);
  if (it_a == state.view().connection_index().spans_by_port.end() || it_b == state.view().connection_index().spans_by_port.end()) {
    return false;
  }

  return contains_id(it_a->second, span->id) && contains_id(it_b->second, span->id) &&
         has_dirty(runtime, DirtyBits::kGeometry) && contains_id(connection.change_set.created_ids, span->id) &&
         contains_id(connection.change_set.dirty_span_ids, span->id);
}

bool test_add_drop_from_pole_creates_service_connection() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  (void)state.ApplyPoleType(pole, pole_type_ids[0]);
  const auto drop = state.AddDropFromPole(pole, {5.0, 3.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(drop.value.span_id);
  const auto* source_port = state.view().edit_state().ports.find(drop.value.source_port_id);
  const auto* target_port = state.view().edit_state().ports.find(drop.value.target_port_id);
  if (span == nullptr || source_port == nullptr || target_port == nullptr) {
    return false;
  }
  return source_port->owner_pole_id == pole && target_port->owner_pole_id == wire::core::kInvalidObjectId &&
         validate_now(state).ok();
}

bool test_add_drop_from_span_splits_and_connects_drop() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {20.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids[0]);
  (void)state.ApplyPoleType(pole_b, pole_type_ids[0]);

  const auto base = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!base.ok) {
    return false;
  }

  const auto drop = state.AddDropFromSpan(base.value.span_id, 0.5, {10.0, 4.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  if (state.view().edit_state().spans.find(base.value.span_id) != nullptr) {
    return false;
  }
  const auto* split_port = state.view().edit_state().ports.find(drop.value.split_port_id);
  const auto* drop_span = state.view().edit_state().spans.find(drop.value.span_id);
  if (split_port == nullptr || drop_span == nullptr) {
    return false;
  }
  auto split_it = state.view().connection_index().spans_by_port.find(drop.value.split_port_id);
  if (split_it == state.view().connection_index().spans_by_port.end()) {
    return false;
  }
  return split_it->second.size() >= 3 && validate_now(state).ok();
}

std::optional<wire::core::PoleTypeDefinition> find_pole_type_by_name(const CoreState& state, const std::string& name) {
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == name) {
      return pole_type;
    }
  }
  return std::nullopt;
}

double average_owned_port_height_for_layer(const wire::core::PoleDetailInfo& detail, wire::core::PortLayer layer) {
  double total = 0.0;
  int count = 0;
  for (const auto* port : detail.owned_ports) {
    if (port != nullptr && port->layer == layer) {
      total += port->world_position.z;
      ++count;
    }
  }
  return count > 0 ? total / static_cast<double>(count) : std::numeric_limits<double>::quiet_NaN();
}

bool test_communication_pole_default_band_order_places_support_hv_comm_optical_top_down() {
  CoreState state;
  const auto pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!pole_type.has_value()) {
    return false;
  }

  double top_support_z = -std::numeric_limits<double>::infinity();
  double high_voltage_z = -std::numeric_limits<double>::infinity();
  double low_voltage_z = -std::numeric_limits<double>::infinity();
  double optical_z = -std::numeric_limits<double>::infinity();
  double communication_z = -std::numeric_limits<double>::infinity();
  double drop_z = -std::numeric_limits<double>::infinity();
  for (const auto& band : pole_type->port_bands) {
    if (!band.enabled) {
      continue;
    }
    if (band.band_id == 600) {
      top_support_z = band.height_center_m;
    } else if (band.category == wire::core::ConnectionCategory::kHighVoltage) {
      high_voltage_z = std::max(high_voltage_z, band.height_center_m);
    } else if (band.category == wire::core::ConnectionCategory::kLowVoltage) {
      low_voltage_z = std::max(low_voltage_z, band.height_center_m);
    } else if (band.category == wire::core::ConnectionCategory::kOptical) {
      optical_z = std::max(optical_z, band.height_center_m);
    } else if (band.category == wire::core::ConnectionCategory::kCommunication && band.band_id != 600) {
      communication_z = std::max(communication_z, band.height_center_m);
    } else if (band.category == wire::core::ConnectionCategory::kDrop) {
      drop_z = std::max(drop_z, band.height_center_m);
    }
  }

  return std::isfinite(top_support_z) && std::isfinite(high_voltage_z) && std::isfinite(low_voltage_z) &&
         std::isfinite(optical_z) && std::isfinite(communication_z) && std::isfinite(drop_z) &&
         top_support_z > high_voltage_z && high_voltage_z > low_voltage_z && low_voltage_z > optical_z &&
         optical_z > communication_z && communication_z > drop_z;
}

bool test_communication_pole_communication_and_optical_bands_are_centered() {
  CoreState state;
  const auto pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!pole_type.has_value()) {
    return false;
  }

  int communication_band_count = 0;
  int optical_band_count = 0;
  for (const auto& band : pole_type->port_bands) {
    if (!band.enabled || band.band_id == 600) {
      continue;
    }
    if (band.category == wire::core::ConnectionCategory::kCommunication) {
      ++communication_band_count;
      if (band.side != wire::core::SlotSide::kCenter) {
        return false;
      }
    } else if (band.category == wire::core::ConnectionCategory::kOptical) {
      ++optical_band_count;
      if (band.side != wire::core::SlotSide::kCenter) {
        return false;
      }
    }
  }

  return communication_band_count == 2 && optical_band_count == 2;
}

bool test_communication_pole_default_category_placement_values_match_viewer_defaults() {
  CoreState state;
  const auto pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!pole_type.has_value()) {
    return false;
  }

  auto band_matches = [](const wire::core::PortPlacementBand& band, wire::core::ConnectionCategory category, int layer) {
    return band.enabled && band.category == category && band.layer == layer;
  };
  auto average_height = [&](wire::core::ConnectionCategory category, int layer) {
    double total = 0.0;
    int count = 0;
    for (const auto& band : pole_type->port_bands) {
      if (!band_matches(band, category, layer)) {
        continue;
      }
      total += band.height_center_m;
      ++count;
    }
    return (count > 0) ? (total / static_cast<double>(count)) : std::numeric_limits<double>::quiet_NaN();
  };
  auto average_offset = [&](wire::core::ConnectionCategory category, int layer) {
    double total = 0.0;
    int count = 0;
    for (const auto& band : pole_type->port_bands) {
      if (!band_matches(band, category, layer)) {
        continue;
      }
      total += band.lateral_center_m;
      ++count;
    }
    return (count > 0) ? (total / static_cast<double>(count)) : std::numeric_limits<double>::quiet_NaN();
  };
  auto max_spread = [&](wire::core::ConnectionCategory category, int layer, double center) {
    double spread = 0.0;
    for (const auto& band : pole_type->port_bands) {
      if (!band_matches(band, category, layer)) {
        continue;
      }
      spread = std::max(spread, std::abs(band.lateral_center_m - center));
    }
    return spread;
  };

  const double hv_offset = average_offset(wire::core::ConnectionCategory::kHighVoltage, 2);
  const double lv_offset = average_offset(wire::core::ConnectionCategory::kLowVoltage, 1);
  const double communication_offset = average_offset(wire::core::ConnectionCategory::kCommunication, 1);
  const double optical_offset = average_offset(wire::core::ConnectionCategory::kOptical, 1);
  const double drop_offset = average_offset(wire::core::ConnectionCategory::kDrop, 0);

  return almost_equal(average_height(wire::core::ConnectionCategory::kHighVoltage, 2), 9.20, 1e-9) &&
         almost_equal(hv_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(wire::core::ConnectionCategory::kHighVoltage, 2, hv_offset), 0.60, 1e-9) &&
         almost_equal(average_height(wire::core::ConnectionCategory::kLowVoltage, 1), 8.03, 1e-9) &&
         almost_equal(lv_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(wire::core::ConnectionCategory::kLowVoltage, 1, lv_offset), 0.40, 1e-9) &&
         almost_equal(average_height(wire::core::ConnectionCategory::kCommunication, 1), 6.25, 1e-9) &&
         almost_equal(communication_offset, 0.22, 1e-9) &&
         almost_equal(max_spread(wire::core::ConnectionCategory::kCommunication, 1, communication_offset), 0.0, 1e-9) &&
         almost_equal(average_height(wire::core::ConnectionCategory::kOptical, 1), 6.60, 1e-9) &&
         almost_equal(optical_offset, 0.04, 1e-9) &&
         almost_equal(max_spread(wire::core::ConnectionCategory::kOptical, 1, optical_offset), 0.0, 1e-9) &&
         almost_equal(average_height(wire::core::ConnectionCategory::kDrop, 0), 4.20, 1e-9) &&
         almost_equal(drop_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(wire::core::ConnectionCategory::kDrop, 0, drop_offset), 0.0, 1e-9);
}

bool test_optical_connection_uses_cable_template_supplemental_path() {
  CoreState state;
  const auto communication_pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!communication_pole_type.has_value()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, communication_pole_type->id).ok ||
      !state.ApplyPoleType(pole_b, communication_pole_type->id).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kOptical;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kOptical, options);
  if (!add.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr || span->endpoint_attachment_a_id != wire::core::kInvalidObjectId ||
      span->endpoint_attachment_b_id != wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kOptical) {
    return false;
  }
  const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_template_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_template_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
  if (cable_template_it == state.view().cable_templates().end()) {
    return false;
  }
  if (cable_template_it->second.supplemental_paths.size() != 1 ||
      cable_template_it->second.default_endpoint_attachment_template_id != wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  bool has_coil_profile = false;
  for (const auto& supplemental : cable_template_it->second.supplemental_paths) {
    has_coil_profile = has_coil_profile ||
                       (supplemental.profile_kind == wire::core::CableSupplementalPathTemplate::ProfileKind::kCoiledCable &&
                        supplemental.anchor_mode ==
                            wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset &&
                        supplemental.interaction_mode == wire::core::AttachmentLineInteractionMode::kAddInternalPath &&
                        std::abs(supplemental.vertical_offset_m) < 1e-9);
  }
  if (!has_coil_profile || bundle_template_it->second.support_wire_pole_band_id != 0) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(add.value.span_id);
  if (curve == nullptr || !curve->detail.hidden_intervals.empty() || !curve->detail.replacement_paths.empty() ||
      curve->detail.supplemental_paths.size() != 1) {
    return false;
  }
  bool has_coiled_supplemental = false;
  const auto& supplemental = curve->detail.supplemental_paths.front();
  if (supplemental.points.size() > 2) {
    const auto chord = supplemental.points.back() - supplemental.points.front();
    double max_chord_deviation = 0.0;
    const double chord_len_sq = Dot(chord, chord);
    for (std::size_t i = 1; i + 1 < supplemental.points.size(); ++i) {
      const auto from_start = supplemental.points[i] - supplemental.points.front();
      double t = 0.0;
      if (chord_len_sq > 1e-12) {
        t = std::clamp(Dot(from_start, chord) / chord_len_sq, 0.0, 1.0);
      }
      const auto on_chord = supplemental.points.front() + ScaleVec(chord, t);
      const auto deviation = supplemental.points[i] - on_chord;
      max_chord_deviation = std::max(max_chord_deviation, std::sqrt(std::max(0.0, Dot(deviation, deviation))));
    }
    has_coiled_supplemental = max_chord_deviation > 0.01;
  }
  if (!has_coiled_supplemental) {
    return false;
  }
  const auto* port_a = state.view().edit_state().ports.find(span->port_a_id);
  const auto* port_b = state.view().edit_state().ports.find(span->port_b_id);
  const auto* pole_a_entity =
      (port_a == nullptr) ? nullptr : state.view().edit_state().poles.find(port_a->owner_pole_id);
  const auto* pole_b_entity =
      (port_b == nullptr) ? nullptr : state.view().edit_state().poles.find(port_b->owner_pole_id);
  if (port_a == nullptr || port_b == nullptr || pole_a_entity == nullptr || pole_b_entity == nullptr) {
    return false;
  }
  const auto point_near_trimmed_main = [&](const wire::core::Vec3d& point, double min_t, double max_t) {
    const auto chord = port_b->world_position - port_a->world_position;
    const double chord_len_sq = Dot(chord, chord);
    if (chord_len_sq <= 1e-12) {
      return false;
    }
    const double t = Dot(point - port_a->world_position, chord) / chord_len_sq;
    const auto on_chord = port_a->world_position + ScaleVec(chord, std::clamp(t, 0.0, 1.0));
    const auto delta = point - on_chord;
    const double distance_to_chord = std::sqrt(std::max(0.0, Dot(delta, delta)));
    return t >= min_t && t <= max_t && distance_to_chord <= 0.08;
  };
  if (!point_near_trimmed_main(supplemental.points.front(), 0.0, 0.08) ||
      !point_near_trimmed_main(supplemental.points.back(), 0.92, 1.0)) {
    return false;
  }
  return validate_now(state).ok();
}

bool test_optical_nonpole_span_does_not_add_top_support_wire() {
  CoreState state;
  const auto communication_pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!communication_pole_type.has_value()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, communication_pole_type->id).ok ||
      !state.ApplyPoleType(pole_b, communication_pole_type->id).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kOptical;
  const auto base = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kOptical, options);
  if (!base.ok) {
    return false;
  }

  const auto branch = state.AddDropFromSpan(base.value.span_id, 0.5, {8.0, 4.0, 0.0}, wire::core::ConnectionCategory::kOptical);
  if (!branch.ok) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(branch.value.span_id);
  if (curve == nullptr || curve->detail.supplemental_paths.size() != 1 || !curve->detail.replacement_paths.empty()) {
    return false;
  }
  return curve->detail.supplemental_paths.front().points.size() > 2 && validate_now(state).ok();
}

bool test_default_templates_do_not_register_optical_with_support_bundle() {
  CoreState state;
  return state.view().bundle_templates().find(wire::core::BundleKind::kOpticalWithSupport) ==
         state.view().bundle_templates().end();
}

bool test_optical_bundle_keeps_coil_without_support_wire() {
  CoreState state;
  const auto communication_pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!communication_pole_type.has_value()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, communication_pole_type->id).ok ||
      !state.ApplyPoleType(pole_b, communication_pole_type->id).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kOptical;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kOptical, options);
  if (!add.ok) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(add.value.span_id);
  return curve != nullptr && curve->detail.replacement_paths.empty() && curve->detail.supplemental_paths.size() == 1 &&
         validate_now(state).ok();
}

bool test_update_pole_type_definition_refreshes_existing_communication_ports() {
  CoreState state;
  const auto communication_pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!communication_pole_type.has_value()) {
    return false;
  }

  const ObjectId pole_id = state.AddPole({}, 10.0, "P").value;
  if (!state.ApplyPoleType(pole_id, communication_pole_type->id).ok) {
    return false;
  }
  const auto before = state.GetPoleDetail(pole_id);
  const double before_optical_height = average_owned_port_height_for_layer(before, wire::core::PortLayer::kOptical);
  if (!std::isfinite(before_optical_height)) {
    return false;
  }
  const double before_pole_height = state.view().edit_state().poles.find(pole_id)->height_m;

  wire::core::PoleTypeDefinition updated = *communication_pole_type;
  updated.default_height_m += 0.35;
  for (auto& band : updated.port_bands) {
    if (band.category == wire::core::ConnectionCategory::kOptical) {
      band.height_center_m += 0.75;
      band.height_min_m += 0.75;
      band.height_max_m += 0.75;
    }
  }
  const auto apply = state.UpdatePoleTypeDefinition(updated);
  if (!apply.ok || !apply.value) {
    return false;
  }

  const auto after = state.GetPoleDetail(pole_id);
  const double after_optical_height = average_owned_port_height_for_layer(after, wire::core::PortLayer::kOptical);
  const double after_pole_height = state.view().edit_state().poles.find(pole_id)->height_m;
  return std::isfinite(after_optical_height) &&
         std::abs((after_pole_height - before_pole_height) - 0.35) < 1e-6 &&
         std::abs((after_optical_height - before_optical_height) - 0.75) < 1e-6 &&
         validate_now(state).ok();
}

bool test_default_pole_types_use_single_hv_height_per_template() {
  CoreState state;
  for (const char* pole_type_name : {"DistributionPole", "CommunicationPole"}) {
    const auto pole_type = find_pole_type_by_name(state, pole_type_name);
    if (!pole_type.has_value()) {
      return false;
    }
    double first_height = std::numeric_limits<double>::quiet_NaN();
    for (const auto& band : pole_type->port_bands) {
      if (!band.enabled || band.category != wire::core::ConnectionCategory::kHighVoltage) {
        continue;
      }
      if (!std::isfinite(first_height)) {
        first_height = band.height_center_m;
        continue;
      }
      if (std::abs(band.height_center_m - first_height) > 1e-9) {
        return false;
      }
    }
    if (!std::isfinite(first_height)) {
      return false;
    }
  }
  return true;
}

bool test_bundle_related_pole_type_applies_to_existing_poles() {
  CoreState state;
  const auto distribution_pole_type = find_pole_type_by_name(state, "DistributionPole");
  const auto communication_pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!distribution_pole_type.has_value() || !communication_pole_type.has_value()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {18.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, distribution_pole_type->id).ok ||
      !state.ApplyPoleType(pole_b, distribution_pole_type->id).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kOptical;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kOptical, options);
  if (!add.ok) {
    return false;
  }

  const auto apply = state.ApplyBundleRelatedPoleTypeToExistingPoles(wire::core::BundleKind::kOptical);
  if (!apply.ok || !apply.value) {
    return false;
  }

  const auto* pole_a_after = state.view().edit_state().poles.find(pole_a);
  const auto* pole_b_after = state.view().edit_state().poles.find(pole_b);
  if (pole_a_after == nullptr || pole_b_after == nullptr) {
    return false;
  }
  return pole_a_after->pole_type_id == communication_pole_type->id &&
         pole_b_after->pole_type_id == communication_pole_type->id &&
         std::abs(pole_a_after->height_m - communication_pole_type->default_height_m) < 1e-9 &&
         std::abs(pole_b_after->height_m - communication_pole_type->default_height_m) < 1e-9 &&
         validate_now(state).ok();
}

void register_editing_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C01_IdGenerator", "IdGenerator monotonic and reset behavior", "Exact", false, test_id_generator_monotonic_and_reset);
  test_registry::AddTest(tests, "C02_ObjectStore", "ObjectStore add/find/remove integrity", "Exact", false, test_object_store_integrity);
  test_registry::AddTest(tests, "C03_Phase3_SpanRuntime_Initialized", "AddSpan initializes runtime+dirty", "Exact", false, test_span_runtime_initialized_on_add);
  test_registry::AddTest(tests, "C04_Phase3_MovePole_LocalDirty", "MovePole dirties only related span", "Invariant", false, test_move_pole_dirties_only_related_span);
  test_registry::AddTest(tests, "C05_Phase3_SplitSpan_Basic", "SplitSpan replaces structure and keeps integrity", "Invariant", false, test_split_span_creates_two_spans_and_port);
  test_registry::AddTest(tests, "C06_Phase35_ApplyPoleType_GeneratesPorts", "ApplyPoleType generates owned template ports", "Invariant", false, test_apply_pole_type_generates_template_ports);
  test_registry::AddTest(tests, "C07_Phase35_PoleType_DifferentLayouts", "Different pole types produce different port hint layout", "Invariant", false, test_different_pole_types_produce_different_port_hints);
  test_registry::AddTest(tests, "C08_Phase35_AutoAlloc_UnusedPriority", "Auto allocation prefers unused ports", "Invariant", false, test_auto_port_allocation_prefers_unused_ports);
  test_registry::AddTest(tests, "C09_Phase35_AddConnectionByPole_Basic", "Pole->Pole connection updates dirty/index/changeset", "Invariant", false, test_add_connection_by_pole_updates_dirty_version_and_indices);
  test_registry::AddTest(tests, "C11_Phase35_AddDropFromPole_Basic", "Drop from pole creates service span", "Invariant", false, test_add_drop_from_pole_creates_service_connection);
  test_registry::AddTest(tests, "C12_Phase35_AddDropFromSpan_Basic", "Drop from span splits and connects", "Invariant", false, test_add_drop_from_span_splits_and_connects_drop);
  test_registry::AddTest(tests, "C287_CommunicationPole_DefaultBandOrder", "CommunicationPole keeps support, HV, LV, optical, communication, drop in descending height order", "Invariant", false, test_communication_pole_default_band_order_places_support_hv_comm_optical_top_down);
  test_registry::AddTest(tests, "C309_CommunicationPole_CommOpticalBandsCentered",
                         "CommunicationPole keeps communication and optical default bands centered instead of split left/right",
                         "Invariant", false, test_communication_pole_communication_and_optical_bands_are_centered);
  test_registry::AddTest(tests, "C313_CommunicationPole_DefaultCategoryPlacementValues",
                         "CommunicationPole default category placement values match viewer defaults",
                         "Invariant", false, test_communication_pole_default_category_placement_values_match_viewer_defaults);
  test_registry::AddTest(tests, "C288_OpticalConnection_UsesCableTemplateSupplementalPath",
                         "Optical connection uses cable-template supplemental line authority instead of endpoint attachments",
                         "Invariant", false, test_optical_connection_uses_cable_template_supplemental_path);
  test_registry::AddTest(tests, "C293_OpticalNonPoleSpan_SkipsTopSupportWire",
                         "Optical non-pole span keeps only the coiled supplemental path",
                         "Invariant", false, test_optical_nonpole_span_does_not_add_top_support_wire);
  test_registry::AddTest(tests, "C310_DefaultTemplates_NoOpticalWithSupportBundle",
                         "Default templates do not register a split optical-with-support bundle type",
                         "Invariant", false, test_default_templates_do_not_register_optical_with_support_bundle);
  test_registry::AddTest(tests, "C312_OpticalBundle_KeepsCoilWithoutSupportWire",
                         "Optical bundle keeps one coiled supplemental path without adding support wire",
                         "Invariant", false, test_optical_bundle_keeps_coil_without_support_wire);
  test_registry::AddTest(tests, "C289_UpdatePoleTypeDefinition_ReappliesExistingPorts", "Updating CommunicationPole port-band heights reapplies owned auto ports on existing poles", "Invariant", false, test_update_pole_type_definition_refreshes_existing_communication_ports);
  test_registry::AddTest(tests, "C296_DefaultPoleTypes_HVCategoryUsesSingleHeight",
                         "Default pole templates keep one HV category height so category-level editing does not average multiple defaults",
                         "Invariant", false, test_default_pole_types_use_single_hv_height_per_template);
  test_registry::AddTest(tests, "C297_BundleTemplate_ApplyRelatedPoleTypeToExistingPoles",
                         "Bundle-linked pole template can be applied to existing poles attached to bundles of that template",
                         "Invariant", false, test_bundle_related_pole_type_applies_to_existing_poles);
}

WIRE_REGISTER_TEST_SUITE(register_editing_tests);

} // namespace
