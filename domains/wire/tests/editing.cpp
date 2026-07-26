#include "registry.hpp"
#include "helpers.hpp"
#include "city/wire/coord_utils.hpp"

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
using city::wire::PortKind;
using city::wire::PortLayer;
using city::wire::SpanKind;
using city::wire::SpanLayer;

namespace {
bool test_id_generator_monotonic_and_reset() {
  city::wire::IdGenerator gen(1);
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

  city::wire::ObjectStore<Dummy> store;
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
  return runtime != nullptr && runtime->span_id == span_result.value && runtime->data_version > 0;
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
  const auto* related_before = state.view().find_span_runtime_state(related);
  const auto* unrelated_before = state.view().find_span_runtime_state(unrelated);
  if (related_before == nullptr || unrelated_before == nullptr) {
    return false;
  }
  const std::uint64_t related_version = related_before->data_version;
  const std::uint64_t unrelated_version = unrelated_before->data_version;

  city::wire::Transformd moved{};
  moved.position = {2.0, 0.0, 0.0};
  const auto move_result = state.MovePole(pole_a, moved);
  if (!move_result.ok) {
    return false;
  }

  const auto* related_after = state.view().find_span_runtime_state(related);
  const auto* unrelated_after = state.view().find_span_runtime_state(unrelated);
  return related_after != nullptr && unrelated_after != nullptr && related_after->data_version > related_version &&
         unrelated_after->data_version == unrelated_version;
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

  city::wire::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  city::wire::Transformd b{};
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

  city::wire::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  city::wire::Transformd b{};
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
         runtime->data_version > 0 && contains_id(connection.change_set.created_ids, span->id) &&
         contains_id(connection.change_set.updated_ids, span->id);
}

std::optional<city::wire::PoleTypeDefinition> find_pole_type_by_name(const CoreState& state, const std::string& name) {
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == name) {
      return pole_type;
    }
  }
  return std::nullopt;
}

double average_owned_port_height_for_layer(const city::wire::PoleDetailInfo& detail, city::wire::PortLayer layer) {
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

bool test_communication_pole_default_band_order_places_japan_distribution_profile() {
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
    } else if (band.category == city::wire::ConnectionCategory::kHighVoltage) {
      high_voltage_z = std::max(high_voltage_z, band.height_center_m);
    } else if (band.category == city::wire::ConnectionCategory::kLowVoltage) {
      low_voltage_z = std::max(low_voltage_z, band.height_center_m);
    } else if (band.category == city::wire::ConnectionCategory::kOptical) {
      optical_z = std::max(optical_z, band.height_center_m);
    } else if (band.category == city::wire::ConnectionCategory::kCommunication && band.band_id != 600) {
      communication_z = std::max(communication_z, band.height_center_m);
    } else if (band.category == city::wire::ConnectionCategory::kDrop) {
      drop_z = std::max(drop_z, band.height_center_m);
    }
  }

  return std::isfinite(top_support_z) && std::isfinite(high_voltage_z) && std::isfinite(low_voltage_z) &&
         std::isfinite(optical_z) && std::isfinite(communication_z) && std::isfinite(drop_z) &&
         almost_equal(top_support_z, communication_z, 1e-9) &&
         high_voltage_z > low_voltage_z && low_voltage_z > drop_z &&
         drop_z > optical_z && almost_equal(optical_z, communication_z, 1e-9);
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
    if (band.category == city::wire::ConnectionCategory::kCommunication) {
      ++communication_band_count;
      if (band.side != city::wire::SlotSide::kCenter) {
        return false;
      }
    } else if (band.category == city::wire::ConnectionCategory::kOptical) {
      ++optical_band_count;
      if (band.side != city::wire::SlotSide::kCenter) {
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

  auto band_matches = [](const city::wire::PortPlacementBand& band, city::wire::ConnectionCategory category, int layer) {
    return band.enabled && band.category == category && band.layer == layer;
  };
  auto average_height = [&](city::wire::ConnectionCategory category, int layer) {
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
  auto average_offset = [&](city::wire::ConnectionCategory category, int layer) {
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
  auto max_spread = [&](city::wire::ConnectionCategory category, int layer, double center) {
    double spread = 0.0;
    for (const auto& band : pole_type->port_bands) {
      if (!band_matches(band, category, layer)) {
        continue;
      }
      spread = std::max(spread, std::abs(band.lateral_center_m - center));
    }
    return spread;
  };

  const double hv_offset = average_offset(city::wire::ConnectionCategory::kHighVoltage, 2);
  const double lv_offset = average_offset(city::wire::ConnectionCategory::kLowVoltage, 1);
  const double communication_offset = average_offset(city::wire::ConnectionCategory::kCommunication, 1);
  const double optical_offset = average_offset(city::wire::ConnectionCategory::kOptical, 1);
  const double drop_offset = average_offset(city::wire::ConnectionCategory::kDrop, 0);

  return almost_equal(average_height(city::wire::ConnectionCategory::kHighVoltage, 2), 9.20, 1e-9) &&
         almost_equal(hv_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(city::wire::ConnectionCategory::kHighVoltage, 2, hv_offset), 0.75, 1e-9) &&
         almost_equal(average_height(city::wire::ConnectionCategory::kLowVoltage, 1), 7.40, 1e-9) &&
         almost_equal(lv_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(city::wire::ConnectionCategory::kLowVoltage, 1, lv_offset), 0.45, 1e-9) &&
         almost_equal(average_height(city::wire::ConnectionCategory::kCommunication, 1), 5.30, 1e-9) &&
         almost_equal(communication_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(city::wire::ConnectionCategory::kCommunication, 1, communication_offset), 0.18, 1e-9) &&
         almost_equal(average_height(city::wire::ConnectionCategory::kOptical, 1), 5.30, 1e-9) &&
         almost_equal(optical_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(city::wire::ConnectionCategory::kOptical, 1, optical_offset), 0.35, 1e-9) &&
         almost_equal(average_height(city::wire::ConnectionCategory::kDrop, 0), 5.50, 1e-9) &&
         almost_equal(drop_offset, 0.0, 1e-9) &&
         almost_equal(max_spread(city::wire::ConnectionCategory::kDrop, 0, drop_offset), 0.0, 1e-9);
}

bool test_core_defaults_own_viewer_startup_semantics() {
  CoreState state;
  if (!state.view().geometry_settings().sag_enabled) {
    return false;
  }
  auto cable_by_name = [&](const std::string& name) -> const city::wire::CableTemplate* {
    for (const auto& item : state.view().cable_templates()) {
      if (item.second.name == name) {
        return &item.second;
      }
    }
    return nullptr;
  };
  const auto* hv = cable_by_name("HV_BARE");
  const auto* lv = cable_by_name("LV_INSULATED");
  const auto* comm = cable_by_name("COMM_MULTI");
  const auto* optical = cable_by_name("OPTICAL_FIBER");
  const auto* drop = cable_by_name("DROP_SERVICE");
  const auto pole_type = find_pole_type_by_name(state, "CommunicationPole");
  return hv != nullptr && lv != nullptr && comm != nullptr && optical != nullptr &&
         drop != nullptr && pole_type.has_value() &&
         almost_equal(hv->outer_diameter_m, 0.024, 1e-12) &&
         almost_equal(lv->outer_diameter_m, 0.020, 1e-12) &&
         almost_equal(comm->outer_diameter_m, 0.016, 1e-12) &&
         almost_equal(optical->outer_diameter_m, 0.012, 1e-12) &&
         almost_equal(drop->outer_diameter_m, 0.016, 1e-12) &&
         almost_equal(pole_type->default_height_m, 10.0, 1e-12);
}

bool test_default_templates_do_not_register_optical_with_support_bundle() {
  CoreState state;
  return state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kOpticalWithSupport)) ==
         state.view().bundle_templates().end();
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
  const double before_optical_height = average_owned_port_height_for_layer(before, city::wire::PortLayer::kOptical);
  if (!std::isfinite(before_optical_height)) {
    return false;
  }
  const double before_pole_height = state.view().edit_state().poles.find(pole_id)->height_m;

  city::wire::PoleTypeDefinition updated = *communication_pole_type;
  updated.default_height_m += 0.35;
  for (auto& band : updated.port_bands) {
    if (band.category == city::wire::ConnectionCategory::kOptical) {
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
  const double after_optical_height = average_owned_port_height_for_layer(after, city::wire::PortLayer::kOptical);
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
      if (!band.enabled || band.category != city::wire::ConnectionCategory::kHighVoltage) {
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

bool test_bundle_related_pole_type_regenerates_generated_topology() {
  CoreState state;
  const auto distribution_pole_type = find_pole_type_by_name(state, "DistributionPole");
  const auto communication_pole_type = find_pole_type_by_name(state, "CommunicationPole");
  if (!distribution_pole_type.has_value() || !communication_pole_type.has_value()) {
    return false;
  }

  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {18.0, 0.0, 0.0}});
  if (!made.ok || made.value.poles.size() != 2) {
    return false;
  }
  const ObjectId pole_a = made.value.poles[0];
  const ObjectId pole_b = made.value.poles[1];
  const auto before_a = *state.view().edit_state().poles.find(pole_a);
  const auto before_b = *state.view().edit_state().poles.find(pole_b);

  city::wire::BackboneSpec add{};
  add.path.polyline = {before_a.world_transform.position, before_b.world_transform.position};
  add.path.node_specs.resize(2);
  add.path.node_specs[0].point_index = 0;
  add.path.node_specs[0].support_kind = city::wire::SupportKind::kPole;
  add.path.node_specs[0].node_id = pole_a;
  add.path.node_specs[1].point_index = 1;
  add.path.node_specs[1].support_kind = city::wire::SupportKind::kPole;
  add.path.node_specs[1].node_id = pole_b;
  add_backbone_bundle(add, city::wire::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(add).ok) {
    return false;
  }

  const auto apply = state.ApplyBundleRelatedPoleTypeToExistingPoles(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kOptical));
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
         pole_a_after->pole_type_id != before_a.pole_type_id &&
         pole_b_after->pole_type_id != before_b.pole_type_id &&
         validate_now(state).ok();
}

void register_editing_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C01_IdGenerator", "IdGenerator monotonic and reset behavior", "Exact", false, test_id_generator_monotonic_and_reset);
  test_registry::AddTest(tests, "C02_ObjectStore", "ObjectStore add/find/remove integrity", "Exact", false, test_object_store_integrity);
  test_registry::AddTest(tests, "C03_Phase3_SpanRuntime_Initialized", "AddSpan initializes runtime+dirty", "Exact", false, test_span_runtime_initialized_on_add);
  test_registry::AddTest(tests, "C04_Phase3_MovePole_LocalDirty", "MovePole dirties only related span", "Invariant", false, test_move_pole_dirties_only_related_span);
  test_registry::AddTest(tests, "C06_Phase35_ApplyPoleType_GeneratesPorts", "ApplyPoleType generates owned template ports", "Invariant", false, test_apply_pole_type_generates_template_ports);
  test_registry::AddTest(tests, "C07_Phase35_PoleType_DifferentLayouts", "Different pole types produce different port hint layout", "Invariant", false, test_different_pole_types_produce_different_port_hints);
  test_registry::AddTest(tests, "C287_CommunicationPole_DefaultBandOrder", "CommunicationPole keeps the Core-owned Japan distribution default band profile", "Invariant", false, test_communication_pole_default_band_order_places_japan_distribution_profile);
  test_registry::AddTest(tests, "C309_CommunicationPole_CommOpticalBandsCentered",
                         "CommunicationPole keeps communication and optical default bands centered instead of split left/right",
                         "Invariant", false, test_communication_pole_communication_and_optical_bands_are_centered);
  test_registry::AddTest(tests, "C313_CommunicationPole_DefaultCategoryPlacementValues",
                         "CommunicationPole default category placement values match viewer defaults",
                         "Invariant", false, test_communication_pole_default_category_placement_values_match_viewer_defaults);
  test_registry::AddTest(tests, "C310_DefaultTemplates_NoOpticalWithSupportBundle",
                         "Default templates do not register a split optical-with-support bundle type",
                         "Invariant", false, test_default_templates_do_not_register_optical_with_support_bundle);
  test_registry::AddTest(tests, "C289_UpdatePoleTypeDefinition_ReappliesExistingPorts", "Updating CommunicationPole port-band heights reapplies owned auto ports on existing poles", "Invariant", false, test_update_pole_type_definition_refreshes_existing_communication_ports);
  test_registry::AddTest(tests, "C296_DefaultPoleTypes_HVCategoryUsesSingleHeight",
                         "Default pole templates keep one HV category height so category-level editing does not average multiple defaults",
                         "Invariant", false, test_default_pole_types_use_single_hv_height_per_template);
  test_registry::AddTest(tests, "C783_CoreDefaults_OwnViewerStartupSemantics",
                         "Core defaults own sag, cable diameter, and pole profile values formerly patched by viewer startup",
                         "Invariant", false, test_core_defaults_own_viewer_startup_semantics);
  test_registry::AddTest(tests, "C297_BundleTemplate_RelatedPoleTypeRegeneratesGeneratedTopology",
                         "Bundle-linked pole template regenerates generated topology",
                         "Invariant", true, test_bundle_related_pole_type_regenerates_generated_topology);
}

WIRE_REGISTER_TEST_SUITE(register_editing_tests);

} // namespace
