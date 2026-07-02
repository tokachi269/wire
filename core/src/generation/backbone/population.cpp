#include "population.hpp"

#include "out.hpp"

#include "wire/core/core_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace wire::core::generation::backbone {
namespace {

constexpr double kEps = 1e-9;
constexpr std::size_t kCandidateAttempts = 32;

std::uint64_t mix64(std::uint64_t value) {
  value += 0x9E3779B97F4A7C15ull;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
  return value ^ (value >> 31);
}

std::uint64_t combine(std::uint64_t seed, std::uint64_t value) {
  return mix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

std::uint64_t key_seed(const PhysicalLinePopulationInput& input, std::size_t instance_index,
                       std::size_t attempt) {
  std::uint64_t seed = mix64(input.explicit_seed);
  seed = combine(seed, input.key.logical_span_id);
  seed = combine(seed, input.key.edge_bundle_id);
  seed = combine(seed, input.key.rule_owner_id);
  seed = combine(seed, input.key.rule_id);
  seed = combine(seed, instance_index);
  return combine(seed, attempt);
}

double unit(std::uint64_t value) {
  constexpr double kInv = 1.0 / static_cast<double>(1ull << 53);
  return static_cast<double>((mix64(value) >> 11) & ((1ull << 53) - 1ull)) * kInv;
}

double centered(std::uint64_t value) { return unit(value) * 2.0 - 1.0; }

bool finite_range(double min_value, double max_value) {
  return std::isfinite(min_value) && std::isfinite(max_value) && min_value <= max_value;
}

bool inside(double value, double min_value, double max_value) {
  return value >= min_value - kEps && value <= max_value + kEps;
}

bool blocked_by_reserve(const PhysicalLinePopulationEndpoint& endpoint,
                        const std::vector<ExperimentalPlacementReserve>& reserves, const Vec3d& local) {
  return std::any_of(reserves.begin(), reserves.end(), [&](const ExperimentalPlacementReserve& reserve) {
    return reserve.pole_type_id == endpoint.pole_type_id && reserve.band_id == endpoint.band_id &&
           inside(local.y, reserve.lateral_min_m, reserve.lateral_max_m) &&
           inside(local.z, reserve.height_min_m, reserve.height_max_m);
  });
}

bool spacing_satisfied(const Vec3d& candidate, const std::vector<Vec3d>& occupied, double min_spacing_m) {
  const double required2 = min_spacing_m * min_spacing_m;
  return std::all_of(occupied.begin(), occupied.end(), [&](const Vec3d& existing) {
    const double dy = candidate.y - existing.y;
    const double dz = candidate.z - existing.z;
    return dy * dy + dz * dz + kEps >= required2;
  });
}

int requested_count(const PhysicalLinePopulationInput& input) {
  const int count_range = input.rule.max_extra_count - input.rule.min_extra_count + 1;
  if (count_range <= 1) {
    return input.rule.min_extra_count;
  }
  std::uint64_t seed = key_seed(input, std::numeric_limits<std::size_t>::max(), 0);
  return input.rule.min_extra_count + static_cast<int>(seed % static_cast<std::uint64_t>(count_range));
}

Vec3d candidate_local(const PhysicalLinePopulationEndpoint& endpoint, const ExperimentalPhysicalLineRule& rule,
                      std::size_t instance_index, std::size_t attempt, std::uint64_t seed) {
  const std::size_t ordinal = instance_index + attempt;
  const double side = ((ordinal & 1u) == 0u) ? 1.0 : -1.0;
  const double ring = static_cast<double>((ordinal / 2u) + 1u);
  const double jitter = 1.0 + centered(combine(seed, 1)) * rule.randomness * 0.35;
  const double lateral_delta = side * std::max(rule.min_spacing_m, 0.01) * ring * jitter;
  const double height_delta =
      centered(combine(seed, 2)) * std::max(rule.min_spacing_m, 0.01) * rule.randomness * 0.75;
  return {endpoint.original_local.x, endpoint.original_local.y + lateral_delta,
          endpoint.original_local.z + height_delta};
}

bool candidate_allowed(const PhysicalLinePopulationEndpoint& endpoint,
                       const std::vector<ExperimentalPlacementReserve>& reserves,
                       const std::vector<Vec3d>& occupied, double min_spacing_m, const Vec3d& local) {
  return inside(local.y, endpoint.lateral_min_m, endpoint.lateral_max_m) &&
         inside(local.z, endpoint.height_min_m, endpoint.height_max_m) &&
         !blocked_by_reserve(endpoint, reserves, local) && spacing_satisfied(local, occupied, min_spacing_m);
}

const SavedBackboneSpanBinding* span_binding_for(const CoreState& state, ObjectId span_id) {
  for (const SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.span_id == span_id) {
      return &binding;
    }
  }
  return nullptr;
}

const ExperimentalPhysicalLineRule* rule_for(const ExperimentalLinePopulationConfig& config,
                                             BundleKind bundle_template_id, std::size_t index) {
  std::vector<const ExperimentalPhysicalLineRule*> matches{};
  for (const ExperimentalPhysicalLineRule& rule : config.rules) {
    if (rule.bundle_template_id == bundle_template_id) {
      matches.push_back(&rule);
    }
  }
  std::sort(matches.begin(), matches.end(), [](const auto* lhs, const auto* rhs) {
    if (lhs->priority != rhs->priority) {
      return lhs->priority > rhs->priority;
    }
    return lhs->rule_id < rhs->rule_id;
  });
  return index < matches.size() ? matches[index] : nullptr;
}

PhysicalLinePopulationEndpoint resolve_endpoint(const CoreState& state, const LayoutEndpoint& layout,
                                                const BundleTemplate& bundle_template,
                                                const ExperimentalPhysicalLineRule& rule) {
  PhysicalLinePopulationEndpoint out{};
  const Port* port = state.view().ports().find(layout.port_id);
  if (port == nullptr || port->owner_pole_id == kInvalidObjectId) {
    out.failure_reason = "endpoint has no pole-owned port";
    return out;
  }
  const Pole* pole = state.view().poles().find(port->owner_pole_id);
  if (pole == nullptr || pole->pole_type_id == kInvalidPoleTypeId) {
    out.failure_reason = "endpoint pole type is missing";
    return out;
  }
  const auto pole_type_it = state.view().pole_types().find(pole->pole_type_id);
  if (pole_type_it == state.view().pole_types().end()) {
    out.failure_reason = "endpoint pole type is missing";
    return out;
  }
  if (has_duplicate_band_ids(pole_type_it->second)) {
    out.failure_reason = "duplicate band_id in endpoint pole type";
    return out;
  }

  out.pole_type_id = pole->pole_type_id;
  out.frame =
      BuildPoleFrame(pole->world_transform, state.effective_port_layout_yaw_deg(*pole, bundle_template.category));
  out.original_local = WorldPointToLocal(out.frame, layout.support_world);
  out.endpoint_offset_world = layout.endpoint_world - layout.support_world;

  const PortPlacementBand* selected = nullptr;
  int selected_identity_score = std::numeric_limits<int>::min();
  double selected_distance2 = std::numeric_limits<double>::max();
  for (const PortPlacementBand& band : pole_type_it->second.port_bands) {
    if (!band.enabled || band.category != bundle_template.category || band.layer != port->template_layer) {
      continue;
    }
    int identity_score = 0;
    if (band.side == port->template_side) {
      identity_score += 2;
    }
    if (port->template_role != SlotRole::kNeutral && band.role == port->template_role) {
      identity_score += 1;
    }
    const double dy = out.original_local.y - band.lateral_center_m;
    const double dz = out.original_local.z - band.height_center_m;
    const double distance2 = dy * dy + dz * dz;
    if (selected == nullptr || identity_score > selected_identity_score ||
        (identity_score == selected_identity_score && distance2 < selected_distance2 - kEps) ||
        (identity_score == selected_identity_score && std::abs(distance2 - selected_distance2) <= kEps &&
         (band.priority > selected->priority ||
          (band.priority == selected->priority && band.band_id < selected->band_id)))) {
      selected = &band;
      selected_identity_score = identity_score;
      selected_distance2 = distance2;
    }
  }
  if (selected == nullptr) {
    out.failure_reason = "no placement band matches endpoint template identity";
    return out;
  }

  out.band_id = selected->band_id;
  const double effective_lateral_min =
      out.original_local.y + (selected->lateral_min_m - selected->lateral_center_m);
  const double effective_lateral_max =
      out.original_local.y + (selected->lateral_max_m - selected->lateral_center_m);
  const double effective_height_min =
      out.original_local.z + (selected->height_min_m - selected->height_center_m);
  const double effective_height_max =
      out.original_local.z + (selected->height_max_m - selected->height_center_m);
  out.lateral_min_m = std::max(effective_lateral_min, rule.lateral_min_m);
  out.lateral_max_m = std::min(effective_lateral_max, rule.lateral_max_m);
  out.height_min_m = std::max(effective_height_min, rule.height_min_m);
  out.height_max_m = std::min(effective_height_max, rule.height_max_m);
  if (!finite_range(out.lateral_min_m, out.lateral_max_m) ||
      !finite_range(out.height_min_m, out.height_max_m)) {
    out.failure_reason = "rule range does not overlap endpoint band";
    return out;
  }
  out.valid = true;
  return out;
}

void apply_appearance(const CoreState& state, ObjectId span_id, VisualCurvePart* part) {
  if (part == nullptr) {
    return;
  }
  const SpanRenderCacheEntry appearance = render(state, span_id, {});
  part->wire_radius_m = appearance.wire_radius_m;
  part->color_rgba = appearance.color_rgba;
  part->material_style = appearance.material_style;
}

} // namespace

bool has_duplicate_band_ids(const PoleTypeDefinition& pole_type) {
  std::unordered_set<int> ids{};
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (!ids.insert(band.band_id).second) {
      return true;
    }
  }
  return false;
}

EditResult<PhysicalLinePopulationOutput> populate_physical_lines(const PhysicalLinePopulationInput& input) {
  EditResult<PhysicalLinePopulationOutput> out{};
  PhysicalLinePopulationDiagnostic& diagnostic = out.value.diagnostic;
  diagnostic.logical_span_id = input.key.logical_span_id;
  diagnostic.edge_bundle_id = input.key.edge_bundle_id;
  diagnostic.rule_id = input.key.rule_id;
  diagnostic.extra_count_requested = requested_count(input);

  if (!input.endpoint_a.valid || !input.endpoint_b.valid) {
    diagnostic.omitted_count = diagnostic.extra_count_requested;
    diagnostic.reason = !input.endpoint_a.valid ? input.endpoint_a.failure_reason : input.endpoint_b.failure_reason;
    out.ok = true;
    return out;
  }

  std::vector<Vec3d> occupied_a = input.occupied_a_local;
  std::vector<Vec3d> occupied_b = input.occupied_b_local;
  for (int index = 0; index < diagnostic.extra_count_requested; ++index) {
    bool accepted = false;
    for (std::size_t attempt = 0; attempt < kCandidateAttempts; ++attempt) {
      const std::uint64_t seed = key_seed(input, static_cast<std::size_t>(index), attempt);
      const Vec3d local_a =
          candidate_local(input.endpoint_a, input.rule, static_cast<std::size_t>(index), attempt, seed);
      const Vec3d local_b =
          candidate_local(input.endpoint_b, input.rule, static_cast<std::size_t>(index), attempt, seed);
      if (!candidate_allowed(input.endpoint_a, input.reserves, occupied_a, input.rule.min_spacing_m, local_a) ||
          !candidate_allowed(input.endpoint_b, input.reserves, occupied_b, input.rule.min_spacing_m, local_b)) {
        continue;
      }

      PhysicalLineInstance instance{};
      instance.key = input.key;
      instance.key.instance_index = static_cast<std::size_t>(index);
      instance.endpoint_a = LocalPointToWorld(input.endpoint_a.frame, local_a) + input.endpoint_a.endpoint_offset_world;
      instance.endpoint_b = LocalPointToWorld(input.endpoint_b.frame, local_b) + input.endpoint_b.endpoint_offset_world;
      instance.endpoint_a_pole_type_id = input.endpoint_a.pole_type_id;
      instance.endpoint_b_pole_type_id = input.endpoint_b.pole_type_id;
      instance.endpoint_a_band_id = input.endpoint_a.band_id;
      instance.endpoint_b_band_id = input.endpoint_b.band_id;
      out.value.instances.push_back(instance);
      occupied_a.push_back(local_a);
      occupied_b.push_back(local_b);
      accepted = true;
      break;
    }
    if (!accepted) {
      ++diagnostic.omitted_count;
    }
  }
  diagnostic.extra_count_accepted = static_cast<int>(out.value.instances.size());
  diagnostic.reason = diagnostic.omitted_count == 0 ? "ok" : "candidate conflict";
  out.ok = true;
  return out;
}

void append_experimental_physical_lines(const CoreState& state, const std::vector<SpanLayoutEntry>& layouts,
                                        VisualCurvePartCache* output) {
  if (output == nullptr) {
    return;
  }
  const ExperimentalLinePopulationConfig& config = state.view().experimental_line_population_config();
  if (!config.enabled) {
    return;
  }

  for (const SpanLayoutEntry& layout : layouts) {
    const Span* span = state.view().spans().find(layout.span_id);
    const SavedBackboneSpanBinding* binding = span_binding_for(state, layout.span_id);
    if (span == nullptr || binding == nullptr) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      continue;
    }
    const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
    if (bundle_template_it == state.view().bundle_templates().end()) {
      continue;
    }

    std::vector<Vec3d> occupied_a{};
    std::vector<Vec3d> occupied_b{};
    std::size_t rule_index = 0;
    while (const ExperimentalPhysicalLineRule* rule =
               rule_for(config, bundle->bundle_template_id, rule_index++)) {
      PhysicalLinePopulationInput input{};
      input.key.logical_span_id = layout.span_id;
      input.key.edge_bundle_id = binding->edge_bundle_id;
      input.key.rule_owner_id = static_cast<std::uint64_t>(bundle->bundle_template_id);
      input.key.rule_id = rule->rule_id;
      input.rule = *rule;
      input.explicit_seed = config.explicit_seed;
      input.endpoint_a = resolve_endpoint(state, layout.start, bundle_template_it->second, *rule);
      input.endpoint_b = resolve_endpoint(state, layout.end, bundle_template_it->second, *rule);
      input.reserves = config.reserves;
      input.occupied_a_local = occupied_a;
      input.occupied_b_local = occupied_b;
      if (input.endpoint_a.valid) {
        input.occupied_a_local.push_back(input.endpoint_a.original_local);
      }
      if (input.endpoint_b.valid) {
        input.occupied_b_local.push_back(input.endpoint_b.original_local);
      }

      EditResult<PhysicalLinePopulationOutput> populated = populate_physical_lines(input);
      if (!populated.ok) {
        PhysicalLinePopulationDiagnostic diagnostic{};
        diagnostic.logical_span_id = layout.span_id;
        diagnostic.edge_bundle_id = binding->edge_bundle_id;
        diagnostic.rule_id = rule->rule_id;
        diagnostic.reason = populated.error;
        output->experimental_population_diagnostics.push_back(std::move(diagnostic));
        continue;
      }
      PhysicalLinePopulationDiagnostic diagnostic = populated.value.diagnostic;
      std::vector<PhysicalLineInstance> rendered_instances{};
      for (const PhysicalLineInstance& instance : populated.value.instances) {
        EditResult<DetailCurve> curve =
            make_curve_between(state, layout.span_id, instance.endpoint_a, instance.endpoint_b);
        if (!curve.ok) {
          diagnostic.reason = "curve generation failed";
          continue;
        }
        VisualCurvePart part{};
        part.kind = VisualCurvePartKind::kExperimentalPhysicalLine;
        part.source_span_id = layout.span_id;
        part.source_bundle_id = span->bundle_id;
        part.bundle_template_id = bundle->bundle_template_id;
        part.lane_index = binding->lane_index;
        part.boundary_a = instance.endpoint_a;
        part.boundary_b = instance.endpoint_b;
        part.samples = curve.value.sample_points;
        part.sag_method = VisualCurveSagMethod::kParabolic;
        part.sag_m = curve.value.sag_amplitude_m;
        part.has_physical_line_key = true;
        part.physical_line_key = instance.key;
        part.endpoint_a_pole_type_id = instance.endpoint_a_pole_type_id;
        part.endpoint_b_pole_type_id = instance.endpoint_b_pole_type_id;
        part.endpoint_a_band_id = instance.endpoint_a_band_id;
        part.endpoint_b_band_id = instance.endpoint_b_band_id;
        apply_appearance(state, layout.span_id, &part);
        part.bounds = bounds(curve.value).whole;
        output->parts.push_back(std::move(part));
        rendered_instances.push_back(instance);
      }

      diagnostic.extra_count_accepted = static_cast<int>(rendered_instances.size());
      diagnostic.omitted_count = diagnostic.extra_count_requested - diagnostic.extra_count_accepted;
      if (diagnostic.omitted_count == 0) {
        diagnostic.reason = "ok";
      } else if (diagnostic.reason == "ok") {
        diagnostic.reason = "candidate conflict";
      }
      output->experimental_population_diagnostics.push_back(std::move(diagnostic));
      for (const PhysicalLineInstance& instance : rendered_instances) {
        occupied_a.push_back(WorldPointToLocal(input.endpoint_a.frame,
                                              instance.endpoint_a - input.endpoint_a.endpoint_offset_world));
        occupied_b.push_back(WorldPointToLocal(input.endpoint_b.frame,
                                              instance.endpoint_b - input.endpoint_b.endpoint_offset_world));
      }
    }
  }
}

} // namespace wire::core::generation::backbone
