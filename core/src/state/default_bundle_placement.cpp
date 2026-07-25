#include "wire/core/core_state.hpp"

#include "../generation/backbone/emit_shared.hpp"

#include <numeric>

namespace wire::core {

EditResult<DefaultBundlePlacementResult>
CoreState::ResolveDefaultBundlePlacement(BundleTemplateId bundle_template_id, PoleTypeId pole_type_id,
                                         int count) const {
  EditResult<DefaultBundlePlacementResult> result{};
  const BundleTemplate* bundle_template = find_bundle_template(bundle_template_id);
  if (bundle_template == nullptr) {
    result.error = "core invalid input: bundle template is missing";
    return result;
  }
  const PoleTypeDefinition* pole_type = find_pole_type(pole_type_id);
  if (pole_type == nullptr) {
    result.error = "core invalid input: pole template is missing";
    return result;
  }

  const int lane_count = bundle_template->count_rule == BundleCountRuleKind::kFixed
      ? bundle_template->fixed_count
      : (count > 0 ? count : bundle_template->default_count);
  const auto bands = generation::backbone::SelectPortPlacementBands(
      *pole_type, bundle_template->category, bundle_template->default_layer, lane_count);
  if (!bands.ok) {
    result.error = bands.error;
    return result;
  }

  const double divisor = static_cast<double>(std::max<std::size_t>(1, bands.value.size()));
  const double height = std::accumulate(bands.value.begin(), bands.value.end(), 0.0,
                                        [](double sum, const PortPlacementBand& band) {
                                          return sum + band.height_center_m;
                                        }) / divisor;
  const double lateral = std::accumulate(bands.value.begin(), bands.value.end(), 0.0,
                                         [](double sum, const PortPlacementBand& band) {
                                           return sum + band.lateral_center_m;
                                         }) / divisor;
  result.value.height_m = height;
  result.value.lateral_m = lateral;
  result.value.spacing_m = bundle_template->default_spacing_m;
  result.ok = true;
  return result;
}

} // namespace wire::core
