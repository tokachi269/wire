#pragma once

#include "city/wire/core_state.hpp"

namespace city::wire::detail {

EditResult<bool> validate_route_bundle_variation_descriptor(
    const CoreState& state, const RouteBundleVariationInput& input);

} // namespace city::wire::detail
