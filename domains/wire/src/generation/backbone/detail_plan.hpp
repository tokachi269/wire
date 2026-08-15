#pragma once

#include "city/wire/core_state.hpp"

namespace city::wire::generation::backbone {

struct DetailVisuals {
  VisualCurvePartCache curves{};
  VisualModelInstanceCache models{};
};

// Derived support/inline visual detail. This never creates Span, Bundle, Port, or
// SavedBackboneGraph entities; callers merge the returned visuals into runtime caches.
[[nodiscard]] DetailVisuals make_detail_visuals(const CoreState& state,
                                                const VisualCurvePartCache& carriers);

} // namespace city::wire::generation::backbone
