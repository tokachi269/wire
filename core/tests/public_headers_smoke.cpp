#include "wire/core/coord_utils.hpp"
#include "wire/core/debug_types.hpp"
#include "wire/core/detail_curve.hpp"
#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/inspection.hpp"
#include "wire/core/object_store.hpp"
#include "wire/core/types.hpp"
#include "wire/core/variation.hpp"
#include "wire/core/workflow_types.hpp"
#include "wire/core/core_state.hpp"

#include <type_traits>
#include <utility>

namespace {

using wire::core::AddConnectionByPoleOptions;
using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::EntityRef;
using wire::core::PickResult;
using wire::core::ResolveBranchPickOptions;
using wire::core::VariationSettings;

static_assert(std::is_default_constructible_v<AddConnectionByPoleOptions>);
static_assert(std::is_default_constructible_v<ResolveBranchPickOptions>);
static_assert(std::is_default_constructible_v<VariationSettings>);
static_assert(std::is_trivially_copyable_v<EntityRef>);

void smoke_public_headers(CoreState& state, const PickResult& pick) {
  [[maybe_unused]] const auto add_default =
      state.AddConnectionByPole(wire::core::kInvalidObjectId, wire::core::kInvalidObjectId,
                                ConnectionCategory::kLowVoltage);
  [[maybe_unused]] const auto add_with_options =
      state.AddConnectionByPole(wire::core::kInvalidObjectId, wire::core::kInvalidObjectId,
                                ConnectionCategory::kLowVoltage, AddConnectionByPoleOptions{});
  [[maybe_unused]] const auto resolve_default = state.ResolveBranchPick(pick);
  [[maybe_unused]] const auto resolve_with_options = state.ResolveBranchPick(pick, ResolveBranchPickOptions{});
}

} // namespace
