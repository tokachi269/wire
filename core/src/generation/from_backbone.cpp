#include "wire/core/core_state.hpp"
#include "backbone_generation_plan_internal.hpp"

namespace wire::core {

EditResult<GenerateBundleFromPathResult> CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  EditResult<std::unique_ptr<generation::detail::BackboneGenerationPlan>> plan_result =
      build_backbone_generation_plan(spec);
  if (!plan_result.ok) {
    EditResult<GenerateBundleFromPathResult> result{};
    result.error = plan_result.error;
    return result;
  }

  EditResult<bool> validate_result = validate_backbone_generation_plan(*plan_result.value);
  if (!validate_result.ok) {
    EditResult<GenerateBundleFromPathResult> result{};
    result.error = validate_result.error;
    return result;
  }

  return commit_backbone_generation_plan(std::move(plan_result.value));
}

} // namespace wire::core
