#include "wire/core/core_state.hpp"
#include "build_backbone/build_backbone_types.hpp"

namespace wire::core {

EditResult<GenerateBundleFromPathResult> CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  EditResult<std::unique_ptr<generation::detail::BackboneBuildDraft>> draft_out = prepare_backbone_build(spec);
  if (!draft_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = draft_out.error;
    return out;
  }

  EditResult<bool> check_out = check_backbone_build_input(*draft_out.value);
  if (!check_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = check_out.error;
    return out;
  }

  return build_backbone_from_draft(std::move(draft_out.value));
}

} // namespace wire::core
