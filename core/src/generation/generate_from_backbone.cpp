#include "wire/core/core_state.hpp"
#include "bb2/pipeline.hpp"

namespace wire::core {

EditResult<GenerateBundleFromPathResult> CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  generation::bb2::pipeline pipeline(*this, spec);
  EditResult<bool> prepare_out = pipeline.prepare();
  if (!prepare_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = prepare_out.error;
    return out;
  }
  EditResult<bool> check_out = pipeline.check();
  if (!check_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = check_out.error;
    return out;
  }
  return pipeline.build();
}

} // namespace wire::core
