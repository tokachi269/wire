#include "wire/core/core_state.hpp"
#include "bb2/pipeline.hpp"

#include <chrono>

namespace wire::core {

EditResult<GenerateBundleFromPathResult> CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  const auto total_started = std::chrono::steady_clock::now();
  generation::bb2::pipeline pipeline(*this, spec);
  const auto prepare_started = std::chrono::steady_clock::now();
  EditResult<bool> prepare_out = pipeline.prepare();
  const double prepare_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - prepare_started).count();
  if (!prepare_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = prepare_out.error;
    return out;
  }
  const auto check_started = std::chrono::steady_clock::now();
  EditResult<bool> check_out = pipeline.check();
  const double check_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - check_started).count();
  if (!check_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = check_out.error;
    return out;
  }
  EditResult<GenerateBundleFromPathResult> out = pipeline.build();
  if (out.ok) {
    out.value.timing.prepare_ms = prepare_ms;
    out.value.timing.check_ms = check_ms;
    out.value.timing.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_started).count();
  }
  return out;
}

} // namespace wire::core
