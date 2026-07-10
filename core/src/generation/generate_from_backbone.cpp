#include "wire/core/core_state.hpp"
#include "backbone/pipeline.hpp"

#include <chrono>
#include <utility>

namespace wire::core {

EditResult<GenerateBundleFromPathResult> CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  const auto total_started = std::chrono::steady_clock::now();
  const auto copy_started = std::chrono::steady_clock::now();
  CoreState trial = *this;
  const double state_copy_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - copy_started).count();
  generation::backbone::pipeline pipeline(trial, spec);
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
  EditResult<GenerateBundleFromPathResult> out = pipeline.build(pipeline.build_input_from_spec());
  if (out.ok) {
    out.value.timing.state_copy_ms = state_copy_ms;
    out.value.timing.prepare_ms = prepare_ms;
    out.value.timing.check_ms = check_ms;
    out.value.timing.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_started).count();
    trial.debug_.last_generation_timing = out.value.timing;
    identity_ = std::move(trial.identity_);
    authoritative_ = std::move(trial.authoritative_);
    runtime_ = std::move(trial.runtime_);
    debug_ = std::move(trial.debug_);
  }
  return out;
}

} // namespace wire::core
