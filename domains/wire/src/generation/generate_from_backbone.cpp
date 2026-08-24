#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"
#include "backbone/pipeline.hpp"

#include <chrono>
#include <algorithm>
#include <unordered_set>
#include <utility>

namespace city::wire {

namespace {

std::unordered_set<ObjectId> referenced_pending_node_ids(const BackboneSpec& spec,
                                                         const CoreState& state) {
  std::unordered_set<ObjectId> ids{};
  for (const BackboneInputSpec::NodeSpec& node_spec : spec.path.node_specs) {
    if (node_spec.node_id == kInvalidObjectId) {
      continue;
    }
    if (state.view().pending_support_node(node_spec.node_id) != nullptr) {
      ids.insert(node_spec.node_id);
    }
  }
  return ids;
}

} // namespace

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
    out.failure_category = prepare_out.effective_failure_category();
    out.reason_code = prepare_out.reason_code.empty()
                          ? CommitFailureReasonCode(out.error, out.failure_category)
                          : prepare_out.reason_code;
    return out;
  }
  const auto check_started = std::chrono::steady_clock::now();
  EditResult<bool> check_out = pipeline.check();
  const double check_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - check_started).count();
  if (!check_out.ok) {
    EditResult<GenerateBundleFromPathResult> out{};
    out.error = check_out.error;
    out.failure_category = check_out.effective_failure_category();
    out.reason_code = check_out.reason_code.empty()
                          ? CommitFailureReasonCode(out.error, out.failure_category)
                          : check_out.reason_code;
    return out;
  }
  EditResult<GenerateBundleFromPathResult> out = pipeline.build(pipeline.build_input_from_spec());
  if (out.ok) {
    const std::unordered_set<ObjectId> consumed_pending =
        referenced_pending_node_ids(spec, trial);
    if (!consumed_pending.empty()) {
      auto& pending = trial.session_.pending_support_nodes;
      pending.erase(std::remove_if(pending.begin(), pending.end(),
                                   [&](const SupportNode& node) {
                                     return consumed_pending.contains(node.node_id);
                                   }),
                    pending.end());
    }
    out.value.timing.state_copy_ms = state_copy_ms;
    out.value.timing.prepare_ms = prepare_ms;
    out.value.timing.check_ms = check_ms;
    out.value.timing.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_started).count();
    trial.debug_.last_generation_timing = out.value.timing;
    identity_ = std::move(trial.identity_);
    authoritative_ = std::move(trial.authoritative_);
    runtime_ = std::move(trial.runtime_);
    session_ = std::move(trial.session_);
    debug_ = std::move(trial.debug_);
  } else {
    out.classify_error();
  }
  return out;
}

} // namespace city::wire
