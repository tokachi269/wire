#include "wire/core/core_state.hpp"

namespace wire::core {

EditResult<SplitSpanResult> CoreState::SplitSpan(ObjectId span_id, double t) {
  EditResult<SplitSpanResult> result;
  (void)span_id;
  (void)t;
  result.error = "backbone unsupported: split span requires SavedBackboneGraph topology mutation";
  return result;
}

EditResult<AddConnectionByPoleResult>
CoreState::AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id, ConnectionCategory category) {
  return AddConnectionByPole(pole_a_id, pole_b_id, category, AddConnectionByPoleOptions{});
}

EditResult<AddConnectionByPoleResult>
CoreState::AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id, ConnectionCategory category,
                               const AddConnectionByPoleOptions& options) {
  EditResult<AddConnectionByPoleResult> result;
  (void)pole_a_id;
  (void)pole_b_id;
  (void)category;
  (void)options;
  result.error = "backbone unsupported: direct pole connection requires BackboneSpec generation";
  return result;
}

EditResult<AddDropResult>
CoreState::AddDropFromPole(ObjectId source_pole_id, const Vec3d& target_world_position,
                           ConnectionCategory category) {
  EditResult<AddDropResult> result;
  (void)source_pole_id;
  (void)target_world_position;
  (void)category;
  result.error = "backbone unsupported: drop creation requires BackboneSpec generation";
  return result;
}

EditResult<AddDropResult> CoreState::AddDropFromSpan(ObjectId source_span_id, double t,
                                                     const Vec3d& target_world_position,
                                                     ConnectionCategory category) {
  EditResult<AddDropResult> result;
  (void)source_span_id;
  (void)t;
  (void)target_world_position;
  (void)category;
  result.error = "backbone unsupported: span drop creation requires SavedBackboneGraph topology mutation";
  return result;
}

} // namespace wire::core
