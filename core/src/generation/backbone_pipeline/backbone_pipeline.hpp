#pragma once

#include "wire/core/core_state.hpp"
#include "../build_backbone/build_backbone_types.hpp"
#include "../pole_facing/pole_facing_rules.hpp"
#include "wire/core/support_layout_types.hpp"

#include <array>
#include <unordered_map>

namespace wire::core::generation::detail {

enum class InputDirection : std::uint8_t {
  kForward = 0,
  kReverse = 1,
};

enum class BuildDirection : std::uint8_t {
  kForward = 0,
  kReverse = 1,
};

struct BackbonePair {
  ObjectId low = kInvalidObjectId;
  ObjectId high = kInvalidObjectId;

  [[nodiscard]] bool valid() const {
    return low != kInvalidObjectId && high != kInvalidObjectId && low != high;
  }
};

struct BackboneGraph {
  std::vector<SupportNode> nodes{};
  std::vector<BackboneEdge> edges{};
  std::vector<JunctionInfo> junctions{};
  InputDirection input_direction = InputDirection::kForward;
  BuildDirection build_direction = BuildDirection::kForward;
};

struct JunctionInputIncident {
  ObjectId neighbor_node_id = kInvalidObjectId;
  Vec3d direction{};
};

struct JunctionInputFacts {
  ObjectId node_id = kInvalidObjectId;
  BuildDirection build_direction = BuildDirection::kForward;
  Vec3d support_position{};
  std::vector<JunctionInputIncident> route_incidents{};
  std::vector<JunctionInputIncident> external_incidents{};
};

struct JunctionPairs {
  std::vector<EdgeFlowInfo> edge_flow_by_segment{};
  std::vector<JunctionRelation> ordered_relations{};
  std::unordered_map<ObjectId, JunctionRelation> relation_by_node{};
  std::unordered_map<ObjectId, BackbonePair> through_pair_by_node{};
  std::unordered_map<ObjectId, BackbonePair> cross_pair_by_node{};
  std::unordered_map<ObjectId, Vec3d> junction_axis_by_node{};
};

enum class JunctionLevelRuleKind : std::uint8_t {
  kSameLevelAllowed = 0,
  kMustLower = 1,
};

struct JunctionLevelRule {
  ObjectId node_id = kInvalidObjectId;
  JunctionLevelRuleKind kind = JunctionLevelRuleKind::kSameLevelAllowed;
  bool same_level_allowed = true;
  bool must_lower = false;
};

struct JunctionLevelRules {
  std::unordered_map<ObjectId, JunctionLevelRule> by_node{};
};

struct PoleFacing {
  std::unordered_map<ObjectId, BackbonePlannedPoleOrientation> by_node{};
};

struct RealizedBackboneSupport {
  std::uint64_t session_id = 0;
  ChangeSet change_set{};
  std::vector<ObjectId> generated_pole_ids{};
  std::vector<ObjectId> ordered_support_node_ids{};
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  std::unordered_map<ObjectId, ObjectId> real_node_id_by_input_node_id{};
};

struct BackboneRuntimeTopology {
  BackboneResult generation_backbone{};
  std::unordered_map<ObjectId, std::vector<ObjectId>> route_neighbors_by_node{};
  std::unordered_map<ObjectId, Vec3d> node_position_by_id{};
  std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
};

struct BackboneRuntimeState {
  std::uint64_t session_id = 0;
  BuildDirection build_direction = BuildDirection::kForward;
  std::vector<ObjectId> ordered_support_node_ids{};
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  std::unordered_map<ObjectId, ObjectId> real_node_id_by_input_node_id{};
  BackboneRuntimeTopology topology{};
  JunctionPairs junction_pairs{};
  JunctionLevelRules junction_level_rules{};
  PoleFacing pole_facing{};
  std::unordered_map<ObjectId, Vec3d> node_side_axis_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, double>> node_side_sign_by_peer{};
  std::unordered_map<ObjectId, std::array<ObjectId, 2>> through_pair_by_node{};
  std::unordered_map<ObjectId, std::array<ObjectId, 2>> cross_pair_by_node{};
};

struct GeneratedBackboneSpans {
  ChangeSet change_set{};
  ObjectId primary_bundle_id = kInvalidObjectId;
  std::vector<ObjectId> bundle_ids{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::unordered_map<ObjectId, JunctionRelation> junctions{};
  SpanLayoutRules layout_rules{};
};

struct BackboneBuilderOutput {
  BackboneGenerationRequestPlan request{};
  BackboneSupportChainPlan support_chain{};
  BackboneGraph backbone{};
};

struct JunctionInputBuilderOutput {
  BackboneTopologyPlan topology{};
  std::vector<JunctionInputFacts> ordered{};
  std::unordered_map<ObjectId, JunctionInputFacts> by_node{};
};

struct JunctionPairResolverOutput {
  BackboneTopologyPlan topology{};
  JunctionPairs pairs{};
};

struct JunctionLevelResolverOutput {
  JunctionLevelRules level_rules{};
};

class BackboneBuilder {
public:
  explicit BackboneBuilder(const CoreState& state) : state_(state) {}

  [[nodiscard]] EditResult<BackboneBuilderOutput> build(const BackboneSpec& spec) const;

private:
  const CoreState& state_;
};

class JunctionInputBuilder {
public:
  explicit JunctionInputBuilder(const CoreState& state) : state_(state) {}

  [[nodiscard]] EditResult<JunctionInputBuilderOutput> build(const BackboneBuilderOutput& builder_output) const;

private:
  const CoreState& state_;
};

class JunctionPairResolver {
public:
  [[nodiscard]] EditResult<JunctionPairResolverOutput> resolve(const BackboneBuilderOutput& builder_output,
                                                               const JunctionInputBuilderOutput& input_output) const;
};

class JunctionLevelResolver {
public:
  [[nodiscard]] EditResult<JunctionLevelResolverOutput> resolve(const JunctionInputBuilderOutput& input_output,
                                                                const JunctionPairResolverOutput& pair_output,
                                                                const BackboneGenerationRequestPlan& request) const;
};

class PoleFacingResolver {
public:
  explicit PoleFacingResolver(const CoreState& state) : state_(state) {}

  [[nodiscard]] EditResult<PoleFacing> resolve(const BackboneBuilderOutput& builder_output,
                                               const JunctionPairResolverOutput& pair_output) const;

private:
  const CoreState& state_;
};

struct BundleSpanBuilderOutput {
  BackboneRuntimeState runtime{};
  GeneratedBackboneSpans spans{};
  std::vector<ObjectId> generated_pole_ids{};
  ChangeSet change_set{};
};

class BundleSpanBuilder {
public:
  explicit BundleSpanBuilder(CoreState& state) : state_(state) {}

  [[nodiscard]] EditResult<BundleSpanBuilderOutput> build(const BackboneBuilderOutput& builder_output,
                                                          const JunctionPairResolverOutput& pair_output,
                                                          const JunctionLevelResolverOutput& level_output,
                                                          const PoleFacing& pole_facing);

private:
  CoreState& state_;
};

class SpanLayoutRuleBuilder {
public:
  explicit SpanLayoutRuleBuilder(const CoreState& state) : state_(state) {}

  [[nodiscard]] SpanLayoutRules build(const GeneratedBackboneSpans& spans) const;

private:
  const CoreState& state_;
};

class BackbonePipeline {
public:
  BackbonePipeline(CoreState& state, const BackboneSpec& spec) : state_(state), spec_(spec) {}

  [[nodiscard]] EditResult<bool> prepare();
  [[nodiscard]] EditResult<bool> check() const;
  [[nodiscard]] EditResult<GenerateBundleFromPathResult> build();

private:
  CoreState& state_;
  const BackboneSpec& spec_;
  bool prepared_ = false;
  BackboneBuilderOutput builder_output_{};
  JunctionInputBuilderOutput junction_input_{};
  JunctionPairResolverOutput junction_pairs_{};
  JunctionLevelResolverOutput junction_levels_{};
  PoleFacing pole_facing_{};
};

} // namespace wire::core::generation::detail
