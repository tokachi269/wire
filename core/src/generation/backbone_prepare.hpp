#pragma once

#include "wire/core/core_state_api_types.hpp"
#include "wire/core/workflow_types.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace wire::core::generation::detail {

struct SupportNodeCandidate {
  Vec3d world{};
  std::size_t segment_index = 0;
  int vertex_index = -1;
  double t = 0.0;
  PlacementMode mode = PlacementMode::kAuto;
  SupportKind support_kind = SupportKind::kPole;
  bool has_tangent_hint = false;
  Vec3d tangent_hint{};
};

using NodeSpecByIndex = std::unordered_map<std::size_t, BackboneInputSpec::NodeSpec>;
using NodeBundleModeByPoint = std::unordered_map<std::size_t, std::unordered_map<BundleKind, BundleNodeMode>>;

bool build_backbone_node_maps(const BackboneSpec& request, NodeSpecByIndex* out_node_spec_by_index,
                              NodeBundleModeByPoint* out_node_bundle_mode_by_point, std::string* error);
void build_backbone_guide_points(const BackboneSpec& request, std::vector<Vec3d>* out_guide_points,
                                 PathDirectionEvaluationDebug* out_direction_debug);
bool build_backbone_candidates(const BackboneSpec& request, const std::vector<Vec3d>& guide_points,
                               const NodeSpecByIndex& node_spec_by_index, std::vector<SupportNodeCandidate>* out_candidates,
                               std::string* error);

} // namespace wire::core::generation::detail
