#include "read.hpp"

#include "geometry.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::build {

const RoadNode *find_node(const SavedRoadGraph &graph, RoadNodeId id) {
  const auto found =
      std::find_if(graph.nodes.begin(), graph.nodes.end(),
                   [id](const RoadNode &node) { return node.id == id; });
  return found == graph.nodes.end() ? nullptr : &*found;
}

const RoadSegment *find_segment(const SavedRoadGraph &graph, RoadSegmentId id) {
  const auto found = std::find_if(
      graph.segments.begin(), graph.segments.end(),
      [id](const RoadSegment &segment) { return segment.id == id; });
  return found == graph.segments.end() ? nullptr : &*found;
}

const CrossSectionTemplate *find_template(const SavedRoadGraph &graph,
                                          CrossSectionTemplateId id) {
  const auto found = std::find_if(
      graph.section_templates.begin(), graph.section_templates.end(),
      [id](const CrossSectionTemplate &section) { return section.id == id; });
  return found == graph.section_templates.end() ? nullptr : &*found;
}

const SectionTransition *find_transition(const SavedRoadGraph &graph,
                                         SectionTransitionId id) {
  const auto found =
      std::find_if(graph.transitions.begin(), graph.transitions.end(),
                   [id](const SectionTransition &transition) {
                     return transition.id == id;
                   });
  return found == graph.transitions.end() ? nullptr : &*found;
}

const NodeConnectionPolicyOverride *
find_policy_override(const SavedRoadGraph &graph, RoadNodeId node_id) {
  const auto found =
      std::find_if(graph.connection_policy_overrides.begin(),
                   graph.connection_policy_overrides.end(),
                   [node_id](const NodeConnectionPolicyOverride &policy) {
                     return policy.node_id == node_id;
                   });
  return found == graph.connection_policy_overrides.end() ? nullptr : &*found;
}

const ApproachGeometryOverride *
find_approach_override(const SavedRoadGraph &graph, const ApproachKey &key) {
  const auto found =
      std::find_if(graph.approach_geometry_overrides.begin(),
                   graph.approach_geometry_overrides.end(),
                   [&key](const ApproachGeometryOverride &value) {
                     return value.key == key;
                   });
  return found == graph.approach_geometry_overrides.end() ? nullptr : &*found;
}

const Path *find_alignment(const DerivedRoad &out, RoadSegmentId segment_id) {
  return FindCanonicalAlignment(out, segment_id);
}

const ApproachConnectionDecision *
find_approach_connection(const NodeConnectionDecision &decision,
                         const ApproachKey &key) {
  const auto found =
      std::find_if(decision.approaches.begin(), decision.approaches.end(),
                   [&key](const ApproachConnectionDecision &approach) {
                     return approach.key == key;
                   });
  return found == decision.approaches.end() ? nullptr : &*found;
}

const ResolvedNodeLayout *find_layout(const DerivedRoad &out,
                                      RoadNodeId node_id) {
  const auto found = std::find_if(out.layouts.begin(), out.layouts.end(),
                                  [node_id](const ResolvedNodeLayout &layout) {
                                    return layout.node_id == node_id;
                                  });
  return found == out.layouts.end() ? nullptr : &*found;
}

const ResolvedApproachLayout *
find_approach_layout(const ResolvedNodeLayout &layout, const ApproachKey &key) {
  const auto found =
      std::find_if(layout.approaches.begin(), layout.approaches.end(),
                   [&key](const ResolvedApproachLayout &approach) {
                     return approach.key == key;
                   });
  return found == layout.approaches.end() ? nullptr : &*found;
}

const SegmentSamplingPlan *find_sampling(const DerivedRoad &out,
                                         RoadSegmentId segment_id) {
  const auto found =
      std::find_if(out.sampling.begin(), out.sampling.end(),
                   [segment_id](const SegmentSamplingPlan &plan) {
                     return plan.segment_id == segment_id;
                   });
  return found == out.sampling.end() ? nullptr : &*found;
}

const SectionEvaluation *find_section(const DerivedRoad &out,
                                      RoadSegmentId segment_id,
                                      double station_m) {
  const SectionEvaluation *match = nullptr;
  for (const SectionEvaluation &section : out.sections) {
    if (section.segment_id != segment_id ||
        std::abs(section.station_m - station_m) > station_epsilon) {
      continue;
    }
    if (match != nullptr)
      return nullptr;
    match = &section;
  }
  return match;
}

} // namespace city::road::build
