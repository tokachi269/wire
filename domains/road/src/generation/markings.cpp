#include "generation.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/section.hpp"
#include "../lookup.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace city::road::generation {
namespace {

using internal::add;
using internal::find_template;
using internal::find_transition;
using internal::scale;
using internal::distance_epsilon;
using internal::marking_width_m;
using internal::tangent_at;

constexpr double kMarkingElevationM = 0.025;
constexpr double kStopLineCenterM = 0.35;
constexpr double kCrosswalkCenterM = 2.0;
constexpr double kCrosswalkHalfLengthM = 1.4;
constexpr double kCrosswalkStripeHalfWidthM = 0.175;
constexpr double kCrosswalkStripeStepM = 0.7;
// No boundary line is emitted where an adjacent strip is at most this wide.
// This is the single decision value for degenerate runs.
constexpr double kDegenerateStripWidthM = 0.05;

MarkingStyleId style_for_role(MarkingRole role) {
  if (role == MarkingRole::kCenterLine)
    return builtin_marking_styles::kCenterLine;
  if (role == MarkingRole::kStopLine)
    return builtin_marking_styles::kStopLine;
  if (role == MarkingRole::kCrosswalk)
    return builtin_marking_styles::kCrosswalk;
  return builtin_marking_styles::kWhiteSolid;
}

bool suppressed(const SavedRoadGraph &graph, const AutoMarkingKey &key) {
  return std::any_of(graph.auto_marking_overrides.begin(),
                     graph.auto_marking_overrides.end(),
                     [&key](const AutoMarkingOverride &value) {
                       return value.suppressed && value.key == key;
                     });
}

double surface_height(const std::vector<SectionBoundarySample> &boundaries,
                      double lateral_m) {
  if (boundaries.empty())
    return 0.0;
  if (lateral_m <= boundaries.front().lateral_m) {
    return boundaries.front().height_m;
  }
  for (std::size_t index = 1; index < boundaries.size(); ++index) {
    const SectionBoundarySample &a = boundaries[index - 1];
    const SectionBoundarySample &b = boundaries[index];
    if (lateral_m > b.lateral_m)
      continue;
    const double width = b.lateral_m - a.lateral_m;
    const double t =
        width <= distance_epsilon ? 0.0 : (lateral_m - a.lateral_m) / width;
    return a.height_m + (b.height_m - a.height_m) * t;
  }
  return boundaries.back().height_m;
}

const DerivedSegment *segment_of(const std::vector<DerivedSegment> &segments,
                                 RoadSegmentId segment_id) {
  const auto found = std::find_if(segments.begin(), segments.end(),
                                  [segment_id](const DerivedSegment &segment) {
                                    return segment.id == segment_id;
                                  });
  return found == segments.end() ? nullptr : &*found;
}

Result<Vec3d> segment_point(const DerivedSegment &segment,
                            double segment_distance_m,
                            double lateral_m) {
  const Result<Vec2d> center =
      EvaluatePath(segment.alignment, segment_distance_m);
  const Result<Vec2d> lateral =
      internal::lateral_at(segment.alignment, segment_distance_m);
  const SectionEvaluation *section =
      FindSectionAt(segment, segment_distance_m);
  if (!center.ok || !lateral.ok || section == nullptr) {
    return Result<Vec3d>::Fail(CommitFailureCategory::kInternalError,
                               "marking segment point is missing");
  }
  const Vec2d point = add(center.value, scale(lateral.value, lateral_m));
  return Result<Vec3d>::Ok(Vec3d{
      point.x, point.y,
      surface_height(section->boundaries, lateral_m) + kMarkingElevationM});
}

Vec3d gate_point(const ConnectionGate &gate, double longitudinal_m,
                 double lateral_m) {
  const double world_lateral_m =
      gate.approach.endpoint_role == EndpointRole::kEnd ? -lateral_m
                                                        : lateral_m;
  return Vec3d{
      gate.position.x + gate.tangent.x * longitudinal_m +
          gate.lateral.x * world_lateral_m,
      gate.position.y + gate.tangent.y * longitudinal_m +
          gate.lateral.y * world_lateral_m,
      gate.position.z + surface_height(gate.boundaries, lateral_m) +
          kMarkingElevationM,
  };
}

std::vector<Vec3d> gate_quad(const ConnectionGate &gate,
                             double longitudinal_center_m,
                             double longitudinal_half_m,
                             double lateral_center_m, double lateral_half_m) {
  std::vector<Vec3d> points{
      gate_point(gate, longitudinal_center_m - longitudinal_half_m,
                 lateral_center_m - lateral_half_m),
      gate_point(gate, longitudinal_center_m + longitudinal_half_m,
                 lateral_center_m - lateral_half_m),
      gate_point(gate, longitudinal_center_m + longitudinal_half_m,
                 lateral_center_m + lateral_half_m),
      gate_point(gate, longitudinal_center_m - longitudinal_half_m,
                 lateral_center_m + lateral_half_m),
  };
  if (gate.approach.endpoint_role == EndpointRole::kEnd)
    std::reverse(points.begin(), points.end());
  return points;
}

std::vector<Vec3d> gate_cross_section_line(const ConnectionGate &gate,
                                           double longitudinal_m,
                                           double left_lateral_m,
                                           double right_lateral_m) {
  std::vector<double> laterals{left_lateral_m};
  for (const SectionBoundarySample &boundary : gate.boundaries) {
    if (boundary.lateral_m > left_lateral_m + distance_epsilon &&
        boundary.lateral_m < right_lateral_m - distance_epsilon) {
      laterals.push_back(boundary.lateral_m);
    }
  }
  laterals.push_back(right_lateral_m);
  std::sort(laterals.begin(), laterals.end());

  std::vector<Vec3d> points{};
  points.reserve(laterals.size());
  for (std::size_t index = 0; index < laterals.size(); ++index) {
    if (index > 0 &&
        std::abs(laterals[index] - laterals[index - 1]) <= distance_epsilon) {
      continue;
    }
    points.push_back(gate_point(gate, longitudinal_m, laterals[index]));
  }
  return points;
}

MarkingRole role_from_boundary(BoundaryRole role,
                               const AutoMarkingPolicy &policy) {
  if (policy.role != MarkingRole::kLaneSeparator)
    return policy.role;
  if (role == BoundaryRole::kLaneDivider)
    return MarkingRole::kLaneSeparator;
  return MarkingRole::kCarriagewayEdge;
}

struct line_track {
  MarkingStyleId style_id{};
  MarkingRole role = MarkingRole::kLaneSeparator;
  std::vector<Vec3d> active_run{};
  std::vector<std::vector<Vec3d>> completed_runs{};
};

void finish_run(line_track &track) {
  if (track.active_run.size() >= 2) {
    track.completed_runs.push_back(std::move(track.active_run));
  }
  track.active_run.clear();
}

const SectionBoundarySample *find_gate_boundary(const ConnectionGate &gate,
                                                std::uint64_t boundary_id) {
  const SectionBoundarySample *match = nullptr;
  for (const SectionBoundarySample &boundary : gate.boundaries) {
    if (boundary.boundary_id != boundary_id)
      continue;
    if (match != nullptr)
      return nullptr;
    match = &boundary;
  }
  return match;
}

const ResolvedApproach *
find_connection_approach(const ResolvedConnection &connection,
                         const ApproachKey &key) {
  const auto found =
      std::find_if(connection.approaches.begin(), connection.approaches.end(),
                   [&key](const ResolvedApproach &approach) {
                     return approach.key == key;
                   });
  return found == connection.approaches.end() ? nullptr : &*found;
}

std::vector<const SectionBoundarySample *>
find_marked_gate_boundaries(const ConnectionGate &gate,
                            std::uint64_t boundary_id) {
  std::vector<const SectionBoundarySample *> matches{};
  for (const SectionBoundarySample &boundary : gate.boundaries) {
    if (boundary.boundary_id == boundary_id && boundary.marking.enabled)
      matches.push_back(&boundary);
  }
  return matches;
}

// Classify the cases where a section transition cannot decide continuation by
// boundary ID. Never rebind automatically to a nearby boundary.
Result<bool> validate_transition_marking_mapping(const SavedRoadGraph &graph) {
  for (const RoadSegment &segment : graph.segments) {
    if (!segment.transition.has_value())
      continue;
    const RoadLayoutTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road segment transition is missing");
    }
    const RoadLayoutTemplate *from =
        find_template(graph, transition->from_template);
    const RoadLayoutTemplate *to =
        find_template(graph, transition->to_template);
    if (from == nullptr || to == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road transition template is missing");
    }
    for (const BoundaryProfile &source_boundary : from->boundaries) {
      const auto target = std::find_if(
          to->boundaries.begin(), to->boundaries.end(),
          [&source_boundary](const BoundaryProfile &candidate) {
            return candidate.boundary_id == source_boundary.boundary_id;
          });
      if (target == to->boundaries.end())
        continue;
      if (target->role != source_boundary.role) {
        return Result<bool>::Fail(
            CommitFailureCategory::kNotImplemented,
            "transition boundary " +
                std::to_string(source_boundary.boundary_id) +
                " changes role and cannot continue markings");
      }
    }
  }
  return Result<bool>::Ok(true);
}

// Segment lines follow the boundary of the same ID. A line begins where the
// boundary appears and ends where it disappears; runs where the neighbouring
// lane has collapsed carry no line at all.
Result<bool> derive_segment_markings(const SavedRoadGraph &graph,
                                     const std::vector<DerivedSegment> &segments,
                                     std::vector<DerivedMarking> &markings) {
  for (const DerivedSegment &segment : segments) {
    std::map<std::pair<std::uint64_t, MarkingRole>, line_track> tracks{};
    for (const double distance : segment.surface_segment_distances_m) {
      const SectionEvaluation *section = FindSectionAt(segment, distance);
      if (section == nullptr) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "marking section sample is missing");
      }
      std::set<std::pair<std::uint64_t, MarkingRole>> active_groups{};
      for (const SectionBoundarySample &boundary : section->boundaries) {
        if (!boundary.marking.enabled)
          continue;
        const MarkingRole role =
            role_from_boundary(boundary.role, boundary.marking);
        const MarkingOwner owner{MarkingOwner::Kind::kRoadSegment, segment.id, 0,
                                 0};
        const MarkingTrackKey track_key{segment.id, boundary.boundary_id, role};
        if (suppressed(graph,
                       AutoMarkingKey{owner, role, track_key, std::nullopt})) {
          continue;
        }
        const auto group = std::pair{boundary.boundary_id, role};
        auto [it, inserted] = tracks.emplace(group, line_track{});
        if (inserted) {
          it->second.style_id = boundary.marking.style_id;
          it->second.role = role;
        } else if (it->second.style_id != boundary.marking.style_id) {
          return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
                                    "conflicting marking policy on boundary");
        }
        const bool degenerate_adjacent_strip =
            std::min(boundary.left_strip_width_m,
                     boundary.right_strip_width_m) <= kDegenerateStripWidthM;
        const bool visible_edge_boundary =
            role == MarkingRole::kCarriagewayEdge &&
            std::max(boundary.left_strip_width_m,
                     boundary.right_strip_width_m) > kDegenerateStripWidthM;
        if (degenerate_adjacent_strip && !visible_edge_boundary) {
          finish_run(it->second);
          continue;
        }
        Result<Vec3d> point =
            segment_point(segment, distance, boundary.marking_lateral_m);
        if (!point.ok)
          return Result<bool>::Fail(point.failure_category, point.error);
        it->second.active_run.push_back(point.value);
        active_groups.insert(group);
      }
      for (auto &[group, track] : tracks) {
        if (!active_groups.contains(group)) {
          finish_run(track);
        }
      }
    }
    for (auto &[group, track] : tracks) {
      finish_run(track);
      for (std::vector<Vec3d> &run : track.completed_runs) {
        DerivedMarking marking{};
        marking.owner =
            MarkingOwner{MarkingOwner::Kind::kRoadSegment, segment.id, 0, 0};
        marking.boundary_id = group.first;
        marking.role = track.role;
        marking.style_id = track.style_id;
        marking.width_m = marking_width_m(track.style_id);
        marking.points = std::move(run);
        markings.push_back(std::move(marking));
      }
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> derive_manual_markings(const SavedRoadGraph &graph,
                                    const std::vector<DerivedSegment> &segments,
                                    std::vector<DerivedMarking> &markings) {
  for (const ManualLineMarking &source : graph.manual_lines) {
    const DerivedSegment *segment =
        segment_of(segments, source.owner_segment_id);
    if (segment == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "manual line owner alignment is missing");
    }
    DerivedMarking marking{};
    marking.owner = MarkingOwner{MarkingOwner::Kind::kManual, 0, 0, source.id};
    marking.role = MarkingRole::kFree;
    marking.style_id = source.style_id;
    marking.width_m = marking_width_m(source.style_id);
    for (const Vec2d local : FlattenPath(source.path)) {
      Result<Vec3d> point = segment_point(*segment, local.x, local.y);
      if (!point.ok)
        return Result<bool>::Fail(point.failure_category, point.error);
      marking.points.push_back(point.value);
    }
    if (marking.points.size() >= 2) {
      markings.push_back(std::move(marking));
    }
  }

  for (const ManualAreaMarking &source : graph.manual_areas) {
    const DerivedSegment *segment =
        segment_of(segments, source.owner_segment_id);
    if (segment == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "manual area owner alignment is missing");
    }
    const double half_width = source.width_m * 0.5;
    const double half_length = source.length_m * 0.5;
    const double c = std::cos(source.rotation_rad);
    const double s = std::sin(source.rotation_rad);
    const auto rotate = [&](Vec2d local) {
      return Vec2d{source.frame_origin.x + local.x * c - local.y * s,
                   source.frame_origin.y + local.x * s + local.y * c};
    };
    const std::array<Vec2d, 4> locals{
        rotate({-half_length, -half_width}),
        rotate({half_length, -half_width}),
        rotate({half_length, half_width}),
        rotate({-half_length, half_width}),
    };
    DerivedMarking marking{};
    marking.owner = MarkingOwner{MarkingOwner::Kind::kManual, 0, 0, source.id};
    marking.role = MarkingRole::kFree;
    marking.style_id = source.style_id;
    marking.width_m = marking_width_m(source.style_id);
    for (const Vec2d local : locals) {
      Result<Vec3d> point = segment_point(*segment, local.x, local.y);
      if (!point.ok)
        return Result<bool>::Fail(point.failure_category, point.error);
      marking.polygon.push_back(point.value);
    }
    markings.push_back(std::move(marking));
  }
  return Result<bool>::Ok(true);
}

Result<bool>
derive_connection_boundary_markings(
    const SavedRoadGraph &graph,
    const std::vector<ResolvedConnection> &connections,
    std::vector<DerivedMarking> &markings) {
  for (const ResolvedConnection &connection : connections) {
    const std::vector<ResolvedBoundaryCurve> *curves = nullptr;
    if (connection.kind == NodeConnectionKind::kCorner) {
      curves = &connection.connection_geometry.boundary_curves;
    } else if (connection.kind == NodeConnectionKind::kJunction) {
      curves = &connection.junction_geometry.perimeter_curves;
    } else {
      continue;
    }
    const MarkingOwner owner{MarkingOwner::Kind::kJunction, 0,
                             connection.node_id, 0};
    for (const ResolvedBoundaryCurve &curve : *curves) {
      const BoundaryEndpointKey source_endpoint{
          curve.source_approach.segment_id, curve.source_boundary_id,
          curve.source_approach.endpoint_role};
      const BoundaryEndpointKey target_endpoint{
          curve.target_approach.segment_id, curve.target_boundary_id,
          curve.target_approach.endpoint_role};
      const bool explicitly_connected = std::any_of(
          graph.boundary_continuations.begin(),
          graph.boundary_continuations.end(),
          [&source_endpoint, &target_endpoint](const auto &continuation) {
            return (continuation.source == source_endpoint &&
                    continuation.target == target_endpoint) ||
                   (continuation.source == target_endpoint &&
                    continuation.target == source_endpoint);
          });
      if (explicitly_connected || !curve.carries_marking)
        continue;
      const ResolvedApproach *source =
          find_connection_approach(connection, curve.source_approach);
      const ResolvedApproach *target =
          find_connection_approach(connection, curve.target_approach);
      if (source == nullptr || target == nullptr) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInternalError,
            "connection boundary marking approach is missing");
      }
      const std::vector<const SectionBoundarySample *> source_boundaries =
          find_marked_gate_boundaries(source->gate,
                                      curve.source_boundary_id);
      const std::vector<const SectionBoundarySample *> target_boundaries =
          find_marked_gate_boundaries(target->gate,
                                      curve.target_boundary_id);
      if (source_boundaries.size() > 1 || target_boundaries.size() > 1) {
        return Result<bool>::Fail(
            CommitFailureCategory::kNotImplemented,
            "connection marking boundary is not uniquely mapped");
      }
      if (source_boundaries.empty() || target_boundaries.empty())
        continue;
      const SectionBoundarySample *source_boundary = source_boundaries.front();
      const SectionBoundarySample *target_boundary = target_boundaries.front();
      const MarkingRole source_role =
          role_from_boundary(source_boundary->role, source_boundary->marking);
      const MarkingRole target_role =
          role_from_boundary(target_boundary->role, target_boundary->marking);
      if (source_role != target_role ||
          source_boundary->marking.style_id !=
              target_boundary->marking.style_id) {
        return Result<bool>::Fail(
            CommitFailureCategory::kNotImplemented,
            "connection marking role or style changes across the connection");
      }
      const MarkingTrackKey source_track{
          source->key.segment_id, curve.source_boundary_id, source_role};
      const MarkingTrackKey target_track{
          target->key.segment_id, curve.target_boundary_id, target_role};
      const MarkingOwner source_owner{MarkingOwner::Kind::kRoadSegment,
                                      source->key.segment_id, 0, 0};
      const MarkingOwner target_owner{MarkingOwner::Kind::kRoadSegment,
                                      target->key.segment_id, 0, 0};
      if (suppressed(graph,
                     AutoMarkingKey{source_owner, source_role, source_track,
                                    std::nullopt}) ||
          suppressed(graph,
                     AutoMarkingKey{target_owner, target_role, target_track,
                                    std::nullopt})) {
        continue;
      }
      DerivedMarking marking{};
      marking.owner = owner;
      marking.boundary_id = curve.source_boundary_id;
      marking.role = source_role;
      marking.style_id = source_boundary->marking.style_id;
      marking.width_m = marking_width_m(marking.style_id);
      marking.points = curve.marking_points;
      for (Vec3d &point : marking.points)
        point.z += kMarkingElevationM;
      if (marking.points.size() >= 2)
        markings.push_back(std::move(marking));
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> derive_boundary_continuation_markings(
    const SavedRoadGraph &graph,
    const std::vector<ResolvedConnection> &connections,
    const std::vector<DerivedBoundaryPath> &boundary_paths,
    std::vector<DerivedMarking> &markings) {
  for (const BoundaryContinuation &continuation :
       graph.boundary_continuations) {
    const auto path = std::find_if(
        boundary_paths.begin(), boundary_paths.end(),
        [&continuation](const DerivedBoundaryPath &candidate) {
          return candidate.continuation_id == continuation.id;
        });
    if (path == boundary_paths.end()) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "boundary continuation marking path is missing");
    }
    if (path->path.spans.empty())
      continue;
    const internal::BoundaryEndpointLookup source_lookup =
        internal::find_boundary_endpoint(graph, continuation.source);
    const internal::BoundaryEndpointLookup target_lookup =
        internal::find_boundary_endpoint(graph, continuation.target);
    const auto connection = std::find_if(
        connections.begin(), connections.end(),
        [&source_lookup](const ResolvedConnection &candidate) {
          return candidate.node_id == source_lookup.node_id;
        });
    if (source_lookup.boundary == nullptr || target_lookup.boundary == nullptr ||
        connection == connections.end()) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "boundary continuation marking endpoint is missing");
    }
    const ResolvedApproach *source = find_connection_approach(
        *connection,
        ApproachKey{source_lookup.node_id, continuation.source.segment_id,
                    continuation.source.endpoint_role});
    const ResolvedApproach *target = find_connection_approach(
        *connection,
        ApproachKey{target_lookup.node_id, continuation.target.segment_id,
                    continuation.target.endpoint_role});
    if (source == nullptr || target == nullptr) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "boundary continuation marking approach is missing");
    }
    const auto source_boundaries = find_marked_gate_boundaries(
        source->gate, continuation.source.boundary_id);
    const auto target_boundaries = find_marked_gate_boundaries(
        target->gate, continuation.target.boundary_id);
    if (source_boundaries.empty() && target_boundaries.empty())
      continue;
    if (source_boundaries.size() != 1 || target_boundaries.size() != 1) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "boundary continuation marking is not uniquely mapped");
    }
    const SectionBoundarySample &source_boundary = *source_boundaries.front();
    const SectionBoundarySample &target_boundary = *target_boundaries.front();
    const MarkingRole source_role =
        role_from_boundary(source_boundary.role, source_boundary.marking);
    const MarkingRole target_role =
        role_from_boundary(target_boundary.role, target_boundary.marking);
    if (continuation.kind == BoundaryContinuationKind::kContinuation &&
        (source_role != target_role ||
         source_boundary.marking.style_id != target_boundary.marking.style_id)) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "continued boundary marking changes role or style");
    }
    const SectionBoundarySample &policy_boundary =
        continuation.kind == BoundaryContinuationKind::kMerge
            ? target_boundary
            : source_boundary;
    const MarkingRole role = continuation.kind == BoundaryContinuationKind::kMerge
                                 ? target_role
                                 : source_role;
    const MarkingOwner source_owner{MarkingOwner::Kind::kRoadSegment,
                                    continuation.source.segment_id, 0, 0};
    const MarkingOwner target_owner{MarkingOwner::Kind::kRoadSegment,
                                    continuation.target.segment_id, 0, 0};
    if (suppressed(graph,
                   AutoMarkingKey{
                       source_owner, source_role,
                       MarkingTrackKey{continuation.source.segment_id,
                                       continuation.source.boundary_id,
                                       source_role},
                       std::nullopt}) ||
        suppressed(graph,
                   AutoMarkingKey{
                       target_owner, target_role,
                       MarkingTrackKey{continuation.target.segment_id,
                                       continuation.target.boundary_id,
                                       target_role},
                       std::nullopt})) {
      continue;
    }
    const std::vector<Vec2d> points = FlattenPath(path->path);
    if (points.size() < 2)
      continue;
    const double source_z =
        gate_point(source->gate, 0.0, source_boundary.marking_lateral_m).z;
    const double target_z =
        gate_point(target->gate, 0.0, target_boundary.marking_lateral_m).z;
    DerivedMarking marking{};
    marking.owner = MarkingOwner{MarkingOwner::Kind::kJunction, 0,
                                 source_lookup.node_id, 0};
    marking.boundary_id = continuation.source.boundary_id;
    marking.role = role;
    marking.style_id = policy_boundary.marking.style_id;
    marking.width_m = marking_width_m(marking.style_id);
    marking.points.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      const double t = static_cast<double>(index) /
                       static_cast<double>(points.size() - 1);
      marking.points.push_back(
          Vec3d{points[index].x, points[index].y,
                source_z + (target_z - source_z) * t});
    }
    markings.push_back(std::move(marking));
  }
  return Result<bool>::Ok(true);
}

Result<bool> derive_junction_markings(const SavedRoadGraph &graph,
                                      const std::vector<ResolvedConnection> &connections,
                                      std::vector<DerivedMarking> &markings) {
  for (const ResolvedConnection &connection : connections) {
    if (connection.kind != NodeConnectionKind::kJunction)
      continue;
    for (const ApproachKey &key : connection.ordered_approaches) {
      const auto found = std::find_if(connection.approaches.begin(),
                                      connection.approaches.end(),
                                      [&key](const ResolvedApproach &approach) {
                                        return approach.key == key;
                                      });
      if (found == connection.approaches.end()) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "junction marking approach is missing");
      }
      const ConnectionGate &gate = found->gate;
      const SectionBoundarySample *left = nullptr;
      const SectionBoundarySample *right = nullptr;
      for (const SectionBoundarySample &boundary : gate.boundaries) {
        if (boundary.carriageway_side == 0.0)
          continue;
        if (left == nullptr || boundary.lateral_m < left->lateral_m)
          left = &boundary;
        if (right == nullptr || boundary.lateral_m > right->lateral_m)
          right = &boundary;
      }
      if (left == nullptr || right == nullptr ||
          right->lateral_m <= left->lateral_m) {
        return Result<bool>::Fail(
            CommitFailureCategory::kNotImplemented,
            "junction marking requires carriageway edge boundaries");
      }
      const double center = (left->lateral_m + right->lateral_m) * 0.5;
      const double half = (right->lateral_m - left->lateral_m) * 0.5;
      const MarkingOwner owner{MarkingOwner::Kind::kJunction, 0,
                               connection.node_id, 0};
      if (!suppressed(graph, AutoMarkingKey{owner, MarkingRole::kStopLine,
                                            std::nullopt, gate.approach})) {
        DerivedMarking stop{};
        stop.owner = owner;
        stop.role = MarkingRole::kStopLine;
        stop.style_id = builtin_marking_styles::kStopLine;
        stop.width_m = marking_width_m(stop.style_id);
        stop.points = gate_cross_section_line(
            gate, kStopLineCenterM, center - half, center + half);
        markings.push_back(std::move(stop));
      }
      if (suppressed(graph, AutoMarkingKey{owner, MarkingRole::kCrosswalk,
                                           std::nullopt, gate.approach})) {
        continue;
      }
      for (double lateral = left->lateral_m + kCrosswalkStripeHalfWidthM;
           lateral + kCrosswalkStripeHalfWidthM <= right->lateral_m;
           lateral += kCrosswalkStripeStepM) {
        DerivedMarking stripe{};
        stripe.owner = owner;
        stripe.role = MarkingRole::kCrosswalk;
        stripe.style_id = builtin_marking_styles::kCrosswalk;
        stripe.width_m = marking_width_m(stripe.style_id);
        stripe.polygon =
            gate_quad(gate, kCrosswalkCenterM, kCrosswalkHalfLengthM, lateral,
                      kCrosswalkStripeHalfWidthM);
        markings.push_back(std::move(stripe));
      }
    }
  }
  return Result<bool>::Ok(true);
}

// Lines only cross a junction when the user names both endpoints. Nothing is
// inferred from angles, distance or approach order.
Result<bool>
derive_junction_override_markings(const SavedRoadGraph &graph,
                                  const std::vector<ResolvedConnection> &connections,
                                  std::vector<DerivedMarking> &markings) {
  for (const JunctionMarkingOverride &override :
       graph.junction_marking_overrides) {
    if (override.action != JunctionMarkingAction::kConnectToApproach)
      continue;
    if (!override.target.has_value()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "junction marking override target is missing");
    }
    const auto connection =
        std::find_if(connections.begin(), connections.end(),
                     [&override](const ResolvedConnection &candidate) {
                       return candidate.node_id == override.node_id &&
                              candidate.kind == NodeConnectionKind::kJunction;
                     });
    if (connection == connections.end()) {
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
                                "junction marking override has no junction");
    }
    const auto approach_of = [&connection](const ApproachKey &key) {
      return std::find_if(connection->approaches.begin(),
                          connection->approaches.end(),
                          [&key](const ResolvedApproach &approach) {
                            return approach.key == key;
                          });
    };
    const auto source = approach_of(override.source.approach);
    const auto target = approach_of(override.target->approach);
    if (source == connection->approaches.end() ||
        target == connection->approaches.end()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "junction marking override gate is missing");
    }
    const SectionBoundarySample *source_boundary =
        find_gate_boundary(source->gate, override.source.boundary_id);
    const SectionBoundarySample *target_boundary =
        find_gate_boundary(target->gate, override.target->boundary_id);
    if (source_boundary == nullptr || target_boundary == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "junction marking endpoint boundary is missing");
    }
    DerivedMarking marking{};
    marking.owner =
        MarkingOwner{MarkingOwner::Kind::kJunction, 0, override.node_id, 0};
    marking.boundary_id = override.source.boundary_id;
    marking.role = override.source.role;
    marking.style_id = style_for_role(override.source.role);
    marking.width_m = marking_width_m(marking.style_id);
    marking.points = {
        gate_point(source->gate, 0.0, source_boundary->marking_lateral_m),
        gate_point(target->gate, 0.0, target_boundary->marking_lateral_m)};
    markings.push_back(std::move(marking));
  }
  return Result<bool>::Ok(true);
}

} // namespace

Result<std::vector<DerivedMarking>>
derive_markings(const SavedRoadGraph &graph,
                const std::vector<DerivedSegment> &segments,
                const std::vector<ResolvedConnection> &connections,
                const std::vector<DerivedBoundaryPath> &boundary_paths) {
  using Out = Result<std::vector<DerivedMarking>>;
  const Result<bool> mapping = validate_transition_marking_mapping(graph);
  if (!mapping.ok)
    return Out::Fail(mapping.failure_category, mapping.error);

  std::vector<DerivedMarking> markings{};
  Result<bool> step = derive_segment_markings(graph, segments, markings);
  if (!step.ok)
    return Out::Fail(step.failure_category, step.error);
  step = derive_manual_markings(graph, segments, markings);
  if (!step.ok)
    return Out::Fail(step.failure_category, step.error);
  step = derive_connection_boundary_markings(graph, connections, markings);
  if (!step.ok)
    return Out::Fail(step.failure_category, step.error);
  step = derive_boundary_continuation_markings(
      graph, connections, boundary_paths, markings);
  if (!step.ok)
    return Out::Fail(step.failure_category, step.error);
  step = derive_junction_markings(graph, connections, markings);
  if (!step.ok)
    return Out::Fail(step.failure_category, step.error);
  step = derive_junction_override_markings(graph, connections, markings);
  if (!step.ok)
    return Out::Fail(step.failure_category, step.error);

  for (const DerivedMarking &marking : markings) {
    if (!IsKnownMarkingStyle(marking.style_id)) {
      return Out::Fail(CommitFailureCategory::kInternalError, "derived marking style is unknown");
    }
  }
  return Out::Ok(std::move(markings));
}

} // namespace city::road::generation
