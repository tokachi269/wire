#include "generation.hpp"

#include "../geometry/geometry.hpp"
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
using internal::station_epsilon;
using internal::tangent_at;

constexpr double kMarkingElevationM = 0.025;
constexpr double kStopLineCenterM = 0.35;
constexpr double kStopLineHalfLengthM = 0.08;
constexpr double kCrosswalkCenterM = 2.0;
constexpr double kCrosswalkHalfLengthM = 1.4;
constexpr double kCrosswalkStripeHalfWidthM = 0.175;
constexpr double kCrosswalkStripeStepM = 0.7;
// No boundary line is emitted where an adjacent strip is at most this wide.
// This is the single decision value for degenerate runs.
constexpr double kDegenerateStripWidthM = 0.05;

// Marking width is resolved once, here, so emit only reads it.
double marking_width_m(MarkingStyleId style_id) {
  if (style_id == builtin_marking_styles::kCenterLine)
    return 0.12;
  if (style_id == builtin_marking_styles::kStopLine)
    return 0.16;
  if (style_id == builtin_marking_styles::kCrosswalk)
    return 0.35;
  return 0.10;
}

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
        width <= station_epsilon ? 0.0 : (lateral_m - a.lateral_m) / width;
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

Result<Vec3d> segment_point(const DerivedSegment &segment, double station_m,
                            double lateral_m) {
  const Result<Vec2d> center = EvaluatePath(segment.alignment, station_m);
  const Result<Vec2d> tangent = tangent_at(segment.alignment, station_m);
  const SectionEvaluation *section = FindSectionAt(segment, station_m);
  if (!center.ok || !tangent.ok || section == nullptr) {
    return Result<Vec3d>::Fail(ErrorKind::kInternal,
                               "marking segment point is missing");
  }
  const Vec2d lateral{-tangent.value.y, tangent.value.x};
  const Vec2d point = add(center.value, scale(lateral, lateral_m));
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
  return {
      gate_point(gate, longitudinal_center_m - longitudinal_half_m,
                 lateral_center_m - lateral_half_m),
      gate_point(gate, longitudinal_center_m + longitudinal_half_m,
                 lateral_center_m - lateral_half_m),
      gate_point(gate, longitudinal_center_m + longitudinal_half_m,
                 lateral_center_m + lateral_half_m),
      gate_point(gate, longitudinal_center_m - longitudinal_half_m,
                 lateral_center_m + lateral_half_m),
  };
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

// Classify the cases where a section transition cannot decide continuation by
// boundary ID. Never rebind automatically to a nearby boundary.
Result<bool> validate_transition_marking_mapping(const SavedRoadGraph &graph) {
  for (const RoadSegment &segment : graph.segments) {
    if (!segment.transition.has_value())
      continue;
    const SectionTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "road segment transition is missing");
    }
    const CrossSectionTemplate *from =
        find_template(graph, transition->from_template);
    const CrossSectionTemplate *to =
        find_template(graph, transition->to_template);
    if (from == nullptr || to == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
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
            ErrorKind::kUnsupported,
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
    for (const double station : segment.surface_stations_m) {
      const SectionEvaluation *section = FindSectionAt(segment, station);
      if (section == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
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
          return Result<bool>::Fail(ErrorKind::kUnsupported,
                                    "conflicting marking policy on boundary");
        }
        if (std::min(boundary.left_strip_width_m,
                     boundary.right_strip_width_m) <=
            kDegenerateStripWidthM) {
          finish_run(it->second);
          continue;
        }
        Result<Vec3d> point =
            segment_point(segment, station, boundary.lateral_m);
        if (!point.ok)
          return Result<bool>::Fail(point.error_kind, point.error);
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
      return Result<bool>::Fail(ErrorKind::kInternal,
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
        return Result<bool>::Fail(point.error_kind, point.error);
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
      return Result<bool>::Fail(ErrorKind::kInternal,
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
        return Result<bool>::Fail(point.error_kind, point.error);
      marking.polygon.push_back(point.value);
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
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "junction marking approach is missing");
      }
      const ConnectionGate &gate = found->gate;
      const SectionBoundarySample *left = nullptr;
      const SectionBoundarySample *right = nullptr;
      for (const SectionBoundarySample &boundary : gate.boundaries) {
        if (boundary.role != BoundaryRole::kCurb)
          continue;
        if (left == nullptr || boundary.lateral_m < left->lateral_m)
          left = &boundary;
        if (right == nullptr || boundary.lateral_m > right->lateral_m)
          right = &boundary;
      }
      if (left == nullptr || right == nullptr ||
          right->lateral_m <= left->lateral_m) {
        return Result<bool>::Fail(
            ErrorKind::kUnsupported,
            "junction marking requires carriageway curb boundaries");
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
        stop.polygon = gate_quad(gate, kStopLineCenterM, kStopLineHalfLengthM,
                                 center, half);
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
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "junction marking override target is missing");
    }
    const auto connection =
        std::find_if(connections.begin(), connections.end(),
                     [&override](const ResolvedConnection &candidate) {
                       return candidate.node_id == override.node_id &&
                              candidate.kind == NodeConnectionKind::kJunction;
                     });
    if (connection == connections.end()) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
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
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "junction marking override gate is missing");
    }
    const SectionBoundarySample *source_boundary =
        find_gate_boundary(source->gate, override.source.boundary_id);
    const SectionBoundarySample *target_boundary =
        find_gate_boundary(target->gate, override.target->boundary_id);
    if (source_boundary == nullptr || target_boundary == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
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
        gate_point(source->gate, 0.0, source_boundary->lateral_m),
        gate_point(target->gate, 0.0, target_boundary->lateral_m)};
    markings.push_back(std::move(marking));
  }
  return Result<bool>::Ok(true);
}

} // namespace

Result<std::vector<DerivedMarking>>
derive_markings(const SavedRoadGraph &graph,
                const std::vector<DerivedSegment> &segments,
                const std::vector<ResolvedConnection> &connections) {
  using Out = Result<std::vector<DerivedMarking>>;
  const Result<bool> mapping = validate_transition_marking_mapping(graph);
  if (!mapping.ok)
    return Out::Fail(mapping.error_kind, mapping.error);

  std::vector<DerivedMarking> markings{};
  Result<bool> step = derive_segment_markings(graph, segments, markings);
  if (!step.ok)
    return Out::Fail(step.error_kind, step.error);
  step = derive_manual_markings(graph, segments, markings);
  if (!step.ok)
    return Out::Fail(step.error_kind, step.error);
  step = derive_junction_markings(graph, connections, markings);
  if (!step.ok)
    return Out::Fail(step.error_kind, step.error);
  step = derive_junction_override_markings(graph, connections, markings);
  if (!step.ok)
    return Out::Fail(step.error_kind, step.error);

  for (const DerivedMarking &marking : markings) {
    if (!IsKnownMarkingStyle(marking.style_id)) {
      return Out::Fail(ErrorKind::kInternal, "derived marking style is unknown");
    }
  }
  return Out::Ok(std::move(markings));
}

} // namespace city::road::generation
