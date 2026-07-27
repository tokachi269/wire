#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace city::road::build {
namespace {

constexpr double kMarkingElevationM = 0.025;
constexpr double kStopLineCenterM = 0.35;
constexpr double kStopLineHalfLengthM = 0.08;
constexpr double kCrosswalkCenterM = 2.0;
constexpr double kCrosswalkHalfLengthM = 1.4;
constexpr double kCrosswalkStripeHalfWidthM = 0.175;
constexpr double kCrosswalkStripeStepM = 0.7;
// No boundary line is emitted where an adjacent band is at most this wide.
// This is the single decision value for degenerate runs.
constexpr double kDegenerateBandWidthM = 0.05;

std::uint64_t mix(std::uint64_t seed, std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t enum_value(MarkingRole value) {
  return static_cast<std::uint64_t>(value);
}

std::uint64_t enum_value(MarkingOwner::Kind value) {
  return static_cast<std::uint64_t>(value);
}

std::uint64_t owner_key(const MarkingOwner &owner) {
  std::uint64_t seed = 1469598103934665603ull;
  seed = mix(seed, enum_value(owner.kind));
  seed = mix(seed, owner.segment_id);
  seed = mix(seed, owner.node_id);
  seed = mix(seed, owner.manual_id);
  return seed;
}

MarkingIntentId intent_id(std::uint64_t salt, const MarkingOwner &owner,
                          MarkingRole role, std::uint64_t local_id) {
  std::uint64_t seed = mix(0xcbf29ce484222325ull, salt);
  seed = mix(seed, owner_key(owner));
  seed = mix(seed, enum_value(role));
  seed = mix(seed, local_id);
  return seed == 0 ? 1 : seed;
}

MarkingStyleDefinition marking_style_definition(MarkingStyleId style_id) {
  if (style_id == builtin_marking_styles::kCenterLine) {
    return {style_id, RenderStyleFromMarking(style_id), 0.12};
  }
  if (style_id == builtin_marking_styles::kStopLine) {
    return {style_id, RenderStyleFromMarking(style_id), 0.16};
  }
  if (style_id == builtin_marking_styles::kCrosswalk) {
    return {style_id, RenderStyleFromMarking(style_id), 0.35};
  }
  return {style_id, RenderStyleFromMarking(style_id), 0.10};
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

std::pair<double, double> segment_marking_range(const pipeline &pipe,
                                                RoadSegmentId segment_id,
                                                double total_m) {
  double start = 0.0;
  double end = total_m;
  for (const ResolvedNodeLayout &layout : pipe.out.layouts) {
    for (const ResolvedApproachLayout &approach : layout.approaches) {
      if (approach.key.segment_id != segment_id)
        continue;
      if (approach.key.endpoint_role == EndpointRole::kStart) {
        start = std::max(start, approach.gate_station_m);
      } else {
        end = std::min(end, approach.gate_station_m);
      }
    }
  }
  return {start, end};
}

Result<Vec3d> segment_point(const DerivedRoad &out, const Path &alignment,
                            RoadSegmentId segment_id, double station_m,
                            double lateral_m, double elevation_m) {
  const Result<Vec2d> center = EvaluatePath(alignment, station_m);
  const Result<Vec2d> tangent = tangent_at(alignment, station_m);
  const SectionEvaluation *section = find_section(out, segment_id, station_m);
  if (!center.ok || !tangent.ok || section == nullptr) {
    return Result<Vec3d>::Fail(ErrorKind::kInternal,
                               "marking segment point is missing");
  }
  const Vec2d lateral{-tangent.value.y, tangent.value.x};
  const Vec2d point = add(center.value, scale(lateral, lateral_m));
  return Result<Vec3d>::Ok(
      Vec3d{point.x, point.y,
            surface_height(section->boundaries, lateral_m) + elevation_m});
}

Vec3d gate_point(const ConnectionGate &gate, double longitudinal_m,
                 double lateral_m) {
  return Vec3d{
      gate.position.x + gate.tangent.x * longitudinal_m +
          gate.lateral.x * lateral_m,
      gate.position.y + gate.tangent.y * longitudinal_m +
          gate.lateral.y * lateral_m,
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

const SectionBoundarySample *find_boundary(const SectionEvaluation &section,
                                           std::uint64_t boundary_id,
                                           MarkingRole role) {
  const BoundaryRole expected_role =
      role == MarkingRole::kCarriagewayEdge ? BoundaryRole::kCurb
      : role == MarkingRole::kCenterLine    ? BoundaryRole::kLaneDivider
                                            : BoundaryRole::kLaneDivider;
  const SectionBoundarySample *match = nullptr;
  for (const SectionBoundarySample &boundary : section.boundaries) {
    if (boundary.boundary_id != boundary_id)
      continue;
    if (role != MarkingRole::kFree && boundary.role != expected_role &&
        role != MarkingRole::kStopLine && role != MarkingRole::kCrosswalk) {
      continue;
    }
    if (match != nullptr)
      return nullptr;
    match = &boundary;
  }
  return match;
}

MarkingRole role_from_boundary(BoundaryRole role,
                               const AutoMarkingPolicy &policy) {
  if (policy.role != MarkingRole::kLaneSeparator)
    return policy.role;
  if (role == BoundaryRole::kLaneDivider)
    return MarkingRole::kLaneSeparator;
  return MarkingRole::kCarriagewayEdge;
}

Result<bool> add_segment_intents(pipeline &pipe) {
  for (const RoadSegment &segment : pipe.source.segments) {
    const Path *alignment = find_alignment(pipe.out, segment.id);
    const SegmentSamplingPlan *plan = find_sampling(pipe.out, segment.id);
    if (alignment == nullptr || plan == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "marking segment source is missing");
    }
    const Result<double> total = PathLength(*alignment);
    if (!total.ok)
      return Result<bool>::Fail(total.error_kind, total.error);
    const auto [range_start, range_end] =
        segment_marking_range(pipe, segment.id, total.value);
    std::map<std::pair<std::uint64_t, MarkingRole>, MarkingIntent> by_track{};
    for (double station : plan->marking_stations_m) {
      if (station < range_start - station_epsilon ||
          station > range_end + station_epsilon) {
        continue;
      }
      const SectionEvaluation *section =
          find_section(pipe.out, segment.id, station);
      if (section == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "marking section sample is missing");
      }
      for (const SectionBoundarySample &boundary : section->boundaries) {
        if (!boundary.marking.enabled)
          continue;
        const MarkingRole role =
            role_from_boundary(boundary.role, boundary.marking);
        const MarkingOwner owner{MarkingOwner::Kind::kRoadSegment, segment.id,
                                 0, 0};
        const MarkingTrackKey track{segment.id, boundary.boundary_id, role};
        const AutoMarkingKey key{owner, role, track, std::nullopt};
        if (suppressed(pipe.source, key))
          continue;
        Result<Vec3d> point =
            segment_point(pipe.out, *alignment, segment.id, station,
                          boundary.lateral_m, kMarkingElevationM);
        if (!point.ok)
          return Result<bool>::Fail(point.error_kind, point.error);
        const auto group = std::pair{boundary.boundary_id, role};
        auto [it, inserted] = by_track.emplace(group, MarkingIntent{});
        if (inserted) {
          it->second.id = intent_id(1, owner, role, boundary.boundary_id);
          it->second.owner = owner;
          it->second.role = role;
          it->second.style_id = boundary.marking.style_id;
          it->second.geometry = MarkingGeometryRule::kFollowBoundary;
          it->second.track = track;
          it->second.range_start_m = range_start;
          it->second.range_end_m = range_end;
        } else if (it->second.style_id != boundary.marking.style_id) {
          return Result<bool>::Fail(ErrorKind::kUnsupported,
                                    "conflicting marking policy on boundary");
        }
        it->second.boundary_samples.push_back(MarkingBoundarySample{
            station, boundary.lateral_m,
            std::min(boundary.left_band_width_m, boundary.right_band_width_m),
            point.value});
      }
    }
    for (auto &[key, intent] : by_track) {
      (void)key;
      pipe.out.marking_intents.push_back(std::move(intent));
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> add_manual_intents(pipeline &pipe) {
  for (const ManualLineMarking &marking : pipe.source.manual_lines) {
    const Path *alignment = find_alignment(pipe.out, marking.owner_segment_id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "manual line owner alignment is missing");
    }
    const std::vector<Vec2d> locals = FlattenPath(marking.path);
    MarkingIntent intent{};
    intent.owner = MarkingOwner{MarkingOwner::Kind::kManual, 0, 0, marking.id};
    intent.role = MarkingRole::kFree;
    intent.style_id = marking.style_id;
    intent.geometry = MarkingGeometryRule::kOwnerLocalPath;
    intent.id = intent_id(2, intent.owner, intent.role, marking.id);
    for (const Vec2d local : locals) {
      Result<Vec3d> point =
          segment_point(pipe.out, *alignment, marking.owner_segment_id, local.x,
                        local.y, kMarkingElevationM);
      if (!point.ok)
        return Result<bool>::Fail(point.error_kind, point.error);
      intent.world_path.push_back(point.value);
    }
    if (intent.world_path.size() >= 2) {
      pipe.out.marking_intents.push_back(std::move(intent));
    }
  }

  for (const ManualAreaMarking &marking : pipe.source.manual_areas) {
    const Path *alignment = find_alignment(pipe.out, marking.owner_segment_id);
    if (alignment == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "manual area owner alignment is missing");
    }
    const double half_width = marking.width_m * 0.5;
    const double half_length = marking.length_m * 0.5;
    const double c = std::cos(marking.rotation_rad);
    const double s = std::sin(marking.rotation_rad);
    const auto rotate = [&](Vec2d local) {
      return Vec2d{marking.frame_origin.x + local.x * c - local.y * s,
                   marking.frame_origin.y + local.x * s + local.y * c};
    };
    const std::array<Vec2d, 4> locals{
        rotate({-half_length, -half_width}),
        rotate({half_length, -half_width}),
        rotate({half_length, half_width}),
        rotate({-half_length, half_width}),
    };
    MarkingIntent intent{};
    intent.owner = MarkingOwner{MarkingOwner::Kind::kManual, 0, 0, marking.id};
    intent.role = MarkingRole::kFree;
    intent.style_id = marking.style_id;
    intent.geometry = MarkingGeometryRule::kOwnerLocalArea;
    intent.id = intent_id(3, intent.owner, intent.role, marking.id);
    for (const Vec2d local : locals) {
      Result<Vec3d> point =
          segment_point(pipe.out, *alignment, marking.owner_segment_id, local.x,
                        local.y, kMarkingElevationM);
      if (!point.ok)
        return Result<bool>::Fail(point.error_kind, point.error);
      intent.world_polygon.push_back(point.value);
    }
    pipe.out.marking_intents.push_back(std::move(intent));
  }
  return Result<bool>::Ok(true);
}

Result<bool> add_junction_intents(pipeline &pipe) {
  for (const JunctionArea &area : pipe.out.junction_areas) {
    for (const ConnectionGate &gate : area.gates) {
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
      MarkingOwner owner{MarkingOwner::Kind::kJunction, 0, area.node_id, 0};
      AutoMarkingKey stop_key{owner, MarkingRole::kStopLine, std::nullopt,
                              gate.approach};
      if (!suppressed(pipe.source, stop_key)) {
        MarkingIntent stop{};
        stop.id = intent_id(4, owner, MarkingRole::kStopLine,
                            gate.approach.segment_id);
        stop.owner = owner;
        stop.role = MarkingRole::kStopLine;
        stop.style_id = builtin_marking_styles::kStopLine;
        stop.geometry = MarkingGeometryRule::kOwnerLocalArea;
        stop.world_polygon = gate_quad(gate, kStopLineCenterM,
                                       kStopLineHalfLengthM, center, half);
        pipe.out.marking_intents.push_back(std::move(stop));
      }
      AutoMarkingKey crosswalk_key{owner, MarkingRole::kCrosswalk, std::nullopt,
                                   gate.approach};
      if (suppressed(pipe.source, crosswalk_key))
        continue;
      for (double lateral = left->lateral_m + kCrosswalkStripeHalfWidthM;
           lateral + kCrosswalkStripeHalfWidthM <= right->lateral_m;
           lateral += kCrosswalkStripeStepM) {
        MarkingIntent stripe{};
        stripe.id = intent_id(
            5 + static_cast<std::uint64_t>(std::round(lateral * 1000.0)), owner,
            MarkingRole::kCrosswalk, gate.approach.segment_id);
        stripe.owner = owner;
        stripe.role = MarkingRole::kCrosswalk;
        stripe.style_id = builtin_marking_styles::kCrosswalk;
        stripe.geometry = MarkingGeometryRule::kOwnerLocalArea;
        stripe.world_polygon =
            gate_quad(gate, kCrosswalkCenterM, kCrosswalkHalfLengthM, lateral,
                      kCrosswalkStripeHalfWidthM);
        pipe.out.marking_intents.push_back(std::move(stripe));
      }
    }
  }
  return Result<bool>::Ok(true);
}

const ConnectionGate *find_gate(const JunctionArea &area,
                                const ApproachKey &approach) {
  const auto found = std::find_if(area.gates.begin(), area.gates.end(),
                                  [&](const ConnectionGate &gate) {
                                    return gate.approach == approach;
                                  });
  return found == area.gates.end() ? nullptr : &*found;
}

const SectionBoundarySample *find_gate_boundary(const ConnectionGate &gate,
                                                const JunctionMarkingEndpoint &endpoint) {
  const SectionBoundarySample *match = nullptr;
  for (const SectionBoundarySample &boundary : gate.boundaries) {
    if (boundary.boundary_id != endpoint.boundary_id)
      continue;
    if (match != nullptr)
      return nullptr;
    match = &boundary;
  }
  return match;
}

Result<Vec3d> gate_boundary_point(const ConnectionGate &gate,
                                  const JunctionMarkingEndpoint &endpoint) {
  const SectionBoundarySample *boundary = find_gate_boundary(gate, endpoint);
  if (boundary == nullptr) {
    return Result<Vec3d>::Fail(ErrorKind::kValidation,
                               "junction marking endpoint boundary is missing");
  }
  return Result<Vec3d>::Ok(gate_point(gate, 0.0, boundary->lateral_m));
}

Result<bool> add_junction_override_intents(pipeline &pipe) {
  for (const JunctionMarkingOverride &override :
       pipe.source.junction_marking_overrides) {
    if (override.action != JunctionMarkingAction::kConnectToApproach)
      continue;
    if (!override.target.has_value()) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "junction marking override target is missing");
    }
    const auto area = std::find_if(
        pipe.out.junction_areas.begin(), pipe.out.junction_areas.end(),
        [&override](const JunctionArea &candidate) {
          return candidate.node_id == override.node_id;
        });
    if (area == pipe.out.junction_areas.end()) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "junction marking override has no junction area");
    }
    const ConnectionGate *source_gate =
        find_gate(*area, override.source.approach);
    const ConnectionGate *target_gate =
        find_gate(*area, override.target->approach);
    if (source_gate == nullptr || target_gate == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "junction marking override gate is missing");
    }
    Result<Vec3d> source = gate_boundary_point(*source_gate, override.source);
    Result<Vec3d> target = gate_boundary_point(*target_gate, *override.target);
    if (!source.ok)
      return Result<bool>::Fail(source.error_kind, source.error);
    if (!target.ok)
      return Result<bool>::Fail(target.error_kind, target.error);
    MarkingOwner owner{MarkingOwner::Kind::kJunction, 0, override.node_id, 0};
    MarkingIntent intent{};
    intent.id = intent_id(6, owner, override.source.role, override.id);
    intent.owner = owner;
    intent.role = override.source.role;
    intent.style_id = style_for_role(override.source.role);
    intent.geometry = MarkingGeometryRule::kConnectAnchors;
    intent.world_path = {source.value, target.value};
    pipe.out.marking_intents.push_back(std::move(intent));
  }
  return Result<bool>::Ok(true);
}

// Classify the cases where a section transition cannot decide continuation by
// boundary ID. Never rebind automatically to a nearby boundary.
Result<bool> validate_transition_marking_mapping(const pipeline &pipe) {
  for (const RoadSegment &segment : pipe.source.segments) {
    if (!segment.transition.has_value())
      continue;
    const SectionTransition *transition =
        find_transition(pipe.source, *segment.transition);
    if (transition == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "road segment transition is missing");
    }
    const CrossSectionTemplate *from =
        find_template(pipe.source, transition->from_template);
    const CrossSectionTemplate *to =
        find_template(pipe.source, transition->to_template);
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
    for (const SectionTransitionRule &rule : transition->rules) {
      if (rule.action != TransitionAction::kUnsupported)
        continue;
      const auto marked = std::find_if(
          to->boundaries.begin(), to->boundaries.end(),
          [&rule](const BoundaryProfile &candidate) {
            return candidate.boundary_id == rule.element_id &&
                   candidate.marking.enabled;
          });
      const auto marked_source = std::find_if(
          from->boundaries.begin(), from->boundaries.end(),
          [&rule](const BoundaryProfile &candidate) {
            return candidate.boundary_id == rule.element_id &&
                   candidate.marking.enabled;
          });
      if (marked != to->boundaries.end() ||
          marked_source != from->boundaries.end()) {
        return Result<bool>::Fail(
            ErrorKind::kUnsupported,
            "transition element " + std::to_string(rule.element_id) +
                " has no supported marking mapping");
      }
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace

Result<bool> make_marking_intents(pipeline &pipe) {
  pipe.out.marking_intents.clear();
  Result<bool> result = add_segment_intents(pipe);
  if (!result.ok)
    return result;
  result = add_manual_intents(pipe);
  if (!result.ok)
    return result;
  result = add_junction_intents(pipe);
  if (!result.ok)
    return result;
  return add_junction_override_intents(pipe);
}

Result<bool> make_marking_continuity(pipeline &pipe) {
  const Result<bool> mapping = validate_transition_marking_mapping(pipe);
  if (!mapping.ok)
    return mapping;

  std::vector<MarkingIntent> kept{};
  kept.reserve(pipe.out.marking_intents.size());
  for (MarkingIntent &intent : pipe.out.marking_intents) {
    if (!intent.track.has_value()) {
      kept.push_back(std::move(intent));
      continue;
    }
    // Drop samples where the lane has collapsed. This is the only place the
    // activation condition is decided.
    std::vector<MarkingBoundarySample> live{};
    for (const MarkingBoundarySample &sample : intent.boundary_samples) {
      if (sample.min_adjacent_band_width_m > kDegenerateBandWidthM) {
        live.push_back(sample);
      }
    }
    if (live.size() < 2) {
      continue;
    }
    std::sort(live.begin(), live.end(),
              [](const MarkingBoundarySample &a,
                 const MarkingBoundarySample &b) {
                return a.station_m < b.station_m;
              });
    intent.source_action = live.front().station_m >
                                   intent.range_start_m + station_epsilon
                               ? MarkingContinuationAction::kBegin
                               : MarkingContinuationAction::kContinue;
    intent.end_action =
        live.back().station_m < intent.range_end_m - station_epsilon
            ? MarkingContinuationAction::kTerminate
            : MarkingContinuationAction::kContinue;
    intent.world_path.clear();
    for (const MarkingBoundarySample &sample : live) {
      intent.world_path.push_back(sample.position);
    }
    intent.boundary_samples = std::move(live);
    kept.push_back(std::move(intent));
  }
  pipe.out.marking_intents = std::move(kept);
  for (const MarkingIntent &intent : pipe.out.marking_intents) {
    if (intent.track.has_value() && intent.world_path.size() < 2) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "marking continuation path is incomplete");
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> resolve_markings(pipeline &pipe) {
  ResolvedMarkingGraph &graph = pipe.out.markings;
  graph.paths.clear();
  graph.areas.clear();
  for (const MarkingIntent &intent : pipe.out.marking_intents) {
    const MarkingStyleDefinition style =
        marking_style_definition(intent.style_id);
    if (!IsKnownMarkingStyle(intent.style_id)) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "resolved marking style is unknown");
    }
    if (!intent.world_path.empty()) {
      if (intent.world_path.size() < 2) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "resolved marking path is too short");
      }
      graph.paths.push_back(ResolvedMarkingPath{
          intent.id, intent.owner, intent.role, intent.style_id,
          intent.source_action, intent.end_action, intent.world_path,
          style.width_m});
    }
    if (!intent.world_polygon.empty()) {
      if (intent.world_polygon.size() < 3) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "resolved marking area is too small");
      }
      ResolvedMarkingArea area{};
      area.id = intent.id;
      area.owner = intent.owner;
      area.role = intent.role;
      area.style_id = intent.style_id;
      area.polygons.push_back(intent.world_polygon);
      area.origin = intent.world_polygon.front();
      if (intent.world_polygon.size() >= 2) {
        area.forward = {intent.world_polygon[1].x - intent.world_polygon[0].x,
                        intent.world_polygon[1].y - intent.world_polygon[0].y,
                        intent.world_polygon[1].z - intent.world_polygon[0].z};
      }
      if (intent.world_polygon.size() >= 4) {
        area.lateral = {intent.world_polygon[3].x - intent.world_polygon[0].x,
                        intent.world_polygon[3].y - intent.world_polygon[0].y,
                        intent.world_polygon[3].z - intent.world_polygon[0].z};
      }
      graph.areas.push_back(std::move(area));
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
