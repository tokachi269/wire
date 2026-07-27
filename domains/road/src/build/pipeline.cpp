#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

#include "../draw/mesh.hpp"

#include <algorithm>
#include <array>
#include <iterator>

namespace city::road::build {
namespace {

Result<bool> emit(pipeline &pipe) {
  for (const CanonicalAlignment &canonical : pipe.out.alignments) {
    const SegmentSamplingPlan *plan =
        find_sampling(pipe.out, canonical.segment_id);
    if (plan == nullptr || plan->surface_stations_m.empty()) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road draw sampling plan is missing");
    }
    draw::segment_input input{};
    input.segment_id = canonical.segment_id;
    for (const double station : plan->surface_stations_m) {
      const Result<Vec2d> center = EvaluatePath(canonical.path, station);
      const Result<Vec2d> tangent = tangent_at(canonical.path, station);
      const SectionEvaluation *section =
          find_section(pipe.out, canonical.segment_id, station);
      if (!center.ok || !tangent.ok || section == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road draw sample is missing");
      }
      input.samples.push_back(draw::segment_sample{center.value, tangent.value,
                                                   section->boundaries,
                                                   section->surface_styles});
    }
    Result<draw::segment_output> output = draw::make_segment(input);
    if (!output.ok)
      return Result<bool>::Fail(output.error_kind, output.error);
    pipe.out.segment_meshes.insert(
        pipe.out.segment_meshes.end(),
        std::make_move_iterator(output.value.surface_meshes.begin()),
        std::make_move_iterator(output.value.surface_meshes.end()));
    pipe.out.terrain_masks.push_back(std::move(output.value.terrain_mask));
  }

  for (const ConnectionGeometry &geometry : pipe.out.connections) {
    Result<std::vector<Mesh>> meshes = draw::make_connection(geometry);
    if (!meshes.ok)
      return Result<bool>::Fail(meshes.error_kind, meshes.error);
    pipe.out.connection_meshes.insert(
        pipe.out.connection_meshes.end(),
        std::make_move_iterator(meshes.value.begin()),
        std::make_move_iterator(meshes.value.end()));
  }
  for (const JunctionGeometry &geometry : pipe.out.junctions) {
    Result<draw::junction_output> output = draw::make_junction(geometry);
    if (!output.ok)
      return Result<bool>::Fail(output.error_kind, output.error);
    pipe.out.junction_meshes.insert(
        pipe.out.junction_meshes.end(),
        std::make_move_iterator(output.value.surface_meshes.begin()),
        std::make_move_iterator(output.value.surface_meshes.end()));
  }

  Result<std::vector<Mesh>> markings = draw::make_markings(pipe.out.markings);
  if (!markings.ok) {
    return Result<bool>::Fail(markings.error_kind, markings.error);
  }
  pipe.out.marking_meshes.insert(
      pipe.out.marking_meshes.end(),
      std::make_move_iterator(markings.value.begin()),
      std::make_move_iterator(markings.value.end()));
  return Result<bool>::Ok(true);
}

} // namespace

Result<DerivedRoad> make(const SavedRoadGraph &source) {
  pipeline pipe(source);
  const auto run = [&pipe](stage step, auto &&function) -> Result<bool> {
    Result<bool> begun = pipe.begin(step);
    if (!begun.ok)
      return begun;
    return function(pipe);
  };

  Result<bool> result = run(stage::topology, make_topology);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::alignments, make_alignments);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::connections, make_connections);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::auto_layout, make_auto_layouts);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::layout, resolve_layouts);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::sampling, make_sampling);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::sections, make_sections);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::gates, make_gates);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::junctions, make_junctions);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::marking_anchors, make_marking_anchors);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::marking_intents, make_marking_intents);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::marking_continuity, make_marking_continuity);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::markings, resolve_markings);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);
  result = run(stage::draw, emit);
  if (!result.ok)
    return Result<DerivedRoad>::Fail(result.error_kind, result.error);

  Result<bool> begun = pipe.begin(stage::invariant);
  if (!begun.ok)
    return Result<DerivedRoad>::Fail(begun.error_kind, begun.error);
  pipe.out.build_stage_runs = pipe.runs;
  Result<bool> invariant = ValidateGraphInvariants(source, pipe.out);
  if (!invariant.ok) {
    return Result<DerivedRoad>::Fail(invariant.error_kind, invariant.error);
  }
  return Result<DerivedRoad>::Ok(std::move(pipe.out));
}

} // namespace city::road::build
