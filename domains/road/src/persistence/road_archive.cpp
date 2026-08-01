#include "road_archive.hpp"

#include "../lookup.hpp"
#include "schema.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>

namespace city::road::persistence {
namespace {

bool finite(double value) {
  return std::isfinite(value);
}

bool finite(Vec2d value) {
  return finite(value.x) && finite(value.y);
}

bool finite(const BezierSpan& span) {
  return finite(span.p0) && finite(span.p1) && finite(span.p2) &&
         finite(span.p3);
}

bool finite(const SegmentShape& shape) {
  if (!finite(shape.start_handle) || !finite(shape.end_handle)) return false;
  return std::all_of(shape.internal_knots.begin(), shape.internal_knots.end(),
                     [](const SegmentKnot& knot) {
                       return finite(knot.position) && finite(knot.handle_in) &&
                              finite(knot.handle_out);
                     });
}

template <typename T>
std::vector<const T*> sorted_by_id(const std::vector<T>& items) {
  std::vector<const T*> sorted{};
  sorted.reserve(items.size());
  for (const T& item : items) sorted.push_back(&item);
  std::sort(sorted.begin(), sorted.end(), [](const T* a, const T* b) {
    return a->id < b->id;
  });
  return sorted;
}

std::string double_text(double value) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10)
         << value;
  return stream.str();
}

class ArchiveWriter {
public:
  void UInt(std::string_view key, std::uint64_t value) {
    out_ << key << '=' << value << '\n';
  }
  void Int(std::string_view key, int value) {
    out_ << key << '=' << value << '\n';
  }
  void Double(std::string_view key, double value) {
    out_ << key << '=' << double_text(value) << '\n';
  }
  [[nodiscard]] std::string Finish() const { return out_.str(); }

private:
  std::ostringstream out_{};
};

std::optional<std::uint64_t> parse_u64(std::string_view text) {
  std::uint64_t value = 0;
  const char* first = text.data();
  const char* last = text.data() + text.size();
  const std::from_chars_result result = std::from_chars(first, last, value);
  if (result.ec != std::errc{} || result.ptr != last) return std::nullopt;
  return value;
}

std::optional<int> parse_int(std::string_view text) {
  int value = 0;
  const char* first = text.data();
  const char* last = text.data() + text.size();
  const std::from_chars_result result = std::from_chars(first, last, value);
  if (result.ec != std::errc{} || result.ptr != last) return std::nullopt;
  return value;
}

std::optional<double> parse_double(std::string_view text) {
  if (text.empty()) return std::nullopt;
  double value = 0.0;
  std::istringstream stream{std::string(text)};
  stream.imbue(std::locale::classic());
  stream >> std::noskipws >> value;
  if (!stream || !stream.eof() || !finite(value)) return std::nullopt;
  return value;
}

class ArchiveReader {
public:
  explicit ArchiveReader(std::string_view text) {
    std::istringstream input{std::string(text)};
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty()) continue;
      const std::size_t eq = line.find('=');
      if (eq == std::string::npos || eq == 0) {
        error_ = "road archive line lacks key";
        return;
      }
      const std::string key = line.substr(0, eq);
      const std::string value = line.substr(eq + 1);
      if (!fields_.emplace(key, value).second) {
        error_ = "duplicate road archive key";
        return;
      }
    }
  }

  Result<bool> Status() const {
    if (!error_.empty()) return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, error_);
    return Result<bool>::Ok(true);
  }

  Result<std::string> RequireString(const std::string& key) {
    auto found = fields_.find(key);
    if (found == fields_.end()) {
      return Result<std::string>::Fail(CommitFailureCategory::kInvalidInput,
                                       "missing road archive key: " + key);
    }
    consumed_.insert(key);
    return Result<std::string>::Ok(found->second);
  }

  Result<std::uint64_t> RequireU64(const std::string& key) {
    Result<std::string> text = RequireString(key);
    if (!text.ok) return Result<std::uint64_t>::Fail(text.failure_category, text.error);
    std::optional<std::uint64_t> parsed = parse_u64(text.value);
    if (!parsed.has_value()) {
      return Result<std::uint64_t>::Fail(CommitFailureCategory::kInvalidInput,
                                         "invalid integer road archive key: " + key);
    }
    return Result<std::uint64_t>::Ok(*parsed);
  }

  Result<int> RequireInt(const std::string& key) {
    Result<std::string> text = RequireString(key);
    if (!text.ok) return Result<int>::Fail(text.failure_category, text.error);
    std::optional<int> parsed = parse_int(text.value);
    if (!parsed.has_value()) {
      return Result<int>::Fail(CommitFailureCategory::kInvalidInput,
                               "invalid enum road archive key: " + key);
    }
    return Result<int>::Ok(*parsed);
  }

  Result<double> RequireDouble(const std::string& key) {
    Result<std::string> text = RequireString(key);
    if (!text.ok) return Result<double>::Fail(text.failure_category, text.error);
    std::optional<double> parsed = parse_double(text.value);
    if (!parsed.has_value()) {
      return Result<double>::Fail(CommitFailureCategory::kInvalidInput,
                                  "invalid double road archive key: " + key);
    }
    return Result<double>::Ok(*parsed);
  }

  Result<bool> Finish() const {
    for (const auto& [key, value] : fields_) {
      (void)value;
      if (!consumed_.contains(key)) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "unknown road archive key: " + key);
      }
    }
    return Result<bool>::Ok(true);
  }

private:
  std::map<std::string, std::string> fields_{};
  std::set<std::string> consumed_{};
  std::string error_{};
};

template <typename Enum>
Result<Enum> enum_value(ArchiveReader& reader, const std::string& key,
                        int min_value, int max_value) {
  Result<int> parsed = reader.RequireInt(key);
  if (!parsed.ok) return Result<Enum>::Fail(parsed.failure_category, parsed.error);
  if (parsed.value < min_value || parsed.value > max_value) {
    return Result<Enum>::Fail(CommitFailureCategory::kInvalidInput,
                              "enum road archive key is out of range: " + key);
  }
  return Result<Enum>::Ok(static_cast<Enum>(parsed.value));
}

Result<Vec2d> vec2(ArchiveReader& reader, const std::string& prefix) {
  Result<double> x = reader.RequireDouble(prefix + ".x");
  Result<double> y = reader.RequireDouble(prefix + ".y");
  if (!x.ok) return Result<Vec2d>::Fail(x.failure_category, x.error);
  if (!y.ok) return Result<Vec2d>::Fail(y.failure_category, y.error);
  return Result<Vec2d>::Ok(Vec2d{x.value, y.value});
}

void write_vec2(ArchiveWriter& writer, const std::string& prefix, Vec2d value) {
  writer.Double(prefix + ".x", value.x);
  writer.Double(prefix + ".y", value.y);
}

void write_span(ArchiveWriter& writer, const std::string& prefix,
                const BezierSpan& span) {
  write_vec2(writer, prefix + ".p0", span.p0);
  write_vec2(writer, prefix + ".p1", span.p1);
  write_vec2(writer, prefix + ".p2", span.p2);
  write_vec2(writer, prefix + ".p3", span.p3);
}

Result<BezierSpan> read_span(ArchiveReader& reader, const std::string& prefix) {
  Result<Vec2d> p0 = vec2(reader, prefix + ".p0");
  Result<Vec2d> p1 = vec2(reader, prefix + ".p1");
  Result<Vec2d> p2 = vec2(reader, prefix + ".p2");
  Result<Vec2d> p3 = vec2(reader, prefix + ".p3");
  if (!p0.ok) return Result<BezierSpan>::Fail(p0.failure_category, p0.error);
  if (!p1.ok) return Result<BezierSpan>::Fail(p1.failure_category, p1.error);
  if (!p2.ok) return Result<BezierSpan>::Fail(p2.failure_category, p2.error);
  if (!p3.ok) return Result<BezierSpan>::Fail(p3.failure_category, p3.error);
  return Result<BezierSpan>::Ok(BezierSpan{p0.value, p1.value, p2.value, p3.value});
}

bool contains_id(const std::set<std::uint64_t>& ids, std::uint64_t id) {
  return id != 0 && ids.contains(id);
}

void write_approach_key(ArchiveWriter& writer, const std::string& prefix,
                        ApproachKey key) {
  writer.UInt(prefix + ".node_id", key.node_id);
  writer.UInt(prefix + ".segment_id", key.segment_id);
  writer.Int(prefix + ".endpoint_role", static_cast<int>(key.endpoint_role));
}

Result<ApproachKey> read_approach_key(ArchiveReader& reader,
                                      const std::string& prefix) {
  Result<std::uint64_t> node_id = reader.RequireU64(prefix + ".node_id");
  Result<std::uint64_t> segment_id = reader.RequireU64(prefix + ".segment_id");
  Result<EndpointRole> endpoint_role =
      enum_value<EndpointRole>(reader, prefix + ".endpoint_role", 0, 1);
  if (!node_id.ok) return Result<ApproachKey>::Fail(node_id.failure_category, node_id.error);
  if (!segment_id.ok) return Result<ApproachKey>::Fail(segment_id.failure_category, segment_id.error);
  if (!endpoint_role.ok) return Result<ApproachKey>::Fail(endpoint_role.failure_category, endpoint_role.error);
  return Result<ApproachKey>::Ok(ApproachKey{node_id.value, segment_id.value,
                                             endpoint_role.value});
}

void write_lane_endpoint_key(ArchiveWriter& writer, const std::string& prefix,
                             LaneEndpointKey key) {
  writer.UInt(prefix + ".segment_id", key.segment_id);
  writer.UInt(prefix + ".lane_id", key.lane_id);
  writer.Int(prefix + ".endpoint_role", static_cast<int>(key.endpoint_role));
}

Result<LaneEndpointKey> read_lane_endpoint_key(ArchiveReader& reader,
                                                const std::string& prefix) {
  Result<std::uint64_t> segment_id = reader.RequireU64(prefix + ".segment_id");
  Result<std::uint64_t> lane_id = reader.RequireU64(prefix + ".lane_id");
  Result<EndpointRole> endpoint_role =
      enum_value<EndpointRole>(reader, prefix + ".endpoint_role", 0, 1);
  if (!segment_id.ok) return Result<LaneEndpointKey>::Fail(segment_id.failure_category, segment_id.error);
  if (!lane_id.ok) return Result<LaneEndpointKey>::Fail(lane_id.failure_category, lane_id.error);
  if (!endpoint_role.ok) return Result<LaneEndpointKey>::Fail(endpoint_role.failure_category, endpoint_role.error);
  return Result<LaneEndpointKey>::Ok(
      LaneEndpointKey{segment_id.value, lane_id.value, endpoint_role.value});
}

void write_boundary_endpoint_key(ArchiveWriter& writer, const std::string& prefix,
                                 BoundaryEndpointKey key) {
  writer.UInt(prefix + ".segment_id", key.segment_id);
  writer.UInt(prefix + ".boundary_id", key.boundary_id);
  writer.Int(prefix + ".endpoint_role", static_cast<int>(key.endpoint_role));
}

Result<BoundaryEndpointKey> read_boundary_endpoint_key(
    ArchiveReader& reader, const std::string& prefix) {
  Result<std::uint64_t> segment_id = reader.RequireU64(prefix + ".segment_id");
  Result<std::uint64_t> boundary_id = reader.RequireU64(prefix + ".boundary_id");
  Result<EndpointRole> endpoint_role =
      enum_value<EndpointRole>(reader, prefix + ".endpoint_role", 0, 1);
  if (!segment_id.ok) return Result<BoundaryEndpointKey>::Fail(segment_id.failure_category, segment_id.error);
  if (!boundary_id.ok) return Result<BoundaryEndpointKey>::Fail(boundary_id.failure_category, boundary_id.error);
  if (!endpoint_role.ok) return Result<BoundaryEndpointKey>::Fail(endpoint_role.failure_category, endpoint_role.error);
  return Result<BoundaryEndpointKey>::Ok(
      BoundaryEndpointKey{segment_id.value, boundary_id.value, endpoint_role.value});
}

void write_marking_owner(ArchiveWriter& writer, const std::string& prefix,
                         MarkingOwner owner) {
  writer.Int(prefix + ".kind", static_cast<int>(owner.kind));
  writer.UInt(prefix + ".segment_id", owner.segment_id);
  writer.UInt(prefix + ".node_id", owner.node_id);
  writer.UInt(prefix + ".manual_id", owner.manual_id);
}

Result<MarkingOwner> read_marking_owner(ArchiveReader& reader,
                                        const std::string& prefix) {
  Result<MarkingOwner::Kind> kind =
      enum_value<MarkingOwner::Kind>(reader, prefix + ".kind", 0, 2);
  Result<std::uint64_t> segment_id = reader.RequireU64(prefix + ".segment_id");
  Result<std::uint64_t> node_id = reader.RequireU64(prefix + ".node_id");
  Result<std::uint64_t> manual_id = reader.RequireU64(prefix + ".manual_id");
  if (!kind.ok) return Result<MarkingOwner>::Fail(kind.failure_category, kind.error);
  if (!segment_id.ok) return Result<MarkingOwner>::Fail(segment_id.failure_category, segment_id.error);
  if (!node_id.ok) return Result<MarkingOwner>::Fail(node_id.failure_category, node_id.error);
  if (!manual_id.ok) return Result<MarkingOwner>::Fail(manual_id.failure_category, manual_id.error);
  return Result<MarkingOwner>::Ok(
      MarkingOwner{kind.value, segment_id.value, node_id.value, manual_id.value});
}

void write_auto_marking_key(ArchiveWriter& writer, const std::string& prefix,
                            const AutoMarkingKey& key) {
  write_marking_owner(writer, prefix + ".owner", key.owner);
  writer.Int(prefix + ".role", static_cast<int>(key.role));
  writer.Int(prefix + ".track.has_value", key.track.has_value() ? 1 : 0);
  if (key.track.has_value()) {
    writer.UInt(prefix + ".track.segment_id", key.track->segment_id);
    writer.UInt(prefix + ".track.boundary_id", key.track->boundary_id);
    writer.Int(prefix + ".track.role", static_cast<int>(key.track->role));
  }
  writer.Int(prefix + ".approach.has_value", key.approach.has_value() ? 1 : 0);
  if (key.approach.has_value()) {
    write_approach_key(writer, prefix + ".approach", *key.approach);
  }
}

Result<AutoMarkingKey> read_auto_marking_key(ArchiveReader& reader,
                                             const std::string& prefix) {
  Result<MarkingOwner> owner = read_marking_owner(reader, prefix + ".owner");
  Result<MarkingRole> role = enum_value<MarkingRole>(reader, prefix + ".role", 0, 5);
  Result<int> track_has = reader.RequireInt(prefix + ".track.has_value");
  Result<int> approach_has = reader.RequireInt(prefix + ".approach.has_value");
  if (!owner.ok) return Result<AutoMarkingKey>::Fail(owner.failure_category, owner.error);
  if (!role.ok) return Result<AutoMarkingKey>::Fail(role.failure_category, role.error);
  if (!track_has.ok) return Result<AutoMarkingKey>::Fail(track_has.failure_category, track_has.error);
  if (!approach_has.ok) return Result<AutoMarkingKey>::Fail(approach_has.failure_category, approach_has.error);
  if ((track_has.value != 0 && track_has.value != 1) ||
      (approach_has.value != 0 && approach_has.value != 1)) {
    return Result<AutoMarkingKey>::Fail(CommitFailureCategory::kInvalidInput,
                                        "auto marking key optional flag is invalid");
  }
  AutoMarkingKey key{};
  key.owner = owner.value;
  key.role = role.value;
  if (track_has.value == 1) {
    Result<std::uint64_t> segment_id = reader.RequireU64(prefix + ".track.segment_id");
    Result<std::uint64_t> boundary_id = reader.RequireU64(prefix + ".track.boundary_id");
    Result<MarkingRole> track_role =
        enum_value<MarkingRole>(reader, prefix + ".track.role", 0, 5);
    if (!segment_id.ok) return Result<AutoMarkingKey>::Fail(segment_id.failure_category, segment_id.error);
    if (!boundary_id.ok) return Result<AutoMarkingKey>::Fail(boundary_id.failure_category, boundary_id.error);
    if (!track_role.ok) return Result<AutoMarkingKey>::Fail(track_role.failure_category, track_role.error);
    key.track = MarkingTrackKey{segment_id.value, boundary_id.value, track_role.value};
  }
  if (approach_has.value == 1) {
    Result<ApproachKey> approach = read_approach_key(reader, prefix + ".approach");
    if (!approach.ok) return Result<AutoMarkingKey>::Fail(approach.failure_category, approach.error);
    key.approach = approach.value;
  }
  return Result<AutoMarkingKey>::Ok(key);
}

} // namespace

Result<bool> ValidateAuthoritativeGraph(const SavedRoadGraph& graph,
                                        std::uint64_t next_id) {
  if (next_id == 0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road next_id is invalid");
  }
  std::set<std::uint64_t> all_ids{};
  std::set<std::uint64_t> node_ids{};
  std::set<std::uint64_t> segment_ids{};
  std::set<std::uint64_t> corridor_ids{};
  std::set<std::uint64_t> template_ids{};
  std::set<std::uint64_t> transition_ids{};
  std::set<std::uint64_t> policy_ids{};
  std::set<std::uint64_t> marking_ids{};
  std::set<std::uint64_t> junction_marking_ids{};
  std::set<std::uint64_t> lane_connection_ids{};
  std::set<std::uint64_t> boundary_continuation_ids{};
  std::uint64_t max_id = 0;
  const auto add_id = [&](std::uint64_t id, std::set<std::uint64_t>* domain,
                          std::string_view label) -> Result<bool> {
    if (id == 0 || !all_ids.insert(id).second || !domain->insert(id).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                std::string("road duplicate or zero ID: ") +
                                    std::string(label));
    }
    max_id = std::max(max_id, id);
    return Result<bool>::Ok(true);
  };
  for (const CrossSectionTemplate& section : graph.section_templates) {
    Result<bool> id = add_id(section.id, &template_ids, "section_template");
    if (!id.ok) return id;
    if (section.strips.empty() ||
        section.boundaries.size() + 1 != section.strips.size()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section template chain is incomplete");
    }
    std::set<SectionStripId> strips{};
    for (const SectionStrip& strip : section.strips) {
      if (strip.id == 0 || !strips.insert(strip.id).second ||
          !finite(strip.width_m) || !finite(strip.cross_slope) ||
          strip.width_m <= 0.0 ||
          static_cast<int>(strip.function) < 0 ||
          static_cast<int>(strip.function) > 3 ||
          !IsKnownSurfaceStyle(strip.style_id)) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template strip is invalid");
      }
      for (const AutoMarkingPolicy& side :
           {strip.side_marking.left, strip.side_marking.right}) {
        if (static_cast<int>(side.role) < 0 || static_cast<int>(side.role) > 5 ||
            (side.enabled && !IsKnownMarkingStyle(side.style_id))) {
          return Result<bool>::Fail(
              CommitFailureCategory::kInvalidInput,
              "section template lane side marking is invalid");
        }
      }
    }
    std::set<LaneId> lane_ids{};
    for (const LaneBand& lane : section.lane_bands) {
      const auto strip = std::find_if(
          section.strips.begin(), section.strips.end(),
          [&lane](const SectionStrip& candidate) {
            return candidate.id == lane.surface_strip_id;
          });
      if (lane.id == 0 || !lane_ids.insert(lane.id).second ||
          strip == section.strips.end() ||
          strip->function != StripFunction::kCarriageway ||
          !finite(lane.lateral_start_m) || !finite(lane.lateral_end_m) ||
          static_cast<int>(lane.direction) < 0 ||
          static_cast<int>(lane.direction) > 1 ||
          lane.lateral_start_m < 0.0 ||
          lane.lateral_end_m <= lane.lateral_start_m ||
          lane.lateral_end_m > strip->width_m) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template lane allocation is invalid");
      }
    }
    for (std::size_t i = 0; i < section.lane_bands.size(); ++i) {
      for (std::size_t j = i + 1; j < section.lane_bands.size(); ++j) {
        const LaneBand& a = section.lane_bands[i];
        const LaneBand& b = section.lane_bands[j];
        if (a.surface_strip_id != b.surface_strip_id) continue;
        const bool overlaps =
            std::max(a.lateral_start_m, b.lateral_start_m) <
            std::min(a.lateral_end_m, b.lateral_end_m);
        if (overlaps) {
          return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                    "section template lane allocations overlap");
        }
      }
    }
    std::set<std::uint64_t> boundaries{};
    for (const BoundaryProfile& boundary : section.boundaries) {
      if (boundary.boundary_id == 0 ||
          !boundaries.insert(boundary.boundary_id).second ||
          !finite(boundary.width_m) || !finite(boundary.height_m) ||
          boundary.width_m < 0.0 ||
          static_cast<int>(boundary.role) < 0 ||
          static_cast<int>(boundary.role) > 3 ||
          static_cast<int>(boundary.marking.role) < 0 ||
          static_cast<int>(boundary.marking.role) > 5 ||
          (boundary.marking.enabled &&
           !IsKnownMarkingStyle(boundary.marking.style_id))) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template boundary is invalid");
      }
    }
  }
  for (const RoadNode& node : graph.nodes) {
    Result<bool> id = add_id(node.id, &node_ids, "node");
    if (!id.ok) return id;
    if (!finite(node.position)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road node position is non-finite");
    }
  }
  for (const SectionTransition& transition : graph.transitions) {
    Result<bool> id = add_id(transition.id, &transition_ids, "transition");
    if (!id.ok) return id;
    if (!contains_id(template_ids, transition.from_template) ||
        !contains_id(template_ids, transition.to_template) ||
        !finite(transition.start.value) || !finite(transition.end.value) ||
        static_cast<int>(transition.start.kind) < 0 ||
        static_cast<int>(transition.start.kind) > 2 ||
        static_cast<int>(transition.end.kind) < 0 ||
        static_cast<int>(transition.end.kind) > 2 ||
        static_cast<int>(transition.anchor) < 0 ||
        static_cast<int>(transition.anchor) > 3 ||
        (transition.anchor == TransitionAnchor::kBoundary) !=
            (transition.anchor_boundary_id != 0)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section transition is invalid");
    }
    if (transition.anchor == TransitionAnchor::kBoundary) {
      const auto has_anchor = [&graph, &transition](CrossSectionTemplateId id) {
        const auto section = std::find_if(
            graph.section_templates.begin(), graph.section_templates.end(),
            [id](const CrossSectionTemplate& candidate) {
              return candidate.id == id;
            });
        return section != graph.section_templates.end() &&
               std::any_of(section->boundaries.begin(),
                           section->boundaries.end(),
                           [&transition](const BoundaryProfile& boundary) {
                             return boundary.boundary_id ==
                                    transition.anchor_boundary_id;
                           });
      };
      if (!has_anchor(transition.from_template) ||
          !has_anchor(transition.to_template)) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInvalidInput,
            "section transition anchor boundary reference is invalid");
      }
    }
    std::set<SectionStripId> rule_strips{};
    for (const SectionTransitionRule& rule : transition.rules) {
      if (rule.strip_id == 0 ||
          !rule_strips.insert(rule.strip_id).second ||
          static_cast<int>(rule.action) < 0 ||
          static_cast<int>(rule.action) > 5) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section transition rule is invalid");
      }
    }
  }
  for (const RoadSegment& segment : graph.segments) {
    Result<bool> id = add_id(segment.id, &segment_ids, "segment");
    if (!id.ok) return id;
    if (!contains_id(node_ids, segment.node_a) ||
        !contains_id(node_ids, segment.node_b) ||
        !contains_id(template_ids, segment.section_template) ||
        (segment.transition.has_value() &&
         !contains_id(transition_ids, *segment.transition)) ||
        !finite(segment.shape) ||
        static_cast<int>(segment.shape.intent) < 0 ||
        static_cast<int>(segment.shape.intent) > 1) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road segment is invalid");
    }
  }
  std::set<std::pair<LaneEndpointKey, LaneEndpointKey>> lane_pairs{};
  std::map<LaneEndpointKey, LaneConnectionKind> lane_sources{};
  std::map<LaneEndpointKey, LaneConnectionKind> lane_targets{};
  const auto lane_exits = [](const internal::LaneEndpointLookup& endpoint,
                             EndpointRole role) {
    return endpoint.lane != nullptr &&
           ((endpoint.lane->direction == LaneTravelDirection::kAlongSegment &&
             role == EndpointRole::kEnd) ||
            (endpoint.lane->direction == LaneTravelDirection::kAgainstSegment &&
             role == EndpointRole::kStart));
  };
  const auto lane_enters = [](const internal::LaneEndpointLookup& endpoint,
                              EndpointRole role) {
    return endpoint.lane != nullptr &&
           ((endpoint.lane->direction == LaneTravelDirection::kAlongSegment &&
             role == EndpointRole::kStart) ||
            (endpoint.lane->direction == LaneTravelDirection::kAgainstSegment &&
             role == EndpointRole::kEnd));
  };
  for (const LaneConnection& connection : graph.lane_connections) {
    Result<bool> id = add_id(connection.id, &lane_connection_ids, "lane_connection");
    if (!id.ok) return id;
    if (connection.source.segment_id == 0 || connection.source.lane_id == 0 ||
        connection.target.segment_id == 0 || connection.target.lane_id == 0 ||
        static_cast<int>(connection.source.endpoint_role) < 0 ||
        static_cast<int>(connection.source.endpoint_role) > 1 ||
        static_cast<int>(connection.target.endpoint_role) < 0 ||
        static_cast<int>(connection.target.endpoint_role) > 1 ||
        static_cast<int>(connection.kind) < 0 ||
        static_cast<int>(connection.kind) > 4) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane connection identity is invalid");
    }
    const internal::LaneEndpointLookup source =
        internal::find_lane_endpoint(graph, connection.source);
    const internal::LaneEndpointLookup target =
        internal::find_lane_endpoint(graph, connection.target);
    if (!lane_exits(source, connection.source.endpoint_role) ||
        !lane_enters(target, connection.target.endpoint_role) ||
        source.node_id == 0 || source.node_id != target.node_id ||
        !lane_pairs.insert({connection.source, connection.target}).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane connection endpoints are invalid");
    }
    const auto source_use = lane_sources.find(connection.source);
    if (source_use != lane_sources.end() &&
        (source_use->second != connection.kind ||
         (connection.kind != LaneConnectionKind::kSplit &&
          connection.kind != LaneConnectionKind::kJunctionMovement))) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane connection source is ambiguous");
    }
    lane_sources.emplace(connection.source, connection.kind);
    const auto target_use = lane_targets.find(connection.target);
    if (target_use != lane_targets.end() &&
        (target_use->second != connection.kind ||
         (connection.kind != LaneConnectionKind::kMerge &&
          connection.kind != LaneConnectionKind::kJunctionMovement))) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane connection target is ambiguous");
    }
    lane_targets.emplace(connection.target, connection.kind);
  }
  std::map<BoundaryEndpointKey, BoundaryContinuationKind> boundary_sources{};
  std::map<BoundaryEndpointKey, BoundaryContinuationKind> boundary_targets{};
  std::set<std::pair<BoundaryEndpointKey, BoundaryEndpointKey>> boundary_pairs{};
  for (const BoundaryContinuation& continuation : graph.boundary_continuations) {
    Result<bool> id = add_id(continuation.id, &boundary_continuation_ids,
                             "boundary_continuation");
    if (!id.ok) return id;
    if (continuation.source.segment_id == 0 || continuation.source.boundary_id == 0 ||
        continuation.target.segment_id == 0 || continuation.target.boundary_id == 0 ||
        static_cast<int>(continuation.source.endpoint_role) < 0 ||
        static_cast<int>(continuation.source.endpoint_role) > 1 ||
        static_cast<int>(continuation.target.endpoint_role) < 0 ||
        static_cast<int>(continuation.target.endpoint_role) > 1 ||
        static_cast<int>(continuation.kind) < 0 ||
        static_cast<int>(continuation.kind) > 2) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "boundary continuation identity is invalid");
    }
    const internal::BoundaryEndpointLookup source =
        internal::find_boundary_endpoint(graph, continuation.source);
    const internal::BoundaryEndpointLookup target =
        internal::find_boundary_endpoint(graph, continuation.target);
    if (source.boundary == nullptr || target.boundary == nullptr ||
        source.node_id == 0 || source.node_id != target.node_id ||
        !boundary_pairs.insert({continuation.source, continuation.target}).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "boundary continuation endpoints are invalid");
    }
    const auto source_use = boundary_sources.find(continuation.source);
    if (source_use != boundary_sources.end() &&
        (source_use->second != continuation.kind ||
         continuation.kind != BoundaryContinuationKind::kSplit)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "boundary continuation source is ambiguous");
    }
    boundary_sources.emplace(continuation.source, continuation.kind);
    const auto target_use = boundary_targets.find(continuation.target);
    if (target_use != boundary_targets.end() &&
        (target_use->second != continuation.kind ||
         continuation.kind != BoundaryContinuationKind::kMerge)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "boundary continuation target is ambiguous");
    }
    boundary_targets.emplace(continuation.target, continuation.kind);
  }
  std::set<RoadSegmentId> corridor_segments{};
  const auto segment_for = [&](RoadSegmentId id) -> const RoadSegment* {
    const auto it =
        std::find_if(graph.segments.begin(), graph.segments.end(),
                     [id](const RoadSegment& segment) {
                       return segment.id == id;
                     });
    return it == graph.segments.end() ? nullptr : &*it;
  };
  for (const RoadCorridor& corridor : graph.corridors) {
    Result<bool> id = add_id(corridor.id, &corridor_ids, "corridor");
    if (!id.ok) return id;
    if (corridor.section_template_id == 0 || corridor.segments.empty()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "road corridor is empty or untyped");
    }
    RoadNodeId expected_start = 0;
    for (std::size_t index = 0; index < corridor.segments.size(); ++index) {
      const DirectedSegmentRef& ref = corridor.segments[index];
      const RoadSegment* segment = segment_for(ref.segment_id);
      if (segment == nullptr ||
          segment->section_template != corridor.section_template_id ||
          !corridor_segments.insert(ref.segment_id).second) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInvalidInput,
            "road corridor segment reference is invalid or duplicated");
      }
      const RoadNodeId start =
          ref.reversed ? segment->node_b : segment->node_a;
      const RoadNodeId end =
          ref.reversed ? segment->node_a : segment->node_b;
      if (index != 0 && start != expected_start) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInvalidInput,
            "road corridor segment references are not endpoint-continuous");
      }
      expected_start = end;
    }
  }
  if (corridor_segments.size() != graph.segments.size()) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road segment does not belong to exactly one corridor");
  }
  for (const NodeConnectionPolicyOverride& policy :
       graph.connection_policy_overrides) {
    Result<bool> id = add_id(policy.id, &policy_ids, "connection_policy");
    if (!id.ok) return id;
    if (!contains_id(node_ids, policy.node_id) ||
        static_cast<int>(policy.policy) < 0 ||
        static_cast<int>(policy.policy) > 2) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "connection policy override is invalid");
    }
  }
  for (const ApproachGeometryOverride& override :
       graph.approach_geometry_overrides) {
    const RoadSegment* segment = nullptr;
    for (const RoadSegment& candidate : graph.segments) {
      if (candidate.id == override.key.segment_id) {
        segment = &candidate;
        break;
      }
    }
    const bool endpoint_matches =
        segment != nullptr &&
        ((override.key.endpoint_role == EndpointRole::kStart &&
          segment->node_a == override.key.node_id) ||
         (override.key.endpoint_role == EndpointRole::kEnd &&
          segment->node_b == override.key.node_id));
    if (!endpoint_matches ||
        (!override.setback_m.has_value && !override.lateral_shift_m.has_value) ||
        (override.setback_m.has_value &&
         (!finite(override.setback_m.value) || override.setback_m.value < 0.0)) ||
        (override.lateral_shift_m.has_value &&
         !finite(override.lateral_shift_m.value))) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "approach geometry override is invalid");
    }
  }
  std::set<AutoMarkingKey> auto_marking_keys{};
  for (const AutoMarkingOverride& override : graph.auto_marking_overrides) {
    if (!auto_marking_keys.insert(override.key).second ||
        static_cast<int>(override.key.role) < 0 ||
        static_cast<int>(override.key.role) > 5) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "auto marking override is invalid");
    }
    if (override.key.owner.kind == MarkingOwner::Kind::kRoadSegment) {
      if (!contains_id(segment_ids, override.key.owner.segment_id) ||
          override.key.owner.node_id != 0 || override.key.owner.manual_id != 0 ||
          !override.key.track.has_value() || override.key.approach.has_value() ||
          override.key.track->segment_id != override.key.owner.segment_id ||
          override.key.track->role != override.key.role) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "auto marking segment override is invalid");
      }
    } else if (override.key.owner.kind == MarkingOwner::Kind::kJunction) {
      if (!contains_id(node_ids, override.key.owner.node_id) ||
          override.key.owner.segment_id != 0 || override.key.owner.manual_id != 0 ||
          override.key.track.has_value() || !override.key.approach.has_value()) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "auto marking junction override is invalid");
      }
    } else {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "auto marking manual override is invalid");
    }
  }
  for (const JunctionMarkingOverride& override :
       graph.junction_marking_overrides) {
    Result<bool> id = add_id(override.id, &junction_marking_ids,
                             "junction_marking_override");
    if (!id.ok) return id;
    if (!contains_id(node_ids, override.node_id) ||
        static_cast<int>(override.action) < 0 ||
        static_cast<int>(override.action) > 2) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "junction marking override is invalid");
    }
    const auto valid_endpoint = [&](const JunctionMarkingEndpoint& endpoint) {
      const auto segment =
          std::find_if(graph.segments.begin(), graph.segments.end(),
                       [&](const RoadSegment& candidate) {
                         return candidate.id == endpoint.approach.segment_id;
                       });
      return segment != graph.segments.end() &&
             endpoint.approach.node_id == override.node_id &&
             ((endpoint.approach.endpoint_role == EndpointRole::kStart &&
               segment->node_a == endpoint.approach.node_id) ||
              (endpoint.approach.endpoint_role == EndpointRole::kEnd &&
               segment->node_b == endpoint.approach.node_id)) &&
             endpoint.boundary_id != 0 &&
             static_cast<int>(endpoint.role) >= 0 &&
             static_cast<int>(endpoint.role) <= 5;
    };
    if (!valid_endpoint(override.source) ||
        (override.action == JunctionMarkingAction::kConnectToApproach &&
         (!override.target.has_value() || !valid_endpoint(*override.target))) ||
        (override.action != JunctionMarkingAction::kConnectToApproach &&
         override.target.has_value())) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "junction marking endpoint is invalid");
    }
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    Result<bool> id = add_id(marking.id, &marking_ids, "manual_line");
    if (!id.ok) return id;
    if (!contains_id(segment_ids, marking.owner_segment_id) ||
        !IsKnownMarkingStyle(marking.style_id) || marking.path.spans.empty() ||
        !std::all_of(marking.path.spans.begin(), marking.path.spans.end(),
                     [](const BezierSpan& span) { return finite(span); })) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "manual line marking is invalid");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    Result<bool> id = add_id(marking.id, &marking_ids, "manual_area");
    if (!id.ok) return id;
    if (!contains_id(segment_ids, marking.owner_segment_id) ||
        !IsKnownMarkingStyle(marking.style_id) ||
        !finite(marking.frame_origin) || !finite(marking.rotation_rad) ||
        !finite(marking.width_m) ||
        !finite(marking.length_m) || marking.width_m <= 0.0 ||
        marking.length_m <= 0.0) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "manual area marking is invalid");
    }
  }
  if (next_id <= max_id) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "road next_id does not exceed existing IDs");
  }
  return Result<bool>::Ok(true);
}

Result<std::string> SaveRoad(const SavedRoadGraph& graph,
                             std::uint64_t next_id) {
  Result<bool> valid = ValidateAuthoritativeGraph(graph, next_id);
  if (!valid.ok) return Result<std::string>::Fail(valid.failure_category, valid.error);
  ArchiveWriter writer{};
  writer.UInt("road_graph_version", kVersion);
  writer.UInt("next_id", next_id);

  const auto sections = sorted_by_id(graph.section_templates);
  writer.UInt("section_template.count", sections.size());
  for (std::size_t i = 0; i < sections.size(); ++i) {
    const CrossSectionTemplate& section = *sections[i];
    const std::string prefix = "section_template." + std::to_string(i);
    writer.UInt(prefix + ".id", section.id);
    writer.UInt(prefix + ".strip.count", section.strips.size());
    for (std::size_t j = 0; j < section.strips.size(); ++j) {
      const SectionStrip& strip = section.strips[j];
      const std::string strip_prefix =
          prefix + ".strip." + std::to_string(j);
      writer.UInt(strip_prefix + ".id", strip.id);
      writer.Int(strip_prefix + ".function",
                 static_cast<int>(strip.function));
      writer.Double(strip_prefix + ".width_m", strip.width_m);
      writer.Double(strip_prefix + ".cross_slope", strip.cross_slope);
      writer.UInt(strip_prefix + ".style_id", strip.style_id.value);
      for (const auto& [side_key, side] :
           {std::pair{std::string_view{"left"}, strip.side_marking.left},
            std::pair{std::string_view{"right"}, strip.side_marking.right}}) {
        const std::string side_prefix =
            strip_prefix + ".side_marking." + std::string(side_key);
        writer.Int(side_prefix + ".enabled", side.enabled ? 1 : 0);
        writer.Int(side_prefix + ".role", static_cast<int>(side.role));
        writer.UInt(side_prefix + ".style_id", side.style_id.value);
      }
    }
    writer.UInt(prefix + ".lane_band.count", section.lane_bands.size());
    for (std::size_t j = 0; j < section.lane_bands.size(); ++j) {
      const LaneBand& lane = section.lane_bands[j];
      const std::string lane_prefix =
          prefix + ".lane_band." + std::to_string(j);
      writer.UInt(lane_prefix + ".id", lane.id);
      writer.UInt(lane_prefix + ".surface_strip_id", lane.surface_strip_id);
      writer.Double(lane_prefix + ".lateral_start_m", lane.lateral_start_m);
      writer.Double(lane_prefix + ".lateral_end_m", lane.lateral_end_m);
      writer.Int(lane_prefix + ".direction", static_cast<int>(lane.direction));
    }
    writer.UInt(prefix + ".boundary.count", section.boundaries.size());
    for (std::size_t j = 0; j < section.boundaries.size(); ++j) {
      const BoundaryProfile& boundary = section.boundaries[j];
      const std::string boundary_prefix =
          prefix + ".boundary." + std::to_string(j);
      writer.UInt(boundary_prefix + ".boundary_id", boundary.boundary_id);
      writer.Int(boundary_prefix + ".role", static_cast<int>(boundary.role));
      writer.Double(boundary_prefix + ".width_m", boundary.width_m);
      writer.Double(boundary_prefix + ".height_m", boundary.height_m);
      writer.Int(boundary_prefix + ".marking.enabled",
                 boundary.marking.enabled ? 1 : 0);
      writer.Int(boundary_prefix + ".marking.role",
                 static_cast<int>(boundary.marking.role));
      writer.UInt(boundary_prefix + ".marking.style_id",
                  boundary.marking.style_id.value);
    }
  }

  const auto transitions = sorted_by_id(graph.transitions);
  writer.UInt("transition.count", transitions.size());
  for (std::size_t i = 0; i < transitions.size(); ++i) {
    const SectionTransition& transition = *transitions[i];
    const std::string prefix = "transition." + std::to_string(i);
    writer.UInt(prefix + ".id", transition.id);
    writer.UInt(prefix + ".from_template", transition.from_template);
    writer.UInt(prefix + ".to_template", transition.to_template);
    writer.Int(prefix + ".start.kind", static_cast<int>(transition.start.kind));
    writer.Double(prefix + ".start.value", transition.start.value);
    writer.Int(prefix + ".end.kind", static_cast<int>(transition.end.kind));
    writer.Double(prefix + ".end.value", transition.end.value);
    writer.Int(prefix + ".anchor", static_cast<int>(transition.anchor));
    writer.UInt(prefix + ".anchor_boundary_id",
                transition.anchor_boundary_id);
    std::vector<SectionTransitionRule> rules = transition.rules;
    std::sort(rules.begin(), rules.end(), [](const auto& a, const auto& b) {
      return a.strip_id < b.strip_id;
    });
    writer.UInt(prefix + ".rule.count", rules.size());
    for (std::size_t j = 0; j < rules.size(); ++j) {
      const std::string rule_prefix = prefix + ".rule." + std::to_string(j);
      writer.UInt(rule_prefix + ".strip_id", rules[j].strip_id);
      writer.Int(rule_prefix + ".action", static_cast<int>(rules[j].action));
    }
  }

  const auto nodes = sorted_by_id(graph.nodes);
  writer.UInt("node.count", nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const RoadNode& node = *nodes[i];
    const std::string prefix = "node." + std::to_string(i);
    writer.UInt(prefix + ".id", node.id);
    write_vec2(writer, prefix + ".position", node.position);
  }

  const auto policies = sorted_by_id(graph.connection_policy_overrides);
  writer.UInt("connection_policy_override.count", policies.size());
  for (std::size_t i = 0; i < policies.size(); ++i) {
    const NodeConnectionPolicyOverride& policy = *policies[i];
    const std::string prefix =
        "connection_policy_override." + std::to_string(i);
    writer.UInt(prefix + ".id", policy.id);
    writer.UInt(prefix + ".node_id", policy.node_id);
    writer.Int(prefix + ".policy", static_cast<int>(policy.policy));
  }

  std::vector<const ApproachGeometryOverride*> approach_overrides{};
  approach_overrides.reserve(graph.approach_geometry_overrides.size());
  for (const ApproachGeometryOverride& override : graph.approach_geometry_overrides) {
    if (override.setback_m.has_value || override.lateral_shift_m.has_value) {
      approach_overrides.push_back(&override);
    }
  }
  std::sort(approach_overrides.begin(), approach_overrides.end(), [](const auto* a, const auto* b) {
    return a->key < b->key;
  });
  writer.UInt("approach_geometry_override.count", approach_overrides.size());
  for (std::size_t i = 0; i < approach_overrides.size(); ++i) {
    const ApproachGeometryOverride& override = *approach_overrides[i];
    const std::string prefix = "approach_geometry_override." + std::to_string(i);
    write_approach_key(writer, prefix, override.key);
    writer.Int(prefix + ".setback.mode", override.setback_m.has_value ? 1 : 0);
    if (override.setback_m.has_value) writer.Double(prefix + ".setback.value", override.setback_m.value);
    writer.Int(prefix + ".lateral_shift.mode", override.lateral_shift_m.has_value ? 1 : 0);
    if (override.lateral_shift_m.has_value) {
      writer.Double(prefix + ".lateral_shift.value", override.lateral_shift_m.value);
    }
  }

  std::vector<const AutoMarkingOverride*> auto_marking_overrides{};
  auto_marking_overrides.reserve(graph.auto_marking_overrides.size());
  for (const AutoMarkingOverride& override : graph.auto_marking_overrides) {
    if (override.suppressed) auto_marking_overrides.push_back(&override);
  }
  std::sort(auto_marking_overrides.begin(), auto_marking_overrides.end(),
            [](const auto* a, const auto* b) { return a->key < b->key; });
  writer.UInt("auto_marking_override.count", auto_marking_overrides.size());
  for (std::size_t i = 0; i < auto_marking_overrides.size(); ++i) {
    const AutoMarkingOverride& override = *auto_marking_overrides[i];
    const std::string prefix = "auto_marking_override." + std::to_string(i);
    write_auto_marking_key(writer, prefix + ".key", override.key);
    writer.Int(prefix + ".suppressed", override.suppressed ? 1 : 0);
  }

  const auto junction_marking_overrides =
      sorted_by_id(graph.junction_marking_overrides);
  writer.UInt("junction_marking_override.count",
              junction_marking_overrides.size());
  for (std::size_t i = 0; i < junction_marking_overrides.size(); ++i) {
    const JunctionMarkingOverride& override = *junction_marking_overrides[i];
    const std::string prefix =
        "junction_marking_override." + std::to_string(i);
    writer.UInt(prefix + ".id", override.id);
    writer.UInt(prefix + ".node_id", override.node_id);
    write_approach_key(writer, prefix + ".source.approach",
                       override.source.approach);
    writer.UInt(prefix + ".source.boundary_id", override.source.boundary_id);
    writer.Int(prefix + ".source.role", static_cast<int>(override.source.role));
    writer.Int(prefix + ".action", static_cast<int>(override.action));
    writer.Int(prefix + ".target.has_value", override.target.has_value() ? 1 : 0);
    if (override.target.has_value()) {
      write_approach_key(writer, prefix + ".target.approach",
                         override.target->approach);
      writer.UInt(prefix + ".target.boundary_id", override.target->boundary_id);
      writer.Int(prefix + ".target.role", static_cast<int>(override.target->role));
    }
  }

  const auto segments = sorted_by_id(graph.segments);
  writer.UInt("segment.count", segments.size());
  for (std::size_t i = 0; i < segments.size(); ++i) {
    const RoadSegment& segment = *segments[i];
    const std::string prefix = "segment." + std::to_string(i);
    writer.UInt(prefix + ".id", segment.id);
    writer.UInt(prefix + ".node_a", segment.node_a);
    writer.UInt(prefix + ".node_b", segment.node_b);
    writer.UInt(prefix + ".section_template", segment.section_template);
    writer.UInt(prefix + ".transition", segment.transition.value_or(0));
    writer.Int(prefix + ".shape.intent", static_cast<int>(segment.shape.intent));
    write_vec2(writer, prefix + ".shape.start_handle", segment.shape.start_handle);
    write_vec2(writer, prefix + ".shape.end_handle", segment.shape.end_handle);
    writer.UInt(prefix + ".shape.knot.count",
                segment.shape.internal_knots.size());
    for (std::size_t j = 0; j < segment.shape.internal_knots.size(); ++j) {
      const SegmentKnot& knot = segment.shape.internal_knots[j];
      const std::string knot_prefix =
          prefix + ".shape.knot." + std::to_string(j);
      write_vec2(writer, knot_prefix + ".position", knot.position);
      write_vec2(writer, knot_prefix + ".handle_in", knot.handle_in);
      write_vec2(writer, knot_prefix + ".handle_out", knot.handle_out);
    }
  }

  const auto lane_connections = sorted_by_id(graph.lane_connections);
  writer.UInt("lane_connection.count", lane_connections.size());
  for (std::size_t i = 0; i < lane_connections.size(); ++i) {
    const LaneConnection& connection = *lane_connections[i];
    const std::string prefix = "lane_connection." + std::to_string(i);
    writer.UInt(prefix + ".id", connection.id);
    write_lane_endpoint_key(writer, prefix + ".source", connection.source);
    write_lane_endpoint_key(writer, prefix + ".target", connection.target);
    writer.Int(prefix + ".kind", static_cast<int>(connection.kind));
  }

  const auto boundary_continuations = sorted_by_id(graph.boundary_continuations);
  writer.UInt("boundary_continuation.count", boundary_continuations.size());
  for (std::size_t i = 0; i < boundary_continuations.size(); ++i) {
    const BoundaryContinuation& continuation = *boundary_continuations[i];
    const std::string prefix = "boundary_continuation." + std::to_string(i);
    writer.UInt(prefix + ".id", continuation.id);
    write_boundary_endpoint_key(writer, prefix + ".source", continuation.source);
    write_boundary_endpoint_key(writer, prefix + ".target", continuation.target);
    writer.Int(prefix + ".kind", static_cast<int>(continuation.kind));
  }

  const auto corridors = sorted_by_id(graph.corridors);
  writer.UInt("corridor.count", corridors.size());
  for (std::size_t i = 0; i < corridors.size(); ++i) {
    const RoadCorridor& corridor = *corridors[i];
    const std::string prefix = "corridor." + std::to_string(i);
    writer.UInt(prefix + ".id", corridor.id);
    writer.UInt(prefix + ".section_template_id", corridor.section_template_id);
    writer.UInt(prefix + ".segment.count", corridor.segments.size());
    for (std::size_t j = 0; j < corridor.segments.size(); ++j) {
      const DirectedSegmentRef& ref = corridor.segments[j];
      const std::string ref_prefix =
          prefix + ".segment." + std::to_string(j);
      writer.UInt(ref_prefix + ".segment_id", ref.segment_id);
      writer.Int(ref_prefix + ".reversed", ref.reversed ? 1 : 0);
    }
  }

  const auto manual_lines = sorted_by_id(graph.manual_lines);
  writer.UInt("manual_line.count", manual_lines.size());
  for (std::size_t i = 0; i < manual_lines.size(); ++i) {
    const ManualLineMarking& marking = *manual_lines[i];
    const std::string prefix = "manual_line." + std::to_string(i);
    writer.UInt(prefix + ".id", marking.id);
    writer.UInt(prefix + ".owner_segment_id", marking.owner_segment_id);
    writer.UInt(prefix + ".style_id", marking.style_id.value);
    writer.UInt(prefix + ".path.span.count", marking.path.spans.size());
    for (std::size_t j = 0; j < marking.path.spans.size(); ++j) {
      write_span(writer, prefix + ".path.span." + std::to_string(j),
                 marking.path.spans[j]);
    }
  }

  const auto manual_areas = sorted_by_id(graph.manual_areas);
  writer.UInt("manual_area.count", manual_areas.size());
  for (std::size_t i = 0; i < manual_areas.size(); ++i) {
    const ManualAreaMarking& marking = *manual_areas[i];
    const std::string prefix = "manual_area." + std::to_string(i);
    writer.UInt(prefix + ".id", marking.id);
    writer.UInt(prefix + ".owner_segment_id", marking.owner_segment_id);
    write_vec2(writer, prefix + ".frame_origin", marking.frame_origin);
    writer.Double(prefix + ".rotation_rad", marking.rotation_rad);
    writer.Double(prefix + ".width_m", marking.width_m);
    writer.Double(prefix + ".length_m", marking.length_m);
    writer.UInt(prefix + ".style_id", marking.style_id.value);
  }
  return Result<std::string>::Ok(writer.Finish());
}

Result<LoadedRoad> LoadRoad(const std::string& text) {
  if (!HasCurrentHeader(text)) {
    if (HasRoadHeader(text)) {
      return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                      "legacy road graph version is unsupported");
    }
    return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                    "unknown road graph version");
  }
  ArchiveReader reader{text};
  Result<bool> status = reader.Status();
  if (!status.ok) return Result<LoadedRoad>::Fail(status.failure_category, status.error);
  Result<std::uint64_t> version = reader.RequireU64("road_graph_version");
  if (!version.ok) return Result<LoadedRoad>::Fail(version.failure_category, version.error);
  if (version.value != kVersion) {
    return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                    "unknown road graph version");
  }
  Result<std::uint64_t> next_id = reader.RequireU64("next_id");
  if (!next_id.ok) return Result<LoadedRoad>::Fail(next_id.failure_category, next_id.error);

  LoadedRoad loaded{};
  loaded.next_id = next_id.value;
  auto require_count = [&reader](const std::string& key) -> Result<std::size_t> {
    Result<std::uint64_t> count = reader.RequireU64(key);
    if (!count.ok) return Result<std::size_t>::Fail(count.failure_category, count.error);
    return Result<std::size_t>::Ok(static_cast<std::size_t>(count.value));
  };

  Result<std::size_t> section_count = require_count("section_template.count");
  if (!section_count.ok) return Result<LoadedRoad>::Fail(section_count.failure_category, section_count.error);
  for (std::size_t i = 0; i < section_count.value; ++i) {
    const std::string prefix = "section_template." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    CrossSectionTemplate section{};
    section.id = id.value;
    Result<std::size_t> strip_count = require_count(prefix + ".strip.count");
    if (!strip_count.ok) return Result<LoadedRoad>::Fail(strip_count.failure_category, strip_count.error);
    for (std::size_t j = 0; j < strip_count.value; ++j) {
      const std::string strip_prefix = prefix + ".strip." + std::to_string(j);
      Result<std::uint64_t> strip_id = reader.RequireU64(strip_prefix + ".id");
      Result<StripFunction> function =
          enum_value<StripFunction>(reader, strip_prefix + ".function", 0, 3);
      Result<double> width = reader.RequireDouble(strip_prefix + ".width_m");
      Result<double> slope = reader.RequireDouble(strip_prefix + ".cross_slope");
      Result<std::uint64_t> style = reader.RequireU64(strip_prefix + ".style_id");
      if (!strip_id.ok) return Result<LoadedRoad>::Fail(strip_id.failure_category, strip_id.error);
      if (!function.ok) return Result<LoadedRoad>::Fail(function.failure_category, function.error);
      if (!width.ok) return Result<LoadedRoad>::Fail(width.failure_category, width.error);
      if (!slope.ok) return Result<LoadedRoad>::Fail(slope.failure_category, slope.error);
      if (!style.ok) return Result<LoadedRoad>::Fail(style.failure_category, style.error);
      LaneSideMarkingPolicy side_marking{};
      for (const std::string_view side_key : {std::string_view{"left"}, std::string_view{"right"}}) {
        const std::string side_prefix =
            strip_prefix + ".side_marking." + std::string(side_key);
        Result<int> enabled = reader.RequireInt(side_prefix + ".enabled");
        Result<MarkingRole> side_role =
            enum_value<MarkingRole>(reader, side_prefix + ".role", 0, 5);
        Result<std::uint64_t> side_style = reader.RequireU64(side_prefix + ".style_id");
        if (!enabled.ok) return Result<LoadedRoad>::Fail(enabled.failure_category, enabled.error);
        if (!side_role.ok) return Result<LoadedRoad>::Fail(side_role.failure_category, side_role.error);
        if (!side_style.ok) return Result<LoadedRoad>::Fail(side_style.failure_category, side_style.error);
        if (enabled.value != 0 && enabled.value != 1) {
          return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                          "lane side marking enabled flag is invalid");
        }
        AutoMarkingPolicy policy{enabled.value == 1, side_role.value,
                                 MarkingStyleId{side_style.value}};
        if (policy.enabled && !IsKnownMarkingStyle(policy.style_id)) {
          return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                          "lane side marking style is unknown");
        }
        if (side_key == "left") {
          side_marking.left = policy;
        } else {
          side_marking.right = policy;
        }
      }
      section.strips.push_back(SectionStrip{strip_id.value, function.value, width.value, slope.value,
                                          SurfaceStyleId{style.value}, side_marking});
    }
    Result<std::size_t> lane_count = require_count(prefix + ".lane_band.count");
    if (!lane_count.ok) return Result<LoadedRoad>::Fail(lane_count.failure_category, lane_count.error);
    for (std::size_t j = 0; j < lane_count.value; ++j) {
      const std::string lane_prefix =
          prefix + ".lane_band." + std::to_string(j);
      Result<std::uint64_t> lane_id = reader.RequireU64(lane_prefix + ".id");
      Result<std::uint64_t> strip_id =
          reader.RequireU64(lane_prefix + ".surface_strip_id");
      Result<double> start =
          reader.RequireDouble(lane_prefix + ".lateral_start_m");
      Result<double> end =
          reader.RequireDouble(lane_prefix + ".lateral_end_m");
      Result<LaneTravelDirection> direction = enum_value<LaneTravelDirection>(
          reader, lane_prefix + ".direction", 0, 1);
      if (!lane_id.ok) return Result<LoadedRoad>::Fail(lane_id.failure_category, lane_id.error);
      if (!strip_id.ok) return Result<LoadedRoad>::Fail(strip_id.failure_category, strip_id.error);
      if (!start.ok) return Result<LoadedRoad>::Fail(start.failure_category, start.error);
      if (!end.ok) return Result<LoadedRoad>::Fail(end.failure_category, end.error);
      if (!direction.ok) return Result<LoadedRoad>::Fail(direction.failure_category, direction.error);
      section.lane_bands.push_back(
          LaneBand{lane_id.value, strip_id.value, start.value, end.value,
                   direction.value});
    }
    Result<std::size_t> boundary_count = require_count(prefix + ".boundary.count");
    if (!boundary_count.ok) return Result<LoadedRoad>::Fail(boundary_count.failure_category, boundary_count.error);
    for (std::size_t j = 0; j < boundary_count.value; ++j) {
      const std::string boundary_prefix = prefix + ".boundary." + std::to_string(j);
      Result<std::uint64_t> boundary_id = reader.RequireU64(boundary_prefix + ".boundary_id");
      Result<BoundaryRole> role = enum_value<BoundaryRole>(reader, boundary_prefix + ".role", 0, 3);
      Result<double> width = reader.RequireDouble(boundary_prefix + ".width_m");
      Result<double> height = reader.RequireDouble(boundary_prefix + ".height_m");
      Result<int> marking_enabled = reader.RequireInt(boundary_prefix + ".marking.enabled");
      Result<MarkingRole> marking_role =
          enum_value<MarkingRole>(reader, boundary_prefix + ".marking.role", 0, 5);
      Result<std::uint64_t> marking_style =
          reader.RequireU64(boundary_prefix + ".marking.style_id");
      if (!boundary_id.ok) return Result<LoadedRoad>::Fail(boundary_id.failure_category, boundary_id.error);
      if (!role.ok) return Result<LoadedRoad>::Fail(role.failure_category, role.error);
      if (!width.ok) return Result<LoadedRoad>::Fail(width.failure_category, width.error);
      if (!height.ok) return Result<LoadedRoad>::Fail(height.failure_category, height.error);
      if (!marking_enabled.ok) {
        return Result<LoadedRoad>::Fail(marking_enabled.failure_category, marking_enabled.error);
      }
      if (!marking_role.ok) return Result<LoadedRoad>::Fail(marking_role.failure_category, marking_role.error);
      if (!marking_style.ok) return Result<LoadedRoad>::Fail(marking_style.failure_category, marking_style.error);
      if (marking_enabled.value != 0 && marking_enabled.value != 1) {
        return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                        "boundary marking enabled flag is invalid");
      }
      section.boundaries.push_back(BoundaryProfile{boundary_id.value, role.value, width.value,
                                                   height.value,
                                                   AutoMarkingPolicy{
                                                       marking_enabled.value == 1,
                                                       marking_role.value,
                                                       MarkingStyleId{marking_style.value}}});
    }
    loaded.graph.section_templates.push_back(std::move(section));
  }

  Result<std::size_t> transition_count = require_count("transition.count");
  if (!transition_count.ok) return Result<LoadedRoad>::Fail(transition_count.failure_category, transition_count.error);
  for (std::size_t i = 0; i < transition_count.value; ++i) {
    const std::string prefix = "transition." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> from = reader.RequireU64(prefix + ".from_template");
    Result<std::uint64_t> to = reader.RequireU64(prefix + ".to_template");
    Result<DistanceRefKind> start_kind = enum_value<DistanceRefKind>(reader, prefix + ".start.kind", 0, 2);
    Result<double> start_value = reader.RequireDouble(prefix + ".start.value");
    Result<DistanceRefKind> end_kind = enum_value<DistanceRefKind>(reader, prefix + ".end.kind", 0, 2);
    Result<double> end_value = reader.RequireDouble(prefix + ".end.value");
    Result<TransitionAnchor> anchor = enum_value<TransitionAnchor>(reader, prefix + ".anchor", 0, 3);
    Result<std::uint64_t> anchor_boundary_id =
        reader.RequireU64(prefix + ".anchor_boundary_id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!from.ok) return Result<LoadedRoad>::Fail(from.failure_category, from.error);
    if (!to.ok) return Result<LoadedRoad>::Fail(to.failure_category, to.error);
    if (!start_kind.ok) return Result<LoadedRoad>::Fail(start_kind.failure_category, start_kind.error);
    if (!start_value.ok) return Result<LoadedRoad>::Fail(start_value.failure_category, start_value.error);
    if (!end_kind.ok) return Result<LoadedRoad>::Fail(end_kind.failure_category, end_kind.error);
    if (!end_value.ok) return Result<LoadedRoad>::Fail(end_value.failure_category, end_value.error);
    if (!anchor.ok) return Result<LoadedRoad>::Fail(anchor.failure_category, anchor.error);
    if (!anchor_boundary_id.ok) {
      return Result<LoadedRoad>::Fail(anchor_boundary_id.failure_category,
                                      anchor_boundary_id.error);
    }
    SectionTransition transition{id.value, from.value, to.value,
                                 DistanceRef{start_kind.value, start_value.value},
                                 DistanceRef{end_kind.value, end_value.value},
                                 anchor.value, anchor_boundary_id.value, {}};
    Result<std::size_t> rule_count = require_count(prefix + ".rule.count");
    if (!rule_count.ok) return Result<LoadedRoad>::Fail(rule_count.failure_category, rule_count.error);
    for (std::size_t j = 0; j < rule_count.value; ++j) {
      const std::string rule_prefix = prefix + ".rule." + std::to_string(j);
      Result<std::uint64_t> strip_id = reader.RequireU64(rule_prefix + ".strip_id");
      Result<TransitionAction> action = enum_value<TransitionAction>(reader, rule_prefix + ".action", 0, 5);
      if (!strip_id.ok) return Result<LoadedRoad>::Fail(strip_id.failure_category, strip_id.error);
      if (!action.ok) return Result<LoadedRoad>::Fail(action.failure_category, action.error);
      transition.rules.push_back(SectionTransitionRule{strip_id.value, action.value});
    }
    loaded.graph.transitions.push_back(std::move(transition));
  }

  Result<std::size_t> node_count = require_count("node.count");
  if (!node_count.ok) return Result<LoadedRoad>::Fail(node_count.failure_category, node_count.error);
  for (std::size_t i = 0; i < node_count.value; ++i) {
    const std::string prefix = "node." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<Vec2d> position = vec2(reader, prefix + ".position");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!position.ok) return Result<LoadedRoad>::Fail(position.failure_category, position.error);
    loaded.graph.nodes.push_back(RoadNode{id.value, position.value});
  }

  Result<std::size_t> policy_count = require_count("connection_policy_override.count");
  if (!policy_count.ok) return Result<LoadedRoad>::Fail(policy_count.failure_category, policy_count.error);
  for (std::size_t i = 0; i < policy_count.value; ++i) {
    const std::string prefix = "connection_policy_override." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> node_id = reader.RequireU64(prefix + ".node_id");
    Result<NodeConnectionPolicy> policy = enum_value<NodeConnectionPolicy>(reader, prefix + ".policy", 0, 2);
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!node_id.ok) return Result<LoadedRoad>::Fail(node_id.failure_category, node_id.error);
    if (!policy.ok) return Result<LoadedRoad>::Fail(policy.failure_category, policy.error);
    loaded.graph.connection_policy_overrides.push_back(
        NodeConnectionPolicyOverride{id.value, node_id.value, policy.value});
  }

  Result<std::size_t> approach_override_count = require_count("approach_geometry_override.count");
  if (!approach_override_count.ok) {
    return Result<LoadedRoad>::Fail(approach_override_count.failure_category, approach_override_count.error);
  }
  for (std::size_t i = 0; i < approach_override_count.value; ++i) {
    const std::string prefix = "approach_geometry_override." + std::to_string(i);
    Result<ApproachKey> key = read_approach_key(reader, prefix);
    Result<int> setback_mode = reader.RequireInt(prefix + ".setback.mode");
    Result<int> lateral_mode = reader.RequireInt(prefix + ".lateral_shift.mode");
    if (!key.ok) return Result<LoadedRoad>::Fail(key.failure_category, key.error);
    if (!setback_mode.ok) return Result<LoadedRoad>::Fail(setback_mode.failure_category, setback_mode.error);
    if (!lateral_mode.ok) return Result<LoadedRoad>::Fail(lateral_mode.failure_category, lateral_mode.error);
    if ((setback_mode.value != 0 && setback_mode.value != 1) ||
        (lateral_mode.value != 0 && lateral_mode.value != 1)) {
      return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                      "approach geometry override mode is invalid");
    }
    ApproachGeometryOverride override{};
    override.key = key.value;
    if (setback_mode.value == 1) {
      Result<double> value = reader.RequireDouble(prefix + ".setback.value");
      if (!value.ok) return Result<LoadedRoad>::Fail(value.failure_category, value.error);
      override.setback_m = ManualDoubleOverride{true, value.value};
    }
    if (lateral_mode.value == 1) {
      Result<double> value = reader.RequireDouble(prefix + ".lateral_shift.value");
      if (!value.ok) return Result<LoadedRoad>::Fail(value.failure_category, value.error);
      override.lateral_shift_m = ManualDoubleOverride{true, value.value};
    }
    loaded.graph.approach_geometry_overrides.push_back(override);
  }

  Result<std::size_t> auto_marking_override_count =
      require_count("auto_marking_override.count");
  if (!auto_marking_override_count.ok) {
    return Result<LoadedRoad>::Fail(auto_marking_override_count.failure_category,
                                    auto_marking_override_count.error);
  }
  for (std::size_t i = 0; i < auto_marking_override_count.value; ++i) {
    const std::string prefix = "auto_marking_override." + std::to_string(i);
    Result<AutoMarkingKey> key = read_auto_marking_key(reader, prefix + ".key");
    Result<int> suppressed = reader.RequireInt(prefix + ".suppressed");
    if (!key.ok) return Result<LoadedRoad>::Fail(key.failure_category, key.error);
    if (!suppressed.ok) return Result<LoadedRoad>::Fail(suppressed.failure_category, suppressed.error);
    if (suppressed.value != 0 && suppressed.value != 1) {
      return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                      "auto marking override suppressed flag is invalid");
    }
    loaded.graph.auto_marking_overrides.push_back(
        AutoMarkingOverride{key.value, suppressed.value == 1});
  }

  Result<std::size_t> junction_marking_override_count =
      require_count("junction_marking_override.count");
  if (!junction_marking_override_count.ok) {
    return Result<LoadedRoad>::Fail(junction_marking_override_count.failure_category,
                                    junction_marking_override_count.error);
  }
  for (std::size_t i = 0; i < junction_marking_override_count.value; ++i) {
    const std::string prefix = "junction_marking_override." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> node_id = reader.RequireU64(prefix + ".node_id");
    Result<ApproachKey> source_approach =
        read_approach_key(reader, prefix + ".source.approach");
    Result<std::uint64_t> source_boundary =
        reader.RequireU64(prefix + ".source.boundary_id");
    Result<MarkingRole> source_role =
        enum_value<MarkingRole>(reader, prefix + ".source.role", 0, 5);
    Result<JunctionMarkingAction> action =
        enum_value<JunctionMarkingAction>(reader, prefix + ".action", 0, 2);
    Result<int> target_has = reader.RequireInt(prefix + ".target.has_value");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!node_id.ok) return Result<LoadedRoad>::Fail(node_id.failure_category, node_id.error);
    if (!source_approach.ok) return Result<LoadedRoad>::Fail(source_approach.failure_category, source_approach.error);
    if (!source_boundary.ok) return Result<LoadedRoad>::Fail(source_boundary.failure_category, source_boundary.error);
    if (!source_role.ok) return Result<LoadedRoad>::Fail(source_role.failure_category, source_role.error);
    if (!action.ok) return Result<LoadedRoad>::Fail(action.failure_category, action.error);
    if (!target_has.ok) return Result<LoadedRoad>::Fail(target_has.failure_category, target_has.error);
    if (target_has.value != 0 && target_has.value != 1) {
      return Result<LoadedRoad>::Fail(CommitFailureCategory::kInvalidInput,
                                      "junction marking override target flag is invalid");
    }
    JunctionMarkingOverride override{};
    override.id = id.value;
    override.node_id = node_id.value;
    override.source = JunctionMarkingEndpoint{source_approach.value,
                                              source_boundary.value,
                                              source_role.value};
    override.action = action.value;
    if (target_has.value == 1) {
      Result<ApproachKey> target_approach =
          read_approach_key(reader, prefix + ".target.approach");
      Result<std::uint64_t> target_boundary =
          reader.RequireU64(prefix + ".target.boundary_id");
      Result<MarkingRole> target_role =
          enum_value<MarkingRole>(reader, prefix + ".target.role", 0, 5);
      if (!target_approach.ok) return Result<LoadedRoad>::Fail(target_approach.failure_category, target_approach.error);
      if (!target_boundary.ok) return Result<LoadedRoad>::Fail(target_boundary.failure_category, target_boundary.error);
      if (!target_role.ok) return Result<LoadedRoad>::Fail(target_role.failure_category, target_role.error);
      override.target = JunctionMarkingEndpoint{target_approach.value,
                                                target_boundary.value,
                                                target_role.value};
    }
    loaded.graph.junction_marking_overrides.push_back(std::move(override));
  }

  Result<std::size_t> segment_count = require_count("segment.count");
  if (!segment_count.ok) return Result<LoadedRoad>::Fail(segment_count.failure_category, segment_count.error);
  for (std::size_t i = 0; i < segment_count.value; ++i) {
    const std::string prefix = "segment." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> node_a = reader.RequireU64(prefix + ".node_a");
    Result<std::uint64_t> node_b = reader.RequireU64(prefix + ".node_b");
    Result<std::uint64_t> section_template = reader.RequireU64(prefix + ".section_template");
    Result<std::uint64_t> transition = reader.RequireU64(prefix + ".transition");
    Result<SegmentShapeIntent> intent =
        enum_value<SegmentShapeIntent>(reader, prefix + ".shape.intent", 0, 1);
    Result<Vec2d> start_handle = vec2(reader, prefix + ".shape.start_handle");
    Result<Vec2d> end_handle = vec2(reader, prefix + ".shape.end_handle");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!node_a.ok) return Result<LoadedRoad>::Fail(node_a.failure_category, node_a.error);
    if (!node_b.ok) return Result<LoadedRoad>::Fail(node_b.failure_category, node_b.error);
    if (!section_template.ok) return Result<LoadedRoad>::Fail(section_template.failure_category, section_template.error);
    if (!transition.ok) return Result<LoadedRoad>::Fail(transition.failure_category, transition.error);
    if (!intent.ok) return Result<LoadedRoad>::Fail(intent.failure_category, intent.error);
    if (!start_handle.ok) return Result<LoadedRoad>::Fail(start_handle.failure_category, start_handle.error);
    if (!end_handle.ok) return Result<LoadedRoad>::Fail(end_handle.failure_category, end_handle.error);
    RoadSegment segment{id.value, node_a.value, node_b.value,
                        SegmentShape{start_handle.value, {}, end_handle.value,
                                     intent.value},
                        section_template.value,
                        transition.value == 0 ? std::nullopt
                                              : std::optional<SectionTransitionId>(transition.value)};
    Result<std::size_t> knot_count = require_count(prefix + ".shape.knot.count");
    if (!knot_count.ok) return Result<LoadedRoad>::Fail(knot_count.failure_category, knot_count.error);
    for (std::size_t j = 0; j < knot_count.value; ++j) {
      const std::string knot_prefix = prefix + ".shape.knot." + std::to_string(j);
      Result<Vec2d> position = vec2(reader, knot_prefix + ".position");
      Result<Vec2d> handle_in = vec2(reader, knot_prefix + ".handle_in");
      Result<Vec2d> handle_out = vec2(reader, knot_prefix + ".handle_out");
      if (!position.ok) return Result<LoadedRoad>::Fail(position.failure_category, position.error);
      if (!handle_in.ok) return Result<LoadedRoad>::Fail(handle_in.failure_category, handle_in.error);
      if (!handle_out.ok) return Result<LoadedRoad>::Fail(handle_out.failure_category, handle_out.error);
      segment.shape.internal_knots.push_back(
          SegmentKnot{position.value, handle_in.value, handle_out.value});
    }
    loaded.graph.segments.push_back(std::move(segment));
  }

  Result<std::size_t> lane_connection_count = require_count("lane_connection.count");
  if (!lane_connection_count.ok) {
    return Result<LoadedRoad>::Fail(lane_connection_count.failure_category,
                                    lane_connection_count.error);
  }
  for (std::size_t i = 0; i < lane_connection_count.value; ++i) {
    const std::string prefix = "lane_connection." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<LaneEndpointKey> source = read_lane_endpoint_key(reader, prefix + ".source");
    Result<LaneEndpointKey> target = read_lane_endpoint_key(reader, prefix + ".target");
    Result<LaneConnectionKind> kind =
        enum_value<LaneConnectionKind>(reader, prefix + ".kind", 0, 4);
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!source.ok) return Result<LoadedRoad>::Fail(source.failure_category, source.error);
    if (!target.ok) return Result<LoadedRoad>::Fail(target.failure_category, target.error);
    if (!kind.ok) return Result<LoadedRoad>::Fail(kind.failure_category, kind.error);
    loaded.graph.lane_connections.push_back(
        LaneConnection{id.value, source.value, target.value, kind.value});
  }

  Result<std::size_t> boundary_continuation_count =
      require_count("boundary_continuation.count");
  if (!boundary_continuation_count.ok) {
    return Result<LoadedRoad>::Fail(boundary_continuation_count.failure_category,
                                    boundary_continuation_count.error);
  }
  for (std::size_t i = 0; i < boundary_continuation_count.value; ++i) {
    const std::string prefix = "boundary_continuation." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<BoundaryEndpointKey> source =
        read_boundary_endpoint_key(reader, prefix + ".source");
    Result<BoundaryEndpointKey> target =
        read_boundary_endpoint_key(reader, prefix + ".target");
    Result<int> kind = reader.RequireInt(prefix + ".kind");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!source.ok) return Result<LoadedRoad>::Fail(source.failure_category, source.error);
    if (!target.ok) return Result<LoadedRoad>::Fail(target.failure_category, target.error);
    if (!kind.ok) return Result<LoadedRoad>::Fail(kind.failure_category, kind.error);
    loaded.graph.boundary_continuations.push_back(
        BoundaryContinuation{id.value, source.value, target.value,
                             static_cast<BoundaryContinuationKind>(kind.value)});
  }

  Result<std::size_t> corridor_count = require_count("corridor.count");
  if (!corridor_count.ok) {
    return Result<LoadedRoad>::Fail(corridor_count.failure_category,
                                    corridor_count.error);
  }
  for (std::size_t i = 0; i < corridor_count.value; ++i) {
    const std::string prefix = "corridor." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> section_template =
        reader.RequireU64(prefix + ".section_template_id");
    Result<std::size_t> ref_count =
        require_count(prefix + ".segment.count");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!section_template.ok) {
      return Result<LoadedRoad>::Fail(section_template.failure_category,
                                      section_template.error);
    }
    if (!ref_count.ok) {
      return Result<LoadedRoad>::Fail(ref_count.failure_category, ref_count.error);
    }
    RoadCorridor corridor{id.value, section_template.value, {}};
    for (std::size_t j = 0; j < ref_count.value; ++j) {
      const std::string ref_prefix =
          prefix + ".segment." + std::to_string(j);
      Result<std::uint64_t> segment =
          reader.RequireU64(ref_prefix + ".segment_id");
      Result<int> reversed = reader.RequireInt(ref_prefix + ".reversed");
      if (!segment.ok) {
        return Result<LoadedRoad>::Fail(segment.failure_category, segment.error);
      }
      if (!reversed.ok || (reversed.value != 0 && reversed.value != 1)) {
        return Result<LoadedRoad>::Fail(
            CommitFailureCategory::kInvalidInput,
            "road corridor reversed flag is invalid");
      }
      corridor.segments.push_back(
          DirectedSegmentRef{segment.value, reversed.value == 1});
    }
    loaded.graph.corridors.push_back(std::move(corridor));
  }

  Result<std::size_t> manual_line_count = require_count("manual_line.count");
  if (!manual_line_count.ok) return Result<LoadedRoad>::Fail(manual_line_count.failure_category, manual_line_count.error);
  for (std::size_t i = 0; i < manual_line_count.value; ++i) {
    const std::string prefix = "manual_line." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> owner = reader.RequireU64(prefix + ".owner_segment_id");
    Result<std::uint64_t> style = reader.RequireU64(prefix + ".style_id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!owner.ok) return Result<LoadedRoad>::Fail(owner.failure_category, owner.error);
    if (!style.ok) return Result<LoadedRoad>::Fail(style.failure_category, style.error);
    ManualLineMarking marking{id.value, owner.value, {}, MarkingStyleId{style.value}};
    Result<std::size_t> span_count = require_count(prefix + ".path.span.count");
    if (!span_count.ok) return Result<LoadedRoad>::Fail(span_count.failure_category, span_count.error);
    for (std::size_t j = 0; j < span_count.value; ++j) {
      Result<BezierSpan> span = read_span(reader, prefix + ".path.span." + std::to_string(j));
      if (!span.ok) return Result<LoadedRoad>::Fail(span.failure_category, span.error);
      marking.path.spans.push_back(span.value);
    }
    loaded.graph.manual_lines.push_back(std::move(marking));
  }

  Result<std::size_t> manual_area_count = require_count("manual_area.count");
  if (!manual_area_count.ok) return Result<LoadedRoad>::Fail(manual_area_count.failure_category, manual_area_count.error);
  for (std::size_t i = 0; i < manual_area_count.value; ++i) {
    const std::string prefix = "manual_area." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> owner = reader.RequireU64(prefix + ".owner_segment_id");
    Result<Vec2d> frame_origin = vec2(reader, prefix + ".frame_origin");
    Result<double> rotation = reader.RequireDouble(prefix + ".rotation_rad");
    Result<double> width = reader.RequireDouble(prefix + ".width_m");
    Result<double> length = reader.RequireDouble(prefix + ".length_m");
    Result<std::uint64_t> style = reader.RequireU64(prefix + ".style_id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.failure_category, id.error);
    if (!owner.ok) return Result<LoadedRoad>::Fail(owner.failure_category, owner.error);
    if (!frame_origin.ok) return Result<LoadedRoad>::Fail(frame_origin.failure_category, frame_origin.error);
    if (!rotation.ok) return Result<LoadedRoad>::Fail(rotation.failure_category, rotation.error);
    if (!width.ok) return Result<LoadedRoad>::Fail(width.failure_category, width.error);
    if (!length.ok) return Result<LoadedRoad>::Fail(length.failure_category, length.error);
    if (!style.ok) return Result<LoadedRoad>::Fail(style.failure_category, style.error);
    loaded.graph.manual_areas.push_back(ManualAreaMarking{
        id.value, owner.value, frame_origin.value, rotation.value, width.value, length.value,
        MarkingStyleId{style.value}});
  }

  Result<bool> finish = reader.Finish();
  if (!finish.ok) return Result<LoadedRoad>::Fail(finish.failure_category, finish.error);
  Result<bool> valid = ValidateAuthoritativeGraph(loaded.graph, loaded.next_id);
  if (!valid.ok) return Result<LoadedRoad>::Fail(valid.failure_category, valid.error);
  return Result<LoadedRoad>::Ok(std::move(loaded));
}

} // namespace city::road::persistence
