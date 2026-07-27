#include "road_archive.hpp"

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
    if (!error_.empty()) return Result<bool>::Fail(ErrorKind::kValidation, error_);
    return Result<bool>::Ok(true);
  }

  Result<std::string> RequireString(const std::string& key) {
    auto found = fields_.find(key);
    if (found == fields_.end()) {
      return Result<std::string>::Fail(ErrorKind::kValidation,
                                       "missing road archive key: " + key);
    }
    consumed_.insert(key);
    return Result<std::string>::Ok(found->second);
  }

  Result<std::uint64_t> RequireU64(const std::string& key) {
    Result<std::string> text = RequireString(key);
    if (!text.ok) return Result<std::uint64_t>::Fail(text.error_kind, text.error);
    std::optional<std::uint64_t> parsed = parse_u64(text.value);
    if (!parsed.has_value()) {
      return Result<std::uint64_t>::Fail(ErrorKind::kValidation,
                                         "invalid integer road archive key: " + key);
    }
    return Result<std::uint64_t>::Ok(*parsed);
  }

  Result<int> RequireInt(const std::string& key) {
    Result<std::string> text = RequireString(key);
    if (!text.ok) return Result<int>::Fail(text.error_kind, text.error);
    std::optional<int> parsed = parse_int(text.value);
    if (!parsed.has_value()) {
      return Result<int>::Fail(ErrorKind::kValidation,
                               "invalid enum road archive key: " + key);
    }
    return Result<int>::Ok(*parsed);
  }

  Result<double> RequireDouble(const std::string& key) {
    Result<std::string> text = RequireString(key);
    if (!text.ok) return Result<double>::Fail(text.error_kind, text.error);
    std::optional<double> parsed = parse_double(text.value);
    if (!parsed.has_value()) {
      return Result<double>::Fail(ErrorKind::kValidation,
                                  "invalid double road archive key: " + key);
    }
    return Result<double>::Ok(*parsed);
  }

  Result<bool> Finish() const {
    for (const auto& [key, value] : fields_) {
      (void)value;
      if (!consumed_.contains(key)) {
        return Result<bool>::Fail(ErrorKind::kValidation,
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
  if (!parsed.ok) return Result<Enum>::Fail(parsed.error_kind, parsed.error);
  if (parsed.value < min_value || parsed.value > max_value) {
    return Result<Enum>::Fail(ErrorKind::kValidation,
                              "enum road archive key is out of range: " + key);
  }
  return Result<Enum>::Ok(static_cast<Enum>(parsed.value));
}

Result<Vec2d> vec2(ArchiveReader& reader, const std::string& prefix) {
  Result<double> x = reader.RequireDouble(prefix + ".x");
  Result<double> y = reader.RequireDouble(prefix + ".y");
  if (!x.ok) return Result<Vec2d>::Fail(x.error_kind, x.error);
  if (!y.ok) return Result<Vec2d>::Fail(y.error_kind, y.error);
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
  if (!p0.ok) return Result<BezierSpan>::Fail(p0.error_kind, p0.error);
  if (!p1.ok) return Result<BezierSpan>::Fail(p1.error_kind, p1.error);
  if (!p2.ok) return Result<BezierSpan>::Fail(p2.error_kind, p2.error);
  if (!p3.ok) return Result<BezierSpan>::Fail(p3.error_kind, p3.error);
  return Result<BezierSpan>::Ok(BezierSpan{p0.value, p1.value, p2.value, p3.value});
}

bool contains_id(const std::set<std::uint64_t>& ids, std::uint64_t id) {
  return id != 0 && ids.contains(id);
}

} // namespace

Result<bool> ValidateAuthoritativeGraph(const SavedRoadGraph& graph,
                                        std::uint64_t next_id) {
  if (next_id == 0) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road next_id is invalid");
  }
  std::set<std::uint64_t> all_ids{};
  std::set<std::uint64_t> node_ids{};
  std::set<std::uint64_t> segment_ids{};
  std::set<std::uint64_t> template_ids{};
  std::set<std::uint64_t> transition_ids{};
  std::set<std::uint64_t> policy_ids{};
  std::set<std::uint64_t> marking_ids{};
  std::uint64_t max_id = 0;
  const auto add_id = [&](std::uint64_t id, std::set<std::uint64_t>* domain,
                          std::string_view label) -> Result<bool> {
    if (id == 0 || !all_ids.insert(id).second || !domain->insert(id).second) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                std::string("road duplicate or zero ID: ") +
                                    std::string(label));
    }
    max_id = std::max(max_id, id);
    return Result<bool>::Ok(true);
  };
  for (const CrossSectionTemplate& section : graph.section_templates) {
    Result<bool> id = add_id(section.id, &template_ids, "section_template");
    if (!id.ok) return id;
    if (section.bands.empty() ||
        section.boundaries.size() + 1 != section.bands.size()) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "section template chain is incomplete");
    }
    std::set<std::uint64_t> elements{};
    for (const SurfaceBand& band : section.bands) {
      if (band.element_id == 0 || !elements.insert(band.element_id).second ||
          !finite(band.width_m) || !finite(band.cross_slope) ||
          band.width_m <= 0.0 || !IsKnownSurfaceStyle(band.style_id)) {
        return Result<bool>::Fail(ErrorKind::kValidation,
                                  "section template surface band is invalid");
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
          static_cast<int>(boundary.marking_rule) < 0 ||
          static_cast<int>(boundary.marking_rule) > 2) {
        return Result<bool>::Fail(ErrorKind::kValidation,
                                  "section template boundary is invalid");
      }
    }
  }
  for (const RoadNode& node : graph.nodes) {
    Result<bool> id = add_id(node.id, &node_ids, "node");
    if (!id.ok) return id;
    if (!finite(node.position)) {
      return Result<bool>::Fail(ErrorKind::kValidation,
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
        static_cast<int>(transition.anchor) > 2) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "section transition is invalid");
    }
    std::set<std::uint64_t> rule_elements{};
    for (const SectionTransitionRule& rule : transition.rules) {
      if (rule.element_id == 0 ||
          !rule_elements.insert(rule.element_id).second ||
          static_cast<int>(rule.action) < 0 ||
          static_cast<int>(rule.action) > 5) {
        return Result<bool>::Fail(ErrorKind::kValidation,
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
        !finite(segment.shape)) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "road segment is invalid");
    }
  }
  for (const NodeConnectionPolicyOverride& policy :
       graph.connection_policy_overrides) {
    Result<bool> id = add_id(policy.id, &policy_ids, "connection_policy");
    if (!id.ok) return id;
    if (!contains_id(node_ids, policy.node_id) ||
        static_cast<int>(policy.policy) < 0 ||
        static_cast<int>(policy.policy) > 2) {
      return Result<bool>::Fail(ErrorKind::kValidation,
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
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "approach geometry override is invalid");
    }
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    Result<bool> id = add_id(marking.id, &marking_ids, "manual_line");
    if (!id.ok) return id;
    if (!contains_id(segment_ids, marking.owner_segment_id) ||
        !IsKnownMarkingStyle(marking.style_id) || marking.path.spans.empty() ||
        !std::all_of(marking.path.spans.begin(), marking.path.spans.end(),
                     [](const BezierSpan& span) { return finite(span); })) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "manual line marking is invalid");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    Result<bool> id = add_id(marking.id, &marking_ids, "manual_area");
    if (!id.ok) return id;
    if (!contains_id(segment_ids, marking.owner_segment_id) ||
        !IsKnownMarkingStyle(marking.style_id) ||
        !finite(marking.frame_origin) || !finite(marking.width_m) ||
        !finite(marking.length_m) || marking.width_m <= 0.0 ||
        marking.length_m <= 0.0) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "manual area marking is invalid");
    }
  }
  if (next_id <= max_id) {
    return Result<bool>::Fail(ErrorKind::kValidation,
                              "road next_id does not exceed existing IDs");
  }
  return Result<bool>::Ok(true);
}

Result<std::string> SaveRoad(const SavedRoadGraph& graph,
                             std::uint64_t next_id) {
  Result<bool> valid = ValidateAuthoritativeGraph(graph, next_id);
  if (!valid.ok) return Result<std::string>::Fail(valid.error_kind, valid.error);
  ArchiveWriter writer{};
  writer.UInt("road_graph_version", kVersion);
  writer.UInt("next_id", next_id);

  const auto sections = sorted_by_id(graph.section_templates);
  writer.UInt("section_template.count", sections.size());
  for (std::size_t i = 0; i < sections.size(); ++i) {
    const CrossSectionTemplate& section = *sections[i];
    const std::string prefix = "section_template." + std::to_string(i);
    writer.UInt(prefix + ".id", section.id);
    writer.UInt(prefix + ".band.count", section.bands.size());
    for (std::size_t j = 0; j < section.bands.size(); ++j) {
      const SurfaceBand& band = section.bands[j];
      const std::string band_prefix =
          prefix + ".band." + std::to_string(j);
      writer.UInt(band_prefix + ".element_id", band.element_id);
      writer.Int(band_prefix + ".role", static_cast<int>(band.role));
      writer.Double(band_prefix + ".width_m", band.width_m);
      writer.Double(band_prefix + ".cross_slope", band.cross_slope);
      writer.UInt(band_prefix + ".style_id", band.style_id.value);
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
      writer.Int(boundary_prefix + ".marking_rule",
                 static_cast<int>(boundary.marking_rule));
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
    std::vector<SectionTransitionRule> rules = transition.rules;
    std::sort(rules.begin(), rules.end(), [](const auto& a, const auto& b) {
      return a.element_id < b.element_id;
    });
    writer.UInt(prefix + ".rule.count", rules.size());
    for (std::size_t j = 0; j < rules.size(); ++j) {
      const std::string rule_prefix = prefix + ".rule." + std::to_string(j);
      writer.UInt(rule_prefix + ".element_id", rules[j].element_id);
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
    writer.UInt(prefix + ".node_id", override.key.node_id);
    writer.UInt(prefix + ".segment_id", override.key.segment_id);
    writer.Int(prefix + ".endpoint_role", static_cast<int>(override.key.endpoint_role));
    writer.Int(prefix + ".setback.mode", override.setback_m.has_value ? 1 : 0);
    if (override.setback_m.has_value) writer.Double(prefix + ".setback.value", override.setback_m.value);
    writer.Int(prefix + ".lateral_shift.mode", override.lateral_shift_m.has_value ? 1 : 0);
    if (override.lateral_shift_m.has_value) {
      writer.Double(prefix + ".lateral_shift.value", override.lateral_shift_m.value);
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
    writer.Double(prefix + ".width_m", marking.width_m);
    writer.Double(prefix + ".length_m", marking.length_m);
    writer.UInt(prefix + ".style_id", marking.style_id.value);
  }
  return Result<std::string>::Ok(writer.Finish());
}

Result<LoadedRoad> LoadRoad(const std::string& text) {
  if (!HasCurrentHeader(text)) {
    if (HasRoadHeader(text)) {
      return Result<LoadedRoad>::Fail(ErrorKind::kValidation,
                                      "legacy road graph version is unsupported");
    }
    return Result<LoadedRoad>::Fail(ErrorKind::kValidation,
                                    "unknown road graph version");
  }
  ArchiveReader reader{text};
  Result<bool> status = reader.Status();
  if (!status.ok) return Result<LoadedRoad>::Fail(status.error_kind, status.error);
  Result<std::uint64_t> version = reader.RequireU64("road_graph_version");
  if (!version.ok) return Result<LoadedRoad>::Fail(version.error_kind, version.error);
  if (version.value != kVersion) {
    return Result<LoadedRoad>::Fail(ErrorKind::kValidation,
                                    "unknown road graph version");
  }
  Result<std::uint64_t> next_id = reader.RequireU64("next_id");
  if (!next_id.ok) return Result<LoadedRoad>::Fail(next_id.error_kind, next_id.error);

  LoadedRoad loaded{};
  loaded.next_id = next_id.value;
  auto require_count = [&reader](const std::string& key) -> Result<std::size_t> {
    Result<std::uint64_t> count = reader.RequireU64(key);
    if (!count.ok) return Result<std::size_t>::Fail(count.error_kind, count.error);
    return Result<std::size_t>::Ok(static_cast<std::size_t>(count.value));
  };

  Result<std::size_t> section_count = require_count("section_template.count");
  if (!section_count.ok) return Result<LoadedRoad>::Fail(section_count.error_kind, section_count.error);
  for (std::size_t i = 0; i < section_count.value; ++i) {
    const std::string prefix = "section_template." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    CrossSectionTemplate section{};
    section.id = id.value;
    Result<std::size_t> band_count = require_count(prefix + ".band.count");
    if (!band_count.ok) return Result<LoadedRoad>::Fail(band_count.error_kind, band_count.error);
    for (std::size_t j = 0; j < band_count.value; ++j) {
      const std::string band_prefix = prefix + ".band." + std::to_string(j);
      Result<std::uint64_t> element_id = reader.RequireU64(band_prefix + ".element_id");
      Result<SurfaceRole> role = enum_value<SurfaceRole>(reader, band_prefix + ".role", 0, 2);
      Result<double> width = reader.RequireDouble(band_prefix + ".width_m");
      Result<double> slope = reader.RequireDouble(band_prefix + ".cross_slope");
      Result<std::uint64_t> style = reader.RequireU64(band_prefix + ".style_id");
      if (!element_id.ok) return Result<LoadedRoad>::Fail(element_id.error_kind, element_id.error);
      if (!role.ok) return Result<LoadedRoad>::Fail(role.error_kind, role.error);
      if (!width.ok) return Result<LoadedRoad>::Fail(width.error_kind, width.error);
      if (!slope.ok) return Result<LoadedRoad>::Fail(slope.error_kind, slope.error);
      if (!style.ok) return Result<LoadedRoad>::Fail(style.error_kind, style.error);
      section.bands.push_back(SurfaceBand{element_id.value, role.value, width.value, slope.value,
                                          SurfaceStyleId{style.value}});
    }
    Result<std::size_t> boundary_count = require_count(prefix + ".boundary.count");
    if (!boundary_count.ok) return Result<LoadedRoad>::Fail(boundary_count.error_kind, boundary_count.error);
    for (std::size_t j = 0; j < boundary_count.value; ++j) {
      const std::string boundary_prefix = prefix + ".boundary." + std::to_string(j);
      Result<std::uint64_t> boundary_id = reader.RequireU64(boundary_prefix + ".boundary_id");
      Result<BoundaryRole> role = enum_value<BoundaryRole>(reader, boundary_prefix + ".role", 0, 3);
      Result<double> width = reader.RequireDouble(boundary_prefix + ".width_m");
      Result<double> height = reader.RequireDouble(boundary_prefix + ".height_m");
      Result<MarkingRule> marking = enum_value<MarkingRule>(reader, boundary_prefix + ".marking_rule", 0, 2);
      if (!boundary_id.ok) return Result<LoadedRoad>::Fail(boundary_id.error_kind, boundary_id.error);
      if (!role.ok) return Result<LoadedRoad>::Fail(role.error_kind, role.error);
      if (!width.ok) return Result<LoadedRoad>::Fail(width.error_kind, width.error);
      if (!height.ok) return Result<LoadedRoad>::Fail(height.error_kind, height.error);
      if (!marking.ok) return Result<LoadedRoad>::Fail(marking.error_kind, marking.error);
      section.boundaries.push_back(BoundaryProfile{boundary_id.value, role.value, width.value,
                                                   height.value, marking.value});
    }
    loaded.graph.section_templates.push_back(std::move(section));
  }

  Result<std::size_t> transition_count = require_count("transition.count");
  if (!transition_count.ok) return Result<LoadedRoad>::Fail(transition_count.error_kind, transition_count.error);
  for (std::size_t i = 0; i < transition_count.value; ++i) {
    const std::string prefix = "transition." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> from = reader.RequireU64(prefix + ".from_template");
    Result<std::uint64_t> to = reader.RequireU64(prefix + ".to_template");
    Result<StationRefKind> start_kind = enum_value<StationRefKind>(reader, prefix + ".start.kind", 0, 2);
    Result<double> start_value = reader.RequireDouble(prefix + ".start.value");
    Result<StationRefKind> end_kind = enum_value<StationRefKind>(reader, prefix + ".end.kind", 0, 2);
    Result<double> end_value = reader.RequireDouble(prefix + ".end.value");
    Result<TransitionAnchor> anchor = enum_value<TransitionAnchor>(reader, prefix + ".anchor", 0, 2);
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    if (!from.ok) return Result<LoadedRoad>::Fail(from.error_kind, from.error);
    if (!to.ok) return Result<LoadedRoad>::Fail(to.error_kind, to.error);
    if (!start_kind.ok) return Result<LoadedRoad>::Fail(start_kind.error_kind, start_kind.error);
    if (!start_value.ok) return Result<LoadedRoad>::Fail(start_value.error_kind, start_value.error);
    if (!end_kind.ok) return Result<LoadedRoad>::Fail(end_kind.error_kind, end_kind.error);
    if (!end_value.ok) return Result<LoadedRoad>::Fail(end_value.error_kind, end_value.error);
    if (!anchor.ok) return Result<LoadedRoad>::Fail(anchor.error_kind, anchor.error);
    SectionTransition transition{id.value, from.value, to.value,
                                 StationRef{start_kind.value, start_value.value},
                                 StationRef{end_kind.value, end_value.value},
                                 anchor.value, {}};
    Result<std::size_t> rule_count = require_count(prefix + ".rule.count");
    if (!rule_count.ok) return Result<LoadedRoad>::Fail(rule_count.error_kind, rule_count.error);
    for (std::size_t j = 0; j < rule_count.value; ++j) {
      const std::string rule_prefix = prefix + ".rule." + std::to_string(j);
      Result<std::uint64_t> element_id = reader.RequireU64(rule_prefix + ".element_id");
      Result<TransitionAction> action = enum_value<TransitionAction>(reader, rule_prefix + ".action", 0, 5);
      if (!element_id.ok) return Result<LoadedRoad>::Fail(element_id.error_kind, element_id.error);
      if (!action.ok) return Result<LoadedRoad>::Fail(action.error_kind, action.error);
      transition.rules.push_back(SectionTransitionRule{element_id.value, action.value});
    }
    loaded.graph.transitions.push_back(std::move(transition));
  }

  Result<std::size_t> node_count = require_count("node.count");
  if (!node_count.ok) return Result<LoadedRoad>::Fail(node_count.error_kind, node_count.error);
  for (std::size_t i = 0; i < node_count.value; ++i) {
    const std::string prefix = "node." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<Vec2d> position = vec2(reader, prefix + ".position");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    if (!position.ok) return Result<LoadedRoad>::Fail(position.error_kind, position.error);
    loaded.graph.nodes.push_back(RoadNode{id.value, position.value});
  }

  Result<std::size_t> policy_count = require_count("connection_policy_override.count");
  if (!policy_count.ok) return Result<LoadedRoad>::Fail(policy_count.error_kind, policy_count.error);
  for (std::size_t i = 0; i < policy_count.value; ++i) {
    const std::string prefix = "connection_policy_override." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> node_id = reader.RequireU64(prefix + ".node_id");
    Result<NodeConnectionPolicy> policy = enum_value<NodeConnectionPolicy>(reader, prefix + ".policy", 0, 2);
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    if (!node_id.ok) return Result<LoadedRoad>::Fail(node_id.error_kind, node_id.error);
    if (!policy.ok) return Result<LoadedRoad>::Fail(policy.error_kind, policy.error);
    loaded.graph.connection_policy_overrides.push_back(
        NodeConnectionPolicyOverride{id.value, node_id.value, policy.value});
  }

  Result<std::size_t> approach_override_count = require_count("approach_geometry_override.count");
  if (!approach_override_count.ok) {
    return Result<LoadedRoad>::Fail(approach_override_count.error_kind, approach_override_count.error);
  }
  for (std::size_t i = 0; i < approach_override_count.value; ++i) {
    const std::string prefix = "approach_geometry_override." + std::to_string(i);
    Result<std::uint64_t> node_id = reader.RequireU64(prefix + ".node_id");
    Result<std::uint64_t> segment_id = reader.RequireU64(prefix + ".segment_id");
    Result<EndpointRole> endpoint_role = enum_value<EndpointRole>(reader, prefix + ".endpoint_role", 0, 1);
    Result<int> setback_mode = reader.RequireInt(prefix + ".setback.mode");
    Result<int> lateral_mode = reader.RequireInt(prefix + ".lateral_shift.mode");
    if (!node_id.ok) return Result<LoadedRoad>::Fail(node_id.error_kind, node_id.error);
    if (!segment_id.ok) return Result<LoadedRoad>::Fail(segment_id.error_kind, segment_id.error);
    if (!endpoint_role.ok) return Result<LoadedRoad>::Fail(endpoint_role.error_kind, endpoint_role.error);
    if (!setback_mode.ok) return Result<LoadedRoad>::Fail(setback_mode.error_kind, setback_mode.error);
    if (!lateral_mode.ok) return Result<LoadedRoad>::Fail(lateral_mode.error_kind, lateral_mode.error);
    if ((setback_mode.value != 0 && setback_mode.value != 1) ||
        (lateral_mode.value != 0 && lateral_mode.value != 1)) {
      return Result<LoadedRoad>::Fail(ErrorKind::kValidation,
                                      "approach geometry override mode is invalid");
    }
    ApproachGeometryOverride override{};
    override.key = ApproachKey{node_id.value, segment_id.value, endpoint_role.value};
    if (setback_mode.value == 1) {
      Result<double> value = reader.RequireDouble(prefix + ".setback.value");
      if (!value.ok) return Result<LoadedRoad>::Fail(value.error_kind, value.error);
      override.setback_m = ManualDoubleOverride{true, value.value};
    }
    if (lateral_mode.value == 1) {
      Result<double> value = reader.RequireDouble(prefix + ".lateral_shift.value");
      if (!value.ok) return Result<LoadedRoad>::Fail(value.error_kind, value.error);
      override.lateral_shift_m = ManualDoubleOverride{true, value.value};
    }
    loaded.graph.approach_geometry_overrides.push_back(override);
  }

  Result<std::size_t> segment_count = require_count("segment.count");
  if (!segment_count.ok) return Result<LoadedRoad>::Fail(segment_count.error_kind, segment_count.error);
  for (std::size_t i = 0; i < segment_count.value; ++i) {
    const std::string prefix = "segment." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> node_a = reader.RequireU64(prefix + ".node_a");
    Result<std::uint64_t> node_b = reader.RequireU64(prefix + ".node_b");
    Result<std::uint64_t> section_template = reader.RequireU64(prefix + ".section_template");
    Result<std::uint64_t> transition = reader.RequireU64(prefix + ".transition");
    Result<Vec2d> start_handle = vec2(reader, prefix + ".shape.start_handle");
    Result<Vec2d> end_handle = vec2(reader, prefix + ".shape.end_handle");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    if (!node_a.ok) return Result<LoadedRoad>::Fail(node_a.error_kind, node_a.error);
    if (!node_b.ok) return Result<LoadedRoad>::Fail(node_b.error_kind, node_b.error);
    if (!section_template.ok) return Result<LoadedRoad>::Fail(section_template.error_kind, section_template.error);
    if (!transition.ok) return Result<LoadedRoad>::Fail(transition.error_kind, transition.error);
    if (!start_handle.ok) return Result<LoadedRoad>::Fail(start_handle.error_kind, start_handle.error);
    if (!end_handle.ok) return Result<LoadedRoad>::Fail(end_handle.error_kind, end_handle.error);
    RoadSegment segment{id.value, node_a.value, node_b.value,
                        SegmentShape{start_handle.value, {}, end_handle.value},
                        section_template.value,
                        transition.value == 0 ? std::nullopt
                                              : std::optional<SectionTransitionId>(transition.value)};
    Result<std::size_t> knot_count = require_count(prefix + ".shape.knot.count");
    if (!knot_count.ok) return Result<LoadedRoad>::Fail(knot_count.error_kind, knot_count.error);
    for (std::size_t j = 0; j < knot_count.value; ++j) {
      const std::string knot_prefix = prefix + ".shape.knot." + std::to_string(j);
      Result<Vec2d> position = vec2(reader, knot_prefix + ".position");
      Result<Vec2d> handle_in = vec2(reader, knot_prefix + ".handle_in");
      Result<Vec2d> handle_out = vec2(reader, knot_prefix + ".handle_out");
      if (!position.ok) return Result<LoadedRoad>::Fail(position.error_kind, position.error);
      if (!handle_in.ok) return Result<LoadedRoad>::Fail(handle_in.error_kind, handle_in.error);
      if (!handle_out.ok) return Result<LoadedRoad>::Fail(handle_out.error_kind, handle_out.error);
      segment.shape.internal_knots.push_back(
          SegmentKnot{position.value, handle_in.value, handle_out.value});
    }
    loaded.graph.segments.push_back(std::move(segment));
  }

  Result<std::size_t> manual_line_count = require_count("manual_line.count");
  if (!manual_line_count.ok) return Result<LoadedRoad>::Fail(manual_line_count.error_kind, manual_line_count.error);
  for (std::size_t i = 0; i < manual_line_count.value; ++i) {
    const std::string prefix = "manual_line." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> owner = reader.RequireU64(prefix + ".owner_segment_id");
    Result<std::uint64_t> style = reader.RequireU64(prefix + ".style_id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    if (!owner.ok) return Result<LoadedRoad>::Fail(owner.error_kind, owner.error);
    if (!style.ok) return Result<LoadedRoad>::Fail(style.error_kind, style.error);
    ManualLineMarking marking{id.value, owner.value, {}, MarkingStyleId{style.value}};
    Result<std::size_t> span_count = require_count(prefix + ".path.span.count");
    if (!span_count.ok) return Result<LoadedRoad>::Fail(span_count.error_kind, span_count.error);
    for (std::size_t j = 0; j < span_count.value; ++j) {
      Result<BezierSpan> span = read_span(reader, prefix + ".path.span." + std::to_string(j));
      if (!span.ok) return Result<LoadedRoad>::Fail(span.error_kind, span.error);
      marking.path.spans.push_back(span.value);
    }
    loaded.graph.manual_lines.push_back(std::move(marking));
  }

  Result<std::size_t> manual_area_count = require_count("manual_area.count");
  if (!manual_area_count.ok) return Result<LoadedRoad>::Fail(manual_area_count.error_kind, manual_area_count.error);
  for (std::size_t i = 0; i < manual_area_count.value; ++i) {
    const std::string prefix = "manual_area." + std::to_string(i);
    Result<std::uint64_t> id = reader.RequireU64(prefix + ".id");
    Result<std::uint64_t> owner = reader.RequireU64(prefix + ".owner_segment_id");
    Result<Vec2d> frame_origin = vec2(reader, prefix + ".frame_origin");
    Result<double> width = reader.RequireDouble(prefix + ".width_m");
    Result<double> length = reader.RequireDouble(prefix + ".length_m");
    Result<std::uint64_t> style = reader.RequireU64(prefix + ".style_id");
    if (!id.ok) return Result<LoadedRoad>::Fail(id.error_kind, id.error);
    if (!owner.ok) return Result<LoadedRoad>::Fail(owner.error_kind, owner.error);
    if (!frame_origin.ok) return Result<LoadedRoad>::Fail(frame_origin.error_kind, frame_origin.error);
    if (!width.ok) return Result<LoadedRoad>::Fail(width.error_kind, width.error);
    if (!length.ok) return Result<LoadedRoad>::Fail(length.error_kind, length.error);
    if (!style.ok) return Result<LoadedRoad>::Fail(style.error_kind, style.error);
    loaded.graph.manual_areas.push_back(ManualAreaMarking{
        id.value, owner.value, frame_origin.value, width.value, length.value,
        MarkingStyleId{style.value}});
  }

  Result<bool> finish = reader.Finish();
  if (!finish.ok) return Result<LoadedRoad>::Fail(finish.error_kind, finish.error);
  Result<bool> valid = ValidateAuthoritativeGraph(loaded.graph, loaded.next_id);
  if (!valid.ok) return Result<LoadedRoad>::Fail(valid.error_kind, valid.error);
  return Result<LoadedRoad>::Ok(std::move(loaded));
}

} // namespace city::road::persistence
