#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"

namespace {

using emscripten::val;
using wire::core::BackboneBundleSpec;
using wire::core::BackboneSpec;
using wire::core::BundleKind;
using wire::core::CoreState;
using wire::core::CoreView;
using wire::core::PoleTypeId;
using wire::core::SpanLayer;
using wire::core::Vec3d;

[[nodiscard]] val result_value(bool ok, const std::string& error) {
  val result = val::object();
  result.set("ok", ok);
  result.set("error", error);
  return result;
}

[[nodiscard]] BundleKind bundle_kind(int raw) {
  if (raw < static_cast<int>(BundleKind::kLowVoltage) ||
      raw > static_cast<int>(BundleKind::kOpticalWithSupport)) {
    throw std::invalid_argument("bundle template id is out of range");
  }
  return static_cast<BundleKind>(raw);
}

[[nodiscard]] SpanLayer span_layer(BundleKind kind) {
  switch (kind) {
  case BundleKind::kHighVoltage:
    return SpanLayer::kHighVoltage;
  case BundleKind::kCommunication:
    return SpanLayer::kCommunication;
  case BundleKind::kOptical:
  case BundleKind::kOpticalWithSupport:
    return SpanLayer::kOptical;
  case BundleKind::kDrop:
    return SpanLayer::kDrop;
  case BundleKind::kLowVoltage:
  default:
    return SpanLayer::kLowVoltage;
  }
}

class WireState {
public:
  WireState() : state_(std::make_unique<CoreState>()) {}

  val generate(const val& flat_points, int bundle_template_id, double interval_m, int pole_type_id, int count) {
    const std::size_t value_count = flat_points["length"].as<std::size_t>();
    if (value_count % 3 != 0) {
      return result_value(false, "point array length must be divisible by 3");
    }

    BackboneSpec spec{};
    spec.path.polyline.reserve(value_count / 3);
    for (std::size_t index = 0; index < value_count; index += 3) {
      spec.path.polyline.push_back(
          Vec3d{flat_points[index].as<double>(), flat_points[index + 1].as<double>(),
                flat_points[index + 2].as<double>()});
    }
    spec.interval_m = interval_m;
    spec.pole_type_id = static_cast<PoleTypeId>(pole_type_id);

    try {
      const BundleKind kind = bundle_kind(bundle_template_id);
      spec.bundles.push_back(BackboneBundleSpec{kind, span_layer(kind), count});
    } catch (const std::invalid_argument& error) {
      return result_value(false, error.what());
    }

    const auto generated = state_->GenerateFromBackboneSpec(spec);
    val result = result_value(generated.ok, generated.error);
    result.set("generatedPoleCount", generated.value.generated_pole_ids.size());
    result.set("generatedSpanCount", generated.value.generated_span_ids.size());
    result.set("totalMs", generated.value.timing.total_ms);
    return result;
  }

  [[nodiscard]] std::size_t visual_part_count() const {
    return state_->visual_curve_parts().parts.size();
  }

  [[nodiscard]] val visual_part(std::size_t index) const {
    const auto& parts = state_->visual_curve_parts().parts;
    if (index >= parts.size()) {
      throw std::out_of_range("visual part index is out of range");
    }
    const auto& part = parts[index];
    val output = val::object();
    output.set("kind", static_cast<int>(part.kind));
    output.set("wireRadius", part.wire_radius_m);
    output.set("colorRgba", part.color_rgba);
    output.set("sourceNodeId", std::to_string(part.source_node_id));
    output.set("sourceEdgeId", std::to_string(part.source_edge_id));
    output.set("sourceSpanId", std::to_string(part.source_span_id));
    output.set("sourceBundleId", std::to_string(part.source_bundle_id));
    output.set("bundleTemplateId", static_cast<int>(part.bundle_template_id));
    output.set("laneIndex", part.lane_index);
    output.set("sampleCount", part.samples.size());
    return output;
  }

  val visual_part_samples(std::size_t index) {
    const auto& parts = state_->visual_curve_parts().parts;
    if (index >= parts.size()) {
      throw std::out_of_range("visual part index is out of range");
    }
    sample_buffer_.clear();
    sample_buffer_.reserve(parts[index].samples.size() * 3);
    for (const Vec3d& point : parts[index].samples) {
      sample_buffer_.push_back(point.x);
      sample_buffer_.push_back(point.y);
      sample_buffer_.push_back(point.z);
    }
    return val(emscripten::typed_memory_view(sample_buffer_.size(), sample_buffer_.data()));
  }

  [[nodiscard]] std::size_t pole_count() const {
    return CoreView(*state_).poles().size();
  }

  [[nodiscard]] val pole(std::size_t index) const {
    const auto& poles = CoreView(*state_).poles();
    const auto* pole = poles.at_index(index);
    if (pole == nullptr) {
      throw std::out_of_range("pole index is out of range");
    }
    val output = val::object();
    output.set("id", std::to_string(pole->id));
    output.set("height", pole->height_m);
    output.set("positionX", pole->world_transform.position.x);
    output.set("positionY", pole->world_transform.position.y);
    output.set("positionZ", pole->world_transform.position.z);
    output.set("rotationX", pole->world_transform.rotation_euler_deg.x);
    output.set("rotationY", pole->world_transform.rotation_euler_deg.y);
    output.set("rotationZ", pole->world_transform.rotation_euler_deg.z);
    output.set("scaleX", pole->world_transform.scale.x);
    output.set("scaleY", pole->world_transform.scale.y);
    output.set("scaleZ", pole->world_transform.scale.z);
    return output;
  }

  [[nodiscard]] std::size_t bundle_template_count() const {
    return CoreView(*state_).bundle_templates().size();
  }

  [[nodiscard]] val bundle_template(std::size_t index) const {
    const auto& templates = CoreView(*state_).bundle_templates();
    std::vector<BundleKind> ids{};
    ids.reserve(templates.size());
    for (const auto& [id, bundle_template] : templates) {
      (void)bundle_template;
      ids.push_back(id);
    }
    std::ranges::sort(ids, {}, [](BundleKind id) { return static_cast<int>(id); });
    if (index >= ids.size()) {
      throw std::out_of_range("bundle template index is out of range");
    }
    const auto& bundle_template = templates.at(ids[index]);
    val output = val::object();
    output.set("id", static_cast<int>(bundle_template.id));
    output.set("name", bundle_template.name);
    output.set("defaultCount", bundle_template.default_count);
    output.set("fixedCount", bundle_template.count_rule == wire::core::BundleCountRuleKind::kFixed);
    return output;
  }

private:
  std::unique_ptr<CoreState> state_;
  std::vector<double> sample_buffer_{};
};

} // namespace

EMSCRIPTEN_BINDINGS(wire_web_core) {
  emscripten::class_<WireState>("WireState")
      .constructor<>()
      .function("generate", &WireState::generate)
      .function("visualPartCount", &WireState::visual_part_count)
      .function("visualPart", &WireState::visual_part)
      .function("visualPartSamples", &WireState::visual_part_samples)
      .function("poleCount", &WireState::pole_count)
      .function("pole", &WireState::pole)
      .function("bundleTemplateCount", &WireState::bundle_template_count)
      .function("bundleTemplate", &WireState::bundle_template);
}
