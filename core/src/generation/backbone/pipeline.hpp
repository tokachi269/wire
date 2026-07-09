#pragma once

#include "wire/core/core_state.hpp"

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace wire::core::generation::backbone {

inline constexpr std::size_t bad = std::numeric_limits<std::size_t>::max();

struct node {
  std::size_t id = bad;
  Vec3d pos{};
  SupportKind support = SupportKind::kPole;
  ObjectId pole = kInvalidObjectId;
  ObjectId saved = kInvalidObjectId;
  bool is_new = true;
  bool on_route = false;
  bool pinned = false;
  bool explicit_support = false;
  bool has_tangent = false;
  Vec3d tangent{};
  bool has_source_edge = false;
  ObjectId source_edge_node_a = kInvalidObjectId;
  ObjectId source_edge_node_b = kInvalidObjectId;
  double source_edge_t = 0.0;
  std::vector<SupportNodeBundleMode> bundle_modes{};
};

struct link {
  std::size_t id = bad;
  std::size_t a = bad;
  std::size_t b = bad;
  std::size_t route = 0;
  std::size_t order = bad;
  Vec3d dir{};
  ObjectId saved = kInvalidObjectId;
  bool is_new = true;
  std::size_t arow = bad;
  std::size_t brow = bad;
};

struct graph {
  std::vector<node> nodes{};
  std::vector<link> links{};
};

struct src {
  bool is_open = false;
  std::size_t id = bad;
};

struct pair {
  std::size_t id = bad;
  std::size_t node = bad;
  std::size_t left = bad;
  std::size_t right = bad;
  Vec3d axis{};
};

struct open {
  std::size_t id = bad;
  std::size_t node = bad;
  std::size_t link = bad;
  Vec3d axis{};
};

struct jumper {
  std::size_t node = bad;
  std::size_t row_a = bad;
  std::size_t row_b = bad;
  double interior_angle_deg = 0.0;
  Vec3d node_forward{};
};

struct row {
  std::size_t id = bad;
  std::size_t node = bad;
  src source{};
  Vec3d axis{};
};

struct pairs {
  std::vector<link> links{};
  std::vector<pair> joins{};
  std::vector<open> opens{};
  std::vector<jumper> jumpers{};
  std::vector<row> rows{};
};

enum class intent_reason : std::uint8_t {
  none = 0,
  conflicting_rows = 1,
};

struct row_intent {
  std::size_t row = bad;
  std::size_t bundle = bad;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  bool lower_required = false;
  intent_reason reason = intent_reason::none;
  double endpoint_offset_m = 0.0;
};

struct intent {
  std::vector<row_intent> rows{};
};

struct group_member {
  std::size_t row = bad;
  std::size_t bundle = bad;
};

struct group {
  std::size_t id = bad;
  std::size_t node = bad;
  std::vector<group_member> row_members{};
  Vec3d group_axis{};
  int vertical_order = 0;
  double endpoint_offset_m = 0.0;
};

struct groups {
  std::vector<group> items{};
};

struct trow {
  std::size_t row = bad;
  std::size_t node = bad;
  src source{};
  Vec3d axis{};
  ObjectId pole = kInvalidObjectId;
  std::vector<std::vector<ObjectId>> ports{};
  std::vector<int> placement_band_ids{};
};

struct tspan {
  ObjectId id = kInvalidObjectId;
  std::size_t link = bad;
  std::size_t bundle = bad;
  std::size_t lane = bad;
  std::size_t arow = bad;
  std::size_t brow = bad;
  bool is_new = true;
};

struct topo {
  std::vector<ObjectId> bundles{};
  std::vector<std::size_t> bundle_specs{};
  std::vector<ObjectId> poles{};
  std::vector<ObjectId> new_poles{};
  std::vector<tspan> spans{};
  std::vector<trow> rows{};
};

struct rules {
  SpanLayoutRules data{};
};

struct layout {
  std::vector<SpanLayoutEntry> entries{};
};

struct curve {
  std::vector<std::pair<ObjectId, DetailCurve>> data{};
};

struct bounds {
  std::vector<std::pair<ObjectId, BoundsCacheEntry>> data{};
};

struct geom {
  curve curves{};
  bounds boxes{};
  VisualCurvePartCache visual_curves{};
};

struct draw {
  std::vector<std::pair<ObjectId, SpanVisualCacheEntry>> visuals{};
  std::vector<std::pair<ObjectId, SpanRenderCacheEntry>> renders{};
};

class pipeline {
public:
  pipeline(CoreState& state, const BackboneSpec& spec) : state_(state), spec_(spec) {}

  enum class run_mode {
    generation,
    saved_scope,
  };

  struct run_input {
    graph made{};
    std::vector<std::size_t> active_bundle_indices{};
    std::vector<std::size_t> local_by_input{};
    std::vector<BundleTemplate> template_overrides{};
    run_mode mode = run_mode::generation;
    bool ready = false;
    bool run_preflight = true;
    bool include_new_poles = true;
    GenerationTiming* timing = nullptr;
  };

  [[nodiscard]] EditResult<bool> prepare();
  [[nodiscard]] EditResult<bool> check() const;
  [[nodiscard]] run_input make_run_input_from_spec() const;
  [[nodiscard]] run_input make_run_input_from_saved_scope(
      graph made, std::vector<std::size_t> active_bundle_indices,
      std::vector<BundleTemplate> template_overrides = {}) const;
  [[nodiscard]] EditResult<GenerateBundleFromPathResult> run(run_input input);

private:
  struct route {
    bool active = false;
    ChangeSet change_set{};
    pairs ps{};
    groups placement{};
    topo made{};
  };

  [[nodiscard]] EditResult<route> emit_route(bool run_preflight, GenerationTiming*);
  [[nodiscard]] EditResult<bool> save_derived(const route&, GenerationTiming*);
  [[nodiscard]] EditResult<pairs> make(const graph& made) const;
  [[nodiscard]] EditResult<intent> make(const pairs& ps) const;
  [[nodiscard]] EditResult<groups> make(const pairs& ps, const intent& intents) const;
  [[nodiscard]] EditResult<bool> check(const pairs& ps) const;
  [[nodiscard]] EditResult<topo> emit(const pairs& ps);
  [[nodiscard]] EditResult<bool> emit_poles(topo* made, const pairs& ps, ChangeSet* changes);
  [[nodiscard]] EditResult<bool> emit_bundles(topo* made, ChangeSet* changes);
  [[nodiscard]] EditResult<bool> emit_ports(topo* made, const pairs& ps, ChangeSet* changes);
  [[nodiscard]] EditResult<bool> emit_spans(topo* made, const pairs& ps, ChangeSet* changes);
  [[nodiscard]] rules make(const topo& made, const pairs& ps, const groups& placement) const;
  [[nodiscard]] EditResult<layout> make(const rules& made) const;
  [[nodiscard]] EditResult<geom> make(const layout& made) const;
  [[nodiscard]] draw make(const layout& placed, const geom& shaped) const;
  void save(const rules& made);
  void save(const layout& made);
  void save(geom made);
  void save(draw made);
  [[nodiscard]] EditResult<bool> save_graph(const topo& made, const pairs& ps);
  [[nodiscard]] std::size_t local(std::size_t input_point) const;
  [[nodiscard]] const BundleTemplate* template_override(BundleTemplateId id) const;

  CoreState& state_;
  const BackboneSpec& spec_;
  bool ready_ = false;
  run_mode mode_ = run_mode::generation;
  graph g_{};
  std::vector<std::size_t> active_bundle_indices_{};
  std::vector<std::size_t> local_by_input_{};
  std::vector<BundleTemplate> template_overrides_{};
};

} // namespace wire::core::generation::backbone
