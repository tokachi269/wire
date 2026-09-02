#include "city/wire/core_state.hpp"
#include "city/wire/coord_utils.hpp"
#include "city/wire/support/numeric_tolerances.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace city::wire {

namespace {

constexpr PoleTypeId kDistributionPoleType = 1;
constexpr PoleTypeId kCommunicationPoleType = 2;
constexpr CableTemplateId kHighVoltageCableTemplate = 1;
constexpr CableTemplateId kLowVoltageCableTemplate = 2;
constexpr CableTemplateId kCommunicationCableTemplate = 3;
constexpr CableTemplateId kOpticalCableTemplate = 4;
constexpr CableTemplateId kDropCableTemplate = 5;
constexpr CableTemplateId kSupportWireCableTemplate = kDefaultSupportWireCableTemplateId;
constexpr AttachmentTemplateId kGenericAttachmentTemplate = 1;
constexpr AttachmentTemplateId kHiddenAttachmentTemplate = 2;
constexpr AttachmentTemplateId kInternalPathAttachmentTemplate = 3;
constexpr AttachmentTemplateId kStraightAuxiliaryAttachmentTemplate = 4;
constexpr AttachmentTemplateId kCoiledAuxiliaryAttachmentTemplate = 5;

} // namespace
void CoreState::register_default_pole_types() {
  auto make_band = [](int band_id, ConnectionCategory category, double lateral_center_m, double height_center_m,
                      int layer, SlotSide side, SlotRole role, int priority, double min_spacing_m, bool allow_multiple,
                      double lateral_half_range, double height_half_range, BandOverflowPolicy overflow_policy) {
    PortPlacementBand band{};
    band.band_id = band_id;
    band.category = category;
    band.layer = layer;
    band.side = side;
    band.role = role;
    band.lateral_center_m = lateral_center_m;
    band.lateral_min_m = lateral_center_m - lateral_half_range;
    band.lateral_max_m = lateral_center_m + lateral_half_range;
    band.height_center_m = height_center_m;
    band.height_min_m = height_center_m - height_half_range;
    band.height_max_m = height_center_m + height_half_range;
    band.priority = priority;
    band.min_spacing_m = min_spacing_m;
    band.allow_multiple = allow_multiple;
    band.overflow_policy = overflow_policy;
    band.enabled = true;
    return band;
  };

  PoleTypeDefinition dist{};
  dist.id = kDistributionPoleType;
  dist.name = "DistributionPole";
  dist.description = "Default distribution pole";
  dist.default_height_m = 10.0;
  dist.port_bands = {
      make_band(100, ConnectionCategory::kHighVoltage, -0.75, 9.2, 2, SlotSide::kLeft, SlotRole::kTrunkPreferred, 30,
                0.40, false, 0.08, 0.06, BandOverflowPolicy::kTrySiblingBand),
      make_band(101, ConnectionCategory::kHighVoltage, 0.0, 9.2, 2, SlotSide::kCenter, SlotRole::kTrunkPreferred, 29,
                0.40, false, 0.08, 0.06, BandOverflowPolicy::kTrySiblingBand),
      make_band(102, ConnectionCategory::kHighVoltage, 0.75, 9.2, 2, SlotSide::kRight, SlotRole::kTrunkPreferred, 28,
                0.40, false, 0.08, 0.06, BandOverflowPolicy::kTrySiblingBand),
      make_band(200, ConnectionCategory::kLowVoltage, -0.45, 7.4, 1, SlotSide::kLeft, SlotRole::kTrunkPreferred, 20,
                0.28, true, 0.07, 0.06, BandOverflowPolicy::kConstrainedFallback),
      make_band(201, ConnectionCategory::kLowVoltage, 0.0, 7.4, 1, SlotSide::kRight, SlotRole::kTrunkPreferred, 19, 0.28,
                true, 0.07, 0.06, BandOverflowPolicy::kConstrainedFallback),
      make_band(202, ConnectionCategory::kLowVoltage, 0.45, 7.4, 1, SlotSide::kCenter, SlotRole::kBranchPreferred, 18,
                0.25, true, 0.07, 0.06, BandOverflowPolicy::kConstrainedFallback),
      make_band(300, ConnectionCategory::kCommunication, -0.55, 5.3, 1, SlotSide::kLeft, SlotRole::kTrunkPreferred, 15,
                0.25, true, 0.0, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(301, ConnectionCategory::kOptical, -0.35, 5.3, 1, SlotSide::kRight, SlotRole::kTrunkPreferred, 14, 0.25,
                true, 0.20, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(400, ConnectionCategory::kDrop, 0.0, 5.5, 0, SlotSide::kCenter, SlotRole::kDropPreferred, 10, 0.18, true,
                0.15, 1.0, BandOverflowPolicy::kConstrainedFallback),
  };
  for (PortPlacementBand& band : dist.port_bands) {
    if (band.category == ConnectionCategory::kCommunication || band.category == ConnectionCategory::kOptical) {
      band.lateral_min_m = -0.55;
      band.lateral_max_m = 0.55;
    }
  }
  dist.anchor_slots = {
      {500, AnchorSupportKind::kGround, {0.0, 0.0, 0.5}, 10, true},
  };
  authoritative_.pole_types[dist.id] = dist;

  PoleTypeDefinition comm{};
  comm.id = kCommunicationPoleType;
  comm.name = "CommunicationPole";
  comm.description = "Communication-first pole";
  comm.default_height_m = 10.0;
  comm.port_bands = {
      make_band(600, ConnectionCategory::kCommunication, -0.55, 5.3, 2, SlotSide::kCenter, SlotRole::kTrunkPreferred, 34,
                0.26, false, 0.0, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(610, ConnectionCategory::kHighVoltage, -0.75, 9.20, 2, SlotSide::kLeft, SlotRole::kTrunkPreferred, 33,
                0.40, false, 0.08, 0.06, BandOverflowPolicy::kTrySiblingBand),
      make_band(611, ConnectionCategory::kHighVoltage, 0.0, 9.20, 2, SlotSide::kCenter, SlotRole::kTrunkPreferred, 32,
                0.40, false, 0.08, 0.06, BandOverflowPolicy::kTrySiblingBand),
      make_band(612, ConnectionCategory::kHighVoltage, 0.75, 9.20, 2, SlotSide::kRight, SlotRole::kTrunkPreferred, 31,
                0.40, false, 0.08, 0.06, BandOverflowPolicy::kTrySiblingBand),
      make_band(800, ConnectionCategory::kLowVoltage, -0.45, 7.4, 1, SlotSide::kLeft, SlotRole::kTrunkPreferred, 30,
                0.24, true, 0.07, 0.06, BandOverflowPolicy::kConstrainedFallback),
      make_band(802, ConnectionCategory::kLowVoltage, 0.0, 7.4, 1, SlotSide::kCenter, SlotRole::kBranchPreferred, 29,
                0.24, true, 0.07, 0.06, BandOverflowPolicy::kConstrainedFallback),
      make_band(803, ConnectionCategory::kLowVoltage, 0.45, 7.4, 1, SlotSide::kRight, SlotRole::kTrunkPreferred, 28,
                0.24, true, 0.07, 0.06, BandOverflowPolicy::kConstrainedFallback),
      make_band(700, ConnectionCategory::kOptical, -0.35, 5.3, 1, SlotSide::kCenter, SlotRole::kTrunkPreferred, 27,
                0.24, true, 0.20, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(701, ConnectionCategory::kOptical, 0.35, 5.3, 1, SlotSide::kCenter, SlotRole::kTrunkPreferred, 26,
                0.24, true, 0.20, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(620, ConnectionCategory::kCommunication, -0.18, 5.3, 1, SlotSide::kCenter, SlotRole::kTrunkPreferred, 25,
                0.24, true, 0.0, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(621, ConnectionCategory::kCommunication, 0.18, 5.3, 1, SlotSide::kCenter, SlotRole::kTrunkPreferred, 24,
                0.24, true, 0.0, 0.5, BandOverflowPolicy::kConstrainedFallback),
      make_band(801, ConnectionCategory::kDrop, 0.0, 5.5, 0, SlotSide::kCenter, SlotRole::kDropPreferred, 9, 0.18, true,
                0.15, 1.0, BandOverflowPolicy::kConstrainedFallback),
  };
  for (PortPlacementBand& band : comm.port_bands) {
    if (band.category == ConnectionCategory::kCommunication || band.category == ConnectionCategory::kOptical) {
      band.lateral_min_m = -0.55;
      band.lateral_max_m = 0.55;
    }
  }
  comm.anchor_slots = {
      {900, AnchorSupportKind::kGround, {0.0, 0.0, 0.5}, 10, true},
  };
  authoritative_.pole_types[comm.id] = comm;
}

const PoleTypeDefinition* CoreState::find_pole_type(PoleTypeId pole_type_id) const {
  auto it = authoritative_.pole_types.find(pole_type_id);
  if (it == authoritative_.pole_types.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::register_default_bundle_templates() {
  auto grouped_support_fanout_spacing_for = [&](CableTemplateId cable_template_id, double fallback_spacing_m) {
    const CableTemplate* cable_template = find_cable_template(cable_template_id);
    if (cable_template == nullptr || cable_template->default_grouped_support_fanout_spacing_m <= kLengthToleranceM) {
      return fallback_spacing_m;
    }
    return cable_template->default_grouped_support_fanout_spacing_m;
  };
  BundleTemplate hv{};
  hv.id = kDefaultHighVoltageBundleTemplateId;
  hv.kind = BundleKind::kHighVoltage;
  hv.name = "HV_3PH";
  hv.category = ConnectionCategory::kHighVoltage;
  hv.cable_template_id = kHighVoltageCableTemplate;
  hv.default_layer = SpanLayer::kHighVoltage;
  hv.related_pole_type_id = kDistributionPoleType;
  hv.preserve_conductor_identity = false;
  hv.count_rule = BundleCountRuleKind::kFixed;
  hv.fixed_count = 3;
  hv.min_count = 3;
  hv.max_count = 3;
  hv.default_count = 3;
  hv.default_spacing_m = 0.45;
  hv.grouped_support_fanout_spacing_m =
      grouped_support_fanout_spacing_for(hv.cable_template_id, hv.default_spacing_m);
  hv.allow_mirror = true;
  hv.allow_midair_node = true;
  hv.allow_midair_branch = false;
  hv.enable_branch_down_offset = true;
  hv.branch_endpoint_offset_m = -0.55;
  hv.order_decision_policy = OrderDecisionPolicyKind::kPermutableHomogeneous;
  hv.row_layout_axis_mode = RowLayoutAxisMode::kSupportAxis;
  hv.span_visual_assembly.support_path_enabled = true;
  hv.span_visual_assembly.endpoint_trim_m = 0.35;
  authoritative_.bundle_templates[hv.id] = hv;

  BundleTemplate lv{};
  lv.id = kDefaultLowVoltageBundleTemplateId;
  lv.kind = BundleKind::kLowVoltage;
  lv.name = "DEFAULT_SINGLE";
  lv.category = ConnectionCategory::kLowVoltage;
  lv.cable_template_id = kLowVoltageCableTemplate;
  lv.default_layer = SpanLayer::kLowVoltage;
  lv.related_pole_type_id = kDistributionPoleType;
  lv.preserve_conductor_identity = false;
  lv.count_rule = BundleCountRuleKind::kFixed;
  lv.fixed_count = 1;
  lv.min_count = 1;
  lv.max_count = 1;
  lv.default_count = 1;
  lv.default_spacing_m = 0.20;
  lv.grouped_support_fanout_spacing_m =
      grouped_support_fanout_spacing_for(lv.cable_template_id, lv.default_spacing_m);
  lv.allow_mirror = true;
  lv.allow_midair_node = true;
  lv.allow_midair_branch = true;
  lv.enable_branch_down_offset = false;
  lv.branch_endpoint_offset_m = 0.0;
  lv.order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  lv.span_visual_assembly.support_path_enabled = true;
  lv.span_visual_assembly.endpoint_trim_m = 0.35;
  authoritative_.bundle_templates[lv.id] = lv;

  BundleTemplate drop{};
  drop.id = kDefaultDropBundleTemplateId;
  drop.kind = BundleKind::kDrop;
  drop.name = "DROP_SERVICE";
  drop.category = ConnectionCategory::kDrop;
  drop.cable_template_id = kDropCableTemplate;
  drop.default_layer = SpanLayer::kDrop;
  drop.related_pole_type_id = kDistributionPoleType;
  drop.preserve_conductor_identity = false;
  drop.count_rule = BundleCountRuleKind::kFixed;
  drop.fixed_count = 1;
  drop.min_count = 1;
  drop.max_count = 1;
  drop.default_count = 1;
  drop.default_spacing_m = 0.18;
  drop.grouped_support_fanout_spacing_m =
      grouped_support_fanout_spacing_for(drop.cable_template_id, drop.default_spacing_m);
  drop.allow_mirror = true;
  drop.allow_midair_node = true;
  drop.allow_midair_branch = true;
  drop.enable_branch_down_offset = false;
  drop.branch_endpoint_offset_m = 0.0;
  drop.order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  authoritative_.bundle_templates[drop.id] = drop;

  BundleTemplate comm{};
  comm.id = kDefaultCommunicationBundleTemplateId;
  comm.kind = BundleKind::kCommunication;
  comm.name = "COMM_BUNDLE";
  comm.category = ConnectionCategory::kCommunication;
  comm.cable_template_id = kCommunicationCableTemplate;
  comm.default_layer = SpanLayer::kCommunication;
  comm.related_pole_type_id = kCommunicationPoleType;
  comm.preserve_conductor_identity = false;
  comm.count_rule = BundleCountRuleKind::kRange;
  comm.fixed_count = 0;
  comm.min_count = 1;
  comm.max_count = 8;
  comm.default_count = 1;
  comm.default_spacing_m = 0.20;
  comm.grouped_support_fanout_spacing_m =
      grouped_support_fanout_spacing_for(comm.cable_template_id, comm.default_spacing_m);
  comm.allow_mirror = true;
  comm.allow_midair_node = true;
  comm.allow_midair_branch = true;
  comm.enable_branch_down_offset = false;
  comm.branch_endpoint_offset_m = 0.0;
  comm.order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  comm.span_visual_assembly.endpoint_trim_m = 1.2;
  comm.span_visual_assembly.visual_member_count = 3;
  comm.span_visual_assembly.visual_member_spacing_m = 0.018;
  authoritative_.bundle_templates[comm.id] = comm;

  BundleTemplate optical{};
  optical.id = kDefaultOpticalBundleTemplateId;
  optical.kind = BundleKind::kOptical;
  optical.name = "OPTICAL";
  optical.category = ConnectionCategory::kOptical;
  optical.cable_template_id = kOpticalCableTemplate;
  optical.default_layer = SpanLayer::kOptical;
  optical.related_pole_type_id = kCommunicationPoleType;
  optical.preserve_conductor_identity = false;
  optical.count_rule = BundleCountRuleKind::kFixed;
  optical.fixed_count = 1;
  optical.min_count = 1;
  optical.max_count = 1;
  optical.default_count = 1;
  optical.default_spacing_m = 0.20;
  optical.grouped_support_fanout_spacing_m =
      grouped_support_fanout_spacing_for(optical.cable_template_id, optical.default_spacing_m);
  optical.allow_mirror = true;
  optical.allow_midair_node = true;
  optical.allow_midair_branch = true;
  optical.enable_branch_down_offset = false;
  optical.branch_endpoint_offset_m = 0.0;
  optical.order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  optical.support_wire_pole_band_id = 0;
  optical.span_visual_assembly.support_path_enabled = true;
  optical.span_visual_assembly.helix_enabled = true;
  optical.span_visual_assembly.helix_clearance_m = 0.015;
  optical.span_visual_assembly.endpoint_trim_m = 1.05;
  optical.span_visual_assembly.visual_member_count = 2;
  optical.span_visual_assembly.visual_member_spacing_m = 0.014;
  authoritative_.bundle_templates[optical.id] = optical;
}

const BundleTemplate* CoreState::find_bundle_template(BundleTemplateId bundle_template_id) const {
  auto it = authoritative_.bundle_templates.find(bundle_template_id);
  if (it == authoritative_.bundle_templates.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::register_default_cable_templates() {
  CableTemplate hv{};
  hv.id = kHighVoltageCableTemplate;
  hv.name = "HV_BARE";
  hv.outer_diameter_m = 0.024;
  hv.default_grouped_support_fanout_spacing_m = 0.75;
  hv.bend_stiffness = 1.4;
  hv.min_bend_radius_m = 0.8;
  hv.material_style = CableMaterialStyleKind::kBareConductor;
  hv.color_rgba = 0xBFC7CFFFu;
  hv.sag_factor = 0.025;
  hv.continuity_policy = CableContinuityPolicyHint::kPreferG1;
  hv.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  authoritative_.cable_templates[hv.id] = hv;

  CableTemplate lv{};
  lv.id = kLowVoltageCableTemplate;
  lv.name = "LV_INSULATED";
  lv.outer_diameter_m = 0.020;
  lv.default_grouped_support_fanout_spacing_m = 0.20;
  lv.bend_stiffness = 0.9;
  lv.min_bend_radius_m = 0.25;
  lv.material_style = CableMaterialStyleKind::kInsulated;
  lv.color_rgba = 0x2E2E2EFFu;
  lv.sag_factor = 0.03;
  lv.continuity_policy = CableContinuityPolicyHint::kAuto;
  lv.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  authoritative_.cable_templates[lv.id] = lv;

  CableTemplate drop{};
  drop.id = kDropCableTemplate;
  drop.name = "DROP_SERVICE";
  drop.outer_diameter_m = 0.016;
  drop.default_grouped_support_fanout_spacing_m = 0.18;
  drop.bend_stiffness = 0.7;
  drop.min_bend_radius_m = 0.20;
  drop.material_style = CableMaterialStyleKind::kInsulated;
  drop.color_rgba = 0x1E1E1EFFu;
  drop.sag_factor = 0.045000000000000005;  // Preserve the pre-v4 effective sag ratio exactly.
  drop.continuity_policy = CableContinuityPolicyHint::kPreferG1;
  drop.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  authoritative_.cable_templates[drop.id] = drop;

  CableTemplate comm{};
  comm.id = kCommunicationCableTemplate;
  comm.name = "COMM_MULTI";
  comm.outer_diameter_m = 0.016;
  comm.default_grouped_support_fanout_spacing_m = 0.20;
  comm.bend_stiffness = 0.6;
  comm.min_bend_radius_m = 0.18;
  comm.material_style = CableMaterialStyleKind::kGeneric;
  comm.color_rgba = 0x5D5D5DFFu;
  comm.sag_factor = 0.03;
  comm.continuity_policy = CableContinuityPolicyHint::kPreferG2;
  comm.attachment_style = CableAttachmentStyleHint::kAuto;
  authoritative_.cable_templates[comm.id] = comm;

  CableTemplate optical{};
  optical.id = kOpticalCableTemplate;
  optical.name = "OPTICAL_FIBER";
  optical.outer_diameter_m = 0.012;
  optical.default_grouped_support_fanout_spacing_m = 0.20;
  optical.bend_stiffness = 0.5;
  optical.min_bend_radius_m = 0.20;
  optical.material_style = CableMaterialStyleKind::kOptical;
  optical.color_rgba = 0x6EC9D8FFu;
  optical.sag_factor = 0.034999999999999996;  // Preserve the pre-v4 effective sag ratio exactly.
  optical.continuity_policy = CableContinuityPolicyHint::kPreferG2;
  optical.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  optical.default_endpoint_attachment_template_id = kInvalidAttachmentTemplateId;
  authoritative_.cable_templates[optical.id] = optical;

  CableTemplate support{};
  support.id = kSupportWireCableTemplate;
  support.name = "SUPPORT_WIRE";
  support.outer_diameter_m = 0.010;
  support.default_grouped_support_fanout_spacing_m = 0.20;
  support.bend_stiffness = 0.6;
  support.min_bend_radius_m = 0.18;
  support.material_style = CableMaterialStyleKind::kGeneric;
  support.color_rgba = 0x1E1E1EFFu;
  support.sag_factor = 0.03;
  support.continuity_policy = CableContinuityPolicyHint::kPreferG2;
  support.attachment_style = CableAttachmentStyleHint::kDirectThrough;
  authoritative_.cable_templates[support.id] = support;
}

void CoreState::register_default_attachment_templates() {
  AttachmentTemplate generic{};
  generic.id = kGenericAttachmentTemplate;
  generic.name = "GENERIC_PASS_THROUGH";
  generic.kind = AttachmentKind::kGeneric;
  generic.line_interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  generic.sockets = {
      AttachmentSocketTemplate{0, {-0.12, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.12, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  authoritative_.attachment_templates[generic.id] = generic;

  AttachmentTemplate hidden{};
  hidden.id = kHiddenAttachmentTemplate;
  hidden.name = "INLINE_HIDE_SEGMENT";
  hidden.kind = AttachmentKind::kMarker;
  hidden.line_interaction_mode = AttachmentLineInteractionMode::kHideSegment;
  hidden.sockets = {
      AttachmentSocketTemplate{0, {-0.10, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.10, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  authoritative_.attachment_templates[hidden.id] = hidden;

  AttachmentTemplate internal{};
  internal.id = kInternalPathAttachmentTemplate;
  internal.name = "INLINE_INTERNAL_PATH";
  internal.kind = AttachmentKind::kSpacer;
  internal.line_interaction_mode = AttachmentLineInteractionMode::kReplaceWithInternalPath;
  internal.sockets = {
      AttachmentSocketTemplate{0, {-0.14, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.14, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  AttachmentInternalPathTemplate path{};
  path.start_socket_id = 0;
  path.end_socket_id = 1;
  path.local_points = {{-0.05, 0.0, 0.10}, {0.05, 0.0, 0.10}};
  internal.internal_paths.push_back(path);
  authoritative_.attachment_templates[internal.id] = internal;

  AttachmentTemplate straight_aux{};
  straight_aux.id = kStraightAuxiliaryAttachmentTemplate;
  straight_aux.name = "INLINE_AUXILIARY_CABLE";
  straight_aux.kind = AttachmentKind::kSpacer;
  straight_aux.line_interaction_mode = AttachmentLineInteractionMode::kAddInternalPath;
  straight_aux.sockets = {
      AttachmentSocketTemplate{0, {-0.16, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.16, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  AttachmentInternalPathTemplate straight_path{};
  straight_path.start_socket_id = 0;
  straight_path.end_socket_id = 1;
  straight_path.profile_kind = AttachmentInternalPathTemplate::ProfileKind::kStraightCable;
  straight_aux.internal_paths.push_back(straight_path);
  authoritative_.attachment_templates[straight_aux.id] = straight_aux;

  AttachmentTemplate coiled_aux{};
  coiled_aux.id = kCoiledAuxiliaryAttachmentTemplate;
  coiled_aux.name = "INLINE_COILED_AUXILIARY_CABLE";
  coiled_aux.kind = AttachmentKind::kSpacer;
  coiled_aux.line_interaction_mode = AttachmentLineInteractionMode::kAddInternalPath;
  coiled_aux.sockets = {
      AttachmentSocketTemplate{0, {-0.18, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
      AttachmentSocketTemplate{1, {0.18, 0.0, 0.0}, {1.0, 0.0, 0.0}},
  };
  AttachmentInternalPathTemplate coiled_path{};
  coiled_path.start_socket_id = 0;
  coiled_path.end_socket_id = 1;
  coiled_path.profile_kind = AttachmentInternalPathTemplate::ProfileKind::kCoiledCable;
  coiled_path.coil_radius_m = 0.06;
  coiled_path.coil_turn_count = 4;
  coiled_path.coil_samples_per_turn = 12;
  coiled_aux.internal_paths.push_back(coiled_path);
  authoritative_.attachment_templates[coiled_aux.id] = coiled_aux;
}

const CableTemplate* CoreState::find_cable_template(CableTemplateId cable_template_id) const {
  auto it = authoritative_.cable_templates.find(cable_template_id);
  if (it == authoritative_.cable_templates.end()) {
    return nullptr;
  }
  return &it->second;
}

const AttachmentTemplate* CoreState::find_attachment_template(AttachmentTemplateId attachment_template_id) const {
  auto it = authoritative_.attachment_templates.find(attachment_template_id);
  if (it == authoritative_.attachment_templates.end()) {
    return nullptr;
  }
  return &it->second;
}


} // namespace city::wire
