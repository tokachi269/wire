#include "cases.hpp"
#include "../registry.hpp"

namespace backbone_tests {

void register_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C368_backbone_smoke_line", "backbone generates the milestone-1 line slice", "Invariant", false,
                         C368_backbone_smoke_line);
  test_registry::AddTest(tests, "C369_backbone_rules_saved", "backbone saves SpanLayoutRules for each generated span", "Invariant",
                         false, C369_backbone_rules_saved);
  test_registry::AddTest(tests, "C370_backbone_no_v1_deps", "backbone source does not depend on v1 generation internals",
                         "Boundary", false, C370_backbone_no_v1_deps);
  test_registry::AddTest(tests, "C371_backbone_rejects_unsupported", "backbone rejects unsupported milestone-1 inputs",
                         "Boundary", true, C371_backbone_rejects_unsupported);
  test_registry::AddTest(tests, "C372_backbone_rules_do_not_seed", "backbone saves rules without creating authority seed",
                         "Boundary", false, C372_backbone_rules_do_not_seed);
  test_registry::AddTest(tests, "C373_backbone_layout_saved_without_recalc", "backbone saves layout immediately from rules",
                         "Boundary", false, C373_backbone_layout_saved_without_recalc);
  test_registry::AddTest(tests, "C374_backbone_layout_is_deterministic", "backbone derives stable layout from identical input",
                         "Invariant", false, C374_backbone_layout_is_deterministic);
  test_registry::AddTest(tests, "C375_backbone_curve_saved_without_recalc", "backbone saves curve immediately from layout",
                         "Boundary", false, C375_backbone_curve_saved_without_recalc);
  test_registry::AddTest(tests, "C376_backbone_curve_is_deterministic", "backbone derives stable curve from identical input",
                         "Invariant", false, C376_backbone_curve_is_deterministic);
  test_registry::AddTest(tests, "C377_backbone_bounds_saved_without_recalc", "backbone saves bounds immediately from curve",
                         "Boundary", false, C377_backbone_bounds_saved_without_recalc);
  test_registry::AddTest(tests, "C378_backbone_bounds_is_deterministic", "backbone derives stable bounds from identical input",
                         "Invariant", false, C378_backbone_bounds_is_deterministic);
  test_registry::AddTest(tests, "C379_backbone_m1_required_outputs", "backbone milestone 1 required outputs are fixed",
                         "Boundary", false, C379_backbone_m1_required_outputs);
  test_registry::AddTest(tests, "C380_backbone_m1_draw_outputs_saved", "backbone milestone 1 saves draw caches",
                         "Boundary", false, C380_backbone_m1_draw_outputs_saved);
  test_registry::AddTest(tests, "C381_backbone_m1_no_recalc_contract", "backbone milestone 1 has no recalc contract",
                         "Boundary", false, C381_backbone_m1_no_recalc_contract);
  test_registry::AddTest(tests, "C382_backbone_geom_is_single_pipeline_layer", "backbone keeps geom as one pipeline layer",
                         "Boundary", false, C382_backbone_geom_is_single_pipeline_layer);
  test_registry::AddTest(tests, "C383_backbone_draw_is_pipeline_layer", "backbone draw is a pipeline layer",
                         "Boundary", false, C383_backbone_draw_is_pipeline_layer);
  test_registry::AddTest(tests, "C384_backbone_topo_is_single_output_layer", "backbone keeps topology as one output layer",
                         "Boundary", false, C384_backbone_topo_is_single_output_layer);
  test_registry::AddTest(tests, "C385_backbone_emit_is_split_by_topology_parts", "backbone splits topology emission by part",
                         "Boundary", false, C385_backbone_emit_is_split_by_topology_parts);
  test_registry::AddTest(tests, "C386_backbone_link_pair_row_are_separate", "backbone separates links, pairs, opens, and rows",
                         "Boundary", false, C386_backbone_link_pair_row_are_separate);
  test_registry::AddTest(tests, "C387_backbone_pairs_are_single_source", "backbone creates pairs from graph in one place",
                         "Boundary", false, C387_backbone_pairs_are_single_source);
  test_registry::AddTest(tests, "C388_backbone_polyline3_pair_model", "backbone represents a three-point route as links, pair, and opens",
                         "Invariant", false, C388_backbone_polyline3_pair_model);
  test_registry::AddTest(tests, "C389_backbone_row_axis_owned_by_pairs", "backbone port rows consume pair-owned axes",
                         "Boundary", false, C389_backbone_row_axis_owned_by_pairs);
  test_registry::AddTest(tests, "C390_backbone_rejects_already_used_incident",
                         "backbone rejects invalid pair incident ownership", "Boundary", true,
                         C390_backbone_rejects_already_used_incident);
  test_registry::AddTest(tests, "C391_backbone_no_kind_label", "backbone does not add junction kind labels",
                         "Boundary", false, C391_backbone_no_kind_label);
  test_registry::AddTest(tests, "C392_backbone_polyline3_outputs", "backbone saves required outputs for three-point routes",
                         "Invariant", false, C392_backbone_polyline3_outputs);
  test_registry::AddTest(tests, "C393_backbone_polyline3_is_deterministic", "backbone derives deterministic three-point output",
                         "Invariant", false, C393_backbone_polyline3_is_deterministic);
  test_registry::AddTest(tests, "C394_backbone_existing_pole_node_is_not_recreated",
                         "backbone uses existing pole nodes without recreating them", "Boundary", false,
                         C394_backbone_existing_pole_node_is_not_recreated);
  test_registry::AddTest(tests, "C395_backbone_is_new_does_not_affect_pairs",
                         "backbone pair output does not change because a node is existing", "Invariant", false,
                         C395_backbone_is_new_does_not_affect_pairs);
  test_registry::AddTest(tests, "C396_backbone_existing_pole_does_not_read_existing_spans",
                         "backbone existing pole nodes do not use existing spans for new meaning", "Boundary", false,
                         C396_backbone_existing_pole_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C397_backbone_rejects_missing_saved_midair_node_spec",
                         "backbone rejects missing saved midair node specs", "Boundary", true,
                         C397_backbone_rejects_missing_saved_midair_node_spec);
  test_registry::AddTest(tests, "C398_backbone_rejects_missing_existing_pole", "backbone rejects missing existing pole ids",
                         "Boundary", true, C398_backbone_rejects_missing_existing_pole);
  test_registry::AddTest(tests, "C399_backbone_existing_pole_sequence_is_deterministic",
                         "backbone derives deterministic output with existing pole nodes", "Invariant", false,
                         C399_backbone_existing_pole_sequence_is_deterministic);
  test_registry::AddTest(tests, "C400_backbone_multiple_bundles_smoke", "backbone generates multiple bundles on one pairs graph",
                         "Invariant", false, C400_backbone_multiple_bundles_smoke);
  test_registry::AddTest(tests, "C401_backbone_multiple_bundles_polyline3_outputs",
                         "backbone saves required outputs for multiple bundles on a three-point route", "Invariant", false,
                         C401_backbone_multiple_bundles_polyline3_outputs);
  test_registry::AddTest(tests, "C402_backbone_bundle_spec_does_not_affect_pairs",
                         "backbone bundle specs do not alter graph pair output", "Boundary", false,
                         C402_backbone_bundle_spec_does_not_affect_pairs);
  test_registry::AddTest(tests, "C403_backbone_existing_pole_with_multiple_bundles",
                         "backbone combines existing pole nodes with multiple bundles", "Invariant", false,
                         C403_backbone_existing_pole_with_multiple_bundles);
  test_registry::AddTest(tests, "C404_backbone_rejects_empty_bundles", "backbone rejects empty bundle requests", "Boundary", true,
                         C404_backbone_rejects_empty_bundles);
  test_registry::AddTest(tests, "C405_backbone_no_bundle_pair_branching", "backbone pair building does not inspect bundle specs",
                         "Boundary", false, C405_backbone_no_bundle_pair_branching);
  test_registry::AddTest(tests, "C406_backbone_port_height_uses_pole_band", "backbone port height comes from pole bands",
                         "Invariant", false, C406_backbone_port_height_uses_pole_band);
  test_registry::AddTest(tests, "C407_backbone_multiple_bundle_heights_are_band_based",
                         "backbone multiple bundle heights come from pole bands", "Invariant", false,
                         C407_backbone_multiple_bundle_heights_are_band_based);
  test_registry::AddTest(tests, "C408_backbone_existing_pole_uses_actual_pole_type_height",
                         "backbone existing pole height uses the actual pole type", "Invariant", false,
                         C408_backbone_existing_pole_uses_actual_pole_type_height);
  test_registry::AddTest(tests, "C409_backbone_rejects_missing_port_band", "backbone rejects missing port bands", "Boundary",
                         true, C409_backbone_rejects_missing_port_band);
  test_registry::AddTest(tests, "C410_placement_height_does_not_affect_pairs",
                         "placement height selection does not affect pairs", "Boundary", false,
                         C410_placement_height_does_not_affect_pairs);
  test_registry::AddTest(tests, "C411_backbone_lateral_offset_moves_ports_along_row_axis",
                         "backbone lateral offset moves generated output along row axis", "Invariant", false,
                         C411_backbone_lateral_offset_moves_ports_along_row_axis);
  test_registry::AddTest(tests, "C412_backbone_lateral_offset_sign_is_deterministic",
                         "backbone lateral offset sign is deterministic", "Invariant", false,
                         C412_backbone_lateral_offset_sign_is_deterministic);
  test_registry::AddTest(tests, "C413_backbone_lateral_offset_does_not_affect_pairs",
                         "backbone lateral offset does not affect pairs", "Boundary", false,
                         C413_backbone_lateral_offset_does_not_affect_pairs);
  test_registry::AddTest(tests, "C414_backbone_simple_avoid_detour_supported",
                         "backbone supports a simple avoid detour", "Boundary", false,
                         C414_backbone_simple_avoid_detour_supported);
  test_registry::AddTest(tests, "C415_backbone_has_no_empty_levels_layer",
                         "backbone has no empty levels layer", "Boundary", false,
                         C415_backbone_has_no_empty_levels_layer);
  test_registry::AddTest(tests, "C416_backbone_node_mode_not_present_is_noop",
                         "backbone accepts kNotPresent node modes as no-op", "Invariant", false,
                         C416_backbone_node_mode_not_present_is_noop);
  test_registry::AddTest(tests, "C417_backbone_node_mode_pass_through_without_target_rejected",
                         "backbone rejects pass-through node modes without a target row", "Boundary", true,
                         C417_backbone_node_mode_pass_through_without_target_rejected);
  test_registry::AddTest(tests, "C418_backbone_node_mode_unknown_bundle_rejected",
                         "backbone rejects node modes for bundles absent from the request", "Boundary", true,
                         C418_backbone_node_mode_unknown_bundle_rejected);
  test_registry::AddTest(tests, "C419_backbone_node_mode_point_index_rejected",
                         "backbone rejects out-of-range node mode point indices", "Boundary", true,
                         C419_backbone_node_mode_point_index_rejected);
  test_registry::AddTest(tests, "C420_backbone_node_mode_does_not_affect_pairs",
                         "backbone node modes do not affect pairs", "Boundary", false,
                         C420_backbone_node_mode_does_not_affect_pairs);
  test_registry::AddTest(tests, "C421_backbone_topo_row_carries_source",
                         "backbone topology rows carry pair row source", "Boundary", false,
                         C421_backbone_topo_row_carries_source);
  test_registry::AddTest(tests, "C422_backbone_rules_consume_topo_and_groups",
                         "backbone rules consume topology, placement, and explicit jumper relations", "Boundary", false,
                         C422_backbone_rules_consume_topo_and_groups);
  test_registry::AddTest(tests, "C423_backbone_tspan_carries_endpoint_rows",
                         "backbone topology spans carry endpoint row indices", "Boundary", false,
                         C423_backbone_tspan_carries_endpoint_rows);
  test_registry::AddTest(tests, "C424_backbone_saves_backbone_graph_nodes_edges",
                         "backbone saves backbone graph nodes and edges", "Boundary", false,
                         C424_backbone_saves_backbone_graph_nodes_edges);
  test_registry::AddTest(tests, "C425_backbone_edge_carries_multiple_spans",
                         "backbone backbone edge carries generated spans", "Boundary", false,
                         C425_backbone_edge_carries_multiple_spans);
  test_registry::AddTest(tests, "C426_backbone_existing_pole_resolves_graph_node",
                         "backbone resolves saved graph node for existing poles", "Boundary", false,
                         C426_backbone_existing_pole_resolves_graph_node);
  test_registry::AddTest(tests, "C427_backbone_graph_index_links_outputs",
                         "backbone backbone index links graph to outputs", "Boundary", false,
                         C427_backbone_graph_index_links_outputs);
  test_registry::AddTest(tests, "C428_backbone_pole_frontier_collects_incident_graph",
                         "backbone pole frontier collects saved graph incidents", "Boundary", false,
                         C428_backbone_pole_frontier_collects_incident_graph);
  test_registry::AddTest(tests, "C429_backbone_span_frontier_collects_edge_bundle_spans",
                         "backbone span frontier collects spans from its backbone edge bundle", "Boundary", false,
                         C429_backbone_span_frontier_collects_edge_bundle_spans);
  test_registry::AddTest(tests, "C430_backbone_frontier_uses_saved_graph_index",
                         "backbone frontier reads saved graph indexes", "Boundary", false,
                         C430_backbone_frontier_uses_saved_graph_index);
  test_registry::AddTest(tests, "C431_backbone_edge_bundle_is_saved_backbone_unit",
                         "backbone stores edge bundle as the saved backbone unit", "Boundary", false,
                         C431_backbone_edge_bundle_is_saved_backbone_unit);
  test_registry::AddTest(tests, "C432_backbone_multiple_bundles_create_multiple_edge_bundles",
                         "backbone multiple bundles create multiple saved edge bundles", "Boundary", false,
                         C432_backbone_multiple_bundles_create_multiple_edge_bundles);
  test_registry::AddTest(tests, "C433_backbone_resolves_edge_for_same_poles",
                         "backbone resolves saved edge for the same pole pair", "Boundary", false,
                         C433_backbone_resolves_edge_for_same_poles);
  test_registry::AddTest(tests, "C434_backbone_reverse_duplicate_same_bundle_rejected",
                         "backbone reverse duplicate same-bundle generation is rejected", "Boundary", false,
                         C434_backbone_reverse_duplicate_same_bundle_rejected);
  test_registry::AddTest(tests, "C435_backbone_edge_metadata_is_not_overwritten_on_duplicate_reject",
                         "backbone saved edge metadata is not overwritten on duplicate reject", "Boundary", false,
                         C435_backbone_edge_metadata_is_not_overwritten_on_duplicate_reject);
  test_registry::AddTest(tests, "C436_backbone_frontier_reads_edge_bundles",
                         "backbone frontier reads edge bundles", "Boundary", false,
                         C436_backbone_frontier_reads_edge_bundles);
  test_registry::AddTest(tests, "C437_backbone_layout_save_is_direct",
                         "backbone layout save uses direct layout storage", "Boundary", false,
                         C437_backbone_layout_save_is_direct);
  test_registry::AddTest(tests, "C438_backbone_layout_save_keeps_no_authority_contract",
                         "backbone direct layout save keeps authority contract empty", "Boundary", false,
                         C438_backbone_layout_save_keeps_no_authority_contract);
  test_registry::AddTest(tests, "C439_backbone_source_still_avoids_support_layout_entrypoint",
                         "backbone source avoids support layout entrypoints", "Boundary", false,
                         C439_backbone_source_still_avoids_support_layout_entrypoint);
  test_registry::AddTest(tests, "C440_backbone_does_not_read_authoritative_backbone_directly",
                         "backbone does not read authoritative backbone directly", "Boundary", false,
                         C440_backbone_does_not_read_authoritative_backbone_directly);
  test_registry::AddTest(tests, "C441_backbone_save_backbone_edge_returns_saved_ref",
                         "backbone backbone edge save returns saved edge ref", "Boundary", false,
                         C441_backbone_save_backbone_edge_returns_saved_ref);
  test_registry::AddTest(tests, "C442_backbone_edge_forward_uses_saved_ref",
                         "backbone edge forward uses saved edge ref", "Boundary", false,
                         C442_backbone_edge_forward_uses_saved_ref);
  test_registry::AddTest(tests, "C443_backbone_edge_resolution_behavior_unchanged",
                         "backbone edge resolution behavior remains unchanged after duplicate reject", "Boundary", false,
                         C443_backbone_edge_resolution_behavior_unchanged);
  test_registry::AddTest(tests, "C444_backbone_layout_uses_neutral_types",
                         "backbone layout source uses neutral layout types", "Boundary", false,
                         C444_backbone_layout_uses_neutral_types);
  test_registry::AddTest(tests, "C445_backbone_cache_span_layout_accepts_neutral_entry",
                         "backbone layout save accepts neutral layout entries", "Boundary", false,
                         C445_backbone_cache_span_layout_accepts_neutral_entry);
  test_registry::AddTest(tests, "C446_backbone_layout_boundary_behavior_unchanged",
                         "backbone neutral layout boundary keeps generated outputs", "Boundary", false,
                         C446_backbone_layout_boundary_behavior_unchanged);
  test_registry::AddTest(tests, "C447_backbone_span_layout_view_reads_neutral_layout",
                         "backbone reads generated layout through neutral view", "Boundary", false,
                         C447_backbone_span_layout_view_reads_neutral_layout);
  test_registry::AddTest(tests, "C448_backbone_tests_use_neutral_layout_read",
                         "backbone tests use neutral layout reads", "Boundary", false,
                         C448_backbone_tests_use_neutral_layout_read);
  test_registry::AddTest(tests, "C449_backbone_layout_read_does_not_expose_authority",
                         "backbone neutral layout read does not expose authority", "Boundary", false,
                         C449_backbone_layout_read_does_not_expose_authority);
  test_registry::AddTest(tests, "C450_backbone_span_layout_state_is_neutral",
                         "backbone layout state observes outputs without old contract", "Boundary", false,
                         C450_backbone_span_layout_state_is_neutral);
  test_registry::AddTest(tests, "C451_backbone_tests_do_not_read_support_layout_contract",
                         "backbone tests do not read support layout contract", "Boundary", false,
                         C451_backbone_tests_do_not_read_old_contract);
  test_registry::AddTest(tests, "C452_backbone_layout_state_does_not_expose_old_contract_names",
                         "backbone layout state does not expose old contract names", "Boundary", false,
                         C452_backbone_layout_state_does_not_expose_old_contract_names);
  test_registry::AddTest(tests, "C453_backbone_layout_state_reads_existing_cache_without_seed_path",
                         "backbone layout state read avoids seed path", "Boundary", false,
                         C453_backbone_layout_state_reads_existing_cache_without_seed_path);
  test_registry::AddTest(tests, "C454_backbone_cache_state_uses_span_layout_cache",
                         "backbone cache state uses span layout cache owner name", "Boundary", false,
                         C454_backbone_cache_state_uses_span_layout_cache);
  test_registry::AddTest(tests, "C455_backbone_neutral_layout_api_uses_span_layout_cache",
                         "backbone neutral layout API uses span layout cache", "Boundary", false,
                         C455_backbone_neutral_layout_api_uses_span_layout_cache);
  test_registry::AddTest(tests, "C456_backbone_source_avoids_old_layout_cache_names",
                         "backbone source avoids old layout cache names", "Boundary", false,
                         C456_backbone_source_avoids_old_layout_cache_names);
  test_registry::AddTest(tests, "C457_backbone_layout_cache_boundary_behavior_unchanged",
                         "backbone layout cache boundary keeps generated outputs", "Boundary", false,
                         C457_backbone_layout_cache_boundary_behavior_unchanged);
  test_registry::AddTest(tests, "C458_backbone_existing_branch_BD_on_ABC",
                         "backbone adds an existing-pole branch using saved graph context", "Boundary", false,
                         C458_backbone_existing_branch_BD_on_ABC);
  test_registry::AddTest(tests, "C459_backbone_existing_cross_DBE_on_ABC",
                         "backbone adds an existing-pole crossing route without kind labels", "Boundary", false,
                         C459_backbone_existing_cross_DBE_on_ABC);
  test_registry::AddTest(tests, "C460_backbone_context_links_are_not_emitted",
                         "backbone saved graph context links are not emitted as new topology", "Boundary", false,
                         C460_backbone_context_links_are_not_emitted);
  test_registry::AddTest(tests, "C461_backbone_same_edge_request_skips_duplicate_context",
                         "backbone skips duplicate saved context for same-edge requests", "Boundary", false,
                         C461_backbone_same_edge_request_skips_duplicate_context);
  test_registry::AddTest(tests, "C462_backbone_no_junction_kind_after_existing_context",
                         "backbone existing graph context does not add junction kind labels", "Boundary", false,
                         C462_backbone_no_junction_kind_after_existing_context);
  test_registry::AddTest(tests, "C463_backbone_duplicate_same_edge_bundle_rejected",
                         "backbone duplicate same edge and bundle requests are rejected", "Boundary", false,
                         C463_backbone_duplicate_same_edge_bundle_rejected);
  test_registry::AddTest(tests, "C464_backbone_different_bundle_on_same_edge_allowed",
                         "backbone different bundles on the same edge are allowed", "Boundary", false,
                         C464_backbone_different_bundle_on_same_edge_allowed);
  test_registry::AddTest(tests, "C465_backbone_duplicate_policy_does_not_read_existing_spans",
                         "backbone duplicate policy reads saved edge bundles, not existing spans", "Boundary", false,
                         C465_backbone_duplicate_policy_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C466_backbone_duplicate_reject_keeps_state_unchanged",
                         "backbone duplicate reject keeps state unchanged", "Boundary", false,
                         C466_backbone_duplicate_reject_keeps_state_unchanged);
  test_registry::AddTest(tests, "C467_backbone_saves_row_port_bindings",
                         "backbone saves row to materialized port bindings", "Boundary", false,
                         C467_backbone_saves_row_port_bindings);
  test_registry::AddTest(tests, "C468_backbone_row_port_binding_is_stable_for_existing_context",
                         "backbone saves new row-port bindings without emitting context rows", "Boundary", false,
                         C468_backbone_row_port_binding_is_stable_for_existing_context);
  test_registry::AddTest(tests, "C469_backbone_row_port_binding_rejects_duplicate_without_resolution",
                         "backbone duplicate row-port binding is rejected without port resolution", "Boundary", false,
                         C469_backbone_row_port_binding_rejects_duplicate_without_resolution);
  test_registry::AddTest(tests, "C470_backbone_row_port_identity_does_not_use_position_match",
                         "backbone row-port identity does not use position matching", "Boundary", false,
                         C470_backbone_row_port_identity_does_not_use_position_match);
  test_registry::AddTest(tests, "C471_backbone_resolves_existing_port_by_binding",
                         "backbone resolves existing ports by saved row-port binding", "Boundary", false,
                         C471_backbone_resolves_existing_port_by_binding);
  test_registry::AddTest(tests, "C472_backbone_port_resolution_requires_saved_binding",
                         "backbone does not resolve ports without saved binding", "Boundary", false,
                         C472_backbone_port_resolution_requires_saved_binding);
  test_registry::AddTest(tests, "C473_backbone_resolved_port_used_by_new_span_endpoint",
                         "backbone new spans use resolved port endpoints", "Boundary", false,
                         C473_backbone_resolved_port_used_by_new_span_endpoint);
  test_registry::AddTest(tests, "C474_backbone_port_resolution_rejects_ambiguous_binding",
                         "backbone port resolution rejects ambiguous row-port bindings", "Boundary", false,
                         C474_backbone_port_resolution_rejects_ambiguous_binding);
  test_registry::AddTest(tests, "C475_backbone_port_resolution_does_not_read_existing_layout",
                         "backbone port resolution does not read existing layout", "Boundary", false,
                         C475_backbone_port_resolution_does_not_read_existing_layout);
  test_registry::AddTest(tests, "C476_backbone_branch_rows_are_separated_without_branch_kind",
                         "backbone branch rows are separated without branch kind", "Boundary", false,
                         C476_backbone_branch_rows_are_separated_without_branch_kind);
  test_registry::AddTest(tests, "C477_backbone_cross_rows_are_separated_without_cross_kind",
                         "backbone cross rows are separated without cross kind", "Boundary", false,
                         C477_backbone_cross_rows_are_separated_without_cross_kind);
  test_registry::AddTest(tests, "C478_backbone_row_separation_is_deterministic",
                         "backbone row separation is deterministic", "Invariant", false,
                         C478_backbone_row_separation_is_deterministic);
  test_registry::AddTest(tests, "C479_backbone_row_separation_does_not_change_pairs",
                         "backbone row separation does not change pair source", "Boundary", false,
                         C479_backbone_row_separation_does_not_change_pairs);
  test_registry::AddTest(tests, "C480_backbone_context_rows_affect_order_but_are_not_emitted",
                         "backbone context rows affect separation order but are not emitted", "Boundary", false,
                         C480_backbone_context_rows_affect_order_but_are_not_emitted);
  test_registry::AddTest(tests, "C481_backbone_pass_through_mode_is_accepted_in_limited_scope",
                         "backbone accepts pass-through mode for saved junction nodes", "Boundary", false,
                         C481_backbone_pass_through_mode_is_accepted_in_limited_scope);
  test_registry::AddTest(tests, "C482_backbone_pass_through_creates_explicit_intent",
                         "backbone pass-through node mode creates explicit layout intent", "Boundary", false,
                         C482_backbone_pass_through_creates_explicit_intent);
  test_registry::AddTest(tests, "C483_backbone_pass_through_ambiguous_target_rejected",
                         "backbone pass-through ambiguous row target is rejected", "Boundary", false,
                         C483_backbone_pass_through_ambiguous_target_rejected);
  test_registry::AddTest(tests, "C484_backbone_lowering_draw_uses_layout_only",
                         "backbone lowering draw uses layout only", "Boundary", false,
                         C484_backbone_lowering_draw_uses_layout_only);
  test_registry::AddTest(tests, "C485_backbone_lowering_intent_does_not_read_existing_spans",
                         "backbone lowering intent does not read existing spans", "Boundary", false,
                         C485_backbone_lowering_intent_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C486_backbone_pass_through_is_deterministic",
                         "backbone pass-through intent is deterministic", "Invariant", false,
                         C486_backbone_pass_through_is_deterministic);
  test_registry::AddTest(tests, "C487_backbone_port_resolution_requires_bundle_compatible_scope",
                         "backbone port resolution requires bundle-compatible scope", "Boundary", false,
                         C487_backbone_port_resolution_requires_bundle_compatible_scope);
  test_registry::AddTest(tests, "C488_backbone_port_resolution_accepts_same_compatible_binding",
                         "backbone accepts multiple same-compatible port bindings", "Boundary", false,
                         C488_backbone_port_resolution_accepts_same_compatible_binding);
  test_registry::AddTest(tests, "C489_backbone_port_binding_index_invariant",
                         "backbone port binding index exposes all compatible bindings", "Boundary", false,
                         C489_backbone_port_binding_index_invariant);
  test_registry::AddTest(tests, "C490_backbone_duplicate_same_edge_bundle_lane_rejected",
                         "backbone duplicate same edge bundle lane requests do not duplicate", "Boundary", false,
                         C490_backbone_duplicate_same_edge_bundle_lane_rejected);
  test_registry::AddTest(tests, "C491_backbone_branch_lowering_v1_affects_geom",
                         "backbone branch lowering v1 affects layout geometry", "Invariant", false,
                         C491_backbone_branch_lowering_v1_affects_geom);
  test_registry::AddTest(tests, "C492_backbone_cross_lowering_v1_affects_only_new_links",
                         "backbone cross lowering v1 affects generated links only", "Invariant", false,
                         C492_backbone_cross_lowering_v1_affects_only_new_links);
  test_registry::AddTest(tests, "C493_backbone_pass_through_does_not_change_pair_open",
                         "backbone pass-through does not change pair open authority", "Boundary", false,
                         C493_backbone_pass_through_does_not_change_pair_open);
  test_registry::AddTest(tests, "C494_backbone_lowering_v1_draw_does_not_redecide",
                         "backbone lowering v1 draw does not redecide", "Boundary", false,
                         C494_backbone_lowering_v1_draw_does_not_redecide);
  test_registry::AddTest(tests, "C495_backbone_lowering_v1_does_not_read_existing_spans",
                         "backbone lowering v1 does not read existing spans", "Boundary", false,
                         C495_backbone_lowering_v1_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C496_backbone_junction_v1_deterministic",
                         "backbone junction v1 output is deterministic", "Invariant", false,
                         C496_backbone_junction_v1_deterministic);
  test_registry::AddTest(tests, "C497_backbone_context_rows_order_but_do_not_materialize",
                         "backbone context rows order placement without materializing", "Boundary", false,
                         C497_backbone_context_rows_order_but_do_not_materialize);
  test_registry::AddTest(tests, "C498_backbone_saved_graph_remains_topology_authority",
                         "backbone saved graph remains topology authority", "Boundary", false,
                         C498_backbone_saved_graph_remains_topology_authority);
  test_registry::AddTest(tests, "C499_backbone_context_link_is_not_saved",
                         "backbone context links are not saved as new edges", "Boundary", false,
                         C499_backbone_context_link_is_not_saved);
  test_registry::AddTest(tests, "C500_backbone_context_link_requires_saved_edge_ref",
                         "backbone context links require saved edge refs", "Boundary", false,
                         C500_backbone_context_link_requires_saved_edge_ref);
  test_registry::AddTest(tests, "C501_backbone_gate3_contract_passes",
                         "backbone save graph keeps context links out of save targets", "Boundary", false,
                         C501_backbone_gate3_contract_passes);
  test_registry::AddTest(tests, "C502_backbone_span_bindings_save_lane",
                         "backbone saved span bindings carry lane identity", "Boundary", false,
                         C502_backbone_span_bindings_save_lane);
  test_registry::AddTest(tests, "C503_backbone_duplicate_span_binding_rejected_by_lane",
                         "backbone rejects duplicate saved span binding lanes", "Boundary", false,
                         C503_backbone_duplicate_span_binding_rejected_by_lane);
  test_registry::AddTest(tests, "C504_backbone_span_resolution_does_not_read_geometry_or_layout",
                         "backbone span resolution reads saved bindings only", "Boundary", false,
                         C504_backbone_span_resolution_does_not_read_geometry_or_layout);
  test_registry::AddTest(tests, "C505_backbone_save_graph_propagates_span_binding_failure",
                         "backbone save graph propagates span binding failures", "Boundary", false,
                         C505_backbone_save_graph_propagates_span_binding_failure);
  test_registry::AddTest(tests, "C506_backbone_support_group_is_placement_layer",
                         "backbone support group is a placement layer", "Boundary", false,
                         C506_backbone_support_group_is_placement_layer);
  test_registry::AddTest(tests, "C507_backbone_support_group_built_after_intent",
                         "backbone support group is built after intent", "Boundary", false,
                         C507_backbone_support_group_built_after_intent);
  test_registry::AddTest(tests, "C508_backbone_support_group_drives_lowered_rules",
                         "backbone support group drives lowered rules", "Boundary", false,
                         C508_backbone_support_group_drives_lowered_rules);
  test_registry::AddTest(tests, "C509_backbone_support_group_avoids_visual_terms",
                         "backbone support group avoids visual terms", "Boundary", false,
                         C509_backbone_support_group_avoids_visual_terms);
  test_registry::AddTest(tests, "C510_backbone_layout_consumes_group_offset",
                         "backbone layout consumes support group offset", "Boundary", false,
                         C510_backbone_layout_consumes_group_offset);
  test_registry::AddTest(tests, "C511_backbone_draw_saved_from_geom",
                         "backbone draw is saved from geom", "Boundary", false,
                         C511_backbone_draw_saved_from_geom);
  test_registry::AddTest(tests, "C512_backbone_draw_does_not_read_topology",
                         "backbone draw does not read topology", "Boundary", false,
                         C512_backbone_draw_does_not_read_topology);
  test_registry::AddTest(tests, "C513_backbone_support_visual_placeholder_from_layout",
                         "backbone support visual placeholder comes from layout", "Boundary", false,
                         C513_backbone_support_visual_placeholder_from_layout);
  test_registry::AddTest(tests, "C514_backbone_draw_save_is_direct",
                         "backbone draw save is direct", "Boundary", false,
                         C514_backbone_draw_save_is_direct);
  test_registry::AddTest(tests, "C515_backbone_rejects_existing_pole_without_saved_graph",
                         "backbone rejects existing poles without saved graph", "Boundary", true,
                         C515_backbone_rejects_existing_pole_without_saved_graph);
  test_registry::AddTest(tests, "C516_backbone_generated_pole_with_saved_graph_still_connects",
                         "backbone generated poles with saved graph still connect", "Boundary", false,
                         C516_backbone_generated_pole_with_saved_graph_still_connects);
  test_registry::AddTest(tests, "C517_backbone_migration_gate_does_not_infer_from_outputs",
                         "backbone migration gate does not infer from outputs", "Boundary", false,
                         C517_backbone_migration_gate_does_not_infer_from_outputs);
  test_registry::AddTest(tests, "C518_backbone_lowered_layout_keeps_support_world_at_port_height",
                         "backbone lowered layout keeps support world at port height", "Boundary", false,
                         C518_backbone_lowered_layout_keeps_support_world_at_port_height);
  test_registry::AddTest(tests, "C519_backbone_draw_placeholder_uses_layout_points",
                         "backbone draw placeholder uses layout points", "Boundary", false,
                         C519_backbone_draw_placeholder_uses_layout_points);
  test_registry::AddTest(tests, "C520_backbone_duplicate_span_binding_preflight_before_emit",
                         "backbone duplicate span binding preflight runs before emit", "Boundary", false,
                         C520_backbone_duplicate_span_binding_preflight_before_emit);
  test_registry::AddTest(tests, "C521_backbone_context_link_preserves_saved_dir",
                         "backbone context link preserves saved direction", "Boundary", false,
                         C521_backbone_context_link_preserves_saved_dir);
  test_registry::AddTest(tests, "C522_backbone_supported_scope_is_documented",
                         "backbone supported generation scope is documented", "Boundary", false,
                         C522_backbone_supported_scope_is_documented);
  test_registry::AddTest(tests, "C523_backbone_scope_gate_matches_entrypoint",
                         "backbone scope gate matches the entrypoint", "Boundary", false,
                         C523_backbone_scope_gate_matches_entrypoint);
  test_registry::AddTest(tests, "C524_backbone_scenario_simple_line_mainline",
                         "backbone simple line scenario covers outputs and authority", "Boundary", false,
                         C524_backbone_scenario_simple_line_mainline);
  test_registry::AddTest(tests, "C525_backbone_scenario_polyline3_connectivity_once",
                         "backbone polyline scenario keeps connectivity authority single", "Boundary", false,
                         C525_backbone_scenario_polyline3_connectivity_once);
  test_registry::AddTest(tests, "C526_backbone_scenario_multiple_bundles_share_connectivity",
                         "backbone multiple-bundle scenario shares connectivity", "Boundary", false,
                         C526_backbone_scenario_multiple_bundles_share_connectivity);
  test_registry::AddTest(tests, "C527_backbone_scenario_existing_pole_continuation_uses_saved_graph",
                         "backbone existing-pole continuation scenario uses saved graph", "Boundary", false,
                         C527_backbone_scenario_existing_pole_continuation_uses_saved_graph);
  test_registry::AddTest(tests, "C528_backbone_scenario_branch_emits_new_link_only",
                         "backbone branch scenario emits only the new link", "Boundary", false,
                         C528_backbone_scenario_branch_emits_new_link_only);
  test_registry::AddTest(tests, "C529_backbone_scenario_cross_without_kind_label",
                         "backbone cross scenario runs without kind labels", "Boundary", false,
                         C529_backbone_scenario_cross_without_kind_label);
  test_registry::AddTest(tests, "C530_backbone_scenario_same_edge_different_bundle",
                         "backbone same-edge different-bundle scenario shares saved edge", "Boundary", false,
                         C530_backbone_scenario_same_edge_different_bundle);
  test_registry::AddTest(tests, "C531_backbone_scenario_duplicate_reject_unchanged",
                         "backbone duplicate scenario rejects without mutation", "Boundary", false,
                         C531_backbone_scenario_duplicate_reject_unchanged);
  test_registry::AddTest(tests, "C532_backbone_scenario_pass_through_lowering_consumer_chain",
                         "backbone pass-through lowering scenario preserves consumer chain", "Boundary", false,
                         C532_backbone_scenario_pass_through_lowering_consumer_chain);
  test_registry::AddTest(tests, "C533_backbone_build_mutation_order_is_fixed",
                         "backbone build mutation order is fixed", "Boundary", false,
                         C533_backbone_build_mutation_order_is_fixed);
  test_registry::AddTest(tests, "C534_backbone_invalid_inputs_stop_before_emit",
                         "backbone invalid inputs stop before emit", "Boundary", true,
                         C534_backbone_invalid_inputs_stop_before_emit);
  test_registry::AddTest(tests, "C535_backbone_duplicate_preflight_is_mutation_boundary",
                         "backbone duplicate preflight is the mutation boundary", "Boundary", false,
                         C535_backbone_duplicate_preflight_is_mutation_boundary);
  test_registry::AddTest(tests, "C536_backbone_draw_consumer_outputs_are_minimal",
                         "backbone draw consumer outputs are minimal", "Boundary", false,
                         C536_backbone_draw_consumer_outputs_are_minimal);
  test_registry::AddTest(tests, "C537_backbone_draw_source_has_no_decision_inputs",
                         "backbone draw source has no decision inputs", "Boundary", false,
                         C537_backbone_draw_source_has_no_decision_inputs);
  test_registry::AddTest(tests, "C538_backbone_viewer_deps_are_not_core_draw_gate",
                         "backbone viewer deps are not the core draw gate", "Boundary", false,
                         C538_backbone_viewer_deps_are_not_core_draw_gate);
  test_registry::AddTest(tests, "C539_backbone_supported_request_creates_saved_graph_outputs",
                         "backbone supported request creates saved graph outputs", "Boundary", false,
                         C539_backbone_supported_request_creates_saved_graph_outputs);
  test_registry::AddTest(tests, "C540_backbone_unsupported_request_does_not_create_v1_outputs",
                         "backbone unsupported request does not create v1 outputs", "Boundary", true,
                         C540_backbone_unsupported_request_does_not_create_v1_outputs);
  test_registry::AddTest(tests, "C541_backbone_manual_existing_pole_without_graph_is_gate_rejected",
                         "backbone manual existing pole without graph is gate rejected", "Boundary", true,
                         C541_backbone_manual_existing_pole_without_graph_is_gate_rejected);
  test_registry::AddTest(tests, "C542_backbone_usable_mainline_architecture_audit_passes",
                         "backbone usable mainline architecture audit passes", "Boundary", false,
                         C542_backbone_usable_mainline_architecture_audit_passes);
  test_registry::AddTest(tests, "C543_backbone_new_route_interior_pass_through_supported",
                         "backbone supports pass-through on a new route interior point", "Boundary", false,
                         C543_backbone_new_route_interior_pass_through_supported);
  test_registry::AddTest(tests, "C544_backbone_pole_placement_pins_generated_poles",
                         "backbone applies pole placement pin options to generated poles", "Boundary", false,
                         C544_backbone_pole_placement_pins_generated_poles);
  test_registry::AddTest(tests, "C545_backbone_interval_generates_intermediate_poles",
                         "backbone interval generates intermediate poles", "Boundary", false,
                         C545_backbone_interval_generates_intermediate_poles);
  test_registry::AddTest(tests, "C546_backbone_explicit_new_pole_node_spec_supported",
                         "backbone supports explicit new-pole node specs", "Boundary", false,
                         C546_backbone_explicit_new_pole_node_spec_supported);
  test_registry::AddTest(tests, "C547_backbone_fixed_bundle_exact_count_is_supported",
                         "backbone supports exact fixed bundle count input", "Boundary", false,
                         C547_backbone_fixed_bundle_exact_count_is_supported);
  test_registry::AddTest(tests, "C548_backbone_avoid_radius_without_points_is_noop",
                         "backbone accepts avoid radius without avoid points as no-op", "Boundary", false,
                         C548_backbone_avoid_radius_without_points_is_noop);
  test_registry::AddTest(tests, "C549_backbone_range_bundle_explicit_count_is_supported",
                         "backbone supports explicit range bundle count", "Boundary", false,
                         C549_backbone_range_bundle_explicit_count_is_supported);
  test_registry::AddTest(tests, "C550_backbone_generated_pole_uses_tangent_hint_yaw",
                         "backbone uses tangent hints for generated pole yaw", "Boundary", false,
                         C550_backbone_generated_pole_uses_tangent_hint_yaw);
  test_registry::AddTest(tests, "C551_backbone_missing_pole_type_resolves_from_bundle_templates",
                         "backbone resolves missing pole type from bundle templates", "Boundary", false,
                         C551_backbone_missing_pole_type_resolves_from_bundle_templates);
  test_registry::AddTest(tests, "C552_backbone_zero_radius_avoid_points_are_noop",
                         "backbone accepts zero-radius avoid points as no-op", "Boundary", false,
                         C552_backbone_zero_radius_avoid_points_are_noop);
  test_registry::AddTest(tests, "C553_backbone_new_midair_route_point_is_supported",
                         "backbone supports new midair route points", "Boundary", false,
                         C553_backbone_new_midair_route_point_is_supported);
  test_registry::AddTest(tests, "C554_backbone_existing_midair_route_point_is_supported",
                         "backbone supports existing saved midair route points", "Boundary", false,
                         C554_backbone_existing_midair_route_point_is_supported);
  test_registry::AddTest(tests, "C555_backbone_new_building_route_point_is_supported",
                         "backbone supports new building route points", "Boundary", false,
                         C555_backbone_new_building_route_point_is_supported);
  test_registry::AddTest(tests, "C556_backbone_building_pick_feeds_new_building_route_point",
                         "backbone supports building picks as new building route points", "Boundary", false,
                         C556_backbone_building_pick_feeds_new_building_route_point);
  test_registry::AddTest(tests, "C557_backbone_building_pick_without_id_is_supported",
                         "backbone supports building picks without object ids", "Boundary", false,
                         C557_backbone_building_pick_without_id_is_supported);
  test_registry::AddTest(tests, "C558_backbone_ground_pick_feeds_new_ground_route_point",
                         "backbone supports ground picks as new ground route points", "Boundary", false,
                         C558_backbone_ground_pick_feeds_new_ground_route_point);
  test_registry::AddTest(tests, "C559_backbone_positive_avoid_clear_of_route_is_noop",
                         "backbone accepts positive avoid constraints when the route is clear", "Boundary", false,
                         C559_backbone_positive_avoid_clear_of_route_is_noop);
  test_registry::AddTest(tests, "C560_backbone_segment_pick_without_bundle_policy_feeds_midair_route_point",
                         "backbone accepts a dry-run segment pick without selected bundle policy as a midair route point",
                         "Boundary", false, C560_backbone_segment_pick_without_bundle_policy_feeds_midair_route_point);
  test_registry::AddTest(tests, "C561_backbone_default_segment_pick_without_bundle_policy_is_ownerless_midair",
                         "backbone accepts the default segment pick without selected bundle policy as an ownerless midair point",
                         "Boundary", false, C561_backbone_default_segment_pick_without_bundle_policy_is_ownerless_midair);
  test_registry::AddTest(tests, "C562_backbone_saved_midair_node_pick_extends_from_saved_graph_node",
                         "backbone extends from a saved midair node pick using the saved graph node", "Boundary", false,
                         C562_backbone_saved_midair_node_pick_extends_from_saved_graph_node);
  test_registry::AddTest(tests, "C563_backbone_segment_pick_snaps_to_saved_ownerless_span_endpoint",
                         "backbone resolves a segment pick endpoint through saved graph ownerless span endpoints",
                         "Boundary", false, C563_backbone_segment_pick_snaps_to_saved_ownerless_span_endpoint);
  test_registry::AddTest(tests, "C564_backbone_selected_bundle_segment_pick_feeds_transient_midair_node",
                         "backbone accepts a selected-bundle segment pick transient midair node as route input",
                         "Boundary", false, C564_backbone_selected_bundle_segment_pick_feeds_transient_midair_node);
  test_registry::AddTest(tests, "C565_backbone_mixed_selected_midair_branch_generates_allowed_bundles_only",
                         "backbone materializes only allowed bundles for mixed selected midair branch picks",
                         "Boundary", false, C565_backbone_mixed_selected_midair_branch_generates_allowed_bundles_only);
  test_registry::AddTest(tests, "C566_backbone_disallowed_selected_midair_branch_is_noop",
                         "backbone treats fully disallowed selected midair branch bundles as no-op",
                         "Boundary", false, C566_backbone_disallowed_selected_midair_branch_is_noop);
  test_registry::AddTest(tests, "C567_backbone_segment_pick_midair_uses_source_span_height",
                         "backbone segment-pick midair branches use source span height", "Boundary", false,
                         C567_backbone_segment_pick_midair_uses_source_span_height);
  test_registry::AddTest(tests, "C568_backbone_source_edge_midair_branch_uses_source_curve_projection",
                         "backbone source-edge midair branches use current source curve projection", "Boundary", false,
                         C568_backbone_source_edge_midair_branch_uses_source_curve_projection);
  test_registry::AddTest(tests, "C569_backbone_render_uses_cable_template_appearance",
                         "backbone render cache uses cable template appearance", "Boundary", false,
                         C569_backbone_render_uses_cable_template_appearance);
  test_registry::AddTest(tests, "C570_backbone_support_visual_uses_visual_settings_radius",
                         "backbone support visual placeholder uses visual settings radius", "Boundary", false,
                         C570_backbone_support_visual_uses_visual_settings_radius);
  test_registry::AddTest(tests, "C571_backbone_support_visual_respects_enable_setting",
                         "backbone support visual placeholder respects enable_support_structures", "Boundary", false,
                         C571_backbone_support_visual_respects_enable_setting);
  test_registry::AddTest(tests, "C572_backbone_support_visual_radius_setting_is_mutable",
                         "backbone support visual placeholder uses updated support arm radius", "Boundary", false,
                         C572_backbone_support_visual_radius_setting_is_mutable);
  test_registry::AddTest(tests, "C573_backbone_saved_context_node_carries_support_metadata",
                         "backbone saved context nodes carry support metadata", "Boundary", false,
                         C573_backbone_saved_context_node_carries_support_metadata);
  test_registry::AddTest(tests, "C574_backbone_same_edge_different_bundle_with_pass_through_is_supported",
                         "backbone supports adding a different bundle on an existing edge with pass-through mode", "Boundary",
                         false, C574_backbone_same_edge_different_bundle_with_pass_through_is_supported);
  test_registry::AddTest(tests, "C575_backbone_stale_segment_pick_midair_duplicate_rejected_unchanged",
                         "backbone rejects duplicate stale segment-pick midair branch requests before mutation", "Boundary",
                         false, C575_backbone_stale_segment_pick_midair_duplicate_rejected_unchanged);
  test_registry::AddTest(tests, "C576_backbone_ownerless_multiple_bundles_do_not_require_pole_type",
                         "backbone ownerless-only multiple bundle routes do not require a pole type", "Boundary", false,
                         C576_backbone_ownerless_multiple_bundles_do_not_require_pole_type);
  test_registry::AddTest(tests, "C577_backbone_missing_port_band_rejects_before_mutation",
                         "backbone rejects missing port bands before topology mutation", "Boundary", false,
                         C577_backbone_missing_port_band_rejects_before_mutation);
  test_registry::AddTest(tests, "C578_backbone_segment_pick_midair_pass_through_supported",
                         "backbone supports pass-through on a segment-pick midair branch", "Boundary", false,
                         C578_backbone_segment_pick_midair_pass_through_supported);
  test_registry::AddTest(tests, "C579_backbone_polyline_avoid_detour_supported",
                         "backbone supports a single avoid detour on one segment of a polyline route", "Boundary", false,
                         C579_backbone_polyline_avoid_detour_supported);
  test_registry::AddTest(tests, "C580_backbone_interval_avoid_combination_orders_inserted_points",
                         "backbone orders interval and avoid inserted points on the source segment", "Boundary", false,
                         C580_backbone_interval_avoid_combination_orders_inserted_points);
  test_registry::AddTest(tests, "C581_backbone_inactive_bundle_missing_band_is_ignored",
                         "backbone ignores missing port bands for inactive bundles", "Boundary", false,
                         C581_backbone_inactive_bundle_missing_band_is_ignored);
  test_registry::AddTest(tests, "C582_backbone_multiple_avoid_points_on_one_segment_supported",
                         "backbone supports multiple avoid points on one route segment", "Boundary", false,
                         C582_backbone_multiple_avoid_points_on_one_segment_supported);
  test_registry::AddTest(tests, "C583_backbone_avoid_points_on_multiple_segments_supported",
                         "backbone supports avoid points on multiple route segments", "Boundary", false,
                         C583_backbone_avoid_points_on_multiple_segments_supported);
  test_registry::AddTest(tests, "C584_backbone_ownerless_interval_inserts_ownerless_nodes",
                         "backbone interval insertion inherits ownerless support", "Boundary", false,
                         C584_backbone_ownerless_interval_inserts_ownerless_nodes);
  test_registry::AddTest(tests, "C585_backbone_duplicate_avoid_points_are_coalesced",
                         "backbone coalesces duplicate avoid detour points", "Boundary", false,
                         C585_backbone_duplicate_avoid_points_are_coalesced);
  test_registry::AddTest(tests, "C594_backbone_avoid_point_at_route_endpoint_is_noop",
                         "backbone treats an avoid point exactly at a route endpoint as no-op", "Boundary", false,
                         C594_backbone_avoid_point_at_route_endpoint_is_noop);
  test_registry::AddTest(tests, "C586_backbone_avoid_detour_replaces_interval_at_same_t",
                         "backbone avoid detour replaces interval insert at the same route position", "Boundary", false,
                         C586_backbone_avoid_detour_replaces_interval_at_same_t);
  test_registry::AddTest(tests, "C587_backbone_create_midair_node_without_selected_bundle_supported",
                         "backbone creates explicit segment-pick midair nodes without selected bundle policy", "Boundary",
                         false, C587_backbone_create_midair_node_without_selected_bundle_supported);
  test_registry::AddTest(tests, "C588_backbone_corner_avoid_detour_supported",
                         "backbone supports avoid detours at an internal route corner", "Boundary", false,
                         C588_backbone_corner_avoid_detour_supported);
  test_registry::AddTest(tests, "C589_backbone_selected_bundle_policy_blocks_unselected_bundle",
                         "backbone selected-bundle midair policy blocks unselected request bundles", "Boundary", false,
                         C589_backbone_selected_bundle_policy_blocks_unselected_bundle);
  test_registry::AddTest(tests, "C590_backbone_inactive_pass_through_bundle_rejected_before_noop",
                         "backbone rejects pass-through modes for bundles made inactive by selected policy", "Boundary",
                         false, C590_backbone_inactive_pass_through_bundle_rejected_before_noop);
  test_registry::AddTest(tests, "C591_backbone_saved_selected_midair_continuation_uses_request_bundles",
                         "backbone saved selected midair continuation uses request bundles", "Boundary", false,
                         C591_backbone_saved_selected_midair_continuation_uses_request_bundles);
  test_registry::AddTest(tests, "C592_backbone_saved_selected_midair_reverse_continuation_uses_request_bundles",
                         "backbone saved selected midair reverse continuation uses request bundles", "Boundary", false,
                         C592_backbone_saved_selected_midair_reverse_continuation_uses_request_bundles);
  test_registry::AddTest(tests, "C593_backbone_saved_selected_midair_allows_request_pass_through",
                         "backbone saved selected midair allows request pass-through", "Boundary", false,
                         C593_backbone_saved_selected_midair_allows_request_pass_through);
  test_registry::AddTest(tests, "C595_backbone_avoid_point_at_explicit_existing_support_is_noop",
                         "backbone treats an avoid point exactly at an explicit existing support as no-op", "Boundary",
                         false, C595_backbone_avoid_point_at_explicit_existing_support_is_noop);
  test_registry::AddTest(tests, "C596_backbone_avoid_point_at_explicit_new_support_is_noop",
                         "backbone treats an avoid point exactly at an explicit new support as no-op", "Boundary", false,
                         C596_backbone_avoid_point_at_explicit_new_support_is_noop);
  test_registry::AddTest(tests, "C597_backbone_selected_building_pick_generates_selected_bundle_only",
                         "backbone selected building picks generate selected bundles only", "Boundary", false,
                         C597_backbone_selected_building_pick_generates_selected_bundle_only);
  test_registry::AddTest(tests, "C598_backbone_selected_saved_building_node_pick_generates_selected_bundle_only",
                         "backbone selected saved building node picks generate selected bundles only", "Boundary", false,
                         C598_backbone_selected_saved_building_node_pick_generates_selected_bundle_only);
  test_registry::AddTest(tests, "C600_backbone_selected_existing_pole_pick_generates_selected_bundle_only",
                         "backbone selected existing pole pick generates selected bundle only", "Boundary", false,
                         C600_backbone_selected_existing_pole_pick_generates_selected_bundle_only);
  test_registry::AddTest(tests, "C601_backbone_context_only_bundle_policy_does_not_filter_new_route",
                         "backbone context-only selected bundle policy does not filter new route bundles",
                         "Boundary", false, C601_backbone_context_only_bundle_policy_does_not_filter_new_route);
  test_registry::AddTest(tests, "C602_backbone_context_only_pole_band_does_not_filter_new_route",
                         "backbone rejects missing saved bands without affecting unrelated route generation",
                         "Boundary", false, C602_backbone_context_only_pole_band_does_not_filter_new_route);
  test_registry::AddTest(tests, "C603_backbone_context_node_does_not_affect_generated_endpoint_yaw",
                         "backbone context nodes do not affect generated endpoint pole yaw",
                         "Boundary", false, C603_backbone_context_node_does_not_affect_generated_endpoint_yaw);
  test_registry::AddTest(tests, "C604_backbone_large_avoid_detour_clears_radius",
                         "backbone large avoid detours clear the requested radius", "Boundary", false,
                         C604_backbone_large_avoid_detour_clears_radius);
  test_registry::AddTest(tests, "C605_backbone_find_backbone_route_uses_saved_ownerless_graph",
                         "backbone route queries use saved ownerless backbone graph", "Boundary", false,
                         C605_backbone_find_backbone_route_uses_saved_ownerless_graph);
  test_registry::AddTest(tests, "C608_backbone_saved_backbone_result_does_not_duplicate_saved_pole_nodes",
                         "backbone SavedBackboneResult does not duplicate saved pole nodes", "Boundary", false,
                         C608_backbone_saved_backbone_result_does_not_duplicate_saved_pole_nodes);
  test_registry::AddTest(tests, "C609_backbone_ordinary_bend_does_not_lower",
                         "backbone ordinary bends do not create lowering intent", "Boundary", false,
                         C609_backbone_ordinary_bend_does_not_lower);
  test_registry::AddTest(tests, "C610_backbone_conflict_lowers_eligible_bundle_endpoint_only",
                         "backbone conflicts lower only the eligible bundle junction endpoint", "Boundary", false,
                         C610_backbone_conflict_lowers_eligible_bundle_endpoint_only);
  test_registry::AddTest(tests, "C607_backbone_saved_backbone_result_preserves_saved_ownerless_route_index",
                         "backbone SavedBackboneResult preserves saved ownerless route index", "Boundary", false,
                         C607_backbone_saved_backbone_result_preserves_saved_ownerless_route_index);
  test_registry::AddTest(tests, "C606_backbone_saved_backbone_result_exposes_saved_ownerless_node",
                         "backbone SavedBackboneResult exposes saved ownerless nodes", "Boundary", false,
                         C606_backbone_saved_backbone_result_exposes_saved_ownerless_node);
  test_registry::AddTest(tests, "C599_backbone_selected_saved_building_node_policy_persists_after_branch",
                         "backbone selected saved building node policy persists after branch", "Boundary", false,
                         C599_backbone_selected_saved_building_node_policy_persists_after_branch);
  test_registry::AddTest(tests, "C611_backbone_direct_derive_restores_saved_span_outputs",
                         "backbone direct derive restores saved span outputs without recalc", "Boundary", false,
                         C611_backbone_direct_derive_restores_saved_span_outputs);
  test_registry::AddTest(tests, "C612_backbone_direct_derive_does_not_call_recalc_paths",
                         "backbone direct derive avoids recalc and materialization paths", "Boundary", false,
                         C612_backbone_direct_derive_does_not_call_recalc_paths);
  test_registry::AddTest(tests, "C613_backbone_port_edit_rederives_generated_span_without_recalc",
                         "backbone port edits rederive generated span outputs without recalc", "Boundary", false,
                         C613_backbone_port_edit_rederives_generated_span_without_recalc);
  test_registry::AddTest(tests, "C614_backbone_update_plan_uses_coarse_kinds",
                         "backbone update planning uses the four coarse update kinds", "Boundary", false,
                         C614_backbone_update_plan_uses_coarse_kinds);
  test_registry::AddTest(tests, "C615_backbone_regenerate_plan_is_not_local_fallback",
                         "backbone migration update plans do not run a local fallback", "Boundary", true,
                         C615_backbone_regenerate_plan_is_not_local_fallback);
  test_registry::AddTest(tests, "C616_backbone_reposition_keeps_saved_graph_identity",
                         "backbone reposition updates keep saved graph identity", "Boundary", false,
                         C616_backbone_reposition_keeps_saved_graph_identity);
  test_registry::AddTest(tests, "C617_backbone_reshape_does_not_rewrite_layout",
                         "backbone reshape updates do not rewrite layout", "Boundary", false,
                         C617_backbone_reshape_does_not_rewrite_layout);
  test_registry::AddTest(tests, "C618_backbone_redraw_does_not_rewrite_layout_or_geom",
                         "backbone redraw updates do not rewrite layout or geom", "Boundary", false,
                         C618_backbone_redraw_does_not_rewrite_layout_or_geom);
  test_registry::AddTest(tests, "C619_backbone_reposition_updates_only_affected_spans",
                         "backbone reposition updates only affected spans", "Boundary", false,
                         C619_backbone_reposition_updates_only_affected_spans);
  test_registry::AddTest(tests, "C620_backbone_update_boundary_has_no_operation_specific_kinds",
                         "backbone update boundary has no operation-specific update kinds", "Boundary", false,
                         C620_backbone_update_boundary_has_no_operation_specific_kinds);
  test_registry::AddTest(tests, "C621_backbone_sag_reshape_updates_geom_only",
                         "backbone sag reshapes curve and bounds without changing layout or topology", "Boundary", false,
                         C621_backbone_sag_reshape_updates_geom_only);
  test_registry::AddTest(tests, "C622_backbone_stage_timing_is_diagnostic_only",
                         "backbone reports generation and update stage timing without changing decisions", "Boundary", false,
                         C622_backbone_stage_timing_is_diagnostic_only);
  test_registry::AddTest(tests, "C623_backbone_layout_settings_reject_before_mutation",
                         "backbone layout settings regenerate derived outputs", "Boundary", false,
                         C623_backbone_layout_settings_reject_before_mutation);
  test_registry::AddTest(tests, "C624_backbone_variation_settings_reject_before_mutation",
                         "backbone variation settings reject before mutation while unsupported", "Boundary", true,
                         C624_backbone_variation_settings_reject_before_mutation);
  test_registry::AddTest(tests, "C625_backbone_context_profile_reject_before_mutation",
                         "backbone context profile rejects before mutation while unsupported", "Boundary", true,
                         C625_backbone_context_profile_reject_before_mutation);
  test_registry::AddTest(tests, "C626_backbone_cable_template_updates_derive_outputs",
                         "backbone cable shape and render updates directly derive outputs", "Boundary", false,
                         C626_backbone_cable_template_updates_derive_outputs);
  test_registry::AddTest(tests, "C627_backbone_legacy_topology_apis_are_removed",
                         "retired topology APIs are absent from the public surface", "Boundary", false,
                         C627_backbone_legacy_topology_apis_are_removed);
  test_registry::AddTest(tests, "C628_backbone_active_pole_type_update_repositions",
                         "active backbone pole type placement updates derive outputs",
                         "Boundary", true,
                         C628_backbone_active_pole_type_update_repositions);
  test_registry::AddTest(tests, "C634_backbone_terminal_nodes_create_no_node_patch_curve",
                         "terminal backbone nodes do not create node patch curves", "Boundary", false,
                         C634_backbone_terminal_nodes_create_no_node_patch_curve);
  test_registry::AddTest(tests, "C635_backbone_simple_continuous_node_creates_node_patch_curve",
                         "simple continuous backbone nodes create one local node patch curve", "Boundary", false,
                         C635_backbone_simple_continuous_node_creates_node_patch_curve);
  test_registry::AddTest(tests, "C636_backbone_edge_body_stops_at_node_patch_boundaries",
                         "backbone edge bodies stop at node patch boundaries", "Boundary", false,
                         C636_backbone_edge_body_stops_at_node_patch_boundaries);
  test_registry::AddTest(tests, "C637_backbone_node_patch_edge_body_boundary_tangents_are_g1",
                         "backbone node patch and edge body boundary tangents are G1-compatible", "Boundary", false,
                         C637_backbone_node_patch_edge_body_boundary_tangents_are_g1);
  test_registry::AddTest(tests, "C638_backbone_visual_curve_parts_are_finite",
                         "backbone visual curve part samples are finite", "Boundary", false,
                         C638_backbone_visual_curve_parts_are_finite);
  test_registry::AddTest(tests, "C639_backbone_node_patch_curve_is_not_straight_chord",
                         "backbone node patch curve is not straight chord", "Boundary", false,
                         C639_backbone_node_patch_curve_is_not_straight_chord);
  test_registry::AddTest(tests, "C640_backbone_node_patch_exposes_bezier_debug_controls",
                         "backbone node patch exposes Bezier debug controls", "Boundary", false,
                         C640_backbone_node_patch_exposes_bezier_debug_controls);
  test_registry::AddTest(tests, "C641_backbone_pole_tilt_refreshes_visual_curve_parts",
                         "backbone pole tilt refreshes visual curve parts from moved endpoints", "Boundary", false,
                         C641_backbone_pole_tilt_refreshes_visual_curve_parts);
  test_registry::AddTest(tests, "C659_backbone_draw_time_tilt_materializes_ports_before_spans",
                         "backbone draw-time tilt materializes ports and spans from tilted support frame",
                         "Boundary", false, C659_backbone_draw_time_tilt_materializes_ports_before_spans);
  test_registry::AddTest(tests, "C642_backbone_edge_body_uses_formal_sag_curve",
                         "backbone edge body shares formal sag samples and non-horizontal support tangents",
                         "Boundary", false, C642_backbone_edge_body_uses_formal_sag_curve);
  test_registry::AddTest(tests, "C643_backbone_node_patch_uses_inner_fillet_with_attachment_reference",
                         "backbone node patch uses an inner fillet while retaining attachment reference", "Boundary",
                         false, C643_backbone_node_patch_uses_inner_fillet_with_attachment_reference);
  test_registry::AddTest(tests, "C644_backbone_patch_boundaries_extend_sag_tangents_below_attachment",
                         "backbone patch boundaries extend main sag tangents below fixed attachments", "Boundary",
                         false, C644_backbone_patch_boundaries_extend_sag_tangents_below_attachment);
  test_registry::AddTest(tests, "C645_backbone_node_patch_inner_fillet_keeps_curvature",
                         "backbone node patch inner fillet keeps local curvature", "Boundary", false,
                         C645_backbone_node_patch_inner_fillet_keeps_curvature);
  test_registry::AddTest(tests, "C646_backbone_node_patch_turns_monotonically_inside_corner",
                         "backbone node patch turns monotonically inside the corner", "Boundary", false,
                         C646_backbone_node_patch_turns_monotonically_inside_corner);
  test_registry::AddTest(tests, "C647_backbone_node_patch_uses_incident_cable_appearance",
                         "backbone node patch uses the same cable appearance as incident edge bodies", "Boundary",
                         false, C647_backbone_node_patch_uses_incident_cable_appearance);
  test_registry::AddTest(tests, "C655_backbone_node_patch_grouping_uses_band_identity",
                         "backbone node patch grouping includes local band identity", "Boundary", false,
                         C655_backbone_node_patch_grouping_uses_band_identity);
  test_registry::AddTest(tests, "C656_backbone_node_patch_does_not_mix_base_and_extra_sections",
                         "backbone node patch does not connect base and populated span sections", "Boundary", false,
                         C656_backbone_node_patch_does_not_mix_base_and_extra_sections);
  test_registry::AddTest(tests, "C657_backbone_node_patch_does_not_mix_extra_instance_indices",
                         "backbone node patch does not connect different populated section instances", "Boundary",
                         false, C657_backbone_node_patch_does_not_mix_extra_instance_indices);
  test_registry::AddTest(tests, "C648_population_same_seed_is_stable",
                         "cable population is stable for the same explicit seed", "Invariant", false,
                         C648_population_same_seed_is_stable);
  test_registry::AddTest(tests, "C649_population_span_identity_changes_placement",
                         "cable population placement is keyed by logical span identity", "Invariant", false,
                         C649_population_span_identity_changes_placement);
  test_registry::AddTest(tests, "C650_population_reserve_blocks_candidates",
                         "cable population reserves block candidate pairs", "Invariant", false,
                         C650_population_reserve_blocks_candidates);
  test_registry::AddTest(tests, "C651_population_spacing_rejects_overlap",
                         "cable population spacing rejects overlapping candidates", "Invariant", false,
                         C651_population_spacing_rejects_overlap);
  test_registry::AddTest(tests, "C652_population_endpoint_failure_omits_pair",
                         "cable population endpoint failure omits the whole pair", "Boundary", true,
                         C652_population_endpoint_failure_omits_pair);
  test_registry::AddTest(tests, "C653_population_rejects_duplicate_band_identity",
                         "cable population rejects duplicate local band identity", "Boundary", true,
                         C653_population_rejects_duplicate_band_identity);
  test_registry::AddTest(tests, "C654_population_does_not_mutate_logical_topology",
                         "cable population changes visual output only", "Boundary", false,
                         C654_population_does_not_mutate_logical_topology);
  test_registry::AddTest(tests, "C686_population_rule_on_bundle_template_adds_visual_only_sections",
                         "bundle template population rules add derived visual sections without topology mutation",
                         "Boundary", false, C686_population_rule_on_bundle_template_adds_visual_only_sections);
  test_registry::AddTest(tests, "C687_population_rule_update_is_reshape_not_regenerate",
                         "bundle template population rule edits are reshape updates instead of regenerate",
                         "Boundary", false, C687_population_rule_update_is_reshape_not_regenerate);
  test_registry::AddTest(tests, "C689_wrap_rule_derives_carried_helix_without_topology",
                         "wrap population rules derive a carried helix around the base cable without topology mutation",
                         "Invariant", false, C689_wrap_rule_derives_carried_helix_without_topology);
  test_registry::AddTest(tests, "C690_wrap_sections_do_not_join_node_patches",
                         "wrap sections stay trimmed off the supports and never join node patches",
                         "Boundary", false, C690_wrap_sections_do_not_join_node_patches);
  test_registry::AddTest(tests, "C730_same_kind_bundle_templates_can_coexist",
                         "bundle template identity is separate from BundleKind category",
                         "Invariant", false, C730_same_kind_bundle_templates_can_coexist);
  test_registry::AddTest(tests, "C731_backbone_spec_references_duplicate_kind_templates",
                         "backbone spec can generate two templates with the same BundleKind",
                         "Invariant", false, C731_backbone_spec_references_duplicate_kind_templates);
  test_registry::AddTest(tests, "C732_population_rule_owner_is_bundle_template_id",
                         "population rules are owned by BundleTemplateId, not BundleKind",
                         "Invariant", false, C732_population_rule_owner_is_bundle_template_id);
  test_registry::AddTest(tests, "C733_regenerate_scope_uses_bundle_template_id",
                         "bundle template regenerate scope matches BundleTemplateId only",
                         "Invariant", false, C733_regenerate_scope_uses_bundle_template_id);
  test_registry::AddTest(tests, "C734_cable_template_lookup_is_not_kind_based",
                         "duplicate-kind bundle templates use their own cable template",
                         "Invariant", false, C734_cable_template_lookup_is_not_kind_based);
  test_registry::AddTest(tests, "C735_bundle_template_id_source_guard",
                         "source guard rejects BundleKind as bundle template identity",
                         "Boundary", false, C735_bundle_template_id_source_guard);
  test_registry::AddTest(tests, "C736_unsupported_hold_docs_do_not_restore_supported_backbone_updates",
                         "source guard keeps supported backbone updates out of the unsupported hold list",
                         "Boundary", false, C736_unsupported_hold_docs_do_not_restore_supported_backbone_updates);
  test_registry::AddTest(tests, "C737_backbone_overlay_edge_endpoint_snap_returns_pole_node_spec_id",
                         "backbone overlay edge endpoint snap returns a node id usable by GenerateFromBackboneSpec",
                         "Boundary", false, C737_backbone_overlay_edge_endpoint_snap_returns_pole_node_spec_id);
  test_registry::AddTest(tests, "C691_cable_run_id_connects_through_sections",
                         "cable run id connects through base edge bodies and their node patch",
                         "Invariant", false, C691_cable_run_id_connects_through_sections);
  test_registry::AddTest(tests, "C692_cable_run_id_connects_terminal_extension",
                         "cable run id connects an existing terminal span to the extension span",
                         "Invariant", false, C692_cable_run_id_connects_terminal_extension);
  test_registry::AddTest(tests, "C693_cable_run_id_keeps_branch_and_dead_end_separate",
                         "cable run id keeps branch spans and sharp dead-end sides separate",
                         "Invariant", false, C693_cable_run_id_keeps_branch_and_dead_end_separate);
  test_registry::AddTest(tests, "C694_cable_run_id_connects_population_instances",
                         "cable run id connects each population instance across through spans without merging base",
                         "Invariant", false, C694_cable_run_id_connects_population_instances);
  test_registry::AddTest(tests, "C695_cable_run_id_is_deterministic",
                         "cable run id assignment is deterministic for identical fresh states",
                         "Invariant", false, C695_cable_run_id_is_deterministic);
  test_registry::AddTest(tests, "C696_cable_run_id_is_visual_derived_only",
                         "cable run id is derived in curve parts and not stored in authoritative backbone state",
                         "Boundary", false, C696_cable_run_id_is_visual_derived_only);
  test_registry::AddTest(tests, "C697_backbone_edge_saves_lateral_offset_echo",
                         "backbone saved edge echoes lateral offset generation input",
                         "Invariant", false, C697_backbone_edge_saves_lateral_offset_echo);
  test_registry::AddTest(tests, "C660_backbone_regenerate_fixed_count_increase_updates_downstream_only",
                         "backbone regenerate fixed count increase updates downstream without graph identity changes",
                         "Boundary", false,
                         C660_backbone_bundle_fixed_count_migration_updates_downstream_only);
  test_registry::AddTest(tests, "C661_backbone_pair_row_axis_uses_unit_tangent_bisector",
                         "backbone pair row axis uses unit tangent bisector instead of chord length",
                         "Invariant", false, C661_backbone_pair_row_axis_uses_unit_tangent_bisector);
  test_registry::AddTest(tests, "C662_backbone_pair_row_axis_does_not_flip_lane_order",
                         "backbone pair row axis remains inside the incident span no-flip wedge",
                         "Invariant", false, C662_backbone_pair_row_axis_does_not_flip_lane_order);
  test_registry::AddTest(tests, "C663_backbone_sharp_corner_uses_dead_end_rows_and_jumpers",
                         "backbone sharp corners use per-edge dead-end rows and derived jumper curves",
                         "Invariant", false, C663_backbone_sharp_corner_uses_dead_end_rows_and_jumpers);
  test_registry::AddTest(tests, "C664_backbone_sharp_pole_facing_consumes_pair_decision",
                         "backbone sharp pole facing consumes the connectivity-owned corner decision",
                         "Boundary", false, C664_backbone_sharp_pole_facing_consumes_pair_decision);
  test_registry::AddTest(tests, "C665_backbone_midair_attachment_uses_derived_curve",
                         "backbone midair attachment evaluates the saved span curve instead of its chord",
                         "Boundary", false, C665_backbone_midair_attachment_uses_derived_curve);
  test_registry::AddTest(tests, "C666_backbone_terminal_extension_creates_connectivity_patch",
                         "backbone terminal extension connects separate generation rows through saved pair identity",
                         "Boundary", false, C666_backbone_terminal_extension_creates_connectivity_patch);
  test_registry::AddTest(tests, "C667_backbone_branch_preserves_through_patch",
                         "backbone branch addition preserves the connectivity-owned through patch",
                         "Boundary", false, C667_backbone_branch_preserves_through_patch);
  test_registry::AddTest(tests, "C668_backbone_regenerate_fixed_count_increase_preserves_lateral_offset",
                         "backbone regenerate fixed count increase preserves saved lateral offset",
                         "Boundary", false,
                         C668_backbone_bundle_count_migration_rejects_unreconstructable_lateral_offset);
  test_registry::AddTest(tests, "C669_backbone_regenerate_multi_bundle_decrease_matches_fresh",
                         "backbone regenerate multi-bundle count decrease matches fresh generation",
                         "Invariant", false, C669_backbone_bundle_count_migration_rejects_multi_bundle_group_offset);
  test_registry::AddTest(tests, "C670_backbone_regenerate_pair_rows_match_fresh",
                         "backbone regenerate pair rows match fresh generation",
                         "Invariant", false, C670_backbone_bundle_count_migration_rejects_pair_rows);
  test_registry::AddTest(tests, "C671_backbone_regenerate_reuses_pipeline_stages",
                         "backbone regenerate reuses pipeline stages instead of local emit copies",
                         "Boundary", false, C671_backbone_bundle_count_migration_reuses_pipeline_stages);
  test_registry::AddTest(tests, "C672_backbone_regenerate_preserves_manual_ports_on_surviving_lanes",
                         "backbone regenerate preserves manual ports on surviving lanes",
                         "Invariant", false, C672_backbone_regenerate_preserves_manual_ports_on_surviving_lanes);
  test_registry::AddTest(tests, "C673_backbone_regenerate_preserves_surviving_attachment",
                         "backbone regenerate preserves attachments on surviving spans",
                         "Invariant", false, C673_backbone_bundle_count_migration_rejects_user_attachments);
  test_registry::AddTest(tests, "C698_backbone_regenerate_fixed_count_decrease_retires_lanes",
                         "backbone regenerate fixed count decrease retires removed lanes",
                         "Invariant", false, C698_backbone_regenerate_fixed_count_decrease_retires_lanes);
  test_registry::AddTest(tests, "C699_backbone_regenerate_fixed_count_decrease_preserves_lateral_offset",
                         "backbone regenerate fixed count decrease preserves saved lateral offset",
                         "Invariant", false,
                         C699_backbone_regenerate_fixed_count_decrease_preserves_lateral_offset);
  test_registry::AddTest(tests, "C700_backbone_regenerate_fixed_count_decrease_rejects_retired_attachment",
                         "backbone regenerate fixed count decrease rejects attachments on retired lanes",
                         "Boundary", false,
                         C700_backbone_regenerate_fixed_count_decrease_rejects_retired_attachment);
  test_registry::AddTest(tests, "C701_backbone_regenerate_source_does_not_infer_topology_from_outputs",
                         "backbone regenerate source does not infer topology from derived outputs",
                         "Boundary", false,
                         C701_backbone_regenerate_source_does_not_infer_topology_from_outputs);
  test_registry::AddTest(tests, "C702_backbone_regenerate_fixed_count_roundtrip_matches_fresh",
                         "backbone regenerate fixed count roundtrip matches fresh count one",
                         "Invariant", false,
                         C702_backbone_regenerate_fixed_count_roundtrip_matches_fresh);
  test_registry::AddTest(tests, "C742_backbone_bundle_count_decrease_allows_metadata_change",
                         "backbone fixed count decrease permits a simultaneous metadata rename",
                         "Invariant", false,
                         C742_backbone_bundle_count_decrease_allows_metadata_change);
  test_registry::AddTest(tests, "C743_backbone_bundle_template_change_classification_has_one_field_owner",
                         "bundle template fields are compared only by the change classifier",
                         "Boundary", false,
                         C743_backbone_bundle_template_change_classification_has_one_field_owner);
  test_registry::AddTest(tests, "C744_backbone_span_layout_group_keys_have_one_definition",
                         "span layout group-key collection has one shared definition",
                         "Boundary", false,
                         C744_backbone_span_layout_group_keys_have_one_definition);
  test_registry::AddTest(tests, "C745_wrap_behavior_has_one_production_owner",
                         "wrap profile behavior is owned only by section behavior",
                         "Boundary", false,
                         C745_wrap_behavior_has_one_production_owner);
  test_registry::AddTest(tests, "C746_backbone_generation_trial_copy_stays_under_cost_gate",
                         "generation trial copy stays within the 66-pole cost gate",
                         "Invariant", false,
                         C746_backbone_generation_trial_copy_stays_under_cost_gate);
  test_registry::AddTest(tests, "C703_backbone_regenerate_removes_migration_symbols",
                         "backbone regenerate removes bundle count migration symbols from core src",
                         "Boundary", false,
                         C703_backbone_regenerate_removes_migration_symbols);
  test_registry::AddTest(tests, "C704_backbone_regenerate_uses_per_api_entrypoint_not_plan_execution",
                         "backbone regenerate uses per API entrypoint instead of update plan execution",
                         "Boundary", false,
                         C704_backbone_regenerate_uses_per_api_entrypoint_not_plan_execution);
  test_registry::AddTest(tests, "C705_backbone_edge_bundle_order_matches_bundle_spec_order",
                         "backbone saved edge bundle order matches request bundle spec order",
                         "Invariant", false,
                         C705_backbone_edge_bundle_order_matches_bundle_spec_order);
  test_registry::AddTest(tests, "C706_backbone_regenerate_multi_bundle_count_change_matches_fresh",
                         "backbone regenerate multi-bundle count change matches fresh generation",
                         "Invariant", false,
                         C706_backbone_regenerate_multi_bundle_count_change_matches_fresh);
  test_registry::AddTest(tests, "C707_backbone_saved_edges_reconstruct_route_order",
                         "backbone saved edges reconstruct route order",
                         "Invariant", false,
                         C707_backbone_saved_edges_reconstruct_route_order);
  test_registry::AddTest(tests, "C708_backbone_regenerate_polyline_decrease_matches_fresh",
                         "backbone regenerate polyline count decrease matches fresh generation",
                         "Invariant", false,
                         C708_backbone_regenerate_polyline_decrease_matches_fresh);
  test_registry::AddTest(tests, "C709_backbone_regenerate_polyline_increase_matches_fresh",
                         "backbone regenerate polyline count increase matches fresh generation",
                         "Invariant", false,
                         C709_backbone_regenerate_polyline_increase_matches_fresh);
  test_registry::AddTest(tests, "C710_backbone_regenerate_polyline_multi_bundle_matches_fresh",
                         "backbone regenerate polyline multi-bundle count change matches fresh generation",
                         "Invariant", false,
                         C710_backbone_regenerate_polyline_multi_bundle_matches_fresh);
  test_registry::AddTest(tests, "C711_backbone_regenerate_decrease_preserves_surviving_attachment",
                         "backbone regenerate count decrease preserves attachments on surviving spans",
                         "Invariant", false,
                         C711_backbone_regenerate_decrease_preserves_surviving_attachment);
  test_registry::AddTest(tests, "C712_backbone_regenerate_cable_decision_matches_fresh",
                         "backbone cable decision updates regenerate and match fresh output",
                         "Invariant", false,
                         C712_backbone_regenerate_cable_decision_matches_fresh);
  test_registry::AddTest(tests, "C738_cable_default_endpoint_attachment_change_rejects_before_mutation",
                         "cable default endpoint attachment changes reject before mutation on existing backbone spans",
                         "Boundary", false,
                         C738_cable_default_endpoint_attachment_change_rejects_before_mutation);
  test_registry::AddTest(tests, "C713_backbone_regenerate_pole_type_structure_matches_fresh",
                         "backbone pole type structural updates regenerate and match fresh output",
                         "Invariant", false,
                         C713_backbone_regenerate_pole_type_structure_matches_fresh);
  test_registry::AddTest(tests, "C714_backbone_regenerate_rejects_retired_manual_port",
                         "backbone regenerate rejects manual ports on retired lanes",
                         "Boundary", false,
                         C714_backbone_regenerate_rejects_retired_manual_port);
  test_registry::AddTest(tests, "C715_backbone_span_branch_down_override_regenerates",
                         "backbone span branch-down override regenerates span layout and curve",
                         "Invariant", false, C715_backbone_span_branch_down_override_regenerates);
  test_registry::AddTest(tests, "C716_backbone_span_socket_override_regenerates",
                         "backbone span endpoint socket override regenerates span layout",
                         "Invariant", false, C716_backbone_span_socket_override_regenerates);
  test_registry::AddTest(tests, "C739_span_override_keeps_unrelated_route_outputs_unchanged",
                         "span override regenerate keeps unrelated route outputs unchanged",
                         "Invariant", false, C739_span_override_keeps_unrelated_route_outputs_unchanged);
  test_registry::AddTest(tests, "C740_visual_curve_part_stats_count_full_curve_builds",
                         "visual curve part stats count only final full curve builds",
                         "Invariant", false, C740_visual_curve_part_stats_count_full_curve_builds);
  test_registry::AddTest(tests, "C741_scoped_visual_curve_rebuild_matches_full_rebuild",
                         "scoped visual curve rebuild matches full rebuild",
                         "Invariant", false, C741_scoped_visual_curve_rebuild_matches_full_rebuild);
  test_registry::AddTest(tests, "C717_backbone_layout_settings_regenerate_matches_fresh",
                         "backbone layout settings regenerate and match fresh generation",
                         "Invariant", false, C717_backbone_layout_settings_regenerate_matches_fresh);
  test_registry::AddTest(tests, "C727_backbone_pipeline_execution_entry_is_build_input",
                         "pipeline execution entry is build(build_input) only",
                         "Boundary", false, C727_backbone_pipeline_execution_entry_is_build_input);
  test_registry::AddTest(tests, "C728_backbone_pipeline_has_no_run_mode_flags",
                         "backbone pipeline has no run mode or skip flags",
                         "Boundary", false, C728_backbone_pipeline_has_no_run_mode_flags);
  test_registry::AddTest(tests, "C729_backbone_regenerate_source_does_not_handbuild_outputs",
                         "backbone regenerate source does not handbuild pipeline outputs",
                         "Boundary", false, C729_backbone_regenerate_source_does_not_handbuild_outputs);
  test_registry::AddTest(tests, "C718_viewer_hit_world_height_is_not_source_edge_branch_authority",
                         "viewer hit world height is not source-edge branch authority",
                         "Boundary", false, C718_viewer_hit_world_height_is_not_source_edge_branch_authority);
  test_registry::AddTest(tests, "C719_source_edge_branch_endpoint_follows_current_curve_projection",
                         "source-edge branch endpoint follows current curve projection",
                         "Invariant", false, C719_source_edge_branch_endpoint_follows_current_curve_projection);
  test_registry::AddTest(tests, "C720_source_edge_pipeline_front_half_does_not_read_curve_projection",
                         "source-edge pipeline front half does not read curve projection",
                         "Boundary", false, C720_source_edge_pipeline_front_half_does_not_read_curve_projection);
  test_registry::AddTest(tests, "C721_source_edge_identity_survives_projection_update",
                         "source-edge identity survives projection update",
                         "Invariant", false, C721_source_edge_identity_survives_projection_update);
  test_registry::AddTest(tests, "C722_unresolved_source_edge_reference_fails_before_mutation",
                         "unresolved source-edge reference fails before mutation",
                         "Boundary", true, C722_unresolved_source_edge_reference_fails_before_mutation);
  test_registry::AddTest(tests, "C723_source_edge_branch_does_not_change_source_sag",
                         "source-edge branch does not change source sag",
                         "Invariant", false, C723_source_edge_branch_does_not_change_source_sag);
  test_registry::AddTest(tests, "C724_source_template_sag_change_updates_branch_projection",
                         "source template sag change updates branch projection",
                         "Invariant", false, C724_source_template_sag_change_updates_branch_projection);
  test_registry::AddTest(tests, "C725_source_layout_settings_update_keeps_branch_projection_current",
                         "source layout settings update keeps branch projection current",
                         "Invariant", false, C725_source_layout_settings_update_keeps_branch_projection_current);
  test_registry::AddTest(tests, "C726_source_edge_branch_projection_does_not_require_prior_curve_cache",
                         "source-edge branch projection does not require prior curve cache",
                         "Boundary", false, C726_source_edge_branch_projection_does_not_require_prior_curve_cache);
  test_registry::AddTest(tests, "C674_backbone_port_band_selection_has_one_owner",
                         "backbone and post-edit port band selection use saved binding or one shared selector",
                         "Boundary", false, C674_backbone_port_band_selection_has_one_owner);
  test_registry::AddTest(tests, "C675_backbone_layout_yaw_does_not_read_debug_records",
                         "production port layout yaw reads saved binding instead of debug records",
                         "Boundary", false, C675_backbone_layout_yaw_does_not_read_debug_records);
  test_registry::AddTest(tests, "C676_backbone_noop_move_preserves_port_positions_exactly",
                         "no-op pole movement preserves generated backbone port positions exactly",
                         "Invariant", false, C676_backbone_noop_move_preserves_port_positions_exactly);
  test_registry::AddTest(tests, "C677_backbone_corner_scale_has_one_definition",
                         "corner side scale and inner turn side each have one production definition",
                         "Boundary", false, C677_backbone_corner_scale_has_one_definition);
}


WIRE_REGISTER_TEST_SUITE(register_tests);

} // namespace backbone_tests
