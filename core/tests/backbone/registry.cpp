#include "cases.hpp"
#include "../registry.hpp"

namespace backbone_tests {

void register_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C368_bb2_smoke_line", "bb2 generates the milestone-1 line slice", "Invariant", false,
                         C368_bb2_smoke_line);
  test_registry::AddTest(tests, "C369_bb2_rules_saved", "bb2 saves SpanLayoutRules for each generated span", "Invariant",
                         false, C369_bb2_rules_saved);
  test_registry::AddTest(tests, "C370_bb2_no_v1_deps", "bb2 source does not depend on v1 generation internals",
                         "Boundary", false, C370_bb2_no_v1_deps);
  test_registry::AddTest(tests, "C371_bb2_rejects_unsupported", "bb2 rejects unsupported milestone-1 inputs",
                         "Boundary", true, C371_bb2_rejects_unsupported);
  test_registry::AddTest(tests, "C372_bb2_rules_do_not_seed", "bb2 saves rules without creating authority seed",
                         "Boundary", false, C372_bb2_rules_do_not_seed);
  test_registry::AddTest(tests, "C373_bb2_layout_saved_without_recalc", "bb2 saves layout immediately from rules",
                         "Boundary", false, C373_bb2_layout_saved_without_recalc);
  test_registry::AddTest(tests, "C374_bb2_layout_is_deterministic", "bb2 derives stable layout from identical input",
                         "Invariant", false, C374_bb2_layout_is_deterministic);
  test_registry::AddTest(tests, "C375_bb2_curve_saved_without_recalc", "bb2 saves curve immediately from layout",
                         "Boundary", false, C375_bb2_curve_saved_without_recalc);
  test_registry::AddTest(tests, "C376_bb2_curve_is_deterministic", "bb2 derives stable curve from identical input",
                         "Invariant", false, C376_bb2_curve_is_deterministic);
  test_registry::AddTest(tests, "C377_bb2_bounds_saved_without_recalc", "bb2 saves bounds immediately from curve",
                         "Boundary", false, C377_bb2_bounds_saved_without_recalc);
  test_registry::AddTest(tests, "C378_bb2_bounds_is_deterministic", "bb2 derives stable bounds from identical input",
                         "Invariant", false, C378_bb2_bounds_is_deterministic);
  test_registry::AddTest(tests, "C379_bb2_m1_required_outputs", "bb2 milestone 1 required outputs are fixed",
                         "Boundary", false, C379_bb2_m1_required_outputs);
  test_registry::AddTest(tests, "C380_bb2_m1_draw_outputs_saved", "bb2 milestone 1 saves draw caches",
                         "Boundary", false, C380_bb2_m1_draw_outputs_saved);
  test_registry::AddTest(tests, "C381_bb2_m1_no_recalc_contract", "bb2 milestone 1 has no recalc contract",
                         "Boundary", false, C381_bb2_m1_no_recalc_contract);
  test_registry::AddTest(tests, "C382_bb2_geom_is_single_pipeline_layer", "bb2 keeps geom as one pipeline layer",
                         "Boundary", false, C382_bb2_geom_is_single_pipeline_layer);
  test_registry::AddTest(tests, "C383_bb2_draw_is_pipeline_layer", "bb2 draw is a pipeline layer",
                         "Boundary", false, C383_bb2_draw_is_pipeline_layer);
  test_registry::AddTest(tests, "C384_bb2_topo_is_single_output_layer", "bb2 keeps topology as one output layer",
                         "Boundary", false, C384_bb2_topo_is_single_output_layer);
  test_registry::AddTest(tests, "C385_bb2_emit_is_split_by_topology_parts", "bb2 splits topology emission by part",
                         "Boundary", false, C385_bb2_emit_is_split_by_topology_parts);
  test_registry::AddTest(tests, "C386_bb2_link_pair_row_are_separate", "bb2 separates links, pairs, opens, and rows",
                         "Boundary", false, C386_bb2_link_pair_row_are_separate);
  test_registry::AddTest(tests, "C387_bb2_pairs_are_single_source", "bb2 creates pairs from graph in one place",
                         "Boundary", false, C387_bb2_pairs_are_single_source);
  test_registry::AddTest(tests, "C388_bb2_polyline3_pair_model", "bb2 represents a three-point route as links, pair, and opens",
                         "Invariant", false, C388_bb2_polyline3_pair_model);
  test_registry::AddTest(tests, "C389_bb2_row_axis_owned_by_pairs", "bb2 port rows consume pair-owned axes",
                         "Boundary", false, C389_bb2_row_axis_owned_by_pairs);
  test_registry::AddTest(tests, "C390_bb2_rejects_already_used_incident",
                         "bb2 rejects invalid pair incident ownership", "Boundary", true,
                         C390_bb2_rejects_already_used_incident);
  test_registry::AddTest(tests, "C391_bb2_no_kind_label", "bb2 does not add junction kind labels",
                         "Boundary", false, C391_bb2_no_kind_label);
  test_registry::AddTest(tests, "C392_bb2_polyline3_outputs", "bb2 saves required outputs for three-point routes",
                         "Invariant", false, C392_bb2_polyline3_outputs);
  test_registry::AddTest(tests, "C393_bb2_polyline3_is_deterministic", "bb2 derives deterministic three-point output",
                         "Invariant", false, C393_bb2_polyline3_is_deterministic);
  test_registry::AddTest(tests, "C394_bb2_existing_pole_node_is_not_recreated",
                         "bb2 uses existing pole nodes without recreating them", "Boundary", false,
                         C394_bb2_existing_pole_node_is_not_recreated);
  test_registry::AddTest(tests, "C395_bb2_is_new_does_not_affect_pairs",
                         "bb2 pair output does not change because a node is existing", "Invariant", false,
                         C395_bb2_is_new_does_not_affect_pairs);
  test_registry::AddTest(tests, "C396_bb2_existing_pole_does_not_read_existing_spans",
                         "bb2 existing pole nodes do not use existing spans for new meaning", "Boundary", false,
                         C396_bb2_existing_pole_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C397_bb2_rejects_missing_saved_midair_node_spec",
                         "bb2 rejects missing saved midair node specs", "Boundary", true,
                         C397_bb2_rejects_missing_saved_midair_node_spec);
  test_registry::AddTest(tests, "C398_bb2_rejects_missing_existing_pole", "bb2 rejects missing existing pole ids",
                         "Boundary", true, C398_bb2_rejects_missing_existing_pole);
  test_registry::AddTest(tests, "C399_bb2_existing_pole_sequence_is_deterministic",
                         "bb2 derives deterministic output with existing pole nodes", "Invariant", false,
                         C399_bb2_existing_pole_sequence_is_deterministic);
  test_registry::AddTest(tests, "C400_bb2_multiple_bundles_smoke", "bb2 generates multiple bundles on one pairs graph",
                         "Invariant", false, C400_bb2_multiple_bundles_smoke);
  test_registry::AddTest(tests, "C401_bb2_multiple_bundles_polyline3_outputs",
                         "bb2 saves required outputs for multiple bundles on a three-point route", "Invariant", false,
                         C401_bb2_multiple_bundles_polyline3_outputs);
  test_registry::AddTest(tests, "C402_bb2_bundle_spec_does_not_affect_pairs",
                         "bb2 bundle specs do not alter graph pair output", "Boundary", false,
                         C402_bb2_bundle_spec_does_not_affect_pairs);
  test_registry::AddTest(tests, "C403_bb2_existing_pole_with_multiple_bundles",
                         "bb2 combines existing pole nodes with multiple bundles", "Invariant", false,
                         C403_bb2_existing_pole_with_multiple_bundles);
  test_registry::AddTest(tests, "C404_bb2_rejects_empty_bundles", "bb2 rejects empty bundle requests", "Boundary", true,
                         C404_bb2_rejects_empty_bundles);
  test_registry::AddTest(tests, "C405_bb2_no_bundle_pair_branching", "bb2 pair building does not inspect bundle specs",
                         "Boundary", false, C405_bb2_no_bundle_pair_branching);
  test_registry::AddTest(tests, "C406_bb2_port_height_uses_pole_band", "bb2 port height comes from pole bands",
                         "Invariant", false, C406_bb2_port_height_uses_pole_band);
  test_registry::AddTest(tests, "C407_bb2_multiple_bundle_heights_are_band_based",
                         "bb2 multiple bundle heights come from pole bands", "Invariant", false,
                         C407_bb2_multiple_bundle_heights_are_band_based);
  test_registry::AddTest(tests, "C408_bb2_existing_pole_uses_actual_pole_type_height",
                         "bb2 existing pole height uses the actual pole type", "Invariant", false,
                         C408_bb2_existing_pole_uses_actual_pole_type_height);
  test_registry::AddTest(tests, "C409_bb2_rejects_missing_port_band", "bb2 rejects missing port bands", "Boundary",
                         true, C409_bb2_rejects_missing_port_band);
  test_registry::AddTest(tests, "C410_bb2_height_does_not_affect_pairs",
                         "bb2 height selection does not affect pairs", "Boundary", false,
                         C410_bb2_height_does_not_affect_pairs);
  test_registry::AddTest(tests, "C411_bb2_lateral_offset_moves_ports_along_row_axis",
                         "bb2 lateral offset moves generated output along row axis", "Invariant", false,
                         C411_bb2_lateral_offset_moves_ports_along_row_axis);
  test_registry::AddTest(tests, "C412_bb2_lateral_offset_sign_is_deterministic",
                         "bb2 lateral offset sign is deterministic", "Invariant", false,
                         C412_bb2_lateral_offset_sign_is_deterministic);
  test_registry::AddTest(tests, "C413_bb2_lateral_offset_does_not_affect_pairs",
                         "bb2 lateral offset does not affect pairs", "Boundary", false,
                         C413_bb2_lateral_offset_does_not_affect_pairs);
  test_registry::AddTest(tests, "C414_bb2_simple_avoid_detour_supported",
                         "bb2 supports a simple avoid detour", "Boundary", false,
                         C414_bb2_simple_avoid_detour_supported);
  test_registry::AddTest(tests, "C415_bb2_has_no_empty_levels_layer",
                         "bb2 has no empty levels layer", "Boundary", false,
                         C415_bb2_has_no_empty_levels_layer);
  test_registry::AddTest(tests, "C416_bb2_node_mode_not_present_is_noop",
                         "bb2 accepts kNotPresent node modes as no-op", "Invariant", false,
                         C416_bb2_node_mode_not_present_is_noop);
  test_registry::AddTest(tests, "C417_bb2_node_mode_pass_through_without_target_rejected",
                         "bb2 rejects pass-through node modes without a target row", "Boundary", true,
                         C417_bb2_node_mode_pass_through_without_target_rejected);
  test_registry::AddTest(tests, "C418_bb2_node_mode_unknown_bundle_rejected",
                         "bb2 rejects node modes for bundles absent from the request", "Boundary", true,
                         C418_bb2_node_mode_unknown_bundle_rejected);
  test_registry::AddTest(tests, "C419_bb2_node_mode_point_index_rejected",
                         "bb2 rejects out-of-range node mode point indices", "Boundary", true,
                         C419_bb2_node_mode_point_index_rejected);
  test_registry::AddTest(tests, "C420_bb2_node_mode_does_not_affect_pairs",
                         "bb2 node modes do not affect pairs", "Boundary", false,
                         C420_bb2_node_mode_does_not_affect_pairs);
  test_registry::AddTest(tests, "C421_bb2_topo_row_carries_source",
                         "bb2 topology rows carry pair row source", "Boundary", false,
                         C421_bb2_topo_row_carries_source);
  test_registry::AddTest(tests, "C422_bb2_rules_consume_topo_and_groups",
                         "bb2 rules consume topology and groups without pairs", "Boundary", false,
                         C422_bb2_rules_consume_topo_and_groups);
  test_registry::AddTest(tests, "C423_bb2_tspan_carries_endpoint_rows",
                         "bb2 topology spans carry endpoint row indices", "Boundary", false,
                         C423_bb2_tspan_carries_endpoint_rows);
  test_registry::AddTest(tests, "C424_bb2_saves_backbone_graph_nodes_edges",
                         "bb2 saves backbone graph nodes and edges", "Boundary", false,
                         C424_bb2_saves_backbone_graph_nodes_edges);
  test_registry::AddTest(tests, "C425_bb2_edge_carries_multiple_spans",
                         "bb2 backbone edge carries generated spans", "Boundary", false,
                         C425_bb2_edge_carries_multiple_spans);
  test_registry::AddTest(tests, "C426_bb2_existing_pole_resolves_graph_node",
                         "bb2 resolves saved graph node for existing poles", "Boundary", false,
                         C426_bb2_existing_pole_resolves_graph_node);
  test_registry::AddTest(tests, "C427_bb2_graph_index_links_outputs",
                         "bb2 backbone index links graph to outputs", "Boundary", false,
                         C427_bb2_graph_index_links_outputs);
  test_registry::AddTest(tests, "C428_bb2_pole_frontier_collects_incident_graph",
                         "bb2 pole frontier collects saved graph incidents", "Boundary", false,
                         C428_bb2_pole_frontier_collects_incident_graph);
  test_registry::AddTest(tests, "C429_bb2_span_frontier_collects_edge_bundle_spans",
                         "bb2 span frontier collects spans from its backbone edge bundle", "Boundary", false,
                         C429_bb2_span_frontier_collects_edge_bundle_spans);
  test_registry::AddTest(tests, "C430_bb2_frontier_uses_saved_graph_index",
                         "bb2 frontier reads saved graph indexes", "Boundary", false,
                         C430_bb2_frontier_uses_saved_graph_index);
  test_registry::AddTest(tests, "C431_bb2_edge_bundle_is_saved_backbone_unit",
                         "bb2 stores edge bundle as the saved backbone unit", "Boundary", false,
                         C431_bb2_edge_bundle_is_saved_backbone_unit);
  test_registry::AddTest(tests, "C432_bb2_multiple_bundles_create_multiple_edge_bundles",
                         "bb2 multiple bundles create multiple saved edge bundles", "Boundary", false,
                         C432_bb2_multiple_bundles_create_multiple_edge_bundles);
  test_registry::AddTest(tests, "C433_bb2_resolves_edge_for_same_poles",
                         "bb2 resolves saved edge for the same pole pair", "Boundary", false,
                         C433_bb2_resolves_edge_for_same_poles);
  test_registry::AddTest(tests, "C434_bb2_reverse_duplicate_same_bundle_rejected",
                         "bb2 reverse duplicate same-bundle generation is rejected", "Boundary", false,
                         C434_bb2_reverse_duplicate_same_bundle_rejected);
  test_registry::AddTest(tests, "C435_bb2_edge_metadata_is_not_overwritten_on_duplicate_reject",
                         "bb2 saved edge metadata is not overwritten on duplicate reject", "Boundary", false,
                         C435_bb2_edge_metadata_is_not_overwritten_on_duplicate_reject);
  test_registry::AddTest(tests, "C436_bb2_frontier_reads_edge_bundles",
                         "bb2 frontier reads edge bundles", "Boundary", false,
                         C436_bb2_frontier_reads_edge_bundles);
  test_registry::AddTest(tests, "C437_bb2_layout_save_is_direct",
                         "bb2 layout save uses direct layout storage", "Boundary", false,
                         C437_bb2_layout_save_is_direct);
  test_registry::AddTest(tests, "C438_bb2_layout_save_keeps_no_authority_contract",
                         "bb2 direct layout save keeps authority contract empty", "Boundary", false,
                         C438_bb2_layout_save_keeps_no_authority_contract);
  test_registry::AddTest(tests, "C439_bb2_source_still_avoids_support_layout_entrypoint",
                         "bb2 source avoids support layout entrypoints", "Boundary", false,
                         C439_bb2_source_still_avoids_support_layout_entrypoint);
  test_registry::AddTest(tests, "C440_bb2_does_not_read_authoritative_backbone_directly",
                         "bb2 does not read authoritative backbone directly", "Boundary", false,
                         C440_bb2_does_not_read_authoritative_backbone_directly);
  test_registry::AddTest(tests, "C441_bb2_save_backbone_edge_returns_saved_ref",
                         "bb2 backbone edge save returns saved edge ref", "Boundary", false,
                         C441_bb2_save_backbone_edge_returns_saved_ref);
  test_registry::AddTest(tests, "C442_bb2_edge_forward_uses_saved_ref",
                         "bb2 edge forward uses saved edge ref", "Boundary", false,
                         C442_bb2_edge_forward_uses_saved_ref);
  test_registry::AddTest(tests, "C443_bb2_edge_resolution_behavior_unchanged",
                         "bb2 edge resolution behavior remains unchanged after duplicate reject", "Boundary", false,
                         C443_bb2_edge_resolution_behavior_unchanged);
  test_registry::AddTest(tests, "C444_bb2_layout_uses_neutral_types",
                         "bb2 layout source uses neutral layout types", "Boundary", false,
                         C444_bb2_layout_uses_neutral_types);
  test_registry::AddTest(tests, "C445_bb2_cache_span_layout_accepts_neutral_entry",
                         "bb2 layout save accepts neutral layout entries", "Boundary", false,
                         C445_bb2_cache_span_layout_accepts_neutral_entry);
  test_registry::AddTest(tests, "C446_bb2_layout_boundary_behavior_unchanged",
                         "bb2 neutral layout boundary keeps generated outputs", "Boundary", false,
                         C446_bb2_layout_boundary_behavior_unchanged);
  test_registry::AddTest(tests, "C447_bb2_span_layout_view_reads_neutral_layout",
                         "bb2 reads generated layout through neutral view", "Boundary", false,
                         C447_bb2_span_layout_view_reads_neutral_layout);
  test_registry::AddTest(tests, "C448_bb2_tests_use_neutral_layout_read",
                         "bb2 tests use neutral layout reads", "Boundary", false,
                         C448_bb2_tests_use_neutral_layout_read);
  test_registry::AddTest(tests, "C449_bb2_layout_read_does_not_expose_authority",
                         "bb2 neutral layout read does not expose authority", "Boundary", false,
                         C449_bb2_layout_read_does_not_expose_authority);
  test_registry::AddTest(tests, "C450_bb2_span_layout_state_is_neutral",
                         "bb2 layout state observes outputs without old contract", "Boundary", false,
                         C450_bb2_span_layout_state_is_neutral);
  test_registry::AddTest(tests, "C451_bb2_tests_do_not_read_support_layout_contract",
                         "bb2 tests do not read support layout contract", "Boundary", false,
                         C451_bb2_tests_do_not_read_old_contract);
  test_registry::AddTest(tests, "C452_bb2_layout_state_does_not_expose_old_contract_names",
                         "bb2 layout state does not expose old contract names", "Boundary", false,
                         C452_bb2_layout_state_does_not_expose_old_contract_names);
  test_registry::AddTest(tests, "C453_bb2_layout_state_reads_existing_cache_without_seed_path",
                         "bb2 layout state read avoids seed path", "Boundary", false,
                         C453_bb2_layout_state_reads_existing_cache_without_seed_path);
  test_registry::AddTest(tests, "C454_bb2_cache_state_uses_span_layout_cache",
                         "bb2 cache state uses span layout cache owner name", "Boundary", false,
                         C454_bb2_cache_state_uses_span_layout_cache);
  test_registry::AddTest(tests, "C455_bb2_neutral_layout_api_uses_span_layout_cache",
                         "bb2 neutral layout API uses span layout cache", "Boundary", false,
                         C455_bb2_neutral_layout_api_uses_span_layout_cache);
  test_registry::AddTest(tests, "C456_bb2_source_avoids_old_layout_cache_names",
                         "bb2 source avoids old layout cache names", "Boundary", false,
                         C456_bb2_source_avoids_old_layout_cache_names);
  test_registry::AddTest(tests, "C457_bb2_layout_cache_boundary_behavior_unchanged",
                         "bb2 layout cache boundary keeps generated outputs", "Boundary", false,
                         C457_bb2_layout_cache_boundary_behavior_unchanged);
  test_registry::AddTest(tests, "C458_bb2_existing_branch_BD_on_ABC",
                         "bb2 adds an existing-pole branch using saved graph context", "Boundary", false,
                         C458_bb2_existing_branch_BD_on_ABC);
  test_registry::AddTest(tests, "C459_bb2_existing_cross_DBE_on_ABC",
                         "bb2 adds an existing-pole crossing route without kind labels", "Boundary", false,
                         C459_bb2_existing_cross_DBE_on_ABC);
  test_registry::AddTest(tests, "C460_bb2_context_links_are_not_emitted",
                         "bb2 saved graph context links are not emitted as new topology", "Boundary", false,
                         C460_bb2_context_links_are_not_emitted);
  test_registry::AddTest(tests, "C461_bb2_same_edge_request_skips_duplicate_context",
                         "bb2 skips duplicate saved context for same-edge requests", "Boundary", false,
                         C461_bb2_same_edge_request_skips_duplicate_context);
  test_registry::AddTest(tests, "C462_bb2_no_junction_kind_after_existing_context",
                         "bb2 existing graph context does not add junction kind labels", "Boundary", false,
                         C462_bb2_no_junction_kind_after_existing_context);
  test_registry::AddTest(tests, "C463_bb2_duplicate_same_edge_bundle_rejected",
                         "bb2 duplicate same edge and bundle requests are rejected", "Boundary", false,
                         C463_bb2_duplicate_same_edge_bundle_rejected);
  test_registry::AddTest(tests, "C464_bb2_different_bundle_on_same_edge_allowed",
                         "bb2 different bundles on the same edge are allowed", "Boundary", false,
                         C464_bb2_different_bundle_on_same_edge_allowed);
  test_registry::AddTest(tests, "C465_bb2_duplicate_policy_does_not_read_existing_spans",
                         "bb2 duplicate policy reads saved edge bundles, not existing spans", "Boundary", false,
                         C465_bb2_duplicate_policy_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C466_bb2_duplicate_reject_keeps_state_unchanged",
                         "bb2 duplicate reject keeps state unchanged", "Boundary", false,
                         C466_bb2_duplicate_reject_keeps_state_unchanged);
  test_registry::AddTest(tests, "C467_bb2_saves_row_port_bindings",
                         "bb2 saves row to materialized port bindings", "Boundary", false,
                         C467_bb2_saves_row_port_bindings);
  test_registry::AddTest(tests, "C468_bb2_row_port_binding_is_stable_for_existing_context",
                         "bb2 saves new row-port bindings without emitting context rows", "Boundary", false,
                         C468_bb2_row_port_binding_is_stable_for_existing_context);
  test_registry::AddTest(tests, "C469_bb2_row_port_binding_rejects_duplicate_without_resolution",
                         "bb2 duplicate row-port binding is rejected without port resolution", "Boundary", false,
                         C469_bb2_row_port_binding_rejects_duplicate_without_resolution);
  test_registry::AddTest(tests, "C470_bb2_row_port_identity_does_not_use_position_match",
                         "bb2 row-port identity does not use position matching", "Boundary", false,
                         C470_bb2_row_port_identity_does_not_use_position_match);
  test_registry::AddTest(tests, "C471_bb2_resolves_existing_port_by_binding",
                         "bb2 resolves existing ports by saved row-port binding", "Boundary", false,
                         C471_bb2_resolves_existing_port_by_binding);
  test_registry::AddTest(tests, "C472_bb2_port_resolution_requires_saved_binding",
                         "bb2 does not resolve ports without saved binding", "Boundary", false,
                         C472_bb2_port_resolution_requires_saved_binding);
  test_registry::AddTest(tests, "C473_bb2_resolved_port_used_by_new_span_endpoint",
                         "bb2 new spans use resolved port endpoints", "Boundary", false,
                         C473_bb2_resolved_port_used_by_new_span_endpoint);
  test_registry::AddTest(tests, "C474_bb2_port_resolution_rejects_ambiguous_binding",
                         "bb2 port resolution rejects ambiguous row-port bindings", "Boundary", false,
                         C474_bb2_port_resolution_rejects_ambiguous_binding);
  test_registry::AddTest(tests, "C475_bb2_port_resolution_does_not_read_existing_layout",
                         "bb2 port resolution does not read existing layout", "Boundary", false,
                         C475_bb2_port_resolution_does_not_read_existing_layout);
  test_registry::AddTest(tests, "C476_bb2_branch_rows_are_separated_without_branch_kind",
                         "bb2 branch rows are separated without branch kind", "Boundary", false,
                         C476_bb2_branch_rows_are_separated_without_branch_kind);
  test_registry::AddTest(tests, "C477_bb2_cross_rows_are_separated_without_cross_kind",
                         "bb2 cross rows are separated without cross kind", "Boundary", false,
                         C477_bb2_cross_rows_are_separated_without_cross_kind);
  test_registry::AddTest(tests, "C478_bb2_row_separation_is_deterministic",
                         "bb2 row separation is deterministic", "Invariant", false,
                         C478_bb2_row_separation_is_deterministic);
  test_registry::AddTest(tests, "C479_bb2_row_separation_does_not_change_pairs",
                         "bb2 row separation does not change pair source", "Boundary", false,
                         C479_bb2_row_separation_does_not_change_pairs);
  test_registry::AddTest(tests, "C480_bb2_context_rows_affect_order_but_are_not_emitted",
                         "bb2 context rows affect separation order but are not emitted", "Boundary", false,
                         C480_bb2_context_rows_affect_order_but_are_not_emitted);
  test_registry::AddTest(tests, "C481_bb2_pass_through_mode_is_accepted_in_limited_scope",
                         "bb2 accepts pass-through mode for saved junction nodes", "Boundary", false,
                         C481_bb2_pass_through_mode_is_accepted_in_limited_scope);
  test_registry::AddTest(tests, "C482_bb2_pass_through_creates_explicit_intent",
                         "bb2 pass-through node mode creates explicit layout intent", "Boundary", false,
                         C482_bb2_pass_through_creates_explicit_intent);
  test_registry::AddTest(tests, "C483_bb2_pass_through_ambiguous_target_rejected",
                         "bb2 pass-through ambiguous row target is rejected", "Boundary", false,
                         C483_bb2_pass_through_ambiguous_target_rejected);
  test_registry::AddTest(tests, "C484_bb2_lowering_draw_uses_layout_only",
                         "bb2 lowering draw uses layout only", "Boundary", false,
                         C484_bb2_lowering_draw_uses_layout_only);
  test_registry::AddTest(tests, "C485_bb2_lowering_intent_does_not_read_existing_spans",
                         "bb2 lowering intent does not read existing spans", "Boundary", false,
                         C485_bb2_lowering_intent_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C486_bb2_pass_through_is_deterministic",
                         "bb2 pass-through intent is deterministic", "Invariant", false,
                         C486_bb2_pass_through_is_deterministic);
  test_registry::AddTest(tests, "C487_bb2_port_resolution_requires_bundle_compatible_scope",
                         "bb2 port resolution requires bundle-compatible scope", "Boundary", false,
                         C487_bb2_port_resolution_requires_bundle_compatible_scope);
  test_registry::AddTest(tests, "C488_bb2_port_resolution_accepts_same_compatible_binding",
                         "bb2 accepts multiple same-compatible port bindings", "Boundary", false,
                         C488_bb2_port_resolution_accepts_same_compatible_binding);
  test_registry::AddTest(tests, "C489_bb2_port_binding_index_invariant",
                         "bb2 port binding index exposes all compatible bindings", "Boundary", false,
                         C489_bb2_port_binding_index_invariant);
  test_registry::AddTest(tests, "C490_bb2_duplicate_same_edge_bundle_lane_rejected",
                         "bb2 duplicate same edge bundle lane requests do not duplicate", "Boundary", false,
                         C490_bb2_duplicate_same_edge_bundle_lane_rejected);
  test_registry::AddTest(tests, "C491_bb2_branch_lowering_v1_affects_geom",
                         "bb2 branch lowering v1 affects layout geometry", "Invariant", false,
                         C491_bb2_branch_lowering_v1_affects_geom);
  test_registry::AddTest(tests, "C492_bb2_cross_lowering_v1_affects_only_new_links",
                         "bb2 cross lowering v1 affects generated links only", "Invariant", false,
                         C492_bb2_cross_lowering_v1_affects_only_new_links);
  test_registry::AddTest(tests, "C493_bb2_pass_through_does_not_change_pair_open",
                         "bb2 pass-through does not change pair open authority", "Boundary", false,
                         C493_bb2_pass_through_does_not_change_pair_open);
  test_registry::AddTest(tests, "C494_bb2_lowering_v1_draw_does_not_redecide",
                         "bb2 lowering v1 draw does not redecide", "Boundary", false,
                         C494_bb2_lowering_v1_draw_does_not_redecide);
  test_registry::AddTest(tests, "C495_bb2_lowering_v1_does_not_read_existing_spans",
                         "bb2 lowering v1 does not read existing spans", "Boundary", false,
                         C495_bb2_lowering_v1_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C496_bb2_junction_v1_deterministic",
                         "bb2 junction v1 output is deterministic", "Invariant", false,
                         C496_bb2_junction_v1_deterministic);
  test_registry::AddTest(tests, "C497_bb2_context_rows_order_but_do_not_materialize",
                         "bb2 context rows order placement without materializing", "Boundary", false,
                         C497_bb2_context_rows_order_but_do_not_materialize);
  test_registry::AddTest(tests, "C498_bb2_saved_graph_remains_topology_authority",
                         "bb2 saved graph remains topology authority", "Boundary", false,
                         C498_bb2_saved_graph_remains_topology_authority);
  test_registry::AddTest(tests, "C499_bb2_context_link_is_not_saved",
                         "bb2 context links are not saved as new edges", "Boundary", false,
                         C499_bb2_context_link_is_not_saved);
  test_registry::AddTest(tests, "C500_bb2_context_link_requires_saved_edge_ref",
                         "bb2 context links require saved edge refs", "Boundary", false,
                         C500_bb2_context_link_requires_saved_edge_ref);
  test_registry::AddTest(tests, "C501_bb2_gate3_contract_passes",
                         "bb2 save graph keeps context links out of save targets", "Boundary", false,
                         C501_bb2_gate3_contract_passes);
  test_registry::AddTest(tests, "C502_bb2_span_bindings_save_lane",
                         "bb2 saved span bindings carry lane identity", "Boundary", false,
                         C502_bb2_span_bindings_save_lane);
  test_registry::AddTest(tests, "C503_bb2_duplicate_span_binding_rejected_by_lane",
                         "bb2 rejects duplicate saved span binding lanes", "Boundary", false,
                         C503_bb2_duplicate_span_binding_rejected_by_lane);
  test_registry::AddTest(tests, "C504_bb2_span_resolution_does_not_read_geometry_or_layout",
                         "bb2 span resolution reads saved bindings only", "Boundary", false,
                         C504_bb2_span_resolution_does_not_read_geometry_or_layout);
  test_registry::AddTest(tests, "C505_bb2_save_graph_propagates_span_binding_failure",
                         "bb2 save graph propagates span binding failures", "Boundary", false,
                         C505_bb2_save_graph_propagates_span_binding_failure);
  test_registry::AddTest(tests, "C506_bb2_support_group_is_placement_layer",
                         "bb2 support group is a placement layer", "Boundary", false,
                         C506_bb2_support_group_is_placement_layer);
  test_registry::AddTest(tests, "C507_bb2_support_group_built_after_intent",
                         "bb2 support group is built after intent", "Boundary", false,
                         C507_bb2_support_group_built_after_intent);
  test_registry::AddTest(tests, "C508_bb2_support_group_drives_lowered_rules",
                         "bb2 support group drives lowered rules", "Boundary", false,
                         C508_bb2_support_group_drives_lowered_rules);
  test_registry::AddTest(tests, "C509_bb2_support_group_avoids_visual_terms",
                         "bb2 support group avoids visual terms", "Boundary", false,
                         C509_bb2_support_group_avoids_visual_terms);
  test_registry::AddTest(tests, "C510_bb2_layout_consumes_group_offset",
                         "bb2 layout consumes support group offset", "Boundary", false,
                         C510_bb2_layout_consumes_group_offset);
  test_registry::AddTest(tests, "C511_bb2_draw_saved_from_geom",
                         "bb2 draw is saved from geom", "Boundary", false,
                         C511_bb2_draw_saved_from_geom);
  test_registry::AddTest(tests, "C512_bb2_draw_does_not_read_topology",
                         "bb2 draw does not read topology", "Boundary", false,
                         C512_bb2_draw_does_not_read_topology);
  test_registry::AddTest(tests, "C513_bb2_support_visual_placeholder_from_layout",
                         "bb2 support visual placeholder comes from layout", "Boundary", false,
                         C513_bb2_support_visual_placeholder_from_layout);
  test_registry::AddTest(tests, "C514_bb2_draw_save_is_direct",
                         "bb2 draw save is direct", "Boundary", false,
                         C514_bb2_draw_save_is_direct);
  test_registry::AddTest(tests, "C515_bb2_rejects_existing_pole_without_saved_graph",
                         "bb2 rejects existing poles without saved graph", "Boundary", true,
                         C515_bb2_rejects_existing_pole_without_saved_graph);
  test_registry::AddTest(tests, "C516_bb2_generated_pole_with_saved_graph_still_connects",
                         "bb2 generated poles with saved graph still connect", "Boundary", false,
                         C516_bb2_generated_pole_with_saved_graph_still_connects);
  test_registry::AddTest(tests, "C517_bb2_migration_gate_does_not_infer_from_outputs",
                         "bb2 migration gate does not infer from outputs", "Boundary", false,
                         C517_bb2_migration_gate_does_not_infer_from_outputs);
  test_registry::AddTest(tests, "C518_bb2_lowered_layout_keeps_support_world_at_port_height",
                         "bb2 lowered layout keeps support world at port height", "Boundary", false,
                         C518_bb2_lowered_layout_keeps_support_world_at_port_height);
  test_registry::AddTest(tests, "C519_bb2_draw_placeholder_uses_layout_points",
                         "bb2 draw placeholder uses layout points", "Boundary", false,
                         C519_bb2_draw_placeholder_uses_layout_points);
  test_registry::AddTest(tests, "C520_bb2_duplicate_span_binding_preflight_before_emit",
                         "bb2 duplicate span binding preflight runs before emit", "Boundary", false,
                         C520_bb2_duplicate_span_binding_preflight_before_emit);
  test_registry::AddTest(tests, "C521_bb2_context_link_preserves_saved_dir",
                         "bb2 context link preserves saved direction", "Boundary", false,
                         C521_bb2_context_link_preserves_saved_dir);
  test_registry::AddTest(tests, "C522_bb2_supported_scope_is_documented",
                         "bb2 supported generation scope is documented", "Boundary", false,
                         C522_bb2_supported_scope_is_documented);
  test_registry::AddTest(tests, "C523_bb2_scope_gate_matches_entrypoint",
                         "bb2 scope gate matches the entrypoint", "Boundary", false,
                         C523_bb2_scope_gate_matches_entrypoint);
  test_registry::AddTest(tests, "C524_bb2_scenario_simple_line_mainline",
                         "bb2 simple line scenario covers outputs and authority", "Boundary", false,
                         C524_bb2_scenario_simple_line_mainline);
  test_registry::AddTest(tests, "C525_bb2_scenario_polyline3_connectivity_once",
                         "bb2 polyline scenario keeps connectivity authority single", "Boundary", false,
                         C525_bb2_scenario_polyline3_connectivity_once);
  test_registry::AddTest(tests, "C526_bb2_scenario_multiple_bundles_share_connectivity",
                         "bb2 multiple-bundle scenario shares connectivity", "Boundary", false,
                         C526_bb2_scenario_multiple_bundles_share_connectivity);
  test_registry::AddTest(tests, "C527_bb2_scenario_existing_pole_continuation_uses_saved_graph",
                         "bb2 existing-pole continuation scenario uses saved graph", "Boundary", false,
                         C527_bb2_scenario_existing_pole_continuation_uses_saved_graph);
  test_registry::AddTest(tests, "C528_bb2_scenario_branch_emits_new_link_only",
                         "bb2 branch scenario emits only the new link", "Boundary", false,
                         C528_bb2_scenario_branch_emits_new_link_only);
  test_registry::AddTest(tests, "C529_bb2_scenario_cross_without_kind_label",
                         "bb2 cross scenario runs without kind labels", "Boundary", false,
                         C529_bb2_scenario_cross_without_kind_label);
  test_registry::AddTest(tests, "C530_bb2_scenario_same_edge_different_bundle",
                         "bb2 same-edge different-bundle scenario shares saved edge", "Boundary", false,
                         C530_bb2_scenario_same_edge_different_bundle);
  test_registry::AddTest(tests, "C531_bb2_scenario_duplicate_reject_unchanged",
                         "bb2 duplicate scenario rejects without mutation", "Boundary", false,
                         C531_bb2_scenario_duplicate_reject_unchanged);
  test_registry::AddTest(tests, "C532_bb2_scenario_pass_through_lowering_consumer_chain",
                         "bb2 pass-through lowering scenario preserves consumer chain", "Boundary", false,
                         C532_bb2_scenario_pass_through_lowering_consumer_chain);
  test_registry::AddTest(tests, "C533_bb2_build_mutation_order_is_fixed",
                         "bb2 build mutation order is fixed", "Boundary", false,
                         C533_bb2_build_mutation_order_is_fixed);
  test_registry::AddTest(tests, "C534_bb2_invalid_inputs_stop_before_emit",
                         "bb2 invalid inputs stop before emit", "Boundary", true,
                         C534_bb2_invalid_inputs_stop_before_emit);
  test_registry::AddTest(tests, "C535_bb2_duplicate_preflight_is_mutation_boundary",
                         "bb2 duplicate preflight is the mutation boundary", "Boundary", false,
                         C535_bb2_duplicate_preflight_is_mutation_boundary);
  test_registry::AddTest(tests, "C536_bb2_draw_consumer_outputs_are_minimal",
                         "bb2 draw consumer outputs are minimal", "Boundary", false,
                         C536_bb2_draw_consumer_outputs_are_minimal);
  test_registry::AddTest(tests, "C537_bb2_draw_source_has_no_decision_inputs",
                         "bb2 draw source has no decision inputs", "Boundary", false,
                         C537_bb2_draw_source_has_no_decision_inputs);
  test_registry::AddTest(tests, "C538_bb2_viewer_deps_are_not_core_draw_gate",
                         "bb2 viewer deps are not the core draw gate", "Boundary", false,
                         C538_bb2_viewer_deps_are_not_core_draw_gate);
  test_registry::AddTest(tests, "C539_bb2_supported_request_creates_saved_graph_outputs",
                         "bb2 supported request creates saved graph outputs", "Boundary", false,
                         C539_bb2_supported_request_creates_saved_graph_outputs);
  test_registry::AddTest(tests, "C540_bb2_unsupported_request_does_not_create_v1_outputs",
                         "bb2 unsupported request does not create v1 outputs", "Boundary", true,
                         C540_bb2_unsupported_request_does_not_create_v1_outputs);
  test_registry::AddTest(tests, "C541_bb2_manual_existing_pole_without_graph_is_gate_rejected",
                         "bb2 manual existing pole without graph is gate rejected", "Boundary", true,
                         C541_bb2_manual_existing_pole_without_graph_is_gate_rejected);
  test_registry::AddTest(tests, "C542_bb2_usable_mainline_architecture_audit_passes",
                         "bb2 usable mainline architecture audit passes", "Boundary", false,
                         C542_bb2_usable_mainline_architecture_audit_passes);
  test_registry::AddTest(tests, "C543_bb2_new_route_interior_pass_through_supported",
                         "bb2 supports pass-through on a new route interior point", "Boundary", false,
                         C543_bb2_new_route_interior_pass_through_supported);
  test_registry::AddTest(tests, "C544_bb2_pole_placement_pins_generated_poles",
                         "bb2 applies pole placement pin options to generated poles", "Boundary", false,
                         C544_bb2_pole_placement_pins_generated_poles);
  test_registry::AddTest(tests, "C545_bb2_interval_generates_intermediate_poles",
                         "bb2 interval generates intermediate poles", "Boundary", false,
                         C545_bb2_interval_generates_intermediate_poles);
  test_registry::AddTest(tests, "C546_bb2_explicit_new_pole_node_spec_supported",
                         "bb2 supports explicit new-pole node specs", "Boundary", false,
                         C546_bb2_explicit_new_pole_node_spec_supported);
  test_registry::AddTest(tests, "C547_bb2_fixed_bundle_exact_count_is_supported",
                         "bb2 supports exact fixed bundle count input", "Boundary", false,
                         C547_bb2_fixed_bundle_exact_count_is_supported);
  test_registry::AddTest(tests, "C548_bb2_avoid_radius_without_points_is_noop",
                         "bb2 accepts avoid radius without avoid points as no-op", "Boundary", false,
                         C548_bb2_avoid_radius_without_points_is_noop);
  test_registry::AddTest(tests, "C549_bb2_range_bundle_explicit_count_is_supported",
                         "bb2 supports explicit range bundle count", "Boundary", false,
                         C549_bb2_range_bundle_explicit_count_is_supported);
  test_registry::AddTest(tests, "C550_bb2_generated_pole_uses_tangent_hint_yaw",
                         "bb2 uses tangent hints for generated pole yaw", "Boundary", false,
                         C550_bb2_generated_pole_uses_tangent_hint_yaw);
  test_registry::AddTest(tests, "C551_bb2_missing_pole_type_resolves_from_bundle_templates",
                         "bb2 resolves missing pole type from bundle templates", "Boundary", false,
                         C551_bb2_missing_pole_type_resolves_from_bundle_templates);
  test_registry::AddTest(tests, "C552_bb2_zero_radius_avoid_points_are_noop",
                         "bb2 accepts zero-radius avoid points as no-op", "Boundary", false,
                         C552_bb2_zero_radius_avoid_points_are_noop);
  test_registry::AddTest(tests, "C553_bb2_new_midair_route_point_is_supported",
                         "bb2 supports new midair route points", "Boundary", false,
                         C553_bb2_new_midair_route_point_is_supported);
  test_registry::AddTest(tests, "C554_bb2_existing_midair_route_point_is_supported",
                         "bb2 supports existing saved midair route points", "Boundary", false,
                         C554_bb2_existing_midair_route_point_is_supported);
  test_registry::AddTest(tests, "C555_bb2_new_building_route_point_is_supported",
                         "bb2 supports new building route points", "Boundary", false,
                         C555_bb2_new_building_route_point_is_supported);
  test_registry::AddTest(tests, "C556_bb2_building_pick_feeds_new_building_route_point",
                         "bb2 supports building picks as new building route points", "Boundary", false,
                         C556_bb2_building_pick_feeds_new_building_route_point);
  test_registry::AddTest(tests, "C557_bb2_building_pick_without_id_is_supported",
                         "bb2 supports building picks without object ids", "Boundary", false,
                         C557_bb2_building_pick_without_id_is_supported);
  test_registry::AddTest(tests, "C558_bb2_ground_pick_feeds_new_ground_route_point",
                         "bb2 supports ground picks as new ground route points", "Boundary", false,
                         C558_bb2_ground_pick_feeds_new_ground_route_point);
  test_registry::AddTest(tests, "C559_bb2_positive_avoid_clear_of_route_is_noop",
                         "bb2 accepts positive avoid constraints when the route is clear", "Boundary", false,
                         C559_bb2_positive_avoid_clear_of_route_is_noop);
  test_registry::AddTest(tests, "C560_bb2_segment_pick_without_bundle_policy_feeds_midair_route_point",
                         "bb2 accepts a dry-run segment pick without selected bundle policy as a midair route point",
                         "Boundary", false, C560_bb2_segment_pick_without_bundle_policy_feeds_midair_route_point);
  test_registry::AddTest(tests, "C561_bb2_default_segment_pick_without_bundle_policy_is_ownerless_midair",
                         "bb2 accepts the default segment pick without selected bundle policy as an ownerless midair point",
                         "Boundary", false, C561_bb2_default_segment_pick_without_bundle_policy_is_ownerless_midair);
  test_registry::AddTest(tests, "C562_bb2_saved_midair_node_pick_extends_from_saved_graph_node",
                         "bb2 extends from a saved midair node pick using the saved graph node", "Boundary", false,
                         C562_bb2_saved_midair_node_pick_extends_from_saved_graph_node);
  test_registry::AddTest(tests, "C563_bb2_segment_pick_snaps_to_saved_ownerless_span_endpoint",
                         "bb2 resolves a segment pick endpoint through saved graph ownerless span endpoints",
                         "Boundary", false, C563_bb2_segment_pick_snaps_to_saved_ownerless_span_endpoint);
  test_registry::AddTest(tests, "C564_bb2_selected_bundle_segment_pick_feeds_transient_midair_node",
                         "bb2 accepts a selected-bundle segment pick transient midair node as route input",
                         "Boundary", false, C564_bb2_selected_bundle_segment_pick_feeds_transient_midair_node);
  test_registry::AddTest(tests, "C565_bb2_mixed_selected_midair_branch_generates_allowed_bundles_only",
                         "bb2 materializes only allowed bundles for mixed selected midair branch picks",
                         "Boundary", false, C565_bb2_mixed_selected_midair_branch_generates_allowed_bundles_only);
  test_registry::AddTest(tests, "C566_bb2_disallowed_selected_midair_branch_is_noop",
                         "bb2 treats fully disallowed selected midair branch bundles as no-op",
                         "Boundary", false, C566_bb2_disallowed_selected_midair_branch_is_noop);
  test_registry::AddTest(tests, "C567_bb2_segment_pick_midair_uses_source_span_height",
                         "bb2 segment-pick midair branches use source span height", "Boundary", false,
                         C567_bb2_segment_pick_midair_uses_source_span_height);
  test_registry::AddTest(tests, "C568_bb2_source_edge_midair_branch_uses_source_context_for_lowering",
                         "bb2 source-edge midair branches use saved edge context for lowering", "Boundary", false,
                         C568_bb2_source_edge_midair_branch_uses_source_context_for_lowering);
  test_registry::AddTest(tests, "C569_bb2_render_uses_cable_template_appearance",
                         "bb2 render cache uses cable template appearance", "Boundary", false,
                         C569_bb2_render_uses_cable_template_appearance);
  test_registry::AddTest(tests, "C570_bb2_support_visual_uses_visual_settings_radius",
                         "bb2 support visual placeholder uses visual settings radius", "Boundary", false,
                         C570_bb2_support_visual_uses_visual_settings_radius);
  test_registry::AddTest(tests, "C571_bb2_support_visual_respects_enable_setting",
                         "bb2 support visual placeholder respects enable_support_structures", "Boundary", false,
                         C571_bb2_support_visual_respects_enable_setting);
  test_registry::AddTest(tests, "C572_bb2_support_visual_radius_setting_is_mutable",
                         "bb2 support visual placeholder uses updated support arm radius", "Boundary", false,
                         C572_bb2_support_visual_radius_setting_is_mutable);
  test_registry::AddTest(tests, "C573_bb2_saved_context_node_carries_support_metadata",
                         "bb2 saved context nodes carry support metadata", "Boundary", false,
                         C573_bb2_saved_context_node_carries_support_metadata);
  test_registry::AddTest(tests, "C574_bb2_same_edge_different_bundle_with_pass_through_is_supported",
                         "bb2 supports adding a different bundle on an existing edge with pass-through mode", "Boundary",
                         false, C574_bb2_same_edge_different_bundle_with_pass_through_is_supported);
  test_registry::AddTest(tests, "C575_bb2_stale_segment_pick_midair_duplicate_rejected_unchanged",
                         "bb2 rejects duplicate stale segment-pick midair branch requests before mutation", "Boundary",
                         false, C575_bb2_stale_segment_pick_midair_duplicate_rejected_unchanged);
  test_registry::AddTest(tests, "C576_bb2_ownerless_multiple_bundles_do_not_require_pole_type",
                         "bb2 ownerless-only multiple bundle routes do not require a pole type", "Boundary", false,
                         C576_bb2_ownerless_multiple_bundles_do_not_require_pole_type);
  test_registry::AddTest(tests, "C577_bb2_missing_port_band_rejects_before_mutation",
                         "bb2 rejects missing port bands before topology mutation", "Boundary", false,
                         C577_bb2_missing_port_band_rejects_before_mutation);
  test_registry::AddTest(tests, "C578_bb2_segment_pick_midair_pass_through_supported",
                         "bb2 supports pass-through on a segment-pick midair branch", "Boundary", false,
                         C578_bb2_segment_pick_midair_pass_through_supported);
  test_registry::AddTest(tests, "C579_bb2_polyline_avoid_detour_supported",
                         "bb2 supports a single avoid detour on one segment of a polyline route", "Boundary", false,
                         C579_bb2_polyline_avoid_detour_supported);
  test_registry::AddTest(tests, "C580_bb2_interval_avoid_combination_orders_inserted_points",
                         "bb2 orders interval and avoid inserted points on the source segment", "Boundary", false,
                         C580_bb2_interval_avoid_combination_orders_inserted_points);
  test_registry::AddTest(tests, "C581_bb2_inactive_bundle_missing_band_is_ignored",
                         "bb2 ignores missing port bands for inactive bundles", "Boundary", false,
                         C581_bb2_inactive_bundle_missing_band_is_ignored);
  test_registry::AddTest(tests, "C582_bb2_multiple_avoid_points_on_one_segment_supported",
                         "bb2 supports multiple avoid points on one route segment", "Boundary", false,
                         C582_bb2_multiple_avoid_points_on_one_segment_supported);
  test_registry::AddTest(tests, "C583_bb2_avoid_points_on_multiple_segments_supported",
                         "bb2 supports avoid points on multiple route segments", "Boundary", false,
                         C583_bb2_avoid_points_on_multiple_segments_supported);
  test_registry::AddTest(tests, "C584_bb2_ownerless_interval_inserts_ownerless_nodes",
                         "bb2 interval insertion inherits ownerless support", "Boundary", false,
                         C584_bb2_ownerless_interval_inserts_ownerless_nodes);
  test_registry::AddTest(tests, "C585_bb2_duplicate_avoid_points_are_coalesced",
                         "bb2 coalesces duplicate avoid detour points", "Boundary", false,
                         C585_bb2_duplicate_avoid_points_are_coalesced);
  test_registry::AddTest(tests, "C594_bb2_avoid_point_at_route_endpoint_is_noop",
                         "bb2 treats an avoid point exactly at a route endpoint as no-op", "Boundary", false,
                         C594_bb2_avoid_point_at_route_endpoint_is_noop);
  test_registry::AddTest(tests, "C586_bb2_avoid_detour_replaces_interval_at_same_t",
                         "bb2 avoid detour replaces interval insert at the same route position", "Boundary", false,
                         C586_bb2_avoid_detour_replaces_interval_at_same_t);
  test_registry::AddTest(tests, "C587_bb2_create_midair_node_without_selected_bundle_supported",
                         "bb2 creates explicit segment-pick midair nodes without selected bundle policy", "Boundary",
                         false, C587_bb2_create_midair_node_without_selected_bundle_supported);
  test_registry::AddTest(tests, "C588_bb2_corner_avoid_detour_supported",
                         "bb2 supports avoid detours at an internal route corner", "Boundary", false,
                         C588_bb2_corner_avoid_detour_supported);
  test_registry::AddTest(tests, "C589_bb2_selected_bundle_policy_blocks_unselected_bundle",
                         "bb2 selected-bundle midair policy blocks unselected request bundles", "Boundary", false,
                         C589_bb2_selected_bundle_policy_blocks_unselected_bundle);
  test_registry::AddTest(tests, "C590_bb2_inactive_pass_through_bundle_rejected_before_noop",
                         "bb2 rejects pass-through modes for bundles made inactive by selected policy", "Boundary",
                         false, C590_bb2_inactive_pass_through_bundle_rejected_before_noop);
  test_registry::AddTest(tests, "C591_bb2_saved_selected_midair_continuation_keeps_bundle_policy",
                         "bb2 saved selected midair continuation keeps bundle policy", "Boundary", false,
                         C591_bb2_saved_selected_midair_continuation_keeps_bundle_policy);
  test_registry::AddTest(tests, "C592_bb2_saved_selected_midair_reverse_continuation_keeps_bundle_policy",
                         "bb2 saved selected midair reverse continuation keeps bundle policy", "Boundary", false,
                         C592_bb2_saved_selected_midair_reverse_continuation_keeps_bundle_policy);
  test_registry::AddTest(tests, "C593_bb2_saved_selected_midair_rejects_inactive_pass_through",
                         "bb2 saved selected midair rejects inactive pass-through", "Boundary", false,
                         C593_bb2_saved_selected_midair_rejects_inactive_pass_through);
  test_registry::AddTest(tests, "C595_bb2_avoid_point_at_explicit_existing_support_is_noop",
                         "bb2 treats an avoid point exactly at an explicit existing support as no-op", "Boundary",
                         false, C595_bb2_avoid_point_at_explicit_existing_support_is_noop);
  test_registry::AddTest(tests, "C596_bb2_avoid_point_at_explicit_new_support_is_noop",
                         "bb2 treats an avoid point exactly at an explicit new support as no-op", "Boundary", false,
                         C596_bb2_avoid_point_at_explicit_new_support_is_noop);
  test_registry::AddTest(tests, "C597_bb2_selected_building_pick_generates_selected_bundle_only",
                         "bb2 selected building picks generate selected bundles only", "Boundary", false,
                         C597_bb2_selected_building_pick_generates_selected_bundle_only);
  test_registry::AddTest(tests, "C598_bb2_selected_saved_building_node_pick_generates_selected_bundle_only",
                         "bb2 selected saved building node picks generate selected bundles only", "Boundary", false,
                         C598_bb2_selected_saved_building_node_pick_generates_selected_bundle_only);
  test_registry::AddTest(tests, "C600_bb2_selected_existing_pole_pick_generates_selected_bundle_only",
                         "bb2 selected existing pole pick generates selected bundle only", "Boundary", false,
                         C600_bb2_selected_existing_pole_pick_generates_selected_bundle_only);
  test_registry::AddTest(tests, "C601_bb2_context_only_bundle_policy_does_not_filter_new_route",
                         "bb2 context-only selected bundle policy does not filter new route bundles",
                         "Boundary", false, C601_bb2_context_only_bundle_policy_does_not_filter_new_route);
  test_registry::AddTest(tests, "C602_bb2_context_only_pole_band_does_not_filter_new_route",
                         "bb2 context-only pole bands do not filter new route bundles",
                         "Boundary", false, C602_bb2_context_only_pole_band_does_not_filter_new_route);
  test_registry::AddTest(tests, "C603_bb2_context_node_does_not_affect_generated_endpoint_yaw",
                         "bb2 context nodes do not affect generated endpoint pole yaw",
                         "Boundary", false, C603_bb2_context_node_does_not_affect_generated_endpoint_yaw);
  test_registry::AddTest(tests, "C604_bb2_large_avoid_detour_clears_radius",
                         "bb2 large avoid detours clear the requested radius", "Boundary", false,
                         C604_bb2_large_avoid_detour_clears_radius);
  test_registry::AddTest(tests, "C605_bb2_find_backbone_route_uses_saved_ownerless_graph",
                         "bb2 route queries use saved ownerless backbone graph", "Boundary", false,
                         C605_bb2_find_backbone_route_uses_saved_ownerless_graph);
  test_registry::AddTest(tests, "C608_bb2_saved_backbone_result_does_not_duplicate_saved_pole_nodes",
                         "bb2 SavedBackboneResult does not duplicate saved pole nodes", "Boundary", false,
                         C608_bb2_saved_backbone_result_does_not_duplicate_saved_pole_nodes);
  test_registry::AddTest(tests, "C609_bb2_acute_corner_lowers_layout_geom_without_port_lowering",
                         "bb2 acute corners lower layout and geom without lowering ports", "Boundary", false,
                         C609_bb2_acute_corner_lowers_layout_geom_without_port_lowering);
  test_registry::AddTest(tests, "C610_bb2_acute_corner_lowering_survives_pole_yaw_override",
                         "bb2 acute corner lowering survives pole yaw override", "Boundary", false,
                         C610_bb2_acute_corner_lowering_survives_pole_yaw_override);
  test_registry::AddTest(tests, "C607_bb2_saved_backbone_result_preserves_saved_ownerless_route_index",
                         "bb2 SavedBackboneResult preserves saved ownerless route index", "Boundary", false,
                         C607_bb2_saved_backbone_result_preserves_saved_ownerless_route_index);
  test_registry::AddTest(tests, "C606_bb2_saved_backbone_result_exposes_saved_ownerless_node",
                         "bb2 SavedBackboneResult exposes saved ownerless nodes", "Boundary", false,
                         C606_bb2_saved_backbone_result_exposes_saved_ownerless_node);
  test_registry::AddTest(tests, "C599_bb2_selected_saved_building_node_policy_persists_after_branch",
                         "bb2 selected saved building node policy persists after branch", "Boundary", false,
                         C599_bb2_selected_saved_building_node_policy_persists_after_branch);
  test_registry::AddTest(tests, "C611_bb2_direct_derive_restores_saved_span_outputs",
                         "bb2 direct derive restores saved span outputs without recalc", "Boundary", false,
                         C611_bb2_direct_derive_restores_saved_span_outputs);
  test_registry::AddTest(tests, "C612_bb2_direct_derive_does_not_call_recalc_paths",
                         "bb2 direct derive avoids recalc and materialization paths", "Boundary", false,
                         C612_bb2_direct_derive_does_not_call_recalc_paths);
  test_registry::AddTest(tests, "C613_bb2_port_edit_rederives_generated_span_without_recalc",
                         "bb2 port edits rederive generated span outputs without recalc", "Boundary", false,
                         C613_bb2_port_edit_rederives_generated_span_without_recalc);
  test_registry::AddTest(tests, "C614_bb2_update_plan_uses_coarse_kinds",
                         "bb2 update planning uses the four coarse update kinds", "Boundary", false,
                         C614_bb2_update_plan_uses_coarse_kinds);
  test_registry::AddTest(tests, "C615_bb2_regenerate_plan_is_not_local_fallback",
                         "bb2 regenerate update plans do not run a local fallback", "Boundary", true,
                         C615_bb2_regenerate_plan_is_not_local_fallback);
  test_registry::AddTest(tests, "C616_bb2_reposition_keeps_saved_graph_identity",
                         "bb2 reposition updates keep saved graph identity", "Boundary", false,
                         C616_bb2_reposition_keeps_saved_graph_identity);
  test_registry::AddTest(tests, "C617_bb2_reshape_does_not_rewrite_layout",
                         "bb2 reshape updates do not rewrite layout", "Boundary", false,
                         C617_bb2_reshape_does_not_rewrite_layout);
  test_registry::AddTest(tests, "C618_bb2_redraw_does_not_rewrite_layout_or_geom",
                         "bb2 redraw updates do not rewrite layout or geom", "Boundary", false,
                         C618_bb2_redraw_does_not_rewrite_layout_or_geom);
  test_registry::AddTest(tests, "C619_bb2_reposition_updates_only_affected_spans",
                         "bb2 reposition updates only affected spans", "Boundary", false,
                         C619_bb2_reposition_updates_only_affected_spans);
  test_registry::AddTest(tests, "C620_bb2_update_boundary_has_no_operation_specific_kinds",
                         "bb2 update boundary has no operation-specific update kinds", "Boundary", false,
                         C620_bb2_update_boundary_has_no_operation_specific_kinds);
  test_registry::AddTest(tests, "C621_bb2_sag_reshape_updates_geom_only",
                         "bb2 sag reshapes curve and bounds without changing layout or topology", "Boundary", false,
                         C621_bb2_sag_reshape_updates_geom_only);
  test_registry::AddTest(tests, "C622_bb2_stage_timing_is_diagnostic_only",
                         "bb2 reports generation and update stage timing without changing decisions", "Boundary", false,
                         C622_bb2_stage_timing_is_diagnostic_only);
  test_registry::AddTest(tests, "C623_bb2_layout_settings_reject_before_mutation",
                         "bb2 layout settings reject before mutation when regeneration is required", "Boundary", true,
                         C623_bb2_layout_settings_reject_before_mutation);
  test_registry::AddTest(tests, "C624_bb2_variation_settings_reject_before_mutation",
                         "bb2 variation settings reject before mutation while unsupported", "Boundary", true,
                         C624_bb2_variation_settings_reject_before_mutation);
  test_registry::AddTest(tests, "C625_bb2_context_profile_reject_before_mutation",
                         "bb2 context profile rejects before mutation while unsupported", "Boundary", true,
                         C625_bb2_context_profile_reject_before_mutation);
  test_registry::AddTest(tests, "C626_bb2_cable_template_updates_derive_outputs",
                         "bb2 cable shape and render updates directly derive outputs", "Boundary", false,
                         C626_bb2_cable_template_updates_derive_outputs);
  test_registry::AddTest(tests, "C627_bb2_legacy_topology_apis_reject_before_mutation",
                         "retired topology APIs reject before mutating SavedBackboneGraph outputs", "Boundary", true,
                         C627_bb2_legacy_topology_apis_reject_before_mutation);
}


WIRE_REGISTER_TEST_SUITE(register_tests);

} // namespace backbone_tests
