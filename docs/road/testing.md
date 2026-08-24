# Road verification

Road verification follows the repository-wide policy in [`../testing.md`](../testing.md). This document owns only
Road-specific verification machinery.

Production edit/load boundaries and representative test observation points use the production
`ValidateGraphInvariants`; tests do not define a competing invariant. `tools/road_arch_lint.py` owns stable Road
source-architecture guards, while `road_architecture_contract_tests` remains for runtime invariant, metamorphic,
differential, and scenario evidence.

Representative catalog examples are `lane_endpoint_identity_ignores_template_order` for stable identity under
reordering, `add_lane_leaves_unrelated_corridors_bit_identical` for non-interference, and
`seeded_operation_sequences_preserve_contracts` for stateful sequence invariants. These names are navigation aids,
not portable policy or required metadata.
