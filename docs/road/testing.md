# Road verification

Road verification follows the repository-wide policy in [`../testing.md`](../testing.md). This document owns only
Road-specific verification machinery.

Production edit/load boundaries and representative test observation points use the production
`ValidateGraphInvariants`; tests do not define a competing invariant. `tools/road_arch_lint.py` owns stable Road
source-architecture guards, while `road_architecture_contract_tests` remains for runtime invariant, metamorphic,
differential, and scenario evidence.
