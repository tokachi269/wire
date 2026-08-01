# Commit failure categories

Road and Wire report a failed explicit commit with a machine-readable category and reason code. Pointer movement and lightweight guides do not create commit failures.

| Category | Meaning | User recovery |
|---|---|---|
| `RequirementConstraint` | A documented product rule intentionally rejects the operation. | Change the input as described by the rule. |
| `InvalidInput` | The request is malformed, incomplete, non-finite, or references an invalid required ID. | Correct the input or select a valid target. |
| `NotImplemented` | The request is meaningful, but the current resolver does not support that case. | Use a supported topology or wait for implementation. |
| `StateConflict` | The request was valid when drawing began, but its referenced state is now stale. | Refresh the anchor and retry. |
| `InternalError` | A supported path violated an internal invariant or failed unexpectedly. | Preserve the reason code and build ID for investigation. |

`RequirementConstraint` is limited to rules already documented by the owning domain. Current examples are a self-intersecting Road input path and a Wire template whose `allow_midair_branch` policy is disabled. Existing Road angle limits remain `NotImplemented` unless a product requirement is separately defined.

Every failed boundary result carries a non-empty `reasonCode`. Generic legacy failures use the category code until their operation-specific audit assigns a narrower code. Human-readable `error` text is diagnostic detail and is not used as UI control flow.
