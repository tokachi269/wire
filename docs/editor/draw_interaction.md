# Draw interaction

Road and wire drawing use the same event contract. Domain geometry, snapping,
validation, and commit requests remain owned by each domain.

## State

A draw session is in one of three transient states:

- `idle`: no start anchor exists.
- `anchored`: a start anchor exists, but there is no candidate interval.
- `guide`: the current pointer input defines a candidate interval. The guide does
  not claim that full domain generation will succeed.

Committed domain entities are not draw-session state. A preview, guide, pointer
gesture, or start anchor is never persisted.

## Click

The first primary action establishes the start anchor. Once an anchor exists, a
primary action submits the currently displayed guide to the domain operation.
Full generation and validation run only at this commit boundary. A successful
commit makes its endpoint the next start anchor and starts previewing the next
interval.

The committed request and guide share the same input request. The guide does not
run the full operation in advance and its presence never gates a primary action.

Every primary Click records and returns one of `anchor-accepted`,
`commit-succeeded`, `commit-rejected`, `operation-applied`, or an explicit
`ignored` reason. A plain early return is not an interaction result.

## Enter

When a guide exists, Enter submits that interval through the same path as Click
and ends the draw session only after success. With no guide, Enter ends the
session. A failed commit preserves the guide and reports the commit failure.
Enter also records its explicit action result; an inactive session is reported
as `ignored/session-inactive` rather than disappearing silently.

## Escape

Escape discards the uncommitted preview, anchor, and pointer gesture, then ends
the draw session. It does not delete or undo committed domain operations.
Escape with no active session reports \ignored/session-inactive\; it does not
claim to have ended a session that never started.

## Undo

Ctrl+Z is the operation for undoing the last committed interval. Escape is not
an undo shortcut.

## Tool switch

Changing tools discards only transient draw-session state. It does not commit a
preview and does not remove previously committed entities.

## Pointer move

Pointer movement after anchoring produces a lightweight guide or no candidate.
It must not execute full Core generation, classify a future commit failure, or
publish a user-facing commit error. The guide is replaced by the current input;
an older candidate must not remain visible as though it were current.

Road guide shows the input path and width envelope needed to understand the
candidate. Wire guide shows the input route and snap marker. Domain geometry,
support placement, junction generation, and model assembly are commit work.
Add Lane selects `2車線から3車線への変化開始位置`, `3車線が完成する位置`, and
`3車線を維持する終点` in that order. Pointer movement updates only the local
position and affected-range guide. The editor converts the selected road points
to segment-local `t`; the section plan and transition validation run only on
Confirm. A rejected confirmation preserves all three selections.

Bezier road handles depend on the start anchor, inherited start tangent, end
point, and chord length. The previous pointer frame is not an input, so two
pointer paths ending at the same point produce the same guide and commit shape.

## Atomicity

One committed interval is one atomic domain operation. Road commits its segment,
nodes, corridor membership, connection, surface, and marking together. Wire
commits its route topology, support binding, and generated wire geometry
together. Guide and topology are not separate commit phases.

## Build identity

The Web bundle and WASM module embed the same Git commit and package version.
Startup compares both values before loading the workspace or mounting the scene.
A mismatch blocks editing and reports both identities; it is not treated as a
domain commit failure.

Text controls own Enter and Escape while focused. IME composition does not send
draw commands. Keyboard input is dispatched to exactly one active draw domain.
