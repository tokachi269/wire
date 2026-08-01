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

## Enter

When a guide exists, Enter submits that interval through the same path as Click
and ends the draw session only after success. With no guide, Enter ends the
session. A failed commit preserves the guide and reports the commit failure.

## Escape

Escape discards the uncommitted preview, anchor, and pointer gesture, then ends
the draw session. It does not delete or undo committed domain operations.
Repeated Escape with no active session is a no-op.

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
