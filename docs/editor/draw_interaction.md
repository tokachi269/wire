# Draw interaction

Road and wire drawing use the same event contract. Domain geometry, snapping,
validation, and commit requests remain owned by each domain.

## State

A draw session is in one of four transient states:

- `idle`: no start anchor exists.
- `anchored`: a start anchor exists, but there is no candidate interval.
- `valid preview`: the current candidate can be committed.
- `invalid preview`: the current candidate cannot be committed and carries a reason.

Committed domain entities are not draw-session state. A preview, guide, pointer
gesture, or start anchor is never persisted.

## Click

The first primary action establishes the start anchor. Once an anchor exists, a
primary action commits the currently displayed valid interval. A successful
commit makes its endpoint the next start anchor and starts previewing the next
interval.

The committed request is the request produced by the preview. Click does not
rebuild the candidate through another shape or validation path.

## Enter

When a valid preview exists, Enter commits that interval through the same path
as Click and then ends the draw session. With no preview, Enter ends the session.
With an invalid preview, Enter does not commit or end the session; it preserves
the candidate and its reason.

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

Pointer movement after anchoring produces one of `valid preview`, `invalid
preview`, or `no candidate`. A valid preview is the exact Core result that the
next Click or Enter will request. An invalid candidate remains visible with an
invalid treatment, a problem location, and a concise reason. The last valid
preview must not be presented as the current candidate.

Road preview includes the surface and affected local connection. Wire preview
includes the route and support candidates. Hover guides are separate scene
objects and never substitute for a Core preview.

## Atomicity

One committed interval is one atomic domain operation. Road commits its segment,
nodes, corridor membership, connection, surface, and marking together. Wire
commits its route topology, support binding, and generated wire geometry
together. Guide and topology are not separate commit phases.

Text controls own Enter and Escape while focused. IME composition does not send
draw commands. Keyboard input is dispatched to exactly one active draw domain.
