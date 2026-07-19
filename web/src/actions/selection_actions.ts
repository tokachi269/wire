import type { ViewerActionContext } from "./context";
import type { OperationResult } from "../model";
import type { SelectionKind } from "../store/viewer";

export class SelectionActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  select(kind: SelectionKind, id: string): void {
    this.ctx.store.update((current) => ({ ...current, selection: { kind, id } }));
  }

  clearSelection(): void {
    this.ctx.store.update((current) => ({ ...current, selection: null }));
  }

  applyTiltToAll(maxTiltDeg: number): void {
    const ids = this.ctx.readSnapshot().poles.map((pole) => pole.id);
    this.ctx.finishOperation(
      this.ctx.bridge.applyPoleTilt(ids, maxTiltDeg),
      `tilt max=${maxTiltDeg.toFixed(2)}`
    );
  }

  applyTiltToSelection(maxTiltDeg: number): void {
    const selection = this.ctx.readSnapshot().selection;
    if (selection?.kind !== "pole") {
      this.ctx.store.setError("pole selection is required");
      return;
    }
    this.ctx.finishOperation(
      this.ctx.bridge.applyPoleTilt([selection.id], maxTiltDeg),
      `pole ${selection.id} tilt max=${maxTiltDeg.toFixed(2)}`
    );
  }

  clearSelectedOverride(which: "pole" | "socketA" | "socketB" | "branchDown"): void {
    const selection = this.ctx.readSnapshot().selection;
    if (selection === null) {
      this.ctx.store.setError("selection is required");
      return;
    }
    let result: OperationResult;
    if (which === "pole" && selection.kind === "pole") {
      result = this.ctx.bridge.clearPoleOrientationOverride(selection.id);
    } else if (which === "socketA" && selection.kind === "span") {
      result = this.ctx.bridge.clearSpanSocketOverride(selection.id, true);
    } else if (which === "socketB" && selection.kind === "span") {
      result = this.ctx.bridge.clearSpanSocketOverride(selection.id, false);
    } else if (which === "branchDown" && selection.kind === "span") {
      result = this.ctx.bridge.clearSpanBranchDownOverride(selection.id);
    } else {
      this.ctx.store.setError("selected entity does not support this operation");
      return;
    }
    this.ctx.finishOperation(result, `${which} override cleared for ${selection.id}`);
  }
}
