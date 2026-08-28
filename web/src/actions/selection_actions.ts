import type { ViewerActionContext } from "./context";
import type { OperationResult } from "../model";
import type { RouteVariationControls } from "../model";
import type { SelectionKind } from "../store/viewer";
import {
  adjustRouteBundleRules,
  DEFAULT_ROUTE_VARIATION_CONTROLS
} from "../profile/defaultBundlePreset";

function newRouteSeed(): number {
  const words = new Uint32Array(2);
  globalThis.crypto.getRandomValues(words);
  return words[0] * 0x200000 + (words[1] & 0x1fffff);
}

export class SelectionActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  select(kind: SelectionKind, id: string): void {
    const span = kind === "span"
      ? this.ctx.readSnapshot().spans.find((item) => item.id === id)
      : undefined;
    const variation = span === undefined
      ? null
      : this.ctx.bridge.backboneBundleVariationForBundle(span.bundleId);
    const selectedVariation = variation?.ok && variation.found ? variation : null;
    this.ctx.store.update((current) => ({
      ...current,
      selection: { kind, id },
      selectedRouteVariation: selectedVariation,
      selectedRouteVariationControls: selectedVariation === null
        ? current.selectedRouteVariationControls
        : { ...DEFAULT_ROUTE_VARIATION_CONTROLS }
    }));
  }

  clearSelection(): void {
    this.ctx.store.update((current) => ({
      ...current, selection: null, selectedRouteVariation: null
    }));
  }

  setRouteVariation<K extends keyof RouteVariationControls>(
    param: K, value: RouteVariationControls[K]
  ): void {
    const snapshot = this.ctx.readSnapshot();
    if (snapshot.selectedRouteVariation === null) return;
    if (snapshot.pathPoints.length > 0 &&
        snapshot.wireVariationId === snapshot.selectedRouteVariation.variationId) {
      this.ctx.store.setError(
        "finish or cancel the active wire route before changing its persisted variation"
      );
      return;
    }
    this.ctx.store.update((current) => ({
      ...current,
      selectedRouteVariationControls: {
        ...current.selectedRouteVariationControls,
        [param]: value
      }
    }));
  }

  rerollRouteVariation(): void {
    const current = this.ctx.readSnapshot();
    if (current.selectedRouteVariation === null) return;
    if (current.pathPoints.length > 0 &&
        current.wireVariationId === current.selectedRouteVariation.variationId) {
      this.ctx.store.setError(
        "finish or cancel the active wire route before rerolling its persisted variation"
      );
      return;
    }
    let routeSeed = newRouteSeed();
    if (routeSeed === current.selectedRouteVariation.routeSeed) {
      routeSeed = (routeSeed + 1) % Number.MAX_SAFE_INTEGER;
    }
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      selectedRouteVariation: snapshot.selectedRouteVariation === null
        ? null
        : { ...snapshot.selectedRouteVariation, routeSeed }
    }));
  }

  applyRouteVariation(): void {
    const current = this.ctx.readSnapshot();
    const variation = current.selectedRouteVariation;
    if (variation?.variationId === undefined || variation.routeSeed === undefined ||
        variation.preferredSideSign === undefined || variation.poleTypeId === undefined) {
      this.ctx.store.setError("select a recipe-backed wire span before applying route variation");
      return;
    }
    if (current.pathPoints.length > 0 &&
        current.wireVariationId === variation.variationId) {
      this.ctx.store.setError(
        "finish or cancel the active wire route before applying its persisted variation"
      );
      return;
    }
    const result = this.ctx.bridge.applyBackboneBundleVariation(
      variation.variationId,
      adjustRouteBundleRules(
        variation.rules ?? [], current.selectedRouteVariationControls
      ),
      variation.routeSeed,
      variation.preferredSideSign,
      variation.poleTypeId
    );
    if (!result.ok) {
      const persisted = this.ctx.bridge.backboneBundleVariation(variation.variationId);
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        selectedRouteVariation: persisted.ok && persisted.found
          ? persisted
          : snapshot.selectedRouteVariation,
        selectedRouteVariationControls: persisted.ok && persisted.found
          ? { ...DEFAULT_ROUTE_VARIATION_CONTROLS }
          : snapshot.selectedRouteVariationControls,
        error: result.error
      }));
      return;
    }
    this.ctx.refreshScene();
    const updated = this.ctx.bridge.backboneBundleVariation(variation.variationId);
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      selectedRouteVariation: updated.ok && updated.found ? updated : null,
      selectedRouteVariationControls: updated.ok && updated.found
        ? { ...DEFAULT_ROUTE_VARIATION_CONTROLS }
        : snapshot.selectedRouteVariationControls,
      error: "",
      logs: [...snapshot.logs, `route variation ${variation.variationId} applied`]
    }));
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
