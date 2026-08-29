import type { ViewerActionContext } from "./context";
import { CommitFailureCategory, type BundleTemplateInfo, type PathPickInfo, type WireIntervalRequest } from "../model";
import type { DrawActionResult, PathPointSpec, WorldPoint } from "../store/viewer";
import {
  DEFAULT_PREFERRED_SIDE_SIGN,
  routeBundleRules
} from "../profile/defaultBundlePreset";
import type { RouteVariationControls } from "../model";

function newRouteSeed(): number {
  const words = new Uint32Array(2);
  globalThis.crypto.getRandomValues(words);
  return words[0] * 0x200000 + (words[1] & 0x1fffff);
}

export class DrawActions {
  private readonly committedHistory: Array<{ before: string; after: string }> = [];

  constructor(private readonly ctx: ViewerActionContext) {}

  setRouteVariation<K extends keyof RouteVariationControls>(
    param: K,
    value: RouteVariationControls[K]
  ): void {
    if (this.ctx.readSnapshot().pathPoints.length > 0) {
      this.ctx.store.setError("finish or cancel the current wire route before changing route variation");
      return;
    }
    this.ctx.store.update((current) => ({
      ...current,
      routeVariation: { ...current.routeVariation, [param]: value },
      wireRouteSeed: null
    }));
  }

  rerollRouteSeed(): void {
    const current = this.ctx.readSnapshot();
    if (current.pathPoints.length > 0) {
      this.ctx.store.setError("finish or cancel the current wire route before rerolling");
      return;
    }
    let routeSeed = newRouteSeed();
    if (routeSeed === current.wireRouteSeed) routeSeed = (routeSeed + 1) % Number.MAX_SAFE_INTEGER;
    this.resolveRouteVariation(routeSeed);
  }

  primaryViewportPoint(point: WorldPoint, pick?: PathPickInfo): DrawActionResult {
    const current = this.ctx.readSnapshot();
    if (current.pathPoints.length === 0) {
      if (!this.beginRouteVariation()) {
        return { kind: "commit-rejected", reasonCode: "wire_route_variation_rejected" };
      }
      const anchor = this.resolveAnchor(point, pick);
      if (anchor === null) {
        this.ctx.store.update((snapshot) => ({ ...snapshot, wireRouteSeed: null }));
        return {
          kind: "commit-rejected",
          reasonCode: this.ctx.readSnapshot().lastCommitFailure?.reasonCode ?? "wire_anchor_rejected"
        };
      }
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        pathPoints: [anchor.point],
        pathPointSpecs: [anchor.spec],
        wirePreview: { state: "none", request: null },
        error: "",
        lastCommitFailure: null
      }));
      return { kind: "anchor-accepted" };
    }

    const request = this.samePreviewTarget(current.wirePreview.request, point, pick)
      ? current.wirePreview.request
      : this.intervalRequest(point, pick);
    if (request === null) {
      this.ctx.store.setCommitFailure({
        ok: false,
        error: "wire endpoint must differ from the current anchor",
        failureCategory: CommitFailureCategory.InvalidInput,
        reasonCode: "wire_endpoint_matches_anchor"
      }, "wire interval", point);
      return { kind: "commit-rejected", reasonCode: "wire_endpoint_matches_anchor" };
    }
    return this.commitWireInterval(request, false);
  }

  previewViewportPoint(point: WorldPoint, pick?: PathPickInfo): void {
    const request = this.intervalRequest(point, pick);
    if (request === null) return;
    this.applyWireGuide(request);
  }

  confirmSession(): DrawActionResult {
    const preview = this.ctx.readSnapshot().wirePreview;
    if (preview.state !== "guide" || preview.request === null) {
      return { kind: "ignored", reasonCode: "preview-unavailable" };
    }
    return this.commitWireInterval(preview.request, false);
  }

  clearPreview(): void {
    this.ctx.store.update((current) => ({
      ...current,
      wirePreview: { state: "none", request: null }
    }));
  }

  finishSession(): DrawActionResult {
    const preview = this.ctx.readSnapshot().wirePreview;
    if (preview.state === "guide" && preview.request !== null) {
      return this.commitWireInterval(preview.request, true);
    }
    return this.cancelSession();
  }

  cancelSession(): DrawActionResult {
    // Escape with nothing drawn is a distinguishable non-event, not the end of
    // a session. Road reports the same.
    const snapshot = this.ctx.readSnapshot();
    if (snapshot.pathPoints.length === 0 && snapshot.wirePreview.state === "none") {
      return { kind: "ignored", reasonCode: "session-inactive" };
    }
    const cleared = this.ctx.bridge.clearPendingSupportNodes();
    if (!cleared.ok) {
      this.ctx.store.setError(cleared.error);
      return { kind: "commit-rejected", reasonCode: cleared.reasonCode || "wire_session_clear_failed" };
    }
    this.ctx.store.update((current) => ({
      ...current,
      pathPoints: [],
      pathPointSpecs: [],
      wireRouteSeed: null,
      wireVariationId: null,
      wirePreview: { state: "none", request: null },
      error: "",
      lastCommitFailure: null
    }));
    return { kind: "session-ended" };
  }

  undoCommitted(clearSelection: () => void): void {
    const saved = this.committedHistory.at(-1);
    if (saved === undefined) {
      clearSelection();
      return;
    }
    if (this.ctx.bridge.saveState() !== saved.after) {
      this.committedHistory.length = 0;
      clearSelection();
      return;
    }
    const loaded = this.ctx.bridge.loadState(saved.before);
    if (!loaded.ok) {
      this.ctx.store.setError(loaded.error);
      return;
    }
    this.committedHistory.pop();
    this.ctx.refreshScene();
    this.ctx.store.update((current) => ({
      ...current,
      pathPoints: [],
      pathPointSpecs: [],
      wireRouteSeed: null,
      wireVariationId: null,
      wirePreview: { state: "none", request: null },
      drawBundlePlacements: current.drawBundlePlacements.map(({ generatedBundleId: _id, ...placement }) => placement),
      error: ""
    }));
  }

  addPathPoint(point: WorldPoint, pick?: PathPickInfo): void {
    if (this.ctx.readSnapshot().pathPoints.length === 0 && !this.beginRouteVariation()) return;
    let nextPoint = point;
    let nextSpec: PathPointSpec | null = null;
    if (pick !== undefined) {
      const before = this.ctx.readSnapshot();
      const resolved = before.wireVariationId === null
        ? this.ctx.bridge.resolveBranchPick(
          pick,
          [...new Set(before.drawBundlePlacements.map((placement) => placement.bundleTemplateId))]
        )
        : this.ctx.bridge.resolveBundleVariationBranchPick(
          pick, before.wireVariationId
        );
      if (!resolved.ok) {
        this.ctx.store.setError(resolved.error);
        return;
      }
      nextPoint = [resolved.positionX, resolved.positionY, resolved.positionZ];
      nextSpec = {
        supportKind: resolved.supportKind,
        nodeId: resolved.nodeId
      };
    }
    this.ctx.store.update((current) => {
      const anchoredIndex = current.pathPointSpecs.findIndex((spec) => spec !== null);
      const anchoredZ = anchoredIndex >= 0
        ? current.pathPoints[anchoredIndex]?.[2]
        : undefined;
      const appendPoint: WorldPoint = pick === undefined && anchoredZ !== undefined
        ? [nextPoint[0], nextPoint[1], anchoredZ]
        : nextPoint;
      const previous = current.pathPoints.at(-1);
      if (previous !== undefined) {
        const dx = appendPoint[0] - previous[0];
        const dy = appendPoint[1] - previous[1];
        const dz = appendPoint[2] - previous[2];
        if (dx * dx + dy * dy + dz * dz <= 1e-18) {
          return current;
        }
      }
      return {
        ...current,
        pathPoints: [...current.pathPoints, appendPoint],
        pathPointSpecs: [...current.pathPointSpecs, nextSpec],
        error: ""
      };
    });
  }

  clearPath(): void {
    const cleared = this.ctx.bridge.clearPendingSupportNodes();
    if (!cleared.ok) {
      this.ctx.store.setError(cleared.error);
      return;
    }
    this.ctx.store.update((current) => ({
      ...current, pathPoints: [], pathPointSpecs: [], wireRouteSeed: null,
      wireVariationId: null, error: ""
    }));
  }

  undoPathPoint(): void {
    this.ctx.store.update((current) => {
      const pathPoints = current.pathPoints.slice(0, -1);
      return {
        ...current,
        pathPoints,
        pathPointSpecs: current.pathPointSpecs.slice(0, -1),
        wireRouteSeed: pathPoints.length === 0 ? null : current.wireRouteSeed,
        wireVariationId: pathPoints.length === 0 ? null : current.wireVariationId,
        error: ""
      };
    });
  }

  undoPathPointOrClearSelection(clearSelection: () => void): void {
    const current = this.ctx.readSnapshot();
    if (current.pathPoints.length > 0) {
      this.undoPathPoint();
      return;
    }
    clearSelection();
  }

  updateDrawBundlePlacement(id: number, change: Partial<Omit<import("../model").BundlePlacement, "id">>): void {
    const before = this.ctx.readSnapshot();
    const currentTarget = before.drawBundlePlacements.find(
      (placement) => placement.id === id
    );
    if (currentTarget === undefined) return;
    if (currentTarget.generatedBundleId !== undefined) {
      const owner = this.ctx.bridge.backboneBundleVariationForBundle(
        currentTarget.generatedBundleId
      );
      if (!owner.ok || owner.found) {
        this.ctx.store.setError(
          owner.error ||
          "recipe-backed Bundle placement must be changed with explicit variation Apply"
        );
        return;
      }
      const result = this.ctx.bridge.updateBackboneBundlePlacement(
        currentTarget.generatedBundleId, { ...currentTarget, ...change }
      );
      if (!result.ok) {
        this.ctx.store.setError(result.error);
        return;
      }
    }
    this.ctx.store.update((current) => {
      const target = current.drawBundlePlacements.find((placement) => placement.id === id);
      return {
        ...current,
        wireRouteSeed: current.wireRouteSeed ?? newRouteSeed(),
        drawBundleSource: target?.generatedBundleId === undefined ? "manual" : current.drawBundleSource,
        drawBundlePlacements: current.drawBundlePlacements.map((placement) =>
          placement.id === id ? { ...placement, ...change } : placement
        )
      };
    });
    if (currentTarget.generatedBundleId !== undefined) this.ctx.refreshScene();
  }

  duplicateDrawBundlePlacement(id: number): void {
    const before = this.ctx.readSnapshot();
    const source = before.drawBundlePlacements.find(
      (placement) => placement.id === id
    );
    if (source?.generatedBundleId !== undefined) {
      const owner = this.ctx.bridge.backboneBundleVariationForBundle(
        source.generatedBundleId
      );
      if (!owner.ok || owner.found) {
        this.ctx.store.setError(
          owner.error ||
          "recipe-backed Bundle cannot be duplicated as an implicit manual Bundle"
        );
        return;
      }
    }
    this.ctx.store.update((current) => {
      const index = current.drawBundlePlacements.findIndex((placement) => placement.id === id);
      if (index < 0) return current;
      const nextId = Math.max(0, ...current.drawBundlePlacements.map((placement) => placement.id)) + 1;
      const placements = [...current.drawBundlePlacements];
      const { generatedBundleId: _generatedBundleId, ...source } = placements[index];
      placements.splice(index + 1, 0, { ...source, id: nextId });
      return {
        ...current,
        wireRouteSeed: current.wireRouteSeed ?? newRouteSeed(),
        drawBundleSource: "manual",
        drawBundlePlacements: placements
      };
    });
  }

  generatePath(): void {
    let points: WorldPoint[] = [];
    let specs: Array<PathPointSpec | null> = [];
    const unsubscribe = this.ctx.store.value.subscribe((current) => {
      points = current.pathPoints;
      specs = current.pathPointSpecs;
    });
    unsubscribe();
    this.generatePoints(points, specs);
  }

  private resolveAnchor(
    point: WorldPoint,
    pick?: PathPickInfo
  ): { point: WorldPoint; spec: PathPointSpec | null } | null {
    if (pick === undefined) return { point, spec: null };
    const current = this.ctx.readSnapshot();
    const resolved = current.wireVariationId === null
      ? this.ctx.bridge.resolveBranchPick(
        pick,
        [...new Set(current.drawBundlePlacements.map((placement) => placement.bundleTemplateId))]
      )
      : this.ctx.bridge.resolveBundleVariationBranchPick(
        pick, current.wireVariationId
      );
    if (!resolved.ok) {
      this.ctx.store.setCommitFailure(resolved, "wire anchor", point);
      return null;
    }
    return {
      point: [resolved.positionX, resolved.positionY, resolved.positionZ],
      spec: { supportKind: resolved.supportKind, nodeId: resolved.nodeId }
    };
  }

  private beginRouteVariation(): boolean {
    const current = this.ctx.readSnapshot();
    if (current.wireRouteSeed !== null) return true;
    return this.resolveRouteVariation(newRouteSeed());
  }

  private resolveRouteVariation(routeSeed: number): boolean {
    const current = this.ctx.readSnapshot();
    const resolved = this.ctx.bridge.resolveRouteBundleVariation(
      routeBundleRules(current.routeVariation),
      routeSeed,
      DEFAULT_PREFERRED_SIDE_SIGN,
      current.selectedPoleTemplateId ?? 1
    );
    if (!resolved.ok || resolved.placements.length === 0) {
      this.ctx.store.setError(resolved.error || "route bundle variation resolved no bundles");
      return false;
    }
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      wireRouteSeed: routeSeed,
      drawBundleSource: "variation",
      drawBundlePlacements: [...resolved.placements]
        .sort((a, b) => b.height - a.height || a.id - b.id)
    }));
    return true;
  }

  private intervalRequest(point: WorldPoint, pick?: PathPickInfo): WireIntervalRequest | null {
    const current = this.ctx.readSnapshot();
    const anchor = current.pathPoints[0];
    if (anchor === undefined) return null;
    const dx = point[0] - anchor[0];
    const dy = point[1] - anchor[1];
    const dz = point[2] - anchor[2];
    if (dx * dx + dy * dy + dz * dz <= 1e-18) return null;
    return {
      points: [anchor, point],
      pointSpecs: [current.pathPointSpecs[0] ?? null, null],
      targetPick: pick,
      bundlePlacements: current.wireVariationId === null
        ? current.drawBundlePlacements.map(({ generatedBundleId: _id, ...placement }) => placement)
        : current.drawBundlePlacements,
      intervalM: current.clickedPointsOnly ? 0 : current.intervalM,
      poleTypeId: current.selectedPoleTemplateId ?? 1,
      directionMode: current.directionMode,
      maxTiltDeg: current.maxTiltDeg,
      variationId: current.drawBundleSource === "variation"
        ? current.wireVariationId ?? undefined
        : undefined,
      variationRules: current.drawBundleSource === "variation" && current.wireVariationId === null
        ? routeBundleRules(current.routeVariation)
        : undefined,
      routeSeed: current.drawBundleSource === "variation" && current.wireVariationId === null
        ? current.wireRouteSeed ?? undefined
        : undefined,
      preferredSideSign: current.drawBundleSource === "variation" && current.wireVariationId === null
        ? DEFAULT_PREFERRED_SIDE_SIGN
        : undefined
    };
  }

  private samePreviewTarget(
    request: WireIntervalRequest | null,
    point: WorldPoint,
    pick?: PathPickInfo
  ): request is WireIntervalRequest {
    if (request === null) return false;
    return request.points[1][0] === point[0] && request.points[1][1] === point[1] &&
      request.points[1][2] === point[2] && JSON.stringify(request.targetPick) === JSON.stringify(pick);
  }

  private applyWireGuide(request: WireIntervalRequest): void {
    this.ctx.store.update((current) => ({
      ...current,
      wirePreview: { state: "guide", request },
      error: ""
    }));
  }

  private commitWireInterval(request: WireIntervalRequest, endSession: boolean): DrawActionResult {
    const beforeState = this.ctx.bridge.saveState();
    const result = this.ctx.bridge.generateWireInterval(request);
    if (!result.ok) {
      this.ctx.store.update((current) => ({
        ...current,
        wirePreview: { state: "guide", request },
        error: ""
      }));
      this.ctx.store.setCommitFailure(result, "wire interval", request.points[1]);
      return { kind: "commit-rejected", reasonCode: result.reasonCode || "wire_commit_rejected" };
    }
    this.committedHistory.push({ before: beforeState, after: this.ctx.bridge.saveState() });
    this.ctx.refreshScene();
    const endpoint = result.endpoint ?? request.points[1];
    const generatedEndpointId = result.generatedPoleIds?.at(-1);
    const generatedEndpointNodeId = result.generatedNodeIds?.[1];
    const endpointSpec = result.endpointSpec === null
      ? (generatedEndpointId === undefined
      ? null
      : { supportKind: 0, nodeId: generatedEndpointId })
      : {
        ...result.endpointSpec,
        nodeId: generatedEndpointNodeId !== undefined && generatedEndpointNodeId !== "0"
          ? generatedEndpointNodeId
          : result.endpointSpec.nodeId
      };
    const variationId = result.variationId || this.ctx.readSnapshot().wireVariationId;
    const variation = variationId === null || variationId === undefined || variationId.length === 0
      ? null
      : this.ctx.bridge.backboneBundleVariation(variationId);
    const bundleIdByPlacementKey = new Map(
      variation?.ok && variation.found
        ? (variation.instances ?? []).map((instance) => [instance.placementKey, instance.bundleId])
        : []
    );
    const generatedBundleIds = result.generatedBundleIds ?? [];
    this.ctx.store.update((current) => ({
      ...current,
      pathPoints: endSession ? [] : [endpoint],
      pathPointSpecs: endSession ? [] : [endpointSpec],
      wireRouteSeed: endSession ? null : current.wireRouteSeed,
      wireVariationId: endSession ? null : variationId ?? null,
      wirePreview: { state: "none", request: null },
      drawBundlePlacements: current.drawBundlePlacements.map((placement, index) => ({
        ...placement,
        generatedBundleId: bundleIdByPlacementKey.get(placement.id) ??
          generatedBundleIds[index] ?? placement.generatedBundleId
      })),
      error: "",
      lastCommitFailure: null
    }));
    return { kind: "commit-succeeded" };
  }

  private generatePoints(
    points: WorldPoint[],
    pointSpecs: Array<PathPointSpec | null>,
    fromPlacementEdit = false
  ): void {
    const before = this.ctx.readSnapshot();
    const placements = before.drawBundlePlacements;
    const bundleTemplates: BundleTemplateInfo[] = before.bundleTemplates;

    if (points.length < 2) {
      this.ctx.store.setError("path needs at least 2 points");
      return;
    }
    if (placements.length === 0) {
      this.ctx.store.setError("add at least one bundle placement");
      return;
    }
    const selectedTemplates = placements.map((placement) =>
      bundleTemplates.find((template) => template.id === placement.bundleTemplateId)
    );
    if (selectedTemplates.some((template) => template === undefined)) {
      this.ctx.store.setError("selected bundle template is not available");
      return;
    }
    const flatPoints = new Float64Array(points.length * 3);
    points.forEach((point, index) => flatPoints.set(point, index * 3));
    const nodeSpecs = pointSpecs
      .map((spec, index) => spec === null ? null : {
        pointIndex: index,
        supportKind: spec.supportKind,
        nodeId: spec.nodeId
      })
      .filter((spec): spec is { pointIndex: number; supportKind: number; nodeId: string } => spec !== null);
    const generateStart = performance.now();
    const result = before.drawBundleSource === "variation"
      ? this.ctx.bridge.generateBundleVariation(
        flatPoints, routeBundleRules(before.routeVariation),
        before.wireRouteSeed ?? newRouteSeed(), DEFAULT_PREFERRED_SIDE_SIGN,
        before.clickedPointsOnly ? 0 : before.intervalM,
        before.selectedPoleTemplateId ?? 1, before.directionMode,
        before.maxTiltDeg, nodeSpecs
      )
      : this.ctx.bridge.generate(
        flatPoints,
        placements.map(({ generatedBundleId: _generatedBundleId, ...placement }) => placement),
        before.clickedPointsOnly ? 0 : before.intervalM,
        before.selectedPoleTemplateId ?? 1,
        before.directionMode,
        before.maxTiltDeg,
        nodeSpecs
      );
    const generateEnd = performance.now();
    if (!result.ok) {
      this.ctx.store.setError(result.error);
      return;
    }
    const sceneStart = performance.now();
    const scene = this.ctx.bridge.scene();
    const generatedVariation = result.variationId === undefined || result.variationId.length === 0
      ? null
      : this.ctx.bridge.backboneBundleVariation(result.variationId);
    const generatedBundleByPlacementKey = new Map(
      generatedVariation?.ok && generatedVariation.found
        ? (generatedVariation.instances ?? []).map((instance) =>
          [instance.placementKey, instance.bundleId]
        )
        : []
    );
    const placementsWithGeneratedIds = placements.map((placement, index) => ({
      ...placement,
      generatedBundleId: generatedBundleByPlacementKey.get(placement.id) ??
        result.generatedBundleIds?.[index] ?? placement.generatedBundleId
    }));
    this.ctx.store.update((current) => ({
      ...current,
      parts: scene.parts,
      models: scene.models,
      poles: scene.poles,
      ports: scene.ports,
      spans: scene.spans,
      supportNodes: scene.supportNodes,
      backboneEdges: scene.backboneEdges,
      error: "",
      generationMs: result.totalMs,
      generationTiming: result.timing,
      generationCallMs: generateEnd - generateStart,
      pathPoints: before.keepPathAfterGenerate ? points : [],
      pathPointSpecs: before.keepPathAfterGenerate ? pointSpecs : [],
      wireVariationId: before.keepPathAfterGenerate ? result.variationId ?? null : null,
      showBackboneOverlay: true,
      bundleTemplates,
      drawBundlePlacements: placementsWithGeneratedIds
    }));
    const sceneUpdateMs = performance.now() - sceneStart;
    const viewerUpdateMs = performance.now() - generateStart;
    const sceneStats = this.ctx.consumeSceneContentSyncStats();
    const sceneDiagnostic = sceneStats === null
      ? ""
      : ` | scene parts total=${sceneStats.total} reused=${sceneStats.reused}` +
        ` rebuilt=${sceneStats.rebuilt} removed=${sceneStats.removed}` +
        ` models total=${sceneStats.modelTotal} reused=${sceneStats.modelReused}` +
        ` updated=${sceneStats.modelUpdated} rebuilt=${sceneStats.modelRebuilt}` +
        ` removed=${sceneStats.modelRemoved}`;
    const timingDiagnostic =
      ` | perf core=${result.totalMs.toFixed(2)}ms` +
      ` wasm-call=${(generateEnd - generateStart).toFixed(2)}ms` +
      ` scene=${sceneUpdateMs.toFixed(2)}ms` +
      ` action=${viewerUpdateMs.toFixed(2)}ms` +
      ` emit=${result.timing.emitMs.toFixed(2)}ms` +
      ` save-graph=${result.timing.saveGraphMs.toFixed(2)}ms` +
      ` geom=${result.timing.geomMs.toFixed(2)}ms`;
    this.ctx.store.update((current) => ({
      ...current,
      sceneUpdateMs,
      viewerUpdateMs,
      logs: [
        ...current.logs,
        `${fromPlacementEdit ? "bundle placement updated" : "route generated"}: ` +
          `${result.generatedPoleCount} poles / ${result.generatedSpanCount} spans` +
          `${timingDiagnostic}${sceneDiagnostic}`
      ]
    }));
  }
}
