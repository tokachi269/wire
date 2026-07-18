import type { ViewerActionContext } from "./context";
import type { BundleTemplateInfo, PathPickInfo } from "../model";
import type { PathPointSpec, WorldPoint } from "../store/viewer";

export class DrawActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  addPathPoint(point: WorldPoint, pick?: PathPickInfo): void {
    const previousPointCount = this.ctx.readSnapshot().pathPoints.length;
    let nextPoint = point;
    let nextSpec: PathPointSpec | null = null;
    if (pick !== undefined) {
      const before = this.ctx.readSnapshot();
      const resolved = this.ctx.bridge.resolveBranchPick(
        pick,
        [...new Set(before.drawBundlePlacements.map((placement) => placement.bundleTemplateId))]
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
    const after = this.ctx.readSnapshot();
    if (after.pathPoints.length > previousPointCount) {
      const index = after.pathPoints.length - 1;
      this.ctx.reproTrace.recordPathPoint(point, after.pathPoints[index], pick, after.pathPointSpecs[index]);
    }
  }

  clearPath(): void {
    const hadPoints = this.ctx.readSnapshot().pathPoints.length > 0;
    this.ctx.store.update((current) => ({ ...current, pathPoints: [], pathPointSpecs: [], error: "" }));
    if (hadPoints) this.ctx.reproTrace.recordPathEdit("clear-path");
  }

  undoPathPoint(): void {
    const hadPoints = this.ctx.readSnapshot().pathPoints.length > 0;
    this.ctx.store.update((current) => ({
      ...current,
      pathPoints: current.pathPoints.slice(0, -1),
      pathPointSpecs: current.pathPointSpecs.slice(0, -1),
      error: ""
    }));
    if (hadPoints) this.ctx.reproTrace.recordPathEdit("undo-path-point");
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
    this.ctx.store.update((current) => ({
      ...current,
      drawBundlePlacements: current.drawBundlePlacements.map((placement) =>
        placement.id === id ? { ...placement, ...change } : placement
      )
    }));
    const placement = this.ctx.readSnapshot().drawBundlePlacements.find((item) => item.id === id);
    if (placement?.generatedBundleId === undefined) return;
    const result = this.ctx.bridge.updateBackboneBundlePlacement(placement.generatedBundleId, placement);
    if (!result.ok) {
      this.ctx.store.setError(result.error);
      return;
    }
    this.ctx.refreshScene();
  }

  duplicateDrawBundlePlacement(id: number): void {
    this.ctx.store.update((current) => {
      const index = current.drawBundlePlacements.findIndex((placement) => placement.id === id);
      if (index < 0) return current;
      const nextId = Math.max(0, ...current.drawBundlePlacements.map((placement) => placement.id)) + 1;
      const placements = [...current.drawBundlePlacements];
      const { generatedBundleId: _generatedBundleId, ...source } = placements[index];
      placements.splice(index + 1, 0, { ...source, id: nextId });
      return { ...current, drawBundlePlacements: placements };
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

  private generatePoints(
    points: WorldPoint[],
    pointSpecs: Array<PathPointSpec | null>,
    fromPlacementEdit = false
  ): void {
    const before = this.ctx.readSnapshot();
    const placements = before.drawBundlePlacements;
    const generationPlacements = placements.map(({ generatedBundleId: _generatedBundleId, ...placement }) => placement);
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
    const result = this.ctx.bridge.generate(
      flatPoints,
      generationPlacements,
      before.clickedPointsOnly ? 0 : before.intervalM,
      before.selectedPoleTemplateId ?? 1,
      before.directionMode,
      before.maxTiltDeg,
      nodeSpecs
    );
    const generateEnd = performance.now();
    if (!result.ok) {
      this.ctx.reproTrace.recordGeneration(before, points, result);
      this.ctx.store.setError(result.error);
      return;
    }
    const sceneStart = performance.now();
    const scene = this.ctx.bridge.scene();
    const placementsWithGeneratedIds = placements.map((placement, index) => ({
      ...placement,
      generatedBundleId: result.generatedBundleIds?.[index] ?? placement.generatedBundleId
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
      showBackboneOverlay: true,
      bundleTemplates,
      drawBundlePlacements: placementsWithGeneratedIds
    }));
    this.ctx.reproTrace.recordGeneration(before, points, result, this.ctx.readSnapshot());
    const sceneUpdateMs = performance.now() - sceneStart;
    const viewerUpdateMs = performance.now() - generateStart;
    const sceneStats = this.ctx.pendingSceneSyncStats;
    this.ctx.pendingSceneSyncStats = null;
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
