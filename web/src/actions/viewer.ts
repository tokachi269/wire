import { ReproTrace } from "./reproTrace";
import type { WireBridge } from "../bridge/wire";
import type {
  BundlePlacement,
  BundleTemplateInfo,
  CableTemplateInfo,
  GeometrySettings,
  LayoutSettings,
  OperationResult,
  PathPickInfo,
  PoleTemplateInfo,
  SceneContentSyncStats,
  VisualSettings
} from "../model";
import type {
  PathPointSpec,
  SelectionKind,
  ViewerSnapshot,
  ViewerStore,
  WorldPoint
} from "../store/viewer";
import { createViewerSnapshot } from "../store/viewer";
import type { WorkspacePreferences } from "../store/workspace";
import { WorkspaceCache } from "../store/workspace";
import { bundleTemplateCategory } from "../labels";

const JAPAN_DISTRIBUTION_PRIMITIVE = {
  poleVisibleHeightM: 10.0,
  highVoltageZ: 9.2,
  lowVoltageZ: 7.4,
  communicationZMin: 4.8,
  communicationZMax: 5.8,
  serviceDropZMin: 4.5,
  serviceDropZMax: 6.5,
  highVoltageX: [-0.75, 0.0, 0.75],
  lowVoltageX: [-0.45, 0.0, 0.45],
  communicationX: [-0.55, -0.18, 0.18, 0.55],
  opticalX: [-0.35, 0.35],
  cableDiameterM: {
    HV_BARE: 0.024,
    LV_INSULATED: 0.020,
    COMM_MULTI: 0.016,
    OPTICAL_FIBER: 0.012,
    DROP_SERVICE: 0.016
  }
} as const;

function patchedCableTemplate(template: CableTemplateInfo): CableTemplateInfo {
  const diameter = JAPAN_DISTRIBUTION_PRIMITIVE.cableDiameterM[
    template.name as keyof typeof JAPAN_DISTRIBUTION_PRIMITIVE.cableDiameterM
  ];
  if (diameter === undefined) {
    return template;
  }
  return {
    ...template,
    outerDiameter: diameter
  };
}

function patchedPoleTemplate(template: PoleTemplateInfo): PoleTemplateInfo {
  if (template.name !== "DistributionPole" && template.name !== "CommunicationPole") {
    return template;
  }
  const nextByCategory = new Map<number, number>();
  const nextPosition = (
    category: number,
    positions: readonly number[],
    fallback: number
  ) => positions[nextByCategory.get(category) ?? 0] ?? fallback;
  const consumePosition = (category: number) => {
    nextByCategory.set(category, (nextByCategory.get(category) ?? 0) + 1);
  };

  return {
    ...template,
    defaultHeight: JAPAN_DISTRIBUTION_PRIMITIVE.poleVisibleHeightM,
    portBands: template.portBands.map((band) => {
      if (band.category === 0) {
        const lateral = nextPosition(0, JAPAN_DISTRIBUTION_PRIMITIVE.highVoltageX, band.lateralCenter);
        consumePosition(0);
        return {
          ...band,
          lateralCenter: lateral,
          lateralMin: lateral - 0.08,
          lateralMax: lateral + 0.08,
          heightCenter: JAPAN_DISTRIBUTION_PRIMITIVE.highVoltageZ,
          heightMin: JAPAN_DISTRIBUTION_PRIMITIVE.highVoltageZ - 0.06,
          heightMax: JAPAN_DISTRIBUTION_PRIMITIVE.highVoltageZ + 0.06,
          minSpacing: Math.max(band.minSpacing, 0.36)
        };
      }
      if (band.category === 1) {
        const lateral = nextPosition(1, JAPAN_DISTRIBUTION_PRIMITIVE.lowVoltageX, band.lateralCenter);
        consumePosition(1);
        return {
          ...band,
          lateralCenter: lateral,
          lateralMin: lateral - 0.07,
          lateralMax: lateral + 0.07,
          heightCenter: JAPAN_DISTRIBUTION_PRIMITIVE.lowVoltageZ,
          heightMin: JAPAN_DISTRIBUTION_PRIMITIVE.lowVoltageZ - 0.06,
          heightMax: JAPAN_DISTRIBUTION_PRIMITIVE.lowVoltageZ + 0.06,
          minSpacing: Math.max(band.minSpacing, 0.24)
        };
      }
      if (band.category === 2 || band.category === 3) {
        const positions = band.category === 2
          ? JAPAN_DISTRIBUTION_PRIMITIVE.communicationX
          : JAPAN_DISTRIBUTION_PRIMITIVE.opticalX;
        const lateral = nextPosition(band.category, positions, band.lateralCenter);
        consumePosition(band.category);
        const heightCenter =
          (JAPAN_DISTRIBUTION_PRIMITIVE.communicationZMin +
            JAPAN_DISTRIBUTION_PRIMITIVE.communicationZMax) / 2;
        return {
          ...band,
          lateralCenter: lateral,
          lateralMin: JAPAN_DISTRIBUTION_PRIMITIVE.communicationX[0],
          lateralMax: JAPAN_DISTRIBUTION_PRIMITIVE.communicationX[3],
          heightCenter,
          heightMin: JAPAN_DISTRIBUTION_PRIMITIVE.communicationZMin,
          heightMax: JAPAN_DISTRIBUTION_PRIMITIVE.communicationZMax,
          minSpacing: Math.max(band.minSpacing, 0.20)
        };
      }
      if (band.category === 4) {
        const heightCenter =
          (JAPAN_DISTRIBUTION_PRIMITIVE.serviceDropZMin +
            JAPAN_DISTRIBUTION_PRIMITIVE.serviceDropZMax) / 2;
        return {
          ...band,
          heightCenter,
          heightMin: JAPAN_DISTRIBUTION_PRIMITIVE.serviceDropZMin,
          heightMax: JAPAN_DISTRIBUTION_PRIMITIVE.serviceDropZMax
        };
      }
      return band;
    })
  };
}

function shallowEqual<T extends object>(a: T, b: T): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

function targetTemplateLayerForCategory(category: number): number {
  if (category === 0) return 2;
  if (category === 4) return 0;
  return 1;
}

function bundlePlacementDefault(
  template: BundleTemplateInfo,
  poleTemplate: PoleTemplateInfo | undefined
): Pick<BundlePlacement, "height" | "offset" | "spacing"> {
  const category = bundleTemplateCategory(template);
  const targetLayer = targetTemplateLayerForCategory(category);
  const laneCount = template.fixedCount ? template.fixedCountValue : template.defaultCount;
  const candidates = (poleTemplate?.portBands ?? [])
    .filter((band) => band.enabled && band.category === category && band.layer === targetLayer)
    .sort((a, b) => b.priority - a.priority || a.bandId - b.bandId);
  const bands = candidates.filter((candidate, index, all) =>
    all.findIndex((item) => Math.abs(item.lateralCenter - candidate.lateralCenter) <= 1e-12) === index
  ).slice(0, Math.max(1, laneCount));
  const divisor = Math.max(1, bands.length);
  return {
    height: bands.reduce((sum, band) => sum + band.heightCenter, 0) / divisor,
    offset: category === 0 ? -0.2 : bands.reduce((sum, band) => sum + band.lateralCenter, 0) / divisor,
    spacing: template.defaultSpacing
  };
}

function defaultBundlePlacements(
  templates: BundleTemplateInfo[],
  poleTemplate: PoleTemplateInfo | undefined
): BundlePlacement[] {
  return templates
    .filter((template) => [0, 1, 2, 3].includes(bundleTemplateCategory(template)))
    .map((template) => ({
      id: 0,
      bundleTemplateId: template.id,
      count: template.defaultCount,
      explicit: true,
      ...bundlePlacementDefault(template, poleTemplate)
    }))
    .sort((a, b) => b.height - a.height ||
      bundleTemplateCategory(templates.find((item) => item.id === a.bundleTemplateId) ??
        { defaultLayer: 0 }) -
      bundleTemplateCategory(templates.find((item) => item.id === b.bundleTemplateId) ??
        { defaultLayer: 0 }))
    .map((placement, index) => ({ ...placement, id: index + 1 }));
}

function placementUsesTransientZeroDefaults(placements: BundlePlacement[]): boolean {
  return placements.length > 0 &&
    placements.every((placement) =>
      Math.abs(placement.height) <= 1e-12 && Math.abs(placement.offset) <= 1e-12
    );
}

export class ViewerActions {
  private pendingPreview: ReturnType<typeof setTimeout> | null = null;
  private activeCancel: (() => void) | null = null;
  private interactionFrames: number[] = [];
  private interactionActive = false;
  private suppressNextCommit = false;
  private readonly reproTrace = new ReproTrace();
  private factoryCoreState = "";
  private workspaceSaveTimer: ReturnType<typeof setTimeout> | null = null;
  private workspaceSubscription: (() => void) | null = null;
  private persistencePaused = true;
  private pendingSceneSyncStats: SceneContentSyncStats | null = null;

  constructor(
    private readonly bridge: WireBridge,
    private readonly store: ViewerStore,
    private readonly workspaceCache: WorkspaceCache | null = null
  ) {}

  initialize(): void {
    const bundleTemplates = this.bridge.bundleTemplates();
    let cableTemplates = this.bridge.cableTemplates();
    let poleTemplates = this.bridge.poleTemplates();
    const geometry = this.bridge.geometrySettings();
    if (!geometry.sagEnabled) {
      geometry.sagEnabled = true;
      const result = this.bridge.updateGeometrySettings(geometry);
      if (!result.ok) {
        this.store.setError(result.error);
      }
    }
    cableTemplates = cableTemplates.map((template) => {
      const patched = patchedCableTemplate(template);
      if (!shallowEqual(template, patched)) {
        const result = this.bridge.updateCableTemplate(patched, []);
        if (!result.ok) {
          this.store.setError(result.error);
          return template;
        }
      }
      return patched;
    });
    poleTemplates = poleTemplates.map((template) => {
      const patched = patchedPoleTemplate(template);
      if (!shallowEqual(template, patched)) {
        const result = this.bridge.updatePoleTemplate(patched);
        if (!result.ok) {
          this.store.setError(result.error);
          return template;
        }
      }
      return patched;
    });
    const defaultBundleId =
      bundleTemplates.find((template) => bundleTemplateCategory(template) === 0)?.id ??
      bundleTemplates[0]?.id ??
      null;
    const defaultCableId =
      cableTemplates.find((template) => template.name === "HV_BARE")?.id ??
      cableTemplates[0]?.id ??
      null;
    const defaultPoleId =
      poleTemplates.find((template) => template.name === "CommunicationPole")?.id ??
      poleTemplates[0]?.id ??
      null;
    this.store.update((current) => ({
      ...current,
      bundleTemplates,
      selectedBundleTemplateId:
        current.selectedBundleTemplateId ?? defaultBundleId,
      drawBundlePlacements:
        current.drawBundlePlacements.length > 0
          ? current.drawBundlePlacements
          : defaultBundlePlacements(
              bundleTemplates,
              poleTemplates.find((template) => template.id === defaultPoleId)
            ),
      cableTemplates,
      selectedCableTemplateId:
        current.selectedCableTemplateId ?? defaultCableId,
      poleTemplates,
      selectedPoleTemplateId:
        current.selectedPoleTemplateId ?? defaultPoleId,
      geometry: this.bridge.geometrySettings(),
      layout: this.bridge.layoutSettings(),
      visual: this.bridge.visualSettings()
    }));
    this.factoryCoreState = this.bridge.saveState();
  }

  async restoreWorkspace(): Promise<void> {
    const cached = await (this.workspaceCache?.read() ?? Promise.resolve(null));
    if (cached !== null) {
      const result = this.bridge.loadState(cached.coreState);
      if (result.ok) {
        this.refreshCatalogs();
        this.restoreWorkspacePreferences(cached.viewer);
        this.store.update((current) => ({
          ...current,
          geometry: this.bridge.geometrySettings(),
          layout: this.bridge.layoutSettings(),
          visual: this.bridge.visualSettings(),
          error: ""
        }));
        this.refreshScene();
      } else {
        this.persistencePaused = true;
        this.store.setError(`Workspace restore failed: ${result.error}`);
        return;
      }
    }
    this.persistencePaused = false;
    this.startWorkspacePersistence();
  }

  addPathPoint(point: WorldPoint, pick?: PathPickInfo): void {
    const previousPointCount = this.readSnapshot().pathPoints.length;
    let nextPoint = point;
    let nextSpec: PathPointSpec | null = null;
    if (pick !== undefined) {
      const before = this.readSnapshot();
      const resolved = this.bridge.resolveBranchPick(
        pick,
        [...new Set(before.drawBundlePlacements.map((placement) => placement.bundleTemplateId))]
      );
      if (!resolved.ok) {
        this.store.setError(resolved.error);
        return;
      }
      nextPoint = [resolved.positionX, resolved.positionY, resolved.positionZ];
      nextSpec = {
        supportKind: resolved.supportKind,
        nodeId: resolved.nodeId
      };
    }
    this.store.update((current) => {
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
    const after = this.readSnapshot();
    if (after.pathPoints.length > previousPointCount) {
      const index = after.pathPoints.length - 1;
      this.reproTrace.recordPathPoint(point, after.pathPoints[index], pick, after.pathPointSpecs[index]);
    }
  }

  clearPath(): void {
    const hadPoints = this.readSnapshot().pathPoints.length > 0;
    this.store.update((current) => ({ ...current, pathPoints: [], pathPointSpecs: [], error: "" }));
    if (hadPoints) this.reproTrace.recordPathEdit("clear-path");
  }

  undoPathPoint(): void {
    const hadPoints = this.readSnapshot().pathPoints.length > 0;
    this.store.update((current) => ({
      ...current,
      pathPoints: current.pathPoints.slice(0, -1),
      pathPointSpecs: current.pathPointSpecs.slice(0, -1),
      error: ""
    }));
    if (hadPoints) this.reproTrace.recordPathEdit("undo-path-point");
  }

  undoPathPointOrClearSelection(): void {
    const current = this.readSnapshot();
    if (current.pathPoints.length > 0) {
      this.undoPathPoint();
      return;
    }
    this.clearSelection();
  }

  setDrawOption(
    param:
      | "cameraFov"
      | "showBackboneOverlay"
      | "showGroundGrid"
      | "showPreview"
      | "keepPathAfterGenerate"
      | "drawPlaneZ"
      | "intervalM"
      | "clickedPointsOnly"
      | "directionMode"
      | "maxTiltDeg"
      | "solidSupportRender"
      | "selectionIncludePoles"
      | "selectionIncludeMidair"
      | "selectionIncludeSpans"
      | "showLeftPanel"
      | "showRightPanel"
      | "workspaceLeftWidth"
      | "workspaceWidth",
    value: number | boolean
  ): void {
    this.store.update((current) => ({ ...current, [param]: value }));
  }

  select(kind: SelectionKind, id: string): void {
    this.store.update((current) => ({ ...current, selection: { kind, id } }));
  }

  clearSelection(): void {
    this.store.update((current) => ({ ...current, selection: null }));
  }

  selectBundleTemplate(id: number): void {
    this.store.update((current) => ({
      ...current,
      selectedBundleTemplateId: id
    }));
  }

  updateDrawBundlePlacement(id: number, change: Partial<Omit<BundlePlacement, "id">>): void {
    this.store.update((current) => ({
      ...current,
      drawBundlePlacements: current.drawBundlePlacements.map((placement) =>
        placement.id === id ? { ...placement, ...change } : placement
      )
    }));
    const placement = this.readSnapshot().drawBundlePlacements.find((item) => item.id === id);
    if (placement?.generatedBundleId === undefined) return;
    const result = this.bridge.updateBackboneBundlePlacement(placement.generatedBundleId, placement);
    if (!result.ok) {
      this.store.setError(result.error);
      return;
    }
    this.refreshScene();
  }

  duplicateDrawBundlePlacement(id: number): void {
    this.store.update((current) => {
      const index = current.drawBundlePlacements.findIndex((placement) => placement.id === id);
      if (index < 0) return current;
      const nextId = Math.max(0, ...current.drawBundlePlacements.map((placement) => placement.id)) + 1;
      const placements = [...current.drawBundlePlacements];
      const { generatedBundleId: _generatedBundleId, ...source } = placements[index];
      placements.splice(index + 1, 0, { ...source, id: nextId });
      return { ...current, drawBundlePlacements: placements };
    });
  }

  selectCableTemplate(id: number): void {
    this.store.update((current) => ({ ...current, selectedCableTemplateId: id }));
  }

  selectPoleTemplate(id: number): void {
    this.store.update((current) => ({ ...current, selectedPoleTemplateId: id }));
  }

  exportReproCapture(): void {
    const text = this.reproTrace.toText(this.readSnapshot());
    const url = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = `wire-repro-${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;
    document.body.append(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
    this.store.update((current) => ({
      ...current,
      logs: [...current.logs, "repro trace downloaded"]
    }));
  }

  async resetWorkspace(): Promise<void> {
    if (this.factoryCoreState.length === 0) {
      this.store.setError("Workspace reset is unavailable before initialization");
      return;
    }
    this.persistencePaused = true;
    this.clearWorkspaceSaveTimer();
    await this.workspaceCache?.clear();
    const result = this.bridge.loadState(this.factoryCoreState);
    if (!result.ok) {
      this.persistencePaused = false;
      this.store.setError(`Workspace reset failed: ${result.error}`);
      return;
    }
    this.store.replace(createViewerSnapshot());
    this.initialize();
    await this.restoreWorkspace();
    this.refreshScene();
    this.store.update((current) => ({
      ...current,
      logs: [...current.logs, "Workspace reset"]
    }));
    await this.flushWorkspaceCache();
  }

  async flushWorkspaceCache(): Promise<void> {
    if (this.workspaceCache === null || this.persistencePaused) return;
    this.clearWorkspaceSaveTimer();
    try {
      await this.workspaceCache.write(this.bridge.saveState(), this.readSnapshot());
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.persistencePaused = true;
      this.store.setError(`Workspace cache failed: ${message}`);
    }
  }

  dispose(): void {
    void this.flushWorkspaceCache();
    this.workspaceSubscription?.();
    this.workspaceSubscription = null;
    this.clearWorkspaceSaveTimer();
  }

  generatePath(): void {
    let points: WorldPoint[] = [];
    let specs: Array<PathPointSpec | null> = [];
    const unsubscribe = this.store.value.subscribe((current) => {
      points = current.pathPoints;
      specs = current.pathPointSpecs;
    });
    unsubscribe();
    this.generatePoints(points, specs);
  }

  previewGeometry<K extends keyof GeometrySettings>(
    param: K,
    value: GeometrySettings[K]
  ): void {
    this.previewSetting(
      `geometry.${String(param)}`,
      String(param),
      value,
      33,
      (current) => current.geometry[param],
      (current, next) => ({ ...current, geometry: { ...current.geometry, [param]: next } }),
      () => this.bridge.updateGeometrySettings(this.readSnapshot().geometry)
    );
  }

  commitGeometry<K extends keyof GeometrySettings>(
    param: K,
    value: GeometrySettings[K]
  ): void {
    this.commitSetting(
      `geometry.${String(param)}`,
      value,
      (current, next) => ({ ...current, geometry: { ...current.geometry, [param]: next } }),
      () => this.bridge.updateGeometrySettings(this.readSnapshot().geometry),
      () =>
        this.store.update((current) => ({
          ...current,
          geometry: this.bridge.geometrySettings()
        }))
    );
  }

  commitLayout<K extends keyof LayoutSettings>(
    param: K,
    value: LayoutSettings[K]
  ): void {
    this.commitSetting(
      `layout.${String(param)}`,
      value,
      (current, next) => ({ ...current, layout: { ...current.layout, [param]: next } }),
      () => this.bridge.updateLayoutSettings(this.readSnapshot().layout),
      () =>
        this.store.update((current) => ({
          ...current,
          layout: this.bridge.layoutSettings()
        }))
    );
  }

  previewVisual<K extends keyof VisualSettings>(
    param: K,
    value: VisualSettings[K]
  ): void {
    this.previewSetting(
      `visual.${String(param)}`,
      String(param),
      value,
      33,
      (current) => current.visual[param],
      (current, next) => ({ ...current, visual: { ...current.visual, [param]: next } }),
      () => this.bridge.updateVisualSettings(this.readSnapshot().visual)
    );
  }

  commitVisual<K extends keyof VisualSettings>(
    param: K,
    value: VisualSettings[K]
  ): void {
    this.commitSetting(
      `visual.${String(param)}`,
      value,
      (current, next) => ({ ...current, visual: { ...current.visual, [param]: next } }),
      () => this.bridge.updateVisualSettings(this.readSnapshot().visual),
      () =>
        this.store.update((current) => ({
          ...current,
          visual: this.bridge.visualSettings()
        }))
    );
  }

  applyTiltToAll(maxTiltDeg: number): void {
    const ids = this.readSnapshot().poles.map((pole) => pole.id);
    this.finishOperation(
      this.bridge.applyPoleTilt(ids, maxTiltDeg),
      `tilt max=${maxTiltDeg.toFixed(2)}`
    );
  }

  applyTiltToSelection(maxTiltDeg: number): void {
    const selection = this.readSnapshot().selection;
    if (selection?.kind !== "pole") {
      this.store.setError("pole selection is required");
      return;
    }
    this.finishOperation(
      this.bridge.applyPoleTilt([selection.id], maxTiltDeg),
      `pole ${selection.id} tilt max=${maxTiltDeg.toFixed(2)}`
    );
  }

  clearSelectedOverride(which: "pole" | "socketA" | "socketB" | "branchDown"): void {
    const selection = this.readSnapshot().selection;
    if (selection === null) {
      this.store.setError("selection is required");
      return;
    }
    let result: OperationResult;
    if (which === "pole" && selection.kind === "pole") {
      result = this.bridge.clearPoleOrientationOverride(selection.id);
    } else if (which === "socketA" && selection.kind === "span") {
      result = this.bridge.clearSpanSocketOverride(selection.id, true);
    } else if (which === "socketB" && selection.kind === "span") {
      result = this.bridge.clearSpanSocketOverride(selection.id, false);
    } else if (which === "branchDown" && selection.kind === "span") {
      result = this.bridge.clearSpanBranchDownOverride(selection.id);
    } else {
      this.store.setError("selected entity does not support this operation");
      return;
    }
    this.finishOperation(result, `${which} override cleared for ${selection.id}`);
  }

  resetSpanReferenceLengths(): void {
    this.finishOperation(
      this.bridge.resetSpanReferenceLengths(),
      "span reference lengths reset"
    );
  }

  previewCableTemplate<K extends keyof CableTemplateInfo>(
    param: K,
    value: CableTemplateInfo[K]
  ): void {
    const selected = this.selectedCableTemplate();
    if (selected === null) {
      this.store.setError("cable template is not selected");
      return;
    }
    this.previewSetting(
      `cable.${String(param)}`,
      String(param),
      value,
      33,
      () => selected[param],
      (current, next) => ({
        ...current,
        cableTemplates: current.cableTemplates.map((template) =>
          template.id === selected.id ? { ...template, [param]: next } : template
        )
      }),
      () => {
        const current = this.selectedCableTemplate();
        return current === null
          ? { ok: false, error: "cable template is not selected" }
          : this.bridge.updateCableTemplate(current, this.preferredSpanIds());
      }
    );
  }

  commitCableTemplate(template: CableTemplateInfo): void {
    if (this.consumeCancelledCommit()) return;
    this.finishInteractionBeforeCommit();
    this.store.update((current) => ({
      ...current,
      cableTemplates: current.cableTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.bridge.updateCableTemplate(template, this.preferredSpanIds());
    this.finishTemplateOperation(result, `cable template ${template.id} updated`);
  }

  commitBundleTemplate(template: BundleTemplateInfo): void {
    if (this.consumeCancelledCommit()) return;
    this.finishInteractionBeforeCommit();
    this.store.update((current) => ({
      ...current,
      bundleTemplates: current.bundleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.bridge.updateBundleTemplate(template);
    this.finishTemplateOperation(result, `bundle template ${template.id} updated`);
  }

  previewBundleTemplate(
    template: BundleTemplateInfo,
    controlId: string,
    param: string,
    startValue: number
  ): void {
    const before = this.readSnapshot();
    if (before.interaction === null) {
      const original = before.bundleTemplates.find(
        (candidate) => candidate.id === template.id
      );
      if (original === undefined) {
        this.store.setError("bundle template is not selected");
        return;
      }
      this.store.update((current) => ({
        ...current,
        interaction: { controlId, param, startValue }
      }));
      this.interactionFrames = [];
      this.interactionActive = true;
      this.activeCancel = () => {
        this.store.update((current) => ({
          ...current,
          bundleTemplates: current.bundleTemplates.map((candidate) =>
            candidate.id === original.id ? original : candidate
          ),
          interaction: null
        }));
        const rollback = this.bridge.updateBundleTemplate(original);
        if (rollback.ok) this.refreshScene();
        else this.store.setError(rollback.error);
      };
    }
    this.store.update((current) => ({
      ...current,
      bundleTemplates: current.bundleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    this.clearPendingPreview();
    this.pendingPreview = setTimeout(() => {
      this.pendingPreview = null;
      const current = this.selectedBundleTemplate();
      const result =
        current === null
          ? { ok: false, error: "bundle template is not selected" }
          : this.bridge.updateBundleTemplate(current);
      if (!result.ok) {
        const error = result.error;
        this.cancel();
        this.store.setError(error);
        return;
      }
      this.refreshScene();
    }, 33);
  }

  applyRelatedPoleType(bundleTemplateId: number): void {
    this.finishTemplateOperation(
      this.bridge.applyRelatedPoleType(bundleTemplateId),
      `bundle ${bundleTemplateId} related pole type applied`
    );
  }

  previewPoleDefaultHeight(value: number): void {
    const selected = this.selectedPoleTemplate();
    if (selected === null) {
      this.store.setError("pole template is not selected");
      return;
    }
    this.previewSetting(
      "pole.defaultHeight",
      "defaultHeight",
      value,
      67,
      () => selected.defaultHeight,
      (current, next) => ({
        ...current,
        poleTemplates: current.poleTemplates.map((template) =>
          template.id === selected.id ? { ...template, defaultHeight: next } : template
        )
      }),
      () => {
        const current = this.selectedPoleTemplate();
        return current === null
          ? { ok: false, error: "pole template is not selected" }
          : this.bridge.updatePoleTemplate(current);
      }
    );
  }

  previewPoleTemplate(
    template: PoleTemplateInfo,
    controlId: string,
    param: string,
    startValue: number
  ): void {
    const before = this.readSnapshot();
    if (before.interaction === null) {
      const original = before.poleTemplates.find(
        (candidate) => candidate.id === template.id
      );
      if (original === undefined) {
        this.store.setError("pole template is not selected");
        return;
      }
      this.store.update((current) => ({
        ...current,
        interaction: { controlId, param, startValue }
      }));
      this.interactionFrames = [];
      this.interactionActive = true;
      this.activeCancel = () => {
        this.store.update((current) => ({
          ...current,
          poleTemplates: current.poleTemplates.map((candidate) =>
            candidate.id === original.id ? original : candidate
          ),
          interaction: null
        }));
        const rollback = this.bridge.updatePoleTemplate(original);
        if (rollback.ok) this.refreshScene();
        else this.store.setError(rollback.error);
      };
    }
    this.store.update((current) => ({
      ...current,
      poleTemplates: current.poleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    this.clearPendingPreview();
    this.pendingPreview = setTimeout(() => {
      this.pendingPreview = null;
      const current = this.selectedPoleTemplate();
      const result =
        current === null
          ? { ok: false, error: "pole template is not selected" }
          : this.bridge.updatePoleTemplate(current);
      if (!result.ok) {
        const error = result.error;
        this.cancel();
        this.store.setError(error);
        return;
      }
      this.refreshScene();
    }, 67);
  }

  commitPoleTemplate(template: PoleTemplateInfo): void {
    if (this.consumeCancelledCommit()) return;
    this.finishInteractionBeforeCommit();
    this.store.update((current) => ({
      ...current,
      poleTemplates: current.poleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.bridge.updatePoleTemplate(template);
    this.finishTemplateOperation(result, `pole template ${template.id} updated`);
  }

  cancel(suppressBlurCommit = false): void {
    this.clearPendingPreview();
    this.activeCancel?.();
    this.activeCancel = null;
    this.finishFrameMeasurement();
    this.suppressNextCommit = suppressBlurCommit;
  }

  recordFrame(deltaMs: number): void {
    if (this.interactionActive && Number.isFinite(deltaMs)) {
      this.interactionFrames.push(deltaMs);
    }
  }

  recordSceneContentSync(stats: SceneContentSyncStats): void {
    this.pendingSceneSyncStats = stats;
  }

  private generatePoints(
    points: WorldPoint[],
    pointSpecs: Array<PathPointSpec | null>,
    fromPlacementEdit = false
  ): void {
    const before = this.readSnapshot();
    const placements = before.drawBundlePlacements;
    const bundleTemplates: BundleTemplateInfo[] = before.bundleTemplates;

    if (points.length < 2) {
      this.store.setError("path needs at least 2 points");
      return;
    }
    if (placements.length === 0) {
      this.store.setError("add at least one bundle placement");
      return;
    }
    const selectedTemplates = placements.map((placement) =>
      bundleTemplates.find((template) => template.id === placement.bundleTemplateId)
    );
    if (selectedTemplates.some((template) => template === undefined)) {
      this.store.setError("selected bundle template is not available");
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
    const result = this.bridge.generate(
      flatPoints,
      placements,
      before.clickedPointsOnly ? 0 : before.intervalM,
      before.selectedPoleTemplateId ?? 1,
      before.directionMode,
      before.maxTiltDeg,
      nodeSpecs
    );
    const generateEnd = performance.now();
    if (!result.ok) {
      this.reproTrace.recordGeneration(before, points, result);
      this.store.setError(result.error);
      return;
    }
    const sceneStart = performance.now();
    const scene = this.bridge.scene();
    const placementsWithGeneratedIds = placements.map((placement, index) => ({
      ...placement,
      generatedBundleId: result.generatedBundleIds?.[index] ?? placement.generatedBundleId
    }));
    this.store.update((current) => ({
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
    this.reproTrace.recordGeneration(before, points, result, this.readSnapshot());
    const sceneUpdateMs = performance.now() - sceneStart;
    const viewerUpdateMs = performance.now() - generateStart;
    const sceneStats = this.pendingSceneSyncStats;
    this.pendingSceneSyncStats = null;
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
    this.store.update((current) => ({
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

  private readSnapshot(): ViewerSnapshot {
    let snapshot!: ViewerSnapshot;
    const unsubscribe = this.store.value.subscribe((current) => {
      snapshot = current;
    });
    unsubscribe();
    return snapshot;
  }

  private previewSetting<T>(
    controlId: string,
    param: string,
    value: T,
    intervalMs: number,
    read: (current: ViewerSnapshot) => T,
    write: (current: ViewerSnapshot, value: T) => ViewerSnapshot,
    apply: () => OperationResult
  ): void {
    const before = this.readSnapshot();
    if (before.interaction === null) {
      const startValue = read(before);
      if (typeof startValue !== "number" && typeof startValue !== "boolean") {
        throw new Error("interaction value must be numeric or boolean");
      }
      this.store.update((current) => ({
        ...current,
        interaction: { controlId, param, startValue }
      }));
      this.interactionFrames = [];
      this.interactionActive = true;
      this.activeCancel = () => {
        this.store.update((current) => write(current, startValue));
        const result = apply();
        if (result.ok) {
          this.refreshScene();
        } else {
          this.store.setError(result.error);
        }
        this.store.update((current) => ({ ...current, interaction: null }));
      };
    }
    this.store.update((current) => write(current, value));
    this.clearPendingPreview();
    this.pendingPreview = setTimeout(() => {
      this.pendingPreview = null;
      const result = apply();
      if (!result.ok) {
        const error = result.error;
        this.cancel();
        this.store.setError(error);
        return;
      }
      this.refreshScene();
    }, intervalMs);
  }

  private commitSetting<T>(
    controlId: string,
    value: T,
    write: (current: ViewerSnapshot, value: T) => ViewerSnapshot,
    apply: () => OperationResult,
    recover: () => void
  ): void {
    if (this.consumeCancelledCommit()) return;
    this.clearPendingPreview();
    this.store.update((current) => write(current, value));
    const result = apply();
    this.activeCancel = null;
    this.store.update((current) => ({ ...current, interaction: null }));
    this.finishFrameMeasurement();
    if (!result.ok) {
      recover();
    }
    this.finishOperation(result, `${controlId}=${String(value)}`);
  }

  private consumeCancelledCommit(): boolean {
    if (!this.suppressNextCommit) return false;
    this.suppressNextCommit = false;
    return true;
  }

  private finishOperation(result: OperationResult, log: string): void {
    if (!result.ok) {
      this.reproTrace.recordFailure(log, result.error);
      this.store.setError(result.error);
      return;
    }
    this.refreshScene();
    this.store.update((current) => ({
      ...current,
      error: "",
      logs: [...current.logs, log]
    }));
    this.reproTrace.recordOperation(log, this.readSnapshot());
  }

  private finishTemplateOperation(result: OperationResult, log: string): void {
    if (!result.ok) {
      this.reproTrace.recordFailure(log, result.error);
      this.refreshCatalogs();
      this.store.setError(result.error);
      return;
    }
    this.refreshCatalogs();
    this.finishOperation(result, log);
  }

  private selectedCableTemplate(): CableTemplateInfo | null {
    const current = this.readSnapshot();
    return (
      current.cableTemplates.find(
        (template) => template.id === current.selectedCableTemplateId
      ) ?? null
    );
  }

  private selectedBundleTemplate(): BundleTemplateInfo | null {
    const current = this.readSnapshot();
    return (
      current.bundleTemplates.find(
        (template) => template.id === current.selectedBundleTemplateId
      ) ?? null
    );
  }

  private selectedPoleTemplate(): PoleTemplateInfo | null {
    const current = this.readSnapshot();
    return (
      current.poleTemplates.find(
        (template) => template.id === current.selectedPoleTemplateId
      ) ?? null
    );
  }

  private refreshCatalogs(): void {
    const bundleTemplates = this.bridge.bundleTemplates();
    const cableTemplates = this.bridge.cableTemplates();
    const poleTemplates = this.bridge.poleTemplates();
    this.store.update((current) => ({
      ...current,
      bundleTemplates,
      cableTemplates,
      poleTemplates
    }));
  }

  private preferredSpanIds(): string[] {
    return [
      ...new Set(
        this.readSnapshot()
          .parts.map((part) => part.info.sourceSpanId)
          .filter((id) => id !== "0")
      )
    ];
  }

  private refreshScene(): void {
    const scene = this.bridge.scene();
    this.store.update((current) => ({
      ...current,
      parts: scene.parts,
      models: scene.models,
      poles: scene.poles,
      ports: scene.ports,
      spans: scene.spans,
      supportNodes: scene.supportNodes,
      backboneEdges: scene.backboneEdges,
      generationMs: scene.lastGenerationTiming?.totalMs ?? current.generationMs,
      generationTiming: scene.lastGenerationTiming ?? current.generationTiming
    }));
  }

  private restoreWorkspacePreferences(preferences: WorkspacePreferences): void {
    this.store.update((current) => {
      const legacyIds = preferences.selectedDrawBundleTemplateIds ?? [];
      const placements = preferences.drawBundlePlacements ?? legacyIds.map((bundleTemplateId, index) => {
        const template = current.bundleTemplates.find((item) => item.id === bundleTemplateId);
        return {
          id: index + 1,
          bundleTemplateId,
          count: preferences.drawBundleCounts?.[bundleTemplateId] ?? template?.defaultCount ?? 1,
          explicit: true,
          ...(template === undefined
            ? { height: 0, offset: 0, spacing: 0.2 }
            : bundlePlacementDefault(
                template,
                current.poleTemplates.find((item) => item.id === current.selectedPoleTemplateId)
              ))
        };
      });
      const transientZeroDefaults = preferences.drawBundlePlacements !== undefined &&
        placementUsesTransientZeroDefaults(preferences.drawBundlePlacements);
      const normalizedPlacements = placements.map((placement) => {
        const legacy = placement as BundlePlacement & {
          heightOffset?: number;
          lateralOffset?: number;
        };
        const template = current.bundleTemplates.find(
          (item) => item.id === placement.bundleTemplateId
        );
        const defaults = template === undefined
          ? { height: 0, offset: 0, spacing: placement.spacing }
          : bundlePlacementDefault(
              template,
              current.poleTemplates.find((item) => item.id === current.selectedPoleTemplateId)
            );
        return {
          id: placement.id,
          bundleTemplateId: placement.bundleTemplateId,
          count: placement.count,
          explicit: legacy.explicit ?? true,
          height: transientZeroDefaults
            ? defaults.height
            : placement.height ?? defaults.height + (legacy.heightOffset ?? 0),
          offset: transientZeroDefaults
            ? defaults.offset
            : placement.offset ?? defaults.offset + (legacy.lateralOffset ?? 0),
          spacing: placement.spacing
        };
      });
      const uniqueTemplates = new Set(
        normalizedPlacements.map((placement) => placement.bundleTemplateId)
      ).size === normalizedPlacements.length;
      const orderedPlacements = uniqueTemplates
        ? [...normalizedPlacements].sort((a, b) => b.height - a.height)
        : normalizedPlacements;
      return {
        ...current,
        ...preferences,
        drawBundlePlacements: orderedPlacements.length > 0
          ? orderedPlacements
          : current.drawBundlePlacements,
        showGroundGrid: preferences.showGroundGrid ?? current.showGroundGrid,
        pathPoints: [],
        pathPointSpecs: []
      };
    });
  }

  private startWorkspacePersistence(): void {
    if (this.workspaceCache === null || this.workspaceSubscription !== null) return;
    this.workspaceSubscription = this.store.value.subscribe(() => {
      if (this.persistencePaused) return;
      this.clearWorkspaceSaveTimer();
      this.workspaceSaveTimer = setTimeout(() => {
        this.workspaceSaveTimer = null;
        void this.flushWorkspaceCache();
      }, 250);
    });
  }

  private clearWorkspaceSaveTimer(): void {
    if (this.workspaceSaveTimer !== null) {
      clearTimeout(this.workspaceSaveTimer);
      this.workspaceSaveTimer = null;
    }
  }

  private clearPendingPreview(): void {
    if (this.pendingPreview !== null) {
      clearTimeout(this.pendingPreview);
      this.pendingPreview = null;
    }
  }

  private finishInteractionBeforeCommit(): void {
    this.clearPendingPreview();
    this.activeCancel = null;
    this.store.update((current) => ({ ...current, interaction: null }));
    this.finishFrameMeasurement();
  }

  private finishFrameMeasurement(): void {
    if (!this.interactionActive) {
      return;
    }
    const frames = this.interactionFrames;
    this.interactionActive = false;
    this.interactionFrames = [];
    this.store.update((current) => ({
      ...current,
      lastInteractionFrames: {
        sampleCount: frames.length,
        maxFrameMs: frames.length === 0 ? 0 : Math.max(...frames),
        longFrameCount: frames.filter((delta) => delta > 34).length
      }
    }));
  }
}
