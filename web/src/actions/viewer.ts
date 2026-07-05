import type { WireBridge } from "../bridge/wire";
import type {
  BundleTemplateInfo,
  CableTemplateInfo,
  GeometrySettings,
  LayoutSettings,
  OperationResult,
  PoleTemplateInfo,
  VisualSettings
} from "../model";
import type {
  SelectionKind,
  ViewerSnapshot,
  ViewerStore,
  WorldPoint
} from "../store/viewer";

export class ViewerActions {
  private pendingPreview: ReturnType<typeof setTimeout> | null = null;
  private activeCancel: (() => void) | null = null;
  private interactionFrames: number[] = [];
  private interactionActive = false;

  constructor(
    private readonly bridge: WireBridge,
    private readonly store: ViewerStore
  ) {}

  initialize(): void {
    const bundleTemplates = this.bridge.bundleTemplates();
    const cableTemplates = this.bridge.cableTemplates();
    const poleTemplates = this.bridge.poleTemplates();
    const geometry = this.bridge.geometrySettings();
    if (!geometry.sagEnabled) {
      geometry.sagEnabled = true;
      const result = this.bridge.updateGeometrySettings(geometry);
      if (!result.ok) {
        this.store.setError(result.error);
      }
    }
    const defaultBundleId =
      bundleTemplates.find((template) => template.id === 1)?.id ??
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
      selectedDrawBundleTemplateIds:
        current.selectedDrawBundleTemplateIds.length > 0
          ? current.selectedDrawBundleTemplateIds
          : bundleTemplates
              .filter((template) => [0, 1, 2, 3].includes(template.id))
              .map((template) => template.id),
      drawBundleCounts: Object.fromEntries(
        bundleTemplates.map((template) => [
          template.id,
          current.drawBundleCounts[template.id] ?? template.defaultCount
        ])
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
  }

  addPathPoint(point: WorldPoint): void {
    this.store.update((current) => ({
      ...current,
      pathPoints: [...current.pathPoints, point],
      error: ""
    }));
  }

  clearPath(): void {
    this.store.update((current) => ({ ...current, pathPoints: [], error: "" }));
  }

  undoPathPoint(): void {
    this.store.update((current) => ({
      ...current,
      pathPoints: current.pathPoints.slice(0, -1),
      error: ""
    }));
  }

  setDrawOption(
    param:
      | "cameraFov"
      | "showBackboneOverlay"
      | "showPreview"
      | "keepPathAfterGenerate"
      | "drawPlaneZ"
      | "intervalM"
      | "clickedPointsOnly"
      | "directionMode"
      | "solidSupportRender"
      | "selectionIncludePoles"
      | "selectionIncludeMidair"
      | "selectionIncludeSpans"
      | "showWorkspace"
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

  toggleDrawBundleTemplate(id: number): void {
    this.store.update((current) => ({
      ...current,
      selectedDrawBundleTemplateIds: current.selectedDrawBundleTemplateIds.includes(id)
        ? current.selectedDrawBundleTemplateIds.filter((candidate) => candidate !== id)
        : [...current.selectedDrawBundleTemplateIds, id]
    }));
  }

  setDrawBundleCount(id: number, count: number): void {
    this.store.update((current) => ({
      ...current,
      drawBundleCounts: { ...current.drawBundleCounts, [id]: count }
    }));
  }

  selectCableTemplate(id: number): void {
    this.store.update((current) => ({ ...current, selectedCableTemplateId: id }));
  }

  selectPoleTemplate(id: number): void {
    this.store.update((current) => ({ ...current, selectedPoleTemplateId: id }));
  }

  generatePath(): void {
    let points: WorldPoint[] = [];
    const unsubscribe = this.store.value.subscribe((current) => {
      points = current.pathPoints;
    });
    unsubscribe();
    this.generatePoints(points);
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
      String(param),
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
      String(param),
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
      String(param),
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
    this.store.update((current) => ({
      ...current,
      bundleTemplates: current.bundleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.bridge.updateBundleTemplate(template);
    this.finishTemplateOperation(result, `bundle template ${template.id} updated`);
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
    startValue: number,
    value: number
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

  cancel(): void {
    this.clearPendingPreview();
    this.activeCancel?.();
    this.activeCancel = null;
    this.finishFrameMeasurement();
  }

  recordFrame(deltaMs: number): void {
    if (this.interactionActive && Number.isFinite(deltaMs)) {
      this.interactionFrames.push(deltaMs);
    }
  }

  private generatePoints(points: WorldPoint[]): void {
    const actionStart = performance.now();
    const before = this.readSnapshot();
    const selectedBundleTemplateIds = before.selectedDrawBundleTemplateIds;
    const bundleTemplates: BundleTemplateInfo[] = before.bundleTemplates;

    if (points.length < 2) {
      this.store.setError("path needs at least 2 points");
      return;
    }
    if (selectedBundleTemplateIds.length === 0) {
      this.store.setError("select at least one bundle template");
      return;
    }
    const selectedTemplates = selectedBundleTemplateIds.map((id) =>
      bundleTemplates.find((template) => template.id === id)
    );
    if (selectedTemplates.some((template) => template === undefined)) {
      this.store.setError("selected bundle template is not available");
      return;
    }
    const flatPoints = new Float64Array(points.length * 3);
    points.forEach((point, index) => flatPoints.set(point, index * 3));
    const result = this.bridge.generate(
      flatPoints,
      selectedTemplates.map((template) => template!.id),
      before.clickedPointsOnly ? 0 : before.intervalM,
      before.selectedPoleTemplateId ?? 1,
      selectedTemplates.map((template) =>
        template!.fixedCount
          ? 0
          : before.drawBundleCounts[template!.id] ?? template!.defaultCount
      ),
      before.directionMode
    );
    if (!result.ok) {
      this.store.setError(result.error);
      return;
    }

    const scene = this.bridge.scene();
    this.store.update((current) => ({
      ...current,
      parts: scene.parts,
      poles: scene.poles,
      ports: scene.ports,
      spans: scene.spans,
      supportNodes: scene.supportNodes,
      error: "",
      generationMs: result.totalMs,
      pathPoints: before.keepPathAfterGenerate ? points : [],
      bundleTemplates,
      selectedDrawBundleTemplateIds: selectedBundleTemplateIds
    }));
    const sceneUpdateMs = performance.now() - actionStart;
    this.store.update((current) => ({
      ...current,
      sceneUpdateMs,
      logs: [
        ...current.logs,
        `route generated: ${result.generatedPoleCount} poles / ${result.generatedSpanCount} spans`
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
    param: string,
    value: T,
    write: (current: ViewerSnapshot, value: T) => ViewerSnapshot,
    apply: () => OperationResult,
    recover: () => void
  ): void {
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

  private finishOperation(result: OperationResult, log: string): void {
    if (!result.ok) {
      this.store.setError(result.error);
      return;
    }
    this.refreshScene();
    this.store.update((current) => ({
      ...current,
      error: "",
      logs: [...current.logs, log]
    }));
  }

  private finishTemplateOperation(result: OperationResult, log: string): void {
    if (!result.ok) {
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
      poles: scene.poles,
      ports: scene.ports,
      spans: scene.spans,
      supportNodes: scene.supportNodes
    }));
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
