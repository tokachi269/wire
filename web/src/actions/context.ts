import { ReproTrace } from "./reproTrace";
import type { WireBridge } from "../bridge/wire";
import type {
  BundleTemplateInfo,
  CableTemplateInfo,
  OperationResult,
  PoleTemplateInfo,
  SceneContentSyncStats
} from "../model";
import type { ViewerSnapshot, ViewerStore } from "../store/viewer";
import type { WorkspaceCache } from "../store/workspace";

export class ViewerActionContext {
  private pendingPreview: ReturnType<typeof setTimeout> | null = null;
  private activeCancel: (() => void) | null = null;
  private interactionFrames: number[] = [];
  private interactionActive = false;
  private suppressNextCommit = false;
  readonly reproTrace = new ReproTrace();
  private factoryCoreState = "";
  private workspaceSaveTimer: ReturnType<typeof setTimeout> | null = null;
  private workspaceSubscription: (() => void) | null = null;
  private persistencePaused = true;
  private pendingSceneSyncStats: SceneContentSyncStats | null = null;

  constructor(
    readonly bridge: WireBridge,
    readonly store: ViewerStore,
    readonly workspaceCache: WorkspaceCache | null = null
  ) {}

  readSnapshot(): ViewerSnapshot {
    let snapshot!: ViewerSnapshot;
    const unsubscribe = this.store.value.subscribe((current) => {
      snapshot = current;
    });
    unsubscribe();
    return snapshot;
  }

  refreshCatalogs(): void {
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

  refreshScene(): void {
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

  preferredSpanIds(): string[] {
    return [
      ...new Set(
        this.readSnapshot()
          .parts.map((part) => part.info.sourceSpanId)
          .filter((id) => id !== "0")
      )
    ];
  }

  selectedCableTemplate(): CableTemplateInfo | null {
    const current = this.readSnapshot();
    return (
      current.cableTemplates.find(
        (template) => template.id === current.selectedCableTemplateId
      ) ?? null
    );
  }

  selectedBundleTemplate(): BundleTemplateInfo | null {
    const current = this.readSnapshot();
    return (
      current.bundleTemplates.find(
        (template) => template.id === current.selectedBundleTemplateId
      ) ?? null
    );
  }

  selectedPoleTemplate(): PoleTemplateInfo | null {
    const current = this.readSnapshot();
    return (
      current.poleTemplates.find(
        (template) => template.id === current.selectedPoleTemplateId
      ) ?? null
    );
  }

  saveFactoryCoreState(): void {
    this.factoryCoreState = this.bridge.saveState();
  }

  hasFactoryCoreState(): boolean {
    return this.factoryCoreState.length > 0;
  }

  loadFactoryCoreState(): OperationResult {
    return this.bridge.loadState(this.factoryCoreState);
  }

  currentCoreState(): string {
    return this.bridge.saveState();
  }

  async readWorkspaceCache(): Promise<Awaited<ReturnType<WorkspaceCache["read"]>> | null> {
    return this.workspaceCache?.read() ?? null;
  }

  async clearWorkspaceCache(): Promise<void> {
    await this.workspaceCache?.clear();
  }

  async writeWorkspaceCache(): Promise<void> {
    if (this.workspaceCache === null) return;
    await this.workspaceCache.write(this.currentCoreState(), this.readSnapshot());
  }

  hasWorkspaceCache(): boolean {
    return this.workspaceCache !== null;
  }

  pausePersistence(): void {
    this.persistencePaused = true;
  }

  resumePersistence(): void {
    this.persistencePaused = false;
  }

  isPersistencePaused(): boolean {
    return this.persistencePaused;
  }

  subscribeWorkspacePersistence(callback: () => void): void {
    if (this.workspaceSubscription !== null) return;
    this.workspaceSubscription = this.store.value.subscribe(callback);
  }

  disposeWorkspacePersistence(): void {
    this.workspaceSubscription?.();
    this.workspaceSubscription = null;
    this.clearWorkspaceSaveTimer();
  }

  hasWorkspacePersistence(): boolean {
    return this.workspaceSubscription !== null;
  }

  scheduleWorkspaceSave(callback: () => void, delayMs: number): void {
    this.clearWorkspaceSaveTimer();
    this.workspaceSaveTimer = setTimeout(() => {
      this.workspaceSaveTimer = null;
      callback();
    }, delayMs);
  }

  beginInteraction(
    controlId: string,
    param: string,
    startValue: number | boolean,
    cancel: () => void
  ): void {
    this.store.update((current) => ({
      ...current,
      interaction: { controlId, param, startValue }
    }));
    this.interactionFrames = [];
    this.interactionActive = true;
    this.activeCancel = cancel;
  }

  schedulePreview(callback: () => void, intervalMs: number): void {
    this.clearPendingPreview();
    this.pendingPreview = setTimeout(() => {
      this.pendingPreview = null;
      callback();
    }, intervalMs);
  }

  consumeSceneContentSyncStats(): SceneContentSyncStats | null {
    const stats = this.pendingSceneSyncStats;
    this.pendingSceneSyncStats = null;
    return stats;
  }

  previewSetting<T>(
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
      this.beginInteraction(controlId, param, startValue, () => {
        this.store.update((current) => write(current, startValue));
        const result = apply();
        if (result.ok) {
          this.refreshScene();
        } else {
          this.store.setError(result.error);
        }
        this.store.update((current) => ({ ...current, interaction: null }));
      });
    }
    this.store.update((current) => write(current, value));
    this.schedulePreview(() => {
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

  commitSetting<T>(
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

  consumeCancelledCommit(): boolean {
    if (!this.suppressNextCommit) return false;
    this.suppressNextCommit = false;
    return true;
  }

  finishOperation(result: OperationResult, log: string): void {
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

  finishTemplateOperation(result: OperationResult, log: string): void {
    if (!result.ok) {
      this.reproTrace.recordFailure(log, result.error);
      this.refreshCatalogs();
      this.store.setError(result.error);
      return;
    }
    this.refreshCatalogs();
    this.finishOperation(result, log);
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

  private clearWorkspaceSaveTimer(): void {
    if (this.workspaceSaveTimer !== null) {
      clearTimeout(this.workspaceSaveTimer);
      this.workspaceSaveTimer = null;
    }
  }

  clearPendingPreview(): void {
    if (this.pendingPreview !== null) {
      clearTimeout(this.pendingPreview);
      this.pendingPreview = null;
    }
  }

  finishInteractionBeforeCommit(): void {
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
