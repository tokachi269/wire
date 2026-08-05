import type { ViewerActionContext } from "./context";
import {
  bundlePlacementDefault,
  defaultBundlePlacements,
  placementUsesTransientZeroDefaults
} from "./draw_defaults";
import { DEFAULT_BUNDLE_PRESET } from "../profile/defaultBundlePreset";
import { seedRoadSections } from "../road_templates";
import { createViewerSnapshot } from "../store/viewer";
import {
  createWorkspaceDocument,
  parseWorkspaceDocument,
  serializeWorkspaceDocument,
  type WorkspacePreferences
} from "../store/workspace";

export class WorkspaceActions {
  // What the road sections registered for this workspace are called, and which
  // one a new road starts on. Reset restores the factory road state, which
  // already holds those sections, so it reuses these instead of registering
  // them a second time.
  private seededRoadSections: {
    labels: Record<number, string>;
    initialId: number;
  } | null = null;

  constructor(private readonly ctx: ViewerActionContext) {}

  // A new workspace starts with no road section at all, so the product
  // catalogue is registered once, here. Reopening a saved workspace goes
  // through loadState instead and keeps the sections it was saved with.
  initialize(): boolean {
    if (!this.seedRoadTemplates()) return false;
    this.initializeCatalogs();
    this.ctx.saveFactoryCoreState();
    return true;
  }

  private seedRoadTemplates(): boolean {
    const seeded = seedRoadSections((section) =>
      this.ctx.bridge.roadAddSectionTemplate(section)
    );
    if (!seeded.ok) {
      this.ctx.store.setError(`Workspace road sections failed: ${seeded.error}`);
      return false;
    }
    this.seededRoadSections = seeded.sections;
    this.applySeededRoadSections();
    return true;
  }

  private applySeededRoadSections(): void {
    const seeded = this.seededRoadSections;
    if (seeded === null) return;
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        sectionTemplateLabels: seeded.labels,
        selectedSectionTemplateId: seeded.initialId
      }
    }));
  }

  private initializeCatalogs(): void {
    const bundleTemplates = this.ctx.bridge.bundleTemplates();
    const cableTemplates = this.ctx.bridge.cableTemplates();
    const poleTemplates = this.ctx.bridge.poleTemplates();
    const presetBundleTemplateId = DEFAULT_BUNDLE_PRESET[0]?.bundleTemplateId;
    const defaultBundleId =
      bundleTemplates.find((template) => template.id === presetBundleTemplateId)?.id ??
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
    this.ctx.store.update((current) => ({
      ...current,
      bundleTemplates,
      selectedBundleTemplateId:
        current.selectedBundleTemplateId ?? defaultBundleId,
      drawBundlePlacements:
        current.drawBundlePlacements.length > 0
          ? current.drawBundlePlacements
          : defaultBundlePlacements(bundleTemplates),
      cableTemplates,
      selectedCableTemplateId:
        current.selectedCableTemplateId ?? defaultCableId,
      poleTemplates,
      selectedPoleTemplateId:
        current.selectedPoleTemplateId ?? defaultPoleId,
      geometry: this.ctx.bridge.geometrySettings(),
      layout: this.ctx.bridge.layoutSettings(),
      visual: this.ctx.bridge.visualSettings(),
      road: {
        ...current.road,
        scene: this.ctx.bridge.roadScene()
      }
    }));
  }

  async restoreWorkspace(): Promise<void> {
    const cached = await this.ctx.readWorkspaceCache();
    if (cached !== null) {
      const result = this.ctx.bridge.loadState(cached.coreState);
      const roadResult = result.ok && cached.roadState !== undefined
        ? this.ctx.bridge.roadLoadState(cached.roadState)
        : { ok: true, error: "" };
      if (result.ok && roadResult.ok) {
        this.ctx.refreshCatalogs();
        this.restoreWorkspacePreferences(cached.viewer);
        this.ctx.store.update((current) => ({
          ...current,
          geometry: this.ctx.bridge.geometrySettings(),
          layout: this.ctx.bridge.layoutSettings(),
          visual: this.ctx.bridge.visualSettings(),
          error: ""
        }));
        this.ctx.refreshScene();
        this.refreshRoadScene();
      } else {
        this.ctx.pausePersistence();
        this.ctx.store.setError(`Workspace restore failed: ${result.ok ? roadResult.error : result.error}`);
        return;
      }
    }
    this.ctx.resumePersistence();
    this.startWorkspacePersistence();
  }

  exportReproCapture(): void {
    const text = this.ctx.reproTrace.toText(this.ctx.readSnapshot());
    const url = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = `wire-repro-${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;
    document.body.append(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
    this.ctx.store.update((current) => ({
      ...current,
      logs: [...current.logs, "repro trace downloaded"]
    }));
  }

  async exportWorkspaceText(): Promise<string> {
    const document = createWorkspaceDocument(
      this.ctx.currentCoreState(),
      this.ctx.readSnapshot(),
      this.ctx.currentRoadState()
    );
    return serializeWorkspaceDocument(document);
  }

  async exportWorkspaceFile(): Promise<void> {
    const text = await this.exportWorkspaceText();
    const url = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = `wire-workspace-${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;
    document.body.append(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
    this.ctx.store.update((current) => ({
      ...current,
      logs: [...current.logs, "workspace downloaded"]
    }));
  }

  async importWorkspaceText(text: string): Promise<void> {
    const document = await parseWorkspaceDocument(text);
    if (document === null) {
      this.ctx.store.setError("Workspace import failed: invalid workspace document");
      return;
    }
    const result = this.ctx.bridge.loadState(document.coreState);
    if (!result.ok) {
      this.ctx.store.setError(`Workspace import failed: ${result.error}`);
      return;
    }
    if (document.roadState !== undefined) {
      const roadResult = this.ctx.bridge.roadLoadState(document.roadState);
      if (!roadResult.ok) {
        this.ctx.store.setError(`Workspace import failed: ${roadResult.error}`);
        return;
      }
    }
    this.ctx.refreshCatalogs();
    this.restoreWorkspacePreferences(document.viewer);
    this.ctx.store.update((current) => ({
      ...current,
      geometry: this.ctx.bridge.geometrySettings(),
      layout: this.ctx.bridge.layoutSettings(),
      visual: this.ctx.bridge.visualSettings(),
      error: ""
    }));
    this.ctx.refreshScene();
    this.refreshRoadScene();
    await this.flushWorkspaceCache();
  }

  async resetWorkspace(): Promise<void> {
    if (!this.ctx.hasFactoryCoreState()) {
      this.ctx.store.setError("Workspace reset is unavailable before initialization");
      return;
    }
    const beforeReset = this.ctx.readSnapshot();
    this.ctx.pausePersistence();
    await this.ctx.clearWorkspaceCache();
    const result = this.ctx.loadFactoryCoreState();
    if (!result.ok) {
      this.ctx.resumePersistence();
      this.ctx.store.setError(`Workspace reset failed: ${result.error}`);
      return;
    }
    this.ctx.store.replace({
      ...createViewerSnapshot(),
      showLeftPanel: beforeReset.showLeftPanel,
      showRightPanel: beforeReset.showRightPanel,
      workspaceLeftWidth: beforeReset.workspaceLeftWidth,
      workspaceWidth: beforeReset.workspaceWidth
    });
    // The factory road state already holds the registered sections.
    this.applySeededRoadSections();
    this.initializeCatalogs();
    this.ctx.refreshScene();
    this.refreshRoadScene();
    this.ctx.resumePersistence();
    this.ctx.store.update((current) => ({
      ...current,
      logs: [...current.logs, "Workspace reset"]
    }));
    await this.flushWorkspaceCache();
  }

  async flushWorkspaceCache(): Promise<void> {
    if (!this.ctx.hasWorkspaceCache() || this.ctx.isPersistencePaused()) return;
    try {
      await this.ctx.writeWorkspaceCache();
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.ctx.pausePersistence();
      this.ctx.store.setError(`Workspace cache failed: ${message}`);
    }
  }

  private refreshRoadScene(): void {
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((current) => ({
      ...current,
      road: { ...current.road, scene, previewMeshes: [], phase: "start", lastError: "" }
    }));
  }

  dispose(): void {
    void this.flushWorkspaceCache();
    this.ctx.disposeWorkspacePersistence();
  }

  private restoreWorkspacePreferences(preferences: WorkspacePreferences): void {
    let defaultPlacementError = "";
    const resolveDefault = this.ctx.bridge.resolveDefaultBundlePlacement.bind(this.ctx.bridge);
    this.ctx.store.update((current) => {
      const legacyIds = preferences.selectedDrawBundleTemplateIds ?? [];
      const placements = preferences.drawBundlePlacements ?? legacyIds.map((bundleTemplateId, index) => {
        const template = current.bundleTemplates.find((item) => item.id === bundleTemplateId);
        const count = preferences.drawBundleCounts?.[bundleTemplateId] ?? template?.defaultCount ?? 1;
        return {
          id: index + 1,
          bundleTemplateId,
          count,
          explicit: true,
          ...(template === undefined
            ? { height: 0, offset: 0, spacing: 0.2 }
            : (() => {
                try {
                  return bundlePlacementDefault(
                    template,
                    current.selectedPoleTemplateId,
                    count,
                    resolveDefault
                  );
                } catch (error) {
                  defaultPlacementError = error instanceof Error ? error.message : String(error);
                  return { height: 0, offset: 0, spacing: template.defaultSpacing };
                }
              })())
        };
      });
      const transientZeroDefaults = preferences.drawBundlePlacements !== undefined &&
        placementUsesTransientZeroDefaults(preferences.drawBundlePlacements);
      const normalizedPlacements = placements.map((placement) => {
        const legacy = placement as import("../model").BundlePlacement & {
          heightOffset?: number;
          lateralOffset?: number;
        };
        const template = current.bundleTemplates.find(
          (item) => item.id === placement.bundleTemplateId
        );
        const defaults = template === undefined
          ? { height: 0, offset: 0, spacing: placement.spacing }
          : (() => {
              try {
                return bundlePlacementDefault(
                  template,
                  current.selectedPoleTemplateId,
                  placement.count,
                  resolveDefault
                );
              } catch (error) {
                defaultPlacementError = error instanceof Error ? error.message : String(error);
                return { height: 0, offset: 0, spacing: placement.spacing };
              }
            })();
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
        pathPointSpecs: [],
        wirePreview: { state: "none", request: null }
      };
    });
    if (defaultPlacementError.length > 0) {
      this.ctx.store.setError(`Workspace placement defaults failed: ${defaultPlacementError}`);
    }
  }

  private startWorkspacePersistence(): void {
    if (!this.ctx.hasWorkspaceCache() || this.ctx.hasWorkspacePersistence()) return;
    this.ctx.subscribeWorkspacePersistence(() => {
      if (this.ctx.isPersistencePaused()) return;
      this.ctx.scheduleWorkspaceSave(() => {
        void this.flushWorkspaceCache();
      }, 250);
    });
  }
}
