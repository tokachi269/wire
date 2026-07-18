import type { ViewerActionContext } from "./context";
import {
  bundlePlacementDefault,
  defaultBundlePlacements,
  placementUsesTransientZeroDefaults
} from "./draw_defaults";
import { createViewerSnapshot } from "../store/viewer";
import type { WorkspacePreferences } from "../store/workspace";

export class WorkspaceActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  initialize(): void {
    const bundleTemplates = this.ctx.bridge.bundleTemplates();
    const cableTemplates = this.ctx.bridge.cableTemplates();
    const poleTemplates = this.ctx.bridge.poleTemplates();
    const defaultBundleId =
      bundleTemplates.find((template) => template.category === 0)?.id ??
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
      geometry: this.ctx.bridge.geometrySettings(),
      layout: this.ctx.bridge.layoutSettings(),
      visual: this.ctx.bridge.visualSettings()
    }));
    this.ctx.factoryCoreState = this.ctx.bridge.saveState();
  }

  async restoreWorkspace(): Promise<void> {
    const cached = await (this.ctx.workspaceCache?.read() ?? Promise.resolve(null));
    if (cached !== null) {
      const result = this.ctx.bridge.loadState(cached.coreState);
      if (result.ok) {
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
      } else {
        this.ctx.persistencePaused = true;
        this.ctx.store.setError(`Workspace restore failed: ${result.error}`);
        return;
      }
    }
    this.ctx.persistencePaused = false;
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

  async resetWorkspace(): Promise<void> {
    if (this.ctx.factoryCoreState.length === 0) {
      this.ctx.store.setError("Workspace reset is unavailable before initialization");
      return;
    }
    this.ctx.persistencePaused = true;
    this.ctx.clearWorkspaceSaveTimer();
    await this.ctx.workspaceCache?.clear();
    const result = this.ctx.bridge.loadState(this.ctx.factoryCoreState);
    if (!result.ok) {
      this.ctx.persistencePaused = false;
      this.ctx.store.setError(`Workspace reset failed: ${result.error}`);
      return;
    }
    this.ctx.store.replace(createViewerSnapshot());
    this.initialize();
    await this.restoreWorkspace();
    this.ctx.refreshScene();
    this.ctx.store.update((current) => ({
      ...current,
      logs: [...current.logs, "Workspace reset"]
    }));
    await this.flushWorkspaceCache();
  }

  async flushWorkspaceCache(): Promise<void> {
    if (this.ctx.workspaceCache === null || this.ctx.persistencePaused) return;
    this.ctx.clearWorkspaceSaveTimer();
    try {
      await this.ctx.workspaceCache.write(this.ctx.bridge.saveState(), this.ctx.readSnapshot());
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.ctx.persistencePaused = true;
      this.ctx.store.setError(`Workspace cache failed: ${message}`);
    }
  }

  dispose(): void {
    void this.flushWorkspaceCache();
    this.ctx.workspaceSubscription?.();
    this.ctx.workspaceSubscription = null;
    this.ctx.clearWorkspaceSaveTimer();
  }

  private restoreWorkspacePreferences(preferences: WorkspacePreferences): void {
    this.ctx.store.update((current) => {
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
        const legacy = placement as import("../model").BundlePlacement & {
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
    if (this.ctx.workspaceCache === null || this.ctx.workspaceSubscription !== null) return;
    this.ctx.workspaceSubscription = this.ctx.store.value.subscribe(() => {
      if (this.ctx.persistencePaused) return;
      this.ctx.clearWorkspaceSaveTimer();
      this.ctx.workspaceSaveTimer = setTimeout(() => {
        this.ctx.workspaceSaveTimer = null;
        void this.flushWorkspaceCache();
      }, 250);
    });
  }
}
