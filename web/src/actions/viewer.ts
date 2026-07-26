import type { WireBridge } from "../bridge/wire";
import type {
  BundlePlacement,
  BundleTemplateInfo,
  CableTemplateInfo,
  GeometrySettings,
  LayoutSettings,
  PathPickInfo,
  PoleTemplateInfo,
  SceneContentSyncStats,
  VisualSettings
} from "../model";
import type { SelectionKind, ViewerStore, WorldPoint } from "../store/viewer";
import type { WorkspaceCache } from "../store/workspace";
import { ViewerActionContext } from "./context";
import { DrawActions } from "./draw_actions";
import { SelectionActions } from "./selection_actions";
import type { RoadSnapInfo } from "../road";
import { RoadActions } from "./road_actions";
import { SettingsActions } from "./settings_actions";
import { TemplateActions } from "./template_actions";
import { WorkspaceActions } from "./workspace_actions";

export class ViewerActions {
  private readonly ctx: ViewerActionContext;
  private readonly draw: DrawActions;
  private readonly selection: SelectionActions;
  private readonly road: RoadActions;
  private readonly settings: SettingsActions;
  private readonly templates: TemplateActions;
  private readonly workspace: WorkspaceActions;

  constructor(
    bridge: WireBridge,
    store: ViewerStore,
    workspaceCache: WorkspaceCache | null = null
  ) {
    this.ctx = new ViewerActionContext(bridge, store, workspaceCache);
    this.draw = new DrawActions(this.ctx);
    this.selection = new SelectionActions(this.ctx);
    this.road = new RoadActions(this.ctx);
    this.settings = new SettingsActions(this.ctx);
    this.templates = new TemplateActions(this.ctx);
    this.workspace = new WorkspaceActions(this.ctx);
  }

  initialize(): void {
    this.workspace.initialize();
  }

  async restoreWorkspace(): Promise<void> {
    return this.workspace.restoreWorkspace();
  }

  addPathPoint(point: WorldPoint, pick?: PathPickInfo): void {
    this.draw.addPathPoint(point, pick);
  }

  setActiveTool(tool: "wire" | "road"): void {
    this.settings.setDrawOption("activeTool", tool);
  }

  addViewportPoint(point: WorldPoint, pick?: PathPickInfo | RoadSnapInfo): void {
    const roadSnap = pick !== undefined && "kind" in pick && pick.kind === "road" ? pick : undefined;
    if (this.ctx.readSnapshot().activeTool === "road") {
      this.road.addViewportPoint(point, roadSnap);
      return;
    }
    this.draw.addPathPoint(point, roadSnap === undefined ? pick as PathPickInfo | undefined : undefined);
  }

  previewViewportPoint(point: WorldPoint): void {
    if (this.ctx.readSnapshot().activeTool === "road") {
      this.road.previewViewportPoint(point);
    }
  }

  undoActiveTool(): void {
    if (this.ctx.readSnapshot().activeTool === "road") {
      this.road.undoOrCancel();
      return;
    }
    this.draw.undoPathPointOrClearSelection(() => this.selection.clearSelection());
  }

  clearActiveTool(): void {
    if (this.ctx.readSnapshot().activeTool === "road") {
      this.road.clear();
      return;
    }
    this.draw.clearPath();
  }

  clearPath(): void {
    this.draw.clearPath();
  }

  undoPathPoint(): void {
    this.draw.undoPathPoint();
  }

  undoPathPointOrClearSelection(): void {
    this.draw.undoPathPointOrClearSelection(() => this.selection.clearSelection());
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
      | "rightPanelMode"
      | "activeTool"
      | "workspaceLeftWidth"
      | "workspaceWidth",
    value: number | boolean | string
  ): void {
    this.settings.setDrawOption(param, value);
  }

  select(kind: SelectionKind, id: string): void {
    this.selection.select(kind, id);
  }

  clearSelection(): void {
    this.selection.clearSelection();
  }

  setRoadMode(mode: "line" | "bezier"): void {
    this.road.setMode(mode);
  }

  setRoadOperation(operation: import("../road").RoadOperation): void {
    this.road.setOperation(operation);
  }

  setRoadSetting<K extends keyof import("../road").RoadToolState>(
    key: K,
    value: import("../road").RoadToolState[K]
  ): void {
    this.road.setSetting(key, value);
  }

  updateSelectedRoadSectionTemplate(input: {
    sidewalkWidthM: number;
    laneWidthM: number;
    medianWidthM: number;
    hasCenterLine: boolean;
    hasOuterLines: boolean;
  }): void {
    this.road.updateSelectedSectionTemplate(input);
  }

  previewRoadEditHandle(handleIndex: number, point: WorldPoint): void {
    this.road.previewEditHandle(handleIndex, point);
  }

  commitRoadEditHandle(): void {
    this.road.commitEditHandle();
  }

  setRoadConnectToFirstNode(value: boolean): void {
    this.road.setConnectToFirstNode(value);
  }

  selectBundleTemplate(id: number): void {
    this.templates.selectBundleTemplate(id);
  }

  updateDrawBundlePlacement(id: number, change: Partial<Omit<BundlePlacement, "id">>): void {
    this.draw.updateDrawBundlePlacement(id, change);
  }

  duplicateDrawBundlePlacement(id: number): void {
    this.draw.duplicateDrawBundlePlacement(id);
  }

  selectCableTemplate(id: number): void {
    this.templates.selectCableTemplate(id);
  }

  selectPoleTemplate(id: number): void {
    this.templates.selectPoleTemplate(id);
  }

  exportReproCapture(): void {
    this.workspace.exportReproCapture();
  }

  exportWorkspaceText(): Promise<string> {
    return this.workspace.exportWorkspaceText();
  }

  exportWorkspaceFile(): Promise<void> {
    return this.workspace.exportWorkspaceFile();
  }

  importWorkspaceText(text: string): Promise<void> {
    return this.workspace.importWorkspaceText(text);
  }

  resetWorkspace(): Promise<void> {
    return this.workspace.resetWorkspace();
  }

  flushWorkspaceCache(): Promise<void> {
    return this.workspace.flushWorkspaceCache();
  }

  dispose(): void {
    this.workspace.dispose();
  }

  generatePath(): void {
    this.draw.generatePath();
  }

  previewGeometry<K extends keyof GeometrySettings>(
    param: K,
    value: GeometrySettings[K]
  ): void {
    this.settings.previewGeometry(param, value);
  }

  commitGeometry<K extends keyof GeometrySettings>(
    param: K,
    value: GeometrySettings[K]
  ): void {
    this.settings.commitGeometry(param, value);
  }

  commitLayout<K extends keyof LayoutSettings>(
    param: K,
    value: LayoutSettings[K]
  ): void {
    this.settings.commitLayout(param, value);
  }

  previewVisual<K extends keyof VisualSettings>(
    param: K,
    value: VisualSettings[K]
  ): void {
    this.settings.previewVisual(param, value);
  }

  commitVisual<K extends keyof VisualSettings>(
    param: K,
    value: VisualSettings[K]
  ): void {
    this.settings.commitVisual(param, value);
  }

  applyTiltToAll(maxTiltDeg: number): void {
    this.selection.applyTiltToAll(maxTiltDeg);
  }

  applyTiltToSelection(maxTiltDeg: number): void {
    this.selection.applyTiltToSelection(maxTiltDeg);
  }

  clearSelectedOverride(which: "pole" | "socketA" | "socketB" | "branchDown"): void {
    this.selection.clearSelectedOverride(which);
  }

  resetSpanReferenceLengths(): void {
    this.templates.resetSpanReferenceLengths();
  }

  previewCableTemplate<K extends keyof CableTemplateInfo>(
    param: K,
    value: CableTemplateInfo[K]
  ): void {
    this.templates.previewCableTemplate(param, value);
  }

  commitCableTemplate(template: CableTemplateInfo): void {
    this.templates.commitCableTemplate(template);
  }

  commitBundleTemplate(template: BundleTemplateInfo): void {
    this.templates.commitBundleTemplate(template);
  }

  previewBundleTemplate(
    template: BundleTemplateInfo,
    controlId: string,
    param: string,
    startValue: number
  ): void {
    this.templates.previewBundleTemplate(template, controlId, param, startValue);
  }

  applyRelatedPoleType(bundleTemplateId: number): void {
    this.templates.applyRelatedPoleType(bundleTemplateId);
  }

  previewPoleDefaultHeight(value: number): void {
    this.templates.previewPoleDefaultHeight(value);
  }

  previewPoleTemplate(
    template: PoleTemplateInfo,
    controlId: string,
    param: string,
    startValue: number
  ): void {
    this.templates.previewPoleTemplate(template, controlId, param, startValue);
  }

  commitPoleTemplate(template: PoleTemplateInfo): void {
    this.templates.commitPoleTemplate(template);
  }

  cancel(suppressBlurCommit = false): void {
    this.ctx.cancel(suppressBlurCommit);
  }

  recordFrame(deltaMs: number): void {
    this.ctx.recordFrame(deltaMs);
  }

  recordSceneContentSync(stats: SceneContentSyncStats): void {
    this.ctx.recordSceneContentSync(stats);
  }
}
