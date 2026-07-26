import type { ViewerActionContext } from "./context";
import type { GeometrySettings, LayoutSettings, VisualSettings } from "../model";

export class SettingsActions {
  constructor(private readonly ctx: ViewerActionContext) {}

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
    this.ctx.store.update((current) => ({ ...current, [param]: value }));
  }

  previewGeometry<K extends keyof GeometrySettings>(
    param: K,
    value: GeometrySettings[K]
  ): void {
    this.ctx.previewSetting(
      `geometry.${String(param)}`,
      String(param),
      value,
      33,
      (current) => current.geometry[param],
      (current, next) => ({ ...current, geometry: { ...current.geometry, [param]: next } }),
      () => this.ctx.bridge.updateGeometrySettings(this.ctx.readSnapshot().geometry)
    );
  }

  commitGeometry<K extends keyof GeometrySettings>(
    param: K,
    value: GeometrySettings[K]
  ): void {
    this.ctx.commitSetting(
      `geometry.${String(param)}`,
      value,
      (current, next) => ({ ...current, geometry: { ...current.geometry, [param]: next } }),
      () => this.ctx.bridge.updateGeometrySettings(this.ctx.readSnapshot().geometry),
      () =>
        this.ctx.store.update((current) => ({
          ...current,
          geometry: this.ctx.bridge.geometrySettings()
        }))
    );
  }

  commitLayout<K extends keyof LayoutSettings>(
    param: K,
    value: LayoutSettings[K]
  ): void {
    this.ctx.commitSetting(
      `layout.${String(param)}`,
      value,
      (current, next) => ({ ...current, layout: { ...current.layout, [param]: next } }),
      () => this.ctx.bridge.updateLayoutSettings(this.ctx.readSnapshot().layout),
      () =>
        this.ctx.store.update((current) => ({
          ...current,
          layout: this.ctx.bridge.layoutSettings()
        }))
    );
  }

  previewVisual<K extends keyof VisualSettings>(
    param: K,
    value: VisualSettings[K]
  ): void {
    this.ctx.previewSetting(
      `visual.${String(param)}`,
      String(param),
      value,
      33,
      (current) => current.visual[param],
      (current, next) => ({ ...current, visual: { ...current.visual, [param]: next } }),
      () => this.ctx.bridge.updateVisualSettings(this.ctx.readSnapshot().visual)
    );
  }

  commitVisual<K extends keyof VisualSettings>(
    param: K,
    value: VisualSettings[K]
  ): void {
    this.ctx.commitSetting(
      `visual.${String(param)}`,
      value,
      (current, next) => ({ ...current, visual: { ...current.visual, [param]: next } }),
      () => this.ctx.bridge.updateVisualSettings(this.ctx.readSnapshot().visual),
      () =>
        this.ctx.store.update((current) => ({
          ...current,
          visual: this.ctx.bridge.visualSettings()
        }))
    );
  }
}
