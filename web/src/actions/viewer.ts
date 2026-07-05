import type { WireBridge } from "../bridge/wire";
import type { BundleTemplateInfo } from "../bridge/wasm";
import type { ViewerStore, WorldPoint } from "../store/viewer";

export class ViewerActions {
  constructor(
    private readonly bridge: WireBridge,
    private readonly store: ViewerStore
  ) {}

  initialize(): void {
    const bundleTemplates = this.bridge.bundleTemplates();
    this.store.update((current) => ({
      ...current,
      bundleTemplates,
      selectedBundleTemplateId:
        current.selectedBundleTemplateId ?? bundleTemplates[0]?.id ?? null
    }));
  }

  generateSample(): void {
    this.generatePoints([
      [0, 0, 0],
      [16, 2, 0],
      [32, 0, 0]
    ]);
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

  selectBundleTemplate(id: number): void {
    this.store.update((current) => ({
      ...current,
      selectedBundleTemplateId: id
    }));
  }

  generatePath(): void {
    let points: WorldPoint[] = [];
    const unsubscribe = this.store.value.subscribe((current) => {
      points = current.pathPoints;
    });
    unsubscribe();
    this.generatePoints(points);
  }

  private generatePoints(points: WorldPoint[]): void {
    let selectedBundleTemplateId: number | null = null;
    let bundleTemplates: BundleTemplateInfo[] = [];
    const unsubscribe = this.store.value.subscribe((current) => {
      selectedBundleTemplateId = current.selectedBundleTemplateId;
      bundleTemplates = current.bundleTemplates;
    });
    unsubscribe();

    if (points.length < 2) {
      this.store.setError("path needs at least 2 points");
      return;
    }
    if (selectedBundleTemplateId === null) {
      this.store.setError("bundle template is not selected");
      return;
    }
    const selectedTemplate = bundleTemplates.find(
      (template) => template.id === selectedBundleTemplateId
    );
    if (selectedTemplate === undefined) {
      this.store.setError("selected bundle template is not available");
      return;
    }

    const flatPoints = new Float64Array(points.length * 3);
    points.forEach((point, index) => flatPoints.set(point, index * 3));
    const result = this.bridge.generate(
      flatPoints,
      selectedTemplate.id,
      0,
      1,
      selectedTemplate.fixedCount ? 0 : selectedTemplate.defaultCount
    );
    if (!result.ok) {
      this.store.setError(result.error);
      return;
    }

    const scene = this.bridge.scene();
    this.store.replace({
      parts: scene.parts,
      poles: scene.poles,
      error: "",
      generationMs: result.totalMs,
      pathPoints: points,
      bundleTemplates,
      selectedBundleTemplateId
    });
  }
}
