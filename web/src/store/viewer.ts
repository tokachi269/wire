import { writable, type Readable } from "svelte/store";
import type { BundleTemplateInfo, PoleInfo, VisualPartInfo } from "../bridge/wasm";

export type WorldPoint = [number, number, number];

export interface VisualPart {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface ViewerSnapshot {
  parts: VisualPart[];
  poles: PoleInfo[];
  error: string;
  generationMs: number | null;
  pathPoints: WorldPoint[];
  bundleTemplates: BundleTemplateInfo[];
  selectedBundleTemplateId: number | null;
}

const initialSnapshot: ViewerSnapshot = {
  parts: [],
  poles: [],
  error: "",
  generationMs: null,
  pathPoints: [],
  bundleTemplates: [],
  selectedBundleTemplateId: null
};

export class ViewerStore {
  readonly value: Readable<ViewerSnapshot>;
  private readonly writable = writable<ViewerSnapshot>(initialSnapshot);

  constructor() {
    this.value = { subscribe: this.writable.subscribe };
  }

  replace(snapshot: ViewerSnapshot): void {
    this.writable.set(snapshot);
  }

  setError(error: string): void {
    this.writable.update((current) => ({ ...current, error }));
  }

  update(change: (current: ViewerSnapshot) => ViewerSnapshot): void {
    this.writable.update(change);
  }
}
