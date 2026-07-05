import { writable, type Readable } from "svelte/store";
import type {
  BundleTemplateInfo,
  CableTemplateInfo,
  GeometrySettings,
  LayoutSettings,
  PoleInfo,
  PoleTemplateInfo,
  PortInfo,
  SpanInfo,
  SupportNodeInfo,
  VisualPartInfo,
  VisualSettings
} from "../model";

export type WorldPoint = [number, number, number];
export type SelectionKind = "pole" | "port" | "span" | "supportNode";

export interface VisualPart {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface ViewerSnapshot {
  parts: VisualPart[];
  poles: PoleInfo[];
  ports: PortInfo[];
  spans: SpanInfo[];
  supportNodes: SupportNodeInfo[];
  error: string;
  generationMs: number | null;
  sceneUpdateMs: number | null;
  pathPoints: WorldPoint[];
  bundleTemplates: BundleTemplateInfo[];
  selectedBundleTemplateId: number | null;
  cableTemplates: CableTemplateInfo[];
  selectedCableTemplateId: number | null;
  poleTemplates: PoleTemplateInfo[];
  selectedPoleTemplateId: number | null;
  geometry: GeometrySettings;
  layout: LayoutSettings;
  visual: VisualSettings;
  interaction: {
    controlId: string;
    param: string;
    startValue: number | boolean;
  } | null;
  logs: string[];
  selection: { kind: SelectionKind; id: string } | null;
  cameraFov: number;
  showBackboneOverlay: boolean;
  showPreview: boolean;
  keepPathAfterGenerate: boolean;
  drawPlaneZ: number;
  intervalM: number;
  clickedPointsOnly: boolean;
  directionMode: number;
  bundleCount: number;
  solidSupportRender: boolean;
  selectionIncludePoles: boolean;
  selectionIncludeMidair: boolean;
  selectionIncludeSpans: boolean;
  showWorkspace: boolean;
  workspaceWidth: number;
  lastInteractionFrames: {
    sampleCount: number;
    maxFrameMs: number;
    longFrameCount: number;
  } | null;
}

export function createViewerSnapshot(): ViewerSnapshot {
  return {
    parts: [],
    poles: [],
    ports: [],
    spans: [],
    supportNodes: [],
    error: "",
    generationMs: null,
    sceneUpdateMs: null,
    pathPoints: [],
    bundleTemplates: [],
    selectedBundleTemplateId: null,
    cableTemplates: [],
    selectedCableTemplateId: null,
    poleTemplates: [],
    selectedPoleTemplateId: null,
    geometry: {
      curveSamples: 16,
      sagEnabled: true,
      sagFactor: 0.03,
      poleClearance: 0.05
    },
    layout: {
      angleCorrectionEnabled: true,
      cornerThresholdDeg: 70,
      minSideScale: 1,
      maxSideScale: 1.7
    },
    visual: {
      enableSupportStructures: true,
      enableInsulators: true,
      supportCenterThreshold: 0.03,
      supportArmExtra: 0.2,
      insulatorRadius: 0.07,
      insulatorLength: 0.16
    },
    interaction: null,
    logs: [],
    selection: null,
    cameraFov: 48,
    showBackboneOverlay: true,
    showPreview: true,
    keepPathAfterGenerate: true,
    drawPlaneZ: 0,
    intervalM: 0,
    clickedPointsOnly: true,
    directionMode: 0,
    bundleCount: 1,
    solidSupportRender: true,
    selectionIncludePoles: true,
    selectionIncludeMidair: true,
    selectionIncludeSpans: true,
    showWorkspace: true,
    workspaceWidth: 220,
    lastInteractionFrames: null
  };
}

export class ViewerStore {
  readonly value: Readable<ViewerSnapshot>;
  private readonly writable = writable<ViewerSnapshot>(createViewerSnapshot());

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
