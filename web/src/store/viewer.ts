import { writable, type Readable } from "svelte/store";
import type {
  BundleTemplateInfo,
  CableTemplateInfo,
  BackboneEdgeInfo,
  GenerationTiming,
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

export interface PathPointSpec {
  supportKind: number;
  nodeId: string;
}

export interface ViewerSnapshot {
  parts: VisualPart[];
  poles: PoleInfo[];
  ports: PortInfo[];
  spans: SpanInfo[];
  supportNodes: SupportNodeInfo[];
  backboneEdges: BackboneEdgeInfo[];
  error: string;
  generationMs: number | null;
  generationTiming: GenerationTiming | null;
  generationCallMs: number | null;
  sceneUpdateMs: number | null;
  viewerUpdateMs: number | null;
  pathPoints: WorldPoint[];
  pathPointSpecs: Array<PathPointSpec | null>;
  bundleTemplates: BundleTemplateInfo[];
  selectedBundleTemplateId: number | null;
  selectedDrawBundleTemplateIds: number[];
  drawBundleCounts: Record<number, number>;
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
  maxTiltDeg: number;
  solidSupportRender: boolean;
  selectionIncludePoles: boolean;
  selectionIncludeMidair: boolean;
  selectionIncludeSpans: boolean;
  showLeftPanel: boolean;
  showRightPanel: boolean;
  workspaceLeftWidth: number;
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
    backboneEdges: [],
    error: "",
    generationMs: null,
    generationTiming: null,
    generationCallMs: null,
    sceneUpdateMs: null,
    viewerUpdateMs: null,
    pathPoints: [],
    pathPointSpecs: [],
    bundleTemplates: [],
    selectedBundleTemplateId: null,
    selectedDrawBundleTemplateIds: [],
    drawBundleCounts: {},
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
    keepPathAfterGenerate: false,
    drawPlaneZ: 0,
    intervalM: 0,
    clickedPointsOnly: true,
    directionMode: 0,
    maxTiltDeg: 9.5,
    solidSupportRender: true,
    selectionIncludePoles: true,
    selectionIncludeMidair: true,
    selectionIncludeSpans: true,
    showLeftPanel: true,
    showRightPanel: true,
    workspaceLeftWidth: 220,
    workspaceWidth: 320,
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
