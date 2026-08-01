import { writable, type Readable } from "svelte/store";
import type {
  BundleTemplateInfo,
  BundlePlacement,
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
  VisualModelInstanceInfo,
  VisualPartInfo,
  VisualSettings,
  WireIntervalRequest
} from "../model";
import { CommitFailureCategory, type CommitFailure, type OperationResult } from "../model";
import { createRoadToolState, type RoadToolState } from "../road";

export type WorldPoint = [number, number, number];
export type SelectionKind = "pole" | "port" | "span" | "supportNode";
export type RightPanelMode = "wire" | "road";
export type ActiveTool = "wire" | "road";

export interface VisualPart {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface PathPointSpec {
  supportKind: number;
  nodeId: string;
}

export interface WirePreviewState {
  state: "none" | "guide";
  request: WireIntervalRequest | null;
}

export interface ViewerSnapshot {
  parts: VisualPart[];
  models: VisualModelInstanceInfo[];
  poles: PoleInfo[];
  ports: PortInfo[];
  spans: SpanInfo[];
  supportNodes: SupportNodeInfo[];
  backboneEdges: BackboneEdgeInfo[];
  error: string;
  lastCommitFailure: CommitFailure | null;
  buildMismatch: { webCommit: string; webVersion: string; wasmCommit: string; wasmVersion: string } | null;
  generationMs: number | null;
  generationTiming: GenerationTiming | null;
  generationCallMs: number | null;
  sceneUpdateMs: number | null;
  viewerUpdateMs: number | null;
  pathPoints: WorldPoint[];
  pathPointSpecs: Array<PathPointSpec | null>;
  wirePreview: WirePreviewState;
  bundleTemplates: BundleTemplateInfo[];
  selectedBundleTemplateId: number | null;
  drawBundlePlacements: BundlePlacement[];
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
  showGroundGrid: boolean;
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
  rightPanelMode: RightPanelMode;
  activeTool: ActiveTool;
  workspaceLeftWidth: number;
  workspaceWidth: number;
  lastInteractionFrames: {
    sampleCount: number;
    maxFrameMs: number;
    longFrameCount: number;
  } | null;
  road: RoadToolState;
}

export function createViewerSnapshot(): ViewerSnapshot {
  return {
    parts: [],
    models: [],
    poles: [],
    ports: [],
    spans: [],
    supportNodes: [],
    backboneEdges: [],
    error: "",
    lastCommitFailure: null,
    buildMismatch: null,
    generationMs: null,
    generationTiming: null,
    generationCallMs: null,
    sceneUpdateMs: null,
    viewerUpdateMs: null,
    pathPoints: [],
    pathPointSpecs: [],
    wirePreview: { state: "none", request: null },
    bundleTemplates: [],
    selectedBundleTemplateId: null,
    drawBundlePlacements: [],
    cableTemplates: [],
    selectedCableTemplateId: null,
    poleTemplates: [],
    selectedPoleTemplateId: null,
    geometry: {
      curveSamples: 24,
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
      enableInsulators: true,
      insulatorRadius: 0.07,
      insulatorLength: 0.16
    },
    interaction: null,
    logs: [],
    selection: null,
    cameraFov: 48,
    showBackboneOverlay: true,
    showGroundGrid: true,
    showPreview: true,
    keepPathAfterGenerate: false,
    drawPlaneZ: 0,
    intervalM: 0,
    clickedPointsOnly: true,
    directionMode: 0,
    maxTiltDeg: 12,
    solidSupportRender: true,
    selectionIncludePoles: true,
    selectionIncludeMidair: true,
    selectionIncludeSpans: true,
    showLeftPanel: true,
    showRightPanel: true,
    rightPanelMode: "wire",
    activeTool: "wire",
    workspaceLeftWidth: 220,
    workspaceWidth: 320,
    lastInteractionFrames: null,
    road: createRoadToolState()
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

  setCommitFailure(
    result: OperationResult,
    operation: string,
    attemptedPosition: WorldPoint | null = null
  ): void {
    const category = result.failureCategory ?? CommitFailureCategory.InternalError;
    this.writable.update((current) => ({
      ...current,
      lastCommitFailure: {
        category,
        reasonCode: result.reasonCode || "internal_error",
        message: result.error || "The operation could not be completed",
        operation,
        attemptedPosition
      }
    }));
  }

  update(change: (current: ViewerSnapshot) => ViewerSnapshot): void {
    this.writable.update(change);
  }
}
