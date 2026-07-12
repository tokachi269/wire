import type {
  PathPointSpec,
  SelectionKind,
  ViewerSnapshot,
  WorldPoint
} from "./store/viewer";

const WORKSPACE_CACHE_KEY = "wire.workspace.v1";
const WORKSPACE_VERSION = 1;

export interface KeyValueStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

export interface WorkspacePreferences {
  pathPoints: WorldPoint[];
  pathPointSpecs: Array<PathPointSpec | null>;
  selectedBundleTemplateId: number | null;
  selectedDrawBundleTemplateIds: number[];
  drawBundleCounts: Record<number, number>;
  selectedCableTemplateId: number | null;
  selectedPoleTemplateId: number | null;
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
}

export interface WorkspaceDocument {
  version: 1;
  coreState: string;
  viewer: WorkspacePreferences;
}

export function captureWorkspacePreferences(
  snapshot: ViewerSnapshot
): WorkspacePreferences {
  return {
    pathPoints: snapshot.pathPoints,
    pathPointSpecs: snapshot.pathPointSpecs,
    selectedBundleTemplateId: snapshot.selectedBundleTemplateId,
    selectedDrawBundleTemplateIds: snapshot.selectedDrawBundleTemplateIds,
    drawBundleCounts: snapshot.drawBundleCounts,
    selectedCableTemplateId: snapshot.selectedCableTemplateId,
    selectedPoleTemplateId: snapshot.selectedPoleTemplateId,
    selection: snapshot.selection,
    cameraFov: snapshot.cameraFov,
    showBackboneOverlay: snapshot.showBackboneOverlay,
    showPreview: snapshot.showPreview,
    keepPathAfterGenerate: snapshot.keepPathAfterGenerate,
    drawPlaneZ: snapshot.drawPlaneZ,
    intervalM: snapshot.intervalM,
    clickedPointsOnly: snapshot.clickedPointsOnly,
    directionMode: snapshot.directionMode,
    maxTiltDeg: snapshot.maxTiltDeg,
    solidSupportRender: snapshot.solidSupportRender,
    selectionIncludePoles: snapshot.selectionIncludePoles,
    selectionIncludeMidair: snapshot.selectionIncludeMidair,
    selectionIncludeSpans: snapshot.selectionIncludeSpans,
    showLeftPanel: snapshot.showLeftPanel,
    showRightPanel: snapshot.showRightPanel,
    workspaceLeftWidth: snapshot.workspaceLeftWidth,
    workspaceWidth: snapshot.workspaceWidth
  };
}

export class WorkspaceCache {
  constructor(private readonly storage: KeyValueStorage) {}

  read(): WorkspaceDocument | null {
    let text: string | null;
    try {
      text = this.storage.getItem(WORKSPACE_CACHE_KEY);
    } catch {
      return null;
    }
    if (text === null) return null;
    try {
      const value: unknown = JSON.parse(text);
      if (!isWorkspaceDocument(value)) {
        this.clear();
        return null;
      }
      return value;
    } catch {
      this.clear();
      return null;
    }
  }

  write(coreState: string, snapshot: ViewerSnapshot): void {
    if (coreState.length === 0) return;
    const document: WorkspaceDocument = {
      version: WORKSPACE_VERSION,
      coreState,
      viewer: captureWorkspacePreferences(snapshot)
    };
    this.storage.setItem(WORKSPACE_CACHE_KEY, JSON.stringify(document));
  }

  clear(): void {
    try {
      this.storage.removeItem(WORKSPACE_CACHE_KEY);
    } catch {
      // Storage can be unavailable in privacy modes. Reset still applies in memory.
    }
  }
}

function isWorkspaceDocument(value: unknown): value is WorkspaceDocument {
  if (typeof value !== "object" || value === null) return false;
  const candidate = value as Partial<WorkspaceDocument>;
  return candidate.version === WORKSPACE_VERSION &&
    typeof candidate.coreState === "string" &&
    candidate.coreState.length > 0 &&
    typeof candidate.viewer === "object" &&
    candidate.viewer !== null;
}
