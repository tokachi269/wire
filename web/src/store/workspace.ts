import type {
  SelectionKind,
  ViewerSnapshot
} from "./viewer";
import type { BundlePlacement } from "../model";

export const WORKSPACE_CACHE_KEY = "wire.workspace.v1";
const WORKSPACE_VERSION = 1;

export interface WorkspaceStorage {
  get(key: string): Promise<string | null>;
  set(key: string, value: string): Promise<void>;
  remove(key: string): Promise<void>;
}

const WORKSPACE_DB_NAME = "wire.workspace";
const WORKSPACE_STORE_NAME = "documents";

export class IndexedDbWorkspaceStorage implements WorkspaceStorage {
  private database: Promise<IDBDatabase> | null = null;

  async get(key: string): Promise<string | null> {
    const store = await this.store("readonly");
    const value = await requestResult(store.get(key));
    return typeof value === "string" ? value : null;
  }

  async set(key: string, value: string): Promise<void> {
    const store = await this.store("readwrite");
    await requestResult(store.put(value, key));
  }

  async remove(key: string): Promise<void> {
    const store = await this.store("readwrite");
    await requestResult(store.delete(key));
  }

  private async store(mode: IDBTransactionMode): Promise<IDBObjectStore> {
    const database = await (this.database ??= openWorkspaceDatabase());
    return database.transaction(WORKSPACE_STORE_NAME, mode).objectStore(WORKSPACE_STORE_NAME);
  }
}

export interface WorkspacePreferences {
  selectedBundleTemplateId: number | null;
  drawBundlePlacements?: BundlePlacement[];
  selectedDrawBundleTemplateIds?: number[];
  drawBundleCounts?: Record<number, number>;
  selectedCableTemplateId: number | null;
  selectedPoleTemplateId: number | null;
  selection: { kind: SelectionKind; id: string } | null;
  cameraFov: number;
  showBackboneOverlay: boolean;
  showGroundGrid?: boolean;
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
    selectedBundleTemplateId: snapshot.selectedBundleTemplateId,
    drawBundlePlacements: snapshot.drawBundlePlacements,
    selectedCableTemplateId: snapshot.selectedCableTemplateId,
    selectedPoleTemplateId: snapshot.selectedPoleTemplateId,
    selection: snapshot.selection,
    cameraFov: snapshot.cameraFov,
    showBackboneOverlay: snapshot.showBackboneOverlay,
    showGroundGrid: snapshot.showGroundGrid,
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
  private pending: Promise<void> = Promise.resolve();

  constructor(private readonly storage: WorkspaceStorage) {}

  async read(): Promise<WorkspaceDocument | null> {
    let text: string | null;
    try {
      await this.pending;
      text = await this.storage.get(WORKSPACE_CACHE_KEY);
    } catch {
      return null;
    }
    if (text === null) return null;
    try {
      const value: unknown = JSON.parse(text);
      if (!isWorkspaceDocument(value)) {
        void this.clear();
        return null;
      }
      return value;
    } catch {
      void this.clear();
      return null;
    }
  }

  write(coreState: string, snapshot: ViewerSnapshot): Promise<void> {
    if (coreState.length === 0) return Promise.resolve();
    const document: WorkspaceDocument = {
      version: WORKSPACE_VERSION,
      coreState,
      viewer: captureWorkspacePreferences(snapshot)
    };
    return this.enqueue(() => this.storage.set(WORKSPACE_CACHE_KEY, JSON.stringify(document)));
  }

  clear(): Promise<void> {
    return this.enqueue(() => this.storage.remove(WORKSPACE_CACHE_KEY));
  }

  private enqueue(operation: () => Promise<void>): Promise<void> {
    const next = this.pending.then(operation, operation);
    this.pending = next.catch(() => undefined);
    return next;
  }
}

function openWorkspaceDatabase(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(WORKSPACE_DB_NAME, 1);
    request.onupgradeneeded = () => {
      const database = request.result;
      if (!database.objectStoreNames.contains(WORKSPACE_STORE_NAME)) {
        database.createObjectStore(WORKSPACE_STORE_NAME);
      }
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error ?? new Error("IndexedDB open failed"));
  });
}

function requestResult<T>(request: IDBRequest<T>): Promise<T> {
  return new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error ?? new Error("IndexedDB request failed"));
  });
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
