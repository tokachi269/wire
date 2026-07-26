import type {
  SelectionKind,
  ViewerSnapshot
} from "./viewer";
import type { BundlePlacement } from "../model";

export const WORKSPACE_CACHE_KEY = "wire.workspace.v1";
const WORKSPACE_VERSION = 1;
const COMPRESSED_WORKSPACE_PREFIX = "wire.workspace.deflate.v1:";

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
  rightPanelMode?: "wire" | "road";
  activeTool?: "wire" | "road";
  workspaceLeftWidth: number;
  workspaceWidth: number;
}

export interface WorkspaceDocument {
  version: 1;
  coreState: string;
  roadState?: string;
  viewer: WorkspacePreferences;
}

export function createWorkspaceDocument(
  coreState: string,
  snapshot: ViewerSnapshot,
  roadState?: string
): WorkspaceDocument {
  return {
    version: WORKSPACE_VERSION,
    coreState,
    ...(roadState === undefined ? {} : { roadState }),
    viewer: captureWorkspacePreferences(snapshot)
  };
}

export async function serializeWorkspaceDocument(document: WorkspaceDocument): Promise<string> {
  return encodeWorkspaceText(JSON.stringify(document));
}

export async function parseWorkspaceDocument(text: string): Promise<WorkspaceDocument | null> {
  const value: unknown = JSON.parse(await decodeWorkspaceText(text));
  return isWorkspaceDocument(value) ? value : null;
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
    rightPanelMode: snapshot.rightPanelMode,
    activeTool: snapshot.activeTool,
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
      const document = await parseWorkspaceDocument(text);
      if (document === null) {
        void this.clear();
        return null;
      }
      return document;
    } catch {
      void this.clear();
      return null;
    }
  }

  write(coreState: string, snapshot: ViewerSnapshot, roadState?: string): Promise<void> {
    if (coreState.length === 0) return Promise.resolve();
    const document = createWorkspaceDocument(coreState, snapshot, roadState);
    return this.enqueue(async () => {
      const text = await serializeWorkspaceDocument(document);
      await this.storage.set(WORKSPACE_CACHE_KEY, text);
    });
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

export async function encodeWorkspaceText(text: string): Promise<string> {
  if (text.length === 0 || typeof CompressionStream === "undefined") {
    return text;
  }
  const stream = new Blob([text]).stream().pipeThrough(new CompressionStream("deflate"));
  const compressed = new Uint8Array(await new Response(stream).arrayBuffer());
  return `${COMPRESSED_WORKSPACE_PREFIX}${bytesToBase64(compressed)}`;
}

export async function decodeWorkspaceText(text: string): Promise<string> {
  if (!text.startsWith(COMPRESSED_WORKSPACE_PREFIX)) {
    return text;
  }
  if (typeof DecompressionStream === "undefined") {
    throw new Error("workspace deflate is not supported by this browser");
  }
  const payload = text.slice(COMPRESSED_WORKSPACE_PREFIX.length);
  const bytes = base64ToBytes(payload);
  const stream = new Blob([ownedArrayBuffer(bytes)]).stream().pipeThrough(new DecompressionStream("deflate"));
  return await new Response(stream).text();
}

function bytesToBase64(bytes: Uint8Array): string {
  let binary = "";
  const chunkSize = 0x8000;
  for (let offset = 0; offset < bytes.length; offset += chunkSize) {
    const chunk = bytes.subarray(offset, offset + chunkSize);
    binary += String.fromCharCode(...chunk);
  }
  return btoa(binary);
}

function base64ToBytes(text: string): Uint8Array {
  const binary = atob(text);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }
  return bytes;
}

function ownedArrayBuffer(bytes: Uint8Array): ArrayBuffer {
  const copy = new Uint8Array(bytes.byteLength);
  copy.set(bytes);
  return copy.buffer;
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
    (candidate.roadState === undefined || typeof candidate.roadState === "string") &&
    typeof candidate.viewer === "object" &&
    candidate.viewer !== null;
}
