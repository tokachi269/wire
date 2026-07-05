export interface EditResult {
  ok: boolean;
  error: string;
  generatedPoleCount: number;
  generatedSpanCount: number;
  totalMs: number;
}

export interface VisualPartInfo {
  kind: number;
  wireRadius: number;
  colorRgba: number;
  sourceNodeId: string;
  sourceEdgeId: string;
  sourceSpanId: string;
  sourceBundleId: string;
  bundleTemplateId: number;
  laneIndex: number;
  sampleCount: number;
}

export interface PoleInfo {
  id: string;
  height: number;
  positionX: number;
  positionY: number;
  positionZ: number;
  rotationX: number;
  rotationY: number;
  rotationZ: number;
  scaleX: number;
  scaleY: number;
  scaleZ: number;
}

export interface WireStateHandle {
  generate(
    points: Float64Array,
    bundleTemplateId: number,
    intervalM: number,
    poleTypeId: number,
    count: number
  ): EditResult;
  visualPartCount(): number;
  visualPart(index: number): VisualPartInfo;
  visualPartSamples(index: number): Float64Array;
  poleCount(): number;
  pole(index: number): PoleInfo;
  delete(): void;
}

export interface WireModule {
  WireState: new () => WireStateHandle;
}

type WireModuleFactory = (options?: Record<string, unknown>) => Promise<WireModule>;

export async function loadWireModule(): Promise<WireModule> {
  const generatedModulePath = "../../wasm/build/wire_web_core.js";
  const module = (await import(/* @vite-ignore */ generatedModulePath)) as {
    default: WireModuleFactory;
  };
  return module.default();
}
