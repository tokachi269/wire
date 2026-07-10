import {
  loadWireModule,
  type WireModuleOptions,
  type WireStateHandle
} from "./wasm";
import type {
  BackboneEdgeInfo,
  BundleTemplateInfo,
  CableTemplateInfo,
  EditResult,
  GenerationTiming,
  GeometrySettings,
  LayoutSettings,
  OperationResult,
  PathPickInfo,
  PoleInfo,
  PoleTemplateInfo,
  PortInfo,
  ResolvedPathPointInfo,
  SpanInfo,
  SupportNodeInfo,
  VisualPartInfo,
  VisualSettings
} from "../model";

export interface VisualPartData {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface SceneData {
  parts: VisualPartData[];
  poles: PoleInfo[];
  ports: PortInfo[];
  spans: SpanInfo[];
  supportNodes: SupportNodeInfo[];
  backboneEdges: BackboneEdgeInfo[];
  lastGenerationTiming?: GenerationTiming | null;
}

export class WireBridge {
  private constructor(private readonly state: WireStateHandle) {}

  static async create(options?: WireModuleOptions): Promise<WireBridge> {
    const module = await loadWireModule(options);
    return new WireBridge(new module.WireState());
  }

  generate(
    points: Float64Array,
    bundleTemplateIds: number[] = [102],
    intervalM = 0,
    poleTypeId = 1,
    counts: number[] = [0],
    directionMode = 0,
    maxTiltDeg = 0,
    nodeSpecs: Array<{ pointIndex: number; supportKind: number; nodeId: string }> = []
  ): EditResult {
    return this.state.generate(
      points,
      bundleTemplateIds,
      intervalM,
      poleTypeId,
      counts,
      directionMode,
      maxTiltDeg,
      nodeSpecs
    );
  }

  resolveBranchPick(
    input: PathPickInfo,
    selectedBundleTemplateIds: number[]
  ): ResolvedPathPointInfo {
    return this.state.resolveBranchPick(input, selectedBundleTemplateIds);
  }

  bundleTemplates(): BundleTemplateInfo[] {
    const templates: BundleTemplateInfo[] = [];
    for (let index = 0; index < this.state.bundleTemplateCount(); index += 1) {
      templates.push(this.state.bundleTemplate(index));
    }
    return templates;
  }

  updateBundleTemplate(template: BundleTemplateInfo): OperationResult {
    return this.state.updateBundleTemplate(template);
  }

  applyRelatedPoleType(bundleTemplateId: number): OperationResult {
    return this.state.applyRelatedPoleType(bundleTemplateId);
  }

  cableTemplates(): CableTemplateInfo[] {
    const templates: CableTemplateInfo[] = [];
    for (let index = 0; index < this.state.cableTemplateCount(); index += 1) {
      templates.push(this.state.cableTemplate(index));
    }
    return templates;
  }

  updateCableTemplate(
    template: CableTemplateInfo,
    preferredSpanIds: string[]
  ): OperationResult {
    return this.state.updateCableTemplate(template, preferredSpanIds);
  }

  poleTemplates(): PoleTemplateInfo[] {
    const templates: PoleTemplateInfo[] = [];
    for (let index = 0; index < this.state.poleTemplateCount(); index += 1) {
      templates.push(this.state.poleTemplate(index));
    }
    return templates;
  }

  updatePoleTemplate(template: PoleTemplateInfo): OperationResult {
    return this.state.updatePoleTemplate(template);
  }

  geometrySettings(): GeometrySettings {
    return this.state.geometrySettings();
  }

  updateGeometrySettings(settings: GeometrySettings): OperationResult {
    return this.state.updateGeometrySettings(settings);
  }

  layoutSettings(): LayoutSettings {
    return this.state.layoutSettings();
  }

  updateLayoutSettings(settings: LayoutSettings): OperationResult {
    return this.state.updateLayoutSettings(settings);
  }

  visualSettings(): VisualSettings {
    return this.state.visualSettings();
  }

  updateVisualSettings(settings: VisualSettings): OperationResult {
    return this.state.updateVisualSettings(settings);
  }

  applyPoleTilt(poleIds: string[], maxTiltDeg: number): OperationResult {
    return this.state.applyPoleTilt(poleIds, maxTiltDeg);
  }

  resetSpanReferenceLengths(): OperationResult {
    return this.state.resetSpanReferenceLengths();
  }

  scene(): SceneData {
    const parts: VisualPartData[] = [];
    for (let index = 0; index < this.state.visualPartCount(); index += 1) {
      const info = this.state.visualPart(index);
      // The wasm view is scratch memory. Copy it before requesting another part.
      const samples = new Float64Array(this.state.visualPartSamples(index));
      parts.push({ info, samples });
    }

    const poles: PoleInfo[] = [];
    for (let index = 0; index < this.state.poleCount(); index += 1) {
      poles.push(this.state.pole(index));
    }
    const ports: PortInfo[] = [];
    for (let index = 0; index < this.state.portCount(); index += 1) {
      ports.push(this.state.port(index));
    }
    const spans: SpanInfo[] = [];
    for (let index = 0; index < this.state.spanCount(); index += 1) {
      spans.push(this.state.span(index));
    }
    const supportNodes: SupportNodeInfo[] = [];
    for (let index = 0; index < this.state.supportNodeCount(); index += 1) {
      supportNodes.push(this.state.supportNode(index));
    }
    const backboneEdges: BackboneEdgeInfo[] = [];
    for (let index = 0; index < this.state.backboneEdgeCount(); index += 1) {
      backboneEdges.push(this.state.backboneEdge(index));
    }
    const timing = this.state.lastGenerationTiming();
    const lastGenerationTiming = timing.totalMs > 0 ? timing : null;
    return { parts, poles, ports, spans, supportNodes, backboneEdges, lastGenerationTiming };
  }

  clearPoleOrientationOverride(poleId: string): OperationResult {
    return this.state.clearPoleOrientationOverride(poleId);
  }

  clearSpanSocketOverride(spanId: string, isStartEndpoint: boolean): OperationResult {
    return this.state.clearSpanSocketOverride(spanId, isStartEndpoint);
  }

  clearSpanBranchDownOverride(spanId: string): OperationResult {
    return this.state.clearSpanBranchDownOverride(spanId);
  }

  dispose(): void {
    this.state.delete();
  }
}
