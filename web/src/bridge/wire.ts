import {
  loadWireModule,
  type RoadMeshPayload,
  type RoadAddLaneInput,
  type RoadSegmentResult,
  type RoadStateHandle,
  type WireModuleOptions,
  type WireStateHandle
} from "./wasm";
import type {
  BackboneEdgeInfo,
  BundlePlacement,
  BundleTemplateInfo,
  CableTemplateInfo,
  DefaultBundlePlacementInfo,
  EditResult,
  GenerationTiming,
  GeometrySettings,
  LayoutSettings,
  ModelAssemblyBootstrapInput,
  OperationResult,
  PathPickInfo,
  PoleInfo,
  PoleTemplateInfo,
  PortInfo,
  ResolvedPathPointInfo,
  SpanInfo,
  SupportNodeInfo,
  VisualPartInfo,
  VisualModelInstanceInfo,
  VisualSettings,
  WireIntervalRequest,
  WireIntervalResult
} from "../model";
import type { RoadMeshData, RoadSceneData, RoadSegmentInput } from "../road";
import type { RoadIntervalPreview } from "./wasm";
import type { RoadSectionInput } from "../road_templates";

export interface VisualPartData {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface SceneData {
  parts: VisualPartData[];
  models: VisualModelInstanceInfo[];
  poles: PoleInfo[];
  ports: PortInfo[];
  spans: SpanInfo[];
  supportNodes: SupportNodeInfo[];
  backboneEdges: BackboneEdgeInfo[];
  lastGenerationTiming?: GenerationTiming | null;
}

export interface WireIntervalPreview extends WireIntervalResult {
  parts: VisualPartData[];
  poles: PoleInfo[];
}

export class WireBridge {
  private modelBootstrap: ModelAssemblyBootstrapInput | null = null;

  private constructor(
    private readonly state: WireStateHandle,
    private readonly roadState: RoadStateHandle,
    private readonly wasmBuildCommit: string,
    private readonly wasmBuildVersion: string
  ) {}

  static async create(options?: WireModuleOptions): Promise<WireBridge> {
    const module = await loadWireModule(options);
    return new WireBridge(
      new module.WireState(),
      new module.RoadState(),
      module.wireBuildCommit(),
      module.wireBuildVersion()
    );
  }

  buildIdentity(): { commit: string; version: string } {
    return { commit: this.wasmBuildCommit, version: this.wasmBuildVersion };
  }

  generate(
    points: Float64Array,
    bundlePlacements: BundlePlacement[],
    intervalM?: number,
    poleTypeId?: number,
    directionMode?: number,
    maxTiltDeg?: number,
    nodeSpecs?: Array<{ pointIndex: number; supportKind: number; nodeId: string }>
  ): EditResult;
  generate(
    points: Float64Array,
    bundleTemplateIds: number[],
    intervalM?: number,
    poleTypeId?: number,
    counts?: number[],
    directionMode?: number,
    maxTiltDeg?: number,
    nodeSpecs?: Array<{ pointIndex: number; supportKind: number; nodeId: string }>
  ): EditResult;
  generate(
    points: Float64Array,
    bundleInput: BundlePlacement[] | number[],
    intervalM = 0,
    poleTypeId = 1,
    countsOrDirection: number[] | number = 0,
    directionOrTilt = 0,
    tiltOrSpecs: number | Array<{ pointIndex: number; supportKind: number; nodeId: string }> = 0,
    legacyNodeSpecs: Array<{ pointIndex: number; supportKind: number; nodeId: string }> = []
  ): EditResult {
    const isPlacementInput = bundleInput.length === 0 || typeof bundleInput[0] === "object";
    const bundlePlacements = isPlacementInput
      ? bundleInput as BundlePlacement[]
      : (bundleInput as number[]).map((bundleTemplateId, index) => {
          const template = this.bundleTemplates().find((item) => item.id === bundleTemplateId);
          return {
            id: index + 1,
            bundleTemplateId,
            count: Array.isArray(countsOrDirection)
              ? countsOrDirection[index] ?? template?.defaultCount ?? 1
              : template?.defaultCount ?? 1,
            explicit: false,
            height: 0,
            offset: 0,
            spacing: template?.defaultSpacing ?? 0.2
          };
        });
    const directionMode = isPlacementInput
      ? countsOrDirection as number
      : directionOrTilt;
    const maxTiltDeg = isPlacementInput
      ? directionOrTilt
      : typeof tiltOrSpecs === "number" ? tiltOrSpecs : 0;
    const nodeSpecs = isPlacementInput
      ? Array.isArray(tiltOrSpecs) ? tiltOrSpecs : []
      : legacyNodeSpecs;
    return this.state.generatePlacements(
      points,
      bundlePlacements,
      intervalM,
      poleTypeId,
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

  previewWireInterval(input: WireIntervalRequest): WireIntervalPreview {
    const prepared = this.prepareWireInterval(input, true);
    if (!prepared.ok) return { ...prepared, parts: [], poles: [] };
    const preview = this.state.previewPlacements(
      prepared.points,
      input.bundlePlacements,
      input.intervalM,
      input.poleTypeId,
      input.directionMode,
      input.maxTiltDeg,
      prepared.nodeSpecs
    );
    if (!preview.ok) {
      return {
        ...preview,
        endpoint: prepared.endpoint,
        endpointSpec: prepared.endpointSpec,
        parts: [],
        poles: []
      };
    }
    const sceneSamples = new Float64Array(preview.samples);
    const generatedSpanIds = new Set(preview.generatedSpanIds ?? []);
    return {
      ...preview,
      endpoint: prepared.endpoint,
      endpointSpec: prepared.endpointSpec,
      parts: preview.parts
        .filter((info) => generatedSpanIds.has(info.sourceSpanId))
        .map((info) => ({
          info,
          samples: sceneSamples.subarray(info.sampleOffset, info.sampleOffset + info.sampleCount * 3)
        })),
      poles: preview.poles
    };
  }

  generateWireInterval(input: WireIntervalRequest): WireIntervalResult {
    const prepared = this.prepareWireInterval(input, false);
    if (!prepared.ok) return prepared;
    const generated = this.state.generatePlacements(
      prepared.points,
      input.bundlePlacements,
      input.intervalM,
      input.poleTypeId,
      input.directionMode,
      input.maxTiltDeg,
      prepared.nodeSpecs
    );
    return {
      ...generated,
      endpoint: prepared.endpoint,
      endpointSpec: prepared.endpointSpec
    };
  }

  private prepareWireInterval(input: WireIntervalRequest, preview: boolean): WireIntervalResult & {
    points: Float64Array;
    nodeSpecs: Array<{ pointIndex: number; supportKind: number; nodeId: string }>;
  } {
    let endpoint = input.points[1];
    let endpointSpec = input.pointSpecs[1];
    if (input.targetPick !== undefined) {
      const selected = [...new Set(input.bundlePlacements.map((placement) => placement.bundleTemplateId))];
      const resolved = preview
        ? this.state.previewResolveBranchPick(input.targetPick, selected)
        : this.state.resolveBranchPick(input.targetPick, selected);
      if (!resolved.ok) {
        return {
          ...emptyWireIntervalResult(endpoint, endpointSpec, resolved.error),
          points: new Float64Array(),
          nodeSpecs: []
        };
      }
      endpoint = [resolved.positionX, resolved.positionY, resolved.positionZ];
      endpointSpec = { supportKind: resolved.supportKind, nodeId: resolved.nodeId };
    }
    const points = new Float64Array([...input.points[0], ...endpoint]);
    const nodeSpecs = [input.pointSpecs[0], endpointSpec]
      .map((spec, pointIndex) => spec === null ? null : ({ pointIndex, ...spec }))
      .filter((spec): spec is { pointIndex: number; supportKind: number; nodeId: string } => spec !== null);
    return {
      ...emptyWireIntervalResult(endpoint, endpointSpec, ""),
      ok: true,
      points,
      nodeSpecs
    };
  }

  clearPendingSupportNodes(): OperationResult {
    return this.state.clearPendingSupportNodes();
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

  resolveDefaultBundlePlacement(
    bundleTemplateId: number,
    poleTypeId: number,
    count: number
  ): DefaultBundlePlacementInfo {
    return this.state.resolveDefaultBundlePlacement(bundleTemplateId, poleTypeId, count);
  }

  updateBackboneBundlePlacement(
    bundleId: string,
    placement: BundlePlacement
  ): OperationResult {
    return this.state.updateBackboneBundlePlacement(bundleId, placement);
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

  configureModelAssemblies(input: ModelAssemblyBootstrapInput): OperationResult {
    const result = this.state.configureModelAssemblies(input);
    if (result.ok) this.modelBootstrap = input;
    return result;
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

  saveState(): string {
    return this.state.saveState();
  }

  loadState(text: string): OperationResult {
    return this.modelBootstrap === null
      ? this.state.loadState(text)
      : this.state.loadStateWithModels(text, this.modelBootstrap);
  }

  scene(): SceneData {
    const visual = this.state.visualScene();
    // The WASM view is one scene-owned scratch buffer. Copy it before any
    // subsequent state call, then keep zero-copy subarray views per part.
    const sceneSamples = new Float64Array(visual.samples);
    const parts = visual.parts.map((info) => ({
      info,
      samples: sceneSamples.subarray(info.sampleOffset, info.sampleOffset + info.sampleCount * 3)
    }));

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
    return {
      parts,
      models: visual.models,
      poles,
      ports,
      spans,
      supportNodes,
      backboneEdges,
      lastGenerationTiming
    };
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

  roadAddSegment(input: RoadSegmentInput): RoadSegmentResult {
    return this.roadState.addSegment(input);
  }

  roadPreviewSegment(input: RoadSegmentInput): OperationResult & { meshes: RoadMeshData[] } {
    const result = this.roadState.previewSegment(input);
    return { ...result, meshes: result.meshes.map(copyRoadMesh) };
  }

  roadPreviewInterval(input: RoadSegmentInput): RoadIntervalPreview {
    return this.roadState.previewInterval(input);
  }

  roadScene(): RoadSceneData {
    const scene = this.roadState.scene();
    return {
      ...scene,
      surfaceMeshes: scene.surfaceMeshes.map(copyRoadMesh),
      markingMeshes: scene.markingMeshes.map(copyRoadMesh)
    };
  }

  roadDeleteSegment(segmentId: number): OperationResult {
    return this.roadState.deleteSegment(segmentId);
  }

  roadAddLane(input: RoadAddLaneInput): OperationResult & { laneId?: number } {
    return this.roadState.addLane(input);
  }

  roadMoveNode(nodeId: number, x: number, y: number): OperationResult {
    return this.roadState.moveNode({ nodeId, x, y });
  }

  roadPreviewMoveNode(
    nodeId: number,
    x: number,
    y: number
  ): OperationResult & { meshes: RoadMeshData[] } {
    const result = this.roadState.previewMoveNode({ nodeId, x, y });
    return { ...result, meshes: result.meshes.map(copyRoadMesh) };
  }

  roadEditSegment(segmentId: number, input: RoadSegmentInput): OperationResult {
    return this.roadState.editSegment(segmentId, input);
  }

  roadPreviewEditSegment(segmentId: number, input: RoadSegmentInput): OperationResult & { meshes: RoadMeshData[] } {
    const result = this.roadState.previewEditSegment(segmentId, input);
    return { ...result, meshes: result.meshes.map(copyRoadMesh) };
  }

  roadAddRoadLayoutTemplate(
    input: RoadSectionInput
  ): OperationResult & { roadLayoutTemplateId?: number } {
    return this.roadState.addRoadLayoutTemplate(input);
  }

  roadUpdateRoadLayoutTemplate(input: Record<string, number | boolean>): OperationResult {
    return this.roadState.updateRoadLayoutTemplate(input);
  }

  roadUndoSegment(): OperationResult {
    return this.roadState.undoSegment();
  }

  roadClear(): OperationResult {
    return this.roadState.clear();
  }

  roadSaveState(): string {
    return this.roadState.saveState();
  }

  roadLoadState(text: string): OperationResult {
    return this.roadState.loadState(text);
  }

  dispose(): void {
    this.roadState.delete();
    this.state.delete();
  }
}

function copyRoadMesh(mesh: RoadMeshPayload): RoadMeshData {
  return {
    ownerSegmentId: mesh.ownerSegmentId,
    material: mesh.material,
    vertices: new Float64Array(mesh.vertices),
    indices: new Uint32Array(mesh.indices)
  };
}

function emptyWireIntervalResult(
  endpoint: [number, number, number],
  endpointSpec: { supportKind: number; nodeId: string } | null,
  error: string
): WireIntervalResult {
  return {
    ok: error.length === 0,
    error,
    generatedPoleCount: 0,
    generatedSpanCount: 0,
    totalMs: 0,
    timing: {
      prepareMs: 0,
      checkMs: 0,
      pairsMs: 0,
      preflightMs: 0,
      intentMs: 0,
      supportGroupsMs: 0,
      emitMs: 0,
      saveGraphMs: 0,
      rulesMs: 0,
      layoutMs: 0,
      geomMs: 0,
      drawMs: 0,
      totalMs: 0
    },
    endpoint,
    endpointSpec
  };
}
