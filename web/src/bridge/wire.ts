import {
  loadWireModule,
  type RoadMeshPayload,
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
  VisualSettings
} from "../model";
import type { RoadMeshData, RoadSceneData, RoadSegmentInput } from "../road";

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

export class WireBridge {
  private modelBootstrap: ModelAssemblyBootstrapInput | null = null;

  private constructor(
    private readonly state: WireStateHandle,
    private readonly roadState: RoadStateHandle
  ) {}

  static async create(options?: WireModuleOptions): Promise<WireBridge> {
    const module = await loadWireModule(options);
    return new WireBridge(new module.WireState(), new module.RoadState());
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

  roadScene(): RoadSceneData {
    const scene = this.roadState.scene();
    return {
      ...scene,
      surfaceMeshes: scene.surfaceMeshes.map(copyRoadMesh),
      markingMeshes: scene.markingMeshes.map(copyRoadMesh)
    };
  }

  roadDeleteSection(segmentId: number): OperationResult {
    return this.roadState.deleteRoadSection(segmentId);
  }

  roadDeleteRange(
    segmentId: number,
    startStationM: number,
    endStationM: number
  ): OperationResult {
    return this.roadState.deleteRoadRange({
      segmentId,
      startStationM,
      endStationM
    });
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

  roadUpdateSectionTemplate(input: Record<string, number | boolean>): OperationResult {
    return this.roadState.updateSectionTemplate(input);
  }

  roadApplyTransition(input: Record<string, number>): OperationResult {
    return this.roadState.applyTransition(input);
  }

  roadSetBoundaryMarkingPolicy(
    input: Parameters<RoadStateHandle["setBoundaryMarkingPolicy"]>[0]
  ): OperationResult {
    return this.roadState.setBoundaryMarkingPolicy(input);
  }

  roadResetBoundaryMarkingPolicy(
    input: Parameters<RoadStateHandle["resetBoundaryMarkingPolicy"]>[0]
  ): OperationResult {
    return this.roadState.resetBoundaryMarkingPolicy(input);
  }

  roadSetLaneSideMarkingPolicy(
    input: Parameters<RoadStateHandle["setLaneSideMarkingPolicy"]>[0]
  ): OperationResult {
    return this.roadState.setLaneSideMarkingPolicy(input);
  }

  roadResetLaneSideMarkingPolicy(
    input: Parameters<RoadStateHandle["resetLaneSideMarkingPolicy"]>[0]
  ): OperationResult {
    return this.roadState.resetLaneSideMarkingPolicy(input);
  }

  roadSuppressSegmentMarking(
    input: Parameters<RoadStateHandle["suppressSegmentMarking"]>[0]
  ): OperationResult {
    return this.roadState.suppressSegmentMarking(input);
  }

  roadResetSegmentMarkingSuppression(
    input: Parameters<RoadStateHandle["resetSegmentMarkingSuppression"]>[0]
  ): OperationResult {
    return this.roadState.resetSegmentMarkingSuppression(input);
  }

  roadSuppressJunctionMarking(
    input: Parameters<RoadStateHandle["suppressJunctionMarking"]>[0]
  ): OperationResult {
    return this.roadState.suppressJunctionMarking(input);
  }

  roadResetJunctionMarkingSuppression(
    input: Parameters<RoadStateHandle["resetJunctionMarkingSuppression"]>[0]
  ): OperationResult {
    return this.roadState.resetJunctionMarkingSuppression(input);
  }

  roadSetJunctionMarkingOverride(
    input: Parameters<RoadStateHandle["setJunctionMarkingOverride"]>[0]
  ): OperationResult & { overrideId?: number } {
    return this.roadState.setJunctionMarkingOverride(input);
  }

  roadDeleteJunctionMarkingOverride(
    input: Parameters<RoadStateHandle["deleteJunctionMarkingOverride"]>[0]
  ): OperationResult {
    return this.roadState.deleteJunctionMarkingOverride(input);
  }

  roadAddManualLine(input: Record<string, number | string>): OperationResult {
    return this.roadState.addManualLine(input);
  }

  roadAddManualArea(input: Record<string, number | string>): OperationResult {
    return this.roadState.addManualArea(input);
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
    material: mesh.material,
    vertices: new Float64Array(mesh.vertices),
    indices: new Uint32Array(mesh.indices)
  };
}
