export interface GenerationTiming {
  prepareMs: number;
  checkMs: number;
  pairsMs: number;
  preflightMs: number;
  intentMs: number;
  supportGroupsMs: number;
  emitMs: number;
  saveGraphMs: number;
  rulesMs: number;
  layoutMs: number;
  geomMs: number;
  drawMs: number;
  totalMs: number;
}

export interface EditResult {
  ok: boolean;
  error: string;
  generatedPoleCount: number;
  generatedSpanCount: number;
  totalMs: number;
  timing: GenerationTiming;
}

export interface OperationResult {
  ok: boolean;
  error: string;
}

export interface VisualPartInfo {
  partKey: string;
  sourceVersion: string;
  sampleOffset: number;
  kind: number;
  wireRadius: number;
  colorRgba: number;
  sourceNodeId: string;
  sourceEdgeId: string;
  sourceSpanId: string;
  sourceBundleId: string;
  bundleTemplateId: number;
  laneIndex: number;
  runId: number;
  sampleCount: number;
}

export interface VisualModelInstanceInfo {
  stableKey: string;
  modelKey: string;
  contentVersion: string;
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

export interface ModelSocketInput {
  name: string;
  positionX: number;
  positionY: number;
  positionZ: number;
  directionX: number;
  directionY: number;
  directionZ: number;
}

export interface ModelTransformInput {
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

export interface ModelAssemblyPartInput {
  partId: number;
  modelKey: string;
  descriptorName: string;
  descriptorVersion: number;
  fitMode: number;
  localTransform: ModelTransformInput;
  sockets: ModelSocketInput[];
}

export interface ModelAssemblyTemplateInput {
  id: number;
  version: number;
  parts: ModelAssemblyPartInput[];
  wireSocket: { partId: number; socketName: string } | null;
}

export interface ModelAssemblyBootstrapInput {
  assemblies: ModelAssemblyTemplateInput[];
  poleAssignments: Array<{ poleTypeId: number; assemblyId: number }>;
  bundleAssignments: Array<{
    bundleTemplateId: number;
    rowAssemblyId: number;
    endpointAssemblyId: number;
  }>;
}

export interface SceneContentSyncStats {
  total: number;
  reused: number;
  rebuilt: number;
  removed: number;
  modelTotal: number;
  modelReused: number;
  modelUpdated: number;
  modelRebuilt: number;
  modelRemoved: number;
}

export interface PoleInfo {
  id: string;
  poleTypeId?: number;
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

export interface PortInfo {
  id: string;
  ownerPoleId: string;
  x: number;
  y: number;
  z: number;
  category: number;
  layer: number;
}

export interface SpanInfo {
  id: string;
  portAId: string;
  portBId: string;
}

export interface SupportNodeInfo {
  id: string;
  kind: number;
  poleId: string;
  x: number;
  y: number;
  z: number;
}

export interface BackboneEdgeInfo {
  nodeAId: string;
  nodeBId: string;
  bundleIds: string[];
}

export interface PathPickInfo {
  hitKind: number;
  hitId: string;
  hitX: number;
  hitY: number;
  hitZ: number;
  hasSegmentEndpoints: boolean;
  segmentNodeAId: string;
  segmentNodeBId: string;
  segmentEndpointAX: number;
  segmentEndpointAY: number;
  segmentEndpointAZ: number;
  segmentEndpointBX: number;
  segmentEndpointBY: number;
  segmentEndpointBZ: number;
}

export interface ResolvedPathPointInfo {
  ok: boolean;
  error: string;
  positionX: number;
  positionY: number;
  positionZ: number;
  supportKind: number;
  nodeId: string;
}

export interface BundleTemplateInfo {
  id: number;
  kind: number;
  name: string;
  defaultCount: number;
  fixedCount: boolean;
  fixedCountValue: number;
  minCount: number;
  maxCount: number;
  cableTemplateId: number;
  relatedPoleTypeId: number;
  defaultLayer: number;
  allowMidairNode: boolean;
  allowMidairBranch: boolean;
  supportWirePoleBandId: number;
  rowFixtureAssemblyId: number;
  endpointFixtureAssemblyId: number;
  spanVisualAssembly: SpanVisualAssemblyInfo;
  populationRules: PopulationRuleInfo[];
}

export interface SpanVisualAssemblyInfo {
  supportPathEnabled: boolean;
  helixEnabled: boolean;
  helixRadius: number;
  helixClearance: number;
  helixTurnsPerMeter: number;
  helixSamplesPerTurn: number;
  endpointTrim: number;
  memberWanderRatio: number;
  memberWanderWavelength: number;
  memberWanderPhaseBias: number;
  memberTwistTurnsPerMeter: number;
  memberTwistPhase: number;
}

export interface PopulationRuleInfo {
  ruleId: number;
  explicitSeed: number;
  priority: number;
  minExtraCount: number;
  maxExtraCount: number;
  minSpacing: number;
  lateralMin: number;
  lateralMax: number;
  heightMin: number;
  heightMax: number;
}

export interface CableTemplateInfo {
  id: number;
  name: string;
  outerDiameter: number;
  bendStiffness: number;
  minBendRadius: number;
  materialStyle: number;
  colorRgba: number;
  sagFactor: number;
  slackFactor: number;
  continuityPolicy: number;
  supplementalEnabled: boolean;
  supplementalLateralOffset: number;
  supplementalVerticalOffset: number;
  supplementalWobbleAmplitude: number;
  supplementalWobbleWavelength: number;
  supplementalWobblePhase: number;
  supplementalEndpointEnvelope: number;
}

export interface PortBandInfo {
  bandId: number;
  category: number;
  layer: number;
  side: number;
  role: number;
  lateralCenter: number;
  lateralMin: number;
  lateralMax: number;
  heightCenter: number;
  heightMin: number;
  heightMax: number;
  priority: number;
  minSpacing: number;
  allowMultiple: boolean;
  overflowPolicy: number;
  enabled: boolean;
}

export interface AnchorSlotInfo {
  slotId: number;
  usage: number;
  localX: number;
  localY: number;
  localZ: number;
  priority: number;
  enabled: boolean;
}

export interface PoleTemplateInfo {
  id: number;
  name: string;
  description: string;
  defaultHeight: number;
  poleVisualAssemblyId: number;
  portBands: PortBandInfo[];
  anchorSlots: AnchorSlotInfo[];
}

export interface GeometrySettings {
  curveSamples: number;
  sagEnabled: boolean;
  sagFactor: number;
  poleClearance: number;
}

export interface LayoutSettings {
  angleCorrectionEnabled: boolean;
  cornerThresholdDeg: number;
  minSideScale: number;
  maxSideScale: number;
}

export interface VisualSettings {
  enableSupportStructures: boolean;
  enableInsulators: boolean;
  supportCenterThreshold: number;
  supportArmExtra: number;
  insulatorRadius: number;
  insulatorLength: number;
}
