export interface EditResult {
  ok: boolean;
  error: string;
  generatedPoleCount: number;
  generatedSpanCount: number;
  totalMs: number;
}

export interface OperationResult {
  ok: boolean;
  error: string;
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
  bundleId: string;
}

export interface SupportNodeInfo {
  id: string;
  kind: number;
  poleId: string;
  x: number;
  y: number;
  z: number;
}

export interface BundleTemplateInfo {
  id: number;
  name: string;
  defaultCount: number;
  fixedCount: boolean;
  cableTemplateId: number;
  relatedPoleTypeId: number;
  defaultLayer: number;
  allowMirror: boolean;
  allowMidairNode: boolean;
  allowMidairBranch: boolean;
  groupedSupportFanoutSpacing: number;
  supportStyle: number;
  branchPolicy: number;
  continuityPolicy: number;
}

export interface CableTemplateInfo {
  id: number;
  name: string;
  outerDiameter: number;
  bendStiffness: number;
  minBendRadius: number;
  materialStyle: number;
  requiresInsulator: boolean;
  insulatorAttachmentHeight: number;
  sagFactor: number;
  slackFactor: number;
  groupedFanoutSpacing: number;
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
