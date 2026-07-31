export type RoadToolMode = "line" | "bezier";
export type RoadToolPhase = "start" | "end" | "bend";
export type RoadOperation = "draw" | "edit" | "delete" | "line-marking" | "area-marking";

export interface RoadPoint {
  x: number;
  y: number;
}

export interface RoadSpanInput {
  kind: RoadToolMode;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  handleAX: number;
  handleAY: number;
  handleBX: number;
  handleBY: number;
}

export interface RoadSegmentInput extends RoadSpanInput {
  spans?: RoadSpanInput[];
  startNodeId: number;
  startSegmentId: number;
  startSegmentDistanceM: number;
  extensionCorridorId?: number;
  connectToFirstNode: boolean;
  sectionTemplateId?: number;
}

export interface RoadMeshData {
  ownerSegmentId: number;
  material: string;
  vertices: Float64Array;
  indices: Uint32Array;
}

export interface RoadSceneData {
  segmentCount: number;
  sectionTemplateCount: number;
  transitionCount: number;
  markingCount: number;
  connectionGateCount: number;
  junctionCount: number;
  corridorCount: number;
  nodes: RoadNodeData[];
  centerlineSegments: RoadCenterlineSegmentData[];
  corridors: RoadCorridorData[];
  sectionTemplates: RoadSectionTemplateData[];
  editableSegments: RoadEditableSegmentData[];
  surfaceMeshes: RoadMeshData[];
  markingMeshes: RoadMeshData[];
  approaches: RoadApproachLayoutData[];
  junctions: RoadJunctionData[];
}

export interface RoadCorridorData {
  id: number;
  sectionTemplateId: number;
  lengthM: number;
  segments: Array<{ segmentId: number; reversed: boolean }>;
}

export interface RoadJunctionMarkingBoundaryData {
  boundaryId: number;
  role: number;
}

export interface RoadJunctionGateData {
  nodeId: number;
  segmentId: number;
  endpointRole: 0 | 1;
  markingBoundaries: RoadJunctionMarkingBoundaryData[];
}

export interface RoadJunctionMarkingOverrideData {
  overrideId: number;
  action: number;
  sourceSegmentId: number;
  sourceBoundaryId: number;
  sourceRole: number;
  hasTarget: boolean;
  targetSegmentId?: number;
  targetBoundaryId?: number;
  targetRole?: number;
}

export interface RoadJunctionData {
  nodeId: number;
  gates: RoadJunctionGateData[];
  markingOverrides: RoadJunctionMarkingOverrideData[];
}

export interface RoadApproachLayoutData {
  nodeId: number;
  segmentId: number;
  endpointRole: 0 | 1;
  kind: number;
  autoSetbackM: number;
  resolvedSetbackM: number;
  manualSetback: boolean;
  manualSetbackM: number;
  autoLateralShiftM: number;
  resolvedLateralShiftM: number;
  manualLateralShift: boolean;
  manualLateralShiftM: number;
}

export interface RoadNodeData {
  id: number;
  x: number;
  y: number;
  extensionCorridorId?: number;
}

export interface RoadCenterlineSegmentData {
  id: number;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  startSegmentDistanceM: number;
  endSegmentDistanceM: number;
}

export interface RoadSectionTemplateData {
  id: number;
  name: string;
  strips: Array<{
    id: number;
    function: "sidewalk" | "shoulder" | "carriageway" | "median";
    widthM: number;
  }>;
  sidewalkWidthM: number;
  laneWidthM: number;
  medianWidthM: number;
  laneCount: number;
  hasCenterLine: boolean;
  hasOuterLines: boolean;
}

export interface RoadEditableSegmentData {
  id: number;
  nodeAId: number;
  nodeBId: number;
  kind: RoadToolMode;
  points: RoadPoint[];
}

export interface RoadSnapInfo {
  kind: "road";
  nodeId: number;
  segmentId: number;
  segmentDistanceM: number;
  extensionCorridorId?: number;
}

export interface RoadToolState {
  mode: RoadToolMode;
  operation: RoadOperation;
  phase: RoadToolPhase;
  draftStart: RoadPoint;
  draftBend: RoadPoint;
  draftEnd: RoadPoint;
  handleA: RoadPoint;
  handleB: RoadPoint;
  curveContinuationTangent: RoadPoint | null;
  draftSpans: RoadSpanInput[];
  draftStartNodeId: number;
  draftStartSegmentId: number;
  draftStartSegmentDistanceM: number;
  draftExtensionCorridorId: number;
  connectToFirstNode: boolean;
  selectedSectionTemplateId: number;
  manualLineOffsetM: number;
  manualAreaWidthM: number;
  manualAreaLengthM: number;
  markingDraftSegmentId: number;
  markingDraftSegmentDistanceM: number;
  hoveredDeleteSegmentId: number;
  selectedEditSegmentId: number;
  selectedEditNodeAId: number;
  selectedEditNodeBId: number;
  activeEditPointIndex: number;
  editKind: RoadToolMode;
  editPoints: RoadPoint[];
  scene: RoadSceneData;
  previewMeshes: RoadMeshData[];
  previewIssue: string;
  lastError: string;
}

export function createRoadToolState(): RoadToolState {
  return {
    mode: "line",
    operation: "draw",
    phase: "start",
    draftStart: { x: 0, y: 0 },
    draftBend: { x: 0, y: 0 },
    draftEnd: { x: 0, y: 0 },
    handleA: { x: 0, y: 0 },
    handleB: { x: 0, y: 0 },
    curveContinuationTangent: null,
    draftSpans: [],
    draftStartNodeId: 0,
    draftStartSegmentId: 0,
    draftStartSegmentDistanceM: 0,
    draftExtensionCorridorId: 0,
    connectToFirstNode: false,
    selectedSectionTemplateId: 1,
    manualLineOffsetM: 0,
    manualAreaWidthM: 4,
    manualAreaLengthM: 6,
    markingDraftSegmentId: 0,
    markingDraftSegmentDistanceM: 0,
    hoveredDeleteSegmentId: 0,
    selectedEditSegmentId: 0,
    selectedEditNodeAId: 0,
    selectedEditNodeBId: 0,
    activeEditPointIndex: -1,
    editKind: "line",
    editPoints: [],
    scene: emptyRoadScene(),
    previewMeshes: [],
    previewIssue: "",
    lastError: ""
  };
}

export function roadSpanInput(state: RoadToolState): RoadSpanInput {
  return {
    kind: state.mode,
    startX: state.draftStart.x,
    startY: state.draftStart.y,
    endX: state.draftEnd.x,
    endY: state.draftEnd.y,
    handleAX: state.handleA.x,
    handleAY: state.handleA.y,
    handleBX: state.handleB.x,
    handleBY: state.handleB.y
  };
}

export function roadSegmentInput(
  state: RoadToolState,
  includeCurrentSpan = true
): RoadSegmentInput {
  const spans = includeCurrentSpan
    ? [...state.draftSpans, roadSpanInput(state)]
    : [...state.draftSpans];
  const first = spans[0] ?? roadSpanInput(state);
  return {
    ...first,
    spans,
    startNodeId: state.draftStartNodeId,
    startSegmentId: state.draftStartSegmentId,
    startSegmentDistanceM: state.draftStartSegmentDistanceM,
    extensionCorridorId: state.draftExtensionCorridorId,
    connectToFirstNode: state.connectToFirstNode,
    sectionTemplateId: state.selectedSectionTemplateId
  };
}

export function withRoadEnd(state: RoadToolState, end: RoadPoint): RoadToolState {
  const dx = end.x - state.draftStart.x;
  const dy = end.y - state.draftStart.y;
  return {
    ...state,
    draftEnd: end,
    handleA: { x: state.draftStart.x + dx / 3, y: state.draftStart.y + dy / 3 },
    handleB: { x: state.draftStart.x + dx * 2 / 3, y: state.draftStart.y + dy * 2 / 3 },
    previewIssue: "",
    lastError: ""
  };
}

export function withRoadBend(state: RoadToolState, bend: RoadPoint): RoadToolState {
  return {
    ...state,
    draftBend: snapBendToContinuation(state.draftStart, bend, state.curveContinuationTangent),
    previewIssue: "",
    lastError: ""
  };
}

export function withRoadCurveEnd(state: RoadToolState, end: RoadPoint): RoadToolState {
  const control = state.draftBend;
  return {
    ...state,
    draftEnd: end,
    handleA: {
      x: state.draftStart.x + (control.x - state.draftStart.x) * 2 / 3,
      y: state.draftStart.y + (control.y - state.draftStart.y) * 2 / 3
    },
    handleB: {
      x: end.x + (control.x - end.x) * 2 / 3,
      y: end.y + (control.y - end.y) * 2 / 3
    },
    previewIssue: "",
    lastError: ""
  };
}

export function emptyRoadScene(): RoadSceneData {
  return {
    segmentCount: 0,
    sectionTemplateCount: 1,
    transitionCount: 0,
    markingCount: 0,
    connectionGateCount: 0,
    junctionCount: 0,
    corridorCount: 0,
    nodes: [],
    centerlineSegments: [],
    corridors: [],
    sectionTemplates: [],
    editableSegments: [],
    surfaceMeshes: [],
    markingMeshes: [],
    approaches: [],
    junctions: []
  };
}

function snapBendToContinuation(
  start: RoadPoint,
  bend: RoadPoint,
  tangent: RoadPoint | null | undefined
): RoadPoint {
  if (tangent == null) return bend;
  const tangentLength = Math.hypot(tangent.x, tangent.y);
  const bendDistance = Math.hypot(bend.x - start.x, bend.y - start.y);
  if (tangentLength <= Number.EPSILON || bendDistance <= Number.EPSILON) {
    return bend;
  }
  return {
    x: start.x + tangent.x / tangentLength * bendDistance,
    y: start.y + tangent.y / tangentLength * bendDistance
  };
}
