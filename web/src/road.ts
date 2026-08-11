export type RoadToolMode = "line" | "bezier";
export type RoadToolPhase = "start" | "end";
export type RoadOperation = "draw" | "edit" | "delete" | "add-lane";

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
  endNodeId?: number;
  endSegmentId?: number;
  endSegmentDistanceM?: number;
  extensionCorridorId?: number;
  connectToFirstNode: boolean;
  roadLayoutTemplateId?: number;
  junctionCornerRadiusM?: number;
}

export interface RoadMeshData {
  ownerSegmentId: number;
  material: string;
  vertices: Float64Array;
  indices: Uint32Array;
}

export interface RoadSceneData {
  segmentCount: number;
  roadLayoutTemplateCount: number;
  transitionCount: number;
  markingCount: number;
  connectionGateCount: number;
  junctionCount: number;
  corridorCount: number;
  nodes: RoadNodeData[];
  centerlineSegments: RoadCenterlineSegmentData[];
  lanePaths: RoadLanePathData[];
  corridors: RoadCorridorData[];
  roadLayoutTemplates: RoadLayoutTemplateData[];
  editableSegments: RoadEditableSegmentData[];
  surfaceMeshes: RoadMeshData[];
  markingMeshes: RoadMeshData[];
  approaches: RoadApproachLayoutData[];
  junctions: RoadJunctionData[];
}

export interface RoadLanePathData {
  segmentId: number;
  laneId: number;
  direction: 0 | 1;
  startRoadLayoutTemplateId: number;
  endRoadLayoutTemplateId: number;
  startSegmentDistanceM: number;
  endSegmentDistanceM: number;
  nodeAId: number;
  nodeBId: number;
  points: number[];
}

export interface RoadCorridorData {
  id: number;
  roadLayoutTemplateId: number;
  lengthM: number;
  segments: Array<{ segmentId: number; reversed: boolean; lengthM: number }>;
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
  pickHalfWidthM: number;
}

export interface RoadLayoutTemplateData {
  id: number;
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
  lanes: Array<{ id: number; direction: 0 | 1 }>;
  boundaries: Array<{ id: number; role: number }>;
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
  laneId?: number;
  laneDirection?: 0 | 1;
  endpointRole?: 0 | 1;
}

export interface RoadToolState {
  mode: RoadToolMode;
  operation: RoadOperation;
  phase: RoadToolPhase;
  draftStart: RoadPoint;
  draftEnd: RoadPoint;
  handleA: RoadPoint;
  handleB: RoadPoint;
  draftStartNodeId: number;
  draftStartSegmentId: number;
  draftStartSegmentDistanceM: number;
  draftExtensionCorridorId: number;
  connectToFirstNode: boolean;
  selectedRoadLayoutTemplateId: number;
  /** Labels for the sections this session registered, keyed by the ID Core assigned. */
  roadLayoutTemplateLabels: Record<number, string>;
  /** Product-preset defaults keyed by the Core section ID assigned at startup. */
  roadJunctionCornerRadiusDefaults: Record<number, number>;
  junctionCornerRadiusM: number;
  manualLineOffsetM: number;
  manualAreaWidthM: number;
  manualAreaLengthM: number;
  markingDraftSegmentId: number;
  markingDraftSegmentDistanceM: number;
  hoveredDeleteSegmentId: number;
  hoveredLaneSegmentId: number;
  hoveredLaneId: number;
  selectedLaneSegmentId: number;
  selectedLaneId: number;
  selectedLaneEndpointRole: 0 | 1;
  selectedLaneNodeId: number;
  selectedLaneDirection: 0 | 1;
  laneEditStage: "select" | "transition-complete" | "target";
  laneSide: "left" | "right";
  laneWidthM: number;
  laneTransitionStartSegmentId: number;
  laneTransitionStartT: number;
  laneTransitionCompleteSegmentId: number;
  laneTransitionCompleteT: number;
  laneCorridorId: number;
  laneTargetTemplateId: number;
  laneTargetLaneId: number;
  laneSourceBoundaryId: number;
  laneTargetBoundaryId: number;
  selectedEditSegmentId: number;
  selectedEditNodeAId: number;
  selectedEditNodeBId: number;
  activeEditPointIndex: number;
  editKind: RoadToolMode;
  editPoints: RoadPoint[];
  scene: RoadSceneData;
  previewMeshes: RoadMeshData[];
  previewState: "none" | "guide" | "valid" | "invalid";
  previewRequest: RoadSegmentInput | null;
  previewIssue: string;
  lastError: string;
}

export function createRoadToolState(): RoadToolState {
  return {
    mode: "line",
    operation: "draw",
    phase: "start",
    draftStart: { x: 0, y: 0 },
    draftEnd: { x: 0, y: 0 },
    handleA: { x: 0, y: 0 },
    handleB: { x: 0, y: 0 },
    draftStartNodeId: 0,
    draftStartSegmentId: 0,
    draftStartSegmentDistanceM: 0,
    draftExtensionCorridorId: 0,
    connectToFirstNode: false,
    selectedRoadLayoutTemplateId: 0,
    roadLayoutTemplateLabels: {},
    roadJunctionCornerRadiusDefaults: {},
    junctionCornerRadiusM: 4,
    manualLineOffsetM: 0,
    manualAreaWidthM: 4,
    manualAreaLengthM: 6,
    markingDraftSegmentId: 0,
    markingDraftSegmentDistanceM: 0,
    hoveredDeleteSegmentId: 0,
    hoveredLaneSegmentId: 0,
    hoveredLaneId: 0,
    selectedLaneSegmentId: 0,
    selectedLaneId: 0,
    selectedLaneEndpointRole: 0,
    selectedLaneNodeId: 0,
    selectedLaneDirection: 0,
    laneEditStage: "select",
    laneSide: "right",
    laneWidthM: 3,
    laneTransitionStartSegmentId: 0,
    laneTransitionStartT: 0,
    laneTransitionCompleteSegmentId: 0,
    laneTransitionCompleteT: 0,
    laneCorridorId: 0,
    laneTargetTemplateId: 1,
    laneTargetLaneId: 0,
    laneSourceBoundaryId: 0,
    laneTargetBoundaryId: 0,
    selectedEditSegmentId: 0,
    selectedEditNodeAId: 0,
    selectedEditNodeBId: 0,
    activeEditPointIndex: -1,
    editKind: "line",
    editPoints: [],
    scene: emptyRoadScene(),
    previewMeshes: [],
    previewState: "none",
    previewRequest: null,
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
  endSnap?: RoadSnapInfo
): RoadSegmentInput {
  const first = roadSpanInput(state);
  return {
    ...first,
    spans: [first],
    startNodeId: state.draftStartNodeId,
    startSegmentId: state.draftStartSegmentId,
    startSegmentDistanceM: state.draftStartSegmentDistanceM,
    endNodeId: endSnap?.nodeId ?? 0,
    endSegmentId: endSnap?.segmentId ?? 0,
    endSegmentDistanceM: endSnap?.segmentDistanceM ?? 0,
    extensionCorridorId: state.draftExtensionCorridorId,
    connectToFirstNode: state.connectToFirstNode,
    roadLayoutTemplateId: state.selectedRoadLayoutTemplateId,
    junctionCornerRadiusM: state.junctionCornerRadiusM
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

// The curve tangent at a shared point is decided by core, which sees the points
// on both sides of it. The viewer only reports where the interval ends.
export function withRoadCurveEnd(state: RoadToolState, end: RoadPoint): RoadToolState {
  return withRoadEnd(state, end);
}

export function emptyRoadScene(): RoadSceneData {
  return {
    segmentCount: 0,
    roadLayoutTemplateCount: 0,
    transitionCount: 0,
    markingCount: 0,
    connectionGateCount: 0,
    junctionCount: 0,
    corridorCount: 0,
    nodes: [],
    centerlineSegments: [],
    lanePaths: [],
    corridors: [],
    roadLayoutTemplates: [],
    editableSegments: [],
    surfaceMeshes: [],
    markingMeshes: [],
    approaches: [],
    junctions: []
  };
}
