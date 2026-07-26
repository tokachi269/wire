export type RoadToolMode = "line" | "bezier";
export type RoadToolPhase = "start" | "end" | "bend";
export type RoadOperation = "draw" | "edit" | "delete" | "line-marking" | "area-marking";

export interface RoadPoint {
  x: number;
  y: number;
}

export interface RoadSegmentInput {
  kind: RoadToolMode;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  handleAX: number;
  handleAY: number;
  handleBX: number;
  handleBY: number;
  startNodeId: number;
  startSegmentId: number;
  startStationM: number;
  extensionSegmentId?: number;
  connectToFirstNode: boolean;
  sectionTemplateId?: number;
}

export interface RoadMeshData {
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
  nodes: RoadNodeData[];
  centerlineSegments: RoadCenterlineSegmentData[];
  sectionTemplates: RoadSectionTemplateData[];
  editableSegments: RoadEditableSegmentData[];
  surfaceMeshes: RoadMeshData[];
  markingMeshes: RoadMeshData[];
}

export interface RoadNodeData {
  id: number;
  x: number;
  y: number;
  extensionSegmentId?: number;
}

export interface RoadCenterlineSegmentData {
  id: number;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  startStationM: number;
  endStationM: number;
}

export interface RoadSectionTemplateData {
  id: number;
  name: string;
  bands: Array<{ elementId: number; role: "sidewalk" | "carriageway" | "median"; widthM: number }>;
  sidewalkWidthM: number;
  laneWidthM: number;
  medianWidthM: number;
  laneCount: number;
  hasCenterLine: boolean;
  hasOuterLines: boolean;
}

export interface RoadEditableSegmentData {
  id: number;
  kind: RoadToolMode;
  points: RoadPoint[];
}

export interface RoadSnapInfo {
  kind: "road";
  nodeId: number;
  segmentId: number;
  stationM: number;
  extensionSegmentId?: number;
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
  draftStartNodeId: number;
  draftStartSegmentId: number;
  draftStartStationM: number;
  draftExtensionSegmentId: number;
  connectToFirstNode: boolean;
  selectedSectionTemplateId: number;
  manualLineOffsetM: number;
  manualAreaWidthM: number;
  manualAreaLengthM: number;
  markingDraftSegmentId: number;
  markingDraftStationM: number;
  selectedEditSegmentId: number;
  editKind: RoadToolMode;
  editPoints: RoadPoint[];
  scene: RoadSceneData;
  previewMeshes: RoadMeshData[];
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
    draftStartNodeId: 0,
    draftStartSegmentId: 0,
    draftStartStationM: 0,
    draftExtensionSegmentId: 0,
    connectToFirstNode: false,
    selectedSectionTemplateId: 1,
    manualLineOffsetM: 0,
    manualAreaWidthM: 4,
    manualAreaLengthM: 6,
    markingDraftSegmentId: 0,
    markingDraftStationM: 0,
    selectedEditSegmentId: 0,
    editKind: "line",
    editPoints: [],
    scene: emptyRoadScene(),
    previewMeshes: [],
    lastError: ""
  };
}

export function roadSegmentInput(state: RoadToolState): RoadSegmentInput {
  return {
    kind: state.mode,
    startX: state.draftStart.x,
    startY: state.draftStart.y,
    endX: state.draftEnd.x,
    endY: state.draftEnd.y,
    handleAX: state.handleA.x,
    handleAY: state.handleA.y,
    handleBX: state.handleB.x,
    handleBY: state.handleB.y,
    startNodeId: state.draftStartNodeId,
    startSegmentId: state.draftStartSegmentId,
    startStationM: state.draftStartStationM,
    extensionSegmentId: state.draftExtensionSegmentId,
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
    lastError: ""
  };
}

export function withRoadBend(state: RoadToolState, bend: RoadPoint): RoadToolState {
  return {
    ...state,
    draftBend: bend,
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
    nodes: [],
    centerlineSegments: [],
    sectionTemplates: [],
    editableSegments: [],
    surfaceMeshes: [],
    markingMeshes: []
  };
}
