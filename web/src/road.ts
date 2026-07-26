export type RoadToolMode = "line" | "arc" | "bezier";
export type RoadToolPhase = "start" | "end" | "bend";

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
  connectToFirstNode: boolean;
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
  surfaceMeshes: RoadMeshData[];
  markingMeshes: RoadMeshData[];
}

export interface RoadNodeData {
  id: number;
  x: number;
  y: number;
}

export interface RoadCenterlineSegmentData {
  id: number;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
}

export interface RoadSnapInfo {
  kind: "road";
  nodeId: number;
  segmentId: number;
}

export interface RoadToolState {
  mode: RoadToolMode;
  phase: RoadToolPhase;
  draftStart: RoadPoint;
  draftBend: RoadPoint;
  draftEnd: RoadPoint;
  handleA: RoadPoint;
  handleB: RoadPoint;
  draftStartNodeId: number;
  draftStartSegmentId: number;
  connectToFirstNode: boolean;
  scene: RoadSceneData;
  previewMeshes: RoadMeshData[];
  lastError: string;
}

export function createRoadToolState(): RoadToolState {
  return {
    mode: "line",
    phase: "start",
    draftStart: { x: 0, y: 0 },
    draftBend: { x: 0, y: 0 },
    draftEnd: { x: 0, y: 0 },
    handleA: { x: 0, y: 0 },
    handleB: { x: 0, y: 0 },
    draftStartNodeId: 0,
    draftStartSegmentId: 0,
    connectToFirstNode: false,
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
    connectToFirstNode: state.connectToFirstNode
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
    surfaceMeshes: [],
    markingMeshes: []
  };
}
