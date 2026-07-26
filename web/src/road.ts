export type RoadToolMode = "line" | "bezier";
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
  connectToFirstNode: boolean;
}

export interface RoadMeshData {
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
  surfaceMeshes: RoadMeshData[];
  markingMeshes: RoadMeshData[];
}

export interface RoadToolState {
  mode: RoadToolMode;
  phase: RoadToolPhase;
  draftStart: RoadPoint;
  draftEnd: RoadPoint;
  handleA: RoadPoint;
  handleB: RoadPoint;
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
    draftEnd: { x: 0, y: 0 },
    handleA: { x: 0, y: 0 },
    handleB: { x: 0, y: 0 },
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

export function withRoadBend(state: RoadToolState, control: RoadPoint): RoadToolState {
  return {
    ...state,
    handleA: {
      x: state.draftStart.x + (control.x - state.draftStart.x) * 2 / 3,
      y: state.draftStart.y + (control.y - state.draftStart.y) * 2 / 3
    },
    handleB: {
      x: state.draftEnd.x + (control.x - state.draftEnd.x) * 2 / 3,
      y: state.draftEnd.y + (control.y - state.draftEnd.y) * 2 / 3
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
    surfaceMeshes: [],
    markingMeshes: []
  };
}
