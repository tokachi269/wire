export type RoadPrimitiveKind = "line" | "bezier";

export interface RoadPoint {
  x: number;
  y: number;
}

export interface RoadPrimitive {
  kind: RoadPrimitiveKind;
  p0: RoadPoint;
  p1: RoadPoint;
  p2?: RoadPoint;
  p3?: RoadPoint;
}

export interface RoadSegment {
  id: number;
  sectionTemplateId: number;
  primitives: RoadPrimitive[];
  connectedNodeId: number | null;
  transitionId: number | null;
}

export interface RoadSectionTemplate {
  id: number;
  name: string;
  laneCount: number;
  laneWidthM: number;
  sidewalkWidthM: number;
  curbWidthM: number;
  curbHeightM: number;
}

export interface RoadTransition {
  id: number;
  fromTemplateId: number;
  toTemplateId: number;
  startFromEndM: number;
  endFromEndM: number;
  action: "taper-in" | "taper-out" | "continue";
}

export interface RoadManualMarking {
  id: number;
  ownerSegmentId: number;
  kind: "line" | "area";
  style: string;
}

export interface RoadDerivedSummary {
  sectionSamples: number;
  surfaceMeshes: number;
  terrainMasks: number;
  connectionGates: number;
  junctionAreas: number;
  manualMarkingMeshes: number;
}

export interface RoadToolState {
  nextId: number;
  mode: "line" | "bezier";
  draftStart: RoadPoint;
  draftEnd: RoadPoint;
  handleA: RoadPoint;
  handleB: RoadPoint;
  activeSectionTemplateId: number;
  connectToFirstNode: boolean;
  segments: RoadSegment[];
  sectionTemplates: RoadSectionTemplate[];
  transitions: RoadTransition[];
  manualMarkings: RoadManualMarking[];
  derived: RoadDerivedSummary;
  lastError: string;
}

export function createRoadToolState(): RoadToolState {
  return rebuildRoadDerived({
    nextId: 2,
    mode: "bezier",
    draftStart: { x: 0, y: 0 },
    draftEnd: { x: 40, y: 0 },
    handleA: { x: 10, y: 14 },
    handleB: { x: 30, y: -14 },
    activeSectionTemplateId: 1,
    connectToFirstNode: false,
    segments: [],
    sectionTemplates: [japaneseUrbanTwoLaneTemplate(1)],
    transitions: [],
    manualMarkings: [],
    derived: emptyRoadDerived(),
    lastError: ""
  });
}

export function japaneseUrbanTwoLaneTemplate(id: number): RoadSectionTemplate {
  return {
    id,
    name: "JP urban 2 lane",
    laneCount: 2,
    laneWidthM: 3,
    sidewalkWidthM: 2,
    curbWidthM: 0.2,
    curbHeightM: 0.15
  };
}

export function threeLaneTemplate(id: number): RoadSectionTemplate {
  return {
    ...japaneseUrbanTwoLaneTemplate(id),
    name: "JP urban 3 lane taper",
    laneCount: 3
  };
}

export function previewRoadPrimitive(state: RoadToolState): RoadPrimitive {
  if (state.mode === "line") {
    return { kind: "line", p0: state.draftStart, p1: state.draftEnd };
  }
  return {
    kind: "bezier",
    p0: state.draftStart,
    p1: state.handleA,
    p2: state.handleB,
    p3: state.draftEnd
  };
}

export function addRoadSegment(state: RoadToolState): RoadToolState {
  const primitive = previewRoadPrimitive(state);
  const error = validatePrimitive(primitive);
  if (error !== "") {
    return { ...state, lastError: error };
  }
  const connectedNodeId = state.connectToFirstNode && state.segments.length > 0 ? firstNodeId(state) : null;
  const segment: RoadSegment = {
    id: state.nextId,
    sectionTemplateId: state.activeSectionTemplateId,
    primitives: [primitive],
    connectedNodeId,
    transitionId: null
  };
  return rebuildRoadDerived({
    ...state,
    nextId: state.nextId + 1,
    segments: [...state.segments, segment],
    lastError: ""
  });
}

export function addRoadTransition(state: RoadToolState): RoadToolState {
  if (state.segments.length === 0) {
    return { ...state, lastError: "road transition needs a segment" };
  }
  const template = threeLaneTemplate(state.nextId);
  const transition: RoadTransition = {
    id: state.nextId + 1,
    fromTemplateId: state.activeSectionTemplateId,
    toTemplateId: template.id,
    startFromEndM: 25,
    endFromEndM: 5,
    action: "taper-in"
  };
  const target = state.segments.at(-1)!;
  return rebuildRoadDerived({
    ...state,
    nextId: state.nextId + 2,
    sectionTemplates: [...state.sectionTemplates, template],
    transitions: [...state.transitions, transition],
    segments: state.segments.map((segment) =>
      segment.id === target.id ? { ...segment, transitionId: transition.id } : segment
    ),
    lastError: ""
  });
}

export function addRoadManualLine(state: RoadToolState): RoadToolState {
  return addRoadManualMarking(state, "line", "white line");
}

export function addRoadManualArea(state: RoadToolState): RoadToolState {
  return addRoadManualMarking(state, "area", "zebra");
}

export function updateRoadDraftPoint(
  state: RoadToolState,
  key: "draftStart" | "draftEnd" | "handleA" | "handleB",
  axis: "x" | "y",
  value: number
): RoadToolState {
  if (!Number.isFinite(value)) {
    return { ...state, lastError: "road draft coordinate must be finite" };
  }
  return {
    ...state,
    [key]: {
      ...state[key],
      [axis]: value
    },
    lastError: ""
  };
}

export function rebuildRoadDerived(state: RoadToolState): RoadToolState {
  const connectionGates = state.segments.length * 2;
  const connectedSegments = state.segments.filter((segment) => segment.connectedNodeId !== null).length;
  return {
    ...state,
    derived: {
      sectionSamples: state.segments.reduce((sum, segment) => sum + sampleCount(segment), 0),
      surfaceMeshes: state.segments.length,
      terrainMasks: state.segments.length,
      connectionGates,
      junctionAreas: connectedSegments > 0 ? 1 : 0,
      manualMarkingMeshes: state.manualMarkings.length
    }
  };
}

function addRoadManualMarking(
  state: RoadToolState,
  kind: RoadManualMarking["kind"],
  style: string
): RoadToolState {
  if (state.segments.length === 0) {
    return { ...state, lastError: "manual marking needs an owner segment" };
  }
  const marking: RoadManualMarking = {
    id: state.nextId,
    ownerSegmentId: state.segments.at(-1)!.id,
    kind,
    style
  };
  return rebuildRoadDerived({
    ...state,
    nextId: state.nextId + 1,
    manualMarkings: [...state.manualMarkings, marking],
    lastError: ""
  });
}

function emptyRoadDerived(): RoadDerivedSummary {
  return {
    sectionSamples: 0,
    surfaceMeshes: 0,
    terrainMasks: 0,
    connectionGates: 0,
    junctionAreas: 0,
    manualMarkingMeshes: 0
  };
}

function firstNodeId(state: RoadToolState): number {
  const first = state.segments[0];
  return first === undefined ? 0 : first.id * 10;
}

function sampleCount(segment: RoadSegment): number {
  const length = primitiveLength(segment.primitives[0]);
  return Math.max(2, Math.ceil(length / 2) + 1);
}

function primitiveLength(primitive: RoadPrimitive): number {
  if (primitive.kind === "line") {
    return distance(primitive.p0, primitive.p1);
  }
  const p2 = primitive.p2 ?? primitive.p1;
  const p3 = primitive.p3 ?? primitive.p1;
  let length = 0;
  let previous = primitive.p0;
  for (let i = 1; i <= 24; i += 1) {
    const t = i / 24;
    const point = bezierPoint(primitive.p0, primitive.p1, p2, p3, t);
    length += distance(previous, point);
    previous = point;
  }
  return length;
}

function validatePrimitive(primitive: RoadPrimitive): string {
  const points = [primitive.p0, primitive.p1, primitive.p2, primitive.p3]
    .filter((point): point is RoadPoint => point !== undefined);
  if (points.some((point) => !Number.isFinite(point.x) || !Number.isFinite(point.y))) {
    return "road path contains non-finite coordinate";
  }
  if (primitiveLength(primitive) <= 1e-6) {
    return "road path has zero length";
  }
  return "";
}

function bezierPoint(p0: RoadPoint, p1: RoadPoint, p2: RoadPoint, p3: RoadPoint, t: number): RoadPoint {
  const u = 1 - t;
  return {
    x: u ** 3 * p0.x + 3 * u ** 2 * t * p1.x + 3 * u * t ** 2 * p2.x + t ** 3 * p3.x,
    y: u ** 3 * p0.y + 3 * u ** 2 * t * p1.y + 3 * u * t ** 2 * p2.y + t ** 3 * p3.y
  };
}

function distance(a: RoadPoint, b: RoadPoint): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}
