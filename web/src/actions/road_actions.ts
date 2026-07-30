import {
  roadSegmentInput,
  withRoadBend,
  withRoadCurveEnd,
  withRoadEnd,
  type RoadSnapInfo,
  type RoadPoint,
  type RoadToolMode,
  type RoadToolState
} from "../road";
import type { WorldPoint } from "../store/viewer";
import { ViewerActionContext } from "./context";

export class RoadActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  setMode(mode: RoadToolMode): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        mode,
        phase: "start",
        curveContinuationTangent: null,
        previewMeshes: [],
        lastError: ""
      }
    }));
  }

  setOperation(operation: RoadToolState["operation"]): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        operation,
        phase: "start",
        curveContinuationTangent: null,
        markingDraftSegmentId: 0,
        deleteDraftSegmentId: 0,
        selectedEditSegmentId: 0,
        selectedEditNodeAId: 0,
        selectedEditNodeBId: 0,
        activeEditPointIndex: -1,
        editPoints: [],
        previewMeshes: [],
        lastError: ""
      }
    }));
  }

  previewEditHandle(handleIndex: number, point: WorldPoint): void {
    const road = this.ctx.readSnapshot().road;
    if (road.operation !== "edit" || road.selectedEditSegmentId === 0 || handleIndex >= road.editPoints.length) return;
    const editPoints = road.editPoints.map((value, index) => index === handleIndex ? { x: point[0], y: point[1] } : value);
    const endpoint = handleIndex === 0 || handleIndex === road.editPoints.length - 1;
    const edited = {
      ...road,
      editPoints,
      activeEditPointIndex: handleIndex,
      editKind: endpoint ? road.editKind : "bezier" as const
    };
    const nodeId = handleIndex === 0
      ? road.selectedEditNodeAId
      : road.selectedEditNodeBId;
    const preview = endpoint
      ? this.ctx.bridge.roadPreviewMoveNode(nodeId, point[0], point[1])
      : this.ctx.bridge.roadPreviewEditSegment(
          road.selectedEditSegmentId,
          editInput(edited)
        );
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...edited,
        previewMeshes: preview.ok ? preview.meshes : [],
        lastError: preview.ok ? "" : preview.error
      },
      error: preview.ok ? "" : preview.error
    }));
  }

  commitEditHandle(): void {
    const road = this.ctx.readSnapshot().road;
    if (road.operation !== "edit" || road.selectedEditSegmentId === 0 ||
        road.activeEditPointIndex < 0) return;
    const handleIndex = road.activeEditPointIndex;
    const endpoint = handleIndex === 0 || handleIndex === road.editPoints.length - 1;
    if (endpoint) {
      const nodeId = handleIndex === 0
        ? road.selectedEditNodeAId
        : road.selectedEditNodeBId;
      const point = road.editPoints[handleIndex];
      this.finish(this.ctx.bridge.roadMoveNode(nodeId, point.x, point.y),
                  "road move node");
      return;
    }
    this.finish(
      this.ctx.bridge.roadEditSegment(
        road.selectedEditSegmentId,
        editInput(road)
      ),
      "road edit alignment"
    );
  }

  setSetting<K extends keyof RoadToolState>(key: K, value: RoadToolState[K]): void {
    this.ctx.store.update((current) => ({ ...current, road: { ...current.road, [key]: value, lastError: "" } }));
  }

  updateSelectedSectionTemplate(input: {
    sidewalkWidthM: number;
    laneWidthM: number;
    medianWidthM: number;
    hasCenterLine: boolean;
    hasOuterLines: boolean;
  }): void {
    const road = this.ctx.readSnapshot().road;
    this.finish(this.ctx.bridge.roadUpdateSectionTemplate({ id: road.selectedSectionTemplateId, ...input }),
                "road update section template");
  }

  setConnectToFirstNode(value: boolean): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: { ...current.road, connectToFirstNode: value, lastError: "" }
    }));
  }

  addViewportPoint(point: WorldPoint, snap?: RoadSnapInfo): void {
    const target = { x: point[0], y: point[1] };
    const current = this.ctx.readSnapshot().road;
    if (current.operation !== "draw") {
      this.applyOperation(current, snap);
      return;
    }
    if (current.phase === "start") {
      this.beginAt(target, snap);
      return;
    }
    if (snap !== undefined && !sameRoadPoint(current.draftStart, target)) {
      this.beginAt(target, snap);
      return;
    }
    if (current.mode !== "line" && current.phase === "bend") {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: { ...withRoadBend(snapshot.road, target), phase: "end" }
      }));
      return;
    }
    const road = current.mode !== "line" ? withRoadCurveEnd(current, target) : withRoadEnd(current, target);
    this.commit(road);
  }

  previewViewportPoint(point: WorldPoint): void {
    const current = this.ctx.readSnapshot().road;
    if (current.operation !== "draw") return;
    if (current.phase === "start") return;
    const target: RoadPoint = { x: point[0], y: point[1] };
    const road = this.previewState(current, target);
    const preview = this.ctx.bridge.roadPreviewSegment(roadSegmentInput(road));
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...road,
        previewMeshes: preview.ok ? preview.meshes : [],
        lastError: preview.ok ? "" : preview.error
      },
      error: preview.ok ? "" : preview.error
    }));
  }

  private applyOperation(road: RoadToolState, snap?: RoadSnapInfo): void {
    if (snap === undefined || snap.segmentId === 0) {
      this.ctx.store.setError("Select a road segment");
      return;
    }
    if (road.operation === "delete") {
      if (road.deleteDraftSegmentId === 0 ||
          road.deleteDraftSegmentId !== snap.segmentId) {
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: {
            ...snapshot.road,
            deleteDraftSegmentId: snap.segmentId,
            deleteDraftStationM: snap.stationM,
            lastError: ""
          },
          error: ""
        }));
        return;
      }
      const startStationM = Math.min(road.deleteDraftStationM, snap.stationM);
      const endStationM = Math.max(road.deleteDraftStationM, snap.stationM);
      if (endStationM - startStationM <= 1e-6) {
        this.ctx.store.setError("Select a different road point");
        return;
      }
      this.finish(
        this.ctx.bridge.roadDeleteRange(
          snap.segmentId,
          startStationM,
          endStationM
        ),
        "road delete range"
      );
      return;
    }
    if (road.operation === "edit") {
      const editable = road.scene.editableSegments.find((segment) => segment.id === snap.segmentId);
      if (editable === undefined) {
        this.ctx.store.setError("Road alignment is not editable");
        return;
      }
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: {
          ...snapshot.road,
          selectedEditSegmentId: editable.id,
          selectedEditNodeAId: editable.nodeAId,
          selectedEditNodeBId: editable.nodeBId,
          activeEditPointIndex: -1,
          editKind: editable.kind,
          editPoints: editable.points.map((point) => ({ ...point })),
          previewMeshes: [],
          lastError: ""
        },
        error: ""
      }));
      return;
    }
    if (road.operation === "area-marking") {
      this.finish(this.ctx.bridge.roadAddManualArea({
        segmentId: snap.segmentId,
        stationM: snap.stationM,
        lateralM: road.manualLineOffsetM,
        widthM: road.manualAreaWidthM,
        lengthM: road.manualAreaLengthM,
        style: "zebra"
      }), "road add manual area marking");
      return;
    }
    if (road.operation === "line-marking") {
      if (road.markingDraftSegmentId === 0) {
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: {
            ...snapshot.road,
            markingDraftSegmentId: snap.segmentId,
            markingDraftStationM: snap.stationM,
            lastError: ""
          }
        }));
        return;
      }
      if (road.markingDraftSegmentId !== snap.segmentId) {
        this.ctx.store.setError("Manual line endpoints must use the same road segment");
        return;
      }
      this.finish(this.ctx.bridge.roadAddManualLine({
        segmentId: snap.segmentId,
        startStationM: road.markingDraftStationM,
        endStationM: snap.stationM,
        lateralM: road.manualLineOffsetM,
        style: "white"
      }), "road add manual line marking");
      return;
    }
    this.ctx.store.setError("Select and drag a road control point");
  }

  undoOrCancel(): void {
    const current = this.ctx.readSnapshot().road;
    if (current.phase !== "start") {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: {
          ...snapshot.road,
          phase: "start",
          curveContinuationTangent: null,
          previewMeshes: [],
          lastError: ""
        },
        error: ""
      }));
      return;
    }
    const result = this.ctx.bridge.roadUndoSegment();
    this.finish(result, "road undo");
  }

  clear(): void {
    const result = this.ctx.bridge.roadClear();
    this.finish(result, "road clear");
  }

  private commit(road: RoadToolState): void {
    const result = this.ctx.bridge.roadAddSegment(roadSegmentInput(road));
    if (!result.ok) {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: { ...road, lastError: result.error },
        error: result.error
      }));
      return;
    }
    const nextStart = road.draftEnd;
    const phase = road.mode !== "line" ? "bend" : "end";
    const curveContinuationTangent =
      road.mode === "bezier"
        ? {
            x: road.draftEnd.x - road.handleB.x,
            y: road.draftEnd.y - road.handleB.y
          }
        : null;
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: withRoadEnd({
        ...road,
        phase,
        draftStart: nextStart,
        draftBend: nextStart,
        curveContinuationTangent,
        draftStartNodeId: result.endNodeId ?? 0,
        draftStartSegmentId: 0,
        draftStartStationM: 0,
        draftExtensionCorridorId: result.corridorId ?? 0,
        scene,
        previewMeshes: [],
        lastError: ""
      }, nextStart),
      error: "",
      logs: [...snapshot.logs, "road add segment"]
    }));
  }

  private previewState(current: RoadToolState, target: RoadPoint): RoadToolState {
    if (current.mode === "line") {
      return withRoadEnd(current, target);
    }
    if (current.phase === "bend") {
      const bent = withRoadBend(current, target);
      return withRoadEnd(bent, bent.draftBend);
    }
    return withRoadCurveEnd(current, target);
  }

  private beginAt(target: RoadPoint, snap?: RoadSnapInfo): void {
    const phase = this.ctx.readSnapshot().road.mode !== "line" ? "bend" : "end";
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: withRoadEnd({
        ...snapshot.road,
        draftStart: target,
        draftBend: target,
        curveContinuationTangent: null,
        draftStartNodeId: snap?.nodeId ?? 0,
        draftStartSegmentId: snap?.segmentId ?? 0,
        draftStartStationM: snap?.stationM ?? 0,
        draftExtensionCorridorId: snap?.extensionCorridorId ?? 0,
        phase
      }, target)
    }));
  }

  private finish(result: { ok: boolean; error: string }, log: string): void {
    if (!result.ok) {
      this.ctx.store.setError(result.error);
      return;
    }
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...snapshot.road,
        phase: "start",
        markingDraftSegmentId: 0,
        deleteDraftSegmentId: 0,
        activeEditPointIndex: -1,
        scene,
        previewMeshes: [],
        lastError: ""
      },
      error: "",
      logs: [...snapshot.logs, log]
    }));
  }
}

function editInput(road: RoadToolState) {
  const start = road.editPoints[0] ?? { x: 0, y: 0 };
  const end = road.editPoints.at(-1) ?? start;
  const handleA = road.editPoints[1] ?? start;
  const handleB = road.editPoints[2] ?? end;
  return {
    kind: road.editKind,
    startX: start.x,
    startY: start.y,
    endX: end.x,
    endY: end.y,
    handleAX: handleA.x,
    handleAY: handleA.y,
    handleBX: handleB.x,
    handleBY: handleB.y,
    startNodeId: 0,
    startSegmentId: 0,
    startStationM: 0,
    extensionCorridorId: 0,
    connectToFirstNode: false,
    sectionTemplateId: road.selectedSectionTemplateId
  };
}

function sameRoadPoint(a: RoadPoint, b: RoadPoint): boolean {
  return Math.hypot(a.x - b.x, a.y - b.y) <= 1e-6;
}
