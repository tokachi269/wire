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
      road: { ...current.road, mode, phase: "start", previewMeshes: [], lastError: "" }
    }));
  }

  setOperation(operation: RoadToolState["operation"]): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        operation,
        phase: "start",
        markingDraftSegmentId: 0,
        selectedEditSegmentId: 0,
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
    const edited = { ...road, editPoints };
    const preview = this.ctx.bridge.roadPreviewEditSegment(road.selectedEditSegmentId, editInput(edited));
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
    if (road.operation !== "edit" || road.selectedEditSegmentId === 0) return;
    this.finish(this.ctx.bridge.roadEditSegment(road.selectedEditSegmentId, editInput(road)), "road edit alignment");
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
      this.finish(this.ctx.bridge.roadDeleteSegment(snap.segmentId), "road delete segment");
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
          editKind: editable.kind,
          editPoints: editable.points.map((point) => ({ ...point })),
          previewMeshes: [],
          lastError: ""
        },
        error: ""
      }));
      return;
    }
    if (road.operation === "transition") {
      this.finish(this.ctx.bridge.roadApplyTransition({
        segmentId: snap.segmentId,
        targetTemplateId: road.transitionTargetTemplateId,
        lengthM: road.transitionLengthM,
        endOffsetM: road.transitionEndOffsetM,
        anchor: road.transitionAnchor
      }), "road apply transition");
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
        road: { ...snapshot.road, phase: "start", previewMeshes: [], lastError: "" },
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
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: withRoadEnd({
        ...road,
        phase,
        draftStart: nextStart,
        draftBend: nextStart,
        draftStartNodeId: 0,
        draftStartSegmentId: 0,
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
      return withRoadEnd(withRoadBend(current, target), target);
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
        draftStartNodeId: snap?.nodeId ?? 0,
        draftStartSegmentId: snap?.segmentId ?? 0,
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
  let handleA = start;
  let handleB = end;
  if (road.editKind === "arc") {
    const through = road.editPoints[1] ?? start;
    handleA = {
      x: start.x + (through.x - start.x) * 2 / 3,
      y: start.y + (through.y - start.y) * 2 / 3
    };
    handleB = {
      x: end.x + (through.x - end.x) * 2 / 3,
      y: end.y + (through.y - end.y) * 2 / 3
    };
  } else if (road.editKind === "bezier") {
    handleA = road.editPoints[1] ?? start;
    handleB = road.editPoints[2] ?? end;
  }
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
    connectToFirstNode: false,
    sectionTemplateId: road.selectedSectionTemplateId
  };
}

function sameRoadPoint(a: RoadPoint, b: RoadPoint): boolean {
  return Math.hypot(a.x - b.x, a.y - b.y) <= 1e-6;
}
