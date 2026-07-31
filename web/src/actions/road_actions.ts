import {
  roadSegmentInput,
  roadSpanInput,
  withRoadBend,
  withRoadCurveEnd,
  withRoadEnd,
  type RoadSnapInfo,
  type RoadPoint,
  type RoadToolMode,
  type RoadToolState
} from "../road";
import { EditErrorKind } from "../model";
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
        draftSpans: [],
        previewMeshes: [],
        previewIssue: "",
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
        draftSpans: [],
        markingDraftSegmentId: 0,
        hoveredDeleteSegmentId: 0,
        selectedEditSegmentId: 0,
        selectedEditNodeAId: 0,
        selectedEditNodeBId: 0,
        activeEditPointIndex: -1,
        editPoints: [],
        previewMeshes: [],
        previewIssue: "",
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
    this.applyPreview(edited, preview);
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
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        [key]: value,
        previewIssue: "",
        lastError: ""
      }
    }));
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
      road: {
        ...current.road,
        connectToFirstNode: value,
        previewIssue: "",
        lastError: ""
      }
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
      if (current.draftSpans.length !== 0) {
        this.ctx.store.setError("Finish or cancel the current road before starting another");
        return;
      }
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
    this.stageSpan(road);
  }

  previewViewportPoint(point: WorldPoint, snap?: RoadSnapInfo): void {
    const current = this.ctx.readSnapshot().road;
    if (current.operation === "delete") {
      const hoveredDeleteSegmentId = snap?.segmentId ?? 0;
      if (hoveredDeleteSegmentId !== current.hoveredDeleteSegmentId) {
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: { ...snapshot.road, hoveredDeleteSegmentId }
        }));
      }
      return;
    }
    if (current.operation !== "draw") return;
    if (current.phase === "start") return;
    const target: RoadPoint = { x: point[0], y: point[1] };
    const road = this.previewState(current, target);
    const preview = this.ctx.bridge.roadPreviewSegment(roadSegmentInput(road));
    this.applyPreview(road, preview);
  }

  private applyOperation(road: RoadToolState, snap?: RoadSnapInfo): void {
    if (snap === undefined || snap.segmentId === 0) {
      this.ctx.store.setError("Select a road segment");
      return;
    }
    if (road.operation === "delete") {
      this.finish(
        this.ctx.bridge.roadDeleteSegment(snap.segmentId),
        "road delete segment"
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
          previewIssue: "",
          lastError: ""
        },
        error: ""
      }));
      return;
    }
    if (road.operation === "area-marking") {
      this.finish(this.ctx.bridge.roadAddManualArea({
        segmentId: snap.segmentId,
        segmentDistanceM: snap.segmentDistanceM,
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
            markingDraftSegmentDistanceM: snap.segmentDistanceM,
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
        startSegmentDistanceM: road.markingDraftSegmentDistanceM,
        endSegmentDistanceM: snap.segmentDistanceM,
        lateralM: road.manualLineOffsetM,
        style: "white"
      }), "road add manual line marking");
      return;
    }
    this.ctx.store.setError("Select and drag a road control point");
  }

  undoOrCancel(): void {
    const current = this.ctx.readSnapshot().road;
    if (current.draftSpans.length !== 0) {
      const draftSpans = current.draftSpans.slice(0, -1);
      if (draftSpans.length === 0) {
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: {
            ...snapshot.road,
            phase: "start",
            draftSpans: [],
            curveContinuationTangent: null,
            previewMeshes: [],
            previewIssue: "",
            lastError: ""
          },
          error: ""
        }));
        return;
      }
      const last = draftSpans[draftSpans.length - 1];
      const next = withRoadEnd({
        ...current,
        draftSpans,
        draftStart: { x: last.endX, y: last.endY },
        draftBend: { x: last.endX, y: last.endY },
        phase: current.mode === "line" ? "end" : "bend",
        curveContinuationTangent: last.kind === "bezier"
          ? { x: last.endX - last.handleBX, y: last.endY - last.handleBY }
          : null
      }, { x: last.endX, y: last.endY });
      const preview = this.ctx.bridge.roadPreviewSegment(
        roadSegmentInput(next, false)
      );
      this.applyPreview(next, preview);
      return;
    }
    if (current.phase !== "start") {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: {
          ...snapshot.road,
          phase: "start",
          curveContinuationTangent: null,
          previewMeshes: [],
          previewIssue: "",
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

  commitPath(): void {
    const road = this.ctx.readSnapshot().road;
    if (road.operation !== "draw" || road.draftSpans.length === 0) return;
    const result = this.ctx.bridge.roadAddSegment(
      roadSegmentInput(road, false)
    );
    if (!result.ok) {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: {
          ...road,
          previewMeshes: [],
          previewIssue: "",
          lastError: result.error
        },
        error: result.error
      }));
      return;
    }
    this.finish(result, "road add segment");
  }

  private stageSpan(road: RoadToolState): void {
    const candidate = {
      ...road,
      draftSpans: [...road.draftSpans, roadSpanInput(road)]
    };
    const preview = this.ctx.bridge.roadPreviewSegment(
      roadSegmentInput(candidate, false)
    );
    if (!preview.ok) {
      this.applyPreview(road, preview);
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
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: withRoadEnd({
        ...candidate,
        phase,
        draftStart: nextStart,
        draftBend: nextStart,
        curveContinuationTangent,
        previewMeshes: preview.meshes,
        previewIssue: "",
        lastError: ""
      }, nextStart),
      error: ""
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
        draftStartSegmentDistanceM: snap?.segmentDistanceM ?? 0,
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
        draftSpans: [],
        draftStartNodeId: 0,
        draftStartSegmentId: 0,
        draftStartSegmentDistanceM: 0,
        draftExtensionCorridorId: 0,
        curveContinuationTangent: null,
        markingDraftSegmentId: 0,
        activeEditPointIndex: -1,
        hoveredDeleteSegmentId: 0,
        scene,
        previewMeshes: [],
        previewIssue: "",
        lastError: ""
      },
      error: "",
      logs: [...snapshot.logs, log]
    }));
  }

  private applyPreview(
    road: RoadToolState,
    preview: {
      ok: boolean;
      error: string;
      errorKind?: EditErrorKind;
      meshes: RoadToolState["previewMeshes"];
    }
  ): void {
    const expectedRejection =
      !preview.ok &&
      (preview.errorKind === EditErrorKind.Validation ||
       preview.errorKind === EditErrorKind.Unsupported);
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...road,
        previewMeshes: preview.ok ? preview.meshes : [],
        previewIssue: expectedRejection ? preview.error : "",
        lastError: !preview.ok && !expectedRejection ? preview.error : ""
      },
      error: !preview.ok && !expectedRejection ? preview.error : snapshot.error
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
    startSegmentDistanceM: 0,
    extensionCorridorId: 0,
    connectToFirstNode: false,
    sectionTemplateId: road.selectedSectionTemplateId
  };
}

function sameRoadPoint(a: RoadPoint, b: RoadPoint): boolean {
  return Math.hypot(a.x - b.x, a.y - b.y) <= 1e-6;
}
