import {
  roadSegmentInput,
  withRoadCurveEnd,
  withRoadEnd,
  type RoadSnapInfo,
  type RoadPoint,
  type RoadToolMode,
  type RoadToolState
} from "../road";
import { CommitFailureCategory } from "../model";
import { seedRoadSections } from "../road_templates";
import type { DrawActionResult, WorldPoint } from "../store/viewer";
import { ViewerActionContext } from "./context";

export class RoadActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  clearPreview(): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        previewMeshes: [],
        previewState: "none",
        previewRequest: null,
        previewIssue: ""
      }
    }));
  }

  setMode(mode: RoadToolMode): void {
    this.ctx.store.update((current) => ({
      ...current,
      lastCommitFailure: null,
      road: {
        ...current.road,
        mode,
        phase: "start",
        previewMeshes: [],
        previewState: "none",
        previewRequest: null,
        previewIssue: "",
        lastError: ""
      }
    }));
  }

  setOperation(operation: RoadToolState["operation"]): void {
    this.ctx.store.update((current) => ({
      ...current,
      lastCommitFailure: null,
      road: {
        ...current.road,
        operation,
        phase: "start",
        markingDraftSegmentId: 0,
        hoveredDeleteSegmentId: 0,
        hoveredLaneSegmentId: 0,
        hoveredLaneId: 0,
        selectedLaneSegmentId: 0,
        selectedLaneId: 0,
        selectedLaneNodeId: 0,
        laneEditStage: "select",
        laneCorridorId: 0,
        laneTransitionStartSegmentId: 0,
        laneTransitionStartT: 0,
        laneTransitionCompleteSegmentId: 0,
        laneTransitionCompleteT: 0,
        laneContinuationEndSegmentId: 0,
        laneContinuationEndT: 0,
        selectedEditSegmentId: 0,
        selectedEditNodeAId: 0,
        selectedEditNodeBId: 0,
        activeEditPointIndex: -1,
        editPoints: [],
        previewMeshes: [],
        previewState: "none",
        previewRequest: null,
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

  selectRoadLayoutTemplate(templateId: number): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        selectedRoadLayoutTemplateId: templateId,
        junctionCornerRadiusM:
          current.road.roadJunctionCornerRadiusDefaults[templateId] ?? 4,
        previewIssue: "",
        lastError: ""
      }
    }));
  }

  setLaneTargetTemplate(templateId: number): void {
    this.ctx.store.update((current) => {
      const template = current.road.scene.roadLayoutTemplates.find((item) => item.id === templateId);
      return {
        ...current,
        road: {
          ...current.road,
          laneTargetTemplateId: templateId,
          laneTargetLaneId: template?.lanes[0]?.id ?? 0,
          laneTargetBoundaryId: template?.boundaries[0]?.id ?? 0,
          previewIssue: "",
          lastError: ""
        }
      };
    });
  }

  updateSelectedRoadLayoutTemplate(input: {
    sidewalkWidthM: number;
    laneWidthM: number;
    medianWidthM: number;
    hasCenterLine: boolean;
    hasOuterLines: boolean;
  }): void {
    const road = this.ctx.readSnapshot().road;
    this.finish(this.ctx.bridge.roadUpdateRoadLayoutTemplate({ id: road.selectedRoadLayoutTemplateId, ...input }),
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

  addViewportPoint(point: WorldPoint, snap?: RoadSnapInfo): DrawActionResult {
    const target = { x: point[0], y: point[1] };
    const current = this.ctx.readSnapshot().road;
    if (current.operation !== "draw") {
      this.applyOperation(current, target, snap);
      return { kind: "operation-applied" };
    }
    if (current.phase === "start") {
      this.beginAt(target, snap);
      return { kind: "anchor-accepted" };
    }
    const road = current.mode !== "line" ? withRoadCurveEnd(current, target) : withRoadEnd(current, target);
    const request = current.previewState === "guide" && current.previewRequest !== null &&
        sameRoadPoint({ x: current.previewRequest.endX, y: current.previewRequest.endY }, target)
      ? current.previewRequest
      : roadSegmentInput(road, snap);
    return this.commitInterval(request, target, true);
  }

  previewViewportPoint(point: WorldPoint, snap?: RoadSnapInfo): void {
    const current = this.ctx.readSnapshot().road;
    if (current.operation === "add-lane") {
      const hoveredLaneSegmentId = snap?.laneId === undefined ? 0 : snap.segmentId;
      const hoveredLaneId = snap?.laneId ?? 0;
      let road = { ...current, hoveredLaneSegmentId, hoveredLaneId };
      if (current.operation === "add-lane") {
        if (current.laneEditStage === "transition-complete" && snap !== undefined) {
          const position = segmentPositionForSnap(current, snap);
          if (position !== null &&
              positionBelongsToCorridor(current, position, current.laneCorridorId)) {
            road = {
              ...road,
              laneTransitionCompleteSegmentId: position.segmentId,
              laneTransitionCompleteT: position.t,
              previewMeshes: [],
              previewState: "guide",
              previewIssue: ""
            };
            this.ctx.store.update((snapshot) => ({ ...snapshot, road }));
            return;
          }
        }
        if (current.laneEditStage === "continuation-end" && snap !== undefined) {
          const position = segmentPositionForSnap(current, snap);
          if (position !== null &&
              positionBelongsToCorridor(current, position, current.laneCorridorId)) {
            road = {
              ...road,
              laneContinuationEndSegmentId: position.segmentId,
              laneContinuationEndT: position.t,
              previewMeshes: [],
              previewState: "guide",
              previewIssue: ""
            };
            this.ctx.store.update((snapshot) => ({ ...snapshot, road }));
            return;
          }
        }
      }
      this.ctx.store.update((snapshot) => ({ ...snapshot, road }));
      return;
    }
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
    // Core owns the curve rule, so the guide asks for the interval it would
    // commit instead of shaping one of its own.
    const shaped = this.ctx.bridge.roadPreviewInterval(roadSegmentInput(road, snap));
    const request = roadSegmentInput({
      ...road,
      handleA: { x: shaped.handleAX, y: shaped.handleAY },
      handleB: { x: shaped.handleBX, y: shaped.handleBY }
    }, snap);
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...road,
        previewMeshes: [],
        previewState: "guide",
        previewRequest: request,
        previewIssue: "",
        lastError: ""
      }
    }));
  }

  private applyOperation(road: RoadToolState, point: RoadPoint, snap?: RoadSnapInfo): void {
    if (road.operation === "add-lane") {
      this.applyLaneOperation(road, point, snap);
      return;
    }
    if (snap === undefined || snap.segmentId === 0) {
      this.rejectInput("road operation", "Select a road segment",
                       "road_segment_not_selected");
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
        this.rejectInput("road edit", "Road alignment is not editable",
                         "road_alignment_not_editable");
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
    this.rejectInput("road edit", "Select and drag a road control point",
                     "road_control_point_not_selected");
  }

  private applyLaneOperation(road: RoadToolState, _point: RoadPoint, snap?: RoadSnapInfo): void {
    if (road.operation === "add-lane") {
      if (road.laneEditStage === "select") {
        if (snap === undefined || snap.segmentId === 0) {
          this.rejectInput("road add lane",
                           "Select the road where the lane begins",
                           "lane_start_not_selected");
          return;
        }
        const position = segmentPositionForSnap(road, snap);
        const corridor = road.scene.corridors.find((item) =>
          item.segments.some((ref) => ref.segmentId === snap.segmentId)
        );
        if (position === null || corridor === undefined) {
          this.rejectInput("road add lane",
                           "Selected road is not in a corridor",
                           "lane_corridor_not_found");
          return;
        }
        const endpointDirections = road.scene.lanePaths
          .filter((lane) => lane.segmentId === snap.segmentId)
          .map((lane) => lane.direction);
        const templateDirections = road.scene.roadLayoutTemplates
          .find((template) => template.id === corridor?.roadLayoutTemplateId)
          ?.lanes.map((lane) => lane.direction) ?? [];
        const availableDirections = Array.from(new Set(
          endpointDirections.length > 0 ? endpointDirections : templateDirections
        ));
        const selectedLaneDirection = availableDirections.includes(
          road.selectedLaneDirection
        )
          ? road.selectedLaneDirection
          : availableDirections[0] ?? road.selectedLaneDirection;
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: {
            ...snapshot.road,
            laneEditStage: "transition-complete",
            laneCorridorId: corridor.id,
            selectedLaneSegmentId: snap.segmentId,
            selectedLaneDirection,
            laneTransitionStartSegmentId: position.segmentId,
            laneTransitionStartT: position.t,
            laneTransitionCompleteSegmentId: position.segmentId,
            laneTransitionCompleteT: position.t,
            laneContinuationEndSegmentId: 0,
            laneContinuationEndT: 0,
            previewIssue: "",
            lastError: ""
          }
        }));
        return;
      }
      if (road.laneEditStage === "transition-complete") {
        const position = snap === undefined ? null : segmentPositionForSnap(road, snap);
        if (position === null || !positionBelongsToCorridor(road, position, road.laneCorridorId)) {
          this.rejectInput("road add lane", "3車線が完成する位置を同じ道路上で選択してください",
                           "lane_transition_complete_not_selected");
          return;
        }
        const startDistance = corridorDistanceForPosition(
          road, { segmentId: road.laneTransitionStartSegmentId,
            t: road.laneTransitionStartT }, road.laneCorridorId
        );
        const completeDistance = corridorDistanceForPosition(
          road, position, road.laneCorridorId
        );
        if (startDistance === null || completeDistance === null ||
            Math.abs(completeDistance - startDistance) <= 1e-9) {
          this.rejectInput("road add lane", "開始位置とは別の道路上を選択してください",
                           "lane_transition_complete_not_after_start");
          return;
        }
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: {
            ...snapshot.road,
            laneEditStage: "continuation-end",
            laneTransitionCompleteSegmentId: position.segmentId,
            laneTransitionCompleteT: position.t,
            laneContinuationEndSegmentId: 0,
            laneContinuationEndT: 0,
            previewIssue: "",
            lastError: ""
          }
        }));
        return;
      }
      if (road.laneEditStage === "continuation-end") {
        const position = snap === undefined ? null : segmentPositionForSnap(road, snap);
        if (position === null || !positionBelongsToCorridor(road, position, road.laneCorridorId)) {
          this.rejectInput("road add lane", "3車線を維持する終点を同じ道路上で選択してください",
                           "lane_continuation_end_not_selected");
          return;
        }
        if (!positionContinuesAddLaneDirection(road, position)) {
          this.rejectInput("road add lane", "2点目より先の道路上を選択してください",
                           "lane_continuation_end_before_completion");
          return;
        }
        this.ctx.store.update((snapshot) => ({
          ...snapshot,
          road: {
            ...snapshot.road,
            laneContinuationEndSegmentId: position.segmentId,
            laneContinuationEndT: position.t,
            previewState: "guide",
            previewIssue: "",
            lastError: ""
          }
        }));
        return;
      }
      return;
    }
    if (road.laneEditStage === "select") {
      if (snap?.laneId === undefined || snap.endpointRole === undefined || snap.nodeId === 0) {
        this.rejectInput("road lane connection", "Select a lane endpoint",
                         "lane_endpoint_not_selected");
        return;
      }
      const path = road.scene.lanePaths.find((item) =>
        item.segmentId === snap.segmentId && item.laneId === snap.laneId);
      const sourceTemplateId = snap.endpointRole === 0
        ? path?.startRoadLayoutTemplateId
        : path?.endRoadLayoutTemplateId;
      const sourceTemplate = road.scene.roadLayoutTemplates.find((item) => item.id === sourceTemplateId);
      const targetTemplate = road.scene.roadLayoutTemplates.find((item) => item.id === road.laneTargetTemplateId) ??
        road.scene.roadLayoutTemplates[0];
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: {
          ...snapshot.road,
          selectedLaneSegmentId: snap.segmentId,
          selectedLaneId: snap.laneId ?? 0,
          selectedLaneEndpointRole: snap.endpointRole ?? 0,
          selectedLaneNodeId: snap.nodeId,
          selectedLaneDirection: snap.laneDirection ?? 0,
          laneEditStage: "target",
          laneTargetTemplateId: targetTemplate?.id ?? snapshot.road.selectedRoadLayoutTemplateId,
          laneTargetLaneId: snapshot.road.laneTargetLaneId || targetTemplate?.lanes[0]?.id || 0,
          laneSourceBoundaryId: snapshot.road.laneSourceBoundaryId || sourceTemplate?.boundaries[0]?.id || 0,
          laneTargetBoundaryId: snapshot.road.laneTargetBoundaryId || targetTemplate?.boundaries[0]?.id || 0,
          previewIssue: "",
          lastError: ""
        }
      }));
      return;
    }
  }

  cancelSession(): DrawActionResult {
    const current = this.ctx.readSnapshot().road;
    if (current.phase === "start" &&
        current.laneEditStage === "select" &&
        current.previewMeshes.length === 0 && current.previewIssue === "") {
      return { kind: "ignored", reasonCode: "session-inactive" };
    }
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...snapshot.road,
        phase: "start",
        draftStartNodeId: 0,
        draftStartSegmentId: 0,
        draftStartSegmentDistanceM: 0,
        draftExtensionCorridorId: 0,
        laneEditStage: "select",
        laneCorridorId: 0,
        selectedLaneSegmentId: 0,
        laneTransitionStartSegmentId: 0,
        laneTransitionStartT: 0,
        laneTransitionCompleteSegmentId: 0,
        laneTransitionCompleteT: 0,
        laneContinuationEndSegmentId: 0,
        laneContinuationEndT: 0,
        previewMeshes: [],
        previewState: "none",
        previewRequest: null,
        previewIssue: "",
        lastError: ""
      },
      error: "",
      lastCommitFailure: null
    }));
    return { kind: "session-ended" };
  }

  confirmSession(): DrawActionResult {
    const road = this.ctx.readSnapshot().road;
    if (road.operation === "add-lane") return this.commitLane(road);
    if (road.operation !== "draw" || road.previewRequest === null) {
      return { kind: "ignored", reasonCode: "preview-unavailable" };
    }
    const request = road.previewRequest;
    return this.commitInterval(
      request,
      { x: request.endX, y: request.endY },
      true
    );
  }

  undoCommitted(): void {
    const result = this.ctx.bridge.roadUndoSegment();
    this.finish(result, "road undo");
  }

  // Clearing throws away the whole road state, sections included, so the
  // catalogue is registered again before the workspace is usable.
  clear(): void {
    const result = this.ctx.bridge.roadClear();
    if (!result.ok) {
      this.finish(result, "road clear");
      return;
    }
    const seeded = seedRoadSections((section) =>
      this.ctx.bridge.roadAddRoadLayoutTemplate(section)
    );
    if (!seeded.ok) {
      this.finish({ ok: false, error: seeded.error }, "road clear");
      return;
    }
    this.ctx.store.update((current) => ({
      ...current,
      road: {
        ...current.road,
        roadLayoutTemplateLabels: seeded.sections.labels,
        roadJunctionCornerRadiusDefaults:
          seeded.sections.junctionCornerRadiusDefaults,
        selectedRoadLayoutTemplateId: seeded.sections.initialId,
        junctionCornerRadiusM:
          seeded.sections.junctionCornerRadiusDefaults[
            seeded.sections.initialId
          ] ?? 4
      }
    }));
    this.finish(result, "road clear");
  }

  commitPath(): DrawActionResult {
    const road = this.ctx.readSnapshot().road;
    if (road.operation === "add-lane") return this.commitLane(road);
    if (road.operation !== "draw") return { kind: "ignored", reasonCode: "non-draw-operation" };
    if (road.phase === "start") {
      return { kind: "ignored", reasonCode: "session-inactive" };
    }
    if (road.previewRequest === null) {
      return this.cancelSession();
    }
    const request = road.previewRequest;
    return this.commitInterval(
      request,
      { x: request.endX, y: request.endY },
      false
    );
  }

  private commitLane(road: RoadToolState): DrawActionResult {
    if (road.laneEditStage !== "continuation-end" ||
        road.laneContinuationEndSegmentId === 0 ||
        road.laneTransitionStartSegmentId === 0 ||
        road.laneTransitionCompleteSegmentId === 0 ||
        Math.abs(road.laneTransitionCompleteT - road.laneTransitionStartT) <= 1e-9) {
      return { kind: "ignored", reasonCode: "lane-range-incomplete" };
    }
    const result = this.ctx.bridge.roadAddLane(laneTransitionInput(road));
    this.finish(result, "road add lane");
    return result.ok
      ? { kind: "commit-succeeded" }
      : {
          kind: "commit-rejected",
          reasonCode: result.reasonCode || "road_add_lane_rejected"
        };
  }

  private commitInterval(
    request: ReturnType<typeof roadSegmentInput>,
    endpoint: RoadPoint,
    continueSession: boolean
  ): DrawActionResult {
    // A road is drawn with a cross section. Without one the workspace never
    // finished registering its catalogue.
    if (request.roadLayoutTemplateId === 0) {
      return { kind: "ignored", reasonCode: "no-section-selected" };
    }
    const result = this.ctx.bridge.roadAddSegment(request);
    if (!result.ok) {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: {
          ...snapshot.road,
          previewState: "guide",
          previewRequest: request,
          previewIssue: "",
          lastError: ""
        }
      }));
      this.ctx.store.setCommitFailure(result, "road segment", [endpoint.x, endpoint.y, 0]);
      return { kind: "commit-rejected", reasonCode: result.reasonCode || "road_commit_rejected" };
    }
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: continueSession
        ? withRoadEnd({
            ...snapshot.road,
            phase: "end",
            draftStart: endpoint,
            draftStartNodeId: result.endNodeId ?? 0,
            draftStartSegmentId: 0,
            draftStartSegmentDistanceM: 0,
            draftExtensionCorridorId: result.corridorId ?? 0,

            scene,
            previewMeshes: [],
            previewState: "none",
            previewRequest: null,
            previewIssue: "",
            lastError: ""
          }, endpoint)
        : {
            ...snapshot.road,
            phase: "start",
            draftStartNodeId: 0,
            draftStartSegmentId: 0,
            draftStartSegmentDistanceM: 0,
            draftExtensionCorridorId: 0,
            scene,
            previewMeshes: [],
            previewState: "none",
            previewRequest: null,
            previewIssue: "",
            lastError: ""
          },
      error: "",
      lastCommitFailure: null,
      logs: [...snapshot.logs, "road add segment"]
    }));
    return { kind: "commit-succeeded" };
  }

  private previewState(current: RoadToolState, target: RoadPoint): RoadToolState {
    if (current.mode === "line") {
      return withRoadEnd(current, target);
    }
    return withRoadCurveEnd(current, target);
  }

  private beginAt(target: RoadPoint, snap?: RoadSnapInfo): void {
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: withRoadEnd({
        ...snapshot.road,
        draftStart: target,
        draftStartNodeId: snap?.nodeId ?? 0,
        draftStartSegmentId: snap?.segmentId ?? 0,
        draftStartSegmentDistanceM: snap?.segmentDistanceM ?? 0,
        draftExtensionCorridorId: snap?.extensionCorridorId ?? 0,
        phase: "end"
      }, target),
      lastCommitFailure: null
    }));
  }

  private finish(result: { ok: boolean; error: string; failureCategory?: CommitFailureCategory; reasonCode?: string }, log: string): void {
    if (!result.ok) {
      this.ctx.store.setCommitFailure(result, log);
      return;
    }
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...snapshot.road,
        phase: "start",
        draftStartNodeId: 0,
        draftStartSegmentId: 0,
        draftStartSegmentDistanceM: 0,
        draftExtensionCorridorId: 0,
        markingDraftSegmentId: 0,
        activeEditPointIndex: -1,
        hoveredDeleteSegmentId: 0,
        hoveredLaneSegmentId: 0,
        hoveredLaneId: 0,
        selectedLaneSegmentId: 0,
        selectedLaneId: 0,
        selectedLaneNodeId: 0,
        laneEditStage: "select",
        scene,
        previewMeshes: [],
        previewState: "none",
        previewRequest: null,
        previewIssue: "",
        lastError: ""
      },
      error: "",
      lastCommitFailure: null,
      logs: [...snapshot.logs, log]
    }));
  }

  private rejectInput(operation: string, error: string,
                      reasonCode: string): void {
    this.ctx.store.setCommitFailure({
      ok: false,
      error,
      failureCategory: CommitFailureCategory.InvalidInput,
      reasonCode
    }, operation);
  }

  private applyPreview(
    road: RoadToolState,
    preview: {
      ok: boolean;
      error: string;
      failureCategory?: CommitFailureCategory;
      meshes: RoadToolState["previewMeshes"];
    },
    request: ReturnType<typeof roadSegmentInput> = roadSegmentInput(road)
  ): void {
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: {
        ...road,
        previewMeshes: preview.ok ? preview.meshes : road.previewMeshes,
        previewState: preview.ok ? "valid" : road.previewState,
        previewRequest: request,
        previewIssue: "",
        lastError: ""
      }
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
    endNodeId: 0,
    endSegmentId: 0,
    endSegmentDistanceM: 0,
    extensionCorridorId: 0,
    connectToFirstNode: false,
    roadLayoutTemplateId: road.selectedRoadLayoutTemplateId
  };
}

function sameRoadPoint(a: RoadPoint, b: RoadPoint): boolean {
  return Math.hypot(a.x - b.x, a.y - b.y) <= 1e-6;
}

function laneTransitionInput(road: RoadToolState) {
  return {
    corridorId: road.laneCorridorId,
    direction: road.selectedLaneDirection,
    side: road.laneSide,
    startSegmentId: road.laneTransitionStartSegmentId,
    startT: road.laneTransitionStartT,
    completeSegmentId: road.laneTransitionCompleteSegmentId,
    completeT: road.laneTransitionCompleteT,
    continuationEndSegmentId: road.laneContinuationEndSegmentId,
    continuationEndT: road.laneContinuationEndT,
    laneWidthM: road.laneWidthM
  };
}
function segmentPositionForSnap(
  road: RoadToolState,
  snap: RoadSnapInfo
): { segmentId: number; t: number } | null {
  const ref = road.scene.corridors
    .flatMap((corridor) => corridor.segments)
    .find((item) => item.segmentId === snap.segmentId);
  if (ref === undefined || ref.lengthM <= 1e-9) return null;
  return {
    segmentId: snap.segmentId,
    t: Math.max(0, Math.min(1, snap.segmentDistanceM / ref.lengthM))
  };
}

function positionBelongsToCorridor(
  road: RoadToolState,
  position: { segmentId: number; t: number },
  corridorId: number
): boolean {
  return road.scene.corridors
    .find((corridor) => corridor.id === corridorId)
    ?.segments.some((ref) => ref.segmentId === position.segmentId) ?? false;
}

function corridorDistanceForPosition(
  road: RoadToolState,
  position: { segmentId: number; t: number },
  corridorId: number
): number | null {
  const corridor = road.scene.corridors.find((item) => item.id === corridorId);
  if (corridor === undefined) return null;
  let distance = 0;
  for (const ref of corridor.segments) {
    if (ref.segmentId === position.segmentId) {
      const localDistance = Math.max(0, Math.min(1, position.t)) * ref.lengthM;
      return distance + (ref.reversed ? ref.lengthM - localDistance : localDistance);
    }
    distance += ref.lengthM;
  }
  return null;
}

function positionContinuesAddLaneDirection(
  road: RoadToolState,
  position: { segmentId: number; t: number }
): boolean {
  const start = corridorDistanceForPosition(
    road,
    { segmentId: road.laneTransitionStartSegmentId,
      t: road.laneTransitionStartT },
    road.laneCorridorId
  );
  const complete = corridorDistanceForPosition(
    road,
    { segmentId: road.laneTransitionCompleteSegmentId,
      t: road.laneTransitionCompleteT },
    road.laneCorridorId
  );
  const end = corridorDistanceForPosition(road, position, road.laneCorridorId);
  if (start === null || complete === null || end === null) return false;
  const delta = complete - start;
  if (Math.abs(delta) <= 1e-9) return false;
  return (end - complete) * Math.sign(delta) >= -1e-9;
}
