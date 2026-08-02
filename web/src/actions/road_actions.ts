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
        curveContinuationTangent: null,
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
        curveContinuationTangent: null,
        markingDraftSegmentId: 0,
        hoveredDeleteSegmentId: 0,
        hoveredLaneSegmentId: 0,
        hoveredLaneId: 0,
        selectedLaneSegmentId: 0,
        selectedLaneId: 0,
        selectedLaneNodeId: 0,
        laneEditStage: "select",
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

  setLaneTargetTemplate(templateId: number): void {
    this.ctx.store.update((current) => {
      const template = current.road.scene.sectionTemplates.find((item) => item.id === templateId);
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
      : roadSegmentInput(road);
    return this.commitInterval(request, target, true);
  }

  previewViewportPoint(point: WorldPoint, snap?: RoadSnapInfo): void {
    const current = this.ctx.readSnapshot().road;
    if (["add-lane", "branch-lane", "merge-lane"].includes(current.operation)) {
      const hoveredLaneSegmentId = snap?.laneId === undefined ? 0 : snap.segmentId;
      const hoveredLaneId = snap?.laneId ?? 0;
      let road = { ...current, hoveredLaneSegmentId, hoveredLaneId };
      if (current.laneEditStage === "target") {
        if (current.operation === "add-lane" && snap !== undefined) {
          const distance = corridorDistanceForSnap(current, snap);
          if (distance !== null && distance.corridorId === current.laneCorridorId) {
            road = {
              ...road,
              laneFullWidthCorridorDistanceM: distance.distanceM,
              previewMeshes: [],
              previewState: "guide",
              previewIssue: ""
            };
            this.ctx.store.update((snapshot) => ({ ...snapshot, road }));
            return;
          }
        } else if (current.operation !== "add-lane") {
          this.applyPreview(road, this.ctx.bridge.roadPreviewConnectedLaneSegment(
            connectedLaneInput(road, { x: point[0], y: point[1] })
          ));
          return;
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
    const request = roadSegmentInput(road);
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
    if (["add-lane", "branch-lane", "merge-lane"].includes(road.operation)) {
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
        this.rejectInput("road add manual line",
                         "Manual line endpoints must use the same road segment",
                         "manual_line_segment_mismatch");
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
    this.rejectInput("road edit", "Select and drag a road control point",
                     "road_control_point_not_selected");
  }

  private applyLaneOperation(road: RoadToolState, point: RoadPoint, snap?: RoadSnapInfo): void {
    if (road.operation === "add-lane") {
      if (road.laneEditStage === "select") {
        if (snap === undefined || snap.segmentId === 0) {
          this.rejectInput("road add lane",
                           "Select the road where the lane begins",
                           "lane_start_not_selected");
          return;
        }
        const distance = corridorDistanceForSnap(road, snap);
        if (distance === null) {
          this.rejectInput("road add lane",
                           "Selected road is not in a corridor",
                           "lane_corridor_not_found");
          return;
        }
        const corridor = road.scene.corridors.find(
          (item) => item.id === distance.corridorId
        );
        const endpointDirections = road.scene.lanePaths
          .filter((lane) => lane.segmentId === snap.segmentId)
          .map((lane) => lane.direction);
        const templateDirections = road.scene.sectionTemplates
          .find((template) => template.id === corridor?.sectionTemplateId)
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
            laneEditStage: "target",
            laneCorridorId: distance.corridorId,
            selectedLaneSegmentId: snap.segmentId,
            selectedLaneDirection,
            laneStartCorridorDistanceM: distance.distanceM,
            laneFullWidthCorridorDistanceM: distance.distanceM,
            previewIssue: "",
            lastError: ""
          }
        }));
        return;
      }
      this.finish(this.ctx.bridge.roadAddLane(laneTransitionInput(road)), "road add lane");
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
        ? path?.startSectionTemplateId
        : path?.endSectionTemplateId;
      const sourceTemplate = road.scene.sectionTemplates.find((item) => item.id === sourceTemplateId);
      const targetTemplate = road.scene.sectionTemplates.find((item) => item.id === road.laneTargetTemplateId) ??
        road.scene.sectionTemplates[0];
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
          laneTargetTemplateId: targetTemplate?.id ?? snapshot.road.selectedSectionTemplateId,
          laneTargetLaneId: snapshot.road.laneTargetLaneId || targetTemplate?.lanes[0]?.id || 0,
          laneSourceBoundaryId: snapshot.road.laneSourceBoundaryId || sourceTemplate?.boundaries[0]?.id || 0,
          laneTargetBoundaryId: snapshot.road.laneTargetBoundaryId || targetTemplate?.boundaries[0]?.id || 0,
          previewIssue: "",
          lastError: ""
        }
      }));
      return;
    }
    this.finish(
      this.ctx.bridge.roadAddConnectedLaneSegment(connectedLaneInput(road, point)),
      road.operation === "branch-lane" ? "road branch lane" : "road merge lane"
    );
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
        curveContinuationTangent: null,
        laneEditStage: "select",
        laneCorridorId: 0,
        selectedLaneSegmentId: 0,
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

  clear(): void {
    const result = this.ctx.bridge.roadClear();
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
    if (road.laneEditStage !== "target" ||
        Math.abs(road.laneFullWidthCorridorDistanceM -
                 road.laneStartCorridorDistanceM) <= 1e-9) {
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
            curveContinuationTangent: request.kind === "bezier"
              ? { x: request.endX - request.handleBX, y: request.endY - request.handleBY }
              : null,
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
            curveContinuationTangent: null,
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
        curveContinuationTangent: null,
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
        curveContinuationTangent: null,
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
    extensionCorridorId: 0,
    connectToFirstNode: false,
    sectionTemplateId: road.selectedSectionTemplateId
  };
}

function sameRoadPoint(a: RoadPoint, b: RoadPoint): boolean {
  return Math.hypot(a.x - b.x, a.y - b.y) <= 1e-6;
}

function laneTransitionInput(road: RoadToolState) {
  const startCorridorDistanceM = Math.min(
    road.laneStartCorridorDistanceM,
    road.laneFullWidthCorridorDistanceM
  );
  const fullWidthCorridorDistanceM = Math.max(
    road.laneStartCorridorDistanceM,
    road.laneFullWidthCorridorDistanceM
  );
  return {
    corridorId: road.laneCorridorId,
    direction: road.selectedLaneDirection,
    side: road.laneSide,
    startCorridorDistanceM,
    fullWidthCorridorDistanceM,
    laneWidthM: road.laneWidthM
  };
}

function connectedLaneInput(road: RoadToolState, end: RoadPoint) {
  const node = road.scene.nodes.find((item) => item.id === road.selectedLaneNodeId);
  const start = node === undefined ? end : { x: node.x, y: node.y };
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const selectedEndpoint = {
    segmentId: road.selectedLaneSegmentId,
    laneId: road.selectedLaneId,
    endpointRole: road.selectedLaneEndpointRole
  };
  const selectedBoundary = {
    segmentId: road.selectedLaneSegmentId,
    boundaryId: road.laneSourceBoundaryId,
    endpointRole: road.selectedLaneEndpointRole
  };
  const branch = road.operation === "branch-lane";
  return {
    kind: "line" as const,
    startX: start.x,
    startY: start.y,
    endX: end.x,
    endY: end.y,
    handleAX: start.x + dx / 3,
    handleAY: start.y + dy / 3,
    handleBX: start.x + dx * 2 / 3,
    handleBY: start.y + dy * 2 / 3,
    startNodeId: road.selectedLaneNodeId,
    startSegmentId: 0,
    startSegmentDistanceM: 0,
    extensionCorridorId: 0,
    connectToFirstNode: false,
    sectionTemplateId: road.laneTargetTemplateId,
    laneConnections: branch ? [{
      source: selectedEndpoint,
      targetLaneId: road.laneTargetLaneId,
      kind: 3
    }] : [],
    boundaryContinuations: branch && road.laneSourceBoundaryId !== 0 && road.laneTargetBoundaryId !== 0 ? [{
      source: selectedBoundary,
      targetBoundaryId: road.laneTargetBoundaryId,
      kind: 2
    }] : [],
    sourceLaneConnections: branch ? [] : [{
      sourceLaneId: road.laneTargetLaneId,
      target: selectedEndpoint,
      kind: 2
    }],
    sourceBoundaryContinuations: !branch && road.laneSourceBoundaryId !== 0 && road.laneTargetBoundaryId !== 0 ? [{
      sourceBoundaryId: road.laneTargetBoundaryId,
      target: selectedBoundary,
      kind: 1
    }] : []
  };
}

function corridorDistanceForSnap(
  road: RoadToolState,
  snap: RoadSnapInfo
): { corridorId: number; distanceM: number } | null {
  for (const corridor of road.scene.corridors) {
    let offset = 0;
    for (const segment of corridor.segments) {
      if (segment.segmentId === snap.segmentId) {
        return {
          corridorId: corridor.id,
          distanceM: offset + (segment.reversed
            ? segment.lengthM - snap.segmentDistanceM
            : snap.segmentDistanceM)
        };
      }
      offset += segment.lengthM;
    }
  }
  return null;
}
