import {
  roadSegmentInput,
  withRoadBend,
  withRoadEnd,
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

  setConnectToFirstNode(value: boolean): void {
    this.ctx.store.update((current) => ({
      ...current,
      road: { ...current.road, connectToFirstNode: value, lastError: "" }
    }));
  }

  addViewportPoint(point: WorldPoint): void {
    const target = { x: point[0], y: point[1] };
    const current = this.ctx.readSnapshot().road;
    if (current.phase === "start") {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: withRoadEnd({ ...snapshot.road, draftStart: target, phase: "end" }, target)
      }));
      return;
    }
    if (current.mode === "bezier" && current.phase === "end") {
      this.ctx.store.update((snapshot) => ({
        ...snapshot,
        road: { ...withRoadEnd(snapshot.road, target), phase: "bend" }
      }));
      return;
    }
    const road = current.phase === "bend" ? withRoadBend(current, target) : withRoadEnd(current, target);
    this.commit(road);
  }

  previewViewportPoint(point: WorldPoint): void {
    const current = this.ctx.readSnapshot().road;
    if (current.phase === "start") return;
    const target: RoadPoint = { x: point[0], y: point[1] };
    const road = current.phase === "bend" ? withRoadBend(current, target) : withRoadEnd(current, target);
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
    const scene = this.ctx.bridge.roadScene();
    this.ctx.store.update((snapshot) => ({
      ...snapshot,
      road: withRoadEnd({
        ...road,
        phase: "end",
        draftStart: nextStart,
        scene,
        previewMeshes: [],
        lastError: ""
      }, nextStart),
      error: "",
      logs: [...snapshot.logs, "road add segment"]
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
      road: { ...snapshot.road, phase: "start", scene, previewMeshes: [], lastError: "" },
      error: "",
      logs: [...snapshot.logs, log]
    }));
  }
}
