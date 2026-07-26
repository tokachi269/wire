import {
  addRoadManualArea,
  addRoadManualLine,
  addRoadSegment,
  addRoadTransition,
  rebuildRoadDerived,
  updateRoadDraftPoint,
  type RoadToolState
} from "../road";
import { ViewerActionContext } from "./context";

export class RoadActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  setRoadMode(mode: RoadToolState["mode"]): void {
    this.update((road) => ({ ...road, mode, lastError: "" }), `road mode ${mode}`);
  }

  setRoadConnectToFirstNode(value: boolean): void {
    this.update((road) => rebuildRoadDerived({ ...road, connectToFirstNode: value }), "road connect toggle");
  }

  updateRoadDraftPoint(
    key: "draftStart" | "draftEnd" | "handleA" | "handleB",
    axis: "x" | "y",
    value: number
  ): void {
    this.update((road) => updateRoadDraftPoint(road, key, axis, value), "road draft edit");
  }

  addRoadSegment(): void {
    this.update(addRoadSegment, "road add segment");
  }

  addRoadTransition(): void {
    this.update(addRoadTransition, "road add transition");
  }

  addRoadManualLine(): void {
    this.update(addRoadManualLine, "road add manual line");
  }

  addRoadManualArea(): void {
    this.update(addRoadManualArea, "road add manual area");
  }

  private update(change: (road: RoadToolState) => RoadToolState, log: string): void {
    this.ctx.store.update((current) => {
      const road = change(current.road);
      return {
        ...current,
        road,
        logs: road.lastError === "" ? [...current.logs, log] : current.logs,
        error: road.lastError
      };
    });
  }
}
