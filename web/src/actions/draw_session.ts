import type { PathPickInfo } from "../model";
import type { RoadSnapInfo } from "../road";
import type { DrawActionResult, WorldPoint } from "../store/viewer";

export type DrawTool = "wire" | "road";
export type DrawPick = PathPickInfo | RoadSnapInfo | undefined;

export interface DrawSessionDomain {
  primary(point: WorldPoint, pick: DrawPick): DrawActionResult;
  confirm(): DrawActionResult;
  preview(point: WorldPoint, pick: DrawPick): void;
  leave(): void;
  enter(): DrawActionResult;
  escape(): DrawActionResult;
  undo(): void;
}

export class DrawSessionController {
  constructor(
    private readonly activeTool: () => DrawTool,
    private readonly domains: Record<DrawTool, DrawSessionDomain>
  ) {}

  primary(point: WorldPoint, pick?: DrawPick): DrawActionResult {
    return this.domains[this.activeTool()].primary(point, pick);
  }

  confirm(): DrawActionResult {
    return this.domains[this.activeTool()].confirm();
  }

  preview(point: WorldPoint, pick?: DrawPick): void {
    this.domains[this.activeTool()].preview(point, pick);
  }

  leave(): void {
    this.domains[this.activeTool()].leave();
  }

  enter(): DrawActionResult {
    return this.domains[this.activeTool()].enter();
  }

  escape(): DrawActionResult {
    return this.domains[this.activeTool()].escape();
  }

  undo(): void {
    this.domains[this.activeTool()].undo();
  }

  switchTool(next: DrawTool, activate: (tool: DrawTool) => void): void {
    const previous = this.activeTool();
    if (previous === next) return;
    this.domains[previous].escape();
    activate(next);
  }
}
