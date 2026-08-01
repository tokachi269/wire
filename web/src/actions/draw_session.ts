import type { PathPickInfo } from "../model";
import type { RoadSnapInfo } from "../road";
import type { WorldPoint } from "../store/viewer";

export type DrawTool = "wire" | "road";
export type DrawPick = PathPickInfo | RoadSnapInfo | undefined;

export interface DrawSessionDomain {
  primary(point: WorldPoint, pick: DrawPick): void;
  preview(point: WorldPoint, pick: DrawPick): void;
  enter(): void;
  escape(): void;
  undo(): void;
}

export class DrawSessionController {
  constructor(
    private readonly activeTool: () => DrawTool,
    private readonly domains: Record<DrawTool, DrawSessionDomain>
  ) {}

  primary(point: WorldPoint, pick?: DrawPick): void {
    this.domains[this.activeTool()].primary(point, pick);
  }

  preview(point: WorldPoint, pick?: DrawPick): void {
    this.domains[this.activeTool()].preview(point, pick);
  }

  enter(): void {
    this.domains[this.activeTool()].enter();
  }

  escape(): void {
    this.domains[this.activeTool()].escape();
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
