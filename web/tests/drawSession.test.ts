import { describe, expect, it, vi } from "vitest";
import { DrawSessionController, type DrawSessionDomain } from "../src/actions/draw_session";

function domain(): DrawSessionDomain {
  return {
    primary: vi.fn(),
    preview: vi.fn(),
    enter: vi.fn(),
    escape: vi.fn(),
    undo: vi.fn()
  };
}

describe("shared draw session input", () => {
  it("dispatches pointer and keyboard events to exactly one active domain", () => {
    let active: "wire" | "road" = "road";
    const road = domain();
    const wire = domain();
    const session = new DrawSessionController(() => active, { road, wire });

    session.primary([1, 2, 3]);
    session.preview([4, 5, 6]);
    session.enter();
    session.escape();
    session.undo();

    expect(road.primary).toHaveBeenCalledOnce();
    expect(road.preview).toHaveBeenCalledOnce();
    expect(road.enter).toHaveBeenCalledOnce();
    expect(road.escape).toHaveBeenCalledOnce();
    expect(road.undo).toHaveBeenCalledOnce();
    expect(wire.primary).not.toHaveBeenCalled();
    expect(wire.preview).not.toHaveBeenCalled();
  });

  it("cancels only the previous transient session when switching tools", () => {
    let active: "wire" | "road" = "road";
    const road = domain();
    const wire = domain();
    const session = new DrawSessionController(() => active, { road, wire });

    session.switchTool("wire", (next) => { active = next; });
    session.switchTool("wire", (next) => { active = next; });

    expect(road.escape).toHaveBeenCalledOnce();
    expect(road.undo).not.toHaveBeenCalled();
    expect(wire.escape).not.toHaveBeenCalled();
    expect(active).toBe("wire");
  });
});
