import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { describe, expect, it } from "vitest";
import { ViewerActions } from "../src/actions/viewer";
import { WireBridge } from "../src/bridge/wire";
import { ViewerStore, type ViewerSnapshot } from "../src/store/viewer";
import { WorkspaceCache } from "../src/store/workspace";

describe("workspace persistence", () => {
  it("restores authoritative core state and viewer settings in a new instance", async () => {
    const wasmBinary = await readFile(resolve("src/wasm-generated/wire_web_core.wasm"));
    const values = new Map<string, string>();
    const cache = new WorkspaceCache({
      getItem: (key) => values.get(key) ?? null,
      setItem: (key, value) => { values.set(key, value); },
      removeItem: (key) => { values.delete(key); }
    });

    const firstBridge = await WireBridge.create({ wasmBinary });
    const firstStore = new ViewerStore();
    const firstActions = new ViewerActions(firstBridge, firstStore, cache);
    firstActions.initialize();
    const factorySagFactor = firstBridge.geometrySettings().sagFactor;
    firstActions.commitGeometry("sagFactor", factorySagFactor + 0.015);
    firstActions.setDrawOption("cameraFov", 67);
    firstActions.setDrawOption("showLeftPanel", false);
    firstActions.setDrawOption("maxTiltDeg", 0);
    firstActions.addPathPoint([0, 0, 0]);
    firstActions.addPathPoint([20, 0, 0]);
    firstActions.generatePath();
    expect(firstBridge.scene().spans.length).toBeGreaterThan(0);
    firstActions.flushWorkspaceCache();
    firstActions.dispose();
    firstBridge.dispose();

    const restoredBridge = await WireBridge.create({ wasmBinary });
    const restoredStore = new ViewerStore();
    const restoredActions = new ViewerActions(restoredBridge, restoredStore, cache);
    restoredActions.initialize();

    expect(restoredBridge.geometrySettings().sagFactor)
      .toBeCloseTo(factorySagFactor + 0.015, 8);
    expect(restoredBridge.scene().spans.length).toBeGreaterThan(0);
    expect(current(restoredStore)).toEqual(expect.objectContaining({
      cameraFov: 67,
      showLeftPanel: false
    }));

    restoredActions.resetWorkspace();
    expect(restoredBridge.geometrySettings().sagFactor)
      .toBeCloseTo(factorySagFactor, 8);
    expect(restoredBridge.scene().spans).toEqual([]);
    expect(current(restoredStore)).toEqual(expect.objectContaining({
      cameraFov: 48,
      showLeftPanel: true,
      pathPoints: []
    }));
    restoredActions.dispose();
    restoredBridge.dispose();
  });
});

function current(store: ViewerStore): ViewerSnapshot {
  let snapshot: ViewerSnapshot | undefined;
  const unsubscribe = store.value.subscribe((value) => {
    snapshot = value;
  });
  unsubscribe();
  if (snapshot === undefined) throw new Error("store did not emit");
  return snapshot;
}
