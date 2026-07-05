import { describe, expect, it } from "vitest";
import { ViewerActions } from "../src/actions/viewer";
import type { SceneData, WireBridge } from "../src/bridge/wire";
import { ViewerStore, type ViewerSnapshot } from "../src/store/viewer";

function current(store: ViewerStore): ViewerSnapshot {
  let snapshot: ViewerSnapshot | undefined;
  const unsubscribe = store.value.subscribe((value) => {
    snapshot = value;
  });
  unsubscribe();
  if (snapshot === undefined) {
    throw new Error("store did not emit");
  }
  return snapshot;
}

describe("viewer actions", () => {
  it("refreshes finite scene data after sample generation", () => {
    const scene: SceneData = {
      parts: [
        {
          info: {
            kind: 0,
            wireRadius: 0.02,
            colorRgba: 0xffffffff,
            sourceNodeId: "0",
            sourceEdgeId: "1",
            sourceSpanId: "2",
            sourceBundleId: "3",
            bundleTemplateId: 0,
            laneIndex: 0,
            sampleCount: 2
          },
          samples: new Float64Array([0, 0, 2, 10, 0, 2])
        }
      ],
      poles: []
    };
    const bridge = {
      bundleTemplates: () => [
        { id: 0, name: "DEFAULT_SINGLE", defaultCount: 1, fixedCount: true }
      ],
      generate: () => ({
        ok: true,
        error: "",
        generatedPoleCount: 2,
        generatedSpanCount: 1,
        totalMs: 1.25
      }),
      scene: () => scene
    } as unknown as WireBridge;
    const store = new ViewerStore();
    const actions = new ViewerActions(bridge, store);
    actions.initialize();

    actions.generateSample();

    const snapshot = current(store);
    expect(snapshot.parts).toHaveLength(1);
    expect([...snapshot.parts[0].samples].every(Number.isFinite)).toBe(true);
    expect(snapshot.generationMs).toBe(1.25);
    expect(snapshot.error).toBe("");
  });

  it("keeps derived parts when generation fails", () => {
    let generatedPoints: Float64Array | undefined;
    const bridge = {
      bundleTemplates: () => [
        { id: 0, name: "DEFAULT_SINGLE", defaultCount: 1, fixedCount: true }
      ],
      generate: (points: Float64Array) => {
        generatedPoints = points;
        return {
          ok: false,
          error: "backbone unsupported: test failure",
          generatedPoleCount: 0,
          generatedSpanCount: 0,
          totalMs: 0
        };
      },
      scene: () => {
        throw new Error("scene must not refresh after failure");
      }
    } as unknown as WireBridge;
    const store = new ViewerStore();
    const existingPart = {
      info: {
        kind: 0,
        wireRadius: 0.02,
        colorRgba: 0xffffffff,
        sourceNodeId: "0",
        sourceEdgeId: "1",
        sourceSpanId: "2",
        sourceBundleId: "3",
        bundleTemplateId: 0,
        laneIndex: 0,
        sampleCount: 2
      },
      samples: new Float64Array([0, 0, 2, 10, 0, 2])
    };
    store.replace({
      parts: [existingPart],
      poles: [],
      error: "",
      generationMs: null,
      pathPoints: [
        [0, 0, 0],
        [10, 0, 0]
      ],
      bundleTemplates: [
        { id: 0, name: "DEFAULT_SINGLE", defaultCount: 1, fixedCount: true }
      ],
      selectedBundleTemplateId: 0
    });

    new ViewerActions(bridge, store).generatePath();

    const snapshot = current(store);
    expect(generatedPoints).toEqual(new Float64Array([0, 0, 0, 10, 0, 0]));
    expect(snapshot.parts).toEqual([existingPart]);
    expect(snapshot.error).toBe("backbone unsupported: test failure");
  });
});
