import { describe, expect, it, vi } from "vitest";
import { ViewerActions } from "../src/actions/viewer";
import type { SceneData, WireBridge } from "../src/bridge/wire";
import type {
  BundleTemplateInfo,
  CableTemplateInfo,
  PoleTemplateInfo
} from "../src/model";
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
  it("refreshes finite scene data after path generation", () => {
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
      poles: [],
      ports: [],
      spans: [],
      supportNodes: []
    };
    const bridge = {
      bundleTemplates: () => [
        {
          id: 0, name: "DEFAULT_SINGLE", defaultCount: 1, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMirror: true, allowMidairNode: true, allowMidairBranch: true,
          groupedSupportFanoutSpacing: 0.2, supportStyle: 0, branchPolicy: 0,
          continuityPolicy: 0,
          populationRules: []
        }
      ],
      cableTemplates: () => [],
      poleTemplates: () => [],
      geometrySettings: () => ({
        curveSamples: 16,
        sagEnabled: true,
        sagFactor: 0.03,
        poleClearance: 0.05
      }),
      layoutSettings: () => ({
        angleCorrectionEnabled: true,
        cornerThresholdDeg: 70,
        minSideScale: 1,
        maxSideScale: 1.7
      }),
      visualSettings: () => ({
        enableSupportStructures: true,
        enableInsulators: true,
        supportCenterThreshold: 0.03,
        supportArmExtra: 0.2,
        insulatorRadius: 0.07,
        insulatorLength: 0.16
      }),
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

    actions.addPathPoint([0, 0, 0]);
    actions.addPathPoint([16, 2, 0]);
    actions.addPathPoint([32, 0, 0]);
    actions.generatePath();

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
        {
          id: 0, name: "DEFAULT_SINGLE", defaultCount: 1, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMirror: true, allowMidairNode: true, allowMidairBranch: true,
          groupedSupportFanoutSpacing: 0.2, supportStyle: 0, branchPolicy: 0,
          continuityPolicy: 0,
          populationRules: []
        }
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
    store.update((current) => ({
      ...current,
      parts: [existingPart],
      pathPoints: [
        [0, 0, 0],
        [10, 0, 0]
      ],
      bundleTemplates: [
        {
          id: 0, name: "DEFAULT_SINGLE", defaultCount: 1, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMirror: true, allowMidairNode: true, allowMidairBranch: true,
          groupedSupportFanoutSpacing: 0.2, supportStyle: 0, branchPolicy: 0,
          continuityPolicy: 0,
          populationRules: []
        }
      ],
      selectedBundleTemplateId: 0,
      selectedDrawBundleTemplateIds: [0]
    }));

    new ViewerActions(bridge, store).generatePath();

    const snapshot = current(store);
    expect(generatedPoints).toEqual(new Float64Array([0, 0, 0, 10, 0, 0]));
    expect(snapshot.parts).toEqual([existingPart]);
    expect(snapshot.error).toBe("backbone unsupported: test failure");
  });
});

const bundleTemplate: BundleTemplateInfo = {
  id: 0,
  name: "DEFAULT_SINGLE",
  defaultCount: 1,
  fixedCount: true,
  fixedCountValue: 1,
  minCount: 1,
  maxCount: 1,
  cableTemplateId: 2,
  relatedPoleTypeId: 1,
  defaultLayer: 2,
  allowMirror: true,
  allowMidairNode: true,
  allowMidairBranch: true,
  groupedSupportFanoutSpacing: 0.2,
  supportStyle: 0,
  branchPolicy: 0,
  continuityPolicy: 0,
  populationRules: []
};

const cableTemplate: CableTemplateInfo = {
  id: 2,
  name: "LV",
  outerDiameter: 0.03,
  bendStiffness: 1,
  minBendRadius: 0.2,
  materialStyle: 2,
  colorRgba: 0xffffffff,
  requiresInsulator: false,
  insulatorAttachmentHeight: 0,
  sagFactor: 0.03,
  slackFactor: 0,
  groupedFanoutSpacing: 0.2,
  continuityPolicy: 0,
  supplementalEnabled: false,
  supplementalLateralOffset: 0,
  supplementalVerticalOffset: 0,
  supplementalWobbleAmplitude: 0,
  supplementalWobbleWavelength: 0,
  supplementalWobblePhase: 0,
  supplementalEndpointEnvelope: 0
};

const poleTemplate: PoleTemplateInfo = {
  id: 1,
  name: "DistributionPole",
  description: "default",
  defaultHeight: 10,
  portBands: [],
  anchorSlots: []
};

function actionBridge(overrides: Partial<WireBridge> = {}): WireBridge {
  const emptyScene: SceneData = {
    parts: [],
    poles: [],
    ports: [],
    spans: [],
    supportNodes: []
  };
  return {
    bundleTemplates: () => [bundleTemplate],
    cableTemplates: () => [cableTemplate],
    poleTemplates: () => [poleTemplate],
    geometrySettings: () => ({
      curveSamples: 16,
      sagEnabled: true,
      sagFactor: 0.03,
      poleClearance: 0.05
    }),
    layoutSettings: () => ({
      angleCorrectionEnabled: true,
      cornerThresholdDeg: 70,
      minSideScale: 1,
      maxSideScale: 1.7
    }),
    visualSettings: () => ({
      enableSupportStructures: true,
      enableInsulators: true,
      supportCenterThreshold: 0.03,
      supportArmExtra: 0.2,
      insulatorRadius: 0.07,
      insulatorLength: 0.16
    }),
    scene: () => emptyScene,
    ...overrides
  } as WireBridge;
}

describe("P1 action contracts", () => {
  it("uses the desktop viewer template defaults", () => {
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({
        bundleTemplates: () => [
          bundleTemplate,
          { ...bundleTemplate, id: 1, name: "HV_3PH" }
        ],
        cableTemplates: () => [
          cableTemplate,
          { ...cableTemplate, id: 7, name: "HV_BARE" }
        ],
        poleTemplates: () => [
          poleTemplate,
          { ...poleTemplate, id: 9, name: "CommunicationPole" }
        ]
      }),
      store
    );

    actions.initialize();

    const snapshot = current(store);
    expect(snapshot.selectedBundleTemplateId).toBe(1);
    expect(snapshot.selectedCableTemplateId).toBe(7);
    expect(snapshot.selectedPoleTemplateId).toBe(9);
  });
  it("enables sag on startup like the desktop viewer", () => {
    let geometry = {
      curveSamples: 8,
      sagEnabled: false,
      sagFactor: 0.03,
      poleClearance: 0.05
    };
    const update = vi.fn((next: typeof geometry) => {
      geometry = { ...next };
      return { ok: true, error: "" };
    });
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({
        geometrySettings: () => geometry,
        updateGeometrySettings: update
      }),
      store
    );

    actions.initialize();

    expect(update).toHaveBeenCalledWith(expect.objectContaining({ sagEnabled: true }));
    expect(current(store).geometry.sagEnabled).toBe(true);
  });


  it("sends geometry preview and logs only its commit", async () => {
    vi.useFakeTimers();
    const update = vi.fn(() => ({ ok: true, error: "" }));
    const bridge = actionBridge({ updateGeometrySettings: update });
    const store = new ViewerStore();
    const actions = new ViewerActions(bridge, store);
    actions.initialize();

    actions.previewGeometry("sagFactor", 0.05);
    expect(current(store).logs).toEqual([]);
    actions.recordFrame(16);
    actions.recordFrame(42);
    await vi.advanceTimersByTimeAsync(33);
    expect(update).toHaveBeenLastCalledWith(
      expect.objectContaining({ sagFactor: 0.05 })
    );

    actions.commitGeometry("sagFactor", 0.06);
    expect(update).toHaveBeenLastCalledWith(
      expect.objectContaining({ sagFactor: 0.06 })
    );
    expect(current(store).logs).toHaveLength(1);
    expect(current(store).lastInteractionFrames).toEqual({
      sampleCount: 2,
      maxFrameMs: 42,
      longFrameCount: 1
    });
    vi.useRealTimers();
  });

  it("routes layout failure to the visible error", () => {
    const update = vi.fn(() => ({ ok: false, error: "layout unsupported" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ updateLayoutSettings: update }),
      store
    );
    actions.initialize();

    actions.commitLayout("cornerThresholdDeg", 55);

    expect(update).toHaveBeenCalledWith(
      expect.objectContaining({ cornerThresholdDeg: 55 })
    );
    expect(current(store).error).toBe("layout unsupported");
    expect(current(store).layout.cornerThresholdDeg).toBe(70);
  });

  it("sends visual settings through the visual API", () => {
    const update = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ updateVisualSettings: update }),
      store
    );
    actions.initialize();

    actions.commitVisual("enableInsulators", false);

    expect(update).toHaveBeenCalledWith(
      expect.objectContaining({ enableInsulators: false })
    );
  });

  it("keeps related pole application independent from bundle update", () => {
    const updateBundle = vi.fn(() => ({ ok: true, error: "" }));
    const applyRelated = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({
        updateBundleTemplate: updateBundle,
        applyRelatedPoleType: applyRelated
      }),
      store
    );
    actions.initialize();

    actions.commitBundleTemplate({ ...bundleTemplate, allowMirror: false });
    expect(updateBundle).toHaveBeenCalledOnce();
    expect(applyRelated).not.toHaveBeenCalled();

    actions.applyRelatedPoleType(bundleTemplate.id);
    expect(applyRelated).toHaveBeenCalledWith(bundleTemplate.id);
  });

  it("sends population rules through bundle template update", () => {
    const update = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ updateBundleTemplate: update }),
      store
    );
    actions.initialize();

    actions.commitBundleTemplate({
      ...bundleTemplate,
      populationRules: [
        {
          ruleId: 7,
          explicitSeed: 11,
          priority: 2,
          minExtraCount: 1,
          maxExtraCount: 3,
          minSpacing: 0.08,
          lateralMin: -0.4,
          lateralMax: 0.4,
          heightMin: 5,
          heightMax: 6,
          randomness: 0.5
        }
      ]
    });

    expect(update).toHaveBeenCalledWith(
      expect.objectContaining({
        populationRules: [
          expect.objectContaining({ ruleId: 7, maxExtraCount: 3, minSpacing: 0.08 })
        ]
      })
    );
  });

  it("previews bundle population numeric edits without logging a commit", async () => {
    vi.useFakeTimers();
    const update = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ updateBundleTemplate: update }),
      store
    );
    actions.initialize();
    const template = {
      ...bundleTemplate,
      populationRules: [
        {
          ruleId: 1,
          explicitSeed: 1,
          priority: 0,
          minExtraCount: 1,
          maxExtraCount: 1,
          minSpacing: 0.09,
          lateralMin: -1,
          lateralMax: 1,
          heightMin: 0,
          heightMax: 20,
          randomness: 0.25
        }
      ]
    };

    actions.previewBundleTemplate(
      template,
      "bundle.population.1.minSpacing",
      "minSpacing",
      0.05,
      0.09
    );
    await vi.advanceTimersByTimeAsync(33);

    expect(update).toHaveBeenCalledWith(
      expect.objectContaining({
        populationRules: [
          expect.objectContaining({ ruleId: 1, minSpacing: 0.09 })
        ]
      })
    );
    expect(current(store).logs).toEqual([]);
    vi.useRealTimers();
  });

  it("sends cable shape updates with the selected template", () => {
    const update = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ updateCableTemplate: update }),
      store
    );
    actions.initialize();

    actions.commitCableTemplate({ ...cableTemplate, sagFactor: 0.08 });

    expect(update).toHaveBeenCalledWith(
      expect.objectContaining({ id: cableTemplate.id, sagFactor: 0.08 }),
      []
    );
  });

  it("sends pole placement updates through pole template update", () => {
    const update = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ updatePoleTemplate: update }),
      store
    );
    actions.initialize();

    actions.commitPoleTemplate({ ...poleTemplate, defaultHeight: 11 });

    expect(update).toHaveBeenCalledWith(
      expect.objectContaining({ id: poleTemplate.id, defaultHeight: 11 })
    );
  });

  it("routes tilt, reset, and override operations explicitly", () => {
    const tilt = vi.fn(() => ({ ok: true, error: "" }));
    const reset = vi.fn(() => ({ ok: true, error: "" }));
    const clearPole = vi.fn(() => ({ ok: true, error: "" }));
    const clearSocket = vi.fn(() => ({ ok: true, error: "" }));
    const clearBranch = vi.fn(() => ({ ok: true, error: "" }));
    const bridge = actionBridge({
      scene: () => ({
        parts: [],
        poles: [
          {
            id: "42",
            height: 10,
            positionX: 0, positionY: 0, positionZ: 0,
            rotationX: 0, rotationY: 0, rotationZ: 0,
            scaleX: 1, scaleY: 1, scaleZ: 1
          }
        ],
        ports: [],
        spans: [],
        supportNodes: []
      }),
      applyPoleTilt: tilt,
      resetSpanReferenceLengths: reset,
      clearPoleOrientationOverride: clearPole,
      clearSpanSocketOverride: clearSocket,
      clearSpanBranchDownOverride: clearBranch
    });
    const store = new ViewerStore();
    const actions = new ViewerActions(bridge, store);
    actions.initialize();
    store.update((current) => ({
      ...current,
      poles: bridge.scene().poles
    }));

    actions.applyTiltToAll(3);
    expect(tilt).toHaveBeenCalledWith(["42"], 3);
    actions.resetSpanReferenceLengths();
    expect(reset).toHaveBeenCalledOnce();

    actions.select("pole", "42");
    actions.clearSelectedOverride("pole");
    expect(clearPole).toHaveBeenCalledWith("42");
    actions.select("span", "77");
    actions.clearSelectedOverride("socketA");
    actions.clearSelectedOverride("socketB");
    actions.clearSelectedOverride("branchDown");
    expect(clearSocket).toHaveBeenNthCalledWith(1, "77", true);
    expect(clearSocket).toHaveBeenNthCalledWith(2, "77", false);
    expect(clearBranch).toHaveBeenCalledWith("77");
  });

  it("updates draw and selection state without touching the bridge", () => {
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge(), store);
    actions.initialize();

    actions.setDrawOption("directionMode", 2);
    actions.setDrawOption("showPreview", false);
    actions.select("span", "8");

    expect(current(store)).toEqual(
      expect.objectContaining({
        directionMode: 2,
        showPreview: false,
        selection: { kind: "span", id: "8" }
      })
    );
    actions.clearSelection();
    expect(current(store).selection).toBeNull();
  });
});
