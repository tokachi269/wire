import { describe, expect, it, vi } from "vitest";
import { ViewerActions } from "../src/actions/viewer";
import type { SceneData, WireBridge } from "../src/bridge/wire";
import type {
  BundleTemplateInfo,
  BundlePlacement,
  CableTemplateInfo,
  GenerationTiming,
  PoleTemplateInfo
} from "../src/model";
import { ViewerStore, type ViewerSnapshot } from "../src/store/viewer";
import { WorkspaceCache } from "../src/store/workspace";

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

describe("workspace cache", () => {
  it("restores core state and viewer preferences, then resets both", async () => {
    vi.useFakeTimers();
    const values = new Map<string, string>();
    const storage = {
      get: async (key: string) => values.get(key) ?? null,
      set: async (key: string, value: string) => { values.set(key, value); },
      remove: async (key: string) => { values.delete(key); }
    };
    const cache = new WorkspaceCache(storage);
    let firstCoreState = "factory-core";
    const firstStore = new ViewerStore();
    const firstActions = new ViewerActions(
      actionBridge({
        saveState: () => firstCoreState,
        loadState: (text: string) => {
          firstCoreState = text;
          return { ok: true, error: "" };
        }
      }),
      firstStore,
      cache
    );
    firstActions.initialize();
    await firstActions.restoreWorkspace();
    firstCoreState = "edited-core";
    firstActions.setDrawOption("cameraFov", 73);
    firstActions.setDrawOption("showRightPanel", false);
    firstActions.addPathPoint([1, 2, 3]);
    await vi.advanceTimersByTimeAsync(251);
    firstActions.dispose();

    let secondCoreState = "fresh-core";
    const secondStore = new ViewerStore();
    const secondActions = new ViewerActions(
      actionBridge({
        saveState: () => secondCoreState,
        loadState: (text: string) => {
          secondCoreState = text;
          return { ok: true, error: "" };
        }
      }),
      secondStore,
      cache
    );
    secondActions.initialize();
    await secondActions.restoreWorkspace();

    expect(secondCoreState).toBe("edited-core");
    expect(current(secondStore)).toEqual(expect.objectContaining({
      cameraFov: 73,
      showRightPanel: false,
      pathPoints: []
    }));
    expect((await cache.read())?.viewer).not.toHaveProperty("pathPoints");
    expect((await cache.read())?.viewer).not.toHaveProperty("pathPointSpecs");

    await secondActions.resetWorkspace();
    expect(secondCoreState).toBe("fresh-core");
    expect(current(secondStore)).toEqual(expect.objectContaining({
      cameraFov: 48,
      showRightPanel: true,
      pathPoints: []
    }));
    expect(await cache.read()).toEqual(expect.objectContaining({
      coreState: "fresh-core",
      viewer: expect.objectContaining({ cameraFov: 48 })
    }));
    secondActions.dispose();
    vi.useRealTimers();
  });

  it("preserves a workspace that the current core cannot restore", async () => {
    const values = new Map<string, string>();
    const cache = new WorkspaceCache({
      get: async (key: string) => values.get(key) ?? null,
      set: async (key: string, value: string) => { values.set(key, value); },
      remove: async (key: string) => { values.delete(key); }
    });
    const store = new ViewerStore();
    await cache.write("future-core-state", current(store));
    const actions = new ViewerActions(
      actionBridge({
        saveState: () => "factory-core",
        loadState: () => ({ ok: false, error: "loaded state failed validation: Example" })
      }),
      store,
      cache
    );

    actions.initialize();
    await actions.restoreWorkspace();
    await actions.flushWorkspaceCache();

    expect((await cache.read())?.coreState).toBe("future-core-state");
    expect(current(store).error).toContain("loaded state failed validation: Example");
    actions.dispose();
  });
});

function timing(totalMs: number): GenerationTiming {
  return {
    prepareMs: 0.1,
    checkMs: 0.1,
    pairsMs: 0.1,
    preflightMs: 0.1,
    intentMs: 0.1,
    supportGroupsMs: 0.1,
    emitMs: 0.1,
    saveGraphMs: 0.1,
    rulesMs: 0.1,
    layoutMs: 0.1,
    geomMs: 0.1,
    drawMs: 0.1,
    totalMs
  };
}

describe("viewer actions", () => {
  it("refreshes finite scene data after path generation", () => {
    const scene: SceneData = {
      parts: [
        {
          info: {
            partKey: "edge:2",
            sourceVersion: "1",
            sampleOffset: 0,
            kind: 0,
            supplementalKind: 0,
            wireRadius: 0.02,
            colorRgba: 0xffffffff,
            sourceNodeId: "0",
            sourceEdgeId: "1",
            sourceSpanId: "2",
            sourceBundleId: "3",
            bundleTemplateId: 0,
            laneIndex: 0,
            runId: 1,
            sampleCount: 2
          },
          samples: new Float64Array([0, 0, 2, 10, 0, 2])
        }
      ],
      models: [],
      poles: [],
      ports: [],
      spans: [],
      supportNodes: [],
      backboneEdges: []
    };
    const bridge = {
      bundleTemplates: () => [
        {
          id: 102, kind: 0, name: "DEFAULT_SINGLE", defaultCount: 1, defaultSpacing: 0.2, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMidairNode: true, allowMidairBranch: true,
          supportWirePoleBandId: 0,
          rowFixtureAssemblyId: 0,
          endpointFixtureAssemblyId: 0,
          spanVisualAssembly: {
            supportPathEnabled: false,
            helixEnabled: false, helixRadius: 0, helixClearance: 0, helixTurnsPerMeter: 0,
            helixSamplesPerTurn: 16, endpointTrim: 0, memberWanderRatio: 0,
            memberWanderWavelength: 0, memberWanderPhaseBias: 0,
            memberTwistTurnsPerMeter: 0, memberTwistPhase: 0
          },
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
        enableInsulators: true,
        insulatorRadius: 0.07,
        insulatorLength: 0.16
      }),
      saveState: () => "factory-state",
      generate: () => ({
        ok: true,
        error: "",
        generatedPoleCount: 2,
        generatedSpanCount: 1,
        totalMs: 1.25,
        timing: timing(1.25)
      }),
      scene: () => scene
    } as unknown as WireBridge;
    const store = new ViewerStore();
    const actions = new ViewerActions(bridge, store);
    actions.initialize();

    actions.addPathPoint([0, 0, 0]);
    actions.addPathPoint([16, 2, 0]);
    actions.addPathPoint([32, 0, 0]);
    actions.recordSceneContentSync({
      total: 1, reused: 0, rebuilt: 1, removed: 0,
      modelTotal: 0, modelReused: 0, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
    actions.generatePath();

    const snapshot = current(store);
    expect(snapshot.parts).toHaveLength(1);
    expect([...snapshot.parts[0].samples].every(Number.isFinite)).toBe(true);
    expect(snapshot.generationMs).toBe(1.25);
    expect(snapshot.error).toBe("");
    expect(snapshot.logs.at(-1)).toMatch(
      /^route generated: 2 poles \/ 1 spans \| perf core=1\.25ms wasm-call=.+ms scene=.+ms action=.+ms emit=0\.10ms save-graph=0\.10ms geom=0\.10ms \| scene parts total=1 reused=0 rebuilt=1 removed=0 models total=0 reused=0 updated=0 rebuilt=0 removed=0$/
    );
  });

  it("keeps derived parts when generation fails", () => {
    let generatedPoints: Float64Array | undefined;
    const bridge = {
      bundleTemplates: () => [
        {
          id: 102, kind: 0, name: "DEFAULT_SINGLE", defaultCount: 1, defaultSpacing: 0.2, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMidairNode: true, allowMidairBranch: true,
           supportWirePoleBandId: 0,
           rowFixtureAssemblyId: 0,
           endpointFixtureAssemblyId: 0,
          spanVisualAssembly: {
            supportPathEnabled: false,
            helixEnabled: false, helixRadius: 0, helixClearance: 0, helixTurnsPerMeter: 0,
            helixSamplesPerTurn: 16, endpointTrim: 0, memberWanderRatio: 0,
            memberWanderWavelength: 0, memberWanderPhaseBias: 0,
            memberTwistTurnsPerMeter: 0, memberTwistPhase: 0
          },
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
          totalMs: 0,
          timing: timing(0)
        };
      },
      scene: () => {
        throw new Error("scene must not refresh after failure");
      }
    } as unknown as WireBridge;
    const store = new ViewerStore();
    const existingPart = {
      info: {
        partKey: "edge:2",
        sourceVersion: "1",
        sampleOffset: 0,
        kind: 0,
        supplementalKind: 0,
        wireRadius: 0.02,
        colorRgba: 0xffffffff,
        sourceNodeId: "0",
        sourceEdgeId: "1",
        sourceSpanId: "2",
        sourceBundleId: "3",
        bundleTemplateId: 0,
        laneIndex: 0,
        runId: 1,
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
          id: 102, kind: 0, name: "DEFAULT_SINGLE", defaultCount: 1, defaultSpacing: 0.2, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMidairNode: true, allowMidairBranch: true,
          supportWirePoleBandId: 0,
          rowFixtureAssemblyId: 0,
          endpointFixtureAssemblyId: 0,
          spanVisualAssembly: {
            supportPathEnabled: false,
            helixEnabled: false, helixRadius: 0, helixClearance: 0, helixTurnsPerMeter: 0,
            helixSamplesPerTurn: 16, endpointTrim: 0, memberWanderRatio: 0,
            memberWanderWavelength: 0, memberWanderPhaseBias: 0,
            memberTwistTurnsPerMeter: 0, memberTwistPhase: 0
          },
          populationRules: []
        }
      ],
      selectedBundleTemplateId: 102,
      drawBundlePlacements: [{
        id: 1, bundleTemplateId: 102, count: 1,
        explicit: true, height: 7.4, offset: 0, spacing: 0.2
      }]
    }));

    new ViewerActions(bridge, store).generatePath();

    const snapshot = current(store);
    expect(generatedPoints).toEqual(new Float64Array([0, 0, 0, 10, 0, 0]));
    expect(snapshot.parts).toEqual([existingPart]);
    expect(snapshot.error).toBe("backbone unsupported: test failure");
  });
});

const bundleTemplate: BundleTemplateInfo = {
  id: 102,
  kind: 0,
  name: "DEFAULT_SINGLE",
  defaultCount: 1,
  defaultSpacing: 0.2,
  fixedCount: true,
  fixedCountValue: 1,
  minCount: 1,
  maxCount: 1,
  cableTemplateId: 2,
  relatedPoleTypeId: 1,
  defaultLayer: 2,
  allowMidairNode: true,
  allowMidairBranch: true,
  supportWirePoleBandId: 0,
  rowFixtureAssemblyId: 0,
  endpointFixtureAssemblyId: 0,
  spanVisualAssembly: {
    supportPathEnabled: false,
    helixEnabled: false, helixRadius: 0, helixClearance: 0, helixTurnsPerMeter: 0,
    helixSamplesPerTurn: 16, endpointTrim: 0, memberWanderRatio: 0,
    memberWanderWavelength: 0, memberWanderPhaseBias: 0,
    memberTwistTurnsPerMeter: 0, memberTwistPhase: 0
  },
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
  sagFactor: 0.03,
  slackFactor: 0,
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
  poleVisualAssemblyId: 0,
  portBands: [],
  anchorSlots: []
};

function actionBridge(overrides: Partial<WireBridge> = {}): WireBridge {
  const emptyScene: SceneData = {
    parts: [],
    models: [],
    poles: [],
    ports: [],
    spans: [],
    supportNodes: [],
    backboneEdges: []
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
      enableInsulators: true,
      insulatorRadius: 0.07,
      insulatorLength: 0.16
    }),
    updateCableTemplate: () => ({ ok: true, error: "" }),
    updateBackboneBundlePlacement: () => ({ ok: true, error: "" }),
    updatePoleTemplate: () => ({ ok: true, error: "" }),
    saveState: () => "factory-state",
    loadState: () => ({ ok: true, error: "" }),
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
    expect(snapshot.selectedBundleTemplateId).toBe(102);
    expect(snapshot.selectedCableTemplateId).toBe(7);
    expect(snapshot.selectedPoleTemplateId).toBe(9);
  });

  it("applies the Japan distribution primitive placement set on startup", () => {
    const cableUpdates: CableTemplateInfo[] = [];
    const poleUpdates: PoleTemplateInfo[] = [];
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({
        cableTemplates: () => [
          { ...cableTemplate, id: 1, name: "HV_BARE", outerDiameter: 0.03 },
          { ...cableTemplate, id: 2, name: "LV_INSULATED", outerDiameter: 0.03 },
          { ...cableTemplate, id: 3, name: "COMM_MULTI", outerDiameter: 0.03 },
          { ...cableTemplate, id: 4, name: "OPTICAL_FIBER", outerDiameter: 0.03 }
        ],
        poleTemplates: () => [{
          ...poleTemplate,
          name: "CommunicationPole",
          defaultHeight: 11.35,
          portBands: [
            { bandId: 1, category: 0, layer: 2, side: 0, role: 1, lateralCenter: -0.6, lateralMin: -0.8, lateralMax: -0.4, heightCenter: 9, heightMin: 8.8, heightMax: 9.2, priority: 3, minSpacing: 0.2, allowMultiple: false, overflowPolicy: 0, enabled: true },
            { bandId: 2, category: 0, layer: 2, side: 1, role: 1, lateralCenter: 0, lateralMin: -0.2, lateralMax: 0.2, heightCenter: 9, heightMin: 8.8, heightMax: 9.2, priority: 2, minSpacing: 0.2, allowMultiple: false, overflowPolicy: 0, enabled: true },
            { bandId: 3, category: 0, layer: 2, side: 2, role: 1, lateralCenter: 0.6, lateralMin: 0.4, lateralMax: 0.8, heightCenter: 9, heightMin: 8.8, heightMax: 9.2, priority: 1, minSpacing: 0.2, allowMultiple: false, overflowPolicy: 0, enabled: true },
            { bandId: 4, category: 1, layer: 1, side: 0, role: 1, lateralCenter: -0.4, lateralMin: -0.5, lateralMax: -0.3, heightCenter: 8, heightMin: 7.8, heightMax: 8.2, priority: 1, minSpacing: 0.2, allowMultiple: true, overflowPolicy: 2, enabled: true },
            { bandId: 5, category: 2, layer: 1, side: 1, role: 1, lateralCenter: 0.2, lateralMin: 0, lateralMax: 0.4, heightCenter: 6.2, heightMin: 6, heightMax: 6.4, priority: 1, minSpacing: 0.2, allowMultiple: true, overflowPolicy: 2, enabled: true },
            { bandId: 6, category: 3, layer: 1, side: 1, role: 1, lateralCenter: 0.1, lateralMin: 0, lateralMax: 0.4, heightCenter: 6.6, heightMin: 6.4, heightMax: 6.8, priority: 1, minSpacing: 0.2, allowMultiple: true, overflowPolicy: 2, enabled: true },
            { bandId: 7, category: 4, layer: 0, side: 1, role: 3, lateralCenter: 0, lateralMin: -0.1, lateralMax: 0.1, heightCenter: 4.2, heightMin: 4.1, heightMax: 4.3, priority: 1, minSpacing: 0.2, allowMultiple: true, overflowPolicy: 2, enabled: true }
          ]
        }],
        updateCableTemplate: (template) => {
          cableUpdates.push(template);
          return { ok: true, error: "" };
        },
        updatePoleTemplate: (template) => {
          poleUpdates.push(template);
          return { ok: true, error: "" };
        }
      }),
      store
    );

    actions.initialize();

    expect(cableUpdates.map((template) => [template.name, template.outerDiameter])).toEqual([
      ["HV_BARE", 0.024],
      ["LV_INSULATED", 0.020],
      ["COMM_MULTI", 0.016],
      ["OPTICAL_FIBER", 0.012]
    ]);
    expect(poleUpdates).toHaveLength(1);
    expect(poleUpdates[0].defaultHeight).toBe(10);
    expect(poleUpdates[0].portBands.filter((band) => band.category === 0).map((band) => band.lateralCenter)).toEqual([
      -0.75,
      0,
      0.75
    ]);
    expect(poleUpdates[0].portBands.find((band) => band.category === 1)?.heightCenter).toBe(7.4);
    expect(poleUpdates[0].portBands.find((band) => band.category === 2)).toEqual(
      expect.objectContaining({ heightMin: 4.8, heightMax: 5.8 })
    );
    expect(poleUpdates[0].portBands.find((band) => band.category === 3)).toEqual(
      expect.objectContaining({ heightMin: 4.8, heightMax: 5.8 })
    );
    expect(poleUpdates[0].portBands.find((band) => band.category === 4)).toEqual(
      expect.objectContaining({ heightMin: 4.5, heightMax: 6.5 })
    );
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

    actions.commitBundleTemplate({ ...bundleTemplate, allowMidairNode: false });
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
          heightMax: 6
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
          heightMax: 20
        }
      ]
    };

    actions.previewBundleTemplate(
      template,
      "bundle.population.1.minSpacing",
      "minSpacing",
      0.05
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
        models: [],
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
        supportNodes: [],
        backboneEdges: []
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
    actions.select("pole", "42");
    actions.applyTiltToSelection(4);
    expect(tilt).toHaveBeenCalledWith(["42"], 4);
    actions.resetSpanReferenceLengths();
    expect(reset).toHaveBeenCalledOnce();

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

  it("ignores a path point that overlaps the previous point", () => {
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge(), store);
    actions.initialize();

    actions.addPathPoint([1, 2, 3]);
    actions.addPathPoint([1, 2, 3]);

    expect(current(store).pathPoints).toEqual([[1, 2, 3]]);
  });

  it("does not send stale generated bundle ids during route generation", () => {
    let placements: BundlePlacement[] | undefined;
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({
      generate: (_points, sentPlacements) => {
        placements = sentPlacements as BundlePlacement[];
        return {
          ok: true,
          error: "",
          generatedPoleCount: 1,
          generatedSpanCount: 1,
          generatedBundleIds: ["new-bundle"],
          totalMs: 1,
          timing: timing(1)
        };
      }
    }), store);
    actions.initialize();
    store.update((snapshot) => ({
      ...snapshot,
      drawBundlePlacements: snapshot.drawBundlePlacements.map((placement) => ({
        ...placement,
        generatedBundleId: "stale-bundle"
      }))
    }));

    actions.addPathPoint([0, 0, 0]);
    actions.addPathPoint([10, 0, 0]);

    expect(placements?.[0]?.generatedBundleId).toBeUndefined();
  });

  it("passes resolved source snap node identity through to generation", () => {
    let nodeSpecs: Array<{ pointIndex: number; supportKind: number; nodeId: string }> | undefined;
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({
      resolveBranchPick: () => ({
        ok: true,
        error: "",
        positionX: 5,
        positionY: 0,
        positionZ: 4,
        supportKind: 1,
        nodeId: "9001"
      }),
      generate: (_points, _placements, _interval, _poleType, _direction, _tilt, specs) => {
        nodeSpecs = Array.isArray(specs) ? specs : undefined;
        return {
          ok: true,
          error: "",
          generatedPoleCount: 1,
          generatedSpanCount: 1,
          totalMs: 1,
          timing: timing(1)
        };
      },
      scene: () => ({
        parts: [],
        models: [],
        poles: [],
        ports: [],
        spans: [],
        supportNodes: [],
        backboneEdges: []
      })
    }), store);
    actions.initialize();

    actions.addPathPoint([5, 0, 0], {
      hitKind: 2,
      hitId: "0",
      hitX: 5,
      hitY: 0,
      hitZ: 0,
      hasSegmentEndpoints: true,
      segmentNodeAId: "10",
      segmentNodeBId: "11",
      segmentEndpointAX: 0,
      segmentEndpointAY: 0,
      segmentEndpointAZ: 0,
      segmentEndpointBX: 10,
      segmentEndpointBY: 0,
      segmentEndpointBZ: 0
    });
    actions.addPathPoint([5, 8, 0]);
    actions.generatePath();

    expect(nodeSpecs).toEqual([{ pointIndex: 0, supportKind: 1, nodeId: "9001" }]);
  });

  it("starts a new path after generation by default and undoes one point", () => {
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({
      generate: () => ({
        ok: true,
        error: "",
        generatedPoleCount: 2,
        generatedSpanCount: 1,
        totalMs: 1,
        timing: timing(1)
      }),
      scene: () => ({
        parts: [],
        models: [],
        poles: [],
        ports: [],
        spans: [],
        supportNodes: [],
        backboneEdges: []
      })
    }), store);
    actions.initialize();
    actions.addPathPoint([0, 0, 0]);
    actions.addPathPoint([10, 0, 0]);

    actions.generatePath();
    expect(current(store).pathPoints).toEqual([]);

    actions.addPathPoint([20, 0, 0]);
    actions.addPathPoint([30, 0, 0]);
    actions.undoPathPoint();
    expect(current(store).pathPoints).toEqual([[20, 0, 0]]);
  });
});
