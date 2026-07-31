import { describe, expect, it, vi } from "vitest";
import { readFileSync } from "node:fs";
import { ViewerActions } from "../src/actions/viewer";
import type { SceneData, WireBridge } from "../src/bridge/wire";
import { startConsoleLogging } from "../src/consoleLog";
import { EditErrorKind } from "../src/model";
import type { RoadSegmentInput } from "../src/road";
import type {
  BundleTemplateInfo,
  BundlePlacement,
  CableTemplateInfo,
  GenerationTiming,
  PoleTemplateInfo
} from "../src/model";
import { ViewerStore, type ViewerSnapshot } from "../src/store/viewer";
import { decodeWorkspaceText, WorkspaceCache, WORKSPACE_CACHE_KEY } from "../src/store/workspace";
import { missingBackboneEntryCells } from "./backbone_semantics_contract";

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
    let firstRoadState = "factory-road";
    const firstStore = new ViewerStore();
    const firstActions = new ViewerActions(
      actionBridge({
        saveState: () => firstCoreState,
        loadState: (text: string) => {
          firstCoreState = text;
          return { ok: true, error: "" };
        },
        roadSaveState: () => firstRoadState,
        roadLoadState: (text: string) => {
          firstRoadState = text;
          return { ok: true, error: "" };
        }
      }),
      firstStore,
      cache
    );
    firstActions.initialize();
    await firstActions.restoreWorkspace();
    firstCoreState = "edited-core";
    firstRoadState = "edited-road";
    firstActions.setDrawOption("cameraFov", 73);
    firstActions.setDrawOption("showRightPanel", false);
    firstActions.setDrawOption("rightPanelMode", "road");
    firstActions.setDrawOption("workspaceLeftWidth", 240);
    firstActions.setDrawOption("workspaceWidth", 370);
    firstActions.addPathPoint([1, 2, 3]);
    await vi.advanceTimersByTimeAsync(251);
    firstActions.dispose();

    let secondCoreState = "fresh-core";
    let secondRoadState = "fresh-road";
    const secondStore = new ViewerStore();
    const secondActions = new ViewerActions(
      actionBridge({
        saveState: () => secondCoreState,
        loadState: (text: string) => {
          secondCoreState = text;
          return { ok: true, error: "" };
        },
        roadSaveState: () => secondRoadState,
        roadLoadState: (text: string) => {
          secondRoadState = text;
          return { ok: true, error: "" };
        }
      }),
      secondStore,
      cache
    );
    secondActions.initialize();
    await secondActions.restoreWorkspace();

    expect(secondCoreState).toBe("edited-core");
    expect(secondRoadState).toBe("edited-road");
    expect(current(secondStore)).toEqual(expect.objectContaining({
      cameraFov: 73,
      showRightPanel: false,
      rightPanelMode: "road",
      workspaceLeftWidth: 240,
      workspaceWidth: 370,
      pathPoints: []
    }));
    expect((await cache.read())?.viewer).not.toHaveProperty("pathPoints");
    expect((await cache.read())?.viewer).not.toHaveProperty("pathPointSpecs");

    await secondActions.resetWorkspace();
    expect(secondCoreState).toBe("fresh-core");
    expect(secondRoadState).toBe("fresh-road");
    expect(current(secondStore)).toEqual(expect.objectContaining({
      cameraFov: 48,
      showRightPanel: false,
      rightPanelMode: "wire",
      workspaceLeftWidth: 240,
      workspaceWidth: 370,
      pathPoints: []
    }));
    expect(await cache.read()).toEqual(expect.objectContaining({
      coreState: "fresh-core",
      roadState: "fresh-road",
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

  it("stores compressed workspace text and reads legacy plain JSON", async () => {
    const values = new Map<string, string>();
    const cache = new WorkspaceCache({
      get: async (key: string) => values.get(key) ?? null,
      set: async (key: string, value: string) => { values.set(key, value); },
      remove: async (key: string) => { values.delete(key); }
    });
    const store = new ViewerStore();
    const coreState = `wire_state_v2\n${"edge_bundle ".repeat(128)}`;

    await cache.write(coreState, current(store));

    const stored = values.get(WORKSPACE_CACHE_KEY);
    expect(stored).toBeDefined();
    expect(stored?.startsWith("{")).toBe(false);
    const decoded = await decodeWorkspaceText(stored!);
    expect(JSON.parse(decoded).coreState).toBe(coreState);
    expect((await cache.read())?.coreState).toBe(coreState);

    const legacy = JSON.stringify({
      version: 1,
      coreState: "plain-core-state",
      viewer: current(store)
    });
    values.set(WORKSPACE_CACHE_KEY, legacy);

    expect((await cache.read())?.coreState).toBe("plain-core-state");
  });

  it("exports compressed workspace text and imports compressed or legacy plain documents", async () => {
    const sourceStore = new ViewerStore();
    const sourceActions = new ViewerActions(
      actionBridge({
        saveState: () => "workspace-core-state"
      }),
      sourceStore
    );
    sourceActions.initialize();
    sourceActions.setDrawOption("cameraFov", 66);

    const exported = await sourceActions.exportWorkspaceText();
    expect(exported.startsWith("{")).toBe(false);
    expect(JSON.parse(await decodeWorkspaceText(exported)).coreState).toBe("workspace-core-state");

    let loadedState = "";
    const targetStore = new ViewerStore();
    const targetActions = new ViewerActions(
      actionBridge({
        loadState: (text: string) => {
          loadedState = text;
          return { ok: true, error: "" };
        }
      }),
      targetStore
    );
    targetActions.initialize();
    await targetActions.importWorkspaceText(exported);
    expect(loadedState).toBe("workspace-core-state");
    expect(current(targetStore).cameraFov).toBe(66);

    const legacyPlain = JSON.stringify({
      version: 1,
      coreState: "plain-file-core",
      viewer: {
        ...current(sourceStore),
        cameraFov: 71
      }
    });
    await targetActions.importWorkspaceText(legacyPlain);
    expect(loadedState).toBe("plain-file-core");
    expect(current(targetStore).cameraFov).toBe(71);
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
          enableBranchDownOffset: false, branchEndpointOffset: 0,
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
      roadSaveState: () => "factory-road-state",
      roadScene: () => actionBridge().roadScene(),
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
          id: 102, kind: 0, category: 1, name: "DEFAULT_SINGLE", defaultCount: 1, defaultSpacing: 0.2, fixedCount: true,
          fixedCountValue: 1, minCount: 1, maxCount: 1,
          cableTemplateId: 2, relatedPoleTypeId: 1, defaultLayer: 2,
          allowMidairNode: true, allowMidairBranch: true,
          enableBranchDownOffset: false, branchEndpointOffset: 0,
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
  category: 1,
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
  enableBranchDownOffset: false,
  branchEndpointOffset: 0,
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
    resolveDefaultBundlePlacement: () => ({
      ok: true,
      error: "",
      height: 7.4,
      offset: 0,
      spacing: bundleTemplate.defaultSpacing
    }),
    updateCableTemplate: () => ({ ok: true, error: "" }),
    updateBackboneBundlePlacement: () => ({ ok: true, error: "" }),
    updatePoleTemplate: () => ({ ok: true, error: "" }),
    clearPendingSupportNodes: () => ({ ok: true, error: "" }),
    roadAddSegment: () => ({ ok: true, error: "" }),
    roadPreviewSegment: () => ({ ok: true, error: "", meshes: [] }),
    roadScene: () => ({
      segmentCount: 0,
      sectionTemplateCount: 1,
      transitionCount: 0,
      markingCount: 0,
      connectionGateCount: 0,
      junctionCount: 0,
      corridorCount: 0,
      nodes: [],
      centerlineSegments: [],
      corridors: [],
      sectionTemplates: [],
      editableSegments: [],
      surfaceMeshes: [],
      markingMeshes: []
    }),
    roadUndoSegment: () => ({ ok: true, error: "" }),
    roadDeleteSection: () => ({ ok: true, error: "" }),
    roadDeleteRange: () => ({ ok: true, error: "" }),
    roadMoveNode: () => ({ ok: true, error: "" }),
    roadPreviewMoveNode: () => ({ ok: true, error: "", meshes: [] }),
    roadEditSegment: () => ({ ok: true, error: "" }),
    roadPreviewEditSegment: () => ({ ok: true, error: "", meshes: [] }),
    roadUpdateSectionTemplate: () => ({ ok: true, error: "" }),
    roadApplyTransition: () => ({ ok: true, error: "" }),
    roadAddManualLine: () => ({ ok: true, error: "" }),
    roadAddManualArea: () => ({ ok: true, error: "" }),
    roadClear: () => ({ ok: true, error: "" }),
    roadSaveState: () => "factory-road-state",
    roadLoadState: () => ({ ok: true, error: "" }),
    saveState: () => "factory-state",
    loadState: () => ({ ok: true, error: "" }),
    scene: () => emptyScene,
    ...overrides
  } as WireBridge;
}

describe("viewport tool routing", () => {
  it("draws a road through two viewport clicks without adding Wire path points", () => {
    const roadAddSegment = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ roadAddSegment }), store);
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint([2, 3, 0]);
    actions.previewViewportPoint([18, 5, 0]);

    expect(current(store).pathPoints).toEqual([]);
    expect(current(store).road.phase).toBe("end");
    expect(current(store).road.draftEnd).toEqual({ x: 18, y: 5 });

    actions.addViewportPoint([18, 5, 0]);

    expect(roadAddSegment).toHaveBeenCalledWith(expect.objectContaining({
      kind: "line",
      startX: 2,
      startY: 3,
      endX: 18,
      endY: 5
    }));
    expect(current(store).pathPoints).toEqual([]);
    expect(current(store).road.phase).toBe("end");
  });

  it("does not report an expected road preview rejection as a global error", () => {
    const roadPreviewSegment = vi.fn(() => ({
      ok: false,
      error: "road span has zero length",
      errorKind: EditErrorKind.Validation,
      meshes: []
    }));
    const consoleError = vi.spyOn(console, "error").mockImplementation(() => undefined);
    const store = new ViewerStore();
    const stopLogging = startConsoleLogging(store);
    const actions = new ViewerActions(
      actionBridge({ roadPreviewSegment }),
      store
    );
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint([0, 0, 0]);
    actions.previewViewportPoint([0, 0, 0]);
    actions.previewViewportPoint([0.1, 0, 0]);

    expect(roadPreviewSegment).toHaveBeenCalledTimes(2);
    expect(current(store).error).toBe("");
    expect(current(store).road.previewIssue).toBe("road span has zero length");
    expect(current(store).road.lastError).toBe("");
    expect(consoleError).not.toHaveBeenCalled();

    stopLogging();
    consoleError.mockRestore();
  });

  it("reports an internal road preview failure as a global error", () => {
    const roadPreviewSegment = vi.fn(() => ({
      ok: false,
      error: "road preview internal failure",
      errorKind: EditErrorKind.Internal,
      meshes: []
    }));
    const consoleError = vi.spyOn(console, "error").mockImplementation(() => undefined);
    const store = new ViewerStore();
    const stopLogging = startConsoleLogging(store);
    const actions = new ViewerActions(
      actionBridge({ roadPreviewSegment }),
      store
    );
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint([0, 0, 0]);
    actions.previewViewportPoint([10, 0, 0]);

    expect(current(store).error).toBe("road preview internal failure");
    expect(current(store).road.previewIssue).toBe("");
    expect(current(store).road.lastError).toBe("road preview internal failure");
    expect(consoleError).toHaveBeenCalledOnce();
    expect(consoleError).toHaveBeenCalledWith("[wire] road preview internal failure");

    stopLogging();
    consoleError.mockRestore();
  });

  it("normalizes continuous road drawing to a new segment in the same corridor", () => {
    const roadAddSegment = vi.fn()
      .mockReturnValueOnce({
        ok: true,
        error: "",
        segmentId: 11,
        corridorId: 21,
        endNodeId: 12
      })
      .mockReturnValue({
        ok: true,
        error: "",
        segmentId: 13,
        corridorId: 21,
        endNodeId: 14
      });
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ roadAddSegment }), store);
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint([0, 0, 0]);
    actions.addViewportPoint([20, 0, 0]);
    actions.addViewportPoint([32, 16, 0]);

    expect(roadAddSegment).toHaveBeenNthCalledWith(2, expect.objectContaining({
      startX: 20,
      startY: 0,
      endX: 32,
      endY: 16,
      startNodeId: 12,
      extensionCorridorId: 21
    }));
  });

  it("normalizes a resumed degree-one endpoint to extension", () => {
    const roadAddSegment = vi.fn(() => ({ ok: true, error: "", segmentId: 11, endNodeId: 12 }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ roadAddSegment }), store);
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint(
      [20, 0, 0],
      { kind: "road", nodeId: 12, segmentId: 0, stationM: 0, extensionCorridorId: 11 }
    );
    actions.addViewportPoint([32, 16, 0]);

    expect(roadAddSegment).toHaveBeenCalledWith(expect.objectContaining({
      startNodeId: 12,
      extensionCorridorId: 11
    }));
  });

  it("passes a road segment snap target to the authoritative road state", () => {
    const roadAddSegment = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ roadAddSegment }), store);
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint([20, 0, 0], { kind: "road", nodeId: 0, segmentId: 7, stationM: 20 });
    actions.addViewportPoint([20, 24, 0]);

    expect(roadAddSegment).toHaveBeenCalledWith(expect.objectContaining({
      startX: 20,
      startY: 0,
      endX: 20,
      endY: 24,
      startNodeId: 0,
      startSegmentId: 7,
      startStationM: 20
    }));
  });

  it("starts a new road branch when an existing road segment is snapped during continuous drawing", () => {
    const roadAddSegment = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ roadAddSegment }), store);
    actions.initialize();

    actions.setActiveTool("road");
    actions.addViewportPoint([0, 0, 0]);
    actions.addViewportPoint([40, 0, 0]);
    actions.addViewportPoint([20, 0, 0], { kind: "road", nodeId: 0, segmentId: 7, stationM: 20 });
    actions.addViewportPoint([20, 24, 0]);

    expect(roadAddSegment).toHaveBeenLastCalledWith(expect.objectContaining({
      startX: 20,
      startY: 0,
      endX: 20,
      endY: 24,
      startNodeId: 0,
      startSegmentId: 7,
      startStationM: 20
    }));
    expect(current(store).road.draftStartSegmentId).toBe(0);
    expect(current(store).road.draftStartStationM).toBe(0);
  });

  it("draws a curved road with a Cities-style bend point before the end point", () => {
    const roadAddSegment = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ roadAddSegment }), store);
    actions.initialize();

    actions.setActiveTool("road");
    actions.setRoadMode("bezier");
    actions.addViewportPoint([0, 0, 0]);

    expect(current(store).road.phase).toBe("bend");

    actions.previewViewportPoint([9, 9, 0]);
    expect(current(store).road.draftBend).toEqual({ x: 9, y: 9 });

    actions.addViewportPoint([9, 9, 0]);
    expect(roadAddSegment).not.toHaveBeenCalled();
    expect(current(store).road.phase).toBe("end");

    actions.addViewportPoint([18, 0, 0]);

    expect(roadAddSegment).toHaveBeenCalledWith(expect.objectContaining({
      kind: "bezier",
      startX: 0,
      startY: 0,
      endX: 18,
      endY: 0,
      handleAX: 6,
      handleAY: 6,
      handleBX: 12,
      handleBY: 6
    }));
    expect(current(store).road.phase).toBe("bend");
  });

  it("snaps a continued curve start handle to the previous end tangent", () => {
    const roadAddSegment = vi.fn((_input: RoadSegmentInput) => ({
      ok: true,
      error: "",
      segmentId: 11,
      endNodeId: 12,
      corridorId: 21
    }));
    const roadPreviewSegment = vi.fn((_input: RoadSegmentInput) => ({
      ok: true,
      error: "",
      meshes: []
    }));
    const store = new ViewerStore();
    const actions = new ViewerActions(
      actionBridge({ roadAddSegment, roadPreviewSegment }),
      store
    );
    actions.initialize();

    actions.setActiveTool("road");
    actions.setRoadMode("bezier");
    actions.addViewportPoint([0, 0, 0]);
    actions.addViewportPoint([9, 9, 0]);
    actions.addViewportPoint([18, 0, 0]);

    actions.previewViewportPoint([18, 12, 0]);
    const hover = roadPreviewSegment.mock.calls.at(-1)?.[0];
    expect(hover).toBeDefined();
    expect(hover?.endX).toBeCloseTo(18 + 12 / Math.sqrt(2), 9);
    expect(hover?.endY).toBeCloseTo(-12 / Math.sqrt(2), 9);

    actions.addViewportPoint([18, 12, 0]);
    actions.addViewportPoint([30, -12, 0]);

    const previous = roadAddSegment.mock.calls[0]?.[0];
    const continued = roadAddSegment.mock.calls[1]?.[0];
    expect(previous).toBeDefined();
    expect(continued).toBeDefined();
    if (previous === undefined || continued === undefined) {
      throw new Error("continued curve inputs were not captured");
    }
    const previousEndTangent = {
      x: previous.endX - previous.handleBX,
      y: previous.endY - previous.handleBY
    };
    const continuedStartTangent = {
      x: continued.handleAX - continued.startX,
      y: continued.handleAY - continued.startY
    };
    const cross =
      previousEndTangent.x * continuedStartTangent.y -
      previousEndTangent.y * continuedStartTangent.x;
    const dot =
      previousEndTangent.x * continuedStartTangent.x +
      previousEndTangent.y * continuedStartTangent.y;

    expect(Math.abs(cross)).toBeLessThan(1e-9);
    expect(dot).toBeGreaterThan(0);
  });

  it("routes marking and one-click road-section deletion from centerline picks", () => {
    const roadAddManualLine = vi.fn(() => ({ ok: true, error: "" }));
    const roadAddManualArea = vi.fn(() => ({ ok: true, error: "" }));
    const roadDeleteSection = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({
      roadAddManualLine,
      roadAddManualArea,
      roadDeleteSection
    }), store);
    actions.initialize();
    actions.setActiveTool("road");

    actions.setRoadOperation("line-marking");
    actions.addViewportPoint([5, 0, 0], { kind: "road", nodeId: 0, segmentId: 12, stationM: 5 });
    actions.addViewportPoint([20, 0, 0], { kind: "road", nodeId: 0, segmentId: 12, stationM: 20 });
    expect(roadAddManualLine).toHaveBeenCalledWith(expect.objectContaining({
      segmentId: 12,
      startStationM: 5,
      endStationM: 20
    }));

    actions.setRoadOperation("area-marking");
    actions.addViewportPoint([15, 0, 0], { kind: "road", nodeId: 0, segmentId: 12, stationM: 15 });
    expect(roadAddManualArea).toHaveBeenCalledWith(expect.objectContaining({ segmentId: 12, stationM: 15 }));

    actions.setRoadOperation("delete");
    actions.addViewportPoint([8, 0, 0], { kind: "road", nodeId: 0, segmentId: 12, stationM: 8 });
    expect(roadDeleteSection).toHaveBeenCalledOnce();
    expect(roadDeleteSection).toHaveBeenCalledWith(12);
  });

  it("routes endpoint and control handle edits to their owning operations", () => {
    const roadPreviewEditSegment = vi.fn(() => ({ ok: true, error: "", meshes: [] }));
    const roadEditSegment = vi.fn(() => ({ ok: true, error: "" }));
    const roadPreviewMoveNode = vi.fn(() => ({ ok: true, error: "", meshes: [] }));
    const roadMoveNode = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const baseScene = actionBridge().roadScene();
    const actions = new ViewerActions(actionBridge({
      roadScene: () => ({
        ...baseScene,
        editableSegments: [{
          id: 12,
          nodeAId: 20,
          nodeBId: 21,
          kind: "line",
          points: [{ x: 0, y: 0 }, { x: 6.6666666667, y: 0 }, { x: 13.3333333333, y: 0 }, { x: 20, y: 0 }]
        }]
      }),
      roadPreviewEditSegment,
      roadEditSegment,
      roadPreviewMoveNode,
      roadMoveNode
    }), store);
    actions.initialize();
    actions.setActiveTool("road");
    actions.setRoadOperation("edit");
    actions.addViewportPoint([10, 0, 0], { kind: "road", nodeId: 0, segmentId: 12, stationM: 10 });

    actions.previewRoadEditHandle(0, [-2, 3, 0]);
    expect(roadPreviewMoveNode).toHaveBeenCalledWith(20, -2, 3);
    actions.commitRoadEditHandle();
    expect(roadMoveNode).toHaveBeenCalledWith(20, -2, 3);
    expect(roadEditSegment).not.toHaveBeenCalled();

    actions.setRoadOperation("edit");
    actions.addViewportPoint([10, 0, 0], { kind: "road", nodeId: 0, segmentId: 12, stationM: 10 });
    actions.previewRoadEditHandle(1, [7, 10, 0]);
    expect(roadPreviewEditSegment).toHaveBeenCalledWith(12, expect.objectContaining({
      kind: "bezier",
      handleAX: 7,
      handleAY: 10
    }));
    actions.commitRoadEditHandle();
    expect(roadEditSegment).toHaveBeenCalledWith(12, expect.objectContaining({ handleAX: 7, handleAY: 10 }));
  });
});

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

  it("does not mutate authoritative Core templates on startup", () => {
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

    expect(cableUpdates).toEqual([]);
    expect(poleUpdates).toEqual([]);
  });

  it("does not mutate authoritative geometry settings on startup", () => {
    let geometry = {
      curveSamples: 8,
      sagEnabled: true,
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

    expect(update).not.toHaveBeenCalled();
    expect(current(store).geometry.sagEnabled).toBe(true);
  });

  it("keeps startup free of authoritative update calls in source", () => {
    const source = readFileSync(new URL("../src/actions/viewer.ts", import.meta.url), "utf8");
    const start = source.indexOf("  initialize(): void {");
    const end = source.indexOf("  async restoreWorkspace()", start);
    expect(start).toBeGreaterThanOrEqual(0);
    expect(end).toBeGreaterThan(start);
    const initializeSource = source.slice(start, end);
    expect(initializeSource).not.toContain("updateCableTemplate(");
    expect(initializeSource).not.toContain("updatePoleTemplate(");
    expect(initializeSource).not.toContain("updateGeometrySettings(");
    expect(source).not.toContain("patchedCableTemplate");
    expect(source).not.toContain("patchedPoleTemplate");
    expect(source).not.toContain("JAPAN_DISTRIBUTION_PRIMITIVE");
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

  it("clears pending support-node drafts when clearing the draw path", () => {
    const clearPendingSupportNodes = vi.fn(() => ({ ok: true, error: "" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ clearPendingSupportNodes }), store);
    actions.initialize();

    actions.addPathPoint([1, 2, 3]);
    actions.clearPath();

    expect(clearPendingSupportNodes).toHaveBeenCalledTimes(1);
    expect(current(store).pathPoints).toEqual([]);
    expect(current(store).pathPointSpecs).toEqual([]);
  });

  it("keeps the draw path when pending support-node draft clear fails", () => {
    const clearPendingSupportNodes = vi.fn(() => ({ ok: false, error: "clear failed" }));
    const store = new ViewerStore();
    const actions = new ViewerActions(actionBridge({ clearPendingSupportNodes }), store);
    actions.initialize();

    actions.addPathPoint([1, 2, 3]);
    actions.clearPath();

    expect(clearPendingSupportNodes).toHaveBeenCalledTimes(1);
    expect(current(store).pathPoints).toEqual([[1, 2, 3]]);
    expect(current(store).error).toBe("clear failed");
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

  it("passes required resolved snap identities through the viewer action entry", () => {
    const coveredEntries = new Set<string>();
    const scenarios = [
      { cell: "BOS:add_one_edge:S1", supportKind: 0, nodeId: "42" },
      { cell: "BOS:add_one_edge:SM", supportKind: 1, nodeId: "9001" }
    ];
    for (const scenario of scenarios) {
      let nodeSpecs: Array<{ pointIndex: number; supportKind: number; nodeId: string }> | undefined;
      const store = new ViewerStore();
      const actions = new ViewerActions(actionBridge({
        resolveBranchPick: () => ({
          ok: true,
          error: "",
          positionX: 5,
          positionY: 0,
          positionZ: 4,
          supportKind: scenario.supportKind,
          nodeId: scenario.nodeId
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

      expect(nodeSpecs).toEqual([{
        pointIndex: 0,
        supportKind: scenario.supportKind,
        nodeId: scenario.nodeId
      }]);
      coveredEntries.add(scenario.cell);
    }
    expect(missingBackboneEntryCells("viewer_action", coveredEntries)).toEqual([]);
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
