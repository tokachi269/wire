import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { loadWireModule, type WireStateHandle } from "../src/bridge/wasm";
import type { ModelAssemblyBootstrapInput, ModelTransformInput } from "../src/model";

function visualParts(state: WireStateHandle) {
  const scene = state.visualScene();
  const samples = new Float64Array(scene.samples);
  return scene.parts.map((info) => ({
    info,
    samples: samples.subarray(info.sampleOffset, info.sampleOffset + info.sampleCount * 3)
  }));
}

const identityTransform = (): ModelTransformInput => ({
  positionX: 0, positionY: 0, positionZ: 0,
  rotationX: 0, rotationY: 0, rotationZ: 0,
  scaleX: 1, scaleY: 1, scaleZ: 1
});

function modelBootstrap(): ModelAssemblyBootstrapInput {
  return {
    assemblies: [{
      id: 9901,
      version: 1,
      parts: [{
        partId: 1, modelKey: "pole_body", descriptorName: "pole", descriptorVersion: 1,
        fitMode: 1, localTransform: identityTransform(), sockets: []
      }],
      wireSocket: null
    }, {
      id: 9902,
      version: 1,
      parts: [{
        partId: 1, modelKey: "hv_crossarm", descriptorName: "crossarm", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(), sockets: []
      }, {
        partId: 2, modelKey: "pole_belt", descriptorName: "belt", descriptorVersion: 1,
        fitMode: 2, localTransform: identityTransform(), sockets: []
      }],
      wireSocket: null
    }, {
      id: 9903,
      version: 1,
      parts: [{
        partId: 1, modelKey: "hv_insulator", descriptorName: "insulator", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(), sockets: [{
          name: "wire",
          positionX: 0, positionY: 0, positionZ: 0.18,
          directionX: 1, directionY: 0, directionZ: 0
        }]
      }],
      wireSocket: { partId: 1, socketName: "wire" }
    }, {
      id: 9904,
      version: 1,
      parts: [{
        partId: 1, modelKey: "communication_clamp_long", descriptorName: "clamp", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(), sockets: [{
          name: "wire",
          positionX: 0, positionY: 0.3, positionZ: 0,
          directionX: 0, directionY: 1, directionZ: 0
        }]
      }],
      wireSocket: { partId: 1, socketName: "wire" }
    }],
    poleAssignments: [
      { poleTypeId: 1, assemblyId: 9901 },
      { poleTypeId: 2, assemblyId: 9901 }
    ],
    bundleAssignments: [
      { bundleTemplateId: 101, rowAssemblyId: 9902, endpointAssemblyId: 9903 },
      { bundleTemplateId: 104, rowAssemblyId: 0, endpointAssemblyId: 9904 }
    ]
  };
}

describe("wire wasm smoke", () => {
  let state: WireStateHandle;
  let createState: () => WireStateHandle;

  beforeAll(async () => {
    const module = await loadWireModule();
    createState = () => new module.WireState();
    state = new module.WireState();
  });

  it("bootstraps one straight communication fixture per Port", () => {
    const modelState = createState();
    const configured = modelState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const generated = modelState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [104], 0, 2, [0], 0, 7, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const models = modelState.visualScene().models;
    expect(models.filter((model) => model.modelKey === "pole_body")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "communication_clamp_long")).toHaveLength(2);
    expect(new Set(models.map((model) => model.stableKey)).size).toBe(models.length);
    modelState.delete();
  });

  it("keeps the current state unchanged when model bootstrap configure or load fails", () => {
    const target = createState();
    const generated = target.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);
    const before = target.saveState();

    const invalid = modelBootstrap();
    invalid.bundleAssignments[0].endpointAssemblyId = 9999;
    const configured = target.configureModelAssemblies(invalid);
    expect(configured.ok).toBe(false);
    expect(target.saveState()).toBe(before);

    const source = createState();
    const sourceConfigured = source.configureModelAssemblies(modelBootstrap());
    expect(sourceConfigured.ok, sourceConfigured.error).toBe(true);
    const sourceGenerated = source.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [101], 0, 1, [0], 0, 0, []
    );
    expect(sourceGenerated.ok, sourceGenerated.error).toBe(true);

    const loaded = target.loadStateWithModels(source.saveState(), invalid);
    expect(loaded.ok).toBe(false);
    expect(target.saveState()).toBe(before);

    source.delete();
    target.delete();
  });

  afterAll(() => {
    state.delete();
  });

  it("generates a finite two-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 0, []);

    expect(result.ok, result.error).toBe(true);
    expect(result.generatedPoleCount).toBe(2);
    expect(result.generatedSpanCount).toBeGreaterThan(0);
    const parts = visualParts(state);
    expect(parts.length).toBeGreaterThan(0);
    expect(new Set(parts.map((part) => part.info.partKey)).size).toBe(parts.length);

    for (const { info, samples } of parts) {
      expect(samples).toBeInstanceOf(Float64Array);
      expect(samples.length).toBe(info.sampleCount * 3);
      expect([...samples].every(Number.isFinite)).toBe(true);
    }
  });

  it("bootstraps straight HV assemblies and returns model instances in the scene payload", () => {
    const modelState = createState();
    const bootstrap = modelBootstrap();
    const configured = modelState.configureModelAssemblies(bootstrap);
    expect(configured.ok, configured.error).toBe(true);
    const generated = modelState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [101], 0, 1, [0], 0, 7, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const models = modelState.visualScene().models;
    expect(models.filter((model) => model.modelKey === "pole_body")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "hv_crossarm")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "pole_belt")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "hv_insulator")).toHaveLength(6);
    expect(new Set(models.map((model) => model.stableKey)).size).toBe(models.length);
    expect(models.every((model) => [
      model.positionX, model.positionY, model.positionZ,
      model.rotationX, model.rotationY, model.rotationZ,
      model.scaleX, model.scaleY, model.scaleZ
    ].every(Number.isFinite))).toBe(true);

    const saved = modelState.saveState();
    const loadedState = createState();
    const loaded = loadedState.loadStateWithModels(saved, bootstrap);
    expect(loaded.ok, loaded.error).toBe(true);
    expect(loadedState.visualScene().models).toEqual(models);
    modelState.delete();
    loadedState.delete();
  });

  it("applies generation-time tilt when a max tilt is requested", () => {
    const tilted = createState();
    const generated = tilted.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 9.5, []
    );
    expect(generated.ok, generated.error).toBe(true);
    const poles = Array.from({ length: tilted.poleCount() }, (_, index) => tilted.pole(index));
    expect(poles.length).toBeGreaterThan(0);
    expect(
      poles.some((pole) => Math.abs(pole.rotationX) > 0.01 || Math.abs(pole.rotationY) > 0.01)
    ).toBe(true);
    tilted.delete();

    const flat = createState();
    const plain = flat.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(plain.ok, plain.error).toBe(true);
    const plainPoles = Array.from({ length: flat.poleCount() }, (_, index) => flat.pole(index));
    expect(
      plainPoles.every(
        (pole) => Math.abs(pole.rotationX) < 1e-9 && Math.abs(pole.rotationY) < 1e-9
      )
    ).toBe(true);
    flat.delete();
  });

  it("exposes a shared run id for through edge bodies", () => {
    const runState = createState();
    const result = runState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0, 20, 10, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(result.ok, result.error).toBe(true);

    const edgeBodies = visualParts(runState).map((part) => part.info)
      .filter((part) => part.kind === 0 && part.bundleTemplateId === 102 && part.laneIndex === 0);

    expect(edgeBodies).toHaveLength(2);
    expect(edgeBodies[0].runId).toBeGreaterThan(0);
    expect(edgeBodies[1].runId).toBe(edgeBodies[0].runId);
    runState.delete();
  });

  it("generates the viewer default bundle selection together", () => {
    const result = state.generate(
      new Float64Array([0, 10, 0, 20, 10, 0]),
      [102, 101, 104, 105],
      0,
      1,
      [0, 0, 0, 0],
      0,
      0,
      []
    );

    expect(result.ok, result.error).toBe(true);
    expect(result.generatedSpanCount).toBeGreaterThanOrEqual(4);
  });

  it("roundtrips visual parts through authoritative save and load", () => {
    const savedState = createState();
    const generated = savedState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0, 20, 10, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);
    const expected = visualParts(savedState).map((part) => ({
      info: part.info,
      samples: [...part.samples]
    }));

    const text = savedState.saveState();
    expect(text.startsWith("wire_state_v1\n")).toBe(true);
    const loadedState = createState();
    const loaded = loadedState.loadState(text);
    expect(loaded.ok, loaded.error).toBe(true);
    const actual = visualParts(loadedState);
    expect(actual).toHaveLength(expected.length);
    for (let index = 0; index < expected.length; index += 1) {
      expect(actual[index].info).toEqual(expected[index].info);
      expect([...actual[index].samples]).toEqual(expected[index].samples);
    }
    savedState.delete();
    loadedState.delete();
  });
  it("returns sagged visual samples when sag is enabled", () => {
    const geometry = state.geometrySettings();
    const update = state.updateGeometrySettings({
      ...geometry,
      sagEnabled: true,
      sagFactor: Math.max(geometry.sagFactor, 0.03)
    });
    expect(update.ok, update.error).toBe(true);

    const result = state.generate(new Float64Array([0, 20, 0, 40, 20, 0]), [102], 0, 1, [0], 0, 0, []);
    expect(result.ok, result.error).toBe(true);

    let maxDrop = 0;
    for (const { samples } of visualParts(state)) {
      if (samples.length < 9) continue;
      const startZ = samples[2];
      const endZ = samples[samples.length - 1];
      let minZ = Number.POSITIVE_INFINITY;
      for (let sample = 2; sample < samples.length; sample += 3) {
        minZ = Math.min(minZ, samples[sample]);
      }
      maxDrop = Math.max(maxDrop, (startZ + endZ) / 2 - minZ);
    }

    expect(maxDrop).toBeGreaterThan(0.1);
  });


  it("returns an explicit error for a one-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0]), [102], 0, 1, [0], 0, 0, []);

    expect(result.ok).toBe(false);
    expect(result.error.length).toBeGreaterThan(0);
  });

  it("updates reshape settings and regenerates layout edits", () => {
    const layoutState = createState();
    const bundleTemplates = Array.from(
      { length: layoutState.bundleTemplateCount() },
      (_, index) => layoutState.bundleTemplate(index)
    );
    const lowVoltage = bundleTemplates.find(
      (template) => template.name === "DEFAULT_SINGLE" && template.fixedCount
    );
    expect(lowVoltage).toBeDefined();

    const generated = layoutState.generate(
      new Float64Array([0, 30, 0, 20, 30, 0, 20, 40, 0]),
      [lowVoltage!.id],
      0,
      1,
      [0],
      0,
      0,
      []
    );
    expect(generated.ok, generated.error).toBe(true);

    const geometry = layoutState.geometrySettings();
    const reshape = layoutState.updateGeometrySettings({
      ...geometry,
      sagFactor: geometry.sagFactor + 0.005
    });
    expect(reshape.ok, reshape.error).toBe(true);
    expect(visualParts(layoutState).length).toBeGreaterThan(0);

    const layout = layoutState.layoutSettings();
    const regenerate = layoutState.updateLayoutSettings({
      ...layout,
      cornerThresholdDeg: layout.cornerThresholdDeg - 1
    });
    expect(regenerate.ok, regenerate.error).toBe(true);
    expect(visualParts(layoutState).length).toBeGreaterThan(0);
    layoutState.delete();
  });

  it("applies pole category height edits to ports and curve endpoints", () => {
    const placementState = createState();
    const templates = Array.from(
      { length: placementState.poleTemplateCount() },
      (_, index) => placementState.poleTemplate(index)
    );
    const original = templates.find((template) => template.name === "CommunicationPole");
    expect(original).toBeDefined();
    const generated = placementState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]),
      [101],
      0,
      original!.id,
      [0],
      0,
      0,
      []
    );
    expect(generated.ok, generated.error).toBe(true);

    const portsBefore = Array.from(
      { length: placementState.portCount() },
      (_, index) => placementState.port(index)
    ).filter((port) => port.category === 0);
    expect(portsBefore.length).toBeGreaterThan(0);
    const partsBefore = visualParts(placementState).map((part) => new Float64Array(part.samples));

    const edited = structuredClone(original!);
    for (const band of edited.portBands.filter((candidate) => candidate.category === 0)) {
      band.heightCenter += 1;
      band.heightMin += 1;
      band.heightMax += 1;
    }

    const update = placementState.updatePoleTemplate(edited);
    expect(update.ok, update.error).toBe(true);

    const portsAfter = Array.from(
      { length: placementState.portCount() },
      (_, index) => placementState.port(index)
    ).filter((port) => port.category === 0);
    expect(portsAfter).toHaveLength(portsBefore.length);
    for (const before of portsBefore) {
      const after = portsAfter.find((candidate) => candidate.id === before.id);
      expect(after?.z).toBeCloseTo(before.z + 1, 8);
    }

    const partsAfter = visualParts(placementState).map((part) => new Float64Array(part.samples));
    expect(partsAfter).toHaveLength(partsBefore.length);
    expect(
      partsAfter.some((samples, index) =>
        Math.abs(samples[2] - partsBefore[index][2] - 1) < 1e-8 ||
        Math.abs(
          samples[samples.length - 1] -
          partsBefore[index][partsBefore[index].length - 1] -
          1
        ) < 1e-8
      )
    ).toBe(true);
    placementState.delete();
  });

});
