import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { loadWireModule, type WireStateHandle } from "../src/bridge/wasm";

describe("wire wasm smoke", () => {
  let state: WireStateHandle;
  let createState: () => WireStateHandle;

  beforeAll(async () => {
    const module = await loadWireModule();
    createState = () => new module.WireState();
    state = new module.WireState();
  });

  afterAll(() => {
    state.delete();
  });

  it("generates a finite two-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0, 20, 0, 0]), [0], 0, 1, [0], 0, 0, []);

    expect(result.ok, result.error).toBe(true);
    expect(result.generatedPoleCount).toBe(2);
    expect(result.generatedSpanCount).toBeGreaterThan(0);
    expect(state.visualPartCount()).toBeGreaterThan(0);

    for (let index = 0; index < state.visualPartCount(); index += 1) {
      const info = state.visualPart(index);
      const samples = new Float64Array(state.visualPartSamples(index));
      expect(samples).toBeInstanceOf(Float64Array);
      expect(samples.length).toBe(info.sampleCount * 3);
      expect([...samples].every(Number.isFinite)).toBe(true);
    }
  });

  it("applies generation-time tilt when a max tilt is requested", () => {
    const tilted = createState();
    const generated = tilted.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [0], 0, 1, [0], 0, 9.5, []
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
      new Float64Array([0, 0, 0, 20, 0, 0]), [0], 0, 1, [0], 0, 0, []
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
      new Float64Array([0, 0, 0, 20, 0, 0, 20, 10, 0]), [0], 0, 1, [0], 0, 0, []
    );
    expect(result.ok, result.error).toBe(true);

    const edgeBodies = Array.from(
      { length: runState.visualPartCount() },
      (_, index) => runState.visualPart(index)
    ).filter((part) => part.kind === 0 && part.bundleTemplateId === 0 && part.laneIndex === 0);

    expect(edgeBodies).toHaveLength(2);
    expect(edgeBodies[0].runId).toBeGreaterThan(0);
    expect(edgeBodies[1].runId).toBe(edgeBodies[0].runId);
    runState.delete();
  });

  it("generates the viewer default bundle selection together", () => {
    const result = state.generate(
      new Float64Array([0, 10, 0, 20, 10, 0]),
      [0, 1, 2, 3],
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
  it("returns sagged visual samples when sag is enabled", () => {
    const geometry = state.geometrySettings();
    const update = state.updateGeometrySettings({
      ...geometry,
      sagEnabled: true,
      sagFactor: Math.max(geometry.sagFactor, 0.03)
    });
    expect(update.ok, update.error).toBe(true);

    const result = state.generate(new Float64Array([0, 20, 0, 40, 20, 0]), [0], 0, 1, [0], 0, 0, []);
    expect(result.ok, result.error).toBe(true);

    let maxDrop = 0;
    for (let index = 0; index < state.visualPartCount(); index += 1) {
      const samples = new Float64Array(state.visualPartSamples(index));
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
    const result = state.generate(new Float64Array([0, 0, 0]), [0], 0, 1, [0], 0, 0, []);

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
    expect(layoutState.visualPartCount()).toBeGreaterThan(0);

    const layout = layoutState.layoutSettings();
    const regenerate = layoutState.updateLayoutSettings({
      ...layout,
      cornerThresholdDeg: layout.cornerThresholdDeg - 1
    });
    expect(regenerate.ok, regenerate.error).toBe(true);
    expect(layoutState.visualPartCount()).toBeGreaterThan(0);
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
      [1],
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
    const partsBefore = Array.from(
      { length: placementState.visualPartCount() },
      (_, index) => new Float64Array(placementState.visualPartSamples(index))
    );

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

    const partsAfter = Array.from(
      { length: placementState.visualPartCount() },
      (_, index) => new Float64Array(placementState.visualPartSamples(index))
    );
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
