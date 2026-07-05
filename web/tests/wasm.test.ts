import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { loadWireModule, type WireStateHandle } from "../src/bridge/wasm";

describe("wire wasm smoke", () => {
  let state: WireStateHandle;

  beforeAll(async () => {
    const module = await loadWireModule();
    state = new module.WireState();
  });

  afterAll(() => {
    state.delete();
  });

  it("generates a finite two-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0, 20, 0, 0]), [0], 0, 1, [0], 0);

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

  it("generates the viewer default bundle selection together", () => {
    const result = state.generate(
      new Float64Array([0, 10, 0, 20, 10, 0]),
      [0, 1, 2, 3],
      0,
      1,
      [0, 0, 0, 0],
      0
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

    const result = state.generate(new Float64Array([0, 20, 0, 40, 20, 0]), [0], 0, 1, [0], 0);
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
    const result = state.generate(new Float64Array([0, 0, 0]), [0], 0, 1, [0], 0);

    expect(result.ok).toBe(false);
    expect(result.error.length).toBeGreaterThan(0);
  });

  it("updates reshape settings and reports unsupported layout edits", () => {
    const geometry = state.geometrySettings();
    const reshape = state.updateGeometrySettings({
      ...geometry,
      sagFactor: geometry.sagFactor + 0.005
    });
    expect(reshape.ok, reshape.error).toBe(true);
    expect(state.visualPartCount()).toBeGreaterThan(0);

    const layout = state.layoutSettings();
    const regenerate = state.updateLayoutSettings({
      ...layout,
      cornerThresholdDeg: layout.cornerThresholdDeg - 1
    });
    expect(regenerate.ok).toBe(false);
    expect(regenerate.error.length).toBeGreaterThan(0);
  });

  it("round-trips placement-only pole template edits", () => {
    const templates = Array.from(
      { length: state.poleTemplateCount() },
      (_, index) => state.poleTemplate(index)
    );
    const original = templates.find((template) => template.name === "CommunicationPole");
    expect(original).toBeDefined();
    const edited = structuredClone(original!);
    expect(edited.portBands.length).toBeGreaterThan(0);
    edited.portBands[0].heightCenter += 0.1;
    edited.portBands[0].heightMin += 0.1;
    edited.portBands[0].heightMax += 0.1;

    const update = state.updatePoleTemplate(edited);
    expect(update.ok, update.error).toBe(true);

    const refreshed = Array.from(
      { length: state.poleTemplateCount() },
      (_, index) => state.poleTemplate(index)
    ).find((template) => template.id === edited.id);
    expect(refreshed?.portBands[0].heightCenter).toBeCloseTo(
      edited.portBands[0].heightCenter,
      8
    );

    const restore = state.updatePoleTemplate(original!);
    expect(restore.ok, restore.error).toBe(true);
  });
});
