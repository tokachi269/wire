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
    const result = state.generate(new Float64Array([0, 0, 0, 20, 0, 0]), 0, 0, 1, 0, 0);

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

  it("returns an explicit error for a one-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0]), 0, 0, 1, 0, 0);

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
});
