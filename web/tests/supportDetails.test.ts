import { describe, expect, it } from "vitest";
import {
  supportDetailReferenceScene,
  SUPPORT_DETAIL_MODEL_KEYS
} from "../src/bridge/supportDetails";

describe("support detail reference scenes", () => {
  it("exposes the transformer equipment reference scene with catalog proxies", () => {
    const scene = supportDetailReferenceScene("transformer");
    const keys = scene.models.map((model) => model.modelKey);

    expect(keys).toContain("pole_body");
    expect(keys).toContain("hv_crossarm");
    expect(keys).toContain("hv_insulator");
    expect(keys).toContain(SUPPORT_DETAIL_MODEL_KEYS.transformer);
    expect(keys.filter((key) => key === SUPPORT_DETAIL_MODEL_KEYS.pc6Cutout).length)
      .toBeGreaterThanOrEqual(2);
    expect(keys).toContain(SUPPORT_DETAIL_MODEL_KEYS.tma13Mount);
    expect(keys).toContain(SUPPORT_DETAIL_MODEL_KEYS.arrester);
    expect(keys).not.toContain("detail_transformer_box");
    expect(keys).not.toContain("detail_inline_device");
  });

  it("exposes triplex fan-out and optical closure reference scenes", () => {
    const triplex = supportDetailReferenceScene("triplex");
    expect(triplex.models.map((model) => model.modelKey))
      .toContain(SUPPORT_DETAIL_MODEL_KEYS.triplexTermination);
    expect(triplex.parts.filter((part) => part.info.partKey.includes("fanout"))).toHaveLength(3);

    const optical = supportDetailReferenceScene("optical");
    expect(optical.models.map((model) => model.modelKey))
      .toContain(SUPPORT_DETAIL_MODEL_KEYS.opticalClosure);
    expect(optical.parts.some((part) => part.info.partKey.includes("hanger"))).toBe(true);
    expect(optical.parts.some((part) => part.info.partKey.includes("branch"))).toBe(true);
  });

  it("keeps reference local cables bounded and out of saved backbone entities", () => {
    for (const kind of ["transformer", "triplex", "optical"] as const) {
      const scene = supportDetailReferenceScene(kind);
      expect(scene.parts.every((part) => Math.floor(part.samples.length / 3) <= 8)).toBe(true);
      expect(scene.parts.every((part) => part.info.kind === 4)).toBe(true);
    }
  });
});
