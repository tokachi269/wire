import { describe, expect, it } from "vitest";
import {
  DEFAULT_BUNDLE_RULES,
  routeBundleRules
} from "../src/profile/defaultBundlePreset";

describe("default route bundle variation controls", () => {
  it("changes non-HV density while leaving the fixed HV rule unchanged", () => {
    const rules = routeBundleRules({ density: 0.5, heightSpread: 1 });

    expect(rules[0]).toEqual(DEFAULT_BUNDLE_RULES[0]);
    expect(rules.slice(1).map(({ minInstances, maxInstances }) =>
      [minInstances, maxInstances])).toEqual([[1, 2], [1, 2], [0, 1]]);
  });

  it("scales each height envelope around its authored center", () => {
    const rules = routeBundleRules({ density: 1, heightSpread: 0.5 });
    const lowVoltage = rules.find((rule) => rule.bundleTemplateId === 102);
    const communication = rules.find((rule) => rule.bundleTemplateId === 104);

    expect(lowVoltage?.heightMin).toBeCloseTo(7.175, 12);
    expect(lowVoltage?.heightMax).toBeCloseTo(7.525, 12);
    expect(communication?.heightMin).toBeCloseTo(5.2, 12);
    expect(communication?.heightMax).toBeCloseTo(5.6, 12);
    expect(rules[0]).toEqual(DEFAULT_BUNDLE_RULES[0]);
  });
});
