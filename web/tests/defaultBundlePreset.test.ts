import { describe, expect, it } from "vitest";
import {
  adjustRouteBundleRules,
  DEFAULT_BUNDLE_RULES,
  routeBundleRules
} from "../src/profile/defaultBundlePreset";

describe("default route bundle variation controls", () => {
  it("changes non-HV density while leaving the fixed HV rule unchanged", () => {
    const rules = routeBundleRules({ density: 0.5, heightSpread: 1, lateralSpread: 1 });

    expect(rules[0]).toEqual(DEFAULT_BUNDLE_RULES[0]);
    expect(rules.slice(1).map(({ minInstances, maxInstances }) =>
      [minInstances, maxInstances])).toEqual([[1, 2], [1, 2], [1, 1]]);
  });

  it("scales each height envelope around its authored center", () => {
    const rules = routeBundleRules({ density: 1, heightSpread: 0.5, lateralSpread: 1 });
    const lowVoltage = rules.find((rule) => rule.bundleTemplateId === 102);
    const communication = rules.find((rule) => rule.bundleTemplateId === 104);

    expect(lowVoltage?.heightMin).toBeCloseTo(7.175, 12);
    expect(lowVoltage?.heightMax).toBeCloseTo(7.525, 12);
    expect(communication?.heightMin).toBeCloseTo(5.2, 12);
    expect(communication?.heightMax).toBeCloseTo(5.6, 12);
    expect(rules[0]).toEqual(DEFAULT_BUNDLE_RULES[0]);
  });

  it("scales each lateral envelope without crossing the route-wide side", () => {
    const rules = routeBundleRules({ density: 1, heightSpread: 1, lateralSpread: 0.5 });
    const lowVoltage = rules.find((rule) => rule.bundleTemplateId === 102);

    expect(lowVoltage?.lateralAbsMin).toBeCloseTo(0.22, 12);
    expect(lowVoltage?.lateralAbsMax).toBeCloseTo(0.42, 12);
    expect(rules[0]).toEqual(DEFAULT_BUNDLE_RULES[0]);
  });

  it("does not add templates that are absent from a persisted descriptor", () => {
    const communicationOnly = [DEFAULT_BUNDLE_RULES[2]];
    const rules = routeBundleRules(
      { density: 0.5, heightSpread: 0.5, lateralSpread: 0.5 },
      communicationOnly
    );

    expect(rules).toHaveLength(1);
    expect(rules[0].bundleTemplateId).toBe(104);
    expect(rules[0].maxInstances).toBe(2);
  });

  it("uses a persisted descriptor as the relative baseline and keeps neutral Apply exact", () => {
    const persisted = [{
      ...DEFAULT_BUNDLE_RULES[2],
      minInstances: 2,
      maxInstances: 3,
      heightMin: 6.0,
      heightMax: 7.0,
      lateralAbsMin: 0.3,
      lateralAbsMax: 0.7
    }];

    expect(adjustRouteBundleRules(
      persisted, { density: 1, heightSpread: 1, lateralSpread: 1 }
    )).toEqual(persisted);
    const changedFutureDefaults = DEFAULT_BUNDLE_RULES.map((rule) => ({
      ...rule,
      minInstances: rule.minInstances + 5,
      maxInstances: rule.maxInstances + 5,
      heightMin: rule.heightMin + 10,
      heightMax: rule.heightMax + 10
    }));
    expect(routeBundleRules(
      { density: 1, heightSpread: 1, lateralSpread: 1 }, persisted
    )).toEqual(persisted);
    expect(routeBundleRules(
      { density: 1, heightSpread: 1, lateralSpread: 1 }, persisted
    )).not.toEqual(changedFutureDefaults);
    expect(adjustRouteBundleRules(
      persisted, { density: 0.5, heightSpread: 0.5, lateralSpread: 0.5 }
    )[0]).toMatchObject({
      minInstances: 1,
      maxInstances: 2,
      heightMin: 6.25,
      heightMax: 6.75,
      lateralAbsMin: 0.4,
      lateralAbsMax: 0.6
    });
  });
});
