import type { RandomBundleRule, RouteVariationControls } from "../model";

// Initial visual-evaluation envelopes. They are input data, not Japanese
// engineering-standard values; Core persists the resolved route descriptor.
export const DEFAULT_BUNDLE_RULES: ReadonlyArray<RandomBundleRule> = [
  {
    bundleTemplateId: 101, minInstances: 1, maxInstances: 1,
    conductorCount: 3,
    heightMin: 9.2, heightMax: 9.2, lateralAbsMin: 0.2, lateralAbsMax: 0.2
  },
  {
    bundleTemplateId: 102, minInstances: 2, maxInstances: 4,
    conductorCount: 1,
    heightMin: 7.0, heightMax: 7.7, lateralAbsMin: 0.12, lateralAbsMax: 0.52
  },
  {
    bundleTemplateId: 104, minInstances: 1, maxInstances: 4,
    conductorCount: 1,
    heightMin: 5.0, heightMax: 5.8, lateralAbsMin: 0.12, lateralAbsMax: 0.50
  },
  {
    bundleTemplateId: 105, minInstances: 0, maxInstances: 2,
    conductorCount: 1,
    heightMin: 4.9, heightMax: 5.7, lateralAbsMin: 0.12, lateralAbsMax: 0.50
  }
];

export const DEFAULT_PREFERRED_SIDE_SIGN = -1;

export const DEFAULT_ROUTE_VARIATION_CONTROLS: RouteVariationControls = {
  density: 1,
  heightSpread: 1,
  lateralSpread: 1
};

export function routeBundleRules(
  controls: RouteVariationControls,
  sourceRules: ReadonlyArray<RandomBundleRule> = DEFAULT_BUNDLE_RULES
): RandomBundleRule[] {
  return adjustRouteBundleRules(sourceRules, controls);
}

export function adjustRouteBundleRules(
  sourceRules: ReadonlyArray<RandomBundleRule>,
  controls: RouteVariationControls
): RandomBundleRule[] {
  if (controls.density === 1 && controls.heightSpread === 1 &&
      controls.lateralSpread === 1) {
    return sourceRules.map((rule) => ({ ...rule }));
  }
  return sourceRules.map((rule) => {
    if (rule.bundleTemplateId === 101) return { ...rule };
    const center = (rule.heightMin + rule.heightMax) * 0.5;
    const halfRange = (rule.heightMax - rule.heightMin) * 0.5 * controls.heightSpread;
    const lateralCenter = (rule.lateralAbsMin + rule.lateralAbsMax) * 0.5;
    const lateralHalfRange = (rule.lateralAbsMax - rule.lateralAbsMin) *
      0.5 * controls.lateralSpread;
    const minInstances = Math.round(rule.minInstances * controls.density);
    const maxInstances = Math.max(
      minInstances,
      Math.round(rule.maxInstances * controls.density)
    );
    return {
      ...rule,
      minInstances,
      maxInstances,
      heightMin: center - halfRange,
      heightMax: center + halfRange,
      lateralAbsMin: Math.max(0, lateralCenter - lateralHalfRange),
      lateralAbsMax: lateralCenter + lateralHalfRange
    };
  });
}
