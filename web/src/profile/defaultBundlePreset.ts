import type { RandomBundleRule, RouteVariationControls } from "../model";

// Initial visual-evaluation envelopes. They are input data, not Japanese
// engineering-standard values and not persisted as Core entities.
export const DEFAULT_BUNDLE_RULES: ReadonlyArray<RandomBundleRule> = [
  {
    bundleTemplateId: 101, minInstances: 1, maxInstances: 1,
    conductorCount: 3,
    heightMin: 9.2, heightMax: 9.2, lateralAbsMin: 0.2, lateralAbsMax: 0.2, minSpacing: 0.25
  },
  {
    bundleTemplateId: 102, minInstances: 2, maxInstances: 4,
    conductorCount: 1,
    heightMin: 7.0, heightMax: 7.7, lateralAbsMin: 0.12, lateralAbsMax: 0.52, minSpacing: 0.20
  },
  {
    bundleTemplateId: 104, minInstances: 1, maxInstances: 4,
    conductorCount: 1,
    heightMin: 5.0, heightMax: 5.8, lateralAbsMin: 0.12, lateralAbsMax: 0.50, minSpacing: 0.16
  },
  {
    bundleTemplateId: 105, minInstances: 0, maxInstances: 2,
    conductorCount: 1,
    heightMin: 4.9, heightMax: 5.7, lateralAbsMin: 0.12, lateralAbsMax: 0.50, minSpacing: 0.16
  }
];

export const DEFAULT_PREFERRED_SIDE_SIGN = -1;

export const DEFAULT_ROUTE_VARIATION_CONTROLS: RouteVariationControls = {
  density: 1,
  heightSpread: 1
};

export function routeBundleRules(
  controls: RouteVariationControls
): RandomBundleRule[] {
  return DEFAULT_BUNDLE_RULES.map((rule, index) => {
    if (index === 0) return { ...rule };
    const center = (rule.heightMin + rule.heightMax) * 0.5;
    const halfRange = (rule.heightMax - rule.heightMin) * 0.5 * controls.heightSpread;
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
      heightMax: center + halfRange
    };
  });
}
