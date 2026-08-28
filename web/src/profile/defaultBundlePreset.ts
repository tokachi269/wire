import type { RandomBundleRule, RouteVariationControls } from "../model";

// Initial visual-evaluation envelopes. They are input data, not Japanese
// engineering-standard values; Core persists the resolved route descriptor.
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
  heightSpread: 1,
  lateralSpread: 1
};

export function routeBundleRules(
  controls: RouteVariationControls,
  sourceRules: ReadonlyArray<RandomBundleRule> = DEFAULT_BUNDLE_RULES
): RandomBundleRule[] {
  return sourceRules.map((rule) => {
    const authored = DEFAULT_BUNDLE_RULES.find((candidate) =>
      candidate.bundleTemplateId === rule.bundleTemplateId
    ) ?? rule;
    if (authored.bundleTemplateId === 101) return { ...rule };
    const center = (authored.heightMin + authored.heightMax) * 0.5;
    const halfRange = (authored.heightMax - authored.heightMin) * 0.5 * controls.heightSpread;
    const lateralCenter = (authored.lateralAbsMin + authored.lateralAbsMax) * 0.5;
    const lateralHalfRange = (authored.lateralAbsMax - authored.lateralAbsMin) *
      0.5 * controls.lateralSpread;
    const minInstances = Math.round(authored.minInstances * controls.density);
    const maxInstances = Math.max(
      minInstances,
      Math.round(authored.maxInstances * controls.density)
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

export function controlsForRouteBundleRules(
  rules: ReadonlyArray<RandomBundleRule>
): RouteVariationControls {
  const current = rules.find((rule) => rule.bundleTemplateId !== 101) ?? rules[0];
  const base = DEFAULT_BUNDLE_RULES.find((rule) =>
    rule.bundleTemplateId === current?.bundleTemplateId
  ) ?? current;
  if (current === undefined || base === undefined) {
    return { ...DEFAULT_ROUTE_VARIATION_CONTROLS };
  }
  const baseHeightRange = base.heightMax - base.heightMin;
  const baseLateralRange = base.lateralAbsMax - base.lateralAbsMin;
  return {
    density: base.maxInstances === 0 ? 1 : current.maxInstances / base.maxInstances,
    heightSpread: baseHeightRange === 0 ? 1 : (current.heightMax - current.heightMin) / baseHeightRange,
    lateralSpread: baseLateralRange === 0
      ? 1
      : (current.lateralAbsMax - current.lateralAbsMin) / baseLateralRange
  };
}
