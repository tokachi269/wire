import type { RandomBundleRule } from "../model";

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
