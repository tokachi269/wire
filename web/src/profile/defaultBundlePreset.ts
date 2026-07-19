import type { BundlePlacement } from "../model";

export const DEFAULT_BUNDLE_PRESET: ReadonlyArray<Omit<BundlePlacement, "spacing">> = [
  { id: 1, bundleTemplateId: 101, count: 3, explicit: true, height: 9.2, offset: -0.2 },
  { id: 2, bundleTemplateId: 102, count: 1, explicit: true, height: 7.7, offset: 0 },
  { id: 3, bundleTemplateId: 102, count: 1, explicit: true, height: 7.35, offset: 0 },
  { id: 4, bundleTemplateId: 102, count: 1, explicit: true, height: 7.0, offset: 0 },
  { id: 5, bundleTemplateId: 104, count: 1, explicit: true, height: 5.5, offset: 0 },
  { id: 6, bundleTemplateId: 105, count: 1, explicit: true, height: 5.3, offset: 0 }
];
