import type { BundlePlacement, BundleTemplateInfo, PoleTemplateInfo } from "../model";
import { DEFAULT_BUNDLE_PRESET } from "../profile/defaultBundlePreset";

function targetTemplateLayerForCategory(category: number): number {
  if (category === 0) return 2;
  if (category === 4) return 0;
  return 1;
}

export function bundlePlacementDefault(
  template: BundleTemplateInfo,
  poleTemplate: PoleTemplateInfo | undefined
): Pick<BundlePlacement, "height" | "offset" | "spacing"> {
  const category = template.category;
  const targetLayer = targetTemplateLayerForCategory(category);
  const laneCount = template.fixedCount ? template.fixedCountValue : template.defaultCount;
  const candidates = (poleTemplate?.portBands ?? [])
    .filter((band) => band.enabled && band.category === category && band.layer === targetLayer)
    .sort((a, b) => b.priority - a.priority || a.bandId - b.bandId);
  const bands = candidates.filter((candidate, index, all) =>
    all.findIndex((item) => Math.abs(item.lateralCenter - candidate.lateralCenter) <= 1e-12) === index
  ).slice(0, Math.max(1, laneCount));
  const divisor = Math.max(1, bands.length);
  return {
    height: bands.reduce((sum, band) => sum + band.heightCenter, 0) / divisor,
    offset: category === 0 ? -0.2 : bands.reduce((sum, band) => sum + band.lateralCenter, 0) / divisor,
    spacing: template.defaultSpacing
  };
}

export function defaultBundlePlacements(
  templates: BundleTemplateInfo[],
  poleTemplate: PoleTemplateInfo | undefined
): BundlePlacement[] {
  return DEFAULT_BUNDLE_PRESET.flatMap((row) => {
    const template = templates.find((item) => item.id === row.bundleTemplateId);
    if (template === undefined) return [];
    const fallback = bundlePlacementDefault(template, poleTemplate);
    return [{
      id: row.id,
      bundleTemplateId: template.id,
      count: row.count,
      explicit: true,
      height: row.height,
      offset: row.offset,
      spacing: fallback.spacing
    }];
  });
}

export function placementUsesTransientZeroDefaults(placements: BundlePlacement[]): boolean {
  return placements.length > 0 &&
    placements.every((placement) =>
      Math.abs(placement.height) <= 1e-12 && Math.abs(placement.offset) <= 1e-12
    );
}
