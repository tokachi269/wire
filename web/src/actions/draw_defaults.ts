import type { BundlePlacement, BundleTemplateInfo, DefaultBundlePlacementInfo } from "../model";

export type BundlePlacementDefaultResolver = (
  bundleTemplateId: number,
  poleTypeId: number,
  count: number
) => DefaultBundlePlacementInfo;

export function bundlePlacementDefault(
  template: BundleTemplateInfo,
  poleTypeId: number | null,
  count: number,
  resolveDefault: BundlePlacementDefaultResolver
): Pick<BundlePlacement, "height" | "offset" | "spacing"> {
  const resolved = resolveDefault(
    template.id,
    poleTypeId ?? template.relatedPoleTypeId,
    count
  );
  if (!resolved.ok) {
    throw new Error(resolved.error);
  }
  return {
    height: resolved.height,
    offset: resolved.offset,
    spacing: resolved.spacing
  };
}

export function placementUsesTransientZeroDefaults(placements: BundlePlacement[]): boolean {
  return placements.length > 0 &&
    placements.every((placement) =>
      Math.abs(placement.height) <= 1e-12 && Math.abs(placement.offset) <= 1e-12
    );
}
