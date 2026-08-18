export const SUPPORT_DETAIL_SCENE_POINTS = new Float64Array([
  0, 0, 0,
  12, 0, 0,
  6, 8, 0
]);

export const SUPPORT_DETAIL_SCENE_BUNDLE_TEMPLATE_IDS = [101, 102];
export const SUPPORT_DETAIL_SCENE_COUNTS = [0, 0];

export const SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS = [
  "pole_decoration_x"
] as const;

export const SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS = {
  localCable: 3,
  inlineCable: 4
} as const;
