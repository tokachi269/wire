export const SUPPORT_DETAIL_SCENE_POINTS = new Float64Array([
  0, 0, 0,
  12, 0, 0,
  6, 8, 0
]);

export const SUPPORT_DETAIL_SCENE_BUNDLE_TEMPLATE_IDS = [101];
export const SUPPORT_DETAIL_SCENE_COUNTS = [0];

export const SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS = [
  "pole_transformer_20kva_proxy",
  "pc6_cutout_proxy",
  "tma13_cutout_mount_proxy",
  "arrester_gl_b6g_proxy",
  "hv_triplex_termination_60_proxy",
  "aerial_optical_closure_rca3ao_proxy"
] as const;

export const SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS = {
  localCable: 3,
  inlineCable: 4
} as const;
