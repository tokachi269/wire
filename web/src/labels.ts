export const CATEGORY_LABELS = [
  "High voltage",
  "Low voltage",
  "Communication",
  "Optical",
  "Drop"
] as const;

export const CATEGORY_SHORT = ["HV", "LV", "COMM", "OPT", "DROP"] as const;

export const SIDE_LABELS = ["Left", "Center", "Right"] as const;

export const ROLE_LABELS = [
  "Neutral",
  "Trunk preferred",
  "Branch preferred",
  "Drop preferred"
] as const;

export const OVERFLOW_LABELS = [
  "Try sibling band",
  "Raise height",
  "Constrained fallback"
] as const;

export const ANCHOR_USAGE_LABELS = ["Generic", "Ground", "External", "Midair"] as const;

export const SPAN_LAYER_LABELS = ["Unknown", "HV", "LV", "COMM", "OPT", "DROP"] as const;

export const CONTINUITY_LABELS = ["Auto", "Prefer G1", "Prefer G2"] as const;

export const MATERIAL_LABELS = ["Generic", "Bare", "Insulated", "Optical"] as const;

export function labelOf(labels: readonly string[], value: number): string {
  return labels[value] ?? `Unknown (${value})`;
}

export function categoryLabel(value: number): string {
  return labelOf(CATEGORY_LABELS, value);
}

export function categoryShort(value: number): string {
  return labelOf(CATEGORY_SHORT, value);
}

export function drawIssueText(issue: string): string {
  const detail = issue
    .replace(/^backbone (?:validation|unsupported|internal):\s*/i, "")
    .replace(/^road (?:validation|unsupported|internal):\s*/i, "")
    .trim();
  if (detail.length === 0) return "This interval cannot be generated.";
  return detail[0].toUpperCase() + detail.slice(1);
}

export function fmt(value: number, digits = 4): number {
  return Number.isFinite(value) ? Number(value.toFixed(digits)) : 0;
}

export function round6(value: number): number {
  return Math.round(value * 1e6) / 1e6;
}
