// The road cross sections this product offers. Core owns the RoadLayoutTemplate
// type, its validation and its IDs; what a road actually measures — lane and
// sidewalk widths, cross slopes, which lines are painted — is a product choice
// and lives here.
//
// `key` is a Web-side name for a preset. It never crosses into Core, is never
// saved, and is not used to decide whether two roads can connect: that is
// settled by the section contents Core compares.

export type RoadStripFunction = "sidewalk" | "shoulder" | "carriageway" | "median";
export type RoadSurfaceStyle = "asphalt" | "sidewalk" | "curb" | "median";
export type RoadBoundaryRole = "outer_edge" | "curb" | "lane_divider" | "median_edge";
export type RoadMarkingRole =
  | "center_line"
  | "lane_separator"
  | "carriageway_edge"
  | "stop_line"
  | "crosswalk";
export type RoadMarkingStyle =
  | "white_solid"
  | "white_dashed"
  | "center_line"
  | "stop_line"
  | "crosswalk";

export interface RoadSectionStripInput {
  id: number;
  function: RoadStripFunction;
  widthM: number;
  crossSlope: number;
  surfaceStyle: RoadSurfaceStyle;
}

export interface RoadSectionLaneInput {
  id: number;
  stripId: number;
  lateralStartM: number;
  lateralEndM: number;
  /** 0 runs with the segment, 1 against it. */
  direction: 0 | 1;
}

export interface RoadSectionMarkingInput {
  role: RoadMarkingRole;
  style: RoadMarkingStyle;
}

export interface RoadSectionBoundaryInput {
  id: number;
  role: RoadBoundaryRole;
  widthM: number;
  heightM: number;
  marking?: RoadSectionMarkingInput;
}

export interface RoadSectionInput {
  strips: RoadSectionStripInput[];
  laneBands: RoadSectionLaneInput[];
  boundaries: RoadSectionBoundaryInput[];
  /**
   * Where the segment alignment runs through the section, measured from the
   * left outer end looking along the segment. Every section here puts it on the
   * middle of the total width, which is what these roads mean; a section that
   * wanted the alignment somewhere else would say so here rather than have Core
   * infer it.
   */
  alignmentOffsetFromLeftM: number;
}

function centredAlignment(
  strips: RoadSectionStripInput[],
  boundaries: RoadSectionBoundaryInput[]
): number {
  const total =
    strips.reduce((sum, strip) => sum + strip.widthM, 0) +
    boundaries.reduce((sum, boundary) => sum + boundary.widthM, 0);
  return total / 2;
}

function centred(
  section: Omit<RoadSectionInput, "alignmentOffsetFromLeftM">
): RoadSectionInput {
  return {
    ...section,
    alignmentOffsetFromLeftM: centredAlignment(section.strips, section.boundaries)
  };
}

export interface RoadTemplatePreset {
  key: string;
  label: string;
  initial: boolean;
  section: RoadSectionInput;
}

const outerLine: RoadSectionMarkingInput = {
  role: "carriageway_edge",
  style: "white_solid"
};
const centerLine: RoadSectionMarkingInput = { role: "center_line", style: "center_line" };

const urbanTwoLane: RoadSectionInput = centred({
  strips: [
    { id: 10, function: "sidewalk", widthM: 2.0, crossSlope: 0.01, surfaceStyle: "sidewalk" },
    { id: 20, function: "carriageway", widthM: 3.0, crossSlope: 0.02, surfaceStyle: "asphalt" },
    { id: 30, function: "carriageway", widthM: 3.0, crossSlope: -0.02, surfaceStyle: "asphalt" },
    { id: 40, function: "sidewalk", widthM: 2.0, crossSlope: -0.01, surfaceStyle: "sidewalk" }
  ],
  laneBands: [
    { id: 1000, stripId: 20, lateralStartM: 0.0, lateralEndM: 3.0, direction: 1 },
    { id: 1010, stripId: 30, lateralStartM: 0.0, lateralEndM: 3.0, direction: 0 }
  ],
  boundaries: [
    { id: 100, role: "curb", widthM: 0.2, heightM: -0.15, marking: outerLine },
    { id: 200, role: "lane_divider", widthM: 0.0, heightM: 0.0, marking: centerLine },
    { id: 300, role: "curb", widthM: 0.2, heightM: 0.15, marking: outerLine }
  ]
});

const threeLane: RoadSectionInput = centred({
  strips: [
    { id: 10, function: "sidewalk", widthM: 2.0, crossSlope: 0.01, surfaceStyle: "sidewalk" },
    { id: 20, function: "carriageway", widthM: 3.0, crossSlope: 0.02, surfaceStyle: "asphalt" },
    { id: 30, function: "carriageway", widthM: 3.0, crossSlope: 0.0, surfaceStyle: "asphalt" },
    { id: 35, function: "carriageway", widthM: 3.0, crossSlope: -0.02, surfaceStyle: "asphalt" },
    { id: 40, function: "sidewalk", widthM: 2.0, crossSlope: -0.01, surfaceStyle: "sidewalk" }
  ],
  laneBands: [
    { id: 1000, stripId: 20, lateralStartM: 0.0, lateralEndM: 3.0, direction: 1 },
    { id: 1010, stripId: 30, lateralStartM: 0.0, lateralEndM: 3.0, direction: 0 },
    { id: 1020, stripId: 35, lateralStartM: 0.0, lateralEndM: 3.0, direction: 0 }
  ],
  boundaries: [
    { id: 100, role: "curb", widthM: 0.2, heightM: -0.15, marking: outerLine },
    { id: 200, role: "lane_divider", widthM: 0.0, heightM: 0.0, marking: centerLine },
    { id: 250, role: "lane_divider", widthM: 0.0, heightM: 0.0, marking: centerLine },
    { id: 300, role: "curb", widthM: 0.2, heightM: 0.15, marking: outerLine }
  ]
});

const noLeftSidewalk: RoadSectionInput = centred({
  strips: urbanTwoLane.strips.slice(1),
  laneBands: urbanTwoLane.laneBands,
  boundaries: urbanTwoLane.boundaries.slice(1)
});

const medianTwoLane: RoadSectionInput = centred({
  strips: [
    urbanTwoLane.strips[0],
    urbanTwoLane.strips[1],
    { id: 25, function: "median", widthM: 2.0, crossSlope: 0.0, surfaceStyle: "median" },
    urbanTwoLane.strips[2],
    urbanTwoLane.strips[3]
  ],
  laneBands: urbanTwoLane.laneBands,
  boundaries: [
    { id: 100, role: "curb", widthM: 0.2, heightM: -0.15, marking: outerLine },
    { id: 210, role: "median_edge", widthM: 0.2, heightM: 0.12 },
    { id: 220, role: "median_edge", widthM: 0.2, heightM: -0.12 },
    { id: 300, role: "curb", widthM: 0.2, heightM: 0.15, marking: outerLine }
  ]
});

const shoulderedTwoLane: RoadSectionInput = centred({
  strips: [
    { id: 10, function: "sidewalk", widthM: 2.0, crossSlope: 0.01, surfaceStyle: "sidewalk" },
    { id: 15, function: "shoulder", widthM: 0.75, crossSlope: 0.02, surfaceStyle: "asphalt" },
    { id: 20, function: "carriageway", widthM: 3.0, crossSlope: 0.02, surfaceStyle: "asphalt" },
    { id: 30, function: "carriageway", widthM: 3.0, crossSlope: -0.02, surfaceStyle: "asphalt" },
    { id: 35, function: "shoulder", widthM: 0.75, crossSlope: -0.02, surfaceStyle: "asphalt" },
    { id: 40, function: "sidewalk", widthM: 2.0, crossSlope: -0.01, surfaceStyle: "sidewalk" }
  ],
  laneBands: [
    { id: 1000, stripId: 20, lateralStartM: 0.0, lateralEndM: 3.0, direction: 1 },
    { id: 1010, stripId: 30, lateralStartM: 0.0, lateralEndM: 3.0, direction: 0 }
  ],
  boundaries: [
    { id: 100, role: "curb", widthM: 0.2, heightM: -0.15 },
    { id: 150, role: "outer_edge", widthM: 0.0, heightM: 0.0, marking: outerLine },
    { id: 200, role: "lane_divider", widthM: 0.0, heightM: 0.0, marking: centerLine },
    { id: 250, role: "outer_edge", widthM: 0.0, heightM: 0.0, marking: outerLine },
    { id: 300, role: "curb", widthM: 0.2, heightM: 0.15 }
  ]
});

export interface SeededRoadSections {
  labels: Record<number, string>;
  initialId: number;
}

/**
 * Registers the catalogue into a road state that has none, and reports the IDs
 * Core assigned. Callers hold those IDs; nothing here assumes what they will be.
 * Returns the reason on the first rejection so the caller can discard the state.
 */
export function seedRoadSections(
  addRoadLayoutTemplate: (
    section: RoadSectionInput
  ) => { ok: boolean; error: string; roadLayoutTemplateId?: number }
): { ok: true; sections: SeededRoadSections } | { ok: false; error: string } {
  const labels: Record<number, string> = {};
  let initialId = 0;
  for (const preset of ROAD_TEMPLATE_PRESETS) {
    const result = addRoadLayoutTemplate(preset.section);
    if (!result.ok || result.roadLayoutTemplateId === undefined) {
      return { ok: false, error: `road section "${preset.label}": ${result.error}` };
    }
    labels[result.roadLayoutTemplateId] = preset.label;
    if (preset.initial) initialId = result.roadLayoutTemplateId;
  }
  return { ok: true, sections: { labels, initialId } };
}

export const ROAD_TEMPLATE_PRESETS: readonly RoadTemplatePreset[] = [
  { key: "urban-two-lane", label: "JP 2 lane", initial: true, section: urbanTwoLane },
  { key: "three-lane", label: "JP 3 lane", initial: false, section: threeLane },
  {
    key: "no-left-sidewalk",
    label: "JP 2 lane / no left sidewalk",
    initial: false,
    section: noLeftSidewalk
  },
  {
    key: "median-two-lane",
    label: "JP 2 lane / median",
    initial: false,
    section: medianTwoLane
  },
  {
    key: "shouldered-two-lane",
    label: "JP 2 lane / shoulder",
    initial: false,
    section: shoulderedTwoLane
  }
];
