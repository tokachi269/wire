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

/** `inside` and `outside` are toward and away from the carriageway. */
export type RoadMarkingPlacement = "center" | "inside" | "outside";

export interface RoadSectionMarkingInput {
  role: RoadMarkingRole;
  style: RoadMarkingStyle;
  placement: RoadMarkingPlacement;
}

/**
 * One corner of a boundary's cross section, measured from the edge datum — the
 * road-facing vertical face of whatever sits there. Lateral runs the way the
 * section does, left to right, and height is relative to the datum.
 */
export interface RoadProfilePointInput {
  lateralM: number;
  heightM: number;
  /** Surface of the face running to the next point. The last point has none. */
  faceStyle?: RoadSurfaceStyle;
}

export interface RoadSectionBoundaryInput {
  id: number;
  role: RoadBoundaryRole;
  profile: RoadProfilePointInput[];
  marking?: RoadSectionMarkingInput;
}

/** Nothing but a place to paint a line: no structure, no width taken. */
const paintedLine: RoadProfilePointInput[] = [{ lateralM: 0, heightM: 0 }];

/**
 * A curb. Its top face comes out of the walkway beside it rather than being
 * added to the road, so a 2.0m walkway with a 0.2m curb is still 2.0m wide.
 */
function curb(
  side: "left" | "right",
  widthM: number,
  heightM: number
): RoadProfilePointInput[] {
  return side === "left"
    ? [{ lateralM: -widthM, heightM, faceStyle: "curb" }, { lateralM: 0, heightM: 0 }]
    : [{ lateralM: 0, heightM: 0, faceStyle: "curb" }, { lateralM: widthM, heightM }];
}

/**
 * L型側溝, 三和コンクリート工業 鉄筋コンクリート L 250B
 * (JIS A 5371 推奨仕様 C-1 / JIS A 5372 推奨仕様 E-4), w=450mm.
 *
 * Reading the product table: w = b + a + c, the wall stands f above the
 * channel, the channel climbs 10% across a and the road-side lip 5% across c.
 * That reproduces the table's own h and i (55 + 0.10x250 = 80, +0.05x100 = 85).
 *
 * The wall top lands in the walkway and the channel and lip in the roadway, so
 * neither declared width grows. e and g describe the buried block, which the
 * visible section does not carry.
 */
const lsGutter = {
  wallTopM: 0.1, // b
  wallFaceM: 0.1, // f
  channelM: 0.25, // a
  channelGrade: 0.1, // 10%
  lipM: 0.1, // c
  lipGrade: 0.05 // 5%
};

function lGutter(side: "left" | "right"): RoadProfilePointInput[] {
  const channelDrop = -lsGutter.wallFaceM;
  const channelEnd = channelDrop + lsGutter.channelM * lsGutter.channelGrade;
  const lipEnd = channelEnd + lsGutter.lipM * lsGutter.lipGrade;
  const outward: RoadProfilePointInput[] = [
    { lateralM: -lsGutter.wallTopM, heightM: 0, faceStyle: "curb" },
    { lateralM: 0, heightM: 0, faceStyle: "curb" },
    { lateralM: 0, heightM: channelDrop, faceStyle: "curb" },
    { lateralM: lsGutter.channelM, heightM: channelEnd, faceStyle: "curb" },
    { lateralM: lsGutter.channelM + lsGutter.lipM, heightM: lipEnd }
  ];
  if (side === "left") return outward;
  return outward
    .map((point) => ({ lateralM: -point.lateralM, heightM: point.heightM }))
    .reverse()
    .map((point, index, all) => (index + 1 === all.length ? point : { ...point, faceStyle: "curb" as const }));
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

function centred(
  section: Omit<RoadSectionInput, "alignmentOffsetFromLeftM">
): RoadSectionInput {
  // Only strips carry layout width; a boundary's structure reaches into them.
  const total = section.strips.reduce((sum, strip) => sum + strip.widthM, 0);
  return { ...section, alignmentOffsetFromLeftM: total / 2 };
}

export interface RoadTemplatePreset {
  key: string;
  label: string;
  initial: boolean;
  section: RoadSectionInput;
}

const outerLine: RoadSectionMarkingInput = {
  role: "carriageway_edge",
  style: "white_solid",
  placement: "inside"
};
const centerLine: RoadSectionMarkingInput = {
  role: "center_line",
  style: "center_line",
  placement: "center"
};

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
    { id: 100, role: "curb", profile: lGutter("left"), marking: outerLine },
    { id: 200, role: "lane_divider", profile: paintedLine, marking: centerLine },
    { id: 300, role: "curb", profile: lGutter("right"), marking: outerLine }
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
    { id: 100, role: "curb", profile: lGutter("left"), marking: outerLine },
    { id: 200, role: "lane_divider", profile: paintedLine, marking: centerLine },
    { id: 250, role: "lane_divider", profile: paintedLine, marking: centerLine },
    { id: 300, role: "curb", profile: lGutter("right"), marking: outerLine }
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
    { id: 100, role: "curb", profile: lGutter("left"), marking: outerLine },
    { id: 210, role: "median_edge", profile: curb("right", 0.2, 0.12) },
    { id: 220, role: "median_edge", profile: curb("left", 0.2, 0.12) },
    { id: 300, role: "curb", profile: lGutter("right"), marking: outerLine }
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
    { id: 100, role: "curb", profile: lGutter("left") },
    { id: 150, role: "outer_edge", profile: paintedLine, marking: outerLine },
    { id: 200, role: "lane_divider", profile: paintedLine, marking: centerLine },
    { id: 250, role: "outer_edge", profile: paintedLine, marking: outerLine },
    { id: 300, role: "curb", profile: lGutter("right") }
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
