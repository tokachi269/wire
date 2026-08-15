import type {
  BundleTemplateInfo,
  VisualModelInstanceInfo,
  VisualPartInfo
} from "../model";

export interface SupportDetailPart {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface SupportDetailScene {
  parts: SupportDetailPart[];
  models: VisualModelInstanceInfo[];
}

export const SUPPORT_DETAIL_MODEL_KEYS = {
  transformer: "pole_transformer_20kva_proxy",
  pc6Cutout: "pc6_cutout_proxy",
  tma13Mount: "tma13_cutout_mount_proxy",
  arrester: "arrester_gl_b6g_proxy",
  triplexTermination: "hv_triplex_termination_60_proxy",
  opticalClosure: "aerial_optical_closure_rca3ao_proxy"
} as const;

export const SUPPORT_DETAIL_DIMENSIONS = {
  transformer: {
    bodyDiameterM: 0.400,
    bodyHeightM: 0.550,
    lidDiameterM: 0.430,
    lidThicknessM: 0.030,
    radialClearanceM: 0.060,
    defaultCenterBelowHvRowM: 1.35
  },
  pc6Cutout: {
    bodyLengthM: 0.250,
    bodyWidthM: 0.090,
    bodyDepthM: 0.094,
    overallLengthM: 0.406,
    mountReachM: 0.190
  },
  tma13Mount: {
    topLengthM: 0.135,
    topWidthM: 0.032,
    verticalHeightM: 0.115,
    innerSpanM: 0.096,
    boltDiameterM: 0.010
  },
  arrester: {
    bodyDiameterM: 0.072,
    capMaxDiameterM: 0.082,
    overallHeightM: 0.250
  },
  triplexTermination: {
    totalLengthM: 0.580,
    mainLengthM: 0.398,
    maxDiameterM: 0.095,
    terminalDiameterM: 0.020,
    terminalSizeM: 0.023,
    baseWidthM: 0.130,
    assemblySpanM: 0.710,
    branchSpacingM: 0.200
  },
  opticalClosure: {
    lengthM: 0.730,
    widthM: 0.150,
    heightM: 0.200
  }
} as const;

const EDGE_BODY_KIND = 0;
const SUPPLEMENTAL_KIND = 4;
const HV_CATEGORY = 0;
const OPTICAL_CATEGORY = 3;
const LOCAL_DETAIL_KIND = 3;
const INLINE_DETAIL_KIND = 4;
const HV_DETAIL_COLOR = 0x262a2dff;
const LV_DETAIL_COLOR = 0x31363aff;
const OPTICAL_DETAIL_COLOR = 0x1f2529ff;
const HANGER_COLOR = 0x8a9294ff;
const MAX_LOCAL_CABLE_SAMPLES = 8;

interface Vec3 {
  x: number;
  y: number;
  z: number;
}

interface EndpointFrame {
  key: string;
  part: SupportDetailPart;
  laneIndex: number;
  position: Vec3;
  forward: Vec3;
}

interface SocketFrame {
  position: Vec3;
  forward: Vec3;
}

const v = (x: number, y: number, z: number): Vec3 => ({ x, y, z });
const add = (a: Vec3, b: Vec3): Vec3 => v(a.x + b.x, a.y + b.y, a.z + b.z);
const sub = (a: Vec3, b: Vec3): Vec3 => v(a.x - b.x, a.y - b.y, a.z - b.z);
const scale = (a: Vec3, k: number): Vec3 => v(a.x * k, a.y * k, a.z * k);
const crossUp = (a: Vec3): Vec3 => v(-a.y, a.x, 0);
const length = (a: Vec3): number => Math.hypot(a.x, a.y, a.z);
const norm = (a: Vec3, fallback: Vec3 = v(1, 0, 0)): Vec3 => {
  const len = length(a);
  return len > 1e-9 ? scale(a, 1 / len) : fallback;
};
const yawDeg = (forward: Vec3): number => Math.atan2(forward.y, forward.x) * 180 / Math.PI;
const keyCoord = (point: Vec3): string =>
  `${Math.round(point.x * 1000)}:${Math.round(point.y * 1000)}:${Math.round(point.z * 1000)}`;

function pointAt(samples: Float64Array, index: number): Vec3 {
  const offset = index * 3;
  return v(samples[offset] ?? 0, samples[offset + 1] ?? 0, samples[offset + 2] ?? 0);
}

function poleRadiusAtHeight(heightM: number): number {
  const visibleHeightM = 10.0;
  const clamped = Math.min(Math.max(heightM, 0), visibleHeightM);
  const distanceFromTopM = visibleHeightM - clamped;
  return (0.190 + distanceFromTopM / 75.0) * 0.5;
}

function makeInfo(
  key: string,
  source: VisualPartInfo,
  supplementalKind: number,
  sampleCount: number,
  radius: number,
  color: number
): VisualPartInfo {
  return {
    ...source,
    partKey: key,
    sourceVersion: `${source.sourceVersion}:presentation-detail`,
    sampleOffset: 0,
    sampleCount,
    kind: SUPPLEMENTAL_KIND,
    supplementalKind,
    wireRadius: radius,
    colorRgba: color,
    runId: 0
  };
}

function flatten(points: Vec3[]): Float64Array {
  const out = new Float64Array(points.length * 3);
  points.slice(0, MAX_LOCAL_CABLE_SAMPLES).forEach((point, index) => {
    out.set([point.x, point.y, point.z], index * 3);
  });
  return out;
}

function pushCable(
  out: SupportDetailScene,
  key: string,
  source: VisualPartInfo,
  supplementalKind: number,
  points: Vec3[],
  radius: number,
  color: number
): void {
  const limited = points.slice(0, MAX_LOCAL_CABLE_SAMPLES);
  out.parts.push({
    info: makeInfo(key, source, supplementalKind, limited.length, radius, color),
    samples: flatten(limited)
  });
}

function localCablePoints(start: SocketFrame, guides: Vec3[], end: SocketFrame): Vec3[] {
  const points = [
    start.position,
    add(start.position, scale(norm(start.forward, sub(end.position, start.position)), 0.16)),
    ...guides.slice(0, 2),
    sub(end.position, scale(norm(end.forward, sub(start.position, end.position)), 0.12)),
    end.position
  ];
  return points.slice(0, MAX_LOCAL_CABLE_SAMPLES);
}

function makeModel(
  stableKey: string,
  modelKey: string,
  contentVersion: string,
  position: Vec3,
  forward: Vec3,
  scaleValue: Vec3 = v(1, 1, 1),
  rotationX = 0,
  rotationY = 0
): VisualModelInstanceInfo {
  return {
    stableKey,
    modelKey,
    contentVersion,
    positionX: position.x,
    positionY: position.y,
    positionZ: position.z,
    rotationX,
    rotationY,
    rotationZ: yawDeg(forward),
    scaleX: scaleValue.x,
    scaleY: scaleValue.y,
    scaleZ: scaleValue.z
  };
}

function templateIdsForCategory(templates: BundleTemplateInfo[], category: number): Set<number> {
  return new Set(templates
    .filter((template) => template.category === category)
    .map((template) => template.id));
}

function collectEndpointGroups(parts: SupportDetailPart[], templateIds: Set<number>): Map<string, EndpointFrame[]> {
  const groups = new Map<string, EndpointFrame[]>();
  for (const part of parts) {
    if (part.info.kind !== EDGE_BODY_KIND || !templateIds.has(part.info.bundleTemplateId)) continue;
    const count = Math.floor(part.samples.length / 3);
    if (count < 2) continue;
    const start = pointAt(part.samples, 0);
    const second = pointAt(part.samples, 1);
    const end = pointAt(part.samples, count - 1);
    const beforeEnd = pointAt(part.samples, count - 2);
    const endpoints: EndpointFrame[] = [{
      key: `${part.info.bundleTemplateId}:${keyCoord(start)}`,
      part,
      laneIndex: part.info.laneIndex,
      position: start,
      forward: norm(sub(second, start))
    }, {
      key: `${part.info.bundleTemplateId}:${keyCoord(end)}`,
      part,
      laneIndex: part.info.laneIndex,
      position: end,
      forward: norm(sub(beforeEnd, end))
    }];
    for (const endpoint of endpoints) {
      const group = groups.get(endpoint.key) ?? [];
      if (!group.some((item) => item.laneIndex === endpoint.laneIndex)) group.push(endpoint);
      groups.set(endpoint.key, group);
    }
  }
  return groups;
}

function appendTransformerEquipment(groupKey: string, lanesInput: EndpointFrame[], out: SupportDetailScene): void {
  const lanes = [...lanesInput].sort((a, b) => a.laneIndex - b.laneIndex);
  if (lanes.length === 0) return;
  const forward = norm(lanes.reduce((sum, lane) => add(sum, lane.forward), v(0, 0, 0)));
  const lateral = norm(crossUp(forward), v(0, 1, 0));
  const rowCenter = scale(lanes.reduce((sum, lane) => add(sum, lane.position), v(0, 0, 0)), 1 / lanes.length);
  const transformerOffset = poleRadiusAtHeight(rowCenter.z) +
    SUPPORT_DETAIL_DIMENSIONS.transformer.bodyDiameterM * 0.5 +
    SUPPORT_DETAIL_DIMENSIONS.transformer.radialClearanceM;
  const transformerCenter = add(
    add(rowCenter, scale(lateral, transformerOffset)),
    v(0, 0, -SUPPORT_DETAIL_DIMENSIONS.transformer.defaultCenterBelowHvRowM)
  );
  const version = `${lanes.map((lane) => lane.part.info.sourceVersion).join("|")}:${groupKey}:catalog-v2`;

  out.models.push(makeModel(
    `support-detail:transformer:${groupKey}`,
    SUPPORT_DETAIL_MODEL_KEYS.transformer,
    version,
    transformerCenter,
    forward
  ));

  const pc6Count = Math.max(1, Math.min(lanes.length, 6));
  for (let index = 0; index < pc6Count; index += 1) {
    const lane = lanes[index % lanes.length];
    const laneOffset = (index - (pc6Count - 1) * 0.5) * 0.18;
    const mountCenter = add(add(rowCenter, scale(lateral, laneOffset)), v(0, 0, -0.18));
    const cutoutCenter = add(mountCenter, scale(lateral, SUPPORT_DETAIL_DIMENSIONS.pc6Cutout.mountReachM));
    const cutoutSocketIn = {
      position: add(cutoutCenter, scale(forward, -SUPPORT_DETAIL_DIMENSIONS.pc6Cutout.bodyLengthM * 0.5)),
      forward
    };
    const cutoutSocketOut = {
      position: add(cutoutCenter, scale(forward, SUPPORT_DETAIL_DIMENSIONS.pc6Cutout.bodyLengthM * 0.5)),
      forward: scale(forward, -1)
    };
    out.models.push(makeModel(
      `support-detail:tma13:${groupKey}:${index}`,
      SUPPORT_DETAIL_MODEL_KEYS.tma13Mount,
      `${version}:mount:${index}`,
      mountCenter,
      forward
    ));
    out.models.push(makeModel(
      `support-detail:pc6:${groupKey}:${index}`,
      SUPPORT_DETAIL_MODEL_KEYS.pc6Cutout,
      `${version}:pc6:${index}`,
      cutoutCenter,
      forward
    ));

    const rowSocket = { position: lane.position, forward: lane.forward };
    pushCable(out, `support-detail:hv-to-pc6:${groupKey}:${index}`, lane.part.info, LOCAL_DETAIL_KIND,
      localCablePoints(
        rowSocket,
        [add(lane.position, scale(lane.forward, 0.22)), add(cutoutSocketIn.position, v(0, 0, 0.08))],
        cutoutSocketIn
      ),
      0.010,
      HV_DETAIL_COLOR
    );
    const transformerSocket = {
      position: add(transformerCenter, v(0, 0, SUPPORT_DETAIL_DIMENSIONS.transformer.bodyHeightM * 0.5)),
      forward: v(0, 0, 1)
    };
    pushCable(out, `support-detail:pc6-to-transformer:${groupKey}:${index}`, lane.part.info, LOCAL_DETAIL_KIND,
      localCablePoints(
        cutoutSocketOut,
        [add(cutoutSocketOut.position, v(0, 0, -0.16 - 0.03 * index)), add(transformerSocket.position, scale(lateral, laneOffset * 0.35))],
        transformerSocket
      ),
      0.010,
      HV_DETAIL_COLOR
    );
  }

  const arresterCount = Math.max(1, Math.min(3, lanes.length));
  for (let index = 0; index < arresterCount; index += 1) {
    const lane = lanes[index % lanes.length];
    const offset = (index - (arresterCount - 1) * 0.5) * 0.16;
    const arresterCenter = add(add(rowCenter, scale(lateral, -0.18 + offset)), v(0, 0, -0.42));
    out.models.push(makeModel(
      `support-detail:arrester:${groupKey}:${index}`,
      SUPPORT_DETAIL_MODEL_KEYS.arrester,
      `${version}:arrester:${index}`,
      arresterCenter,
      forward
    ));
    pushCable(out, `support-detail:arrester-line:${groupKey}:${index}`, lane.part.info, LOCAL_DETAIL_KIND,
      localCablePoints(
        { position: lane.position, forward: lane.forward },
        [add(lane.position, scale(lane.forward, 0.14)), add(arresterCenter, v(0, 0, 0.18))],
        { position: add(arresterCenter, v(0, 0, SUPPORT_DETAIL_DIMENSIONS.arrester.overallHeightM * 0.5)), forward: v(0, 0, 1) }
      ),
      0.009,
      HV_DETAIL_COLOR
    );
  }

  const lvSocketA = add(transformerCenter, v(0, 0, -0.16));
  const lvSocketB = add(add(transformerCenter, scale(lateral, 0.36)), v(0, 0, -0.28));
  pushCable(out, `support-detail:transformer-lv:${groupKey}`, lanes[0].part.info, LOCAL_DETAIL_KIND,
    localCablePoints(
      { position: lvSocketA, forward: scale(lateral, 1) },
      [add(transformerCenter, v(0, 0, -0.34)), add(lvSocketB, v(0, 0, -0.10))],
      { position: lvSocketB, forward: scale(lateral, -1) }
    ),
    0.011,
    LV_DETAIL_COLOR
  );
}

function appendTriplexTermination(carrier: SupportDetailPart, out: SupportDetailScene): void {
  const count = Math.floor(carrier.samples.length / 3);
  if (count < 4) return;
  const mid = Math.floor(count / 2);
  const before = pointAt(carrier.samples, mid - 1);
  const center = pointAt(carrier.samples, mid);
  const after = pointAt(carrier.samples, Math.min(mid + 1, count - 1));
  const tangent = norm(sub(after, before));
  const lateral = norm(crossUp(tangent), v(0, 1, 0));
  const version = `${carrier.info.sourceVersion}:triplex-catalog-v2`;
  const position = add(center, v(0, 0, -0.18));
  out.models.push(makeModel(
    `support-detail:triplex-termination:${carrier.info.partKey}`,
    SUPPORT_DETAIL_MODEL_KEYS.triplexTermination,
    version,
    position,
    tangent
  ));
  const incoming = sub(position, scale(tangent, 0.58));
  const bundled = sub(position, scale(tangent, 0.30));
  pushCable(out, `support-detail:triplex-incoming:${carrier.info.partKey}`, carrier.info, INLINE_DETAIL_KIND,
    [incoming, add(incoming, scale(tangent, 0.20)), bundled],
    0.018,
    HV_DETAIL_COLOR
  );
  for (const [index, offset] of [-0.200, 0, 0.200].entries()) {
    const branch = add(position, scale(lateral, offset));
    const terminal = add(branch, scale(tangent, 0.29));
    pushCable(out, `support-detail:triplex-fanout:${carrier.info.partKey}:${index}`, carrier.info, INLINE_DETAIL_KIND,
      [
        bundled,
        add(add(position, scale(tangent, -0.12)), scale(lateral, offset * 0.35)),
        add(add(position, scale(tangent, 0.08)), scale(lateral, offset * 0.80)),
        terminal
      ],
      0.010,
      HV_DETAIL_COLOR
    );
  }
}

function appendOpticalClosure(carrier: SupportDetailPart, out: SupportDetailScene): void {
  const count = Math.floor(carrier.samples.length / 3);
  if (count < 4) return;
  const mid = Math.floor(count / 2);
  const before = pointAt(carrier.samples, mid - 1);
  const centerOnCarrier = pointAt(carrier.samples, mid);
  const after = pointAt(carrier.samples, Math.min(mid + 1, count - 1));
  const tangent = norm(sub(after, before));
  const lateral = norm(crossUp(tangent), v(0, 1, 0));
  const down = v(0, 0, -1);
  const closureCenter = add(centerOnCarrier, scale(down, SUPPORT_DETAIL_DIMENSIONS.opticalClosure.heightM * 0.5 + 0.030));
  const version = `${carrier.info.sourceVersion}:optical-closure-catalog-v2`;
  out.models.push(makeModel(
    `support-detail:optical-closure:${carrier.info.partKey}`,
    SUPPORT_DETAIL_MODEL_KEYS.opticalClosure,
    version,
    closureCenter,
    tangent
  ));
  for (const offset of [-0.225, 0.225]) {
    const carrierPoint = add(centerOnCarrier, scale(tangent, offset));
    const closurePoint = add(closureCenter, scale(tangent, offset));
    pushCable(out, `support-detail:optical-hanger:${carrier.info.partKey}:${offset}`, carrier.info, INLINE_DETAIL_KIND,
      [carrierPoint, closurePoint],
      0.008,
      HANGER_COLOR
    );
  }
  const endA = sub(closureCenter, scale(tangent, SUPPORT_DETAIL_DIMENSIONS.opticalClosure.lengthM * 0.5));
  const endB = add(closureCenter, scale(tangent, SUPPORT_DETAIL_DIMENSIONS.opticalClosure.lengthM * 0.5));
  pushCable(out, `support-detail:optical-through:${carrier.info.partKey}`, carrier.info, INLINE_DETAIL_KIND,
    localCablePoints(
      { position: sub(endA, scale(tangent, 0.24)), forward: tangent },
      [endA, endB],
      { position: add(endB, scale(tangent, 0.24)), forward: scale(tangent, -1) }
    ),
    0.009,
    OPTICAL_DETAIL_COLOR
  );
  pushCable(out, `support-detail:optical-branch:${carrier.info.partKey}`, carrier.info, INLINE_DETAIL_KIND,
    localCablePoints(
      { position: closureCenter, forward: lateral },
      [add(add(closureCenter, scale(lateral, 0.24)), v(0, 0, -0.08))],
      { position: add(add(closureCenter, scale(lateral, 0.72)), v(0, 0, -0.18)), forward: scale(lateral, -1) }
    ),
    0.007,
    OPTICAL_DETAIL_COLOR
  );
}

export function deriveSupportDetails(
  parts: SupportDetailPart[],
  templates: BundleTemplateInfo[]
): SupportDetailScene {
  const hvTemplates = templateIdsForCategory(templates, HV_CATEGORY);
  const opticalTemplates = templateIdsForCategory(templates, OPTICAL_CATEGORY);
  const out: SupportDetailScene = { parts: [], models: [] };
  const hvCarriers = parts
    .filter((part) => part.info.kind === EDGE_BODY_KIND && hvTemplates.has(part.info.bundleTemplateId))
    .sort((a, b) => a.info.partKey.localeCompare(b.info.partKey));
  const groups = [...collectEndpointGroups(hvCarriers, hvTemplates).entries()]
    .sort(([left], [right]) => left.localeCompare(right));
  for (const [groupKey, lanes] of groups) appendTransformerEquipment(groupKey, lanes, out);
  if (hvCarriers.length > 0) appendTriplexTermination(hvCarriers[0], out);

  const opticalCarriers = parts
    .filter((part) => part.info.kind === EDGE_BODY_KIND && opticalTemplates.has(part.info.bundleTemplateId))
    .sort((a, b) => a.info.partKey.localeCompare(b.info.partKey));
  if (opticalCarriers.length > 0) appendOpticalClosure(opticalCarriers[0], out);
  return out;
}

export type SupportDetailReferenceKind = "transformer" | "triplex" | "optical";

function referenceInfo(key: string, bundleTemplateId: number, laneIndex: number, color: number): VisualPartInfo {
  return {
    partKey: key,
    sourceVersion: "support-detail-reference-v1",
    sampleOffset: 0,
    sampleCount: 0,
    kind: EDGE_BODY_KIND,
    supplementalKind: 0,
    wireRadius: 0.012,
    colorRgba: color,
    sourceNodeId: "0",
    sourceEdgeId: "0",
    sourceSpanId: "0",
    sourceBundleId: "0",
    bundleTemplateId,
    laneIndex,
    runId: 0
  };
}

function referencePart(
  key: string,
  source: VisualPartInfo,
  points: Vec3[],
  radius: number,
  color: number,
  supplementalKind = LOCAL_DETAIL_KIND
): SupportDetailPart {
  return {
    info: makeInfo(key, source, supplementalKind, points.length, radius, color),
    samples: flatten(points)
  };
}

function referenceModel(
  stableKey: string,
  modelKey: string,
  position: Vec3,
  forward: Vec3 = v(1, 0, 0),
  scaleValue: Vec3 = v(1, 1, 1)
): VisualModelInstanceInfo {
  return makeModel(stableKey, modelKey, "support-detail-reference-v1", position, forward, scaleValue);
}

function basePoleModels(prefix: string, crossarmZ = 8.85): VisualModelInstanceInfo[] {
  return [
    referenceModel(`${prefix}:pole`, "pole_body", v(0, 0, 0)),
    referenceModel(`${prefix}:belt`, "pole_belt", v(0, 0, crossarmZ - 0.08)),
    referenceModel(`${prefix}:crossarm`, "hv_crossarm", v(0, 0, crossarmZ), v(0, 1, 0)),
    ...[-0.60, 0, 0.60].map((x, index) =>
      referenceModel(`${prefix}:insulator:${index}`, "hv_insulator", v(x, 0, crossarmZ + 0.18))
    )
  ];
}

function transformerReferenceScene(): SupportDetailScene {
  const source = referenceInfo("reference:hv:source", 101, 0, HV_DETAIL_COLOR);
  const out: SupportDetailScene = {
    parts: [],
    models: basePoleModels("reference:transformer")
  };
  out.models.push(
    referenceModel("reference:transformer:body", SUPPORT_DETAIL_MODEL_KEYS.transformer, v(0.38, 0.42, 7.50)),
    referenceModel("reference:transformer:pc6:0", SUPPORT_DETAIL_MODEL_KEYS.pc6Cutout, v(-0.34, 0.34, 8.48)),
    referenceModel("reference:transformer:pc6:1", SUPPORT_DETAIL_MODEL_KEYS.pc6Cutout, v(0.00, 0.34, 8.48)),
    referenceModel("reference:transformer:pc6:2", SUPPORT_DETAIL_MODEL_KEYS.pc6Cutout, v(0.34, 0.34, 8.48)),
    referenceModel("reference:transformer:tma13:0", SUPPORT_DETAIL_MODEL_KEYS.tma13Mount, v(-0.34, 0.16, 8.48)),
    referenceModel("reference:transformer:tma13:1", SUPPORT_DETAIL_MODEL_KEYS.tma13Mount, v(0.00, 0.16, 8.48)),
    referenceModel("reference:transformer:tma13:2", SUPPORT_DETAIL_MODEL_KEYS.tma13Mount, v(0.34, 0.16, 8.48)),
    referenceModel("reference:transformer:arrester:0", SUPPORT_DETAIL_MODEL_KEYS.arrester, v(-0.52, -0.18, 8.32)),
    referenceModel("reference:transformer:arrester:1", SUPPORT_DETAIL_MODEL_KEYS.arrester, v(0.52, -0.18, 8.32))
  );
  for (const [index, x] of [-0.60, 0, 0.60].entries()) {
    out.parts.push(referencePart(`reference:transformer:hv-local:${index}`, source, [
      v(x, 0, 9.03),
      v(x, 0.20, 8.83),
      v(x * 0.56, 0.34, 8.55),
      v(x * 0.56, 0.34, 8.48)
    ], 0.010, HV_DETAIL_COLOR));
    out.parts.push(referencePart(`reference:transformer:drop:${index}`, source, [
      v(x * 0.56, 0.34, 8.36),
      v(x * 0.40, 0.50, 8.03),
      v(0.38 + x * 0.08, 0.48, 7.83),
      v(0.38, 0.42, 7.81)
    ], 0.010, HV_DETAIL_COLOR));
  }
  out.parts.push(referencePart("reference:transformer:lv-local", source, [
    v(0.38, 0.42, 7.25),
    v(0.54, 0.55, 7.15),
    v(0.86, 0.60, 7.18),
    v(1.12, 0.54, 7.28)
  ], 0.011, LV_DETAIL_COLOR));
  return out;
}

function triplexReferenceScene(): SupportDetailScene {
  const source = referenceInfo("reference:triplex:source", 101, 0, HV_DETAIL_COLOR);
  const out: SupportDetailScene = {
    parts: [],
    models: [
      referenceModel("reference:triplex:termination", SUPPORT_DETAIL_MODEL_KEYS.triplexTermination, v(0.80, 0, 6.1))
    ]
  };
  out.parts.push(referencePart("reference:triplex:incoming", source, [
    v(-1.40, 0, 6.1),
    v(-0.70, 0, 6.1),
    v(0.36, 0, 6.1)
  ], 0.018, HV_DETAIL_COLOR, INLINE_DETAIL_KIND));
  for (const [index, y] of [-0.20, 0, 0.20].entries()) {
    out.parts.push(referencePart(`reference:triplex:fanout:${index}`, source, [
      v(0.36, 0, 6.1),
      v(0.52, y * 0.45, 6.12),
      v(0.70, y, 6.1),
      v(1.09, y, 6.1)
    ], 0.010, HV_DETAIL_COLOR, INLINE_DETAIL_KIND));
  }
  return out;
}

function opticalReferenceScene(): SupportDetailScene {
  const source = referenceInfo("reference:optical:source", 105, 0, OPTICAL_DETAIL_COLOR);
  const out: SupportDetailScene = {
    parts: [],
    models: [
      referenceModel("reference:optical:closure", SUPPORT_DETAIL_MODEL_KEYS.opticalClosure, v(0, 0, 5.65))
    ]
  };
  out.parts.push(referencePart("reference:optical:carrier", source, [
    v(-2.2, 0, 5.78),
    v(-0.6, 0, 5.80),
    v(0.6, 0, 5.80),
    v(2.2, 0, 5.78)
  ], 0.009, OPTICAL_DETAIL_COLOR, INLINE_DETAIL_KIND));
  for (const x of [-0.225, 0.225]) {
    out.parts.push(referencePart(`reference:optical:hanger:${x}`, source, [
      v(x, 0, 5.80),
      v(x, 0, 5.76)
    ], 0.008, HANGER_COLOR, INLINE_DETAIL_KIND));
  }
  out.parts.push(referencePart("reference:optical:branch", source, [
    v(0, 0, 5.65),
    v(0.12, 0.28, 5.58),
    v(0.62, 0.70, 5.46)
  ], 0.007, OPTICAL_DETAIL_COLOR, INLINE_DETAIL_KIND));
  return out;
}

export function supportDetailReferenceScene(kind: SupportDetailReferenceKind): SupportDetailScene {
  const scene = kind === "transformer"
    ? transformerReferenceScene()
    : kind === "triplex"
      ? triplexReferenceScene()
      : opticalReferenceScene();
  return translateScene(scene, v(16, 0, 0));
}

function translateScene(scene: SupportDetailScene, offset: Vec3): SupportDetailScene {
  return {
    models: scene.models.map((model) => ({
      ...model,
      positionX: model.positionX + offset.x,
      positionY: model.positionY + offset.y,
      positionZ: model.positionZ + offset.z
    })),
    parts: scene.parts.map((part) => {
      const samples = new Float64Array(part.samples);
      for (let index = 0; index < samples.length; index += 3) {
        samples[index] += offset.x;
        samples[index + 1] += offset.y;
        samples[index + 2] += offset.z;
      }
      return { info: part.info, samples };
    })
  };
}
