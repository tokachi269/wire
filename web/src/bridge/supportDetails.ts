import type {
  BundleTemplateInfo,
  PoleInfo,
  SupportNodeInfo,
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
  transformerIntermediateInsulator: "transformer_intermediate_insulator_proxy",
  transformerSupportBracket: "transformer_support_bracket_proxy"
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
  transformerIntermediateInsulator: {
    bodyLengthM: 0.220,
    bodyDiameterM: 0.104
  },
  transformerSupportBracket: {
    lengthM: 0.700,
    widthM: 0.260,
    heightM: 0.045
  }
} as const;

const EDGE_BODY_KIND = 0;
const SUPPLEMENTAL_KIND = 4;
const HV_CATEGORY = 0;
const LOCAL_DETAIL_KIND = 3;
const HV_DETAIL_COLOR = 0x262a2dff;
const LV_DETAIL_COLOR = 0x31363aff;
const DECORATIVE_DETAIL_COLOR = 0x2f3437ff;
const MAX_LOCAL_CABLE_SAMPLES = 8;
const SUPPORT_ASSIGN_RADIUS_M = 1.60;

type PoleDetailPatternName = "hv_plain" | "transformer_basic";

interface Vec3 {
  x: number;
  y: number;
  z: number;
}

interface SocketFrame {
  position: Vec3;
  forward: Vec3;
}

interface LaneEndpoint {
  part: SupportDetailPart;
  laneIndex: number;
  edgeId: string;
  position: Vec3;
  forward: Vec3;
}

interface SupportContext {
  id: string;
  poleId: string;
  position: Vec3;
  hvLanes: LaneEndpoint[];
  hvEdgeIds: Set<string>;
}

interface PatternFrame {
  rowCenter: Vec3;
  forward: Vec3;
  lateral: Vec3;
}

interface TransformerBasicSockets {
  insulator0In: SocketFrame;
  insulator0Out: SocketFrame;
  insulator1In: SocketFrame;
  insulator1Out: SocketFrame;
  transformerHv0: SocketFrame;
  transformerHv1: SocketFrame;
  transformerLv: SocketFrame;
}

interface PoleDetailPattern {
  name: PoleDetailPatternName;
  support: SupportContext;
  frame: PatternFrame | null;
  sockets: TransformerBasicSockets | null;
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

function hashString(text: string): number {
  let hash = 2166136261;
  for (let index = 0; index < text.length; index += 1) {
    hash ^= text.charCodeAt(index);
    hash = Math.imul(hash, 16777619);
  }
  return hash >>> 0;
}

function jitter(seed: number, index: number, amplitude: number): number {
  const mixed = Math.imul(seed ^ Math.imul(index + 1, 0x9e3779b1), 2246822519) >>> 0;
  return ((mixed & 0xffff) / 0xffff * 2 - 1) * amplitude;
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
    sourceVersion: `${source.sourceVersion}:pole-detail-pattern`,
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
  const limited = points.slice(0, MAX_LOCAL_CABLE_SAMPLES);
  const out = new Float64Array(limited.length * 3);
  limited.forEach((point, index) => out.set([point.x, point.y, point.z], index * 3));
  return out;
}

function pushCable(
  out: SupportDetailScene,
  key: string,
  source: VisualPartInfo,
  points: Vec3[],
  radius: number,
  color: number
): void {
  const limited = points.slice(0, MAX_LOCAL_CABLE_SAMPLES);
  out.parts.push({
    info: makeInfo(key, source, LOCAL_DETAIL_KIND, limited.length, radius, color),
    samples: flatten(limited)
  });
}

function routePoints(
  start: SocketFrame,
  guides: Vec3[],
  end: SocketFrame,
  variant: "direct" | "small_loop" | "side_loop" | "drop_then_in"
): Vec3[] {
  const base = [
    start.position,
    add(start.position, scale(norm(start.forward, sub(end.position, start.position)), 0.14)),
    ...guides.slice(0, 2),
    sub(end.position, scale(norm(end.forward, sub(start.position, end.position)), 0.12)),
    end.position
  ];
  if (variant === "direct") return base;
  const mid = scale(add(start.position, end.position), 0.5);
  if (variant === "small_loop") return [base[0], base[1], add(mid, v(0, 0, -0.18)), base[base.length - 2], base[base.length - 1]];
  if (variant === "side_loop") return [base[0], base[1], ...guides.slice(0, 1), add(mid, v(0, 0, 0.10)), base[base.length - 2], base[base.length - 1]];
  return [base[0], add(base[0], v(0, 0, -0.16)), ...guides.slice(0, 1), base[base.length - 2], base[base.length - 1]];
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

function makeSupportContexts(poles: PoleInfo[], supportNodes: SupportNodeInfo[]): SupportContext[] {
  const contexts = supportNodes.length > 0
    ? supportNodes.map((support) => ({
        id: `support-node:${support.id}`,
        poleId: support.poleId,
        position: v(support.x, support.y, support.z),
        hvLanes: [],
        hvEdgeIds: new Set<string>()
      }))
    : poles.map((pole) => ({
        id: `pole:${pole.id}`,
        poleId: pole.id,
        position: v(pole.positionX, pole.positionY, pole.positionZ),
        hvLanes: [],
        hvEdgeIds: new Set<string>()
      }));
  return contexts.sort((a, b) => a.id.localeCompare(b.id));
}

function nearestSupportContext(point: Vec3, contexts: SupportContext[]): SupportContext | null {
  let best: SupportContext | null = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (const context of contexts) {
    const distance = Math.hypot(point.x - context.position.x, point.y - context.position.y);
    if (distance < bestDistance) {
      best = context;
      bestDistance = distance;
    }
  }
  return best !== null && bestDistance <= SUPPORT_ASSIGN_RADIUS_M ? best : null;
}

function collectSupportContexts(
  parts: SupportDetailPart[],
  templateIds: Set<number>,
  poles: PoleInfo[],
  supportNodes: SupportNodeInfo[]
): SupportContext[] {
  const contexts = makeSupportContexts(poles, supportNodes);
  for (const part of parts) {
    if (part.info.kind !== EDGE_BODY_KIND || !templateIds.has(part.info.bundleTemplateId)) continue;
    const count = Math.floor(part.samples.length / 3);
    if (count < 2) continue;
    const start = pointAt(part.samples, 0);
    const second = pointAt(part.samples, 1);
    const end = pointAt(part.samples, count - 1);
    const beforeEnd = pointAt(part.samples, count - 2);
    const endpoints: LaneEndpoint[] = [{
      part,
      laneIndex: part.info.laneIndex,
      edgeId: part.info.sourceEdgeId,
      position: start,
      forward: norm(sub(second, start))
    }, {
      part,
      laneIndex: part.info.laneIndex,
      edgeId: part.info.sourceEdgeId,
      position: end,
      forward: norm(sub(beforeEnd, end))
    }];
    for (const endpoint of endpoints) {
      const support = nearestSupportContext(endpoint.position, contexts);
      if (support === null) continue;
      support.hvEdgeIds.add(endpoint.edgeId);
      if (!support.hvLanes.some((item) => item.laneIndex === endpoint.laneIndex)) support.hvLanes.push(endpoint);
    }
  }
  return contexts.filter((context) => context.hvLanes.length > 0);
}

function patternFrame(support: SupportContext): PatternFrame | null {
  const lanes = [...support.hvLanes].sort((a, b) => a.laneIndex - b.laneIndex);
  if (lanes.length === 0) return null;
  const rowCenter = scale(lanes.reduce((sum, lane) => add(sum, lane.position), v(0, 0, 0)), 1 / lanes.length);
  const forward = norm(lanes.reduce((sum, lane) => add(sum, lane.forward), v(0, 0, 0)));
  return { rowCenter, forward, lateral: norm(crossUp(forward), v(0, 1, 0)) };
}

function choosePatternName(support: SupportContext): PoleDetailPatternName {
  return support.hvEdgeIds.size >= 2 ? "transformer_basic" : "hv_plain";
}

function makePattern(support: SupportContext): PoleDetailPattern {
  const frame = patternFrame(support);
  const name = choosePatternName(support);
  return {
    name,
    support,
    frame,
    sockets: name === "transformer_basic" && frame !== null ? transformerBasicSockets(support, frame) : null
  };
}

function transformerBasicSockets(support: SupportContext, frame: PatternFrame): TransformerBasicSockets {
  const poleAtHvRow = v(support.position.x, support.position.y, frame.rowCenter.z);
  const transformerOffset = poleRadiusAtHeight(frame.rowCenter.z) +
    SUPPORT_DETAIL_DIMENSIONS.transformer.bodyDiameterM * 0.5 +
    SUPPORT_DETAIL_DIMENSIONS.transformer.radialClearanceM;
  const transformerCenter = add(
    add(poleAtHvRow, scale(frame.lateral, transformerOffset)),
    v(0, 0, -SUPPORT_DETAIL_DIMENSIONS.transformer.defaultCenterBelowHvRowM)
  );
  const insulator0 = add(add(frame.rowCenter, scale(frame.lateral, 0.25)), v(0, 0, -0.35));
  const insulator1 = add(add(frame.rowCenter, scale(frame.lateral, 0.50)), v(0, 0, -0.35));
  return {
    insulator0In: { position: add(insulator0, scale(frame.forward, -0.12)), forward: frame.forward },
    insulator0Out: { position: add(insulator0, scale(frame.forward, 0.12)), forward: scale(frame.forward, -1) },
    insulator1In: { position: add(insulator1, scale(frame.forward, -0.12)), forward: frame.forward },
    insulator1Out: { position: add(insulator1, scale(frame.forward, 0.12)), forward: scale(frame.forward, -1) },
    transformerHv0: { position: add(add(transformerCenter, scale(frame.lateral, -0.06)), v(0, 0, 0.28)), forward: v(0, 0, 1) },
    transformerHv1: { position: add(add(transformerCenter, scale(frame.lateral, 0.06)), v(0, 0, 0.28)), forward: v(0, 0, 1) },
    transformerLv: { position: add(transformerCenter, v(0, 0, -0.16)), forward: frame.lateral }
  };
}

function appendTransformerBasicParts(pattern: PoleDetailPattern, out: SupportDetailScene): void {
  if (pattern.frame === null || pattern.sockets === null) return;
  const seed = hashString(pattern.support.id);
  const version = `${pattern.support.id}:transformer-basic:v1`;
  const transformerCenter = add(pattern.sockets.transformerLv.position, v(0, 0, 0.16));
  const insulator0Center = scale(add(pattern.sockets.insulator0In.position, pattern.sockets.insulator0Out.position), 0.5);
  const insulator1Center = scale(add(pattern.sockets.insulator1In.position, pattern.sockets.insulator1Out.position), 0.5);
  const bracketCenter = scale(add(insulator0Center, insulator1Center), 0.5);
  const jittered = (point: Vec3, index: number): Vec3 =>
    add(add(point, scale(pattern.frame!.lateral, jitter(seed, index, 0.025))), v(0, 0, jitter(seed, index + 7, 0.020)));

  out.models.push(makeModel(
    `support-detail:${pattern.support.id}:transformer`,
    SUPPORT_DETAIL_MODEL_KEYS.transformer,
    version,
    jittered(transformerCenter, 1),
    pattern.frame.forward
  ));
  out.models.push(makeModel(
    `support-detail:${pattern.support.id}:transformer-bracket`,
    SUPPORT_DETAIL_MODEL_KEYS.transformerSupportBracket,
    `${version}:bracket`,
    jittered(bracketCenter, 2),
    pattern.frame.forward
  ));
  out.models.push(makeModel(
    `support-detail:${pattern.support.id}:intermediate-insulator:0`,
    SUPPORT_DETAIL_MODEL_KEYS.transformerIntermediateInsulator,
    `${version}:insulator:0`,
    jittered(insulator0Center, 3),
    pattern.frame.forward
  ));
  out.models.push(makeModel(
    `support-detail:${pattern.support.id}:intermediate-insulator:1`,
    SUPPORT_DETAIL_MODEL_KEYS.transformerIntermediateInsulator,
    `${version}:insulator:1`,
    jittered(insulator1Center, 4),
    pattern.frame.forward
  ));
}

function laneFor(pattern: PoleDetailPattern, preferredLane: number): LaneEndpoint | null {
  const lanes = [...pattern.support.hvLanes].sort((a, b) => a.laneIndex - b.laneIndex);
  return lanes.find((lane) => lane.laneIndex === preferredLane) ?? lanes[Math.min(preferredLane, lanes.length - 1)] ?? null;
}

function appendTransformerBasicConnections(pattern: PoleDetailPattern, out: SupportDetailScene): void {
  if (pattern.frame === null || pattern.sockets === null) return;
  const source = pattern.support.hvLanes[0]?.part.info;
  if (source === undefined) return;
  const lane0 = laneFor(pattern, 0);
  const lane2 = laneFor(pattern, 2);
  const seed = hashString(pattern.support.id);
  const variantA = (seed & 1) === 0 ? "small_loop" : "drop_then_in";
  const variantB = (seed & 2) === 0 ? "side_loop" : "small_loop";
  if (lane0 !== null) {
    pushCable(out, `support-detail:${pattern.support.id}:hv-lane0-to-insulator0`, lane0.part.info,
      routePoints(
        { position: lane0.position, forward: lane0.forward },
        [add(lane0.position, scale(lane0.forward, 0.18))],
        pattern.sockets.insulator0In,
        variantA
      ),
      0.010,
      HV_DETAIL_COLOR
    );
  }
  pushCable(out, `support-detail:${pattern.support.id}:insulator0-to-transformer`, source,
    routePoints(
      pattern.sockets.insulator0Out,
      [add(pattern.sockets.transformerHv0.position, scale(pattern.frame.lateral, -0.06))],
      pattern.sockets.transformerHv0,
      "drop_then_in"
    ),
    0.010,
    HV_DETAIL_COLOR
  );
  if (lane2 !== null) {
    pushCable(out, `support-detail:${pattern.support.id}:hv-lane2-to-insulator1`, lane2.part.info,
      routePoints(
        { position: lane2.position, forward: lane2.forward },
        [add(lane2.position, scale(lane2.forward, 0.18))],
        pattern.sockets.insulator1In,
        variantB
      ),
      0.010,
      HV_DETAIL_COLOR
    );
  }
  pushCable(out, `support-detail:${pattern.support.id}:insulator1-to-transformer`, source,
    routePoints(
      pattern.sockets.insulator1Out,
      [add(pattern.sockets.transformerHv1.position, scale(pattern.frame.lateral, 0.06))],
      pattern.sockets.transformerHv1,
      "drop_then_in"
    ),
    0.010,
    HV_DETAIL_COLOR
  );
  const lvEnd = add(add(pattern.sockets.transformerLv.position, scale(pattern.frame.lateral, 0.50)), v(0, 0, -0.10));
  pushCable(out, `support-detail:${pattern.support.id}:transformer-lv-main`, source,
    routePoints(
      pattern.sockets.transformerLv,
      [add(pattern.sockets.transformerLv.position, v(0, 0, -0.18))],
      { position: lvEnd, forward: scale(pattern.frame.lateral, -1) },
      "side_loop"
    ),
    0.011,
    LV_DETAIL_COLOR
  );
  pushCable(out, `support-detail:${pattern.support.id}:decorative-loop`, source,
    [
      add(pattern.sockets.transformerLv.position, scale(pattern.frame.lateral, -0.10)),
      add(add(pattern.sockets.transformerLv.position, scale(pattern.frame.lateral, -0.18)), v(0, 0, -0.20)),
      add(add(pattern.sockets.transformerLv.position, scale(pattern.frame.lateral, 0.20)), v(0, 0, -0.18)),
      add(pattern.sockets.transformerLv.position, scale(pattern.frame.lateral, 0.12))
    ],
    0.008,
    DECORATIVE_DETAIL_COLOR
  );
}

function appendPattern(pattern: PoleDetailPattern, out: SupportDetailScene): void {
  if (pattern.name === "hv_plain") return;
  appendTransformerBasicParts(pattern, out);
  appendTransformerBasicConnections(pattern, out);
}

export function deriveSupportDetails(
  parts: SupportDetailPart[],
  templates: BundleTemplateInfo[],
  poles: PoleInfo[] = [],
  supportNodes: SupportNodeInfo[] = []
): SupportDetailScene {
  const hvTemplates = templateIdsForCategory(templates, HV_CATEGORY);
  const out: SupportDetailScene = { parts: [], models: [] };
  for (const support of collectSupportContexts(parts, hvTemplates, poles, supportNodes)) {
    appendPattern(makePattern(support), out);
  }
  return out;
}
