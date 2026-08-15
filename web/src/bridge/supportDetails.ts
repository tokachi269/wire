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

const EDGE_BODY_KIND = 0;
const SUPPLEMENTAL_KIND = 4;
const HV_CATEGORY = 0;
const HV_LAYER = 1;
const LOCAL_DETAIL_KIND = 3;
const INLINE_DETAIL_KIND = 4;
const HV_DETAIL_COLOR = 0x202124ff;
const DEVICE_DETAIL_COLOR = 0x2e2b26ff;

const TRANSFORMER_MODEL_KEY = "detail_transformer_box";
const TERMINAL_MODEL_KEY = "detail_terminal_post";
const INLINE_DEVICE_MODEL_KEY = "detail_inline_device";

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
const keyCoord = (point: Vec3): string =>
  `${Math.round(point.x * 1000)}:${Math.round(point.y * 1000)}:${Math.round(point.z * 1000)}`;
const yawDeg = (forward: Vec3): number => Math.atan2(forward.y, forward.x) * 180 / Math.PI;

function pointAt(samples: Float64Array, index: number): Vec3 {
  const offset = index * 3;
  return v(samples[offset] ?? 0, samples[offset + 1] ?? 0, samples[offset + 2] ?? 0);
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

function localCablePoints(start: Vec3, startForward: Vec3, guideA: Vec3, guideB: Vec3, endForward: Vec3, end: Vec3) {
  const sf = norm(startForward, sub(end, start));
  const ef = norm(endForward, sub(start, end));
  return [
    start,
    add(start, scale(sf, 0.18)),
    guideA,
    guideB,
    sub(end, scale(ef, 0.12)),
    end
  ];
}

function flatten(points: Vec3[]): Float64Array {
  const out = new Float64Array(points.length * 3);
  points.forEach((point, index) => out.set([point.x, point.y, point.z], index * 3));
  return out;
}

function makeModel(
  stableKey: string,
  modelKey: string,
  contentVersion: string,
  position: Vec3,
  forward: Vec3,
  scaleValue: Vec3
): VisualModelInstanceInfo {
  return {
    stableKey,
    modelKey,
    contentVersion,
    positionX: position.x,
    positionY: position.y,
    positionZ: position.z,
    rotationX: 0,
    rotationY: 0,
    rotationZ: yawDeg(forward),
    scaleX: scaleValue.x,
    scaleY: scaleValue.y,
    scaleZ: scaleValue.z
  };
}

function hvTemplateIds(templates: BundleTemplateInfo[]): Set<number> {
  return new Set(templates
    .filter((template) => template.category === HV_CATEGORY && template.defaultLayer === HV_LAYER)
    .map((template) => template.id));
}

function collectEndpointGroups(parts: SupportDetailPart[], hvTemplates: Set<number>): Map<string, EndpointFrame[]> {
  const groups = new Map<string, EndpointFrame[]>();
  for (const part of parts) {
    if (part.info.kind !== EDGE_BODY_KIND || !hvTemplates.has(part.info.bundleTemplateId)) continue;
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

function appendSupportDetail(groupKey: string, lanesInput: EndpointFrame[], out: SupportDetailScene): void {
  const lanes = [...lanesInput].sort((a, b) => a.laneIndex - b.laneIndex);
  if (lanes.length === 0) return;
  const forward = norm(lanes.reduce((sum, lane) => add(sum, lane.forward), v(0, 0, 0)));
  const lateral = norm(crossUp(forward), v(0, 1, 0));
  const center = scale(lanes.reduce((sum, lane) => add(sum, lane.position), v(0, 0, 0)), 1 / lanes.length);
  const equipment = add(add(center, scale(lateral, 0.38)), v(0, 0, -0.44));
  const version = `${lanes.map((lane) => lane.part.info.sourceVersion).join("|")}:${groupKey}`;
  out.models.push(makeModel(`detail:transformer:${groupKey}`, TRANSFORMER_MODEL_KEY, version,
    equipment, forward, v(0.28, 0.16, 0.36)));

  const spacing = lanes.length > 1 ? 0.18 : 0;
  const origin = -0.5 * spacing * (lanes.length - 1);
  lanes.forEach((lane, index) => {
    const offset = origin + spacing * index;
    const terminal = add(add(equipment, scale(lateral, offset)), v(0, 0, 0.20));
    out.models.push(makeModel(`detail:terminal:${groupKey}:${index}`, TERMINAL_MODEL_KEY,
      `${version}:${index}`, terminal, forward, v(0.08, 0.08, 0.20)));
    const guideA = add(add(lane.position, scale(lane.forward, 0.22)), v(0, 0, -0.10 - 0.03 * index));
    const guideB = add(add(equipment, scale(lateral, offset * 0.65)), v(0, 0, 0.10 + 0.03 * index));
    const points = localCablePoints(lane.position, lane.forward, guideA, guideB, v(0, 0, 1), terminal);
    out.parts.push({
      info: makeInfo(`detail:local:${groupKey}:${index}`, lane.part.info, LOCAL_DETAIL_KIND, points.length, 0.012, HV_DETAIL_COLOR),
      samples: flatten(points)
    });
  });

  const seed = lanes[0];
  const loopStart = add(add(equipment, scale(lateral, -0.14)), v(0, 0, -0.08));
  const loopEnd = add(add(equipment, scale(lateral, 0.14)), v(0, 0, -0.09));
  const loopPoints = localCablePoints(
    loopStart,
    scale(lateral, -1),
    add(add(equipment, scale(lateral, -0.16)), v(0, 0, -0.20)),
    add(add(equipment, scale(lateral, 0.14)), v(0, 0, -0.19)),
    lateral,
    loopEnd
  );
  out.parts.push({
    info: makeInfo(`detail:loop:${groupKey}`, seed.part.info, LOCAL_DETAIL_KIND, loopPoints.length, 0.010, DEVICE_DETAIL_COLOR),
    samples: flatten(loopPoints)
  });
}

function appendInlineDetail(carrier: SupportDetailPart, out: SupportDetailScene): void {
  const count = Math.floor(carrier.samples.length / 3);
  if (count < 4) return;
  const mid = Math.floor(count / 2);
  const before = pointAt(carrier.samples, mid - 1);
  const center = pointAt(carrier.samples, mid);
  const after = pointAt(carrier.samples, Math.min(mid + 1, count - 1));
  const tangent = norm(sub(after, before));
  const lateral = norm(crossUp(tangent), v(0, 1, 0));
  const position = add(center, scale(lateral, 0.045));
  const key = `detail:inline:${carrier.info.partKey}`;
  out.models.push(makeModel(key, INLINE_DEVICE_MODEL_KEY, `${carrier.info.sourceVersion}:inline`,
    position, tangent, v(0.34, 0.12, 0.12)));
  const start = sub(position, scale(tangent, 0.34));
  const end = add(position, scale(tangent, 0.34));
  const bypass = add(add(position, scale(lateral, 0.18)), v(0, 0, 0.04));
  const points = localCablePoints(start, tangent, bypass, add(bypass, v(0, 0, -0.08)), tangent, end);
  out.parts.push({
    info: makeInfo(`${key}:bypass`, carrier.info, INLINE_DETAIL_KIND, points.length, 0.009, DEVICE_DETAIL_COLOR),
    samples: flatten(points)
  });
}

export function deriveSupportDetails(
  parts: SupportDetailPart[],
  templates: BundleTemplateInfo[]
): SupportDetailScene {
  const hvTemplates = hvTemplateIds(templates);
  const out: SupportDetailScene = { parts: [], models: [] };
  const carriers = parts
    .filter((part) => part.info.kind === EDGE_BODY_KIND && hvTemplates.has(part.info.bundleTemplateId))
    .sort((a, b) => a.info.partKey.localeCompare(b.info.partKey));
  const groups = [...collectEndpointGroups(carriers, hvTemplates).entries()]
    .sort(([left], [right]) => left.localeCompare(right));
  for (const [groupKey, lanes] of groups) appendSupportDetail(groupKey, lanes, out);
  if (carriers.length > 0) appendInlineDetail(carriers[0], out);
  return out;
}
