import type {
  BundleTemplateInfo,
  CableTemplateInfo,
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
  transformerSupportBracket: {
    lengthM: 0.700,
    widthM: 0.260,
    heightM: 0.045
  }
} as const;

const EDGE_BODY_KIND = 0;
const SUPPLEMENTAL_KIND = 4;
const HV_CATEGORY = 0;
const LV_CATEGORY = 1;
const LOCAL_DETAIL_KIND = 3;
const MAX_LOCAL_CABLE_SAMPLES = 8;
const SUPPORT_ASSIGN_RADIUS_M = 1.60;
const TRANSFORMER_BASIC_HASH_SLOT = 1;
const HV_INSULATOR_MODEL_KEY = "hv_insulator";

type AttachmentAnchorKind = "pole" | "span";
type AttachmentTemplateId = "transformer_basic";

interface Vec3 {
  x: number;
  y: number;
  z: number;
}

interface SocketFrame {
  position: Vec3;
  forward: Vec3;
}

interface CarrierRef {
  category: number;
  bundleTemplateId: number;
  appearance: CableAppearance;
  part: SupportDetailPart;
  laneIndex: number;
  position: Vec3;
  forward: Vec3;
}

interface PoleDetailInput {
  id: string;
  poleId: string;
  position: Vec3;
  rotation: Vec3;
  up: Vec3;
  carriers: CarrierRef[];
}

interface AttachmentAnchor {
  kind: AttachmentAnchorKind;
  ownerId: string;
  t?: number;
}

interface PresentationAttachment {
  id: string;
  anchor: AttachmentAnchor;
  templateId: AttachmentTemplateId;
  seed: number;
}

interface PatternFrame {
  rowCenter: Vec3;
  forward: Vec3;
  lateral: Vec3;
  up: Vec3;
  heightAlongPole: number;
}

interface TransformerBasicSockets {
  transformerCenter: Vec3;
  bracketCenter: Vec3;
  insulator0: SocketFrame;
  insulator1: SocketFrame;
  transformerHv0: SocketFrame;
  transformerHv1: SocketFrame;
  transformerLv0: SocketFrame;
  transformerLv1: SocketFrame;
  transformerLv2: SocketFrame;
}

interface TransformerBasicRecipe {
  attachment: PresentationAttachment;
  input: PoleDetailInput;
  frame: PatternFrame;
  sockets: TransformerBasicSockets;
  hvCarriers: CarrierRef[];
  lvCarriers: CarrierRef[];
}

interface CableAppearance {
  radius: number;
  materialStyle: number;
  color: number;
}

function fallbackAppearance(info: VisualPartInfo): CableAppearance {
  return {
    radius: info.wireRadius,
    materialStyle: info.materialStyle,
    color: info.colorRgba
  };
}

const v = (x: number, y: number, z: number): Vec3 => ({ x, y, z });
const add = (a: Vec3, b: Vec3): Vec3 => v(a.x + b.x, a.y + b.y, a.z + b.z);
const sub = (a: Vec3, b: Vec3): Vec3 => v(a.x - b.x, a.y - b.y, a.z - b.z);
const scale = (a: Vec3, k: number): Vec3 => v(a.x * k, a.y * k, a.z * k);
const dot = (a: Vec3, b: Vec3): number => a.x * b.x + a.y * b.y + a.z * b.z;
const cross = (a: Vec3, b: Vec3): Vec3 => v(
  a.y * b.z - a.z * b.y,
  a.z * b.x - a.x * b.z,
  a.x * b.y - a.y * b.x
);
const length = (a: Vec3): number => Math.hypot(a.x, a.y, a.z);
const norm = (a: Vec3, fallback: Vec3 = v(1, 0, 0)): Vec3 => {
  const len = length(a);
  return len > 1e-9 ? scale(a, 1 / len) : fallback;
};
const projectOntoPlane = (a: Vec3, normal: Vec3): Vec3 => sub(a, scale(normal, dot(a, normal)));
const yawDeg = (forward: Vec3): number => Math.atan2(forward.y, forward.x) * 180 / Math.PI;

function rotateX(value: Vec3, deg: number): Vec3 {
  const rad = deg * Math.PI / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return v(value.x, value.y * c - value.z * s, value.y * s + value.z * c);
}

function rotateY(value: Vec3, deg: number): Vec3 {
  const rad = deg * Math.PI / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return v(value.x * c + value.z * s, value.y, -value.x * s + value.z * c);
}

function rotateZ(value: Vec3, deg: number): Vec3 {
  const rad = deg * Math.PI / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return v(value.x * c - value.y * s, value.x * s + value.y * c, value.z);
}

function rotateEulerXYZ(value: Vec3, eulerDeg: Vec3): Vec3 {
  return rotateZ(rotateY(rotateX(value, eulerDeg.x), eulerDeg.y), eulerDeg.z);
}

function poleRelative(frame: PatternFrame, x: number, y: number, z: number): Vec3 {
  return add(add(scale(frame.forward, x), scale(frame.lateral, y)), scale(frame.up, z));
}

function worldFromPole(input: PoleDetailInput, frame: PatternFrame, x: number, y: number, z: number): Vec3 {
  return add(input.position, poleRelative(frame, x, y, z));
}

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
  appearance: CableAppearance
): VisualPartInfo {
  return {
    ...source,
    partKey: key,
    sourceVersion: `${source.sourceVersion}:attachment-detail`,
    sampleOffset: 0,
    sampleCount,
    kind: SUPPLEMENTAL_KIND,
    supplementalKind,
    wireRadius: appearance.radius,
    materialStyle: appearance.materialStyle,
    colorRgba: appearance.color,
    runId: 0
  };
}

function flatten(points: Vec3[]): Float64Array {
  const limited = points.slice(0, MAX_LOCAL_CABLE_SAMPLES);
  const out = new Float64Array(limited.length * 3);
  limited.forEach((point, index) => out.set([point.x, point.y, point.z], index * 3));
  return out;
}

function catmullRom(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: number): Vec3 {
  const t2 = t * t;
  const t3 = t2 * t;
  return v(
    0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
      (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
      (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
    0.5 * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 +
      (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3)
  );
}

function smoothLocalCablePoints(points: Vec3[]): Vec3[] {
  if (points.length <= 2) return points;
  const sampleCount = MAX_LOCAL_CABLE_SAMPLES;
  const out: Vec3[] = [];
  const segmentCount = points.length - 1;
  for (let sampleIndex = 0; sampleIndex < sampleCount; sampleIndex += 1) {
    const pathT = sampleIndex / (sampleCount - 1) * segmentCount;
    const segmentIndex = Math.min(Math.floor(pathT), segmentCount - 1);
    const localT = pathT - segmentIndex;
    const p0 = points[Math.max(0, segmentIndex - 1)];
    const p1 = points[segmentIndex];
    const p2 = points[segmentIndex + 1];
    const p3 = points[Math.min(points.length - 1, segmentIndex + 2)];
    out.push(sampleIndex === 0 ? points[0]
      : sampleIndex === sampleCount - 1 ? points[points.length - 1]
        : catmullRom(p0, p1, p2, p3, localT));
  }
  return out;
}

function pushCable(
  out: SupportDetailScene,
  key: string,
  source: VisualPartInfo,
  points: Vec3[],
  appearance: CableAppearance
): void {
  const limited = smoothLocalCablePoints(points);
  out.parts.push({
    info: makeInfo(key, source, LOCAL_DETAIL_KIND, limited.length, appearance),
    samples: flatten(limited)
  });
}

function routePoints(
  start: SocketFrame,
  guides: Vec3[],
  end: SocketFrame
): Vec3[] {
  return [
    start.position,
    add(start.position, scale(norm(start.forward, sub(end.position, start.position)), 0.14)),
    ...guides.slice(0, 2),
    sub(end.position, scale(norm(end.forward, sub(start.position, end.position)), 0.12)),
    end.position
  ];
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

function templateCategoryById(templates: BundleTemplateInfo[]): Map<number, number> {
  return new Map(templates.map((template) => [template.id, template.category]));
}

function templateAppearanceById(
  templates: BundleTemplateInfo[],
  cableTemplates: CableTemplateInfo[]
): Map<number, CableAppearance> {
  const cableById = new Map(cableTemplates.map((template) => [template.id, template]));
  const out = new Map<number, CableAppearance>();
  for (const template of templates) {
    const cable = cableById.get(template.cableTemplateId);
    if (cable === undefined) continue;
    out.set(template.id, {
      radius: cable.outerDiameter * 0.5,
      materialStyle: cable.materialStyle,
      color: cable.colorRgba
    });
  }
  return out;
}

function makePoleDetailInputs(poles: PoleInfo[], supportNodes: SupportNodeInfo[]): PoleDetailInput[] {
  const poleById = new Map(poles.map((pole) => [pole.id, pole]));
  const poleFrame = (pole: PoleInfo | undefined): { rotation: Vec3; up: Vec3 } => {
    const rotation = pole === undefined ? v(0, 0, 0) : v(pole.rotationX, pole.rotationY, pole.rotationZ);
    return { rotation, up: norm(rotateEulerXYZ(v(0, 0, 1), rotation), v(0, 0, 1)) };
  };
  const inputs = supportNodes.length > 0
    ? supportNodes.map((support) => {
        const frame = poleFrame(poleById.get(support.poleId));
        return {
          id: `support-node:${support.id}`,
          poleId: support.poleId,
          position: v(support.x, support.y, support.z),
          rotation: frame.rotation,
          up: frame.up,
          carriers: []
        };
      })
    : poles.map((pole) => {
        const frame = poleFrame(pole);
        return {
          id: `pole:${pole.id}`,
          poleId: pole.id,
          position: v(pole.positionX, pole.positionY, pole.positionZ),
          rotation: frame.rotation,
          up: frame.up,
          carriers: []
        };
      });
  return inputs.sort((a, b) => a.id.localeCompare(b.id));
}

function nearestPoleDetailInput(point: Vec3, inputs: PoleDetailInput[]): PoleDetailInput | null {
  let best: PoleDetailInput | null = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (const input of inputs) {
    const distance = Math.hypot(point.x - input.position.x, point.y - input.position.y);
    if (distance < bestDistance) {
      best = input;
      bestDistance = distance;
    }
  }
  return best !== null && bestDistance <= SUPPORT_ASSIGN_RADIUS_M ? best : null;
}

function collectPoleDetailInputs(
  parts: SupportDetailPart[],
  templateCategories: Map<number, number>,
  templateAppearances: Map<number, CableAppearance>,
  poles: PoleInfo[],
  supportNodes: SupportNodeInfo[]
): PoleDetailInput[] {
  const inputs = makePoleDetailInputs(poles, supportNodes);
  for (const part of parts) {
    const category = templateCategories.get(part.info.bundleTemplateId);
    if (part.info.kind !== EDGE_BODY_KIND || category === undefined) continue;
    const count = Math.floor(part.samples.length / 3);
    if (count < 2) continue;
    const start = pointAt(part.samples, 0);
    const second = pointAt(part.samples, 1);
    const end = pointAt(part.samples, count - 1);
    const beforeEnd = pointAt(part.samples, count - 2);
    const appearance = templateAppearances.get(part.info.bundleTemplateId) ?? fallbackAppearance(part.info);
    const endpoints: CarrierRef[] = [{
      category,
      bundleTemplateId: part.info.bundleTemplateId,
      appearance,
      part,
      laneIndex: part.info.laneIndex,
      position: start,
      forward: norm(sub(second, start))
    }, {
      category,
      bundleTemplateId: part.info.bundleTemplateId,
      appearance,
      part,
      laneIndex: part.info.laneIndex,
      position: end,
      forward: norm(sub(beforeEnd, end))
    }];
    for (const endpoint of endpoints) {
      const input = nearestPoleDetailInput(endpoint.position, inputs);
      if (input === null) continue;
      if (!input.carriers.some((item) =>
        item.part.info.partKey === endpoint.part.info.partKey && item.laneIndex === endpoint.laneIndex
      )) {
        input.carriers.push(endpoint);
      }
    }
  }
  return inputs.filter((input) => input.carriers.length > 0);
}

function patternFrame(input: PoleDetailInput, carriersInput: CarrierRef[]): PatternFrame | null {
  const lanes = [...carriersInput].sort((a, b) => a.laneIndex - b.laneIndex);
  if (lanes.length === 0) return null;
  const rowCenter = scale(lanes.reduce((sum, lane) => add(sum, lane.position), v(0, 0, 0)), 1 / lanes.length);
  const rawForward = lanes.reduce((sum, lane) => add(sum, lane.forward), v(0, 0, 0));
  const fallbackForward = rotateEulerXYZ(v(1, 0, 0), input.rotation);
  const forward = norm(projectOntoPlane(rawForward, input.up), fallbackForward);
  return {
    rowCenter,
    forward,
    lateral: norm(cross(input.up, forward), rotateEulerXYZ(v(0, 1, 0), input.rotation)),
    up: input.up,
    heightAlongPole: dot(sub(rowCenter, input.position), input.up)
  };
}

function transformerBasicSockets(input: PoleDetailInput, frame: PatternFrame): TransformerBasicSockets {
  const transformerOffset = poleRadiusAtHeight(frame.heightAlongPole) +
    SUPPORT_DETAIL_DIMENSIONS.transformer.bodyDiameterM * 0.5 +
    SUPPORT_DETAIL_DIMENSIONS.transformer.radialClearanceM;
  const transformerCenter = worldFromPole(
    input,
    frame,
    0,
    transformerOffset,
    frame.heightAlongPole - SUPPORT_DETAIL_DIMENSIONS.transformer.defaultCenterBelowHvRowM
  );
  const bracketCenter = worldFromPole(
    input,
    frame,
    0,
    SUPPORT_DETAIL_DIMENSIONS.transformerSupportBracket.lengthM * 0.5,
    frame.heightAlongPole - SUPPORT_DETAIL_DIMENSIONS.transformer.defaultCenterBelowHvRowM + 0.03
  );
  const insulator0 = add(add(bracketCenter, scale(frame.lateral, 0.06)), scale(frame.forward, -0.18));
  const insulator1 = add(add(bracketCenter, scale(frame.lateral, 0.06)), scale(frame.forward, 0.18));
  const lvBase = add(transformerCenter, scale(frame.up, -0.16));
  return {
    transformerCenter,
    bracketCenter,
    insulator0: { position: insulator0, forward: frame.lateral },
    insulator1: { position: insulator1, forward: frame.lateral },
    transformerHv0: { position: add(add(transformerCenter, scale(frame.lateral, -0.06)), scale(frame.up, 0.28)), forward: frame.up },
    transformerHv1: { position: add(add(transformerCenter, scale(frame.lateral, 0.06)), scale(frame.up, 0.28)), forward: frame.up },
    transformerLv0: { position: add(lvBase, scale(frame.forward, -0.045)), forward: frame.lateral },
    transformerLv1: { position: lvBase, forward: frame.lateral },
    transformerLv2: { position: add(lvBase, scale(frame.forward, 0.045)), forward: frame.lateral }
  };
}

function populatePoleAttachments(inputs: PoleDetailInput[]): PresentationAttachment[] {
  return inputs
    .filter((input) => hashString(input.id) % 4 === TRANSFORMER_BASIC_HASH_SLOT)
    .map((input) => ({
      id: `attachment:${input.id}:transformer_basic`,
      anchor: { kind: "pole", ownerId: input.id },
      templateId: "transformer_basic",
      seed: hashString(input.id)
    }));
}

function transformerBasicRecipe(
  attachment: PresentationAttachment,
  input: PoleDetailInput
): TransformerBasicRecipe | null {
  if (attachment.anchor.kind !== "pole" || attachment.templateId !== "transformer_basic") return null;
  const hvCarriers = input.carriers
    .filter((carrier) => carrier.category === HV_CATEGORY)
    .sort((a, b) => a.laneIndex - b.laneIndex);
  const lvCarriers = input.carriers
    .filter((carrier) => carrier.category === LV_CATEGORY)
    .sort((a, b) => a.laneIndex - b.laneIndex);
  const frame = patternFrame(input, hvCarriers);
  if (frame === null || hvCarriers.length === 0) return null;
  return {
    attachment,
    input,
    frame,
    sockets: transformerBasicSockets(input, frame),
    hvCarriers,
    lvCarriers
  };
}

function appendTransformerBasicParts(recipe: TransformerBasicRecipe, out: SupportDetailScene): void {
  const version = `${recipe.attachment.id}:v1`;

  out.models.push(makeModel(
    `${recipe.attachment.id}:transformer`,
    SUPPORT_DETAIL_MODEL_KEYS.transformer,
    version,
    recipe.sockets.transformerCenter,
    recipe.frame.forward,
    v(1, 1, 1),
    recipe.input.rotation.x,
    recipe.input.rotation.y
  ));
  out.models.push(makeModel(
    `${recipe.attachment.id}:transformer-bracket`,
    SUPPORT_DETAIL_MODEL_KEYS.transformerSupportBracket,
    `${version}:bracket`,
    recipe.sockets.bracketCenter,
    recipe.frame.lateral,
    v(1, 1, 1),
    recipe.input.rotation.x,
    recipe.input.rotation.y
  ));
  out.models.push(makeModel(
    `${recipe.attachment.id}:intermediate-insulator:0`,
    HV_INSULATOR_MODEL_KEY,
    `${version}:insulator:0`,
    recipe.sockets.insulator0.position,
    recipe.frame.lateral,
    v(0.42, 0.42, 0.42),
    recipe.input.rotation.x,
    recipe.input.rotation.y + 90
  ));
  out.models.push(makeModel(
    `${recipe.attachment.id}:intermediate-insulator:1`,
    HV_INSULATOR_MODEL_KEY,
    `${version}:insulator:1`,
    recipe.sockets.insulator1.position,
    recipe.frame.lateral,
    v(0.42, 0.42, 0.42),
    recipe.input.rotation.x,
    recipe.input.rotation.y + 90
  ));
}

function laneFor(recipe: TransformerBasicRecipe, preferredLane: number): CarrierRef | null {
  const lanes = [...recipe.hvCarriers].sort((a, b) => a.laneIndex - b.laneIndex);
  return lanes.find((lane) => lane.laneIndex === preferredLane) ?? lanes[Math.min(preferredLane, lanes.length - 1)] ?? null;
}

function laneTapSocket(recipe: TransformerBasicRecipe, lane: CarrierRef, side: number): SocketFrame {
  const tapPosition = add(
    add(lane.position, scale(recipe.frame.lateral, 0.05 * side)),
    scale(recipe.frame.up, -0.035)
  );
  return {
    position: tapPosition,
    forward: norm(add(scale(recipe.frame.lateral, 0.80 * side), scale(recipe.frame.up, -0.20)), lane.forward)
  };
}

function lvCarrierFor(recipe: TransformerBasicRecipe, index: number): CarrierRef | null {
  return recipe.lvCarriers[index] ?? recipe.lvCarriers[Math.min(index, recipe.lvCarriers.length - 1)] ?? null;
}

function lvTapSocket(recipe: TransformerBasicRecipe, carrier: CarrierRef, index: number): SocketFrame {
  const offset = add(
    scale(recipe.frame.forward, (index - 1) * 0.08),
    scale(recipe.frame.up, -0.025)
  );
  return {
    position: add(carrier.position, offset),
    forward: scale(carrier.forward, -1)
  };
}

function appendTransformerBasicConnections(recipe: TransformerBasicRecipe, out: SupportDetailScene): void {
  const source = recipe.hvCarriers[0]?.part.info;
  if (source === undefined) return;
  const lane0 = laneFor(recipe, 0);
  const lane2 = laneFor(recipe, 2);
  const seed = recipe.attachment.seed;
  const mirror = (seed & 1) === 0 ? 1 : -1;
  const guideJitter = (index: number, lateralM = 0.035, verticalM = 0.030): Vec3 =>
    add(scale(recipe.frame.lateral, jitter(seed, index, lateralM) * mirror), scale(recipe.frame.up, jitter(seed, index + 11, verticalM)));
  if (lane0 !== null) {
    const lane0Tap = laneTapSocket(recipe, lane0, -1);
    const lane0DropGuide = add(
      add(lane0Tap.position, scale(recipe.frame.lateral, -0.14)),
      scale(recipe.frame.up, -0.11)
    );
    const insulator0Approach = add(
      add(recipe.sockets.insulator0.position, scale(recipe.frame.forward, -0.08)),
      scale(recipe.frame.up, -0.045)
    );
    pushCable(out, `${recipe.attachment.id}:hv-lane0-to-insulator0`, lane0.part.info,
      routePoints(
        lane0Tap,
        [
          add(lane0DropGuide, guideJitter(1, 0.018, 0.018)),
          add(insulator0Approach, guideJitter(2, 0.016, 0.016))
        ],
        recipe.sockets.insulator0
      ),
      lane0.appearance
    );
  }
  const hv0Appearance = lane0?.appearance ?? recipe.hvCarriers[0].appearance;
  const hv0LeadGuideA = add(
    add(recipe.sockets.insulator0.position, scale(recipe.frame.lateral, -0.06)),
    scale(recipe.frame.up, -0.08)
  );
  const hv0LeadGuideB = add(
    add(recipe.sockets.transformerHv0.position, scale(recipe.frame.lateral, -0.16)),
    scale(recipe.frame.up, 0.06)
  );
  pushCable(out, `${recipe.attachment.id}:insulator0-to-transformer`, source,
    routePoints(
      recipe.sockets.insulator0,
      [
        add(hv0LeadGuideA, guideJitter(3, 0.018, 0.018)),
        add(hv0LeadGuideB, guideJitter(4, 0.018, 0.018))
      ],
      recipe.sockets.transformerHv0
    ),
    hv0Appearance
  );
  if (lane2 !== null) {
    const lane2Tap = laneTapSocket(recipe, lane2, 1);
    const lane2DropGuide = add(
      add(lane2Tap.position, scale(recipe.frame.lateral, 0.14)),
      scale(recipe.frame.up, -0.11)
    );
    const insulator1Approach = add(
      add(recipe.sockets.insulator1.position, scale(recipe.frame.forward, -0.08)),
      scale(recipe.frame.up, -0.045)
    );
    pushCable(out, `${recipe.attachment.id}:hv-lane2-to-insulator1`, lane2.part.info,
      routePoints(
        lane2Tap,
        [
          add(lane2DropGuide, guideJitter(5, 0.018, 0.018)),
          add(insulator1Approach, guideJitter(6, 0.016, 0.016))
        ],
        recipe.sockets.insulator1
      ),
      lane2.appearance
    );
  }
  const hv1Appearance = lane2?.appearance ?? recipe.hvCarriers[0].appearance;
  const hv1LeadGuideA = add(
    add(recipe.sockets.insulator1.position, scale(recipe.frame.lateral, 0.06)),
    scale(recipe.frame.up, -0.08)
  );
  const hv1LeadGuideB = add(
    add(recipe.sockets.transformerHv1.position, scale(recipe.frame.lateral, 0.16)),
    scale(recipe.frame.up, 0.06)
  );
  pushCable(out, `${recipe.attachment.id}:insulator1-to-transformer`, source,
    routePoints(
      recipe.sockets.insulator1,
      [
        add(hv1LeadGuideA, guideJitter(7, 0.018, 0.018)),
        add(hv1LeadGuideB, guideJitter(8, 0.018, 0.018))
      ],
      recipe.sockets.transformerHv1
    ),
    hv1Appearance
  );
  const lvSockets = [recipe.sockets.transformerLv0, recipe.sockets.transformerLv1, recipe.sockets.transformerLv2];
  lvSockets.forEach((socket, index) => {
    const lvCarrier = lvCarrierFor(recipe, index);
    if (lvCarrier === null) return;
    const lvTap = lvTapSocket(recipe, lvCarrier, index);
    const closeGuide = add(add(socket.position, scale(recipe.frame.lateral, 0.16)), guideJitter(5 + index, 0.018, 0.018));
    pushCable(out, `${recipe.attachment.id}:transformer-lv:${index}`, source,
      routePoints(
        socket,
        [closeGuide, add(add(lvTap.position, scale(recipe.frame.lateral, -0.16)), guideJitter(8 + index, 0.025, 0.025))],
        lvTap
      ),
      lvCarrier.appearance
    );
  });
  pushCable(out, `${recipe.attachment.id}:pole-drop`, source,
    [
      worldFromPole(recipe.input, recipe.frame, 0, 0.10, dot(sub(recipe.sockets.transformerCenter, recipe.input.position), recipe.frame.up) + 0.10),
      worldFromPole(recipe.input, recipe.frame, 0, 0.11, dot(sub(recipe.sockets.transformerCenter, recipe.input.position), recipe.frame.up) - 0.20),
      worldFromPole(recipe.input, recipe.frame, 0, 0.10, dot(sub(recipe.sockets.transformerCenter, recipe.input.position), recipe.frame.up) - 0.62),
      worldFromPole(recipe.input, recipe.frame, 0, 0.12, dot(sub(recipe.sockets.transformerCenter, recipe.input.position), recipe.frame.up) - 0.96)
    ],
    recipe.hvCarriers[0].appearance
  );
}

function appendAttachmentVisual(
  attachment: PresentationAttachment,
  inputById: Map<string, PoleDetailInput>,
  out: SupportDetailScene
): void {
  if (attachment.anchor.kind !== "pole") return;
  const input = inputById.get(attachment.anchor.ownerId);
  if (input === undefined) return;
  const recipe = transformerBasicRecipe(attachment, input);
  if (recipe === null) return;
  appendTransformerBasicParts(recipe, out);
  appendTransformerBasicConnections(recipe, out);
}

export function deriveSupportDetails(
  parts: SupportDetailPart[],
  templates: BundleTemplateInfo[],
  cableTemplates: CableTemplateInfo[] = [],
  poles: PoleInfo[] = [],
  supportNodes: SupportNodeInfo[] = []
): SupportDetailScene {
  const templateCategories = templateCategoryById(templates);
  const templateAppearances = templateAppearanceById(templates, cableTemplates);
  const out: SupportDetailScene = { parts: [], models: [] };
  const inputs = collectPoleDetailInputs(parts, templateCategories, templateAppearances, poles, supportNodes);
  const inputById = new Map(inputs.map((input) => [input.id, input]));
  for (const attachment of populatePoleAttachments(inputs)) {
    appendAttachmentVisual(attachment, inputById, out);
  }
  return out;
}
