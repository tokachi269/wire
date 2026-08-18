import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

import type {
  ModelAssemblyBootstrapInput,
  ModelAssemblyPartInput,
  ModelSocketInput,
  ModelTransformInput
} from "../model";
import beltUrl from "../assets/belt.glb?url";
import communicationClampUrl from "../assets/communication_clamp_1.glb?url";
import communicationClampLongUrl from "../assets/communication_clamp_long_1.glb?url";
import crossarmHvUrl from "../assets/crossarm_hv_1p7m.glb?url";
import hvInsulatorUrl from "../assets/hv_phase_1_disc_3.glb?url";
import poleBodyUrl from "../assets/pole_body_tapered_12m_visible10m.glb?url";

export type ModelAssetKind =
  | "belt"
  | "communicationClamp"
  | "communicationClampLong"
  | "crossarmHv"
  | "hvInsulator"
  | "poleBody"
  | "poleDecorationX"
  | "poleTransformer20kvaProxy"
  | "transformerIntermediateInsulatorProxy"
  | "transformerSupportBracketProxy"
  | "pc6CutoutProxy"
  | "tma13CutoutMountProxy"
  | "arresterGlB6gProxy"
  | "hvTriplexTermination60Proxy"
  | "aerialOpticalClosureRca3aoProxy";

export type ModelKey =
  | "pole_belt"
  | "communication_clamp"
  | "communication_clamp_long"
  | "hv_crossarm"
  | "hv_insulator"
  | "pole_body"
  | "pole_decoration_x"
  | "pole_transformer_20kva_proxy"
  | "transformer_intermediate_insulator_proxy"
  | "transformer_support_bracket_proxy"
  | "pc6_cutout_proxy"
  | "tma13_cutout_mount_proxy"
  | "arrester_gl_b6g_proxy"
  | "hv_triplex_termination_60_proxy"
  | "aerial_optical_closure_rca3ao_proxy";

type MountRule = "center" | "bottom" | "pole-ground";

export interface ModelAssetAdapter {
  modelKey: ModelKey;
  url: string;
  mountRule: MountRule;
  visibleLengthM?: number;
  radialReferenceM?: number;
  radialTopM?: number;
  adapterVersion: number;
}

export interface LoadedModelAsset {
  kind: ModelAssetKind;
  modelKey: ModelKey;
  source: THREE.Group;
  bounds: THREE.Box3;
  size: THREE.Vector3;
  mountAnchor: THREE.Vector3;
  radialReferenceM: number | null;
  radialTopM: number | null;
  descriptorVersion: number;
  adapter: ModelAssetAdapter;
}

type SceneLoader = (url: string) => Promise<THREE.Group>;

const polePrimitive = {
  totalLengthM: 12.0,
  visibleHeightM: 10.0,
  topDiameterM: 0.190,
  taperRatio: 75.0
} as const;
const poleRadiusAtDistanceFromTop = (distanceM: number): number =>
  (polePrimitive.topDiameterM + distanceM / polePrimitive.taperRatio) * 0.5;

const adapters: Record<ModelAssetKind, ModelAssetAdapter> = {
  belt: {
    modelKey: "pole_belt",
    url: beltUrl,
    mountRule: "center",
    // The authored lower-end radius is only the radial reference; the belt is
    // anchored at its own center and scaled to the actual placement height.
    radialReferenceM: poleRadiusAtDistanceFromTop(polePrimitive.totalLengthM),
    adapterVersion: 5
  },
  communicationClamp: {
    modelKey: "communication_clamp",
    url: communicationClampUrl,
    mountRule: "center",
    adapterVersion: 2
  },
  communicationClampLong: {
    modelKey: "communication_clamp_long",
    url: communicationClampLongUrl,
    mountRule: "center",
    adapterVersion: 2
  },
  crossarmHv: {
    modelKey: "hv_crossarm",
    url: crossarmHvUrl,
    mountRule: "center",
    adapterVersion: 2
  },
  hvInsulator: {
    modelKey: "hv_insulator",
    url: hvInsulatorUrl,
    mountRule: "bottom",
    adapterVersion: 2
  },
  poleBody: {
    modelKey: "pole_body",
    url: poleBodyUrl,
    mountRule: "pole-ground",
    // tools/create_japan_distribution_pole_primitive.py POLE settings.
    visibleLengthM: polePrimitive.visibleHeightM,
    radialReferenceM: poleRadiusAtDistanceFromTop(polePrimitive.visibleHeightM),
    radialTopM: poleRadiusAtDistanceFromTop(0.0),
    adapterVersion: 5
  },
  poleDecorationX: {
    modelKey: "pole_decoration_x",
    url: "primitive:pole_decoration_x",
    mountRule: "center",
    adapterVersion: 1
  },
  poleTransformer20kvaProxy: {
    modelKey: "pole_transformer_20kva_proxy",
    url: "primitive:pole_transformer_20kva_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  transformerIntermediateInsulatorProxy: {
    modelKey: "transformer_intermediate_insulator_proxy",
    url: "primitive:transformer_intermediate_insulator_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  transformerSupportBracketProxy: {
    modelKey: "transformer_support_bracket_proxy",
    url: "primitive:transformer_support_bracket_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  pc6CutoutProxy: {
    modelKey: "pc6_cutout_proxy",
    url: "primitive:pc6_cutout_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  tma13CutoutMountProxy: {
    modelKey: "tma13_cutout_mount_proxy",
    url: "primitive:tma13_cutout_mount_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  arresterGlB6gProxy: {
    modelKey: "arrester_gl_b6g_proxy",
    url: "primitive:arrester_gl_b6g_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  hvTriplexTermination60Proxy: {
    modelKey: "hv_triplex_termination_60_proxy",
    url: "primitive:hv_triplex_termination_60_proxy",
    mountRule: "center",
    adapterVersion: 1
  },
  aerialOpticalClosureRca3aoProxy: {
    modelKey: "aerial_optical_closure_rca3ao_proxy",
    url: "primitive:aerial_optical_closure_rca3ao_proxy",
    mountRule: "center",
    adapterVersion: 1
  }
};

const kindByModelKey = new Map<ModelKey, ModelAssetKind>(
  Object.entries(adapters).map(([kind, adapter]) => [adapter.modelKey, kind as ModelAssetKind])
);

function mountAnchor(bounds: THREE.Box3, size: THREE.Vector3, rule: MountRule): THREE.Vector3 {
  const center = bounds.getCenter(new THREE.Vector3());
  if (rule === "center") return center;
  if (rule === "bottom") return new THREE.Vector3(center.x, bounds.min.y, center.z);
  const buriedRatio =
    (polePrimitive.totalLengthM - polePrimitive.visibleHeightM) / polePrimitive.totalLengthM;
  return new THREE.Vector3(0, bounds.min.y + size.y * buriedRatio, 0);
}

function descriptorVersion(
  kind: ModelAssetKind,
  bounds: THREE.Box3,
  anchor: THREE.Vector3,
  radialReferenceM: number | null,
  radialTopM: number | null,
  adapter: ModelAssetAdapter
): number {
  let hash = 2166136261;
  const versionParts: Array<string | number> = [
    kind,
    adapter.adapterVersion,
    bounds.min.x,
    bounds.min.y,
    bounds.min.z,
    bounds.max.x,
    bounds.max.y,
    bounds.max.z,
    anchor.x,
    anchor.y,
    anchor.z
  ];
  if (radialReferenceM !== null) versionParts.push(radialReferenceM);
  if (radialTopM !== null) versionParts.push(radialTopM);
  if (adapter.visibleLengthM !== undefined) versionParts.push(adapter.visibleLengthM);
  const bytes = new TextEncoder().encode(versionParts.join(":"));
  for (const value of bytes) {
    hash ^= value;
    hash = Math.imul(hash, 16777619);
  }
  return hash >>> 0 || 1;
}

function material(color: number, metalness = 0.12, roughness = 0.72): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color, metalness, roughness });
}

function addMesh(
  group: THREE.Group,
  geometry: THREE.BufferGeometry,
  color: number,
  position: [number, number, number] = [0, 0, 0],
  rotation: [number, number, number] = [0, 0, 0],
  metalness = 0.12,
  roughness = 0.72
): THREE.Mesh {
  const mesh = new THREE.Mesh(geometry, material(color, metalness, roughness));
  mesh.position.set(...position);
  mesh.rotation.set(...rotation);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  group.add(mesh);
  return mesh;
}

function makePoleTransformer20kvaProxy(): THREE.Group {
  const group = new THREE.Group();
  const zinc = 0x8f9698;
  addMesh(group, new THREE.CylinderGeometry(0.200, 0.200, 0.550, 12), zinc, [0, 0, 0], [Math.PI / 2, 0, 0]);
  addMesh(group, new THREE.CylinderGeometry(0.215, 0.215, 0.030, 12), 0xa8adb0, [0, 0, 0.290], [Math.PI / 2, 0, 0], 0.22);
  addMesh(group, new THREE.CylinderGeometry(0.215, 0.215, 0.030, 12), 0x777f82, [0, 0, -0.290], [Math.PI / 2, 0, 0], 0.22);
  return group;
}

function makeTransformerIntermediateInsulatorProxy(): THREE.Group {
  const group = new THREE.Group();
  const porcelain = 0xd9d8cf;
  const metal = 0x858b8e;
  addMesh(group, new THREE.CylinderGeometry(0.045, 0.045, 0.220, 8), porcelain, [0, 0, 0], [0, 0, Math.PI / 2]);
  addMesh(group, new THREE.CylinderGeometry(0.052, 0.052, 0.020, 8), porcelain, [-0.045, 0, 0], [0, 0, Math.PI / 2]);
  addMesh(group, new THREE.CylinderGeometry(0.052, 0.052, 0.020, 8), porcelain, [0.045, 0, 0], [0, 0, Math.PI / 2]);
  addMesh(group, new THREE.CylinderGeometry(0.012, 0.012, 0.070, 8), metal, [-0.145, 0, 0], [0, 0, Math.PI / 2], 0.48);
  addMesh(group, new THREE.CylinderGeometry(0.012, 0.012, 0.070, 8), metal, [0.145, 0, 0], [0, 0, Math.PI / 2], 0.48);
  return group;
}

function makeTransformerSupportBracketProxy(): THREE.Group {
  const group = new THREE.Group();
  const zinc = 0x969d9f;
  addMesh(group, new THREE.BoxGeometry(0.700, 0.040, 0.045), zinc, [0, 0, 0], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.045, 0.260, 0.035), zinc, [0.245, -0.110, -0.020], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.045, 0.260, 0.035), zinc, [-0.245, -0.110, -0.020], [0, 0, 0], 0.35);
  return group;
}

function makePoleDecorationX(): THREE.Group {
  const group = new THREE.Group();
  const zinc = 0x969d9f;
  const porcelain = 0xd9d8cf;
  const tank = 0x8f9698;
  addMesh(group, new THREE.BoxGeometry(0.700, 0.040, 0.045), zinc, [-0.34, 0, -0.05], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.045, 0.260, 0.035), zinc, [-0.16, -0.110, -0.070], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.045, 0.260, 0.035), zinc, [-0.52, -0.110, -0.070], [0, 0, 0], 0.35);
  addMesh(group, new THREE.CylinderGeometry(0.200, 0.200, 0.550, 12), tank, [0.08, 0, 0], [Math.PI / 2, 0, 0]);
  addMesh(group, new THREE.CylinderGeometry(0.215, 0.215, 0.030, 12), 0xa8adb0, [0.08, 0, 0.290], [Math.PI / 2, 0, 0], 0.22);
  addMesh(group, new THREE.CylinderGeometry(0.215, 0.215, 0.030, 12), 0x777f82, [0.08, 0, -0.290], [Math.PI / 2, 0, 0], 0.22);
  for (const y of [-0.18, 0.18]) {
    addMesh(group, new THREE.CylinderGeometry(0.045, 0.045, 0.220, 8), porcelain, [-0.46, y, 0.38], [0, 0, Math.PI / 2]);
    addMesh(group, new THREE.CylinderGeometry(0.012, 0.012, 0.080, 8), zinc, [-0.60, y, 0.38], [0, 0, Math.PI / 2], 0.48);
    addMesh(group, new THREE.CylinderGeometry(0.012, 0.012, 0.080, 8), zinc, [-0.32, y, 0.38], [0, 0, Math.PI / 2], 0.48);
  }
  return group;
}

function makePc6CutoutProxy(): THREE.Group {
  const group = new THREE.Group();
  const porcelain = 0xd9d8cf;
  const metal = 0x8e9699;
  addMesh(group, new THREE.BoxGeometry(0.250, 0.090, 0.094, 2, 1, 1), porcelain);
  addMesh(group, new THREE.CylinderGeometry(0.014, 0.014, 0.060, 8), metal, [-0.167, 0, 0], [0, 0, Math.PI / 2], 0.45);
  addMesh(group, new THREE.CylinderGeometry(0.014, 0.014, 0.060, 8), metal, [0.167, 0, 0], [0, 0, Math.PI / 2], 0.45);
  addMesh(group, new THREE.BoxGeometry(0.032, 0.072, 0.030), metal, [-0.187, 0, 0], [0, 0, 0], 0.45);
  addMesh(group, new THREE.BoxGeometry(0.032, 0.072, 0.030), metal, [0.187, 0, 0], [0, 0, 0], 0.45);
  return group;
}

function makeTma13CutoutMountProxy(): THREE.Group {
  const group = new THREE.Group();
  const zinc = 0x9aa0a1;
  addMesh(group, new THREE.BoxGeometry(0.135, 0.032, 0.010), zinc, [0, 0, 0.052], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.010, 0.032, 0.115), zinc, [-0.048, 0, 0], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.010, 0.032, 0.115), zinc, [0.048, 0, 0], [0, 0, 0], 0.35);
  addMesh(group, new THREE.CylinderGeometry(0.005, 0.005, 0.150, 8), zinc, [0, 0, -0.010], [0, 0, Math.PI / 2], 0.45);
  return group;
}

function makeArresterGlB6gProxy(): THREE.Group {
  const group = new THREE.Group();
  const porcelain = 0xdeddd4;
  const metal = 0x858b8e;
  addMesh(group, new THREE.CylinderGeometry(0.036, 0.036, 0.190, 8), porcelain, [0, 0, 0], [Math.PI / 2, 0, 0]);
  addMesh(group, new THREE.CylinderGeometry(0.041, 0.041, 0.018, 8), porcelain, [0, 0, 0.052], [Math.PI / 2, 0, 0]);
  addMesh(group, new THREE.CylinderGeometry(0.041, 0.041, 0.018, 8), porcelain, [0, 0, -0.052], [Math.PI / 2, 0, 0]);
  addMesh(group, new THREE.CylinderGeometry(0.010, 0.010, 0.020, 8), metal, [0, 0, 0.115], [Math.PI / 2, 0, 0], 0.48);
  addMesh(group, new THREE.CylinderGeometry(0.010, 0.010, 0.020, 8), metal, [0, 0, -0.115], [Math.PI / 2, 0, 0], 0.48);
  return group;
}

function makeHvTriplexTermination60Proxy(): THREE.Group {
  const group = new THREE.Group();
  const porcelain = 0xd9d8cf;
  const metal = 0x878d90;
  for (const y of [-0.200, 0, 0.200]) {
    addMesh(group, new THREE.CylinderGeometry(0.0475, 0.034, 0.398, 8), porcelain, [-0.091, y, 0], [0, 0, Math.PI / 2]);
    addMesh(group, new THREE.CylinderGeometry(0.010, 0.010, 0.182, 8), metal, [0.199, y, 0], [0, 0, Math.PI / 2], 0.48);
    addMesh(group, new THREE.BoxGeometry(0.023, 0.023, 0.023), metal, [-0.250, y, 0], [0, 0, 0], 0.48);
  }
  addMesh(group, new THREE.BoxGeometry(0.040, 0.130, 0.035), metal, [-0.245, 0, -0.052], [0, 0, 0], 0.48);
  return group;
}

function makeAerialOpticalClosureRca3aoProxy(): THREE.Group {
  const group = new THREE.Group();
  const body = 0x3e464a;
  const strap = 0x8a9294;
  addMesh(group, new THREE.BoxGeometry(0.730, 0.150, 0.200, 4, 1, 1), body, [0, 0, 0], [0, 0, 0], 0.06, 0.82);
  addMesh(group, new THREE.BoxGeometry(0.018, 0.180, 0.220), strap, [-0.225, 0, 0], [0, 0, 0], 0.35);
  addMesh(group, new THREE.BoxGeometry(0.018, 0.180, 0.220), strap, [0.225, 0, 0], [0, 0, 0], 0.35);
  return group;
}

export class ModelAssetCache {
  private readonly promises = new Map<ModelAssetKind, Promise<LoadedModelAsset>>();
  private readonly loaded = new Map<ModelAssetKind, LoadedModelAsset>();
  private readonly loads = new Map<ModelAssetKind, number>();

  constructor(private readonly loadScene: SceneLoader = loadGltfScene) {
    this.registerPrimitiveSource("poleDecorationX", makePoleDecorationX());
    this.registerPrimitiveSource("poleTransformer20kvaProxy", makePoleTransformer20kvaProxy());
    this.registerPrimitiveSource("transformerIntermediateInsulatorProxy", makeTransformerIntermediateInsulatorProxy());
    this.registerPrimitiveSource("transformerSupportBracketProxy", makeTransformerSupportBracketProxy());
    this.registerPrimitiveSource("pc6CutoutProxy", makePc6CutoutProxy());
    this.registerPrimitiveSource("tma13CutoutMountProxy", makeTma13CutoutMountProxy());
    this.registerPrimitiveSource("arresterGlB6gProxy", makeArresterGlB6gProxy());
    this.registerPrimitiveSource("hvTriplexTermination60Proxy", makeHvTriplexTermination60Proxy());
    this.registerPrimitiveSource("aerialOpticalClosureRca3aoProxy", makeAerialOpticalClosureRca3aoProxy());
  }

  private registerPrimitiveSource(kind: ModelAssetKind, source: THREE.Group): void {
    const adapter = adapters[kind];
    source.updateMatrixWorld(true);
    const bounds = new THREE.Box3().setFromObject(source, true);
    const size = bounds.getSize(new THREE.Vector3());
    const anchor = mountAnchor(bounds, size, adapter.mountRule);
    source.position.sub(anchor);
    source.updateMatrixWorld(true);
    const asset: LoadedModelAsset = {
      kind,
      modelKey: adapter.modelKey,
      source,
      bounds,
      size,
      mountAnchor: anchor,
      radialReferenceM: null,
      radialTopM: null,
      descriptorVersion: descriptorVersion(kind, bounds, anchor, null, null, adapter),
      adapter
    };
    this.loaded.set(kind, asset);
  }

  load(kind: ModelAssetKind): Promise<LoadedModelAsset> {
    const loaded = this.loaded.get(kind);
    if (loaded !== undefined) return Promise.resolve(loaded);
    const cached = this.promises.get(kind);
    if (cached !== undefined) return cached;
    const adapter = adapters[kind];
    this.loads.set(kind, (this.loads.get(kind) ?? 0) + 1);
    const pending = this.loadScene(adapter.url).then((rawSource) => {
      rawSource.updateMatrixWorld(true);
      const bounds = new THREE.Box3().setFromObject(rawSource, true);
      if (bounds.isEmpty()) throw new Error(`GLB has no visible bounds: ${kind}`);
      const size = bounds.getSize(new THREE.Vector3());
      const anchor = mountAnchor(bounds, size, adapter.mountRule);
      const radialReferenceM = adapter.radialReferenceM ?? null;
      const radialTopM = adapter.radialTopM ?? null;

      // Local adapter normalization only: glTF Y-up becomes Core-local Z-up,
      // and the declared mount anchor becomes the assembly-local origin.
      rawSource.position.sub(anchor);
      const source = new THREE.Group();
      source.rotation.x = Math.PI / 2;
      source.add(rawSource);
      source.updateMatrixWorld(true);

      const asset: LoadedModelAsset = {
        kind,
        modelKey: adapter.modelKey,
        source,
        bounds,
        size,
        mountAnchor: anchor,
        radialReferenceM,
        radialTopM,
        descriptorVersion: descriptorVersion(
          kind, bounds, anchor, radialReferenceM, radialTopM, adapter
        ),
        adapter
      };
      this.loaded.set(kind, asset);
      return asset;
    });
    this.promises.set(kind, pending);
    return pending;
  }

  loadModel(modelKey: string): Promise<LoadedModelAsset> {
    const kind = kindByModelKey.get(modelKey as ModelKey);
    return kind === undefined
      ? Promise.reject(new Error(`Unknown model key: ${modelKey}`))
      : this.load(kind);
  }

  loadedModel(modelKey: string): LoadedModelAsset | null {
    const kind = kindByModelKey.get(modelKey as ModelKey);
    return kind === undefined ? null : this.loaded.get(kind) ?? null;
  }

  loadCount(kind: ModelAssetKind): number {
    return this.loads.get(kind) ?? 0;
  }
}

export function cloneSharedAsset(asset: LoadedModelAsset): THREE.Group {
  return asset.source.clone(true);
}

const identityTransform = (): ModelTransformInput => ({
  positionX: 0,
  positionY: 0,
  positionZ: 0,
  rotationX: 0,
  rotationY: 0,
  rotationZ: 0,
  scaleX: 1,
  scaleY: 1,
  scaleZ: 1
});

function part(
  asset: LoadedModelAsset,
  partId: number,
  fitMode: number,
  localTransform = identityTransform(),
  wireSocket: ModelSocketInput | ModelSocketInput[] | null = null
): ModelAssemblyPartInput {
  return {
    partId,
    modelKey: asset.modelKey,
    descriptorName: asset.kind,
    descriptorVersion: asset.descriptorVersion,
    fitMode,
    localTransform,
    sockets: wireSocket === null ? [] : Array.isArray(wireSocket) ? wireSocket : [wireSocket]
  };
}

function insertionFixture(asset: LoadedModelAsset): {
  transform: ModelTransformInput;
  wireSocket: ModelSocketInput;
} {
  // Blender +Y is the insertion axis. After glTF Y-up normalization its
  // measured length is assembly-local Y.
  const length = asset.size.z;
  if (!Number.isFinite(length) || length <= 0) {
    throw new Error("Insertion fixture adapter requires a positive length");
  }
  const transform = identityTransform();
  transform.positionY = length * 0.5;
  transform.rotationZ = 180;
  const socketInsetRatio = asset.kind === "communicationClampLong" ? 0.5 : 0;
  const socketLocalY = -length * (0.5 - socketInsetRatio);
  const socketLocalZ = 0;
  return {
    transform,
    wireSocket: {
      name: "wire",
      positionX: 0,
      positionY: socketLocalY,
      positionZ: socketLocalZ,
      directionX: 0,
      directionY: -1,
      directionZ: 0
    }
  };
}

export function buildDefaultModelBootstrap(
  pole: LoadedModelAsset,
  crossarm: LoadedModelAsset,
  belt: LoadedModelAsset,
  insulator: LoadedModelAsset,
  communicationClamp: LoadedModelAsset,
  communicationClampLong: LoadedModelAsset,
  poleDecoration: LoadedModelAsset
): ModelAssemblyBootstrapInput {
  const poleVisibleLength = pole.adapter.visibleLengthM;
  if (poleVisibleLength === undefined || !Number.isFinite(poleVisibleLength) || poleVisibleLength <= 0) {
    throw new Error("Pole adapter requires a positive configured visible length");
  }
  const poleGroundRadius = pole.radialReferenceM;
  const poleTopRadius = pole.radialTopM;
  if (poleGroundRadius === null || poleTopRadius === null ||
      !Number.isFinite(poleGroundRadius) || !Number.isFinite(poleTopRadius) ||
      poleGroundRadius <= 0 || poleTopRadius <= 0 || poleTopRadius > poleGroundRadius) {
    throw new Error("Pole adapter requires a valid measured radial profile");
  }
  const distributionPoleTransform = identityTransform();
  distributionPoleTransform.scaleZ = 10.0 / poleVisibleLength;
  const communicationPoleTransform = identityTransform();
  communicationPoleTransform.scaleZ = 11.35 / poleVisibleLength;
  const beltInnerRadius = belt.radialReferenceM;
  if (beltInnerRadius === null || !Number.isFinite(beltInnerRadius) || beltInnerRadius <= 0) {
    throw new Error("Belt adapter requires a positive measured inner radius");
  }
  const beltTransform = identityTransform();
  // Normalize the authored belt cross-section before Core applies the
  // height-dependent pole radius. This keeps belts on tapered poles instead of
  // preserving the source model's lower-pole radius.
  beltTransform.scaleX = 1 / beltInnerRadius;
  beltTransform.scaleY = 1 / beltInnerRadius;
  const crossarmTransform = identityTransform();
  crossarmTransform.rotationZ = 90;
  const endpointMountSocket: ModelSocketInput = {
    name: "endpoint_mount",
    positionX: 0,
    positionY: 0,
    positionZ: crossarm.size.y * 0.5,
    directionX: 0,
    directionY: 0,
    directionZ: 1
  };

  const hvWireSocket: ModelSocketInput = {
    name: "wire",
    positionX: 0,
    positionY: 0,
    positionZ: insulator.size.y,
    directionX: 1,
    directionY: 0,
    directionZ: 0
  };

  const communicationInsertion = insertionFixture(communicationClamp);
  const lowVoltageInsertion = insertionFixture(communicationClampLong);
  const decorationTransform = identityTransform();
  decorationTransform.positionX = 0.62;
  decorationTransform.positionY = 0.42;
  decorationTransform.positionZ = 7.70;
  const decorationSockets: ModelSocketInput[] = [
    {
      name: "connect_hv_0",
      positionX: -0.60, positionY: -0.18, positionZ: 0.38,
      directionX: -1, directionY: 0, directionZ: 0
    },
    {
      name: "connect_hv_1",
      positionX: -0.60, positionY: 0.18, positionZ: 0.38,
      directionX: -1, directionY: 0, directionZ: 0
    },
    {
      name: "connect_lv_0",
      positionX: 0.16, positionY: -0.16, positionZ: -0.30,
      directionX: 0, directionY: -1, directionZ: -0.2
    },
    {
      name: "connect_lv_1",
      positionX: 0.20, positionY: 0, positionZ: -0.32,
      directionX: 0, directionY: -1, directionZ: -0.2
    },
    {
      name: "connect_lv_2",
      positionX: 0.16, positionY: 0.16, positionZ: -0.30,
      directionX: 0, directionY: -1, directionZ: -0.2
    }
  ];

  const poleAssemblyId = 9201;
  const hvRowAssemblyId = 9202;
  const hvEndpointAssemblyId = 9203;
  const communicationEndpointAssemblyId = 9204;
  const communicationPoleAssemblyId = 9205;
  const lowVoltageEndpointAssemblyId = 9206;
  const beltRowAssemblyId = 9207;
  const poleDecorationAssemblyId = 9208;
  return {
    assemblies: [
      {
        id: poleAssemblyId,
        version: 4,
        parts: [part(pole, 1, 1, distributionPoleTransform)],
        wireSocket: null
      },
      {
        id: hvRowAssemblyId,
        version: 6,
        parts: [
          part(crossarm, 1, 3, crossarmTransform, endpointMountSocket),
          part(belt, 2, 2, beltTransform)
        ],
        wireSocket: null,
        endpointMountSocket: { partId: 1, socketName: "endpoint_mount" }
      },
      {
        id: hvEndpointAssemblyId,
        version: 1,
        parts: [part(insulator, 1, 0, identityTransform(), hvWireSocket)],
        wireSocket: { partId: 1, socketName: "wire" }
      },
      {
        id: communicationEndpointAssemblyId,
        version: 3,
        parts: [
          part(
            communicationClamp, 1, 3,
            communicationInsertion.transform, communicationInsertion.wireSocket
          )
        ],
        wireSocket: { partId: 1, socketName: "wire" }
      },
      {
        id: communicationPoleAssemblyId,
        version: 4,
        parts: [part(pole, 1, 1, communicationPoleTransform)],
        wireSocket: null
      },
      {
        id: lowVoltageEndpointAssemblyId,
        version: 4,
        parts: [
          part(
            communicationClampLong, 1, 0,
            lowVoltageInsertion.transform, lowVoltageInsertion.wireSocket
          )
        ],
        wireSocket: { partId: 1, socketName: "wire" }
      },
      {
        id: beltRowAssemblyId,
        version: 4,
        parts: [part(belt, 1, 2, beltTransform)],
        wireSocket: null
      },
      {
        id: poleDecorationAssemblyId,
        version: 1,
        parts: [part(poleDecoration, 1, 0, decorationTransform, decorationSockets)],
        wireSocket: null
      }
    ],
    poleAssignments: [
      {
        poleTypeId: 1, assemblyId: poleAssemblyId,
        radiusBaseM: poleGroundRadius, radiusTopM: poleTopRadius
      },
      {
        poleTypeId: 2, assemblyId: communicationPoleAssemblyId,
        radiusBaseM: poleGroundRadius, radiusTopM: poleTopRadius
      }
    ],
    bundleAssignments: [
      {
        bundleTemplateId: 101,
        rowAssemblyId: hvRowAssemblyId,
        endpointAssemblyId: hvEndpointAssemblyId
      },
      {
        bundleTemplateId: 102,
        rowAssemblyId: beltRowAssemblyId,
        endpointAssemblyId: lowVoltageEndpointAssemblyId
      },
      {
        bundleTemplateId: 104,
        rowAssemblyId: beltRowAssemblyId,
        endpointAssemblyId: communicationEndpointAssemblyId
      },
      {
        bundleTemplateId: 105,
        rowAssemblyId: beltRowAssemblyId,
        endpointAssemblyId: communicationEndpointAssemblyId
      }
    ]
  };
}

export async function loadDefaultModelBootstrap(): Promise<ModelAssemblyBootstrapInput> {
  const [pole, crossarm, belt, insulator, communicationClamp, communicationClampLong, poleDecoration] =
    await Promise.all([
      modelAssetCache.load("poleBody"),
      modelAssetCache.load("crossarmHv"),
      modelAssetCache.load("belt"),
      modelAssetCache.load("hvInsulator"),
      modelAssetCache.load("communicationClamp"),
      modelAssetCache.load("communicationClampLong"),
      modelAssetCache.load("poleDecorationX")
    ]);
  return buildDefaultModelBootstrap(
    pole, crossarm, belt, insulator, communicationClamp, communicationClampLong, poleDecoration
  );
}

async function loadGltfScene(url: string): Promise<THREE.Group> {
  const gltf = await new GLTFLoader().loadAsync(url);
  return gltf.scene;
}

export const modelAssetCache = new ModelAssetCache();
