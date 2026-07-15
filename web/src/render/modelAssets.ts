import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

import type {
  ModelAssemblyBootstrapInput,
  ModelAssemblyPartInput,
  ModelSocketInput,
  ModelTransformInput
} from "../model";

export type ModelAssetKind =
  | "belt"
  | "communicationClamp"
  | "communicationClampLong"
  | "crossarmHv"
  | "hvInsulator"
  | "poleBody";

export type ModelKey =
  | "pole_belt"
  | "communication_clamp"
  | "communication_clamp_long"
  | "hv_crossarm"
  | "hv_insulator"
  | "pole_body";

type MountRule = "center" | "bottom" | "pole-ground";

export interface ModelAssetAdapter {
  modelKey: ModelKey;
  url: string;
  mountRule: MountRule;
  radialReferenceM?: number;
}

export interface LoadedModelAsset {
  kind: ModelAssetKind;
  modelKey: ModelKey;
  source: THREE.Group;
  bounds: THREE.Box3;
  size: THREE.Vector3;
  mountAnchor: THREE.Vector3;
  descriptorVersion: number;
  adapter: ModelAssetAdapter;
}

type SceneLoader = (url: string) => Promise<THREE.Group>;

const adapters: Record<ModelAssetKind, ModelAssetAdapter> = {
  belt: {
    modelKey: "pole_belt",
    url: new URL("../assets/belt.glb", import.meta.url).href,
    mountRule: "center",
    // The source belt was authored against the lower pole radius. The adapter
    // normalizes that local radius before Core applies kPoleRadial.
    radialReferenceM: 0.20
  },
  communicationClamp: {
    modelKey: "communication_clamp",
    url: new URL("../assets/communication_clamp_1.glb", import.meta.url).href,
    mountRule: "center"
  },
  communicationClampLong: {
    modelKey: "communication_clamp_long",
    url: new URL("../assets/communication_clamp_long_1.glb", import.meta.url).href,
    mountRule: "center"
  },
  crossarmHv: {
    modelKey: "hv_crossarm",
    url: new URL("../assets/crossarm_hv_1p7m.glb", import.meta.url).href,
    mountRule: "center"
  },
  hvInsulator: {
    modelKey: "hv_insulator",
    url: new URL("../assets/hv_phase_1_disc_3.glb", import.meta.url).href,
    mountRule: "bottom"
  },
  poleBody: {
    modelKey: "pole_body",
    url: new URL("../assets/pole_body_tapered_12m_visible10m.glb", import.meta.url).href,
    mountRule: "pole-ground"
  }
};

const kindByModelKey = new Map<ModelKey, ModelAssetKind>(
  Object.entries(adapters).map(([kind, adapter]) => [adapter.modelKey, kind as ModelAssetKind])
);

function mountAnchor(bounds: THREE.Box3, size: THREE.Vector3, rule: MountRule): THREE.Vector3 {
  const center = bounds.getCenter(new THREE.Vector3());
  if (rule === "center") return center;
  if (rule === "bottom") return new THREE.Vector3(center.x, bounds.min.y, center.z);
  return new THREE.Vector3(center.x, bounds.min.y + size.y * (2 / 12), center.z);
}

function descriptorVersion(kind: ModelAssetKind, bounds: THREE.Box3): number {
  let hash = 2166136261;
  const bytes = new TextEncoder().encode([
    kind,
    bounds.min.x,
    bounds.min.y,
    bounds.min.z,
    bounds.max.x,
    bounds.max.y,
    bounds.max.z
  ].join(":"));
  for (const value of bytes) {
    hash ^= value;
    hash = Math.imul(hash, 16777619);
  }
  return hash >>> 0 || 1;
}

export class ModelAssetCache {
  private readonly promises = new Map<ModelAssetKind, Promise<LoadedModelAsset>>();
  private readonly loaded = new Map<ModelAssetKind, LoadedModelAsset>();
  private readonly loads = new Map<ModelAssetKind, number>();

  constructor(private readonly loadScene: SceneLoader = loadGltfScene) {}

  load(kind: ModelAssetKind): Promise<LoadedModelAsset> {
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
        descriptorVersion: descriptorVersion(kind, bounds),
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
  wireSocket: ModelSocketInput | null = null
): ModelAssemblyPartInput {
  return {
    partId,
    modelKey: asset.modelKey,
    descriptorName: asset.kind,
    descriptorVersion: asset.descriptorVersion,
    fitMode,
    localTransform,
    sockets: wireSocket === null ? [] : [wireSocket]
  };
}

export function buildDefaultModelBootstrap(
  pole: LoadedModelAsset,
  crossarm: LoadedModelAsset,
  belt: LoadedModelAsset,
  insulator: LoadedModelAsset,
  communicationClamp: LoadedModelAsset
): ModelAssemblyBootstrapInput {
  const poleVisibleLength = pole.bounds.max.y - pole.mountAnchor.y;
  if (!Number.isFinite(poleVisibleLength) || poleVisibleLength <= 0) {
    throw new Error("Pole adapter requires a positive visible length");
  }
  const distributionPoleTransform = identityTransform();
  distributionPoleTransform.scaleZ = 10.0 / poleVisibleLength;
  const communicationPoleTransform = identityTransform();
  communicationPoleTransform.scaleZ = 11.35 / poleVisibleLength;
  const beltReference = belt.adapter.radialReferenceM;
  if (beltReference === undefined || beltReference <= 0) {
    throw new Error("Belt adapter requires a positive radial reference");
  }
  const beltTransform = identityTransform();
  beltTransform.scaleX = 1 / beltReference;
  beltTransform.scaleY = 1 / beltReference;
  const crossarmTransform = identityTransform();
  // The default 10 m distribution pole is 0.129 m in radius at the 9.2 m HV
  // row. Offset the 0.08 m-deep arm so its pole-facing surface meets the pole.
  crossarmTransform.positionX = 0.169;
  crossarmTransform.rotationZ = 90;
  const lowVoltageArmTransform = identityTransform();
  // At the default 7.4 m LV row the pole radius is 0.147 m; the long clamp is
  // 0.047 m deep. Its authored long axis becomes assembly-local Y.
  lowVoltageArmTransform.positionX = 0.170;
  const hvWireSocket: ModelSocketInput = {
    name: "wire",
    positionX: 0,
    positionY: 0,
    positionZ: insulator.size.y,
    directionX: 1,
    directionY: 0,
    directionZ: 0
  };

  // Blender +Y is the clamp insertion axis. After glTF Y-up normalization it
  // is assembly-local +Y. Rotate that end toward the pole and place the wire
  // socket at the opposite end, using the measured model length.
  const clampLength = communicationClamp.size.z;
  if (!Number.isFinite(clampLength) || clampLength <= 0) {
    throw new Error("Communication clamp adapter requires a positive insertion length");
  }
  const clampTransform = identityTransform();
  clampTransform.positionY = clampLength * 0.5;
  clampTransform.rotationZ = 180;
  const clampWireSocket: ModelSocketInput = {
    name: "wire",
    positionX: 0,
    positionY: -clampLength * 0.5,
    positionZ: 0,
    directionX: 0,
    directionY: -1,
    directionZ: 0
  };

  const poleAssemblyId = 9201;
  const hvRowAssemblyId = 9202;
  const hvEndpointAssemblyId = 9203;
  const communicationEndpointAssemblyId = 9204;
  const communicationPoleAssemblyId = 9205;
  const lowVoltageRowAssemblyId = 9206;
  return {
    assemblies: [
      {
        id: poleAssemblyId,
        version: 1,
        parts: [part(pole, 1, 1, distributionPoleTransform)],
        wireSocket: null
      },
      {
        id: hvRowAssemblyId,
        version: 1,
        parts: [
          part(crossarm, 1, 0, crossarmTransform),
          part(belt, 2, 2, beltTransform)
        ],
        wireSocket: null
      },
      {
        id: hvEndpointAssemblyId,
        version: 1,
        parts: [part(insulator, 1, 0, identityTransform(), hvWireSocket)],
        wireSocket: { partId: 1, socketName: "wire" }
      },
      {
        id: communicationEndpointAssemblyId,
        version: 1,
        parts: [part(communicationClamp, 1, 0, clampTransform, clampWireSocket)],
        wireSocket: { partId: 1, socketName: "wire" }
      },
      {
        id: communicationPoleAssemblyId,
        version: 1,
        parts: [part(pole, 1, 1, communicationPoleTransform)],
        wireSocket: null
      },
      {
        id: lowVoltageRowAssemblyId,
        version: 1,
        parts: [
          part(communicationClamp, 1, 0, lowVoltageArmTransform),
          part(belt, 2, 2, beltTransform)
        ],
        wireSocket: null
      }
    ],
    poleAssignments: [
      { poleTypeId: 1, assemblyId: poleAssemblyId },
      { poleTypeId: 2, assemblyId: communicationPoleAssemblyId }
    ],
    bundleAssignments: [
      {
        bundleTemplateId: 101,
        rowAssemblyId: hvRowAssemblyId,
        endpointAssemblyId: hvEndpointAssemblyId
      },
      {
        bundleTemplateId: 102,
        rowAssemblyId: lowVoltageRowAssemblyId,
        endpointAssemblyId: 0
      },
      {
        bundleTemplateId: 104,
        rowAssemblyId: 0,
        endpointAssemblyId: communicationEndpointAssemblyId
      }
    ]
  };
}

export async function loadDefaultModelBootstrap(): Promise<ModelAssemblyBootstrapInput> {
  const [pole, crossarm, belt, insulator, communicationClamp] = await Promise.all([
    modelAssetCache.load("poleBody"),
    modelAssetCache.load("crossarmHv"),
    modelAssetCache.load("belt"),
    modelAssetCache.load("hvInsulator"),
    modelAssetCache.load("communicationClampLong")
  ]);
  return buildDefaultModelBootstrap(pole, crossarm, belt, insulator, communicationClamp);
}

async function loadGltfScene(url: string): Promise<THREE.Group> {
  const gltf = await new GLTFLoader().loadAsync(url);
  return gltf.scene;
}

export const modelAssetCache = new ModelAssetCache();
