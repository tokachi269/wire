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

const adapters: Record<ModelAssetKind, ModelAssetAdapter> = {
  belt: {
    modelKey: "pole_belt",
    url: beltUrl,
    mountRule: "center",
    adapterVersion: 3
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
    adapterVersion: 4
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

function descriptorVersion(
  kind: ModelAssetKind,
  bounds: THREE.Box3,
  anchor: THREE.Vector3,
  radialReferenceM: number | null,
  radialTopM: number | null,
  adapterVersion: number
): number {
  let hash = 2166136261;
  const versionParts: Array<string | number> = [
    kind,
    adapterVersion,
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
  const bytes = new TextEncoder().encode(versionParts.join(":"));
  for (const value of bytes) {
    hash ^= value;
    hash = Math.imul(hash, 16777619);
  }
  return hash >>> 0 || 1;
}

function measuredInnerRadius(source: THREE.Group): number | null {
  let radius = Number.POSITIVE_INFINITY;
  source.traverse((object) => {
    const mesh = object as THREE.Mesh;
    const geometry = mesh.geometry as THREE.BufferGeometry | undefined;
    const position = geometry?.getAttribute("position");
    if (position === undefined) return;
    for (let i = 0; i < position.count; i += 1) {
      const point = new THREE.Vector3(position.getX(i), position.getY(i), position.getZ(i));
      point.applyMatrix4(mesh.matrixWorld);
      const r = Math.hypot(point.x, point.z);
      if (Number.isFinite(r) && r > 1e-6) radius = Math.min(radius, r);
    }
  });
  return Number.isFinite(radius) ? radius : null;
}

function measuredOuterRadiusAtHeight(source: THREE.Group, height: number): number | null {
  let nearest = Number.POSITIVE_INFINITY;
  let outerRadius = 0;
  source.traverse((object) => {
    const mesh = object as THREE.Mesh;
    const geometry = mesh.geometry as THREE.BufferGeometry | undefined;
    const position = geometry?.getAttribute("position");
    if (position === undefined) return;
    for (let i = 0; i < position.count; i += 1) {
      const point = new THREE.Vector3(position.getX(i), position.getY(i), position.getZ(i));
      point.applyMatrix4(mesh.matrixWorld);
      const radius = Math.hypot(point.x, point.z);
      if (!Number.isFinite(radius)) continue;
      const distance = Math.abs(point.y - height);
      if (distance < nearest - 1e-6) {
        nearest = distance;
        outerRadius = radius;
      } else if (Math.abs(distance - nearest) <= 1e-6) {
        outerRadius = Math.max(outerRadius, radius);
      }
    }
  });
  return Number.isFinite(nearest) && outerRadius > 1e-6 ? outerRadius : null;
}

function measuredPoleRadiusProfile(
  source: THREE.Group,
  groundHeight: number,
  visibleLength: number
): { base: number; top: number } | null {
  const base = measuredOuterRadiusAtHeight(source, groundHeight);
  const referenceDistance = visibleLength * 0.6;
  const reference = measuredOuterRadiusAtHeight(source, groundHeight + referenceDistance);
  if (base === null || reference === null || referenceDistance <= 1e-6) return null;
  const top = base + (reference - base) * (visibleLength / referenceDistance);
  return Number.isFinite(top) && top > 1e-6 && top <= base ? { base, top } : null;
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
      const poleRadiusProfile = kind === "poleBody"
        ? measuredPoleRadiusProfile(rawSource, anchor.y, bounds.max.y - anchor.y)
        : null;
      const radialReferenceM = kind === "belt"
        ? measuredInnerRadius(rawSource)
        : poleRadiusProfile?.base ?? null;
      const radialTopM = poleRadiusProfile?.top ?? null;

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
          kind, bounds, anchor, radialReferenceM, radialTopM, adapter.adapterVersion
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
  const socketLocalZ = asset.kind === "communicationClampLong" ? asset.size.y * 0.5 : 0;
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
  communicationClampLong: LoadedModelAsset
): ModelAssemblyBootstrapInput {
  const poleVisibleLength = pole.bounds.max.y - pole.mountAnchor.y;
  if (!Number.isFinite(poleVisibleLength) || poleVisibleLength <= 0) {
    throw new Error("Pole adapter requires a positive visible length");
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

  const poleAssemblyId = 9201;
  const hvRowAssemblyId = 9202;
  const hvEndpointAssemblyId = 9203;
  const communicationEndpointAssemblyId = 9204;
  const communicationPoleAssemblyId = 9205;
  const lowVoltageEndpointAssemblyId = 9206;
  const beltRowAssemblyId = 9207;
  return {
    assemblies: [
      {
        id: poleAssemblyId,
        version: 3,
        parts: [part(pole, 1, 1, distributionPoleTransform)],
        wireSocket: null
      },
      {
        id: hvRowAssemblyId,
        version: 4,
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
        version: 3,
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
        version: 2,
        parts: [part(belt, 1, 2, beltTransform)],
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
  const [pole, crossarm, belt, insulator, communicationClamp, communicationClampLong] =
    await Promise.all([
      modelAssetCache.load("poleBody"),
      modelAssetCache.load("crossarmHv"),
      modelAssetCache.load("belt"),
      modelAssetCache.load("hvInsulator"),
      modelAssetCache.load("communicationClamp"),
      modelAssetCache.load("communicationClampLong")
    ]);
  return buildDefaultModelBootstrap(
    pole, crossarm, belt, insulator, communicationClamp, communicationClampLong
  );
}

async function loadGltfScene(url: string): Promise<THREE.Group> {
  const gltf = await new GLTFLoader().loadAsync(url);
  return gltf.scene;
}

export const modelAssetCache = new ModelAssetCache();
