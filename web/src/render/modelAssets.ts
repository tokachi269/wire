import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

export type ModelAssetKind =
  | "belt"
  | "communicationClamp"
  | "communicationClampLong"
  | "crossarmHv"
  | "hvInsulator"
  | "poleBody";

export type ModelAxis = "x" | "y" | "z" | "-x" | "-y" | "-z";

export interface ModelAssetAdapter {
  url: string;
  mainAxis: ModelAxis;
  upAxis: ModelAxis;
  mountSide: ModelAxis | "center" | "radial-inner";
}

export interface LoadedModelAsset {
  kind: ModelAssetKind;
  source: THREE.Group;
  bounds: THREE.Box3;
  size: THREE.Vector3;
  adapter: ModelAssetAdapter;
}

type SceneLoader = (url: string) => Promise<THREE.Group>;

const adapters: Record<ModelAssetKind, ModelAssetAdapter> = {
  belt: {
    url: new URL("../assets/belt.glb", import.meta.url).href,
    mainAxis: "y",
    upAxis: "y",
    mountSide: "radial-inner"
  },
  communicationClamp: {
    url: new URL("../assets/communication_clamp_1.glb", import.meta.url).href,
    mainAxis: "z",
    upAxis: "y",
    mountSide: "-z"
  },
  communicationClampLong: {
    url: new URL("../assets/communication_clamp_long_1.glb", import.meta.url).href,
    mainAxis: "z",
    upAxis: "y",
    // Blender +Y is glTF -Z under the unchanged standard export.
    // The user-defined insertion direction therefore points toward -Z.
    mountSide: "-z"
  },
  crossarmHv: {
    url: new URL("../assets/crossarm_hv_1p7m.glb", import.meta.url).href,
    mainAxis: "x",
    upAxis: "y",
    mountSide: "center"
  },
  hvInsulator: {
    url: new URL("../assets/hv_phase_1_disc_3.glb", import.meta.url).href,
    mainAxis: "y",
    upAxis: "y",
    mountSide: "-y"
  },
  poleBody: {
    url: new URL("../assets/pole_body_tapered_12m_visible10m.glb", import.meta.url).href,
    mainAxis: "y",
    upAxis: "y",
    mountSide: "-y"
  }
};

export class ModelAssetCache {
  private readonly promises = new Map<ModelAssetKind, Promise<LoadedModelAsset>>();
  private readonly loads = new Map<ModelAssetKind, number>();

  constructor(private readonly loadScene: SceneLoader = loadGltfScene) {}

  load(kind: ModelAssetKind): Promise<LoadedModelAsset> {
    const cached = this.promises.get(kind);
    if (cached !== undefined) return cached;
    const adapter = adapters[kind];
    this.loads.set(kind, (this.loads.get(kind) ?? 0) + 1);
    const pending = this.loadScene(adapter.url).then((source) => {
      source.updateMatrixWorld(true);
      const bounds = new THREE.Box3().setFromObject(source, true);
      if (bounds.isEmpty()) throw new Error(`GLB has no visible bounds: ${kind}`);
      return {
        kind,
        source,
        bounds,
        size: bounds.getSize(new THREE.Vector3()),
        adapter
      };
    });
    this.promises.set(kind, pending);
    return pending;
  }

  loadCount(kind: ModelAssetKind): number {
    return this.loads.get(kind) ?? 0;
  }
}

export function cloneSharedAsset(asset: LoadedModelAsset): THREE.Group {
  return asset.source.clone(true);
}

export function poleGroundAnchor(asset: LoadedModelAsset): THREE.Vector3 {
  const center = asset.bounds.getCenter(new THREE.Vector3());
  const groundY = asset.bounds.min.y + asset.size.y * (2 / 12);
  return new THREE.Vector3(center.x, groundY, center.z);
}

export function poleVisibleLength(asset: LoadedModelAsset): number {
  return asset.size.y * (10 / 12);
}

async function loadGltfScene(url: string): Promise<THREE.Group> {
  const gltf = await new GLTFLoader().loadAsync(url);
  return gltf.scene;
}

export const modelAssetCache = new ModelAssetCache();
