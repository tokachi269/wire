import { describe, expect, it, vi } from "vitest";
import * as THREE from "three";
import {
  cloneSharedAsset,
  ModelAssetCache,
  poleGroundAnchor,
  poleVisibleLength
} from "../src/render/modelAssets";

describe("model asset cache", () => {
  it("loads one source and shares geometry and material across clones", async () => {
    const geometry = new THREE.BoxGeometry(1, 12, 1);
    geometry.translate(0, 4, 0);
    const material = new THREE.MeshStandardMaterial();
    const source = new THREE.Group();
    source.add(new THREE.Mesh(geometry, material));
    const load = vi.fn(async () => source);
    const cache = new ModelAssetCache(load);

    const [first, second] = await Promise.all([cache.load("poleBody"), cache.load("poleBody")]);
    expect(first).toBe(second);
    expect(load).toHaveBeenCalledOnce();
    expect(cache.loadCount("poleBody")).toBe(1);

    const cloneA = cloneSharedAsset(first);
    const cloneB = cloneSharedAsset(first);
    const meshA = cloneA.children[0] as THREE.Mesh;
    const meshB = cloneB.children[0] as THREE.Mesh;
    expect(meshA.geometry).toBe(meshB.geometry);
    expect(meshA.material).toBe(meshB.material);
    expect(poleGroundAnchor(first).y).toBeCloseTo(0, 12);
    expect(poleVisibleLength(first)).toBeCloseTo(10, 12);
  });
});
