import { describe, expect, it, vi } from "vitest";
import * as THREE from "three";
import {
  buildDefaultModelBootstrap,
  cloneSharedAsset,
  ModelAssetCache
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
    let meshA: THREE.Mesh | null = null;
    let meshB: THREE.Mesh | null = null;
    cloneA.traverse((object) => { if (object instanceof THREE.Mesh) meshA = object; });
    cloneB.traverse((object) => { if (object instanceof THREE.Mesh) meshB = object; });
    expect(meshA).not.toBeNull();
    expect(meshB).not.toBeNull();
    expect(meshA!.geometry).toBe(meshB!.geometry);
    expect(meshA!.material).toBe(meshB!.material);
    expect(first.mountAnchor.y).toBeCloseTo(0, 12);
    expect(first.size.y * (10 / 12)).toBeCloseTo(10, 12);
  });

  it("builds HV, LV, and communication assemblies from measured local assets", async () => {
    const load = vi.fn(async () => {
      const geometry = new THREE.BoxGeometry(0.1, 0.2, 0.4);
      const source = new THREE.Group();
      source.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial()));
      return source;
    });
    const cache = new ModelAssetCache(load);
    const [pole, crossarm, belt, insulator, clamp, clampLong] = await Promise.all([
      cache.load("poleBody"),
      cache.load("crossarmHv"),
      cache.load("belt"),
      cache.load("hvInsulator"),
      cache.load("communicationClamp"),
      cache.load("communicationClampLong")
    ]);
    expect(cache.loadCount("poleBody")).toBe(1);
    expect(cache.loadCount("crossarmHv")).toBe(1);
    expect(cache.loadCount("belt")).toBe(1);
    expect(cache.loadCount("hvInsulator")).toBe(1);
    expect(cache.loadCount("communicationClampLong")).toBe(1);
    expect(cache.loadCount("communicationClamp")).toBe(1);
    const bootstrap = buildDefaultModelBootstrap(
      pole, crossarm, belt, insulator, clamp, clampLong
    );

    expect(bootstrap.poleAssignments).toEqual([
      {
        poleTypeId: 1, assemblyId: 9201,
        radiusBaseM: pole.radialReferenceM, radiusTopM: pole.radialTopM
      },
      {
        poleTypeId: 2, assemblyId: 9205,
        radiusBaseM: pole.radialReferenceM, radiusTopM: pole.radialTopM
      }
    ]);
    const distributionPole = bootstrap.assemblies.find((assembly) => assembly.id === 9201)!;
    const communicationPole = bootstrap.assemblies.find((assembly) => assembly.id === 9205)!;
    expect(distributionPole.parts[0].localTransform.scaleZ).toBeCloseTo(
      10 / (pole.bounds.max.y - pole.mountAnchor.y), 12
    );
    expect(communicationPole.parts[0].localTransform.scaleZ).toBeCloseTo(
      11.35 / (pole.bounds.max.y - pole.mountAnchor.y), 12
    );
    expect(bootstrap.bundleAssignments).toContainEqual({
      bundleTemplateId: 102,
      rowAssemblyId: 9207,
      endpointAssemblyId: 9206
    });
    expect(bootstrap.bundleAssignments).toContainEqual({
      bundleTemplateId: 104,
      rowAssemblyId: 9207,
      endpointAssemblyId: 9204
    });
    expect(bootstrap.bundleAssignments).toContainEqual({
      bundleTemplateId: 105,
      rowAssemblyId: 9207,
      endpointAssemblyId: 9204
    });
    const communication = bootstrap.assemblies.find((assembly) => assembly.id === 9204)!;
    const clampPart = communication.parts[0];
    expect(clampPart.modelKey).toBe("communication_clamp");
    expect(communication.parts.map((part) => part.modelKey)).toEqual(["communication_clamp"]);
    const beltRow = bootstrap.assemblies.find((assembly) => assembly.id === 9207)!;
    const beltPart = beltRow.parts[0];
    expect(pole.radialReferenceM).not.toBeNull();
    expect(pole.radialTopM).not.toBeNull();
    expect(pole.radialTopM!).toBeLessThanOrEqual(pole.radialReferenceM!);
    expect(distributionPole.parts[0].localTransform.scaleX).toBe(1);
    expect(distributionPole.parts[0].localTransform.scaleY).toBe(1);
    expect(belt.radialReferenceM).not.toBeNull();
    expect(beltPart.localTransform.scaleX).toBeCloseTo(1 / belt.radialReferenceM!, 12);
    expect(beltPart.localTransform.scaleY).toBeCloseTo(1 / belt.radialReferenceM!, 12);
    expect(beltPart.localTransform.scaleZ).toBe(1);
    expect(clampPart.fitMode).toBe(3);
    expect(clampPart.localTransform.rotationZ).toBe(180);
    expect(clampPart.localTransform.positionY).toBeCloseTo(clamp.size.z * 0.5, 12);
    expect(clampPart.sockets[0].positionY).toBeCloseTo(-clamp.size.z * 0.5, 12);
    expect(communication.wireSocket).toEqual({ partId: 1, socketName: "wire" });
    const highVoltage = bootstrap.assemblies.find((assembly) => assembly.id === 9202)!;
    const highVoltageArm = highVoltage.parts.find((part) => part.modelKey === "hv_crossarm")!;
    expect(highVoltageArm.fitMode).toBe(3);
    expect(highVoltageArm.localTransform.positionX).toBeCloseTo(0, 12);
    expect(highVoltage.endpointMountSocket).toEqual({ partId: 1, socketName: "endpoint_mount" });
    expect(highVoltageArm.sockets[0].positionZ).toBeCloseTo(crossarm.size.y * 0.5, 12);
    const lowVoltage = bootstrap.assemblies.find((assembly) => assembly.id === 9206)!;
    expect(lowVoltage.parts.map((part) => part.modelKey)).toEqual(["communication_clamp_long"]);
    expect(lowVoltage.parts[0].fitMode).toBe(0);
    expect(lowVoltage.parts[0].localTransform.rotationZ).toBe(180);
    expect(lowVoltage.parts[0].localTransform.positionY).toBeCloseTo(clampLong.size.z * 0.5, 12);
    expect(lowVoltage.parts[0].sockets[0].positionY).toBeCloseTo(0, 12);
    expect(lowVoltage.parts[0].sockets[0].positionZ).toBeCloseTo(clampLong.size.y * 0.5, 12);
    expect(lowVoltage.wireSocket).toEqual({ partId: 1, socketName: "wire" });
  });
});
