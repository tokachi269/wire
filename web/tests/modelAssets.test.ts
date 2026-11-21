import fs from "node:fs/promises";
import { fileURLToPath } from "node:url";

import { afterEach, describe, expect, it, vi } from "vitest";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import {
  buildDefaultModelBootstrap,
  cloneSharedAsset,
  loadModelAssets,
  ModelAssetCache
} from "../src/render/modelAssets";

afterEach(() => vi.unstubAllGlobals());

function fixtureScene(): THREE.Group {
  const source = new THREE.Group();
  const root = new THREE.Group();
  root.name = "transformer_m_01_ROOT";
  root.add(new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.2, 0.4), new THREE.MeshStandardMaterial()));
  source.add(root);
  return source;
}

async function readFittings(): Promise<THREE.Group> {
  // Node has no image decoder; geometry, materials and their shared identities
  // are real GLB data. This test does not verify texture appearance.
  vi.stubGlobal("self", globalThis);
  vi.stubGlobal("createImageBitmap", async () => ({ width: 1, height: 1, close() {} }));
  const bytes = await fs.readFile(fileURLToPath(new URL("../src/assets/pole_fittings.glb", import.meta.url)));
  return (await new GLTFLoader().parseAsync(
    bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), ""
  )).scene;
}

describe("model asset cache", () => {
  it("registers catalog support-detail proxies with fixed dimensions and no GLB load", async () => {
    const load = vi.fn(async () => {
      throw new Error("catalog support-detail proxies must not request GLB assets");
    });
    const cache = new ModelAssetCache(load);
    const [transformer, intermediateInsulator, supportBracket, pc6, mount, arrester, triplex, optical] = await Promise.all([
      cache.load("poleTransformer20kvaProxy"),
      cache.load("transformerIntermediateInsulatorProxy"),
      cache.load("transformerSupportBracketProxy"),
      cache.load("pc6CutoutProxy"),
      cache.load("tma13CutoutMountProxy"),
      cache.load("arresterGlB6gProxy"),
      cache.load("hvTriplexTermination60Proxy"),
      cache.load("aerialOpticalClosureRca3aoProxy")
    ]);

    expect(load).not.toHaveBeenCalled();
    expect(transformer.size.x).toBeCloseTo(0.430, 3);
    expect(transformer.size.y).toBeCloseTo(0.430, 3);
    expect(transformer.size.z).toBeCloseTo(0.610, 3);
    expect(intermediateInsulator.size.x).toBeCloseTo(0.360, 3);
    expect(intermediateInsulator.size.y).toBeCloseTo(0.104, 3);
    expect(intermediateInsulator.size.z).toBeCloseTo(0.104, 3);
    expect(supportBracket.size.x).toBeCloseTo(0.700, 3);
    expect(supportBracket.size.y).toBeCloseTo(0.260, 3);
    expect(supportBracket.size.z).toBeCloseTo(0.060, 3);
    expect(pc6.size.x).toBeCloseTo(0.406, 3);
    expect(pc6.size.y).toBeCloseTo(0.090, 3);
    expect(pc6.size.z).toBeCloseTo(0.094, 3);
    expect(mount.size.x).toBeCloseTo(0.150, 3);
    expect(mount.size.y).toBeCloseTo(0.032, 3);
    expect(mount.size.z).toBeCloseTo(0.115, 3);
    expect(arrester.size.x).toBeCloseTo(0.082, 3);
    expect(arrester.size.y).toBeCloseTo(0.082, 3);
    expect(arrester.size.z).toBeCloseTo(0.250, 3);
    expect(triplex.size.x).toBeCloseTo(0.580, 3);
    expect(triplex.size.y).toBeCloseTo(0.495, 3);
    expect(triplex.size.z).toBeLessThanOrEqual(0.130);
    expect(optical.size.x).toBeCloseTo(0.730, 3);
    expect(optical.size.y).toBeCloseTo(0.180, 3);
    expect(optical.size.z).toBeCloseTo(0.220, 3);
  });

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
    expect(first.mountAnchor.x).toBeCloseTo(0, 12);
    expect(first.mountAnchor.z).toBeCloseTo(0, 12);
    expect(first.size.y * (10 / 12)).toBeCloseTo(10, 12);
  });

  it("builds HV, LV, and communication assemblies from measured local assets", async () => {
    const load = vi.fn(async () => {
      return fixtureScene();
    });
    const cache = new ModelAssetCache(load);
    const assets = await loadModelAssets(cache);
    const { poleBody: pole, crossarmHv: crossarm, belt } = assets;
    const { communicationClamp: clamp, communicationClampLong: clampLong } = assets;
    expect(cache.loadCount("poleBody")).toBe(1);
    expect(cache.loadCount("crossarmHv")).toBe(1);
    expect(cache.loadCount("belt")).toBe(1);
    expect(cache.loadCount("hvInsulator")).toBe(1);
    expect(cache.loadCount("communicationClampLong")).toBe(1);
    expect(cache.loadCount("communicationClamp")).toBe(1);
    const bootstrap = buildDefaultModelBootstrap(assets);

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
    expect(distributionPole.parts[0].localTransform.scaleZ).toBeCloseTo(1, 12);
    expect(communicationPole.parts[0].localTransform.scaleZ).toBeCloseTo(1.135, 12);
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
    expect(pole.radialReferenceM).toBeCloseTo((0.190 + 10 / 75) * 0.5, 12);
    expect(pole.radialTopM).toBeCloseTo(0.190 * 0.5, 12);
    expect(distributionPole.parts[0].localTransform.scaleX).toBe(1);
    expect(distributionPole.parts[0].localTransform.scaleY).toBe(1);
    expect(belt.radialReferenceM).not.toBeNull();
    expect(belt.radialReferenceM).toBeCloseTo((0.190 + 12 / 75) * 0.5, 12);
    expect(belt.mountAnchor.y).toBeCloseTo(belt.bounds.getCenter(new THREE.Vector3()).y, 12);
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
    expect(lowVoltage.parts[0].sockets[0].positionZ).toBeCloseTo(0, 12);
    expect(lowVoltage.wireSocket).toEqual({ partId: 1, socketName: "wire" });
  });
  it("reads the transformer's wire sockets out of the shipped GLB", async () => {
    const cache = new ModelAssetCache(async (url) => {
      return url.endsWith("pole_fittings.glb") ? readFittings() : fixtureScene();
    });
    const assets = await loadModelAssets(cache);
    const transformer = assets.poleTransformerM01;

    // Y-up (x, y, z) becomes Core-local (x, -z, y), and a marker's local +X is
    // the direction the wire leaves. Hard numbers because the GLB is the only
    // place these live: a lost marker has to fail here, not render wrong.
    expect(transformer.sockets.map((socket) => socket.name)).toEqual([
      "connect_hv_0", "connect_hv_1", "connect_lv_0", "connect_lv_1", "connect_lv_2"
    ]);
    const [hv0, , lv0] = transformer.sockets;
    expect(hv0.positionX).toBeCloseTo(0.2292666666666667, 6);
    expect(hv0.positionY).toBeCloseTo(-0.125, 6);
    expect(hv0.positionZ).toBeCloseTo(0.655, 6);
    expect(hv0.directionX).toBeCloseTo(-0.330350, 5);
    expect(hv0.directionZ).toBeCloseTo(0.943858, 5);
    expect(lv0.positionX).toBeCloseTo(0.3292666666666667, 6);
    expect(lv0.positionY).toBeCloseTo(-0.1, 6);
    expect(lv0.positionZ).toBeCloseTo(0, 6);
    expect(lv0.directionY).toBeCloseTo(-0.242536, 5);
    expect(lv0.directionZ).toBeCloseTo(-0.970143, 5);
    // mountRule "origin" keeps the authored datum, so Core sees these unshifted.
    expect(transformer.mountAnchor.length()).toBeCloseTo(0, 12);

    const bootstrap = buildDefaultModelBootstrap(assets);
    const assembly = bootstrap.assemblies.find((entry) => entry.id === 9208)!;
    const tank = assembly.parts.find((part) => part.modelKey === transformer.modelKey)!;
    expect(tank.sockets).toEqual(transformer.sockets);
    // curve_parts.cpp drops a socket whose part is not kRigid.
    expect(tank.fitMode).toBe(0);
    expect(assembly.parts).toHaveLength(1);
    expect(tank.localTransform.positionX).toBe(0);
    const left = transformer.source.getObjectByName("support_left") as THREE.Mesh;
    const right = transformer.source.getObjectByName("support_right") as THREE.Mesh;
    expect(left.geometry).toBe(right.geometry);
    expect(left.material).toBe(right.material);
    const armBounds = new THREE.Box3().setFromObject(left);
    expect(armBounds.min.x).toBeCloseTo(-0.25, 6);
    expect(armBounds.max.x).toBeCloseTo(1.25, 6);
    expect(armBounds.max.z).toBeCloseTo(0, 6);
    const insulator = transformer.source.getObjectByName("arm_insulator")!;
    expect(new THREE.Box3().setFromObject(insulator).min.z).toBeCloseTo(0, 6);
  });

  it("keeps authored child edits and sockets while excluding the root display transform", async () => {
    const source = await readFittings();
    const root = source.getObjectByName("transformer_m_01_ROOT")!;
    root.position.set(12, 5, -3);
    root.rotation.set(0.4, -0.7, 1.2);
    root.scale.set(2, 2, 2);
    const tank = root.getObjectByName("transformer_m_01")!;
    tank.position.x += 0.15;
    root.getObjectByName("connect_hv_0")!.name = "External HV connector";
    const asset = await new ModelAssetCache(async () => source).load("poleTransformerM01");
    const socket = asset.sockets.find((entry) => entry.name === "connect_hv_0")!;
    expect(socket.positionX).toBeCloseTo(0.2292666666666667 + 0.15, 6);
    const arm = asset.source.getObjectByName("support_left")!;
    const position = new THREE.Vector3().setFromMatrixPosition(arm.matrixWorld);
    expect(position.x).toBeCloseTo(0.5, 6);
    expect(root.position.x).toBe(12);
    expect(root.scale.x).toBe(2);
  });

  it("rejects a missing pattern root instead of loading the whole display", async () => {
    const cache = new ModelAssetCache(async () => new THREE.Group());
    await expect(cache.load("poleTransformerM01")).rejects.toThrow("pattern root");
  });
});
