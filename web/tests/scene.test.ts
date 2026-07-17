import { describe, expect, it } from "vitest";
import * as THREE from "three";
import {
  POLE_RENDER_SIDES,
  SampledWireCurve,
  WireScene,
  WIRE_RADIAL_SEGMENTS,
  setPoleRotation
} from "../src/render/scene";
import { modelAssetCache } from "../src/render/modelAssets";
import { createViewerSnapshot } from "../src/store/viewer";
import { WireBridge } from "../src/bridge/wire";
import type { BundlePlacement, ModelAssemblyBootstrapInput, ModelTransformInput } from "../src/model";

function rotateCoreXyz(value: THREE.Vector3, xDeg: number, yDeg: number, zDeg: number): THREE.Vector3 {
  const radians = [xDeg, yDeg, zDeg].map(THREE.MathUtils.degToRad);
  const [x, y, z] = radians;
  return value.clone()
    .applyAxisAngle(new THREE.Vector3(1, 0, 0), x)
    .applyAxisAngle(new THREE.Vector3(0, 1, 0), y)
    .applyAxisAngle(new THREE.Vector3(0, 0, 1), z);
}

describe("pole rendering", () => {
  it("uses the same X then Y then Z tilt composition as core port placement", () => {
    const rotation = { x: 3.4379, y: 1.75989, z: -57.805 };
    const pole = new THREE.Object3D();
    setPoleRotation(pole, rotation.x, rotation.y, rotation.z);

    const renderedTop = new THREE.Vector3(0, 0, 10).applyEuler(pole.rotation);
    const coreTop = rotateCoreXyz(new THREE.Vector3(0, 0, 10), rotation.x, rotation.y, rotation.z);

    expect(renderedTop.distanceTo(coreTop)).toBeLessThan(1e-12);
  });

  it("keeps the pole round while using triangular wire tubes", () => {
    expect(POLE_RENDER_SIDES).toBe(16);
    expect(WIRE_RADIAL_SEGMENTS).toBe(3);
  });
});

describe("sampled wire curve", () => {
  it("evaluates by arc distance across an uneven sample table", () => {
    const curve = new SampledWireCurve([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(5, 0, 0),
      new THREE.Vector3(5, 5, 0)
    ]);

    expect(curve.getPoint(0.5).toArray()).toEqual([5, 0, 0]);
    expect(curve.getPoint(0.75).toArray()).toEqual([5, 2.5, 0]);
  });
});

describe("scene part reuse", () => {
  it("rebuilds only the part whose stable key version changed", () => {
    const scene = Object.create(WireScene.prototype) as any;
    scene.content = new THREE.Group();
    scene.partMeshes = new Map();
    scene.modelObjects = new Map();
    scene.pendingModelKeys = new Set();
    scene.poleMeshes = new Map();
    const snapshot = createViewerSnapshot();
    snapshot.parts = [{
      info: {
        partKey: "edge:1:lane:0",
        sourceVersion: "10",
        sampleOffset: 0,
        kind: 0,
        supplementalKind: 0,
        wireRadius: 0.02,
        colorRgba: 0xffffffff,
        sourceNodeId: "0",
        sourceEdgeId: "1",
        sourceSpanId: "2",
        sourceBundleId: "3",
        bundleTemplateId: 102,
        laneIndex: 0,
        runId: 1,
        sampleCount: 2
      },
      samples: new Float64Array([0, 0, 0, 10, 0, 0])
    }, {
      info: {
        partKey: "edge:2:lane:0",
        sourceVersion: "20",
        sampleOffset: 6,
        kind: 0,
        supplementalKind: 0,
        wireRadius: 0.02,
        colorRgba: 0xffffffff,
        sourceNodeId: "0",
        sourceEdgeId: "2",
        sourceSpanId: "4",
        sourceBundleId: "3",
        bundleTemplateId: 102,
        laneIndex: 0,
        runId: 2,
        sampleCount: 2
      },
      samples: new Float64Array([0, 1, 0, 10, 1, 0])
    }];

    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.contentSyncStats).toEqual({
      total: 2, reused: 0, rebuilt: 2, removed: 0,
      modelTotal: 0, modelReused: 0, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
    const first = scene.partMeshes.get("edge:1:lane:0").mesh;
    const second = scene.partMeshes.get("edge:2:lane:0").mesh;
    snapshot.logs.push("unrelated store update");
    expect(scene.syncContent(snapshot)).toBe(false);
    expect(scene.contentSyncStats).toEqual({
      total: 2, reused: 2, rebuilt: 0, removed: 0,
      modelTotal: 0, modelReused: 0, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
    expect(scene.partMeshes.get("edge:1:lane:0").mesh).toBe(first);

    snapshot.parts[0].info.sourceVersion = "11";
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.contentSyncStats).toEqual({
      total: 2, reused: 1, rebuilt: 1, removed: 0,
      modelTotal: 0, modelReused: 0, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
    expect(scene.partMeshes.get("edge:1:lane:0").mesh).not.toBe(first);
    expect(scene.partMeshes.get("edge:2:lane:0").mesh).toBe(second);

    snapshot.parts.splice(1, 1);
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.contentSyncStats).toEqual({
      total: 1, reused: 1, rebuilt: 0, removed: 1,
      modelTotal: 0, modelReused: 0, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
  });

  it("shares one dedicated support material and renders conductor colors near black", () => {
    const scene = Object.create(WireScene.prototype) as any;
    scene.content = new THREE.Group();
    scene.partMeshes = new Map();
    scene.modelObjects = new Map();
    scene.pendingModelKeys = new Set();
    scene.poleMeshes = new Map();
    scene.supportWireMaterial = null;
    const snapshot = createViewerSnapshot();
    const part = (partKey: string, supplementalKind: number, colorRgba: number, y: number) => ({
      info: {
        partKey,
        sourceVersion: "1",
        sampleOffset: 0,
        kind: supplementalKind === 0 ? 0 : 3,
        supplementalKind,
        wireRadius: 0.02,
        colorRgba,
        sourceNodeId: "0",
        sourceEdgeId: "1",
        sourceSpanId: partKey,
        sourceBundleId: "3",
        bundleTemplateId: 101,
        laneIndex: 0,
        runId: 1,
        sampleCount: 2
      },
      samples: new Float64Array([0, y, 0, 10, y, 0])
    });
    snapshot.parts = [
      part("main", 0, 0xffffffff, 0),
      part("support-a", 1, 0xffffffff, 1),
      part("support-b", 1, 0x112233ff, 2)
    ];

    expect(scene.syncContent(snapshot)).toBe(true);
    const mainMaterial = scene.partMeshes.get("main").mesh.material as THREE.MeshStandardMaterial;
    const supportA = scene.partMeshes.get("support-a").mesh.material;
    const supportB = scene.partMeshes.get("support-b").mesh.material;
    expect(supportA).toBe(supportB);
    expect(supportA).not.toBe(mainMaterial);
    expect(Math.max(mainMaterial.color.r, mainMaterial.color.g, mainMaterial.color.b)).toBeLessThan(0.2);

    snapshot.parts[1].info.sourceVersion = "2";
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.partMeshes.get("support-a").mesh.material).toBe(supportB);
  });
});

function defaultHvPlacement(): BundlePlacement[] {
  return [{
    id: 1,
    bundleTemplateId: 101,
    count: 3,
    explicit: true,
    height: 9.2,
    offset: -0.2,
    spacing: 0.45
  }];
}

const identityTransform = (): ModelTransformInput => ({
  positionX: 0, positionY: 0, positionZ: 0,
  rotationX: 0, rotationY: 0, rotationZ: 0,
  scaleX: 1, scaleY: 1, scaleZ: 1
});

function modelBootstrap(): ModelAssemblyBootstrapInput {
  return {
    assemblies: [{
      id: 9901,
      version: 1,
      parts: [{ partId: 1, modelKey: "pole_body", descriptorName: "pole", descriptorVersion: 1,
        fitMode: 1, localTransform: identityTransform(), sockets: [] }],
      wireSocket: null
    }, {
      id: 9902,
      version: 1,
      parts: [{
        partId: 1, modelKey: "hv_crossarm", descriptorName: "crossarm", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(),
        sockets: [{ name: "endpoint_mount", positionX: 0, positionY: 0, positionZ: 0.04,
          directionX: 0, directionY: 0, directionZ: 1 }]
      }],
      wireSocket: null,
      endpointMountSocket: { partId: 1, socketName: "endpoint_mount" }
    }, {
      id: 9903,
      version: 1,
      parts: [{
        partId: 1, modelKey: "hv_insulator", descriptorName: "insulator", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(),
        sockets: [{ name: "wire", positionX: 0, positionY: 0, positionZ: -0.25,
          directionX: 1, directionY: 0, directionZ: 0 }]
      }],
      wireSocket: { partId: 1, socketName: "wire" }
    }],
    poleAssignments: [{ poleTypeId: 1, assemblyId: 9901, radiusBaseM: 0.16, radiusTopM: 0.10 }],
    bundleAssignments: [{ bundleTemplateId: 101, rowAssemblyId: 9902, endpointAssemblyId: 9903 }]
  };
}

describe("scene geometry from wasm", () => {
  it("creates three distinct model-aware HV meshes on first sync from a bridge snapshot", async () => {
    const bridge = await WireBridge.create();
    const configured = bridge.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const generated = bridge.generate(
      new Float64Array([0, 0, 0, 12, 0, 0]),
      defaultHvPlacement(),
      0,
      1,
      0,
      0,
      []
    );
    expect(generated.ok, generated.error).toBe(true);

    const bridgeScene = bridge.scene();
    const snapshot = createViewerSnapshot();
    snapshot.parts = bridgeScene.parts;
    snapshot.models = bridgeScene.models;

    const scene = Object.create(WireScene.prototype) as any;
    scene.content = new THREE.Group();
    scene.partMeshes = new Map();
    scene.modelObjects = new Map();
    scene.pendingModelKeys = new Set();
    scene.poleMeshes = new Map();
    scene.supportWireMaterial = null;

    expect(scene.syncContent(snapshot)).toBe(true);
    const hvKeys = snapshot.parts
      .filter((part) => part.info.kind === 0 && part.info.bundleTemplateId === 101)
      .map((part) => part.info.partKey);
    expect(hvKeys).toHaveLength(3);
    expect(new Set(hvKeys).size).toBe(3);

    const centers = hvKeys.map((key) => {
      const mesh = scene.partMeshes.get(key).mesh as THREE.Mesh;
      mesh.geometry.computeBoundingBox();
      const center = new THREE.Vector3();
      mesh.geometry.boundingBox!.getCenter(center);
      return center;
    });
    expect(new Set(centers.map((center) => center.y.toFixed(3)))).toEqual(
      new Set(["-0.650", "-0.200", "0.250"])
    );
    const insulatorYs = snapshot.models
      .filter((model) => model.modelKey === "hv_insulator")
      .map((model) => model.positionY.toFixed(3));
    expect(new Set(insulatorYs)).toEqual(new Set(["-0.650", "-0.200", "0.250"]));
    bridge.dispose();
  });
});

describe("scene model reuse", () => {
  it("reuses the Object3D for the same stable key and updates only its Core transform", () => {
    const source = new THREE.Group();
    source.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial()));
    const bounds = new THREE.Box3(new THREE.Vector3(-0.5, -0.5, -0.5), new THREE.Vector3(0.5, 0.5, 0.5));
    (modelAssetCache as any).loaded.set("poleBody", {
      kind: "poleBody",
      modelKey: "pole_body",
      source,
      bounds,
      size: new THREE.Vector3(1, 1, 1),
      mountAnchor: new THREE.Vector3(),
      descriptorVersion: 1,
      adapter: { modelKey: "pole_body", url: "test", mountRule: "center" }
    });

    const scene = Object.create(WireScene.prototype) as any;
    scene.content = new THREE.Group();
    scene.partMeshes = new Map();
    scene.modelObjects = new Map();
    scene.pendingModelKeys = new Set();
    scene.poleMeshes = new Map();
    const snapshot = createViewerSnapshot();
    snapshot.models = [{
      stableKey: "pole:1:9201:1",
      modelKey: "pole_body",
      contentVersion: "10",
      positionX: 1,
      positionY: 2,
      positionZ: 3,
      rotationX: 4,
      rotationY: 5,
      rotationZ: 6,
      scaleX: 1,
      scaleY: 1,
      scaleZ: 1
    }];

    expect(scene.syncContent(snapshot)).toBe(true);
    const first = scene.modelObjects.get("pole:1:9201:1").object;
    expect(first.position.toArray()).toEqual([1, 2, 3]);

    expect(scene.syncContent(snapshot)).toBe(false);
    expect(scene.modelObjects.get("pole:1:9201:1").object).toBe(first);

    snapshot.models[0] = {
      ...snapshot.models[0],
      contentVersion: "11",
      positionX: 8
    };
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.modelObjects.get("pole:1:9201:1").object).toBe(first);
    expect(first.position.x).toBe(8);
    expect(scene.contentSyncStats.modelUpdated).toBe(1);

    snapshot.models = [];
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.modelObjects.size).toBe(0);
    expect(scene.contentSyncStats.modelRemoved).toBe(1);
    (modelAssetCache as any).loaded.delete("poleBody");
  });
});
