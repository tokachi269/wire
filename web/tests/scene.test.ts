import { describe, expect, it } from "vitest";
import * as THREE from "three";
import {
  POLE_RENDER_SIDES,
  SampledWireCurve,
  WireScene,
  WIRE_RADIAL_SEGMENTS,
  backboneNodeHitId,
  defaultRoadMaterialTextureDefinition,
  distanceToScreenSegmentPx,
  makeSampledTubeGeometry,
  makeRoadSurfaceMaterial,
  makeRoadMeshGeometry,
  roadGuideHalfWidth,
  roadGuidePoints,
  roadSurfaceColor,
  makeBackbonePick,
  poleAxisEndpoints,
  setPoleRotation
} from "../src/render/scene";
import { modelAssetCache } from "../src/render/modelAssets";
import { createViewerSnapshot } from "../src/store/viewer";
import { WireBridge } from "../src/bridge/wire";
import type {
  BundlePlacement,
  ModelAssemblyBootstrapInput,
  ModelTransformInput,
  PoleInfo,
  SupportNodeInfo
} from "../src/model";

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

  it("builds one tube ring per core sample without resampling", () => {
    const samples = new Float64Array([
      0, 0, 0,
      1, 0, 0,
      2, 1, 0,
      3, 1, 1
    ]);
    const geometry = makeSampledTubeGeometry(samples, 0.02);
    expect(geometry.getAttribute("position").count).toBe(4 * WIRE_RADIAL_SEGMENTS);
    expect(geometry.getAttribute("normal").count).toBe(4 * WIRE_RADIAL_SEGMENTS);
    expect(geometry.getIndex()?.count).toBe(3 * WIRE_RADIAL_SEGMENTS * 6);
  });
});

describe("road rendering", () => {
  it("keeps authoritative road content stable across transient pointer updates", () => {
    const scene = Object.create(WireScene.prototype) as any;
    const snapshot = createViewerSnapshot();
    snapshot.road.scene.surfaceMeshes = [{
      ownerSegmentId: 7,
      material: "asphalt",
      uvMapping: "patch_quantized",
      vertices: new Float64Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
      indices: new Uint32Array([0, 1, 2]),
      normals: new Float64Array([0, 0, 1, 0, 0, 1, 0, 0, 1]),
      uv0: new Float64Array([0, 0, 0.25, 0, 0, 1]),
      materialGroups: [{ material: "asphalt", indexStart: 0, indexCount: 3 }]
    }];
    expect(scene.roadContentSourcesChanged(snapshot)).toBe(true);
    const contentSignature = scene.sceneRoadContentSignature(snapshot);
    const firstOverlay = scene.sceneRoadOverlaySignature(snapshot);
    for (let index = 1; index <= 100; ++index) {
      snapshot.road.previewState = "guide";
      snapshot.road.laneTransitionCompleteT = index / 100;
      snapshot.road.hoveredDeleteSegmentId = index % 2 === 0 ? 7 : 0;
      expect(scene.roadContentSourcesChanged(snapshot)).toBe(false);
    }
    expect(scene.sceneRoadContentSignature(snapshot)).toBe(contentSignature);
    expect(scene.sceneRoadOverlaySignature(snapshot)).not.toBe(firstOverlay);
  });

  it("does not rescan wire content for pointer-only snapshot updates", () => {
    const scene = Object.create(WireScene.prototype) as any;
    const snapshot = createViewerSnapshot();
    expect(scene.wireContentSourcesChanged(snapshot)).toBe(true);
    for (let index = 1; index <= 100; ++index) {
      snapshot.road.laneTransitionCompleteT = index / 100;
      expect(scene.wireContentSourcesChanged(snapshot)).toBe(false);
    }
    snapshot.parts = [...snapshot.parts];
    expect(scene.wireContentSourcesChanged(snapshot)).toBe(true);
  });

  it("uses core material keys instead of deriving materials from vertex height", () => {
    const geometry = makeRoadMeshGeometry({
      vertices: new Float64Array([
        0, 0, 0.15,
        1, 0, 0,
        0, 1, 0.15
      ]),
      indices: new Uint32Array([0, 1, 2])
    });
    expect(geometry.getAttribute("color")).toBeUndefined();
    expect(roadSurfaceColor("sidewalk")).not.toBe(roadSurfaceColor("asphalt"));
    expect(roadSurfaceColor("curb")).not.toBe(roadSurfaceColor("asphalt"));
  });

  it("uses image texture placeholders for authored material channels", () => {
    expect(defaultRoadMaterialTextureDefinition.albedoUrl).toMatch(/Asphalt025A_2K-JPG_Color\.jpg/);
    expect(defaultRoadMaterialTextureDefinition.normalUrl).toMatch(/Asphalt025A_2K-JPG_NormalGL\.jpg/);
    expect(defaultRoadMaterialTextureDefinition.roughnessUrl).toMatch(/Asphalt025A_2K-JPG_Roughness\.jpg/);
    expect(defaultRoadMaterialTextureDefinition.specularUrl).toMatch(/Asphalt025A_2K-JPG_Specular\.png/);

    const material = makeRoadSurfaceMaterial("asphalt");
    expect(material.map).toBeInstanceOf(THREE.Texture);
    expect(material.normalMap).toBeInstanceOf(THREE.Texture);
    expect(material.roughnessMap).toBeInstanceOf(THREE.Texture);
    expect(material.specularIntensityMap).toBeInstanceOf(THREE.Texture);
    expect(material.map).not.toBeInstanceOf(THREE.DataTexture);
    expect(material.map?.wrapS).toBe(THREE.RepeatWrapping);
    expect(material.map?.wrapT).toBe(THREE.RepeatWrapping);
    material.dispose();
  });

  it("keeps non-asphalt road materials untextured", () => {
    const sidewalk = makeRoadSurfaceMaterial("sidewalk");
    const crosswalk = makeRoadSurfaceMaterial("road_marking_crosswalk");
    expect(sidewalk.map).toBeNull();
    expect(sidewalk.color.getHex()).toBe(roadSurfaceColor("sidewalk"));
    expect(crosswalk.map).toBeNull();
    expect(crosswalk.color.getHex()).toBe(roadSurfaceColor("road_marking_crosswalk"));
    sidewalk.dispose();
    crosswalk.dispose();
  });

  it("builds a local width guide from the same cubic request used for commit", () => {
    const points = roadGuidePoints({
      kind: "bezier",
      startX: 0,
      startY: 0,
      endX: 9,
      endY: 0,
      handleAX: 3,
      handleAY: 4,
      handleBX: 6,
      handleBY: 4,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      connectToFirstNode: false,
      roadLayoutTemplateId: 7
    }, 8);
    expect(points).toHaveLength(9);
    expect(points[0].toArray()).toEqual([0, 0, 0]);
    expect(points.at(-1)?.toArray()).toEqual([9, 0, 0]);
    expect(points[4].y).toBeGreaterThan(0);
    expect(roadGuideHalfWidth([{
      id: 7,
      strips: [
        { id: 1, function: "sidewalk", widthM: 2 },
        { id: 2, function: "carriageway", widthM: 7 },
        { id: 3, function: "sidewalk", widthM: 2 }
      ],
      sidewalkWidthM: 2,
      laneWidthM: 3.5,
      medianWidthM: 0,
      laneCount: 2,
      hasCenterLine: true,
      hasOuterLines: false,
      lanes: [],
      boundaries: []
    }], 7)).toBe(5.5);
  });

  it("picks road surface width instead of requiring centerline clicks", () => {
    const scene = Object.create(WireScene.prototype) as any;
    scene.snapshot = createViewerSnapshot();
    scene.snapshot.road.scene.centerlineSegments = [{
      id: 12,
      startX: 0,
      startY: 0,
      endX: 100,
      endY: 0,
      startSegmentDistanceM: 0,
      endSegmentDistanceM: 100,
      pickHalfWidthM: 5
    }];
    scene.snapshot.road.scene.nodes = [];
    scene.renderer = {
      domElement: {
        getBoundingClientRect: () => ({ left: 0, top: 0, width: 1000, height: 1000 })
      }
    };
    scene.projectToCanvas = (point: THREE.Vector3) =>
      new THREE.Vector2(point.x * 10, -point.y * 10);

    const hit = scene.pickRoadPoint(200, -40);
    expect(hit).not.toBeNull();
    expect(hit?.snap.segmentId).toBe(12);
    expect(hit?.snap.segmentDistanceM).toBeCloseTo(20);
  });
});

describe("backbone pick payload", () => {
  it("uses the support node id for pole snap picks instead of the pole id", () => {
    const node: SupportNodeInfo = {
      id: "node-10",
      kind: 0,
      poleId: "pole-2",
      x: 1,
      y: 2,
      z: 3
    };
    const pick = makeBackbonePick([node.x, node.y, node.z], 1, backboneNodeHitId(node));
    expect(pick.hitKind).toBe(1);
    expect(pick.hitId).toBe("node-10");
  });

  it("snaps pole-body screen hits to the support node instead of falling through to midair", () => {
    const pole: PoleInfo = {
      id: "pole-2",
      poleTypeId: 1,
      height: 100,
      positionX: 0,
      positionY: 0,
      positionZ: 0,
      rotationX: 0,
      rotationY: 0,
      rotationZ: 0,
      scaleX: 1,
      scaleY: 1,
      scaleZ: 1
    };
    const node: SupportNodeInfo = {
      id: "node-10",
      kind: 0,
      poleId: pole.id,
      x: 0,
      y: 0,
      z: 0
    };
    const [axisBase, axisTop] = poleAxisEndpoints(pole);
    expect(axisBase.toArray()).toEqual([0, 0, 0]);
    expect(axisTop.toArray()).toEqual([0, 0, 100]);
    expect(distanceToScreenSegmentPx(
      new THREE.Vector2(0, -92),
      new THREE.Vector2(axisBase.x, -axisBase.z),
      new THREE.Vector2(axisTop.x, -axisTop.z)
    )).toBe(0);

    const scene = Object.create(WireScene.prototype) as any;
    scene.snapshot = {
      ...createViewerSnapshot(),
      showBackboneOverlay: true,
      poles: [pole],
      supportNodes: [node],
      backboneEdges: []
    };
    scene.renderer = {
      domElement: {
        getBoundingClientRect: () => ({ left: 0, top: 0, width: 200, height: 200 })
      }
    };
    scene.projectToCanvas = (point: THREE.Vector3) => new THREE.Vector2(point.x, -point.z);

    const hit = scene.pickBackbonePoint(0, -92);
    expect(hit).not.toBeNull();
    expect(hit.pick.hitKind).toBe(1);
    expect(hit.pick.hitId).toBe(node.id);
  });
});

describe("scene part reuse", () => {
  it("rebuilds only the part whose stable key version changed", () => {
    const scene = Object.create(WireScene.prototype) as any;
    scene.content = new THREE.Group();
    scene.partMeshes = new Map();
    scene.modelObjects = new Map();
    scene.modelBatches = new Map();
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

    const firstGeometry = first.geometry;
    snapshot.parts[0].info.sourceVersion = "11";
    snapshot.parts[0].samples = new Float64Array([0, 2, 0, 10, 2, 0]);
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.contentSyncStats).toEqual({
      total: 2, reused: 1, rebuilt: 1, removed: 0,
      modelTotal: 0, modelReused: 0, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
    expect(scene.partMeshes.get("edge:1:lane:0").mesh).toBe(first);
    expect(scene.partMeshes.get("edge:1:lane:0").mesh.geometry).toBe(firstGeometry);
    expect(scene.partMeshes.get("edge:2:lane:0").mesh).toBe(second);
    first.geometry.computeBoundingBox();
    const center = new THREE.Vector3();
    first.geometry.boundingBox!.getCenter(center);
    expect(center.y).toBeCloseTo(2, 2);

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
    scene.modelBatches = new Map();
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
    scene.modelBatches = new Map();
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
    const centerYs = centers.map((center) => center.y).sort((a, b) => a - b);
    expect(centerYs[0]).toBeCloseTo(-0.65, 2);
    expect(centerYs[1]).toBeCloseTo(-0.20, 2);
    expect(centerYs[2]).toBeCloseTo(0.25, 2);
    expect(centerYs[1] - centerYs[0]).toBeCloseTo(0.45, 2);
    expect(centerYs[2] - centerYs[1]).toBeCloseTo(0.45, 2);
    const insulatorYs = snapshot.models
      .filter((model) => model.modelKey === "hv_insulator")
      .map((model) => model.positionY.toFixed(3));
    expect(new Set(insulatorYs)).toEqual(new Set(["-0.650", "-0.200", "0.250"]));
    bridge.dispose();
  });
});

describe("scene model reuse", () => {
  it("reuses the model batch for the same stable key and updates only its Core transform", () => {
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
    scene.modelBatches = new Map();
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
    const firstBatch = scene.modelBatches.get("pole_body")!;
    expect(firstBatch.meshes).toHaveLength(1);
    const firstMesh = firstBatch.meshes[0] as THREE.InstancedMesh;
    const matrix = new THREE.Matrix4();
    firstMesh.getMatrixAt(0, matrix);
    expect(new THREE.Vector3().setFromMatrixPosition(matrix).toArray()).toEqual([1, 2, 3]);

    expect(scene.syncContent(snapshot)).toBe(false);
    expect(scene.modelBatches.get("pole_body")!.meshes[0]).toBe(firstMesh);

    snapshot.models[0] = {
      ...snapshot.models[0],
      contentVersion: "11",
      positionX: 8
    };
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.modelBatches.get("pole_body")!.meshes[0]).toBe(firstMesh);
    firstMesh.getMatrixAt(0, matrix);
    expect(new THREE.Vector3().setFromMatrixPosition(matrix).x).toBe(8);
    expect(scene.contentSyncStats.modelUpdated).toBe(1);

    snapshot.models = [];
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.modelObjects.size).toBe(0);
    expect(scene.modelBatches.size).toBe(0);
    expect(scene.contentSyncStats.modelRemoved).toBe(1);
    (modelAssetCache as any).loaded.delete("poleBody");
  });

  it("batches models with the same modelKey into one InstancedMesh without changing sync stats", () => {
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
    scene.modelBatches = new Map();
    scene.pendingModelKeys = new Set();
    scene.poleMeshes = new Map();
    const model = (stableKey: string, x: number) => ({
      stableKey,
      modelKey: "pole_body",
      contentVersion: "1",
      positionX: x,
      positionY: 0,
      positionZ: 0,
      rotationX: 0,
      rotationY: 0,
      rotationZ: 0,
      scaleX: 1,
      scaleY: 1,
      scaleZ: 1
    });
    const snapshot = createViewerSnapshot();
    snapshot.models = [model("pole:1:9201:1", 1), model("pole:2:9201:1", 2)];

    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.content.children).toHaveLength(1);
    expect(scene.content.children[0]).toBeInstanceOf(THREE.InstancedMesh);
    expect(scene.modelBatches.get("pole_body")!.capacity).toBe(2);
    expect(scene.contentSyncStats).toEqual({
      total: 0, reused: 0, rebuilt: 0, removed: 0,
      modelTotal: 2, modelReused: 0, modelUpdated: 0, modelRebuilt: 2, modelRemoved: 0
    });

    expect(scene.syncContent(snapshot)).toBe(false);
    expect(scene.contentSyncStats).toEqual({
      total: 0, reused: 0, rebuilt: 0, removed: 0,
      modelTotal: 2, modelReused: 2, modelUpdated: 0, modelRebuilt: 0, modelRemoved: 0
    });
    (modelAssetCache as any).loaded.delete("poleBody");
  });

  it("renders support-detail proxy model keys through primitive assets without GLB load", () => {
    const scene = Object.create(WireScene.prototype) as any;
    scene.content = new THREE.Group();
    scene.partMeshes = new Map();
    scene.modelObjects = new Map();
    scene.modelBatches = new Map();
    scene.pendingModelKeys = new Set();
    scene.poleMeshes = new Map();
    const snapshot = createViewerSnapshot();
    snapshot.models = [{
      stableKey: "detail:transformer:1",
      modelKey: "pole_transformer_20kva_proxy",
      contentVersion: "1",
      positionX: 1,
      positionY: 2,
      positionZ: 3,
      rotationX: 0,
      rotationY: 0,
      rotationZ: 45,
      scaleX: 0.5,
      scaleY: 0.4,
      scaleZ: 0.8
    }];

    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.pendingModelKeys.size).toBe(0);
    expect(scene.modelBatches.get("pole_transformer_20kva_proxy")!.capacity).toBe(1);
    expect(scene.content.children[0]).toBeInstanceOf(THREE.InstancedMesh);
  });
});
