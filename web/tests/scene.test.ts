import { describe, expect, it } from "vitest";
import * as THREE from "three";
import {
  POLE_RENDER_SIDES,
  SampledWireCurve,
  WireScene,
  WIRE_RADIAL_SEGMENTS,
  setPoleRotation
} from "../src/render/scene";
import { createViewerSnapshot } from "../src/store/viewer";

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
    scene.poleMeshes = new Map();
    const snapshot = createViewerSnapshot();
    snapshot.parts = [{
      info: {
        partKey: "edge:1:lane:0",
        sourceVersion: "10",
        sampleOffset: 0,
        kind: 0,
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
    expect(scene.contentSyncStats).toEqual({ total: 2, reused: 0, rebuilt: 2, removed: 0 });
    const first = scene.partMeshes.get("edge:1:lane:0").mesh;
    const second = scene.partMeshes.get("edge:2:lane:0").mesh;
    snapshot.logs.push("unrelated store update");
    expect(scene.syncContent(snapshot)).toBe(false);
    expect(scene.contentSyncStats).toEqual({ total: 2, reused: 2, rebuilt: 0, removed: 0 });
    expect(scene.partMeshes.get("edge:1:lane:0").mesh).toBe(first);

    snapshot.parts[0].info.sourceVersion = "11";
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.contentSyncStats).toEqual({ total: 2, reused: 1, rebuilt: 1, removed: 0 });
    expect(scene.partMeshes.get("edge:1:lane:0").mesh).not.toBe(first);
    expect(scene.partMeshes.get("edge:2:lane:0").mesh).toBe(second);

    snapshot.parts.splice(1, 1);
    expect(scene.syncContent(snapshot)).toBe(true);
    expect(scene.contentSyncStats).toEqual({ total: 1, reused: 1, rebuilt: 0, removed: 1 });
  });
});
