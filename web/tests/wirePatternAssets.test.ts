import fs from "node:fs/promises";
import { fileURLToPath } from "node:url";

import { describe, expect, it } from "vitest";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

import type { VisualCurveAppearancePieceInfo } from "../src/model";
import {
  deformWirePattern,
  extractLooseEdgeChain,
  materializeWirePattern,
  type WirePatternAsset
} from "../src/render/wirePatternAssets";

async function loadAsset(relativePath: string): Promise<WirePatternAsset> {
  const path = fileURLToPath(new URL(`../src/assets/wire-patterns/${relativePath}.glb`, import.meta.url));
  const bytes = await fs.readFile(path);
  const data = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
  const gltf = await new GLTFLoader().parseAsync(data, "");
  return extractLooseEdgeChain(gltf.scene);
}

function piece(
  assetKey: string,
  start: number,
  end: number,
  reverse = false,
  localOffsetScaleM = 1
): VisualCurveAppearancePieceInfo {
  return {
    assetKey,
    curveStartM: start,
    curveEndM: end,
    sourceLengthM: assetKey.startsWith("main/") ? 4 : 1,
    localOffsetScaleM,
    reverse
  };
}

describe("wire pattern GLB adapter", () => {
  it("loads all authored LINES variants with the declared centered length", async () => {
    for (const directory of ["main", "connection"]) {
      const expectedLength = directory === "main" ? 4 : 1;
      for (const family of ["wire", "helix"]) {
        for (const variant of [1, 2]) {
          const asset = await loadAsset(`${directory}/${family}_${variant}`);
          expect(asset.maxX - asset.minX).toBeCloseTo(expectedLength, 6);
          expect(asset.minX).toBeCloseTo(-expectedLength * 0.5, 6);
          expect(asset.maxX).toBeCloseTo(expectedLength * 0.5, 6);
          expect(asset.points.length).toBeGreaterThanOrEqual(5);
          expect(asset.points[0].y).toBeCloseTo(0, 7);
          expect(asset.points[0].z).toBeCloseTo(0, 7);
          expect(asset.points.at(-1)!.y).toBeCloseTo(0, 7);
          expect(asset.points.at(-1)!.z).toBeCloseTo(0, 7);
        }
      }
    }
  });

  it("extracts only a non-branching LINES chain and ignores triangle face edges", () => {
    const root = new THREE.Group();
    const lineGeometry = new THREE.BufferGeometry();
    lineGeometry.setAttribute("position", new THREE.Float32BufferAttribute([
      -2, 0, 0, 0, 0, 0, 2, 0, 0
    ], 3));
    lineGeometry.setIndex([0, 1, 1, 2]);
    root.add(new THREE.LineSegments(lineGeometry));
    root.add(new THREE.Mesh(new THREE.BoxGeometry(20, 20, 20)));

    const asset = extractLooseEdgeChain(root);
    expect(asset.points.map((point) => point.toArray())).toEqual([
      [-2, 0, 0], [0, 0, 0], [2, 0, 0]
    ]);
    expect(asset.minX).toBe(-2);
    expect(asset.maxX).toBe(2);
  });

  it("maps intermediate straight-asset stations onto the Core sag curve", async () => {
    const asset = await loadAsset("main/wire_1");
    const sag = new Float64Array([
      0, 0, 0,
      0.95, 0, -0.15,
      1.9, 0, -0.3,
      2.85, 0, -0.15,
      3.8, 0, 0
    ]);
    const deformed = deformWirePattern(asset, sag, piece("main/wire_1", 0,
      4 * Math.sqrt(0.95 ** 2 + 0.15 ** 2)));

    expect(deformed.length).toBe(asset.points.length);
    expect(deformed[0].distanceTo(new THREE.Vector3(0, 0, 0))).toBeLessThan(1e-8);
    expect(deformed.at(-1)!.distanceTo(new THREE.Vector3(3.8, 0, 0))).toBeLessThan(1e-8);
    expect(Math.min(...deformed.map((point) => point.z))).toBeLessThan(-0.25);
    expect(deformed.some((point) => point.z < -0.1)).toBe(true);
  });

  it("keeps adjacent and reversed GLB pieces joined at Core-owned intervals", async () => {
    const asset = await loadAsset("main/wire_2");
    const samples = new Float64Array([0, 0, 0, 3, 0, -0.5, 6, 0, 0]);
    const total = 2 * Math.sqrt(9.25);
    const pieces = [
      piece("main/wire_2", 0, total * 0.5),
      piece("main/wire_2", total * 0.5, total, true)
    ];
    const result = materializeWirePattern(samples, pieces, () => asset);
    const points = Array.from({ length: result.length / 3 }, (_, index) =>
      new THREE.Vector3(result[index * 3], result[index * 3 + 1], result[index * 3 + 2]));

    expect(points[0].distanceTo(new THREE.Vector3(0, 0, 0))).toBeLessThan(1e-8);
    expect(points.at(-1)!.distanceTo(new THREE.Vector3(6, 0, 0))).toBeLessThan(1e-8);
    const joinCount = points.filter((point) => point.distanceTo(new THREE.Vector3(3, 0, -0.5)) < 1e-8).length;
    expect(joinCount).toBe(1);
  });
});
