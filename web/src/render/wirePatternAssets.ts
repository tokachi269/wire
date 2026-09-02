import * as THREE from "three";
import type { VisualCurveAppearancePieceInfo } from "../model";
import connectionHelix1Url from "../assets/wire-patterns/connection/helix_1.glb?url";
import connectionHelix2Url from "../assets/wire-patterns/connection/helix_2.glb?url";
import connectionWire1Url from "../assets/wire-patterns/connection/wire_1.glb?url";
import connectionWire2Url from "../assets/wire-patterns/connection/wire_2.glb?url";
import mainHelix1Url from "../assets/wire-patterns/main/helix_1.glb?url";
import mainHelix2Url from "../assets/wire-patterns/main/helix_2.glb?url";
import mainWire1Url from "../assets/wire-patterns/main/wire_1.glb?url";
import mainWire2Url from "../assets/wire-patterns/main/wire_2.glb?url";
import { loadGltfScene } from "./modelAssets";

export interface WirePatternAsset {
  points: THREE.Vector3[];
  minX: number;
  maxX: number;
}

const wirePatternUrls: Readonly<Record<string, string>> = {
  "connection/helix_1": connectionHelix1Url,
  "connection/helix_2": connectionHelix2Url,
  "connection/wire_1": connectionWire1Url,
  "connection/wire_2": connectionWire2Url,
  "main/helix_1": mainHelix1Url,
  "main/helix_2": mainHelix2Url,
  "main/wire_1": mainWire1Url,
  "main/wire_2": mainWire2Url
};

export function extractLooseEdgeChain(root: THREE.Object3D): WirePatternAsset {
  root.updateMatrixWorld(true);
  const points: THREE.Vector3[] = [];
  const edges: Array<[number, number]> = [];
  root.traverse((object) => {
    if (!(object instanceof THREE.LineSegments)) return;
    const positions = object.geometry.getAttribute("position");
    if (!(positions instanceof THREE.BufferAttribute)) {
      throw new Error("wire pattern LINES primitive has no position attribute");
    }
    const offset = points.length;
    for (let index = 0; index < positions.count; index += 1) {
      points.push(new THREE.Vector3(
        positions.getX(index), positions.getY(index), positions.getZ(index)
      ).applyMatrix4(object.matrixWorld));
    }
    const indices = object.geometry.getIndex();
    const count = indices?.count ?? positions.count;
    if (count % 2 !== 0) throw new Error("wire pattern LINES primitive has an odd index count");
    for (let index = 0; index < count; index += 2) {
      const a = indices?.getX(index) ?? index;
      const b = indices?.getX(index + 1) ?? index + 1;
      edges.push([offset + a, offset + b]);
    }
  });
  if (edges.length === 0) throw new Error("wire pattern contains no glTF LINES primitive");

  const adjacency = Array.from({ length: points.length }, () => [] as number[]);
  for (const [a, b] of edges) {
    adjacency[a].push(b);
    adjacency[b].push(a);
  }
  const endpoints = adjacency
    .map((neighbors, index) => ({ neighbors, index }))
    .filter(({ neighbors }) => neighbors.length === 1);
  if (endpoints.length !== 2 || adjacency.some((neighbors) => neighbors.length < 1 || neighbors.length > 2)) {
    throw new Error("wire pattern must be one non-branching loose-edge chain");
  }

  const ordered: THREE.Vector3[] = [];
  let previous = -1;
  let current = endpoints[0].index;
  while (current >= 0) {
    ordered.push(points[current]);
    const next = adjacency[current].find((candidate) => candidate !== previous) ?? -1;
    previous = current;
    current = next;
  }
  if (ordered.length !== edges.length + 1) {
    throw new Error("wire pattern LINES primitive contains multiple chains or a cycle");
  }
  if (ordered[0].x > ordered[ordered.length - 1].x) ordered.reverse();
  const minX = Math.min(...ordered.map((point) => point.x));
  const maxX = Math.max(...ordered.map((point) => point.x));
  if (!(maxX > minX)) throw new Error("wire pattern has no positive local X extent");
  return { points: ordered, minX, maxX };
}

class WirePatternAssetCache {
  private readonly loaded = new Map<string, WirePatternAsset>();

  async loadAll(): Promise<void> {
    await Promise.all(Object.entries(wirePatternUrls).map(async ([key, url]) => {
      const scene = await loadGltfScene(url);
      this.loaded.set(key, extractLooseEdgeChain(scene));
    }));
  }

  asset(key: string): WirePatternAsset {
    const asset = this.loaded.get(key);
    if (asset === undefined) throw new Error(`wire pattern asset is not loaded: ${key}`);
    return asset;
  }
}

interface CurveTable {
  points: THREE.Vector3[];
  distances: number[];
  total: number;
}

function curveTable(samples: Float64Array): CurveTable {
  const points: THREE.Vector3[] = [];
  const distances: number[] = [];
  let total = 0;
  for (let index = 0; index + 2 < samples.length; index += 3) {
    const point = new THREE.Vector3(samples[index], samples[index + 1], samples[index + 2]);
    if (points.length > 0) total += point.distanceTo(points[points.length - 1]);
    points.push(point);
    distances.push(total);
  }
  if (points.length < 2 || !(total > 0)) throw new Error("wire curve requires two distinct Core samples");
  return { points, distances, total };
}

function sampleCurve(table: CurveTable, requestedDistance: number): { point: THREE.Vector3; tangent: THREE.Vector3 } {
  const distance = THREE.MathUtils.clamp(requestedDistance, 0, table.total);
  let high = 1;
  while (high + 1 < table.distances.length && table.distances[high] < distance) high += 1;
  const low = high - 1;
  const segmentLength = table.distances[high] - table.distances[low];
  const u = segmentLength > 0 ? (distance - table.distances[low]) / segmentLength : 0;
  const tangent = table.points[high].clone().sub(table.points[low]).normalize();
  return { point: table.points[low].clone().lerp(table.points[high], u), tangent };
}

export function deformWirePattern(
  asset: WirePatternAsset,
  coreSamples: Float64Array,
  piece: VisualCurveAppearancePieceInfo
): THREE.Vector3[] {
  const table = curveTable(coreSamples);
  const sourceExtent = asset.maxX - asset.minX;
  if (Math.abs(sourceExtent - piece.sourceLengthM) > 1e-5) {
    throw new Error(`wire pattern source length mismatch: ${piece.assetKey}`);
  }
  if (piece.curveEndM - piece.curveStartM > piece.sourceLengthM + 1e-9) {
    throw new Error(`wire pattern piece would stretch its source asset: ${piece.assetKey}`);
  }
  const source = piece.reverse ? [...asset.points].reverse() : asset.points;
  let previousLateral: THREE.Vector3 | null = null;
  return source.map((authored) => {
    const localX = piece.reverse ? -authored.x : authored.x;
    const localY = piece.reverse ? -authored.y : authored.y;
    const u = THREE.MathUtils.clamp((localX - asset.minX) / sourceExtent, 0, 1);
    const sampled = sampleCurve(table, piece.curveStartM +
      (piece.curveEndM - piece.curveStartM) * u);
    let lateral = new THREE.Vector3(0, 0, 1).cross(sampled.tangent);
    if (lateral.lengthSq() <= 1e-12) {
      lateral = previousLateral?.clone() ?? new THREE.Vector3(1, 0, 0).cross(sampled.tangent);
    }
    if (lateral.lengthSq() <= 1e-12) lateral = new THREE.Vector3(0, 1, 0);
    lateral.normalize();
    previousLateral = lateral.clone();
    const up = sampled.tangent.clone().cross(lateral).normalize();
    return sampled.point
      .addScaledVector(lateral, localY * piece.localOffsetScaleM)
      .addScaledVector(up, authored.z * piece.localOffsetScaleM);
  });
}

export function materializeWirePattern(
  coreSamples: Float64Array,
  pieces: VisualCurveAppearancePieceInfo[],
  resolveAsset: (key: string) => WirePatternAsset = (key) => wirePatternAssetCache.asset(key)
): Float64Array {
  const result: number[] = [];
  for (const piece of pieces) {
    const deformed = deformWirePattern(resolveAsset(piece.assetKey), coreSamples, piece);
    for (let index = 0; index < deformed.length; index += 1) {
      const point = deformed[index];
      const previousOffset = result.length - 3;
      if (index === 0 && previousOffset >= 0 &&
          Math.hypot(result[previousOffset] - point.x, result[previousOffset + 1] - point.y,
            result[previousOffset + 2] - point.z) <= 1e-9) {
        continue;
      }
      result.push(point.x, point.y, point.z);
    }
  }
  return new Float64Array(result);
}

export const wirePatternAssetCache = new WirePatternAssetCache();
