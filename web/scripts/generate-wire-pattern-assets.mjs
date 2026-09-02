import fs from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

import * as THREE from "three";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter.js";

const webRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const outputRoot = path.join(webRoot, "src", "assets", "wire-patterns");

if (globalThis.FileReader === undefined) {
  globalThis.FileReader = class {
    result = null;
    error = null;
    onloadend = null;
    onerror = null;

    async readAsArrayBuffer(blob) {
      try {
        this.result = await blob.arrayBuffer();
        this.onloadend?.({ target: this });
      } catch (error) {
        this.error = error;
        this.onerror?.({ target: this });
      }
    }
  };
}

function wireStations(lengthM, variant) {
  const half = lengthM * 0.5;
  const stationCount = Math.max(5, Math.ceil(lengthM * 2) + 1);
  return Array.from({ length: stationCount }, (_, index) => {
    const u = index / (stationCount - 1);
    const x = -half + lengthM * u;
    if (index === 0 || index === stationCount - 1 || variant === 1) return [x, 0, 0];
    const envelope = Math.sin(Math.PI * u);
    return [
      x,
      0.012 * envelope * Math.sin(4 * Math.PI * u + 0.35),
      0.008 * envelope * Math.sin(6 * Math.PI * u + 1.1)
    ];
  });
}

function helixStations(lengthM, variant) {
  const half = lengthM * 0.5;
  const turnsPerMeter = variant === 1 ? 1.75 : 2.0;
  // Helix assets use a unit cross-section. Core supplies the resolved radius
  // as the appearance-piece scale, so the GLB does not become its owner.
  const radius = 1.0;
  const stationCount = Math.max(17, Math.ceil(lengthM * turnsPerMeter * 16) + 1);
  return Array.from({ length: stationCount }, (_, index) => {
    const u = index / (stationCount - 1);
    const x = -half + lengthM * u;
    if (index === 0 || index === stationCount - 1) return [x, 0, 0];
    const envelope = Math.sin(Math.PI * u) ** 2;
    const phase = 2 * Math.PI * turnsPerMeter * lengthM * u;
    return [x, radius * envelope * Math.cos(phase), radius * envelope * Math.sin(phase)];
  });
}

function lineObject(name, stations) {
  const positions = new Float32Array(stations.flat());
  const indices = new Uint32Array((stations.length - 1) * 2);
  for (let index = 0; index + 1 < stations.length; index += 1) {
    indices[index * 2] = index;
    indices[index * 2 + 1] = index + 1;
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  const line = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: 0xffffff }));
  line.name = name;
  return line;
}

function validateStations(name, stations, lengthM) {
  if (stations.length < 5) throw new Error(`${name}: at least five longitudinal stations are required`);
  const half = lengthM * 0.5;
  const first = stations[0];
  const last = stations[stations.length - 1];
  const tolerance = 1e-9;
  if (Math.abs(first[0] + half) > tolerance || Math.abs(last[0] - half) > tolerance) {
    throw new Error(`${name}: authored X bounds do not match ${lengthM}m`);
  }
  if ([first[1], first[2], last[1], last[2]].some((value) => Math.abs(value) > tolerance)) {
    throw new Error(`${name}: endpoints must return to the center line`);
  }
  for (let index = 1; index < stations.length; index += 1) {
    if (!(stations[index][0] > stations[index - 1][0])) {
      throw new Error(`${name}: stations must form one non-branching +X chain`);
    }
  }
}

async function writeGlb(relativeDirectory, name, lengthM, stations) {
  validateStations(name, stations, lengthM);
  const directory = path.join(outputRoot, relativeDirectory);
  await fs.mkdir(directory, { recursive: true });
  const object = lineObject(name, stations);
  object.updateMatrixWorld(true);
  const exporter = new GLTFExporter();
  const data = await exporter.parseAsync(object, { binary: true, onlyVisible: true });
  await fs.writeFile(path.join(directory, `${name}.glb`), Buffer.from(data));
}

for (const [directory, lengthM] of [["main", 4], ["connection", 1]]) {
  for (const variant of [1, 2]) {
    await writeGlb(directory, `wire_${variant}`, lengthM, wireStations(lengthM, variant));
    await writeGlb(directory, `helix_${variant}`, lengthM, helixStations(lengthM, variant));
  }
}

console.log(`Generated wire pattern GLBs in ${outputRoot}`);
