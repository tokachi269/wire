// Bakes pole_transformer_m_01.glb for web/src/assets.
//
// The straps encircle both the tank and the pole, so the pole radius cannot be
// recovered by a radial scale the way belt.glb is: it has to be geometry. The
// radius is therefore taken from the viewer's pole primitive at the nominal
// mounting height and the taper is carried inside the model, which keeps both
// straps correct over the band the transformer is actually placed in.
//
// Run: node tools/models/export_pole_transformer.mjs
// (bundles tools/models/pole_transformer_m_01.ts with the web workspace's esbuild)

import { execFileSync } from "node:child_process";
import fs from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, "..", "..");
const webModules = path.join(root, "web", "node_modules");
const buildDir = path.join(here, ".build");
const entry = path.join(buildDir, "export_entry.mjs");
const bundle = path.join(buildDir, "export_bundle.mjs");

// web/src/render/modelAssets.ts polePrimitive, and tools/create_japan_distribution_pole_primitive.py.
const POLE = { totalLengthM: 12.0, visibleHeightM: 10.0, topDiameterM: 0.190, taperRatio: 75.0 };
const poleRadiusAtHeight = (heightM) =>
  (POLE.topDiameterM + (POLE.visibleHeightM - heightM) / POLE.taperRatio) * 0.5;

// World height of the model origin (the underside of the tank tray).
export const MOUNT_HEIGHT_M = 7.26;
// Tank wall radius at its widest, from TANK_PROFILE.
const TANK_MAX_RADIUS_M = 0.2525;
// Pole face to tank wall, as measured off the reference photo.
const TANK_TO_POLE_GAP_M = 0.0635;

const poleRadius = poleRadiusAtHeight(MOUNT_HEIGHT_M);
const pole = {
  radius: poleRadius,
  taperPerM: -1 / (POLE.taperRatio * 2),
  axisDistance: poleRadius + TANK_TO_POLE_GAP_M + TANK_MAX_RADIUS_M
};

await fs.mkdir(buildDir, { recursive: true });
await fs.writeFile(
  entry,
  `import fs from 'node:fs/promises';
import * as THREE from 'three';
import { GLTFExporter } from 'three/examples/jsm/exporters/GLTFExporter.js';
import { createTransformerM01, triangleCount } from ${JSON.stringify(path.join(here, "pole_transformer_m_01.ts").replace(/\\/g, "/"))};

if (globalThis.FileReader === undefined) {
  globalThis.FileReader = class {
    result = null; error = null; onloadend = null; onerror = null;
    async readAsArrayBuffer(blob) {
      try { this.result = await blob.arrayBuffer(); this.onloadend?.({ target: this }); }
      catch (e) { this.error = e; this.onerror?.({ target: this }); }
    }
  };
}

const pole = ${JSON.stringify(pole)};
const model = createTransformerM01({ pole });
// The arm metal comes from the existing textured crossarm asset, and the
// arrester, end box and knob insulator are separate pole fittings.
const arm = model.getObjectByName('arm');
if (arm === undefined) throw new Error('arm group missing');
model.remove(arm);
if (model.children.length !== 2) throw new Error('expected body and mount only');
// The authored stub is driven into the pole; the placement now carries the tank
// on two arms that clamp the pole instead, so nothing may pierce it.
const stub = model.getObjectByName('stub');
if (stub === undefined) throw new Error('stub mesh missing');
stub.removeFromParent();

const scene = new THREE.Scene();
scene.name = 'pole_transformer_m_01';
scene.add(model);
const glb = await new GLTFExporter().parseAsync(scene, { binary: true, onlyVisible: false });
await fs.writeFile(process.argv[2], Buffer.from(glb));

const sockets = [];
model.traverse((o) => { if (o.name.startsWith('socket_')) sockets.push([o.name, o.position.toArray()]); });
console.log(JSON.stringify({
  triangles: triangleCount(model),
  groups: model.children.map((g) => [g.name, triangleCount(g)]),
  bytes: glb.byteLength,
  pole,
  sockets
}, null, 1));
`,
  "utf8"
);

const env = { ...process.env, NODE_PATH: webModules };
// The .bin shim is a .cmd on Windows, which execFileSync cannot spawn directly.
const [esbuild] = await fs.readdir(path.join(webModules, "@esbuild")).then((dirs) =>
  dirs.map((d) => path.join(webModules, "@esbuild", d, process.platform === "win32" ? "esbuild.exe" : "bin/esbuild"))
);
execFileSync(
  esbuild,
  [entry, "--bundle", "--platform=node", "--format=esm", `--outfile=${bundle}`, "--log-level=warning"],
  { stdio: "inherit", env }
);
const out = path.join(root, "web", "src", "assets", "pole_transformer_m_01.glb");
execFileSync(process.execPath, [bundle, out], { stdio: "inherit", env });
console.log(`wrote ${out}`);
