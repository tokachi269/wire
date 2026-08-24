import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import { computeWasmSourceHash } from "./wasm-source-hash.mjs";

const webRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const generated = path.join(webRoot, "src", "wasm-generated");
const manifest = path.join(generated, "source-hash.txt");
const expected = computeWasmSourceHash();
const actual = existsSync(manifest) ? readFileSync(manifest, "utf8").trim() : "";
const outputsExist = existsSync(path.join(generated, "wire_web_core.js")) &&
  existsSync(path.join(generated, "wire_web_core.wasm"));

if (outputsExist && actual === expected) {
  console.log(`WASM is current (${expected}).`);
  process.exit(0);
}

console.log(`WASM is stale (${actual || "missing"} -> ${expected}); rebuilding once.`);
const built = spawnSync(
  path.join(webRoot, "scripts", "wasm-nmake-build.cmd"),
  ["build"],
  { cwd: webRoot, stdio: "inherit", shell: true }
);
process.exit(built.status ?? 1);
