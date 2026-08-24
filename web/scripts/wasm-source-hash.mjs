import { createHash } from "node:crypto";
import { readdirSync, readFileSync, statSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const scriptPath = fileURLToPath(import.meta.url);
const webRoot = path.resolve(path.dirname(scriptPath), "..");
const repoRoot = path.resolve(webRoot, "..");
const sourceRoots = [
  "domains/wire/include",
  "domains/wire/src",
  "domains/road/include",
  "domains/road/src",
  "web/wasm"
];
const sourceFiles = [
  "CMakeLists.txt",
  "domains/wire/CMakeLists.txt",
  "domains/road/CMakeLists.txt",
  "web/scripts/wasm-nmake-build.cmd"
];
const excludedDirectories = new Set(["build", "build-nmake"]);

function collectFiles(directory, output) {
  for (const entry of readdirSync(directory, { withFileTypes: true })) {
    if (entry.isDirectory() && excludedDirectories.has(entry.name)) continue;
    const absolute = path.join(directory, entry.name);
    if (entry.isDirectory()) collectFiles(absolute, output);
    else if (entry.isFile()) output.push(absolute);
  }
}

export function computeWasmSourceHash() {
  const files = sourceFiles.map((file) => path.join(repoRoot, file));
  for (const root of sourceRoots) collectFiles(path.join(repoRoot, root), files);
  files.sort((a, b) => a.localeCompare(b));

  const hash = createHash("sha256");
  for (const absolute of files) {
    if (!statSync(absolute).isFile()) continue;
    hash.update(path.relative(repoRoot, absolute).replaceAll("\\", "/"));
    hash.update("\0");
    hash.update(readFileSync(absolute));
    hash.update("\0");
  }
  return hash.digest("hex").slice(0, 12);
}

if (process.argv[1] && path.resolve(process.argv[1]) === scriptPath) {
  process.stdout.write(`${computeWasmSourceHash()}\n`);
}
