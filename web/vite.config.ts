import { defineConfig } from "vite";
import type { Plugin, ViteDevServer } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";
import { execSync, spawn } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import packageJson from "./package.json" with { type: "json" };

const webRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)));
const repoRoot = path.resolve(webRoot, "..");
const generatedRoot = path.join(webRoot, "src", "wasm-generated");
const manifestPath = path.join(generatedRoot, "source-hash.txt");
const wasmBuildScript = path.join(webRoot, "scripts", "wasm-nmake-build.cmd");
const wasmWatchRoots = [
  path.join(repoRoot, "CMakeLists.txt"),
  path.join(repoRoot, "domains", "wire", "CMakeLists.txt"),
  path.join(repoRoot, "domains", "wire", "include"),
  path.join(repoRoot, "domains", "wire", "src"),
  path.join(repoRoot, "domains", "road", "CMakeLists.txt"),
  path.join(repoRoot, "domains", "road", "include"),
  path.join(repoRoot, "domains", "road", "src"),
  path.join(webRoot, "wasm"),
  path.join(webRoot, "scripts", "wasm-nmake-build.cmd")
];

function readGitValue(command: string, fallback: string): string {
  try {
    return execSync(command, { encoding: "utf8", stdio: ["ignore", "pipe", "ignore"] }).trim() || fallback;
  } catch {
    return fallback;
  }
}

function computeWasmSourceHash(): string {
  return readGitValue("node scripts/wasm-source-hash.mjs", "unknown");
}

export function buildWireBuildInfo(): {
  commit: string;
  wasmSourceHash: string;
  builtAt: string;
  packageVersion: string;
} {
  return {
    commit: readGitValue("git rev-parse --short=12 HEAD", "unknown"),
    wasmSourceHash: computeWasmSourceHash(),
    builtAt: new Date().toISOString(),
    packageVersion: packageJson.version
  };
}

function readGeneratedWasmSourceHash(): string {
  return existsSync(manifestPath) ? readFileSync(manifestPath, "utf8").trim() : "";
}

export function isWasmSourcePath(changedPath: string): boolean {
  const normalized = path.resolve(changedPath);
  if (normalized.startsWith(path.join(webRoot, "src", "wasm-generated"))) return false;
  if (normalized.startsWith(path.join(webRoot, "wasm", "build-nmake"))) return false;
  return wasmWatchRoots.some((root) => normalized === root || normalized.startsWith(`${root}${path.sep}`));
}

function wireWasmDevSyncPlugin(): Plugin {
  let debounce: NodeJS.Timeout | null = null;
  let building = false;
  let queued = false;

  function scheduleBuild(server: ViteDevServer, changedPath: string): void {
    if (!isWasmSourcePath(changedPath)) return;
    if (debounce !== null) clearTimeout(debounce);
    debounce = setTimeout(() => void buildIfStale(server), 500);
  }

  async function buildIfStale(server: ViteDevServer): Promise<void> {
    if (building) {
      queued = true;
      return;
    }
    const expected = computeWasmSourceHash();
    const actual = readGeneratedWasmSourceHash();
    if (actual === expected) return;

    building = true;
    server.config.logger.info(`WASM is stale (${actual || "missing"} -> ${expected}); rebuilding.`);
    try {
      await new Promise<void>((resolve, reject) => {
        const child = spawn(wasmBuildScript, ["build"], {
          cwd: webRoot,
          stdio: "inherit",
          shell: true
        });
        child.on("error", reject);
        child.on("exit", (code) => {
          if (code === 0) resolve();
          else reject(new Error(`WASM build failed with exit code ${code ?? "unknown"}`));
        });
      });
      server.config.logger.info("WASM rebuilt; reloading the browser.");
      server.ws.send({ type: "full-reload" });
    } catch (error) {
      server.config.logger.error(error instanceof Error ? error.message : String(error));
    } finally {
      building = false;
      if (queued) {
        queued = false;
        await buildIfStale(server);
      }
    }
  }

  return {
    name: "wire-wasm-dev-sync",
    apply: "serve",
    configureServer(server) {
      server.watcher.add(wasmWatchRoots);
      server.watcher.on("add", (changedPath) => scheduleBuild(server, changedPath));
      server.watcher.on("change", (changedPath) => scheduleBuild(server, changedPath));
      server.watcher.on("unlink", (changedPath) => scheduleBuild(server, changedPath));
      server.middlewares.use("/__wire_build_info", (_request, response) => {
        response.setHeader("content-type", "application/json");
        response.setHeader("cache-control", "no-store");
        response.end(JSON.stringify(buildWireBuildInfo()));
      });
    }
  };
}

const buildInfo = buildWireBuildInfo();

export default defineConfig({
  base: "/wire/",
  define: {
    __WIRE_BUILD_INFO__: JSON.stringify(buildInfo)
  },
  plugins: [wireWasmDevSyncPlugin(), svelte()]
});
