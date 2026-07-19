import { defineConfig } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";
import { execSync } from "node:child_process";
import packageJson from "./package.json" with { type: "json" };

function readGitValue(command: string, fallback: string): string {
  try {
    return execSync(command, { encoding: "utf8", stdio: ["ignore", "pipe", "ignore"] }).trim() || fallback;
  } catch {
    return fallback;
  }
}

const buildInfo = {
  commit: readGitValue("git rev-parse --short=12 HEAD", "unknown"),
  builtAt: new Date().toISOString(),
  packageVersion: packageJson.version
};

export default defineConfig({
  base: "/wire/",
  define: {
    __WIRE_BUILD_INFO__: JSON.stringify(buildInfo)
  },
  plugins: [svelte()]
});
