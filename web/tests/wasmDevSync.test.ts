import { describe, expect, it } from "vitest";
import path from "node:path";
import { isWasmSourcePath } from "../vite.config";

const repoRoot = path.resolve("..");
const webRoot = path.resolve(".");

describe("WASM dev server sync", () => {
  it("watches core sources but not generated outputs", () => {
    expect(isWasmSourcePath(path.join(repoRoot, "domains", "road", "src", "generation", "connections.cpp")))
      .toBe(true);
    expect(isWasmSourcePath(path.join(repoRoot, "domains", "wire", "include", "city", "wire", "core.hpp")))
      .toBe(true);
    expect(isWasmSourcePath(path.join(webRoot, "wasm", "bindings.cpp")))
      .toBe(true);

    expect(isWasmSourcePath(path.join(webRoot, "src", "wasm-generated", "wire_web_core.js")))
      .toBe(false);
    expect(isWasmSourcePath(path.join(webRoot, "wasm", "build-nmake", "wire_web_core.wasm")))
      .toBe(false);
    expect(isWasmSourcePath(path.join(repoRoot, "docs", "road", "architecture.md")))
      .toBe(false);
  });
});
