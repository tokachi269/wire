import { describe, expect, it } from "vitest";
import { buildIdentitiesMatch } from "../src/buildInfo";

describe("web and wasm build identity", () => {
  it("matches the wasm source fingerprint instead of the enclosing commit", () => {
    const web = { commit: "new-commit", wasmSourceHash: "source123", packageVersion: "0.2.0" };
    expect(buildIdentitiesMatch(web, {
      sourceHash: "source123", version: "0.2.0"
    })).toBe(true);
    expect(buildIdentitiesMatch(web, {
      sourceHash: "stale456", version: "0.2.0"
    })).toBe(false);
    expect(buildIdentitiesMatch(web, {
      sourceHash: "source123", version: "0.3.0"
    })).toBe(false);
  });

  it("allows the unbundled test environment to load a real wasm module", () => {
    expect(buildIdentitiesMatch(
      { commit: "dev", wasmSourceHash: "dev", packageVersion: "0.2.0" },
      { sourceHash: "any-local-build", version: "0.2.0" }
    )).toBe(true);
  });
});
