import { describe, expect, it } from "vitest";
import { buildIdentitiesMatch } from "../src/buildInfo";

describe("web and wasm build identity", () => {
  it("accepts only the same commit and package version outside dev mode", () => {
    const web = { commit: "abc123", packageVersion: "0.2.0" };
    expect(buildIdentitiesMatch(web, { commit: "abc123", version: "0.2.0" })).toBe(true);
    expect(buildIdentitiesMatch(web, { commit: "def456", version: "0.2.0" })).toBe(false);
    expect(buildIdentitiesMatch(web, { commit: "abc123", version: "0.3.0" })).toBe(false);
  });

  it("allows the unbundled test environment to load a real wasm module", () => {
    expect(buildIdentitiesMatch(
      { commit: "dev", packageVersion: "0.2.0" },
      { commit: "any-local-build", version: "0.2.0" }
    )).toBe(true);
  });
});
