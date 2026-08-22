import { existsSync, readFileSync } from "node:fs";
import { resolve } from "node:path";

import { describe, expect, it } from "vitest";

function repoPath(path: string): string {
  return resolve(process.cwd(), path);
}

function text(path: string): string {
  return readFileSync(repoPath(path), "utf8");
}

describe("wire architecture boundary", () => {
  it("keeps decoration semantic generation out of the web bridge and viewer", () => {
    expect(existsSync(repoPath("src/bridge/supportDetails.ts"))).toBe(false);

    const bridge = text("src/bridge/wire.ts");
    const scene = text("src/render/scene.ts");
    const banned = [
      "deriveSupportDetails",
      "PresentationAttachment",
      "PoleDetailInput",
      "nearestPoleDetailInput",
      "populatePoleAttachments",
      "transformer_basic",
      "transformerBasicRecipe",
      "curvedRoutePoints",
      "smoothLocalCablePoints",
      "pushCable"
    ];
    for (const token of banned) {
      expect(bridge, `bridge contains ${token}`).not.toContain(token);
      expect(scene, `scene contains ${token}`).not.toContain(token);
    }
  });

  it("does not override core wire material semantics by supplemental kind", () => {
    const scene = text("src/render/scene.ts");
    expect(scene).not.toContain("SUPPORT_PATH_SUPPLEMENTAL_KIND");
    expect(scene).not.toContain("getSupportWireMaterial");
  });

  it("does not retain the removed road lane connection editor", () => {
    const road = text("src/road.ts");
    expect(road).toContain(
      'export type RoadOperation = "draw" | "edit" | "delete" | "add-lane"'
    );
    expect(road).not.toMatch(/laneEditStage:\s*[^;\n]*["']target["']/);

    const production = [
      "src/road.ts",
      "src/actions/road_actions.ts",
      "src/actions/viewer.ts",
      "src/panels/RoadPanel.svelte",
      "src/render/scene.ts"
    ].map(text).join("\n");
    for (const token of [
      "selectedLaneId",
      "selectedLaneEndpointRole",
      "selectedLaneNodeId",
      "laneTargetTemplateId",
      "laneTargetLaneId",
      "laneSourceBoundaryId",
      "laneTargetBoundaryId",
      "setLaneTargetTemplate",
      "setRoadLaneTargetTemplate",
      "road lane connection",
      "lane_endpoint_not_selected"
    ]) {
      expect(production, `production contains removed lane editor token ${token}`)
        .not.toContain(token);
    }
  });
});
