import { describe, expect, it } from "vitest";
import { ReproTrace } from "../src/actions/reproTrace";
import { createViewerSnapshot } from "../src/store/viewer";

describe("repro trace", () => {
  it("keeps draw order and anchor visual coverage without curve samples", () => {
    const before = createViewerSnapshot();
    before.pathPointSpecs = [null, { supportKind: 2, nodeId: "node-7" }];
    before.drawBundlePlacements = [{
      id: 1, bundleTemplateId: 102, count: 3,
      explicit: true, height: 7.4, offset: 0, spacing: 0.2
    }];
    before.selectedPoleTemplateId = 1;
    const after = {
      ...before,
      supportNodes: [{ id: "node-7", kind: 2, poleId: "pole-1", x: 10, y: 2, z: 5 }],
      backboneEdges: [{ nodeAId: "node-7", nodeBId: "node-8", bundleIds: ["bundle-3"] }],
      parts: [{
        info: {
          partKey: "patch:node-7",
          sourceVersion: "1",
          sampleOffset: 0,
          kind: 1, supplementalKind: 0, wireRadius: 0.02, colorRgba: 0xffffffff,
          sourceNodeId: "node-7", sourceEdgeId: "", sourceSpanId: "span-1",
          sourceBundleId: "bundle-3", bundleTemplateId: 102, laneIndex: 0,
          runId: 4, sampleCount: 99
        },
        samples: new Float64Array([0, 0, 0, 999, 999, 999])
      }]
    };
    const trace = new ReproTrace();
    trace.recordPathPoint([0, 0, 0], [0, 0, 0], undefined, null);
    trace.recordPathPoint([10, 2, 5], [10, 2, 5], {
      hitKind: 1, hitId: "span-1", hitX: 10, hitY: 2, hitZ: 5,
      hasSegmentEndpoints: false, segmentNodeAId: "", segmentNodeBId: "",
      segmentEndpointAX: 0, segmentEndpointAY: 0, segmentEndpointAZ: 0,
      segmentEndpointBX: 0, segmentEndpointBY: 0, segmentEndpointBZ: 0
    }, before.pathPointSpecs[1]);
    trace.recordGeneration(before, [[0, 0, 0], [10, 2, 5]], {
      ok: true, error: "", generatedPoleCount: 2, generatedSpanCount: 3,
      totalMs: 1.5,
      timing: {
        prepareMs: 0, checkMs: 0, pairsMs: 0, preflightMs: 0, intentMs: 0,
        supportGroupsMs: 0, emitMs: 0, saveGraphMs: 0, rulesMs: 0,
        layoutMs: 0, geomMs: 0, drawMs: 0, totalMs: 1.5
      }
    }, after);

    const output = trace.toText(after);
    expect(output).toContain("0001 path-point requested=(0.000,0.000,0.000)");
    expect(output).toContain("0002 path-point requested=(10.000,2.000,5.000)");
    expect(output).toContain("anchor=node-7/2");
    expect(output).toContain("anchor node=node-7 kind=2 pole=pole-1");
    expect(output).toContain("visual={NodePatch=1}");
    expect(output).not.toContain("999,999,999");
  });
});
