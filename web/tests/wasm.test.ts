import { afterAll, beforeAll, describe, expect, it } from "vitest";
import * as THREE from "three";
import { loadWireModule, type RoadStateHandle, type WireStateHandle } from "../src/bridge/wasm";
import { WireBridge, type SceneData } from "../src/bridge/wire";
import type { RoadSectionInput } from "../src/road_templates";
import { ROAD_TEMPLATE_PRESETS, seedRoadSections } from "../src/road_templates";
import { CommitFailureCategory, type BundlePlacement, type ModelAssemblyBootstrapInput, type ModelTransformInput } from "../src/model";
import {
  buildDefaultModelBootstrap,
  ModelAssetCache
} from "../src/render/modelAssets";
import { missingBackboneEntryCells } from "./backbone_semantics_contract";
import {
  SUPPORT_DETAIL_SCENE_BUNDLE_TEMPLATE_IDS,
  SUPPORT_DETAIL_SCENE_COUNTS,
  SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS,
  SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS,
  SUPPORT_DETAIL_SCENE_POINTS
} from "./supportDetailScene";

function visualParts(state: WireStateHandle) {
  const scene = state.visualScene();
  const samples = new Float64Array(scene.samples);
  return scene.parts.map((info) => ({
    info,
    samples: samples.subarray(info.sampleOffset, info.sampleOffset + info.sampleCount * 3)
  }));
}

function detailSceneSignature(scene: SceneData): string {
  const records = scene.parts
    .filter((part) =>
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.localCable ||
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.inlineCable
    )
    .map((part) => [
      "part",
      part.info.partKey,
      part.info.supplementalKind,
      Array.from(part.samples).map((value) => value.toPrecision(17)).join(",")
    ].join(":"));
  records.push(...scene.models
    .filter((model) => model.modelKey.startsWith("detail_"))
    .map((model) => [
      "model",
      model.stableKey,
      model.modelKey,
      model.positionX.toPrecision(17),
      model.positionY.toPrecision(17),
      model.positionZ.toPrecision(17),
      model.rotationX.toPrecision(17),
      model.rotationY.toPrecision(17),
      model.rotationZ.toPrecision(17),
      model.scaleX.toPrecision(17),
      model.scaleY.toPrecision(17),
      model.scaleZ.toPrecision(17)
    ].join(":")));
  return records.sort().join("\n");
}

const identityTransform = (): ModelTransformInput => ({
  positionX: 0, positionY: 0, positionZ: 0,
  rotationX: 0, rotationY: 0, rotationZ: 0,
  scaleX: 1, scaleY: 1, scaleZ: 1
});

function modelBootstrap(): ModelAssemblyBootstrapInput {
  return {
    assemblies: [{
      id: 9901,
      version: 1,
      parts: [{
        partId: 1, modelKey: "pole_body", descriptorName: "pole", descriptorVersion: 1,
        fitMode: 1, localTransform: identityTransform(), sockets: []
      }],
      wireSocket: null
    }, {
      id: 9902,
      version: 1,
      parts: [{
        partId: 1, modelKey: "hv_crossarm", descriptorName: "crossarm", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(), sockets: [{
          name: "endpoint_mount",
          positionX: 0, positionY: 0, positionZ: 0.12,
          directionX: 0, directionY: 0, directionZ: 1
        }]
      }, {
        partId: 2, modelKey: "pole_belt", descriptorName: "belt", descriptorVersion: 1,
        fitMode: 2, localTransform: identityTransform(), sockets: []
      }],
      wireSocket: null,
      endpointMountSocket: { partId: 1, socketName: "endpoint_mount" }
    }, {
      id: 9903,
      version: 1,
      parts: [{
        partId: 1, modelKey: "hv_insulator", descriptorName: "insulator", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(), sockets: [{
          name: "wire",
          positionX: 0, positionY: 0, positionZ: 0.18,
          directionX: 1, directionY: 0, directionZ: 0
        }]
      }],
      wireSocket: { partId: 1, socketName: "wire" }
    }, {
      id: 9904,
      version: 1,
      parts: [{
        partId: 1, modelKey: "communication_clamp", descriptorName: "clamp", descriptorVersion: 1,
        fitMode: 0, localTransform: identityTransform(), sockets: [{
          name: "wire",
          positionX: 0, positionY: 0.3, positionZ: 0,
          directionX: 0, directionY: 1, directionZ: 0
        }]
      }],
      wireSocket: { partId: 1, socketName: "wire" }
    }],
    poleAssignments: [
      { poleTypeId: 1, assemblyId: 9901, radiusBaseM: 0.16, radiusTopM: 0.10 },
      { poleTypeId: 2, assemblyId: 9901, radiusBaseM: 0.16, radiusTopM: 0.10 }
    ],
    bundleAssignments: [
      { bundleTemplateId: 101, rowAssemblyId: 9902, endpointAssemblyId: 9903 },
      { bundleTemplateId: 104, rowAssemblyId: 0, endpointAssemblyId: 9904 }
    ]
  };
}

async function productionLikeModelBootstrap(): Promise<ModelAssemblyBootstrapInput> {
  const load = async () => {
    const geometry = new THREE.BoxGeometry(0.1, 0.2, 0.4);
    const source = new THREE.Group();
    source.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial()));
    return source;
  };
  const cache = new ModelAssetCache(load);
  const [pole, crossarm, belt, insulator, clamp, clampLong] = await Promise.all([
    cache.load("poleBody"),
    cache.load("crossarmHv"),
    cache.load("belt"),
    cache.load("hvInsulator"),
    cache.load("communicationClamp"),
    cache.load("communicationClampLong")
  ]);
  return buildDefaultModelBootstrap(pole, crossarm, belt, insulator, clamp, clampLong);
}

function defaultBundlePlacements(): BundlePlacement[] {
  return [
    { id: 1, bundleTemplateId: 101, count: 3, explicit: true, height: 9.2, offset: -0.2, spacing: 0.45 },
    { id: 2, bundleTemplateId: 102, count: 1, explicit: true, height: 7.7, offset: 0, spacing: 0.2 },
    { id: 3, bundleTemplateId: 102, count: 1, explicit: true, height: 7.35, offset: 0, spacing: 0.2 },
    { id: 4, bundleTemplateId: 102, count: 1, explicit: true, height: 7.0, offset: 0, spacing: 0.2 },
    { id: 5, bundleTemplateId: 104, count: 1, explicit: true, height: 5.5, offset: 0, spacing: 0.2 },
    { id: 6, bundleTemplateId: 105, count: 1, explicit: true, height: 5.3, offset: 0, spacing: 0.2 }
  ];
}

function hvBundlePlacement(): BundlePlacement[] {
  return [defaultBundlePlacements()[0]];
}

function uniqueRounded(values: number[]): number[] {
  return [...new Set(values.map((value) => Math.round(value * 1000) / 1000))];
}

function hvEdgeBodies(state: WireStateHandle) {
  return visualParts(state).filter((part) =>
    part.info.kind === 0 && part.info.bundleTemplateId === 101
  );
}

function startPoint(part: ReturnType<typeof visualParts>[number]): [number, number, number] {
  return [part.samples[0], part.samples[1], part.samples[2]];
}

function endPoint(part: ReturnType<typeof visualParts>[number]): [number, number, number] {
  const offset = part.samples.length - 3;
  return [part.samples[offset], part.samples[offset + 1], part.samples[offset + 2]];
}

function distance(a: [number, number, number], b: [number, number, number]): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function assertSeparatedPoints(points: Array<[number, number, number]>, minDistance: number) {
  for (let index = 0; index < points.length; index += 1) {
    for (let other = index + 1; other < points.length; other += 1) {
      expect(distance(points[index], points[other])).toBeGreaterThan(minDistance);
    }
  }
}

function assertStraightHvSpacing(parts: ReturnType<typeof visualParts>) {
  expect(parts).toHaveLength(3);
  const sorted = [...parts].sort((a, b) => a.info.laneIndex - b.info.laneIndex);
  expect(sorted.map((part) => part.info.laneIndex)).toEqual([0, 1, 2]);
  expect(new Set(sorted.map((part) => part.info.partKey)).size).toBe(3);
  expect(new Set(sorted.map((part) => part.info.sourceSpanId)).size).toBe(3);
  expect(sorted.map((part) => Number(startPoint(part)[1].toFixed(3)))).toEqual([-0.65, -0.2, 0.25]);
  expect(sorted.map((part) => Number(endPoint(part)[1].toFixed(3)))).toEqual([-0.65, -0.2, 0.25]);
  expect(Number((startPoint(sorted[1])[1] - startPoint(sorted[0])[1]).toFixed(3))).toBe(0.45);
  expect(Number((startPoint(sorted[2])[1] - startPoint(sorted[1])[1]).toFixed(3))).toBe(0.45);
  expect(Number((endPoint(sorted[1])[1] - endPoint(sorted[0])[1]).toFixed(3))).toBe(0.45);
  expect(Number((endPoint(sorted[2])[1] - endPoint(sorted[1])[1]).toFixed(3))).toBe(0.45);
}

function modelPosition(model: { positionX: number; positionY: number; positionZ: number }): [number, number, number] {
  return [model.positionX, model.positionY, model.positionZ];
}

function assertThreeYSpacing(points: Array<[number, number, number]>, spacing: number) {
  expect(points).toHaveLength(3);
  const ys = points.map((point) => Number(point[1].toFixed(3))).sort((a, b) => a - b);
  expect(Number((ys[1] - ys[0]).toFixed(3))).toBe(spacing);
  expect(Number((ys[2] - ys[1]).toFixed(3))).toBe(spacing);
}

function assertModelHvInsulatorsSeparatedByPole(state: WireStateHandle) {
  const ports = new Map<string, { ownerPoleId: string }>();
  for (let index = 0; index < state.portCount(); index += 1) {
    const port = state.port(index);
    ports.set(port.id, { ownerPoleId: port.ownerPoleId });
  }
  const groups = new Map<string, Array<[number, number, number]>>();
  for (const model of state.visualScene().models) {
    if (model.modelKey !== "hv_insulator") continue;
    const match = /^port:([^:]+):/.exec(model.stableKey);
    expect(match, model.stableKey).not.toBeNull();
    const port = ports.get(match![1]);
    expect(port, model.stableKey).toBeDefined();
    const group = groups.get(port!.ownerPoleId) ?? [];
    group.push(modelPosition(model));
    groups.set(port!.ownerPoleId, group);
  }
  expect(groups.size).toBeGreaterThan(0);
  for (const points of groups.values()) {
    expect(points.length % 3).toBe(0);
    assertSeparatedPoints(points, 0.05);
  }
}

function expectBootstrapInsulatorSocketsOnStraightHvEndpoints(
  hvParts: ReturnType<typeof visualParts>,
  models: ReturnType<WireStateHandle["visualScene"]>["models"]
) {
  const sortedParts = [...hvParts].sort((a, b) => a.info.laneIndex - b.info.laneIndex);
  const crossarms = models
    .filter((model) => model.modelKey === "hv_crossarm")
    .sort((a, b) => a.positionX - b.positionX);
  expect(crossarms).toHaveLength(2);
  const assertPole = (poleX: number, partsPoint: typeof startPoint) => {
    const crossarm = crossarms.find((model) => Math.abs(model.positionX - poleX) < 1e-6);
    expect(crossarm).toBeDefined();
    const insulators = models
      .filter((model) => model.modelKey === "hv_insulator" && Math.abs(model.positionX - poleX) < 1e-6)
      .sort((a, b) => a.positionY - b.positionY);
    expect(insulators).toHaveLength(3);
    for (let index = 0; index < sortedParts.length; index += 1) {
      const endpoint = partsPoint(sortedParts[index]);
      const insulator = insulators[index];
      expect(Number((insulator.positionZ + 0.18).toFixed(6))).toBe(Number(endpoint[2].toFixed(6)));
      expect(Number((insulator.positionZ - crossarm!.positionZ).toFixed(6))).toBe(0.12);
      expect(distance(modelPosition(insulator), modelPosition(crossarm!))).toBeGreaterThan(0.05);
    }
  };
  assertPole(0, startPoint);
  assertPole(12, endPoint);
}

function projectionLaneOrder(
  group: ReturnType<typeof visualParts>,
  point: (part: ReturnType<typeof visualParts>[number]) => [number, number, number]
): number[] {
  const sorted = [...group].sort((a, b) => a.info.laneIndex - b.info.laneIndex);
  const centerStart = startPoint(sorted[1]);
  const centerEnd = endPoint(sorted[1]);
  const dx = centerEnd[0] - centerStart[0];
  const dy = centerEnd[1] - centerStart[1];
  const length = Math.hypot(dx, dy);
  expect(length).toBeGreaterThan(1e-9);
  const lateral: [number, number, number] = [-dy / length, dx / length, 0];
  return sorted
    .map((part) => ({
      lane: part.info.laneIndex,
      projection: point(part)[0] * lateral[0] + point(part)[1] * lateral[1]
    }))
    .sort((a, b) => a.projection - b.projection)
    .map((item) => item.lane);
}

function assertHvSeparatedByEdge(state: WireStateHandle) {
  const groups = new Map<string, ReturnType<typeof visualParts>>();
  for (const part of hvEdgeBodies(state)) {
    const existing = groups.get(part.info.sourceEdgeId) ?? [];
    existing.push(part);
    groups.set(part.info.sourceEdgeId, existing);
  }
  expect(groups.size).toBeGreaterThan(0);
  for (const group of groups.values()) {
    expect(group).toHaveLength(3);
    expect(new Set(group.map((part) => part.info.laneIndex))).toEqual(new Set([0, 1, 2]));
    expect(new Set(group.map((part) => part.info.partKey)).size).toBe(3);
    expect(new Set(group.map((part) => part.info.sourceSpanId)).size).toBe(3);
    assertSeparatedPoints(group.map(startPoint), 0.1);
    assertSeparatedPoints(group.map(endPoint), 0.1);
    expect(projectionLaneOrder(group, startPoint)).toEqual(projectionLaneOrder(group, endPoint));
  }
}

function assertNonFlaggedSpanLayoutsAtPortHeight(state: WireStateHandle) {
  const ports = new Map<string, { z: number }>();
  for (let index = 0; index < state.portCount(); index += 1) {
    const port = state.port(index);
    ports.set(port.id, { z: port.z });
  }
  const templates = new Map<number, { enableBranchDownOffset: boolean; branchEndpointOffset: number }>();
  for (let index = 0; index < state.bundleTemplateCount(); index += 1) {
    const template = state.bundleTemplate(index);
    templates.set(template.id, {
      enableBranchDownOffset: template.enableBranchDownOffset,
      branchEndpointOffset: template.branchEndpointOffset
    });
  }
  for (let index = 0; index < state.spanCount(); index += 1) {
    const span = state.span(index);
    const part = visualParts(state).find((candidate) =>
      candidate.info.kind === 0 && candidate.info.sourceSpanId === span.id
    );
    if (part === undefined) continue;
    const template = templates.get(part.info.bundleTemplateId);
    const branchDownEnabled =
      template !== undefined &&
      template.enableBranchDownOffset &&
      template.branchEndpointOffset < -1e-9;
    if (branchDownEnabled) continue;
    const layout = state.spanLayout(span.id);
    expect(layout.ok, layout.error).toBe(true);
    expect(layout.start).not.toBeNull();
    expect(layout.end).not.toBeNull();
    for (const endpoint of [layout.start!, layout.end!]) {
      const port = ports.get(endpoint.portId);
      expect(port, span.id).toBeDefined();
      expect(endpoint.defaultLowerRequired, span.id).toBe(false);
      expect(endpoint.lowerRequired, span.id).toBe(false);
      expect(endpoint.branchDownOffset, span.id).toBeLessThanOrEqual(1e-9);
      expect(Number(endpoint.supportZ.toFixed(6)), span.id).toBe(Number(endpoint.endpointZ.toFixed(6)));
    }
  }
}

function scenePartSnapshot(state: WireStateHandle) {
  return new Map(visualParts(state).map((part) => [
    part.info.partKey,
    {
      version: part.info.sourceVersion,
      samples: [...part.samples]
    }
  ]));
}

function sameSamples(a: number[], b: number[]): boolean {
  return a.length === b.length && a.every((value, index) => Math.abs(value - b[index]) <= 1e-9);
}

function expectSamplesChangedOnlyWithVersionChange(
  before: ReturnType<typeof scenePartSnapshot>,
  after: ReturnType<typeof scenePartSnapshot>
) {
  for (const [key, previous] of before) {
    const next = after.get(key);
    if (next === undefined || sameSamples(previous.samples, next.samples)) continue;
    expect(next.version, key).not.toBe(previous.version);
  }
}

describe("wire wasm smoke", () => {
  let state: WireStateHandle;
  let createState: () => WireStateHandle;
  let wasmBuildSourceHash = "";
  let wasmBuildVersion = "";

  beforeAll(async () => {
    const module = await loadWireModule();
    createState = () => new module.WireState();
    wasmBuildSourceHash = module.wireBuildSourceHash();
    wasmBuildVersion = module.wireBuildVersion();
    state = new module.WireState();
  });

  it("loads the generated wasm module", () => {
    expect(createState).toBeTypeOf("function");
    expect(wasmBuildSourceHash).toMatch(/^(unknown|[0-9a-f]{12})$/);
    expect(wasmBuildVersion).toMatch(/^(unknown|\d+\.\d+\.\d+)$/);
    const loaded = createState();
    expect(loaded.poleCount()).toBe(0);
    loaded.delete();
  });

  it("returns machine-readable edit error kinds from wasm operations", () => {
    const runState = createState();
    const validation = runState.generatePlacements(
      new Float64Array([Number.NaN, 0, 0, 12, 0, 0]),
      hvBundlePlacement(),
      0,
      2,
      0,
      0,
      []
    );
    expect(validation.ok).toBe(false);
    expect(validation.failureCategory).toBe(CommitFailureCategory.InvalidInput);
    expect(validation.reasonCode).toBe("invalid_input");

    const unsupported = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0]),
      [],
      0,
      2,
      0,
      0,
      []
    );
    expect(unsupported.ok).toBe(false);
    expect(unsupported.failureCategory).toBe(CommitFailureCategory.NotImplemented);
    expect(unsupported.reasonCode).toBe("not_implemented");
    runState.delete();
  });

  it("previews one wire interval through the commit generator without mutating state", () => {
    const runState = createState();
    expect(runState.configureModelAssemblies(modelBootstrap()).ok).toBe(true);
    const points = new Float64Array([0, 0, 0, 20, 4, 0]);
    const placements = hvBundlePlacement();
    const before = runState.saveState();

    const preview = runState.previewPlacements(points, placements, 0, 1, 0, 12, []);
    expect(preview.ok, preview.error).toBe(true);
    expect(preview.generatedPoleIds).toHaveLength(2);
    expect(preview.generatedSpanIds).toHaveLength(3);
    expect(preview.parts.length).toBeGreaterThan(0);
    expect(preview.poles).toHaveLength(2);
    expect(runState.saveState()).toBe(before);
    expect(runState.poleCount()).toBe(0);

    const generated = runState.generatePlacements(points, placements, 0, 1, 0, 12, []);
    expect(generated.ok, generated.error).toBe(true);
    expect(generated.generatedPoleIds).toEqual(preview.generatedPoleIds);
    expect(generated.generatedSpanIds).toEqual(preview.generatedSpanIds);
    const committedParts = visualParts(runState);
    const previewSamples = new Float64Array(preview.samples);
    const previewByKey = new Map(preview.parts.map((part) => [
      part.partKey,
      [...previewSamples.subarray(part.sampleOffset, part.sampleOffset + part.sampleCount * 3)]
    ]));
    for (const part of committedParts) {
      expect(previewByKey.get(part.info.partKey)).toEqual([...part.samples]);
    }
    runState.delete();
  });

  it("continues previewing and committing wire intervals after two committed corners", () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);

    let placements = defaultBundlePlacements();
    const first = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0]), placements, 0, 2, 0, 12, []
    );
    expect(first.ok, first.error).toBe(true);
    expect(first.generatedPoleIds).toHaveLength(2);
    expect(first.generatedBundleIds).toBeDefined();
    expect(first.generatedPoleIds).toBeDefined();
    placements = placements.map((placement, index) => ({
      ...placement,
      generatedBundleId: first.generatedBundleIds![index]
    }));

    const secondStart = first.generatedPoleIds!.at(-1)!;
    const second = runState.generatePlacements(
      new Float64Array([12, 0, 0, 12, 10, 0]), placements, 0, 2, 0, 12,
      [{ pointIndex: 0, supportKind: 0, nodeId: secondStart }]
    );
    expect(second.ok, second.error).toBe(true);
    expect(second.generatedPoleIds).toHaveLength(1);
    expect(second.generatedPoleIds).toBeDefined();

    const beforeThird = runState.saveState();
    const thirdStart = second.generatedPoleIds!.at(-1)!;
    const third = runState.previewPlacements(
      new Float64Array([12, 10, 0, 0, 10, 0]), placements, 0, 2, 0, 12,
      [{ pointIndex: 0, supportKind: 0, nodeId: thirdStart }]
    );
    expect(third.ok, third.error).toBe(true);
    expect(runState.saveState()).toBe(beforeThird);

    const thirdCommitted = runState.generatePlacements(
      new Float64Array([12, 10, 0, 0, 10, 0]), placements, 0, 2, 0, 12,
      [{ pointIndex: 0, supportKind: 0, nodeId: thirdStart }]
    );
    expect(thirdCommitted.ok, thirdCommitted.error).toBe(true);
    expect(thirdCommitted.generatedPoleIds).toHaveLength(1);
    expect(thirdCommitted.generatedPoleIds).toBeDefined();

    const fourthStart = thirdCommitted.generatedPoleIds!.at(-1)!;
    const fourth = runState.generatePlacements(
      new Float64Array([0, 10, 0, 0, 20, 0]), placements, 0, 2, 0, 12,
      [{ pointIndex: 0, supportKind: 0, nodeId: fourthStart }]
    );
    expect(fourth.ok, fourth.error).toBe(true);
    expect(fourth.generatedPoleIds).toHaveLength(1);
    expect(fourth.generatedPoleIds).toBeDefined();

    const beforeFifth = runState.saveState();
    const fifthStart = fourth.generatedPoleIds!.at(-1)!;
    const fifth = runState.previewPlacements(
      new Float64Array([0, 20, 0, 10, 20, 0]), placements, 0, 2, 0, 12,
      [{ pointIndex: 0, supportKind: 0, nodeId: fifthStart }]
    );
    expect(fifth.ok, fifth.error).toBe(true);
    expect(runState.saveState()).toBe(beforeFifth);
    runState.delete();
  });

  it("creates twelve independent model-aware wire routes without stale session state", () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);

    for (let routeIndex = 0; routeIndex < 12; routeIndex += 1) {
      const originX = routeIndex * 30;
      let placements = defaultBundlePlacements();
      const first = runState.generatePlacements(
        new Float64Array([originX, 0, 0, originX + 12, 0, 0]),
        placements,
        0,
        2,
        0,
        12,
        []
      );
      expect(first.ok, `route ${routeIndex} first: ${first.error}`).toBe(true);
      expect(first.generatedPoleIds, `route ${routeIndex} first poles`).toHaveLength(2);
      expect(first.generatedBundleIds, `route ${routeIndex} bundle ids`).toBeDefined();
      placements = placements.map((placement, index) => ({
        ...placement,
        generatedBundleId: first.generatedBundleIds![index]
      }));

      const secondStart = first.generatedPoleIds!.at(-1)!;
      const beforePreview = runState.saveState();
      const secondPoints = new Float64Array([
        originX + 12, 0, 0,
        originX + 12, 10, 0
      ]);
      const nodeSpecs = [{ pointIndex: 0, supportKind: 0, nodeId: secondStart }];
      const preview = runState.previewPlacements(
        secondPoints, placements, 0, 2, 0, 12, nodeSpecs
      );
      expect(preview.ok, `route ${routeIndex} preview: ${preview.error}`).toBe(true);
      expect(runState.saveState(), `route ${routeIndex} preview atomicity`).toBe(beforePreview);

      const second = runState.generatePlacements(
        secondPoints, placements, 0, 2, 0, 12, nodeSpecs
      );
      expect(second.ok, `route ${routeIndex} second: ${second.error}`).toBe(true);
      expect(second.generatedPoleIds, `route ${routeIndex} second poles`).toHaveLength(1);
    }

    expect(runState.poleCount()).toBe(36);
    expect(runState.visualScene().models.length).toBeGreaterThan(36);
    runState.delete();
  });

  it("bootstraps one straight communication fixture per Port", () => {
    const modelState = createState();
    const configured = modelState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const generated = modelState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [104], 0, 2, [0], 0, 7, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const models = modelState.visualScene().models;
    expect(models.filter((model) => model.modelKey === "pole_body")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "communication_clamp")).toHaveLength(2);
    expect(new Set(models.map((model) => model.stableKey)).size).toBe(models.length);
    modelState.delete();
  });

  it("keeps the current state unchanged when model bootstrap configure or load fails", () => {
    const target = createState();
    const generated = target.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);
    const before = target.saveState();

    const invalid = modelBootstrap();
    invalid.bundleAssignments[0].endpointAssemblyId = 9999;
    const configured = target.configureModelAssemblies(invalid);
    expect(configured.ok).toBe(false);
    expect(target.saveState()).toBe(before);

    const source = createState();
    const sourceConfigured = source.configureModelAssemblies(modelBootstrap());
    expect(sourceConfigured.ok, sourceConfigured.error).toBe(true);
    const sourceGenerated = source.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [101], 0, 1, [0], 0, 0, []
    );
    expect(sourceGenerated.ok, sourceGenerated.error).toBe(true);

    const loaded = target.loadStateWithModels(source.saveState(), invalid);
    expect(loaded.ok).toBe(false);
    expect(target.saveState()).toBe(before);

    source.delete();
    target.delete();
  });

  it("resolves default Bundle placement through Core", () => {
    const placement = state.resolveDefaultBundlePlacement(101, 1, 3);
    expect(placement.ok, placement.error).toBe(true);
    expect(placement.height).toBeCloseTo(9.2, 6);
    expect(placement.offset).toBeCloseTo(0.0, 6);
    expect(placement.spacing).toBeCloseTo(0.45, 6);
  });

  afterAll(() => {
    state.delete();
  });

  it("generates a finite two-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 0, []);

    expect(result.ok, result.error).toBe(true);
    expect(result.generatedPoleCount).toBe(2);
    expect(result.generatedSpanCount).toBeGreaterThan(0);
    const parts = visualParts(state);
    expect(parts.length).toBeGreaterThan(0);
    expect(new Set(parts.map((part) => part.info.partKey)).size).toBe(parts.length);
    expect(parts.filter((part) => part.info.supplementalKind === 1)).toHaveLength(
      result.generatedSpanCount
    );

    for (const { info, samples } of parts) {
      expect(samples).toBeInstanceOf(Float64Array);
      expect(samples.length).toBe(info.sampleCount * 3);
      expect([...samples].every(Number.isFinite)).toBe(true);
    }
  });

  it("keeps duplicate-template source bundle identity for a mid-edge branch", () => {
    const branchState = createState();
    const placements = [{
      id: 1, bundleTemplateId: 105, count: 1, explicit: true,
      height: 5.3, offset: 0, spacing: 0.2
    }, {
      id: 2, bundleTemplateId: 105, count: 1, explicit: true,
      height: 5.8, offset: 0.25, spacing: 0.2
    }];
    const base = branchState.generatePlacements(
      new Float64Array([0, 0, 0, 20, 0, 0]), placements, 0, 2, 0, 0, []
    );
    expect(base.ok, base.error).toBe(true);
    expect(base.generatedBundleIds).toHaveLength(2);

    expect(branchState.backboneEdgeCount()).toBe(1);
    const edge = branchState.backboneEdge(0);
    const nodes = Array.from(
      { length: branchState.supportNodeCount() },
      (_, index) => branchState.supportNode(index)
    );
    const nodeA = nodes.find((node) => node.id === edge.nodeAId)!;
    const nodeB = nodes.find((node) => node.id === edge.nodeBId)!;
    const resolved = branchState.resolveBranchPick({
      hitKind: 2,
      hitId: "0",
      hitX: 10,
      hitY: 0,
      hitZ: 0,
      hasSegmentEndpoints: true,
      segmentNodeAId: edge.nodeAId,
      segmentNodeBId: edge.nodeBId,
      segmentEndpointAX: nodeA.x,
      segmentEndpointAY: nodeA.y,
      segmentEndpointAZ: nodeA.z,
      segmentEndpointBX: nodeB.x,
      segmentEndpointBY: nodeB.y,
      segmentEndpointBZ: nodeB.z
    }, [105]);
    expect(resolved.ok, resolved.error).toBe(true);

    const branch = branchState.generatePlacements(
      new Float64Array([
        resolved.positionX, resolved.positionY, resolved.positionZ,
        10, 8, 0
      ]),
      placements.map((placement, index) => ({
        ...placement,
        generatedBundleId: base.generatedBundleIds![index]
      })),
      0, 2, 0, 0,
      [{ pointIndex: 0, supportKind: resolved.supportKind, nodeId: resolved.nodeId }]
    );
    expect(branch.ok, branch.error).toBe(true);
    expect(branch.generatedSpanCount).toBe(2);
    branchState.delete();
  });

  it("bootstraps straight HV assemblies and returns model instances in the scene payload", () => {
    const modelState = createState();
    const bootstrap = modelBootstrap();
    const configured = modelState.configureModelAssemblies(bootstrap);
    expect(configured.ok, configured.error).toBe(true);
    const generated = modelState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [101], 0, 1, [0], 0, 7, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const models = modelState.visualScene().models;
    expect(models.filter((model) => model.modelKey === "pole_body")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "hv_crossarm")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "pole_belt")).toHaveLength(2);
    expect(models.filter((model) => model.modelKey === "hv_insulator")).toHaveLength(6);
    expect(new Set(models.map((model) => model.stableKey)).size).toBe(models.length);
    expect(models.every((model) => [
      model.positionX, model.positionY, model.positionZ,
      model.rotationX, model.rotationY, model.rotationZ,
      model.scaleX, model.scaleY, model.scaleZ
    ].every(Number.isFinite))).toBe(true);

    const saved = modelState.saveState();
    const loadedState = createState();
    const loaded = loadedState.loadStateWithModels(saved, bootstrap);
    expect(loaded.ok, loaded.error).toBe(true);
    expect(loadedState.visualScene().models).toEqual(models);
    modelState.delete();
    loadedState.delete();
  });

  it("does not expose obsolete HV row-step connectors in the web scene", () => {
    const modelState = createState();
    const configured = modelState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const generated = modelState.generate(
      new Float64Array([0, 0, 0, 10, 0, 0, 5, 8.660254037844386, 0]),
      [101], 0, 1, [0], 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const obsoleteConnectors = visualParts(modelState).filter((part) =>
      part.info.kind === 5 || part.info.partKey.startsWith("support-arm:")
    );
    expect(obsoleteConnectors).toHaveLength(0);
    expect(modelState.visualScene().models.some((model) => model.modelKey === "hv_insulator")).toBe(true);
    modelState.delete();
  });

  it("keeps support detail out of the raw wasm scene and derives it in the web scene adapter", async () => {
    const rawState = createState();
    const rawGenerated = rawState.generate(
      SUPPORT_DETAIL_SCENE_POINTS,
      SUPPORT_DETAIL_SCENE_BUNDLE_TEMPLATE_IDS, 0, 1,
      SUPPORT_DETAIL_SCENE_COUNTS, 0, 0, []
    );
    expect(rawGenerated.ok, rawGenerated.error).toBe(true);
    expect(rawState.visualScene().models.some((model) => model.modelKey.startsWith("detail_"))).toBe(false);
    expect(visualParts(rawState).some((part) =>
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.localCable ||
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.inlineCable
    )).toBe(false);
    rawState.delete();

    const bridge = await WireBridge.create();
    const generated = bridge.generate(
      SUPPORT_DETAIL_SCENE_POINTS,
      SUPPORT_DETAIL_SCENE_BUNDLE_TEMPLATE_IDS, 0, 1,
      SUPPORT_DETAIL_SCENE_COUNTS, 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const scene = bridge.scene();
    const models = scene.models;
    expect(models.some((model) => model.modelKey === SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS[0])).toBe(true);
    expect(models.filter((model) => model.modelKey === SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS[1]).length)
      .toBeGreaterThanOrEqual(3);
    expect(models.filter((model) => model.modelKey === SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS[2])).toHaveLength(1);
    const transformers = models.filter((model) => model.modelKey === SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS[0]);
    expect(transformers.every((model) =>
      model.scaleX <= 0.35 && model.scaleY <= 0.22 && model.scaleZ <= 0.45
    )).toBe(true);

    const detailParts = scene.parts;
    expect(detailParts.filter((part) =>
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.localCable
    ).length).toBeGreaterThanOrEqual(3);
    expect(detailParts.filter((part) =>
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.inlineCable
    )).toHaveLength(1);
    expect(bridge.saveState()).not.toMatch(/detail_|LocalDetail|InlineDetail/);

    const repeatedBridge = await WireBridge.create();
    const repeated = repeatedBridge.generate(
      SUPPORT_DETAIL_SCENE_POINTS,
      SUPPORT_DETAIL_SCENE_BUNDLE_TEMPLATE_IDS, 0, 1,
      SUPPORT_DETAIL_SCENE_COUNTS, 0, 0, []
    );
    expect(repeated.ok, repeated.error).toBe(true);
    expect(detailSceneSignature(repeatedBridge.scene())).toBe(detailSceneSignature(scene));
  });

  it("does not attach HV support detail to an optical-only scene fixture", async () => {
    const bridge = await WireBridge.create();
    const generated = bridge.generate(
      SUPPORT_DETAIL_SCENE_POINTS,
      [105], 0, 1,
      [0], 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);

    const scene = bridge.scene();
    expect(scene.models.some((model) =>
      model.modelKey === SUPPORT_DETAIL_SCENE_EXPECTED_MODEL_KEYS[0]
    )).toBe(false);
    expect(scene.parts.some((part) =>
      part.info.supplementalKind === SUPPORT_DETAIL_SCENE_EXPECTED_SUPPLEMENTAL_KINDS.localCable
    )).toBe(false);
  });

  it("upgrades an older saved row assembly by adapter version during restore", () => {
    const oldState = createState();
    const oldBootstrap = modelBootstrap();
    expect(oldState.configureModelAssemblies(oldBootstrap).ok).toBe(true);
    expect(oldState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [101], 0, 1, [0], 0, 0, []
    ).ok).toBe(true);
    const oldModels = oldState.visualScene().models;

    const upgraded = modelBootstrap();
    const row = upgraded.assemblies.find((assembly) => assembly.id === 9902)!;
    row.version = 2;
    row.parts[0].localTransform.positionX = 0.17;
    row.parts[0].sockets[0].positionZ = 0.04;
    row.endpointMountSocket = { partId: 1, socketName: "endpoint_mount" };

    const restored = createState();
    const loaded = restored.loadStateWithModels(oldState.saveState(), upgraded);
    expect(loaded.ok, loaded.error).toBe(true);
    const newModels = restored.visualScene().models;
    expect(new Set(newModels.map((model) => model.stableKey)))
      .toEqual(new Set(oldModels.map((model) => model.stableKey)));
    const oldCrossarm = oldModels.find((model) => model.modelKey === "hv_crossarm")!;
    const newCrossarm = newModels.find((model) => model.stableKey === oldCrossarm.stableKey)!;
    expect(newCrossarm.contentVersion).not.toBe(oldCrossarm.contentVersion);
    expect(Math.hypot(
      newCrossarm.positionX - oldCrossarm.positionX,
      newCrossarm.positionY - oldCrossarm.positionY,
      newCrossarm.positionZ - oldCrossarm.positionZ
    )).toBeGreaterThan(0.17);
    const oldInsulator = oldModels.find((model) => model.modelKey === "hv_insulator")!;
    const newInsulator = newModels.find((model) => model.stableKey === oldInsulator.stableKey)!;
    expect(newInsulator.contentVersion).not.toBe(oldInsulator.contentVersion);
    expect(Math.hypot(
      newInsulator.positionX - oldInsulator.positionX,
      newInsulator.positionY - oldInsulator.positionY,
      newInsulator.positionZ - oldInsulator.positionZ
    )).toBeGreaterThan(0.05);
    oldState.delete();
    restored.delete();
  });

  it("applies generation-time tilt when a max tilt is requested", () => {
    const tilted = createState();
    const generated = tilted.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 9.5, []
    );
    expect(generated.ok, generated.error).toBe(true);
    const poles = Array.from({ length: tilted.poleCount() }, (_, index) => tilted.pole(index));
    expect(poles.length).toBeGreaterThan(0);
    expect(
      poles.some((pole) => Math.abs(pole.rotationX) > 0.01 || Math.abs(pole.rotationY) > 0.01)
    ).toBe(true);
    tilted.delete();

    const flat = createState();
    const plain = flat.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(plain.ok, plain.error).toBe(true);
    const plainPoles = Array.from({ length: flat.poleCount() }, (_, index) => flat.pole(index));
    expect(
      plainPoles.every(
        (pole) => Math.abs(pole.rotationX) < 1e-9 && Math.abs(pole.rotationY) < 1e-9
      )
    ).toBe(true);
    flat.delete();
  });

  it("exposes a shared run id for through edge bodies", () => {
    const runState = createState();
    const result = runState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0, 20, 10, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(result.ok, result.error).toBe(true);

    const edgeBodies = visualParts(runState).map((part) => part.info)
      .filter((part) => part.kind === 0 && part.bundleTemplateId === 102 && part.laneIndex === 0);

    expect(edgeBodies).toHaveLength(2);
    expect(edgeBodies[0].runId).toBeGreaterThan(0);
    expect(edgeBodies[1].runId).toBe(edgeBodies[0].runId);
    runState.delete();
  });

  it("generates the viewer default bundle selection together", () => {
    const result = state.generate(
      new Float64Array([0, 10, 0, 20, 10, 0]),
      [102, 101, 104, 105],
      0,
      2,
      [0, 0, 0, 0],
      0,
      0,
      []
    );

    expect(result.ok, result.error).toBe(true);
    expect(result.generatedSpanCount).toBeGreaterThanOrEqual(4);
  });

  it("separates fresh straight HV phases by the requested lane spacing", () => {
    const runState = createState();
    const result = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0]),
      hvBundlePlacement(),
      0,
      2,
      0,
      0,
      []
    );
    expect(result.ok, result.error).toBe(true);
    expect(result.generatedSpanCount).toBe(3);
    assertStraightHvSpacing(hvEdgeBodies(runState));
    runState.delete();
  });

  it("separates model-aware fresh straight HV endpoint fixtures and curve endpoints", () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const result = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0]),
      hvBundlePlacement(),
      0,
      2,
      0,
      0,
      []
    );
    expect(result.ok, result.error).toBe(true);
    const hvParts = hvEdgeBodies(runState);
    assertStraightHvSpacing(hvParts);
    const models = runState.visualScene().models;
    const insulators = models.filter((model) => model.modelKey === "hv_insulator");
    expect(insulators).toHaveLength(6);
    assertThreeYSpacing(insulators.slice(0, 3).map(modelPosition), 0.45);
    assertThreeYSpacing(insulators.slice(3, 6).map(modelPosition), 0.45);
    assertThreeYSpacing(hvParts.map(startPoint), 0.45);
    assertThreeYSpacing(hvParts.map(endPoint), 0.45);
    expectBootstrapInsulatorSocketsOnStraightHvEndpoints(hvParts, models);
    runState.delete();
  });

  it("separates fresh viewer default HV phases before any branch completion", () => {
    const runState = createState();
    const result = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0]),
      defaultBundlePlacements(),
      0,
      2,
      0,
      0,
      []
    );
    expect(result.ok, result.error).toBe(true);
    expect(result.generatedSpanCount).toBe(8);
    assertStraightHvSpacing(hvEdgeBodies(runState));
    assertHvSeparatedByEdge(runState);
    runState.delete();
  });

  it("keeps every HV edge separated when completing a T and cross with viewer default placements", () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const placements = defaultBundlePlacements();
    const base = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0, 12, 8, 0]),
      placements,
      0,
      2,
      0,
      0,
      []
    );
    expect(base.ok, base.error).toBe(true);
    expect(base.generatedSpanCount).toBe(16);
    assertHvSeparatedByEdge(runState);
    assertModelHvInsulatorsSeparatedByPole(runState);
    const baseHvSpanIds = new Set(hvEdgeBodies(runState).map((part) => part.info.sourceSpanId));
    const baseSnapshot = scenePartSnapshot(runState);

    const poleB = runState.pole(1);
    const bd = runState.generatePlacements(
      new Float64Array([poleB.positionX, poleB.positionY, poleB.positionZ, 12, -8, 0]),
      placements,
      0,
      1,
      0,
      0,
      [{ pointIndex: 0, supportKind: 0, nodeId: poleB.id }]
    );
    expect(bd.ok, bd.error).toBe(true);
    expect(bd.generatedSpanCount).toBe(8);
    const afterBdHvParts = hvEdgeBodies(runState);
    expect(afterBdHvParts).toHaveLength(9);
    const bdHvParts = afterBdHvParts.filter((part) => !baseHvSpanIds.has(part.info.sourceSpanId));
    expect(bdHvParts).toHaveLength(3);
    expect(uniqueRounded(bdHvParts.map((part) => part.info.laneIndex))).toHaveLength(3);
    assertHvSeparatedByEdge(runState);
    assertModelHvInsulatorsSeparatedByPole(runState);
    const bdHvSpanIds = new Set(afterBdHvParts.map((part) => part.info.sourceSpanId));
    const bdSnapshot = scenePartSnapshot(runState);
    expectSamplesChangedOnlyWithVersionChange(baseSnapshot, bdSnapshot);

    const eb = runState.generatePlacements(
      new Float64Array([20, 0, 0, poleB.positionX, poleB.positionY, poleB.positionZ]),
      placements,
      0,
      1,
      0,
      0,
      [{ pointIndex: 1, supportKind: 0, nodeId: poleB.id }]
    );
    expect(eb.ok, eb.error).toBe(true);
    expect(eb.generatedSpanCount).toBe(8);
    const completedHvParts = hvEdgeBodies(runState);
    expect(completedHvParts).toHaveLength(12);
    const ebHvParts = completedHvParts.filter((part) => !bdHvSpanIds.has(part.info.sourceSpanId));
    expect(ebHvParts).toHaveLength(3);
    expect(uniqueRounded(ebHvParts.map((part) => part.info.laneIndex))).toHaveLength(3);
    assertHvSeparatedByEdge(runState);
    assertModelHvInsulatorsSeparatedByPole(runState);
    expectSamplesChangedOnlyWithVersionChange(bdSnapshot, scenePartSnapshot(runState));
    runState.delete();
  });

  it("restores a model-aware T branch with viewer default placements", () => {
    const runState = createState();
    const bootstrap = modelBootstrap();
    const configured = runState.configureModelAssemblies(bootstrap);
    expect(configured.ok, configured.error).toBe(true);
    const hvTemplate = runState.bundleTemplate(0);
    expect(hvTemplate.id).toBe(101);
    expect(hvTemplate.enableBranchDownOffset).toBe(true);
    expect(hvTemplate.branchEndpointOffset).toBe(-0.55);
    const placements = defaultBundlePlacements();
    const base = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0, 12, 8, 0]),
      placements,
      0,
      2,
      0,
      12,
      []
    );
    expect(base.ok, base.error).toBe(true);

    const poleB = runState.pole(1);
    const branch = runState.generatePlacements(
      new Float64Array([-5, 4, 0, poleB.positionX, poleB.positionY, poleB.positionZ]),
      placements,
      0,
      2,
      0,
      12,
      [{ pointIndex: 1, supportKind: 0, nodeId: poleB.id }]
    );
    expect(branch.ok, branch.error).toBe(true);
    expect(branch.generatedSpanCount).toBe(8);

    const restored = createState();
    const loaded = restored.loadStateWithModels(runState.saveState(), bootstrap);
    expect(loaded.ok, loaded.error).toBe(true);
    expect(restored.visualScene().models.some((model) => model.modelKey === "hv_crossarm")).toBe(true);
    expect(restored.visualScene().models.some((model) => model.modelKey === "hv_insulator")).toBe(true);
    assertHvSeparatedByEdge(restored);
    assertModelHvInsulatorsSeparatedByPole(restored);
    assertNonFlaggedSpanLayoutsAtPortHeight(restored);
    runState.delete();
    restored.delete();
  });

  it("keeps a repro T branch HV support slot at the requested port height", () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const placements = defaultBundlePlacements();
    const base = runState.generatePlacements(
      new Float64Array([
        17.397, 23.190, 0,
        10.065, -2.878, 0,
        23.238, -21.868, 0
      ]),
      placements,
      0,
      2,
      0,
      12,
      []
    );
    expect(base.ok, base.error).toBe(true);
    const baseHvSpanIds = new Set(hvEdgeBodies(runState).map((part) => part.info.sourceSpanId));

    const poleB = runState.pole(1);
    const branch = runState.generatePlacements(
      new Float64Array([
        -5.477, 4.187, 0,
        poleB.positionX, poleB.positionY, poleB.positionZ
      ]),
      placements,
      0,
      2,
      0,
      12,
      [{ pointIndex: 1, supportKind: 0, nodeId: poleB.id }]
    );
    expect(branch.ok, branch.error).toBe(true);
    expect(branch.generatedSpanCount).toBe(8);
    const afterBranchHvParts = hvEdgeBodies(runState);
    expect(afterBranchHvParts).toHaveLength(9);
    const newBranchHvParts = afterBranchHvParts.filter((part) => !baseHvSpanIds.has(part.info.sourceSpanId));
    expect(newBranchHvParts).toHaveLength(3);
    expect(uniqueRounded(newBranchHvParts.map((part) => part.info.laneIndex))).toEqual([0, 1, 2]);

    const bHvPorts = Array.from({ length: runState.portCount() }, (_, index) => runState.port(index))
      .filter((port) => port.ownerPoleId === poleB.id && port.category === 0);
    expect(bHvPorts).toHaveLength(12);

    const bHvRows = runState.visualScene().models
      .filter((model) => model.modelKey === "hv_crossarm" && model.stableKey.startsWith(`row:${poleB.id}:`));
    expect(bHvRows).toHaveLength(2);
    expect(Math.abs(bHvRows[0].positionZ - bHvRows[1].positionZ)).toBeGreaterThan(0.1);
    assertHvSeparatedByEdge(runState);
    assertModelHvInsulatorsSeparatedByPole(runState);
    assertNonFlaggedSpanLayoutsAtPortHeight(runState);
    runState.delete();
  });

  it("keeps production-bootstrap non-branch-down edge endpoints at port height on reverse T branch", async () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(await productionLikeModelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const placements = defaultBundlePlacements();
    const base = runState.generatePlacements(
      new Float64Array([
        17.397, 23.190, 0,
        10.065, -2.878, 0,
        23.238, -21.868, 0
      ]),
      placements,
      0,
      1,
      0,
      12,
      []
    );
    expect(base.ok, base.error).toBe(true);

    const poleB = runState.pole(1);
    const branch = runState.generatePlacements(
      new Float64Array([
        -5.477, 4.187, 0,
        poleB.positionX, poleB.positionY, poleB.positionZ
      ]),
      placements,
      0,
      1,
      0,
      12,
      [{ pointIndex: 1, supportKind: 0, nodeId: poleB.id }]
    );
    expect(branch.ok, branch.error).toBe(true);
    expect(branch.generatedSpanCount).toBe(8);
    assertHvSeparatedByEdge(runState);
    assertNonFlaggedSpanLayoutsAtPortHeight(runState);
    runState.delete();
  });

  it("uses branch-down template flags, not HV category, on the production viewer pole type", async () => {
    const runState = createState();
    const configured = runState.configureModelAssemblies(await productionLikeModelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const hvTemplate = runState.bundleTemplate(0);
    const lvTemplate = runState.bundleTemplate(1);
    expect(hvTemplate.id).toBe(101);
    expect(lvTemplate.id).toBe(102);
    expect(runState.updateBundleTemplate({
      ...hvTemplate,
      enableBranchDownOffset: false,
      branchEndpointOffset: -0.275
    }).ok).toBe(true);
    expect(runState.updateBundleTemplate({
      ...lvTemplate,
      enableBranchDownOffset: true,
      branchEndpointOffset: -0.325
    }).ok).toBe(true);
    const placements = [
      defaultBundlePlacements()[0],
      defaultBundlePlacements()[1]
    ];
    const base = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0, 12, 8, 0]),
      placements,
      0,
      2,
      0,
      12,
      []
    );
    expect(base.ok, base.error).toBe(true);
    const baseHvSpanIds = new Set(hvEdgeBodies(runState).map((part) => part.info.sourceSpanId));
    const poleB = runState.pole(1);
    const branch = runState.generatePlacements(
      new Float64Array([-5, 4, 0, poleB.positionX, poleB.positionY, poleB.positionZ]),
      placements,
      0,
      2,
      0,
      12,
      [{ pointIndex: 1, supportKind: 0, nodeId: poleB.id }]
    );
    expect(branch.ok, branch.error).toBe(true);
    expect(branch.generatedSpanCount).toBe(4);
    const newHvParts = hvEdgeBodies(runState)
      .filter((part) => !baseHvSpanIds.has(part.info.sourceSpanId));
    expect(newHvParts).toHaveLength(3);
    for (let index = 0; index < runState.spanCount(); index += 1) {
      const span = runState.span(index);
      const part = visualParts(runState).find((candidate) =>
        candidate.info.kind === 0 && candidate.info.sourceSpanId === span.id
      );
      if (part === undefined) continue;
      const layout = runState.spanLayout(span.id);
      expect(layout.ok, layout.error).toBe(true);
      const endpoints = [layout.start!, layout.end!];
      if (part.info.bundleTemplateId === 101) {
        for (const endpoint of endpoints) {
          expect(endpoint.defaultLowerRequired, span.id).toBe(false);
          expect(endpoint.lowerRequired, span.id).toBe(false);
          expect(endpoint.branchDownOffset, span.id).toBeLessThanOrEqual(1e-9);
        }
      } else if (part.info.bundleTemplateId === 102 && endpoints.some((endpoint) => endpoint.defaultLowerRequired)) {
        expect(endpoints.some((endpoint) => endpoint.branchDownOffset > 0.1)).toBe(true);
      }
    }
    assertHvSeparatedByEdge(runState);
    runState.delete();
  });

  it("keeps HV when resolving a pole snap through the UI hit payload and excludes HV for midair branch", () => {
    const coveredEntries = new Set<string>();
    const runState = createState();
    const configured = runState.configureModelAssemblies(modelBootstrap());
    expect(configured.ok, configured.error).toBe(true);
    const placements = defaultBundlePlacements();
    const selectedBundleTemplateIds = [101, 102, 104, 105];
    const base = runState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0, 12, 8, 0]),
      placements,
      0,
      2,
      0,
      9.5,
      []
    );
    expect(base.ok, base.error).toBe(true);
    const poleB = runState.pole(1);
    const beforePoleHvSpanIds = new Set(hvEdgeBodies(runState).map((part) => part.info.sourceSpanId));

    const poleBNode = Array.from({ length: runState.supportNodeCount() }, (_, index) => runState.supportNode(index))
      .find((node) => node.poleId === poleB.id);
    expect(poleBNode).toBeDefined();
    const poleBEdge = Array.from({ length: runState.backboneEdgeCount() }, (_, index) => runState.backboneEdge(index))
      .find((edge) => edge.nodeAId === poleBNode!.id || edge.nodeBId === poleBNode!.id);
    expect(poleBEdge).toBeDefined();
    const otherNodeId = poleBEdge!.nodeAId === poleBNode!.id ? poleBEdge!.nodeBId : poleBEdge!.nodeAId;
    const otherNode = Array.from({ length: runState.supportNodeCount() }, (_, index) => runState.supportNode(index))
      .find((node) => node.id === otherNodeId);
    expect(otherNode).toBeDefined();
    const resolvedPole = runState.resolveBranchPick({
      hitKind: 2,
      hitId: "0",
      hitX: poleB.positionX + 0.12,
      hitY: poleB.positionY - 0.10,
      hitZ: 9.2,
      hasSegmentEndpoints: true,
      segmentNodeAId: poleBNode!.id,
      segmentNodeBId: otherNode!.id,
      segmentEndpointAX: poleBNode!.x,
      segmentEndpointAY: poleBNode!.y,
      segmentEndpointAZ: poleBNode!.z,
      segmentEndpointBX: otherNode!.x,
      segmentEndpointBY: otherNode!.y,
      segmentEndpointBZ: otherNode!.z
    }, selectedBundleTemplateIds);
    expect(resolvedPole.ok, resolvedPole.error).toBe(true);
    expect(resolvedPole.supportKind).toBe(0);
    expect(resolvedPole.nodeId).toBe(poleBNode!.id);
    expect(Array.from({ length: runState.supportNodeCount() }, (_, index) => runState.supportNode(index).id))
      .toContain(resolvedPole.nodeId);

    const poleBranch = runState.generatePlacements(
      new Float64Array([
        -5, 4, 0,
        resolvedPole.positionX, resolvedPole.positionY, resolvedPole.positionZ
      ]),
      placements,
      0,
      2,
      0,
      9.5,
      [{ pointIndex: 1, supportKind: resolvedPole.supportKind, nodeId: resolvedPole.nodeId }]
    );
    expect(poleBranch.ok, poleBranch.error).toBe(true);
    const newPoleHvParts = hvEdgeBodies(runState)
      .filter((part) => !beforePoleHvSpanIds.has(part.info.sourceSpanId));
    expect(newPoleHvParts).toHaveLength(3);
    expect(uniqueRounded(newPoleHvParts.map((part) => part.info.laneIndex))).toEqual([0, 1, 2]);
    coveredEntries.add("BOS:add_one_edge:S1");

    const midairState = createState();
    const midairConfigured = midairState.configureModelAssemblies(modelBootstrap());
    expect(midairConfigured.ok, midairConfigured.error).toBe(true);
    const midairBase = midairState.generatePlacements(
      new Float64Array([0, 0, 0, 12, 0, 0, 12, 8, 0]),
      placements,
      0,
      2,
      0,
      9.5,
      []
    );
    expect(midairBase.ok, midairBase.error).toBe(true);
    const edge = midairState.backboneEdge(0);
    const nodeA = midairState.supportNode(0);
    const nodeB = midairState.supportNode(1);
    const beforeMidairHvSpanIds = new Set(hvEdgeBodies(midairState).map((part) => part.info.sourceSpanId));
    const resolvedMidair = midairState.resolveBranchPick({
      hitKind: 2,
      hitId: "0",
      hitX: (nodeA.x + nodeB.x) * 0.5,
      hitY: (nodeA.y + nodeB.y) * 0.5,
      hitZ: (nodeA.z + nodeB.z) * 0.5,
      hasSegmentEndpoints: true,
      segmentNodeAId: edge.nodeAId,
      segmentNodeBId: edge.nodeBId,
      segmentEndpointAX: nodeA.x,
      segmentEndpointAY: nodeA.y,
      segmentEndpointAZ: nodeA.z,
      segmentEndpointBX: nodeB.x,
      segmentEndpointBY: nodeB.y,
      segmentEndpointBZ: nodeB.z
    }, selectedBundleTemplateIds);
    expect(resolvedMidair.ok, resolvedMidair.error).toBe(true);
    const midairBranch = midairState.generatePlacements(
      new Float64Array([
        resolvedMidair.positionX, resolvedMidair.positionY, resolvedMidair.positionZ,
        6, 5, 0
      ]),
      placements,
      0,
      2,
      0,
      9.5,
      [{ pointIndex: 0, supportKind: resolvedMidair.supportKind, nodeId: resolvedMidair.nodeId }]
    );
    expect(midairBranch.ok, midairBranch.error).toBe(true);
    const newMidairHvParts = hvEdgeBodies(midairState)
      .filter((part) => !beforeMidairHvSpanIds.has(part.info.sourceSpanId));
    expect(newMidairHvParts).toHaveLength(0);
    coveredEntries.add("BOS:add_one_edge:SM");
    expect(missingBackboneEntryCells("wasm_adapter", coveredEntries)).toEqual([]);
    midairState.delete();
    runState.delete();
  });

  it("roundtrips visual parts through authoritative save and load", () => {
    const savedState = createState();
    const generated = savedState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0, 20, 10, 0]), [102], 0, 1, [0], 0, 0, []
    );
    expect(generated.ok, generated.error).toBe(true);
    const expected = visualParts(savedState).map((part) => ({
      info: part.info,
      samples: [...part.samples]
    }));

    const text = savedState.saveState();
    expect(text.startsWith("wire_state_v2\n")).toBe(true);
    const loadedState = createState();
    const loaded = loadedState.loadState(text);
    expect(loaded.ok, loaded.error).toBe(true);
    const actual = visualParts(loadedState);
    expect(actual).toHaveLength(expected.length);
    for (let index = 0; index < expected.length; index += 1) {
      expect(actual[index].info).toEqual(expected[index].info);
      expect([...actual[index].samples]).toEqual(expected[index].samples);
    }
    savedState.delete();
    loadedState.delete();
  });
  it("returns sagged visual samples when sag is enabled", () => {
    const geometry = state.geometrySettings();
    const update = state.updateGeometrySettings({
      ...geometry,
      sagEnabled: true,
      sagFactor: Math.max(geometry.sagFactor, 0.03)
    });
    expect(update.ok, update.error).toBe(true);

    const result = state.generate(new Float64Array([0, 20, 0, 40, 20, 0]), [102], 0, 1, [0], 0, 0, []);
    expect(result.ok, result.error).toBe(true);

    let maxDrop = 0;
    for (const { samples } of visualParts(state)) {
      if (samples.length < 9) continue;
      const startZ = samples[2];
      const endZ = samples[samples.length - 1];
      let minZ = Number.POSITIVE_INFINITY;
      for (let sample = 2; sample < samples.length; sample += 3) {
        minZ = Math.min(minZ, samples[sample]);
      }
      maxDrop = Math.max(maxDrop, (startZ + endZ) / 2 - minZ);
    }

    expect(maxDrop).toBeGreaterThan(0.1);
  });


  it("returns an explicit error for a one-point route", () => {
    const result = state.generate(new Float64Array([0, 0, 0]), [102], 0, 1, [0], 0, 0, []);

    expect(result.ok).toBe(false);
    expect(result.error.length).toBeGreaterThan(0);
  });

  it("updates reshape settings and regenerates layout edits", () => {
    const layoutState = createState();
    const bundleTemplates = Array.from(
      { length: layoutState.bundleTemplateCount() },
      (_, index) => layoutState.bundleTemplate(index)
    );
    const lowVoltage = bundleTemplates.find(
      (template) => template.name === "DEFAULT_SINGLE" && template.fixedCount
    );
    expect(lowVoltage).toBeDefined();

    const generated = layoutState.generate(
      new Float64Array([0, 30, 0, 20, 30, 0, 20, 40, 0]),
      [lowVoltage!.id],
      0,
      1,
      [0],
      0,
      0,
      []
    );
    expect(generated.ok, generated.error).toBe(true);

    const geometry = layoutState.geometrySettings();
    const reshape = layoutState.updateGeometrySettings({
      ...geometry,
      sagFactor: geometry.sagFactor + 0.005
    });
    expect(reshape.ok, reshape.error).toBe(true);
    expect(visualParts(layoutState).length).toBeGreaterThan(0);

    const layout = layoutState.layoutSettings();
    const regenerate = layoutState.updateLayoutSettings({
      ...layout,
      cornerThresholdDeg: layout.cornerThresholdDeg - 1
    });
    expect(regenerate.ok, regenerate.error).toBe(true);
    expect(visualParts(layoutState).length).toBeGreaterThan(0);
    layoutState.delete();
  });

  it("applies pole category height edits to ports and curve endpoints", () => {
    const placementState = createState();
    const templates = Array.from(
      { length: placementState.poleTemplateCount() },
      (_, index) => placementState.poleTemplate(index)
    );
    const original = templates.find((template) => template.name === "CommunicationPole");
    expect(original).toBeDefined();
    const generated = placementState.generate(
      new Float64Array([0, 0, 0, 20, 0, 0]),
      [101],
      0,
      original!.id,
      [0],
      0,
      0,
      []
    );
    expect(generated.ok, generated.error).toBe(true);

    const portsBefore = Array.from(
      { length: placementState.portCount() },
      (_, index) => placementState.port(index)
    ).filter((port) => port.category === 0);
    expect(portsBefore.length).toBeGreaterThan(0);
    const partsBefore = visualParts(placementState).map((part) => new Float64Array(part.samples));

    const edited = structuredClone(original!);
    for (const band of edited.portBands.filter((candidate) => candidate.category === 0)) {
      band.heightCenter += 1;
      band.heightMin += 1;
      band.heightMax += 1;
    }

    const update = placementState.updatePoleTemplate(edited);
    expect(update.ok, update.error).toBe(true);

    const portsAfter = Array.from(
      { length: placementState.portCount() },
      (_, index) => placementState.port(index)
    ).filter((port) => port.category === 0);
    expect(portsAfter).toHaveLength(portsBefore.length);
    for (const before of portsBefore) {
      const after = portsAfter.find((candidate) => candidate.id === before.id);
      expect(after?.z).toBeCloseTo(before.z + 1, 8);
    }

    const partsAfter = visualParts(placementState).map((part) => new Float64Array(part.samples));
    expect(partsAfter).toHaveLength(partsBefore.length);
    expect(
      partsAfter.some((samples, index) =>
        Math.abs(samples[2] - partsBefore[index][2] - 1) < 1e-8 ||
        Math.abs(
          samples[samples.length - 1] -
          partsBefore[index][partsBefore[index].length - 1] -
          1
        ) < 1e-8
      )
    ).toBe(true);
    placementState.delete();
  });

});

describe("road wasm smoke", () => {
  let state: RoadStateHandle;
  let createRoadState: () => RoadStateHandle;
  // A road state with nothing registered, which is what Core now constructs.
  let createEmptyRoadState: () => RoadStateHandle;
  // A road state starts with no cross section, so these tests register the
  // product catalogue the way a new workspace does and draw with the ID Core
  // handed back. Clearing discards the sections too, so it re-registers them.
  let sectionId = 0;
  const seedRoad = (road: RoadStateHandle): void => {
    const seeded = seedRoadSections((section) => road.addRoadLayoutTemplate(section));
    if (!seeded.ok) throw new Error(seeded.error);
    sectionId = seeded.sections.initialId;
  };
  const clearRoad = (road: RoadStateHandle): void => {
    road.clear();
    seedRoad(road);
  };

  beforeAll(async () => {
    const module = await loadWireModule();
    createEmptyRoadState = () => new module.RoadState();
    createRoadState = () => {
      const road = createEmptyRoadState();
      seedRoad(road);
      return road;
    };
    state = createRoadState();
  });

  afterAll(() => {
    state.delete();
  });

  it("preserves road draw elevations through the wasm scene", () => {
    const road = createRoadState();
    try {
      const drawn = road.addSegment({
        roadLayoutTemplateId: sectionId,
        kind: "line",
        startX: 0,
        startY: 0,
        endX: 30,
        endY: 0,
        handleAX: 10,
        handleAY: 0,
        handleBX: 20,
        handleBY: 0,
        startNodeId: 0,
        startSegmentId: 0,
        startSegmentDistanceM: 0,
        startElevationM: 3,
        endElevationM: 9,
        connectToFirstNode: false
      });
      expect(drawn.ok, drawn.error).toBe(true);

      const scene = road.scene();
      expect(scene.nodes.map((node) => node.z).sort((a, b) => a - b))
        .toEqual([3, 9]);
      expect(scene.centerlineSegments).toHaveLength(1);
      expect(scene.centerlineSegments[0]).toEqual(expect.objectContaining({
        startZ: 3,
        endZ: 9
      }));
    } finally {
      road.delete();
    }
  });

  it("previews a continued curve as a curve, matching what it commits", () => {
    const road = createRoadState();
    try {
      const first = road.addSegment({
      roadLayoutTemplateId: sectionId,
        kind: "bezier",
        startX: 0,
        startY: 0,
        endX: 40,
        endY: 0,
        handleAX: 40 / 3,
        handleAY: 0,
        handleBX: (40 * 2) / 3,
        handleBY: 0,
        startNodeId: 0,
        startSegmentId: 0,
        startSegmentDistanceM: 0,
        connectToFirstNode: false
      });
      expect(first.ok, first.error).toBe(true);
      const corridorId = road.scene().corridors[0].id;
      const endNodeId = first.endNodeId ?? 0;

      // The chord the viewer reports for the pending interval.
      const pending = {
        roadLayoutTemplateId: sectionId,
        kind: "bezier" as const,
        startX: 40,
        startY: 0,
        endX: 70,
        endY: 22,
        handleAX: 50,
        handleAY: 22 / 3,
        handleBX: 60,
        handleBY: (22 * 2) / 3,
        startNodeId: endNodeId,
        startSegmentId: 0,
        startSegmentDistanceM: 0,
        extensionCorridorId: corridorId,
        connectToFirstNode: false
      };

      const preview = road.previewInterval(pending);
      const chordHandleAX = 40 + (70 - 40) / 3;
      const chordHandleAY = 0 + 22 / 3;
      // A straight guide would sit exactly on the chord.
      expect(
        Math.hypot(preview.handleAX - chordHandleAX, preview.handleAY - chordHandleAY)
      ).toBeGreaterThan(0.5);

      const committed = road.addSegment({
        ...pending,
        handleAX: preview.handleAX,
        handleAY: preview.handleAY,
        handleBX: preview.handleBX,
        handleBY: preview.handleBY
      });
      expect(committed.ok, committed.error).toBe(true);
      const editable = road
        .scene()
        .editableSegments.find((segment) => segment.id === committed.segmentId);
      expect(editable).toBeDefined();
      // The guide and the committed road are the same four control points.
      expect(editable?.points).toHaveLength(4);
      expect(Math.abs((editable?.points[1].x ?? 0) - preview.handleAX)).toBeLessThan(1e-6);
      expect(Math.abs((editable?.points[1].y ?? 0) - preview.handleAY)).toBeLessThan(1e-6);
      expect(Math.abs((editable?.points[2].x ?? 0) - preview.handleBX)).toBeLessThan(1e-6);
      expect(Math.abs((editable?.points[2].y ?? 0) - preview.handleBY)).toBeLessThan(1e-6);
    } finally {
      road.delete();
    }
  });

  it("starts with no cross section until the product registers its own", () => {
    const road = createEmptyRoadState();
    try {
      expect(road.scene().roadLayoutTemplateCount).toBe(0);
      const drawn = road.addSegment({
        roadLayoutTemplateId: 1,
        kind: "line",
        startX: 0,
        startY: 0,
        endX: 24,
        endY: 0,
        handleAX: 8,
        handleAY: 0,
        handleBX: 16,
        handleBY: 0,
        startNodeId: 0,
        startSegmentId: 0,
        startSegmentDistanceM: 0,
        extensionCorridorId: 0,
        connectToFirstNode: false
      });
      expect(drawn.ok).toBe(false);
      expect(drawn.error).toContain("section template");

      const seeded = seedRoadSections((section) => road.addRoadLayoutTemplate(section));
      expect(seeded.ok).toBe(true);
      if (!seeded.ok) return;
      const registered = road.scene().roadLayoutTemplates;
      expect(registered).toHaveLength(ROAD_TEMPLATE_PRESETS.length);
      // Every ID came back from Core; none of them is chosen by the catalogue.
      const ids = registered.map((template) => template.id);
      expect(new Set(ids).size).toBe(ids.length);
      expect(Object.keys(seeded.sections.labels).map(Number).sort()).toEqual(
        [...ids].sort()
      );
      expect(ids).toContain(seeded.sections.initialId);
      expect(seeded.sections.labels[seeded.sections.initialId]).toBe(
        ROAD_TEMPLATE_PRESETS.find((preset) => preset.initial)?.label
      );
    } finally {
      road.delete();
    }
  });

  it("carries each registered section's own measurements into core", () => {
    const road = createRoadState();
    try {
      const registered = road.scene().roadLayoutTemplates;
      const shape = (
        strips: ReadonlyArray<{ function: string; widthM: number }>
      ) => strips.map((strip) => `${strip.function}:${strip.widthM}`).join("|");
      for (const preset of ROAD_TEMPLATE_PRESETS) {
        const match = registered.find(
          (template) => shape(template.strips) === shape(preset.section.strips)
        );
        expect(match, `no core section matches ${preset.key}`).toBeDefined();
        expect(match!.laneCount).toBe(preset.section.laneBands.length);
        expect(match!.lanes.map((lane) => lane.id)).toEqual(
          preset.section.laneBands.map((lane) => lane.id)
        );
        expect(match!.boundaries.map((boundary) => boundary.id)).toEqual(
          preset.section.boundaries.map((boundary) => boundary.id)
        );
      }
    } finally {
      road.delete();
    }
  });

  it("registers shouldered opposing four and six lane road presets", () => {
    const road = createRoadState();
    try {
      const templates = road.scene().roadLayoutTemplates;
      const fourPreset = ROAD_TEMPLATE_PRESETS.find((preset) => preset.key === "shouldered-four-lane");
      const sixPreset = ROAD_TEMPLATE_PRESETS.find((preset) => preset.key === "shouldered-six-lane");
      expect(fourPreset).toBeDefined();
      expect(sixPreset).toBeDefined();

      const findPresetTemplate = (preset: NonNullable<typeof fourPreset>) =>
        templates.find((template) =>
          template.laneCount === preset.section.laneBands.length &&
          template.strips.map((strip) => `${strip.function}:${strip.widthM}`).join("|") ===
            preset.section.strips.map((strip) => `${strip.function}:${strip.widthM}`).join("|")
        );

      const four = findPresetTemplate(fourPreset!);
      const six = findPresetTemplate(sixPreset!);
      expect(four).toBeDefined();
      expect(six).toBeDefined();
      expect(four!.lanes.map((lane) => lane.direction)).toEqual([1, 1, 0, 0]);
      expect(six!.lanes.map((lane) => lane.direction)).toEqual([1, 1, 1, 0, 0, 0]);
      expect(four!.strips.filter((strip) => strip.function === "shoulder").map((strip) => strip.widthM))
        .toEqual([0.75, 0.75]);
      expect(six!.strips.filter((strip) => strip.function === "shoulder").map((strip) => strip.widthM))
        .toEqual([0.75, 0.75]);
    } finally {
      road.delete();
    }
  });

  it("keeps saved sections when a workspace is loaded back", () => {
    const road = createRoadState();
    const reopened = createEmptyRoadState();
    try {
      const drawn = road.addSegment({
        roadLayoutTemplateId: sectionId,
        kind: "line",
        startX: 0,
        startY: 0,
        endX: 40,
        endY: 0,
        handleAX: 40 / 3,
        handleAY: 0,
        handleBX: 80 / 3,
        handleBY: 0,
        startNodeId: 0,
        startSegmentId: 0,
        startSegmentDistanceM: 0,
        extensionCorridorId: 0,
        connectToFirstNode: false
      });
      expect(drawn.ok, drawn.error).toBe(true);
      const saved = road.saveState();
      const before = road.scene().roadLayoutTemplates;

      const loaded = reopened.loadState(saved);
      expect(loaded.ok, loaded.error).toBe(true);
      const after = reopened.scene().roadLayoutTemplates;
      // Loading uses what the archive holds. It neither adds the catalogue
      // again nor refreshes the saved measurements from it.
      expect(after).toHaveLength(before.length);
      expect(after).toEqual(before);
      expect(reopened.saveState()).toBe(saved);
    } finally {
      reopened.delete();
      road.delete();
    }
  });

  it("registers each preset with the alignment offset the preset asked for", () => {
    const road = createRoadState();
    try {
      const registered = road.scene().roadLayoutTemplates;
      expect(registered.length).toBe(ROAD_TEMPLATE_PRESETS.length);
      // The presets are registered in order, so the ID Core handed back for the
      // Nth registration belongs to the Nth preset.
      registered.forEach((template, index) => {
        const preset = ROAD_TEMPLATE_PRESETS[index];
        expect(
          template.alignmentOffsetFromLeftM,
          `${preset.label} lost its alignment offset`
        ).toBeCloseTo(preset.section.alignmentOffsetFromLeftM, 12);
      });
      // A section with no sidewalk on one side is still centred on its own
      // width, which is what these presets mean; Core does not re-derive it.
      const noLeftSidewalk = registered[2];
      expect(noLeftSidewalk.alignmentOffsetFromLeftM).toBeCloseTo(4.0, 12);
    } finally {
      road.delete();
    }
  });

  it("rejects a section that does not say where its alignment runs", () => {
    const road = createEmptyRoadState();
    try {
      const { alignmentOffsetFromLeftM, ...withoutOffset } =
        ROAD_TEMPLATE_PRESETS[0].section;
      expect(alignmentOffsetFromLeftM).toBeGreaterThan(0);
      const added = road.addRoadLayoutTemplate(
        withoutOffset as unknown as RoadSectionInput
      );
      expect(added.ok).toBe(false);
      expect(road.scene().roadLayoutTemplateCount).toBe(0);
    } finally {
      road.delete();
    }
  });

  it("edits one shared section without touching roads on another", () => {
    const road = createRoadState();
    try {
      const other = road
        .scene()
        .roadLayoutTemplates.find((template) => template.id !== sectionId);
      expect(other).toBeDefined();
      const draw = (templateId: number, y: number) =>
        road.addSegment({
          roadLayoutTemplateId: templateId,
          kind: "line",
          startX: 0,
          startY: y,
          endX: 40,
          endY: y,
          handleAX: 40 / 3,
          handleAY: y,
          handleBX: 80 / 3,
          handleBY: y,
          startNodeId: 0,
          startSegmentId: 0,
          startSegmentDistanceM: 0,
          extensionCorridorId: 0,
          connectToFirstNode: false
        });
      expect(draw(sectionId, 0).ok).toBe(true);
      expect(draw(other!.id, 200).ok).toBe(true);
      const templateOf = (templateId: number) =>
        road.scene().roadLayoutTemplates.find((template) => template.id === templateId)!;
      const widthOf = (templateId: number) => templateOf(templateId).laneWidthM;
      const otherBefore = widthOf(other!.id);
      const otherOffsetBefore = templateOf(other!.id).alignmentOffsetFromLeftM;

      // Widening the strips moves the section's outer ends, so the caller says
      // where the alignment should end up. This one keeps the road centred:
      // 2.5 + 3.5 + 3.5 + 2.5 = 12m wide, with the curbs coming out of the
      // walkways rather than adding to them.
      const edited = road.updateRoadLayoutTemplate({
        id: sectionId,
        sidewalkWidthM: 2.5,
        laneWidthM: 3.5,
        medianWidthM: 2,
        alignmentOffsetFromLeftM: 6.0,
        hasCenterLine: true,
        hasOuterLines: true
      });
      expect(edited.ok, edited.error).toBe(true);
      expect(widthOf(sectionId)).toBeCloseTo(3.5, 9);
      expect(templateOf(sectionId).alignmentOffsetFromLeftM).toBeCloseTo(6.0, 9);
      expect(widthOf(other!.id)).toBe(otherBefore);
      expect(templateOf(other!.id).alignmentOffsetFromLeftM).toBe(otherOffsetBefore);
    } finally {
      road.delete();
    }
  });

  it("builds the Japanese two-lane surface from a clicked line", () => {
    const added = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 24,
      endY: 0,
      handleAX: 8,
      handleAY: 0,
      handleBX: 16,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      connectToFirstNode: false
    });

    expect(added.ok, added.error).toBe(true);
    const scene = state.scene();
    expect(scene.segmentCount).toBe(1);
    expect(scene.corridorCount).toBe(1);
    expect(scene.corridors).toHaveLength(1);
    expect(scene.corridors[0].segments).toEqual([
      expect.objectContaining({ segmentId: added.segmentId, reversed: false })
    ]);
    expect(new Set(scene.surfaceMeshes.map((mesh) => mesh.material))).toEqual(
      new Set(["asphalt", "sidewalk", "curb"])
    );
    expect(scene.surfaceMeshes[0].vertices.length).toBeGreaterThan(0);
    expect(scene.surfaceMeshes[0].indices.length).toBeGreaterThan(0);
    expect(scene.surfaceMeshes[0].normals.length).toBe(scene.surfaceMeshes[0].vertices.length);
    expect(scene.surfaceMeshes[0].uv0.length).toBe(
      Math.floor(scene.surfaceMeshes[0].vertices.length / 3) * 2
    );
    expect(scene.surfaceMeshes[0].materialGroups).toEqual([
      expect.objectContaining({ material: scene.surfaceMeshes[0].material, indexStart: 0 })
    ]);
    expect(scene.markingMeshes.length).toBeGreaterThan(0);
    expect(scene.markingMeshes[0].normals.length).toBe(scene.markingMeshes[0].vertices.length);
    expect(scene.markingMeshes[0].uv0.length).toBe(
      Math.floor(scene.markingMeshes[0].vertices.length / 3) * 2
    );
    expect(scene.nodes.length).toBe(2);
    expect(scene.centerlineSegments.length).toBe(1);

    const restored = createRoadState();
    const loaded = restored.loadState(state.saveState());
    expect(loaded.ok, loaded.error).toBe(true);
    expect(restored.scene().segmentCount).toBe(1);
    restored.delete();
  });

  it("exposes stable lane paths and commits an outer lane", () => {
    clearRoad(state);
    const added = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 60,
      endY: 0,
      handleAX: 20,
      handleAY: 0,
      handleBX: 40,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(added.ok, added.error).toBe(true);

    const initial = state.scene();
    expect(initial.lanePaths).toEqual(expect.arrayContaining([
      expect.objectContaining({
        segmentId: added.segmentId,
        laneId: 1000,
        direction: 1,
        startRoadLayoutTemplateId: 1,
        endRoadLayoutTemplateId: 1
      }),
      expect.objectContaining({
        segmentId: added.segmentId,
        laneId: 1010,
        direction: 0,
        startRoadLayoutTemplateId: 1,
        endRoadLayoutTemplateId: 1
      })
    ]));
    for (const lane of initial.lanePaths) {
      expect(lane.points.length).toBeGreaterThanOrEqual(6);
      expect(lane.points.every(Number.isFinite)).toBe(true);
    }

    const request = {
      corridorId: added.corridorId!,
      direction: 0 as const,
      side: "right" as const,
      startSegmentId: added.segmentId!,
      startT: 0.25,
      completeSegmentId: added.segmentId!,
      completeT: 1,
      laneWidthM: 3
    };
    expect(state.scene().transitionCount).toBe(0);

    const committed = state.addLane(request);
    expect(committed.ok, committed.error).toBe(true);
    expect(committed.laneId).toBeGreaterThan(0);
    const after = state.scene();
    expect(after.transitionCount).toBe(1);
    expect(after.lanePaths.some((lane) =>
      lane.segmentId === added.segmentId && lane.laneId === committed.laneId
    )).toBe(true);
  });

  it("rejects an Add Lane taper across segment boundaries", () => {
    clearRoad(state);
    const first = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 60,
      endY: 0,
      handleAX: 20,
      handleAY: 0,
      handleBX: 40,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: 0,
      connectToFirstNode: false
    });
    expect(first.ok, first.error).toBe(true);
    const second = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 60,
      startY: 0,
      endX: 120,
      endY: 0,
      handleAX: 80,
      handleAY: 0,
      handleBX: 100,
      handleBY: 0,
      startNodeId: first.endNodeId!,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: first.corridorId!,
      connectToFirstNode: false
    });
    expect(second.ok, second.error).toBe(true);

    const before = state.scene();
    const committed = state.addLane({
      corridorId: first.corridorId!,
      direction: 0,
      side: "right",
      startSegmentId: first.segmentId!,
      startT: 0.5,
      completeSegmentId: second.segmentId!,
      completeT: 0.5,
      laneWidthM: 3
    });
    expect(committed.ok).toBe(false);
    expect(committed.error).toContain("taper must stay within one road segment");
    const scene = state.scene();
    expect(scene.segmentCount).toBe(before.segmentCount);
    expect(scene.transitionCount).toBe(before.transitionCount);
    expect(scene.corridors[0].segments).toHaveLength(before.corridors[0].segments.length);
  });


  it("extends a degree-one corridor by adding a local segment", () => {
    clearRoad(state);
    const first = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 20,
      endY: 0,
      handleAX: 20 / 3,
      handleAY: 0,
      handleBX: 40 / 3,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: 0,
      connectToFirstNode: false
    });
    expect(first.ok, first.error).toBe(true);
    expect(first.segmentId).toBeGreaterThan(0);
    expect(first.endNodeId).toBeGreaterThan(0);

    const extended = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 20.2,
      startY: 0,
      endX: 32,
      endY: 16,
      handleAX: 24,
      handleAY: 16 / 3,
      handleBX: 28,
      handleBY: 32 / 3,
      startNodeId: first.endNodeId!,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: first.corridorId!,
      connectToFirstNode: false
    });
    expect(extended.ok, extended.error).toBe(true);
    expect(extended.segmentId).not.toBe(first.segmentId);
    expect(extended.endNodeId).not.toBe(first.endNodeId);
    expect(extended.corridorId).toBe(first.corridorId);
    const scene = state.scene();
    expect(scene.segmentCount).toBe(2);
    expect(scene.corridorCount).toBe(1);
    expect(scene.nodes).toHaveLength(3);
    expect(new Set(scene.centerlineSegments.map((segment) => segment.id))).toEqual(
      new Set([first.segmentId!, extended.segmentId!])
    );
    const editedExtension = scene.editableSegments.find(
      (segment) => segment.id === extended.segmentId
    );
    expect(editedExtension).toMatchObject({
      kind: "line",
      nodeAId: first.endNodeId,
      nodeBId: extended.endNodeId
    });
    expect(editedExtension?.points).toHaveLength(4);
    expect(editedExtension?.points[0]).toEqual({ x: 20, y: 0 });
    expect(editedExtension?.points[3]).toEqual({ x: 32, y: 16 });
    expect(scene.corridors[0]).toMatchObject({
      id: first.corridorId,
      segments: [
        { segmentId: first.segmentId, reversed: false },
        { segmentId: extended.segmentId, reversed: false }
      ]
    });
    const endpoint = scene.nodes.find((node) => node.id === extended.endNodeId);
    expect(endpoint).toMatchObject({
      x: 32,
      y: 16,
      extensionCorridorId: first.corridorId
    });
  });

  it("exposes local split through the wasm boundary", () => {
    clearRoad(state);
    const added = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 60,
      endY: 0,
      handleAX: 20,
      handleAY: 0,
      handleBX: 40,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: 0,
      connectToFirstNode: false
    });
    expect(added.ok, added.error).toBe(true);

    const split = state.splitSegmentAtDistance({
      segmentId: added.segmentId!,
      segmentDistanceM: 20
    });
    expect(split.ok, split.error).toBe(true);
    let scene = state.scene();
    expect(scene.segmentCount).toBe(2);
    expect(scene.corridorCount).toBe(1);
    expect(scene.corridors[0].segments).toHaveLength(2);

  });

  it("exposes exact segment ownership and standard deletion through wasm", () => {
    clearRoad(state);
    const first = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 20,
      endY: 0,
      handleAX: 20 / 3,
      handleAY: 0,
      handleBX: 40 / 3,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: 0,
      connectToFirstNode: false
    });
    expect(first.ok, first.error).toBe(true);
    const middle = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 20,
      startY: 0,
      endX: 40,
      endY: 0,
      handleAX: 80 / 3,
      handleAY: 0,
      handleBX: 100 / 3,
      handleBY: 0,
      startNodeId: first.endNodeId!,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: first.corridorId!,
      connectToFirstNode: false
    });
    expect(middle.ok, middle.error).toBe(true);
    const last = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 40,
      startY: 0,
      endX: 60,
      endY: 0,
      handleAX: 140 / 3,
      handleAY: 0,
      handleBX: 160 / 3,
      handleBY: 0,
      startNodeId: middle.endNodeId!,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: first.corridorId!,
      connectToFirstNode: false
    });
    expect(last.ok, last.error).toBe(true);

    const before = state.scene();
    expect(before.surfaceMeshes.some(
      (mesh) => mesh.ownerSegmentId === middle.segmentId
    )).toBe(true);
    expect(before.markingMeshes.some(
      (mesh) => mesh.ownerSegmentId === middle.segmentId
    )).toBe(true);

    const deleted = state.deleteSegment(middle.segmentId!);
    expect(deleted.ok, deleted.error).toBe(true);
    const after = state.scene();
    expect(after.segmentCount).toBe(2);
    expect(after.corridorCount).toBe(2);
    expect(after.centerlineSegments.map((segment) => segment.id)).toEqual(
      expect.arrayContaining([first.segmentId!, last.segmentId!])
    );
    expect(after.centerlineSegments.some(
      (segment) => segment.id === middle.segmentId
    )).toBe(false);
    expect(after.surfaceMeshes.some(
      (mesh) => mesh.ownerSegmentId === middle.segmentId
    )).toBe(false);
    expect(after.markingMeshes.some(
      (mesh) => mesh.ownerSegmentId === middle.segmentId
    )).toBe(false);
  });

  it("keeps one continuous multi-span drawing as one deletion unit through wasm", () => {
    clearRoad(state);
    const added = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 20,
      endY: 0,
      handleAX: 20 / 3,
      handleAY: 0,
      handleBX: 40 / 3,
      handleBY: 0,
      spans: [
        {
          kind: "line",
          startX: 0,
          startY: 0,
          endX: 20,
          endY: 0,
          handleAX: 20 / 3,
          handleAY: 0,
          handleBX: 40 / 3,
          handleBY: 0
        },
        {
          kind: "line",
          startX: 20,
          startY: 0,
          endX: 34,
          endY: 12,
          handleAX: 74 / 3,
          handleAY: 4,
          handleBX: 88 / 3,
          handleBY: 8
        }
      ],
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: 0,
      connectToFirstNode: false
    });
    expect(added.ok, added.error).toBe(true);

    const before = state.scene();
    expect(before.segmentCount).toBe(1);
    expect(before.centerlineSegments.length).toBeGreaterThan(1);
    expect(before.centerlineSegments.every(
      (segment) => segment.id === added.segmentId
    )).toBe(true);
    expect(before.surfaceMeshes.length).toBeGreaterThan(0);
    expect(before.surfaceMeshes.every(
      (mesh) => mesh.ownerSegmentId === added.segmentId
    )).toBe(true);
    expect(before.markingMeshes.length).toBeGreaterThan(0);
    expect(before.markingMeshes.every(
      (mesh) => mesh.ownerSegmentId === added.segmentId
    )).toBe(true);

    const deleted = state.deleteSegment(added.segmentId!);
    expect(deleted.ok, deleted.error).toBe(true);
    const after = state.scene();
    expect(after.segmentCount).toBe(0);
    expect(after.surfaceMeshes).toHaveLength(0);
    expect(after.markingMeshes).toHaveLength(0);
  });

  it("previews and commits endpoint movement through the node operation", () => {
    clearRoad(state);
    const added = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 30,
      endY: 0,
      handleAX: 10,
      handleAY: 0,
      handleBX: 20,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      extensionCorridorId: 0,
      connectToFirstNode: false
    });
    expect(added.ok, added.error).toBe(true);
    const before = state.scene();
    const editable = before.editableSegments.find(
      (segment) => segment.id === added.segmentId
    );
    expect(editable).toBeDefined();

    const preview = state.previewMoveNode({
      nodeId: editable!.nodeBId,
      x: 30,
      y: 8
    });
    expect(preview.ok, preview.error).toBe(true);
    expect(preview.meshes.length).toBeGreaterThan(0);
    expect(state.scene().nodes.find((node) => node.id === editable!.nodeBId))
      .toMatchObject({ x: 30, y: 0 });

    const moved = state.moveNode({
      nodeId: editable!.nodeBId,
      x: 30,
      y: 8
    });
    expect(moved.ok, moved.error).toBe(true);
    expect(state.scene().nodes.find((node) => node.id === editable!.nodeBId))
      .toMatchObject({ x: 30, y: 8 });
  });

  it("keeps the final cross section perpendicular to an angled road", () => {
    clearRoad(state);
    const added = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 24,
      endY: 12,
      handleAX: 8,
      handleAY: 4,
      handleBX: 16,
      handleBY: 8,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(added.ok, added.error).toBe(true);
    const vertices = state.scene().surfaceMeshes[0].vertices;
    const rowCount = Math.ceil(Math.hypot(24, 12) / 2) + 1;
    const rowWidth = vertices.length / 3 / rowCount;
    const first = (rowCount - 1) * rowWidth * 3;
    const last = first + (rowWidth - 1) * 3;
    const crossX = vertices[last] - vertices[first];
    const crossY = vertices[last + 1] - vertices[first + 1];
    const tangentLength = Math.hypot(24, 12);
    const dot = crossX * 24 / tangentLength + crossY * 12 / tangentLength;
    expect(Math.abs(dot)).toBeLessThan(1e-6);
  });

  it("derives a curved degree-two connector without junction markings", () => {
    clearRoad(state);
    const base = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line", startX: 0, startY: 0, endX: 20, endY: 0,
      handleAX: 6, handleAY: 0, handleBX: 14, handleBY: 0,
      startNodeId: 0, startSegmentId: 0, startSegmentDistanceM: 0, connectToFirstNode: false
    });
    expect(base.ok, base.error).toBe(true);
    const endpoint = state.scene().nodes.find((node) => Math.abs(node.x - 20) < 1e-6 && Math.abs(node.y) < 1e-6);
    expect(endpoint).toBeDefined();
    const corner = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line", startX: 20, startY: 0, endX: 20, endY: 24,
      handleAX: 20, handleAY: 8, handleBX: 20, handleBY: 16,
      startNodeId: endpoint!.id, startSegmentId: 0, startSegmentDistanceM: 0, connectToFirstNode: false
    });
    expect(corner.ok, corner.error).toBe(true);
    const scene = state.scene();
    expect(scene.junctionCount).toBe(0);
    expect(scene.markingMeshes.length).toBeGreaterThanOrEqual(2);
    expect(scene.markingMeshes.some((mesh) => mesh.material === "road_marking_stop")).toBe(false);
    expect(scene.markingMeshes.some((mesh) => mesh.material === "road_marking_crosswalk")).toBe(false);
    expect(scene.surfaceMeshes.filter((mesh) => mesh.material === "asphalt").length).toBeGreaterThan(2);
    expect(scene.surfaceMeshes.filter((mesh) => mesh.material === "sidewalk").length).toBeGreaterThan(2);
    expect(scene.surfaceMeshes.filter((mesh) => mesh.material === "curb").length).toBeGreaterThan(2);
  });

  it("splits a straight segment when a branch starts from a centerline snap", () => {
    clearRoad(state);
    const base = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0,
      startY: 0,
      endX: 40,
      endY: 0,
      handleAX: 13,
      handleAY: 0,
      handleBX: 27,
      handleBY: 0,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(base.ok, base.error).toBe(true);
    const baseSegmentId = state.scene().centerlineSegments[0].id;
    const branch = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 20,
      startY: 0,
      endX: 32,
      endY: 24,
      handleAX: 24,
      handleAY: 8,
      handleBX: 28,
      handleBY: 16,
      startNodeId: 0,
      startSegmentId: baseSegmentId,
      startSegmentDistanceM: 20,
      connectToFirstNode: false
    });
    expect(branch.ok, branch.error).toBe(true);
    const scene = state.scene();
    expect(scene.segmentCount).toBe(3);
    expect(scene.nodes.length).toBe(4);
    expect(scene.connectionGateCount).toBe(3);
    expect(scene.junctionCount).toBe(1);
    expect(scene.centerlineSegments.every((segment) =>
      scene.editableSegments.some((editable) => editable.id === segment.id)
    )).toBe(true);
    expect(scene.surfaceMeshes.filter((mesh) => mesh.material === "asphalt").length).toBeGreaterThan(3);
    expect(scene.surfaceMeshes.filter((mesh) => mesh.material === "sidewalk").length).toBeGreaterThan(3);
    expect(scene.surfaceMeshes.filter((mesh) => mesh.material === "curb").length).toBeGreaterThan(3);
    expect(scene.markingMeshes.length).toBeGreaterThanOrEqual(9);
  });

  it("keeps a zero-radius T local when the existing road uses 4m", () => {
    clearRoad(state);
    const base = state.addSegment({
      roadLayoutTemplateId: sectionId,
      junctionCornerRadiusM: 4,
      kind: "line",
      startX: -40, startY: 0, endX: 40, endY: 0,
      handleAX: -13, handleAY: 0, handleBX: 13, handleBY: 0,
      startNodeId: 0, startSegmentId: 0, startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(base.ok, base.error).toBe(true);
    const baseSegmentId = state.scene().centerlineSegments[0].id;
    const branch = state.addSegment({
      roadLayoutTemplateId: sectionId,
      junctionCornerRadiusM: 0,
      kind: "line",
      startX: 0, startY: 0, endX: 0, endY: 40,
      handleAX: 0, handleAY: 13, handleBX: 0, handleBY: 27,
      startNodeId: 0, startSegmentId: baseSegmentId,
      startSegmentDistanceM: 40, connectToFirstNode: false
    });
    expect(branch.ok, branch.error).toBe(true);
    const scene = state.scene();
    const crosswalks = scene.markingMeshes.filter(
      (mesh) => mesh.material === "road_marking_crosswalk"
    );
    expect(crosswalks.length).toBeGreaterThan(0);
    const farthestCoordinate = Math.max(...crosswalks.flatMap((mesh) =>
      Array.from(mesh.vertices).filter((_value, index) => index % 3 !== 2).map(Math.abs)
    ));
    expect(farthestCoordinate).toBeLessThanOrEqual(10);
  });

  it("splits a straight segment when a branch ends at a centerline snap", () => {
    clearRoad(state);
    const base = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0, startY: 0, endX: 40, endY: 0,
      handleAX: 13, handleAY: 0, handleBX: 27, handleBY: 0,
      startNodeId: 0, startSegmentId: 0, startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(base.ok, base.error).toBe(true);
    const baseSegmentId = state.scene().centerlineSegments[0].id;
    const branch = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 20, startY: 24, endX: 20, endY: 0,
      handleAX: 20, handleAY: 16, handleBX: 20, handleBY: 8,
      startNodeId: 0, startSegmentId: 0, startSegmentDistanceM: 0,
      endNodeId: 0, endSegmentId: baseSegmentId, endSegmentDistanceM: 20,
      connectToFirstNode: false
    });
    expect(branch.ok, branch.error).toBe(true);
    const scene = state.scene();
    expect(scene.segmentCount).toBe(3);
    expect(scene.nodes.length).toBe(4);
    expect(scene.connectionGateCount).toBe(3);
    expect(scene.junctionCount).toBe(1);
  });

  it("connects two existing roads in one atomic interval", () => {
    clearRoad(state);
    const lower = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: -20, startY: 0, endX: 20, endY: 0,
      handleAX: -7, handleAY: 0, handleBX: 7, handleBY: 0,
      startNodeId: 0, startSegmentId: 0, startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    const upper = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: -20, startY: 40, endX: 20, endY: 40,
      handleAX: -7, handleAY: 40, handleBX: 7, handleBY: 40,
      startNodeId: 0, startSegmentId: 0, startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(lower.ok && upper.ok).toBe(true);
    const [lowerId, upperId] = state.scene().editableSegments.map((item) => item.id);
    const preview = state.previewSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0, startY: 0, endX: 0, endY: 40,
      handleAX: 0, handleAY: 13, handleBX: 0, handleBY: 27,
      startNodeId: 0, startSegmentId: lowerId, startSegmentDistanceM: 20,
      endNodeId: 0, endSegmentId: upperId, endSegmentDistanceM: 20,
      connectToFirstNode: false
    });
    expect(preview.ok, preview.error).toBe(true);
    expect(preview.meshes.length).toBeGreaterThan(0);
    const connected = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: 0, startY: 0, endX: 0, endY: 40,
      handleAX: 0, handleAY: 13, handleBX: 0, handleBY: 27,
      startNodeId: 0, startSegmentId: lowerId, startSegmentDistanceM: 20,
      endNodeId: 0, endSegmentId: upperId, endSegmentDistanceM: 20,
      connectToFirstNode: false
    });
    expect(connected.ok, connected.error).toBe(true);
    const scene = state.scene();
    expect(scene.segmentCount).toBe(5);
    expect(scene.junctionCount).toBe(2);
    expect(scene.corridorCount).toBe(3);
  });

  it("splits a Bezier segment at the explicit centerline distance", () => {
    clearRoad(state);
    const base = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "bezier",
      startX: 0,
      startY: 0,
      endX: 80,
      endY: 0,
      handleAX: 20,
      handleAY: 10,
      handleBX: 60,
      handleBY: 10,
      startNodeId: 0,
      startSegmentId: 0,
      startSegmentDistanceM: 0,
      connectToFirstNode: false
    });
    expect(base.ok, base.error).toBe(true);
    const centerlines = state.scene().centerlineSegments;
    const total = centerlines.at(-1)!.endSegmentDistanceM;
    const target = centerlines.reduce((best, segment) =>
      Math.abs(segment.startSegmentDistanceM - total * 0.5) < Math.abs(best.startSegmentDistanceM - total * 0.5)
        ? segment
        : best
    );
    const branch = state.addSegment({
      roadLayoutTemplateId: sectionId,
      kind: "line",
      startX: target.startX,
      startY: target.startY,
      endX: target.startX,
      endY: target.startY + 30,
      handleAX: target.startX,
      handleAY: target.startY + 10,
      handleBX: target.startX,
      handleBY: target.startY + 20,
      startNodeId: 0,
      startSegmentId: target.id,
      startSegmentDistanceM: target.startSegmentDistanceM,
      connectToFirstNode: false
    });
    expect(branch.ok, branch.error).toBe(true);
    expect(state.scene().segmentCount).toBe(3);
    expect(state.scene().junctionCount).toBe(1);
  });
});
