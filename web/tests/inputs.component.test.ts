// @vitest-environment happy-dom

import { mount, tick, unmount } from "svelte";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import App from "../src/App.svelte";
import { ViewerActions } from "../src/actions/viewer";
import { WireBridge } from "../src/bridge/wire";
import { CommitFailureCategory } from "../src/model";
import { ViewerStore, type ViewerSnapshot } from "../src/store/viewer";

describe("viewer numeric inputs", () => {
  let bridge: WireBridge | null = null;
  let app: ReturnType<typeof mount> | null = null;

  afterEach(async () => {
    if (app !== null) await unmount(app);
    bridge?.dispose();
    document.body.replaceChildren();
  });

  it("initializes Bundle placement with visible template order and absolute heights", async () => {
    const mounted = await mountViewer(false);
    const snapshot = current(mounted.store);
    const visible = snapshot.drawBundlePlacements
      .map((placement) => ({
        placement,
        template: snapshot.bundleTemplates.find((template) =>
          template.id === placement.bundleTemplateId
        )
      }))
      .filter((entry) => entry.template !== undefined &&
        entry.template.category !== 4);

    expect(visible.map((entry) => entry.template!.category)).not.toContain(4);
    expect(visible[0].template!.category).toBe(0);
    expect(visible.map((entry) => entry.placement.height)).toEqual(
      [...visible.map((entry) => entry.placement.height)].sort((a, b) => b - a)
    );
    expect(visible[0].placement.height).toBeGreaterThan(0);
    expect(visible[0].placement.height).toBeCloseTo(9.2, 1);
    const placementPanel = document.querySelector(".bundle-picker");
    expect(placementPanel?.textContent).toContain("HV");
    expect(placementPanel?.textContent).toContain("OPT");
    expect(placementPanel?.textContent).not.toContain("DROP");
    expect([...(placementPanel?.querySelectorAll("button") ?? [])]
      .some((button) => button.textContent?.trim() === "Remove")).toBe(false);
    expect(placementPanel?.querySelectorAll('input[aria-label$="count"]')).toHaveLength(0);
  });

  it("updates existing Bundle placement without regenerating the route", async () => {
    const mounted = await mountViewer();
    expect(current(mounted.store).pathPoints).toHaveLength(0);
    const source = current(mounted.store).drawBundlePlacements[0];
    expect(source.generatedBundleId).toBeDefined();
    const beforeScene = mounted.bridge.scene();
    const beforeLogCount = current(mounted.store).logs.length;
    const beforeZ = Math.max(...mounted.bridge.scene().ports.map((port) => port.z));
    mounted.actions.updateDrawBundlePlacement(source.id, { height: source.height + 0.5 });
    await tick();
    const afterScene = mounted.bridge.scene();
    const afterZ = Math.max(...afterScene.ports.map((port) => port.z));

    expect(afterScene.poles).toHaveLength(beforeScene.poles.length);
    expect(afterScene.spans).toHaveLength(beforeScene.spans.length);
    expect(current(mounted.store).logs).toHaveLength(beforeLogCount);
    expect(afterZ).toBeGreaterThan(beforeZ + 0.25);
  });

  it("duplicates a Bundle placement and generates independent support and helix output", async () => {
    const mounted = await mountViewer(false);
    const optical = current(mounted.store).bundleTemplates.find((template) =>
      template.category === 3
    );
    const source = current(mounted.store).drawBundlePlacements.find(
      (placement) => placement.bundleTemplateId === optical?.id
    );
    expect(optical).toBeDefined();
    expect(source).toBeDefined();

    const duplicate = [...document.querySelectorAll("button")].find(
      (button) => button.getAttribute("aria-label") === `Duplicate ${optical!.name}`
    );
    expect(duplicate).toBeInstanceOf(HTMLButtonElement);
    (duplicate as HTMLButtonElement).click();
    await tick();

    const copies = current(mounted.store).drawBundlePlacements.filter(
      (placement) => placement.bundleTemplateId === optical!.id
    );
    expect(copies).toHaveLength(2);
    const placementIds = current(mounted.store).drawBundlePlacements.map((placement) => placement.id);
    expect(placementIds.indexOf(copies[1].id)).toBe(placementIds.indexOf(copies[0].id) + 1);
    expect(document.body.textContent).toContain("OPT 1");
    expect(document.body.textContent).toContain("OPT 2");
    mounted.actions.updateDrawBundlePlacement(copies[1].id, {
      height: 8.4,
      offset: 0.4,
      spacing: 0.47
    });
    mounted.actions.addPathPoint([0, 0, 0]);
    mounted.actions.addPathPoint([20, 0, 0]);
    mounted.actions.generatePath();

    const opticalParts = mounted.bridge.scene().parts.filter(
      (part) => part.info.bundleTemplateId === optical!.id
    );
    const bundleIds = [...new Set(opticalParts.map((part) => part.info.sourceBundleId))];
    expect(bundleIds).toHaveLength(2);
    for (const bundleId of bundleIds) {
      expect(opticalParts.filter((part) =>
        part.info.sourceBundleId === bundleId && part.info.supplementalKind === 1
      )).toHaveLength(1);
      expect(opticalParts.filter((part) =>
        part.info.sourceBundleId === bundleId && part.info.supplementalKind === 2
      )).toHaveLength(1);
    }
  });

  it("applies Settings number input to core geometry", async () => {
    const mounted = await mountViewer();
    const input = inputForLabel("Sag factor");
    const next = Number(input.value) + 0.01;
    input.value = String(next);
    input.dispatchEvent(new Event("input", { bubbles: true }));
    await new Promise((resolve) => setTimeout(resolve, 40));
    input.dispatchEvent(new FocusEvent("blur", { bubbles: true }));
    await tick();

    expect(mounted.bridge.geometrySettings().sagFactor).toBeCloseTo(next, 8);
  });

  it("starts with 24 curve samples and toggles the ground grid", async () => {
    const mounted = await mountViewer();
    expect(mounted.bridge.geometrySettings().curveSamples).toBe(24);
    const grid = inputForLabel("Ground grid");
    expect(grid.checked).toBe(true);
    grid.checked = false;
    grid.dispatchEvent(new Event("change", { bubbles: true }));
    await tick();
    expect(current(mounted.store).showGroundGrid).toBe(false);
  });

  it("applies bundle population input to the selected template", async () => {
    const mounted = await mountViewer();
    let selectedId: number | null = null;
    const unsubscribe = mounted.store.value.subscribe((snapshot) => {
      selectedId = snapshot.selectedBundleTemplateId;
    });
    unsubscribe();
    let before = mounted.bridge.bundleTemplates()
      .find((template) => template.id === selectedId);
    expect(before).toBeDefined();
    if (before!.populationRules.length === 0) {
      mounted.actions.commitBundleTemplate({
        ...before!,
        populationRules: [{
          ruleId: 1,
          explicitSeed: 1,
          priority: 0,
          minExtraCount: 1,
          maxExtraCount: 1,
          minSpacing: 0.05,
          lateralMin: -1,
          lateralMax: 1,
          heightMin: 0,
          heightMax: 20
        }]
      });
      await tick();
      before = mounted.bridge.bundleTemplates()
        .find((template) => template.id === selectedId);
    }
    const rule = before!.populationRules[0];
    const input = inputForDetails("Population rules", "Min spacing");
    const next = rule.minSpacing + 0.01;
    input.value = String(next);
    input.dispatchEvent(new Event("input", { bubbles: true }));
    await new Promise((resolve) => setTimeout(resolve, 40));
    input.dispatchEvent(new FocusEvent("blur", { bubbles: true }));
    await tick();

    const after = mounted.bridge.bundleTemplates()
      .find((template) => template.id === selectedId);
    expect(after?.populationRules.find((candidate) => candidate.ruleId === rule.ruleId)
      ?.minSpacing).toBeCloseTo(next, 8);
  });

  it("exposes helix controls and applies them to the selected bundle template", async () => {
    const mounted = await mountViewer();
    const selectedId = current(mounted.store).selectedBundleTemplateId;
    const before = mounted.bridge.bundleTemplates()
      .find((template) => template.id === selectedId);
    expect(before).toBeDefined();

    const heading = [...document.querySelectorAll("h3")]
      .find((candidate) => candidate.textContent?.trim() === "Helix");
    expect(heading).toBeDefined();

    const input = inputForLabel("Helix clearance");
    const next = before!.spanVisualAssembly.helixClearance + 0.005;
    input.value = String(next);
    input.dispatchEvent(new Event("change", { bubbles: true }));
    await tick();

    const after = mounted.bridge.bundleTemplates()
      .find((template) => template.id === selectedId);
    expect(after?.spanVisualAssembly.helixClearance).toBeCloseTo(next, 8);
  });

  it("restores an uncommitted number input with Escape before blur", async () => {
    const mounted = await mountViewer();
    const input = inputForLabel("Max tilt");
    const before = current(mounted.store).maxTiltDeg;

    input.dispatchEvent(new FocusEvent("focusin", { bubbles: true }));
    input.value = String(before + 7);
    input.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }));
    input.dispatchEvent(new FocusEvent("blur", { bubbles: true }));
    await tick();

    expect(input.value).toBe(String(before));
    expect(current(mounted.store).maxTiltDeg).toBe(before);
  });

  it("keeps generated backbone visible after generation", async () => {
    const mounted = await mountViewer();
    mounted.actions.setDrawOption("showBackboneOverlay", false);
    mounted.actions.addPathPoint([0, 8, 0]);
    mounted.actions.addPathPoint([8, 8, 0]);
    mounted.actions.generatePath();

    const snapshot = current(mounted.store);
    expect(snapshot.parts.length).toBeGreaterThan(0);
    expect(snapshot.showBackboneOverlay).toBe(true);
  });

  it("clears the in-progress draw path with Escape", async () => {
    const mounted = await mountViewer();
    const generatedParts = current(mounted.store).parts;
    mounted.actions.addPathPoint([1, 0, 0]);
    mounted.actions.addPathPoint([2, 0, 0]);
    expect(current(mounted.store).pathPoints).toHaveLength(2);

    window.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }));
    await tick();

    expect(current(mounted.store).pathPoints).toEqual([]);
    expect(current(mounted.store).parts).toEqual(generatedParts);
  });

  it("keeps road drawing on the viewport tool instead of a form submit panel", async () => {
    const mounted = await mountViewer(false);
    expect(document.body.textContent).toContain("DRAW PATH");
    expect(document.querySelector('[aria-label="Road tool"]')).toBeInstanceOf(HTMLButtonElement);

    (document.querySelector('[aria-label="Road tool"]') as HTMLButtonElement).click();
    await tick();
    expect(current(mounted.store).activeTool).toBe("road");
    expect(document.querySelector('[aria-label="Apply section transition"]')).toBeNull();
    expect(document.querySelector('[aria-label="Circular arc road"]')).toBeNull();
    expect(current(mounted.store).rightPanelMode).toBe("wire");
    expect(document.body.textContent).toContain("DRAW PATH");
    expect(document.body.textContent).not.toContain("Add segment");
    expect(document.body.textContent).toContain("Repro capture");

    mounted.actions.addViewportPoint([0, 0, 0]);
    mounted.actions.previewViewportPoint([24, 0, 0]);
    expect(current(mounted.store).road.previewState).toBe("guide");
    expect(current(mounted.store).road.previewRequest).toEqual(expect.objectContaining({
      startX: 0,
      endX: 24
    }));
    expect(current(mounted.store).road.previewMeshes).toEqual([]);
    mounted.actions.addViewportPoint([24, 0, 0]);
    await tick();
    expect(current(mounted.store).road.scene.segmentCount).toBe(1);
    expect(current(mounted.store).road.scene.surfaceMeshes.length).toBeGreaterThan(0);

    const roadTab = [...document.querySelectorAll(".domain-tabs button")]
      .find((button) => button.textContent?.trim() === "Road") as HTMLButtonElement;
    roadTab.click();
    await tick();
    expect(current(mounted.store).rightPanelMode).toBe("road");
    expect(document.body.textContent).not.toContain("Repro capture");

    (document.querySelector('[aria-label="Wire tool"]') as HTMLButtonElement).click();
    await tick();
    expect(current(mounted.store).activeTool).toBe("wire");
    expect(current(mounted.store).rightPanelMode).toBe("road");
  });

  it("shows one categorized commit failure without turning pointer movement into an error", async () => {
    const mounted = await mountViewer(false);
    mounted.actions.setActiveTool("road");
    mounted.actions.addViewportPoint([0, 0, 0]);
    mounted.store.setCommitFailure({
      ok: false,
      error: "road connection needs more length",
      failureCategory: CommitFailureCategory.NotImplemented,
      reasonCode: "road_connection_too_short"
    }, "road segment", [12, 4, 0]);
    await tick();

    const failure = document.querySelector(".commit-failure");
    expect(failure?.textContent).toContain("Not implemented");
    expect(failure?.textContent).toContain("road connection needs more length");
    expect(failure?.textContent).toContain("road_connection_too_short");
    expect(document.querySelectorAll(".commit-failure")).toHaveLength(1);

    mounted.actions.previewViewportPoint([20, 0, 0]);
    await tick();
    expect(document.querySelector(".commit-failure")?.textContent)
      .toContain("road_connection_too_short");
  });

  it("uses one operation error surface and keeps the reason code in details", async () => {
    const mounted = await mountViewer(false);
    mounted.store.update((snapshot) => ({
      ...snapshot,
      error: "legacy operation error",
      road: { ...snapshot.road, lastError: "legacy road panel error" }
    }));
    mounted.store.setCommitFailure({
      ok: false,
      error: "the selected lane cannot be extended",
      failureCategory: CommitFailureCategory.NotImplemented,
      reasonCode: "lane_destination_ambiguous"
    }, "road add lane");
    await tick();

    expect(document.querySelectorAll('[role="alert"]')).toHaveLength(1);
    expect(document.querySelector(".road-error")).toBeNull();
    const details = document.querySelector(".commit-failure details");
    expect(details).toBeInstanceOf(HTMLDetailsElement);
    expect(details?.hasAttribute("open")).toBe(false);
    expect(details?.querySelector("code")?.textContent)
      .toContain("lane_destination_ambiguous");
  });

  it("returns an explicit outcome for every primary click and Enter action", async () => {
    const mounted = await mountViewer(false);

    expect(mounted.actions.addViewportPoint([0, 0, 0])).toEqual({ kind: "anchor-accepted" });
    mounted.actions.previewViewportPoint([12, 0, 0]);
    expect(mounted.actions.finishDrawSession()).toEqual({ kind: "commit-succeeded" });
    expect(current(mounted.store).lastDrawActionResult).toEqual({ kind: "commit-succeeded" });

    mounted.actions.setActiveTool("road");
    expect(mounted.actions.finishDrawSession()).toEqual({
      kind: "ignored",
      reasonCode: "session-inactive"
    });
    expect(mounted.actions.addViewportPoint([0, 40, 0])).toEqual({ kind: "anchor-accepted" });
    expect(mounted.actions.finishDrawSession()).toEqual({ kind: "session-ended" });
    expect(current(mounted.store).road.phase).toBe("start");
  });

  it("offers the same confirm finish cancel and undo controls on screen", async () => {
    const mounted = await mountViewer(false);
    mounted.actions.setActiveTool("road");
    mounted.actions.addViewportPoint([0, 0, 0]);
    mounted.actions.previewViewportPoint([20, 0, 0]);
    await tick();

    const controls = document.querySelector('[aria-label="Drawing session controls"]');
    expect(controls).toBeInstanceOf(HTMLElement);
    const button = (label: string) => controls?.querySelector(
      `button[aria-label="${label}"]`
    ) as HTMLButtonElement;
    expect(button("Confirm").disabled).toBe(false);
    expect(button("Finish").disabled).toBe(false);
    expect(button("Cancel").disabled).toBe(false);
    expect(button("Undo")).toBeInstanceOf(HTMLButtonElement);

    button("Confirm").click();
    await tick();
    expect(current(mounted.store).road.scene.segmentCount).toBe(1);
    expect(current(mounted.store).road.phase).toBe("end");

    mounted.actions.previewViewportPoint([40, 0, 0]);
    await tick();
    button("Finish").click();
    await tick();
    expect(current(mounted.store).road.scene.segmentCount).toBe(2);
    expect(current(mounted.store).road.phase).toBe("start");

    button("Undo").click();
    await tick();
    expect(current(mounted.store).road.scene.segmentCount).toBe(1);
  });

  it("runs repeated wire and road sessions through ViewerActions and the real wasm bridge", async () => {
    const mounted = await mountViewer(false);

    for (let index = 0; index < 12; index += 1) {
      const x = index * 30;
      expect(mounted.actions.addViewportPoint([x, 0, 0])).toEqual({ kind: "anchor-accepted" });
      mounted.actions.previewViewportPoint([x + 12, 0, 0]);
      expect(mounted.actions.addViewportPoint([x + 12, 0, 0])).toEqual({ kind: "commit-succeeded" });
      mounted.actions.previewViewportPoint([x + 12, 10, 0]);
      expect(mounted.actions.finishDrawSession()).toEqual({ kind: "commit-succeeded" });
      expect(current(mounted.store).pathPoints).toEqual([]);
      expect(current(mounted.store).lastCommitFailure).toBeNull();
    }
    expect(current(mounted.store).poles).toHaveLength(36);

    mounted.actions.setActiveTool("road");
    for (let index = 0; index < 10; index += 1) {
      const y = 40 + index * 12;
      expect(mounted.actions.addViewportPoint([0, y, 0])).toEqual({ kind: "anchor-accepted" });
      mounted.actions.previewViewportPoint([18, y, 0]);
      expect(mounted.actions.finishDrawSession()).toEqual({ kind: "commit-succeeded" });
    }
    expect(current(mounted.store).road.scene.segmentCount).toBe(10);

    mounted.actions.setRoadMode("bezier");
    for (let index = 0; index < 10; index += 1) {
      const y = 260 + index * 12;
      expect(mounted.actions.addViewportPoint([0, y, 0])).toEqual({ kind: "anchor-accepted" });
      mounted.actions.previewViewportPoint([18, y + 4, 0]);
      expect(mounted.actions.finishDrawSession()).toEqual({ kind: "commit-succeeded" });
    }
    expect(current(mounted.store).road.scene.segmentCount).toBe(20);

    mounted.actions.setRoadMode("line");
    expect(mounted.actions.addViewportPoint([0, 400, 0])).toEqual({ kind: "anchor-accepted" });
    mounted.actions.previewViewportPoint([0, 400, 0]);
    expect(mounted.actions.finishDrawSession()).toEqual({
      kind: "commit-rejected",
      reasonCode: "invalid_input"
    });
    expect(current(mounted.store).road.phase).toBe("end");
    mounted.actions.previewViewportPoint([18, 400, 0]);
    expect(mounted.actions.finishDrawSession()).toEqual({ kind: "commit-succeeded" });
    expect(current(mounted.store).lastCommitFailure).toBeNull();

    expect(mounted.actions.addViewportPoint([0, 180, 0])).toEqual({ kind: "anchor-accepted" });
    expect(mounted.actions.cancelDrawSession()).toEqual({ kind: "session-ended" });
    mounted.actions.setActiveTool("wire");
    expect(mounted.actions.addViewportPoint([0, 200, 0])).toEqual({ kind: "anchor-accepted" });
    mounted.actions.setActiveTool("road");
    expect(current(mounted.store).pathPoints).toEqual([]);
    expect(mounted.actions.addViewportPoint([0, 220, 0])).toEqual({ kind: "anchor-accepted" });
    mounted.actions.previewViewportPoint([18, 220, 0]);
    expect(mounted.actions.finishDrawSession()).toEqual({ kind: "commit-succeeded" });
  }, 15_000);

  it("blocks the editor when the Web and WASM build identities differ", async () => {
    const mounted = await mountViewer(false);
    mounted.store.update((snapshot) => ({
      ...snapshot,
      buildMismatch: {
        webSourceHash: "source123",
        webVersion: "0.2.0",
        wasmSourceHash: "stale456",
        wasmVersion: "0.2.0"
      }
    }));
    await tick();

    const mismatch = document.querySelector(".build-mismatch");
    expect(mismatch?.getAttribute("role")).toBe("alertdialog");
    expect(mismatch?.textContent).toContain("Web and WASM builds do not match");
    expect(mismatch?.textContent).toContain("source123");
    expect(mismatch?.textContent).toContain("stale456");
  });

  it("resets both core state and viewer settings", async () => {
    const mounted = await mountViewer();
    const factorySagFactor = mounted.bridge.geometrySettings().sagFactor;
    mounted.actions.commitGeometry("sagFactor", factorySagFactor + 0.02);
    mounted.actions.setDrawOption("cameraFov", 81);
    expect(current(mounted.store).parts.length).toBeGreaterThan(0);

    const reset = [...document.querySelectorAll("button")]
      .find((button) => button.textContent?.trim() === "Reset");
    if (!(reset instanceof HTMLButtonElement)) {
      throw new Error("Reset button not found");
    }
    reset.click();
    await tick();

    expect(mounted.bridge.geometrySettings().sagFactor)
      .toBeCloseTo(factorySagFactor, 8);
    expect(current(mounted.store)).toEqual(expect.objectContaining({
      cameraFov: 48,
      pathPoints: [],
      parts: []
    }));
  });

  async function mountViewer(generate = true) {
    const wasmBinary = await readFile(resolve("src/wasm-generated/wire_web_core.wasm"));
    bridge = await WireBridge.create({ wasmBinary });
    const store = new ViewerStore();
    const actions = new ViewerActions(bridge, store);
    actions.initialize();
    actions.setDrawOption("maxTiltDeg", 0);
    if (generate) {
      actions.addPathPoint([0, 0, 0]);
      actions.addPathPoint([20, 0, 0]);
      actions.generatePath();
    }

    const target = document.createElement("div");
    document.body.append(target);
    app = mount(App, {
      target,
      props: { actions, store, mountScene: () => undefined }
    });
    await tick();
    return { bridge, actions, store };
  }
});

function current(store: ViewerStore): ViewerSnapshot {
  let snapshot: ViewerSnapshot | undefined;
  const unsubscribe = store.value.subscribe((value) => {
    snapshot = value;
  });
  unsubscribe();
  if (snapshot === undefined) {
    throw new Error("store did not emit");
  }
  return snapshot;
}

function inputForLabel(text: string): HTMLInputElement {
  const label = [...document.querySelectorAll("label")]
    .find((candidate) => candidate.textContent?.trim().startsWith(text));
  const input = label?.querySelector("input");
  if (!(input instanceof HTMLInputElement)) {
    throw new Error(`input not found for label: ${text}`);
  }
  return input;
}

function inputForDetails(summaryText: string, labelText: string): HTMLInputElement {
  const details = [...document.querySelectorAll("details")]
    .find((candidate) =>
      candidate.querySelector("summary")?.textContent?.trim().startsWith(summaryText)
    );
  const label = [...(details?.querySelectorAll("label") ?? [])]
    .find((candidate) => candidate.textContent?.trim().startsWith(labelText));
  const input = label?.querySelector("input");
  if (!(input instanceof HTMLInputElement)) {
    throw new Error(`input not found: ${summaryText} / ${labelText}`);
  }
  return input;
}
