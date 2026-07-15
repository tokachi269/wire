// @vitest-environment happy-dom

import { mount, tick, unmount } from "svelte";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import App from "../src/App.svelte";
import { ViewerActions } from "../src/actions/viewer";
import { WireBridge } from "../src/bridge/wire";
import { bundleTemplateCategory } from "../src/labels";
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
        bundleTemplateCategory(entry.template) !== 4);

    expect(visible.map((entry) => bundleTemplateCategory(entry.template!))).not.toContain(4);
    expect(bundleTemplateCategory(visible[0].template!)).toBe(0);
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
  });

  it("duplicates a Bundle placement and generates independent support and helix output", async () => {
    const mounted = await mountViewer(false);
    const optical = current(mounted.store).bundleTemplates.find((template) =>
      bundleTemplateCategory(template) === 3
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
    mounted.actions.addPathPoint([0, 0, 0]);
    mounted.actions.addPathPoint([8, 0, 0]);
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
