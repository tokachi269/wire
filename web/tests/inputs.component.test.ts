// @vitest-environment happy-dom

import { mount, tick, unmount } from "svelte";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import App from "../src/App.svelte";
import { ViewerActions } from "../src/actions/viewer";
import { WireBridge } from "../src/bridge/wire";
import { ViewerStore, type ViewerSnapshot } from "../src/store/viewer";

describe("viewer numeric inputs", () => {
  let bridge: WireBridge | null = null;
  let app: ReturnType<typeof mount> | null = null;

  afterEach(async () => {
    if (app !== null) await unmount(app);
    bridge?.dispose();
    document.body.replaceChildren();
  });

  it("applies Placement by category height, offset, and spread to generated ports", async () => {
    const mounted = await mountViewer();
    const polesBefore = mounted.bridge.scene().poles.map((pole) => pole.id);
    const hvPartsBefore = mounted.bridge.scene().parts
      .filter((part) => part.info.bundleTemplateId === 1)
      .map((part) => `${part.info.sourceSpanId}:${part.info.laneIndex}`)
      .sort();
    const nonHvPortsBefore = mounted.bridge.scene().ports
      .filter((port) => port.category !== 0);
    const nonHvPartsBefore = mounted.bridge.scene().parts
      .filter((part) => part.info.bundleTemplateId !== 1)
      .map((part) => ({
        key: `${part.info.bundleTemplateId}:${part.info.sourceSpanId}:${part.info.laneIndex}`,
        samples: [...part.samples]
      }))
      .sort((a, b) => a.key.localeCompare(b.key));
    const before = mounted.bridge.scene().ports.filter((port) => port.category === 0);
    expect(before.length).toBeGreaterThan(0);

    await editNumberInput("High voltage height", 1, 80);

    const afterHeight = mounted.bridge.scene().ports.filter((port) => port.category === 0);
    expect(mounted.bridge.scene().poles.map((pole) => pole.id)).toEqual(polesBefore);
    expect(
      mounted.bridge.scene().parts
        .filter((part) => part.info.bundleTemplateId === 1)
        .map((part) => `${part.info.sourceSpanId}:${part.info.laneIndex}`)
        .sort()
    ).toEqual(hvPartsBefore);
    const nonHvPortsAfter = mounted.bridge.scene().ports
      .filter((port) => port.category !== 0);
    expect(nonHvPortsAfter).toEqual(nonHvPortsBefore);
    const nonHvPartsAfter = mounted.bridge.scene().parts
      .filter((part) => part.info.bundleTemplateId !== 1)
      .map((part) => ({
        key: `${part.info.bundleTemplateId}:${part.info.sourceSpanId}:${part.info.laneIndex}`,
        samples: [...part.samples]
      }))
      .sort((a, b) => a.key.localeCompare(b.key));
    expect(nonHvPartsAfter).toEqual(nonHvPartsBefore);
    const hvSamplesAfter = mounted.bridge.scene().parts
      .filter((part) => part.info.bundleTemplateId === 1)
      .map((part) => [...part.samples]);
    expect(new Set(hvSamplesAfter.map((samples) => samples.join(","))).size)
      .toBe(hvSamplesAfter.length);
    for (const port of before) {
      expect(afterHeight.find((candidate) => candidate.id === port.id)?.z)
        .toBeCloseTo(port.z + 1, 8);
    }

    await editNumberInput("High voltage offset", 0.2, 80);
    const afterOffset = mounted.bridge.scene().ports.filter((port) => port.category === 0);
    expect(horizontalPositionChanged(afterHeight, afterOffset)).toBe(true);

    await editNumberInput("High voltage spread", 0.2, 80);
    const afterSpread = mounted.bridge.scene().ports.filter((port) => port.category === 0);
    expect(horizontalPositionChanged(afterOffset, afterSpread)).toBe(true);
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
          heightMax: 20,
          randomness: 0.25,
        profile: 0,
        wrapRadius: 0,
        wrapTurnsPerMeter: 0,
        wrapPhase: 0,
        wrapDirection: 1,
        endTrim: 0
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

  async function mountViewer() {
    const wasmBinary = await readFile(resolve("src/wasm-generated/wire_web_core.wasm"));
    bridge = await WireBridge.create({ wasmBinary });
    const store = new ViewerStore();
    const actions = new ViewerActions(bridge, store);
    actions.initialize();
    actions.setDrawOption("maxTiltDeg", 0);
    actions.addPathPoint([0, 0, 0]);
    actions.addPathPoint([20, 0, 0]);
    actions.generatePath();

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

function requiredInput(selector: string): HTMLInputElement {
  const input = document.querySelector(selector);
  if (!(input instanceof HTMLInputElement)) {
    throw new Error(`input not found: ${selector}`);
  }
  return input;
}

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

async function editNumberInput(
  label: string,
  delta: number,
  previewDelayMs: number
): Promise<void> {
  const input = requiredInput(`input[aria-label="${label}"]`);
  input.value = String(Number(input.value) + delta);
  input.dispatchEvent(new Event("input", { bubbles: true }));
  await new Promise((resolve) => setTimeout(resolve, previewDelayMs));
  input.dispatchEvent(new FocusEvent("blur", { bubbles: true }));
  await tick();
}

function horizontalPositionChanged(
  before: Array<{ id: string; x: number; y: number }>,
  after: Array<{ id: string; x: number; y: number }>
): boolean {
  return before.some((port) => {
    const next = after.find((candidate) => candidate.id === port.id);
    return next !== undefined && Math.hypot(next.x - port.x, next.y - port.y) > 1e-8;
  });
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
