<script lang="ts">
  import { onMount } from "svelte";
  import type { ViewerActions } from "./actions/viewer";
  import Settings from "./panels/Settings.svelte";
  import Templates from "./panels/Templates.svelte";
  import Outliner from "./panels/Outliner.svelte";
  import SelectionInspector from "./panels/SelectionInspector.svelte";
  import {
    createViewerSnapshot,
    type ViewerSnapshot,
    type ViewerStore
  } from "./store/viewer";

  interface Props {
    actions: ViewerActions;
    store: ViewerStore;
    mountScene: (host: HTMLElement) => void;
  }

  let { actions, store, mountScene }: Props = $props();
  let sceneHost: HTMLElement;
  let snapshot: ViewerSnapshot = $state(createViewerSnapshot());
  let stopResize: (() => void) | null = null;
  let editStart:
    | {
        element: HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement;
        value: string;
        checked: boolean | null;
      }
    | null = null;

  function beginResize(side: "left" | "right", event: PointerEvent): void {
    event.preventDefault();
    const startX = event.clientX;
    const startWidth =
      side === "left" ? snapshot.workspaceLeftWidth : snapshot.workspaceWidth;
    const move = (moveEvent: PointerEvent) => {
      const delta = moveEvent.clientX - startX;
      const width = Math.max(
        side === "left" ? 180 : 260,
        Math.min(side === "left" ? 420 : 560, startWidth + (side === "left" ? delta : -delta))
      );
      actions.setDrawOption(
        side === "left" ? "workspaceLeftWidth" : "workspaceWidth",
        width
      );
    };
    const stop = () => {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", stop);
      stopResize = null;
    };
    stopResize?.();
    stopResize = stop;
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", stop);
  }

  onMount(() => {
    const unsubscribe = store.value.subscribe((value) => {
      snapshot = value;
    });
    mountScene(sceneHost);
    const handleFocusIn = (event: FocusEvent) => {
      const target = event.target;
      if (
        target instanceof HTMLInputElement ||
        target instanceof HTMLSelectElement ||
        target instanceof HTMLTextAreaElement
      ) {
        editStart = {
          element: target,
          value: target.value,
          checked: target instanceof HTMLInputElement ? target.checked : null
        };
      }
    };
    const handleFocusOut = (event: FocusEvent) => {
      if (event.target === editStart?.element) {
        editStart = null;
      }
    };
    const handleKey = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        event.preventDefault();
        const active = event.target;
        if (
          snapshot.interaction &&
          active instanceof HTMLInputElement &&
          (typeof snapshot.interaction.startValue === "number" ||
            typeof snapshot.interaction.startValue === "boolean")
        ) {
          if (active.type === "checkbox") {
            active.checked = Boolean(snapshot.interaction.startValue);
          } else {
            active.value = String(snapshot.interaction.startValue);
          }
        } else if (active === editStart?.element) {
          active.value = editStart.value;
          if (active instanceof HTMLInputElement && editStart.checked !== null) {
            active.checked = editStart.checked;
          }
        }
        actions.cancel(
          active instanceof HTMLInputElement ||
          active instanceof HTMLSelectElement ||
          active instanceof HTMLTextAreaElement
        );
        if (
          active instanceof HTMLInputElement ||
          active instanceof HTMLSelectElement ||
          active instanceof HTMLTextAreaElement
        ) {
          active.blur();
        }
        return;
      }
      const target = event.target as HTMLElement | null;
      const editing =
        target instanceof HTMLInputElement ||
        target instanceof HTMLSelectElement ||
        target instanceof HTMLTextAreaElement ||
        target?.isContentEditable;
      if (event.key === "Enter" && !editing) {
        event.preventDefault();
        actions.generatePath();
      }
    };
    window.addEventListener("focusin", handleFocusIn);
    window.addEventListener("focusout", handleFocusOut);
    window.addEventListener("keydown", handleKey);
    return () => {
      unsubscribe();
      stopResize?.();
      window.removeEventListener("focusin", handleFocusIn);
      window.removeEventListener("focusout", handleFocusOut);
      window.removeEventListener("keydown", handleKey);
    };
  });
</script>

<svelte:head>
  <meta
    name="description"
    content="Saved backbone derived-output viewer"
  />
</svelte:head>

<main>
  <header>
    <div>
      <p class="eyebrow">WIRE</p>
      <h1>Backbone viewer</h1>
    </div>
    <div class="header-actions">
      <button class="secondary panel-toggle" class:active={snapshot.showLeftPanel} type="button"
        onclick={() => actions.setDrawOption("showLeftPanel", !snapshot.showLeftPanel)}>
        Outliner
      </button>
      <button class="secondary panel-toggle" class:active={snapshot.showRightPanel} type="button"
        onclick={() => actions.setDrawOption("showRightPanel", !snapshot.showRightPanel)}>
        Inspector
      </button>
      <button type="button" onclick={() => actions.generatePath()}>
        Path生成
      </button>
    </div>
  </header>

  <section
    class:panels-hidden={!snapshot.showWorkspace}
    class="workspace"
    style={`--left-width:${snapshot.workspaceLeftWidth}px;--right-width:${snapshot.workspaceWidth}px`}
  >
    {#if snapshot.showWorkspace && snapshot.showLeftPanel}
      <div class="left-region">
        <Outliner {actions} {snapshot} />
      </div>
      <button
        class="resize-handle"
        aria-label="左サイドバーの幅を変更"
        type="button"
        onpointerdown={(event) => beginResize("left", event)}
      ></button>
    {/if}

    <div class="viewport" bind:this={sceneHost}></div>

    {#if snapshot.showWorkspace && snapshot.showRightPanel}
      <button
        class="resize-handle"
        aria-label="右サイドバーの幅を変更"
        type="button"
        onpointerdown={(event) => beginResize("right", event)}
      ></button>
      <div class="right-region">
        <div class="draw-panel">
          <p class="panel-label">DRAW PATH</p>
          <label>
            Pole template
            <select
              value={snapshot.selectedPoleTemplateId ?? ""}
              onchange={(event) =>
                actions.selectPoleTemplate(Number(event.currentTarget.value))}
            >
              {#each snapshot.poleTemplates as template}
                <option value={template.id}>{template.name}</option>
              {/each}
            </select>
          </label>
          <fieldset class="bundle-picker">
            <legend>Bundle templates</legend>
            {#each snapshot.bundleTemplates as template}
              <div class="bundle-choice">
                <label class="check">
                  <input type="checkbox"
                    checked={snapshot.selectedDrawBundleTemplateIds.includes(template.id)}
                    onchange={() => actions.toggleDrawBundleTemplate(template.id)} />
                  {template.name}
                </label>
                {#if template.fixedCount}
                  <span>{template.fixedCountValue}</span>
                {:else}
                  <input aria-label={`${template.name} count`} type="number"
                    min={template.minCount} max={template.maxCount}
                    value={snapshot.drawBundleCounts[template.id] ?? template.defaultCount}
                    onchange={(event) =>
                      actions.setDrawBundleCount(template.id, Number(event.currentTarget.value))} />
                {/if}
              </div>
            {/each}
          </fieldset>
          <label class="check"><input type="checkbox" checked={snapshot.showBackboneOverlay}
            onchange={(event) => actions.setDrawOption("showBackboneOverlay", event.currentTarget.checked)} />
            Backbone overlay</label>
          <label class="check"><input type="checkbox" checked={snapshot.showPreview}
            onchange={(event) => actions.setDrawOption("showPreview", event.currentTarget.checked)} />
            Preview</label>
          <label class="check"><input type="checkbox" checked={snapshot.keepPathAfterGenerate}
            onchange={(event) => actions.setDrawOption("keepPathAfterGenerate", event.currentTarget.checked)} />
            Keep path</label>
          <label class="check"><input type="checkbox" checked={snapshot.clickedPointsOnly}
            onchange={(event) => actions.setDrawOption("clickedPointsOnly", event.currentTarget.checked)} />
            Clicked points only</label>
          <label>Interval
            <input type="number" step="0.5" value={snapshot.intervalM}
              disabled={snapshot.clickedPointsOnly}
              onchange={(event) => actions.setDrawOption("intervalM", Number(event.currentTarget.value))} />
          </label>
          <label>Direction
            <select value={snapshot.directionMode}
              onchange={(event) => actions.setDrawOption("directionMode", Number(event.currentTarget.value))}>
              <option value="0">Auto</option><option value="1">Forward</option><option value="2">Reverse</option>
            </select>
          </label>
          <button class="secondary" type="button"
            onclick={() => actions.setDrawOption("directionMode", snapshot.directionMode === 1 ? 2 : 1)}>
            Direction反転
          </button>
          <label>Plane Z
            <input type="number" step="0.1" value={snapshot.drawPlaneZ}
              onchange={(event) => actions.setDrawOption("drawPlaneZ", Number(event.currentTarget.value))} />
          </label>
          <p class="hint">LMB: 点追加 / RMB: 1点戻す / Enter: 生成 / Esc: 取消</p>
          <strong class="point-count">{snapshot.pathPoints.length} points</strong>
          <div class="path-actions">
            <button class="secondary" type="button" onclick={() => actions.undoPathPoint()}>
              1点戻す
            </button>
            <button class="secondary" type="button" onclick={() => actions.clearPath()}>
              クリア
            </button>
          </div>
        </div>
        <SelectionInspector {actions} {snapshot} />
        <Settings {actions} {snapshot} />
        <Templates {actions} {snapshot} />
      </div>
    {/if}
  </section>

  <footer class="bottom-region">
    <aside>
      <span>parts <strong>{snapshot.parts.length}</strong></span>
      <span>poles <strong>{snapshot.poles.length}</strong></span>
      <span>
        generation
        <strong>{snapshot.generationMs === null ? "—" : `${snapshot.generationMs.toFixed(2)} ms`}</strong>
      </span>
      <span>
        scene update
        <strong>{snapshot.sceneUpdateMs === null ? "—" : `${snapshot.sceneUpdateMs.toFixed(2)} ms`}</strong>
      </span>
      <span>
        interaction frames
        <strong>
          {snapshot.lastInteractionFrames === null
            ? "—"
            : `${snapshot.lastInteractionFrames.longFrameCount}/${snapshot.lastInteractionFrames.sampleCount} long · max ${snapshot.lastInteractionFrames.maxFrameMs.toFixed(1)} ms`}
        </strong>
      </span>
    </aside>

    {#if snapshot.error}
      <div class="error" role="alert">{snapshot.error}</div>
    {/if}

    {#if snapshot.logs.length > 0}
      <div class="last-log">{snapshot.logs.at(-1)}</div>
    {/if}
  </footer>

  {#if snapshot.interaction}
    <div class="interaction">
      editing {snapshot.interaction.param} · Escで取消
    </div>
  {/if}
</main>
