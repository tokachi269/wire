<script lang="ts">
  import { onMount } from "svelte";
  import type { ViewerActions } from "./actions/viewer";
  import Settings from "./panels/Settings.svelte";
  import Templates from "./panels/Templates.svelte";
  import Outliner from "./panels/Outliner.svelte";
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

  onMount(() => {
    const unsubscribe = store.value.subscribe((value) => {
      snapshot = value;
    });
    mountScene(sceneHost);
    const cancel = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        actions.cancel();
      }
    };
    window.addEventListener("keydown", cancel);
    return () => {
      unsubscribe();
      window.removeEventListener("keydown", cancel);
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
      <p class="eyebrow">WIRE / DERIVED OUTPUT</p>
      <h1>Backbone field viewer</h1>
    </div>
    <div class="header-actions">
      <label class="top-toggle"><input type="checkbox" checked={snapshot.showWorkspace}
        onchange={(event) => actions.setDrawOption("showWorkspace", event.currentTarget.checked)} />
        panels</label>
      <input class="workspace-width" aria-label="Workspace width" type="range" min="190" max="420"
        value={snapshot.workspaceWidth}
        oninput={(event) => actions.setDrawOption("workspaceWidth", Number(event.currentTarget.value))} />
      <button class="secondary" type="button" onclick={() => actions.generateSample()}>
        サンプル
      </button>
      <button type="button" onclick={() => actions.generatePath()}>
        Path生成
      </button>
    </div>
  </header>

  <section class="workspace" style={`grid-template-columns:${snapshot.showWorkspace ? `minmax(0,1fr) ${snapshot.workspaceWidth}px` : "1fr"}`}>
    <div class="viewport" bind:this={sceneHost}></div>
    {#if snapshot.showWorkspace}
      <div class="draw-panel">
      <p class="panel-label">DRAW PATH</p>
      <label>
        Bundle
        <select
          value={snapshot.selectedBundleTemplateId ?? ""}
          onchange={(event) =>
            actions.selectBundleTemplate(Number(event.currentTarget.value))}
        >
          {#each snapshot.bundleTemplates as template}
            <option value={template.id}>{template.name}</option>
          {/each}
        </select>
      </label>
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
      <label>Bundle count
        <input type="number" min="1" value={snapshot.bundleCount}
          onchange={(event) => actions.setDrawOption("bundleCount", Number(event.currentTarget.value))} />
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
      <p class="hint">地面をクリックして点を追加</p>
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
    {/if}
  </section>

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

  {#if snapshot.showWorkspace}
    <Settings {actions} {snapshot} />
    <Outliner {actions} {snapshot} />
    <Templates {actions} {snapshot} />
  {/if}

  {#if snapshot.logs.length > 0}
    <div class="last-log">{snapshot.logs.at(-1)}</div>
  {/if}

  {#if snapshot.interaction}
    <div class="interaction">
      editing {snapshot.interaction.param} · Escで取消
    </div>
  {/if}
</main>
