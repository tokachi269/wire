<script lang="ts">
  import { onMount } from "svelte";
  import type { ViewerActions } from "./actions/viewer";
  import type { ViewerSnapshot, ViewerStore } from "./store/viewer";

  interface Props {
    actions: ViewerActions;
    store: ViewerStore;
    mountScene: (host: HTMLElement) => void;
  }

  let { actions, store, mountScene }: Props = $props();
  let sceneHost: HTMLElement;
  let snapshot: ViewerSnapshot = $state({
    parts: [],
    poles: [],
    error: "",
    generationMs: null,
    pathPoints: [],
    bundleTemplates: [],
    selectedBundleTemplateId: null
  });

  onMount(() => {
    const unsubscribe = store.value.subscribe((value) => {
      snapshot = value;
    });
    mountScene(sceneHost);
    return unsubscribe;
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
      <button class="secondary" type="button" onclick={() => actions.generateSample()}>
        サンプル
      </button>
      <button type="button" onclick={() => actions.generatePath()}>
        Path生成
      </button>
    </div>
  </header>

  <section class="workspace">
    <div class="viewport" bind:this={sceneHost}></div>
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
      <p class="hint">地面をクリックして点を追加</p>
      <strong class="point-count">{snapshot.pathPoints.length} points</strong>
      <div class="path-actions">
        <button class="secondary" type="button" onclick={() => actions.clearPath()}>
          クリア
        </button>
      </div>
    </div>
  </section>

  <aside>
    <span>parts <strong>{snapshot.parts.length}</strong></span>
    <span>poles <strong>{snapshot.poles.length}</strong></span>
    <span>
      generation
      <strong>{snapshot.generationMs === null ? "—" : `${snapshot.generationMs.toFixed(2)} ms`}</strong>
    </span>
  </aside>

  {#if snapshot.error}
    <div class="error" role="alert">{snapshot.error}</div>
  {/if}
</main>
