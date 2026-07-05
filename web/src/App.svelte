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
    generationMs: null
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
    <button type="button" onclick={() => actions.generateSample()}>
      サンプル route 生成
    </button>
  </header>

  <section class="viewport" bind:this={sceneHost}></section>

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
