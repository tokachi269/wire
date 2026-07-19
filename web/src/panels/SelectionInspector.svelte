<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();
</script>

<section class="selection-inspector">
  <h2>Inspector</h2>
  {#if snapshot.selection}
    <strong>{snapshot.selection.kind} {snapshot.selection.id}</strong>
    {#if snapshot.selection.kind === "pole"}
      {@const pole = snapshot.poles.find((item) => item.id === snapshot.selection?.id)}
      {#if pole}
        <span>height {pole.height.toFixed(2)}</span>
        <span>position {pole.positionX.toFixed(2)}, {pole.positionY.toFixed(2)}, {pole.positionZ.toFixed(2)}</span>
      {/if}
      <button class="secondary" onclick={() => actions.clearSelectedOverride("pole")}>
        Clear orientation override
      </button>
    {:else if snapshot.selection.kind === "span"}
      {@const span = snapshot.spans.find((item) => item.id === snapshot.selection?.id)}
      {#if span}<span>ports {span.portAId} -&gt; {span.portBId}</span>{/if}
      <button class="secondary" onclick={() => actions.clearSelectedOverride("socketA")}>Clear socket A</button>
      <button class="secondary" onclick={() => actions.clearSelectedOverride("socketB")}>Clear socket B</button>
      <button class="secondary" onclick={() => actions.clearSelectedOverride("branchDown")}>Clear branch down</button>
    {:else}
      <span>read-only selection</span>
    {/if}
  {:else}
    <span>No selection: edit scene settings</span>
  {/if}
</section>
