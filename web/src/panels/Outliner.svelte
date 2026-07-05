<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { SelectionKind, ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();

  const selected = (kind: SelectionKind, id: string) =>
    snapshot.selection?.kind === kind && snapshot.selection.id === id;
</script>

<section class="outliner">
  <div class="outliner-title">
    <h2>Outliner</h2>
    <button class="secondary" type="button" onclick={() => actions.clearSelection()}>
      選択解除
    </button>
  </div>
  <div class="entity-lists">
    <div>
      <h3>Poles</h3>
      {#each snapshot.selectionIncludePoles ? snapshot.poles : [] as pole}
        <button class:active={selected("pole", pole.id)}
          onclick={() => actions.select("pole", pole.id)}>Pole {pole.id}</button>
      {/each}
    </div>
    <div>
      <h3>Ports</h3>
      {#each snapshot.ports as port}
        <button class:active={selected("port", port.id)}
          onclick={() => actions.select("port", port.id)}>Port {port.id}</button>
      {/each}
    </div>
    <div>
      <h3>Spans</h3>
      {#each snapshot.selectionIncludeSpans ? snapshot.spans : [] as span}
        <button class:active={selected("span", span.id)}
          onclick={() => actions.select("span", span.id)}>Span {span.id}</button>
      {/each}
    </div>
    <div>
      <h3>Midair</h3>
      {#each snapshot.selectionIncludeMidair ? snapshot.supportNodes.filter((node) => node.kind === 1) : [] as node}
        <button class:active={selected("supportNode", node.id)}
          onclick={() => actions.select("supportNode", node.id)}>Node {node.id}</button>
      {/each}
    </div>
  </div>

  {#if snapshot.selection}
    <div class="inspector">
      <strong>{snapshot.selection.kind} {snapshot.selection.id}</strong>
      {#if snapshot.selection.kind === "pole"}
        {@const pole = snapshot.poles.find((item) => item.id === snapshot.selection?.id)}
        {#if pole}
          <span>height {pole.height.toFixed(2)}</span>
          <span>position {pole.positionX.toFixed(2)}, {pole.positionY.toFixed(2)}, {pole.positionZ.toFixed(2)}</span>
        {/if}
        <button class="secondary" onclick={() => actions.clearSelectedOverride("pole")}>
          orientation override解除
        </button>
      {:else if snapshot.selection.kind === "span"}
        {@const span = snapshot.spans.find((item) => item.id === snapshot.selection?.id)}
        {#if span}<span>ports {span.portAId} → {span.portBId}</span>{/if}
        <button class="secondary" onclick={() => actions.clearSelectedOverride("socketA")}>socket A解除</button>
        <button class="secondary" onclick={() => actions.clearSelectedOverride("socketB")}>socket B解除</button>
        <button class="secondary" onclick={() => actions.clearSelectedOverride("branchDown")}>branch down解除</button>
      {/if}
    </div>
  {/if}
</section>
