<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();
  const selectionTitle = () => {
    const selection = snapshot.selection;
    if (selection === null) return "";
    const label = selection.kind === "supportNode"
      ? "Support node"
      : selection.kind === "roadSegment"
        ? "Road segment"
        : selection.kind[0].toUpperCase() + selection.kind.slice(1);
    return `${label} ${selection.id}`;
  };
</script>

<section class="selection-inspector sidebar-card">
  <h2>Inspector</h2>
  {#if snapshot.selection}
    <strong>{selectionTitle()}</strong>
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
    {:else if snapshot.selection.kind === "road"}
      {@const road = snapshot.road.scene.corridors.find((item) => String(item.id) === snapshot.selection?.id)}
      {#if road}
        <span>length {road.lengthM.toFixed(2)} m</span>
        <span>segments {road.segments.length}</span>
        <span>section {snapshot.road.roadLayoutTemplateLabels[road.roadLayoutTemplateId] ?? road.roadLayoutTemplateId}</span>
      {/if}
    {:else if snapshot.selection.kind === "roadSegment"}
      {@const segment = snapshot.road.scene.editableSegments.find((item) => String(item.id) === snapshot.selection?.id)}
      {#if segment}
        <span>alignment {segment.kind}</span>
        <span>nodes {segment.nodeAId} -&gt; {segment.nodeBId}</span>
      {/if}
    {:else}
      <span>read-only selection</span>
    {/if}
  {:else}
    <span>No selection: edit scene settings</span>
  {/if}
</section>
