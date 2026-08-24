<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { SelectionKind, ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();
  let filter = $state("");

  const selected = (kind: SelectionKind, id: string) =>
    snapshot.selection?.kind === kind && snapshot.selection.id === id;
  const matches = (label: string) =>
    label.toLowerCase().includes(filter.trim().toLowerCase());
  const portsForPole = (poleId: string) =>
    snapshot.ports.filter((port) => port.ownerPoleId === poleId);
  const unownedPorts = () =>
    snapshot.ports.filter(
      (port) => !snapshot.poles.some((pole) => pole.id === port.ownerPoleId)
    );
  const spansForPole = (poleId: string) => {
    const portIds = new Set(portsForPole(poleId).map((port) => port.id));
    return snapshot.spans.filter(
      (span) => portIds.has(span.portAId) || portIds.has(span.portBId)
    );
  };
  const poleMatches = (poleId: string) =>
    matches(`Pole ${poleId}`) ||
    portsForPole(poleId).some((port) => matches(`Port ${port.id}`)) ||
    spansForPole(poleId).some((span) => matches(`Span ${span.id}`));
  const roadMatches = (roadId: number) => {
    const road = snapshot.road.scene.corridors.find((corridor) => corridor.id === roadId);
    return matches(`Road ${roadId}`) ||
      road?.segments.some((segment) => matches(`Segment ${segment.segmentId}`)) === true;
  };
</script>

<section class="outliner sidebar-card">
  <div class="outliner-title">
    <h2>Outliner</h2>
    <button class="secondary" type="button" onclick={() => actions.clearSelection()}>
      Clear selection
    </button>
  </div>
  <label>
    Filter
    <input bind:value={filter} placeholder="pole / port / span / road id" />
  </label>
  <div class="entity-lists">
    <details open>
      <summary>Poles ({snapshot.poles.length})</summary>
      {#each snapshot.selectionIncludePoles ? snapshot.poles : [] as pole}
        {#if poleMatches(pole.id)}
          {@const ownedPorts = portsForPole(pole.id)}
          {@const connectedSpans = spansForPole(pole.id)}
          <div class="entity-node">
            <button class:active={selected("pole", pole.id)}
              onclick={() => actions.select("pole", pole.id)}>Pole {pole.id}</button>
            <details>
              <summary>Ports ({ownedPorts.length})</summary>
              {#each ownedPorts as port}
                {#if matches(`Port ${port.id}`) || filter.trim() === ""}
                  <button class:active={selected("port", port.id)}
                    onclick={() => actions.select("port", port.id)}>Port {port.id}</button>
                {/if}
              {/each}
            </details>
            <details>
              <summary>Spans ({connectedSpans.length})</summary>
              {#each snapshot.selectionIncludeSpans ? connectedSpans : [] as span}
                {#if matches(`Span ${span.id}`) || filter.trim() === ""}
                  <button class:active={selected("span", span.id)}
                    onclick={() => actions.select("span", span.id)}>Span {span.id}</button>
                {/if}
              {/each}
            </details>
          </div>
        {/if}
      {/each}
    </details>
    {#if unownedPorts().length > 0}
      <details>
        <summary>Unowned ports ({unownedPorts().length})</summary>
        {#each unownedPorts() as port}
          {#if matches(`Port ${port.id}`) || filter.trim() === ""}
            <button class:active={selected("port", port.id)}
              onclick={() => actions.select("port", port.id)}>Port {port.id}</button>
          {/if}
        {/each}
      </details>
    {/if}
    <details>
      <summary>Midair ({snapshot.supportNodes.filter((node) => node.kind === 1).length})</summary>
      {#each snapshot.selectionIncludeMidair ? snapshot.supportNodes.filter((node) => node.kind === 1) : [] as node}
        {#if matches(`Node ${node.id}`)}
          <button class:active={selected("supportNode", node.id)}
            onclick={() => actions.select("supportNode", node.id)}>Node {node.id}</button>
        {/if}
      {/each}
    </details>
    <details open>
      <summary>Roads ({snapshot.road.scene.corridors.length})</summary>
      {#each snapshot.road.scene.corridors as road}
        {#if roadMatches(road.id)}
          <div class="entity-node">
            <button
              type="button"
              class:active={selected("road", String(road.id))}
              aria-label={`Select road ${road.id}`}
              onclick={() => actions.select("road", String(road.id))}
            >Road {road.id}</button>
            <details open>
              <summary>Segments ({road.segments.length})</summary>
              {#each road.segments as segment}
                {#if matches(`Segment ${segment.segmentId}`) || filter.trim() === ""}
                  <button
                    type="button"
                    class:active={selected("roadSegment", String(segment.segmentId))}
                    aria-label={`Select road segment ${segment.segmentId}`}
                    onclick={() => actions.select("roadSegment", String(segment.segmentId))}
                  >Segment {segment.segmentId}</button>
                {/if}
              {/each}
            </details>
          </div>
        {/if}
      {/each}
    </details>
  </div>
</section>
