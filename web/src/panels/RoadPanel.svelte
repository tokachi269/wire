<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import { previewRoadPrimitive } from "../road";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();
  let preview = $derived(previewRoadPrimitive(snapshot.road));

  const pointRows = [
    ["Start", "draftStart"],
    ["End", "draftEnd"],
    ["Handle A", "handleA"],
    ["Handle B", "handleB"]
  ] as const;

  function updatePoint(
    key: "draftStart" | "draftEnd" | "handleA" | "handleB",
    axis: "x" | "y",
    value: string
  ): void {
    actions.updateRoadDraftPoint(key, axis, Number(value));
  }
</script>

<section class="road-panel">
  <div class="draw-panel-head">
    <p class="panel-label">ROAD</p>
    <strong class="point-count">P0-P2</strong>
  </div>

  <div class="segmented">
    <button
      class:active={snapshot.road.mode === "line"}
      type="button"
      onclick={() => actions.setRoadMode("line")}
    >
      Line
    </button>
    <button
      class:active={snapshot.road.mode === "bezier"}
      type="button"
      onclick={() => actions.setRoadMode("bezier")}
    >
      Bezier
    </button>
  </div>

  <div class="road-grid">
    {#each pointRows as [label, key]}
      <label>
        {label} X
        <input
          type="number"
          step="0.5"
          value={snapshot.road[key].x}
          oninput={(event) => updatePoint(key, "x", event.currentTarget.value)}
        />
      </label>
      <label>
        {label} Y
        <input
          type="number"
          step="0.5"
          value={snapshot.road[key].y}
          oninput={(event) => updatePoint(key, "y", event.currentTarget.value)}
        />
      </label>
    {/each}
  </div>

  <label class="check">
    <input
      type="checkbox"
      checked={snapshot.road.connectToFirstNode}
      onchange={(event) => actions.setRoadConnectToFirstNode(event.currentTarget.checked)}
    />
    Connect to first node
  </label>

  <div class="path-preview">
    <span>{preview.kind}</span>
    <strong>
      {preview.p0.x.toFixed(1)},{preview.p0.y.toFixed(1)} →
      {(preview.p3 ?? preview.p1).x.toFixed(1)},{(preview.p3 ?? preview.p1).y.toFixed(1)}
    </strong>
  </div>

  <div class="road-actions">
    <button type="button" onclick={() => actions.addRoadSegment()}>Add segment</button>
    <button class="secondary" type="button" onclick={() => actions.addRoadTransition()}>Add transition</button>
    <button class="secondary" type="button" onclick={() => actions.addRoadManualLine()}>Manual line</button>
    <button class="secondary" type="button" onclick={() => actions.addRoadManualArea()}>Zebra area</button>
  </div>

  <div class="road-summary">
    <span>segments <strong>{snapshot.road.segments.length}</strong></span>
    <span>sections <strong>{snapshot.road.sectionTemplates.length}</strong></span>
    <span>transitions <strong>{snapshot.road.transitions.length}</strong></span>
    <span>markings <strong>{snapshot.road.manualMarkings.length}</strong></span>
    <span>gates <strong>{snapshot.road.derived.connectionGates}</strong></span>
    <span>junctions <strong>{snapshot.road.derived.junctionAreas}</strong></span>
  </div>

  {#if snapshot.road.lastError}
    <p class="road-error">{snapshot.road.lastError}</p>
  {/if}
</section>
