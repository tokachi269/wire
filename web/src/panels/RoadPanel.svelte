<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();
</script>

<section class="road-panel">
  <div class="draw-panel-head">
    <p class="panel-label">ROAD</p>
    <strong class="point-count">JP 2 lane</strong>
  </div>

  <div class="road-profile" aria-label="Road cross section">
    <span class="sidewalk">2.0m</span>
    <span class="lane">3.0m</span>
    <span class="lane">3.0m</span>
    <span class="sidewalk">2.0m</span>
  </div>

  <label class="check">
    <input
      type="checkbox"
      checked={snapshot.road.connectToFirstNode}
      onchange={(event) => actions.setRoadConnectToFirstNode(event.currentTarget.checked)}
    />
    Connect to first node
  </label>

  <div class="road-summary">
    <span>segments <strong>{snapshot.road.scene.segmentCount}</strong></span>
    <span>sections <strong>{snapshot.road.scene.sectionTemplateCount}</strong></span>
    <span>gates <strong>{snapshot.road.scene.connectionGateCount}</strong></span>
    <span>junctions <strong>{snapshot.road.scene.junctionCount}</strong></span>
  </div>

  {#if snapshot.road.lastError}
    <p class="road-error">{snapshot.road.lastError}</p>
  {/if}
</section>
