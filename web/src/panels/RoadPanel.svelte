<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();

  const selectedTemplate = $derived(
    snapshot.road.scene.sectionTemplates.find((template) => template.id === snapshot.road.selectedSectionTemplateId)
  );
  const selectedBands = $derived(selectedTemplate?.bands ?? []);

  function updateTemplate(patch: Partial<{
    sidewalkWidthM: number;
    laneWidthM: number;
    medianWidthM: number;
    hasCenterLine: boolean;
    hasOuterLines: boolean;
  }>): void {
    if (selectedTemplate === undefined) return;
    actions.updateSelectedRoadSectionTemplate({
      sidewalkWidthM: selectedTemplate.sidewalkWidthM,
      laneWidthM: selectedTemplate.laneWidthM,
      medianWidthM: selectedTemplate.medianWidthM,
      hasCenterLine: selectedTemplate.hasCenterLine,
      hasOuterLines: selectedTemplate.hasOuterLines,
      ...patch
    });
  }
</script>

<section class="road-panel">
  <div class="draw-panel-head">
    <p class="panel-label">ROAD</p>
    <strong class="point-count">P0-P2</strong>
  </div>

  <label>
    <span>New road section</span>
    <select value={snapshot.road.selectedSectionTemplateId}
      onchange={(event) => actions.setRoadSetting("selectedSectionTemplateId", Number(event.currentTarget.value))}>
      {#each snapshot.road.scene.sectionTemplates as template}
        <option value={template.id}>{template.name}</option>
      {/each}
    </select>
  </label>

  {#if selectedTemplate}
    <div class="road-profile" aria-label="Road cross section"
      style:grid-template-columns={selectedBands.map((band) => `${band.widthM}fr`).join(" ")}>
      {#each selectedBands as band}
        <span class={band.role}>{band.widthM.toFixed(1)}m</span>
      {/each}
    </div>
    <label><span>Sidewalk width</span><input type="number" min="0.5" step="0.1"
      value={selectedTemplate.sidewalkWidthM}
      onchange={(event) => updateTemplate({ sidewalkWidthM: Number(event.currentTarget.value) })} /></label>
    <label><span>Lane width</span><input type="number" min="2.5" step="0.1"
      value={selectedTemplate.laneWidthM}
      onchange={(event) => updateTemplate({ laneWidthM: Number(event.currentTarget.value) })} /></label>
    {#if selectedTemplate.medianWidthM > 0}
      <label><span>Median width</span><input type="number" min="0.5" step="0.1"
        value={selectedTemplate.medianWidthM}
        onchange={(event) => updateTemplate({ medianWidthM: Number(event.currentTarget.value) })} /></label>
    {/if}
    <label class="check"><input type="checkbox" checked={selectedTemplate.hasCenterLine}
      onchange={(event) => updateTemplate({ hasCenterLine: event.currentTarget.checked })} />Center / lane lines</label>
    <label class="check"><input type="checkbox" checked={selectedTemplate.hasOuterLines}
      onchange={(event) => updateTemplate({ hasOuterLines: event.currentTarget.checked })} />Outer lines</label>
  {/if}

  <div class="draw-panel-head"><p class="panel-label">TRANSITION</p></div>
  <label><span>Target section</span><select value={snapshot.road.transitionTargetTemplateId}
    onchange={(event) => actions.setRoadSetting("transitionTargetTemplateId", Number(event.currentTarget.value))}>
    {#each snapshot.road.scene.sectionTemplates as template}<option value={template.id}>{template.name}</option>{/each}
  </select></label>
  <label><span>Length</span><input type="number" min="4" step="1" value={snapshot.road.transitionLengthM}
    onchange={(event) => actions.setRoadSetting("transitionLengthM", Number(event.currentTarget.value))} /></label>
  <label><span>End offset</span><input type="number" min="0" step="0.5" value={snapshot.road.transitionEndOffsetM}
    onchange={(event) => actions.setRoadSetting("transitionEndOffsetM", Number(event.currentTarget.value))} /></label>
  <label><span>Anchor</span><select value={snapshot.road.transitionAnchor}
    onchange={(event) => actions.setRoadSetting("transitionAnchor", Number(event.currentTarget.value) as 0 | 1 | 2)}>
    <option value={0}>Center</option><option value={1}>Left edge</option><option value={2}>Right edge</option>
  </select></label>

  <div class="draw-panel-head"><p class="panel-label">MARKING</p></div>
  <label><span>Lateral offset</span><input type="number" step="0.1" value={snapshot.road.manualLineOffsetM}
    onchange={(event) => actions.setRoadSetting("manualLineOffsetM", Number(event.currentTarget.value))} /></label>
  <label><span>Area width</span><input type="number" min="0.2" step="0.2" value={snapshot.road.manualAreaWidthM}
    onchange={(event) => actions.setRoadSetting("manualAreaWidthM", Number(event.currentTarget.value))} /></label>
  <label><span>Area length</span><input type="number" min="0.2" step="0.2" value={snapshot.road.manualAreaLengthM}
    onchange={(event) => actions.setRoadSetting("manualAreaLengthM", Number(event.currentTarget.value))} /></label>

  <div class="road-summary">
    <span>segments <strong>{snapshot.road.scene.segmentCount}</strong></span>
    <span>sections <strong>{snapshot.road.scene.sectionTemplateCount}</strong></span>
    <span>transitions <strong>{snapshot.road.scene.transitionCount}</strong></span>
    <span>markings <strong>{snapshot.road.scene.markingCount}</strong></span>
    <span>gates <strong>{snapshot.road.scene.connectionGateCount}</strong></span>
    <span>junctions <strong>{snapshot.road.scene.junctionCount}</strong></span>
  </div>

  {#if snapshot.road.lastError}
    <p class="road-error">{snapshot.road.lastError}</p>
  {/if}
</section>
