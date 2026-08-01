<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import { drawIssueText } from "../labels";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();

  const selectedTemplate = $derived(
    snapshot.road.scene.sectionTemplates.find((template) => template.id === snapshot.road.selectedSectionTemplateId)
  );
  const selectedStrips = $derived(selectedTemplate?.strips ?? []);
  const laneTargetTemplate = $derived(
    snapshot.road.scene.sectionTemplates.find((template) => template.id === snapshot.road.laneTargetTemplateId)
  );
  const selectedLanePath = $derived(
    snapshot.road.scene.lanePaths.find((lane) =>
      lane.segmentId === snapshot.road.selectedLaneSegmentId && lane.laneId === snapshot.road.selectedLaneId)
  );
  const selectedLaneTemplateId = $derived(snapshot.road.selectedLaneEndpointRole === 0
    ? selectedLanePath?.startSectionTemplateId
    : selectedLanePath?.endSectionTemplateId);
  const selectedLaneTemplate = $derived(
    snapshot.road.scene.sectionTemplates.find((template) => template.id === selectedLaneTemplateId)
  );

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
      style:grid-template-columns={selectedStrips.map((strip) => `${strip.widthM}fr`).join(" ")}>
      {#each selectedStrips as strip}
        <span class={strip.function}>{strip.widthM.toFixed(1)}m</span>
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

  {#if snapshot.road.operation === "add-lane"}
    <div class="draw-panel-head"><p class="panel-label">ADD LANE</p><strong class="point-count">{snapshot.road.laneEditStage}</strong></div>
    <div class="road-grid">
      <label><span>Direction</span><select value={snapshot.road.selectedLaneDirection}
        onchange={(event) => actions.setRoadSetting("selectedLaneDirection", Number(event.currentTarget.value) as 0 | 1)}>
        <option value={0}>Along</option><option value={1}>Against</option>
      </select></label>
      <label><span>Side</span><select value={snapshot.road.laneSide}
        onchange={(event) => actions.setRoadSetting("laneSide", event.currentTarget.value as "left" | "right")}>
        <option value="left">Left</option><option value="right">Right</option>
      </select></label>
      <label><span>Width</span><input type="number" min="2.5" step="0.1" value={snapshot.road.laneWidthM}
        onchange={(event) => actions.setRoadSetting("laneWidthM", Number(event.currentTarget.value))} /></label>
    </div>
  {:else if snapshot.road.operation === "branch-lane" || snapshot.road.operation === "merge-lane"}
    <div class="draw-panel-head"><p class="panel-label">{snapshot.road.operation === "branch-lane" ? "BRANCH LANE" : "MERGE LANE"}</p><strong class="point-count">{snapshot.road.laneEditStage}</strong></div>
    <label><span>New road section</span><select value={snapshot.road.laneTargetTemplateId}
      onchange={(event) => actions.setRoadLaneTargetTemplate(Number(event.currentTarget.value))}>
      {#each snapshot.road.scene.sectionTemplates as template}<option value={template.id}>{template.name}</option>{/each}
    </select></label>
    <div class="road-grid">
      <label><span>New lane</span><select value={snapshot.road.laneTargetLaneId}
        onchange={(event) => actions.setRoadSetting("laneTargetLaneId", Number(event.currentTarget.value))}>
        {#each laneTargetTemplate?.lanes ?? [] as lane}<option value={lane.id}>{lane.id} / {lane.direction === 0 ? "along" : "against"}</option>{/each}
      </select></label>
      <label><span>New boundary</span><select value={snapshot.road.laneTargetBoundaryId}
        onchange={(event) => actions.setRoadSetting("laneTargetBoundaryId", Number(event.currentTarget.value))}>
        {#each laneTargetTemplate?.boundaries ?? [] as boundary}<option value={boundary.id}>{boundary.id}</option>{/each}
      </select></label>
      <label><span>Existing boundary</span><select value={snapshot.road.laneSourceBoundaryId}
        onchange={(event) => actions.setRoadSetting("laneSourceBoundaryId", Number(event.currentTarget.value))}>
        {#each selectedLaneTemplate?.boundaries ?? [] as boundary}<option value={boundary.id}>{boundary.id}</option>{/each}
      </select></label>
    </div>
  {/if}

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
  {:else if snapshot.road.previewIssue}
    <p class="road-preview-issue">{drawIssueText(snapshot.road.previewIssue)}</p>
  {/if}
</section>
