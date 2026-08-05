<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import { drawIssueText } from "../labels";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();

  // Sections this workspace registered carry the preset label. Anything else —
  // a section a load brought in, or one Add Lane derived — is named by its ID.
  const sectionLabel = (id: number): string =>
    snapshot.road.sectionTemplateLabels[id] ?? `Section ${id}`;

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
  const selectedLaneCorridor = $derived(
    snapshot.road.scene.corridors.find((corridor) =>
      corridor.segments.some((segment) => segment.segmentId === snapshot.road.selectedLaneSegmentId)
    )
  );
  const addLaneDirections = $derived.by(() => {
    if (snapshot.road.laneEditStage === "select") return [0, 1] as Array<0 | 1>;
    const fromPaths = snapshot.road.scene.lanePaths
      .filter((lane) => lane.segmentId === snapshot.road.selectedLaneSegmentId)
      .map((lane) => lane.direction);
    const fromTemplate = snapshot.road.scene.sectionTemplates
      .find((template) => template.id === selectedLaneCorridor?.sectionTemplateId)
      ?.lanes.map((lane) => lane.direction) ?? [];
    const available = Array.from(new Set(fromPaths.length > 0 ? fromPaths : fromTemplate));
    return available.length > 0 ? available : [snapshot.road.selectedLaneDirection];
  });
  const addLaneStep = $derived.by(() => {
    if (snapshot.road.laneEditStage === "select") return "2車線から3車線への変化開始位置";
    if (snapshot.road.laneEditStage === "transition-complete") return "3車線が完成する位置";
    if (snapshot.road.laneEditStage === "continuation-end") return "3車線を維持する終点";
    return "";
  });

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
        <option value={template.id}>{sectionLabel(template.id)}</option>
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
    <div class="draw-panel-head"><p class="panel-label">ADD LANE</p><strong class="point-count">{addLaneStep}</strong></div>
    <div class="road-grid">
      <label><span>Direction</span><select value={snapshot.road.selectedLaneDirection}
        onchange={(event) => actions.setRoadSetting("selectedLaneDirection", Number(event.currentTarget.value) as 0 | 1)}>
        {#each addLaneDirections as direction}
          <option value={direction}>{direction === 0 ? "Along" : "Against"}</option>
        {/each}
      </select></label>
      <label><span>Side</span><select value={snapshot.road.laneSide}
        onchange={(event) => actions.setRoadSetting("laneSide", event.currentTarget.value as "left" | "right")}>
        <option value="left">Left</option><option value="right">Right</option>
      </select></label>
      <label><span>Width</span><input type="number" min="2.5" step="0.1" value={snapshot.road.laneWidthM}
        onchange={(event) => actions.setRoadSetting("laneWidthM", Number(event.currentTarget.value))} /></label>
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

  {#if snapshot.road.previewIssue}
    <p class="road-preview-issue">{drawIssueText(snapshot.road.previewIssue)}</p>
  {/if}
</section>
