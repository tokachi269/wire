<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type {
    BundleTemplateInfo,
    CableTemplateInfo,
    PoleTemplateInfo,
    PortBandInfo
  } from "../model";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();

  const numberValue = (event: Event) =>
    Number((event.currentTarget as HTMLInputElement | HTMLSelectElement).value);
  const textValue = (event: Event) =>
    (event.currentTarget as HTMLInputElement).value;
  const checkedValue = (event: Event) =>
    (event.currentTarget as HTMLInputElement).checked;

  const selectedCable = () =>
    snapshot.cableTemplates.find(
      (template) => template.id === snapshot.selectedCableTemplateId
    );
  const selectedBundle = () =>
    snapshot.bundleTemplates.find(
      (template) => template.id === snapshot.selectedBundleTemplateId
    );
  const selectedPole = () =>
    snapshot.poleTemplates.find(
      (template) => template.id === snapshot.selectedPoleTemplateId
    );

  function patchCable<K extends keyof CableTemplateInfo>(
    template: CableTemplateInfo,
    key: K,
    value: CableTemplateInfo[K]
  ): CableTemplateInfo {
    return { ...template, [key]: value };
  }

  function patchBundle<K extends keyof BundleTemplateInfo>(
    template: BundleTemplateInfo,
    key: K,
    value: BundleTemplateInfo[K]
  ): BundleTemplateInfo {
    return { ...template, [key]: value };
  }

  function updatePole(
    template: PoleTemplateInfo,
    change: (draft: PoleTemplateInfo) => void
  ): void {
    const draft = structuredClone(template);
    change(draft);
    actions.commitPoleTemplate(draft);
  }

  function categoryBands(template: PoleTemplateInfo, category: number) {
    return template.portBands.filter((band) => band.category === category);
  }

  function categoryAverage(
    template: PoleTemplateInfo,
    category: number,
    field: "heightCenter" | "lateralCenter"
  ): number {
    const bands = categoryBands(template, category);
    return bands.length === 0
      ? 0
      : bands.reduce((sum, band) => sum + band[field], 0) / bands.length;
  }

  function categorySpread(template: PoleTemplateInfo, category: number): number {
    const bands = categoryBands(template, category);
    if (bands.length === 0) return 0;
    const center = categoryAverage(template, category, "lateralCenter");
    return Math.max(...bands.map((band) => Math.abs(band.lateralCenter - center)));
  }

  function setCategoryHeight(
    template: PoleTemplateInfo,
    category: number,
    height: number,
    preview = false
  ): void {
    const start = categoryAverage(template, category, "heightCenter");
    const draft = structuredClone(template);
    for (const band of draft.portBands.filter((item) => item.category === category)) {
      const half = Math.max(0, (band.heightMax - band.heightMin) / 2);
      band.heightCenter = height;
      band.heightMin = height - half;
      band.heightMax = height + half;
    }
    if (preview) {
      actions.previewPoleTemplate(draft, `pole.category.${category}.height`, "height", start, height);
    } else {
      actions.commitPoleTemplate(draft);
    }
  }

  function setCategoryOffset(
    template: PoleTemplateInfo,
    category: number,
    offset: number,
    preview = false
  ): void {
    const oldCenter = categoryAverage(template, category, "lateralCenter");
    const draft = structuredClone(template);
    for (const band of draft.portBands.filter((item) => item.category === category)) {
      const delta = offset - oldCenter;
      band.lateralCenter += delta;
      band.lateralMin += delta;
      band.lateralMax += delta;
    }
    if (preview) {
      actions.previewPoleTemplate(draft, `pole.category.${category}.offset`, "offset", oldCenter, offset);
    } else {
      actions.commitPoleTemplate(draft);
    }
  }

  function setCategorySpread(
    template: PoleTemplateInfo,
    category: number,
    spread: number,
    preview = false
  ): void {
    const bands = categoryBands(template, category);
    const center = categoryAverage(template, category, "lateralCenter");
    const ordered = [...bands].sort((a, b) => a.lateralCenter - b.lateralCenter);
    const draft = structuredClone(template);
    ordered.forEach((original, index) => {
      const target = draft.portBands.find((band) => band.bandId === original.bandId);
      if (target === undefined) return;
      const t = ordered.length === 1 ? 0 : index / (ordered.length - 1) * 2 - 1;
      const half = Math.max(0, (target.lateralMax - target.lateralMin) / 2);
      target.lateralCenter = center + t * Math.max(0, spread);
      target.lateralMin = target.lateralCenter - half;
      target.lateralMax = target.lateralCenter + half;
    });
    if (preview) {
      actions.previewPoleTemplate(
        draft, `pole.category.${category}.spread`, "spread",
        categorySpread(template, category), spread
      );
    } else {
      actions.commitPoleTemplate(draft);
    }
  }

  function updateBand(
    template: PoleTemplateInfo,
    bandId: number,
    change: (band: PortBandInfo) => void
  ): void {
    updatePole(template, (draft) => {
      const band = draft.portBands.find((candidate) => candidate.bandId === bandId);
      if (band !== undefined) change(band);
    });
  }
</script>

<div class="template-grid">
  <section>
    <h2>Cable template</h2>
    <select
      value={snapshot.selectedCableTemplateId ?? ""}
      onchange={(event) => actions.selectCableTemplate(numberValue(event))}
    >
      {#each snapshot.cableTemplates as template}
        <option value={template.id}>{template.name}</option>
      {/each}
    </select>
    {#if selectedCable()}
      {@const cable = selectedCable()!}
      <label>Outer diameter
        <input type="number" step="0.001" value={cable.outerDiameter}
          oninput={(event) => actions.previewCableTemplate("outerDiameter", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "outerDiameter", numberValue(event)))} />
      </label>
      <label>Bend stiffness
        <input type="number" step="0.1" value={cable.bendStiffness}
          oninput={(event) => actions.previewCableTemplate("bendStiffness", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "bendStiffness", numberValue(event)))} />
      </label>
      <label>Min bend radius
        <input type="number" step="0.01" value={cable.minBendRadius}
          oninput={(event) => actions.previewCableTemplate("minBendRadius", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "minBendRadius", numberValue(event)))} />
      </label>
      <label>Material
        <select value={cable.materialStyle}
          onchange={(event) => actions.commitCableTemplate(patchCable(cable, "materialStyle", numberValue(event)))}>
          <option value="0">Generic</option><option value="1">Bare</option>
          <option value="2">Insulated</option><option value="3">Optical</option>
        </select>
      </label>
      <label class="check"><input type="checkbox" checked={cable.requiresInsulator}
        onchange={(event) => actions.commitCableTemplate(patchCable(cable, "requiresInsulator", checkedValue(event)))} />
        Requires insulator</label>
      <label>Insulator attach height
        <input type="number" step="0.005" value={cable.insulatorAttachmentHeight}
          onchange={(event) => actions.commitCableTemplate(patchCable(cable, "insulatorAttachmentHeight", numberValue(event)))} />
      </label>
      <label>Sag factor
        <input type="number" step="0.005" value={cable.sagFactor}
          oninput={(event) => actions.previewCableTemplate("sagFactor", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "sagFactor", numberValue(event)))} />
      </label>
      <label>Slack factor
        <input type="number" step="0.005" value={cable.slackFactor}
          oninput={(event) => actions.previewCableTemplate("slackFactor", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "slackFactor", numberValue(event)))} />
      </label>
      <label>Grouped fanout
        <input type="number" step="0.01" value={cable.groupedFanoutSpacing}
          onchange={(event) => actions.commitCableTemplate(patchCable(cable, "groupedFanoutSpacing", numberValue(event)))} />
      </label>
      <label>Continuity
        <select value={cable.continuityPolicy}
          onchange={(event) => actions.commitCableTemplate(patchCable(cable, "continuityPolicy", numberValue(event)))}>
          <option value="0">Auto</option><option value="1">G1</option><option value="2">G2</option>
        </select>
      </label>
      <label class="check"><input type="checkbox" checked={cable.supplementalEnabled}
        onchange={(event) => actions.commitCableTemplate(patchCable(cable, "supplementalEnabled", checkedValue(event)))} />
        Straight supplemental</label>
      {#if cable.supplementalEnabled}
        {#each [
          ["supplementalLateralOffset", "Lateral offset"],
          ["supplementalVerticalOffset", "Vertical offset"],
          ["supplementalWobbleAmplitude", "Wobble amplitude"],
          ["supplementalWobbleWavelength", "Wobble wavelength"],
          ["supplementalWobblePhase", "Wobble phase"],
          ["supplementalEndpointEnvelope", "Endpoint envelope"]
        ] as field}
          <label>{field[1]}
            <input type="number" step="0.01" value={cable[field[0] as keyof CableTemplateInfo] as number}
              onchange={(event) => actions.commitCableTemplate(
                patchCable(cable, field[0] as keyof CableTemplateInfo, numberValue(event) as never)
              )} />
          </label>
        {/each}
      {/if}
    {/if}
  </section>

  <section>
    <h2>Bundle template</h2>
    <select value={snapshot.selectedBundleTemplateId ?? ""}
      onchange={(event) => actions.selectBundleTemplate(numberValue(event))}>
      {#each snapshot.bundleTemplates as template}
        <option value={template.id}>{template.name}</option>
      {/each}
    </select>
    {#if selectedBundle()}
      {@const bundle = selectedBundle()!}
      <label>Cable template
        <select value={bundle.cableTemplateId}
          onchange={(event) => actions.commitBundleTemplate(patchBundle(bundle, "cableTemplateId", numberValue(event)))}>
          {#each snapshot.cableTemplates as template}<option value={template.id}>{template.name}</option>{/each}
        </select>
      </label>
      <label>Related pole
        <select value={bundle.relatedPoleTypeId}
          onchange={(event) => actions.commitBundleTemplate(patchBundle(bundle, "relatedPoleTypeId", numberValue(event)))}>
          {#each snapshot.poleTemplates as template}<option value={template.id}>{template.name}</option>{/each}
        </select>
      </label>
      <label>Default layer
        <select value={bundle.defaultLayer}
          onchange={(event) => actions.commitBundleTemplate(patchBundle(bundle, "defaultLayer", numberValue(event)))}>
          {#each [1, 2, 3, 4, 5] as layer}<option value={layer}>{layer}</option>{/each}
        </select>
      </label>
      {#each [
        ["allowMirror", "Allow mirror"],
        ["allowMidairNode", "Allow midair node"],
        ["allowMidairBranch", "Allow midair branch"]
      ] as field}
        <label class="check"><input type="checkbox" checked={bundle[field[0] as keyof BundleTemplateInfo] as boolean}
          onchange={(event) => actions.commitBundleTemplate(
            patchBundle(bundle, field[0] as keyof BundleTemplateInfo, checkedValue(event) as never)
          )} />{field[1]}</label>
      {/each}
      <label>Grouped fanout
        <input type="number" step="0.01" value={bundle.groupedSupportFanoutSpacing}
          onchange={(event) => actions.commitBundleTemplate(patchBundle(bundle, "groupedSupportFanoutSpacing", numberValue(event)))} />
      </label>
      {#each [
        ["supportStyle", "Support style", [0, 1, 2]],
        ["branchPolicy", "Branch policy", [0, 1, 2]],
        ["continuityPolicy", "Continuity", [0, 1, 2]]
      ] as field}
        <label>{field[1]}
          <select value={bundle[field[0] as keyof BundleTemplateInfo] as number}
            onchange={(event) => actions.commitBundleTemplate(
              patchBundle(bundle, field[0] as keyof BundleTemplateInfo, numberValue(event) as never)
            )}>
            {#each field[2] as option}<option value={option}>{option}</option>{/each}
          </select>
        </label>
      {/each}
      <button class="secondary" type="button" onclick={() => actions.applyRelatedPoleType(bundle.id)}>
        related poleを既存へ適用
      </button>
    {/if}
  </section>

  <section class="pole-template">
    <h2>Pole template</h2>
    <select value={snapshot.selectedPoleTemplateId ?? ""}
      onchange={(event) => actions.selectPoleTemplate(numberValue(event))}>
      {#each snapshot.poleTemplates as template}<option value={template.id}>{template.name}</option>{/each}
    </select>
    {#if selectedPole()}
      {@const pole = selectedPole()!}
      <label>Name
        <input value={pole.name}
          onchange={(event) => updatePole(pole, (draft) => draft.name = textValue(event))} />
      </label>
      <label>Description
        <input value={pole.description}
          onchange={(event) => updatePole(pole, (draft) => draft.description = textValue(event))} />
      </label>
      <label>Default height
        <input type="number" step="0.05" value={pole.defaultHeight}
          oninput={(event) => actions.previewPoleDefaultHeight(numberValue(event))}
          onblur={(event) => updatePole(pole, (draft) => draft.defaultHeight = numberValue(event))} />
      </label>

      <h3>Placement by category</h3>
      {#each [0, 1, 2, 3, 4] as category}
        {#if categoryBands(pole, category).length > 0}
          <div class="category-row">
            <strong>C{category}</strong>
            <input aria-label={`Category ${category} height`} type="number" step="0.05"
              value={categoryAverage(pole, category, "heightCenter")}
              oninput={(event) => setCategoryHeight(pole, category, numberValue(event), true)}
              onblur={(event) => setCategoryHeight(pole, category, numberValue(event))} />
            <input aria-label={`Category ${category} offset`} type="number" step="0.02"
              value={categoryAverage(pole, category, "lateralCenter")}
              oninput={(event) => setCategoryOffset(pole, category, numberValue(event), true)}
              onblur={(event) => setCategoryOffset(pole, category, numberValue(event))} />
            <input aria-label={`Category ${category} spread`} type="number" step="0.02"
              value={categorySpread(pole, category)}
              oninput={(event) => setCategorySpread(pole, category, numberValue(event), true)}
              onblur={(event) => setCategorySpread(pole, category, numberValue(event))} />
          </div>
        {/if}
      {/each}

      <details>
        <summary>Port bands</summary>
        {#each pole.portBands as band}
          <div class="record">
            <strong>Band {band.bandId}</strong>
            {#each [
              ["bandId", "Id"], ["category", "Category"], ["layer", "Layer"],
              ["side", "Side"], ["role", "Role"], ["lateralCenter", "Lateral center"],
              ["lateralMin", "Lateral min"], ["lateralMax", "Lateral max"],
              ["heightCenter", "Height center"], ["heightMin", "Height min"],
              ["heightMax", "Height max"], ["priority", "Priority"], ["minSpacing", "Min spacing"],
              ["overflowPolicy", "Overflow"]
            ] as field}
              <label>{field[1]}<input type="number" value={band[field[0] as keyof PortBandInfo] as number}
                onchange={(event) => updateBand(pole, band.bandId, (draft) => {
                  (draft[field[0] as keyof PortBandInfo] as number) = numberValue(event);
                })} /></label>
            {/each}
            <label class="check"><input type="checkbox" checked={band.allowMultiple}
              onchange={(event) => updateBand(pole, band.bandId, (draft) => draft.allowMultiple = checkedValue(event))} />Allow multiple</label>
            <label class="check"><input type="checkbox" checked={band.enabled}
              onchange={(event) => updateBand(pole, band.bandId, (draft) => draft.enabled = checkedValue(event))} />Enabled</label>
            <button class="secondary" type="button" onclick={() => updatePole(pole, (draft) => {
              draft.portBands = draft.portBands.filter((item) => item.bandId !== band.bandId);
            })}>Band削除</button>
          </div>
        {/each}
        <button type="button" onclick={() => updatePole(pole, (draft) => {
          const nextId = Math.max(0, ...draft.portBands.map((band) => band.bandId)) + 1;
          draft.portBands.push({
            bandId: nextId, category: 1, layer: 1, side: 1, role: 0,
            lateralCenter: 0, lateralMin: -0.1, lateralMax: 0.1,
            heightCenter: draft.defaultHeight, heightMin: draft.defaultHeight - 0.1,
            heightMax: draft.defaultHeight + 0.1, priority: 0, minSpacing: 0.2,
            allowMultiple: false, overflowPolicy: 2, enabled: true
          });
        })}>Band追加</button>
      </details>

      <details>
        <summary>Anchor slots</summary>
        {#each pole.anchorSlots as slot}
          <div class="record">
            <strong>Slot {slot.slotId}</strong>
            {#each [
              ["slotId", "Id"], ["usage", "Usage"], ["localX", "Local X"],
              ["localY", "Local Y"], ["localZ", "Local Z"], ["priority", "Priority"]
            ] as field}
              <label>{field[1]}<input type="number" value={slot[field[0] as keyof typeof slot] as number}
                onchange={(event) => updatePole(pole, (draft) => {
                  const target = draft.anchorSlots.find((item) => item.slotId === slot.slotId);
                  if (target) (target[field[0] as keyof typeof target] as number) = numberValue(event);
                })} /></label>
            {/each}
            <label class="check"><input type="checkbox" checked={slot.enabled}
              onchange={(event) => updatePole(pole, (draft) => {
                const target = draft.anchorSlots.find((item) => item.slotId === slot.slotId);
                if (target) target.enabled = checkedValue(event);
              })} />Enabled</label>
            <button class="secondary" type="button" onclick={() => updatePole(pole, (draft) => {
              draft.anchorSlots = draft.anchorSlots.filter((item) => item.slotId !== slot.slotId);
            })}>Slot削除</button>
          </div>
        {/each}
        <button type="button" onclick={() => updatePole(pole, (draft) => {
          const nextId = Math.max(0, ...draft.anchorSlots.map((slot) => slot.slotId)) + 1;
          draft.anchorSlots.push({slotId: nextId, usage: 0, localX: 0, localY: 0, localZ: 0, priority: 0, enabled: true});
        })}>Slot追加</button>
      </details>
    {/if}
  </section>
</div>
