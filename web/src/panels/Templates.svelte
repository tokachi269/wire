<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type {
    BundleTemplateInfo,
    CableTemplateInfo,
    PoleTemplateInfo,
    PortBandInfo
  } from "../model";
  import type { ViewerSnapshot } from "../store/viewer";
  import {
    ANCHOR_USAGE_LABELS,
    CATEGORY_LABELS,
    CONTINUITY_LABELS,
    MATERIAL_LABELS,
    OVERFLOW_LABELS,
    ROLE_LABELS,
    SIDE_LABELS,
    SPAN_LAYER_LABELS,
    categoryShort,
    fmt,
    labelOf,
    round6
  } from "../labels";

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
  const colorValue = (rgba: number) =>
    `#${((rgba >>> 8) & 0xffffff).toString(16).padStart(6, "0")}`;
  const rgbaValue = (hex: string, current: number) =>
    ((Number.parseInt(hex.slice(1), 16) << 8) | (current & 0xff)) >>> 0;

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
  const selectedPoleUsage = () =>
    snapshot.poles.filter(
      (pole) => pole.poleTypeId === snapshot.selectedPoleTemplateId
    ).length;

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

  function cloneBundle(template: BundleTemplateInfo): BundleTemplateInfo {
    return {
      ...template,
      spanVisualAssembly: { ...template.spanVisualAssembly }
    };
  }

  function clonePole(template: PoleTemplateInfo): PoleTemplateInfo {
    return {
      ...template,
      portBands: template.portBands.map((band) => ({ ...band })),
      anchorSlots: template.anchorSlots.map((slot) => ({ ...slot }))
    };
  }

  function updateBundle(
    template: BundleTemplateInfo,
    change: (draft: BundleTemplateInfo) => void
  ): void {
    const draft = cloneBundle(template);
    change(draft);
    actions.commitBundleTemplate(draft);
  }

  function updatePole(
    template: PoleTemplateInfo,
    change: (draft: PoleTemplateInfo) => void
  ): void {
    const draft = clonePole(template);
    change(draft);
    actions.commitPoleTemplate(draft);
  }

  function sortedBands(template: PoleTemplateInfo): PortBandInfo[] {
    return [...template.portBands].sort(
      (a, b) => a.category - b.category || b.heightCenter - a.heightCenter || a.bandId - b.bandId
    );
  }

  function bandTitle(band: PortBandInfo): string {
    return `Band ${band.bandId} · ${categoryShort(band.category)} / ${labelOf(SIDE_LABELS, band.side)} / h${fmt(band.heightCenter, 2)}`;
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
  <section class="pole-template">
    <h2>Pole template</h2>
    <select value={snapshot.selectedPoleTemplateId ?? ""}
      onchange={(event) => actions.selectPoleTemplate(numberValue(event))}>
      {#each snapshot.poleTemplates as template}<option value={template.id}>{template.name}</option>{/each}
    </select>
    {#if selectedPole()}
      {@const pole = selectedPole()!}
      <p class:warning={selectedPoleUsage() === 0}>
        Poles using this template: {selectedPoleUsage()}
        {#if selectedPoleUsage() === 0}
          · Changes to this template are not visible in the current scene
        {/if}
      </p>
      <label>Default height
        <input type="number" step="0.05" value={fmt(pole.defaultHeight)}
          oninput={(event) => actions.previewPoleDefaultHeight(numberValue(event))}
          onblur={(event) => updatePole(pole, (draft) => draft.defaultHeight = round6(numberValue(event)))} />
      </label>

      <details>
        <summary>Name / description</summary>
        <label>Name
          <input value={pole.name}
            onchange={(event) => updatePole(pole, (draft) => draft.name = textValue(event))} />
        </label>
        <label>Description
          <input value={pole.description}
            onchange={(event) => updatePole(pole, (draft) => draft.description = textValue(event))} />
        </label>
      </details>

      <details>
        <summary>Port bands ({pole.portBands.length})</summary>
        {#each sortedBands(pole) as band (band.bandId)}
          <div class="record">
            <strong>{bandTitle(band)}</strong>
            <label>Category
              <select value={band.category}
                onchange={(event) => updateBand(pole, band.bandId, (draft) => {
                  draft.category = numberValue(event);
                })}>
                {#each CATEGORY_LABELS as label, category}
                  <option value={category}>{label}</option>
                {/each}
              </select>
            </label>
            <label>Side
              <select value={band.side}
                onchange={(event) => updateBand(pole, band.bandId, (draft) => {
                  draft.side = numberValue(event);
                })}>
                {#each SIDE_LABELS as label, side}<option value={side}>{label}</option>{/each}
              </select>
            </label>
            <label>Role
              <select value={band.role}
                onchange={(event) => updateBand(pole, band.bandId, (draft) => {
                  draft.role = numberValue(event);
                })}>
                {#each ROLE_LABELS as label, role}<option value={role}>{label}</option>{/each}
              </select>
            </label>
            <label>Overflow
              <select value={band.overflowPolicy}
                onchange={(event) => updateBand(pole, band.bandId, (draft) => {
                  draft.overflowPolicy = numberValue(event);
                })}>
                {#each OVERFLOW_LABELS as label, policy}<option value={policy}>{label}</option>{/each}
              </select>
            </label>
            {#each [
              ["bandId", "Id"], ["layer", "Layer"],
              ["lateralCenter", "Lateral center"],
              ["lateralMin", "Lateral min"], ["lateralMax", "Lateral max"],
              ["heightCenter", "Height center"], ["heightMin", "Height min"],
              ["heightMax", "Height max"], ["priority", "Priority"], ["minSpacing", "Min spacing"]
            ] as field}
              <label>{field[1]}<input type="number" value={fmt(band[field[0] as keyof PortBandInfo] as number)}
                onchange={(event) => updateBand(pole, band.bandId, (draft) => {
                  (draft[field[0] as keyof PortBandInfo] as number) = round6(numberValue(event));
                })} /></label>
            {/each}
            <label class="check"><input type="checkbox" checked={band.allowMultiple}
              onchange={(event) => updateBand(pole, band.bandId, (draft) => draft.allowMultiple = checkedValue(event))} />Allow multiple</label>
            <label class="check"><input type="checkbox" checked={band.enabled}
              onchange={(event) => updateBand(pole, band.bandId, (draft) => draft.enabled = checkedValue(event))} />Enabled</label>
            <button class="secondary" type="button" onclick={() => updatePole(pole, (draft) => {
              draft.portBands = draft.portBands.filter((item) => item.bandId !== band.bandId);
            })}>Remove band</button>
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
        })}>Add band</button>
      </details>

      <details>
        <summary>Anchor slots ({pole.anchorSlots.length})</summary>
        {#each pole.anchorSlots as slot (slot.slotId)}
          <div class="record">
            <strong>Slot {slot.slotId} · {labelOf(ANCHOR_USAGE_LABELS, slot.usage)}</strong>
            <label>Usage
              <select value={slot.usage}
                onchange={(event) => updatePole(pole, (draft) => {
                  const target = draft.anchorSlots.find((item) => item.slotId === slot.slotId);
                  if (target) target.usage = numberValue(event);
                })}>
                {#each ANCHOR_USAGE_LABELS as label, usage}<option value={usage}>{label}</option>{/each}
              </select>
            </label>
            {#each [
              ["slotId", "Id"], ["localX", "Local X"],
              ["localY", "Local Y"], ["localZ", "Local Z"], ["priority", "Priority"]
            ] as field}
              <label>{field[1]}<input type="number" value={fmt(slot[field[0] as keyof typeof slot] as number)}
                onchange={(event) => updatePole(pole, (draft) => {
                  const target = draft.anchorSlots.find((item) => item.slotId === slot.slotId);
                  if (target) (target[field[0] as keyof typeof target] as number) = round6(numberValue(event));
                })} /></label>
            {/each}
            <label class="check"><input type="checkbox" checked={slot.enabled}
              onchange={(event) => updatePole(pole, (draft) => {
                const target = draft.anchorSlots.find((item) => item.slotId === slot.slotId);
                if (target) target.enabled = checkedValue(event);
              })} />Enabled</label>
            <button class="secondary" type="button" onclick={() => updatePole(pole, (draft) => {
              draft.anchorSlots = draft.anchorSlots.filter((item) => item.slotId !== slot.slotId);
            })}>Remove slot</button>
          </div>
        {/each}
        <button type="button" onclick={() => updatePole(pole, (draft) => {
          const nextId = Math.max(0, ...draft.anchorSlots.map((slot) => slot.slotId)) + 1;
          draft.anchorSlots.push({slotId: nextId, usage: 0, localX: 0, localY: 0, localZ: 0, priority: 0, enabled: true});
        })}>Add slot</button>
      </details>
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
      {@const relatedPole = snapshot.poleTemplates.find((template) => template.id === bundle.relatedPoleTypeId)}
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
          {#each [1, 2, 3, 4, 5] as layer}
            <option value={layer}>{labelOf(SPAN_LAYER_LABELS, layer)}</option>
          {/each}
        </select>
      </label>
      {#each [
        ["allowMidairNode", "Allow midair node"],
        ["allowMidairBranch", "Allow midair branch"]
      ] as field}
        <label class="check"><input type="checkbox" checked={bundle[field[0] as keyof BundleTemplateInfo] as boolean}
          onchange={(event) => actions.commitBundleTemplate(
            patchBundle(bundle, field[0] as keyof BundleTemplateInfo, checkedValue(event) as never)
          )} />{field[1]}</label>
      {/each}
      <button class="secondary" type="button" onclick={() => actions.applyRelatedPoleType(bundle.id)}>
        Apply related pole type to existing poles
      </button>
      <div class="helix-editor">
        <div class="helix-heading">
          <h3>Helix</h3>
          <span class:enabled={bundle.spanVisualAssembly.supportPathEnabled}>
            {bundle.spanVisualAssembly.supportPathEnabled ? "On" : "Off"}
          </span>
        </div>
        <label class="check"><input type="checkbox" checked={bundle.spanVisualAssembly.supportPathEnabled}
          onchange={(event) => updateBundle(bundle, (draft) => {
            draft.spanVisualAssembly.supportPathEnabled = checkedValue(event);
          })} />Enable support path</label>
        <label class="check"><input type="checkbox" checked={bundle.spanVisualAssembly.helixEnabled}
          onchange={(event) => updateBundle(bundle, (draft) => {
            draft.spanVisualAssembly.helixEnabled = checkedValue(event);
          })} />Enable helix</label>
        <label>Support band
          <select value={bundle.supportWirePoleBandId}
            onchange={(event) => updateBundle(bundle, (draft) => {
              draft.supportWirePoleBandId = Math.trunc(numberValue(event));
            })}>
            <option value="0">Member band</option>
            {#if relatedPole}
              {#each sortedBands(relatedPole) as band}
                <option value={band.bandId} disabled={!band.enabled}>
                  {bandTitle(band)}{band.enabled ? "" : " · disabled"}
                </option>
              {/each}
            {/if}
          </select>
        </label>
        {#each [
          ["helixRadius", "Helix radius (0 = auto-fit)", 0.005],
          ["helixClearance", "Helix clearance", 0.005],
          ["helixTurnsPerMeter", "Helix turns / m", 0.1],
          ["endpointTrim", "Endpoint trim", 0.05]
        ] as field}
          <label>{field[1]}<input type="number" step={field[2]}
            value={fmt(bundle.spanVisualAssembly[field[0] as keyof typeof bundle.spanVisualAssembly] as number)}
            onchange={(event) => updateBundle(bundle, (draft) => {
              (draft.spanVisualAssembly[field[0] as keyof typeof draft.spanVisualAssembly] as number) =
                round6(numberValue(event));
            })} /></label>
        {/each}
        <p class="hint">Band 0 follows each member's saved band. Helix requires an explicit support band.</p>
        <details>
          <summary>Wander, twist, and sampling</summary>
          {#each [
            ["memberWanderRatio", "Member wander ratio", 0.05],
            ["memberWanderWavelength", "Member wander wavelength", 0.1],
            ["memberWanderPhaseBias", "Member wander phase", 0.1],
            ["memberTwistTurnsPerMeter", "Member twist turns / m", 0.1],
            ["memberTwistPhase", "Member twist phase", 0.1]
          ] as field}
            <label>{field[1]}<input type="number" step={field[2]}
              value={fmt(bundle.spanVisualAssembly[field[0] as keyof typeof bundle.spanVisualAssembly] as number)}
              onchange={(event) => updateBundle(bundle, (draft) => {
                (draft.spanVisualAssembly[field[0] as keyof typeof draft.spanVisualAssembly] as number) =
                  round6(numberValue(event));
              })} /></label>
          {/each}
          <label>Helix samples / turn
            <input type="number" min="4" step="1" value={bundle.spanVisualAssembly.helixSamplesPerTurn}
              onchange={(event) => updateBundle(bundle, (draft) => {
                draft.spanVisualAssembly.helixSamplesPerTurn = Math.trunc(numberValue(event));
              })} />
          </label>
          <p class="hint">Member twist also works when helix is disabled.</p>
        </details>
      </div>
    {/if}
  </section>

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
        <input type="number" step="0.001" value={fmt(cable.outerDiameter)}
          oninput={(event) => actions.previewCableTemplate("outerDiameter", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "outerDiameter", numberValue(event)))} />
      </label>
      <label>Bend stiffness
        <input type="number" step="0.1" value={fmt(cable.bendStiffness)}
          oninput={(event) => actions.previewCableTemplate("bendStiffness", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "bendStiffness", numberValue(event)))} />
      </label>
      <label>Min bend radius
        <input type="number" step="0.01" value={fmt(cable.minBendRadius)}
          oninput={(event) => actions.previewCableTemplate("minBendRadius", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "minBendRadius", numberValue(event)))} />
      </label>
      <label>Material
        <select value={cable.materialStyle}
          onchange={(event) => actions.commitCableTemplate(patchCable(cable, "materialStyle", numberValue(event)))}>
          {#each MATERIAL_LABELS as label, style}<option value={style}>{label}</option>{/each}
        </select>
      </label>
      <label>Wire color
        <input type="color" value={colorValue(cable.colorRgba)}
          onchange={(event) => actions.commitCableTemplate(
            patchCable(cable, "colorRgba", rgbaValue(textValue(event), cable.colorRgba))
          )} />
      </label>
      <label>Sag factor
        <input type="number" step="0.005" value={fmt(cable.sagFactor)}
          oninput={(event) => actions.previewCableTemplate("sagFactor", numberValue(event))}
          onblur={(event) => actions.commitCableTemplate(patchCable(cable, "sagFactor", numberValue(event)))} />
      </label>
      <label>Continuity
        <select value={cable.continuityPolicy}
          onchange={(event) => actions.commitCableTemplate(patchCable(cable, "continuityPolicy", numberValue(event)))}>
          {#each CONTINUITY_LABELS as label, policy}<option value={policy}>{label}</option>{/each}
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
            <input type="number" step="0.01" value={fmt(cable[field[0] as keyof CableTemplateInfo] as number)}
              oninput={(event) =>
                actions.previewCableTemplate(field[0] as keyof CableTemplateInfo, numberValue(event) as never)}
              onblur={(event) => actions.commitCableTemplate(
                patchCable(cable, field[0] as keyof CableTemplateInfo, numberValue(event) as never)
              )} />
          </label>
        {/each}
      {/if}
    {/if}
  </section>
</div>
