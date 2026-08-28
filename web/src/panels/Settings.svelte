<script lang="ts">
  import type { ViewerActions } from "../actions/viewer";
  import type { ViewerSnapshot } from "../store/viewer";

  interface Props {
    actions: ViewerActions;
    snapshot: ViewerSnapshot;
  }

  let { actions, snapshot }: Props = $props();

  const numberValue = (event: Event) =>
    Number((event.currentTarget as HTMLInputElement).value);
  const checkedValue = (event: Event) =>
    (event.currentTarget as HTMLInputElement).checked;
</script>

<div class="settings">
  <section>
    <h2>Viewport</h2>
    <label>
      Camera FOV
      <input type="range" min="20" max="110" value={snapshot.cameraFov}
        oninput={(event) => actions.setDrawOption("cameraFov", numberValue(event))} />
      <output>{snapshot.cameraFov.toFixed(0)}</output>
    </label>
    <label class="check"><input type="checkbox" checked={snapshot.showGroundGrid}
      onchange={(event) => actions.setDrawOption("showGroundGrid", checkedValue(event))} />
      Ground grid</label>
    <label class="check"><input type="checkbox" checked={snapshot.solidSupportRender}
      onchange={(event) => actions.setDrawOption("solidSupportRender", checkedValue(event))} />
      Solid support render</label>
  </section>

  <section>
    <h2>Selection</h2>
    <label class="check"><input type="checkbox" checked={snapshot.selectionIncludePoles}
      onchange={(event) => actions.setDrawOption("selectionIncludePoles", checkedValue(event))} />
      Select poles</label>
    <label class="check"><input type="checkbox" checked={snapshot.selectionIncludeMidair}
      onchange={(event) => actions.setDrawOption("selectionIncludeMidair", checkedValue(event))} />
      Select midair</label>
    <label class="check"><input type="checkbox" checked={snapshot.selectionIncludeSpans}
      onchange={(event) => actions.setDrawOption("selectionIncludeSpans", checkedValue(event))} />
      Select spans</label>
  </section>

  <section>
    <h2>New route variation</h2>
    <p class="setting-note">新しく描き始めるrouteの初期descriptorです。描画開始後は変更しません。</p>
    <label>
      Line density
      <input type="range" min="0.5" max="1.25" step="0.05"
        disabled={snapshot.pathPoints.length > 0}
        value={snapshot.routeVariation.density}
        oninput={(event) => actions.setRouteVariation("density", numberValue(event))} />
      <output>{snapshot.routeVariation.density.toFixed(2)}x</output>
    </label>
    <label>
      Height spread
      <input type="range" min="0.25" max="1.75" step="0.05"
        disabled={snapshot.pathPoints.length > 0}
        value={snapshot.routeVariation.heightSpread}
        oninput={(event) => actions.setRouteVariation("heightSpread", numberValue(event))} />
      <output>{snapshot.routeVariation.heightSpread.toFixed(2)}x</output>
    </label>
    <label>
      Lateral spread
      <input type="range" min="0.25" max="1.75" step="0.05"
        disabled={snapshot.pathPoints.length > 0}
        value={snapshot.routeVariation.lateralSpread}
        oninput={(event) => actions.setRouteVariation("lateralSpread", numberValue(event))} />
      <output>{snapshot.routeVariation.lateralSpread.toFixed(2)}x</output>
    </label>
    <button class="secondary" type="button" disabled={snapshot.pathPoints.length > 0}
      onclick={() => actions.rerollRouteSeed()}>
      Reroll seed
    </button>
    <output class="route-seed">
      {snapshot.wireRouteSeed === null ? "Seed: next route" : `Seed: ${snapshot.wireRouteSeed}`}
    </output>
  </section>

  {#if snapshot.selectedRouteVariation}
    <section>
      <h2>Selected route variation</h2>
      <p class="setting-note">選択中のrecipe-backed wireへ、Apply時に1 transactionで反映します。</p>
      <label>
        Line density
        <input type="range" min="0.5" max="1.25" step="0.05"
          disabled={snapshot.pathPoints.length > 0 &&
            snapshot.wireVariationId === snapshot.selectedRouteVariation.variationId}
          value={snapshot.selectedRouteVariationControls.density}
          oninput={(event) => actions.setSelectedRouteVariation("density", numberValue(event))} />
        <output>{snapshot.selectedRouteVariationControls.density.toFixed(2)}x</output>
      </label>
      <label>
        Height spread
        <input type="range" min="0.25" max="1.75" step="0.05"
          disabled={snapshot.pathPoints.length > 0 &&
            snapshot.wireVariationId === snapshot.selectedRouteVariation.variationId}
          value={snapshot.selectedRouteVariationControls.heightSpread}
          oninput={(event) => actions.setSelectedRouteVariation("heightSpread", numberValue(event))} />
        <output>{snapshot.selectedRouteVariationControls.heightSpread.toFixed(2)}x</output>
      </label>
      <label>
        Lateral spread
        <input type="range" min="0.25" max="1.75" step="0.05"
          disabled={snapshot.pathPoints.length > 0 &&
            snapshot.wireVariationId === snapshot.selectedRouteVariation.variationId}
          value={snapshot.selectedRouteVariationControls.lateralSpread}
          oninput={(event) => actions.setSelectedRouteVariation("lateralSpread", numberValue(event))} />
        <output>{snapshot.selectedRouteVariationControls.lateralSpread.toFixed(2)}x</output>
      </label>
      <button class="secondary" type="button"
        disabled={snapshot.pathPoints.length > 0 &&
          snapshot.wireVariationId === snapshot.selectedRouteVariation.variationId}
        onclick={() => actions.rerollSelectedRouteVariation()}>
        Reroll seed
      </button>
      <output class="route-seed">Seed: {snapshot.selectedRouteVariation.routeSeed}</output>
      <button type="button"
        disabled={snapshot.pathPoints.length > 0 &&
          snapshot.wireVariationId === snapshot.selectedRouteVariation.variationId}
        onclick={() => actions.applySelectedRouteVariation()}>
        Apply to selected route
      </button>
    </section>
  {/if}

  <section>
    <h2>Current wire appearance</h2>
    <p class="setting-note">確定済みwireへ即時反映します。Non-HV irregularityはHVを変更しません。</p>
    <label>
      Sag spread
      <input type="range" min="0" max="2" step="0.05"
        value={snapshot.variation.sagVariationScale / 0.12}
        oninput={(event) => actions.previewVariation(
          "sagVariationScale", numberValue(event) * 0.12)}
        onchange={(event) => actions.commitVariation(
          "sagVariationScale", numberValue(event) * 0.12)} />
      <output>{(snapshot.variation.sagVariationScale / 0.12).toFixed(2)}x</output>
    </label>
    <label>
      Non-HV irregularity
      <input type="range" min="0" max="2" step="0.05"
        value={snapshot.visual.wireIrregularityScale}
        oninput={(event) => actions.previewVisual(
          "wireIrregularityScale", numberValue(event))}
        onchange={(event) => actions.commitVisual(
          "wireIrregularityScale", numberValue(event))} />
      <output>{snapshot.visual.wireIrregularityScale.toFixed(2)}x</output>
    </label>
  </section>

  <section>
    <h2>Pole tilt</h2>
    <label>
      Max tilt (also used for generation)
      <input type="number" step="0.5" value={snapshot.maxTiltDeg}
        onchange={(event) => actions.setDrawOption("maxTiltDeg", numberValue(event))} />
    </label>
    <button
      class="secondary"
      type="button"
      disabled={snapshot.selection?.kind !== "pole"}
      onclick={() => actions.applyTiltToSelection(snapshot.maxTiltDeg)}
    >
      Apply tilt to selected pole
    </button>
    <button type="button" onclick={() => actions.applyTiltToAll(snapshot.maxTiltDeg)}>
      Apply tilt to all poles
    </button>
  </section>

  <section>
    <h2>Geometry</h2>
    <label>
      Curve samples
      <input
        type="range"
        min="2"
        max="64"
        value={snapshot.geometry.curveSamples}
        oninput={(event) =>
          actions.previewGeometry("curveSamples", numberValue(event))}
        onpointerup={(event) =>
          actions.commitGeometry("curveSamples", numberValue(event))}
      />
      <output>{snapshot.geometry.curveSamples}</output>
    </label>
    <label class="check">
      <input
        type="checkbox"
        checked={snapshot.geometry.sagEnabled}
        onchange={(event) =>
          actions.commitGeometry("sagEnabled", checkedValue(event))}
      />
      Sag enabled
    </label>
    <label>
      Sag factor
      <input
        type="number"
        step="0.005"
        value={snapshot.geometry.sagFactor}
        oninput={(event) =>
          actions.previewGeometry("sagFactor", numberValue(event))}
        onblur={(event) =>
          actions.commitGeometry("sagFactor", numberValue(event))}
      />
    </label>
    <label>
      Pole clearance
      <input
        type="number"
        step="0.005"
        value={snapshot.geometry.poleClearance}
        oninput={(event) =>
          actions.previewGeometry("poleClearance", numberValue(event))}
        onblur={(event) =>
          actions.commitGeometry("poleClearance", numberValue(event))}
      />
    </label>
    <button class="secondary" type="button" onclick={() => actions.resetSpanReferenceLengths()}>
      Reset span reference lengths
    </button>
  </section>

  <section>
    <h2>Layout</h2>
    <label class="check">
      <input
        type="checkbox"
        checked={snapshot.layout.angleCorrectionEnabled}
        onchange={(event) =>
          actions.commitLayout("angleCorrectionEnabled", checkedValue(event))}
      />
      Angle correction
    </label>
    <label>
      Corner threshold
      <input
        type="number"
        step="1"
        value={snapshot.layout.cornerThresholdDeg}
        onchange={(event) =>
          actions.commitLayout("cornerThresholdDeg", numberValue(event))}
      />
    </label>
    <label>
      Min side scale
      <input
        type="number"
        step="0.05"
        value={snapshot.layout.minSideScale}
        onchange={(event) =>
          actions.commitLayout("minSideScale", numberValue(event))}
      />
    </label>
    <label>
      Max side scale
      <input
        type="number"
        step="0.05"
        value={snapshot.layout.maxSideScale}
        onchange={(event) =>
          actions.commitLayout("maxSideScale", numberValue(event))}
      />
    </label>
  </section>

  <section>
    <h2>Visual</h2>
    <label class="check">
      <input
        type="checkbox"
        checked={snapshot.visual.enableInsulators}
        onchange={(event) =>
          actions.commitVisual("enableInsulators", checkedValue(event))}
      />
      Insulators
    </label>
    <label>
      Insulator radius
      <input
        type="number"
        step="0.005"
        value={snapshot.visual.insulatorRadius}
        oninput={(event) =>
          actions.previewVisual("insulatorRadius", numberValue(event))}
        onblur={(event) =>
          actions.commitVisual("insulatorRadius", numberValue(event))}
      />
    </label>
    <label>
      Insulator length
      <input
        type="number"
        step="0.005"
        value={snapshot.visual.insulatorLength}
        oninput={(event) =>
          actions.previewVisual("insulatorLength", numberValue(event))}
        onblur={(event) =>
          actions.commitVisual("insulatorLength", numberValue(event))}
      />
    </label>
  </section>
</div>
