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
        checked={snapshot.visual.enableSupportStructures}
        onchange={(event) =>
          actions.commitVisual("enableSupportStructures", checkedValue(event))}
      />
      Support structures
    </label>
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
      Support center threshold
      <input
        type="number"
        step="0.005"
        value={snapshot.visual.supportCenterThreshold}
        oninput={(event) =>
          actions.previewVisual("supportCenterThreshold", numberValue(event))}
        onblur={(event) =>
          actions.commitVisual("supportCenterThreshold", numberValue(event))}
      />
    </label>
    <label>
      Support arm extra
      <input
        type="number"
        step="0.01"
        value={snapshot.visual.supportArmExtra}
        oninput={(event) =>
          actions.previewVisual("supportArmExtra", numberValue(event))}
        onblur={(event) =>
          actions.commitVisual("supportArmExtra", numberValue(event))}
      />
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

  <section>
    <h2>Orientation</h2>
    <label>
      Camera FOV
      <input type="range" min="20" max="110" value={snapshot.cameraFov}
        oninput={(event) => actions.setDrawOption("cameraFov", numberValue(event))} />
      <output>{snapshot.cameraFov.toFixed(0)}</output>
    </label>
    <label class="check"><input type="checkbox" checked={snapshot.solidSupportRender}
      onchange={(event) => actions.setDrawOption("solidSupportRender", checkedValue(event))} />
      Solid support render</label>
    <label class="check"><input type="checkbox" checked={snapshot.selectionIncludePoles}
      onchange={(event) => actions.setDrawOption("selectionIncludePoles", checkedValue(event))} />
      Select poles</label>
    <label class="check"><input type="checkbox" checked={snapshot.selectionIncludeMidair}
      onchange={(event) => actions.setDrawOption("selectionIncludeMidair", checkedValue(event))} />
      Select midair</label>
    <label class="check"><input type="checkbox" checked={snapshot.selectionIncludeSpans}
      onchange={(event) => actions.setDrawOption("selectionIncludeSpans", checkedValue(event))} />
      Select spans</label>
    <label>
      Max tilt (生成時にも適用)
      <input type="number" step="0.5" value={snapshot.maxTiltDeg}
        onchange={(event) => actions.setDrawOption("maxTiltDeg", numberValue(event))} />
    </label>
    <button type="button" onclick={() => actions.applyTiltToAll(snapshot.maxTiltDeg)}>
      全poleへtilt適用
    </button>
    <button
      class="secondary"
      type="button"
      disabled={snapshot.selection?.kind !== "pole"}
      onclick={() => actions.applyTiltToSelection(snapshot.maxTiltDeg)}
    >
      選択poleへtilt適用
    </button>
    <button
      class="secondary"
      type="button"
      onclick={() => actions.resetSpanReferenceLengths()}
    >
      span基準長をreset
    </button>
  </section>
</div>
