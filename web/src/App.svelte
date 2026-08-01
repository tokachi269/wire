<script lang="ts">
  import { onMount } from "svelte";
  import {
    Cable,
    GitBranch,
    ListPlus,
    Merge,
    Minus,
    PenLine,
    Pencil,
    Route,
    Spline,
    SquareDashed,
    Trash2
  } from "@lucide/svelte";
  import type { ViewerActions } from "./actions/viewer";
  import Settings from "./panels/Settings.svelte";
  import Templates from "./panels/Templates.svelte";
  import Outliner from "./panels/Outliner.svelte";
  import RoadPanel from "./panels/RoadPanel.svelte";
  import SelectionInspector from "./panels/SelectionInspector.svelte";
  import { buildInfo } from "./buildInfo";
  import { categoryShort } from "./labels";
  import type { GenerationTiming } from "./model";
  import {
    createViewerSnapshot,
    type ViewerSnapshot,
    type ViewerStore
  } from "./store/viewer";

  interface Props {
    actions: ViewerActions;
    store: ViewerStore;
    mountScene: (host: HTMLElement) => void;
  }

  let { actions, store, mountScene }: Props = $props();
  let sceneHost: HTMLElement;
  let snapshot: ViewerSnapshot = $state(createViewerSnapshot());
  let stopResize: (() => void) | null = null;
  let editStart:
    | {
        element: HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement;
        value: string;
        checked: boolean | null;
      }
    | null = null;

  const generationTimingRows: Array<[string, keyof GenerationTiming]> = [
    ["prepare", "prepareMs"],
    ["check", "checkMs"],
    ["pairs", "pairsMs"],
    ["preflight", "preflightMs"],
    ["intent", "intentMs"],
    ["support groups", "supportGroupsMs"],
    ["emit", "emitMs"],
    ["save graph", "saveGraphMs"],
    ["rules", "rulesMs"],
    ["layout", "layoutMs"],
    ["geom", "geomMs"],
    ["draw", "drawMs"],
    ["total", "totalMs"]
  ];

  function formatMs(value: number | null | undefined): string {
    return value === null || value === undefined || !Number.isFinite(value)
      ? "—"
      : `${value.toFixed(2)} ms`;
  }

  function formatBuildTime(value: string): string {
    const date = new Date(value);
    return Number.isNaN(date.getTime()) ? value : date.toLocaleString();
  }

  function overheadMs(total: number | null, core: number | null): number | null {
    if (total === null || core === null) {
      return null;
    }
    return Math.max(0, total - core);
  }

  function beginResize(side: "left" | "right", event: PointerEvent): void {
    event.preventDefault();
    const startX = event.clientX;
    const startWidth =
      side === "left" ? snapshot.workspaceLeftWidth : snapshot.workspaceWidth;
    const move = (moveEvent: PointerEvent) => {
      const delta = moveEvent.clientX - startX;
      const width = Math.max(
        side === "left" ? 180 : 260,
        Math.min(side === "left" ? 420 : 560, startWidth + (side === "left" ? delta : -delta))
      );
      actions.setDrawOption(
        side === "left" ? "workspaceLeftWidth" : "workspaceWidth",
        width
      );
    };
    const stop = () => {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", stop);
      stopResize = null;
    };
    stopResize?.();
    stopResize = stop;
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", stop);
  }

  onMount(() => {
    const unsubscribe = store.value.subscribe((value) => {
      snapshot = value;
    });
    mountScene(sceneHost);
    if (typeof window.matchMedia === "function" && window.matchMedia("(max-width: 700px)").matches) {
      actions.setDrawOption("showLeftPanel", false);
      actions.setDrawOption("showRightPanel", false);
    }
    const handleFocusIn = (event: FocusEvent) => {
      const target = event.target;
      if (
        target instanceof HTMLInputElement ||
        target instanceof HTMLSelectElement ||
        target instanceof HTMLTextAreaElement
      ) {
        editStart = {
          element: target,
          value: target.value,
          checked: target instanceof HTMLInputElement ? target.checked : null
        };
      }
    };
    const handleFocusOut = (event: FocusEvent) => {
      if (event.target === editStart?.element) {
        editStart = null;
      }
    };
    const handleKey = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        event.preventDefault();
        const active = event.target;
        const editing =
          active instanceof HTMLInputElement ||
          active instanceof HTMLSelectElement ||
          active instanceof HTMLTextAreaElement;
        if (
          snapshot.interaction &&
          active instanceof HTMLInputElement &&
          (typeof snapshot.interaction.startValue === "number" ||
            typeof snapshot.interaction.startValue === "boolean")
        ) {
          if (active.type === "checkbox") {
            active.checked = Boolean(snapshot.interaction.startValue);
          } else {
            active.value = String(snapshot.interaction.startValue);
          }
        } else if (active === editStart?.element) {
          active.value = editStart.value;
          if (active instanceof HTMLInputElement && editStart.checked !== null) {
            active.checked = editStart.checked;
          }
        } else if (!editing) {
          if (snapshot.activeTool === "wire" && snapshot.pathPoints.length > 0) {
            actions.clearPath();
          } else if (snapshot.activeTool === "road") {
            actions.undoActiveTool();
          }
        }
        actions.cancel(editing);
        if (editing) {
          active.blur();
        }
        return;
      }
      const target = event.target as HTMLElement | null;
      const editing =
        target instanceof HTMLInputElement ||
        target instanceof HTMLSelectElement ||
        target instanceof HTMLTextAreaElement ||
        target?.isContentEditable;
      if (event.key === "Enter" && !editing) {
        if (snapshot.activeTool === "wire") {
          event.preventDefault();
          actions.generatePath();
        } else if (snapshot.activeTool === "road") {
          event.preventDefault();
          actions.commitRoadPath();
        }
      }
    };
    window.addEventListener("focusin", handleFocusIn);
    window.addEventListener("focusout", handleFocusOut);
    window.addEventListener("keydown", handleKey);
    return () => {
      unsubscribe();
      stopResize?.();
      window.removeEventListener("focusin", handleFocusIn);
      window.removeEventListener("focusout", handleFocusOut);
      window.removeEventListener("keydown", handleKey);
    };
  });
</script>

<svelte:head>
  <meta
    name="description"
    content="Saved backbone derived-output viewer"
  />
</svelte:head>

<main>
  <header>
    <div>
      <p class="eyebrow">WIRE</p>
      <h1>Viewer</h1>
    </div>
    <div class="header-actions">
      <button class="secondary mini-action" type="button" onclick={() => actions.undoPathPoint()}>
        Undo
      </button>
      <button class="secondary mini-action" type="button" onclick={() => actions.clearPath()}>
        Clear
      </button>
      <button class="secondary mini-action" type="button" onclick={() => void actions.exportWorkspaceFile()}>
        Export
      </button>
      <button class="secondary" type="button" onclick={() => actions.resetWorkspace()}>
        Reset
      </button>
      <button type="button" onclick={() => actions.generatePath()}>
        Generate Path
      </button>
    </div>
  </header>

  <section
    class="workspace"
    style={`--left-width:${snapshot.workspaceLeftWidth}px;--right-width:${snapshot.workspaceWidth}px`}
  >
    <button
      class="side-tab left-tab"
      class:active={snapshot.showLeftPanel}
      type="button"
      aria-label={snapshot.showLeftPanel ? "Hide Outliner" : "Show Outliner"}
      onclick={() => actions.setDrawOption("showLeftPanel", !snapshot.showLeftPanel)}
    >
      Outliner
    </button>
    <button
      class="side-tab right-tab"
      class:active={snapshot.showRightPanel}
      type="button"
      aria-label={snapshot.showRightPanel ? "Hide Inspector" : "Show Inspector"}
      onclick={() => actions.setDrawOption("showRightPanel", !snapshot.showRightPanel)}
    >
      Inspector
    </button>

    {#if snapshot.showLeftPanel}
      <div class="left-region">
        <Outliner {actions} {snapshot} />
      </div>
      <button
        class="resize-handle"
        aria-label="Resize left sidebar"
        type="button"
        onpointerdown={(event) => beginResize("left", event)}
      ></button>
    {/if}

    <div class="viewport" bind:this={sceneHost}></div>

    <div class="viewport-tools" aria-label="Viewport draw tools">
      <div class="tool-group domain-tools">
        <button
          class:active={snapshot.activeTool === "wire"}
          type="button"
          aria-label="Wire tool"
          title="Wire tool"
          onclick={() => actions.setActiveTool("wire")}
        ><Cable size={20} aria-hidden="true" /></button>
        <button
          class:active={snapshot.activeTool === "road"}
          type="button"
          aria-label="Road tool"
          title="Road tool"
          onclick={() => actions.setActiveTool("road")}
        ><Route size={20} aria-hidden="true" /></button>
      </div>
      {#if snapshot.activeTool === "road"}
        <div class="tool-group road-operation-tools">
          <button class:active={snapshot.road.operation === "draw"} type="button"
            aria-label="Draw road" title="Draw road" onclick={() => actions.setRoadOperation("draw")}
          ><Pencil size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "edit"} type="button"
            aria-label="Edit road alignment" title="Edit road alignment"
            onclick={() => actions.setRoadOperation("edit")}
          ><Spline size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "delete"} type="button"
            aria-label="Delete road" title="Delete road" onclick={() => actions.setRoadOperation("delete")}
          ><Trash2 size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "line-marking"} type="button"
            aria-label="Add line marking" title="Add line marking"
            onclick={() => actions.setRoadOperation("line-marking")}
          ><PenLine size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "area-marking"} type="button"
            aria-label="Add area marking" title="Add area marking"
            onclick={() => actions.setRoadOperation("area-marking")}
          ><SquareDashed size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "add-lane"} type="button"
            aria-label="Add lane" title="Add lane"
            onclick={() => actions.setRoadOperation("add-lane")}
          ><ListPlus size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "branch-lane"} type="button"
            aria-label="Branch lane" title="Branch lane"
            onclick={() => actions.setRoadOperation("branch-lane")}
          ><GitBranch size={19} aria-hidden="true" /></button>
          <button class:active={snapshot.road.operation === "merge-lane"} type="button"
            aria-label="Merge lane" title="Merge lane"
            onclick={() => actions.setRoadOperation("merge-lane")}
          ><Merge size={19} aria-hidden="true" /></button>
        </div>
      {/if}
      {#if snapshot.activeTool === "road" && snapshot.road.operation === "draw"}
        <div class="tool-group road-shape-tools">
          <button
            class:active={snapshot.road.mode === "line"}
            type="button"
            aria-label="Straight road"
            title="Straight road"
            onclick={() => actions.setRoadMode("line")}
          ><Minus size={21} aria-hidden="true" /></button>
          <button
            class:active={snapshot.road.mode === "bezier"}
            type="button"
            aria-label="Curved road"
            title="Curved road"
            onclick={() => actions.setRoadMode("bezier")}
          ><Spline size={20} aria-hidden="true" /></button>
        </div>
      {/if}
    </div>

    {#if snapshot.showRightPanel}
      <button
        class="resize-handle"
        aria-label="Resize right sidebar"
        type="button"
        onpointerdown={(event) => beginResize("right", event)}
      ></button>
      <div class="right-region">
        <div class="domain-tabs">
          <button
            class:active={snapshot.rightPanelMode === "wire"}
            type="button"
            onclick={() => actions.setDrawOption("rightPanelMode", "wire")}
          >
            Wire
          </button>
          <button
            class:active={snapshot.rightPanelMode === "road"}
            type="button"
            onclick={() => actions.setDrawOption("rightPanelMode", "road")}
          >
            Road
          </button>
        </div>
        {#if snapshot.rightPanelMode === "wire"}
          <div class="draw-panel">
            <div class="draw-panel-head">
              <p class="panel-label">DRAW PATH</p>
              <strong class="point-count">{snapshot.pathPoints.length} points</strong>
            </div>
            <button class="secondary repro-button" type="button" onclick={() => actions.exportReproCapture()}>
              Repro capture
            </button>
            <label>
              Pole template
              <select
                value={snapshot.selectedPoleTemplateId ?? ""}
                onchange={(event) =>
                  actions.selectPoleTemplate(Number(event.currentTarget.value))}
              >
                {#each snapshot.poleTemplates as template}
                  <option value={template.id}>{template.name}</option>
                {/each}
              </select>
            </label>
            <fieldset class="bundle-picker">
              <legend>Bundle placement</legend>
              <div class="bundle-placement bundle-placement-head">
                <span>Bundle</span><span>Height</span><span>Offset</span><span>Spread</span><span>Count</span>
              </div>
              {#each snapshot.drawBundlePlacements as placement (placement.id)}
                {@const template = snapshot.bundleTemplates.find((item) => item.id === placement.bundleTemplateId)}
                {#if template !== undefined && template.category !== 4}
                  {@const siblings = snapshot.drawBundlePlacements.filter((item) => item.bundleTemplateId === template.id)}
                  {@const index = siblings.findIndex((item) => item.id === placement.id)}
                  <div class="bundle-placement">
                    <span class="bundle-placement-name">
                      <strong>{categoryShort(template.category)}{siblings.length > 1 ? ` ${index + 1}` : ""}</strong>
                      <button class="bundle-placement-add" type="button"
                        aria-label={`Duplicate ${template.name}`} title="Duplicate bundle placement"
                        onclick={() => actions.duplicateDrawBundlePlacement(placement.id)}>+</button>
                    </span>
                    <input aria-label={`${template.name} ${index + 1} height`} type="number" step="0.05" value={placement.height}
                      oninput={(event) => actions.updateDrawBundlePlacement(placement.id, { height: Number(event.currentTarget.value) })} />
                    <input aria-label={`${template.name} ${index + 1} offset`} type="number" step="0.02" value={placement.offset}
                      oninput={(event) => actions.updateDrawBundlePlacement(placement.id, { offset: Number(event.currentTarget.value) })} />
                    <input aria-label={`${template.name} ${index + 1} spread`} type="number" min="0.001" step="0.02" value={placement.spacing}
                      oninput={(event) => actions.updateDrawBundlePlacement(placement.id, { spacing: Number(event.currentTarget.value) })} />
                    <span>{template.fixedCount ? template.fixedCountValue : placement.count}</span>
                  </div>
                {/if}
              {/each}
            </fieldset>
            <label class="check"><input type="checkbox" checked={snapshot.showBackboneOverlay}
              onchange={(event) => actions.setDrawOption("showBackboneOverlay", event.currentTarget.checked)} />
              Backbone overlay</label>
            <label class="check"><input type="checkbox" checked={snapshot.showPreview}
              onchange={(event) => actions.setDrawOption("showPreview", event.currentTarget.checked)} />
              Preview</label>
            <label class="check"><input type="checkbox" checked={snapshot.keepPathAfterGenerate}
              onchange={(event) => actions.setDrawOption("keepPathAfterGenerate", event.currentTarget.checked)} />
              Keep path</label>
            <label class="check"><input type="checkbox" checked={snapshot.clickedPointsOnly}
              onchange={(event) => actions.setDrawOption("clickedPointsOnly", event.currentTarget.checked)} />
              Clicked points only</label>
            <label>Interval
              <input type="number" step="0.5" value={snapshot.intervalM}
                disabled={snapshot.clickedPointsOnly}
                onchange={(event) => actions.setDrawOption("intervalM", Number(event.currentTarget.value))} />
            </label>
            <label>Direction
              <select value={snapshot.directionMode}
                onchange={(event) => actions.setDrawOption("directionMode", Number(event.currentTarget.value))}>
                <option value="0">Auto</option><option value="1">Forward</option><option value="2">Reverse</option>
              </select>
            </label>
            <button class="secondary" type="button"
              onclick={() => actions.setDrawOption("directionMode", snapshot.directionMode === 1 ? 2 : 1)}>
              Flip Direction
            </button>
            <label>Plane Z
              <input type="number" step="0.1" value={snapshot.drawPlaneZ}
                onchange={(event) => actions.setDrawOption("drawPlaneZ", Number(event.currentTarget.value))} />
            </label>
            <p class="hint">LMB: add point / RMB: undo point / Enter: generate / Esc: cancel</p>
          </div>
          <SelectionInspector {actions} {snapshot} />
          <Settings {actions} {snapshot} />
          <Templates {actions} {snapshot} />
        {:else}
          <RoadPanel {actions} {snapshot} />
        {/if}
      </div>
    {/if}
  </section>

  <footer class="bottom-region">
    <aside>
      <span>parts <strong>{snapshot.parts.length}</strong></span>
      <span>poles <strong>{snapshot.poles.length}</strong></span>
      <button class="metric metric-button" type="button" aria-label="pipeline timing details">
        pipeline
        <strong>{formatMs(snapshot.generationMs)}</strong>
        <span class="metric-tooltip" role="tooltip">
          <strong>core pipeline</strong>
          {#if snapshot.generationTiming}
            <span class="metric-grid">
              {#each generationTimingRows as [label, key]}
                <span>{label}</span>
                <strong>{formatMs(snapshot.generationTiming[key])}</strong>
              {/each}
            </span>
            <span class="metric-line">
              <span>wasm call</span>
              <strong>{formatMs(snapshot.generationCallMs)}</strong>
            </span>
            <span class="metric-line">
              <span>js/wasm overhead</span>
              <strong>{formatMs(overheadMs(snapshot.generationCallMs, snapshot.generationMs))}</strong>
            </span>
          {:else}
            <em>no generation yet</em>
          {/if}
        </span>
      </button>
      <button class="metric metric-button" type="button" aria-label="scene update timing details">
        scene update
        <strong>{formatMs(snapshot.sceneUpdateMs)}</strong>
        <span class="metric-tooltip" role="tooltip">
          <strong>viewer / wasm</strong>
          <span class="metric-line">
            <span>scene read + store</span>
            <strong>{formatMs(snapshot.sceneUpdateMs)}</strong>
          </span>
          <span class="metric-line">
            <span>generate action total</span>
            <strong>{formatMs(snapshot.viewerUpdateMs)}</strong>
          </span>
          <span class="metric-line">
            <span>core pipeline</span>
            <strong>{formatMs(snapshot.generationMs)}</strong>
          </span>
        </span>
      </button>
      <span>
        interaction frames
        <strong>
          {snapshot.lastInteractionFrames === null
            ? "—"
            : `${snapshot.lastInteractionFrames.longFrameCount}/${snapshot.lastInteractionFrames.sampleCount} long · max ${snapshot.lastInteractionFrames.maxFrameMs.toFixed(1)} ms`}
        </strong>
      </span>
      <button class="metric metric-button build" type="button" aria-label="build identity details">
        build
        <strong>{buildInfo.commit}</strong>
        <span class="metric-tooltip" role="tooltip">
          <strong>deployed build</strong>
          <span class="metric-line">
            <span>commit</span>
            <strong>{buildInfo.commit}</strong>
          </span>
          <span class="metric-line">
            <span>built</span>
            <strong>{formatBuildTime(buildInfo.builtAt)}</strong>
          </span>
          <span class="metric-line">
            <span>web</span>
            <strong>v{buildInfo.packageVersion}</strong>
          </span>
        </span>
      </button>
    </aside>

    {#if snapshot.error}
      <div class="error" role="alert">{snapshot.error}</div>
    {/if}

    {#if snapshot.logs.length > 0}
      <div class="last-log">{snapshot.logs.at(-1)}</div>
    {/if}
  </footer>

  {#if snapshot.interaction}
    <div class="interaction">
      editing {snapshot.interaction.param} · Esc to cancel
    </div>
  {/if}
</main>
