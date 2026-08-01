import { mount } from "svelte";
import App from "./App.svelte";
import { ViewerActions } from "./actions/viewer";
import { WireBridge } from "./bridge/wire";
import { startConsoleLogging } from "./consoleLog";
import { loadDefaultModelBootstrap } from "./render/modelAssets";
import { WireScene } from "./render/scene";
import { ViewerStore } from "./store/viewer";
import {
  IndexedDbWorkspaceStorage,
  WORKSPACE_CACHE_KEY,
  WorkspaceCache
} from "./store/workspace";
import "./style.css";

const target = document.getElementById("app");
if (target === null) {
  throw new Error("app mount point is missing");
}
const mountTarget = target;

async function main(): Promise<void> {
  const store = new ViewerStore();
  const stopConsoleLogging = startConsoleLogging(store);
  const modelBootstrap = await loadDefaultModelBootstrap();
  const bridge = await WireBridge.create();
  const modelBootstrapResult = bridge.configureModelAssemblies(modelBootstrap);
  if (!modelBootstrapResult.ok) {
    throw new Error(`Model bootstrap failed: ${modelBootstrapResult.error}`);
  }
  const workspaceStorage = new IndexedDbWorkspaceStorage();
  try {
    const legacyWorkspace = window.localStorage.getItem(WORKSPACE_CACHE_KEY);
    if (legacyWorkspace !== null) {
      if (await workspaceStorage.get(WORKSPACE_CACHE_KEY) === null) {
        await workspaceStorage.set(WORKSPACE_CACHE_KEY, legacyWorkspace);
      }
      window.localStorage.removeItem(WORKSPACE_CACHE_KEY);
    }
  } catch {
    // Keep legacy data when migration cannot complete. IndexedDB remains the active store.
  }
  const actions = new ViewerActions(
    bridge,
    store,
    new WorkspaceCache(workspaceStorage)
  );
  actions.initialize();
  await actions.restoreWorkspace();
  const scene = new WireScene(
    store,
    (point, pick) => actions.addViewportPoint(point, pick),
    (point, pick) => actions.previewViewportPoint(point, pick),
    () => actions.cancelDrawSession(),
    (deltaMs) => actions.recordFrame(deltaMs),
    (stats) => actions.recordSceneContentSync(stats),
    (handleIndex, point) => actions.previewRoadEditHandle(handleIndex, point),
    () => actions.commitRoadEditHandle()
  );

  mount(App, {
    target: mountTarget,
    props: {
      actions,
      store,
      mountScene: (host: HTMLElement) => scene.mount(host)
    }
  });

  window.addEventListener("beforeunload", () => {
    stopConsoleLogging();
    actions.dispose();
    scene.dispose();
    bridge.dispose();
  });
}

void main();
