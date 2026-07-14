import { mount } from "svelte";
import App from "./App.svelte";
import { ViewerActions } from "./actions/viewer";
import { devBuild } from "./buildInfo";
import { WireBridge } from "./bridge/wire";
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

const store = new ViewerStore();
const bridge = await WireBridge.create();
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
  (point, pick) => actions.addPathPoint(point, pick),
  () => actions.undoPathPointOrClearSelection(),
  (deltaMs) => actions.recordFrame(deltaMs),
  devBuild ? (stats) => actions.recordSceneContentSync(stats) : undefined
);

mount(App, {
  target,
  props: {
    actions,
    store,
    mountScene: (host: HTMLElement) => scene.mount(host)
  }
});

window.addEventListener("beforeunload", () => {
  actions.dispose();
  scene.dispose();
  bridge.dispose();
});
