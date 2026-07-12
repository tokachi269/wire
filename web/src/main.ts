import { mount } from "svelte";
import App from "./App.svelte";
import { ViewerActions } from "./actions/viewer";
import { WireBridge } from "./bridge/wire";
import { WireScene } from "./render/scene";
import { ViewerStore } from "./store/viewer";
import { WorkspaceCache } from "./workspaceCache";
import "./style.css";

const target = document.getElementById("app");
if (target === null) {
  throw new Error("app mount point is missing");
}

const store = new ViewerStore();
const bridge = await WireBridge.create();
const actions = new ViewerActions(
  bridge,
  store,
  new WorkspaceCache(window.localStorage)
);
actions.initialize();
const scene = new WireScene(
  store,
  (point, pick) => actions.addPathPoint(point, pick),
  () => actions.undoPathPointOrClearSelection(),
  (deltaMs) => actions.recordFrame(deltaMs)
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
