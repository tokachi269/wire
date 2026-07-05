import { mount } from "svelte";
import App from "./App.svelte";
import { ViewerActions } from "./actions/viewer";
import { WireBridge } from "./bridge/wire";
import { WireScene } from "./render/scene";
import { ViewerStore } from "./store/viewer";
import "./style.css";

const target = document.getElementById("app");
if (target === null) {
  throw new Error("app mount point is missing");
}

const store = new ViewerStore();
const bridge = await WireBridge.create();
const actions = new ViewerActions(bridge, store);
actions.initialize();
const scene = new WireScene(
  store,
  (point) => actions.addPathPoint(point),
  () => actions.undoPathPoint(),
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
  scene.dispose();
  bridge.dispose();
});
