import type { WireBridge } from "../bridge/wire";
import type { ViewerStore } from "../store/viewer";

export class ViewerActions {
  constructor(
    private readonly bridge: WireBridge,
    private readonly store: ViewerStore
  ) {}

  generateSample(): void {
    const result = this.bridge.generate(
      new Float64Array([0, 0, 0, 16, 2, 0, 32, 0, 0])
    );
    if (!result.ok) {
      this.store.setError(result.error);
      return;
    }

    const scene = this.bridge.scene();
    this.store.replace({
      parts: scene.parts,
      poles: scene.poles,
      error: "",
      generationMs: result.totalMs
    });
  }
}
