import {
  loadWireModule,
  type EditResult,
  type PoleInfo,
  type VisualPartInfo,
  type WireStateHandle
} from "./wasm";

export interface VisualPartData {
  info: VisualPartInfo;
  samples: Float64Array;
}

export interface SceneData {
  parts: VisualPartData[];
  poles: PoleInfo[];
}

export class WireBridge {
  private constructor(private readonly state: WireStateHandle) {}

  static async create(): Promise<WireBridge> {
    const module = await loadWireModule();
    return new WireBridge(new module.WireState());
  }

  generate(
    points: Float64Array,
    bundleTemplateId = 0,
    intervalM = 0,
    poleTypeId = 1,
    count = 0
  ): EditResult {
    return this.state.generate(points, bundleTemplateId, intervalM, poleTypeId, count);
  }

  scene(): SceneData {
    const parts: VisualPartData[] = [];
    for (let index = 0; index < this.state.visualPartCount(); index += 1) {
      const info = this.state.visualPart(index);
      // The wasm view is scratch memory. Copy it before requesting another part.
      const samples = new Float64Array(this.state.visualPartSamples(index));
      parts.push({ info, samples });
    }

    const poles: PoleInfo[] = [];
    for (let index = 0; index < this.state.poleCount(); index += 1) {
      poles.push(this.state.pole(index));
    }
    return { parts, poles };
  }

  dispose(): void {
    this.state.delete();
  }
}
