import type { ViewerActionContext } from "./context";
import type { BundleTemplateInfo, CableTemplateInfo, PoleTemplateInfo } from "../model";

export class TemplateActions {
  constructor(private readonly ctx: ViewerActionContext) {}

  selectBundleTemplate(id: number): void {
    this.ctx.store.update((current) => ({
      ...current,
      selectedBundleTemplateId: id
    }));
  }

  selectCableTemplate(id: number): void {
    this.ctx.store.update((current) => ({ ...current, selectedCableTemplateId: id }));
  }

  selectPoleTemplate(id: number): void {
    this.ctx.store.update((current) => ({ ...current, selectedPoleTemplateId: id }));
  }

  resetSpanReferenceLengths(): void {
    this.ctx.finishOperation(
      this.ctx.bridge.resetSpanReferenceLengths(),
      "span reference lengths reset"
    );
  }

  previewCableTemplate<K extends keyof CableTemplateInfo>(
    param: K,
    value: CableTemplateInfo[K]
  ): void {
    const selected = this.ctx.selectedCableTemplate();
    if (selected === null) {
      this.ctx.store.setError("cable template is not selected");
      return;
    }
    this.ctx.previewSetting(
      `cable.${String(param)}`,
      String(param),
      value,
      33,
      () => selected[param],
      (current, next) => ({
        ...current,
        cableTemplates: current.cableTemplates.map((template) =>
          template.id === selected.id ? { ...template, [param]: next } : template
        )
      }),
      () => {
        const current = this.ctx.selectedCableTemplate();
        return current === null
          ? { ok: false, error: "cable template is not selected" }
          : this.ctx.bridge.updateCableTemplate(current, this.ctx.preferredSpanIds());
      }
    );
  }

  commitCableTemplate(template: CableTemplateInfo): void {
    if (this.ctx.consumeCancelledCommit()) return;
    this.ctx.finishInteractionBeforeCommit();
    this.ctx.store.update((current) => ({
      ...current,
      cableTemplates: current.cableTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.ctx.bridge.updateCableTemplate(template, this.ctx.preferredSpanIds());
    this.ctx.finishTemplateOperation(result, `cable template ${template.id} updated`);
  }

  commitBundleTemplate(template: BundleTemplateInfo): void {
    if (this.ctx.consumeCancelledCommit()) return;
    this.ctx.finishInteractionBeforeCommit();
    this.ctx.store.update((current) => ({
      ...current,
      bundleTemplates: current.bundleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.ctx.bridge.updateBundleTemplate(template);
    this.ctx.finishTemplateOperation(result, `bundle template ${template.id} updated`);
  }

  previewBundleTemplate(
    template: BundleTemplateInfo,
    controlId: string,
    param: string,
    startValue: number
  ): void {
    const before = this.ctx.readSnapshot();
    if (before.interaction === null) {
      const original = before.bundleTemplates.find(
        (candidate) => candidate.id === template.id
      );
      if (original === undefined) {
        this.ctx.store.setError("bundle template is not selected");
        return;
      }
      this.ctx.store.update((current) => ({
        ...current,
        interaction: { controlId, param, startValue }
      }));
      this.ctx.interactionFrames = [];
      this.ctx.interactionActive = true;
      this.ctx.activeCancel = () => {
        this.ctx.store.update((current) => ({
          ...current,
          bundleTemplates: current.bundleTemplates.map((candidate) =>
            candidate.id === original.id ? original : candidate
          ),
          interaction: null
        }));
        const rollback = this.ctx.bridge.updateBundleTemplate(original);
        if (rollback.ok) this.ctx.refreshScene();
        else this.ctx.store.setError(rollback.error);
      };
    }
    this.ctx.store.update((current) => ({
      ...current,
      bundleTemplates: current.bundleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    this.ctx.clearPendingPreview();
    this.ctx.pendingPreview = setTimeout(() => {
      this.ctx.pendingPreview = null;
      const current = this.ctx.selectedBundleTemplate();
      const result =
        current === null
          ? { ok: false, error: "bundle template is not selected" }
          : this.ctx.bridge.updateBundleTemplate(current);
      if (!result.ok) {
        const error = result.error;
        this.ctx.cancel();
        this.ctx.store.setError(error);
        return;
      }
      this.ctx.refreshScene();
    }, 33);
  }

  applyRelatedPoleType(bundleTemplateId: number): void {
    this.ctx.finishTemplateOperation(
      this.ctx.bridge.applyRelatedPoleType(bundleTemplateId),
      `bundle ${bundleTemplateId} related pole type applied`
    );
  }

  previewPoleDefaultHeight(value: number): void {
    const selected = this.ctx.selectedPoleTemplate();
    if (selected === null) {
      this.ctx.store.setError("pole template is not selected");
      return;
    }
    this.ctx.previewSetting(
      "pole.defaultHeight",
      "defaultHeight",
      value,
      67,
      () => selected.defaultHeight,
      (current, next) => ({
        ...current,
        poleTemplates: current.poleTemplates.map((template) =>
          template.id === selected.id ? { ...template, defaultHeight: next } : template
        )
      }),
      () => {
        const current = this.ctx.selectedPoleTemplate();
        return current === null
          ? { ok: false, error: "pole template is not selected" }
          : this.ctx.bridge.updatePoleTemplate(current);
      }
    );
  }

  previewPoleTemplate(
    template: PoleTemplateInfo,
    controlId: string,
    param: string,
    startValue: number
  ): void {
    const before = this.ctx.readSnapshot();
    if (before.interaction === null) {
      const original = before.poleTemplates.find(
        (candidate) => candidate.id === template.id
      );
      if (original === undefined) {
        this.ctx.store.setError("pole template is not selected");
        return;
      }
      this.ctx.store.update((current) => ({
        ...current,
        interaction: { controlId, param, startValue }
      }));
      this.ctx.interactionFrames = [];
      this.ctx.interactionActive = true;
      this.ctx.activeCancel = () => {
        this.ctx.store.update((current) => ({
          ...current,
          poleTemplates: current.poleTemplates.map((candidate) =>
            candidate.id === original.id ? original : candidate
          ),
          interaction: null
        }));
        const rollback = this.ctx.bridge.updatePoleTemplate(original);
        if (rollback.ok) this.ctx.refreshScene();
        else this.ctx.store.setError(rollback.error);
      };
    }
    this.ctx.store.update((current) => ({
      ...current,
      poleTemplates: current.poleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    this.ctx.clearPendingPreview();
    this.ctx.pendingPreview = setTimeout(() => {
      this.ctx.pendingPreview = null;
      const current = this.ctx.selectedPoleTemplate();
      const result =
        current === null
          ? { ok: false, error: "pole template is not selected" }
          : this.ctx.bridge.updatePoleTemplate(current);
      if (!result.ok) {
        const error = result.error;
        this.ctx.cancel();
        this.ctx.store.setError(error);
        return;
      }
      this.ctx.refreshScene();
    }, 67);
  }

  commitPoleTemplate(template: PoleTemplateInfo): void {
    if (this.ctx.consumeCancelledCommit()) return;
    this.ctx.finishInteractionBeforeCommit();
    this.ctx.store.update((current) => ({
      ...current,
      poleTemplates: current.poleTemplates.map((candidate) =>
        candidate.id === template.id ? template : candidate
      )
    }));
    const result = this.ctx.bridge.updatePoleTemplate(template);
    this.ctx.finishTemplateOperation(result, `pole template ${template.id} updated`);
  }
}
