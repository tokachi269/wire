import { describe, expect, it, vi } from "vitest";
import { startConsoleLogging } from "../src/consoleLog";
import { ViewerStore } from "../src/store/viewer";

describe("console logging", () => {
  it("writes each operation and distinct error once", () => {
    const info = vi.spyOn(console, "info").mockImplementation(() => undefined);
    const error = vi.spyOn(console, "error").mockImplementation(() => undefined);
    const store = new ViewerStore();
    const stop = startConsoleLogging(store);

    store.update((current) => ({ ...current, logs: [...current.logs, "route generated"] }));
    store.update((current) => ({ ...current, cameraFov: 60 }));
    store.setError("load failed");
    store.setError("load failed");

    expect(info).toHaveBeenCalledOnce();
    expect(info).toHaveBeenCalledWith("[wire] route generated");
    expect(error).toHaveBeenCalledOnce();
    expect(error).toHaveBeenCalledWith("[wire] load failed");

    stop();
    info.mockRestore();
    error.mockRestore();
  });
});
