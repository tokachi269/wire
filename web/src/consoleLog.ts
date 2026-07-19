import type { ViewerStore } from "./store/viewer";

export function startConsoleLogging(store: ViewerStore): () => void {
  let consumedLogs = 0;
  let previousError = "";
  return store.value.subscribe((snapshot) => {
    if (snapshot.logs.length < consumedLogs) consumedLogs = 0;
    for (const message of snapshot.logs.slice(consumedLogs)) {
      console.info(`[wire] ${message}`);
    }
    consumedLogs = snapshot.logs.length;

    if (snapshot.error.length > 0 && snapshot.error !== previousError) {
      console.error(`[wire] ${snapshot.error}`);
    }
    previousError = snapshot.error;
  });
}
