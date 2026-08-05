import { expect } from "vitest";
import type { CommitFailureCategory } from "../src/model";
import type { DrawActionResult } from "../src/store/viewer";

/**
 * What a draw tool has to report, whichever domain it belongs to. Road and wire
 * answer the same questions so the viewer never has to read an error string to
 * decide what happened.
 */
export interface DrawContractDriver {
  /** First click. Takes the anchor the next commit runs from. */
  anchor(): DrawActionResult;
  /** Pointer move over a target the core would accept. */
  hoverValid(): void;
  /** Second click on a target the core accepts. */
  commit(): DrawActionResult;
  /** A commit the core rejects, from the same anchor. */
  commitRejected(): DrawActionResult;
  /** Enter. */
  confirm(): DrawActionResult;
  /** Escape. */
  cancel(): DrawActionResult;

  hasAnchor(): boolean;
  committedCount(): number;
  lastFailure(): { category: CommitFailureCategory; reasonCode: string } | null;
}

export function verifySharedDrawContract(driver: DrawContractDriver): void {
  // Escape without a session is a distinguishable non-event, not a failure.
  const idle = driver.cancel();
  expect(idle.kind).toBe("ignored");
  expect(idle.kind === "ignored" && idle.reasonCode).not.toBe("");
  expect(driver.lastFailure()).toBeNull();

  expect(driver.anchor().kind).toBe("anchor-accepted");
  expect(driver.hasAnchor()).toBe(true);
  const before = driver.committedCount();

  // A rejected commit keeps the anchor, reports a category and a reason code,
  // and commits nothing.
  const rejected = driver.commitRejected();
  expect(rejected.kind).toBe("commit-rejected");
  expect(rejected.kind === "commit-rejected" && rejected.reasonCode).not.toBe("");
  expect(driver.committedCount()).toBe(before);
  expect(driver.hasAnchor()).toBe(true);
  const failure = driver.lastFailure();
  expect(failure).not.toBeNull();
  expect(failure!.reasonCode).not.toBe("");

  // Pointer movement is not an outcome. It must not overwrite what the last
  // commit reported.
  driver.hoverValid();
  expect(driver.lastFailure()).toEqual(failure);

  // The same anchor can still commit, so a rejection is retryable.
  expect(driver.commit().kind).toBe("commit-succeeded");
  expect(driver.committedCount()).toBe(before + 1);

  // Enter reports an outcome rather than a bare boolean, and Escape ends the
  // session explicitly.
  const confirmed = driver.confirm();
  expect(["commit-succeeded", "session-ended", "ignored"]).toContain(confirmed.kind);
  const ended = driver.cancel();
  expect(["session-ended", "ignored"]).toContain(ended.kind);
  expect(driver.hasAnchor()).toBe(false);
}
