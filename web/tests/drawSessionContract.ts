import { expect } from "vitest";

export interface DrawContractDriver {
  committedCount(): number;
  hasAnchor(): boolean;
  previewState(): "none" | "valid" | "invalid";
  reason(): string;
  start(): void;
  previewValid(): void;
  previewInvalid(): void;
  click(): void;
  enter(): void;
  escape(): void;
  undo(): void;
}

export function verifySharedDrawContract(driver: DrawContractDriver): void {
  driver.start();
  expect(driver.hasAnchor()).toBe(true);

  driver.previewValid();
  expect(driver.previewState()).toBe("valid");
  driver.click();
  expect(driver.committedCount()).toBe(1);
  expect(driver.hasAnchor()).toBe(true);

  driver.previewInvalid();
  driver.enter();
  expect(driver.committedCount()).toBe(1);
  expect(driver.previewState()).toBe("invalid");
  expect(driver.reason()).not.toBe("");

  driver.previewValid();
  driver.enter();
  expect(driver.committedCount()).toBe(2);
  expect(driver.hasAnchor()).toBe(false);

  driver.start();
  driver.previewValid();
  driver.escape();
  driver.escape();
  expect(driver.committedCount()).toBe(2);
  expect(driver.hasAnchor()).toBe(false);
  expect(driver.previewState()).toBe("none");

  driver.undo();
  expect(driver.committedCount()).toBe(1);
}
