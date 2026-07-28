import { existsSync, readFileSync } from "node:fs";
import { resolve } from "node:path";

export type BackboneEntry = "wasm_adapter" | "viewer_action";

function semanticsPath(): string {
  const candidates = [
    resolve(process.cwd(), "../docs/wire/backbone_operation_semantics.md"),
    resolve(process.cwd(), "docs/wire/backbone_operation_semantics.md")
  ];
  const found = candidates.find((candidate) => existsSync(candidate));
  if (found === undefined) {
    throw new Error("backbone operation semantics document is missing");
  }
  return found;
}

export function requiredBackboneEntryCells(entry: BackboneEntry): string[] {
  const lines = readFileSync(semanticsPath(), "utf8").split(/\r?\n/);
  const heading = lines.findIndex((line) => line.trim() === "## 入口境界");
  if (heading < 0) {
    throw new Error("backbone entry coverage table is missing");
  }
  const required: string[] = [];
  for (const line of lines.slice(heading + 1)) {
    if (line.startsWith("## ")) break;
    if (!line.trim().startsWith("| BOS:")) continue;
    const cells = line.trim().replace(/^\||\|$/g, "").split("|").map((cell) => cell.trim());
    if (cells.length !== 2) {
      throw new Error(`malformed backbone entry coverage row: ${line}`);
    }
    if (cells[1].includes(`\`${entry}\``)) required.push(cells[0]);
  }
  if (required.length === 0) {
    throw new Error(`backbone entry coverage has no required cells for ${entry}`);
  }
  return required.sort();
}

export function missingBackboneEntryCells(entry: BackboneEntry, covered: Iterable<string>): string[] {
  const observed = new Set(covered);
  return requiredBackboneEntryCells(entry).filter((cell) => !observed.has(cell));
}