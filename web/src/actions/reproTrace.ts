import type { EditResult, PathPickInfo } from "../model";
import type { PathPointSpec, ViewerSnapshot, WorldPoint } from "../store/viewer";

const PART_KIND_NAMES = ["EdgeBody", "NodePatch", "Lead", "Jumper"];

function formatNumber(value: number): string {
  return Number.isFinite(value) ? value.toFixed(3) : "non-finite";
}

function formatPoint(point: WorldPoint): string {
  return `(${point.map(formatNumber).join(",")})`;
}

function partKindName(kind: number): string {
  return PART_KIND_NAMES[kind] ?? `unknown-${kind}`;
}

function countParts(snapshot: ViewerSnapshot, nodeId?: string): string {
  const counts = new Map<string, number>();
  for (const part of snapshot.parts) {
    if (nodeId !== undefined && part.info.sourceNodeId !== nodeId) continue;
    const name = partKindName(part.info.kind);
    counts.set(name, (counts.get(name) ?? 0) + 1);
  }
  return [...counts.entries()]
    .sort(([a], [b]) => a.localeCompare(b))
    .map(([name, count]) => `${name}=${count}`)
    .join(",") || "none";
}

function sceneDigest(snapshot: ViewerSnapshot, anchorNodeIds: readonly string[] = []): string[] {
  const lines = [
    `scene poles=${snapshot.poles.length} ports=${snapshot.ports.length} spans=${snapshot.spans.length}` +
      ` nodes=${snapshot.supportNodes.length} edges=${snapshot.backboneEdges.length}` +
      ` parts=${snapshot.parts.length} visual={${countParts(snapshot)}}`
  ];
  for (const nodeId of [...new Set(anchorNodeIds)]) {
    const node = snapshot.supportNodes.find((candidate) => candidate.id === nodeId);
    const incidentEdges = snapshot.backboneEdges.filter(
      (edge) => edge.nodeAId === nodeId || edge.nodeBId === nodeId
    ).length;
    const position = node === undefined
      ? "not-in-scene"
      : `kind=${node.kind} pole=${node.poleId} at=(${formatNumber(node.x)},${formatNumber(node.y)},${formatNumber(node.z)})`;
    lines.push(
      `anchor node=${nodeId} ${position} incident-edges=${incidentEdges}` +
        ` visual={${countParts(snapshot, nodeId)}}`
    );
  }
  return lines;
}

function formatPick(pick: PathPickInfo): string {
  return `hit-kind=${pick.hitKind} hit-id=${pick.hitId}` +
    ` hit=(${formatNumber(pick.hitX)},${formatNumber(pick.hitY)},${formatNumber(pick.hitZ)})`;
}

function formatSettings(snapshot: ViewerSnapshot): string {
  const { geometry, layout, visual } = snapshot;
  return "settings " + [
    `geometry(curveSamples=${geometry.curveSamples},sagEnabled=${geometry.sagEnabled},sagFactor=${formatNumber(geometry.sagFactor)},poleClearance=${formatNumber(geometry.poleClearance)})`,
    `layout(angleCorrectionEnabled=${layout.angleCorrectionEnabled},cornerThresholdDeg=${formatNumber(layout.cornerThresholdDeg)},minSideScale=${formatNumber(layout.minSideScale)},maxSideScale=${formatNumber(layout.maxSideScale)})`,
    `visual(supports=${visual.enableSupportStructures},insulators=${visual.enableInsulators})`
  ].join(" ");
}

function formatGenerationRequest(snapshot: ViewerSnapshot, points: readonly WorldPoint[]): string {
  const bundleCounts = snapshot.selectedDrawBundleTemplateIds.map((id) => {
    const bundle = snapshot.bundleTemplates.find((candidate) => candidate.id === id);
    const cable = bundle === undefined
      ? undefined
      : snapshot.cableTemplates.find((candidate) => candidate.id === bundle.cableTemplateId);
    const count = snapshot.drawBundleCounts[id] ?? bundle?.defaultCount ?? "default";
    return bundle === undefined
      ? `${id}:count=${count}`
      : `${id}:count=${count},kind=${bundle.kind},branch=${bundle.branchPolicy},continuity=${bundle.continuityPolicy}` +
        `,cable=${bundle.cableTemplateId}${cable === undefined ? "" : `/sag=${formatNumber(cable.sagFactor)}`}`;
  }).join(";");
  const anchors = snapshot.pathPointSpecs
    .map((spec, index) => spec === null ? null : `${index}:${spec.nodeId}/${spec.supportKind}`)
    .filter((value): value is string => value !== null)
    .join(",") || "none";
  return `generate points=${points.map(formatPoint).join("->")}` +
    ` bundles=${bundleCounts} pole-template=${snapshot.selectedPoleTemplateId ?? "none"}` +
    ` interval=${formatNumber(snapshot.clickedPointsOnly ? 0 : snapshot.intervalM)}` +
    ` direction=${snapshot.directionMode} max-tilt=${formatNumber(snapshot.maxTiltDeg)}` +
    ` anchors=${anchors}`;
}

export class ReproTrace {
  private readonly entries: string[] = [];

  recordPathPoint(
    requested: WorldPoint,
    resolved: WorldPoint,
    pick: PathPickInfo | undefined,
    spec: PathPointSpec | null
  ): void {
    this.entries.push(
      `path-point requested=${formatPoint(requested)} resolved=${formatPoint(resolved)}` +
        (pick === undefined ? "" : ` ${formatPick(pick)}`) +
        (spec === null ? "" : ` anchor=${spec.nodeId}/${spec.supportKind}`)
    );
  }

  recordPathEdit(kind: "undo-path-point" | "clear-path"): void {
    this.entries.push(kind);
  }

  recordGeneration(
    before: ViewerSnapshot,
    points: readonly WorldPoint[],
    result: EditResult,
    after?: ViewerSnapshot
  ): void {
    const request = formatGenerationRequest(before, points);
    if (!result.ok) {
      this.entries.push(`${request} result=failed error=${result.error}`);
      return;
    }
    this.entries.push(
      `${request} result=ok poles=${result.generatedPoleCount} spans=${result.generatedSpanCount}` +
        ` total-ms=${formatNumber(result.totalMs)}`
    );
    if (after !== undefined) {
      this.entries.push(...sceneDigest(
        after,
        before.pathPointSpecs.flatMap((spec) => spec === null ? [] : [spec.nodeId])
      ));
    }
  }

  recordOperation(label: string, snapshot: ViewerSnapshot): void {
    this.entries.push(`operation ${label}`);
    this.entries.push(...sceneDigest(snapshot));
  }

  recordFailure(label: string, error: string): void {
    this.entries.push(`operation ${label} result=failed error=${error}`);
  }

  toText(snapshot: ViewerSnapshot): string {
    const body = this.entries.map((entry, index) =>
      `${String(index + 1).padStart(4, "0")} ${entry}`
    );
    return [
      "WIRE REPRO TRACE v1",
      "Contains operation order and compact visual-part coverage; it omits curve samples and UI logs.",
      formatSettings(snapshot),
      ...body,
      "FINAL " + sceneDigest(snapshot).join(" | ")
    ].join("\n") + "\n";
  }
}