export function drawIssueText(issue: string): string {
  const detail = issue
    .replace(/^backbone (?:validation|unsupported|internal):\s*/i, "")
    .replace(/^road (?:validation|unsupported|internal):\s*/i, "")
    .trim();
  if (detail.length === 0) return "This interval cannot be generated.";
  return detail[0].toUpperCase() + detail.slice(1);
}
