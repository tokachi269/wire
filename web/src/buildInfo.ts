declare const __WIRE_BUILD_INFO__: {
  commit: string;
  wasmSourceHash: string;
  builtAt: string;
  packageVersion: string;
};

export const buildInfo =
  typeof __WIRE_BUILD_INFO__ === "undefined"
    ? {
        commit: "dev",
        wasmSourceHash: "dev",
        builtAt: "dev",
        packageVersion: "0.2.0"
      }
    : __WIRE_BUILD_INFO__;

export interface WasmBuildIdentity {
  sourceHash: string;
  version: string;
}

export async function loadRuntimeBuildInfo(): Promise<typeof buildInfo> {
  if (!import.meta.env.DEV || typeof window === "undefined") return buildInfo;
  try {
    const response = await fetch("/__wire_build_info", { cache: "no-store" });
    if (!response.ok) return buildInfo;
    return await response.json() as typeof buildInfo;
  } catch {
    return buildInfo;
  }
}

export function buildIdentitiesMatch(
  web: Pick<typeof buildInfo, "commit" | "wasmSourceHash" | "packageVersion">,
  wasm: WasmBuildIdentity
): boolean {
  if (web.commit === "dev") return true;
  return web.wasmSourceHash === wasm.sourceHash &&
    web.packageVersion === wasm.version;
}
