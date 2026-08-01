declare const __WIRE_BUILD_INFO__: {
  commit: string;
  builtAt: string;
  packageVersion: string;
};

export const buildInfo =
  typeof __WIRE_BUILD_INFO__ === "undefined"
    ? {
        commit: "dev",
        builtAt: "dev",
        packageVersion: "0.2.0"
      }
    : __WIRE_BUILD_INFO__;

export interface WasmBuildIdentity {
  commit: string;
  version: string;
}

export function buildIdentitiesMatch(
  web: Pick<typeof buildInfo, "commit" | "packageVersion">,
  wasm: WasmBuildIdentity
): boolean {
  if (web.commit === "dev") return true;
  return web.commit === wasm.commit && web.packageVersion === wasm.version;
}
