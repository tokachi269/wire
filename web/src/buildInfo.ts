declare const __WIRE_BUILD_INFO__: {
  commit: string;
  builtAt: string;
  packageVersion: string;
};
declare const __WIRE_DEV__: boolean;

export const buildInfo =
  typeof __WIRE_BUILD_INFO__ === "undefined"
    ? {
        commit: "dev",
        builtAt: "dev",
        packageVersion: "0.1.0"
      }
    : __WIRE_BUILD_INFO__;

export const devBuild = typeof __WIRE_DEV__ !== "undefined" && __WIRE_DEV__;
