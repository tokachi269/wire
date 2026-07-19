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
        packageVersion: "0.1.0"
      }
    : __WIRE_BUILD_INFO__;
