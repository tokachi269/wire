import { describe, expect, it } from "vitest";
import * as THREE from "three";
import { setPoleRotation } from "../src/render/scene";

function rotateCoreXyz(value: THREE.Vector3, xDeg: number, yDeg: number, zDeg: number): THREE.Vector3 {
  const radians = [xDeg, yDeg, zDeg].map(THREE.MathUtils.degToRad);
  const [x, y, z] = radians;
  return value.clone()
    .applyAxisAngle(new THREE.Vector3(1, 0, 0), x)
    .applyAxisAngle(new THREE.Vector3(0, 1, 0), y)
    .applyAxisAngle(new THREE.Vector3(0, 0, 1), z);
}

describe("pole rendering", () => {
  it("uses the same X then Y then Z tilt composition as core port placement", () => {
    const rotation = { x: 3.4379, y: 1.75989, z: -57.805 };
    const pole = new THREE.Object3D();
    setPoleRotation(pole, rotation.x, rotation.y, rotation.z);

    const renderedTop = new THREE.Vector3(0, 0, 10).applyEuler(pole.rotation);
    const coreTop = rotateCoreXyz(new THREE.Vector3(0, 0, 10), rotation.x, rotation.y, rotation.z);

    expect(renderedTop.distanceTo(coreTop)).toBeLessThan(1e-12);
  });
});
