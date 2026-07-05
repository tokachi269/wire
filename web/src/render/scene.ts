import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { ViewerSnapshot, ViewerStore } from "../store/viewer";
import type { WorldPoint } from "../store/viewer";

function colorFromRgba(rgba: number): { color: THREE.Color; opacity: number } {
  const red = (rgba >>> 24) & 0xff;
  const green = (rgba >>> 16) & 0xff;
  const blue = (rgba >>> 8) & 0xff;
  const alpha = rgba & 0xff;
  return {
    color: new THREE.Color(red / 255, green / 255, blue / 255),
    opacity: alpha / 255
  };
}

export class WireScene {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(48, 1, 0.05, 2000);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true });
  private readonly controls: OrbitControls;
  private readonly content = new THREE.Group();
  private readonly guide = new THREE.Group();
  private readonly unsubscribe: () => void;
  private frame = 0;
  private resizeObserver: ResizeObserver | null = null;
  private snapshot: ViewerSnapshot | null = null;
  private lastFrameTime: number | null = null;

  constructor(
    private readonly store: ViewerStore,
    private readonly onGroundClick: (point: WorldPoint) => void,
    private readonly onFrame: (deltaMs: number) => void
  ) {
    this.scene.background = new THREE.Color(0xdde7e7);
    this.scene.add(this.content);
    this.scene.add(this.guide);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x6f776f, 2.1));

    const grid = new THREE.GridHelper(80, 40, 0x7c8882, 0xbac3be);
    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);

    this.camera.up.set(0, 0, 1);
    this.camera.position.set(24, -30, 20);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(16, 0, 5);
    this.controls.enableDamping = true;

    this.unsubscribe = this.store.value.subscribe((snapshot) => this.rebuild(snapshot));
  }

  mount(host: HTMLElement): void {
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    host.appendChild(this.renderer.domElement);
    let pointerDown: { x: number; y: number } | null = null;
    this.renderer.domElement.addEventListener("pointerdown", (event) => {
      pointerDown = { x: event.clientX, y: event.clientY };
    });
    this.renderer.domElement.addEventListener("pointerup", (event) => {
      if (
        event.button !== 0 ||
        pointerDown === null ||
        Math.hypot(event.clientX - pointerDown.x, event.clientY - pointerDown.y) > 4
      ) {
        pointerDown = null;
        return;
      }
      pointerDown = null;
      const bounds = this.renderer.domElement.getBoundingClientRect();
      const pointer = new THREE.Vector2(
        ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
        -((event.clientY - bounds.top) / bounds.height) * 2 + 1
      );
      const ray = new THREE.Raycaster();
      ray.setFromCamera(pointer, this.camera);
      const hit = new THREE.Vector3();
      const planeZ = this.snapshot?.drawPlaneZ ?? 0;
      if (
        ray.ray.intersectPlane(
          new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ),
          hit
        )
      ) {
        this.onGroundClick([hit.x, hit.y, hit.z]);
      }
    });
    this.resizeObserver = new ResizeObserver(() => this.resize(host));
    this.resizeObserver.observe(host);
    this.resize(host);
    this.frame = requestAnimationFrame(this.animate);
  }

  dispose(): void {
    cancelAnimationFrame(this.frame);
    this.resizeObserver?.disconnect();
    this.unsubscribe();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private resize(host: HTMLElement): void {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  private animate = (time: number): void => {
    if (this.lastFrameTime !== null) {
      this.onFrame(time - this.lastFrameTime);
    }
    this.lastFrameTime = time;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.frame = requestAnimationFrame(this.animate);
  };

  private rebuild(snapshot: ViewerSnapshot): void {
    this.snapshot = snapshot;
    this.camera.fov = snapshot.cameraFov;
    this.camera.updateProjectionMatrix();
    this.content.visible = snapshot.showBackboneOverlay;
    this.guide.visible = snapshot.showPreview;
    this.disposeGroup(this.content);
    this.disposeGroup(this.guide);

    for (const part of snapshot.parts) {
      const positions = new Float32Array(part.samples.length);
      positions.set(part.samples);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      const appearance = colorFromRgba(part.info.colorRgba);
      const material = new THREE.LineBasicMaterial({
        color: appearance.color,
        opacity: appearance.opacity,
        transparent: appearance.opacity < 1,
        linewidth: Math.max(1, part.info.wireRadius * 100)
      });
      this.content.add(new THREE.Line(geometry, material));
    }

    for (const pole of snapshot.poles) {
      const geometry = new THREE.CylinderGeometry(0.15, 0.22, pole.height, 10);
      geometry.rotateX(Math.PI / 2);
      geometry.translate(0, 0, pole.height / 2);
      const material = new THREE.MeshStandardMaterial({
        color: 0x434b48,
        roughness: 0.88,
        wireframe: !snapshot.solidSupportRender
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(pole.positionX, pole.positionY, pole.positionZ);
      mesh.rotation.set(
        THREE.MathUtils.degToRad(pole.rotationX),
        THREE.MathUtils.degToRad(pole.rotationY),
        THREE.MathUtils.degToRad(pole.rotationZ)
      );
      mesh.scale.set(pole.scaleX, pole.scaleY, pole.scaleZ);
      this.content.add(mesh);
    }

    if (snapshot.pathPoints.length > 0) {
      const guidePoints = snapshot.pathPoints.map(
        (point) => new THREE.Vector3(point[0], point[1], point[2] + 0.03)
      );
      const guideGeometry = new THREE.BufferGeometry().setFromPoints(guidePoints);
      this.guide.add(
        new THREE.Line(
          guideGeometry,
          new THREE.LineBasicMaterial({ color: 0xd0ed35 })
        )
      );
      for (const point of guidePoints) {
        const marker = new THREE.Mesh(
          new THREE.SphereGeometry(0.18, 10, 8),
          new THREE.MeshBasicMaterial({ color: 0xd0ed35 })
        );
        marker.position.copy(point);
        this.guide.add(marker);
      }
    }
  }

  private disposeGroup(group: THREE.Group): void {
    for (const child of [...group.children]) {
      child.traverse((object) => {
        if (!(object instanceof THREE.Mesh || object instanceof THREE.Line)) {
          return;
        }
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        for (const material of materials) {
          material.dispose();
        }
      });
      group.remove(child);
    }
  }
}
