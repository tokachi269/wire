import * as THREE from "three";
import type { ViewerSnapshot, ViewerStore } from "../store/viewer";
import type { WorldPoint } from "../store/viewer";

function colorFromRgba(rgba: number): { color: THREE.Color; opacity: number } {
  const red = (rgba >>> 24) & 0xff;
  const green = (rgba >>> 16) & 0xff;
  const blue = (rgba >>> 8) & 0xff;
  const alpha = rgba & 0xff;
  const source = new THREE.Color(red / 255, green / 255, blue / 255);
  const charcoal = new THREE.Color(42 / 255, 46 / 255, 52 / 255);
  return { color: charcoal.lerp(source, 0.22), opacity: Math.max(210, alpha) / 255 };
}

export class WireScene {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(48, 1, 0.05, 2000);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true });
  private readonly cameraTarget = new THREE.Vector3(16, 0, 5);
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
    this.scene.background = new THREE.Color(0xc8d6e4);
    this.scene.add(this.content);
    this.scene.add(this.guide);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x6f776f, 2.1));

    const grid = new THREE.GridHelper(80, 40, 0x587288, 0x72889a);
    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);

    this.camera.up.set(0, 0, 1);
    this.camera.position.set(24, -30, 20);
    this.camera.lookAt(this.cameraTarget);

    this.unsubscribe = this.store.value.subscribe((snapshot) => this.rebuild(snapshot));
  }

  mount(host: HTMLElement): void {
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.domElement.tabIndex = 0;
    host.appendChild(this.renderer.domElement);
    let pointerDown: {
      x: number;
      y: number;
      button: number;
      mode: "orbit" | "pan" | "dolly" | "draw";
    } | null = null;
    this.renderer.domElement.addEventListener("pointerdown", (event) => {
      this.renderer.domElement.focus();
      const mode =
        event.button === 1
          ? event.shiftKey
            ? "pan"
            : event.ctrlKey
              ? "dolly"
              : "orbit"
          : "draw";
      pointerDown = { x: event.clientX, y: event.clientY, button: event.button, mode };
      if (event.button === 1) {
        event.preventDefault();
        this.renderer.domElement.setPointerCapture(event.pointerId);
      }
    });
    this.renderer.domElement.addEventListener("pointermove", (event) => {
      if (pointerDown === null || pointerDown.button !== 1) return;
      const dx = event.clientX - pointerDown.x;
      const dy = event.clientY - pointerDown.y;
      pointerDown.x = event.clientX;
      pointerDown.y = event.clientY;
      if (pointerDown.mode === "orbit") this.orbit(dx, dy);
      if (pointerDown.mode === "pan") this.pan(dx, dy);
      if (pointerDown.mode === "dolly") this.dolly(dy * 0.01);
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
      if (ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ), hit)) {
        if (event.altKey) {
          this.cameraTarget.copy(hit);
          this.camera.lookAt(this.cameraTarget);
        } else {
          this.onGroundClick([hit.x, hit.y, hit.z]);
        }
      }
    });
    this.renderer.domElement.addEventListener("contextmenu", (event) => event.preventDefault());
    this.renderer.domElement.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.dolly(event.deltaY * 0.0012);
    }, { passive: false });
    this.resizeObserver = new ResizeObserver(() => this.resize(host));
    this.resizeObserver.observe(host);
    this.resize(host);
    this.frame = requestAnimationFrame(this.animate);
  }

  dispose(): void {
    cancelAnimationFrame(this.frame);
    this.resizeObserver?.disconnect();
    this.unsubscribe();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private resize(host: HTMLElement): void {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  private animate = (time: number): void => {
    if (this.lastFrameTime !== null) {
      this.onFrame(time - this.lastFrameTime);
    }
    this.lastFrameTime = time;
    this.renderer.render(this.scene, this.camera);
    this.frame = requestAnimationFrame(this.animate);
  };

  private orbit(dx: number, dy: number): void {
    const offset = this.camera.position.clone().sub(this.cameraTarget);
    offset.applyAxisAngle(this.camera.up, -dx * 0.006);
    const forward = offset.clone().negate().normalize();
    const right = forward.clone().cross(this.camera.up).normalize();
    const pitched = offset.clone().applyAxisAngle(right, -dy * 0.006);
    if (Math.abs(pitched.clone().normalize().dot(this.camera.up)) < 0.995) {
      offset.copy(pitched);
    }
    this.camera.position.copy(this.cameraTarget).add(offset);
    this.camera.lookAt(this.cameraTarget);
  }

  private pan(dx: number, dy: number): void {
    const distance = this.camera.position.distanceTo(this.cameraTarget);
    const forward = this.cameraTarget.clone().sub(this.camera.position).normalize();
    const right = forward.clone().cross(this.camera.up).normalize();
    const shift = right.multiplyScalar(-dx * distance * 0.002)
      .add(this.camera.up.clone().normalize().multiplyScalar(dy * distance * 0.002));
    this.camera.position.add(shift);
    this.cameraTarget.add(shift);
  }

  private dolly(amount: number): void {
    const offset = this.camera.position.clone().sub(this.cameraTarget);
    const distance = THREE.MathUtils.clamp(offset.length() * Math.exp(amount), 0.2, 2000);
    this.camera.position.copy(this.cameraTarget).add(offset.normalize().multiplyScalar(distance));
  }

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
          new THREE.LineBasicMaterial({ color: 0x5a9ab0 })
        )
      );
      for (const point of guidePoints) {
        const marker = new THREE.Mesh(
          new THREE.SphereGeometry(0.18, 10, 8),
          new THREE.MeshBasicMaterial({ color: 0x5a9ab0 })
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
