import * as THREE from "three";
import type { PathPickInfo, SupportNodeInfo } from "../model";
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

const POLE_TOP_DIAMETER_M = 0.190;
const POLE_TAPER_RATIO = 75;
const POLE_RENDER_SIDES = 16;
const BACKBONE_DISPLAY_PLANE_Z = 0.0;

class SampledWireCurve extends THREE.Curve<THREE.Vector3> {
  private readonly lengths: number[] = [0];
  private readonly totalLength: number;

  constructor(private readonly points: THREE.Vector3[]) {
    super();
    for (let index = 1; index < points.length; index += 1) {
      this.lengths[index] = this.lengths[index - 1] + points[index].distanceTo(points[index - 1]);
    }
    this.totalLength = this.lengths[this.lengths.length - 1] ?? 0;
  }

  override getPoint(t: number, target = new THREE.Vector3()): THREE.Vector3 {
    if (this.points.length === 0) return target.set(0, 0, 0);
    if (this.points.length === 1 || this.totalLength <= 0) return target.copy(this.points[0]);

    const distance = THREE.MathUtils.clamp(t, 0, 1) * this.totalLength;
    let index = 1;
    while (index < this.lengths.length - 1 && this.lengths[index] < distance) {
      index += 1;
    }

    const start = this.points[index - 1];
    const end = this.points[index];
    const segmentLength = this.lengths[index] - this.lengths[index - 1];
    const segmentT = segmentLength > 0
      ? (distance - this.lengths[index - 1]) / segmentLength
      : 0;
    return target.copy(start).lerp(end, segmentT);
  }
}

export class WireScene {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(48, 1, 0.05, 2000);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true });
  private readonly cameraTarget = new THREE.Vector3(16, 0, 5);
  private readonly content = new THREE.Group();
  private readonly backbone = new THREE.Group();
  private readonly guide = new THREE.Group();
  private readonly unsubscribe: () => void;
  private frame = 0;
  private resizeObserver: ResizeObserver | null = null;
  private detachInput: (() => void) | null = null;
  private snapshot: ViewerSnapshot | null = null;
  private lastFrameTime: number | null = null;

  constructor(
    private readonly store: ViewerStore,
    private readonly onGroundClick: (point: WorldPoint, pick?: PathPickInfo) => void,
    private readonly onUndoPathPoint: () => void,
    private readonly onFrame: (deltaMs: number) => void
  ) {
    this.scene.background = new THREE.Color(0xc8d6e4);
    this.scene.add(this.backbone);
    this.scene.add(this.content);
    this.scene.add(this.guide);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x6f776f, 1.45));

    const sun = new THREE.DirectionalLight(0xfff5e4, 3.1);
    sun.position.set(-18, -24, 42);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 120;
    sun.shadow.camera.left = -42;
    sun.shadow.camera.right = 42;
    sun.shadow.camera.top = 42;
    sun.shadow.camera.bottom = -42;
    this.scene.add(sun);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(90, 90),
      new THREE.ShadowMaterial({ color: 0x45515a, opacity: 0.24 })
    );
    ground.position.z = -0.01;
    ground.receiveShadow = true;
    this.scene.add(ground);

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
    const activePointers = new Map<number, { x: number; y: number }>();
    let pointerDown: {
      x: number;
      y: number;
      startX: number;
      startY: number;
      button: number;
      mode: "orbit" | "pan" | "dolly";
      moved: boolean;
    } | null = null;
    let pinch: {
      distance: number;
      centerX: number;
      centerY: number;
    } | null = null;
    const canvas = this.renderer.domElement;
    canvas.style.touchAction = "none";
    const currentPinch = () => {
      const pointers = [...activePointers.values()];
      if (pointers.length < 2) return null;
      const [a, b] = pointers;
      return {
        distance: Math.max(1, Math.hypot(a.x - b.x, a.y - b.y)),
        centerX: (a.x + b.x) / 2,
        centerY: (a.y + b.y) / 2
      };
    };
    const onPointerDown = (event: PointerEvent) => {
      if (event.button !== 0 && event.button !== 1) return;
      this.renderer.domElement.focus();
      if (event.pointerType === "mouse" && event.button === 0) {
        this.addGroundPoint(event);
        return;
      }
      activePointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
      if (activePointers.size >= 2) {
        pinch = currentPinch();
        pointerDown = null;
        event.preventDefault();
        this.renderer.domElement.setPointerCapture(event.pointerId);
        return;
      }
      const mode = event.button === 1
        ? event.shiftKey ? "pan" : event.ctrlKey ? "dolly" : "orbit"
        : "orbit";
      pointerDown = {
        x: event.clientX,
        y: event.clientY,
        startX: event.clientX,
        startY: event.clientY,
        button: event.button,
        mode,
        moved: false
      };
      event.preventDefault();
      this.renderer.domElement.setPointerCapture(event.pointerId);
    };
    const onPointerMove = (event: PointerEvent) => {
      if (activePointers.has(event.pointerId)) {
        activePointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
      }
      if (activePointers.size >= 2) {
        const next = currentPinch();
        if (pinch !== null && next !== null) {
          this.dolly(-Math.log(next.distance / pinch.distance));
          this.pan(next.centerX - pinch.centerX, next.centerY - pinch.centerY);
        }
        pinch = next;
        event.preventDefault();
        return;
      }
      if (pointerDown === null) return;
      const dx = event.clientX - pointerDown.x;
      const dy = event.clientY - pointerDown.y;
      const totalDx = event.clientX - pointerDown.startX;
      const totalDy = event.clientY - pointerDown.startY;
      pointerDown.x = event.clientX;
      pointerDown.y = event.clientY;
      pointerDown.moved ||= Math.hypot(totalDx, totalDy) > 6;
      if (!pointerDown.moved) return;
      if (pointerDown.mode === "orbit") this.orbit(dx, dy);
      if (pointerDown.mode === "pan") this.pan(dx, dy);
      if (pointerDown.mode === "dolly") this.dolly(dy * 0.01);
      event.preventDefault();
    };
    const onPointerUp = (event: PointerEvent) => {
      const finished = pointerDown;
      activePointers.delete(event.pointerId);
      pinch = activePointers.size >= 2 ? currentPinch() : null;
      if (finished !== null && event.button === 0 && !finished.moved) {
        this.addGroundPoint(event);
      }
      pointerDown = null;
    };
    const onPointerCancel = () => {
      activePointers.clear();
      pinch = null;
      pointerDown = null;
    };
    const onContextMenu = (event: MouseEvent) => {
      event.preventDefault();
      this.onUndoPathPoint();
    };
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      this.dolly(event.deltaY * 0.0012);
    };
    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("pointercancel", onPointerCancel);
    canvas.addEventListener("contextmenu", onContextMenu);
    canvas.addEventListener("wheel", onWheel, { passive: false });
    this.detachInput = () => {
      canvas.removeEventListener("pointerdown", onPointerDown);
      canvas.removeEventListener("pointermove", onPointerMove);
      canvas.removeEventListener("pointerup", onPointerUp);
      canvas.removeEventListener("pointercancel", onPointerCancel);
      canvas.removeEventListener("contextmenu", onContextMenu);
      canvas.removeEventListener("wheel", onWheel);
    };
    this.resizeObserver = new ResizeObserver(() => this.resize(host));
    this.resizeObserver.observe(host);
    this.resize(host);
    this.frame = requestAnimationFrame(this.animate);
  }

  dispose(): void {
    cancelAnimationFrame(this.frame);
    this.resizeObserver?.disconnect();
    this.detachInput?.();
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

  private addGroundPoint(event: PointerEvent): void {
    const bounds = this.renderer.domElement.getBoundingClientRect();
    const pointer = new THREE.Vector2(
      ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
      -((event.clientY - bounds.top) / bounds.height) * 2 + 1
    );
    const ray = new THREE.Raycaster();
    ray.params.Line = { threshold: 0.35 };
    ray.setFromCamera(pointer, this.camera);
    const backboneHit = this.pickBackbonePoint(ray);
    if (backboneHit !== null) {
      this.onGroundClick(backboneHit.point, backboneHit.pick);
      return;
    }
    const hit = new THREE.Vector3();
    const planeZ = this.snapshot?.drawPlaneZ ?? 0;
    if (!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ), hit)) {
      return;
    }
    if (event.altKey) {
      this.cameraTarget.copy(hit);
      this.camera.lookAt(this.cameraTarget);
      return;
    }
    this.onGroundClick([hit.x, hit.y, hit.z]);
  }

  private pickBackbonePoint(ray: THREE.Raycaster): { point: WorldPoint; pick: PathPickInfo } | null {
    if (this.snapshot?.showBackboneOverlay !== true) return null;
    const hits = ray.intersectObjects(this.backbone.children, true)
      .filter((hit) => hit.object.userData.pickableBackbone);
    const hit = hits[0];
    if (hit === undefined) return null;
    const data = hit.object.userData;
    let point: WorldPoint = [hit.point.x, hit.point.y, BACKBONE_DISPLAY_PLANE_Z];
    const defaultPick = {
      hitKind: 0,
      hitId: "0",
      hitX: point[0],
      hitY: point[1],
      hitZ: point[2],
      hasSegmentEndpoints: false,
      segmentNodeAId: "0",
      segmentNodeBId: "0",
      segmentEndpointAX: 0,
      segmentEndpointAY: 0,
      segmentEndpointAZ: 0,
      segmentEndpointBX: 0,
      segmentEndpointBY: 0,
      segmentEndpointBZ: 0
    };
    if (data.pickKind === "node") {
      return {
        point,
        pick: {
          ...defaultPick,
          hitKind: 1,
          hitId: data.hitId ?? "0"
        }
      };
    }
    if (data.pickKind === "edge") {
      const endpointA = data.endpointA as WorldPoint;
      const endpointB = data.endpointB as WorldPoint;
      point = this.closestBackbonePointXY(point, endpointA, endpointB);
      return {
        point,
        pick: {
          ...defaultPick,
          hitKind: 2,
          hitX: point[0],
          hitY: point[1],
          hitZ: point[2],
          hasSegmentEndpoints: true,
          segmentNodeAId: data.nodeAId ?? "0",
          segmentNodeBId: data.nodeBId ?? "0",
          segmentEndpointAX: endpointA[0],
          segmentEndpointAY: endpointA[1],
          segmentEndpointAZ: endpointA[2],
          segmentEndpointBX: endpointB[0],
          segmentEndpointBY: endpointB[1],
          segmentEndpointBZ: endpointB[2]
        }
      };
    }
    return null;
  }

  private closestBackbonePointXY(point: WorldPoint, endpointA: WorldPoint, endpointB: WorldPoint): WorldPoint {
    const dx = endpointB[0] - endpointA[0];
    const dy = endpointB[1] - endpointA[1];
    const length2 = dx * dx + dy * dy;
    const t = length2 > 0
      ? THREE.MathUtils.clamp(
          ((point[0] - endpointA[0]) * dx + (point[1] - endpointA[1]) * dy) / length2,
          0,
          1
        )
      : 0;
    return [
      endpointA[0] + dx * t,
      endpointA[1] + dy * t,
      BACKBONE_DISPLAY_PLANE_Z
    ];
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
    this.content.visible = true;
    this.backbone.visible = snapshot.showBackboneOverlay;
    this.guide.visible = snapshot.showPreview;
    this.disposeGroup(this.backbone);
    this.disposeGroup(this.content);
    this.disposeGroup(this.guide);

    for (const part of snapshot.parts) {
      const points: THREE.Vector3[] = [];
      for (let index = 0; index + 2 < part.samples.length; index += 3) {
        points.push(new THREE.Vector3(
          part.samples[index],
          part.samples[index + 1],
          part.samples[index + 2]
        ));
      }
      if (points.length < 2) continue;

      const curve = new SampledWireCurve(points);
      const radius = THREE.MathUtils.clamp(part.info.wireRadius, 0.006, 0.08);
      const segments = Math.max(4, points.length - 1);
      const geometry = new THREE.TubeGeometry(curve, segments, radius, 10, false);
      const appearance = colorFromRgba(part.info.colorRgba);
      const material = new THREE.MeshStandardMaterial({
        color: appearance.color,
        metalness: 0.12,
        roughness: 0.58,
        opacity: appearance.opacity,
        transparent: appearance.opacity < 1
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.content.add(mesh);
    }

    this.buildBackboneOverlay(snapshot);

    for (const pole of snapshot.poles) {
      const topRadius = POLE_TOP_DIAMETER_M / 2;
      const groundRadius = (POLE_TOP_DIAMETER_M + pole.height / POLE_TAPER_RATIO) / 2;
      const geometry = new THREE.CylinderGeometry(topRadius, groundRadius, pole.height, POLE_RENDER_SIDES);
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
      mesh.castShadow = true;
      mesh.receiveShadow = true;
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

  private buildBackboneOverlay(snapshot: ViewerSnapshot): void {
    const nodeById = new Map<string, SupportNodeInfo>();
    for (const node of snapshot.supportNodes) {
      nodeById.set(node.id, node);
    }
    const lineMaterial = new THREE.LineBasicMaterial({
      color: 0x1f6f88,
      transparent: true,
      opacity: 0.9,
      depthTest: false,
      depthWrite: false
    });
    const nodeMaterial = new THREE.MeshBasicMaterial({
      color: 0xffc247,
      depthTest: false,
      depthWrite: false
    });
    for (const edge of snapshot.backboneEdges) {
      const nodeA = nodeById.get(edge.nodeAId);
      const nodeB = nodeById.get(edge.nodeBId);
      if (nodeA === undefined || nodeB === undefined) continue;
      const endpointA: WorldPoint = [nodeA.x, nodeA.y, BACKBONE_DISPLAY_PLANE_Z];
      const endpointB: WorldPoint = [nodeB.x, nodeB.y, BACKBONE_DISPLAY_PLANE_Z];
      const geometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...endpointA),
        new THREE.Vector3(...endpointB)
      ]);
      const line = new THREE.Line(geometry, lineMaterial.clone());
      line.renderOrder = 20;
      line.userData = {
        pickableBackbone: true,
        pickKind: "edge",
        nodeAId: edge.nodeAId,
        nodeBId: edge.nodeBId,
        endpointA,
        endpointB
      };
      this.backbone.add(line);
    }
    for (const node of snapshot.supportNodes) {
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.14, 10, 8),
        nodeMaterial.clone()
      );
      marker.position.set(node.x, node.y, BACKBONE_DISPLAY_PLANE_Z);
      marker.renderOrder = 21;
      marker.userData = {
        pickableBackbone: true,
        pickKind: "node",
        hitId: node.kind === 0 && node.poleId !== "0" ? node.poleId : node.id
      };
      this.backbone.add(marker);
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
