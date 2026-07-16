import * as THREE from "three";
import type {
  PathPickInfo,
  SceneContentSyncStats,
  SupportNodeInfo,
  VisualModelInstanceInfo
} from "../model";
import type { ViewerSnapshot, ViewerStore } from "../store/viewer";
import type { WorldPoint } from "../store/viewer";
import {
  cloneSharedAsset,
  type LoadedModelAsset,
  modelAssetCache
} from "./modelAssets";

function colorFromRgba(rgba: number): { color: THREE.Color; opacity: number } {
  const red = (rgba >>> 24) & 0xff;
  const green = (rgba >>> 16) & 0xff;
  const blue = (rgba >>> 8) & 0xff;
  const alpha = rgba & 0xff;
  const source = new THREE.Color(red / 255, green / 255, blue / 255);
  const nearBlack = new THREE.Color(12 / 255, 14 / 255, 16 / 255);
  return { color: nearBlack.lerp(source, 0.12), opacity: Math.max(210, alpha) / 255 };
}

const POLE_TOP_DIAMETER_M = 0.190;
const POLE_TAPER_RATIO = 75;
export const POLE_RENDER_SIDES = 16;
export const WIRE_RADIAL_SEGMENTS = 3;
const BACKBONE_DISPLAY_PLANE_Z = 0.0;
const BRANCH_ENDPOINT_SNAP_RADIUS_M = 0.6;
const SUPPORT_PATH_SUPPLEMENTAL_KIND = 1;

export function setPoleRotation(
  object: THREE.Object3D,
  rotationXDeg: number,
  rotationYDeg: number,
  rotationZDeg: number
): void {
  // Core applies X, then Y, then Z to the local vector. Three.js expresses
  // that same composition with a ZYX Euler order.
  object.rotation.set(
    THREE.MathUtils.degToRad(rotationXDeg),
    THREE.MathUtils.degToRad(rotationYDeg),
    THREE.MathUtils.degToRad(rotationZDeg),
    "ZYX"
  );
}

export class SampledWireCurve extends THREE.Curve<THREE.Vector3> {
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
    let low = 1;
    let high = this.lengths.length - 1;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (this.lengths[middle] < distance) {
        low = middle + 1;
      } else {
        high = middle;
      }
    }
    const index = low;

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
  private readonly snapPreview = new THREE.Group();
  private readonly groundGrid = new THREE.GridHelper(80, 40, 0x587288, 0x72889a);
  private readonly unsubscribe: () => void;
  private frame = 0;
  private resizeObserver: ResizeObserver | null = null;
  private detachInput: (() => void) | null = null;
  private snapshot: ViewerSnapshot | null = null;
  private lastFrameTime: number | null = null;
  private backboneSignature = "";
  private guideSignature = "";
  private cameraFov: number | null = null;
  private readonly partMeshes = new Map<string, { mesh: THREE.Mesh; version: string }>();
  private supportWireMaterial: THREE.MeshStandardMaterial | null = null;
  private readonly modelObjects = new Map<string, {
    object: THREE.Object3D;
    modelKey: string;
    version: string;
  }>();
  private readonly pendingModelKeys = new Set<string>();
  private readonly poleMeshes = new Map<string, {
    object: THREE.Object3D;
    version: string;
    ownsResources: boolean;
  }>();
  private contentSyncStats: SceneContentSyncStats = {
    total: 0,
    reused: 0,
    rebuilt: 0,
    removed: 0,
    modelTotal: 0,
    modelReused: 0,
    modelUpdated: 0,
    modelRebuilt: 0,
    modelRemoved: 0
  };

  constructor(
    private readonly store: ViewerStore,
    private readonly onGroundClick: (point: WorldPoint, pick?: PathPickInfo) => void,
    private readonly onContextAction: () => void,
    private readonly onFrame: (deltaMs: number) => void,
    private readonly onContentSync?: (stats: SceneContentSyncStats) => void
  ) {
    this.scene.background = new THREE.Color(0xc8d6e4);
    this.scene.add(this.backbone);
    this.scene.add(this.content);
    this.scene.add(this.guide);
    this.scene.add(this.snapPreview);
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

    this.groundGrid.rotateX(Math.PI / 2);
    this.scene.add(this.groundGrid);

    this.camera.up.set(0, 0, 1);
    this.camera.position.set(24, -30, 20);
    this.camera.lookAt(this.cameraTarget);
    this.unsubscribe = this.store.value.subscribe((snapshot) => this.applySnapshot(snapshot));
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
      orbitAnchor: THREE.Vector3 | null;
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
      this.clearSnapPreview();
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
        moved: false,
        orbitAnchor: mode === "orbit" ? this.pointerWorldPoint(event.clientX, event.clientY) : null
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
          this.dolly(-Math.log(next.distance / pinch.distance), this.pointerWorldPoint(next.centerX, next.centerY));
          this.pan(next.centerX - pinch.centerX, next.centerY - pinch.centerY);
        }
        pinch = next;
        event.preventDefault();
        return;
      }
      if (pointerDown === null) {
        if (event.pointerType === "mouse") {
          this.updateSnapPreview(event);
        }
        return;
      }
      this.clearSnapPreview();
      const dx = event.clientX - pointerDown.x;
      const dy = event.clientY - pointerDown.y;
      const totalDx = event.clientX - pointerDown.startX;
      const totalDy = event.clientY - pointerDown.startY;
      pointerDown.x = event.clientX;
      pointerDown.y = event.clientY;
      pointerDown.moved ||= Math.hypot(totalDx, totalDy) > 6;
      if (!pointerDown.moved) return;
      if (pointerDown.mode === "orbit") this.orbit(dx, dy, pointerDown.orbitAnchor);
      if (pointerDown.mode === "pan") this.pan(dx, dy);
      if (pointerDown.mode === "dolly") this.dolly(dy * 0.01, this.pointerWorldPoint(event.clientX, event.clientY));
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
      this.clearSnapPreview();
    };
    const onContextMenu = (event: MouseEvent) => {
      event.preventDefault();
      this.onContextAction();
    };
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      this.dolly(event.deltaY * 0.0012, this.pointerWorldPoint(event.clientX, event.clientY));
    };
    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerleave", onPointerCancel);
    canvas.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("pointercancel", onPointerCancel);
    canvas.addEventListener("contextmenu", onContextMenu);
    canvas.addEventListener("wheel", onWheel, { passive: false });
    this.detachInput = () => {
      canvas.removeEventListener("pointerdown", onPointerDown);
      canvas.removeEventListener("pointermove", onPointerMove);
      canvas.removeEventListener("pointerleave", onPointerCancel);
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
    for (const item of this.partMeshes.values()) this.disposeContentMesh(item.mesh);
    this.supportWireMaterial?.dispose();
    this.supportWireMaterial = null;
    for (const item of this.modelObjects.values()) this.content.remove(item.object);
    for (const item of this.poleMeshes.values()) this.disposePoleObject(item);
    this.partMeshes.clear();
    this.modelObjects.clear();
    this.poleMeshes.clear();
    this.disposeGroup(this.backbone);
    this.disposeGroup(this.guide);
    this.disposeGroup(this.snapPreview);
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
    const pointer = this.pointerFromClient(event.clientX, event.clientY);
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

  private pointerFromClient(clientX: number, clientY: number): THREE.Vector2 {
    const bounds = this.renderer.domElement.getBoundingClientRect();
    return new THREE.Vector2(
      ((clientX - bounds.left) / bounds.width) * 2 - 1,
      -((clientY - bounds.top) / bounds.height) * 2 + 1
    );
  }

  private pointerWorldPoint(clientX: number, clientY: number): THREE.Vector3 | null {
    const ray = new THREE.Raycaster();
    ray.setFromCamera(this.pointerFromClient(clientX, clientY), this.camera);
    const hit = new THREE.Vector3();
    const forward = this.cameraTarget.clone().sub(this.camera.position).normalize();
    if (forward.lengthSq() <= 0) return null;
    const targetPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(forward, this.cameraTarget);
    if (ray.ray.intersectPlane(targetPlane, hit)) {
      return hit.clone();
    }
    return null;
  }

  private pickBackbonePoint(ray: THREE.Raycaster): { point: WorldPoint; pick: PathPickInfo } | null {
    if (this.snapshot?.showBackboneOverlay !== true) return null;
    const hits = ray.intersectObjects(this.backbone.children, true)
      .filter((hit) => hit.object.userData.pickableBackbone);
    const hit = hits[0];
    if (hit === undefined) return null;
    const data = hit.object.userData;
    const displayPoint: WorldPoint = [hit.point.x, hit.point.y, BACKBONE_DISPLAY_PLANE_Z];
    const makePick = (point: WorldPoint, hitKind: number, hitId: string): PathPickInfo => ({
      hitKind,
      hitId,
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
    });
    if (data.pickKind === "node") {
      const point = (data.worldPoint as WorldPoint | undefined) ?? displayPoint;
      return {
        point,
        pick: makePick(point, 1, data.hitId ?? "0")
      };
    }
    if (data.pickKind === "edge") {
      const endpointA = data.endpointA as WorldPoint;
      const endpointB = data.endpointB as WorldPoint;
      const edgePoint = this.closestBackbonePointXY(displayPoint, endpointA, endpointB);
      const snappedEndpoint = this.snappedBackboneEndpoint(edgePoint, endpointA, endpointB);
      if (snappedEndpoint !== null) {
        const useA = snappedEndpoint === endpointA;
        const nodeId = useA ? data.nodeAId : data.nodeBId;
        if (nodeId !== undefined) {
          return {
            point: snappedEndpoint,
            pick: makePick(snappedEndpoint, 1, nodeId)
          };
        }
      }
      const point = edgePoint;
      return {
        point,
        pick: {
          ...makePick(point, 2, data.hitId ?? "0"),
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

  private snappedBackboneEndpoint(point: WorldPoint, endpointA: WorldPoint, endpointB: WorldPoint): WorldPoint | null {
    const da2 = this.distanceSquared(point, endpointA);
    const db2 = this.distanceSquared(point, endpointB);
    const snapR2 = BRANCH_ENDPOINT_SNAP_RADIUS_M * BRANCH_ENDPOINT_SNAP_RADIUS_M;
    if (da2 > snapR2 && db2 > snapR2) return null;
    return da2 <= db2 ? endpointA : endpointB;
  }

  private distanceSquared(a: WorldPoint, b: WorldPoint): number {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return dx * dx + dy * dy + dz * dz;
  }

  private closestBackbonePointXY(point: WorldPoint, endpointA: WorldPoint, endpointB: WorldPoint): WorldPoint {
    const dx = endpointB[0] - endpointA[0];
    const dy = endpointB[1] - endpointA[1];
    const dz = endpointB[2] - endpointA[2];
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
      endpointA[2] + dz * t
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

  private orbit(dx: number, dy: number, anchor: THREE.Vector3 | null = null): void {
    if (anchor !== null) {
      const cameraOffset = this.camera.position.clone().sub(anchor);
      const targetOffset = this.cameraTarget.clone().sub(anchor);
      const yawAxis = this.camera.up.clone().normalize();
      cameraOffset.applyAxisAngle(yawAxis, -dx * 0.006);
      targetOffset.applyAxisAngle(yawAxis, -dx * 0.006);
      const forward = targetOffset.clone().sub(cameraOffset).normalize();
      const right = forward.clone().cross(this.camera.up).normalize();
      if (right.lengthSq() > 0) {
        const pitchedCamera = cameraOffset.clone().applyAxisAngle(right, -dy * 0.006);
        const pitchedTarget = targetOffset.clone().applyAxisAngle(right, -dy * 0.006);
        const pitchedForward = pitchedTarget.clone().sub(pitchedCamera).normalize();
        if (Math.abs(pitchedForward.dot(this.camera.up)) < 0.995) {
          cameraOffset.copy(pitchedCamera);
          targetOffset.copy(pitchedTarget);
        }
      }
      this.camera.position.copy(anchor).add(cameraOffset);
      this.cameraTarget.copy(anchor).add(targetOffset);
      this.camera.lookAt(this.cameraTarget);
      return;
    }
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

  private dolly(amount: number, anchor: THREE.Vector3 | null = null): void {
    const offset = this.camera.position.clone().sub(this.cameraTarget);
    const currentDistance = offset.length();
    if (!Number.isFinite(currentDistance) || currentDistance <= 0) return;
    const distance = THREE.MathUtils.clamp(currentDistance * Math.exp(amount), 0.2, 2000);
    const factor = distance / currentDistance;
    if (anchor !== null) {
      const nextPosition = anchor.clone().add(this.camera.position.clone().sub(anchor).multiplyScalar(factor));
      const nextTarget = anchor.clone().add(this.cameraTarget.clone().sub(anchor).multiplyScalar(factor));
      if (!Number.isFinite(nextPosition.x) || !Number.isFinite(nextPosition.y) ||
          !Number.isFinite(nextPosition.z) || !Number.isFinite(nextTarget.x) ||
          !Number.isFinite(nextTarget.y) || !Number.isFinite(nextTarget.z)) {
        return;
      }
      this.camera.position.copy(nextPosition);
      this.cameraTarget.copy(nextTarget);
      this.camera.lookAt(this.cameraTarget);
      return;
    }
    this.camera.position.copy(this.cameraTarget).add(offset.normalize().multiplyScalar(distance));
  }

  private applySnapshot(snapshot: ViewerSnapshot): void {
    this.snapshot = snapshot;
    this.content.visible = true;
    this.backbone.visible = snapshot.showBackboneOverlay;
    this.guide.visible = snapshot.showPreview;
    this.groundGrid.visible = snapshot.showGroundGrid;

    if (this.cameraFov !== snapshot.cameraFov) {
      this.cameraFov = snapshot.cameraFov;
      this.camera.fov = snapshot.cameraFov;
      this.camera.updateProjectionMatrix();
    }

    if (this.syncContent(snapshot)) {
      this.clearSnapPreview();
    }
    this.onContentSync?.(this.contentSyncStats);

    const nextBackboneSignature = this.sceneBackboneSignature(snapshot);
    if (this.backboneSignature !== nextBackboneSignature) {
      this.backboneSignature = nextBackboneSignature;
      this.rebuildBackbone(snapshot);
      this.clearSnapPreview();
    }

    const nextGuideSignature = this.sceneGuideSignature(snapshot);
    if (this.guideSignature !== nextGuideSignature) {
      this.guideSignature = nextGuideSignature;
      this.rebuildGuide(snapshot);
    }
  }

  private sceneBackboneSignature(snapshot: ViewerSnapshot): string {
    const nodeKey = snapshot.supportNodes.map((node) => [
      node.id,
      node.kind,
      node.poleId,
      node.x,
      node.y,
      node.z
    ].join(":")).join("|");
    const edgeKey = snapshot.backboneEdges.map((edge) => [
      edge.nodeAId,
      edge.nodeBId,
      edge.bundleIds.join(",")
    ].join(":")).join("|");
    return `${nodeKey};${edgeKey}`;
  }

  private sceneGuideSignature(snapshot: ViewerSnapshot): string {
    const points = snapshot.pathPoints.map((point) => point.join(":")).join("|");
    const specs = snapshot.pathPointSpecs.map((spec) =>
      spec === null ? "null" : `${spec.supportKind}:${spec.nodeId}`
    ).join("|");
    return `${points};${specs}`;
  }

  private syncContent(snapshot: ViewerSnapshot): boolean {
    let changed = false;
    let reused = 0;
    let rebuilt = 0;
    let removed = 0;
    const nextPartKeys = new Set<string>();
    for (const part of snapshot.parts) {
      const key = part.info.partKey;
      nextPartKeys.add(key);
      const version = [
        part.info.sourceVersion,
        part.info.wireRadius,
        part.info.colorRgba,
        part.info.sampleCount
      ].join(":");
      const previous = this.partMeshes.get(key);
      if (previous?.version === version) {
        reused += 1;
        continue;
      }
      if (previous !== undefined) {
        this.disposeContentMesh(previous.mesh);
        this.partMeshes.delete(key);
      }
      const points: THREE.Vector3[] = [];
      for (let index = 0; index + 2 < part.samples.length; index += 3) {
        points.push(new THREE.Vector3(
          part.samples[index],
          part.samples[index + 1],
          part.samples[index + 2]
        ));
      }
      if (points.length < 2) {
        changed = true;
        continue;
      }

      const curve = new SampledWireCurve(points);
      const radius = THREE.MathUtils.clamp(part.info.wireRadius, 0.006, 0.08);
      const segments = Math.max(4, points.length - 1);
      const geometry = new THREE.TubeGeometry(curve, segments, radius, WIRE_RADIAL_SEGMENTS, false);
      const material = part.info.supplementalKind === SUPPORT_PATH_SUPPLEMENTAL_KIND
        ? this.getSupportWireMaterial()
        : this.makeWireMaterial(part.info.colorRgba);
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.content.add(mesh);
      this.partMeshes.set(key, { mesh, version });
      rebuilt += 1;
      changed = true;
    }
    for (const [key, previous] of [...this.partMeshes]) {
      if (nextPartKeys.has(key)) continue;
      this.disposeContentMesh(previous.mesh);
      this.partMeshes.delete(key);
      removed += 1;
      changed = true;
    }

    let modelReused = 0;
    let modelUpdated = 0;
    let modelRebuilt = 0;
    let modelRemoved = 0;
    const modeledPoleIds = new Set<string>();
    const nextModelKeys = new Set<string>();
    for (const model of snapshot.models) {
      nextModelKeys.add(model.stableKey);
      if (model.stableKey.startsWith("pole:")) {
        const separator = model.stableKey.indexOf(":", 5);
        if (separator > 5) modeledPoleIds.add(model.stableKey.slice(5, separator));
      }
      const previous = this.modelObjects.get(model.stableKey);
      if (previous?.modelKey === model.modelKey) {
        if (previous.version === model.contentVersion) {
          modelReused += 1;
        } else {
          this.applyModelTransform(previous.object, model);
          previous.version = model.contentVersion;
          modelUpdated += 1;
          changed = true;
        }
        continue;
      }
      const asset = modelAssetCache.loadedModel(model.modelKey);
      if (asset === null) {
        if (!this.pendingModelKeys.has(model.modelKey)) {
          this.pendingModelKeys.add(model.modelKey);
          void modelAssetCache.loadModel(model.modelKey).then(() => {
            this.pendingModelKeys.delete(model.modelKey);
            if (this.snapshot !== null) this.syncContent(this.snapshot);
          }).catch((error: unknown) => {
            this.pendingModelKeys.delete(model.modelKey);
            console.error(`[wire] model asset unavailable: ${model.modelKey}: ${String(error)}`);
          });
        }
        continue;
      }
      if (previous !== undefined) {
        this.content.remove(previous.object);
        this.modelObjects.delete(model.stableKey);
      }
      const object = this.makeModelObject(asset, model);
      this.content.add(object);
      this.modelObjects.set(model.stableKey, {
        object,
        modelKey: model.modelKey,
        version: model.contentVersion
      });
      modelRebuilt += 1;
      changed = true;
    }
    for (const [key, previous] of [...this.modelObjects]) {
      if (nextModelKeys.has(key)) continue;
      this.content.remove(previous.object);
      this.modelObjects.delete(key);
      modelRemoved += 1;
      changed = true;
    }

    const nextPoleKeys = new Set<string>();
    for (const pole of snapshot.poles) {
      if (modeledPoleIds.has(pole.id)) continue;
      const key = pole.id;
      nextPoleKeys.add(key);
      const version = [
        pole.poleTypeId,
        pole.positionX,
        pole.positionY,
        pole.positionZ,
        pole.rotationX,
        pole.rotationY,
        pole.rotationZ,
        pole.scaleX,
        pole.scaleY,
        pole.scaleZ,
        pole.height,
        snapshot.solidSupportRender ? 1 : 0
      ].join(":");
      const previous = this.poleMeshes.get(key);
      if (previous?.version === version) continue;
      if (previous !== undefined) {
        this.disposePoleObject(previous);
        this.poleMeshes.delete(key);
      }
      const entry = this.makePolePrimitive(pole, snapshot.solidSupportRender);
      this.content.add(entry.object);
      this.poleMeshes.set(key, { ...entry, version });
      changed = true;
    }
    for (const [key, previous] of [...this.poleMeshes]) {
      if (nextPoleKeys.has(key)) continue;
      this.disposePoleObject(previous);
      this.poleMeshes.delete(key);
      changed = true;
    }
    this.contentSyncStats = {
      total: snapshot.parts.length,
      reused,
      rebuilt,
      removed,
      modelTotal: snapshot.models.length,
      modelReused,
      modelUpdated,
      modelRebuilt,
      modelRemoved
    };
    return changed;
  }

  private makeWireMaterial(colorRgba: number): THREE.MeshStandardMaterial {
    const appearance = colorFromRgba(colorRgba);
    return new THREE.MeshStandardMaterial({
      color: appearance.color,
      metalness: 0.12,
      roughness: 0.58,
      opacity: appearance.opacity,
      transparent: appearance.opacity < 1
    });
  }

  private getSupportWireMaterial(): THREE.MeshStandardMaterial {
    if (this.supportWireMaterial === null || this.supportWireMaterial === undefined) {
      this.supportWireMaterial = new THREE.MeshStandardMaterial({
        color: new THREE.Color(18 / 255, 21 / 255, 24 / 255),
        metalness: 0.18,
        roughness: 0.72
      });
    }
    return this.supportWireMaterial;
  }

  private disposeContentMesh(mesh: THREE.Mesh): void {
    mesh.geometry.dispose();
    const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const material of materials) {
      if (material !== this.supportWireMaterial) material.dispose();
    }
    this.content.remove(mesh);
  }

  private makeModelObject(
    asset: LoadedModelAsset,
    model: VisualModelInstanceInfo
  ): THREE.Object3D {
    const root = new THREE.Group();
    const source = cloneSharedAsset(asset);
    source.traverse((object) => {
      if (!(object instanceof THREE.Mesh)) return;
      object.castShadow = true;
      object.receiveShadow = true;
    });
    root.add(source);
    this.applyModelTransform(root, model);
    root.userData.sourceKind = "model";
    root.userData.sourceId = model.stableKey;
    root.userData.modelKey = model.modelKey;
    return root;
  }

  private applyModelTransform(object: THREE.Object3D, model: VisualModelInstanceInfo): void {
    object.position.set(model.positionX, model.positionY, model.positionZ);
    setPoleRotation(object, model.rotationX, model.rotationY, model.rotationZ);
    object.scale.set(model.scaleX, model.scaleY, model.scaleZ);
  }

  private makePolePrimitive(
    pole: ViewerSnapshot["poles"][number],
    solidSupportRender: boolean
  ): { object: THREE.Object3D; ownsResources: boolean } {
    const topRadius = POLE_TOP_DIAMETER_M / 2;
    const groundRadius = (POLE_TOP_DIAMETER_M + pole.height / POLE_TAPER_RATIO) / 2;
    const geometry = new THREE.CylinderGeometry(topRadius, groundRadius, pole.height, POLE_RENDER_SIDES);
    geometry.rotateX(Math.PI / 2);
    geometry.translate(0, 0, pole.height / 2);
    const material = new THREE.MeshStandardMaterial({
      color: 0x434b48,
      roughness: 0.88,
      wireframe: !solidSupportRender
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(pole.positionX, pole.positionY, pole.positionZ);
    setPoleRotation(mesh, pole.rotationX, pole.rotationY, pole.rotationZ);
    mesh.scale.set(pole.scaleX, pole.scaleY, pole.scaleZ);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData.sourceKind = "pole";
    mesh.userData.sourceId = pole.id;
    return { object: mesh, ownsResources: true };
  }

  private disposePoleObject(entry: {
    object: THREE.Object3D;
    ownsResources: boolean;
  }): void {
    if (entry.ownsResources) {
      entry.object.traverse((object) => {
        if (!(object instanceof THREE.Mesh)) return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) material.dispose();
      });
    }
    this.content.remove(entry.object);
  }

  private rebuildBackbone(snapshot: ViewerSnapshot): void {
    this.disposeGroup(this.backbone);
    this.buildBackboneOverlay(snapshot);
  }

  private rebuildGuide(snapshot: ViewerSnapshot): void {
    this.disposeGroup(this.guide);
    this.buildPathGuide(snapshot);
  }

  private buildPathGuide(snapshot: ViewerSnapshot): void {
    if (snapshot.pathPoints.length === 0) return;

    const guidePoints = snapshot.pathPoints.map(
      (point) => new THREE.Vector3(point[0], point[1], point[2] + 0.12)
    );
    const guideLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(guidePoints),
      new THREE.LineBasicMaterial({
        color: 0x4db6d1,
        transparent: true,
        opacity: 0.92,
        depthTest: false,
        depthWrite: false
      })
    );
    guideLine.renderOrder = 40;
    this.guide.add(guideLine);

    for (let index = 0; index < guidePoints.length; index += 1) {
      const point = guidePoints[index];
      const snapped = snapshot.pathPointSpecs[index] !== null;
      if (snapped) {
        this.addSnappedPathMarker(point);
        continue;
      }
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.18, 10, 8),
        new THREE.MeshBasicMaterial({
          color: 0x4db6d1,
          depthTest: false,
          depthWrite: false
        })
      );
      marker.position.copy(point);
      marker.renderOrder = 41;
      this.guide.add(marker);
    }
  }

  private addSnappedPathMarker(point: THREE.Vector3): void {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.52, 0.035, 8, 40),
      new THREE.MeshBasicMaterial({
        color: 0xffb13b,
        depthTest: false,
        depthWrite: false
      })
    );
    ring.position.copy(point);
    ring.renderOrder = 44;
    this.guide.add(ring);
  }

  private updateSnapPreview(event: PointerEvent): void {
    this.clearSnapPreview();
    const bounds = this.renderer.domElement.getBoundingClientRect();
    const pointer = new THREE.Vector2(
      ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
      -((event.clientY - bounds.top) / bounds.height) * 2 + 1
    );
    const ray = new THREE.Raycaster();
    ray.params.Line = { threshold: 0.35 };
    ray.setFromCamera(pointer, this.camera);
    const hit = this.pickBackbonePoint(ray);
    if (hit === null) return;

    const point = new THREE.Vector3(hit.point[0], hit.point[1], hit.point[2] + 0.08);
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.28, 0.018, 8, 32),
      new THREE.MeshBasicMaterial({
        color: 0xffb13b,
        depthTest: false,
        depthWrite: false
      })
    );
    ring.position.copy(point);
    ring.renderOrder = 60;
    this.snapPreview.add(ring);

    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(0.08, 10, 8),
      new THREE.MeshBasicMaterial({
        color: 0xffd36f,
        depthTest: false,
        depthWrite: false
      })
    );
    dot.position.copy(point);
    dot.renderOrder = 61;
    this.snapPreview.add(dot);
  }

  private clearSnapPreview(): void {
    this.disposeGroup(this.snapPreview);
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
      const endpointA: WorldPoint = [nodeA.x, nodeA.y, nodeA.z];
      const endpointB: WorldPoint = [nodeB.x, nodeB.y, nodeB.z];
      const displayA: WorldPoint = [nodeA.x, nodeA.y, BACKBONE_DISPLAY_PLANE_Z];
      const displayB: WorldPoint = [nodeB.x, nodeB.y, BACKBONE_DISPLAY_PLANE_Z];
      const geometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...displayA),
        new THREE.Vector3(...displayB)
      ]);
      const pickData = {
        pickableBackbone: true,
        pickKind: "edge",
        nodeAId: edge.nodeAId,
        nodeBId: edge.nodeBId,
        endpointA,
        endpointB
      };
      const line = new THREE.Line(geometry, lineMaterial.clone());
      line.renderOrder = 20;
      line.userData = pickData;
      this.backbone.add(line);

      const hitCurve = new THREE.LineCurve3(new THREE.Vector3(...displayA), new THREE.Vector3(...displayB));
      const hitMesh = new THREE.Mesh(
        new THREE.TubeGeometry(hitCurve, 1, 0.45, 8, false),
        new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false })
      );
      hitMesh.userData = pickData;
      this.backbone.add(hitMesh);
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
        hitId: node.kind === 0 && node.poleId !== "0" ? node.poleId : node.id,
        worldPoint: [node.x, node.y, node.z]
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
