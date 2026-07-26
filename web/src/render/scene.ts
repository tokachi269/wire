import * as THREE from "three";
import type {
  PathPickInfo,
  PoleInfo,
  SceneContentSyncStats,
  SupportNodeInfo,
  VisualModelInstanceInfo
} from "../model";
import type { ViewerSnapshot, ViewerStore } from "../store/viewer";
import type { WorldPoint } from "../store/viewer";
import {
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
const SUPPORT_PATH_SUPPLEMENTAL_KIND = 1;

interface ModelMeshSource {
  geometry: THREE.BufferGeometry;
  material: THREE.Material | THREE.Material[];
  localMatrix: THREE.Matrix4;
}

interface SampledTubeBuffers {
  positions: Float32Array;
  normals: Float32Array;
  indices: Uint16Array | Uint32Array;
}
const BACKBONE_NODE_SNAP_PX = 24;
const BACKBONE_ENDPOINT_SNAP_PX = 40;
const BACKBONE_EDGE_SNAP_PX = 16;

export function makeBackbonePick(point: WorldPoint, hitKind: number, hitId: string): PathPickInfo {
  return {
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
  };
}

export function backboneNodeHitId(node: SupportNodeInfo): string {
  return node.id;
}

export function poleAxisEndpoints(pole: PoleInfo): [THREE.Vector3, THREE.Vector3] {
  const root = new THREE.Object3D();
  setPoleRotation(root, pole.rotationX, pole.rotationY, pole.rotationZ);
  const base = new THREE.Vector3(pole.positionX, pole.positionY, pole.positionZ);
  const top = new THREE.Vector3(0, 0, pole.height * pole.scaleZ).applyEuler(root.rotation).add(base);
  return [base, top];
}

export function distanceToScreenSegmentPx(point: THREE.Vector2, a: THREE.Vector2, b: THREE.Vector2): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length2 = dx * dx + dy * dy;
  const t = length2 > 0
    ? THREE.MathUtils.clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / length2, 0, 1)
    : 0;
  return point.distanceTo(new THREE.Vector2(a.x + dx * t, a.y + dy * t));
}

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

export function makeSampledTubeGeometry(samples: Float64Array, radius: number): THREE.BufferGeometry {
  const pointCount = Math.floor(samples.length / 3);
  const geometry = new THREE.BufferGeometry();
  if (pointCount < 2) return geometry;

  const buffers = makeSampledTubeBuffers(pointCount);
  writeSampledTubeBuffers(samples, radius, buffers);
  geometry.setAttribute("position", new THREE.BufferAttribute(buffers.positions, 3));
  geometry.setAttribute("normal", new THREE.BufferAttribute(buffers.normals, 3));
  geometry.setIndex(new THREE.BufferAttribute(buffers.indices, 1));
  geometry.computeBoundingSphere();
  return geometry;
}

export function makeRoadMeshGeometry(data: {
  vertices: Float64Array;
  indices: Uint32Array;
}): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();
  const positions = new Float32Array(data.vertices);
  const colors = new Float32Array(positions.length);
  for (let index = 0; index + 2 < positions.length; index += 3) {
    const raised = positions[index + 2] > 0.08;
    colors[index] = raised ? 0.55 : 0.22;
    colors[index + 1] = raised ? 0.58 : 0.24;
    colors[index + 2] = raised ? 0.56 : 0.25;
  }
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(data.indices), 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  return geometry;
}

function makeSampledTubeBuffers(pointCount: number): SampledTubeBuffers {
  const radialSegments = WIRE_RADIAL_SEGMENTS;
  const vertexCount = pointCount * radialSegments;
  const IndexArray = vertexCount <= 0xffff ? Uint16Array : Uint32Array;
  return {
    positions: new Float32Array(vertexCount * 3),
    normals: new Float32Array(vertexCount * 3),
    indices: new IndexArray((pointCount - 1) * radialSegments * 6)
  };
}

function writeSampledTubeBuffers(samples: Float64Array, radius: number, buffers: SampledTubeBuffers): void {
  const pointCount = Math.floor(samples.length / 3);
  const radialSegments = WIRE_RADIAL_SEGMENTS;
  const centers: THREE.Vector3[] = [];
  for (let index = 0; index < pointCount; index += 1) {
    centers.push(new THREE.Vector3(
      samples[index * 3],
      samples[index * 3 + 1],
      samples[index * 3 + 2]
    ));
  }

  const tangentAt = (index: number): THREE.Vector3 => {
    const prev = centers[Math.max(0, index - 1)];
    const next = centers[Math.min(pointCount - 1, index + 1)];
    const tangent = next.clone().sub(prev);
    if (tangent.lengthSq() <= 1e-18 && index + 1 < pointCount) {
      tangent.copy(centers[index + 1]).sub(centers[index]);
    }
    if (tangent.lengthSq() <= 1e-18 && index > 0) {
      tangent.copy(centers[index]).sub(centers[index - 1]);
    }
    return tangent.lengthSq() <= 1e-18
      ? new THREE.Vector3(1, 0, 0)
      : tangent.normalize();
  };

  let tangent = tangentAt(0);
  const reference = Math.abs(tangent.z) < 0.9
    ? new THREE.Vector3(0, 0, 1)
    : new THREE.Vector3(0, 1, 0);
  let normal = reference.clone().cross(tangent).normalize();
  let binormal = tangent.clone().cross(normal).normalize();
  const nextTangent = new THREE.Vector3();
  const rotation = new THREE.Quaternion();
  const radial = new THREE.Vector3();
  for (let pointIndex = 0; pointIndex < pointCount; pointIndex += 1) {
    if (pointIndex > 0) {
      nextTangent.copy(tangentAt(pointIndex));
      rotation.setFromUnitVectors(tangent, nextTangent);
      normal.applyQuaternion(rotation).normalize();
      binormal.copy(nextTangent).cross(normal).normalize();
      tangent = nextTangent.clone();
    }
    const center = centers[pointIndex];
    for (let radialIndex = 0; radialIndex < radialSegments; radialIndex += 1) {
      const angle = (radialIndex / radialSegments) * Math.PI * 2;
      radial.copy(normal).multiplyScalar(Math.cos(angle))
        .addScaledVector(binormal, Math.sin(angle))
        .normalize();
      const vertexOffset = (pointIndex * radialSegments + radialIndex) * 3;
      buffers.positions[vertexOffset] = center.x + radial.x * radius;
      buffers.positions[vertexOffset + 1] = center.y + radial.y * radius;
      buffers.positions[vertexOffset + 2] = center.z + radial.z * radius;
      buffers.normals[vertexOffset] = radial.x;
      buffers.normals[vertexOffset + 1] = radial.y;
      buffers.normals[vertexOffset + 2] = radial.z;
    }
  }

  let indexOffset = 0;
  for (let pointIndex = 0; pointIndex + 1 < pointCount; pointIndex += 1) {
    const row = pointIndex * radialSegments;
    const nextRow = (pointIndex + 1) * radialSegments;
    for (let radialIndex = 0; radialIndex < radialSegments; radialIndex += 1) {
      const nextRadial = (radialIndex + 1) % radialSegments;
      buffers.indices[indexOffset++] = row + radialIndex;
      buffers.indices[indexOffset++] = nextRow + radialIndex;
      buffers.indices[indexOffset++] = row + nextRadial;
      buffers.indices[indexOffset++] = row + nextRadial;
      buffers.indices[indexOffset++] = nextRow + radialIndex;
      buffers.indices[indexOffset++] = nextRow + nextRadial;
    }
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
  private readonly road = new THREE.Group();
  private readonly roadPreview = new THREE.Group();
  private readonly snapPreviewRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.28, 0.018, 8, 32),
    new THREE.MeshBasicMaterial({
      color: 0xffb13b,
      depthTest: false,
      depthWrite: false
    })
  );
  private readonly groundGrid = new THREE.GridHelper(80, 40, 0x587288, 0x72889a);
  private readonly unsubscribe: () => void;
  private frame = 0;
  private resizeObserver: ResizeObserver | null = null;
  private detachInput: (() => void) | null = null;
  private snapshot: ViewerSnapshot | null = null;
  private lastFrameTime: number | null = null;
  private backboneSignature = "";
  private guideSignature = "";
  private roadSignature = "";
  private cameraFov: number | null = null;
  private readonly partMeshes = new Map<string, {
    mesh: THREE.Mesh;
    version: string;
    materialKey: string;
  }>();
  private supportWireMaterial: THREE.MeshStandardMaterial | null = null;
  private readonly modelObjects = new Map<string, {
    modelKey: string;
    version: string;
  }>();
  private readonly modelBatches = new Map<string, {
    meshes: THREE.InstancedMesh[];
    capacity: number;
    meshSources: ModelMeshSource[];
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
    private readonly onGroundPreview: (point: WorldPoint) => void,
    private readonly onContextAction: () => void,
    private readonly onFrame: (deltaMs: number) => void,
    private readonly onContentSync?: (stats: SceneContentSyncStats) => void
  ) {
    this.scene.background = new THREE.Color(0xc8d6e4);
    this.scene.add(this.backbone);
    this.scene.add(this.content);
    this.scene.add(this.guide);
    this.scene.add(this.snapPreview);
    this.scene.add(this.road);
    this.scene.add(this.roadPreview);
    this.snapPreviewRing.renderOrder = 60;
    this.snapPreview.add(this.snapPreviewRing);
    this.snapPreview.visible = false;
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
    for (const batch of this.modelBatches.values()) this.disposeModelBatch(batch);
    for (const item of this.poleMeshes.values()) this.disposePoleObject(item);
    this.partMeshes.clear();
    this.modelObjects.clear();
    this.modelBatches.clear();
    this.poleMeshes.clear();
    this.disposeGroup(this.backbone);
    this.disposeGroup(this.guide);
    this.disposeGroup(this.snapPreview);
    this.disposeGroup(this.road);
    this.disposeGroup(this.roadPreview);
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
    const backboneHit = this.snapshot?.activeTool === "wire"
      ? this.pickBackbonePoint(event.clientX, event.clientY)
      : null;
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

  private pickBackbonePoint(
    clientX: number,
    clientY: number
  ): { point: WorldPoint; pick: PathPickInfo } | null {
    if (this.snapshot?.showBackboneOverlay !== true) return null;
    const bounds = this.renderer.domElement.getBoundingClientRect();
    const pointerPx = new THREE.Vector2(clientX - bounds.left, clientY - bounds.top);
    const poleById = new Map(this.snapshot.poles.map((pole) => [pole.id, pole]));
    let bestNode: { distance: number; point: WorldPoint; pick: PathPickInfo } | null = null;
    for (const node of this.snapshot.supportNodes) {
      const displayPoint = new THREE.Vector3(node.x, node.y, BACKBONE_DISPLAY_PLANE_Z);
      const screenPoint = this.projectToCanvas(displayPoint, bounds);
      let distance = screenPoint === null ? Number.POSITIVE_INFINITY : screenPoint.distanceTo(pointerPx);
      const pole = node.poleId === "0" ? undefined : poleById.get(node.poleId);
      if (pole !== undefined) {
        const [base, top] = poleAxisEndpoints(pole);
        const screenBase = this.projectToCanvas(base, bounds);
        const screenTop = this.projectToCanvas(top, bounds);
        if (screenBase !== null && screenTop !== null) {
          distance = Math.min(distance, distanceToScreenSegmentPx(pointerPx, screenBase, screenTop));
        }
      }
      if (distance <= BACKBONE_NODE_SNAP_PX && (bestNode === null || distance < bestNode.distance)) {
        const point: WorldPoint = [node.x, node.y, node.z];
        bestNode = {
          distance,
          point,
          pick: makeBackbonePick(point, 1, backboneNodeHitId(node))
        };
      }
    }
    if (bestNode !== null) {
      return { point: bestNode.point, pick: bestNode.pick };
    }

    const nodeById = new Map(this.snapshot.supportNodes.map((node) => [node.id, node]));
    let bestEdge: { distance: number; point: WorldPoint; pick: PathPickInfo } | null = null;
    for (const edge of this.snapshot.backboneEdges) {
      const nodeA = nodeById.get(edge.nodeAId);
      const nodeB = nodeById.get(edge.nodeBId);
      if (nodeA === undefined || nodeB === undefined) continue;
      const screenA = this.projectToCanvas(new THREE.Vector3(nodeA.x, nodeA.y, BACKBONE_DISPLAY_PLANE_Z), bounds);
      const screenB = this.projectToCanvas(new THREE.Vector3(nodeB.x, nodeB.y, BACKBONE_DISPLAY_PLANE_Z), bounds);
      if (screenA === null || screenB === null) continue;
      const distanceA = screenA.distanceTo(pointerPx);
      const distanceB = screenB.distanceTo(pointerPx);
      if (distanceA <= BACKBONE_ENDPOINT_SNAP_PX || distanceB <= BACKBONE_ENDPOINT_SNAP_PX) {
        const useA = distanceA <= distanceB;
        const node = useA ? nodeA : nodeB;
        const point: WorldPoint = [node.x, node.y, node.z];
        return {
          point,
          pick: makeBackbonePick(point, 1, backboneNodeHitId(node))
        };
      }
      const dx = screenB.x - screenA.x;
      const dy = screenB.y - screenA.y;
      const length2 = dx * dx + dy * dy;
      const t = length2 > 0
        ? THREE.MathUtils.clamp(
            ((pointerPx.x - screenA.x) * dx + (pointerPx.y - screenA.y) * dy) / length2,
            0,
            1
          )
        : 0;
      const closest = new THREE.Vector2(screenA.x + dx * t, screenA.y + dy * t);
      const distance = closest.distanceTo(pointerPx);
      if (distance > BACKBONE_EDGE_SNAP_PX || (bestEdge !== null && distance >= bestEdge.distance)) {
        continue;
      }
      const endpointA: WorldPoint = [nodeA.x, nodeA.y, nodeA.z];
      const endpointB: WorldPoint = [nodeB.x, nodeB.y, nodeB.z];
      const point: WorldPoint = [
        endpointA[0] + (endpointB[0] - endpointA[0]) * t,
        endpointA[1] + (endpointB[1] - endpointA[1]) * t,
        endpointA[2] + (endpointB[2] - endpointA[2]) * t
      ];
      bestEdge = {
        distance,
        point,
        pick: {
          ...makeBackbonePick(point, 2, "0"),
          hasSegmentEndpoints: true,
          segmentNodeAId: edge.nodeAId,
          segmentNodeBId: edge.nodeBId,
          segmentEndpointAX: endpointA[0],
          segmentEndpointAY: endpointA[1],
          segmentEndpointAZ: endpointA[2],
          segmentEndpointBX: endpointB[0],
          segmentEndpointBY: endpointB[1],
          segmentEndpointBZ: endpointB[2]
        }
      };
    }
    if (bestEdge !== null) {
      return { point: bestEdge.point, pick: bestEdge.pick };
    }

    return null;
  }

  private projectToCanvas(point: THREE.Vector3, bounds: DOMRect): THREE.Vector2 | null {
    const projected = point.clone().project(this.camera);
    if (projected.z < -1 || projected.z > 1) return null;
    return new THREE.Vector2(
      ((projected.x + 1) * 0.5) * bounds.width,
      ((1 - projected.y) * 0.5) * bounds.height
    );
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

    const nextRoadSignature = this.sceneRoadSignature(snapshot);
    if (this.roadSignature !== nextRoadSignature) {
      this.roadSignature = nextRoadSignature;
      this.rebuildRoad(snapshot);
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

  private sceneRoadSignature(snapshot: ViewerSnapshot): string {
    const meshKey = (mesh: { vertices: Float64Array; indices: Uint32Array }) =>
      `${mesh.vertices.length}:${mesh.indices.length}:${mesh.vertices[0] ?? 0}:${mesh.vertices.at(-1) ?? 0}`;
    return [
      ...snapshot.road.scene.surfaceMeshes.map(meshKey),
      ...snapshot.road.scene.markingMeshes.map(meshKey),
      "preview",
      ...snapshot.road.previewMeshes.map(meshKey)
    ].join("|");
  }

  private rebuildRoad(snapshot: ViewerSnapshot): void {
    this.disposeGroup(this.road);
    this.disposeGroup(this.roadPreview);
    const surfaceMaterial = new THREE.MeshStandardMaterial({
      color: 0x4a4e50,
      roughness: 0.9,
      metalness: 0,
      vertexColors: true
    });
    const markingMaterial = new THREE.MeshStandardMaterial({
      color: 0xf2f0d9,
      roughness: 0.75,
      polygonOffset: true,
      polygonOffsetFactor: -2
    });
    for (const data of snapshot.road.scene.surfaceMeshes) {
      const mesh = new THREE.Mesh(makeRoadMeshGeometry(data), surfaceMaterial.clone());
      mesh.receiveShadow = true;
      mesh.castShadow = true;
      this.road.add(mesh);
    }
    for (const data of snapshot.road.scene.markingMeshes) {
      this.road.add(new THREE.Mesh(makeRoadMeshGeometry(data), markingMaterial.clone()));
    }
    for (const data of snapshot.road.previewMeshes) {
      const material = new THREE.MeshStandardMaterial({
        color: 0x66a8d8,
        transparent: true,
        opacity: 0.68,
        depthWrite: false,
        side: THREE.DoubleSide
      });
      const mesh = new THREE.Mesh(makeRoadMeshGeometry(data), material);
      mesh.position.z += 0.03;
      this.roadPreview.add(mesh);
    }
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
      if (part.samples.length < 6) {
        if (previous !== undefined) {
          this.disposeContentMesh(previous.mesh);
          this.partMeshes.delete(key);
        }
        changed = true;
        continue;
      }

      const radius = THREE.MathUtils.clamp(part.info.wireRadius, 0.006, 0.08);
      const materialKey = `${part.info.supplementalKind}:${part.info.colorRgba}`;
      if (previous !== undefined && this.updateSampledTubeGeometry(previous.mesh.geometry, part.samples, radius)) {
        if (previous.materialKey !== materialKey) {
          this.disposeMeshMaterial(previous.mesh);
          previous.mesh.material = this.makePartMaterial(part);
          previous.materialKey = materialKey;
        }
        previous.version = version;
        rebuilt += 1;
        changed = true;
        continue;
      }
      if (previous !== undefined) {
        this.disposeContentMesh(previous.mesh);
        this.partMeshes.delete(key);
      }
      const geometry = makeSampledTubeGeometry(part.samples, radius);
      const material = this.makePartMaterial(part);
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.content.add(mesh);
      this.partMeshes.set(key, { mesh, version, materialKey });
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
    const modelsByKey = new Map<string, VisualModelInstanceInfo[]>();
    for (const model of snapshot.models) {
      nextModelKeys.add(model.stableKey);
      if (model.stableKey.startsWith("pole:")) {
        const separator = model.stableKey.indexOf(":", 5);
        if (separator > 5) modeledPoleIds.add(model.stableKey.slice(5, separator));
      }
      const group = modelsByKey.get(model.modelKey) ?? [];
      group.push(model);
      modelsByKey.set(model.modelKey, group);
      const previous = this.modelObjects.get(model.stableKey);
      if (previous?.modelKey === model.modelKey) {
        if (previous.version === model.contentVersion) {
          modelReused += 1;
        } else {
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
        this.modelObjects.delete(model.stableKey);
      }
      this.modelObjects.set(model.stableKey, {
        modelKey: model.modelKey,
        version: model.contentVersion
      });
      modelRebuilt += 1;
      changed = true;
    }
    for (const [key] of [...this.modelObjects]) {
      if (nextModelKeys.has(key)) continue;
      this.modelObjects.delete(key);
      modelRemoved += 1;
      changed = true;
    }
    const liveModelKeys = new Set(modelsByKey.keys());
    for (const [modelKey, batch] of [...this.modelBatches]) {
      if (liveModelKeys.has(modelKey)) continue;
      this.disposeModelBatch(batch);
      this.modelBatches.delete(modelKey);
      changed = true;
    }
    for (const [modelKey, models] of modelsByKey) {
      const asset = modelAssetCache.loadedModel(modelKey);
      if (asset === null) continue;
      let batch = this.modelBatches.get(modelKey);
      if (batch === undefined || batch.capacity !== models.length) {
        if (batch !== undefined) this.disposeModelBatch(batch);
        batch = this.makeModelBatch(asset, models.length);
        this.modelBatches.set(modelKey, batch);
        changed = true;
      }
      this.updateModelBatch(batch, models);
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

  private makePartMaterial(part: ViewerSnapshot["parts"][number]): THREE.Material {
    return part.info.supplementalKind === SUPPORT_PATH_SUPPLEMENTAL_KIND
      ? this.getSupportWireMaterial()
      : this.makeWireMaterial(part.info.colorRgba);
  }

  private updateSampledTubeGeometry(geometry: THREE.BufferGeometry, samples: Float64Array, radius: number): boolean {
    const position = geometry.getAttribute("position");
    const normal = geometry.getAttribute("normal");
    const index = geometry.getIndex();
    if (!(position instanceof THREE.BufferAttribute) ||
        !(normal instanceof THREE.BufferAttribute) ||
        !(index instanceof THREE.BufferAttribute) ||
        !(position.array instanceof Float32Array) ||
        !(normal.array instanceof Float32Array) ||
        !(index.array instanceof Uint16Array || index.array instanceof Uint32Array)) {
      return false;
    }
    const pointCount = Math.floor(samples.length / 3);
    if (position.count !== pointCount * WIRE_RADIAL_SEGMENTS ||
        normal.count !== pointCount * WIRE_RADIAL_SEGMENTS ||
        index.count !== (pointCount - 1) * WIRE_RADIAL_SEGMENTS * 6) {
      return false;
    }
    writeSampledTubeBuffers(samples, radius, {
      positions: position.array,
      normals: normal.array,
      indices: index.array
    });
    position.needsUpdate = true;
    normal.needsUpdate = true;
    index.needsUpdate = true;
    geometry.computeBoundingSphere();
    geometry.computeBoundingBox();
    return true;
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
    this.disposeMeshMaterial(mesh);
    this.content.remove(mesh);
  }

  private disposeMeshMaterial(mesh: THREE.Mesh): void {
    const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const material of materials) {
      if (material !== this.supportWireMaterial) material.dispose();
    }
  }

  private makeModelBatch(asset: LoadedModelAsset, capacity: number): {
    meshes: THREE.InstancedMesh[];
    capacity: number;
    meshSources: ModelMeshSource[];
  } {
    const meshSources = this.modelMeshSources(asset);
    const meshes = meshSources.map((source) => {
      const mesh = new THREE.InstancedMesh(source.geometry, source.material, capacity);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.userData.sourceKind = "model";
      mesh.userData.modelKey = asset.modelKey;
      this.content.add(mesh);
      return mesh;
    });
    return { meshes, capacity, meshSources };
  }

  private modelMeshSources(asset: LoadedModelAsset): ModelMeshSource[] {
    const sources: ModelMeshSource[] = [];
    asset.source.updateMatrixWorld(true);
    asset.source.traverse((object) => {
      if (!(object instanceof THREE.Mesh)) return;
      object.updateMatrixWorld(true);
      sources.push({
        geometry: object.geometry,
        material: object.material,
        localMatrix: object.matrixWorld.clone()
      });
    });
    return sources;
  }

  private updateModelBatch(
    batch: {
      meshes: THREE.InstancedMesh[];
      capacity: number;
      meshSources: ModelMeshSource[];
    },
    models: VisualModelInstanceInfo[]
  ): void {
    const rootMatrix = new THREE.Matrix4();
    const modelMatrix = new THREE.Matrix4();
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    const euler = new THREE.Euler(0, 0, 0, "XYZ");
    for (let index = 0; index < models.length; index += 1) {
      const model = models[index];
      position.set(model.positionX, model.positionY, model.positionZ);
      euler.set(
        THREE.MathUtils.degToRad(model.rotationX),
        THREE.MathUtils.degToRad(model.rotationY),
        THREE.MathUtils.degToRad(model.rotationZ),
        "ZYX"
      );
      quaternion.setFromEuler(euler);
      scale.set(model.scaleX, model.scaleY, model.scaleZ);
      rootMatrix.compose(position, quaternion, scale);
      for (let meshIndex = 0; meshIndex < batch.meshes.length; meshIndex += 1) {
        modelMatrix.multiplyMatrices(rootMatrix, batch.meshSources[meshIndex].localMatrix);
        batch.meshes[meshIndex].setMatrixAt(index, modelMatrix);
      }
    }
    for (const mesh of batch.meshes) {
      mesh.count = models.length;
      mesh.instanceMatrix.needsUpdate = true;
    }
  }

  private disposeModelBatch(batch: { meshes: THREE.InstancedMesh[] }): void {
    for (const mesh of batch.meshes) {
      this.content.remove(mesh);
      mesh.dispose();
    }
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
    if (this.snapshot?.activeTool === "road") {
      const ray = new THREE.Raycaster();
      ray.setFromCamera(this.pointerFromClient(event.clientX, event.clientY), this.camera);
      const hit = new THREE.Vector3();
      const planeZ = this.snapshot.drawPlaneZ;
      if (ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ), hit)) {
        this.onGroundPreview([hit.x, hit.y, hit.z]);
      }
      return;
    }
    const hit = this.pickBackbonePoint(event.clientX, event.clientY);
    if (hit === null) return;

    const point = new THREE.Vector3(hit.point[0], hit.point[1], hit.point[2] + 0.08);
    this.snapPreviewRing.position.copy(point);
    this.snapPreview.visible = true;
  }

  private clearSnapPreview(): void {
    this.snapPreview.visible = false;
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
        hitId: backboneNodeHitId(node),
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
