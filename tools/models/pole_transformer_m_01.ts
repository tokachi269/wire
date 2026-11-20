import * as THREE from 'three';

// Units: meters. glTF Y-up. Origin on the tank axis, y=0 at the underside of the tank tray.
// Camera-side of the reference photo is +Z; the pole sits toward -X and the arm runs toward +X.

export type PassId = 'blockout' | 'structural-pass' | 'form-refinement' | 'material-pass';

export interface PoleSpec {
  /** Pole radius at y=0 of the model. */
  radius: number;
  /** Radius change per meter of height (tapered pole: smaller going up). Negative narrows upward. */
  taperPerM: number;
  /** Distance from the tank axis to the pole axis. */
  axisDistance: number;
}

export interface TransformerOptions {
  pass?: PassId;
  /** One neutral grey material instead of the four role materials (used for pre-material passes). */
  neutral?: boolean;
  pole?: Partial<PoleSpec>;
}

export const DEFAULT_POLE: PoleSpec = { radius: 0.118, taperPerM: -0.0133, axisDistance: 0.434 };

const PASS_ORDER: PassId[] = ['blockout', 'structural-pass', 'form-refinement', 'material-pass'];
const COLORS = { paint: 0xa9adb0, galv: 0xbcc0c2, porcelain: 0xe6e1cf, dark: 0x2b2c2f } as const;
type Role = keyof typeof COLORS;

export function makeMaterials(neutral = false): Record<Role, THREE.MeshStandardMaterial> {
  const mk = (name: Role, color: number, roughness: number, metalness: number) => {
    const m = new THREE.MeshStandardMaterial({ color: neutral ? 0xb4b8ba : color, roughness, metalness });
    m.name = name;
    return m;
  };
  if (neutral) {
    const n = mk('paint', 0, 0.6, 0.05);
    return { paint: n, galv: n, porcelain: n, dark: n };
  }
  return {
    paint: mk('paint', COLORS.paint, 0.6, 0.05),
    galv: mk('galv', COLORS.galv, 0.45, 0.25),
    porcelain: mk('porcelain', COLORS.porcelain, 0.25, 0),
    dark: mk('dark', COLORS.dark, 0.7, 0),
  };
}

// Tank wall radius at height y (matches the lathe profile below).
const TANK_PROFILE: [number, number][] = [[0, 0], [0.215, 0.02], [0.24, 0.06], [0.2525, 0.11], [0.2475, 0.5], [0.2355, 0.82]];
export function tankRadiusAt(y: number): number {
  for (let i = 1; i < TANK_PROFILE.length; i++) {
    const [r0, y0] = TANK_PROFILE[i - 1];
    const [r1, y1] = TANK_PROFILE[i];
    if (y <= y1) return r0 + ((r1 - r0) * (y - y0)) / (y1 - y0);
  }
  return TANK_PROFILE[TANK_PROFILE.length - 1][0];
}

function lathe(points: [number, number][], segments: number): THREE.BufferGeometry {
  return new THREE.LatheGeometry(points.map(([r, y]) => new THREE.Vector2(r, y)), segments);
}

/** Box without the listed faces (0:+x 1:-x 2:+y 3:-y 4:+z 5:-z) to save triangles on hidden sides. */
function box(w: number, h: number, d: number, omit: number[] = []): THREE.BufferGeometry {
  const g = new THREE.BoxGeometry(w, h, d).toNonIndexed();
  if (!omit.length) return g;
  const pos = g.getAttribute('position');
  const nor = g.getAttribute('normal');
  const p: number[] = [];
  const n: number[] = [];
  for (let f = 0; f < 6; f++) {
    if (omit.includes(f)) continue;
    for (let v = f * 6; v < f * 6 + 6; v++) {
      p.push(pos.getX(v), pos.getY(v), pos.getZ(v));
      n.push(nor.getX(v), nor.getY(v), nor.getZ(v));
    }
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
  out.setAttribute('normal', new THREE.Float32BufferAttribute(n, 3));
  return out;
}

export interface StrapParams {
  /** Radius of the loop around the tank (wall + fins + strap). */
  tankRadius: number;
  /** Radius of the loop around the pole at the strap height. */
  poleRadius: number;
  /** Tank axis to pole axis distance at the strap height. */
  centerDistance: number;
  width: number;
  thickness: number;
  segmentsPerCircle: number;
}

/**
 * Closed belt that encircles BOTH the tank and the pole: an arc around the tank, an arc around the
 * pole and two straight tangent runs between them. Loop lies in the XZ plane at y=0 of the geometry.
 * Returns the ribbon (outer + inner faces) and the outer-surface samples used to place perforations.
 */
export function buildStrap(p: StrapParams): { geometry: THREE.BufferGeometry; tankArc: { pos: THREE.Vector3; normal: THREE.Vector3 }[] } {
  const { tankRadius: rt, poleRadius: rp, centerDistance: d, width, thickness, segmentsPerCircle } = p;
  const gamma = Math.acos((rt - rp) / d);
  const step = (2 * Math.PI) / segmentsPerCircle;
  const loop: { c: THREE.Vector2; r: number; a: number }[] = [];
  const poleC = new THREE.Vector2(-d, 0);
  const tankC = new THREE.Vector2(0, 0);
  // tank arc: normals from pi-gamma going through 0 (the +X side) to -(pi-gamma)
  const tankSpan = 2 * (Math.PI - gamma);
  const nT = Math.max(2, Math.ceil(tankSpan / step));
  for (let i = 0; i <= nT; i++) loop.push({ c: tankC, r: rt, a: Math.PI - gamma - (tankSpan * i) / nT });
  // pole arc: normals from -(pi-gamma) (= pi+gamma) around pi to pi-gamma
  const poleSpan = 2 * gamma;
  const nP = Math.max(2, Math.ceil(poleSpan / step));
  for (let i = 0; i <= nP; i++) loop.push({ c: poleC, r: rp, a: Math.PI + gamma - (poleSpan * i) / nP });
  const tankArc: { pos: THREE.Vector3; normal: THREE.Vector3 }[] = [];
  const pos: number[] = [];
  const nor: number[] = [];
  const idx: number[] = [];
  const hw = width / 2;
  const N = loop.length;
  const pts = loop.map((s) => {
    const n = new THREE.Vector2(Math.cos(s.a), Math.sin(s.a));
    return { n, inner: s.c.clone().addScaledVector(n, s.r), outer: s.c.clone().addScaledVector(n, s.r + thickness) };
  });
  // Normals use the sample normal (smooth) which is exact on arcs and exact on the straight runs
  // because the tangent points share the tangent line's normal.
  for (let i = 0; i < N; i++) {
    const { n, inner, outer } = pts[i];
    // vertex order: outer-top, outer-bottom, inner-top, inner-bottom
    pos.push(outer.x, hw, outer.y, outer.x, -hw, outer.y, inner.x, hw, inner.y, inner.x, -hw, inner.y);
    nor.push(n.x, 0, n.y, n.x, 0, n.y, -n.x, 0, -n.y, -n.x, 0, -n.y);
  }
  for (let i = 0; i < N; i++) {
    const j = (i + 1) % N;
    const a = i * 4, b = j * 4;
    // outer face (faces +n): winding so the normal points outward
    idx.push(a, a + 1, b, a + 1, b + 1, b);
    // inner face
    idx.push(a + 2, b + 2, a + 3, a + 3, b + 2, b + 3);
  }
  for (let i = 0; i <= nT; i++) {
    const s = loop[i];
    const pt = pts[i];
    tankArc.push({ pos: new THREE.Vector3(pt.outer.x, 0, pt.outer.y), normal: new THREE.Vector3(Math.cos(s.a), 0, Math.sin(s.a)) });
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
  g.setIndex(idx);
  return { geometry: g, tankArc };
}

function quadGeometry(w: number, h: number): THREE.BufferGeometry {
  return new THREE.PlaneGeometry(w, h);
}

function socket(name: string, position: THREE.Vector3, dir: THREE.Vector3, zHint = new THREE.Vector3(0, 1, 0)): THREE.Object3D {
  const x = dir.clone().normalize();
  let z = zHint.clone().addScaledVector(x, -zHint.dot(x));
  if (z.lengthSq() < 1e-6) z = new THREE.Vector3(0, 0, 1);
  z.normalize();
  const y = new THREE.Vector3().crossVectors(z, x);
  const o = new THREE.Object3D();
  o.name = name;
  o.position.copy(position);
  o.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
  o.userData = { wireLeaves: '+X', up: '+Z', note: 'X = wire leaving direction; Z = world up where X is horizontal, else the horizontal hint' };
  return o;
}

const rad = THREE.MathUtils.degToRad;

export function createTransformerM01(options: TransformerOptions = {}): THREE.Group {
  const pass = options.pass ?? 'material-pass';
  const level = PASS_ORDER.indexOf(pass) + 1; // 1 macro, 2 +meso, 3 +micro, 4 materials
  const pole: PoleSpec = { ...DEFAULT_POLE, ...options.pole };
  const M = makeMaterials(options.neutral ?? level < 4);

  const root = new THREE.Group();
  root.name = 'transformer_m_01';
  const body = new THREE.Group();
  body.name = 'body';
  const mount = new THREE.Group();
  mount.name = 'mount';
  mount.userData = {
    poleDependent: true,
    note: 'Origin on the tank axis. Straps encircle pole and tank; rebuild with createTransformerM01({pole}) or scale to pole radius.',
    poleRadiusRef: pole.radius,
    poleAxisDistance: pole.axisDistance,
  };
  const arm = new THREE.Group();
  arm.name = 'arm';
  root.add(body, mount, arm);

  const add = (parent: THREE.Object3D, lvl: number, name: string, geo: THREE.BufferGeometry, role: Role, at: [number, number, number], rot: [number, number, number] = [0, 0, 0]) => {
    if (lvl > level && !(level === 4)) return null;
    geo.deleteAttribute('uv'); // project rule: no UVs
    const m = new THREE.Mesh(geo, M[role]);
    m.name = name;
    m.position.set(...at);
    m.rotation.set(...rot);
    parent.add(m);
    return m;
  };

  // ---------------- body (pole independent) ----------------
  add(body, 1, 'tank', lathe(TANK_PROFILE, 24), 'paint', [0, 0, 0]);
  add(body, 1, 'lid', lathe([[0.24, 0.82], [0.24, 0.848], [0.222, 0.852], [0.205, 0.872], [0, 0.88]], 24), 'paint', [0, 0, 0]);
  add(body, 2, 'base_plate', new THREE.CylinderGeometry(0.225, 0.225, 0.025, 24), 'dark', [0, 0.0125, 0]);

  // fin bank: 16 radial plates, theta -10..140 deg every 10 (theta from +X toward +Z), tops stepped
  if (level >= 1) {
    const finGeos: THREE.BufferGeometry[] = [];
    for (let k = 0; k < 16; k++) {
      const th = -10 + 10 * k;
      const top = 0.7 - 0.12 * Math.min(1, Math.abs(th - 95) / 55);
      const bottom = 0.15;
      const g = box(0.04, top - bottom, 0.039, [1]);
      g.translate(0.242, (top + bottom) / 2, 0);
      g.rotateY(-rad(th));
      finGeos.push(g);
    }
    const merged = mergeGeometries(finGeos);
    add(body, 1, 'fins', merged, 'paint', [0, 0, 0]);
  }

  // Wedge boot: side profile (u outward, v up) extruded across the tangent direction.
  const hood = (name: string, thDeg: number, y: number, len: number, hIn: number, hOut: number, width: number, r: number) => {
    const L = len / 2;
    const sh = new THREE.Shape();
    sh.moveTo(-L, -hIn * 0.8);
    sh.lineTo(L * 0.45, -hOut * 0.95);
    sh.lineTo(L * 0.92, -hOut * 0.55);
    sh.lineTo(L, hOut * 0.1);
    sh.lineTo(L * 0.8, hOut * 0.7);
    sh.lineTo(-L * 0.1, hIn * 1.0);
    sh.lineTo(-L, hIn * 1.0);
    sh.closePath();
    const g = new THREE.ExtrudeGeometry(sh, { depth: width, bevelEnabled: false });
    g.translate(0, 0, -width / 2);
    const d = new THREE.Vector3(Math.cos(rad(thDeg)), 0, Math.sin(rad(thDeg)));
    const up = new THREE.Vector3(0, 1, 0);
    const z = new THREE.Vector3().crossVectors(d, up);
    g.applyMatrix4(new THREE.Matrix4().makeBasis(d, up, z));
    return add(body, 1, name, g, 'paint', [r * d.x, y, r * d.z]);
  };
  hood('hood_a', 155, 0.72, 0.17, 0.075, 0.06, 0.13, 0.205);
  hood('hood_b', 25, 0.745, 0.15, 0.07, 0.055, 0.12, 0.2);
  add(body, 2, 'boot_a', new THREE.CylinderGeometry(0.035, 0.03, 0.04, 8), 'dark', [-0.2, 0.675, 0.125]);
  add(body, 2, 'sleeve_b', new THREE.CylinderGeometry(0.031, 0.031, 0.125, 12), 'porcelain', [0.225, 0.665, 0.125]);

  [[178, 0.83], [106, 0.85], [-15, 0.83]].forEach(([th, y], i) => {
    const px = 0.235 * Math.cos(rad(th));
    const pz = 0.235 * Math.sin(rad(th));
    const ring = new THREE.TorusGeometry(0.016, 0.0035, 3, 8);
    ring.rotateY(-rad(th) + Math.PI / 2);
    ring.translate(0, 0.058, 0);
    const brk = box(0.02, 0.12, 0.012, [3]);
    brk.translate(0, -0.03, 0);
    brk.rotateY(-rad(th) + Math.PI / 2);
    const g = mergeGeometries([ring, brk]);
    add(body, 3, `lug_${i + 1}`, g, 'galv', [px, y, pz]);
  });
  add(body, 3, 'plate_a', box(0.08, 0.09, 0.02, [5]), 'galv', [-0.053, 0.62, 0.249], [0, rad(-12), 0]);
  add(body, 3, 'plate_b', box(0.045, 0.052, 0.012, [5]), 'dark', [-0.02, 0.34, 0.282]);
  add(body, 3, 'seam', box(0.008, 0.2, 0.006, [5]), 'paint', [0.05, 0.72, tankRadiusAt(0.72) * Math.cos(Math.asin(0.05 / 0.24)) + 0.001], [0, rad(-12), 0]);

  const sockets = new THREE.Group();
  sockets.name = 'sockets_body';
  body.add(sockets);
  sockets.add(socket('socket_hood_0', new THREE.Vector3(-0.2, 0.655, 0.125), new THREE.Vector3(0, -1, 0), new THREE.Vector3(-1, 0, 0)));
  sockets.add(socket('socket_hood_1', new THREE.Vector3(0.225, 0.602, 0.125), new THREE.Vector3(0, -1, 0), new THREE.Vector3(1, 0, 0)));

  // ---------------- mount (pole dependent) ----------------
  const poleR = (y: number) => pole.radius + pole.taperPerM * y;
  const strap = (name: string, y: number, lvl: number) => {
    const wall = tankRadiusAt(Math.min(y, 0.82));
    const d = pole.axisDistance + 0; // axis distance is the mounting datum; taper changes the pole radius, not its axis
    const { geometry, tankArc } = buildStrap({
      tankRadius: wall + 0.036,
      poleRadius: poleR(y) + 0.002,
      centerDistance: d,
      width: 0.027,
      thickness: 0.004,
      segmentsPerCircle: 48,
    });
    add(mount, lvl, name, geometry, 'galv', [0, y, 0]);
    if (level >= 3) {
      // square perforations on the visible (+Z) half of the tank arc, every 6 deg
      const holes: THREE.BufferGeometry[] = [];
      for (let a = 3; a < 180; a += 6) {
        const n = new THREE.Vector3(Math.cos(rad(a)), 0, Math.sin(rad(a)));
        const q = quadGeometry(0.011, 0.011);
        q.rotateY(Math.atan2(n.x, n.z));
        const r = wall + 0.036 + 0.004 + 0.002;
        q.translate(r * n.x, 0, r * n.z);
        holes.push(q);
      }
      add(mount, 3, `${name}_holes`, mergeGeometries(holes), 'dark', [0, y, 0]);
    }
    void tankArc;
  };
  strap('band_1', 0.42, 1);
  strap('band_2', 0.2, 1);

  // arm stub bridges pole surface -> arm; its length absorbs the pole radius / taper
  {
    const y = -0.065;
    const poleSurface = -(pole.axisDistance - poleR(y));
    const x0 = poleSurface - 0.03; // embedded 0.03 into the pole
    const x1 = -0.22;
    const g = box(x1 - x0, 0.14, 0.05);
    add(mount, 2, 'stub', g, 'galv', [(x0 + x1) / 2, y, 0]);
  }

  // ---------------- arm (pole independent) ----------------
  {
    const web = box(0.92, 0.106, 0.006);
    const fl1 = box(0.92, 0.006, 0.05);
    fl1.translate(0, 0.05, -0.022);
    const fl2 = box(0.92, 0.006, 0.05);
    fl2.translate(0, -0.05, -0.022);
    web.translate(0, 0, 0.003);
    add(arm, 1, 'arm_main', mergeGeometries([web, fl1, fl2]), 'galv', [0.16, -0.093, 0.022]);
    if (level >= 3) {
      const cuts: THREE.BufferGeometry[] = [];
      [-0.02, 0.16, 0.32].forEach((x) => {
        const q = quadGeometry(0.045, 0.012);
        q.translate(x, 0.015, 0);
        cuts.push(q);
      });
      [-0.1, 0.08, 0.24, 0.41, 0.5, 0.56].forEach((x) => {
        const q = quadGeometry(0.014, 0.014);
        q.translate(x, -0.02, 0);
        cuts.push(q);
      });
      add(arm, 3, 'arm_slots', mergeGeometries(cuts), 'dark', [0, -0.093, 0.0305]);
    }
    add(arm, 1, 'arm_seat', box(0.41, 0.146, 0.04), 'galv', [-0.015, -0.073, -0.005]);
    add(arm, 1, 'arm_low', box(0.68, 0.049, 0.05), 'galv', [0.14, -0.17, 0]);
    add(arm, 2, 'brace', box(0.34, 0.03, 0.02), 'dark', [0.45, -0.005, -0.03], [0, 0, rad(15)]);

    // arrester: 12-sided, barrel + 5 petticoat skirts (double cones) + base
    const pts: [number, number][] = [[0, -0.018], [0.046, -0.018], [0.046, 0.045]];
    let y = 0.045;
    const skirt = (y0: number, rMax: number, h: number) => {
      pts.push([0.044, y0], [rMax, y0 + 0.006], [0.044, y0 + h]);
    };
    // lower skirts (2), clamp gap, upper skirts (3)
    skirt(0.055, 0.056, 0.018);
    skirt(0.09, 0.056, 0.018);
    pts.push([0.044, 0.125]);
    skirt(0.14, 0.056, 0.018);
    skirt(0.172, 0.056, 0.018);
    skirt(0.204, 0.056, 0.022);
    pts.push([0.034, 0.236], [0.0, 0.236]);
    void y;
    add(arm, 1, 'arr_body', lathe(pts, 12), 'porcelain', [0.62, 0, 0.03]);
    add(arm, 1, 'arr_cap', new THREE.ConeGeometry(0.031, 0.05, 12), 'dark', [0.62, 0.261, 0.03]);
    add(arm, 1, 'arr_base', new THREE.CylinderGeometry(0.054, 0.054, 0.058, 12), 'dark', [0.62, -0.047, 0.03]);
    const clampV = box(0.016, 0.19, 0.02, [5]);
    clampV.translate(0.035, 0, 0);
    const clampH = box(0.09, 0.014, 0.02, [5]);
    clampH.translate(-0.005, 0.08, 0);
    add(arm, 1, 'arr_clamp', mergeGeometries([clampV, clampH]), 'galv', [0.67, 0.04, 0.03]);
    add(arm, 1, 'end_box', box(0.13, 0.095, 0.05), 'galv', [0.755, 0.02, 0.03]);
    add(arm, 1, 'knob_rod', new THREE.CylinderGeometry(0.014, 0.014, 0.07, 8), 'galv', [0.85, 0.034, 0.03], [0, 0, Math.PI / 2]);
    // the disc faces the camera side partly (about 40 deg off the arm axis), as in the photo
    const disc = new THREE.CylinderGeometry(0.05, 0.05, 0.04, 12);
    disc.rotateZ(Math.PI / 2);
    disc.rotateY(-rad(40));
    add(arm, 1, 'knob', disc, 'porcelain', [0.882, 0.034, 0.03]);
    const core = new THREE.CylinderGeometry(0.02, 0.02, 0.046, 12);
    core.rotateZ(Math.PI / 2);
    core.rotateY(-rad(40));
    add(arm, 3, 'knob_core', core, 'dark', [0.882, 0.034, 0.03]);

    const s = new THREE.Group();
    s.name = 'sockets_arm';
    arm.add(s);
    s.add(socket('socket_arrester_0', new THREE.Vector3(0.62, 0.286, 0.03), new THREE.Vector3(1, 0.5, 0)));
    s.add(socket('socket_knob_0', new THREE.Vector3(0.88, 0.084, 0.03), new THREE.Vector3(0, 0, 1)));
  }

  root.userData = { asset: 'transformer_m_01', units: 'm', pass, groups: ['body', 'mount', 'arm'], pole };
  // Runtime maps are non-enumerable so GLTFExporter does not try to serialise Object3D references.
  const nodes: Record<string, THREE.Object3D> = {};
  const meshes: Record<string, THREE.Mesh> = {};
  const socketMap: Record<string, THREE.Object3D> = {};
  root.traverse((o) => {
    if (o === root) return;
    nodes[o.name] = o;
    if ((o as THREE.Mesh).isMesh) meshes[o.name] = o as THREE.Mesh;
    if (o.name.startsWith('socket_')) socketMap[o.name] = o;
  });
  Object.defineProperty(root.userData, 'sculptRuntime', {
    value: { nodes, meshes, sockets: socketMap, colliders: {}, destructionGroups: { body: [body], mount: [mount], arm: [arm] } },
    enumerable: false,
  });
  return root;
}

export function mergeGeometries(list: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const ni = list.map((g) => (g.index ? g.toNonIndexed() : g));
  let n = 0;
  ni.forEach((g) => (n += g.getAttribute('position').count));
  const pos = new Float32Array(n * 3);
  const nor = new Float32Array(n * 3);
  let o = 0;
  for (const g of ni) {
    if (!g.getAttribute('normal')) g.computeVertexNormals();
    pos.set(g.getAttribute('position').array as Float32Array, o * 3);
    nor.set(g.getAttribute('normal').array as Float32Array, o * 3);
    o += g.getAttribute('position').count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  return out;
}

export function triangleCount(obj: THREE.Object3D): number {
  let t = 0;
  obj.traverse((o) => {
    const m = o as THREE.Mesh;
    if (m.isMesh) {
      const g = m.geometry;
      const idx = g.getIndex();
      t += (idx ? idx.count : g.getAttribute('position').count) / 3;
    }
  });
  return t;
}
