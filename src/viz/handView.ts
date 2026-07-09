/**
 * Fabrication-accurate 3D assembly of the hand, driven by simulator FK.
 *
 * Structure per finger link: two side plates + palmar/dorsal tendon guide
 * blocks + hinge pins (owned by the outer-fork link at each joint) + tip pad
 * on the distal link. The palm is an open two-rail frame: side rails run
 * from the wrist to the fingers, a front crossbar carries the finger lugs,
 * and a rear crossbar (palmar + dorsal termination bars) carries the
 * drilled tendon-end channels — no enclosing box, no spacer columns.
 *
 * Tendon tubes are threaded through the actual drilled channels (channel
 * offsets == simulation moment arms), wrapping each joint's pin along a
 * pulley arc (not a straight chord) so they never cut through the guide
 * blocks at high flexion. The tube mesh is built with an explicit per-point
 * orientation quaternion (slerped straight from the joints' own FK frames)
 * rather than three.js's TubeGeometry + computeFrenetFrames: that Frenet
 * computation can still flip the cross-section on nearly-parallel or
 * degenerate segments (confirmed by a regression sweep — it flipped on
 * several fingers at high flexion, not just the thumb), so it can't be
 * trusted to not reintroduce the twisting bug. Deriving frames straight
 * from the rotating link frames instead makes a flip structurally
 * impossible: consecutive orientations are always either identical or a
 * monotonic slerp between two fixed endpoints. Parts register themselves
 * in a PartsManifest for STL/DXF export.
 */

import * as THREE from 'three';
import type { FingerParams, HandParams } from '../sim/types';
import type { HandSimulator, FKFrame } from '../sim/simulator';
import { mat3FromEulerXYZ, type Mat3 } from '../sim/math3';
import {
  computeLinkMech,
  LinkMech,
  tendonChannelX,
} from '../sim/mechGeometry';
import {
  guideBlock,
  hingePin,
  PartsManifest,
  PLATE_QUAT,
  GUIDE_QUAT,
  PIN_QUAT,
  LUG_QUAT,
  roundedRectPlate,
  stadiumPlate,
  tipPad,
} from './parts';

const PLATE_MAT = new THREE.MeshStandardMaterial({
  color: 0xb9c4d2, metalness: 0.75, roughness: 0.35,
});
const GUIDE_MAT = new THREE.MeshStandardMaterial({
  color: 0x3c4d63, metalness: 0.15, roughness: 0.7,
});
const PIN_MAT = new THREE.MeshStandardMaterial({
  color: 0x9aa3ab, metalness: 0.9, roughness: 0.25,
});
const PALM_MAT = new THREE.MeshStandardMaterial({
  color: 0x8e9dae, metalness: 0.7, roughness: 0.4,
});
const TIP_MAT = new THREE.MeshStandardMaterial({
  color: 0x2e3b4d, metalness: 0.05, roughness: 0.9,
});

const COLOR_COLD = new THREE.Color(0x3282e0);
const COLOR_WARM = new THREE.Color(0xf5c542);
const COLOR_HOT = new THREE.Color(0xe8442e);

interface TendonView {
  mesh: THREE.Mesh;
  material: THREE.MeshStandardMaterial;
  fingerIndex: number;
  side: 1 | -1; // -1 palmar (flexor), +1 dorsal (extensor)
  jointLocals: number[];
  arms: number[];
  channelX: number;
  barPoint: THREE.Vector3; // wrist guide-bar entry (palm frame)
  barExitY: number;
}

const tmpM4 = new THREE.Matrix4();

export class HandView {
  readonly group = new THREE.Group();
  /** Printable parts only (exported to STL); tendons live outside. */
  readonly partsRoot = new THREE.Group();
  private tendonRoot = new THREE.Group();
  manifest = new PartsManifest();

  private linkGroups: THREE.Group[][] = [];
  private tendonViews: TendonView[] = [];
  private sim: HandSimulator;

  constructor(sim: HandSimulator, parent: THREE.Object3D) {
    this.sim = sim;
    parent.add(this.group);
    this.group.add(this.partsRoot);
    this.group.add(this.tendonRoot);
    this.rebuild();
  }

  rebuild(): void {
    this.partsRoot.traverse((o) => {
      if (o instanceof THREE.Mesh) o.geometry.dispose();
    });
    this.partsRoot.clear();
    this.tendonRoot.traverse((o) => {
      if (o instanceof THREE.Mesh) o.geometry.dispose();
    });
    this.tendonRoot.clear();
    this.manifest.dispose();
    this.manifest = new PartsManifest();
    this.pinGeoCache.clear(); // geometries were disposed via the manifest
    this.linkGroups = [];
    this.tendonViews = [];

    const params = this.sim.params;
    this.buildPalm(params);
    params.fingers.forEach((finger, fi) => this.buildFinger(params, fi));
    this.update();
  }

  // -------------------------------------------------------------- frame ---
  /**
   * Open two-rail frame: side rails run from the wrist to the fingers, a
   * front crossbar carries the finger lugs, a rear crossbar (palmar +
   * dorsal termination bars) carries the drilled tendon-end channels.
   * Replaces the earlier plate-sandwich-with-spacers chassis, which read as
   * a solid box for no structural reason — a frame is lighter, simpler to
   * print/cut (4 flat parts instead of 12), and matches how the tendons
   * actually need to run: along the outside of open rails, not sandwiched
   * between two plates.
   */
  private buildPalm(params: HandParams): void {
    const s = params.scale;
    const mech = params.mech;
    const pt = mech.palmPlateThickness;

    // front rail end sits proximal of the MCP axes so flexing proximal
    // links never sweep through it
    const firstLinks = params.fingers.map(
      (f) => computeLinkMech(f.phalanges, mech)[0],
    );
    const mcpY = Math.min(
      ...params.fingers
        .filter((f) => f.name !== 'thumb')
        .map((f) => f.basePosition[1]),
    );
    const maxEndR = Math.max(...firstLinks.map((lk) => lk.endR));
    const railY1 = mcpY - maxEndR - 0.001 * s; // front (finger) end
    const railLen = 0.105 * s;
    const railY0 = railY1 - railLen; // rear (wrist) end
    const cx = 0.004 * s;
    const Wp = 0.088 * s; // rail spacing (centerline to centerline)
    const railH = 0.011 * s;
    const railT = pt * 1.6;
    const railEndR = railH / 2;
    const boltR = 0.0016 * s;

    // two side rails
    const { geo: railGeo, profile: railProfile } = stadiumPlate(
      railLen, railEndR, boltR, boltR, railT, 'frame-rail',
    );
    this.manifest.register('frame-rail', railGeo, 2, railProfile);
    for (const side of [-1, 1] as const) {
      const mesh = new THREE.Mesh(railGeo, PALM_MAT);
      mesh.quaternion.copy(PLATE_QUAT);
      mesh.position.set(cx + side * (Wp / 2) - railT / 2, railY0, 0);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.partsRoot.add(mesh);
    }

    // front crossbar: holds the finger lugs
    const crossDepth = 0.016 * s;
    const crossW = Wp + railH * 2 + 0.006 * s;
    {
      const { geo, profile } = roundedRectPlate(
        crossW, crossDepth, 0.006 * s, pt,
        [
          { x: -Wp / 2, y: 0, r: boltR },
          { x: Wp / 2, y: 0, r: boltR },
        ],
        'frame-crossbar-front',
      );
      const mesh = new THREE.Mesh(geo, PALM_MAT);
      mesh.position.set(cx, railY1 - crossDepth / 2, 0);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.partsRoot.add(mesh);
      this.manifest.register('frame-crossbar-front', geo, 1, profile);
    }

    // rear crossbar: palmar + dorsal tendon-termination bars
    this.buildTerminationBars(params, cx, railY0);

    // per-finger tongue lugs, pinned to the front crossbar
    params.fingers.forEach((finger) => {
      const links = computeLinkMech(finger.phalanges, mech);
      const clr = mech.hingeClearance;
      const tongueW = links[0].G - 2 * clr;
      const lugLen = 0.024 * s;
      const lugEndR = links[0].endR * 0.9; // nests inside link0's fork
      const name = `${finger.name}/palm-lug`;
      const { geo, profile } = stadiumPlate(
        lugLen,
        Math.min(lugEndR, links[0].endR),
        links[0].holeR,
        boltR, // mounting hole at the buried end
        tongueW,
        name,
      );
      const base = new THREE.Group();
      base.position.fromArray(finger.basePosition);
      const m = mat3FromEulerXYZ(finger.baseEulerXYZ);
      tmpM4.set(
        m[0], m[3], m[6], 0,
        m[1], m[4], m[7], 0,
        m[2], m[5], m[8], 0,
        0, 0, 0, 1,
      );
      base.quaternion.setFromRotationMatrix(tmpM4);
      const mesh = new THREE.Mesh(geo, PALM_MAT);
      mesh.quaternion.copy(LUG_QUAT); // stadium extends toward local −Y
      mesh.position.x = -tongueW / 2; // center extrusion on the finger plane
      mesh.castShadow = true;
      base.add(mesh);
      this.partsRoot.add(base);
      this.manifest.register(name, geo, 1, profile);
    });
  }

  /** Rear crossbar: drilled palmar/dorsal bars where every tendon ends. */
  private buildTerminationBars(params: HandParams, cx: number, railY0: number): void {
    const s = params.scale;
    const mech = params.mech;
    const chR = mech.channelDiameter / 2;
    const barLen = 0.011 * s;
    const y1 = railY0; // flush with the rails' rear end
    const y0 = y1 - barLen; // extends further back, away from the fingers
    // wide enough to carry the thumb channels beyond the rails
    const xMin = cx - 0.050 * s;
    const xMax = cx + 0.046 * s;

    for (const side of [-1, 1] as const) {
      const channels: { z: number; x: number; r: number }[] = [];
      params.fingers.forEach((finger) => {
        const hasFds = finger.tendons.some((t) => t.name.endsWith('.fds'));
        for (const tendon of finger.tendons) {
          const tSide = tendon.kind === 'flexor' ? -1 : 1;
          if (tSide !== side) continue;
          const xOff = tendon.kind === 'flexor' && hasFds
            ? (tendon.name.endsWith('.fds') ? 0.0022 * s : -0.0022 * s)
            : 0;
          channels.push({
            z: side * 0.0035 * s,
            x: finger.basePosition[0] + xOff,
            r: chR,
          });
        }
      });
      const z0 = side * 0.001 * s;
      const z1 = side * 0.006 * s;
      const name = side === -1 ? 'termination-bar-flexor' : 'termination-bar-extensor';
      const geo = guideBlock(z0, z1, xMin, xMax, barLen, channels);
      const mesh = new THREE.Mesh(geo, GUIDE_MAT);
      mesh.quaternion.copy(GUIDE_QUAT);
      mesh.position.set(0, y0, 0);
      mesh.castShadow = true;
      this.partsRoot.add(mesh);
      this.manifest.register(name, geo, 1);
    }
    this.terminationY0 = y0;
    this.terminationY1 = y1;
  }

  private terminationY0 = 0;
  private terminationY1 = 0;

  // ------------------------------------------------------------- finger ---
  private buildFinger(params: HandParams, fi: number): void {
    const finger = params.fingers[fi];
    const mech = params.mech;
    const s = params.scale;
    const links = computeLinkMech(finger.phalanges, mech);
    const hasFds = finger.tendons.some((t) => t.name.endsWith('.fds'));
    const groups: THREE.Group[] = [];

    links.forEach((lk, li) => {
      const g = new THREE.Group();
      this.partsRoot.add(g);
      groups.push(g);
      const isLast = li === links.length - 1;

      // --- side plates ---
      const plateName = `${finger.name}/plate-${li}`;
      const { geo: plateGeo, profile } = stadiumPlate(
        lk.L,
        lk.endR,
        lk.holeR,
        isLast ? 0 : lk.holeR,
        lk.plateT,
        plateName,
      );
      this.manifest.register(plateName, plateGeo, 2, profile);
      for (const sideX of [-1, 1] as const) {
        const mesh = new THREE.Mesh(plateGeo, PLATE_MAT);
        mesh.quaternion.copy(PLATE_QUAT);
        mesh.position.x = sideX * lk.plateCx - lk.plateT / 2;
        mesh.castShadow = true;
        g.add(mesh);
      }

      // --- guide blocks with drilled tendon channels ---
      const chR = mech.channelDiameter / 2;
      const wall = mech.guideWall;
      const guideLen = lk.guideY1 - lk.guideY0;

      // palmar: one channel per flexor tendon still routed through this link
      const palmarChannels: { z: number; x: number; r: number }[] = [];
      for (const tendon of finger.tendons) {
        if (tendon.kind !== 'flexor') continue;
        const entry = tendon.routing.find((r) => r.jointIndex === li);
        if (!entry) continue;
        palmarChannels.push({
          z: -entry.momentArm,
          x: tendonChannelX(tendon, lk, hasFds),
          r: chR,
        });
      }
      if (palmarChannels.length > 0) {
        const zDeep = Math.min(...palmarChannels.map((c) => c.z - c.r)) - wall;
        const geo = guideBlock(zDeep, -lk.H * 0.1, -lk.G / 2, lk.G / 2, guideLen, palmarChannels);
        const mesh = new THREE.Mesh(geo, GUIDE_MAT);
        mesh.quaternion.copy(GUIDE_QUAT);
        mesh.position.set(0, lk.guideY0, 0);
        mesh.castShadow = true;
        g.add(mesh);
        this.manifest.register(`${finger.name}/guide-palmar-${li}`, geo, 1);
      }

      // dorsal: extensor channel
      const ext = finger.tendons.find((t) => t.kind === 'extensor');
      const extEntry = ext?.routing.find((r) => r.jointIndex === li);
      if (extEntry) {
        const zTop = extEntry.momentArm + chR + wall;
        const geo = guideBlock(
          lk.H * 0.1, zTop, -lk.G / 2, lk.G / 2, guideLen,
          [{ z: extEntry.momentArm, x: 0, r: chR }],
        );
        const mesh = new THREE.Mesh(geo, GUIDE_MAT);
        mesh.quaternion.copy(GUIDE_QUAT);
        mesh.position.set(0, lk.guideY0, 0);
        mesh.castShadow = true;
        g.add(mesh);
        this.manifest.register(`${finger.name}/guide-dorsal-${li}`, geo, 1);
      }

      // --- fingertip pad on the distal link ---
      if (isLast) {
        const geo = tipPad(lk.endR * 0.92, lk.G);
        const mesh = new THREE.Mesh(geo, TIP_MAT);
        mesh.quaternion.copy(PIN_QUAT);
        mesh.position.set(0, lk.L * 0.97, 0);
        mesh.castShadow = true;
        g.add(mesh);
        this.manifest.register(`${finger.name}/tip-pad`, geo, 1);
      }

      // --- hinge pins: owned by the outer-fork link at each joint ---
      const pinR = mech.pinDiameter / 2;
      const headT = 0.0012 * s;
      const addPin = (yLocal: number, outerW: number) => {
        const len = outerW + 2 * mech.hingeClearance;
        const name = `pin-d${(mech.pinDiameter * 1000).toFixed(1)}-l${(len * 1000).toFixed(1)}`;
        const geo = this.pinGeo(name, pinR, len, headT);
        const mesh = new THREE.Mesh(geo, PIN_MAT);
        mesh.quaternion.copy(PIN_QUAT);
        mesh.position.set(-(len / 2 + headT), yLocal, 0);
        mesh.castShadow = true;
        g.add(mesh);
        this.manifest.register(name, geo, 1);
      };
      if (lk.outer) {
        addPin(0, lk.outerW); // proximal pin (through palm lug / inner link)
        if (!isLast) addPin(lk.L, lk.outerW); // distal pin through next link
      }
      void li;
    });

    this.linkGroups.push(groups);

    // --- tendon tubes threaded through the channels ---
    finger.tendons.forEach((tendon) => {
      const material = new THREE.MeshStandardMaterial({
        color: 0x3282e0, roughness: 0.35, metalness: 0.1,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
      mesh.frustumCulled = false;
      this.tendonRoot.add(mesh);
      const side = tendon.kind === 'flexor' ? -1 : 1;
      const xOff = tendonChannelX(tendon, links[0], hasFds);
      this.tendonViews.push({
        mesh,
        material,
        fingerIndex: fi,
        side: side as 1 | -1,
        jointLocals: tendon.routing.map((r) => r.jointIndex),
        arms: tendon.routing.map((r) => r.momentArm),
        channelX: xOff,
        barPoint: new THREE.Vector3(
          finger.basePosition[0] + xOff,
          this.terminationY0,
          side * 0.0035 * s,
        ),
        barExitY: this.terminationY1,
      });
    });
  }

  /** Cache pin geometries by name so identical pins share one geometry. */
  private pinGeoCache = new Map<string, THREE.BufferGeometry>();
  private pinGeo(name: string, r: number, len: number, headT: number): THREE.BufferGeometry {
    let geo = this.pinGeoCache.get(name);
    if (!geo) {
      geo = hingePin(r, len, r * 1.9, headT);
      this.pinGeoCache.set(name, geo);
    }
    return geo;
  }

  // ------------------------------------------------------------- update ---
  update(): void {
    const frames = this.sim.fkFrames;
    const params = this.sim.params;

    // pose link groups straight from FK
    for (let fi = 0; fi < this.linkGroups.length; fi++) {
      const frame = frames[fi];
      this.linkGroups[fi].forEach((g, li) => {
        const p = frame.jointPos[li];
        const m = frame.jointMat[li];
        g.position.set(p[0], p[1], p[2]);
        tmpM4.set(
          m[0], m[3], m[6], 0,
          m[1], m[4], m[7], 0,
          m[2], m[5], m[8], 0,
          0, 0, 0, 1,
        );
        g.quaternion.setFromRotationMatrix(tmpM4);
      });
    }

    // tendons: bar → pulley arcs → channels, oriented-tube mesh, tension-colored
    const tensions = this.sim.tension;
    const maxSafe = this.sim.maxSafeTensions;
    let ti = 0;
    for (const tv of this.tendonViews) {
      const finger = params.fingers[tv.fingerIndex];
      const frame = frames[tv.fingerIndex];
      const links = computeLinkMech(finger.phalanges, params.mech);
      const { pts, quats } = this.tendonPolyline(tv, finger, frame, links);
      const geo = this.buildOrientedTube(pts, quats, 0.0011 * params.scale, 6);
      tv.mesh.geometry.dispose();
      tv.mesh.geometry = geo;

      const t01 = Math.min(1, tensions[ti] / maxSafe[ti]);
      if (t01 < 0.5) {
        tv.material.color.lerpColors(COLOR_COLD, COLOR_WARM, t01 * 2);
      } else {
        tv.material.color.lerpColors(COLOR_WARM, COLOR_HOT, (t01 - 0.5) * 2);
      }
      tv.material.emissive.copy(tv.material.color).multiplyScalar(t01 * 0.45);
      ti++;
    }
  }

  private tendonPolyline(
    tv: TendonView,
    finger: FingerParams,
    frame: FKFrame,
    links: LinkMech[],
  ): { pts: THREE.Vector3[]; quats: THREE.Quaternion[] } {
    const pts: THREE.Vector3[] = [];
    const quats: THREE.Quaternion[] = [];
    const baseMat = mat3FromEulerXYZ(finger.baseEulerXYZ);
    const baseQuat = this.quatFromMat3(baseMat);

    // termination bar (palm frame == world)
    pts.push(tv.barPoint.clone());
    quats.push(baseQuat);
    pts.push(new THREE.Vector3(tv.barPoint.x, tv.barExitY, tv.barPoint.z));
    quats.push(baseQuat);

    const last = tv.jointLocals.length - 1;
    for (let k = 0; k < tv.jointLocals.length; k++) {
      const lj = tv.jointLocals[k];
      const arm = tv.arms[k];
      const lk = links[lj];
      // the pulley: instead of a straight chord from the previous link's
      // frame across to this joint's (which cuts inside the guide block at
      // high flexion — the "melting into the parts" artifact), sample an
      // arc that slerps from the proximal link's orientation to this
      // link's, so the tendon visually wraps the hinge pin like a real
      // cable over a pulley.
      const parentMat = lj === 0 ? baseMat : frame.jointMat[lj - 1];
      const childMat = frame.jointMat[lj];
      const arc = this.pulleyArc(parentMat, childMat, frame.jointPos[lj], tv.channelX, tv.side * arm);
      pts.push(...arc.pts);
      quats.push(...arc.quats);

      // through the drilled channel of this link's guide block — the same
      // orientation as the arc's last sample (this link's own frame), so
      // there is no seam between "wrapping the pin" and "inside the bore"
      const childQuat = arc.quats[arc.quats.length - 1];
      const yEnd = k === last ? (lk.guideY0 + lk.guideY1) / 2 : lk.guideY1;
      pts.push(this.linkPoint(frame, lj, tv.channelX, lk.guideY0, tv.side * arm));
      quats.push(childQuat);
      pts.push(this.linkPoint(frame, lj, tv.channelX, yEnd, tv.side * arm));
      quats.push(childQuat);
    }
    return { pts, quats };
  }

  /** Points + orientations on the arc a cable traces wrapping a joint's pin. */
  private pulleyArc(
    parentMat: Mat3,
    childMat: Mat3,
    jointPos: readonly [number, number, number],
    x: number,
    z: number,
    samples = 4,
  ): { pts: THREE.Vector3[]; quats: THREE.Quaternion[] } {
    const qA = this.quatFromMat3(parentMat);
    const qB = this.quatFromMat3(childMat);
    const p = new THREE.Vector3(jointPos[0], jointPos[1], jointPos[2]);
    const local = new THREE.Vector3(x, 0, z);
    const pts: THREE.Vector3[] = [];
    const quats: THREE.Quaternion[] = [];
    for (let i = 0; i <= samples; i++) {
      const q = qA.clone().slerp(qB, i / samples);
      pts.push(local.clone().applyQuaternion(q).add(p));
      quats.push(q);
    }
    return { pts, quats };
  }

  private static readonly UNIT_X = new THREE.Vector3(1, 0, 0);
  private static readonly UNIT_Z = new THREE.Vector3(0, 0, 1);

  /**
   * Tube mesh built ring-by-ring from explicit per-point orientations
   * instead of THREE.TubeGeometry's computeFrenetFrames — see the class
   * doc comment for why the latter isn't safe to rely on here.
   */
  private buildOrientedTube(
    pts: THREE.Vector3[],
    quats: THREE.Quaternion[],
    radius: number,
    radialSegments: number,
  ): THREE.BufferGeometry {
    const n = pts.length;
    const positions = new Float32Array(n * radialSegments * 3);
    const normals = new Float32Array(n * radialSegments * 3);
    const xAxis = new THREE.Vector3();
    const zAxis = new THREE.Vector3();
    for (let i = 0; i < n; i++) {
      xAxis.copy(HandView.UNIT_X).applyQuaternion(quats[i]);
      zAxis.copy(HandView.UNIT_Z).applyQuaternion(quats[i]);
      for (let j = 0; j < radialSegments; j++) {
        const theta = (j / radialSegments) * Math.PI * 2;
        const c = Math.cos(theta);
        const s = Math.sin(theta);
        const nx = xAxis.x * c + zAxis.x * s;
        const ny = xAxis.y * c + zAxis.y * s;
        const nz = xAxis.z * c + zAxis.z * s;
        const idx = (i * radialSegments + j) * 3;
        positions[idx] = pts[i].x + nx * radius;
        positions[idx + 1] = pts[i].y + ny * radius;
        positions[idx + 2] = pts[i].z + nz * radius;
        normals[idx] = nx;
        normals[idx + 1] = ny;
        normals[idx + 2] = nz;
      }
    }
    const indices: number[] = [];
    for (let i = 0; i < n - 1; i++) {
      for (let j = 0; j < radialSegments; j++) {
        const jn = (j + 1) % radialSegments;
        const a = i * radialSegments + j;
        const b = (i + 1) * radialSegments + j;
        const c = (i + 1) * radialSegments + jn;
        const d = i * radialSegments + jn;
        indices.push(a, b, d, b, c, d);
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    geo.setIndex(indices);
    return geo;
  }

  private quatFromMat3(m: Mat3): THREE.Quaternion {
    tmpM4.set(
      m[0], m[3], m[6], 0,
      m[1], m[4], m[7], 0,
      m[2], m[5], m[8], 0,
      0, 0, 0, 1,
    );
    return new THREE.Quaternion().setFromRotationMatrix(tmpM4);
  }

  private linkPoint(
    frame: FKFrame,
    li: number,
    x: number,
    y: number,
    z: number,
  ): THREE.Vector3 {
    const p = frame.jointPos[li];
    const m = frame.jointMat[li];
    return new THREE.Vector3(
      p[0] + m[0] * x + m[3] * y + m[6] * z,
      p[1] + m[1] * x + m[4] * y + m[7] * z,
      p[2] + m[2] * x + m[5] * y + m[8] * z,
    );
  }
}
