/**
 * Fabrication-accurate 3D assembly of the hand, driven by simulator FK.
 *
 * Structure per finger link: two side plates + palmar/dorsal tendon guide
 * blocks + hinge pins (owned by the outer-fork link at each joint) + tip pad
 * on the distal link. The palm is a plate chassis: palmar/dorsal plates,
 * spacer columns, one tongue lug per finger and wrist tendon-guide bars.
 *
 * Tendon tubes are rendered as polylines threaded through the actual drilled
 * channels (channel offsets == simulation moment arms) and colored by live
 * tension. Parts register themselves in a PartsManifest for STL/DXF export.
 */

import * as THREE from 'three';
import type { HandParams } from '../sim/types';
import type { HandSimulator, FKFrame } from '../sim/simulator';
import { mat3FromEulerXYZ } from '../sim/math3';
import {
  computeLinkMech,
  guideBlock,
  hingePin,
  LinkMech,
  PartsManifest,
  PLATE_QUAT,
  GUIDE_QUAT,
  PIN_QUAT,
  LUG_QUAT,
  roundedRectPlate,
  stadiumPlate,
  tendonChannelX,
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

  // --------------------------------------------------------------- palm ---
  private buildPalm(params: HandParams): void {
    const s = params.scale;
    const mech = params.mech;
    const pt = mech.palmPlateThickness;
    const P = params.palm.thickness;

    // chassis plates end proximal of the MCP axes so flexing proximal links
    // never sweep through them; the thumb corner stays clear of the plates
    // (its lug cantilevers out) so the rotated thumb fork can move freely
    const firstLinks = params.fingers.map(
      (f) => computeLinkMech(f, mech)[0],
    );
    const mcpY = Math.min(
      ...params.fingers
        .filter((f) => f.name !== 'thumb')
        .map((f) => f.basePosition[1]),
    );
    const maxEndR = Math.max(...firstLinks.map((lk) => lk.endR));
    const topY = mcpY - maxEndR - 0.001 * s;
    const Lp = 0.105 * s;
    const cy = topY - Lp / 2;
    const cx = 0.004 * s;
    const Wp = 0.088 * s;

    const spacerXY: { x: number; y: number }[] = [
      { x: cx - Wp * 0.36, y: cy - Lp * 0.36 },
      { x: cx + Wp * 0.36, y: cy - Lp * 0.36 },
      { x: cx - Wp * 0.32, y: cy + Lp * 0.36 },
      { x: cx + Wp * 0.36, y: cy + Lp * 0.32 },
    ];
    const spacerR = 0.004 * s;
    const boltR = 0.0016 * s;

    const holes = spacerXY.map((p) => ({ x: p.x - cx, y: p.y - cy, r: boltR }));
    for (const [name, z] of [
      ['palm-plate-palmar', -P / 2],
      ['palm-plate-dorsal', P / 2 - pt],
    ] as const) {
      const { geo, profile } = roundedRectPlate(
        Wp, Lp, 0.009 * s, pt, holes, name,
      );
      const mesh = new THREE.Mesh(geo, PALM_MAT);
      mesh.position.set(cx, cy, z);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.partsRoot.add(mesh);
      this.manifest.register(name, geo, 1, profile);
    }

    // spacer columns between the plates
    const spacerLen = P - 2 * pt;
    const spacerGeo = new THREE.CylinderGeometry(spacerR, spacerR, spacerLen, 16);
    spacerGeo.rotateX(Math.PI / 2); // axis → Z
    this.manifest.register('palm-spacer', spacerGeo, spacerXY.length);
    for (const p of spacerXY) {
      const mesh = new THREE.Mesh(spacerGeo, GUIDE_MAT);
      mesh.position.set(p.x, p.y, 0);
      mesh.castShadow = true;
      this.partsRoot.add(mesh);
    }

    // wrist tendon-guide bars (channels for every tendon actuator)
    this.buildWristBars(params, cy - Lp / 2);

    // per-finger tongue lugs, slim enough to slide between the chassis plates
    const lugEndR = (P - 2 * pt) / 2 - 0.0002 * s;
    params.fingers.forEach((finger) => {
      const links = computeLinkMech(finger, mech);
      const clr = mech.hingeClearance;
      const tongueW = links[0].G - 2 * clr;
      const lugLen = 0.024 * s;
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

  private buildWristBars(params: HandParams, yWrist: number): void {
    const s = params.scale;
    const mech = params.mech;
    const chR = mech.channelDiameter / 2;
    const barLen = 0.011 * s;
    const y0 = yWrist + 0.006 * s;
    // wide enough to carry the thumb channels beyond the chassis plates
    const xMin = -0.050 * s;
    const xMax = 0.046 * s;

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
      const name = side === -1 ? 'wrist-bar-flexor' : 'wrist-bar-extensor';
      const geo = guideBlock(z0, z1, xMin, xMax, barLen, channels);
      const mesh = new THREE.Mesh(geo, GUIDE_MAT);
      mesh.quaternion.copy(GUIDE_QUAT);
      mesh.position.set(0, y0, 0);
      mesh.castShadow = true;
      this.partsRoot.add(mesh);
      this.manifest.register(name, geo, 1);
    }
    this.wristBarY0 = y0;
    this.wristBarY1 = y0 + barLen;
  }

  private wristBarY0 = 0;
  private wristBarY1 = 0;

  // ------------------------------------------------------------- finger ---
  private buildFinger(params: HandParams, fi: number): void {
    const finger = params.fingers[fi];
    const mech = params.mech;
    const s = params.scale;
    const links = computeLinkMech(finger, mech);
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
          this.wristBarY0,
          side * 0.0035 * s,
        ),
        barExitY: this.wristBarY1,
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

    // tendons: straight runs bar → pulleys → channels, colored by tension
    const tensions = this.sim.tension;
    const maxSafe = this.sim.maxSafeTensions;
    let ti = 0;
    for (const tv of this.tendonViews) {
      const finger = params.fingers[tv.fingerIndex];
      const frame = frames[tv.fingerIndex];
      const links = computeLinkMech(finger, params.mech);
      const pts = this.tendonPolyline(tv, frame, links);
      const path = new THREE.CurvePath<THREE.Vector3>();
      for (let i = 0; i < pts.length - 1; i++) {
        path.add(new THREE.LineCurve3(pts[i], pts[i + 1]));
      }
      const geo = new THREE.TubeGeometry(
        path, pts.length * 4, 0.0011 * params.scale, 5, false,
      );
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
    frame: FKFrame,
    links: LinkMech[],
  ): THREE.Vector3[] {
    const pts: THREE.Vector3[] = [];
    // wrist guide bar (palm frame == world)
    pts.push(tv.barPoint.clone());
    pts.push(new THREE.Vector3(tv.barPoint.x, tv.barExitY, tv.barPoint.z));

    const last = tv.jointLocals.length - 1;
    for (let k = 0; k < tv.jointLocals.length; k++) {
      const lj = tv.jointLocals[k];
      const arm = tv.arms[k];
      const lk = links[lj];
      // pulley point at the joint pin, offset by the moment arm
      pts.push(this.linkPoint(frame, lj, tv.channelX, 0, tv.side * arm));
      // through the drilled channel of this link's guide block
      const yEnd = k === last ? (lk.guideY0 + lk.guideY1) / 2 : lk.guideY1;
      pts.push(this.linkPoint(frame, lj, tv.channelX, lk.guideY0, tv.side * arm));
      pts.push(this.linkPoint(frame, lj, tv.channelX, yEnd, tv.side * arm));
    }
    return pts;
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
