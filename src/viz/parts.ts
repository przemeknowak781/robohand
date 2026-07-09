/**
 * Parametric factory of fabrication-ready mechanical parts.
 *
 * Every phalanx link is built from real manufacturable pieces:
 *   - two flat side plates ("płaskowniki"): stadium-outline flat bars with
 *     hinge-pin holes at both ends (2D profile → extrusion, CNC/DXF-ready)
 *   - a palmar and a dorsal guide block joining the plates, with drilled
 *     tendon channels placed exactly at the simulation moment arms
 *   - hinge pins with a head (clevis-pin style)
 * Links alternate outer-fork / inner-fork so consecutive links nest with a
 * printable clearance; the palm carries one tongue lug per finger.
 *
 * RAW geometries are created in their print/CNC orientation (plates and palm
 * flat on XY, channels vertical); the assembly re-orients them with the
 * PLATE/GUIDE/PIN quaternions below. This lets the exporters reuse raw
 * geometry for a lay-flat parts kit and 2D profiles for DXF.
 */

import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import type { FingerParams, MechParams, TendonParams } from '../sim/types';

/** shape-space → link-space: x̂→ŷ (length), ŷ→ẑ (height), ẑ→x̂ (thickness) */
export const PLATE_QUAT = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(1, 1, 1).normalize(),
  (2 * Math.PI) / 3,
);
/** shape-space → link-space: x̂→ẑ (height), ŷ→x̂ (width), ẑ→ŷ (length) */
export const GUIDE_QUAT = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(1, 1, 1).normalize(),
  (-2 * Math.PI) / 3,
);
/** raw pin axis Z → link hinge axis X */
export const PIN_QUAT = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(0, 1, 0),
  Math.PI / 2,
);
/** extra shape-space Z flip (for lugs that extend toward −Y). */
const SHAPE_Z_PI = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(0, 0, 1),
  Math.PI,
);
export const LUG_QUAT = new THREE.Quaternion().multiplyQuaternions(
  PLATE_QUAT,
  SHAPE_Z_PI,
);

/** 2D profile of a flat part, for DXF/CNC export (meters, part frame). */
export interface PlateProfile {
  label: string;
  outline: { x: number; y: number }[];
  circles: { x: number; y: number; r: number }[];
  thickness: number;
  count: number;
}

/** One fabricable part: raw geometry is already in print orientation. */
export interface PartEntry {
  name: string;
  raw: THREE.BufferGeometry;
  count: number;
  profile?: PlateProfile;
}

/** Registry of unique parts collected while building the assembly. */
export class PartsManifest {
  entries = new Map<string, PartEntry>();

  register(
    name: string,
    raw: THREE.BufferGeometry,
    count = 1,
    profile?: PlateProfile,
  ): void {
    const existing = this.entries.get(name);
    if (existing) {
      existing.count += count;
      if (existing.profile) existing.profile.count = existing.count;
    } else {
      const p = profile ? { ...profile, count } : undefined;
      this.entries.set(name, { name, raw, count, profile: p });
    }
  }

  list(): PartEntry[] {
    return [...this.entries.values()];
  }

  profiles(): PlateProfile[] {
    return this.list()
      .filter((e) => e.profile)
      .map((e) => e.profile!);
  }

  dispose(): void {
    for (const e of this.entries.values()) e.raw.dispose();
    this.entries.clear();
  }
}

/** Stadium (rounded-bar) outline of given length/end radius with pin holes. */
export function stadiumPlate(
  length: number,
  endR: number,
  holeRProx: number,
  holeRDist: number,
  thickness: number,
  label: string,
): { geo: THREE.BufferGeometry; profile: PlateProfile } {
  const shape = new THREE.Shape();
  shape.absarc(0, 0, endR, Math.PI / 2, (3 * Math.PI) / 2, false);
  shape.lineTo(length, -endR);
  shape.absarc(length, 0, endR, (3 * Math.PI) / 2, Math.PI / 2, false);
  shape.closePath();

  const circles: { x: number; y: number; r: number }[] = [];
  const addHole = (x: number, r: number) => {
    const h = new THREE.Path();
    h.absarc(x, 0, r, 0, Math.PI * 2, true);
    shape.holes.push(h);
    circles.push({ x, y: 0, r });
  };
  if (holeRProx > 0) addHole(0, holeRProx);
  if (holeRDist > 0) addHole(length, holeRDist);

  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: thickness,
    bevelEnabled: false,
    curveSegments: 28,
  });
  const profile: PlateProfile = {
    label,
    outline: shape.getPoints(40).map((p) => ({ x: p.x, y: p.y })),
    circles,
    thickness,
    count: 1,
  };
  return { geo, profile };
}

/**
 * Guide block: rectangular cross-section (shape X = link height Z,
 * shape Y = link width X) with drilled tendon channels, extruded along the
 * link length. Channels run parallel to the extrusion (vertical in print).
 */
export function guideBlock(
  zFrom: number,
  zTo: number,
  xMin: number,
  xMax: number,
  length: number,
  channels: { z: number; x: number; r: number }[],
): THREE.BufferGeometry {
  const shape = new THREE.Shape();
  const z0 = Math.min(zFrom, zTo);
  const z1 = Math.max(zFrom, zTo);
  shape.moveTo(z0, xMin);
  shape.lineTo(z1, xMin);
  shape.lineTo(z1, xMax);
  shape.lineTo(z0, xMax);
  shape.closePath();
  for (const c of channels) {
    const h = new THREE.Path();
    h.absarc(c.z, c.x, c.r, 0, Math.PI * 2, true);
    shape.holes.push(h);
  }
  return new THREE.ExtrudeGeometry(shape, {
    depth: length,
    bevelEnabled: false,
    curveSegments: 20,
  });
}

/** Clevis-style hinge pin: shaft along +Z with a head at z=0 (print flat). */
export function hingePin(
  shaftR: number,
  shaftLen: number,
  headR: number,
  headT: number,
): THREE.BufferGeometry {
  const shaft = new THREE.CylinderGeometry(shaftR, shaftR, shaftLen, 20);
  shaft.rotateX(Math.PI / 2); // axis Y → Z
  shaft.translate(0, 0, headT + shaftLen / 2);
  const head = new THREE.CylinderGeometry(headR, headR, headT, 24);
  head.rotateX(Math.PI / 2);
  head.translate(0, 0, headT / 2);
  return mergeGeometries([shaft, head])!;
}

/** Fingertip pad: cylinder printed on its flat face (axis Z raw). */
export function tipPad(radius: number, width: number): THREE.BufferGeometry {
  const geo = new THREE.CylinderGeometry(radius, radius, width, 24);
  geo.rotateX(Math.PI / 2); // axis → Z (print), assembly re-orients to X
  return geo;
}

/** Rounded-rectangle plate (palm chassis), flat in XY, extruded +Z. */
export function roundedRectPlate(
  width: number,
  height: number,
  cornerR: number,
  thickness: number,
  holes: { x: number; y: number; r: number }[],
  label: string,
): { geo: THREE.BufferGeometry; profile: PlateProfile } {
  const w = width / 2;
  const h = height / 2;
  const r = Math.min(cornerR, w * 0.9, h * 0.9);
  const shape = new THREE.Shape();
  shape.moveTo(-w + r, -h);
  shape.lineTo(w - r, -h);
  shape.absarc(w - r, -h + r, r, -Math.PI / 2, 0, false);
  shape.lineTo(w, h - r);
  shape.absarc(w - r, h - r, r, 0, Math.PI / 2, false);
  shape.lineTo(-w + r, h);
  shape.absarc(-w + r, h - r, r, Math.PI / 2, Math.PI, false);
  shape.lineTo(-w, -h + r);
  shape.absarc(-w + r, -h + r, r, Math.PI, (3 * Math.PI) / 2, false);
  shape.closePath();
  for (const c of holes) {
    const hole = new THREE.Path();
    hole.absarc(c.x, c.y, c.r, 0, Math.PI * 2, true);
    shape.holes.push(hole);
  }
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: thickness,
    bevelEnabled: false,
    curveSegments: 24,
  });
  const profile: PlateProfile = {
    label,
    outline: shape.getPoints(32).map((p) => ({ x: p.x, y: p.y })),
    circles: holes.map((c) => ({ ...c })),
    thickness,
    count: 1,
  };
  return { geo, profile };
}

/** Per-link mechanical layout derived from finger + fabrication params. */
export interface LinkMech {
  /** ± center X of each side plate. */
  plateCx: number;
  plateT: number;
  /** Plate height (stadium width). */
  H: number;
  /** Clear span between the two plates (guide block width). */
  G: number;
  /** Link length (pin to pin / pin to tip). */
  L: number;
  /** Stadium end radius. */
  endR: number;
  /** Pin hole radius (pin + clearance). */
  holeR: number;
  /** Outer overall width at this link. */
  outerW: number;
  /** True for outer-fork links (even index). */
  outer: boolean;
  /** Guide block start/end along the link Y axis. */
  guideY0: number;
  guideY1: number;
}

export function computeLinkMech(
  finger: FingerParams,
  mech: MechParams,
): LinkMech[] {
  const t = mech.plateThickness;
  const clr = mech.hingeClearance;
  const W0 = 2 * finger.phalanges[0].radius;
  const gEven = W0 - 2 * t;
  const links: LinkMech[] = [];
  for (let i = 0; i < finger.phalanges.length; i++) {
    const ph = finger.phalanges[i];
    const outer = i % 2 === 0;
    const plateCx = outer
      ? W0 / 2 - t / 2
      : gEven / 2 - clr - t / 2;
    const G = outer ? gEven : gEven - 2 * clr - 2 * t;
    const H = 2 * ph.radius * 0.95;
    const endR = H / 2;
    const isLast = i === finger.phalanges.length - 1;
    const guideY0 = endR + 0.0025;
    const guideY1 = isLast
      ? ph.length * 0.72
      : ph.length - endR - 0.0025;
    links.push({
      plateCx,
      plateT: t,
      H,
      G,
      L: ph.length,
      endR,
      holeR: mech.pinDiameter / 2 + clr / 2,
      outerW: outer ? W0 : gEven - 2 * clr,
      outer,
      guideY0,
      guideY1: Math.max(guideY1, guideY0 + 0.003),
    });
  }
  return links;
}

/** Lateral channel offset for a tendon within a finger's guide blocks. */
export function tendonChannelX(
  tendon: TendonParams,
  link: LinkMech,
  hasFds: boolean,
): number {
  if (tendon.kind === 'extensor' || !hasFds) return 0;
  const off = Math.min(0.0022, link.G / 4);
  return tendon.name.endsWith('.fds') ? off : -off;
}
