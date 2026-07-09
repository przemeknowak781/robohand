/**
 * Pure fabrication-geometry math shared by the sim core and the 3D view.
 *
 * Lives under src/sim/ (not src/viz/) because defaultHand.ts needs it too:
 * joint flexion limits are clamped to whatever the physical link geometry
 * can actually achieve without its guide blocks colliding with the next
 * link's guide blocks (see computeMechJointLimits) — a mechanical fact that
 * belongs in the same place as the rest of the parametric hand model, not
 * bolted on in the renderer. No THREE.js / DOM dependency here.
 */

import type { MechParams, PhalanxParams, TendonParams } from './types';

/** Per-link mechanical layout derived from phalanx + fabrication params. */
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
  phalanges: PhalanxParams[],
  mech: MechParams,
): LinkMech[] {
  const t = mech.plateThickness;
  const clr = mech.hingeClearance;
  const W0 = 2 * phalanges[0].radius;
  const gEven = W0 - 2 * t;
  const links: LinkMech[] = [];
  for (let i = 0; i < phalanges.length; i++) {
    const ph = phalanges[i];
    const outer = i % 2 === 0;
    const plateCx = outer
      ? W0 / 2 - t / 2
      : gEven / 2 - clr - t / 2;
    const G = outer ? gEven : gEven - 2 * clr - 2 * t;
    const H = 2 * ph.radius * 0.95;
    const endR = H / 2;
    const isLast = i === phalanges.length - 1;
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

/**
 * Mechanically-safe max flexion angle per joint, so the printed guide
 * blocks on two adjacent links never sweep into each other.
 *
 * Both links' tendon-channel guide blocks sit at a fixed setback `d` from
 * the shared pin (see computeLinkMech's guideY0/guideY1) and jut out to
 * depth `zExt` (channel moment arm + channel radius + wall) in the flexion
 * plane. At rest the two blocks point away from each other across the pin
 * (~180° apart); flexion by angle θ rotates one link's block toward the
 * other's. Treating each block as occupying an angular half-width
 * β = atan(zExt / d) around its own axis, the blocks first touch once the
 * remaining gap (π − θ) drops to β_A + β_B — i.e. at
 * θ_safe = π − β_A − β_B, minus a safety margin.
 */
export function computeMechJointLimits(
  links: LinkMech[],
  flexArmAtJoint: number[],
  mech: MechParams,
  marginRad = 0.09,
): number[] {
  const chR = mech.channelDiameter / 2;
  const wall = mech.guideWall;
  const floor = 0.35; // rad — never clamp below this even if geometry is tight
  const limits = links.map(() => Infinity);
  for (let i = 1; i < links.length; i++) {
    const zExtA = (flexArmAtJoint[i - 1] ?? 0) + chR + wall;
    const zExtB = (flexArmAtJoint[i] ?? 0) + chR + wall;
    const dA = Math.max(1e-6, links[i - 1].L - links[i - 1].guideY1);
    const dB = Math.max(1e-6, links[i].guideY0);
    const betaA = Math.atan2(zExtA, dA);
    const betaB = Math.atan2(zExtB, dB);
    limits[i] = Math.max(floor, Math.PI - betaA - betaB - marginRad);
  }
  return limits;
}
